#include "Cache.h"

#ifndef DEBUG_CACHE
#define debug(...)
#else
#define debug(...)                                   \
	do                                               \
	{                                                \
		printf("\033[36m[DEBUG] %s ", __FUNCTION__); \
		printf(__VA_ARGS__);                         \
		printf("\033[0m\n");                         \
	} while (0)
#endif

namespace ramulator
{

	Cache::Cache(int size, int assoc, int block_size,
				 int mshr_entry_num, Level level,
				 std::shared_ptr<CacheSystem> cachesys) : level(level), cachesys(cachesys), higher_cache(0),
														  lower_cache(nullptr), size(size), assoc(assoc),
														  block_size(block_size), mshr_entry_num(mshr_entry_num)
	{

		debug("level %d size %d assoc %d block_size %d\n",
			  int(level), size, assoc, block_size);

		if (level == Level::L1)
		{
			level_string = "L1";
		}
		else if (level == Level::L2)
		{
			level_string = "L2";
		}
		else if (level == Level::L3)
		{
			level_string = "L3";
		}

		is_first_level = (level == cachesys->first_level);
		is_last_level = (level == cachesys->last_level);

		// Check size, block size and assoc are 2^N
		assert((size & (size - 1)) == 0);
		assert((block_size & (block_size - 1)) == 0);
		assert((assoc & (assoc - 1)) == 0);
		assert(size >= block_size);

		// Initialize cache configuration
		block_num = size / (block_size * assoc);
		index_mask = block_num - 1;
		index_offset = calc_log2(block_size);
		tag_offset = calc_log2(block_num) + index_offset;

		debug("index_offset %d", index_offset);
		debug("index_mask 0x%x", index_mask);
		debug("tag_offset %d", tag_offset);

		// regStats
		cache_read_miss.name(level_string + string("_cache_read_miss"))
			.desc("cache read miss count")
			.precision(0);

		cache_write_miss.name(level_string + string("_cache_write_miss"))
			.desc("cache write miss count")
			.precision(0);

		cache_total_miss.name(level_string + string("_cache_total_miss"))
			.desc("cache total miss count")
			.precision(0);

		cache_eviction.name(level_string + string("_cache_eviction"))
			.desc("number of evict from this level to lower level")
			.precision(0);

		cache_read_access.name(level_string + string("_cache_read_access"))
			.desc("cache read access count")
			.precision(0);

		cache_write_access.name(level_string + string("_cache_write_access"))
			.desc("cache write access count")
			.precision(0);

		cache_total_access.name(level_string + string("_cache_total_access"))
			.desc("cache total access count")
			.precision(0);

		cache_mshr_hit.name(level_string + string("_cache_mshr_hit"))
			.desc("cache mshr hit count")
			.precision(0);
		cache_mshr_unavailable.name(level_string + string("_cache_mshr_unavailable"))
			.desc("cache mshr not available count")
			.precision(0);
		cache_set_unavailable.name(level_string + string("_cache_set_unavailable"))
			.desc("cache set not available")
			.precision(0);
	}

	bool Cache::send(Request req)
	{
		if (req.type == Request::Type::GPIC)
		{
			debug("level %s req.opcode %s req.en %d req.addr 0x%lx",
				  level_string.c_str(), req.opcode.c_str(), req.en, req.addr);

			if (gpic_instruction_queue.size() >= MAX_GPIC_QUEUE_SIZE)
			{
				return false;
			}
			else
			{
				gpic_instruction_queue.push_back(make_pair(latency_each[int(level)] + GPIC_DELAY[req.opcode], req));
				return true;
			}
		}

		debug("level %s %s, index %d, tag %ld\n",
			  level_string.c_str(), req.c_str(), get_index(req.addr),
			  get_tag(req.addr));

		cache_total_access++;
		if (req.type == Request::Type::WRITE)
		{
			cache_write_access++;
		}
		else
		{
			assert(req.type == Request::Type::READ);
			cache_read_access++;
		}
		// If there isn't a set, create it.
		auto &lines = get_lines(req.addr);
		std::list<Line>::iterator line;

		if (is_hit(lines, req.addr, &line))
		{
			lines.push_back(Line(req.addr, get_tag(req.addr), false,
								 line->dirty || (req.type == Request::Type::WRITE)));
			lines.erase(line);
			cachesys->hit_list.push_back(
				make_pair(cachesys->clk + latency_each[int(level)], req));

			debug("hit, update timestamp %ld", cachesys->clk);
			debug("hit finish time %ld",
				  cachesys->clk + latency_each[int(level)]);

			return true;
		}
		else
		{
			debug("miss @level %d", int(level));
			cache_total_miss++;
			if (req.type == Request::Type::WRITE)
			{
				cache_write_miss++;
			}
			else
			{
				assert(req.type == Request::Type::READ);
				cache_read_miss++;
			}

			// The dirty bit will be set if this is a write request and @L1
			bool dirty = (req.type == Request::Type::WRITE);

			// Modify the type of the request to lower level
			if (req.type == Request::Type::WRITE)
			{
				req.type = Request::Type::READ;
			}

			// Look it up in MSHR entries
			assert(req.type == Request::Type::READ);
			auto mshr = hit_mshr(req.addr);
			if (mshr != mshr_entries.end())
			{
				debug("hit mshr");
				cache_mshr_hit++;
				mshr->second->dirty = (dirty || mshr->second->dirty);
				return true;
			}

			// All requests come to this stage will be READ, so they
			// should be recorded in MSHR entries.
			if (mshr_entries.size() == mshr_entry_num)
			{
				// When no MSHR entries available, the miss request
				// is stalling.
				cache_mshr_unavailable++;
				debug("no mshr entry available");
				return false;
			}

			// Check whether there is a line available
			if (all_sets_locked(lines))
			{
				cache_set_unavailable++;
				return false;
			}

			auto newline = allocate_line(lines, req.addr);
			if (newline == lines.end())
			{
				return false;
			}

			newline->dirty = dirty;

			// Add to MSHR entries
			mshr_entries.push_back(make_pair(req.addr, newline));

			// Send the request to next level;
			std::pair<long, Request> time_req = make_pair(cachesys->clk + latency_each[int(level)], req);
			if (!is_last_level)
			{
				retry_list.push_back(time_req);
			}
			else
			{
				printf("added to cachesystem waitlist\n");
				cachesys->wait_list.push_back(time_req);
			}
			return true;
		}
	}

	void Cache::evictline(long addr, bool dirty)
	{

		auto it = cache_lines.find(get_index(addr));
		assert(it != cache_lines.end()); // check inclusive cache
		auto &lines = it->second;
		auto line = find_if(lines.begin(), lines.end(),
							[addr, this](Line l)
							{ return (l.tag == get_tag(addr)); });

		assert(line != lines.end());
		// Update LRU queue. The dirty bit will be set if the dirty
		// bit inherited from higher level(s) is set.
		lines.push_back(Line(addr, get_tag(addr), false,
							 dirty || line->dirty));
		lines.erase(line);
	}

	std::pair<long, bool> Cache::invalidate(long addr)
	{
		long delay = latency_each[int(level)];
		bool dirty = false;

		auto &lines = get_lines(addr);
		if (lines.size() == 0)
		{
			// The line of this address doesn't exist.
			return make_pair(0, false);
		}
		auto line = find_if(lines.begin(), lines.end(),
							[addr, this](Line l)
							{ return (l.tag == get_tag(addr)); });

		// If the line is in this level cache, then erase it from
		// the buffer.
		if (line != lines.end())
		{
			assert(!line->lock);
			debug("invalidate %lx @ level %d", addr, int(level));
			lines.erase(line);
		}
		else
		{
			// If it's not in current level, then no need to go up.
			return make_pair(delay, false);
		}

		if (higher_cache.size())
		{
			long max_delay = delay;
			for (auto hc : higher_cache)
			{
				auto result = hc->invalidate(addr);
				if (result.second)
				{
					max_delay = max(max_delay, delay + result.first * 2);
				}
				else
				{
					max_delay = max(max_delay, delay + result.first);
				}
				dirty = dirty || line->dirty || result.second;
			}
			delay = max_delay;
		}
		else
		{
			dirty = line->dirty;
		}
		return make_pair(delay, dirty);
	}

	void Cache::evict(std::list<Line> *lines,
					  std::list<Line>::iterator victim)
	{
		debug("level %d miss evict victim %lx", int(level), victim->addr);
		cache_eviction++;

		long addr = victim->addr;
		long invalidate_time = 0;
		bool dirty = victim->dirty;

		// First invalidate the victim line in higher level.
		if (higher_cache.size())
		{
			for (auto hc : higher_cache)
			{
				auto result = hc->invalidate(addr);
				invalidate_time = max(invalidate_time,
									  result.first + (result.second ? latency_each[int(level)] : 0));
				dirty = dirty || result.second || victim->dirty;
			}
		}

		debug("invalidate delay: %ld, dirty: %s", invalidate_time,
			  dirty ? "true" : "false");

		if (!is_last_level)
		{
			// not LLC eviction
			assert(lower_cache != nullptr);
			lower_cache->evictline(addr, dirty);
		}
		else
		{
			// LLC eviction
			if (dirty)
			{
				Request write_req(addr, Request::Type::WRITE);
				cachesys->wait_list.push_back(make_pair(
					cachesys->clk + invalidate_time + latency_each[int(level)],
					write_req));

				debug("inject one write request to memory system "
					  "addr %lx, invalidate time %ld, issue time %ld",
					  write_req.addr, invalidate_time,
					  cachesys->clk + invalidate_time + latency_each[int(level)]);
			}
		}

		lines->erase(victim);
	}

	std::list<Cache::Line>::iterator Cache::allocate_line(
		std::list<Line> &lines, long addr)
	{
		// See if an eviction is needed
		if (need_eviction(lines, addr))
		{
			// Get victim.
			// The first one might still be locked due to reorder in MC
			auto victim = find_if(lines.begin(), lines.end(),
								  [this](Line line)
								  {
									  bool check = !line.lock;
									  if (!is_first_level)
									  {
										  for (auto hc : higher_cache)
										  {
											  if (!check)
											  {
												  return check;
											  }
											  check = check && hc->check_unlock(line.addr);
										  }
									  }
									  return check;
								  });
			if (victim == lines.end())
			{
				return victim; // doesn't exist a line that's already unlocked in each level
			}
			assert(victim != lines.end());
			evict(&lines, victim);
		}

		// Allocate newline, with lock bit on and dirty bit off
		lines.push_back(Line(addr, get_tag(addr)));
		auto last_element = lines.end();
		--last_element;
		return last_element;
	}

	bool Cache::is_hit(std::list<Line> &lines, long addr,
					   std::list<Line>::iterator *pos_ptr)
	{
		auto pos = find_if(lines.begin(), lines.end(),
						   [addr, this](Line l)
						   { return (l.tag == get_tag(addr)); });
		*pos_ptr = pos;
		if (pos == lines.end())
		{
			return false;
		}
		return !pos->lock;
	}

	void Cache::concatlower(Cache *lower)
	{
		lower_cache = lower;
		assert(lower != nullptr);
		lower->higher_cache.push_back(this);
	};

	bool Cache::need_eviction(const std::list<Line> &lines, long addr)
	{
		if (find_if(lines.begin(), lines.end(),
					[addr, this](Line l)
					{ return (get_tag(addr) == l.tag); }) != lines.end())
		{
			// Due to MSHR, the program can't reach here. Just for checking
			assert(false);
		}
		else
		{
			if (lines.size() < assoc)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
	}

	void Cache::callback(Request &req)
	{
		debug("level %d", int(level));

		if (mem_addr_to_gpic_op.count(req.addr) != 0)
		{
			// It's the result of a GPIC load/store command
			Request gpic_req = mem_addr_to_gpic_op[req.addr];
			mem_addr_to_gpic_op.erase(req.addr);
			gpic_addr_to_num_mem_op[gpic_req.addr]--;
			if (gpic_addr_to_num_mem_op[gpic_req.addr] == 0)
			{
				gpic_addr_to_num_mem_op.erase(gpic_req.addr);
				gpic_req.callback(gpic_req);
			}
		}
		else
		{
			printf("%s received in %s\n", req.c_str(), level_string.c_str());
			auto it = find_if(mshr_entries.begin(), mshr_entries.end(),
							  [&req, this](std::pair<long, std::list<Line>::iterator> mshr_entry)
							  {
								  return (align(mshr_entry.first) == align(req.addr));
							  });

			if (it != mshr_entries.end())
			{
				it->second->lock = false;
				mshr_entries.erase(it);
				printf("MSHR entry removed at %s\n", level_string.c_str());
			}
			else
			{
				printf("NO MSHR entry removed at %s\n", level_string.c_str());
			}

			if (higher_cache.size())
			{
				for (auto hc : higher_cache)
				{
					hc->callback(req);
				}
			}
		}
	}

	void Cache::tick()
	{

		if (!lower_cache->is_last_level)
			lower_cache->tick();

		auto it = retry_list.begin();
		while (it != retry_list.end())
		{
			if (cachesys->clk >= it->first)
			{
				printf("level %d sending req.addr 0x%lx\n", int(level), it->second.addr);
				if (lower_cache->send(it->second))
				{
					it = retry_list.erase(it);
				}
			}
			else
			{
				++it;
			}
		}

		if (last_gpic_instruction_clk != -1)
		{

			// There must be an instruction going on
			assert(gpic_instruction_queue.size() != 0);

			// Check if the instruction is done
			if (cachesys->clk - last_gpic_instruction_clk >= gpic_instruction_queue.at(0).first)
			{
				Request req = gpic_instruction_queue.at(0).second;

				if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos))
				{
					// If it's a load or store, make new queries and send to this cache level's queue
					int access_needed = (long)(std::ceil((float)(req.addr_end - req.addr + 1) / (float)(block_size)));
					assert(gpic_addr_to_num_mem_op.count(req.addr) == 0);
					gpic_addr_to_num_mem_op[req.addr] = access_needed;
					for (int i = 0; i < access_needed; i++)
					{
						// make the request and send it to itself
						Request::Type req_type = (req.opcode.find("load") != string::npos) ? Request::Type::READ : Request::Type::WRITE;
						long req_addr = req.addr + i * block_size;
						Request mem_req(req_addr, req_type, std::bind(&Cache::callback, this, placeholders::_1), req.coreid);
						assert(mem_addr_to_gpic_op.count(mem_req.addr) == 0);
						mem_addr_to_gpic_op[mem_req.addr] = req;
						send(mem_req);
					}
				}
				else
				{
					// Otherwise, we are ready to send the call back
					req.callback(req);
				}

				gpic_instruction_queue.erase(gpic_instruction_queue.begin());
				last_gpic_instruction_clk = -1;
			}
		}

		if ((gpic_instruction_queue.size() != 0) && (last_gpic_instruction_clk == -1))
		{
			// A new instruction must be started
			last_gpic_instruction_clk = cachesys->clk;
		}
	}

	void CacheSystem::tick()
	{
		debug("clk %ld", clk);

		++clk;

		// Sends ready waiting request to memory
		auto it = wait_list.begin();
		while (it != wait_list.end() && clk >= it->first)
		{
			if (!send_memory(it->second))
			{
				++it;
			}
			else
			{

				debug("complete req: addr %lx", (it->second).addr);

				it = wait_list.erase(it);
			}
		}

		// hit request callback
		it = hit_list.begin();
		while (it != hit_list.end())
		{
			if (clk >= it->first)
			{
				it->second.callback(it->second);

				debug("finish hit: addr %lx", (it->second).addr);

				it = hit_list.erase(it);
			}
			else
			{
				++it;
			}
		}
	}

} // namespace ramulator
