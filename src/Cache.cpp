#include "Cache.h"

#ifndef DEBUG_CACHE
#define debug(...)
#else
#define debug(...)                                   \
    do {                                             \
        printf("\033[36m[DEBUG] %s ", __FUNCTION__); \
        printf(__VA_ARGS__);                         \
        printf("\033[0m\n");                         \
    } while (0)
#endif

namespace ramulator {

Cache::Cache(int size, int assoc, int block_size,
    int mshr_entry_num, Level level,
    std::shared_ptr<CacheSystem> cachesys, int core_id)
    : level(level)
    , cachesys(cachesys)
    , higher_cache(0)
    , lower_cache(nullptr)
    , core_id(core_id)
    , size(size)
    , assoc(assoc)
    , block_size(block_size)
    , mshr_entry_num(mshr_entry_num)
{

    debug("level %d size %d assoc %d block_size %d\n",
        int(level), size, assoc, block_size);

    if (level == Level::L1) {
        level_string = "L1";
    } else if (level == Level::L2) {
        level_string = "L2";
    } else if (level == Level::L3) {
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
    if (req.type == Request::Type::GPIC) {
        debug("level %s received %s", level_string.c_str(), req.c_str());

        if (gpic_instruction_queue.size() >= MAX_GPIC_QUEUE_SIZE) {
            return false;
        } else {
            hint("%s set for %ld clock cycles (%ld)\n", req.c_str(), latency_each[int(level)] + GPIC_DELAY[req.opcode], last_gpic_instruction_clk);
            gpic_instruction_queue.push_back(make_pair(latency_each[int(level)] + GPIC_DELAY[req.opcode], req));
            return true;
        }
    }

    debug("level %s received %s, index %d, tag %ld\n",
        level_string.c_str(), req.c_str(), get_index(req.addr),
        get_tag(req.addr));

    cache_total_access++;
    if (req.type == Request::Type::WRITE) {
        cache_write_access++;
    } else {
        assert(req.type == Request::Type::READ);
        cache_read_access++;
    }
    // If there isn't a set, create it.
    auto& lines = get_lines(req.addr);
    std::list<Line>::iterator line;

    if (is_hit(lines, req.addr, &line)) {
        lines.push_back(Line(req.addr, get_tag(req.addr), false,
            line->dirty || (req.type == Request::Type::WRITE)));
        lines.erase(line);
        cachesys->hit_list.push_back(
            make_pair(cachesys->clk + latency_each[int(level)], req));

        debug("hit, update timestamp %ld", cachesys->clk);
        debug("hit finish time %ld",
            cachesys->clk + latency_each[int(level)]);

        return true;
    } else {
        debug("miss @level %d", int(level));
        cache_total_miss++;
        if (req.type == Request::Type::WRITE) {
            cache_write_miss++;
        } else {
            assert(req.type == Request::Type::READ);
            cache_read_miss++;
        }

        // The dirty bit will be set if this is a write request and @L1
        bool dirty = (req.type == Request::Type::WRITE);

        // Modify the type of the request to lower level
        if (req.type == Request::Type::WRITE) {
            req.type = Request::Type::READ;
        }

        // Look it up in MSHR entries
        assert(req.type == Request::Type::READ);
        auto mshr = hit_mshr(req.addr);
        if (mshr != mshr_entries.end()) {
            debug("hit mshr");
            cache_mshr_hit++;
            mshr->second->dirty = (dirty || mshr->second->dirty);
            return true;
        }

        // All requests come to this stage will be READ, so they
        // should be recorded in MSHR entries.
        if (mshr_entries.size() == mshr_entry_num) {
            // When no MSHR entries available, the miss request
            // is stalling.
            cache_mshr_unavailable++;
            debug("no mshr entry available");
            return false;
        }

        // Check whether there is a line available
        if (all_sets_locked(lines)) {
            cache_set_unavailable++;
            return false;
        }

        auto newline = allocate_line(lines, req.addr);
        if (newline == lines.end()) {
            return false;
        }

        newline->dirty = dirty;

        // Add to MSHR entries
        mshr_entries.push_back(make_pair(req.addr, newline));

        // Send the request to next level;
        std::pair<long, Request> time_req = make_pair(cachesys->clk + latency_each[int(level)], req);
        if (!is_last_level) {
            retry_list.push_back(time_req);
        } else {
            hint("11- %s sending %s to cachesystem waitlist\n", level_string.c_str(), req.c_str());
            cachesys->wait_list.push_back(time_req);
        }
        return true;
    }
}

void Cache::evictline(long addr, bool dirty)
{

    auto it = cache_lines.find(get_index(addr));
    assert(it != cache_lines.end()); // check inclusive cache
    auto& lines = it->second;
    auto line = find_if(lines.begin(), lines.end(),
        [addr, this](Line l) { return (l.tag == get_tag(addr)); });

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

    auto& lines = get_lines(addr);
    if (lines.size() == 0) {
        // The line of this address doesn't exist.
        return make_pair(0, false);
    }
    auto line = find_if(lines.begin(), lines.end(),
        [addr, this](Line l) { return (l.tag == get_tag(addr)); });

    // If the line is in this level cache, then erase it from
    // the buffer.
    if (line != lines.end()) {
        assert(!line->lock);
        debug("invalidate %lx @ level %d", addr, int(level));
        lines.erase(line);
    } else {
        // If it's not in current level, then no need to go up.
        return make_pair(delay, false);
    }

    if (higher_cache.size()) {
        long max_delay = delay;
        for (auto hc : higher_cache) {
            auto result = hc->invalidate(addr);
            if (result.second) {
                max_delay = max(max_delay, delay + result.first * 2);
            } else {
                max_delay = max(max_delay, delay + result.first);
            }
            dirty = dirty || line->dirty || result.second;
        }
        delay = max_delay;
    } else {
        dirty = line->dirty;
    }
    return make_pair(delay, dirty);
}

void Cache::evict(std::list<Line>* lines,
    std::list<Line>::iterator victim)
{
    debug("level %d miss evict victim %lx", int(level), victim->addr);
    cache_eviction++;

    long addr = victim->addr;
    long invalidate_time = 0;
    bool dirty = victim->dirty;

    // First invalidate the victim line in higher level.
    if (higher_cache.size()) {
        for (auto hc : higher_cache) {
            auto result = hc->invalidate(addr);
            invalidate_time = max(invalidate_time,
                result.first + (result.second ? latency_each[int(level)] : 0));
            dirty = dirty || result.second || victim->dirty;
        }
    }

    debug("invalidate delay: %ld, dirty: %s", invalidate_time,
        dirty ? "true" : "false");

    if (!is_last_level) {
        // not LLC eviction
        assert(lower_cache != nullptr);
        lower_cache->evictline(addr, dirty);
    } else {
        // LLC eviction
        if (dirty) {
            Request write_req(addr, Request::Type::WRITE, MAX_CORE_ID, (Request::UnitID)(level));
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
    std::list<Line>& lines, long addr)
{
    // See if an eviction is needed
    if (need_eviction(lines, addr)) {
        // Get victim.
        // The first one might still be locked due to reorder in MC
        auto victim = find_if(lines.begin(), lines.end(),
            [this](Line line) {
                bool check = !line.lock;
                if (!is_first_level) {
                    for (auto hc : higher_cache) {
                        if (!check) {
                            return check;
                        }
                        check = check && hc->check_unlock(line.addr);
                    }
                }
                return check;
            });
        if (victim == lines.end()) {
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

bool Cache::is_hit(std::list<Line>& lines, long addr,
    std::list<Line>::iterator* pos_ptr)
{
    auto pos = find_if(lines.begin(), lines.end(),
        [addr, this](Line l) { return (l.tag == get_tag(addr)); });
    *pos_ptr = pos;
    if (pos == lines.end()) {
        return false;
    }
    return !pos->lock;
}

void Cache::concatlower(Cache* lower)
{
    lower_cache = lower;
    assert(lower != nullptr);
    lower->higher_cache.push_back(this);
};

bool Cache::need_eviction(const std::list<Line>& lines, long addr)
{
    if (find_if(lines.begin(), lines.end(),
            [addr, this](Line l) { return (get_tag(addr) == l.tag); })
        != lines.end()) {
        // Due to MSHR, the program can't reach here. Just for checking
        assert(false);
    } else {
        if (lines.size() < assoc) {
            return false;
        } else {
            return true;
        }
    }
}

void Cache::callback(Request& req)
{
    debug("level %d", int(level));
    hint("%s received in %s\n", req.c_str(), level_string.c_str());

    // Remove related MSHR entries
    auto MSHR_it = find_if(mshr_entries.begin(), mshr_entries.end(),
        [&req, this](std::pair<long, std::list<Line>::iterator> mshr_entry) {
            return (align(mshr_entry.first) == align(req.addr));
        });

    if (MSHR_it != mshr_entries.end()) {
        MSHR_it->second->lock = false;
        mshr_entries.erase(MSHR_it);
        hint("MSHR entry removed at %s\n", level_string.c_str());
    } else {
        hint("NO MSHR entry removed at %s\n", level_string.c_str());
    }

    // Remove corresponding GPIC instructions
    if ((last_gpic_instruction_started == true) && (last_gpic_instruction_sent == true)) {
        Request GPIC_req = gpic_instruction_queue[0].second;
        if ((align(req.addr) >= align(GPIC_req.addr)) && (align(req.addr) <= align(GPIC_req.addr_end))) {
            hint("%s: %s calls back for %s, %d instructions remained\n", level_string.c_str(), req.c_str(), GPIC_req.c_str(), gpic_op_to_num_mem_op[GPIC_req] - 1);
            if ((--gpic_op_to_num_mem_op[GPIC_req]) == 0) {
                hint("%s: calling back %s\n", level_string.c_str(), GPIC_req.c_str());
                GPIC_req.callback(GPIC_req);
                gpic_op_to_num_mem_op.erase(GPIC_req);
                gpic_instruction_queue.erase(gpic_instruction_queue.begin());
                last_gpic_instruction_clk = -1;
                last_gpic_instruction_started = false;
                last_gpic_instruction_sent = false;
            }
        }
    }

    if (higher_cache.size()) {
        for (auto hc : higher_cache) {
            hc->callback(req);
        }
    }
}

void Cache::tick()
{

    if (!is_last_level)
        if (!lower_cache->is_last_level)
            lower_cache->tick();

    auto it = retry_list.begin();
    while (it != retry_list.end()) {
        if (cachesys->clk >= it->first) {
            hint("9- %s sending %s to %s\n", level_string.c_str(), it->second.c_str(), lower_cache->level_string.c_str());
            if (lower_cache->send(it->second)) {
                it = retry_list.erase(it);
            }
        } else {
            ++it;
        }
    }

    if ((last_gpic_instruction_started == true) && (last_gpic_instruction_sent == false)) {

        // There must be an instruction going on
        assert(gpic_instruction_queue.size() != 0);

        // Check if the instruction is done
        if (cachesys->clk - last_gpic_instruction_clk >= gpic_instruction_queue.at(0).first) {
            Request req = gpic_instruction_queue.at(0).second;

            if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos)) {
                // If it's a load or store, make new queries and send to this cache level's queue
                int access_needed = (long)(std::ceil((float)(req.addr_end - req.addr + 1) / (float)(block_size)));
                assert(gpic_op_to_num_mem_op.count(req) == 0);
                gpic_op_to_num_mem_op[req] = access_needed;
                hint("10- %s unpacking %d instructions for %s\n", level_string.c_str(), access_needed, req.c_str());
                for (int i = 0; i < access_needed; i++) {

                    // make the request
                    long req_addr = req.addr + i * block_size;
                    Request::Type req_type = (req.opcode.find("load") != string::npos) ? Request::Type::READ : Request::Type::WRITE;
                    int req_coreid = req.coreid;
                    Request::UnitID req_unitid = (Request::UnitID)(level);
                    Request mem_req(req_addr, req_type, processor_callback, req_coreid, req_unitid);
                    mem_req.reqid = last_id;
                    last_id++;

                    // send it
                    hint("10- %s sending %s to %s\n", level_string.c_str(), mem_req.c_str(), level_string.c_str());
                    send(mem_req);
                }
                last_gpic_instruction_sent = true;
            } else {
                // Otherwise, we are ready to send the call back
                req.callback(req);
                gpic_instruction_queue.erase(gpic_instruction_queue.begin());
                last_gpic_instruction_clk = -1;
                last_gpic_instruction_started = false;
                last_gpic_instruction_sent = false;
            }
        }
    }

    if (last_gpic_instruction_started == false) {

        // The last instruction must not be sent
        assert(last_gpic_instruction_sent == false);

        if (gpic_instruction_queue.size() != 0) {
            // A new instruction must be started
            hint("Starting %s at %ld, %zu instructions in queue\n", gpic_instruction_queue.at(0).second.c_str(), cachesys->clk, gpic_instruction_queue.size());
            last_gpic_instruction_clk = cachesys->clk;
            last_gpic_instruction_started = true;
        }
    }
}

void Cache::reset_state()
{
    if (level == Level::L3)
        hint("Cache %s state reset\n", level_string.c_str());
    else
        hint("Core %d's Cache %s state reset\n", core_id, level_string.c_str());

    cache_read_miss = 0;
    cache_write_miss = 0;
    cache_total_miss = 0;
    cache_eviction = 0;
    cache_read_access = 0;
    cache_write_access = 0;
    cache_total_access = 0;
    cache_mshr_hit = 0;
    cache_mshr_unavailable = 0;
    cache_set_unavailable = 0;
    last_id = 0;
    last_gpic_instruction_clk = -1;
    last_gpic_instruction_started = false;
    last_gpic_instruction_sent = false;
    assert(mshr_entries.size() == 0);
    assert(retry_list.size() == 0);
    assert(gpic_op_to_num_mem_op.size() == 0);
    assert(gpic_instruction_queue.size() == 0);
}

void CacheSystem::tick()
{
    debug("clk %ld", clk);

    ++clk;

    // Sends ready waiting request to memory
    auto it = wait_list.begin();
    while (it != wait_list.end() && clk >= it->first) {
        if (!send_memory(it->second)) {
            ++it;
        } else {

            debug("complete req: addr %lx", (it->second).addr);

            it = wait_list.erase(it);
        }
    }

    // hit request callback
    it = hit_list.begin();
    while (it != hit_list.end()) {
        if (clk >= it->first) {
            it->second.callback(it->second);

            debug("finish hit: addr %lx", (it->second).addr);

            it = hit_list.erase(it);
        } else {
            ++it;
        }
    }
}

void CacheSystem::reset_state()
{
    hint("CacheSystem state reset\n");
    clk = 0;
    assert(wait_list.size() == 0);
    assert(hit_list.size() == 0);
}

} // namespace ramulator
