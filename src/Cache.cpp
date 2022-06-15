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
    int mshr_entry_num, float access_energy, Level level,
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
    , access_energy(access_energy)
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

    init_intrinsic_latency();

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
    cache_access_energy.name(level_string + string("_cache_access_energy"))
        .desc("cache access energy in pJ")
        .precision(0);

    GPIC_host_device_total_cycles.name(level_string + string("_GPIC_host_device_total_cycles"))
        .desc("total cycles at which cache GPIC instruction queue is empty")
        .precision(0);
    GPIC_move_stall_total_cycles.name(level_string + string("_GPIC_move_stall_total_cycles"))
        .desc("total cycles at which cache GPIC is stalled because of move instructions")
        .precision(0);
    GPIC_compute_total_cycles.name(level_string + string("_GPIC_compute_total_cycles"))
        .desc("total cycles at which cache GPIC doing computation or W/R")
        .precision(0);
    GPIC_memory_total_cycles.name(level_string + string("_GPIC_memory_total_cycles"))
        .desc("total cycles at which cache GPIC waiting for mem requests")
        .precision(0);

    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        GPIC_host_device_cycles[SA_id].name(level_string + string("_GPIC_host_device_cycles[") + to_string(SA_id) + string("]")).desc("cache GPIC instruction queue is empty").precision(0);
        GPIC_move_stall_cycles[SA_id].name(level_string + string("_GPIC_move_stall_cycles[") + to_string(SA_id) + string("]")).desc("cache GPIC is stalled for move instruction").precision(0);
        GPIC_compute_cycles[SA_id].name(level_string + string("_GPIC_compute_cycles[") + to_string(SA_id) + string("]")).desc("cache GPIC doing computation or W/R").precision(0);
        GPIC_memory_cycles[SA_id].name(level_string + string("_GPIC_memory_cycles[") + to_string(SA_id) + string("]")).desc("cache GPIC waiting for mem requests").precision(0);
    }

    GPIC_compute_total_energy.name(level_string + string("_GPIC_compute_total_energy"))
        .desc("total cache GPIC compute energy in pJ")
        .precision(0);
    GPIC_compute_comp_total_energy.name(level_string + string("_GPIC_compute_comp_total_energy"))
        .desc("total cache GPIC compute energy [compute part] in pJ")
        .precision(0);
    GPIC_compute_rdwr_total_energy.name(level_string + string("_GPIC_compute_rdwr_total_energy"))
        .desc("total cache GPIC compute energy [read/write part] in pJ")
        .precision(0);

    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        GPIC_compute_energy[SA_id].name(level_string + string("_GPIC_compute_energy[") + to_string(SA_id) + string("]")).desc("cache GPIC compute energy in pJ").precision(0);
        GPIC_compute_comp_energy[SA_id].name(level_string + string("_GPIC_compute_comp_energy[") + to_string(SA_id) + string("]")).desc("cache GPIC compute energy [compute part] in pJ").precision(0);
        GPIC_compute_rdwr_energy[SA_id].name(level_string + string("_GPIC_compute_rdwr_energy[") + to_string(SA_id) + string("]")).desc("cache GPIC compute energy [read/write part] in pJ").precision(0);
    }
}

void Cache::init_intrinsic_latency()
{
    std::string line, intrinsic, rd_wr_latency, compute_latency;

    fstream file("/home/arkhadem/GPIC/ramulator/data/gpic_intrinsics_latency.csv", ios::in);
    if (file.is_open()) {

        // reading and ignoring header file
        std::getline(file, line);

        // reading all intrinsics
        while (std::getline(file, line)) {

            stringstream str(line);

            std::getline(str, intrinsic, ',');
            std::getline(str, rd_wr_latency, ',');
            std::getline(str, compute_latency, ',');

            GPIC_ACCESS_DELAY[intrinsic.c_str()] = atoi(rd_wr_latency.c_str());
            GPIC_COMPUTE_DELAY[intrinsic.c_str()] = atoi(compute_latency.c_str());
        }
    } else {
        printf("Error: could not find /home/arkhadem/GPIC/ramulator/data/gpic_intrinsics_latency.csv\n");
    }
    file.close();

    file.open("/home/arkhadem/GPIC/ramulator/data/dc_intrinsics_latency.csv", ios::in);
    if (file.is_open()) {

        // reading and ignoring header file
        std::getline(file, line);

        // reading all intrinsics
        while (std::getline(file, line)) {

            stringstream str(line);

            std::getline(str, intrinsic, ',');
            std::getline(str, rd_wr_latency, ',');
            std::getline(str, compute_latency, ',');

            DC_ACCESS_DELAY[intrinsic.c_str()] = atoi(rd_wr_latency.c_str());
            DC_COMPUTE_DELAY[intrinsic.c_str()] = atoi(compute_latency.c_str());
        }
    } else {
        printf("Error: could not find /home/arkhadem/GPIC/ramulator/data/dc_intrinsics_latency.csv\n");
    }
    file.close();
}

bool Cache::send(Request req)
{
    if (req.type == Request::Type::GPIC) {
        debug("level %s received %s", level_string.c_str(), req.c_str());

        int total_SAs = ((req.en - 1) / 256) + 1;
        assert((total_SAs > 0) && (total_SAs <= GPIC_SA_NUM));

        int SA_min = req.SA_id * total_SAs;
        if ((SA_min < 0) || (SA_min >= GPIC_SA_NUM)) {
            cout << "((" << SA_min << " >= 0) && (" << SA_min << "<" << GPIC_SA_NUM << "))" << endl;
            std::cout << req.c_str() << endl;
        }
        assert((SA_min >= 0) && (SA_min < GPIC_SA_NUM));

        int SA_max = (req.SA_id + 1) * total_SAs;
        assert((SA_max > 0) && (SA_max <= GPIC_SA_NUM));

        for (int SA_id = SA_min; SA_id < SA_max; SA_id++) {
            if (gpic_instruction_queue[SA_id].size() >= MAX_GPIC_QUEUE_SIZE) {
                return false;
            }
        }
        if (req.opcode.find("move") != string::npos) {
            int SA_min_dst = req.SA_id_dst * total_SAs;
            assert((SA_min_dst >= 0) && (SA_min_dst < GPIC_SA_NUM));

            int SA_max_dst = (req.SA_id_dst + 1) * total_SAs;
            assert((SA_max_dst > 0) && (SA_max_dst <= GPIC_SA_NUM));

            for (int SA_index = SA_min_dst; SA_index < total_SAs; SA_index++) {
                if (gpic_instruction_queue[SA_min_dst + SA_index].size() >= MAX_GPIC_QUEUE_SIZE) {
                    return false;
                }
            }
            int en = req.en;
            for (int SA_index = SA_min_dst; SA_index < total_SAs; SA_index++) {
                if (en > 256)
                    req.en = 256;
                else
                    req.en = en;

                req.SA_id = SA_min + SA_index;
                req.SA_id_dst = SA_min_dst + SA_index;

                hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);

                gpic_instruction_queue[SA_min + SA_index].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
                gpic_instruction_queue[SA_min_dst + SA_index].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
                en -= 256;
            }
            return true;
        } else {
            int en = req.en;
            long addr = req.addr;
            for (int SA_id = SA_min; SA_id < SA_max; SA_id++) {
                req.SA_id = SA_id;

                if (en > 256)
                    req.en = 256;
                else
                    req.en = en;

                if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos)) {
                    if ((req.opcode.find("load1") == string::npos) && (req.opcode.find("store1") == string::npos)) {
                        req.addr = addr;
                        req.addr_end = (long)(std::ceil((float)(req.data_type * req.en / 8))) + req.addr - 1;
                        addr = req.addr_end + 1;
                    }
                }
                hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
                gpic_instruction_queue[SA_id].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
                en -= 256;
            }
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

        // Reading/writing block_size bytes from cache for a hit
        cache_access_energy += access_energy;

        return true;
    } else {
        debug("%s missed @level %d", req.c_str(), (level));
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
            debug("%s hitted mshr", req.c_str());
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
        hint("pair(0x%lx, line(0x%lx, %d, %d, 0x%lx)) added to mshr entries at level %s\n", req.addr, newline->addr, newline->dirty, newline->lock, newline->tag, level_string.c_str());

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
            // Request write_req(addr, Request::Type::WRITE, MAX_CORE_ID, (Request::UnitID)(level));
            Request write_req(addr, Request::Type::WRITE, 0, (Request::UnitID)(level));
            cachesys->wait_list.push_back(make_pair(
                cachesys->clk + invalidate_time + latency_each[int(level)],
                write_req));

            // Reading block_size for eviction from LLC and writing to memory
            cache_access_energy += access_energy;

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

    // Writing block_size bytes to cache as a result of a miss
    cache_access_energy += access_energy;

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
        auto first = MSHR_it->first;
        auto second = MSHR_it->second;
        hint("pair(0x%lx, line(0x%lx, %d, %d, 0x%lx)) removed from mshr entries at level %s\n", first, second->addr, second->dirty, second->lock, second->tag, level_string.c_str());
        mshr_entries.erase(MSHR_it);
    } else {
        hint("NO MSHR entry removed at %s\n", level_string.c_str());
    }

    // Remove corresponding GPIC instructions
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        if ((last_gpic_instruction_computed[SA_id] == true) && (last_gpic_instruction_sent[SA_id] == true)) {
            Request gpic_req = gpic_compute_queue[SA_id][0].second;
            if ((align(req.addr) >= align(gpic_req.addr)) && (align(req.addr) <= align(gpic_req.addr_end))) {
                hint("%s: %s calls back for %s, %d instructions remained\n", level_string.c_str(), req.c_str(), gpic_req.c_str(), gpic_op_to_num_mem_op[SA_id][gpic_req] - 1);
                if ((--gpic_op_to_num_mem_op[SA_id][gpic_req]) == 0) {
                    op_trace << cachesys->clk << " " << SA_id << " F " << gpic_req.opcode << endl;

                    hint("%s: calling back %s\n", level_string.c_str(), gpic_req.c_str());
                    gpic_req.callback(gpic_req);
                    gpic_op_to_num_mem_op[SA_id].erase(gpic_req);
                    gpic_compute_queue[SA_id].erase(gpic_compute_queue[SA_id].begin());
                    last_gpic_instruction_compute_clk[SA_id] = -1;
                    last_gpic_instruction_computed[SA_id] = false;
                    last_gpic_instruction_sent[SA_id] = false;
                }
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
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    while (self_retry_list.size() != 0) {
        hint("12- %s sending %s to %s\n", level_string.c_str(), self_retry_list[0].c_str(), level_string.c_str());
        if (send(self_retry_list[0])) {
            self_retry_list.erase(self_retry_list.begin());
        } else {
            break;
        }
    }

    // Instruction received by cache, sent to GPIC core queue
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        while ((gpic_instruction_queue[SA_id].size() > 0) && (cachesys->clk >= gpic_instruction_queue[SA_id][0].first)) {
            Request req = gpic_instruction_queue[SA_id][0].second;
            long compute_delay, access_delay, bitlines;
            if (req.opcode.find("_dc_") != string::npos) {
                assert(req.opcode.find("_pc_") == string::npos);
                compute_delay = DC_COMPUTE_DELAY[req.opcode];
                access_delay = DC_ACCESS_DELAY[req.opcode];
                bitlines = 4;
            } else {
                assert(req.opcode.find("_pc_") != string::npos);
                compute_delay = GPIC_COMPUTE_DELAY[req.opcode];
                access_delay = GPIC_ACCESS_DELAY[req.opcode];
                bitlines = 1;
            }
            // printf("%s %d %d\n", req.opcode.c_str(), compute_delay, access_delay);
            hint("%s set for compute in %ld clock cycles\n", req.c_str(), compute_delay + access_delay);
            gpic_instruction_queue[SA_id].erase(gpic_instruction_queue[SA_id].begin());
            gpic_compute_queue[SA_id].push_back(make_pair(compute_delay + access_delay, req));
            GPIC_compute_total_energy += ((float)compute_delay * 15.4 + (float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_comp_total_energy += ((float)compute_delay * 15.4) * (float)bitlines;
            GPIC_compute_rdwr_total_energy += ((float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_energy[SA_id] += ((float)compute_delay * 15.4 + (float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_comp_energy[SA_id] += ((float)compute_delay * 15.4) * (float)bitlines;
            GPIC_compute_rdwr_energy[SA_id] += ((float)access_delay * 8.6) * (float)bitlines;
        }
    }

    // Instruction at the head of the GPIC queue is computed
    // If it's a store or load it is unpacked
    // Otherwise, it is called back
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {

        // Check if there is any instruction ready for completion
        if ((last_gpic_instruction_computed[SA_id] == true) && (last_gpic_instruction_sent[SA_id] == false)) {

            // There must be an instruction going on
            assert(gpic_compute_queue[SA_id].size() != 0);

            // Check if the instruction is done
            if (cachesys->clk - last_gpic_instruction_compute_clk[SA_id] >= gpic_compute_queue[SA_id].at(0).first) {
                Request req = gpic_compute_queue[SA_id].at(0).second;

                if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos)) {
                    // If it's a load or store, make new queries and send to this cache level's queue
                    int access_needed = (long)(std::ceil((float)(req.addr_end - req.addr + 1) / (float)(block_size)));
                    assert(access_needed != 0);
                    assert(gpic_op_to_num_mem_op[SA_id].count(req) == 0);
                    gpic_op_to_num_mem_op[SA_id][req] = access_needed;
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
                        if (send(mem_req) == false) {
                            self_retry_list.push_back(mem_req);
                        }
                    }
                    last_gpic_instruction_sent[SA_id] = true;
                } else {
                    // Otherwise, we are ready to send the call back
                    op_trace << cachesys->clk << " " << SA_id << " F " << req.opcode << endl;

                    hint("10- %s instruction %s completed\n", level_string.c_str(), req.c_str());
                    req.callback(req);
                    gpic_compute_queue[SA_id].erase(gpic_compute_queue[SA_id].begin());
                    last_gpic_instruction_compute_clk[SA_id] = -1;
                    last_gpic_instruction_computed[SA_id] = false;
                    last_gpic_instruction_sent[SA_id] = false;
                }
            }
        }
    }

    // Instruction at the head of the GPIC queue gets ready for compute
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        // Check if there is any instructions ready to be computed
        if (last_gpic_instruction_computed[SA_id] == false) {

            // The last instruction must not be sent
            assert(last_gpic_instruction_sent[SA_id] == false);

            if (gpic_compute_queue[SA_id].size() != 0) {
                // A new instruction must be computed
                if (gpic_compute_queue[SA_id].at(0).second.opcode.find("move") != string::npos) {

                    // we should wait for the dst SA as well
                    int SA_id_dst = gpic_compute_queue[SA_id].at(0).second.SA_id_dst;
                    assert(gpic_compute_queue[SA_id].at(0).second.SA_id != SA_id_dst);
                    assert(SA_id_dst != -1);

                    // only source SA checks for the dst, not vice versa
                    if (SA_id != SA_id_dst) {

                        // Check if there is any instructions ready to be computed in the dst queue
                        if (last_gpic_instruction_computed[SA_id_dst] == false) {

                            // The last instruction must not be sent
                            assert(last_gpic_instruction_sent[SA_id_dst] == false);

                            if (gpic_compute_queue[SA_id_dst].size() != 0) {

                                // Check if these are the same instructions
                                if (gpic_compute_queue[SA_id].at(0).second == gpic_compute_queue[SA_id_dst].at(0).second) {

                                    // Compute both
                                    op_trace << cachesys->clk << " " << SA_id << " S " << gpic_compute_queue[SA_id].at(0).second.opcode << endl;
                                    op_trace << cachesys->clk << " " << SA_id_dst << " S " << gpic_compute_queue[SA_id].at(0).second.opcode << endl;
                                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[SA_id].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[SA_id].size());
                                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[SA_id_dst].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[SA_id_dst].size());
                                    last_gpic_instruction_compute_clk[SA_id] = cachesys->clk;
                                    last_gpic_instruction_compute_clk[SA_id_dst] = cachesys->clk;
                                    last_gpic_instruction_computed[SA_id] = true;
                                    last_gpic_instruction_computed[SA_id_dst] = true;
                                }
                            }
                        }

                        if (last_gpic_instruction_computed[SA_id] == false) {
                            hint("%s is waiting for dst SA\n", gpic_compute_queue[SA_id].at(0).second.c_str());
                        }
                    }

                } else {
                    op_trace << cachesys->clk << " " << SA_id << " S " << gpic_compute_queue[SA_id].at(0).second.opcode << endl;
                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[SA_id].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[SA_id].size());
                    last_gpic_instruction_compute_clk[SA_id] = cachesys->clk;
                    last_gpic_instruction_computed[SA_id] = true;
                }
            }
        }
    }

    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        if ((last_gpic_instruction_computed[SA_id] == false) && (last_gpic_instruction_sent[SA_id] == false)) {

            if (gpic_compute_queue[SA_id].size() == 0) {
                // There is no instruction ready for execute
                GPIC_host_device_total_cycles++;
                GPIC_host_device_cycles[SA_id]++;
            } else {
                // Instruction at top must be move and stalled by dst SA
                assert(gpic_compute_queue[SA_id].at(0).second.opcode.find("move") != string::npos);
                GPIC_move_stall_total_cycles++;
                GPIC_move_stall_cycles[SA_id]++;
            }
        }

        if ((last_gpic_instruction_computed[SA_id] == true) && (last_gpic_instruction_sent[SA_id] == false)) {
            // An instruction is being computed
            GPIC_compute_total_cycles++;
            GPIC_compute_cycles[SA_id]++;
        }

        if ((last_gpic_instruction_computed[SA_id] == true) && (last_gpic_instruction_sent[SA_id] == true)) {
            // An instruction is computed and sent, waiting for memory instructions to be called back
            GPIC_memory_total_cycles++;
            GPIC_memory_cycles[SA_id]++;
        }
    }
}

bool Cache::finished()
{
    if (mshr_entries.size() != 0)
        return false;
    if (retry_list.size() != 0)
        return false;
    if (self_retry_list.size() != 0)
        return false;
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        if (gpic_op_to_num_mem_op[SA_id].size() != 0)
            return false;

        if (gpic_instruction_queue[SA_id].size() != 0)
            return false;

        if (gpic_compute_queue[SA_id].size() != 0)
            return false;
    }
    return true;
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
    cache_access_energy = 0;
    GPIC_host_device_total_cycles = 0;
    GPIC_move_stall_total_cycles = 0;
    GPIC_compute_total_cycles = 0;
    GPIC_memory_total_cycles = 0;
    GPIC_compute_total_energy = 0;
    GPIC_compute_comp_total_energy = 0;
    GPIC_compute_rdwr_total_energy = 0;
    for (int SA_id = 0; SA_id < GPIC_SA_NUM; SA_id++) {
        GPIC_host_device_cycles[SA_id] = 0;
        GPIC_move_stall_cycles[SA_id] = 0;
        GPIC_compute_cycles[SA_id] = 0;
        GPIC_memory_cycles[SA_id] = 0;
        GPIC_compute_energy[SA_id] = 0;
        GPIC_compute_comp_energy[SA_id] = 0;
        GPIC_compute_rdwr_energy[SA_id] = 0;
        last_gpic_instruction_computed[SA_id] = false;
        last_gpic_instruction_sent[SA_id] = false;
        assert(gpic_op_to_num_mem_op[SA_id].size() == 0);
        for (int i = 0; i < gpic_instruction_queue[SA_id].size(); i++) {
            printf("ERROR: %s remained in gpic instruction queue %s\n", gpic_instruction_queue[SA_id].at(0).second.c_str(), level_string.c_str());
        }
        assert(gpic_instruction_queue[SA_id].size() == 0);
        for (int i = 0; i < gpic_compute_queue[SA_id].size(); i++) {
            printf("ERROR: %s remained in gpic compute queue %s\n", gpic_compute_queue[SA_id].at(0).second.c_str(), level_string.c_str());
        }
        assert(gpic_compute_queue[SA_id].size() == 0);
    }

    last_id = 0;

    for (int idx = 0; idx < mshr_entries.size(); idx++) {
        auto first = mshr_entries[idx].first;
        auto second = mshr_entries[idx].second;
        printf("Error: pair(0x%lx, line(0x%lx, %d, %d, 0x%lx)) remained in mshr entries at level %s\n", first, second->addr, second->dirty, second->lock, second->tag, level_string.c_str());
    }
    assert(mshr_entries.size() == 0);
    assert(retry_list.size() == 0);
    assert(self_retry_list.size() == 0);
}

void CacheSystem::tick()
{
    debug("clk %ld", clk);

    ++clk;

    // Sends ready waiting request to memory
    auto it = wait_list.begin();
    while (it != wait_list.end() && clk >= it->first) {
        if (!send_memory(it->second)) {

            debug("failed sending %s to memory", (it->second).c_str());

            ++it;
        } else {

            debug("complete req: %s", (it->second).c_str());

            it = wait_list.erase(it);
        }
    }

    // hit request callback
    it = hit_list.begin();
    while (it != hit_list.end()) {
        if (clk >= it->first) {
            it->second.callback(it->second);

            debug("finish hit: %s", (it->second).c_str());

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
    auto it = wait_list.begin();
    while (it != wait_list.end()) {
        debug("%s\n", it->second.c_str());
        ++it;
    }
    assert(wait_list.size() == 0);

    it = hit_list.begin();
    while (it != hit_list.end()) {
        debug("%s\n", it->second.c_str());
        ++it;
    }
    assert(hit_list.size() == 0);
}

bool CacheSystem::finished()
{
    return (wait_list.size() == 0) && (hit_list.size() == 0);
}

} // namespace ramulator
