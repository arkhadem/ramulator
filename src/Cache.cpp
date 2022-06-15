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

    for (int i = 0; i < GPIC_SA_NUM; i++) {
        last_gpic_instruction_compute_clk[i] = -1;
        last_gpic_instruction_computed[i] = false;
        last_gpic_instruction_sent[i] = false;
    }

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

    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        GPIC_host_device_cycles[sid].name(level_string + string("_GPIC_host_device_cycles[") + to_string(sid) + string("]")).desc("cache GPIC instruction queue is empty").precision(0);
        GPIC_move_stall_cycles[sid].name(level_string + string("_GPIC_move_stall_cycles[") + to_string(sid) + string("]")).desc("cache GPIC is stalled for move instruction").precision(0);
        GPIC_compute_cycles[sid].name(level_string + string("_GPIC_compute_cycles[") + to_string(sid) + string("]")).desc("cache GPIC doing computation or W/R").precision(0);
        GPIC_memory_cycles[sid].name(level_string + string("_GPIC_memory_cycles[") + to_string(sid) + string("]")).desc("cache GPIC waiting for mem requests").precision(0);
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

    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        GPIC_compute_energy[sid].name(level_string + string("_GPIC_compute_energy[") + to_string(sid) + string("]")).desc("cache GPIC compute energy in pJ").precision(0);
        GPIC_compute_comp_energy[sid].name(level_string + string("_GPIC_compute_comp_energy[") + to_string(sid) + string("]")).desc("cache GPIC compute energy [compute part] in pJ").precision(0);
        GPIC_compute_rdwr_energy[sid].name(level_string + string("_GPIC_compute_rdwr_energy[") + to_string(sid) + string("]")).desc("cache GPIC compute energy [read/write part] in pJ").precision(0);
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

int Cache::vid_to_sid(int vid, int base = 0)
{
    if (VL_reg <= 256) {
        return vid / V_PER_SA + base;
    } else {
        return vid * SA_PER_V + base;
    }
}

bool Cache::check_full_queue(Request req)
{
    if (req.vid == -1) {
        assert(req.opcode.find("move") == string::npos);
        for (int vid = 0; vid < VC_reg; vid += V_PER_SA) {
            for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
                if (gpic_instruction_queue[vid_to_sid(vid, sid_offset)].size() >= MAX_GPIC_QUEUE_SIZE) {
                    return false;
                }
            }
        }
    } else {
        for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
            if (gpic_instruction_queue[vid_to_sid(req.vid, sid_offset)].size() >= MAX_GPIC_QUEUE_SIZE) {
                return false;
            }
        }
    }

    if (req.opcode.find("move") != string::npos) {
        for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
            if (gpic_instruction_queue[vid_to_sid(req.vid_dst, sid_offset)].size() >= MAX_GPIC_QUEUE_SIZE) {
                return false;
            }
        }
    }
    return true;
}

void Cache::instrinsic_decoder(Request req)
{

    assert(gpic_vop_to_num_sop.count(req) == 0);

    if (req.opcode.find("move") != string::npos) {
        // move cannot have vid = -1
        assert(req.vid >= 0);
        gpic_vop_to_num_sop[req] = SA_PER_V;
        if ((req.vid / V_PER_SA) != (req.vid_dst / V_PER_SA)) {
            gpic_vop_to_num_sop[req] *= 2;
        }
        // For each SA of the vector
        for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
            req.sid = vid_to_sid(req.vid, sid_offset);
            req.sid_dst = vid_to_sid(req.vid_dst, sid_offset);
            hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
            gpic_instruction_queue[req.sid].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
            if (req.sid != req.sid_dst) {
                gpic_instruction_queue[req.sid_dst].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
            }
        }
    } else if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos)) {

        std::vector<long> addr_starts = req.addr_starts;
        std::vector<long> addr_ends = req.addr_ends;

        if (req.vid == -1) {
            gpic_vop_to_num_sop[req] = ((VC_reg - 1) / V_PER_SA) + 1;
            gpic_vop_to_num_sop[req] *= SA_PER_V;

            // For each vector
            for (int vid_base = 0; vid_base < VC_reg; vid_base += V_PER_SA) {
                // For each SA of the vector
                for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
                    req.addr_starts.clear();
                    req.addr_ends.clear();
                    req.sid = vid_to_sid(vid_base, sid_offset);

                    // For each vector of the SA
                    int remaining_vectors = (V_PER_SA < (VC_reg - vid_base)) ? (V_PER_SA) : (VC_reg - vid_base);
                    for (int vid_offset = 0; vid_offset < remaining_vectors; vid_offset++) {
                        // VID shows which address pair should be used
                        int vid = vid_base + vid_offset;

                        if ((req.opcode.find("load1") != string::npos) || (req.opcode.find("store1") != string::npos)) {
                            // It's a load1
                            req.addr_starts.push_back(addr_starts[vid]);
                            req.addr_ends.push_back(addr_ends[vid]);
                        } else {
                            // It's an ordinary load or store
                            long addr_start = addr_starts[vid] + (long)(std::ceil((float)(sid_offset * 256 * req.data_type / 8)));
                            long addr_end = min(((long)(std::ceil((float)(256 * req.data_type / 8))) + addr_start - 1), addr_ends[vid]);
                            req.addr_starts.push_back(addr_start);
                            req.addr_ends.push_back(addr_end);
                        }
                        if (vid_offset == 0) {
                            req.addr = req.addr_starts[0];
                        }
                    }

                    // Schedule the instruction
                    hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
                    gpic_instruction_queue[req.sid].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
                }
            }
        } else {
            assert((addr_starts.size() == 1) && (addr_ends.size() == 1));
            gpic_vop_to_num_sop[req] = SA_PER_V;

            // For each SA of the vector
            for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
                req.sid = vid_to_sid(req.vid, sid_offset);

                if ((req.opcode.find("load1") == string::npos) && (req.opcode.find("store1") == string::npos)) {
                    // It's an ordinary load or store
                    req.addr_starts.clear();
                    req.addr_ends.clear();
                    long addr_start = addr_starts[0] + (long)(std::ceil((float)(sid_offset * 256 * req.data_type / 8)));
                    long addr_end = min(((long)(std::ceil((float)(256 * req.data_type / 8))) + addr_start - 1), addr_ends[0]);
                    req.addr_starts.push_back(addr_start);
                    req.addr_ends.push_back(addr_end);
                    req.addr = req.addr_starts[0];
                }

                // Schedule the instruction
                hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
                gpic_instruction_queue[req.sid].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
            }
        }
    } else {
        if (req.vid == -1) {
            gpic_vop_to_num_sop[req] = ((VC_reg - 1) / V_PER_SA) + 1;
            gpic_vop_to_num_sop[req] *= SA_PER_V;

            // For each vector
            for (int vid_base = 0; vid_base < VC_reg; vid_base += V_PER_SA) {
                // For each SA of the vector
                for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
                    req.sid = vid_to_sid(vid_base, sid_offset);
                    // Schedule the instruction
                    hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
                    gpic_instruction_queue[req.sid].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
                }
            }
        } else {
            gpic_vop_to_num_sop[req] = SA_PER_V;
            // For each SA of the vector
            for (int sid_offset = 0; sid_offset < SA_PER_V; sid_offset++) {
                req.sid = vid_to_sid(req.vid, sid_offset);
                // Schedule the instruction
                hint("%s set for start in %d clock cycles\n", req.c_str(), latency_each[int(level)]);
                gpic_instruction_queue[req.sid].push_back(make_pair(cachesys->clk + latency_each[int(level)], req));
            }
        }
    }
}

bool Cache::memory_controller(Request req)
{
    if (req.opcode.find("_set_") != string::npos) {
        // it's a config GPIC instruction
        if (req.opcode.find("stride") != string::npos) {
            if (req.opcode.find("load") != string::npos) {
                LS_reg = req.value;
            } else if (req.opcode.find("store") != string::npos) {
                SS_reg = req.value;
            } else {
                assert(false);
            }
        } else if (req.opcode.find("vector") != string::npos) {
            if (req.opcode.find("length") != string::npos) {
                VL_reg = (req.value == 0) ? (GPIC_SA_NUM * 256) : req.value;
                if (VL_reg <= 256) {
                    if ((128 < VL_reg) && (VL_reg <= 256))
                        V_PER_SA = 1;
                    else if ((64 < VL_reg) && (VL_reg <= 128))
                        V_PER_SA = 2;
                    else if ((32 < VL_reg) && (VL_reg <= 64))
                        V_PER_SA = 4;
                    else if ((0 < VL_reg) && (VL_reg <= 32))
                        V_PER_SA = 8;
                    else
                        assert(false);
                    SA_PER_V = 1;
                } else {
                    SA_PER_V = ((VL_reg - 1) / 256) + 1;
                    V_PER_SA = 1;
                }
            } else if (req.opcode.find("count") != string::npos) {
                VC_reg = (req.value == 0) ? 1 : req.value;
            } else {
                assert(false);
            }
        } else {
            assert(false);
        }
        hint("LS_reg: %ld, SS_reg: %ld, VL_reg: %ld, VC_reg: %ld, V_PER_SA: %d, SA_PER_V: %d\n", LS_reg, SS_reg, VL_reg, VC_reg, V_PER_SA, SA_PER_V);
    } else {

        assert(((((VC_reg * SA_PER_V) + 1) / V_PER_SA) - 1) <= GPIC_SA_NUM);

        if (check_full_queue(req) == false)
            return false;

        instrinsic_decoder(req);
    }
    return true;
}

bool Cache::send(Request req)
{
    if (req.type == Request::Type::GPIC) {
        debug("level %s received %s", level_string.c_str(), req.c_str());

        return memory_controller(req);
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

void Cache::evict(std::list<Line>* lines, std::list<Line>::iterator victim)
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

std::list<Cache::Line>::iterator Cache::allocate_line(std::list<Line>& lines, long addr)
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

bool Cache::is_hit(std::list<Line>& lines, long addr, std::list<Line>::iterator* pos_ptr)
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
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {

        // Check if the SA has sent the memory operations
        if ((last_gpic_instruction_computed[sid] == true) && (last_gpic_instruction_sent[sid] == true)) {
            Request gpic_req = gpic_compute_queue[sid][0].second;

            // Check all start-end address pairs
            for (int gpic_idx = 0; gpic_idx < gpic_req.addr_starts.size(); gpic_idx++) {

                // If the address has overlap with the start-end pair
                if ((align(req.addr) >= align(gpic_req.addr_starts[gpic_idx])) && (align(req.addr) <= align(gpic_req.addr_ends[gpic_idx]))) {

                    // Check if this memory access has been ocurred because of this GPIC instruction
                    for (int mem_idx = 0; mem_idx < gpic_op_to_mem_ops[sid][gpic_req].size(); mem_idx++) {

                        if (align(req.addr) == align(gpic_op_to_mem_ops[sid][gpic_req][mem_idx])) {
                            hint("%s: %s calls back for %s, %lu instructions remained\n", level_string.c_str(), req.c_str(), gpic_req.c_str(), gpic_op_to_mem_ops[sid][gpic_req].size() - 1);

                            // Remove this instruction from gpic list
                            gpic_op_to_mem_ops[sid][gpic_req].erase(gpic_op_to_mem_ops[sid][gpic_req].begin() + mem_idx);
                        }
                    }
                }
            }
            if (gpic_op_to_mem_ops[sid][gpic_req].size() == 0) {
                op_trace << cachesys->clk << " " << sid << " F " << gpic_req.opcode << endl;
                hint("%s: calling back %s\n", level_string.c_str(), gpic_req.c_str());
                callbacker(gpic_req);
                gpic_op_to_mem_ops[sid].erase(gpic_req);
                gpic_compute_queue[sid].erase(gpic_compute_queue[sid].begin());
                last_gpic_instruction_compute_clk[sid] = -1;
                last_gpic_instruction_computed[sid] = false;
                last_gpic_instruction_sent[sid] = false;
            }
        }
    }

    if (higher_cache.size()) {
        for (auto hc : higher_cache) {
            hc->callback(req);
        }
    }
}

void Cache::callbacker(Request& req)
{
    assert(gpic_vop_to_num_sop.count(req) == 1);
    assert(gpic_vop_to_num_sop[req] > 0);
    gpic_vop_to_num_sop[req] -= 1;
    if (gpic_vop_to_num_sop[req] == 0) {
        hint("Calling back %s to core\n", req.c_str());
        req.callback(req);
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
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        while ((gpic_instruction_queue[sid].size() > 0) && (cachesys->clk >= gpic_instruction_queue[sid][0].first)) {
            Request req = gpic_instruction_queue[sid][0].second;
            long compute_delay, access_delay, bitlines;
            if (req.opcode.find("_dc_") != string::npos) {
                assert(req.opcode.find("_pc2_") == string::npos);
                compute_delay = DC_COMPUTE_DELAY[req.opcode];
                access_delay = DC_ACCESS_DELAY[req.opcode];
                bitlines = 4;
            } else {
                assert(req.opcode.find("_pc2_") != string::npos);
                compute_delay = GPIC_COMPUTE_DELAY[req.opcode];
                access_delay = GPIC_ACCESS_DELAY[req.opcode];
                bitlines = 1;
            }
            // printf("%s %d %d\n", req.opcode.c_str(), compute_delay, access_delay);
            assert((compute_delay + access_delay) > 0);
            hint("%s set for compute in %ld clock cycles\n", req.c_str(), compute_delay + access_delay);
            gpic_instruction_queue[sid].erase(gpic_instruction_queue[sid].begin());
            gpic_compute_queue[sid].push_back(make_pair(compute_delay + access_delay, req));
            GPIC_compute_total_energy += ((float)compute_delay * 15.4 + (float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_comp_total_energy += ((float)compute_delay * 15.4) * (float)bitlines;
            GPIC_compute_rdwr_total_energy += ((float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_energy[sid] += ((float)compute_delay * 15.4 + (float)access_delay * 8.6) * (float)bitlines;
            GPIC_compute_comp_energy[sid] += ((float)compute_delay * 15.4) * (float)bitlines;
            GPIC_compute_rdwr_energy[sid] += ((float)access_delay * 8.6) * (float)bitlines;
        }
    }

    // Instruction at the head of the GPIC queue is computed
    // If it's a store or load it is unpacked
    // Otherwise, it is called back
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {

        // Check if there is any instruction ready for completion
        if ((last_gpic_instruction_computed[sid] == true) && (last_gpic_instruction_sent[sid] == false)) {

            // There must be an instruction going on
            assert(gpic_compute_queue[sid].size() != 0);

            // Check if the instruction is done
            if (cachesys->clk - last_gpic_instruction_compute_clk[sid] >= gpic_compute_queue[sid].at(0).first) {
                Request req = gpic_compute_queue[sid].at(0).second;

                if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos)) {
                    // If it's a load or store, make new queries and send to this cache level's queue

                    assert(gpic_op_to_mem_ops[sid].count(req) == 0);
                    gpic_op_to_mem_ops[sid][req] = std::vector<long>();
                    for (int idx = 0; idx < req.addr_starts.size(); idx++) {
                        long lower_cache_line = align(req.addr_starts[idx]);
                        long upper_cache_line = align(req.addr_ends[idx]);
                        int access_needed = (long)(std::ceil((float)(upper_cache_line - lower_cache_line) / (float)(block_size))) + 1;
                        hint("10- %s unpacking %d instructions for %s\n", level_string.c_str(), access_needed, req.c_str());
                        assert(access_needed > 0);
                        long req_addr = lower_cache_line;
                        for (int i = 0; i < access_needed; i++) {

                            // Check if this memory access has happenned before
                            if (std::count(gpic_op_to_mem_ops[sid][req].begin(), gpic_op_to_mem_ops[sid][req].end(), req_addr)) {
                                hint("10- %s NOT sending 0x%lx to %s\n", level_string.c_str(), req_addr, level_string.c_str());
                            } else {
                                // make the request
                                Request::Type req_type = (req.opcode.find("load") != string::npos) ? Request::Type::READ : Request::Type::WRITE;
                                int req_coreid = req.coreid;
                                Request::UnitID req_unitid = (Request::UnitID)(level);
                                Request mem_req(req_addr, req_type, processor_callback, req_coreid, req_unitid);
                                mem_req.reqid = last_id;
                                last_id++;
                                gpic_op_to_mem_ops[sid][req].push_back(req_addr);
                                req_addr += block_size;

                                // send it
                                hint("10- %s sending %s to %s\n", level_string.c_str(), mem_req.c_str(), level_string.c_str());
                                if (send(mem_req) == false) {
                                    self_retry_list.push_back(mem_req);
                                }
                            }
                        }
                    }
                    last_gpic_instruction_sent[sid] = true;
                } else {
                    // Otherwise, we are ready to send the call back
                    op_trace << cachesys->clk << " " << sid << " F " << req.opcode << endl;

                    hint("10- %s instruction %s completed\n", level_string.c_str(), req.c_str());
                    callbacker(req);
                    gpic_compute_queue[sid].erase(gpic_compute_queue[sid].begin());
                    last_gpic_instruction_compute_clk[sid] = -1;
                    last_gpic_instruction_computed[sid] = false;
                    last_gpic_instruction_sent[sid] = false;
                }
            }
        }
    }

    // Instruction at the head of the GPIC queue gets ready for compute
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        // Check if there is any instructions ready to be computed
        if (last_gpic_instruction_computed[sid] == false) {

            // The last instruction must not be sent
            assert(last_gpic_instruction_sent[sid] == false);

            if (gpic_compute_queue[sid].size() != 0) {
                // A new instruction must be computed
                if (gpic_compute_queue[sid].at(0).second.opcode.find("move") != string::npos) {

                    // we should wait for the dst SA as well
                    int sid_src = gpic_compute_queue[sid].at(0).second.sid;
                    int sid_dst = gpic_compute_queue[sid].at(0).second.sid_dst;
                    assert(sid_dst != -1);

                    if (sid_src == sid_dst) {
                        op_trace << cachesys->clk << " " << sid << " S " << gpic_compute_queue[sid].at(0).second.opcode << endl;
                        hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[sid].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[sid].size());
                        last_gpic_instruction_compute_clk[sid] = cachesys->clk;
                        last_gpic_instruction_computed[sid] = true;
                    } else if (sid != sid_dst) {
                        // only src SA checks for the dst, not vice versa

                        // Check if there is any instructions ready to be computed in the dst queue
                        if (last_gpic_instruction_computed[sid_dst] == false) {

                            // The last instruction must not be sent
                            assert(last_gpic_instruction_sent[sid_dst] == false);

                            if (gpic_compute_queue[sid_dst].size() != 0) {

                                // Check if these are the same instructions
                                if (gpic_compute_queue[sid].at(0).second == gpic_compute_queue[sid_dst].at(0).second) {

                                    // Compute both
                                    op_trace << cachesys->clk << " " << sid << " S " << gpic_compute_queue[sid].at(0).second.opcode << endl;
                                    op_trace << cachesys->clk << " " << sid_dst << " S " << gpic_compute_queue[sid].at(0).second.opcode << endl;
                                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[sid].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[sid].size());
                                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[sid_dst].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[sid_dst].size());
                                    last_gpic_instruction_compute_clk[sid] = cachesys->clk;
                                    last_gpic_instruction_compute_clk[sid_dst] = cachesys->clk;
                                    last_gpic_instruction_computed[sid] = true;
                                    last_gpic_instruction_computed[sid_dst] = true;
                                }
                            }
                        }

                        if (last_gpic_instruction_computed[sid] == false) {
                            hint("%s is waiting for dst SA\n", gpic_compute_queue[sid].at(0).second.c_str());
                        }
                    }

                } else {
                    op_trace << cachesys->clk << " " << sid << " S " << gpic_compute_queue[sid].at(0).second.opcode << endl;
                    hint("Computing %s at %ld, %zu instructions in compute queue\n", gpic_compute_queue[sid].at(0).second.c_str(), cachesys->clk, gpic_compute_queue[sid].size());
                    last_gpic_instruction_compute_clk[sid] = cachesys->clk;
                    last_gpic_instruction_computed[sid] = true;
                }
            }
        }
    }

    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        if ((last_gpic_instruction_computed[sid] == false) && (last_gpic_instruction_sent[sid] == false)) {

            if (gpic_compute_queue[sid].size() == 0) {
                // There is no instruction ready for execute
                GPIC_host_device_total_cycles++;
                GPIC_host_device_cycles[sid]++;
            } else {
                // Instruction at top must be move and stalled by dst SA
                assert(gpic_compute_queue[sid].at(0).second.opcode.find("move") != string::npos);
                GPIC_move_stall_total_cycles++;
                GPIC_move_stall_cycles[sid]++;
            }
        }

        if ((last_gpic_instruction_computed[sid] == true) && (last_gpic_instruction_sent[sid] == false)) {
            // An instruction is being computed
            GPIC_compute_total_cycles++;
            GPIC_compute_cycles[sid]++;
        }

        if ((last_gpic_instruction_computed[sid] == true) && (last_gpic_instruction_sent[sid] == true)) {
            // An instruction is computed and sent, waiting for memory instructions to be called back
            GPIC_memory_total_cycles++;
            GPIC_memory_cycles[sid]++;
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
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        if (gpic_op_to_mem_ops[sid].size() != 0)
            return false;

        if (gpic_instruction_queue[sid].size() != 0)
            return false;

        if (gpic_compute_queue[sid].size() != 0)
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
    for (int sid = 0; sid < GPIC_SA_NUM; sid++) {
        GPIC_host_device_cycles[sid] = 0;
        GPIC_move_stall_cycles[sid] = 0;
        GPIC_compute_cycles[sid] = 0;
        GPIC_memory_cycles[sid] = 0;
        GPIC_compute_energy[sid] = 0;
        GPIC_compute_comp_energy[sid] = 0;
        GPIC_compute_rdwr_energy[sid] = 0;
        last_gpic_instruction_computed[sid] = false;
        last_gpic_instruction_sent[sid] = false;
        assert(gpic_op_to_mem_ops[sid].size() == 0);
        for (int i = 0; i < gpic_instruction_queue[sid].size(); i++) {
            printf("ERROR: %s remained in gpic instruction queue %s\n", gpic_instruction_queue[sid].at(0).second.c_str(), level_string.c_str());
        }
        assert(gpic_instruction_queue[sid].size() == 0);
        for (int i = 0; i < gpic_compute_queue[sid].size(); i++) {
            printf("ERROR: %s remained in gpic compute queue %s\n", gpic_compute_queue[sid].at(0).second.c_str(), level_string.c_str());
        }
        assert(gpic_compute_queue[sid].size() == 0);
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
