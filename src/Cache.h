#ifndef __CACHE_H
#define __CACHE_H

#include "Config.h"
#include "Request.h"
#include "Statistics.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#define MAX_GPIC_QUEUE_SIZE 32
// #define DEBUG_CACHE

namespace ramulator {
class CacheSystem;

// These variables are set in Main.cpp
extern int l1_size;
extern int l1_assoc;
extern int l1_blocksz;
extern int l1_mshr_num;
extern float l1_access_energy;

extern int l2_size;
extern int l2_assoc;
extern int l2_blocksz;
extern int l2_mshr_num;
extern float l2_access_energy;

extern int l3_size;
extern int l3_assoc;
extern int l3_blocksz;
extern int mshr_per_bank;
extern float l3_access_energy;

class Cache {
protected:
    ScalarStat cache_read_miss;
    ScalarStat cache_write_miss;
    ScalarStat cache_total_miss;
    ScalarStat cache_eviction;
    ScalarStat cache_read_access;
    ScalarStat cache_write_access;
    ScalarStat cache_total_access;
    ScalarStat cache_mshr_hit;
    ScalarStat cache_mshr_unavailable;
    ScalarStat cache_set_unavailable;
    ScalarStat cache_access_energy;

    ScalarStat GPIC_compute_energy[GPIC_SA_NUM];
    ScalarStat GPIC_compute_comp_energy[GPIC_SA_NUM];
    ScalarStat GPIC_compute_rdwr_energy[GPIC_SA_NUM];
    ScalarStat GPIC_compute_total_energy;
    ScalarStat GPIC_compute_comp_total_energy;
    ScalarStat GPIC_compute_rdwr_total_energy;

    ScalarStat GPIC_host_device_total_cycles;
    ScalarStat GPIC_move_stall_total_cycles;
    ScalarStat GPIC_compute_total_cycles;
    ScalarStat GPIC_memory_total_cycles;
    ScalarStat GPIC_host_device_cycles[GPIC_SA_NUM];
    ScalarStat GPIC_move_stall_cycles[GPIC_SA_NUM];
    ScalarStat GPIC_compute_cycles[GPIC_SA_NUM];
    ScalarStat GPIC_memory_cycles[GPIC_SA_NUM];

public:
    enum class Level {
        CORE,
        L1,
        L2,
        L3,
        MAX
    } level;
    std::string level_string;

    struct Line {
        long addr;
        long tag;
        bool lock; // When the lock is on, the value is not valid yet.
        bool dirty;
        Line(long addr, long tag)
            : addr(addr)
            , tag(tag)
            , lock(true)
            , dirty(false)
        {
        }
        Line(long addr, long tag, bool lock, bool dirty)
            : addr(addr)
            , tag(tag)
            , lock(lock)
            , dirty(dirty)
        {
        }
    };

    Cache(int size, int assoc, int block_size, int mshr_entry_num, float access_energy,
        Level level, std::shared_ptr<CacheSystem> cachesys, int core_id = -1);

    void tick();
    void reset_state();

    // L1, L2, L3 accumulated latencies
    int latency_each[int(Level::MAX)] = { 0, 4, 12, 31 };

    std::shared_ptr<CacheSystem> cachesys;
    // LLC has multiple higher caches
    std::vector<Cache*> higher_cache;
    Cache* lower_cache;

    bool send(Request req);

    void concatlower(Cache* lower);

    void callback(Request& req);

    function<void(Request&)> processor_callback;

    bool finished();

    int vid_to_sid(int vid, int base);

    bool check_full_queue(Request req);

    bool memory_controller(Request req);

    void instrinsic_decoder(Request req);

    void callbacker(Request& req);

protected:
    int core_id;
    bool is_first_level;
    bool is_last_level;
    size_t size;
    unsigned int assoc;
    unsigned int block_num;
    unsigned int index_mask;
    unsigned int block_size;
    unsigned int index_offset;
    unsigned int tag_offset;
    unsigned int mshr_entry_num;
    float access_energy;
    long last_id = 0;
    std::vector<std::pair<long, std::list<Line>::iterator>> mshr_entries;

    std::vector<Request> self_retry_list;

    std::list<std::pair<long, Request>> retry_list;

    std::map<int, std::list<Line>> cache_lines;

    std::map<std::string, long> GPIC_COMPUTE_DELAY;
    std::map<std::string, long> GPIC_ACCESS_DELAY;
    std::map<std::string, long> DC_COMPUTE_DELAY;
    std::map<std::string, long> DC_ACCESS_DELAY;

    void init_intrinsic_latency();

    std::map<Request, std::vector<long>> gpic_op_to_mem_ops[GPIC_SA_NUM];
    std::vector<std::pair<long, Request>> gpic_instruction_queue[GPIC_SA_NUM];
    std::vector<std::pair<long, Request>> gpic_compute_queue[GPIC_SA_NUM];
    long last_gpic_instruction_compute_clk[GPIC_SA_NUM];
    bool last_gpic_instruction_computed[GPIC_SA_NUM];
    bool last_gpic_instruction_sent[GPIC_SA_NUM];
    std::map<Request, int> gpic_vop_to_num_sop;

    long VL_reg = GPIC_SA_NUM * 256;
    long VC_reg = 1;
    long LS_reg = 0;
    long SS_reg = 0;
    int V_PER_SA = 1;
    int SA_PER_V = GPIC_SA_NUM;

    int calc_log2(int val)
    {
        int n = 0;
        while ((val >>= 1))
            n++;
        return n;
    }

    int get_index(long addr)
    {
        return (addr >> index_offset) & index_mask;
    };

    long get_tag(long addr)
    {
        return (addr >> tag_offset);
    }

    // Align the address to cache line size
    long align(long addr)
    {
        return (addr & ~(block_size - 1l));
    }

    // Evict the cache line from higher level to this level.
    // Pass the dirty bit and update LRU queue.
    void evictline(long addr, bool dirty);

    // Invalidate the line from this level to higher levels
    // The return value is a pair. The first element is invalidation
    // latency, and the second is wether the value has new version
    // in higher level and this level.
    std::pair<long, bool> invalidate(long addr);

    // Evict the victim from current set of lines.
    // First do invalidation, then call evictline(L1 or L2) or send
    // a write request to memory(L3) when dirty bit is on.
    void evict(std::list<Line>* lines,
        std::list<Line>::iterator victim);

    // First test whether need eviction, if so, do eviction by
    // calling evict function. Then allocate a new line and return
    // the iterator points to it.
    std::list<Line>::iterator allocate_line(
        std::list<Line>& lines, long addr);

    // Check whether the set to hold addr has space or eviction is
    // needed.
    bool need_eviction(const std::list<Line>& lines, long addr);

    // Check whether this addr is hit and fill in the pos_ptr with
    // the iterator to the hit line or lines.end()
    bool is_hit(std::list<Line>& lines, long addr,
        std::list<Line>::iterator* pos_ptr);

    bool all_sets_locked(const std::list<Line>& lines)
    {
        if (lines.size() < assoc) {
            return false;
        }
        for (const auto& line : lines) {
            if (!line.lock) {
                return false;
            }
        }
        return true;
    }

    bool check_unlock(long addr)
    {
        auto it = cache_lines.find(get_index(addr));
        if (it == cache_lines.end()) {
            return true;
        } else {
            auto& lines = it->second;
            auto line = find_if(lines.begin(), lines.end(),
                [addr, this](Line l) { return (l.tag == get_tag(addr)); });
            if (line == lines.end()) {
                return true;
            } else {
                bool check = !line->lock;
                if (!is_first_level) {
                    for (auto hc : higher_cache) {
                        if (!check) {
                            return check;
                        }
                        check = check && hc->check_unlock(line->addr);
                    }
                }
                return check;
            }
        }
    }

    std::vector<std::pair<long, std::list<Line>::iterator>>::iterator
    hit_mshr(long addr)
    {
        auto mshr_it = find_if(mshr_entries.begin(), mshr_entries.end(),
            [addr, this](std::pair<long, std::list<Line>::iterator>
                    mshr_entry) {
                return (align(mshr_entry.first) == align(addr));
            });
        return mshr_it;
    }

    std::list<Line>& get_lines(long addr)
    {
        if (cache_lines.find(get_index(addr)) == cache_lines.end()) {
            cache_lines.insert(make_pair(get_index(addr),
                std::list<Line>()));
        }
        return cache_lines[get_index(addr)];
    }
};

class CacheSystem {
public:
    CacheSystem(const Config& configs, std::function<bool(Request)> send_memory)
        : send_memory(send_memory)
    {
        if (configs.has_core_caches()) {
            first_level = Cache::Level::L1;
        } else if (configs.has_l3_cache()) {
            first_level = Cache::Level::L3;
        } else {
            last_level = Cache::Level::MAX; // no cache
        }

        if (configs.has_l3_cache()) {
            last_level = Cache::Level::L3;
        } else if (configs.has_core_caches()) {
            last_level = Cache::Level::L2;
        } else {
            last_level = Cache::Level::MAX; // no cache
        }
    }

    // wait_list contains miss requests with their latencies in
    // cache. When this latency is met, the send_memory function
    // will be called to send the request to the memory system.
    std::list<std::pair<long, Request>> wait_list;

    // hit_list contains hit requests with their latencies in cache.
    // callback function will be called when this latency is met and
    // set the instruction status to ready in processor's window.
    std::list<std::pair<long, Request>> hit_list;

    std::function<bool(Request)> send_memory;

    long clk = 0;
    void tick();
    void reset_state();
    bool finished();

    Cache::Level first_level;
    Cache::Level last_level;
};

} // namespace ramulator

#endif /* __CACHE_H */
