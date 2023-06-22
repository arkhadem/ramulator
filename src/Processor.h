#ifndef __PROCESSOR_H
#define __PROCESSOR_H

#include "Cache.h"
#include "Config.h"
#include "Memory.h"
#include "Request.h"
#include "Statistics.h"
#include <cstdio>
#include <ctype.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace ramulator {

class Core;

class Trace {
public:
    Trace(const char *trace_fname);
    // trace file format 1:
    // [# of bubbles(non-mem instructions)] [read address(dec or hex)] <optional: write address(evicted cacheline)>
    bool get_neon_request(std::string &req_opcode,
                          long &req_dst_reg_count, long &req_dst_mem_count, long &req_src_reg_count, long &req_src_mem_count,
                          long &req_dst_reg, long &req_dst_reg_type,
                          long &req_src1_reg, long &req_src1_reg_type,
                          long &req_src2_reg, long &req_src2_reg_type,
                          long &req_src3_reg, long &req_src3_reg_type,
                          long &req_addr, long &req_addr_end,
                          long &req_latency, long &req_guard,
                          long &pipeline_1, long &pipeline_2,
                          Request::Type &req_type,
                          long &bubble_cnt);

    bool get_unfiltered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type);
    bool get_filtered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type);
    // trace file format 2:
    // [address(hex)] [R/W]
    bool get_dramtrace_request(long &req_addr, Request::Type &req_type);

    long expected_limit_insts = 0;

private:
    std::ifstream file;
    std::string trace_name;
};

class Window {
public:
    int depth = 128;

    Window(Core *_core, bool _out_of_order, int _ipc)
        : out_of_order(_out_of_order), ipc(_ipc),
          dispatch_list(depth, Request()),
          core(_core) {
        printf("window ipc: %d\n", ipc);
        execute_v0_lists = std::vector<std::vector<std::pair<long, long>>>(NUM_V0_PIPELINES);
        execute_v1_lists = std::vector<std::vector<std::pair<long, long>>>(NUM_V1_PIPELINES);
        execute_l_lists = std::vector<long>(NUM_L_PIPELINES, -1);
        ld_to_mem_ops = std::vector<std::vector<long>>(NUM_L_PIPELINES);
    }
    bool is_dispatch_full();
    bool is_dispatch_empty();
    void set_ready(Request req);
    long tick();
    void reset_state();
    void dispatch(Request &req);
    void issue();
    void execute();
    long retire();
    long align(long addr);
    bool finished();

private:
    bool check_memory_dependency(int location);
    bool check_RAW_dependency(int location);
    int get_location(int location);
    int dispatch_load = 0;
    int dispatch_head = 0;
    int dispatch_tail = 0;
    long last_id = 0;
    bool out_of_order;
    int ipc = 0;

    std::vector<Request> dispatch_list;
    std::vector<long> issue_v_list;
    std::vector<long> issue_l_list;
    std::vector<std::vector<std::pair<long, long>>> execute_v0_lists;
    std::vector<std::vector<std::pair<long, long>>> execute_v1_lists;
    std::vector<long> execute_l_lists;
    std::vector<std::vector<long>> ld_to_mem_ops;
    Core *core;
};

class Core {
public:
    long clk = 0;
    long retired = 0;
    int id = 0;
    core_type_t core_type;
    int ipc = 0;
    bool out_of_order = true;
    function<bool(Request)> send;

    Core(const Config &configs, int coreid,
         const char *trace_fname,
         function<bool(Request)> send_next, Cache *llc,
         std::shared_ptr<CacheSystem> cachesys, MemoryBase &memory);
    void tick();
    void reset_state();
    void receive(Request &req);
    void reset_stats();
    double calc_ipc();
    bool finished();
    bool has_reached_limit();
    long get_insts(); // the number of the instructions issued to the core
    function<void(Request &)> callback;
    bool has_retired();

    bool no_core_caches = true;
    bool no_shared_cache = true;

    std::vector<std::shared_ptr<Cache>> caches;
    Cache *llc;

    ScalarStat record_cycs;
    ScalarStat record_insts;
    ScalarStat pipeline_v0_wait[NUM_V0_PIPELINES];
    ScalarStat pipeline_v0_exec[NUM_V0_PIPELINES];
    ScalarStat pipeline_v1_wait[NUM_V1_PIPELINES];
    ScalarStat pipeline_v1_exec[NUM_V1_PIPELINES];
    ScalarStat pipeline_l_wait[NUM_L_PIPELINES];
    ScalarStat pipeline_l_exec[NUM_L_PIPELINES];
    long expected_limit_insts;
    // This is set true iff expected number of instructions has been executed or all instructions are executed.
    bool reached_limit = false;

private:
    Trace trace;
    char *my_trace_fname;
    Window window;
    Request request;

    std::string req_opcode = "NULL";
    long req_dst_reg_count = -1;
    long req_dst_mem_count = -1;
    long req_src_reg_count = -1;
    long req_src_mem_count = -1;
    long req_dst_reg = -1;
    long req_dst_reg_type = -1;
    long req_src1_reg = -1;
    long req_src1_reg_type = -1;
    long req_src2_reg = -1;
    long req_src2_reg_type = -1;
    long req_src3_reg = -1;
    long req_src3_reg_type = -1;
    long req_addr = -1;
    long req_addr_end = -1;
    long req_latency = -1;
    long req_guard = -1;
    long bubble_cnt = -1;
    long pipeline_1 = -1;
    long pipeline_2 = -1;

    Request::Type req_type = Request::Type::MAX;
    bool more_reqs = true;
    long last = 0;

    bool did_retired;

    Cache *first_level_cache = nullptr;

    ScalarStat memory_access_cycles;
    ScalarStat cpu_inst;
    MemoryBase &memory;
};

class Processor {
public:
    Processor(const Config &configs, vector<const char *> trace_list,
              function<bool(Request)> send, MemoryBase &memory);
    void tick();
    void receive(Request &req);
    void reset_state();
    void reset_stats();
    bool finished();
    bool has_reached_limit();
    long get_insts(); // the total number of instructions issued to all cores

    std::vector<std::unique_ptr<Core>> cores;
    std::vector<double> ipcs;
    double ipc = 0;

    // When early_exit is true, the simulation exits when the earliest trace finishes.
    bool early_exit;

    bool no_core_caches = true;
    bool no_shared_cache = true;

    std::shared_ptr<CacheSystem> cachesys;
    Cache llc;

    ScalarStat cpu_cycles;
};

} // namespace ramulator
#endif /* __PROCESSOR_H */
