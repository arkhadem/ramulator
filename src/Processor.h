#ifndef __PROCESSOR_H
#define __PROCESSOR_H

#include "Cache.h"
#include "Config.h"
#include "Memory.h"
#include "Request.h"
#include "Statistics.h"
#include <ctype.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

// #define ADVANCED_ROB

namespace ramulator {

class Core;

class Trace {
public:
    Trace(vector<const char*> trace_fname);
    // trace file format 1:
    // [# of bubbles(non-mem instructions)] [read address(dec or hex)] <optional: write address(evicted cacheline)>
    bool get_gpic_request(long& bubble_cnt, std::string& req_opcode, int& req_en, long& req_addr, Request::Type& req_type, int& SA_id, int& SA_id_dst);
    bool get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type);
    bool get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type);
    // trace file format 2:
    // [address(hex)] [R/W]
    bool get_dramtrace_request(long& req_addr, Request::Type& req_type);

    long expected_limit_insts = 0;

private:
    int last_trace;
    vector<std::ifstream*> files;
    vector<bool> warmed_ups;
    vector<std::string> trace_names;
};

class Window {
public:
    int ipc = 4;
    int depth = 128;

    Window(Core* core)
        : ready_list(depth, false)
        , sent_list(depth, false)
        , addr_list(depth, -1)
        , req_list(depth, Request())
        , core(core)
    {
    }
    bool is_full();
    bool is_empty();
    void insert(bool ready, Request& req);
    void set_ready(Request req);
    void set_ready(Request req, int mask);
    long tick();
    void reset_state();

private:
    bool find_older_stores(long a_s, long a_e, Request::Type& type, int location);
    bool find_older_unsent(int SA_id, int SA_id_dst, int location);
#ifdef ADVANCED_ROB
    bool check_send(Request& req, int location);
#endif
    int load = 0;
    int head = 0;
    int tail = 0;
    long last_id = 0;
    std::vector<bool> ready_list;
    std::vector<bool> sent_list;
    std::vector<long> addr_list;
    std::vector<Request> req_list;
    vector<Request> retry_list;
    Core* core;
};

class Core {
public:
    long clk = 0;
    long retired = 0;
    int id = 0;
    function<bool(Request)> ls_send;
    function<bool(Request)> gpic_send;

    Core(const Config& configs, int coreid,
        const std::vector<const char*>& trace_fnames,
        function<bool(Request)> send_next, Cache* llc,
        std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory);
    void tick();
    void reset_state();
    void receive(Request& req);
    void reset_stats();
    double calc_ipc();
    bool finished();
    bool has_reached_limit();
    long get_insts(); // the number of the instructions issued to the core
    bool is_warmed_up();
    function<void(Request&)> callback;

    bool no_core_caches = true;
    bool no_shared_cache = true;
    bool gpic_mode = false;
    int gpic_level = 1;

    std::vector<std::shared_ptr<Cache>> caches;
    Cache* llc;

    ScalarStat record_cycs;
    ScalarStat record_insts;
    long expected_limit_insts;
    // This is set true iff expected number of instructions has been executed or all instructions are executed.
    bool reached_limit = false;

private:
    Trace trace;
    Window window;
    Request request;

    int SA_id = 0;
    int SA_id_dst = 0;
    long bubble_cnt = -1;
    long req_addr = -1;
    int req_en = -1;
    std::string req_opcode = "NULL";
    Request::Type req_type = Request::Type::MAX;
    bool more_reqs = true;
    long last = 0;

    Cache* first_level_cache = nullptr;

    ScalarStat memory_access_cycles;
    ScalarStat cpu_inst;
    MemoryBase& memory;

    bool warmed_up = false;
};

class Processor {
public:
    Processor(const Config& configs, vector<const char*> trace_list,
        function<bool(Request)> send, MemoryBase& memory);
    void tick();
    void reset_state();
    void receive(Request& req);
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

}
#endif /* __PROCESSOR_H */
