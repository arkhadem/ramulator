#include "Processor.h"
#include "Cache.h"
#include "Config.h"
#include "Request.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <ios>
#include <string>
#include <vector>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config &configs,
                     vector<const char *> trace_list,
                     function<bool(Request)> send_memory,
                     MemoryBase &memory)
    : ipcs(trace_list.size(), -1),
      early_exit(configs.is_early_exit()),
      no_core_caches(!configs.has_core_caches()),
      no_shared_cache(!configs.has_l3_cache()),
      cachesys(new CacheSystem(configs, send_memory)),
      llc(l3_size, l3_assoc, l3_blocksz,
          mshr_per_bank * trace_list.size(),
          Cache::Level::L3, cachesys) {

    assert(cachesys != nullptr);
    int tracenum = trace_list.size();
    assert(tracenum > 0);
    printf("tracenum: %d\n", tracenum);
    for (int i = 0; i < tracenum; ++i) {
        printf("trace_list[%d]: %s\n", i, trace_list[i]);
    }
    if (no_shared_cache) {
        for (int i = 0; i < tracenum; ++i) {
            cores.emplace_back(new Core(
                configs, i, trace_list[i], send_memory, nullptr,
                cachesys, memory));
        }
    } else {
        for (int i = 0; i < tracenum; ++i) {
            cores.emplace_back(new Core(configs, i, trace_list[i],
                                        std::bind(&Cache::send, &llc, std::placeholders::_1),
                                        &llc, cachesys, memory));
        }
    }
    for (int i = 0; i < tracenum; ++i) {
        cores[i]->callback = std::bind(&Processor::receive, this,
                                       placeholders::_1);
    }

    // regStats
    cpu_cycles.name("cpu_cycles")
        .desc("cpu cycle number")
        .precision(0);
    cpu_cycles = 0;
}

void Processor::tick() {
    cpu_cycles++;

    if ((int(cpu_cycles.value()) % 1000000) == 0) {
        printf("CPU heartbeat, cycles: %d \n", (int(cpu_cycles.value())));
        for (unsigned int i = 0; i < cores.size(); ++i) {
            Core *core = cores[i].get();
            if (core->has_retired() == false) {
                printf("No instruction retired since the last 1000000 clock cycles of core %d\n", i);
                exit(-1);
            }
        }
    }

    for (unsigned int i = 0; i < cores.size(); ++i) {
        Core *core = cores[i].get();
        core->tick();
    }

    if (!no_shared_cache) {
        llc.tick();
    }

    if (!(no_core_caches && no_shared_cache)) {
        cachesys->tick();
    }
}

void Processor::reset_state() {
    hint("Processor state reset\n");
    cpu_cycles = 0;
    ipc = 0;
    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;

    for (unsigned int i = 0; i < cores.size(); ++i) {
        Core *core = cores[i].get();
        core->reset_state();
    }

    if (!no_shared_cache) {
        llc.reset_state();
    }

    if (!(no_core_caches && no_shared_cache)) {
        cachesys->reset_state();
    }
}

void Processor::receive(Request &req) {
    hint("Processor received %s\n", req.c_str());
    if (!no_shared_cache) {
        llc.callback(req);
    } else if (!cores[0]->no_core_caches) {
        // Assume all cores have caches or don't have caches
        // at the same time.
        for (unsigned int i = 0; i < cores.size(); ++i) {
            Core *core = cores[i].get();
            core->caches[0]->callback(req);
        }
    }
    for (unsigned int i = 0; i < cores.size(); ++i) {
        Core *core = cores[i].get();
        core->receive(req);
    }
}

bool Processor::finished() {
    if (early_exit) {
        for (unsigned int i = 0; i < cores.size(); ++i) {
            if (cores[i]->finished()) {
                for (unsigned int j = 0; j < cores.size(); ++j) {
                    ipc += cores[j]->calc_ipc();
                }
                return true;
            }
        }
        return false;
    } else {
        if (!(no_core_caches && no_shared_cache)) {
            if (cachesys->finished() == false) {
                return false;
            }
        }

        for (unsigned int i = 0; i < cores.size(); ++i) {
            if (!cores[i]->finished()) {
                return false;
            }
            if (ipcs[i] < 0) {
                ipcs[i] = cores[i]->calc_ipc();
                ipc += ipcs[i];
            }
        }
        return true;
    }
}

bool Processor::has_reached_limit() {
    for (unsigned int i = 0; i < cores.size(); ++i) {
        if (!cores[i]->has_reached_limit()) {
            return false;
        }
    }
    return true;
}

long Processor::get_insts() {
    long insts_total = 0;
    for (unsigned int i = 0; i < cores.size(); i++) {
        insts_total += cores[i]->get_insts();
    }

    return insts_total;
}

void Processor::reset_stats() {
    for (unsigned int i = 0; i < cores.size(); i++) {
        cores[i]->reset_stats();
    }

    ipc = 0;

    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;
}

Core::Core(const Config &configs, int coreid,
           const char *trace_fname, function<bool(Request)> send_next,
           Cache *llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase &memory)
    : id(coreid), core_type(core_type_t::PRIME),
      ipc(core_configs[core_type_t::PRIME].ipc),
      out_of_order(core_configs[core_type_t::PRIME].out_of_order),
      no_core_caches(!configs.has_core_caches()),
      no_shared_cache(!configs.has_l3_cache()),
      llc(llc), trace(trace_fname),
      window(this, core_configs[core_type_t::PRIME].out_of_order, core_configs[core_type_t::PRIME].ipc),
      memory(memory) {

    printf("core type: %d, ipc: %d, #V0: %d, #V1: %d\n", core_type, ipc, NUM_V0_PIPELINES, NUM_V1_PIPELINES);

    my_trace_fname = (char *)malloc(1024);
    strcpy(my_trace_fname, trace_fname);

    // set expected limit instruction for calculating weighted speedup
    expected_limit_insts = configs.get_expected_limit_insts();
    trace.expected_limit_insts = expected_limit_insts;

    // Build cache hierarchy
    if (no_core_caches) {
        send = send_next;
    } else {
        // L2 caches[0]
        caches.emplace_back(new Cache(
            core_configs[core_type].l2_cache_config.size,
            core_configs[core_type].l2_cache_config.assoc,
            core_configs[core_type].l2_cache_config.blocksz,
            core_configs[core_type].l2_cache_config.mshr_num,
            Cache::Level::L2, cachesys));
        // L1 caches[1]
        caches.emplace_back(new Cache(
            core_configs[core_type].l1_cache_config.size,
            core_configs[core_type].l1_cache_config.assoc,
            core_configs[core_type].l1_cache_config.blocksz,
            core_configs[core_type].l1_cache_config.mshr_num,
            Cache::Level::L1, cachesys));
        send = bind(&Cache::send, caches[1].get(), placeholders::_1);
        if (llc != nullptr) {
            caches[0]->concatlower(llc);
        }
        caches[1]->concatlower(caches[0].get());

        first_level_cache = caches[1].get();
    }

    // regStats
    record_cycs.name("record_cycs_core_" + to_string(id))
        .desc("Record cycle number for calculating weighted speedup. (Only valid when expected limit instruction number is non zero in config file.)")
        .precision(0);

    record_insts.name("record_insts_core_" + to_string(id))
        .desc("Retired instruction number when record cycle number. (Only valid when expected limit instruction number is non zero in config file.)")
        .precision(0);

    memory_access_cycles.name("memory_access_cycles_core_" + to_string(id))
        .desc("memory access cycles in memory time domain")
        .precision(0);
    memory_access_cycles = 0;
    cpu_inst.name("cpu_instructions_core_" + to_string(id))
        .desc("cpu instruction number")
        .precision(0);
    cpu_inst = 0;

    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        pipeline_v0_wait[i].name("pipeline_v0_wait_" + to_string(i)).desc("Record cycles that Pipeline V0[idx] is waiting for front-end to issue an instruction with V/V0 pipeline type").precision(0);
        pipeline_v0_wait[i] = 0;
        pipeline_v0_exec[i].name("pipeline_v0_execute_" + to_string(i)).desc("Record cycles that Pipeline V0[idx] is indeed executing at least 1 instruction").precision(0);
        pipeline_v0_exec[i] = 0;
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        pipeline_v1_wait[i].name("pipeline_v1_wait_" + to_string(i)).desc("Record cycles that Pipeline V1[idx] is waiting for front-end to issue an instruction with V/V1 pipeline type").precision(0);
        pipeline_v1_wait[i] = 0;
        pipeline_v1_exec[i].name("pipeline_v1_execute_" + to_string(i)).desc("Record cycles that Pipeline V1[idx] is indeed executing at least 1 instruction").precision(0);
        pipeline_v1_exec[i] = 0;
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        pipeline_l_wait[i].name("pipeline_l_wait_" + to_string(i)).desc("Record cycles that Pipeline L[idx] is waiting for front-end to issue a load/store").precision(0);
        pipeline_l_wait[i] = 0;
        pipeline_l_exec[i].name("pipeline_l_execute_" + to_string(i)).desc("Record cycles that Pipeline L[idx] is indeed waiting for memory operations").precision(0);
        pipeline_l_exec[i] = 0;
    }
}

double Core::calc_ipc() {
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
    return (double)retired / clk;
}

void Core::tick() {
    clk++;

    hint("Core clock: %ld\n", clk);

    if (first_level_cache != nullptr)
        first_level_cache->tick();

    // Retire
    int current_retired = window.retire();
    retired += current_retired;
    if (current_retired != 0)
        did_retired = true;

    // Execute
    window.execute();

    // Issue
    window.issue();

    if (expected_limit_insts == 0 && !more_reqs)
        return;

    // Dispatch
    int inserted = 0;
    while ((inserted < ipc) && (window.is_dispatch_full() == false)) {
        if (bubble_cnt < 0) {
            more_reqs = trace.get_neon_request(req_opcode,
                                               req_dst_reg_count, req_dst_mem_count, req_src_reg_count, req_src_mem_count,
                                               req_dst_reg, req_dst_reg_type,
                                               req_src1_reg, req_src1_reg_type,
                                               req_src2_reg, req_src2_reg_type,
                                               req_src3_reg, req_src3_reg_type,
                                               req_addr, req_addr_end,
                                               req_latency, req_guard,
                                               pipeline_1, pipeline_2,
                                               req_type,
                                               bubble_cnt);
            if (!more_reqs) {
                break;
            }
        }
        if (bubble_cnt > 0) {
            request = Request(Request::Type::MAX);
        } else if ((req_type == Request::Type::READ) || (req_type == Request::Type::WRITE)) {
            request = Request(req_opcode,
                              req_addr,
                              req_addr_end,
                              req_type,
                              callback);
        } else if (req_type == Request::Type::NEON) {
            request = Request(
                req_opcode,
                req_dst_reg_count,
                req_dst_mem_count,
                req_src_reg_count,
                req_src_mem_count,
                req_dst_reg,
                req_dst_reg_type,
                req_src1_reg,
                req_src1_reg_type,
                req_src2_reg,
                req_src2_reg_type,
                req_src3_reg,
                req_src3_reg_type,
                req_addr,
                req_addr_end,
                req_latency,
                req_guard,
                pipeline_1,
                pipeline_2,
                callback);
        } else {
            assert(false);
        }

        window.dispatch(request);

        inserted++;
        bubble_cnt--;
        cpu_inst++;
        if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
            record_cycs = clk;
            record_insts = long(cpu_inst.value());
            memory.record_core(id);
            reached_limit = true;
        }
    }

    if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
        record_cycs = clk;
        record_insts = long(cpu_inst.value());
        memory.record_core(id);
        reached_limit = true;
    }

    if (!more_reqs) {
        if (!reached_limit) { // if the length of this trace is shorter than expected length, then record it when the whole trace finishes, and set reached_limit to true.
            // Hasan: overriding this behavior. We start the trace from the
            // beginning until the requested amount of instructions are
            // simulated. This should never be reached now.
            assert((expected_limit_insts == 0) && "Shouldn't be reached when expected_limit_insts > 0 since we start over the trace");
            record_cycs = clk;
            record_insts = long(cpu_inst.value());
            memory.record_core(id);
            reached_limit = true;
        }
    }
}

void Core::reset_state() {
    hint("Core %d state reset\n", id);

    clk = 0;
    retired = 0;
    did_retired = false;

    record_cycs = 0;
    record_insts = 0;

    memory_access_cycles = 0;
    cpu_inst = 0;
    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        pipeline_v0_wait[i] = 0;
        pipeline_v0_exec[i] = 0;
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        pipeline_v1_wait[i] = 0;
        pipeline_v1_exec[i] = 0;
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        pipeline_l_wait[i] = 0;
        pipeline_l_exec[i] = 0;
    }

    req_opcode = "NULL";
    req_dst_reg_count = -1;
    req_dst_mem_count = -1;
    req_src_reg_count = -1;
    req_src_mem_count = -1;
    req_dst_reg = -1;
    req_dst_reg_type = -1;
    req_src1_reg = -1;
    req_src1_reg_type = -1;
    req_src2_reg = -1;
    req_src2_reg_type = -1;
    req_src3_reg = -1;
    req_src3_reg_type = -1;
    req_addr = -1;
    req_addr_end = -1;
    req_latency = -1;
    req_guard = -1;
    bubble_cnt = -1;
    pipeline_1 = -1;
    pipeline_2 = -1;

    req_type = Request::Type::MAX;
    more_reqs = true;
    last = 0;
    trace = Trace(my_trace_fname);

    window.reset_state();
    if (!no_core_caches) {
        for (int idx = 0; idx < caches.size(); idx++) {
            caches[idx]->reset_state();
        }
    }
}

bool Core::finished() {
    for (auto cache : caches) {
        if (cache->finished() == false) {
            return false;
        }
    }

    return !more_reqs && window.finished();
}

bool Core::has_reached_limit() {
    return reached_limit;
}

long Core::get_insts() {
    return long(cpu_inst.value());
}

bool Core::has_retired() {
    hint("Retired checked! instruction has retired!\n");
    if (did_retired == false)
        return false;
    did_retired = false;
    return true;
}

void Core::receive(Request &req) {
    window.set_ready(req);
    if (req.arrive != -1 && req.depart > last) {
        memory_access_cycles += (req.depart - max(last, req.arrive));
        last = req.depart;
    }
}

void Core::reset_stats() {
    clk = 0;
    retired = 0;
    cpu_inst = 0;
    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        pipeline_v0_wait[i] = 0;
        pipeline_v0_exec[i] = 0;
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        pipeline_v1_wait[i] = 0;
        pipeline_v1_exec[i] = 0;
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        pipeline_l_wait[i] = 0;
        pipeline_l_exec[i] = 0;
    }
}

bool Window::is_dispatch_full() {
    return dispatch_load == depth;
}

bool Window::is_dispatch_empty() {
    return dispatch_load == 0;
}

void Window::dispatch(Request &req) {
    // core itself
    req.reqid = last_id;
    last_id++;

    assert(dispatch_load <= depth);

    hint("dispatch called for %s\n", req.c_str());

    if (req.type == Request::Type::MAX) {
        assert((req.pipeline_1_ready == true) && (req.pipeline_2_ready == true));
        req.state = INSTRUCTION_RETIREMENT;
    } else {
        req.state = INSTRUCTION_DISPATCHED;
    }
    req.dispatch_cycle = core->clk;
    req.dispatch_idx = dispatch_head;

    dispatch_list.at(dispatch_head) = req;

    dispatch_head = (dispatch_head + 1) % depth;
    dispatch_load++;
}

long Window::retire() {
    assert(dispatch_load <= depth);

    if (dispatch_load == 0)
        return 0;

    int retired = 0;
    while (dispatch_load > 0 && retired < ipc) {
        if (dispatch_list.at(dispatch_tail).state != INSTRUCTION_RETIREMENT)
            break;

        assert(dispatch_list.at(dispatch_tail).pipeline_1_ready && dispatch_list.at(dispatch_tail).pipeline_2_ready);
        hint("%s retired\n", dispatch_list.at(dispatch_tail).c_str());
        dispatch_list.at(dispatch_tail).retired_cycle = core->clk;

        Request req = dispatch_list.at(dispatch_tail);
        if (op_trace.is_open()) {
            if (req.type == Request::Type::READ)
                op_trace << "Load{0x" << std::hex << req.addr << std::dec;
            else if (req.type == Request::Type::WRITE)
                op_trace << "Write{0x" << std::hex << req.addr << std::dec;
            else if (req.type == Request::Type::NEON) {
                op_trace << "Neon{OPC(" << req.opcode << "), ";
                if (req.dst_mem_count == 1) {
                    op_trace << "MDST(0x" << std::hex << req.addr << "-" << req.addr_end << std::dec << "), ";
                }
                if (req.dst_reg_count == 1) {
                    op_trace << "RDST(" << req.dst_reg << "), ";
                }
                if (req.src_mem_count == 1) {
                    op_trace << "MSRC(0x" << std::hex << req.addr << "-" << req.addr_end << std::dec << "), ";
                }
                if (req.src_reg_count >= 1) {
                    op_trace << "RSRC1(" << req.src1_reg << "), ";
                }
                if (req.src_reg_count >= 2) {
                    op_trace << "RSRC2(" << req.src2_reg << "), ";
                }
                if (req.src_reg_count >= 3) {
                    op_trace << "RSRC3(" << req.src3_reg << "), ";
                }
            } else if (req.type == Request::Type::MAX) {
                op_trace << "Bubble{";
            }
            op_trace << "}, ";

            op_trace << "D(" << req.dispatch_cycle << "), ";
            if (req.pipeline_1 != -1) {
                std::string pipeline_name = (req.pipeline_1 == PIPELINE_V0_TYPE) ? "V0" : (req.pipeline_1 == PIPELINE_V1_TYPE) ? "V1"
                                                                                      : (req.pipeline_1 == PIPELINE_L_TYPE)    ? "L"
                                                                                                                               : "UNK";
                op_trace << "P1-" << pipeline_name << "[" << req.pipeline_1_idx << "]: [";
                op_trace << "I(" << req.issued_P1_cycle << "), ";
                op_trace << "SE(" << req.start_P1_execute_cycle << "), ";
                op_trace << "FE(" << req.finish_P1_execute_cycle << ")], ";
            }
            long pipeline_2 = req.pipeline_2;
            if (pipeline_2 != -1) {
                std::string pipeline_name = (pipeline_2 == PIPELINE_V0_TYPE) ? "V0" : (pipeline_2 == PIPELINE_V1_TYPE) ? "V1"
                                                                                  : (pipeline_2 == PIPELINE_L_TYPE)    ? "L"
                                                                                                                       : "UNK";
                op_trace << "P2-" << pipeline_name << "[" << req.pipeline_2_idx << "]: [";
                op_trace << "I(" << req.issued_P2_cycle << "), ";
                op_trace << "SE(" << req.start_P2_execute_cycle << "), ";
                op_trace << "FE(" << req.finish_P2_execute_cycle << ")], ";
            }
            op_trace << "R(" << req.retired_cycle << ")" << endl;
        }

        dispatch_tail = (dispatch_tail + 1) % depth;
        dispatch_load--;
        retired++;
    }

    return retired;
}

bool Window::check_RAW_dependency(int location) {

    Request query_req = dispatch_list.at(location);
    int idx = dispatch_tail;

    while (idx != location) {
        Request curr_req = dispatch_list.at(idx);
        if ((curr_req.type == Request::Type::NEON) && (curr_req.state != INSTRUCTION_RETIREMENT)) {
            if (curr_req.dst_reg != -1) {
                if ((curr_req.dst_reg == query_req.src1_reg) || (curr_req.dst_reg == query_req.src2_reg) || (curr_req.dst_reg == query_req.src3_reg)) {
                    return true;
                }
            }
        }
        idx = (idx + 1) % depth;
    }
    return false;
}

void Window::issue() {
    int issued = 0;
    for (int i = 0; (i < dispatch_load) && (issued < ipc); i++) {
        int index = (dispatch_tail + i) % depth;
        Request req = dispatch_list.at(index);
        if (req.state == INSTRUCTION_DISPATCHED) {
            if (check_RAW_dependency(index) == false) {
                if (req.pipeline_1_ready == false) {
                    if (req.pipeline_1 <= PIPELINE_V_TYPE) {
                        hint("Req(%s) issued to the first pipeline: V\n", req.c_str());
                        dispatch_list.at(index).state = INSTRUCTION_ISSUED_P1;
                        issue_v_list.push_back(index);
                    } else if (req.pipeline_1 == PIPELINE_L_TYPE) {
                        hint("Req(%s) issued to the first pipeline: L\n", req.c_str());
                        dispatch_list.at(index).state = INSTRUCTION_ISSUED_P1;
                        issue_l_list.push_back(index);
                    } else {
                        assert(false);
                    }
                    dispatch_list.at(index).issued_P1_cycle = core->clk;
                } else if (req.pipeline_2_ready == false) {
                    if ((req.pipeline_2 == PIPELINE_V0_TYPE) || (req.pipeline_2 == PIPELINE_V1_TYPE) || (req.pipeline_2 == PIPELINE_V_TYPE)) {
                        hint("Req(%s) issued to the second pipeline: V\n", req.c_str());
                        dispatch_list.at(index).state = INSTRUCTION_ISSUED_P2;
                        issue_v_list.push_back(index);
                    } else if (req.pipeline_2 == PIPELINE_L_TYPE) {
                        hint("Req(%s) issued to the second pipeline: L\n", req.c_str());
                        dispatch_list.at(index).state = INSTRUCTION_ISSUED_P2;
                        issue_l_list.push_back(index);
                    } else {
                        assert(false);
                    }
                    dispatch_list.at(index).issued_P2_cycle = core->clk;
                } else {
                    assert(false);
                }
            }
        }
    }
}

bool overlap(long a_s, long a_e, long b_s, long b_e) {
    if ((a_s <= b_s) && (b_s <= a_e)) {
        return true;
    }
    if ((b_s <= a_s) && (a_s <= b_e)) {
        return true;
    }
    return false;
}

bool Window::check_memory_dependency(int location) {
    Request req = dispatch_list.at(location);

    // This is because stores must be issued at the head of the ROB
    if ((req.type == Request::Type::WRITE) || ((req.type == Request::Type::NEON) && (req.opcode.find("ST"))))
        return location != dispatch_tail;

    int idx = dispatch_tail;
    while (idx != location) {
        Request curr_request = dispatch_list.at(idx);
        if ((curr_request.type == Request::Type::NEON) && (curr_request.opcode.find("ST") != string::npos)) {
            // It's a NEON store
            if (overlap(req.addr, req.addr_end, curr_request.addr, curr_request.addr_end)) {
                // It has overlap
                return true;
            }
        }
        if (curr_request.type == Request::Type::WRITE) {
            // It's a CPU store
            if (overlap(req.addr, req.addr_end, curr_request.addr, curr_request.addr_end)) {
                // It has overlap
                return true;
            }
        }
        idx = (idx + 1) % depth;
    }
    return false;
}
long Window::align(long addr) {
    return (addr & ~(core_configs[core->core_type].l1_cache_config.blocksz - 1l));
}
void Window::execute() {
    // Check if there's any instruction ready for retirement or next dispatch
    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        // Finish executions first
        std::vector<std::pair<long, long>>::iterator it = execute_v0_lists[i].begin();
        while (it != execute_v0_lists[i].end()) {
            Request req = dispatch_list.at(it->second);
            if (core->clk - it->first >= req.latency) {
                // Execution finished, go to the next step (dispatch of piepline2 or retirement)
                hint("Req(%s) finished V0[%d] execution\n", req.c_str(), i);
                // if this is the first pipeline check the second pipeline
                if (req.state == INSTRUCTION_EXECUTED_P1) {
                    assert((req.pipeline_1 == PIPELINE_V0_TYPE) || (req.pipeline_1 == PIPELINE_V_TYPE));
                    dispatch_list.at(it->second).pipeline_1_ready = true;
                    // Check the second pipeline
                    if (req.pipeline_2_ready == false) {
                        // return it back to dispatch
                        dispatch_list.at(it->second).state = INSTRUCTION_DISPATCHED;
                        hint("Req(%s) sent for issue of second pipeline\n", req.c_str());
                    } else {
                        // otherwise, it's ready for retirement
                        dispatch_list.at(it->second).state = INSTRUCTION_RETIREMENT;
                        hint("Req(%s) ready for retirement\n", req.c_str());
                    }
                    dispatch_list.at(it->second).finish_P1_execute_cycle = core->clk;
                } else {
                    // This must be the second pipeline
                    assert((req.pipeline_2 == PIPELINE_V0_TYPE) || (req.pipeline_2 == PIPELINE_V_TYPE));
                    assert(req.pipeline_1_ready == true);
                    dispatch_list.at(it->second).pipeline_2_ready = true;
                    dispatch_list.at(it->second).state = INSTRUCTION_RETIREMENT;
                    hint("Req(%s) ready for retirement\n", req.c_str());
                    dispatch_list.at(it->second).finish_P2_execute_cycle = core->clk;
                }
                it = execute_v0_lists[i].erase(it);
            } else {
                ++it;
            }
        }
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        // Finish executions first
        std::vector<std::pair<long, long>>::iterator it = execute_v1_lists[i].begin();
        while (it != execute_v1_lists[i].end()) {
            Request req = dispatch_list.at(it->second);
            if (core->clk - it->first >= req.latency) {
                // Execution finished, go to the next step (dispatch of piepline2 or retirement)
                hint("Req(%s) finished V1[%d] execution\n", req.c_str(), i);
                // if this is the first pipeline check the second pipeline
                if (req.state == INSTRUCTION_EXECUTED_P1) {
                    assert((req.pipeline_1 == PIPELINE_V1_TYPE) || (req.pipeline_1 == PIPELINE_V_TYPE));
                    dispatch_list.at(it->second).pipeline_1_ready = true;
                    // Check the second pipeline
                    if (req.pipeline_2_ready == false) {
                        // return it back to dispatch
                        dispatch_list.at(it->second).state = INSTRUCTION_DISPATCHED;
                        hint("Req(%s) sent for issue of second pipeline\n", req.c_str());
                    } else {
                        // otherwise, it's ready for retirement
                        dispatch_list.at(it->second).state = INSTRUCTION_RETIREMENT;
                        hint("Req(%s) ready for retirement\n", req.c_str());
                    }
                    dispatch_list.at(it->second).finish_P1_execute_cycle = core->clk;
                } else {
                    // This must be the second pipeline
                    assert((req.pipeline_2 == PIPELINE_V1_TYPE) || (req.pipeline_2 == PIPELINE_V_TYPE));
                    assert(req.pipeline_1_ready == true);
                    dispatch_list.at(it->second).pipeline_2_ready = true;
                    dispatch_list.at(it->second).state = INSTRUCTION_RETIREMENT;
                    hint("Req(%s) ready for retirement\n", req.c_str());
                    dispatch_list.at(it->second).finish_P2_execute_cycle = core->clk;
                }
                it = execute_v1_lists[i].erase(it);
            } else {
                ++it;
            }
        }
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        // First check if the pipeline is executing any instructions
        if (execute_l_lists[i] == -1) {
            hint("L[%d] No instruction currently running.\n", i);
            continue;
        }
        // Finish executions first
        Request req = dispatch_list.at(execute_l_lists[i]);
        assert(req.mem_sent == true);
        if (req.mem_received) {
            // Execution finished, go to the next step (dispatch of piepline2 or retirement)
            hint("Req(%s) finished L[%d] execution\n", req.c_str(), i);
            // if this is the first pipeline check the second pipeline
            if (req.pipeline_1 == PIPELINE_L_TYPE) {
                dispatch_list.at(execute_l_lists[i]).pipeline_1_ready = true;
                // Check the second pipeline
                if (req.pipeline_2_ready == false) {
                    // return it back to dispatch
                    dispatch_list.at(execute_l_lists[i]).state = INSTRUCTION_DISPATCHED;
                    hint("Req(%s) sent for issue of second pipeline\n", req.c_str());
                } else {
                    // otherwise, it's ready for retirement
                    dispatch_list.at(execute_l_lists[i]).state = INSTRUCTION_RETIREMENT;
                    hint("Req(%s) ready for retirement\n", req.c_str());
                }
                dispatch_list.at(execute_l_lists[i]).finish_P1_execute_cycle = core->clk;
            } else {
                // This must be the second pipeline
                assert(req.pipeline_2 == PIPELINE_L_TYPE);
                assert(req.pipeline_1_ready == true);
                dispatch_list.at(execute_l_lists[i]).pipeline_2_ready = true;
                dispatch_list.at(execute_l_lists[i]).state = INSTRUCTION_RETIREMENT;
                hint("Req(%s) ready for retirement\n", req.c_str());
                dispatch_list.at(execute_l_lists[i]).finish_P2_execute_cycle = core->clk;
            }
            execute_l_lists[i] = -1;
        } else {
            hint("L[%d] %s has not been received yet.\n", i, req.c_str());
        }
    }

    // Now check if there is availablity for execution and start it
    int p_idx_start;
    int v_idx_start = rand() % 2;
    for (int v_idx = 0; v_idx < 2; v_idx++) {
        if (v_idx_start == 0) {
            int p_idx_start = rand() % NUM_V0_PIPELINES;
            for (int i = 0; i < NUM_V0_PIPELINES; i++) {
                int idx = (i + p_idx_start) % NUM_V0_PIPELINES;
                bool ready_for_execute = false;
                if (execute_v0_lists[idx].size() == 0) {
                    ready_for_execute = true;
                } else if (execute_v0_lists[idx].size() == 8) {
                    ready_for_execute = false;
                } else {
                    std::pair<long, long> req_pair = execute_v0_lists[idx][execute_v0_lists[idx].size() - 1];
                    Request req = dispatch_list.at(req_pair.second);
                    if (core->clk - req_pair.first >= req.guard) {
                        ready_for_execute = true;
                    } else {
                        hint("Pipeline V0[%d] is not ready to execute because of %s's guard\n", idx, req.c_str());
                    }
                }
                if (ready_for_execute) {
                    hint("Pipeline V0[%d] is ready to execute a new instruction\n", idx);
                    // Now check if there is any instructions in the issue list ready for execution
                    std::vector<long>::iterator it = issue_v_list.begin();
                    while (it != issue_v_list.end()) {
                        Request req = dispatch_list.at(*it);
                        bool for_this_pipeline = false;
                        if (req.state == INSTRUCTION_ISSUED_P1) {
                            if ((req.pipeline_1 == PIPELINE_V_TYPE) || (req.pipeline_1 == PIPELINE_V0_TYPE)) {
                                for_this_pipeline = true;
                            }
                        } else {
                            assert(req.state == INSTRUCTION_ISSUED_P2);
                            if ((req.pipeline_2 == PIPELINE_V_TYPE) || (req.pipeline_2 == PIPELINE_V0_TYPE)) {
                                for_this_pipeline = true;
                            }
                        }
                        if (for_this_pipeline) {
                            execute_v0_lists[idx].push_back(std::pair<long, long>(core->clk, *it));
                            if (req.state == INSTRUCTION_ISSUED_P1) {
                                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P1;
                                dispatch_list.at(*it).start_P1_execute_cycle = core->clk;
                                dispatch_list.at(*it).pipeline_1 = PIPELINE_V0_TYPE;
                                dispatch_list.at(*it).pipeline_1_idx = idx;
                            } else {
                                assert(req.state == INSTRUCTION_ISSUED_P2);
                                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P2;
                                dispatch_list.at(*it).start_P2_execute_cycle = core->clk;
                                dispatch_list.at(*it).pipeline_2 = PIPELINE_V0_TYPE;
                                dispatch_list.at(*it).pipeline_2_idx = idx;
                            }
                            hint("Req(%s) executing on V0[%d]\n", req.c_str(), idx);
                            it = issue_v_list.erase(it);
                            break;
                        } else {
                            hint("Req(%s) executing is not meant to be executed on V0[%d]\n", req.c_str(), idx);
                            ++it;
                        }
                    }
                }
            }
        } else {
            p_idx_start = rand() % NUM_V1_PIPELINES;
            for (int i = 0; i < NUM_V1_PIPELINES; i++) {
                int idx = (i + p_idx_start) % NUM_V1_PIPELINES;
                bool ready_for_execute = false;
                if (execute_v1_lists[idx].size() == 0) {
                    ready_for_execute = true;
                } else if (execute_v1_lists[idx].size() == 8) {
                    ready_for_execute = false;
                } else {
                    std::pair<long, long> req_pair = execute_v1_lists[idx][execute_v1_lists[idx].size() - 1];
                    Request req = dispatch_list.at(req_pair.second);
                    if (core->clk - req_pair.first >= req.guard) {
                        ready_for_execute = true;
                    } else {
                        hint("Pipeline V1[%d] is not ready to execute because of %s's guard\n", idx, req.c_str());
                    }
                }
                if (ready_for_execute) {
                    hint("Pipeline V1[%d] is ready to execute a new instruction\n", idx);
                    // Now check if there is any instructions in the issue list ready for execution
                    std::vector<long>::iterator it = issue_v_list.begin();
                    while (it != issue_v_list.end()) {
                        Request req = dispatch_list.at(*it);
                        bool for_this_pipeline = false;
                        if (req.state == INSTRUCTION_ISSUED_P1) {
                            if ((req.pipeline_1 == PIPELINE_V_TYPE) || (req.pipeline_1 == PIPELINE_V1_TYPE)) {
                                for_this_pipeline = true;
                            }
                        } else {
                            assert(req.state == INSTRUCTION_ISSUED_P2);
                            if ((req.pipeline_2 == PIPELINE_V_TYPE) || (req.pipeline_2 == PIPELINE_V1_TYPE)) {
                                for_this_pipeline = true;
                            }
                        }
                        if (for_this_pipeline) {
                            execute_v1_lists[idx].push_back(std::pair<long, long>(core->clk, *it));
                            if (req.state == INSTRUCTION_ISSUED_P1) {
                                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P1;
                                dispatch_list.at(*it).start_P1_execute_cycle = core->clk;
                                dispatch_list.at(*it).pipeline_1 = PIPELINE_V1_TYPE;
                                dispatch_list.at(*it).pipeline_1_idx = idx;
                            } else {
                                assert(req.state == INSTRUCTION_ISSUED_P2);
                                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P2;
                                dispatch_list.at(*it).start_P2_execute_cycle = core->clk;
                                dispatch_list.at(*it).pipeline_2 = PIPELINE_V1_TYPE;
                                dispatch_list.at(*it).pipeline_2_idx = idx;
                            }
                            hint("Req(%s) executing on V1[%d]\n", req.c_str(), idx);
                            it = issue_v_list.erase(it);
                            break;
                        } else {
                            hint("Req(%s) executing is not meant to be executed on V1[%d]\n", req.c_str(), idx);
                            ++it;
                        }
                    }
                }
            }
        }
        v_idx_start = 1 - v_idx_start;
    }
    p_idx_start = rand() % NUM_L_PIPELINES;
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        int idx = (i + p_idx_start) % NUM_L_PIPELINES;
        if (execute_l_lists[idx] != -1) {
            hint("Pipeline L[%d] is not ready to execute, executing %s\n", idx, dispatch_list.at(execute_l_lists[idx]).c_str());
            continue;
        }
        hint("Pipeline L[%d] is ready to execute a new instruction\n", idx);
        // Now check if there is any instructions in the issue list ready for execution
        std::vector<long>::iterator it = issue_l_list.begin();
        while (it != issue_l_list.end()) {
            Request req = dispatch_list.at(*it);
            if (check_memory_dependency(*it) == true) {
                // hint("L[%d] cannot execute %s because of memory dependencies\n", idx, req.c_str());
                ++it;
                continue;
            }
            hint("Req(%s) executing on L[%d]\n", req.c_str(), idx);
            execute_l_lists[idx] = *it;
            dispatch_list.at(*it).mem_sent = true;
            Request::Type req_type = Request::Type::READ;
            if ((req.type == Request::Type::WRITE) || ((req.type == Request::Type::NEON) && (req.opcode.find("ST") != string::npos))) {
                dispatch_list.at(*it).mem_received = true;
                req_type = Request::Type::WRITE;
            }
            if (req.state == INSTRUCTION_ISSUED_P1) {
                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P1;
                dispatch_list.at(*it).start_P1_execute_cycle = core->clk;
                dispatch_list.at(*it).pipeline_1_idx = idx;
            } else {
                assert(req.state == INSTRUCTION_ISSUED_P2);
                dispatch_list.at(*it).state = INSTRUCTION_EXECUTED_P2;
                dispatch_list.at(*it).start_P2_execute_cycle = core->clk;
                dispatch_list.at(*it).pipeline_2_idx = idx;
            }
            it = issue_l_list.erase(it);
            if (req_type == Request::Type::READ) {
                assert(ld_to_mem_ops[idx].size() == 0);
            }
            long lower_cache_line = align(req.addr);
            long upper_cache_line = align(req.addr_end);
            int access_needed = ((upper_cache_line - lower_cache_line) / core_configs[core->core_type].l1_cache_config.blocksz) + 1;
            long req_addr = lower_cache_line;
            for (int acc = 0; acc < access_needed; acc++) {
                Request mem_req;
                mem_req = Request(req_addr, req_type, core->callback);
                mem_req.reqid = last_id;
                last_id++;
                if (req_type == Request::Type::READ) {
                    ld_to_mem_ops[idx].push_back(req_addr);
                }
                // send it
                hint("L[%d] sending %s to L1 cache\n", idx, mem_req.c_str());
                assert(core->send(mem_req));
                req_addr += core_configs[core->core_type].l1_cache_config.blocksz;
            }
            break;
        }
    }

    // Handle statistics
    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        if (execute_v0_lists[i].size() == 0) {
            core->pipeline_v0_wait[i]++;
        } else {
            core->pipeline_v0_exec[i]++;
        }
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        if (execute_v1_lists[i].size() == 0) {
            core->pipeline_v1_wait[i]++;
        } else {
            core->pipeline_v1_exec[i]++;
        }
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        if (execute_l_lists[i] == -1) {
            core->pipeline_l_wait[i]++;
        } else {
            core->pipeline_l_exec[i]++;
        }
    }
}

void Window::reset_state() {
    hint("Core %d's window state reset\n", core->id);
    assert(dispatch_load == 0);
    assert(dispatch_head == dispatch_tail);
    assert(issue_v_list.size() == 0);
    assert(issue_l_list.size() == 0);
    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        assert(execute_v0_lists[i].size() == 0);
    }
    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        assert(execute_v1_lists[i].size() == 0);
    }
    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        assert(execute_l_lists[i] == -1);
        assert(ld_to_mem_ops[i].size() == 0);
    }
    dispatch_load = 0;
    dispatch_head = 0;
    dispatch_tail = 0;
    last_id = 0;

    for (int idx = 0; idx < depth; idx++) {
        dispatch_list.at(idx) = Request();
    }
}

bool Window::finished() {
    if (is_dispatch_empty() == false)
        return false;

    if (issue_v_list.size() != 0)
        return false;

    if (issue_l_list.size() != 0)
        return false;

    for (int i = 0; i < NUM_V0_PIPELINES; i++) {
        if (execute_v0_lists[i].size() != 0)
            return false;
    }

    for (int i = 0; i < NUM_V1_PIPELINES; i++) {
        if (execute_v1_lists[i].size() != 0)
            return false;
    }

    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        if (execute_l_lists[i] != -1)
            return false;
    }

    return true;
}

void Window::set_ready(Request req) {

    hint("Setting ready for %s\n", req.c_str());

    for (int i = 0; i < NUM_L_PIPELINES; i++) {
        int index = execute_l_lists[i];
        if (index == -1) {
            continue;
            hint("Setting Ready: no instruction on L[%d]\n", i);
        }
        hint("Setting Ready: Checking L[%d]'s %zu mem operations\n", i, ld_to_mem_ops[i].size());
        assert(dispatch_list.at(index).mem_sent);
        auto it = ld_to_mem_ops[i].begin();
        while (it != ld_to_mem_ops[i].end()) {
            if (align(req.addr) == align(*it)) {
                hint("Setting Ready: %s calls back for %s, %lu instructions remained\n", req.c_str(), dispatch_list.at(index).c_str(), ld_to_mem_ops[i].size() - 1);
                // Remove this instruction from awaiting random accesses
                it = ld_to_mem_ops[i].erase(it);
            } else {
                hint("Setting Ready: %ld and %ld are not aligned\n", req.addr, *it);
                ++it;
            }
        }
        if (ld_to_mem_ops[i].size() == 0) {
            hint("Setting Ready: all memory operations of L[%d] received\n", i);
            dispatch_list.at(index).mem_received = true;
        }
    }
}

Trace::Trace(const char *trace_fname) : file(trace_fname), trace_name(trace_fname) {
    if (!file.good()) {
        std::cerr << "Bad trace file: " << trace_fname << std::endl;
        exit(1);
    }
}

bool Trace::get_unfiltered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type) {
    string line;
    getline(file, line);
    if (file.eof()) {
        file.clear();
        file.seekg(0, file.beg);
        getline(file, line);
        //return false;
    }
    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);
    pos = line.find_first_not_of(' ', pos + 1);
    req_addr = std::stoul(line.substr(pos), &end, 0);

    pos = line.find_first_not_of(' ', pos + end);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else
        assert(false);
    return true;
}

bool Trace::get_filtered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type) {
    static bool has_write = false;
    static long write_addr;
    static int line_num = 0;
    if (has_write) {
        bubble_cnt = 0;
        req_addr = write_addr;
        req_type = Request::Type::WRITE;
        has_write = false;
        return true;
    }
    string line;
    getline(file, line);
    line_num++;
    if (file.eof() || line.size() == 0) {
        file.clear();
        file.seekg(0, file.beg);
        line_num = 0;

        if (expected_limit_insts == 0) {
            has_write = false;
            return false;
        } else { // starting over the input trace file
            getline(file, line);
            line_num++;
        }
    }

    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);

    pos = line.find_first_not_of(' ', pos + 1);
    req_addr = stoul(line.substr(pos), &end, 0);
    req_type = Request::Type::READ;

    pos = line.find_first_not_of(' ', pos + end);
    if (pos != string::npos) {
        has_write = true;
        write_addr = stoul(line.substr(pos), NULL, 0);
    }
    return true;
}

bool Trace::get_dramtrace_request(long &req_addr, Request::Type &req_type) {
    string line;
    getline(file, line);
    if (file.eof()) {
        return false;
    }
    size_t pos;
    req_addr = std::stoul(line, &pos, 16);

    pos = line.find_first_not_of(' ', pos + 1);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else
        assert(false);
    return true;
}

std::string get_remove_first_word(std::string &line) {
    size_t pos;

    // First remove extra space at the start
    pos = line.find_first_not_of(' ');

    if (pos == std::string::npos) {
        line = "";
        return "";
    }

    line = line.substr(pos);

    // Now find the next space
    pos = line.find(' ');

    // find the word
    std::string first_word = line.substr(0, pos);

    // remove the word
    if (pos == std::string::npos) {
        line = "";
    } else {
        line = line.substr(pos);
    }

    return first_word;
}

bool Trace::get_neon_request(std::string &req_opcode,
                             long &req_dst_reg_count, long &req_dst_mem_count, long &req_src_reg_count, long &req_src_mem_count,
                             long &req_dst_reg, long &req_dst_reg_type,
                             long &req_src1_reg, long &req_src1_reg_type,
                             long &req_src2_reg, long &req_src2_reg_type,
                             long &req_src3_reg, long &req_src3_reg_type,
                             long &req_addr, long &req_addr_end,
                             long &req_latency, long &req_guard,
                             long &pipeline_1, long &pipeline_2,
                             Request::Type &req_type,
                             long &bubble_cnt) {
    string line;
    req_opcode = "NOP";
    req_dst_reg_count = -1;
    req_dst_mem_count = -1;
    req_src_reg_count = -1;
    req_src_mem_count = -1;
    req_dst_reg = -1;
    req_dst_reg_type = -1;
    req_src1_reg = -1;
    req_src1_reg_type = -1;
    req_src2_reg = -1;
    req_src2_reg_type = -1;
    req_src3_reg = -1;
    req_src3_reg_type = -1;
    req_addr = -1;
    req_addr_end = -1;
    req_latency = -1;
    req_guard = -1;
    pipeline_1 = -1;
    pipeline_2 = -1;
    req_type = Request::Type::MAX;
    bubble_cnt = -1;
    req_opcode = -1;

    while (true) {
        try {
            getline(file, line);
        } catch (const std::runtime_error &ex) {
            std::cout << ex.what() << std::endl;
        }
        if (file.eof()) {
            file.close();
            return false;
        } else if (line.find("#") != string::npos) {
            // It's a comment
            continue;
        } else if (line.empty()) {
            // It's a white space
            continue;
        } else {
            break;
        }
    }

    req_opcode = get_remove_first_word(line);

    if (req_opcode.compare("load") == 0) {
        req_type = Request::Type::READ;
        req_addr = std::stoul(get_remove_first_word(line), nullptr, 16);
        req_addr_end = req_addr + 15;
        bubble_cnt = std::stoul(get_remove_first_word(line));
    } else if (req_opcode.compare("store") == 0) {
        req_type = Request::Type::WRITE;
        req_addr = std::stoul(get_remove_first_word(line), nullptr, 16);
        req_addr_end = req_addr + 15;
        bubble_cnt = std::stoul(get_remove_first_word(line));
    } else {
        req_type = Request::Type::NEON;
        req_dst_reg_count = std::stoul(get_remove_first_word(line));
        req_dst_mem_count = std::stoul(get_remove_first_word(line));
        req_src_reg_count = std::stoul(get_remove_first_word(line));
        req_src_mem_count = std::stoul(get_remove_first_word(line));
        req_dst_reg = std::stoul(get_remove_first_word(line));
        req_dst_reg_type = std::stoul(get_remove_first_word(line));
        long req_dst_mem_start = std::stoul(get_remove_first_word(line), nullptr, 16);
        long req_dst_mem_end = std::stoul(get_remove_first_word(line), nullptr, 16);
        req_src1_reg = std::stoul(get_remove_first_word(line));
        req_src1_reg_type = std::stoul(get_remove_first_word(line));
        req_src2_reg = std::stoul(get_remove_first_word(line));
        req_src2_reg_type = std::stoul(get_remove_first_word(line));
        req_src3_reg = std::stoul(get_remove_first_word(line));
        req_src3_reg_type = std::stoul(get_remove_first_word(line));
        long req_src_mem_start = std::stoul(get_remove_first_word(line), nullptr, 16);
        long req_src_mem_end = std::stoul(get_remove_first_word(line), nullptr, 16);
        req_latency = std::stoul(get_remove_first_word(line));
        req_guard = std::stoul(get_remove_first_word(line));
        bubble_cnt = std::stoul(get_remove_first_word(line));
        int pipeline_count = std::stoul(get_remove_first_word(line));
        assert(pipeline_count == 1 || pipeline_count == 2);
        pipeline_1 = std::stoul(get_remove_first_word(line));
        if (pipeline_count == 2)
            pipeline_2 = std::stoul(get_remove_first_word(line));
        if (req_dst_mem_count != 0) {
            req_addr = req_dst_mem_start;
            req_addr_end = req_dst_mem_end;
        } else if (req_src_mem_count != 0) {
            req_addr = req_src_mem_start;
            req_addr_end = req_src_mem_end;
        }
    }
    return true;
}