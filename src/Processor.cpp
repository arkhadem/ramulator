#include "Processor.h"
#include <cassert>
#include <cstdio>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config& configs,
    vector<const char*> trace_list,
    function<bool(Request)> send_memory,
    MemoryBase& memory)
    : ipcs(configs.get_core_num(), -1)
    , early_exit(configs.is_early_exit())
    , no_core_caches(!configs.has_core_caches())
    , no_shared_cache(!configs.has_l3_cache())
    , cachesys(new CacheSystem(configs, send_memory))
    , llc(l3_size, l3_assoc, l3_blocksz,
          mshr_per_bank * configs.get_core_num(), l3_access_energy,
          Cache::Level::L3, cachesys)
{

    assert(cachesys != nullptr);
    int tracenum = trace_list.size();
    assert(tracenum > 0);

    vector<vector<const char*>> trace_lists;
    for (int i = 0; i < configs.get_core_num(); ++i) {
        trace_lists.push_back(vector<const char*>());
    }
    int core_id = 0;
    for (auto& trace : trace_list) {
        trace_lists[core_id].push_back(trace);
        core_id = (core_id + 1) % configs.get_core_num();
    }

    if (no_shared_cache) {
        for (int i = 0; i < configs.get_core_num(); ++i) {
            cores.emplace_back(new Core(
                configs, i, trace_lists[i], send_memory, nullptr,
                cachesys, memory));
        }
    } else {
        llc.processor_callback = std::bind(&Processor::receive, this, placeholders::_1);
        for (int i = 0; i < configs.get_core_num(); ++i) {
            cores.emplace_back(new Core(configs, i, trace_lists[i],
                std::bind(&Cache::send, &llc, std::placeholders::_1),
                &llc, cachesys, memory));
        }
    }

    for (int i = 0; i < configs.get_core_num(); ++i) {
        cores[i]->callback = std::bind(&Processor::receive, this, placeholders::_1);
        for (int idx = 0; idx < cores[i].get()->caches.size(); idx++) {
            Cache* cache = cores[i].get()->caches[idx].get();
            cache->processor_callback = std::bind(&Processor::receive, this, placeholders::_1);
        }
    }

    // regStats
    cpu_cycles.name("cpu_cycles")
        .desc("cpu cycle number")
        .precision(0);
    cpu_cycles = 0;
}

void Processor::tick()
{
    cpu_cycles++;

    if ((int(cpu_cycles.value()) % 1000000) == 0)
        printf("CPU heartbeat, cycles: %d \n", (int(cpu_cycles.value())));

    for (unsigned int i = 0; i < cores.size(); ++i) {
        Core* core = cores[i].get();
        core->tick();
    }

    if (!no_shared_cache) {
        llc.tick();
    }

    if (!(no_core_caches && no_shared_cache)) {
        cachesys->tick();
    }
}

void Processor::reset_state()
{
    hint("Processor state reset\n");
    cpu_cycles = 0;
    ipc = 0;
    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;

    for (unsigned int i = 0; i < cores.size(); ++i) {
        Core* core = cores[i].get();
        core->reset_state();
    }

    if (!no_shared_cache) {
        llc.reset_state();
    }

    if (!(no_core_caches && no_shared_cache)) {
        cachesys->reset_state();
    }
}

void Processor::receive(Request& req)
{
    hint("Processor received %s\n", req.c_str());
    if (req.type == Request::Type::GPIC) {
        cores[req.coreid].get()->receive(req);
    } else {
        if (!no_shared_cache) {
            llc.callback(req);
        } else if (!cores[0]->no_core_caches) {
            // Assume all cores have caches or don't have caches
            // at the same time.
            for (unsigned int i = 0; i < cores.size(); ++i) {
                Core* core = cores[i].get();
                core->caches[0]->callback(req);
            }
        }
        for (unsigned int i = 0; i < cores.size(); ++i) {
            Core* core = cores[i].get();
            core->receive(req);
        }
    }
}

bool Processor::finished()
{
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

bool Processor::has_reached_limit()
{
    for (unsigned int i = 0; i < cores.size(); ++i) {
        if (!cores[i]->has_reached_limit()) {
            return false;
        }
    }
    return true;
}

long Processor::get_insts()
{
    long insts_total = 0;
    for (unsigned int i = 0; i < cores.size(); i++) {
        insts_total += cores[i]->get_insts();
    }

    return insts_total;
}

void Processor::reset_stats()
{
    for (unsigned int i = 0; i < cores.size(); i++) {
        cores[i]->reset_stats();
    }

    ipc = 0;

    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;
}

Core::Core(const Config& configs, int coreid,
    const std::vector<const char*>& trace_fnames, function<bool(Request)> send_next,
    Cache* llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory)
    : id(coreid)
    , no_core_caches(!configs.has_core_caches())
    , no_shared_cache(!configs.has_l3_cache())
    , gpic_mode(configs.is_gpic())
    , llc(llc)
    , trace(trace_fnames)
    , window(this)
    , request(coreid, Request::UnitID::CORE)
    , memory(memory)
{

    assert(coreid < MAX_CORE_ID);
    // set expected limit instruction for calculating weighted speedup
    expected_limit_insts = configs.get_expected_limit_insts();
    trace.expected_limit_insts = expected_limit_insts;

    // Build cache hierarchy
    if (no_core_caches) {
        ls_send = send_next;
        assert(configs.is_gpic() == false);
    } else {
        // L2 caches[0]
        caches.emplace_back(new Cache(
            l2_size, l2_assoc, l2_blocksz, l2_mshr_num, l2_access_energy,
            Cache::Level::L2, cachesys, id));
        // L1 caches[1]
        caches.emplace_back(new Cache(
            l1_size, l1_assoc, l1_blocksz, l1_mshr_num, l1_access_energy,
            Cache::Level::L1, cachesys, id));
        if (llc != nullptr) {
            caches[0]->concatlower(llc);
        }
        caches[1]->concatlower(caches[0].get());

        first_level_cache = caches[1].get();

        if (configs.is_gpic()) {
            switch (configs.get_gpic_level()) {
            case 1:
                gpic_send = bind(&Cache::send, caches[1].get(), placeholders::_1);
                break;
            case 2:
                gpic_send = bind(&Cache::send, caches[0].get(), placeholders::_1);
                break;
            case 3:
                gpic_send = send_next;
                break;
            default:
                assert(false && "gpic cache level must be 1, 2, or 3");
                break;
            }
        }
        ls_send = bind(&Cache::send, caches[1].get(), placeholders::_1);
    }

    if (gpic_mode) {
        // DO NOTHING
    } else if (no_core_caches) {
        more_reqs = trace.get_filtered_request(
            bubble_cnt, req_addr, req_type);
        req_addr = memory.page_allocator(req_addr, id);
    } else {
        more_reqs = trace.get_unfiltered_request(
            bubble_cnt, req_addr, req_type);
        req_addr = memory.page_allocator(req_addr, id);
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
}

double Core::calc_ipc()
{
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
    return (double)retired / clk;
}

void Core::tick()
{
    clk++;

    if (first_level_cache != nullptr)
        first_level_cache->tick();

    retired += window.tick();

    if (expected_limit_insts == 0 && !more_reqs)
        return;

    // bubbles (non-memory operations)
    int inserted = 0;

    if (gpic_mode) {

        if ((warmed_up == true) && (warmup_complete == false)) {
            return;
        }

        while ((inserted < window.ipc) && (window.is_full() == false)) {

            if (bubble_cnt == -1) {
                more_reqs = trace.get_gpic_request(bubble_cnt, req_opcode, req_en, req_addr, req_type, SA_id, SA_id_dst);
                if (!more_reqs) {
                    break;
                }
            }

            if (bubble_cnt > 0) {
                request = Request(id, Request::UnitID::CORE);
                window.insert(true, request);
            } else if (req_type == Request::Type::GPIC) {
                if ((req_opcode.find("load") != string::npos) || (req_opcode.find("store") != string::npos)) {
                    // it's a load or store GPIC instruction
                    long addr_end = -1;
                    int data_type = 0;
                    if (req_opcode.find("_b") != string::npos) {
                        data_type = 8;
                    } else if ((req_opcode.find("_w") != string::npos) || (req_opcode.find("_hf") != string::npos)) {
                        data_type = 16;
                    } else if ((req_opcode.find("_dw") != string::npos) || (req_opcode.find("_f") != string::npos)) {
                        data_type = 32;
                    } else if ((req_opcode.find("_qw") != string::npos) || (req_opcode.find("_df") != string::npos)) {
                        data_type = 64;
                    } else {
                        // It's just a 64-byte load/store
                        data_type = 0;
                    }
                    if (data_type != 0) {
                        if ((req_opcode.find("load1") != string::npos) || (req_opcode.find("store1") != string::npos)) {
                            addr_end = (long)(std::ceil((float)(data_type * 1 / 8))) + req_addr - 1;
                        } else {
                            addr_end = (long)(std::ceil((float)(data_type * req_en / 8))) + req_addr - 1;
                        }
                    } else {
                        addr_end = req_addr + 63;
                    }
                    request = Request(req_opcode, req_en, req_addr, addr_end, data_type, SA_id, SA_id_dst, callback, id, Request::UnitID::CORE);
                    window.insert(false, request);
                } else {
                    // it's a computational GPIC instruction
                    request = Request(req_opcode, req_en, -1, -1, -1, SA_id, SA_id_dst, callback, id, Request::UnitID::CORE);
                    window.insert(true, request);
                }
            } else if (req_type == Request::Type::INITIALIZED) {
                // warmed up, wait for other cores
                warmed_up = true;
                return;
            } else if (req_type == Request::Type::READ) {
                // it's a CPU load instrunction
                request = Request(req_addr, req_type, callback, id, Request::UnitID::CORE);
                window.insert(false, request);
            } else {
                // it's a CPU store instrunction
                assert(req_type == Request::Type::WRITE);
                request = Request(req_addr, req_type, callback, id, Request::UnitID::CORE);
                window.insert(true, request);
            }

            bubble_cnt--;
            cpu_inst++;
            inserted++;
            if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
                record_cycs = clk;
                record_insts = long(cpu_inst.value());
                memory.record_core(id);
                reached_limit = true;
            }
        }
    } else {
        while (bubble_cnt > 0) {
            if (inserted == window.ipc)
                return;
            if (window.is_full())
                return;

            request = Request(id, Request::UnitID::CORE);

            window.insert(true, request);
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

        if (req_type == Request::Type::READ) {
            // read request
            if (inserted == window.ipc)
                return;
            if (window.is_full())
                return;

            request = Request(req_addr, req_type, callback, id, Request::UnitID::CORE);

            window.insert(false, request);
            cpu_inst++;
        } else {
            // write request
            assert(req_type == Request::Type::WRITE);
            Request req(req_addr, req_type, callback, id, Request::UnitID::CORE);
            hint("5- CORE sending %s to cache\n", req.c_str());
            if (!ls_send(req))
                return;
            cpu_inst++;
        }

        if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
            record_cycs = clk;
            record_insts = long(cpu_inst.value());
            memory.record_core(id);
            reached_limit = true;
        }

        if (no_core_caches) {
            more_reqs = trace.get_filtered_request(bubble_cnt, req_addr, req_type);
            if (req_addr != -1) {
                req_addr = memory.page_allocator(req_addr, id);
            }
        } else {
            more_reqs = trace.get_unfiltered_request(bubble_cnt, req_addr, req_type);
            if (req_addr != -1) {
                req_addr = memory.page_allocator(req_addr, id);
            }
        }
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

void Core::reset_state()
{
    hint("Core %d state reset\n", id);

    clk = 0;
    retired = 0;

    record_cycs = 0;
    record_insts = 0;

    memory_access_cycles = 0;
    cpu_inst = 0;

    bubble_cnt = -1;
    req_addr = -1;
    req_en = -1;
    req_opcode = "NULL";
    req_type = Request::Type::MAX;
    more_reqs = true;
    last = 0;

    window.reset_state();
    if (!no_core_caches) {
        for (int idx = 0; idx < caches.size(); idx++) {
            caches[idx]->reset_state();
        }
    }
}

bool Core::finished()
{
    for (auto cache : caches) {
        if (cache->finished() == false) {
            return false;
        }
    }

    return !more_reqs && window.is_empty();
}

bool Core::has_reached_limit()
{
    return reached_limit;
}

long Core::get_insts()
{
    return long(cpu_inst.value());
}

bool Core::is_warmed_up()
{
    return (warmed_up && window.is_empty());
}

void Core::receive(Request& req)
{
    hint("Core received %s\n", req.c_str());
    if (req.type == Request::Type::GPIC) {
        window.set_ready(req);
    } else {
        // fix me: if block sizes are different? not related to gpic_level
        switch (gpic_level) {
        case 1:
            window.set_ready(req, ~(l1_blocksz - 1l));
            break;
        case 2:
            window.set_ready(req, ~(l2_blocksz - 1l));
            break;
        case 3:
            window.set_ready(req, ~(l3_blocksz - 1l));
            break;
        default:
            assert(false);
            break;
        }
        if (req.arrive != -1 && req.depart > last) {
            memory_access_cycles += (req.depart - max(last, req.arrive));
            last = req.depart;
        }
    }
}

void Core::reset_stats()
{
    clk = 0;
    retired = 0;
    cpu_inst = 0;
}

bool Window::is_full()
{
    return load == depth;
}

bool Window::is_empty()
{
    return load == 0;
}

bool overlap(long a_s, long a_e, long b_s, long b_e)
{
    if ((a_s <= b_s) && (b_s <= a_e)) {
        return true;
    }
    if ((b_s <= a_s) && (a_s <= b_e)) {
        return true;
    }
    return false;
}

bool Window::find_older_stores(long a_s, long a_e, Request::Type& type, int location)
{
    bool found = false;
    int idx = tail;
    while (idx != location) {
        Request curr_request = req_list.at(idx);
        if ((curr_request.type == Request::Type::GPIC) && (curr_request.opcode.find("store") != string::npos) && (overlap(a_s, a_e, curr_request.addr, curr_request.addr_end))) {
            // It's a GPIC store and has overlap
            type = Request::Type::GPIC;
            return true;
        }
        if ((curr_request.type == Request::Type::WRITE) && (overlap(a_s, a_e, curr_request.addr, curr_request.addr_end))) {
            // It's a CPU store and has overlap
            found = true;
            type = Request::Type::WRITE;
        }
        idx = (idx + 1) % depth;
    }
    return found;
}

bool Window::find_older_unsent(int SA_id, int SA_id_dst, int location)
{
    assert(SA_id != -1);
    int idx = tail;
    while (idx != location) {
        if ((req_list.at(idx).type == Request::Type::GPIC) && (sent_list.at(idx) == false)) {
            if (req_list.at(idx).SA_id == SA_id) {
                return true;
            }
            if ((SA_id_dst != -1) && (req_list.at(idx).SA_id_dst == SA_id_dst)) {
                return true;
            }
        }
        idx = (idx + 1) % depth;
    }
    return false;
}

#ifdef ADVANCED_ROB
bool Window::check_send(Request& req, int location)
{
    if (req.type == Request::Type::GPIC) {
        // Find if there is any unsent instruction from the same SRAM array
        if (find_older_unsent(req.SA_id, req.SA_id_dst, location) == false) {
            if (req.opcode.find("store") != string::npos) {
                // GPIC STORE: Do Nothing
                hint("failed to send %s because store must be at the head of ROB\n", req.c_str());
                return false;
            } else if (req.opcode.find("load") != string::npos) {
                // GPIC LOAD: Find older stores
                Request::Type type;
                if (find_older_stores(req.addr, req.addr_end, type, location)) {
                    // Do Nothing
                    hint("failed to send %s because of older store\n", req.c_str());
                    return false;
                } else {
                    // Send it
                    hint("6- CORE sending %s to cache\n", req.c_str());
                    if (!core->gpic_send(req)) {
                        retry_list.push_back(req);
                    } else {
                        op_trace << core->clk << " core " << req.SA_id << " " << req.opcode << endl;
                    }
                    return true;
                }
            } else {
                // GPIC COMPUTATIONAL
                // Send it
                hint("7- CORE sending %s to cache\n", req.c_str());
                if (!core->gpic_send(req)) {
                    retry_list.push_back(req);
                } else {
                    op_trace << core->clk << " core " << req.SA_id << " " << req.opcode << endl;
                }
                return true;
            }
        } else {
            // Do Nothing
            hint("failed to send %s because of same older instructions\n", req.c_str());
            return false;
        }
    } else if (req.type == Request::Type::READ) {
        // GPIC LOAD: Find older stores
        Request::Type type;
        if (find_older_stores(req.addr, req.addr_end, type, location)) {
            if (type == Request::Type::WRITE) {
                // Set complete and sent = true
                hint("Load to store forwarding for %s\n", req.c_str());
                ready_list.at(head) = true;
                sent_list.at(head) = true;
                return true;
            } else {
                // It's a GPIC store
                // Do Nothing
                return false;
            }
        } else {
            // Send it
            hint("8- CORE sending %s to cache\n", req.c_str());
            if (!core->ls_send(req)) {
                retry_list.push_back(req);
            }
            return true;
        }
    } else {
        assert(req.type == Request::Type::WRITE);
        // Do Nothing
        return false;
    }
    return false;
}
#endif

void Window::insert(bool ready, Request& req)
{
    assert(load <= depth);

    // core itself
    req.reqid = last_id;
    last_id++;

    hint("insert called for %s\n", req.c_str());

    ready_list.at(head) = ready;
    req_list.at(head) = req;
    addr_list.at(head) = req.addr;

    if (req.type != Request::Type::MAX) {
#ifdef ADVANCED_ROB
        if (check_send(req, head)) {
            sent_list.at(head) = true;
        } else {
            sent_list.at(head) = false;
        }
#else
        sent_list.at(head) = false;
#endif
    } else {
        sent_list.at(head) = true;
    }

    head = (head + 1) % depth;
    load++;
}

long Window::tick()
{
    assert(load <= depth);

    while (retry_list.size()) {
        hint("1- CORE sending %s to cache\n", retry_list.at(0).c_str());
        if (retry_list.at(0).type == Request::Type::GPIC) {
            if (core->gpic_send(retry_list.at(0))) {
                op_trace << core->clk << " core " << retry_list.at(0).SA_id << " " << retry_list.at(0).opcode << endl;
                retry_list.erase(retry_list.begin());
            } else {
                break;
            }
        } else {
            assert((retry_list.at(0).type == Request::Type::READ) || (retry_list.at(0).type == Request::Type::WRITE));
            if (core->ls_send(retry_list.at(0))) {
                retry_list.erase(retry_list.begin());
            } else {
                break;
            }
        }
    }

    if (load == 0) {
        // hint("returned for zero load\n");
        return 0;
    }

    int retired = 0;
    while (load > 0 && retired < ipc) {

        if (sent_list.at(tail) == false) {
            // Send it
            Request req = req_list.at(tail);
            hint("2- CORE sending %s to cache\n", req.c_str());
            if (req.type == Request::Type::GPIC) {
                if (!core->gpic_send(req)) {
                    retry_list.push_back(req);
                } else {
                    op_trace << core->clk << " core " << req.SA_id << " " << req.opcode << endl;
                }
            } else {
                assert((req.type == Request::Type::READ) || (req.type == Request::Type::WRITE));
                if (!core->ls_send(req)) {
                    retry_list.push_back(req);
                }
            }

            sent_list.at(tail) = true;
        }

        if (!ready_list.at(tail))
            break;

        hint("Retired: %s\n", req_list.at(tail).c_str());

        // remove all data from tail
        ready_list.at(tail) = false;
        sent_list.at(tail) = false;
        addr_list.at(tail) = -1;
        req_list.at(tail) = Request();

        tail = (tail + 1) % depth;
        load--;
        retired++;
    }

#ifdef ADVANCED_ROB
    int idx = tail;
    for (int i = 0; i < load; i++) {
        if (!sent_list.at(idx)) {
            if (check_send(req_list.at(idx), idx)) {
                sent_list.at(idx) = true;
            }
        }
        idx = (idx + 1) % depth;
    }
#endif

    return retired;
}

void Window::reset_state()
{
    hint("Core %d's window state reset\n", core->id);
    assert(load == 0);
    assert(head == tail);
    head = 0;
    tail = 0;
    last_id = 0;

    for (int idx = 0; idx < depth; idx++) {
        ready_list.at(idx) = false;
        assert(sent_list.at(idx) == false);
        addr_list.at(idx) = -1;
        req_list.at(idx) = Request();
    }

    assert(retry_list.size() == 0);
}

void Window::set_ready(Request req)
{
    assert(req.type == Request::Type::GPIC);

    if (load == 0)
        return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        if ((sent_list.at(index) == true) && (ready_list.at(index) == false)) {
            if ((req_list.at(index).coreid == req.coreid) && (req_list.at(index).unitid == req.unitid) && (req_list.at(index).reqid == req.reqid)) {
                if ((req.opcode.find("load") != string::npos) || (req.opcode.find("store") != string::npos))
                    assert(overlap(req_list.at(index).addr, req_list.at(index).addr_end, req.addr, req.addr_end));
                req_list.at(index).en -= req.en;
                if (req_list.at(index).en == 0) {
                    hint("ready set for %s at location %d\n", req_list.at(index).c_str(), index);
                    ready_list.at(index) = true;
                }
            }
        }
    }
}

void Window::set_ready(Request req, int mask)
{

    assert((req.type == Request::Type::READ) || (req.type == Request::Type::WRITE));

    if (load == 0)
        return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        Request cur_req = req_list.at(index);
        if ((sent_list.at(index) == true) && (ready_list.at(index) == false) && ((cur_req.addr & mask) == (req.addr & mask)) && (cur_req.coreid == req.coreid) && (cur_req.unitid == req.unitid)) {
            hint("ready set for %s at location %d\n", cur_req.c_str(), index);
            ready_list.at(index) = true;
        }
    }
}

const char* req_type_names[] = { "READ", "WRITE", "REFRESH", "POWERDOWN", "SELFREFRESH", "EXTENSION", "MAX" };

Trace::Trace(vector<const char*> trace_fnames)
{
    std::ifstream* files_arr = new std::ifstream[trace_fnames.size()]();
    for (int idx = 0; idx < trace_fnames.size(); idx++) {
        trace_names.push_back(trace_fnames[idx]);
        files_arr[idx].open(trace_fnames[idx]);
        if (!files_arr[idx].good()) {
            std::cerr << "Bad trace file: " << trace_fnames[idx] << std::endl;
            exit(1);
        }
        files.push_back(files_arr + idx);
        warmed_ups.push_back(false);
    }
}

bool Trace::get_gpic_request(long& bubble_cnt, std::string& req_opcode, int& req_en, long& req_addr, Request::Type& req_type, int& SA_id, int& SA_id_dst)
{
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        if ((warmed_ups[trace_idx] == true) && (warmup_complete == false)) {
            continue;
        }
        try {
            getline(*files[trace_idx], line);
        } catch (const std::runtime_error& ex) {
            std::cout << ex.what() << std::endl;
        }
        if (files[trace_idx]->eof()) {
            assert((warmup_complete == true) && ("warm-up is requested and no \"initialized\" exists in a trace file"));
            files[trace_idx]->close();
            files.erase(files.begin() + trace_idx);
            warmed_ups.erase(warmed_ups.begin() + trace_idx);
            trace_offset--;
            continue;
        }
        size_t pos;
        pos = line.find(' ');
        req_opcode = line.substr(0, pos);
        bubble_cnt = 0;
        if (pos == string::npos) {
            assert(req_opcode.compare("initialized") == 0 || req_opcode.compare("flushed") == 0);
            if (req_opcode.compare("initialized") == 0) {
                assert((warmed_ups[trace_idx] == false) && (warmup_complete == false));
                warmed_ups[trace_idx] = true;
                bool warmed_up = true;
                for (int warmup_idx = 0; warmup_idx < (int)warmed_ups.size(); warmup_idx++) {
                    if (warmed_ups[warmup_idx] == false) {
                        warmed_up = false;
                    }
                }
                if (warmed_up == true) {
                    req_addr = -1;
                    req_en = -1;
                    req_type = Request::Type::INITIALIZED;
                    return true;
                }
            }
            trace_offset--;
            continue;
        } else {
            pos = line.find_first_not_of(' ', pos);
            line = line.substr(pos);
            if (req_opcode.compare("load") == 0) {
                req_type = Request::Type::READ;
                req_addr = std::stoul(line, &pos, 16);
                req_en = -1;
            } else if (req_opcode.compare("store") == 0) {
                req_type = Request::Type::WRITE;
                req_addr = std::stoul(line, &pos, 16);
                req_en = -1;
            } else {
                req_type = Request::Type::GPIC;
                req_en = std::stoi(line, &pos, 10);
                if (req_en == 0) {
                    req_en = GPIC_SA_NUM * 256;
                }
                if ((req_opcode.find("load") != string::npos) || (req_opcode.find("store") != string::npos)) {
                    // it's a load or store, read address
                    pos = line.find_first_not_of(' ', pos);
                    line = line.substr(pos);
                    req_addr = std::stoul(line, &pos, 16);
                } else {
                    // It's a non-memory operation
                    req_addr = -1;
                }
                pos = line.find_first_not_of(' ', pos);
                line = line.substr(pos);
                SA_id = std::stoi(line, &pos, 10);
                // SA_id = SA_id % 2;
                if (req_opcode.find("move") != string::npos) {
                    // it's a move. read dst SA num
                    pos = line.find_first_not_of(' ', pos);
                    line = line.substr(pos);
                    SA_id_dst = std::stoi(line, &pos, 10);
                    // SA_id = SA_id % 2;
                } else {
                    // It's a non-memory operation
                    SA_id_dst = -1;
                }
                assert(SA_id < GPIC_SA_NUM);
                assert(SA_id_dst < GPIC_SA_NUM);
            }
            pos = line.find_first_not_of(' ', pos);
            if (pos != string::npos) {
                // There is a bubble count
                line = line.substr(pos);
                bubble_cnt = std::stoul(line, &pos, 10);
            }
        }

        if (req_en != -1)
            if (req_addr != -1) {
                hint("get_gpic_request returned type: GPIC opcode: %s, enable: %d, address: 0x%lx, bubble: %ld, SA: %d\n", req_opcode.c_str(), req_en, req_addr, bubble_cnt, SA_id);
            } else {
                if (SA_id_dst == -1)
                    hint("get_gpic_request returned type: GPIC opcode: %s, enable: %d, bubble: %ld, SA: %d\n", req_opcode.c_str(), req_en, bubble_cnt, SA_id);
                else
                    hint("get_gpic_request returned type: GPIC opcode: %s, enable: %d, bubble: %ld, SA src: %d, SA dst: %d\n", req_opcode.c_str(), req_en, bubble_cnt, SA_id, SA_id_dst);
            }
        else if (req_addr != -1)
            hint("get_gpic_request returned type: LOAD/STORE opcode: %s, address: 0x%lx, bubble: %ld\n", req_opcode.c_str(), req_addr, bubble_cnt);
        else
            hint("get_gpic_request type: INITIALIZED returned opcode: %s\n", req_opcode.c_str());
        last_trace++;
        return true;
    }
    return false;
}

bool Trace::get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*files[trace_idx], line);
        } catch (const std::runtime_error& ex) {
            std::cout << ex.what() << std::endl;
        }
        if (files[trace_idx]->eof()) {
            files[trace_idx]->clear();
            files[trace_idx]->seekg(0, files[trace_idx]->beg);
            if (expected_limit_insts == 0) {
                files[trace_idx]->close();
                files.erase(files.begin() + trace_idx);
                trace_offset--;
                continue;
            } else { // starting over the input trace file
                try {
                    getline(*files[trace_idx], line);
                } catch (const std::runtime_error& ex) {
                    std::cout << ex.what() << std::endl;
                }
            }
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
        hint("get_unfiltered_request returned bubble count: %ld, request address: %ld, type: %s\n", bubble_cnt, req_addr, req_type_names[(int)req_type]);
        last_trace++;
        return true;
    }
    return false;
}

bool Trace::get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*files[trace_idx], line);
        } catch (const std::runtime_error& ex) {
            std::cout << ex.what() << std::endl;
        }
        if (files[trace_idx]->eof()) {
            files[trace_idx]->clear();
            files[trace_idx]->seekg(0, files[trace_idx]->beg);
            if (expected_limit_insts == 0) {
                files[trace_idx]->close();
                files.erase(files.begin() + trace_idx);
                trace_offset--;
                continue;
            } else { // starting over the input trace file
                try {
                    getline(*files[trace_idx], line);
                } catch (const std::runtime_error& ex) {
                    std::cout << ex.what() << std::endl;
                }
            }
        }
        size_t pos, end;
        bubble_cnt = std::stoul(line, &pos, 10);

        pos = line.find_first_not_of(' ', pos + 1);
        req_addr = stoul(line.substr(pos), &end, 0);
        req_type = Request::Type::READ;

        pos = line.find_first_not_of(' ', pos + end);
        hint("get_filtered_request returned bubble count: %ld, request address: %ld, type: %s\n", bubble_cnt, req_addr, req_type_names[(int)req_type]);
        last_trace++;
        return true;
    }
    return false;
}

bool Trace::get_dramtrace_request(long& req_addr, Request::Type& req_type)
{
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*(files[trace_idx]), line);
        } catch (const std::runtime_error& ex) {
            std::cout << ex.what() << std::endl;
        }
        if (files[trace_idx]->eof()) {
            files[trace_idx]->close();
            files.erase(files.begin() + trace_idx);
            trace_offset--;
            continue;
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
        hint("get_dramtrace_request returned request address: %ld, type: %s\n", req_addr, req_type_names[(int)req_type]);
        last_trace++;
        return true;
    }
    return false;
}
