#include "Processor.h"
#include "Cache.h"
#include "Config.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config &configs,
                     vector<const char *> trace_list,
                     function<bool(Request)> send_memory,
                     MemoryBase &memory)
    : ipcs(configs.get_core_num(), -1), early_exit(configs.is_early_exit()), no_core_caches(!configs.has_core_caches()), no_shared_cache(!configs.has_l3_cache()), cachesys(new CacheSystem(configs, send_memory)),
      llc(l3_size, l3_assoc, l3_blocksz, mshr_per_bank * configs.get_core_num(), l3_access_energy, Cache::Level::L3, cachesys, l3_gpic_core_num) {

    assert(cachesys != nullptr);
    int tracenum = trace_list.size();
    assert(tracenum > 0);

    vector<vector<const char *>> trace_lists;
    for (int i = 0; i < configs.get_core_num(); ++i) {
        trace_lists.push_back(vector<const char *>());
    }
    int core_id = 0;
    for (auto &trace : trace_list) {
        trace_lists[core_id].push_back(trace);
        core_id = (core_id + 1) % configs.get_core_num();
    }

    if (no_shared_cache) {
        for (int i = 0; i < configs.get_core_num(); ++i) {
            cores.emplace_back(new Core(
                configs, i, configs.get_core_type(i), trace_lists[i], send_memory, nullptr,
                cachesys, memory));
        }
    } else {
        llc.processor_callback = std::bind(&Processor::receive, this, placeholders::_1);
        for (int i = 0; i < configs.get_core_num(); ++i) {
            cores.emplace_back(new Core(configs, i, configs.get_core_type(i), trace_lists[i],
                                        std::bind(&Cache::send, &llc, std::placeholders::_1),
                                        &llc, cachesys, memory));
        }
    }

    for (int i = 0; i < configs.get_core_num(); ++i) {
        cores[i]->callback = std::bind(&Processor::receive, this, placeholders::_1);
        for (int idx = 0; idx < cores[i].get()->caches.size(); idx++) {
            Cache *cache = cores[i].get()->caches[idx].get();
            cache->processor_callback = std::bind(&Processor::receive, this, placeholders::_1);
        }
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
    if (req.type == Request::Type::GPIC) {
        cores[req.coreid].get()->receive(req);
    } else {
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

Core::Core(const Config &configs, int coreid, core_type_t core_type,
           const std::vector<const char *> &trace_fnames, function<bool(Request)> send_next,
           Cache *llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase &memory)
    : id(coreid), core_type(core_type), ipc(core_configs[core_type].ipc), gpic_core_num(core_configs[core_type].gpic_core_num), out_of_order(core_configs[core_type].out_of_order), no_core_caches(!configs.has_core_caches()), no_shared_cache(!configs.has_l3_cache()), gpic_mode(configs.is_gpic()), llc(llc), trace(trace_fnames), window(this, core_configs[core_type].out_of_order), request(coreid, Request::UnitID::CORE), memory(memory) {

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
            core_configs[core_type].l2_cache_config.size,
            core_configs[core_type].l2_cache_config.assoc,
            core_configs[core_type].l2_cache_config.blocksz,
            core_configs[core_type].l2_cache_config.mshr_num,
            core_configs[core_type].l2_cache_config.access_energy,
            Cache::Level::L2,
            cachesys,
            core_configs[core_type].gpic_core_num,
            id));
        // L1 caches[1]
        caches.emplace_back(new Cache(
            core_configs[core_type].l1_cache_config.size,
            core_configs[core_type].l1_cache_config.assoc,
            core_configs[core_type].l1_cache_config.blocksz,
            core_configs[core_type].l1_cache_config.mshr_num,
            core_configs[core_type].l1_cache_config.access_energy,
            Cache::Level::L1,
            cachesys,
            core_configs[core_type].gpic_core_num,
            id));
        if (llc != nullptr) {
            caches[0]->concatlower(llc);
        }
        caches[1]->concatlower(caches[0].get());

        first_level_cache = caches[1].get();

        if (configs.is_gpic()) {
            DC_reg = 1;
            VL_reg[0] = gpic_core_num * 256;
            VL_reg[1] = VL_reg[2] = VL_reg[3] = 1;
            LS_reg[0] = LS_reg[1] = LS_reg[2] = LS_reg[3] = 0;
            SS_reg[0] = SS_reg[1] = SS_reg[2] = SS_reg[3] = 0;
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

#if (EXETYPE == OUTORDER) || (EXETYPE == DVI)
    stalled_cycs.name("stall_cycs_core_" + to_string(id))
        .desc("Record cycle number for dispatch stalls.")
        .precision(0);
#endif

    memory_access_cycles.name("memory_access_cycles_core_" + to_string(id))
        .desc("memory access cycles in memory time domain")
        .precision(0);
    memory_access_cycles = 0;
    cpu_inst.name("cpu_instructions_core_" + to_string(id))
        .desc("cpu instruction number")
        .precision(0);
    cpu_inst = 0;

    did_retired = false;
}

double Core::calc_ipc() {
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
    return (double)retired / clk;
}

int *Core::stride_evaluator(long *rstride, bool load) {
    int *lstride = new int[4];
    for (int dim = 0; dim < 4; dim++) {
        switch (rstride[dim]) {
        case 3:
            if (load)
                lstride[dim] = LS_reg[dim];
            else
                lstride[dim] = SS_reg[dim];
            break;
        case 2:
            if (dim == 0)
                lstride[dim] = 1;
            else
                lstride[dim] = VL_reg[dim - 1] * lstride[dim - 1];
            break;
        case 1:
            lstride[dim] = rstride[dim];
            break;
        case 0:
            lstride[dim] = rstride[dim];
            break;
        default:
            assert(false);
        }
    }
    return lstride;
}

bool Core::dispatch_gpic() {
#if (EXETYPE == DVI) || (EXETYPE == OUTORDER)
    if (req_dst != -1) {
        // it needs physical register allocation
        if (free_pr >= data_type) {
            free_pr -= data_type;
            hint("%d registers allocated for request %s\n", data_type, request.c_str());
            window.insert(request);
            dispatch_stalled = false;
            return true;
        } else {
            // we have to stall the dispatch
            hint("No %d physical registers (%ld) for request %s, waiting...\n", data_type, free_pr, request.c_str());
            dispatch_stalled = true;
            stalled_cycs += 1;
            return false;
        }
    } else {
        // it does not need physical register allocation
        window.insert(request);
        dispatch_stalled = false;
        return true;
    }
#else
    window.insert(request);
    return true;
#endif
}

void Core::tick() {
    clk++;

    if (first_level_cache != nullptr)
        first_level_cache->tick();

    int current_retired = window.tick();
    retired += current_retired;
    if (current_retired != 0)
        did_retired = true;

    if (expected_limit_insts == 0 && !more_reqs)
        return;

    // bubbles (non-memory operations)
    int inserted = 0;

    if (gpic_mode) {

        if ((warmed_up == true) && (warmup_complete == false)) {
            return;
        }

        while ((inserted < window.ipc) && (window.is_full() == false)) {
#if (EXETYPE == DVI) || (EXETYPE == OUTORDER)
            if (dispatch_stalled == true) {
                if (dispatch_gpic() == false) {
                    break;
                }
                bubble_cnt -= 1;
                inserted = 1;
            }
#endif
            if (bubble_cnt < 0) {
                more_reqs = trace.get_gpic_request(bubble_cnt, req_opcode, req_dst, req_src1, req_src2, req_dim, req_value, req_addr, req_addr_starts, req_stride, req_type); //, req_vid, req_vid_dst);
                if (!more_reqs) {
                    break;
                }
                if (req_opcode.find("_b") != string::npos) {
                    data_type = 8;
                } else if ((req_opcode.find("_w") != string::npos) || (req_opcode.find("_hf") != string::npos)) {
                    data_type = 16;
                } else if ((req_opcode.find("_dw") != string::npos) || (req_opcode.find("_f") != string::npos)) {
                    data_type = 32;
                } else if ((req_opcode.find("_qw") != string::npos) || (req_opcode.find("_df") != string::npos)) {
                    data_type = 64;
                } else {
                    // It's a CPU load / store
                    data_type = 0;
                }
            }

            if (bubble_cnt > 0) {
                request = Request(id, Request::UnitID::CORE);
                window.insert(request);
#if (EXETYPE == DVI)
            } else if (req_type == Request::Type::FREE) {
                window.add_free_instr(data_type);
                bubble_cnt = -1;
                continue;
#endif
            } else if (req_type == Request::Type::GPIC) {
                if ((req_opcode.find("_set_") != string::npos) || (req_opcode.find("_unset_") != string::npos)) {
                    // it's a config GPIC instruction
                    request = Request(req_opcode, req_dim, req_value, true, callback, id, Request::UnitID::CORE);
                    assert((req_value >= 0) || (req_value == -1));
                    if (req_opcode.find("stride") != string::npos) {
                        if (req_opcode.find("load") != string::npos) {
                            LS_reg[req_dim] = req_value;
                        } else if (req_opcode.find("store") != string::npos) {
                            SS_reg[req_dim] = req_value;
                        } else {
                            assert(false);
                        }
                    } else if (req_opcode.find("dim") != string::npos) {
                        if (req_opcode.find("length") != string::npos) {
                            assert(req_dim < DC_reg);
                            VL_reg[req_dim] = req_value;
                            assert((VL_reg[0] * VL_reg[1] * VL_reg[2] * VL_reg[3]) <= (256 * gpic_core_num));
                        } else if (req_opcode.find("count") != string::npos) {
                            DC_reg = req_value;
                        } else {
                            assert(false);
                        }
                    } else if (req_opcode.find("element") != string::npos) {
                        // DO NOTHING \ NONE OF CORE'S BUSINESS
                    } else {
                        assert(req_opcode.find("mask") != string::npos);
                    }
                    assert(dispatch_gpic());
                } else if (req_opcode.find("_dict_") != string::npos) {
                    // Only 8-bit dict is implemented
                    assert(data_type == 8);

                    long req_addr_start = req_addr;
                    long req_addr_end = req_addr + 255;

                    req_addr_starts.clear();
                    req_addr_ends.clear();

                    request = Request(req_opcode, req_dst, req_src1, req_src2, req_addr_start, req_addr_end, req_addr_starts, req_addr_ends, data_type, -1, false, callback, id, Request::UnitID::CORE);

                    if (dispatch_gpic() == false) {
                        break;
                    }
                } else {
                    if ((req_opcode.find("load") != string::npos) || (req_opcode.find("store") != string::npos)) {
                        // it's a load or store GPIC instruction

                        int *stride = stride_evaluator(req_stride.data(), (req_opcode.find("load") != string::npos));

                        long address_length;

                        if (stride[0] != 0) {
                            address_length = (long)(std::ceil((float)((data_type * VL_reg[0] * stride[0]) / 8)));
                            hint("address_length = (%d*%ld*%d)/8 = %ld\n", data_type, VL_reg[0], stride[0], address_length);
                        } else {
                            address_length = (long)(std::ceil((float)(data_type / 8)));
                            hint("address_length = %d/8 = %ld\n", data_type, address_length);
                        }

                        req_addr_ends.clear();

                        if ((req_opcode.find("loadr") != string::npos) || (req_opcode.find("storer") != string::npos)) {
                            // It's a random load or store
                            vector<long> req_addr_starts_temp;
                            long addr_3;
                            long addr_2;
                            long addr_1;
                            for (int i = 0; i < VL_reg[3]; i++) {
                                if (DC_reg == 4)
                                    addr_3 = req_addr_starts[i];
                                for (int j = 0; j < VL_reg[2]; j++) {
                                    if (DC_reg == 3)
                                        addr_2 = req_addr_starts[j];
                                    else
                                        addr_2 = addr_3 + (stride[2] * j * data_type / 8);
                                    for (int k = 0; k < VL_reg[1]; k++) {
                                        if (DC_reg == 2)
                                            addr_1 = req_addr_starts[k];
                                        else
                                            addr_1 = addr_2 + (stride[1] * j * data_type / 8);
                                        req_addr_starts_temp.push_back(addr_1);
                                        req_addr_ends.push_back(addr_1 + address_length - 1);
                                    }
                                }
                            }
                            req_addr_starts = req_addr_starts_temp;
                        } else {
                            // It's a regular strided load/store
                            long addr;
                            req_addr_starts.clear();
                            for (int i = 0; i < VL_reg[3]; i++) {
                                for (int j = 0; j < VL_reg[2]; j++) {
                                    for (int k = 0; k < VL_reg[1]; k++) {
                                        addr = req_addr + (((i * stride[3] + j * stride[2] + k * stride[1]) * data_type) / 8);
                                        req_addr_starts.push_back(addr);
                                        req_addr_ends.push_back(addr + address_length - 1);
                                    }
                                }
                            }
                        }

                        long req_addr_start = req_addr;
                        long req_addr_end = req_addr + (((VL_reg[3] * stride[3] + VL_reg[2] * stride[2] + VL_reg[1] * stride[1]) * data_type) / 8) + address_length - 1;

                        request = Request(req_opcode, req_dst, req_src1, req_src2, req_addr_start, req_addr_end, req_addr_starts, req_addr_ends, data_type, req_stride[0], false, callback, id, Request::UnitID::CORE);

                        if (dispatch_gpic() == false) {
                            break;
                        }
                    } else {
                        // it's a computational GPIC instruction
                        request = Request(req_opcode, req_dst, req_src1, req_src2, data_type, false, callback, id, Request::UnitID::CORE);

                        if (dispatch_gpic() == false) {
                            break;
                        }
                    }
                }
            } else if (req_type == Request::Type::INITIALIZED) {
                // warmed up, wait for other cores
                warmed_up = true;
                return;
            } else if (req_type == Request::Type::READ) {
                // it's a CPU load instrunction
                request = Request(req_addr, req_type, false, callback, id, Request::UnitID::CORE);
                assert(dispatch_gpic());
            } else {
                // it's a CPU store instrunction
                assert(req_type == Request::Type::WRITE);
                request = Request(req_addr, req_type, true, callback, id, Request::UnitID::CORE);
                assert(dispatch_gpic());
            }

            bubble_cnt -= 1;
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

            window.insert(request);
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

            request = Request(req_addr, req_type, false, callback, id, Request::UnitID::CORE);

            window.insert(request);
            cpu_inst++;
        } else {
            // write request
            assert(req_type == Request::Type::WRITE);
            Request req(req_addr, req_type, true, callback, id, Request::UnitID::CORE);
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

void Core::reset_state() {
    hint("Core %d state reset\n", id);

    clk = 0;
    retired = 0;
    did_retired = false;

    record_cycs = 0;
    record_insts = 0;
#if (EXETYPE == OUTORDER) || (EXETYPE == DVI)
    stalled_cycs = 0;
#endif

    memory_access_cycles = 0;
    cpu_inst = 0;

    bubble_cnt = -1;
    req_addr = -1;
    req_stride.clear();
    req_dim = -1;
    req_value = -1;
    req_opcode = "NULL";
    req_type = Request::Type::MAX;
    req_dst = -1;
    req_src1 = -1;
    req_src2 = -1;
#if (EXETYPE == OUTORDER) || (EXETYPE == DVI)
    dispatch_stalled = false;
#endif
#if (EXETYPE == DVI)
    free_pr = 256;
#elif (EXETYPE == OUTORDER)
    free_pr = 64;
#endif
    more_reqs = true;
    last = 0;
    req_addr_starts.clear();
    req_addr_ends.clear();

    VL_reg[0] = gpic_core_num * 256;
    VL_reg[1] = VL_reg[2] = VL_reg[3] = 1;
    LS_reg[0] = LS_reg[1] = LS_reg[2] = LS_reg[3] = 0;
    SS_reg[0] = SS_reg[1] = SS_reg[2] = SS_reg[3] = 0;

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

    return !more_reqs && window.is_empty();
}

bool Core::has_reached_limit() {
    return reached_limit;
}

long Core::get_insts() {
    return long(cpu_inst.value());
}

bool Core::is_warmed_up() {
    return (warmed_up && window.is_empty());
}

bool Core::has_retired() {
    hint("Retired checked! instruction has retired!\n");
    if (did_retired == false)
        return false;
    did_retired = false;
    return true;
}

void Core::receive(Request &req) {
    hint("Core received %s\n", req.c_str());
    if (req.type == Request::Type::GPIC) {
        window.set_ready(req);
    } else {
        // fix me: if block sizes are different? not related to gpic_level
        switch (gpic_level) {
        case 1:
            window.set_ready(req, ~(core_configs[core_type].l1_cache_config.blocksz - 1l));
            break;
        case 2:
            window.set_ready(req, ~(core_configs[core_type].l2_cache_config.blocksz - 1l));
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

void Core::reset_stats() {
    clk = 0;
    retired = 0;
    cpu_inst = 0;
}

bool Window::is_full() {
    return load == depth;
}

bool Window::is_empty() {
    return load == 0;
}

bool Window::no_retry() {
    return retry_list.size() == 0;
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

#if (EXETYPE == DVI)
void Window::add_free_instr(int data_type) {
    int location;
    if (head == 0) {
        location = depth - 1;
    } else {
        location = head - 1;
    }
    req_list.at(location).free_reg += data_type;
    hint("%d registers added to the free list of %s\n", data_type, req_list.at(location).c_str());
}
#endif

bool Window::find_older_stores(long a_s, long a_e, Request::Type &type, int location) {
    bool found = false;
    int idx = tail;
    while (idx != location) {
        Request curr_request = req_list.at(idx);
        if ((curr_request.type == Request::Type::GPIC) && (curr_request.opcode.find("store") != string::npos)) {
            // It's a GPIC store
            if (curr_request.opcode.find("storer") != string::npos) {
                // It's a random store, not predictable
                type = Request::Type::GPIC;
                return true;
            }
            if (overlap(a_s, a_e, curr_request.addr, curr_request.addr_end)) {
                // It has overlap
                type = Request::Type::GPIC;
                return true;
            }
        }
        if (curr_request.type == Request::Type::WRITE) {
            // It's a CPU store
            if (overlap(a_s, a_e, curr_request.addr, curr_request.addr_end)) {
                // It has overlap
                found = true;
                type = Request::Type::WRITE;
            }
        }
        idx = (idx + 1) % depth;
    }
    return found;
}

bool Window::find_any_older_stores(int location, Request::Type type) {
    int idx = tail;
    while (idx != location) {
        Request curr_request = req_list.at(idx);
        if ((type == Request::Type::WRITE) && (curr_request.type == Request::Type::WRITE)) {
            return true;
        }
        if ((type == Request::Type::GPIC) && (curr_request.type == Request::Type::GPIC) && (curr_request.opcode.find("storer") != string::npos)) {
            return true;
        }
        idx = (idx + 1) % depth;
    }
    return false;
}

bool Window::find_older_unsent(int location, long dst_reg) {
    if (dst_reg == -1) {
        return false;
    }

    int idx = tail;

    while (idx != location) {
        Request curr_req = req_list.at(idx);
        if ((curr_req.type == Request::Type::GPIC) && (sent_list.at(idx) == false)) {
            if ((dst_reg == curr_req.src1) || (dst_reg == curr_req.src2))
                return true;
        }
        idx = (idx + 1) % depth;
    }
    return false;
}

int Window::get_location(int location) {
    if (location >= tail)
        return location - tail;
    return depth + location - tail;
}

bool Window::check_send(Request &req, int location) {
    if (req.type == Request::Type::GPIC) {
        if (out_of_order == false)
            return false;
#if (EXETYPE == INORDER)
        return false;
#endif
        // Find if there is any unsent instruction
        if (find_older_unsent(location, req.dst) == false) {
            if ((req.addr != -1) && (req.opcode.find("store") != string::npos)) {
                // GPIC STORE: Do Nothing
                hint("failed to send @%d %s because store must be at the head of ROB\n", get_location(location), req.c_str());
                return false;
            } else if ((req.addr != -1) && (req.opcode.find("load") != string::npos)) {
                // GPIC LOAD: Find older stores
                bool older_store = false;
                if (req.opcode.find("loadr") != string::npos) {
                    // If it's a random load, check with all older stores
                    older_store = false; //find_any_older_stores(location, Request::Type::WRITE);
                } else {
                    // It's a strided load, find stores to the same addresses
                    Request::Type type;
                    older_store = find_older_stores(req.addr, req.addr_end, type, location);
                }
                if (older_store == true) {
                    // Do Nothing
                    hint("failed to send @%d %s because of older CPU store\n", get_location(location), req.c_str());
                    return false;
                } else {
                    // Send it
                    hint("6- CORE sending %s to cache\n", req.c_str());
                    if (!core->gpic_send(req)) {
                        retry_list.push_back(req);
                    } else {
                        op_trace << core->clk << " core controller " << req.opcode << " " << req.dst << endl;
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
                    op_trace << core->clk << " core controller " << req.opcode << " " << req.dst << endl;
                }
                return true;
            }
        } else {
            // Do Nothing
            hint("failed to send @%d %s because of same older instructions\n", get_location(location), req.c_str());
            return false;
        }
    } else if (req.type == Request::Type::READ) {
        if (out_of_order == false)
            return false;
        // CPU LOAD: Find older stores
        Request::Type type;
        assert((req.addr_starts.size() == 1) && (req.addr_ends.size() == 1) && (req.addr_starts[0] == req.addr));
        if (find_any_older_stores(location, Request::Type::GPIC)) {
            // There is a random GPIC store
            // Do Nothing
            hint("failed to send @%d %s because of any older GPIC random store\n", get_location(location), req.c_str());
            return false;
        } else if (find_older_stores(req.addr, req.addr_end, type, location)) {
            if (type == Request::Type::WRITE) {
                // Set complete and sent = true
                hint("Load to store forwarding for %s\n", req.c_str());
                req_list.at(head).ready = true;
                sent_list.at(head) = true;
                return true;
            } else {
                // It's a strided GPIC store
                // Do Nothing
                hint("failed to send @%d %s because of the same older GPIC store\n", get_location(location), req.c_str());
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

void Window::insert(Request &req) {
    assert(load <= depth);

    // core itself
    req.reqid = last_id;
    last_id++;

    hint("insert called for %s\n", req.c_str());

    req_list.at(head) = req;
    // addr_list.at(head) = req.addr;

    if (req.type != Request::Type::MAX) {
        if (check_send(req, head)) {
            sent_list.at(head) = true;
        } else {
            sent_list.at(head) = false;
        }
    } else {
        sent_list.at(head) = true;
    }

    head = (head + 1) % depth;
    load++;
}

long Window::tick() {
    assert(load <= depth);

    while (retry_list.size()) {
        hint("1- CORE sending %s to cache\n", retry_list.at(0).c_str());
        if (retry_list.at(0).type == Request::Type::GPIC) {
            if (core->gpic_send(retry_list.at(0))) {
                op_trace << core->clk << " core controller " << retry_list.at(0).opcode << " ";
                if (retry_list.at(0).dst != -1) {
                    op_trace << retry_list.at(0).dst;
                } else if (retry_list.at(0).addr != -1) {
                    op_trace << retry_list.at(0).addr;
                } else {
                    op_trace << "-1";
                }
                op_trace << endl;
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
                    op_trace << core->clk << " core controller " << req.opcode << " ";
                    if (req.dst != -1) {
                        op_trace << req.dst;
                    } else if (req.addr != -1) {
                        op_trace << req.addr;
                    } else {
                        op_trace << "-1";
                    }
                    op_trace << endl;
                }
            } else {
                assert((req.type == Request::Type::READ) || (req.type == Request::Type::WRITE));
                if (!core->ls_send(req)) {
                    retry_list.push_back(req);
                }
            }

            sent_list.at(tail) = true;
        }

        if (!req_list.at(tail).ready)
            break;

        hint("Retired: %s\n", req_list.at(tail).c_str());

#if (EXETYPE == DVI)
        // add physical registers to free list
        if (req_list.at(tail).free_reg != 0) {
            core->free_pr += req_list.at(tail).free_reg;
            assert(core->free_pr <= 256);
            hint("%ld registers freed\n", req_list.at(tail).free_reg);
        }
#elif (EXETYPE == OUTORDER)
        // add physical registers to free list
        if (req_list.at(tail).dst != -1) {
            // allocated_pr += req_list.at(tail).data_type;
            // if (allocated_pr > 176) {
            //     hint("Enough VR allocated, start freeing PRs.\n");
            //     all_vr_allocated = true;
            // }
            // if (all_vr_allocated) {
            assert((req_list.at(tail).data_type != 0) && (req_list.at(tail).data_type % 8 == 0));
            core->free_pr += req_list.at(tail).data_type;
            assert(core->free_pr <= 256);
            hint("%ld registers freed\n", req_list.at(tail).data_type);
            // }
        }
#endif
        // remove all data from tail
        sent_list.at(tail) = false;
        req_list.at(tail) = Request();

        tail = (tail + 1) % depth;
        load--;
        retired++;
    }

    int idx = tail;
    for (int i = 0; i < load; i++) {
        if (!sent_list.at(idx)) {
            if (check_send(req_list.at(idx), idx)) {
                sent_list.at(idx) = true;
                // This is added to make sure only the first
                // unsent instructions is checked. If instruction is
                // sent, no more bw for the next instructions to send
                break;
            }
        }
        idx = (idx + 1) % depth;
    }

    return retired;
}

void Window::reset_state() {
    hint("Core %d's window state reset\n", core->id);
    assert(load == 0);
    assert(head == tail);
    head = 0;
    tail = 0;
    last_id = 0;
    // #if (EXETYPE == OUTORDER)
    //     allocated_pr = 0;
    //     all_vr_allocated = false;
    // #endif

    for (int idx = 0; idx < depth; idx++) {
        assert(sent_list.at(idx) == false);
        // addr_list.at(idx) = -1;
        req_list.at(idx) = Request();
    }
    for (int idx = 0; idx < retry_list.size(); idx++) {
        printf("retry_list[%d]: %s\n", idx, retry_list[idx].c_str());
    }
    assert(retry_list.size() == 0);
}

void Window::set_ready(Request req) {
    assert(req.type == Request::Type::GPIC);

    if (load == 0)
        return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        if ((sent_list.at(index) == true) && (req_list.at(index).ready == false)) {
            if ((req_list.at(index).coreid == req.coreid) && (req_list.at(index).unitid == req.unitid) && (req_list.at(index).reqid == req.reqid)) {
                hint("ready set for %s at location %d\n", req_list.at(index).c_str(), index);
                req_list.at(index).ready = true;
            }
        }
    }
}

void Window::set_ready(Request req, int mask) {

    assert((req.type == Request::Type::READ) || (req.type == Request::Type::WRITE));

    if (load == 0)
        return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        Request cur_req = req_list.at(index);
        if ((sent_list.at(index) == true) && (cur_req.ready == false) && ((cur_req.addr & mask) == (req.addr & mask)) && (cur_req.coreid == req.coreid) && (cur_req.unitid == req.unitid)) {
            hint("ready set for %s at location %d\n", cur_req.c_str(), index);
            req_list.at(index).ready = true;
        }
    }
}

const char *req_type_names[] = {"READ", "WRITE", "REFRESH", "POWERDOWN", "SELFREFRESH", "EXTENSION", "MAX"};

Trace::Trace(vector<const char *> trace_fnames) {
    std::ifstream *files_arr = new std::ifstream[trace_fnames.size()]();
    for (int idx = 0; idx < trace_fnames.size(); idx++) {
        trace_names.push_back(trace_fnames[idx]);
        files_arr[idx].open(trace_fnames[idx]);
        if (!files_arr[idx].good()) {
            std::cerr << "Bad trace file: " << trace_fnames[idx] << std::endl;
            exit(1);
        }
        files.push_back(files_arr + idx);
    }
    warmed_up = false;
    last_trace = 0;
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

bool Trace::get_gpic_request(long &bubble_cnt, std::string &req_opcode, long &req_dst, long &req_src1, long &req_src2, long &req_dim, long &req_value, long &req_addr, std::vector<long> &req_addr_starts, std::vector<long> &req_stride, Request::Type &req_type) { //, int &req_vid, int &req_vid_dst) {
    string line;
    bubble_cnt = -1;
    req_opcode = -1;
    req_dim = -1;
    req_value = -1;
    req_addr = -1;
    req_stride.clear();
    req_type = Request::Type::MAX;
    req_addr_starts.clear();
    req_dst = -1;
    req_src1 = -1;
    req_src2 = -1;

    if ((warmed_up == true) && (warmup_complete == false)) {
        printf("returned false here\n");
        return false;
    }
    while (true) {
        try {
            getline(*files[last_trace], line);
        } catch (const std::runtime_error &ex) {
            std::cout << ex.what() << std::endl;
        }
        if (files[last_trace]->eof()) {
            if (warmup_complete == false) {
                // It is not OK
                // Processor waiting for core yet initialized is not fetched
                assert(false && ("warm-up is requested and no \"initialized\" exists in a trace file"));
            } else {
                // It does not matter if warmed_up is true or false
                // If true: it means that warm up is requested and reached
                // If false: it means that warm up is never requested
                files[last_trace]->close();
                last_trace++;
                if (last_trace == files.size())
                    return false;
            }
        } else if (line.find("#") != string::npos) {
            // It's a comment
            continue;
        } else if (line.empty()) {
            // It's a white space
            continue;
        } else {
            req_opcode = get_remove_first_word(line);
            bubble_cnt = 0;
            if (line.size() == 0) {
                assert(req_opcode.compare("initialized") == 0 || req_opcode.compare("flushed") == 0 || req_opcode.find("free") != string::npos);
                if (req_opcode.compare("initialized") == 0) {
                    assert((warmed_up == false) && (warmup_complete == false));
                    warmed_up = true;
                    req_type = Request::Type::INITIALIZED;
                    return true;
                } else if (req_opcode.find("free") != string::npos) {
#if (EXETYPE == DVI)
                    req_type = Request::Type::FREE;
                    return true;
#else
                    continue;
#endif
                }
                continue;
            } else {
                break;
            }
        }
    }

    if (req_opcode.compare("load") == 0) {
        req_type = Request::Type::READ;
        req_addr = std::stoul(get_remove_first_word(line), nullptr, 16);
    } else if (req_opcode.compare("store") == 0) {
        req_type = Request::Type::WRITE;
        req_addr = std::stoul(get_remove_first_word(line), nullptr, 16);
    } else {
        req_type = Request::Type::GPIC;
        // general arguments
        req_dst = std::stoul(get_remove_first_word(line));
        std::string src1_word = get_remove_first_word(line);
        req_src1 = std::stoul(src1_word);
        req_src2 = std::stoul(get_remove_first_word(line));

        if ((req_opcode.find("pc3_load") != string::npos) || (req_opcode.find("pc3_store") != string::npos) || (req_opcode.find("pc3_dict") != string::npos)) {
            // memory-related arguments

            req_addr = std::stoul(src1_word, nullptr, 16);
            req_src1 = -1;
            long stride3 = std::stoul(get_remove_first_word(line));
            long stride2 = std::stoul(get_remove_first_word(line));
            long stride1 = std::stoul(get_remove_first_word(line));
            long stride0 = std::stoul(get_remove_first_word(line));
            req_stride.push_back(stride0);
            req_stride.push_back(stride1);
            req_stride.push_back(stride2);
            req_stride.push_back(stride3);

            if ((req_opcode.find("loadr") != string::npos) || (req_opcode.find("storer") != string::npos)) {
                long req_addr_start;
                while (line.size() != 0) {
                    req_addr_start = std::stoul(get_remove_first_word(line), nullptr, 16);
                    req_addr_starts.push_back(req_addr_start);
                }
            }
        } else {
            // register-related arguments
            // config
            req_dim = std::stoi(get_remove_first_word(line));
            // value
            req_value = std::stoi(get_remove_first_word(line));
            if (req_opcode.find("_set_") == string::npos) {
                // It's not a config intrinsic, config and value must be -1
                assert(req_dim == -1);
                assert(req_value == -1);
            }
        }
    }
#if (EXETYPE == DVI)
    if (req_type != Request::Type::FREE) {
        bubble_cnt = std::stoul(get_remove_first_word(line));
    }
#else
    assert(req_type != Request::Type::FREE);
    bubble_cnt = std::stoul(get_remove_first_word(line));
#endif

    if (req_type == Request::Type::GPIC) {
        if (req_dim != -1) {
            // it's a config instr
            hint("get_gpic_request returned type: GPIC, opcode: %s, dst: %ld, src1: %ld, src2: %ld, dim: %ld, value: %ld, bubble: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2, req_dim, req_value, bubble_cnt);
        } else if (req_stride.size() != 0) {
            // it's a load or store instruction
            hint("get_gpic_request returned type: GPIC, opcode: %s, dst: %ld, src1: %ld, src2: %ld, address: 0x%lx, stride: [%ld, %ld, %ld, %ld], bubble: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2, req_addr, req_stride[0], req_stride[1], req_stride[2], req_stride[3], bubble_cnt);
        } else {
            // it's a compute instruction
            hint("get_gpic_request returned type: GPIC, opcode: %s, dst: %ld, src1: %ld, src2: %ld, bubble: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2, bubble_cnt);
        }
#if (EXETYPE == DVI)
    } else if (req_type == Request::Type::FREE) {
        hint("get_gpic_request returned type: FREE, opcode: %s\n", req_opcode.c_str());
#endif
    } else if (req_type == Request::Type::READ) {
        hint("get_gpic_request returned type: LOAD, opcode: %s, dst: %ld, src1: %ld, src2: %ld, address: 0x%lx, bubble: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2, req_addr, bubble_cnt);
    } else if (req_type == Request::Type::WRITE) {
        hint("get_gpic_request returned type: STORE, opcode: %s, dst: %ld, src1: %ld, src2: %ld, address: 0x%lx, bubble: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2, req_addr, bubble_cnt);
    } else if (req_type == Request::Type::INITIALIZED) {
        hint("get_gpic_request type: INITIALIZED returned opcode: %s, dst: %ld, src1: %ld, src2: %ld\n", req_opcode.c_str(), req_dst, req_src1, req_src2);
    } else {
        assert(false);
    }
    return true;
}

bool Trace::get_unfiltered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type) {
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*files[trace_idx], line);
        } catch (const std::runtime_error &ex) {
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
                } catch (const std::runtime_error &ex) {
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

bool Trace::get_filtered_request(long &bubble_cnt, long &req_addr, Request::Type &req_type) {
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*files[trace_idx], line);
        } catch (const std::runtime_error &ex) {
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
                } catch (const std::runtime_error &ex) {
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

bool Trace::get_dramtrace_request(long &req_addr, Request::Type &req_type) {
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
        int trace_idx = (last_trace + trace_offset) % files.size();
        try {
            getline(*(files[trace_idx]), line);
        } catch (const std::runtime_error &ex) {
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
