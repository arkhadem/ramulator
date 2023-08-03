#include "Config.h"
#include "Controller.h"
#include "DRAM.h"
#include "Memory.h"
#include "Processor.h"
#include "Request.h"
#include "SpeedyController.h"
#include "Statistics.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <stdlib.h>
#include <string>
#include <vector>

/* Standards */
#include "ALDRAM.h"
#include "DDR3.h"
#include "DDR4.h"
#include "DSARP.h"
#include "GDDR5.h"
#include "Gem5Wrapper.h"
#include "HBM.h"
#include "LPDDR3.h"
#include "LPDDR4.h"
#include "PCM.h"
#include "SALP.h"
#include "STTMRAM.h"
#include "TLDRAM.h"
#include "WideIO.h"
#include "WideIO2.h"

using namespace std;
using namespace ramulator;

bool ramulator::warmup_complete = false;

std::map<core_type_t, core_config_t> ramulator::core_configs;

// int ramulator::l3_size = 1 << 23;
int ramulator::l3_size = 1 << 22;
// int ramulator::l3_assoc = 1 << 3;
int ramulator::l3_assoc = 1 << 2;
int ramulator::l3_blocksz = 1 << 6;
int ramulator::mshr_per_bank = 64;
float ramulator::l3_access_energy = 167.581634688;
int ramulator::l3_gpic_SA_num = 1024;

void declare_configuration(const Config &configs) {

    ramulator::core_configs[core_type_t::PRIME].l1_cache_config.size = 1 << 15;
    ramulator::core_configs[core_type_t::PRIME].l1_cache_config.assoc = 1 << 3;
    ramulator::core_configs[core_type_t::PRIME].l1_cache_config.blocksz = 1 << 6;
    ramulator::core_configs[core_type_t::PRIME].l1_cache_config.mshr_num = 12;
    ramulator::core_configs[core_type_t::PRIME].l1_cache_config.access_energy = 9.006323047;

    ramulator::core_configs[core_type_t::PRIME].l2_cache_config.size = 1 << 20;
    ramulator::core_configs[core_type_t::PRIME].l2_cache_config.assoc = 1 << 4;
    ramulator::core_configs[core_type_t::PRIME].l2_cache_config.blocksz = 1 << 6;
    ramulator::core_configs[core_type_t::PRIME].l2_cache_config.mshr_num = 46;
    ramulator::core_configs[core_type_t::PRIME].l2_cache_config.access_energy = 20.655976172;

    ramulator::core_configs[core_type_t::PRIME].ipc = 5;

    ramulator::core_configs[core_type_t::PRIME].gpic_SA_num = 1024;

    ramulator::core_configs[core_type_t::PRIME].out_of_order = true;
}

std::ofstream ramulator::op_trace;

template <typename T>
void run_dramtrace(const Config &configs, Memory<T, Controller> &memory, const std::vector<const char *> &files) {

    /* initialize DRAM trace */
    // assuming only one file
    assert(files.size() == 1);
    Trace trace(files);

    /* run simulation */
    bool stall = false, end = false;
    int reads = 0, writes = 0, clks = 0;
    long addr = 0;
    Request::Type type = Request::Type::READ;
    map<int, int> latencies;
    auto read_complete = [&latencies](Request &r) { latencies[r.depart - r.arrive]++; };

    Request req(addr, type, read_complete);

    while (!end || memory.pending_requests()) {
        if (!end && !stall) {
            end = !trace.get_dramtrace_request(addr, type, 0);
        }

        if (!end) {
            req.addr = addr;
            req.type = type;
            stall = !memory.send(req);
            if (!stall) {
                if (type == Request::Type::READ)
                    reads++;
                else if (type == Request::Type::WRITE)
                    writes++;
            }
        } else {
            memory.set_high_writeq_watermark(0.0f); // make sure that all write requests in the
                                                    // write queue are drained
        }

        memory.tick();
        clks++;
        Stats::curTick++; // memory clock, global, for Statistics
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}

template <typename T>
void run_cputrace(const Config &configs, Memory<T, Controller> &memory, const std::vector<const char *> &files) {
    int cpu_tick = configs.get_cpu_tick();
    int mem_tick = configs.get_mem_tick();
    auto send = bind(&Memory<T, Controller>::send, &memory, placeholders::_1);
    declare_configuration(configs);
    Processor proc(configs, files, send, memory);

    long warmup_insts = configs.get_warmup_insts();
    bool is_warming_up = (warmup_insts != 0);

    for (long i = 0; is_warming_up; i++) {
        proc.tick();
        Stats::curTick++;
        if (i % cpu_tick == (cpu_tick - 1))
            for (int j = 0; j < mem_tick; j++)
                memory.tick();

        is_warming_up = false;
        for (int c = 0; c < proc.cores.size(); c++) {
            if (proc.cores[c]->get_insts() < warmup_insts)
                is_warming_up = true;
        }

        if (is_warming_up && proc.has_reached_limit()) {
            printf("WARNING: The end of the input trace file was reached during warmup. "
                   "Consider changing warmup_insts in the config file. \n");
            break;
        }
    }

    warmup_complete = true;
    printf("Warmup complete! Resetting stats...\n");
    Stats::reset_stats();
    proc.reset_stats();
    assert(proc.get_insts() == 0);

    printf("Starting the simulation...\n");

    int tick_mult = cpu_tick * mem_tick;
    for (long i = 0;; i++) {
        if (((i % tick_mult) % mem_tick) == 0) { // When the CPU is ticked cpu_tick times,
            // the memory controller should be ticked mem_tick times
            proc.tick();
            Stats::curTick++; // processor clock, global, for Statistics

            if (configs.calc_weighted_speedup()) {
                if (proc.has_reached_limit()) {
                    break;
                }
            } else {
                if (configs.is_early_exit()) {
                    if (proc.finished())
                        break;
                } else {
                    if (proc.finished() && (memory.pending_requests() == 0))
                        break;
                }
            }
        }

        if (((i % tick_mult) % cpu_tick) == 0) // TODO_hasan: Better if the processor ticks the memory controller
            memory.tick();
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
}

template <typename T>
void run_gpictrace(const Config &configs, Memory<T, Controller> &memory, const std::vector<const char *> &files) {
    int cpu_tick = configs.get_cpu_tick();
    int mem_tick = configs.get_mem_tick();
    auto send = bind(&Memory<T, Controller>::send, &memory, placeholders::_1);
    declare_configuration(configs);
    Processor proc(configs, files, send, memory);

    bool is_warming_up = configs.is_warming_up();

    for (long i = 0; is_warming_up; i++) {
        proc.tick();
        Stats::curTick++;
        if (i % cpu_tick == (cpu_tick - 1))
            for (int j = 0; j < mem_tick; j++)
                memory.tick();

        is_warming_up = false;
        for (int c = 0; c < proc.cores.size(); c++) {
            if (proc.cores[c]->is_warmed_up() == false) {
                is_warming_up = true;
                break;
            }
        }

        if (is_warming_up && proc.has_reached_limit()) {
            printf("WARNING: The end of the input trace file was reached during warmup. "
                   "Consider changing warmup_insts in the config file. \n");
            break;
        }
    }

    warmup_complete = true;
    printf("Warmup complete! Resetting stats...\n");
    Stats::curTick = 0;
    Stats::reset_stats();
    proc.reset_state();
    assert(proc.get_insts() == 0);

    printf("Starting the simulation...\n");

    int tick_mult = cpu_tick * mem_tick;
    for (long i = 0;; i++) {
        if (((i % tick_mult) % mem_tick) == 0) { // When the CPU is ticked cpu_tick times,
            // the memory controller should be ticked mem_tick times
            proc.tick();
            Stats::curTick++; // processor clock, global, for Statistics

            if (configs.calc_weighted_speedup()) {
                if (proc.has_reached_limit()) {
                    break;
                }
            } else {
                if (configs.is_early_exit()) {
                    if (proc.finished())
                        break;
                } else {
                    if (proc.finished() && (memory.pending_requests() == 0))
                        break;
                }
            }
        }

        if (((i % tick_mult) % cpu_tick) == 0) // TODO_hasan: Better if the processor ticks the memory controller
            memory.tick();
    }
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
    proc.reset_state();
}

function<bool(Request &)> dc_send_mem;
vector<Request> dc_block_tosend_instrs[256];
int dc_instr_count[256];
bool dc_block_skip_instrs[256];
vector<Request> dc_block_sent_requests[256];
Request::Type dc_block_next_instr_type[256];
bool dc_trace_finished[256];
Trace *dc_trace;
bool read_new_block = false;
int id = 0;
int dc_total_accesses = 0;
long dc_instruction_retired = 0;
long dc_cpu_clks = 0;
std::vector<long> dc_mshr_entries;
int mshr_entry_num = 512;

long dc_align(long addr) {
    return (addr & ~(64 - 1l));
}

int hit_mshr(long addr) {
    for (int i = 0; i < dc_mshr_entries.size(); i++) {
        if (dc_align(addr) == dc_align(dc_mshr_entries[i]))
            return i;
    }
    return -1;
}

void dc_receive(Request &req) {
    hint("DC received %s\n", req.c_str());
    // Remove related MSHR entries
    int mshr_idx = hit_mshr(req.addr);

    if (mshr_idx != dc_mshr_entries.size()) {
        hint("0x%lx removed from mshr entries.\n", dc_mshr_entries[mshr_idx]);
        dc_mshr_entries.erase(dc_mshr_entries.begin() + mshr_idx);
    }

    // Removing corresponding DC sent requests
    for (int block_idx = 0; block_idx < 256; block_idx++) {
        auto req_it = dc_block_sent_requests[block_idx].begin();
        bool hit = false;
        while (req_it != dc_block_sent_requests[block_idx].end()) {
            if (dc_align(req.addr) == dc_align(req_it->addr)) {
                hint("Block [%d]: %s hitted with %s, removing from sent list\n", block_idx, req.c_str(), req_it->c_str());
                req_it = dc_block_sent_requests[block_idx].erase(req_it);
                hit = true;
            } else {
                ++req_it;
            }
        }
        if (hit && (dc_block_sent_requests[block_idx].size() == 0)) {
            if (dc_block_tosend_instrs[block_idx].size() == 1) {
                assert(dc_block_tosend_instrs[block_idx][0].type == Request::Type::MAX);
                op_trace << dc_cpu_clks << " " << block_idx << " F " << dc_instr_count[block_idx] << endl;
                dc_block_tosend_instrs[block_idx].erase(dc_block_tosend_instrs[block_idx].begin());
                hint("Block [%d]: removed instruction!\n", block_idx);
                dc_instruction_retired = dc_cpu_clks;
            }
        }
    }
}

bool dc_send(Request req) {
    if (hit_mshr(req.addr) != -1) {
        hint("Hit in MSHR\n");
        return true;
    }
    if (dc_mshr_entries.size() == mshr_entry_num) {
        // When no MSHR entries available, the miss request is stalling.
        hint("No MSHR entry available\n");
        return false;
    }
    if (dc_send_mem(req) == true) {
        hint("MSHR allocated, sending to memory!\n");
        dc_mshr_entries.push_back(req.addr);
        return true;
    } else {
        return false;
    }
    return true;
}

bool get_new_instruction(int block) {
    long addr = 0;
    Request::Type type = Request::Type::READ;
    assert(dc_trace_finished[block] == false);
    assert(dc_block_tosend_instrs[block].size() == 0);
    while (dc_trace_finished[block] == false) {
        dc_trace_finished[block] = !dc_trace->get_dramtrace_request(addr, type, block);
        if (dc_trace_finished[block] == true)
            break;
        if (type == Request::Type::DC_BLOCK) {
            dc_block_skip_instrs[block] = (addr % 256 == block) ? false : true;
            if (dc_block_tosend_instrs[block].size() != 0) {
                break;
            }
        } else {
            if (dc_block_skip_instrs[block] == true)
                continue;
            if (type == Request::Type::READ) {
                dc_block_next_instr_type[block] = Request::Type::READ;
                if (dc_block_tosend_instrs[block].size() != 0) {
                    break;
                }
            } else if (type == Request::Type::WRITE) {
                dc_block_next_instr_type[block] = Request::Type::WRITE;
                if (dc_block_tosend_instrs[block].size() != 0) {
                    break;
                }
            } else {
                assert(type == Request::Type::MAX);
                assert((dc_block_next_instr_type[block] == Request::Type::READ) || (dc_block_next_instr_type[block] == Request::Type::WRITE));
                Request req(addr, dc_block_next_instr_type[block]);
                req.coreid = 0;
                req.callback = dc_receive;
                req.dc_blockid = block;
                req.addr = dc_align(addr);
                req.type = dc_block_next_instr_type[block];
                req.reqid = id++;
                dc_block_tosend_instrs[block].push_back(req);
                dc_total_accesses++;
            }
        }
    }
    if (dc_block_tosend_instrs[block].size() != 0) {
        Request fake_req = Request(0, Request::Type::MAX);
        assert(fake_req.type == Request::Type::MAX);
        dc_block_tosend_instrs[block].push_back(fake_req);
        dc_instr_count[block] += 1;
        op_trace << dc_cpu_clks << " " << block << " S " << dc_instr_count[block] << endl;
        hint("Block [%d]: received new %s instructions with %d memory accesses!\n", block, dc_block_tosend_instrs[block][0].type == Request::Type::READ ? "R" : "W", (int)dc_block_tosend_instrs[block].size());
        return true;
    } else {
        return false;
    }
}

bool dc_blocks_clock(int block) {
    hint("Block [%d]: Clocking...\n", block);
    if (dc_block_tosend_instrs[block].size() == 0) {
        if (dc_trace_finished[block])
            return true;
        hint("Block [%d]: instructions finished, requesting new instructions!\n", block);
        if (get_new_instruction(block) == false) {
            hint("Block [%d]: finished everything, quitting!\n", block);
            return true;
        }
        assert(dc_block_tosend_instrs[block].size() != 0);
    }
    while (dc_block_tosend_instrs[block].size() > 1) {
        Request req = dc_block_tosend_instrs[block][0];
        assert((req.type == Request::Type::READ) || (req.type == Request::Type::WRITE));
        if (dc_send(req) == false) {
            hint("Block [%d]: Mem addr failed to be sent (%s)\n", block, req.c_str());
            return false;
        } else {
            hint("Block [%d]: Mem addr sent, removed from tosend and added to sent (%s)\n", block, req.c_str());
            dc_block_sent_requests[block].push_back(req);
            dc_block_tosend_instrs[block].erase(dc_block_tosend_instrs[block].begin());
        }
    }
    return true;
}

void dc_blocks_clock_all() {
    hint("Clocker@[%ld]: clocking blocks\n", dc_cpu_clks);
    int block_start = rand() % 256;
    for (int block_offset = 0; block_offset < 256; block_offset++) {
        int block_idx = (block_start + block_offset) % 256;
        if (dc_blocks_clock(block_idx) == false) {
            hint("Clocker: ignoring clocking rest of blocks because %d failed to send\n", block_idx);
            break;
        }
    }
}

template <typename T>
void run_dctrace(const Config &configs, Memory<T, Controller> &memory, const std::vector<const char *> &files) {
    dc_trace = new Trace(files, 256);
    int cpu_tick = configs.get_cpu_tick();
    int mem_tick = configs.get_mem_tick();

    dc_send_mem = bind(&Memory<T, Controller>::send, &memory, placeholders::_1);
    declare_configuration(configs);

    for (int i = 0; i < 256; i++) {
        dc_block_next_instr_type[i] = Request::Type::MAX;
        dc_block_skip_instrs[i] = true;
        dc_trace_finished[i] = false;
        dc_instr_count[i] = 0;
    }

    /* run simulation */
    int clks = 0;
    int tick_mult = cpu_tick * mem_tick;
    bool sim_finished = false;
    while (sim_finished == false) {
        if (dc_trace_finished[0] && memory.pending_requests() == false) {
            for (int block_idx = 0; block_idx < 256; block_idx++) {
                if (dc_trace_finished[block_idx] == false)
                    break;
            }
            sim_finished = true;
            for (int block_idx = 0; block_idx < 256; block_idx++) {
                if (dc_block_tosend_instrs[block_idx].size() != 0) {
                    sim_finished = false;
                    break;
                }
                if (dc_block_sent_requests[block_idx].size() != 0) {
                    sim_finished = false;
                    break;
                }
            }
        }

        if (((clks % tick_mult) % mem_tick) == 0) { // When the CPU is ticked cpu_tick times,
            dc_blocks_clock_all();
            dc_cpu_clks++;
            if (dc_cpu_clks % 100000 == 0) {
                printf("DC heartbeat, cycles: %ld \n", dc_cpu_clks);
                if (dc_cpu_clks - dc_instruction_retired > 1000000) {
                    printf("Error: last instruction received goes back to cycle %ld, quitting!\n", dc_instruction_retired);
                    exit(-1);
                }
            }
        }
        if (((clks % tick_mult) % cpu_tick) == 0) {
            memory.tick();
        }
        clks++;
        Stats::curTick++;
    }
    assert(dc_mshr_entries.size() == 0);
    // This a workaround for statistics set only initially lost in the end
    memory.finish();
    Stats::statlist.printall();
    printf("finished %d accesses in %ld clks\n", dc_total_accesses, dc_cpu_clks);
}

template <typename T>
void start_run(const Config &configs, T *spec, const vector<const char *> &files) {
    // initiate controller and memory
    int C = configs.get_channels(), R = configs.get_ranks();
    // Check and Set channel, rank number
    spec->set_channel_number(C);
    spec->set_rank_number(R);
    std::vector<Controller<T> *> ctrls;
    int prev_children = 0;
    for (int c = 0; c < C; c++) {
        DRAM<T> *channel = new DRAM<T>(spec, T::Level::Channel);
        channel->id = c;
        channel->regStats("");
        prev_children = channel->set_index(prev_children);
        Controller<T> *ctrl = new Controller<T>(configs, channel);
        ctrls.push_back(ctrl);
    }
    Memory<T, Controller> memory(configs, ctrls);

    assert(files.size() != 0);
    if (configs["trace_type"] == "CPU") {
        run_cputrace(configs, memory, files);
    } else if (configs["trace_type"] == "DRAM") {
        run_dramtrace(configs, memory, files);
    } else if (configs["trace_type"] == "GPIC") {
        run_gpictrace(configs, memory, files);
    } else if (configs["trace_type"] == "DC") {
        run_dctrace(configs, memory, files);
    }
}

int main(int argc, const char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <configs-file> --mode=cpu,dram,gpic [--warmup] [--core=<core-num> <core1-type> ...] [--stats <filename>] <trace-filename1> <trace-filename2>\n"
               "Example: %s ramulator-configs.cfg --mode=gpic --core=1 gold cpu.trace cpu.trace\n",
               argv[0], argv[0]);
        return 0;
    }

    Config configs(argv[1]);

    const std::string &standard = configs["standard"];
    assert(standard != "" || "DRAM standard should be specified.");

    const char *trace_type = strstr(argv[2], "=");
    trace_type++;
    if (strcmp(trace_type, "cpu") == 0) {
        configs.add("trace_type", "CPU");
    } else if (strcmp(trace_type, "dram") == 0) {
        configs.add("trace_type", "DRAM");
    } else if (strcmp(trace_type, "gpic") == 0) {
        configs.add("trace_type", "GPIC");
        assert((configs.has_core_caches() && configs.has_l3_cache()) || "GPIC mode need \"all\" cache levels");
    } else if (strcmp(trace_type, "dc") == 0) {
        configs.add("trace_type", "DC");
    } else {
        printf("invalid trace type: %s\n", trace_type);
        assert(false);
    }

    int trace_start = 3;

    if (strstr(argv[3], "--warmup") != nullptr) {
        configs.set_warming_up();
        trace_start++;
    }

    int core_num = 0;
    std::vector<std::string> core_types;
    const char *core_num_str = strstr(argv[trace_start], "--core=");
    if (core_num_str != nullptr) {
        core_num_str = strstr(argv[trace_start], "=");
        core_num_str++;
        core_num = stoi(core_num_str);
        if (core_num < 1 || core_num > 16) {
            printf("core num must be >= 1 and <= 16\n");
            assert(false);
        }
        trace_start++;
        for (int core_idx = 0; core_idx < core_num; core_idx++) {
            const char *core_type = argv[trace_start];
            trace_start++;
            assert((strcmp(core_type, "silver") == 0) || (strcmp(core_type, "gold") == 0) || (strcmp(core_type, "prime") == 0));
            core_types.push_back(std::string(core_type));
        }
    }

    string stats_out;
    if (strcmp(argv[trace_start], "--stats") == 0) {
        Stats::statlist.output(argv[trace_start + 1]);
        stats_out = argv[trace_start + 1];
        trace_start += 2;
    } else {
        Stats::statlist.output(standard + ".stats");
        stats_out = standard + string(".stats");
    }

    op_trace.open(stats_out + ".map");
    if (!op_trace.is_open()) {
        printf("Could not open %s.map\n", (stats_out + ".map").c_str());
        return -1;
    }

    // A separate file defines mapping for easy config.
    if (strcmp(argv[trace_start], "--mapping") == 0) {
        configs.add("mapping", argv[trace_start + 1]);
        trace_start += 2;
    } else {
        configs.add("mapping", "defaultmapping");
    }

    std::vector<const char *> files(&argv[trace_start], &argv[argc]);
    if (core_num != 0) {
        if (argc - trace_start < core_num) {
            printf("number of trace files must be >= core num\n");
            assert(false);
        }
        configs.set_core_num(core_num);
        configs.set_core_types(core_types);
    } else {
        configs.set_core_num(argc - trace_start);
    }

    if (standard == "DDR3") {
        DDR3 *ddr3 = new DDR3(configs["org"], configs["speed"]);
        start_run(configs, ddr3, files);
    } else if (standard == "DDR4") {
        DDR4 *ddr4 = new DDR4(configs["org"], configs["speed"]);
        start_run(configs, ddr4, files);
    } else if (standard == "SALP-MASA") {
        SALP *salp8 = new SALP(configs["org"], configs["speed"], "SALP-MASA", configs.get_subarrays());
        start_run(configs, salp8, files);
    } else if (standard == "LPDDR3") {
        LPDDR3 *lpddr3 = new LPDDR3(configs["org"], configs["speed"]);
        start_run(configs, lpddr3, files);
    } else if (standard == "LPDDR4") {
        // total cap: 2GB, 1/2 of others
        LPDDR4 *lpddr4 = new LPDDR4(configs["org"], configs["speed"]);
        start_run(configs, lpddr4, files);
    } else if (standard == "GDDR5") {
        GDDR5 *gddr5 = new GDDR5(configs["org"], configs["speed"]);
        start_run(configs, gddr5, files);
    } else if (standard == "HBM") {
        HBM *hbm = new HBM(configs["org"], configs["speed"]);
        start_run(configs, hbm, files);
    } else if (standard == "WideIO") {
        // total cap: 1GB, 1/4 of others
        WideIO *wio = new WideIO(configs["org"], configs["speed"]);
        start_run(configs, wio, files);
    } else if (standard == "WideIO2") {
        // total cap: 2GB, 1/2 of others
        WideIO2 *wio2 = new WideIO2(configs["org"], configs["speed"], configs.get_channels());
        wio2->channel_width *= 2;
        start_run(configs, wio2, files);
    } else if (standard == "STTMRAM") {
        STTMRAM *sttmram = new STTMRAM(configs["org"], configs["speed"]);
        start_run(configs, sttmram, files);
    } else if (standard == "PCM") {
        PCM *pcm = new PCM(configs["org"], configs["speed"]);
        start_run(configs, pcm, files);
    }
    // Various refresh mechanisms
    else if (standard == "DSARP") {
        DSARP *dsddr3_dsarp = new DSARP(configs["org"], configs["speed"], DSARP::Type::DSARP, configs.get_subarrays());
        start_run(configs, dsddr3_dsarp, files);
    } else if (standard == "ALDRAM") {
        ALDRAM *aldram = new ALDRAM(configs["org"], configs["speed"]);
        start_run(configs, aldram, files);
    } else if (standard == "TLDRAM") {
        TLDRAM *tldram = new TLDRAM(configs["org"], configs["speed"], configs.get_subarrays());
        start_run(configs, tldram, files);
    }

    op_trace.close();
    printf("Simulation done. Statistics written to %s\n", stats_out.c_str());

    return 0;
}
