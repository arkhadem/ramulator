#ifndef __CONFIG_H
#define __CONFIG_H

#include <cassert>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// #define DEBUG

#ifndef DEBUG
#define hint(...)
#else
#define hint(...)            \
    do {                     \
        printf(__VA_ARGS__); \
    } while (0)
#endif

#define PIPELINE_V0_TYPE 0
#define PIPELINE_V1_TYPE 1
#define PIPELINE_V_TYPE 2
#define PIPELINE_L_TYPE 3
#define PIPELINE_B_TYPE 4
#define PIPELINE_S_TYPE 5
#define PIPELINE_I_TYPE 6
#define PIPELINE_M_TYPE 7
#define PIPELINE_D_TYPE 8

#define INSTRUCTION_NOT_FETCHED -1
#define INSTRUCTION_DISPATCHED 0
#define INSTRUCTION_ISSUED_P1 1
#define INSTRUCTION_ISSUED_P2 2
#define INSTRUCTION_EXECUTED_P1 3
#define INSTRUCTION_EXECUTED_P2 4
#define INSTRUCTION_RETIREMENT 5

#ifndef NUM_V0_PIPELINES
#define NUM_V0_PIPELINES 1
#endif

#ifndef NUM_V1_PIPELINES
#define NUM_V1_PIPELINES 1
#endif

#ifndef NUM_L_PIPELINES
#define NUM_L_PIPELINES 2
#endif

namespace ramulator {

enum core_type_t { SILVER,
                   GOLD,
                   PRIME };

struct cache_config_t {
    int size;
    int assoc;
    int blocksz;
    int mshr_num;
};

struct core_config_t {
    int ipc;
    bool out_of_order;
    cache_config_t l1_cache_config;
    cache_config_t l2_cache_config;
};

extern std::ofstream op_trace;

class Config {

private:
    std::map<std::string, std::string> options;
    int channels;
    int ranks;
    int subarrays;
    int cpu_tick;
    int mem_tick;
    int core_num = 0;
    long expected_limit_insts = 0;
    long warmup_insts = 0;

public:
    Config() {}
    Config(const std::string &fname);
    void parse(const std::string &fname);
    std::string operator[](const std::string &name) const {
        if (options.find(name) != options.end()) {
            return (options.find(name))->second;
        } else {
            return "";
        }
    }

    bool contains(const std::string &name) const {
        if (options.find(name) != options.end()) {
            return true;
        } else {
            return false;
        }
    }

    void add(const std::string &name, const std::string &value) {
        if (!contains(name)) {
            options.insert(make_pair(name, value));
        } else {
            printf("ramulator::Config::add options[%s] already set.\n", name.c_str());
        }
    }

    void set_core_num(int _core_num) { core_num = _core_num; }

    int get_channels() const { return channels; }
    int get_subarrays() const { return subarrays; }
    int get_ranks() const { return ranks; }
    int get_cpu_tick() const { return cpu_tick; }
    int get_mem_tick() const { return mem_tick; }
    int get_core_num() const { return core_num; }
    long get_expected_limit_insts() const { return expected_limit_insts; }
    long get_warmup_insts() const { return warmup_insts; }

    bool has_l3_cache() const {
        if (options.find("cache") != options.end()) {
            const std::string &cache_option = (options.find("cache"))->second;
            return (cache_option == "all") || (cache_option == "L3");
        } else {
            return false;
        }
    }
    bool has_core_caches() const {
        if (options.find("cache") != options.end()) {
            const std::string &cache_option = (options.find("cache"))->second;
            return (cache_option == "all" || cache_option == "L1L2");
        } else {
            return false;
        }
    }
    bool is_early_exit() const {
        // the default value is true
        if (options.find("early_exit") != options.end()) {
            if ((options.find("early_exit"))->second == "off") {
                return false;
            }
            return true;
        }
        return true;
    }
    bool calc_weighted_speedup() const {
        return (expected_limit_insts != 0);
    }
    bool record_cmd_trace() const {
        // the default value is false
        if (options.find("record_cmd_trace") != options.end()) {
            if ((options.find("record_cmd_trace"))->second == "on") {
                return true;
            }
            return false;
        }
        return false;
    }
    bool print_cmd_trace() const {
        // the default value is false
        if (options.find("print_cmd_trace") != options.end()) {
            if ((options.find("print_cmd_trace"))->second == "on") {
                return true;
            }
            return false;
        }
        return false;
    }
};

} /* namespace ramulator */

#endif /* _CONFIG_H */
