#ifndef __CONFIG_H
#define __CONFIG_H

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#define MAX_CORE_ID 16
#define MAX_GPIC_SA_NUM 64

#ifndef LANES_PER_SA
#define LANES_PER_SA 1024
#endif

// #define DEBUG

#ifndef DEBUG
#define hint(...)
#else
#define hint(...)            \
    do {                     \
        printf(__VA_ARGS__); \
    } while (0)
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
    float access_energy;
};

struct core_config_t {
    int ipc;
    int gpic_core_num;
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
    bool warmup = false;
    long warmup_insts = 0;
    int gpic_level = 1;
    std::vector<core_type_t> core_types;

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
    void set_core_types(std::vector<std::string> _core_types) {
        assert(_core_types.size() == core_num);
        for (int i = 0; i < core_num; i++) {
            if (_core_types[i].compare("silver") == 0) {
                core_types.push_back(SILVER);
            } else if (_core_types[i].compare("gold") == 0) {
                core_types.push_back(GOLD);
            } else if (_core_types[i].compare("prime") == 0) {
                core_types.push_back(PRIME);
            } else {
                assert(false);
            }
        }
    }
    void set_warming_up() { warmup = true; }

    int get_channels() const { return channels; }
    int get_subarrays() const { return subarrays; }
    int get_ranks() const { return ranks; }
    int get_cpu_tick() const { return cpu_tick; }
    int get_mem_tick() const { return mem_tick; }
    int get_core_num() const { return core_num; }
    core_type_t get_core_type(int _core_num) const { return core_types[_core_num]; }
    long get_expected_limit_insts() const { return expected_limit_insts; }
    long get_warmup_insts() const { return warmup_insts; }
    int get_gpic_level() const { return gpic_level; }

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
    bool is_gpic() const {
        // the default value is false
        if ((options.find("trace_type"))->second == "GPIC") {
            return true;
        }
        return false;
    }
    bool is_warming_up() const {
        return warmup;
    }
};

} /* namespace ramulator */

#endif /* _CONFIG_H */
