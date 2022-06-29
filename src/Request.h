#ifndef __REQUEST_H
#define __REQUEST_H

#include "Config.h"
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

namespace ramulator {

typedef struct BatchStat {
    long total_batches = 0;
    long total_requests = 0;
    long total_cycles = 0;
    long late_batches = 0;
    long total_late_cycles = 0;

    BatchStat operator+(const BatchStat& p) const
    {
        BatchStat tmp;
        tmp.total_batches = total_batches + p.total_batches;
        tmp.total_requests = total_requests + p.total_requests;
        tmp.total_cycles = total_cycles + p.total_cycles;
        tmp.late_batches = late_batches + p.late_batches;
        tmp.total_late_cycles = total_late_cycles + p.total_late_cycles;
        return tmp;
    }

    BatchStat operator-(const BatchStat& p) const
    {
        if (total_cycles > p.total_cycles)
            return *this;
        else
            return p;
    }
} BatchStat;

class Request {
public:
    bool is_first_command;
    long addr;
    // long addr_row;
    vector<int> addr_vec;
    // specify which core this request sent from, for virtual address translation
    int coreid;

    enum class Type {
        READ,
        WRITE,
        REFRESH,
        POWERDOWN,
        SELFREFRESH,
        EXTENSION,
        MAX
    } type;

    long arrive = -1;
    long depart = -1;
    function<void(Request&)> callback; // call back with more info

    Request(long addr, Type type, int coreid = 0)
        : is_first_command(true)
        , addr(addr)
        , coreid(coreid)
        , type(type)
        , callback([](Request& req) {})
    {
    }

    Request(long addr, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true)
        , addr(addr)
        , coreid(coreid)
        , type(type)
        , callback(callback)
    {
    }

    Request(vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true)
        , addr_vec(addr_vec)
        , coreid(coreid)
        , type(type)
        , callback(callback)
    {
    }

    Request()
        : is_first_command(true)
        , coreid(0)
    {
    }

    const char* c_str()
    {
        std::stringstream req_stream;
        char* name = new char[512];
        assert((type == Type::READ) || (type == Type::WRITE));
        req_stream << "Request[type(" << ((type == Type::READ) ? "READ" : "WRITE") << "), addr(0x" << std::hex << addr << ")";
        strcpy(name, req_stream.str().c_str());
        return name;
    }
};
} /*namespace ramulator*/

#endif /*__REQUEST_H*/
