#ifndef __REQUEST_H
#define __REQUEST_H

#include "Config.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace ramulator {

class Request {
public:
    int reqid = -1;

    std::string opcode = "BUBBLE";
    long dst_reg_count = -1;
    long dst_mem_count = -1;
    long src_reg_count = -1;
    long src_mem_count = -1;
    long dst_reg = -1;
    long dst_reg_type = -1;
    long src1_reg = -1;
    long src1_reg_type = -1;
    long src2_reg = -1;
    long src2_reg_type = -1;
    long src3_reg = -1;
    long src3_reg_type = -1;
    long addr = -1;
    long addr_end = -1;
    long latency = -1;
    long guard = -1;
    long pipeline_1 = -1;
    bool pipeline_1_ready = true;
    bool pipeline_1_idx = -1;
    long pipeline_2 = -1;
    bool pipeline_2_ready = true;
    bool pipeline_2_idx = -1;
    int state = INSTRUCTION_NOT_FETCHED;
    bool mem_sent = false;
    bool mem_received = false;

    long dispatch_cycle = -1;
    long issued_P1_cycle = -1;
    long issued_P2_cycle = -1;
    long start_P1_execute_cycle = -1;
    long start_P2_execute_cycle = -1;
    long finish_P1_execute_cycle = -1;
    long finish_P2_execute_cycle = -1;
    long retired_cycle = -1;

    int dispatch_idx = -1;
    int issue_idx = -1;

    bool is_first_command;
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
        NEON,
        MAX
    } type;

    long arrive = -1;
    long depart = -1;
    function<void(Request &)> callback; // call back with more info

    // For NEON Computations
    Request(std::string opcode,
            long dst_reg_count,
            long dst_mem_count,
            long src_reg_count,
            long src_mem_count,
            long dst_reg,
            long dst_reg_type,
            long src1_reg,
            long src1_reg_type,
            long src2_reg,
            long src2_reg_type,
            long src3_reg,
            long src3_reg_type,
            long addr,
            long addr_end,
            long latency,
            long guard,
            long pipeline_1,
            long pipeline_2,
            function<void(Request &)> callback)
        : opcode(opcode),
          dst_reg_count(dst_reg_count),
          dst_mem_count(dst_mem_count),
          src_reg_count(src_reg_count),
          src_mem_count(src_mem_count),
          dst_reg(dst_reg),
          dst_reg_type(dst_reg_type),
          src1_reg(src1_reg),
          src1_reg_type(src1_reg_type),
          src2_reg(src2_reg),
          src2_reg_type(src2_reg_type),
          src3_reg(src3_reg),
          src3_reg_type(src3_reg_type),
          addr(addr),
          addr_end(addr_end),
          latency(latency),
          guard(guard),
          pipeline_1(pipeline_1),
          pipeline_2(pipeline_2),
          callback(callback) {

        coreid = 0;
        type = Type::NEON;
        pipeline_1_ready = (pipeline_1 == -1) ? true : false;
        pipeline_2_ready = (pipeline_2 == -1) ? true : false;
    }

    Request(std::string opcode,
            long addr,
            long addr_end,
            Type type,
            function<void(Request &)> callback)
        : opcode(opcode),
          addr(addr),
          addr_end(addr_end),
          type(type),
          callback(callback) {

        assert(type == Type::READ || type == Type::WRITE);
        dst_reg_count = -1;
        dst_mem_count = -1;
        src_reg_count = -1;
        src_mem_count = -1;
        dst_reg = -1;
        dst_reg_type = -1;
        src1_reg = -1;
        src1_reg_type = -1;
        src2_reg = -1;
        src2_reg_type = -1;
        src3_reg = -1;
        src3_reg_type = -1;
        coreid = 0;
        latency = -1;
        guard = -1;
        pipeline_1 = PIPELINE_L_TYPE;
        pipeline_2 = -1;
        pipeline_1_ready = false;
        pipeline_2_ready = true;
    }

    Request(Type type) : type(type) {
        assert(type == Type::MAX);
        pipeline_1 = -1;
        pipeline_2 = -1;
        pipeline_1_ready = true;
        pipeline_2_ready = true;
    }

    Request(long addr, Type type, int coreid = 0)
        : addr(addr), is_first_command(true), coreid(coreid), type(type),
          callback([](Request &req) {}) {
        assert(type == Type::WRITE);
        pipeline_1 = -1;
        pipeline_2 = -1;
        pipeline_1_ready = true;
        pipeline_2_ready = true;
    }

    Request(long addr, Type type, function<void(Request &)> callback, int coreid = 0)
        : addr(addr), is_first_command(true), coreid(coreid), type(type), callback(callback) {
        assert((type == Type::READ) || (type == Type::WRITE));
        pipeline_1 = -1;
        pipeline_2 = -1;
        pipeline_1_ready = true;
        pipeline_2_ready = true;
    }

    Request(vector<int> &addr_vec, Type type, function<void(Request &)> callback, int coreid = 0)
        : is_first_command(true), addr_vec(addr_vec), coreid(coreid), type(type), callback(callback) {}

    Request()
        : is_first_command(true), coreid(0) {}

    const char *c_str() const {
        std::stringstream req_stream;
        char *name = new char[1024];
        if (type == Type::NEON) {
            req_stream << "Request[Core(" << coreid << "), ";
            req_stream << "ID(" << reqid << "), ";
            req_stream << "TYPE(NEON), ";
            req_stream << "OPC(" << opcode << "), ";
            if (dst_reg_count == 1) {
                req_stream << "RDST(" << dst_reg << "), ";
            }
            if (dst_mem_count == 1) {
                req_stream << "MDST(0x" << std::hex << addr << "-0x" << addr_end << "), ";
            }
            if (src_reg_count == 1) {
                req_stream << "RSRC1(" << src1_reg << "), ";
            }
            if (src_reg_count == 2) {
                req_stream << "RSRC2(" << src2_reg << "), ";
            }
            if (src_reg_count == 3) {
                req_stream << "RSRC3(" << src3_reg << "), ";
            }
            if (src_mem_count == 1) {
                req_stream << "MSRC(0x" << std::hex << addr << "-0x" << addr_end << "), ";
            }
            req_stream << "LAT(" << latency << "), ";
            req_stream << "GRD(" << guard << "), ";
            if (pipeline_1 != -1) {
                req_stream << "P1(" << pipeline_1 << ": " << pipeline_1_ready << "), ";
            }
            if (pipeline_2 != -1) {
                req_stream << "P2(" << pipeline_2 << ": " << pipeline_2_ready << "), ";
            }
            req_stream << "STATE(" << state << "), ";
            req_stream << "MSENT(" << mem_sent << "), ";
            req_stream << "MRECE(" << mem_received << ")]";
        } else if (type == Type::READ) {
            req_stream << "Request[Core(" << coreid << "), ID(" << reqid << "), type(READ), addr(0x" << std::hex << addr << "-0x" << addr_end << ")]";
        } else if (type == Type::WRITE) {
            req_stream << "Request[Core(" << coreid << "), ID(" << reqid << "), type(WRITE), addr(0x" << std::hex << addr << "-0x" << addr_end << ")]";
        } else if (type == Type::MAX) {
            req_stream << "Request[Core(" << coreid << "), ID(" << reqid << "), type(BUBBLE)]";
        } else {
            assert(false);
        }

        strcpy(name, req_stream.str().c_str());
        return name;
    }
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/
