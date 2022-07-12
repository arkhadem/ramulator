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

class Request {
public:
    // specify request id within the unit of core
    int reqid = -1;

    bool is_first_command = true;
    std::string opcode;
    long value = -1;
    long addr = -1;
    long stride = -1;
    enum class Type {
        READ,
        WRITE,
        REFRESH,
        POWERDOWN,
        SELFREFRESH,
        EXTENSION,
        GPIC,
        INITIALIZED,
        MAX
    } type;
    int vid = -1;
    int vid_dst = -1;
    int sid = -1;
    int sid_dst = -1;
    long data_type;
    std::vector<long> addr_starts;
    std::vector<long> addr_ends;
    // long addr_row;
    vector<int> addr_vec;

    // specify which core sent this request
    int coreid = -1;

    // specify which unit in the core sent this request
    enum class UnitID {
        CORE,
        L1,
        L2,
        L3,
        MAX
    } unitid;

    std::string unitid_string;
    std::map<UnitID, string> UNITID_2_STRING = {
        { UnitID::CORE, "CORE" },
        { UnitID::L1, "L1" },
        { UnitID::L2, "L2" },
        { UnitID::L3, "L3" },
        { UnitID::MAX, "MAX" },
    };

    long arrive = -1;
    long depart = -1;
    function<void(Request&)> callback; // call back with more info

    // For GPIC loads and stores
    Request(std::string& opcode, long addr, std::vector<long> addr_starts, std::vector<long> addr_ends, long data_type, long stride, int vid, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : opcode(opcode)
        , addr(addr)
        , stride(stride)
        , type(Type::GPIC)
        , vid(vid)
        , data_type(data_type)
        , addr_starts(addr_starts)
        , addr_ends(addr_ends)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    // For GPIC Computations
    Request(std::string& opcode, long data_type, int vid, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : opcode(opcode)
        , type(Type::GPIC)
        , vid(vid)
        , data_type(data_type)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    // For GPIC Configurations
    Request(std::string& opcode, long value, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : opcode(opcode)
        , value(value)
        , type(Type::GPIC)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    // For GPIC Moves
    Request(std::string& opcode, long data_type, int vid, int vid_dst, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : opcode(opcode)
        , type(Type::GPIC)
        , vid(vid)
        , vid_dst(vid_dst)
        , data_type(data_type)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    // For Bubbles
    Request(long addr, Type type, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : addr(addr)
        , type(type)
        , data_type(8)
        , coreid(coreid)
        , unitid(unitid)
        , callback([](Request& req) {})
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    // For CPU Loads and Stores
    Request(long addr, Type type, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : addr(addr)
        , type(type)
        , data_type(8)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        addr_starts.push_back(addr);
        addr_ends.push_back(addr + 7);
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : type(type)
        , addr_vec(addr_vec)
        , coreid(coreid)
        , unitid(unitid)
        , callback(callback)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : addr(-1)
        , type(Request::Type::MAX)
        , data_type(-1)
        , coreid(coreid)
        , unitid(unitid)
    {
        // assert(coreid == 0);
        unitid_string = UNITID_2_STRING[unitid];
    }

    friend bool operator==(const Request& req1, const Request& req2)
    {
        if ((req1.coreid == req2.coreid) && (req1.unitid == req2.unitid) && (req1.reqid == req2.reqid) && (req1.vid == req2.vid) && (req1.vid_dst == req2.vid_dst) && (req1.sid == req2.sid) && (req1.sid_dst == req2.sid_dst)) {
            assert(req1.data_type == req2.data_type);
            assert(req1.type == req2.type);
            assert(req1.opcode == req2.opcode);
            assert(req1.value == req2.value);
            return true;
        }
        return false;
    }

    friend bool operator<(const Request& req1, const Request& req2)
    {
        if (req1.coreid < req2.coreid) {
            return true;
        } else if (req1.coreid == req2.coreid) {
            if (req1.unitid < req2.unitid) {
                return true;
            } else if (req1.unitid == req2.unitid) {
                if (req1.reqid < req2.reqid) {
                    return true;
                }
            }
        }
        return false;
    }

    const char* c_str() const
    {
        std::stringstream req_stream;
        char* name = new char[1024];

        if (type == Type::GPIC) {
            if (value != -1) {
                // it's a config instr
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), value(" << value << ")]";
            } else if (addr != -1) {
                // it's a load or store instruction
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), data(" << data_type << "), addr(0x" << std::hex << addr << "), starts-ends(";
                int addr_to_print = addr_starts.size() > 8 ? 8 : addr_starts.size();
                for (int idx = 0; idx < addr_to_print; idx++) {
                    req_stream << "0x" << addr_starts[idx] << "-0x" << addr_ends[idx];
                    if (idx != addr_starts.size() - 1) {
                        req_stream << ", ";
                    }
                }
                if (addr_starts.size() > 8) {
                    req_stream << "...";
                }
                req_stream << std::dec << "), Stride(" << stride << "), VID(" << vid << ")";
                if (sid != -1) {
                    req_stream << ", SID(" << sid << ")";
                }
                req_stream << "]";
            } else if (vid_dst != -1) {
                // it's a move instruction
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), VID src(" << vid << "), VID dst(" << vid_dst << ")";
                if (sid != -1) {
                    req_stream << ", SID src(" << sid << "), SID dst(" << sid_dst << ")";
                }
                req_stream << "]";
            } else {
                // it's a compute instruction
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), VID(" << vid << ")";
                if (sid != -1) {
                    req_stream << ", SID(" << sid << ")";
                }
                req_stream << "]";
            }
        } else if (type == Type::READ) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(READ), addr(0x" << std::hex << addr << ")]";
        } else if (type == Type::WRITE) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(WRITE), addr(0x" << std::hex << addr << ")]";
        } else if (type == Type::INITIALIZED) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(INITIALIZED)]";
        } else if (type == Type::MAX) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(BUBBLE)]";
        } else {
            assert(false);
        }

        strcpy(name, req_stream.str().c_str());
        return name;
    }
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/
