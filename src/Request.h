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
    bool is_first_command;
    long addr;
    long addr_end;

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
    // specify request id within the unit of core
    int reqid = -1;

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

    std::string opcode;
    int en = 0;

    long arrive = -1;
    long depart = -1;
    function<void(Request&)> callback; // call back with more info

    Request(std::string& opcode, int en, long addr, long addr_end, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : is_first_command(true)
        , addr(addr)
        , addr_end(addr_end)
        , coreid(coreid)
        , unitid(unitid)
        , type(Type::GPIC)
        , opcode(opcode)
        , en(en)
        , callback(callback)
    {
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(long addr, Type type, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : is_first_command(true)
        , addr(addr)
        , addr_end(addr + 7)
        , coreid(coreid)
        , unitid(unitid)
        , type(type)
        , callback([](Request& req) {})
    {
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(long addr, Type type, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : is_first_command(true)
        , addr(addr)
        , addr_end(addr + 7)
        , coreid(coreid)
        , unitid(unitid)
        , type(type)
        , callback(callback)
    {
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : is_first_command(true)
        , addr_vec(addr_vec)
        , coreid(coreid)
        , unitid(unitid)
        , type(type)
        , callback(callback)
    {
        unitid_string = UNITID_2_STRING[unitid];
    }

    Request(int coreid = MAX_CORE_ID, UnitID unitid = UnitID::MAX)
        : is_first_command(true)
        , addr(-1)
        , addr_end(-1)
        , coreid(coreid)
        , unitid(unitid)
        , type(Request::Type::MAX)
    {
        unitid_string = UNITID_2_STRING[unitid];
    }

    friend bool operator==(const Request& req1, const Request& req2)
    {
        if ((req1.coreid == req2.coreid) && (req1.unitid == req2.unitid) && (req1.reqid == req2.reqid)) {
            assert(req1.addr == req2.addr);
            assert(req1.addr_end == req2.addr_end);
            assert(req1.type == req2.type);
            assert(req1.opcode == req2.opcode);
            assert(req1.en == req2.en);
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

    const char* c_str()
    {
        std::stringstream req_stream;
        char* name = new char[100];

        if (type == Type::GPIC) {
            if (addr != -1)
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), en(" << en << "), addr(0x" << std::hex << addr << ")]";
            else
                req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(GPIC), opcode(" << opcode << "), en(" << en << ")]";
        } else if (type == Type::READ) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(READ), addr(0x" << std::hex << addr << ")]";
        } else if (type == Type::WRITE) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(WRITE), addr(0x" << std::hex << addr << ")]";
        } else if (type == Type::INITIALIZED) {
            req_stream << "Request[Core(" << coreid << "), Unit(" << unitid_string << "), ID(" << reqid << ")][type(INITIALIZED)]";
        } else {
            assert(false);
        }

        strcpy(name, req_stream.str().c_str());
        return name;
    }
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/
