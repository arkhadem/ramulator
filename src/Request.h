#ifndef __REQUEST_H
#define __REQUEST_H

#include <vector>
#include <functional>
#include <cstdio>
#include <iostream> 
#include <sstream>

using namespace std;

namespace ramulator
{

class Request
{
public:
    bool is_first_command;
    long addr;
    long addr_end;
    // long addr_row;
    vector<int> addr_vec;
    // specify which core this request sent from, for virtual address translation
    int coreid;

    enum class Type
    {
        READ,
        WRITE,
        REFRESH,
        POWERDOWN,
        SELFREFRESH,
        EXTENSION,
        GPIC,
        MAX
    } type;

    std::string opcode;
    int en = 0;

    long arrive = -1;
    long depart = -1;
    function<void(Request&)> callback; // call back with more info

    Request(std::string& opcode, int en, long addr, long addr_end, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), addr(addr), addr_end(addr_end), coreid(coreid), type(Type::GPIC), opcode(opcode), en(en), callback(callback) {}

    Request(long addr, Type type, int coreid = 0)
        : is_first_command(true), addr(addr), addr_end(addr+7), coreid(coreid), type(type),
      callback([](Request& req){}) {}

    Request(long addr, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), addr(addr), addr_end(addr+7), coreid(coreid), type(type), callback(callback) {}

    Request(vector<int>& addr_vec, Type type, function<void(Request&)> callback, int coreid = 0)
        : is_first_command(true), addr_vec(addr_vec), coreid(coreid), type(type), callback(callback) {}

    Request()
        : is_first_command(true), coreid(0) {}

    char* c_str() {
        std::stringstream req_stream;

        if (type == Type::GPIC) {
            req_stream << "[type(GPIC), opcode(" << opcode << "), en(" << en << "), addr(0x" << std::hex << addr << ")";
        } else if (type == Type::READ) {
            req_stream << "[type(READ), addr(0x" << std::hex << addr << ")";
        } else if (type == Type::WRITE) {
            req_stream << "[type(WRITE), addr(0x" << std::hex << addr << ")";
        }
        
        return (char*)(req_stream.str().c_str());
    }
};

} /*namespace ramulator*/

#endif /*__REQUEST_H*/

