#ifndef __ENGINE_H
#define __ENGINE_H

#include "Cache.h"
#include "Config.h"
#include "Memory.h"
#include "Processor.h"
#include "Request.h"
#include "Statistics.h"
#include <ctype.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

// #define DEBUG

namespace ramulator {

class Engine {
public:
    Engine();

    void reset(int id, Trace* trace, MemoryBase* memory);

    void tick();
    bool finished();

    void receive(Request& req);

    BatchStat get_batchstat();

    int id;
    Trace* trace;
    MemoryBase* memory;

private:
    std::vector<Request> Requests;

    BatchStat batchstat;

    long total_allowed_clocks, batch_start_clock;
    long completed_requests;
    int to_send;
    int to_receive;
    long req_addr, req_clock;
    Request::Type req_type;
    bool exe_finished;
    long clks;
    long batch_requests;
};
}
#endif /* __PROCESSOR_H */