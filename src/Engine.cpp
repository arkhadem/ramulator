#include "Engine.h"
#include <cassert>

using namespace std;
using namespace ramulator;

#ifndef DEBUG
#define hint(...)
#else
#define hint(...)          \
    do {                   \
        hint(__VA_ARGS__); \
    } while (0)
#endif

Engine::Engine()
{
    id = 0;
}

void Engine::reset(int id, Trace* trace, MemoryBase* memory)
{
    this->id = id;
    this->trace = trace;
    this->memory = memory;
    this->req_clock = 0;
    this->total_allowed_clocks = 0;
    this->batch_start_clock = 0;
    this->completed_requests = 0;
    this->to_send = 0;
    this->to_receive = 0;
    this->exe_finished = false;
    this->clks = 0;
    this->batch_requests = 0;
}

void Engine::tick()
{
    if (exe_finished == true)
        return;
    if (to_receive == batch_requests) {

        // Check if the compute is also finished
        if (clks < batch_start_clock + total_allowed_clocks) {
            hint("[%ld][%d] waiting for compute\n", clks, id);
            clks++;
            return;
        }

        // Getting new batch of requests
        Requests.clear();

        if (trace->get_timedtrace_request(req_addr, req_type, req_clock) == false) {
            exe_finished = true;
            batchstat.total_cycles = clks;
            return;
        }

        assert(req_type == Request::Type::MAX);
        batch_start_clock = clks;
        total_allowed_clocks = req_clock;
        batch_requests = req_addr;
        batchstat.total_batches++;
        batchstat.total_requests += batch_requests;
        for (int i = 0; i < batch_requests; i++) {
            assert(trace->get_timedtrace_request(req_addr, req_type, req_clock));
            assert((req_type == Request::Type::READ) || (req_type == Request::Type::WRITE));
            Request req(req_addr, req_type, std::bind(&Engine::receive, this, placeholders::_1), id);
            Requests.push_back(req);
        }
        to_receive = 0;
        to_send = 0;
        hint("[%ld][%d] new batch started with %ld requests. Compute latency: %ld clock cycles!\n", clks, id, batch_requests, total_allowed_clocks);
    }
    if (to_send != batch_requests) {
        assert(to_send < batch_requests);
        if (memory->send(Requests[to_send]) == true) {
            hint("[%ld][%d] %d/%ld requests sent: %s\n", clks, id, to_send + 1, batch_requests, Requests[to_send].c_str());
            to_send++;
        } else {
            hint("[%ld][%d] req stalled\n", clks, id);
        }
    } else {
        hint("[%ld][%d] waiting for memory\n", clks, id);
    }
    clks++;
}

bool Engine::finished()
{
    return exe_finished;
}

void Engine::receive(Request& req)
{
    to_receive++;
    hint("[%ld][%d] %d/%ld requests received: %s\n", clks, id, to_receive, batch_requests, req.c_str());
    assert(to_receive <= batch_requests);
    if (to_receive == batch_requests) {
        // Checking the last batch
        if (clks - batch_start_clock > total_allowed_clocks) {
            batchstat.late_batches++;
            batchstat.total_late_cycles += clks - batch_start_clock - total_allowed_clocks;
            hint("[%ld][%d] batch completed in %ld - %ld = %ld late clock cycles!\n", clks, id, clks - batch_start_clock, total_allowed_clocks, clks - batch_start_clock - total_allowed_clocks);
        } else {
            hint("[%ld][%d] batch completed successfully in %ld < %ld clock cycles!\n", clks, id, clks - batch_start_clock, total_allowed_clocks);
        }
    }
}

BatchStat Engine::get_batchstat()
{
    return batchstat;
}