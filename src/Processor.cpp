#include "Processor.h"
#include <cassert>
#include <cstdio>

using namespace std;
using namespace ramulator;

Processor::Processor(const Config& configs,
    vector<const char*> trace_list,
    function<bool(Request)> send_memory,
    MemoryBase& memory)
    : ipcs(trace_list.size(), -1),
    early_exit(configs.is_early_exit()),
    no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()),
    cachesys(new CacheSystem(configs, send_memory)),
    llc(l3_size, l3_assoc, l3_blocksz,
         mshr_per_bank * trace_list.size(),
         Cache::Level::L3, cachesys) {

  assert(cachesys != nullptr);
  int tracenum = trace_list.size();
  assert(tracenum > 0);
  printf("tracenum: %d\n", tracenum);
  for (int i = 0 ; i < tracenum ; ++i) {
    printf("trace_list[%d]: %s\n", i, trace_list[i]);
  }

  vector< vector<const char*> > trace_lists;
  for (int i = 0 ; i < configs.get_core_num() ; ++i) {
    trace_lists.push_back(vector<const char*>());
  }
  int core_id = 0;
  for (auto &trace : trace_list) {
    trace_lists[core_id].push_back(trace);
    core_id = (core_id + 1) % configs.get_core_num();
  }


  if (no_shared_cache) {
    for (int i = 0 ; i < configs.get_core_num() ; ++i) {
      cores.emplace_back(new Core(
          configs, i, trace_lists[i], send_memory, nullptr,
          cachesys, memory));
    }
  } else {
    for (int i = 0 ; i < configs.get_core_num() ; ++i) {
      cores.emplace_back(new Core(configs, i, trace_lists[i],
          std::bind(&Cache::send, &llc, std::placeholders::_1),
          &llc, cachesys, memory));
    }
  }

  for (int i = 0 ; i < configs.get_core_num() ; ++i) {
    cores[i]->callback = std::bind(&Processor::receive, this,
        placeholders::_1);
  }

  // regStats
  cpu_cycles.name("cpu_cycles")
            .desc("cpu cycle number")
            .precision(0)
            ;
  cpu_cycles = 0;
}

void Processor::tick() {
  cpu_cycles++;

  if((int(cpu_cycles.value()) % 50000000) == 0)
      printf("CPU heartbeat, cycles: %d \n", (int(cpu_cycles.value())));

  if (!(no_core_caches && no_shared_cache)) {
    cachesys->tick();
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->tick();
  }
}

void Processor::receive(Request& req) {
  if (!no_shared_cache) {
    llc.callback(req);
  } else if (!cores[0]->no_core_caches) {
    // Assume all cores have caches or don't have caches
    // at the same time.
    for (unsigned int i = 0 ; i < cores.size() ; ++i) {
      Core* core = cores[i].get();
      core->caches[0]->callback(req);
    }
  }
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    Core* core = cores[i].get();
    core->receive(req);
  }
}

bool Processor::finished() {
  if (early_exit) {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (cores[i]->finished()) {
        for (unsigned int j = 0 ; j < cores.size() ; ++j) {
          ipc += cores[j]->calc_ipc();
        }
        return true;
      }
    }
    return false;
  } else {
    for (unsigned int i = 0 ; i < cores.size(); ++i) {
      if (!cores[i]->finished()) {
        return false;
      }
      if (ipcs[i] < 0) {
        ipcs[i] = cores[i]->calc_ipc();
        ipc += ipcs[i];
      }
    }
    return true;
  }
}

bool Processor::has_reached_limit() {
  for (unsigned int i = 0 ; i < cores.size() ; ++i) {
    if (!cores[i]->has_reached_limit()) {
      return false;
    }
  }
  return true;
}

long Processor::get_insts() {
    long insts_total = 0;
    for (unsigned int i = 0 ; i < cores.size(); i++) {
        insts_total += cores[i]->get_insts();
    }

    return insts_total;
}

void Processor::reset_stats() {
    for (unsigned int i = 0 ; i < cores.size(); i++) {
        cores[i]->reset_stats();
    }

    ipc = 0;

    for (unsigned int i = 0; i < ipcs.size(); i++)
        ipcs[i] = -1;
}

Core::Core(const Config& configs, int coreid,
    const std::vector<const char *>& trace_fnames, function<bool(Request)> send_next,
    Cache* llc, std::shared_ptr<CacheSystem> cachesys, MemoryBase& memory)
    : id(coreid), no_core_caches(!configs.has_core_caches()),
    no_shared_cache(!configs.has_l3_cache()), gpic_mode(configs.is_gpic()),
    llc(llc), trace(trace_fnames), memory(memory)
{
  // set expected limit instruction for calculating weighted speedup
  expected_limit_insts = configs.get_expected_limit_insts();
  trace.expected_limit_insts = expected_limit_insts;

  // Build cache hierarchy
  if (no_core_caches) {
    send = send_next;
  } else {
    // L2 caches[0]
    caches.emplace_back(new Cache(
        l2_size, l2_assoc, l2_blocksz, l2_mshr_num,
        Cache::Level::L2, cachesys));
    // L1 caches[1]
    caches.emplace_back(new Cache(
        l1_size, l1_assoc, l1_blocksz, l1_mshr_num,
        Cache::Level::L1, cachesys));
    if (llc != nullptr) {
      caches[0]->concatlower(llc);
    }
    caches[1]->concatlower(caches[0].get());

    first_level_cache = caches[1].get();

    if (configs.is_gpic()) {
      switch (configs.get_gpic_level())
      {
      case 1:
        send = bind(&Cache::send, caches[1].get(), placeholders::_1);
        break;
      case 2:
        send = bind(&Cache::send, caches[0].get(), placeholders::_1);
        break;
      case 3:
        send = send_next;
      default:
        break;
      }
    } else {
      send = bind(&Cache::send, caches[1].get(), placeholders::_1);
    }
  }

  if (gpic_mode) {
    // DO NOTHING
  } else if (no_core_caches) {
    more_reqs = trace.get_filtered_request(
        bubble_cnt, req_addr, req_type);
    req_addr = memory.page_allocator(req_addr, id);
  } else {
    more_reqs = trace.get_unfiltered_request(
        bubble_cnt, req_addr, req_type);
    req_addr = memory.page_allocator(req_addr, id);
  }

  // regStats
  record_cycs.name("record_cycs_core_" + to_string(id))
             .desc("Record cycle number for calculating weighted speedup. (Only valid when expected limit instruction number is non zero in config file.)")
             .precision(0)
             ;

  record_insts.name("record_insts_core_" + to_string(id))
              .desc("Retired instruction number when record cycle number. (Only valid when expected limit instruction number is non zero in config file.)")
              .precision(0)
              ;

  memory_access_cycles.name("memory_access_cycles_core_" + to_string(id))
                      .desc("memory access cycles in memory time domain")
                      .precision(0)
                      ;
  memory_access_cycles = 0;
  cpu_inst.name("cpu_instructions_core_" + to_string(id))
          .desc("cpu instruction number")
          .precision(0)
          ;
  cpu_inst = 0;
}


double Core::calc_ipc()
{
#ifdef DEBUG
    printf("[%d]retired: %ld, clk, %ld\n", id, retired, clk);
#endif
    return (double) retired / clk;
}

void Core::tick()
{
    clk++;

    if(first_level_cache != nullptr)
        first_level_cache->tick();

    retired += window.retire();

    if (expected_limit_insts == 0 && !more_reqs) return;

    // bubbles (non-memory operations)
    int inserted = 0;
    if (gpic_mode) {
      while ((inserted < window.ipc) && (window.is_full() == false)) {
        more_reqs = trace.get_gpic_request(req_opcode, req_en, req_addr);

        if (!more_reqs) {
          break;
        }

        if (req_addr != -1) {
          // it's a load or store instruction
          window.insert(false, req_opcode, req_addr);
        } else {
          // it's a computational instruction
          window.insert(true, req_opcode, -1);
        }

        Request req(req_opcode, req_en, req_addr, callback, id);
        if (!send(req)) return;

        cpu_inst++;
        inserted++;
        if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
          record_cycs = clk;
          record_insts = long(cpu_inst.value());
          memory.record_core(id);
          reached_limit = true;
        }
      }
    } else {
      while (bubble_cnt > 0) {
          if (inserted == window.ipc) return;
          if (window.is_full()) return;

          window.insert(true, -1);
          inserted++;
          bubble_cnt--;
          cpu_inst++;
          if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
            record_cycs = clk;
            record_insts = long(cpu_inst.value());
            memory.record_core(id);
            reached_limit = true;
          }
      }

      if (req_type == Request::Type::READ) {
          // read request
          if (inserted == window.ipc) return;
          if (window.is_full()) return;

          Request req(req_addr, req_type, callback, id);
          if (!send(req)) return;

          window.insert(false, req_addr);
          cpu_inst++;
      }
      else {
          // write request
          assert(req_type == Request::Type::WRITE);
          Request req(req_addr, req_type, callback, id);
          if (!send(req)) return;
          cpu_inst++;
      }

      if (long(cpu_inst.value()) == expected_limit_insts && !reached_limit) {
        record_cycs = clk;
        record_insts = long(cpu_inst.value());
        memory.record_core(id);
        reached_limit = true;
      }

      if (no_core_caches) {
        more_reqs = trace.get_filtered_request(bubble_cnt, req_addr, req_type);
        if (req_addr != -1) {
          req_addr = memory.page_allocator(req_addr, id);
        }
      } else {
        more_reqs = trace.get_unfiltered_request(bubble_cnt, req_addr, req_type);
        if (req_addr != -1) {
          req_addr = memory.page_allocator(req_addr, id);
        }
      }
    }

    if (!more_reqs) {
      if (!reached_limit) { // if the length of this trace is shorter than expected length, then record it when the whole trace finishes, and set reached_limit to true.
        // Hasan: overriding this behavior. We start the trace from the
        // beginning until the requested amount of instructions are
        // simulated. This should never be reached now.
        assert((expected_limit_insts == 0) && "Shouldn't be reached when expected_limit_insts > 0 since we start over the trace");
        record_cycs = clk;
        record_insts = long(cpu_inst.value());
        memory.record_core(id);
        reached_limit = true;
      }
    }
}

bool Core::finished()
{
    return !more_reqs && window.is_empty();
}

bool Core::has_reached_limit() {
  return reached_limit;
}

long Core::get_insts() {
    return long(cpu_inst.value());
}

void Core::receive(Request& req)
{
    window.set_ready(req.addr, ~(l1_blocksz - 1l));
    if (req.arrive != -1 && req.depart > last) {
      memory_access_cycles += (req.depart - max(last, req.arrive));
      last = req.depart;
    }
}

void Core::reset_stats() {
    clk = 0;
    retired = 0;
    cpu_inst = 0;
}

bool Window::is_full()
{
    return load == depth;
}

bool Window::is_empty()
{
    return load == 0;
}


void Window::insert(bool ready, long addr)
{
    assert(load <= depth);

    ready_list.at(head) = ready;
    addr_list.at(head) = addr;
    opcode_list.at(head) = "NULL";

    head = (head + 1) % depth;
    load++;
}

void Window::insert(bool ready, std::string opcode, long addr)
{
    assert(load <= depth);

    ready_list.at(head) = ready;
    addr_list.at(head) = addr;
    opcode_list.at(head) = opcode;

    head = (head + 1) % depth;
    load++;
}


long Window::retire()
{
    assert(load <= depth);

    if (load == 0) return 0;

    int retired = 0;
    while (load > 0 && retired < ipc) {
        if (!ready_list.at(tail))
            break;

#ifdef DEBUG
        cout << "Retired: opc(" << opcode_list.at(tail) << "), addr: " << addr_list.at(tail) << endl;
#endif
        tail = (tail + 1) % depth;
        cout << 
        load--;
        retired++;
    }

    return retired;
}


void Window::set_ready(long addr, int mask)
{
    if (load == 0) return;

    for (int i = 0; i < load; i++) {
        int index = (tail + i) % depth;
        if ((addr_list.at(index) & mask) != (addr & mask))
            continue;
        ready_list.at(index) = true;
    }
}

const char* req_type_names[] = { "READ", "WRITE", "REFRESH", "POWERDOWN", "SELFREFRESH", "EXTENSION", "MAX" };

Trace::Trace(vector<const char*> trace_fnames)
{
  std::ifstream* files_arr = new std::ifstream[trace_fnames.size()]();
  for (int idx = 0; idx < trace_fnames.size(); idx++) {
    trace_names.push_back(trace_fnames[idx]);
    files_arr[idx].open(trace_fnames[idx]);
    if (!files_arr[idx].good()) {
      std::cerr << "Bad trace file: " << trace_fnames[idx] << std::endl;
      exit(1);
    }
    files.push_back(files_arr + idx);
  }
}

bool Trace::get_gpic_request(std::string& req_opcode, int& req_en, long& req_addr, Request::Type& req_type)
{
  string line;
  for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
    int trace_idx = (last_trace + trace_offset) % files.size();
    try {
      getline(*files[trace_idx], line);
    } catch (const std::runtime_error &ex) {
      std::cout << ex.what() << std::endl;
    }
    if (files[trace_idx]->eof()) {
      files[trace_idx]->close();
      files.erase (files.begin()+trace_idx);
      trace_offset--;
      continue;
    }
    size_t pos;
    pos = line.find(' ');
    assert(pos != string::npos);
    req_opcode = line.substr(0, pos);
    if (req_opcode.compare("load") == 0) {
      req_type = Request::Type::READ;
    }
    if (req_opcode.compare("store") == 0) {
      req_type = Request::Type::WRITE;
    }
    pos = line.find_first_not_of(' ', pos);
    req_en = std::stoul(line, &pos, 10);
    pos = line.find_first_not_of(' ', pos);
    req_addr = -1;
    if (pos == string::npos) {
      // It's a non-memory operation
      assert((req_opcode.find("load") == string::npos) && (req_opcode.find("load") == string::npos));
    } else {
      // it's a load or store, read address
      assert((req_opcode.find("load") != string::npos) || (req_opcode.find("load") != string::npos));
      req_addr = std::stoul(line, &pos, 10);
    }

#ifdef DEBUG
    printf("get_gpic_request returned opcode: %s, enable: %d, address: %ld\n", req_opcode.c_str(), req_en, req_addr);
#endif
    last_trace++;
    return true;
  }
  return false;
}

bool Trace::get_unfiltered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
    string line;
    for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
      int trace_idx = (last_trace + trace_offset) % files.size();
      try {
        getline(*files[trace_idx], line);
      } catch (const std::runtime_error &ex) {
        std::cout << ex.what() << std::endl;
      }
      if (files[trace_idx]->eof()) {
        files[trace_idx]->clear();
        files[trace_idx]->seekg(0, files[trace_idx]->beg);
        if(expected_limit_insts == 0) {
          files[trace_idx]->close();
          files.erase (files.begin()+trace_idx);
          trace_offset--;
          continue;
        }
        else { // starting over the input trace file
          try {
            getline(*files[trace_idx], line);
          } catch (const std::runtime_error &ex) {
            std::cout << ex.what() << std::endl;
          }
        }
      }
      size_t pos, end;
      bubble_cnt = std::stoul(line, &pos, 10);
      pos = line.find_first_not_of(' ', pos+1);
      req_addr = std::stoul(line.substr(pos), &end, 0);

      pos = line.find_first_not_of(' ', pos+end);

      if (pos == string::npos || line.substr(pos)[0] == 'R')
          req_type = Request::Type::READ;
      else if (line.substr(pos)[0] == 'W')
          req_type = Request::Type::WRITE;
      else assert(false);
#ifdef DEBUG
    printf("get_unfiltered_request returned bubble count: %ld, request address: %ld, type: %s\n", bubble_cnt, req_addr, req_type_names[(int)req_type]);
#endif
      last_trace++;
      return true;
    }
    return false;
}

bool Trace::get_filtered_request(long& bubble_cnt, long& req_addr, Request::Type& req_type)
{
  string line;
  for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
    int trace_idx = (last_trace + trace_offset) % files.size();
    try {
      getline(*files[trace_idx], line);
    } catch (const std::runtime_error &ex) {
      std::cout << ex.what() << std::endl;
    }
    if (files[trace_idx]->eof()) {
      files[trace_idx]->clear();
      files[trace_idx]->seekg(0, files[trace_idx]->beg);
      if(expected_limit_insts == 0) {
        files[trace_idx]->close();
        files.erase (files.begin()+trace_idx);
        trace_offset--;
        continue;
      }
      else { // starting over the input trace file
        try {
          getline(*files[trace_idx], line);
        } catch (const std::runtime_error &ex) {
          std::cout << ex.what() << std::endl;
        }
      }
    }
    size_t pos, end;
    bubble_cnt = std::stoul(line, &pos, 10);

    pos = line.find_first_not_of(' ', pos+1);
    req_addr = stoul(line.substr(pos), &end, 0);
    req_type = Request::Type::READ;

    pos = line.find_first_not_of(' ', pos+end);
#ifdef DEBUG
  printf("get_filtered_request returned bubble count: %ld, request address: %ld, type: %s\n", bubble_cnt, req_addr, req_type_names[(int)req_type]);
#endif
    last_trace++;
    return true;
  }
  return false;
}

bool Trace::get_dramtrace_request(long& req_addr, Request::Type& req_type)
{
  string line;
  for (int trace_offset = 0; trace_offset < (int)files.size(); trace_offset++) {
    int trace_idx = (last_trace + trace_offset) % files.size();
    try {
      getline(*(files[trace_idx]), line);
    } catch (const std::runtime_error &ex) {
      std::cout << ex.what() << std::endl;
    }
    if (files[trace_idx]->eof()) {
      files[trace_idx]->close();
      files.erase (files.begin()+trace_idx);
      trace_offset--;
      continue;
    }

    size_t pos;
    req_addr = std::stoul(line, &pos, 16);

    pos = line.find_first_not_of(' ', pos+1);

    if (pos == string::npos || line.substr(pos)[0] == 'R')
        req_type = Request::Type::READ;
    else if (line.substr(pos)[0] == 'W')
        req_type = Request::Type::WRITE;
    else assert(false);
#ifdef DEBUG
    printf("get_dramtrace_request returned request address: %ld, type: %s\n", req_addr, req_type_names[(int)req_type]);
#endif
    last_trace++;
    return true;
  }
  return false;
}
