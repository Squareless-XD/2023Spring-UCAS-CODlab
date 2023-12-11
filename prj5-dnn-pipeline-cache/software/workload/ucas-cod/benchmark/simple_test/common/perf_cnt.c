#include "perf_cnt.h"

#define CYCLE_CNT        0x60010000
#define MEM_LOAD_CNT     0x60010008
#define MEM_STORE_CNT    0x60011000
#define INST_CNT         0x60011008
#define MEM_LOAD_CYCLE   0x60012000
#define INST_ALL_CNT     0x60012008
#define INST_FETCH_CYCLE 0x60013000
#define JUMP_CNT         0x60013008
#define CYCLE_CNT_HI     0x60014000
#define PERF_CNT_09_ADDR 0x60014008
#define PERF_CNT_10_ADDR 0x60015000
#define PERF_CNT_11_ADDR 0x60015008
#define PERF_CNT_12_ADDR 0x60016000
#define PERF_CNT_13_ADDR 0x60016008
#define PERF_CNT_14_ADDR 0x60017000
#define PERF_CNT_15_ADDR 0x60017008

unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  return *((volatile unsigned long *)CYCLE_CNT);
}

unsigned long _mem_load_time() {
  return *((volatile unsigned long *)MEM_LOAD_CNT);
}

unsigned long _mem_store_time() {
  return *((volatile unsigned long *)MEM_STORE_CNT);
}

unsigned long _inst_num() {
  return *((volatile unsigned long *)INST_CNT);
}

unsigned long _mem_load_cycle() {
  return *((volatile unsigned long *)MEM_LOAD_CYCLE);
}

unsigned long _inst_all_cnt() {
  return *((volatile unsigned long *)INST_ALL_CNT);
}

unsigned long _inst_fetch_cycle() {
  return *((volatile unsigned long *)INST_FETCH_CYCLE);
}

unsigned long _jump_num() {
  return *((volatile unsigned long *)JUMP_CNT);
}

unsigned long _uptime_hi() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  return *((volatile unsigned long *)CYCLE_CNT_HI);
}

void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec             = _uptime()          ;
  res->mem_load_time    = _mem_load_time()   ;
  res->mem_store_time   = _mem_store_time()  ;
  res->inst_num         = _inst_num()        ;
  res->mem_load_cycle   = _mem_load_cycle()  ;
  res->inst_all_cnt     = _inst_all_cnt()    ;
  res->inst_fetch_cycle = _inst_fetch_cycle();
  res->jump_num         = _jump_num()        ;
  res->msec_hi          = _uptime_hi()       ;
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  res->msec             = _uptime()           - res->msec            ;
  res->mem_load_time    = _mem_load_time()    - res->mem_load_time   ;
  res->mem_store_time   = _mem_store_time()   - res->mem_store_time  ;
  res->inst_num         = _inst_num()         - res->inst_num        ;
  res->mem_load_cycle   = _mem_load_cycle()   - res->mem_load_cycle  ;
  res->inst_all_cnt     = _inst_all_cnt()     - res->inst_all_cnt    ;
  res->inst_fetch_cycle = _inst_fetch_cycle() - res->inst_fetch_cycle;
  res->jump_num         = _jump_num()         - res->jump_num        ;
  res->msec_hi          = _uptime_hi()        - res->msec_hi         ;
}

