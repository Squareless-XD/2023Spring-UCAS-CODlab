
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Result {
	int pass;
	unsigned long msec;
	unsigned long mem_load_time;
	unsigned long mem_store_time;
	unsigned long inst_num;
	unsigned long mem_load_cycle;
	unsigned long mem_store_cycle;
	unsigned long inst_fetch_cycle;
	unsigned long jump_num;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif
