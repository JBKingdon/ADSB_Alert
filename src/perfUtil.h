#ifndef _PERF_UTIL

#define _PERF_UTIL

#include <stdint.h>

void initPerfUtil();
uint32_t getCycles();
uint32_t getDeltaUs(uint32_t start, uint32_t end);


#endif  // _PERF_UTIL