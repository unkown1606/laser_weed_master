#ifndef __BOARD_H__
#define __BOARD_H__

#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define SIGN(x) ((x) < 0 ? -1 : 1)


#ifdef __cplusplus
extern "C" {
#endif

void allInit();
void schedule();


extern uint64_t sysTickTime;

	
	
#ifdef __cplusplus
}
#endif


#endif