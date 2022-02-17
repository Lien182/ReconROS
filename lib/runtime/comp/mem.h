#ifndef MEM_H
#define MEM_H

#include "cpuarch.h"

#include <stdint.h>

struct mem {
	RRUBASETYPE *data;
    RRUBASETYPE size;
};


extern int mem_init(struct mem *mem, RRUBASETYPE size);

extern int mem_destroy(struct mem *mem);

extern void * mem_getdataptr(struct mem * mem);

#endif