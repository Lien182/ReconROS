#ifndef MEM_H
#define MEM_H


#include <stdint.h>

struct mem {
	uint32_t * data;
    uint32_t size;
};


extern int mem_init(struct mem *mem, uint32_t size);

extern int mem_destroy(struct mem *mem);

#endif