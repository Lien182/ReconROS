#include "mem.h"
#include <stdio.h>
#include <stdlib.h>


int mem_init(struct mem *mem, RRUBASETYPE size)
{
    mem->data = malloc(size * sizeof(RRUBASETYPE));
    if(mem->data == 0)
    {
        printf("[MEM] Failed to malloc memory \n");
        return -1;
    }
    mem->size = size;

    return 0;
}


int mem_destroy(struct mem *mem)
{
    if(mem->size != 0)
    {
        free(mem->data);
    }
    return 0;
}


void * mem_getdataptr(struct mem * mem)
{
    return (void*)mem->data;
}