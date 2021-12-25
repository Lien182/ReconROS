#include "mem.h"

#include <stdlib.h>


int mem_init(struct mem *mem, uint32_t size)
{
    mem->data = malloc(size * 4);
    if(mem->data == 0)
    {
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