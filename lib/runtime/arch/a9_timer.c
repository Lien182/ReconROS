/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Architecture specific code
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *   description:  Functions needed for ReconOS which are architecure
 *                 specific and are implemented here.
 *
 * ======================================================================
 */

#include "a9_timer.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <stdio.h>

//Location and size are fixed
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define MAP_BASEADR 0xF8F00200

#define CLK_FREQ 333333333

volatile static t_a9timer *__a9tmr;


/* == Timer functions ================================================== */

/*
 * @see header
 */
int a9timer_init() 
{
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		printf("ERROR: Could not open /dev/mem\n");
		close(fd);
		return -1;
	}

	__a9tmr = (t_a9timer *)mmap(0, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MAP_BASEADR & ~MAP_MASK );
	if (__a9tmr == MAP_FAILED) {
		printf("[A9 Timer] Init could not map memory\n");
		return -2;
	}

    __a9tmr = (t_a9timer*)((void*)__a9tmr + (MAP_BASEADR & MAP_MASK));

	close(fd);
	return 0;
}

uint64_t a9timer_get(void) 
{
	if (__a9tmr) 
	{
		return (uint64_t)__a9tmr->TMR_CNT_REG_L + ((uint64_t)__a9tmr->TMR_CNT_REG_H << 32);
	} 
	else 
	{
		return 0;
	}
}

void a9timer_cleanup(void) 
{	
	if(__a9tmr)
		munmap((void *)__a9tmr, 0x10000);
	__a9tmr = 0;
}

float a9timer_toms(uint64_t t) 
{
	return t / (CLK_FREQ / 1000.0);
}

uint64_t a9timer_msto(float t) 
{
	return (uint64_t)(t * (CLK_FREQ / 1000.0));
}
