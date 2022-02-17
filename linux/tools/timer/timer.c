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

#include "timer.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <stdio.h>

#define TIMER_BASE_ADDR 0x64a00000
#define CLK_FREQ 100000000

volatile uint32_t *ptr = 0;


/* == Timer functions ================================================== */

/*
 * @see header
 */
void timer_init() {
	int fd;

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		printf("ERROR: Could not open /dev/mem\n");
		close(fd);
		return;
	}

	ptr = (uint32_t *)mmap(0, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, TIMER_BASE_ADDR);
	if (ptr == MAP_FAILED) {
		printf("ERROR: Could not map memory\n");
		close(fd);
		return;
	}

	close(fd);

	timer_reset();
	timer_setstep(0);
}

/*
 * @see header
 */
void timer_reset() {
	if (ptr) {
		ptr[0] = 0;
	}
}

/*
 * @see header
 */
void timer_setstep(unsigned int step) {
	if (ptr) {
		ptr[1] = step;
	}
}

/*
 * @see header
 */
unsigned int timer_get() {
	if (ptr) {
		return *ptr;
	} else {
		return 0;
	}
}

/*
 * @see header
 */
void timer_cleanup() {
	munmap((void *)ptr, 0x10000);
	ptr = 0;
}

float timer_toms(unsigned int t) {
	return t / (CLK_FREQ / 1000.0);
}


unsigned int timer_msto(float t)
{
	return (unsigned int) (t * (CLK_FREQ / 1000.0));
}