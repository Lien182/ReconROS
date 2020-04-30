#include "reconos.h"
#include "reconos_app.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define USE_HW 1



int main(int argc, char **argv) {

	reconos_init();
	reconos_app_init();

	int clk = reconos_clock_threads_set(100000);

#if USE_HW == 1
	reconos_thread_create_swt_filterdemo(0, 0);
#else
	reconos_thread_create_hwt_filterdemo(0);
#endif

	while(1) sleep(1);





	reconos_app_cleanup();
	reconos_cleanup();

	return clk;
}