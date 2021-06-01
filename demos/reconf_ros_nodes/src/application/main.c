#define  _GNU_SOURCE

#include "reconos.h"
#include "reconos_app.h"
#include "mbox.h"
#include "timer.h"

#include "utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>





int main(int argc, char **argv) {

	reconos_init();
	reconos_app_init();


	reconos_thread_pr_create_hwt_sortdemo(0, "bitstreams");


	while(1)
	{
		sleep(10);
	}

	reconos_app_cleanup();
	reconos_cleanup();

	return 0;
}