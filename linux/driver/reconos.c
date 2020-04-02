/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Linux Driver - OSIF (AXI FIFO)
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *   description:  Driver for the entire ReconOS system including the
 *                 OSIFs and the proc control.
 *
 * ======================================================================
 */

#include "reconos.h"

#include "osif_intc.h"
#include "proc_control.h"


// extern variables available in the entire module
int NUM_HWTS = 0;
int DYNAMIC_REG_COUNT = 0;

static __init int reconos_init(void) {
	int ret;

	__printk(KERN_WARNING "[reconos] initializing driver ...\n");

	__printk(KERN_INFO "[reconos] initializing driver ...\n");

	NUM_HWTS = proc_control_num_hwts_static();
	DYNAMIC_REG_COUNT = (NUM_HWTS - 1) / 32 + 1;
	__printk(KERN_INFO "[reconos] detected %d HWTs\n", NUM_HWTS);
	if (NUM_HWTS < 0) {
		goto num_hwts_failed;
	}

	ret = proc_control_init();
	if (ret < 0) {
		goto proc_control_failed;
	}

	ret = osif_intc_init();
	if (ret < 0) {
		goto osif_intc_failed;
	}

	return 0;

osif_intc_failed:
	proc_control_exit();

proc_control_failed:
num_hwts_failed:
	return -1;
}

static __exit void reconos_exit(void) {
	__printk(KERN_INFO "[reconos] removing driver ...\n");

	osif_intc_exit();
	proc_control_exit();

	return;
}

module_init(reconos_init);
module_exit(reconos_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Christoph Rüthing <ruething@mail.upb.de>");
