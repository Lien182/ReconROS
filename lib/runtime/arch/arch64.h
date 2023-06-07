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

#ifndef RECONOS_ARCH_H
#define RECONOS_ARCH_H

#include "../utils.h"

#include <stdint.h>


/* == OSIF related functions ============================================ */

extern int reconos_osif_open(int num);
extern uint64_t reconos_osif_read(int fd);
extern uint64_t reconos_osif_tryread(int fd, uint64_t * data);
extern void reconos_osif_write(int fd, uint64_t data);
extern void reconos_osif_break(int fd);
extern void reconos_osif_close(int fd);


/* == Proc control related functions ==================================== */

extern int reconos_proc_control_open();
extern int reconos_proc_control_get_num_hwts(int fd);
extern int reconos_proc_control_get_tlb_hits(int fd);
extern int reconos_proc_control_get_tlb_misses(int fd);
extern uint64_t reconos_proc_control_get_fault_addr(int fd);
extern void reconos_proc_control_clear_page_fault(int fd);
extern void reconos_proc_control_set_pgd(int fd);
extern void reconos_proc_control_sys_reset(int fd);
extern void reconos_proc_control_hwt_reset(int fd, int num, int reset);
extern void reconos_proc_control_hwt_signal(int fd, int num, int signal);
extern void reconos_proc_control_cache_flush(int fd);
extern void reconos_proc_control_close(int fd);


/* == Proc control related functions ==================================== */

extern int reconos_clock_open();
extern void reconos_clock_set_divider(int fd, int clk, int divd);
extern void reconos_clock_close(int fd);


/* == Reconfiguration related functions ================================= */

extern int load_partial_bitstream(uint32_t *bitstream, unsigned int bitstream_length);


/* == Initialization function =========================================== */

extern void reconos_drv_init();

#endif /* RECONOS_ARCH_H */
