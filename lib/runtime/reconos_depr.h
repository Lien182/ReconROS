/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Deprecated functions
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Header file for compatability of deprecated functions.
 *
 * ======================================================================
 */

#ifndef RECONOS_DEPR_H
#define RECONOS_DEPR_H

#include "reconos.h"

#define RECONOS_TYPE_MBOX RECONOS_RESOURCE_TYPE_MBOX
#define RECONOS_TYPE_SEM RECONOS_RESOURCE_TYPE_SEM
#define RECONOS_TYPE_MUTEX RECONOS_RESOURCE_TYPE_MUTEX
#define RECONOS_TYPE_COND RECONOS_RESOURCE_TYPE_COND
//#define RECONOS_TYPE_RQ RECONOS_RESOURCE_TYPE_RQ


#define reconos_hwt reconos_thread


inline void __attribute__((deprecated))
            reconos_hwt_setresources(struct reconos_hwt *hwt,
                                     struct reconos_resource *resource,
                                     int resource_count) {
	reconos_thread_init(hwt, "HWT", 0);
	reconos_thread_setresources(hwt, resource, resource_count);
}

inline void __attribute__((deprecated))
            reconos_hwt_setinitdata(struct reconos_hwt *hwt,
                                    void* init_data) {
	reconos_thread_setinitdata(hwt, init_data);
}

inline void __attribute__((deprecated))
            reconos_hwt_create(struct reconos_hwt *hwt,
                               int slot, void *arg) {
	reconos_thread_create(hwt, slot);
}

inline void __attribute__((deprecated))
            reconos_mmu_stats(int *tlb_hits, int *tlb_misses,
                              int *page_faults) {
	*tlb_hits = 0;
	*tlb_misses = 0;
	*page_faults = 0;
}

#endif /* RECONOS_DEPR_H */
