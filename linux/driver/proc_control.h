/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Linux Driver - ReconOS - Proc control
 *
 *   project:      ReconOS
 *   author:       Christoph Rüthing, University of Paderborn
 *   description:  Driver for the proc control of the ReconOS system. It
 *                 allows to control the dirfferent components, reset the
 *                 entire system and read out some information like the
 *                 number of OSIFs, ...
 *
 * ======================================================================
 */

#ifndef RECONOS_DRV_PROC_CONTROL_H
#define RECONOS_DRV_PROC_CONTROl_H

#include "reconos.h"

/*
 * Reads the number of hardware threads without need of initialization
 *
 *   no parameters
 *
 *   @returns -1 on failure, otherwise the number of hardware threads
 */
extern int proc_control_num_hwts_static(void);

/*
 * Initialization function called by module loading
 *
 *   no parameters
 *
 *   @returns -1 on failure, otherwise 0
 */
extern int proc_control_init(void);

/*
 * Exit function called by module unloading
 *
 *   no parameters
 *
 *   @returns always 0
 */
extern int proc_control_exit(void);

#endif /* RECONOS_DRV_OSIF_H */
