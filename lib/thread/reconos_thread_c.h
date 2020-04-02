/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        Thread library header file
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Auto-generated thread specific header file including
 *                 resource definitions and helper macros.
 *
 * ======================================================================
 */

<<reconos_preproc>>

#ifndef RECONOS_THREAD_H
#define RECONOS_THREAD_H

#include "reconos_app.h"

/* == Thread resources ================================================= */

/*
 * Definition of resource ids local to this thread. Always use the pointers
 * directory and not resource array indexed by these ids.
 */
<<generate for RESOURCES>>
#define <<NameUpper>> 0x<<HexLocalId>>
<<end generate>>

/* == Thread helper macros ============================================= */

/*
 * Definition of the entry function to the ReconOS thread. Every ReconOS
 * thread should be defined using this macro:
 *
 *   THREAD_ENTRY() {
 *     // thread code here
 *   }
 }
 */
#define THREAD_ENTRY()\
	void rt_<<NAME>>(void *data)

#endif /* RECONOS_THREAD_H */