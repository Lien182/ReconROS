/*
 *                                                        ____  _____
 *                            ________  _________  ____  / __ \/ ___/
 *                           / ___/ _ \/ ___/ __ \/ __ \/ / / /\__ \
 *                          / /  /  __/ /__/ /_/ / / / / /_/ /___/ /
 *                         /_/   \___/\___/\____/_/ /_/\____//____/
 *
 * ======================================================================
 *
 *   title:        ap_cint header file
 *
 *   project:      ReconOS
 *   author:       Andreas Agne, University of Paderborn
 *                 Christoph Rüthing, University of Paderborn
 *   description:  Header file to provide compatability with ap_cint.h
 *
 * ======================================================================
 */



#ifndef AP_CINT_H
#define AP_CINT_H

#include "stdint.h"

/* == Datatype definitions ============================================= */

/*
 * Definition of resource ids local to this thread. Always use the pointers
 * directory and not resource array indexed by these ids.
 */
typedef uint32_t uint32;

#endif /* AP_CINT_H */