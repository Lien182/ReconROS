/*
 * Copyright 2012 Markus Happe <markus.happe@upb.de>
 * Copyright 2012 Daniel Borkmann <dborkma@tik.ee.ethz.ch>
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include "rqueue.h"
#include "../utils.h"

int rq_init(rqueue *rq, size_t size)
{
	return mbox_init((struct mbox *) rq, size);
}

void rq_close(rqueue *rq)
{
	mbox_destroy((struct mbox *) rq);
}

void rq_send(rqueue *rq, RRUBASETYPE *msg, size_t size)
{
	/* XXX: Can we also avoid allocation + copy?! ---DB */
	RRUBASETYPE *clone = malloc((size + 1) * sizeof(RRUBASETYPE));
	if (!clone)
		panic("rq_send malloc failed\n");

	clone[0] = (RRUBASETYPE) size;
	__builtin_memcpy(&clone[1], msg, size);

	mbox_put((struct mbox *) rq, (RRUBASETYPE) clone);
}

ssize_t rq_receive(rqueue *rq, RRUBASETYPE *msg, size_t size)
{
	RRUBASETYPE *clone;
	ssize_t __size;

	clone = (RRUBASETYPE *) mbox_get((struct mbox *) rq);
	__size = clone[0];

	if (__size == 0 || __size > size)
		return -ENOMEM;

	__builtin_memcpy(msg, &clone[1], __size);
	free(clone);

	return __size;
}
