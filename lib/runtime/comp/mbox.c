/*
 * Copyright 2012 Andreas Agne <agne@upb.de>
 * Copyright 2012 Daniel Borkmann <dborkma@tik.ee.ethz.ch>
 */

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "mbox.h"
#include "../utils.h"

int mbox_init(struct mbox *mb, size_t size)
{
	int ret;
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);

	mb->read_idx = 0;
	mb->write_idx = 0;
	mb->size = size;

	ret = sem_init(&mb->sem_read, 0, 0);
	if (ret)
		goto out_err;
	ret = sem_init(&mb->sem_write, 0, size);
	if (ret)
		goto out_err;
	ret = pthread_mutex_init(&mb->mutex_read, &attr);
	if (ret)
		goto out_err;
	ret = pthread_mutex_init(&mb->mutex_write, &attr);
	if (ret)
		goto out_err;

	mb->messages = malloc(mb->size * sizeof(uint32_t));
	if (!mb->messages)
		goto out_err;

	return 0;
out_err:
	return -EIO;
}

void mbox_destroy(struct mbox *mb)
{
	free(mb->messages);

	sem_destroy(&mb->sem_write);
	sem_destroy(&mb->sem_read);

	pthread_mutex_destroy(&mb->mutex_read);
	pthread_mutex_destroy(&mb->mutex_write);
}

int mbox_put(struct mbox *mb, uint32_t msg)
{
	pthread_mutex_lock(&mb->mutex_write);
	sem_wait(&mb->sem_write);

	mb->messages[mb->write_idx] = msg;
	mb->write_idx = (mb->write_idx + 1)  % mb->size;

	sem_post(&mb->sem_read);
	pthread_mutex_unlock(&mb->mutex_write);

	return 0;
}

int mbox_put_interruptible(struct mbox *mb, uint32_t msg)
{
	if (pthread_mutex_lock(&mb->mutex_write) < 0) {
		return -1;
	}
	if (sem_wait(&mb->sem_write) < 0) {
		pthread_mutex_unlock(&mb->mutex_write);
		return -1;
	}

	mb->messages[mb->write_idx] = msg;
	mb->write_idx = (mb->write_idx + 1)  % mb->size;

	sem_post(&mb->sem_read);
	pthread_mutex_unlock(&mb->mutex_write);

	return 0;
}

uint32_t mbox_get(struct mbox *mb)
{
	uint32_t msg;
	
	pthread_mutex_lock(&mb->mutex_read);
	sem_wait(&mb->sem_read);

	msg = mb->messages[mb->read_idx];
	mb->read_idx = (mb->read_idx + 1) % mb->size;

	sem_post(&mb->sem_write);
	pthread_mutex_unlock(&mb->mutex_read);
	
	return msg;
}

int mbox_get_interruptible(struct mbox *mb, uint32_t *msg)
{
	if (pthread_mutex_lock(&mb->mutex_read) < 0) {
		return -1;
	}
	if (sem_wait(&mb->sem_read) < 0) {
		pthread_mutex_unlock(&mb->mutex_read);
		return -1;
	}

	*msg = mb->messages[mb->read_idx];
	mb->read_idx = (mb->read_idx + 1) % mb->size;

	sem_post(&mb->sem_write);
	pthread_mutex_unlock(&mb->mutex_read);
	
	return 0;
}

int mbox_tryget(struct mbox *mb, uint32_t *msg)
{
	int success, fill;

	pthread_mutex_lock(&mb->mutex_read);

	sem_getvalue(&mb->sem_read, &fill);
	if (fill <= 0) {
		success = 0;
	} else {
		success = 1;

		sem_wait(&mb->sem_read);

		*msg = mb->messages[mb->read_idx];
		mb->read_idx = (mb->read_idx + 1) % mb->size;

		sem_post(&mb->sem_write);
	}

	pthread_mutex_unlock(&mb->mutex_read);

	return success;
}

int mbox_tryput(struct mbox * mb, uint32_t msg)
{
	int success, rem;

	pthread_mutex_lock(&mb->mutex_write);

	sem_getvalue(&mb->sem_write, &rem);
	if (rem <= 0) {
		success = 0;
	} else {
		success = 1;

	        sem_wait(&mb->sem_write);

		mb->messages[mb->write_idx] = msg;
		mb->write_idx = (mb->write_idx + 1)  % mb->size;

		sem_post(&mb->sem_read);
	}

	pthread_mutex_unlock(&mb->mutex_write);

	return success;
}
