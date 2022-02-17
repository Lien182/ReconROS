/*
 * Copyright 2012 Andreas Agne <agne@upb.de>
 * Copyright 2012 Daniel Borkmann <dborkma@tik.ee.ethz.ch>
 */

#ifndef MBOX_H
#define MBOX_H

#include "cpuarch.h"

#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>



/*
 * Structure representing a mbox
 */
struct mbox {
	sem_t sem_read;
	sem_t sem_write;
	pthread_mutex_t mutex_read;
	pthread_mutex_t mutex_write;	
	RRUBASETYPE *messages;
	off_t read_idx;
	off_t write_idx;
	size_t size;
};

/*
 * Initializes the mbox. You must call this method before you
 * can use the mbox.
 *
 *  mb   - pointer to the mbox
 *  size - size of the mbox in 32bit-words
 */
extern int mbox_init(struct mbox *mb, size_t size);

/*
 * Frees all used memory of the mbox.
 *
 *   mb - pointer to the mbox
 */
extern void mbox_destroy(struct mbox *mb);

/*
 * Puts a single word into the mbox and blocks if it is full.
 *
 *   mb  - pointer to the mbox
 *   msg - message to put into the mbox
 */
extern int mbox_put(struct mbox *mb, RRUBASETYPE msg);


/*
 * Puts a single word into the mbox and blocks if it is full.
 *
 *   mb  - pointer to the mbox
 *   msg - message to put into the mbox
 *
 *   @returns -1 if interrupted, otherwise 0
 */
extern int mbox_put_interruptible(struct mbox *mb, RRUBASETYPE msg);

/*
 * Gets a single word out of the mbox and blocks if it is full.
 *
 *   mb - pointer to the mbox
 *
 *   @returns the message out of the mbox
 */
extern RRUBASETYPE mbox_get(struct mbox *mb);

/*
 * Gets a single word out the mbox and blocks if it is full.
 *
 *   mb  - pointer to the mbox
 *   msg - message out of the mbox
 *
 *   @returns -1 if interrupted, otherwise 0
 *
 */
 extern int mbox_get_interruptible(struct mbox *mb, RRUBASETYPE *msg);

/*
 * Tries to get a single word out of the mbox but does not block.
 *
 *   mb  - pointer to the mbox
 *   msg - pointer to store the message in
 *         (only valid if returns true)
 *
 *   returns if a word was read or not
 */
extern int mbox_tryget(struct mbox *mb, RRUBASETYPE *msg);

/*
 * Tries to put a single word into the mbox but does not block.
 *
 *   mb  - pointer to the mbox
 *   msg - data to put into the mbox
 *         (only stored in the mbox if returns true)
 *
 *   returns if the word could be stored in the mbox
 */
extern int mbox_tryput(struct mbox *mb, RRUBASETYPE msg);

#endif /* MBOX_H */
