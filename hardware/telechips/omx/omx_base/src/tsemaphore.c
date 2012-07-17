/**
  @file src/tsemaphore.c

  Implements a simple inter-thread semaphore so not to have to deal with IPC
  creation and the like.

  Copyright (C) 2007  STMicroelectronics and Nokia

  This library is free software; you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St, Fifth Floor, Boston, MA
  02110-1301  USA

  $Date: 2008/08/21 09:26:02 $
  Revision $Rev: 479 $
  Author $Author: B060452 $
*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include "tsemaphore.h"

/** Initializes the semaphore at a given value
 * 
 * @param tsem the semaphore to initialize
 * @param val the initial value of the semaphore
 * 
 */
void tsem_init(tsem_t* tsem, unsigned int val) {
  pthread_cond_init(&tsem->condition, NULL);
  pthread_mutex_init(&tsem->mutex, NULL);
  tsem->semval = val;
}

/** Destroy the semaphore
 *  
 * @param tsem the semaphore to destroy
 */
void tsem_deinit(tsem_t* tsem) {
  pthread_cond_destroy(&tsem->condition);
  pthread_mutex_destroy(&tsem->mutex);
}

/** Decreases the value of the semaphore. Blocks if the semaphore
 * value is zero.
 * 
 * @param tsem the semaphore to decrease
 */
void tsem_down(tsem_t* tsem) {
  pthread_mutex_lock(&tsem->mutex);
  while (tsem->semval == 0) {
	  pthread_cond_wait(&tsem->condition, &tsem->mutex);
  }
  tsem->semval--;
  pthread_mutex_unlock(&tsem->mutex);
}

int tsem_down_timewait(tsem_t* tsem,int expire_time)
{
	int err;
    struct timeval now;
    struct timespec ts;

    gettimeofday(&now, NULL);
    ts.tv_sec = now.tv_sec + expire_time; 	// sec 단위로 입력..
    ts.tv_nsec = now.tv_usec * 1000;

	pthread_mutex_lock(&tsem->mutex);
	while (tsem->semval == 0) {
		err = pthread_cond_timedwait(&tsem->condition, &tsem->mutex , &ts);

		if(err == ETIMEDOUT)
		{
			tsem->semval++;
		}
	}
	tsem->semval--;
	pthread_mutex_unlock(&tsem->mutex);

	if(err == ETIMEDOUT)
		return 0;
	else return 1;
}


/** Increases the value of the semaphore
 * 
 * @param tsem the semaphore to increase
 */
void tsem_up(tsem_t* tsem) {
  pthread_mutex_lock(&tsem->mutex);
  tsem->semval++;
  pthread_cond_signal(&tsem->condition);
  pthread_mutex_unlock(&tsem->mutex);
}

/** Reset the value of the semaphore
 * 
 * @param tsem the semaphore to reset
 */
void tsem_reset(tsem_t* tsem) {
  pthread_mutex_lock(&tsem->mutex);
  tsem->semval=0;
  pthread_mutex_unlock(&tsem->mutex);
}

/** Wait on the condition.
 * 
 * @param tsem the semaphore to wait
 */
void tsem_wait(tsem_t* tsem) {
  pthread_mutex_lock(&tsem->mutex);
  pthread_cond_wait(&tsem->condition, &tsem->mutex);
  pthread_mutex_unlock(&tsem->mutex);
}

/** Signal the condition,if waiting
 * 
 * @param tsem the semaphore to signal
 */
void tsem_signal(tsem_t* tsem) {
  pthread_mutex_lock(&tsem->mutex);
  pthread_cond_signal(&tsem->condition);
  pthread_mutex_unlock(&tsem->mutex);
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
