/**
  @file src/queue.c

  Implements a simple LIFO structure used for queueing OMX buffers.

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

  $Date: 2008/10/18 11:06:06 $
  Revision $Rev: 241 $
  Author $Author: B060500 $
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "queue.h"
#include "TCCMemory.h"

/** Initialize a queue descriptor
 * 
 * @param queue The queue descriptor to initialize. 
 * The user needs to allocate the queue
 */
void queue_init (queue_t * queue)
{
	int       i;
	qelem_t  *newelem;
	qelem_t  *current;
	queue->first = TCC_malloc (sizeof (qelem_t));
	memset (queue->first, 0, sizeof (qelem_t));
	current = queue->last = queue->first;
	queue->nelem = 0;
	queue->nmax_elem = MAX_QUEUE_ELEMENTS;
	for (i = 0; i < queue->nmax_elem - 2; i++)
	{
		newelem = TCC_malloc (sizeof (qelem_t));
		memset (newelem, 0, sizeof (qelem_t));
		current->q_forw = newelem;
		current = newelem;
	}
	current->q_forw = queue->first;

	pthread_mutex_init (&queue->mutex, NULL);
}

void queue_init_ex (queue_t * queue, int nmax_elem)
{
	int       i;
	qelem_t  *newelem;
	qelem_t  *current;
	queue->first = TCC_malloc (sizeof (qelem_t));
	memset (queue->first, 0, sizeof (qelem_t));
	current = queue->last = queue->first;
	queue->nelem = 0;
	queue->nmax_elem = nmax_elem+2;
	for (i = 0; i < queue->nmax_elem - 2; i++)
	{
		newelem = TCC_malloc (sizeof (qelem_t));
		memset (newelem, 0, sizeof (qelem_t));
		current->q_forw = newelem;
		current = newelem;
	}
	current->q_forw = queue->first;

	pthread_mutex_init (&queue->mutex, NULL);
}

/** Deinitialize a queue descriptor
 * flushing all of its internal data
 * 
 * @param queue the queue descriptor to dump
 */
void queue_deinit (queue_t * queue)
{
	int       i;
	qelem_t  *current;
	current = queue->first;
	for (i = 0; i < queue->nmax_elem - 2; i++)
	{
		if (current != NULL)
		{
			current = current->q_forw;
			TCC_free (queue->first);
			queue->first = current;
		}
	}
	if (queue->first)
	{
		TCC_free (queue->first);
		queue->first = NULL;
	}
	pthread_mutex_destroy (&queue->mutex);
}


/** Enqueue an element to the given queue descriptor
 * 
 * @param queue the queue descritpor where to queue data
 * 
 * @param data the data to be enqueued
 */
void queue (queue_t * queue, void *data)
{
	if (queue->last->data != NULL)
	{
		return;
	}
	if (queue->nelem + 1 > queue->nmax_elem - 2)
	{
		printf ("\n---------- queue full!!! ----------\n");
	}
	pthread_mutex_lock (&queue->mutex);
	queue->last->data = data;
	queue->last = queue->last->q_forw;
	queue->nelem++;
	pthread_mutex_unlock (&queue->mutex);
}

int queue_ex (queue_t * queue, void *data)
{
	if (queue->last->data != NULL)
	{
		return 0;
	}
	if (queue->nelem + 1 > queue->nmax_elem - 2)
	{
		printf ("\n---------- queue full!!! ----------\n");
		return 0;
	}
	pthread_mutex_lock (&queue->mutex);
	queue->last->data = data;
	queue->last = queue->last->q_forw;
	queue->nelem++;
	pthread_mutex_unlock (&queue->mutex);
	
	return 1;
}

int queuefirst (queue_t * queue, void *data)
{
	void *data_c, *data_n;
	qelem_t  *current;
	if (queue->last->data != NULL)
	{
		return 0;
	}
	if (queue->nelem + 1 > queue->nmax_elem - 2)
	{
		printf ("\n---------- queue full!!! ----------\n");
		return 0;
	}
	pthread_mutex_lock (&queue->mutex);
	if (queue->first->data == NULL)
	{
		queue->last->data = data;	
	}
	else
	{
		current =  queue->first->q_forw;
		data_c = current->data;
		current->data = data;	
		while(1)
		{
			if( data_c == NULL )
				break;		
			data_n = current->q_forw->data;
			current->q_forw->data = data_c;
			current = current->q_forw;
			data_c = data_n;
		}	
	}	
	queue->last = queue->last->q_forw;
	queue->nelem++;
	pthread_mutex_unlock (&queue->mutex);
	
	return 1;
}

/** Dequeue an element from the given queue descriptor
 * 
 * @param queue the queue descriptor from which to dequeue the element
 * 
 * @return the element that has bee dequeued. If the queue is empty
 *  a NULL value is returned
 */
void     *dequeue (queue_t * queue)
{
	void     *data;
	if (queue->first->data == NULL)
	{
		return NULL;
	}
	pthread_mutex_lock (&queue->mutex);
	data = queue->first->data;
	queue->first->data = NULL;
	queue->first = queue->first->q_forw;
	queue->nelem--;
	pthread_mutex_unlock (&queue->mutex);

	return data;
}

void     *queue_getdata (queue_t * queue)
{
	if (queue->first->data == NULL)
	{
		return NULL;
	}
	return queue->first->data;
}

/** Returns the number of elements hold in the queue
 * 
 * @param queue the requested queue
 * 
 * @return the number of elements in the queue
 */
int getquenelem (queue_t * queue)
{
	int       qelem;
	pthread_mutex_lock (&queue->mutex);
	qelem = queue->nelem;
	pthread_mutex_unlock (&queue->mutex);
	return qelem;
}
