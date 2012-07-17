/******************************************************************************
*
*  (C)Copyright All Rights Reserved by Telechips Inc.
*                                   
*  This material is confidential and shall remain as such. 
*  Any unauthorized use, distribution, reproduction is strictly prohibited. 
*
*   FileName    : TCCMemory.h
*   Description : 
*   TCC Version 1.0
*   Copyright (c) Telechips, Inc.
*   ALL RIGHTS RESERVED
*******************************************************************************/
#ifndef	_TCC_MEMORY_H__
#define	_TCC_MEMORY_H__

/******************************************************************************
* include 
******************************************************************************/
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
* typedefs & structure
******************************************************************************/


/******************************************************************************
* defines 
******************************************************************************/

#define	TCC_LINUX_MEMORY_SYTEM

//#define	TCC_MEMORY_DEBUG	// for Telechips Memory  Debugging 


/******************************************************************************
* globals
******************************************************************************/

/******************************************************************************
* locals
******************************************************************************/


/******************************************************************************
* declarations
******************************************************************************/
void* TCC_malloc (unsigned int iSize);
void* TCC_calloc (unsigned int isize_t, unsigned int iSize);
void* TCC_realloc (void *p,unsigned int iSize);
int TCC_free(void *pvPtr);

 #ifdef	TCC_MEMORY_DEBUG
 int TCC_memcmp(char *s0, char *s1, int size);
int TCC_memcpy(char *d, char *s, int size);
int TCC_memset(char *p, char val, int size);
void* TCC_malloc_1(unsigned int iSize);
 void* TCC_calloc_1(unsigned int size_t, unsigned int iSize);
 void* TCC_realloc_1(void *p,unsigned int iSize);
 int TCC_free_1(void *pvPtr);
#endif

#ifdef __cplusplus
}
#endif

#endif //_TCC_UTIL_H___
