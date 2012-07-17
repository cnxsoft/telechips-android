/******************************************************************************
* include 
******************************************************************************/
#include "TCCMemory.h"
#define LOG_TAG	"TCCMemory"
#include <utils/Log.h>


#ifdef	TCC_MEMORY_DEBUG
static unsigned int gMemallocCnt = 0;
static unsigned int gMemRemain = 0;
#endif


/******************************************************************************
*	FUNCTIONS			: TCC_malloc
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void* TCC_malloc (unsigned int iSize)
{
#ifdef TCC_LINUX_MEMORY_SYTEM
	void *ptr;
	ptr = malloc(iSize);
#ifdef TCC_MEMORY_DEBUG
	LOGD("mAlloc, 0x%08x, size[%d], mem cnt [%d]\n", ptr, iSize, gMemallocCnt);		
	gMemallocCnt++;	
#endif

	return ptr;
#else  
#endif
}
/******************************************************************************
*	FUNCTIONS			: TCC_malloc
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void* TCC_calloc (unsigned int isize_t, unsigned int iSize)
{
#ifdef TCC_LINUX_MEMORY_SYTEM
	void *ptr;
	ptr = calloc(isize_t, iSize);
#ifdef TCC_MEMORY_DEBUG
	LOGD("cAlloc, 0x%08x, size[%d], mem cnt [%d]\n", ptr, iSize, gMemallocCnt);		
	gMemallocCnt++;	
#endif
	return ptr;
#else  
#endif
}
/******************************************************************************
*	FUNCTIONS			: TCC_malloc
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void* TCC_realloc (void *p,unsigned int iSize)
{
#ifdef TCC_LINUX_MEMORY_SYTEM
	void *ptr;
	ptr = realloc(p, iSize);
#ifdef TCC_MEMORY_DEBUG
	LOGD("reAlloc, 0x%08x, size[%d], mem cnt [%d]\n", ptr, iSize, gMemallocCnt);		
	gMemallocCnt++;	

#endif
	return ptr;	
#else  
#endif
}
/******************************************************************************
*	FUNCTIONS			: TCC_free()
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/

int TCC_free(void *pvPtr)
{
#ifdef TCC_LINUX_MEMORY_SYTEM
#ifdef TCC_MEMORY_DEBUG
	gMemallocCnt--;
	LOGD("MemFree, 0x%08x, mem cnt [%d]\n", pvPtr, gMemallocCnt);	
#endif
	free(pvPtr);
#else
#endif

	return 0;
}



#if 0//def	TCC_MEMORY_DEBUG
/******************************************************************************
*	FUNCTIONS			: TCC_memcmp
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/

int TCC_memcmp(char *s0, char *s1, int size)
{
	int i;
	//printf("In %s, [0x%08x] & [0x%08x], size[%d]\n", __func__, s0, s1, size);	
	for(i=0;i<size;i++)
	{
		if( *(s0+i) != *(s1+i) ) 
		{
			return 1;
		}		
	}
	return 0;
}
/******************************************************************************
*	FUNCTIONS			: TCC_memcpy
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/

int TCC_memcpy(char *d, char *s, int size)
{
	int cnt = 0;
	//printf("In %s, [0x%08x] -> [0x%08x], size[%d]\n", __func__, s, d, size);	
	while(size--)
	{
		*d++ = *s++;
		cnt++;
	}
	return cnt;
}
/******************************************************************************
*	FUNCTIONS			: TCC_memset
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/

int TCC_memset(char *p, char val, int size)
{
	int cnt = 0;
	//printf("In %s, 0x%08x, size[%d]\n", __func__, p, size);
	while(size--)
	{
		*p++ = val;
		cnt++;
	}
	return cnt;
}

/******************************************************************************
*	FUNCTIONS			: TCC_malloc_1
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/

void* TCC_malloc_1(unsigned int iSize)
{
	void *ptr;
//	iSize += 1024;
	ptr = malloc(iSize);
	gMemallocCnt++;	
//	printf("In %s, 0x%08x, size[%d], mem cnt [%d]\n", __func__, ptr, iSize, gMemallocCnt);		
	return ptr;
}
/******************************************************************************
*	FUNCTIONS			: TCC_calloc_1
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void* TCC_calloc_1(unsigned int size_t, unsigned int iSize)
{
	void *ptr;
//	iSize += 1024;
	ptr = calloc(size_t, iSize);
//	printf("In %s, 0x%08x, size[%d], mem cnt [%d]\n", __func__, ptr, iSize, gMemallocCnt);		
	gMemallocCnt++;		
	return ptr;
}
/******************************************************************************
*	FUNCTIONS			: TCC_realloc_1
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void* TCC_realloc_1(void *p,unsigned int iSize)
{
	void *ptr;
//	iSize += 1024;	
	ptr = realloc(p, iSize);
//	printf("In %s, 0x%08x, size[%d], mem cnt [%d]\n", __func__, ptr, iSize, gMemallocCnt);		
	gMemallocCnt++;		
	return ptr;	
}
/******************************************************************************
*	FUNCTIONS			: TCC_free_1
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
int TCC_free_1(void *pvPtr)
{
	gMemallocCnt--;
//	printf("In %s, 0x%08x, mem cnt [%d], remain[%d]\n", __func__, pvPtr, gMemallocCnt);	
	free(pvPtr);
}


#endif



