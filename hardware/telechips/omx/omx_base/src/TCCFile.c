/******************************************************************************
* include 
******************************************************************************/
#include "TCCFile.h"

static int g_file_start_measure = 0;
static unsigned long long g_file_total_read_size = 0;
static unsigned long long g_file_total_measure  = 0;
static unsigned long long g_file_min  = 0x0FFFFFFF;
static unsigned long long g_file_max  = 0;
/******************************************************************************
*	FUNCTIONS			: TCC_fopen
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
TCC_FILE *TCC_fopen(char *path,char *mode)
{
#ifdef TCC_LINUX_FILE_SYTEM
	FILE *fp;
	fp = fopen(path, mode);
	if(fp == 0)
		printf("In %s : %s open error\n", __func__,path);
#ifdef	TCC_FILE_DEBUG
        printf("In %s : %s opened\n", __func__,path);
	printf("In %s : File Handle[0x%08x]\n", __func__, fp);	
#endif
	return fp;
#else

#endif

}
/******************************************************************************
*	FUNCTIONS			: TCC_fopen
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
int TCC_fclose(TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM

#ifdef	TCC_FILE_DEBUG
	printf("In %s, File Handle[0x%08x]\n", __func__, stream);		
#endif
	return fclose(stream);
#else

#endif

}
/******************************************************************************
*	FUNCTIONS			: TCC_fseek
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
int TCC_fseek(TCC_FILE *stream, long offset, int whence)
{
#ifdef TCC_LINUX_FILE_SYTEM
	int ret =	fseek(stream, offset, whence);
	
#ifdef	TCC_FILE_DEBUG
	printf("In %s[0x%08x] : offset[%d], whence[%d], now[%d]\n", __func__, stream, offset, whence, TCC_ftell(stream));
#endif

	return ret;
#else

#endif

}



/******************************************************************************
*	FUNCTIONS			: TCC_ftell
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
long  TCC_ftell(TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	return ( ftell(stream));
#else

#endif

}


/******************************************************************************
*	FUNCTIONS			: TCC_rewind
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
void  TCC_rewind(TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
#if 1
	return ( rewind(stream));
#else
	return( TCC_fseek(stream,0L,TCC_SEEK_SET) );
#endif
#else

#endif

}
		

/******************************************************************************
*	FUNCTIONS			: TCC_feof
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:	return 0 
*	ERRNO				:
******************************************************************************/
int TCC_feof(TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	return ((int)feof(stream));
#else
	
#endif

}

/******************************************************************************
*	FUNCTIONS			: TCC_fgetc
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				: 
******************************************************************************/
int TCC_fgetc(TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	return ((int)fgetc(stream));
#else
	
#endif

}


/******************************************************************************
*	FUNCTIONS			: TCC_fgets
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:
*	ERRNO				:
******************************************************************************/
char *TCC_fgets(char *s, int size, TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	return ((char *)fgets(s,size,stream));
#else
	
#endif

}

/******************************************************************************
*	FUNCTIONS			: TCC_fputs
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:	N > =
*	ERRNO				:	-1 or EOF
******************************************************************************/
int TCC_fputs(char *s, TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	return ((int)fputs(s,stream));
#else

	
#endif

}
	

/******************************************************************************
*	FUNCTIONS			: TCC_fread
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:	
*	ERRNO				:	
******************************************************************************/
int TCC_fread(char *s, int size_t, int size, TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	int ret;
	ret = fread(s, size_t, size, stream);
	if(ret < 0 )
		printf("In %s : read  error\n", __func__);
#ifdef	TCC_FILE_DEBUG
	printf("In %s[0x%08x] : addr[0x%08x], size[%d], now[%d]\n", __func__, stream, s, size, TCC_ftell(stream));
#endif
	return ret;
#else

	
#endif

}

/******************************************************************************
*	FUNCTIONS			: TCC_fwrite
*	SYNOPSIS			:
*	EXTERNAL EFFECTS	:
*	PARAMETERS			:
*	RETURNS				:	
*	ERRNO				:	
******************************************************************************/
int TCC_fwrite(char *s, int size_t, int size, TCC_FILE *stream)
{
#ifdef TCC_LINUX_FILE_SYTEM
	int ret;
	ret = fwrite(s, size_t, size, stream);
	if(ret < 0 )
                printf("In %s : write  error\n", __func__);

#ifdef	TCC_FILE_DEBUG
	printf("In %s[0x%08x] : addr[0x%08x], size[%d], now[%d]\n", __func__, stream, s, size, TCC_ftell(stream));
#endif
	return ret;
#else

	
#endif

}

void TCC_fStartMeasure(void)
{
	g_file_total_read_size = 0;
	g_file_total_measure   = 0;

	g_file_min  = 0xFFFFFFFF;
	g_file_max  = 0;

	g_file_start_measure = 1;

}

void TCC_fStopMeasure(void)
{
	g_file_start_measure = 0;
}

unsigned long long TCC_fGetMeasure(unsigned long long *min, unsigned long long *max)
{
	#if 0
	/* micro sec per 1KByte */
	unsigned int ret;
	if(g_file_total_read_size)
		ret = g_file_total_measure / ( g_file_total_read_size/1024) ;
	else
		ret = 0;
	#else
	unsigned long long ret;
	ret = g_file_total_measure;	

	*min = g_file_min;
	*max = g_file_max;
	#endif
	return ret;
}
