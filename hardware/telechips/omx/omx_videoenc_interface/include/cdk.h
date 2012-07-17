/*!
 ***********************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2009 Telechips Inc.
  Korad Bldg, 1000-12 Daechi-dong, Kangnam-Ku, Seoul, Korea					
 \endverbatim
 ***********************************************************************
 */
/*!
 ***********************************************************************
 *
 * \file
 *		cdk.h
 * \date
 *		2009/05/29
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		cdk library main api
 * \version
 *		- 0.0.1 : 2009/05/29
 *
 ***********************************************************************
 */
#ifndef _CDK_H_
#define _CDK_H_
#ifdef __cplusplus
extern "C" {
#endif

#define CDK_VERSION_STRING "CDK_V023_Build_0078"

/************************************************************************/
/*						                                                */
/* CDK Type				                                                */
/*						                                                */
/************************************************************************/
#if defined(__arm__) || defined(__arm) || defined(ARM) // Linux or ADS or WinCE
	#if defined(_WIN32_WCE)					// ARM WinCE
		typedef unsigned __int64  CDK_U64;
		typedef __int64 CDK_S64;
	#else
		typedef unsigned long long CDK_U64;
		typedef signed long long CDK_S64;
	#endif
#endif

#ifndef CDK_TYPES
	/*
#if defined(_WIN32_WCE) && ( defined(ARM) || defined(__arm__) || defined(__arm) ) // ARM WinCE
	typedef unsigned __int64  uint64_t;
	typedef __int64 int64_t;
#else
	typedef unsigned long long uint64_t;
	typedef signed long long int64_t;
#endif
	*/
typedef int cdk_handle_t;
typedef int cdk_result_t;
typedef int cdk_bool_t;
#endif

//! Callback Func
typedef struct cdk_callback_func_t
{
	void* (*m_pfMalloc			) ( unsigned int );								//!< malloc
	void* (*m_pfNonCacheMalloc	) ( unsigned int );								//!< non-cacheable malloc 
	void  (*m_pfFree			) ( void* );									//!< free
	void  (*m_pfNonCacheFree	) ( void* );									//!< non-cacheable free
	void* (*m_pfMemcpy			) ( void*, const void*, unsigned int );			//!< memcpy
	void* (*m_pfMemset			) ( void*, int, unsigned int );					//!< memset
	void* (*m_pfRealloc			) ( void*, unsigned int );						//!< realloc
	void* (*m_pfMemmove			) ( void*, const void*, unsigned int );			//!< memmove
	void* (*m_pfCalloc			) ( unsigned int , unsigned int );				//!< calloc
	int  (*m_pfMemcmp			) ( const void* , const void*, unsigned int );	//!< memcmp

	void* (*m_pfPhysicalAlloc	) ( unsigned int );								//!< alloc function for physical memory allocation
	void  (*m_pfPhysicalFree	) ( void*, unsigned int );						//!< free function for physical memory free
	void* (*m_pfVirtualAlloc	) ( int*, unsigned int, unsigned int );			//!< alloc function for virtual memory allocation
	void  (*m_pfVirtualFree		) ( int*, unsigned int, unsigned int );			//!< free function for virtual memory free
	int m_Reserved1[16-13];

	void*		 (*m_pfFopen ) ( const char *, const char * );						//!< fopen
	unsigned int (*m_pfFread ) ( void*, unsigned int, unsigned int, void* );		//!< fread
	int			 (*m_pfFseek ) ( void*, long, int );								//!< fseek
	long		 (*m_pfFtell ) ( void* );											//!< ftell
	unsigned int (*m_pfFwrite) ( const void*, unsigned int, unsigned int, void* );	//!< fwrite
	int			 (*m_pfFclose) ( void* );											//!< fclose
	int			 (*m_pfUnlink) ( const char* );										//!< _unlink
	unsigned int (*m_pfFeof  ) ( void* );											//!< feof
	unsigned int (*m_pfFflush) ( void* );											//!< fflush

	int			 (*m_pfFseek64) ( void*, CDK_S64, int );							//!< fseek 64bit io
	CDK_S64		 (*m_pfFtell64) ( void* );											//!< ftell 64bit io
	int m_Reserved2[16-11];
} cdk_callback_func_t;

typedef int* cdk_param_t;
typedef struct cdk_command_t
{
	int m_iCmdNumber;			//!< command total number
	cdk_param_t m_tMessage[8];	//!< command message
} cdk_command_t;

typedef char cdk_string_t;
typedef struct cdk_core_t 
{
	cdk_string_t* m_pcOpenFileName;		//!< open file name

	cdk_string_t* m_pcOutYuvFileName;	//!< file name for output yuv
	void* m_pfOutYuvFile;				//!< file handle for output yuv 
	unsigned char* m_pOutYuv;			//!< pointer for output yuv
	cdk_string_t* m_pcOutYuvRefFileName;//!< ref file name for output yuv
	void* m_pfOutYuvRefFile;			//!< ref file handle for output yuv 
	unsigned char* m_pOutYuvRef;		//!< ref pointer for output yuv

	cdk_string_t* m_pcOutWavFileName;	//!< file name for output wav
	void* m_pfOutWavFile;				//!< file handle for output wav
	unsigned char* m_pOutWav;			//!< pointer for output wav
	cdk_string_t* m_pcOutWavRefFileName;//!< ref file name for output wav
	void* m_pfOutWavRefFile;			//!< ref file handle for output wav 
	unsigned char* m_pOutWavRef;		//!< ref pointer for output wav

	cdk_string_t* m_pcSeekCommandFileName;  //!< command file name for seek
	void* m_pfSeekCommandFile;				//!< command file handle for seek	
	int m_iSeekPatternTest;					//!< seek pattern

	cdk_callback_func_t* m_psCallback;	//!< callback function
	int 	m_iAudioHandle;				//!< audio handle
	int		m_iAudioProcessMode;		//!< processing mode for audio
	int		m_iVideoProcessMode;		//!< processing mode for video
	int		m_iPlayMode;				//!< play mode
	float		m_fPlaybackSpeed;		//!< Enhance playback speed.(1.0~2.0)
	int		m_iSkimmingModeSpeed;		//!< Enhance playback speed.(-32~32)

	unsigned int m_bM4vDeblk;
	unsigned int m_uiVpuDecOption;
	unsigned int m_uiVideoMaxResolution;

	// For encoding
	cdk_string_t* m_pszInputVideoFileName;		//!< input video file name
	cdk_string_t* m_pszInputAudioFileName;		//!< input audio file name
	cdk_string_t* m_pszEncodedOutFileName;		//!< encoded output file name
	void* m_pfInputVideoFile;					//!< input video file handle
	void* m_pfInputAudioFile;					//!< input audio file handle
	void* m_pfEncodedOutFile;					//!< encoded out file handle

	// set video
	int m_iVideoCodec;
	int m_iVideoWidth;
	int m_iVideoHeight;
	int m_iVideoKbps;
	int m_iVideoFramesPerSec;
	int m_iVideoIFrameInterval;
	// set audio
	int m_iAudioCodec;
	int m_iAudioSamplePerSec;
	int m_iAudioBitsPerSample;
	int m_iAudioChannels;
	int m_iAudioBitRates;
	int m_iAudioSource;
	int m_iAudioRecTimeSec;
	int m_iAudioSubCodec;						

	int m_iAudioMode;							//!< decoding or encoding
		
		

} cdk_core_t;

typedef int (cdk_func_t) ( int iOpCode, int* pHandle, void* pParam1, void* pParam2, void* pParam3);
typedef int (cdk_audio_func_t) ( int iOpCode, int* pHandle, void* pParam1, void* pParam2);

#define CP(x) ((cdk_param_t)(x))

/************************************************************************/
/*						                                                */
/* CDK Defines			                                                */
/*						                                                */
/************************************************************************/
// Op Code 
#define CDK_COMMAND_CREATE	0
#define CDK_COMMAND_DESTROY 1
#define CDK_COMMAND_RUN		2
#define CDK_COMMAND_STOP	3
#define CDK_COMMAND_CLOSE	4
#define CDK_COMMAND_RUN_DECODING	5
#define CDK_COMMAND_RUN_ENCODING	6
#define CDK_COMMAND_RUN_TRANSCODING	7

#define CDK_COMMAND_SET_OPEN_FILE			100
#define CDK_COMMAND_SET_VIDEO_OUT_FILE		101
#define CDK_COMMAND_SET_AUDIO_OUT_FILE		102
#define CDK_COMMAND_SET_IO_FILES			103
#define CDK_COMMAND_SET_SEEK_COMMAND_FILE	104
#define CDK_COMMAND_SET_ENC_OPEN_FILE		105
#define CDK_COMMAND_SET_ENC_VIDEO_PARAM		106
#define CDK_COMMAND_SET_ENC_AUDIO_PARAM		107

#define CONTAINER_TYPE_NONE 0
#define CONTAINER_TYPE_MKV	1 
#define CONTAINER_TYPE_MP4	2 
#define CONTAINER_TYPE_AVI	3 
#define CONTAINER_TYPE_MPG	4 
#define CONTAINER_TYPE_TS	5
#define CONTAINER_TYPE_ASF	6
#define CONTAINER_TYPE_RMFF 7
#define CONTAINER_TYPE_AUDIO 8
#define CONTAINER_TYPE_OGG 9
#define CONTAINER_TYPE_FLV 10

#define PLAYMODE_PROF_DMX			0x0001
#define PLAYMODE_PROF_ADEC			0x0002
#define PLAYMODE_PROF_VDEC			0x0004
#define PLAYMODE_PROF_CUMULATE		0x0010
#define PLAYMODE_PRINT_STATUS		0x0100
#define PLAYMODE_CROP_ENABLE		0x0200
#define PLAYMODE_CHROMA_INTERLEAVE	0x0400
#define PLAYMODE_STREAM_SELECT		0x0800
#define PLAYMODE_SKIP_FRAMES			0x1000
#define PLAYMODE_USE_USERDATA		0x2000
#define PLAYMODE_SKIMMING_WITH_SEEK				0x00004000
#define PLAYMODE_SKIMMING_WITH_DISCARD_AUDIO	0x00008000


/*!
 ***********************************************************************
 * \brief
 *		TCC_CDK_PROC	: main api function of CDK
 * \param
 *		[in]iOpCode		: operation code
 * \param
 *		[in,out]pHandle	: handle of CDK
 * \param
 *		[in]pParam1		: init or input parameter
 * \param
 *		[in]pParam2		: output or information parameter
 * \return
 *		If successful, TCC_CDK_PROC returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
cdk_result_t 
TCC_CDK_PROC( int iOpCode, cdk_handle_t* pHandle, void* pParam1, void* pParam2 );

#ifdef __cplusplus
}
#endif
#endif //_CDK_H_
