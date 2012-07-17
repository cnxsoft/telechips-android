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
 *		cdk_pre_define.h : This file is part of libcdk
 * \date
 *		2009/06/01
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		pre-defines
 *
 ***********************************************************************
 */
#ifndef _CDK_PRE_DEFINE_H_
#define _CDK_PRE_DEFINE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "cdk_error.h"

/************************************************************************

	Check Compiler Preprocessor
		- defines on preprocessor of arm compiler for generating lib.
		- TCC_ONBOARD

************************************************************************/
#if defined(__arm__) || defined(__arm) // Linux or ADS or WinCE
	#ifndef ARM
		#define ARM
	#endif
#endif

#ifdef ARM
/*
	#if	!defined( TCC_ONBOARD )
	#	Error in preprocessor, cannot find TCC_ONBOARD definition
	#	It must be defined TCC_ONBOARD in arm preprocessor
	#endif//TCC_ONBOARD
*/
#endif//ARM


/************************************************************************

							PREFIX-DEFINES

*************************************************************************/
#ifdef ARM
#	if ( ( defined(__GNUC__) && defined(ARM) ) && !defined(__ANDROID__) ) || defined(__LINUX__)		// ARM Linux
#		define ARM_LINUX
#	elif ( ( defined(__GNUC__) && defined(ARM) ) && !defined(__LINUX__) ) || defined(__ANDROID__)	// ARM Android
#		define ARM_ANDROID
#	elif defined(_WIN32_WCE) && defined(ARM)														// ARM WinCE
#		define ARM_WINCE
#			if defined(TCC89x) || defined(TCC92x)
#				define ARM_ARCH_V6
#			endif
#	else
#		if defined(TCC89x) || defined(TCC92x)
#			define ARM_RVDS												// ARM RVDS2.2
#		else
#			define ARM_ADS												// ARM ADS1.2
#		endif
#	endif
#else
#	define X86_WIN32
#endif

	//------------------------------------------------------
	// pre-define align
	//------------------------------------------------------
	#ifdef ARM
	#	define PRE_ALIGN_BYTES 4
	#else//WIN32
	#	define PRE_ALIGN_BYTES 8 // or 16 (SSE)
	#endif

/************************************************************************

				CDK Options

*************************************************************************/
#ifdef ARM_LINUX
//	#define ENABLE_TERMINAL_INTERFACE
#endif



/************************************************************************

				Container(Muxer/Demuxer) Options

*************************************************************************/
	//------------------------------------------------------
	// Supported Demuxer
	//------------------------------------------------------
	#define INCLUDE_MP4_DMX
	#define INCLUDE_MPG_DMX
	#define INCLUDE_MKV_DMX
	#define INCLUDE_AVI_DMX
	#define INCLUDE_TS_DMX
	#define INCLUDE_ASF_DMX
	#define INCLUDE_RM_DMX
	#define INCLUDE_OGG_DMX	
	#define INCLUDE_AUDIO_DMX
	#define INCLUDE_FLV_DMX
	#ifdef INCLUDE_FLV_DMX
	#	define FLV_DMX_AVC_TS_OPTION	1  //(0:DTS, 1:PTS)
	#endif

	//------------------------------------------------------
	// Supported Muxer
	//------------------------------------------------------
//	#define INCLUDE_MP4_MUX

/************************************************************************

				Video Codec Options

*************************************************************************/
	
	//------------------------------------------------------
	// CLK ON/OFF
	//------------------------------------------------------
//	#define VPU_CLK_CONTROL

	//------------------------------------------------------
	// Base Address of VCODEC register
	//------------------------------------------------------
#if defined(TCC_93XX_INCLUDE)
	#define VPU_REG_BASE_ADDR			0xB0900000	//Video CODEC
	#define VIDEOCACHE_BASE_ADDR		0xB0910000	//Video cache controller
	#define VIDEOBUSCONF_BASE_ADDR		0xB0920000	//Video Bus Configuration
	#define VIDEOCODECBUS_CLK_BASE_ADDR 0xB0500000	//CLKCTRL5: 0xB0500014 Core clock control register for video codec
#elif defined(TCC_88XX_INCLUDE) || defined(TCC_89XX_INCLUDE)
	#define VPU_REG_BASE_ADDR			0xF0700000	//Video CODE
	#if defined(TCC_88XX_INCLUDE)
	#define VIDEOCACHE_BASE_ADDR		0xF0710000	//Video cache controller
	#define VIDEOBUSCONF_BASE_ADDR		0xF0720000	//Video Bus Configuration
	#else
	#define VIDEOCACHE_BASE_ADDR		0xF0701000	//Video cache controller
	#define VIDEOBUSCONF_BASE_ADDR		0xF0702000	//Video Bus Configuration
	#endif
	#define VIDEOCODECBUS_CLK_BASE_ADDR 0xF0400000	//CLKCTRL5: 0xF0400014 Core clock control register for video codec
#elif defined(TCC_892X_INCLUDE) 
	#define VPU_REG_BASE_ADDR			0x75000000	//Video CODEC
	#define VIDEOCACHE_BASE_ADDR		0x75010000	//Video cache controller
	#define VIDEOBUSCONF_BASE_ADDR		0x75020000	//Video Bus Configuration
	#define VIDEOCODECBUS_CLK_BASE_ADDR 0x74000000	//CLKCTRL5: 0x74000014 Core clock control register for video codec
#else
#endif

	//------------------------------------------------------
	// Supported Video Decoder
	//------------------------------------------------------
	#define INCLUDE_VPU_DEC
	#ifdef INCLUDE_VPU_DEC
		#define INCLUDE_H264_DEC
		#define INCLUDE_H263_DEC
		#define INCLUDE_MPEG4_DEC
		#define INCLUDE_MPEG2_DEC
		#define INCLUDE_RV9_DEC
		#define INCLUDE_VC1_DEC
		#define INCLUDE_DIV3_DEC
	#endif

//	#if defined(ARM_LINUX) || defined(__LINUX__) || defined(__ANDROID__)
		#define INCLUDE_WMV78_DEC
//	#endif

	#if defined(TCC_89XX_INCLUDE)
		#define INCLUDE_SORENSON263_DEC
		#ifdef INCLUDE_SORENSON263_DEC
			#define SORENSON263_DEC_OUTPUT_MEM_ALIGN16_STRIDE	//A stride is multiple of 16
			#define SORENSON263_DEC_OUTPUT_MEM_ALIGN16			//16x16 pixel alignment
		#endif
	#endif

	//------------------------------------------------------
	// 1. Sequence header
	//------------------------------------------------------
	#define ATTACH_SEQ_HEADER_FOR_VPU_DEC // FIXME

	//------------------------------------------------------
	// 2. Decode
	//------------------------------------------------------


	//------------------------------------------------------
	// Supported Video Encoder
	//------------------------------------------------------
	#define INCLUDE_VPU_ENC
	#ifdef INCLUDE_VPU_ENC
		#define INCLUDE_H264_ENC
		#define INCLUDE_H263_ENC
		#define INCLUDE_MPEG4_ENC
	#endif


/************************************************************************

				Audio Codec Options

*************************************************************************/
	#define INCLUDE_AAC_DEC
	#define INCLUDE_MP3_DEC
	#define INCLUDE_MP2_DEC
	#define INCLUDE_DTS_DEC
	#define INCLUDE_AC3_DEC

	#define INCLUDE_WMA_DEC
	#define INCLUDE_RAG2_DEC
	#define INCLUDE_FLAC_DEC
	#define INCLUDE_APE_DEC
	#define INCLUDE_WAV_DEC
	#define INCLUDE_VORBIS_DEC
	#define INCLUDE_BSAC_DEC
	//#define INCLUDE_DRA_DEC
	//#define INCLUDE_MP3HD_DEC
	
	#define INCLUDE_QCELP_DEC
	#define INCLUDE_ARMNB_DEC
	#define INCLUDE_ARMWBPLUS_DEC
	#define INCLUDE_EVRC_DEC
	//#define INCLUDE_G722_DEC
	//#define INCLUDE_G729AB_DEC
	

/************************************************************************

				MACROS(DEBUGGING and ALIGNMENT)

*************************************************************************/

#ifndef SYS_ANSI_PRINT_COLOR
#define SYS_ANSI_PRINT_COLOR

#ifdef ARM_LINUX

/* ANSI Output Definitions */
#define T_DEFAULT		"\x1b[0m"
#define TS_BOLD			"\x1b[1m"
#define TS_ITALIC		"\x1b[3m"
#define TS_UNDER		"\x1b[4m"
#define TS_UPSET		"\x1b[7m"
#define TS_LINE			"\x1b[9m"

#define NS_BOLD			"\x1b[22m"
#define NS_ITALIC		"\x1b[23m"
#define NS_UNDER		"\x1b[24m"
#define NS_UPSET		"\x1b[27m"
#define NS_LINE			"\x1b[29m"

#define TC_BLACK		"\x1b[30m"
#define TC_RED			"\x1b[31m"
#define TC_GREEN		"\x1b[32m"
#define TC_YELLOW		"\x1b[33m"
#define TC_BLUE			"\x1b[34m"
#define TC_MAGENTA		"\x1b[35m"	//磊全
#define TC_CYAN			"\x1b[36m"	//没废
#define TC_WHITE		"\x1b[37m"
#define TC_RESET		"\x1b[39m"

#define BC_BLACK		"\x1b[40m"
#define BC_RED			"\x1b[41m"
#define BC_GREEN		"\x1b[42m"
#define BC_YELLOW		"\x1b[43m"
#define BC_BLUE			"\x1b[44m"
#define BC_MAGENTA		"\x1b[45m"	//磊全
#define BC_CYAN			"\x1b[46m"	//没废
#define BC_WHITE		"\x1b[47m"
#define BC_RESET		"\x1b[49m"

#define STYLE_TITLE		"\x1b[1;4;37;40m"
#define STYLE_SUBTITLE	"\x1b[1;37;40m"
#define STYLE_LINE1		"\x1b[1;37;40m"
#define STYLE_LINE2		"\x1b[1;37;40m"
#define STYLE_STEPLINE	"\x1b[1;32;40m"
#define STYLE_RESET		T_DEFAULT

#else

/* ANSI Output Definitions */
#define T_DEFAULT	""
#define TS_BOLD		""
#define TS_ITALIC	""
#define TS_UNDER	""
#define TS_UPSET	""
#define TS_LINE		""

#define NS_BOLD		""
#define NS_ITALIC	""
#define NS_UNDER	""
#define NS_UPSET	""
#define NS_LINE		""

#define TC_BLACK	""
#define TC_RED		""
#define TC_GREEN	""
#define TC_YELLOW	""
#define TC_BLUE		""
#define TC_MAGENTA	""
#define TC_CYAN		""
#define TC_WHITE	""
#define TC_RESET	""

#define BC_BLACK	""
#define BC_RED		""
#define BC_GREEN	""
#define BC_YELLOW	""
#define BC_BLUE		""
#define BC_MAGENTA	""
#define BC_CYAN		""
#define BC_WHITE	""
#define BC_RESET	""

#define STYLE_TITLE	""
#define STYLE_SUBTITLE	""
#define STYLE_LINE1	""
#define STYLE_LINE2	""
#define STYLE_STEPLINE	""
#define STYLE_RESET		T_DEFAULT

#endif

#endif//#ifndef SYS_ANSI_PRINT_COLOR


#if defined(ARM_WINCE) || defined(X86_WIN32)
	#define PRTI640		"%I64d"
	#define PRTI64(x)	"%"x"I64d"
	#define PRTUI640	"%UI64d"
	#define PRTUI64(x)	"%"x"UI64d"
#else
	#define PRTI640		"%lld"
	#define PRTI64(x)	"%"x"lld"
	#define PRTUI640	"%ulld"
	#define PRTUI64(x)	"%"x"ulld"
#endif



	//------------------------------------------------------
	// macros for debugging and alignment
	//------------------------------------------------------
	#if defined(ARM_ADS) || defined(ARM_RVDS)		// ARM ADS
	#	define inline __inline
	#	define ALIGNED __align(PRE_ALIGN_BYTES)
	#	define _T(x)
	#elif defined(ARM_WINCE)						// ARM WinCE
	#	define inline __inline
	#	define ALIGNED //__alignof(PRE_ALIGN_BYTES) : FIXME
	#elif defined(ARM_LINUX) || defined(__LINUX__)	// Linux/ARM Linux
	#	define DECLARE_ALIGNED( n ) __attribute__((aligned(n)))
	#	define ALIGNED __attribute__((aligned(PRE_ALIGN_BYTES)))
	#	define _T(x)
	#else//if defined(WIN32) && !defined(ARM_WINCE)
	#	define inline __inline
	#	define DECLARE_ALIGNED( n ) __declspec(align(n))
	#	define ALIGNED __declspec(align(PRE_ALIGN_BYTES))
	#endif

#if 0
#ifndef DPRINTF
	#if defined(_DEBUG) || defined(DEBUG)//----------
	#	if defined(_WIN32)
	#		include <windows.h>
	#		if defined(_WIN32_WCE)
				static inline void dprintf( char *fmt, ...)
				{
					va_list args;
					char buf[1024];
					wchar_t temp[1024];
					va_start(args, fmt);
					vsprintf(buf, fmt, args);
					va_end(args);
					wsprintf( temp, L"%hs", buf );
					OutputDebugString((LPCWSTR)buf);
					//printf(TC_RED);
					printf("%s", buf);
					//printf(T_DEFAULT);
				}
	#		define DPRINTF dprintf
	#		else
	#			include <stdio.h>
				static inline void dprintf( char *fmt, ...)
				{
					va_list args;
					char buf[1024];
					va_start(args, fmt);
					vsprintf(buf, fmt, args);
					va_end(args);
					OutputDebugString(buf);
					//printf("%s", buf);
				}
	#		define DPRINTF dprintf
	#		ifdef DEBUG_OUT_LOG_FILE
				static inline void lprintf( char *fmt, ...)
				{
					va_list args;
					char buf[1024];
					FILE *fp_log = fopen( OUT_LOG_FILE, "a+" );
					va_start(args, fmt);
					vsprintf(buf, fmt, args);
					va_end(args);
					fprintf(fp_log, "%s", buf);
					fclose(fp_log);
				}
	#		undef DPRINTF
	#		define DPRINTF lprintf
	#		endif//DEBUG_OUT_LOG_FILE
	#		endif
	#	else//#if !defined(_WIN32)
	#		include <stdio.h>
	#		define DPRINTF printf // printf
	#	endif//#if !defined(_WIN32)
	#else//RELEASE MODE---------------------------------
		static inline void RPRINTF( char *fmt, ... ) { }
	#	define DPRINTF RPRINTF
	#endif//end of if defined(_DEBUG) || defined(DEBUG)
#endif//DPRINTF

	#define ERR_PRINTF_STATUS
	#ifdef ERR_PRINTF_STATUS
		#undef DPRINTF
		#include <stdio.h>
		#define DPRINTF printf
	#endif

	//------------------------------------------------------
	// For Debugging
	//------------------------------------------------------
	#if defined(_DEBUG)
		#define DEBUG_HEAP_INFO
	#endif

	#define DBG_PRINTF_RUN_STATUS
	#ifdef DBG_PRINTF_RUN_STATUS
		#include <stdio.h>
		#define DSTATUS printf
		//for VPU log		
		//#define PRINT_VPU_INPUT_STREAM
		#ifdef PRINT_VPU_INPUT_STREAM
			#define PRINT_BYTES 40
		#endif
	#else
		#define DSTATUS DPRINTF
	#endif

/************************************************************************/
#endif

#ifdef __cplusplus
}
#endif
#endif//_CDK_PRE_DEFINE_H_
