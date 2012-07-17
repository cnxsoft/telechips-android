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
 *		venc.h
 * \date
 *		2009/07/23
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		video encoder
 *
 ***********************************************************************
 */
#ifndef _VENC_H_
#define _VENC_H_

#ifdef HAVE_ANDROID_OS
#include <mach/TCC_VPU_CODEC.h>
#else
#include "../cdk/cdk_core.h"
#include "../cdk/cdk_port.h"
#ifdef INCLUDE_VPU_ENC
	#include "vpu/TCC_VPU_CODEC.h" // VPU video codec
#endif
#endif

#ifdef HAVE_ANDROID_OS
//Caution: Don't exceed the number of queue allocated in the queue.h [MAX_QUEUE_ELEMENTS 100].
#define VIDEO_ENC_BUFFER_COUNT 10 //20//150
#ifdef MOVE_VPU_IN_KERNEL
#define K_VA 2
#endif
#endif

#if !defined(_TCC8920_)
#define ENABLE_RATE_CONTROL
#define MULTI_SLICES_AVC
#endif

#if 0// from vpu v0.0.57
#define CHANGE_QP_FOR_IFRAME
#else
#define REMOVE_RC_AUTO_SKIP
#endif

#define VPU_ENV_INIT_ERROR		10000

#define VENC_PIC_TYPE_I	0
#define VENC_PIC_TYPE_P	1

typedef struct venc_init_t
{
	unsigned int m_RegBaseVirtualAddr;	//!< virtual address BIT_BASE

	// Encoding Info
	int m_iBitstreamFormat;				
	int m_iPicWidth;					//!< Width  : multiple of 16
	int m_iPicHeight;					//!< Height : multiple of 16
	int m_iFrameRate;					//!< Frame rate
	int m_iTargetKbps;					//!< Target bit rate in Kbps. if 0, there will be no rate control, 
										//!< and pictures will be encoded with a quantization parameter equal to quantParam

	int m_iKeyInterval;					//!< Key interval : max 32767
	int m_iAvcFastEncoding;				//!< fast encoding for AVC( 0: default, 1: encode intra 16x16 only )

	unsigned int m_BitstreamBufferAddr; //!< physical address : multiple of 4
	unsigned int m_BitstreamBufferAddr_VA;
	int m_iBitstreamBufferSize;			//!< bitstream buffer size : multiple of 1024

	//! Options
	int m_iSliceMode;
	int m_iSliceSizeMode;
	int m_iSliceSize;

	//! Callback Func
	void* (*m_pfMemcpy ) ( void*, const void*, unsigned int );	//!< memcpy
	void  (*m_pfMemset ) ( void*, int, unsigned int );			//!< memset
	int   (*m_pfInterrupt ) ( void );								//!< hw interrupt
} venc_init_t;

typedef struct venc_seq_header_t
{
	codec_addr_t m_SeqHeaderBuffer[2];	//!< [in]  Seqence header buffer
	int m_iSeqHeaderBufferSize;			//!< [in]  Seqence header buffer size
	unsigned char* m_pSeqHeaderOut;		//!< [out] Seqence header pointer
	int m_iSeqHeaderOutSize;			//!< [out] Seqence header size
} venc_seq_header_t;

typedef struct venc_input_t
{
	int m_bCbCrInterleaved;
	unsigned char* m_pInputY;
	unsigned char* m_pInputCbCr[2];

	int m_iChangeRcParamFlag;	//0: disable, 3:enable(change a bitrate), 5: enable(change a framerate), 7:enable(change bitrate and framerate)
	int m_iChangeTargetKbps;
	int m_iChangeFrameRate;
	int m_iQuantParam;

	codec_addr_t m_BitstreamBufferPA;	//!< [in] physical address
	int m_iBitstreamBufferSize;
	unsigned char request_IntraFrame;
} venc_input_t;

typedef struct venc_output_t 
{
	unsigned char* m_pBitstreamOut;
	int m_iBitstreamOutSize;
	int	m_iSliceCount;
	unsigned int* m_pSliceInfo;
	int m_iPicType;
} venc_output_t;

#define VENC_INIT		0
#define VENC_CLOSE		1
#define VENC_SEQ_HEADER	2
#define VENC_ENCODE		3


int
venc_vpu( int iOpCode, int* pHandle, void* pParam1, void* pParam2 );


unsigned char *vpu_get_BitstreamBufAddr(unsigned int index);

unsigned char *vpu_getStreamOutPhyAddr(unsigned char *convert_addr, unsigned int base_index);

#endif //_VENC_H_ 
