/**
  @file src/components/ffmpeg/omx_videodec_component.h
  
  This component implements All Video decoder. (MPEG4/AVC/H.263/WMV/RV)
  The H.264 / MPEG-4 AVC Video decoder is based on the FFmpeg software library.

  Copyright (C) 2007-2008 STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies).
  Copyright (C) 2009-2010 Telechips Inc.

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

  $Date: 2009/03/10 13:33:29 $
  Revision $Rev: 554 $
  Author $Author: B060934 $
  Android revised by ZzaU.
*/

#ifndef _OMX_RVDEC_COMPONENT_H_
#define _OMX_RVDEC_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <OMX_Video.h>
#include <OMX_TCC_Index.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <omx_base_filter.h>

#include <string.h>
#include "TCCMemory.h"

#include <tcc_video_common.h>

#include <vdec.h>
#include <libpmap/pmap.h>

#define ICS_THUMBNAIL_CREATION

#define ANDROID_USE_GRALLOC_BUFFER
#define MAXNUMOFPORTS		2

#ifdef ANDROID_USE_GRALLOC_BUFFER
#include <hardware/gralloc.h>
#include <hardware/hardware.h>
#endif

#define TIMESTAMP_CORRECTION
/*==================================================================================*/
// PROXY_BUFFER_TYPE : 
// This enumeration tells the type of buffer pointers coming to OMX in UseBuffer call.
/*==================================================================================*/
typedef struct PORT_TYPE
{
	BUFFER_TYPE BufferType;   	/*Used when buffer pointers which come from the normal virtual space */
	OMX_U32 IsBuffer2D;   		/*Used when buffer pointers which come from Gralloc allocations */
} PORT_TYPE;

typedef enum COPY_MODE
{
	COPY_NONE,
	COPY_START,
	COPY_DONE,
	COPY_FAILED,
	COPY_PRIV_DATA
}COPY_MODE;

#ifdef MOVE_HW_OPERATION
typedef enum COPY_OPER_MODE
{
	SEND_CMD = 0x1,
	WAIT_RESPOND = 0x2
}COPY_OPER_MODE;
#endif

typedef struct gralloc_use_t 
{
#ifdef ANDROID_USE_GRALLOC_BUFFER
	gralloc_module_t const *grallocModule;
#endif
	PORT_TYPE PortBuffers[MAXNUMOFPORTS];
	OMX_S32   fd_copy;
	unsigned char* m_pDispOut[2][3];	//!< physical address of Y, Cb, Cr component
} gralloc_use_t;
#define RMVB_DECODER_TR_TEST
#ifdef RMVB_DECODER_TR_TEST

typedef struct rmff_frame_t{
	int Current_TR;
	int Previous_TR;
	int Current_time_stamp;
	int Previous_time_stamp;
} rmff_frame_t;

typedef struct rmff_frame_time_t {
	rmff_frame_t  ref_frame;		
	rmff_frame_t  frame_P1;
	rmff_frame_t  frame_P2;
} rmff_frame_time_t;

#endif
typedef struct dec_disp_info_ctrl_t
{
	int       m_iTimeStampType;							   //! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS)
	int       m_iStdType;								   //! STD type
	int       m_iFmtType;								   //! Formater Type

	int       m_iUsedIdxPTS;							   //! total number of decoded index for PTS
	int       m_iRegIdxPTS[32];							   //! decoded index for PTS
	void     *m_pRegInfoPTS[32];						   //! side information of the decoded index for PTS

	int       m_iDecodeIdxDTS;							   //! stored DTS index of decoded frame
	int       m_iDispIdxDTS;							   //! display DTS index of DTS array
	int       m_iDTS[32];								   //! Decode Timestamp (decoding order)

	int       m_Reserved;
} dec_disp_info_ctrl_t;

typedef struct dec_disp_info_t
{
	int       m_iFrameType;								   //! Frame Type

	int       m_iTimeStamp;								   //! Time Stamp
	int       m_iRvTimeStamp;							   //! TR(RV)

	int       m_iPicStructure;							   //! PictureStructure
	int       m_iM2vFieldSequence;						   //! Field sequence(MPEG2) 
	int       m_iFrameDuration;							   //! MPEG2 Frame Duration

	int       m_iFrameSize;								   //! Frame size

	int       m_iTopFieldFirst;                            //! Top Field First
	int       m_iIsProgressive;                            //! Interlace information :: 0:interlace, 1:progressive	
} dec_disp_info_t;

typedef struct dec_disp_info_input_t
{
	int       m_iFrameIdx;								   //! Display frame buffer index for CVDEC_DISP_INFO_UPDATE command
	//! Decoded frame buffer index for CVDEC_DISP_INFO_GET command
	int       m_iStdType;								   //! STD type for CVDEC_DISP_INFO_INIT
	int       m_iTimeStampType;							   //! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS) for CVDEC_DISP_INFO_INIT
	int       m_iFmtType;								   //! Formater Type specification
	int       m_iFrameRate;
} dec_disp_info_input_t;

typedef struct mpeg2_pts_ctrl
{
	int       m_iLatestPTS;
	int       m_iPTSInterval;
	int       m_iRamainingDuration;
} mpeg2_pts_ctrl;

typedef struct VideoStartInfo
{
	OMX_U32		nDevID;
	OMX_U32		nState;
	OMX_U32		nFormat;
	pthread_mutex_t mutex;
} VideoStartInfo;


typedef struct ts_pts_ctrl{
	int m_iLatestPTS;
	int m_iPTSInterval; //[usec]
	int m_iRamainingDuration;
	int m_iRealPTS;
	int m_iInterpolationCount;
} ts_pts_ctrl;
//ts_pts_ctrl gsTSPtsInfo;

#ifdef TIMESTAMP_CORRECTION
typedef struct pts_ctrl{
	int m_iLatestPTS;
	int m_iPTSInterval;
	int m_iRamainingDuration;
} pts_ctrl;
#endif

typedef struct _VIDEO_DECOD_INSTANCE_ {
	/** @param avcodecReady boolean flag that is true when the video coded has been initialized */
	OMX_BOOL avcodecReady;
	/** @param video_coding_type Field that indicate the supported video format of video decoder */
	OMX_U32 video_coding_type;
	OMX_U8	container_type;
	OMX_U32  bitrate_mbps;
	vdec_input_t gsVDecInput;
	vdec_output_t gsVDecOutput;
	vdec_init_t gsVDecInit;
	vdec_user_info_t gsVDecUserInfo;
	pts_ctrl gsPtsInfo;
	/** @param Android Only flag */
	OMX_BOOL avcodecInited;
	OMX_BOOL  isVPUClosed;
	cdmx_info_t cdmx_info;
	OMX_U32 video_dec_idx;
	dec_disp_info_ctrl_t dec_disp_info_ctrl;
	dec_disp_info_t dec_disp_info[32];
	dec_disp_info_input_t dec_disp_info_input;
	OMX_PTR pVdec_Instance;
	cdk_func_t *gspfVDec;
	int gsRvTRDelta;
	int gsRvP_frame_cnt;
	int gsRvReference_Flag;
	rmff_frame_time_t gsRmff_frame_time;
} _VIDEO_DECOD_INSTANCE_;
/** Video Decoder component private structure.
  */
/**
 * Pixel format. Notes:
 *
 * PIX_FMT_RGB32 is handled in an endian-specific manner. A RGBA
 * color is put together as:
 *  (A << 24) | (R << 16) | (G << 8) | B
 * This is stored as BGRA on little endian CPU architectures and ARGB on
 * big endian CPUs.
 *
 * When the pixel format is palettized RGB (PIX_FMT_PAL8), the palettized
 * image data is stored in AVFrame.data[0]. The palette is transported in
 * AVFrame.data[1] and, is 1024 bytes long (256 4-byte entries) and is
 * formatted the same as in PIX_FMT_RGB32 described above (i.e., it is
 * also endian-specific). Note also that the individual RGB palette
 * components stored in AVFrame.data[1] should be in the range 0..255.
 * This is important as many custom PAL8 video codecs that were designed
 * to run on the IBM VGA graphics adapter use 6-bit palette components.
 */
 
DERIVEDCLASS(omx_videodec_component_PrivateType, omx_base_filter_PrivateType)
#define omx_videodec_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
  /** @param semaphore for avcodec access syncrhonization */\
  tsem_t* avCodecSyncSem; \
  /** @param pVideorV Referece to OMX_VIDEO_PARAM_RVTYPE structure*/ \
  OMX_VIDEO_PARAM_RVTYPE pVideoRv; \
  /** @param pVideoMpeg4 Referece to OMX_VIDEO_PARAM_H263TYPE structure*/ \
  OMX_VIDEO_PARAM_H263TYPE pVideoH263; \
  /** @param pVideoAvc Reference to OMX_VIDEO_PARAM_AVCTYPE structure */ \
  OMX_VIDEO_PARAM_AVCTYPE pVideoAvc; \
  /** @param pVideoMpeg4 Referece to OMX_VIDEO_PARAM_MPEG4TYPE structure*/ \
  OMX_VIDEO_PARAM_MPEG4TYPE pVideoMpeg4; \
  /** @param pVideoWmv Reference to OMX_VIDEO_PARAM_WMVTYPE structure */ \
  OMX_VIDEO_PARAM_WMVTYPE pVideoWmv; \
  /** @param pVideoWmv Reference to OMX_VIDEO_PARAM_DIVXTYPE structure */ \
  /** OMX_VIDEO_PARAM_DIVXTYPE pVideoDivx; */ \
  /** @param pVideoWmv Reference to OMX_VIDEO_PARAM_MPEG2TYPE structure */ \
  OMX_VIDEO_PARAM_MPEG2TYPE pVideoMpeg2; \
  /** @param avcodecReady boolean flag that is true when the video coded has been initialized */ \
/*  OMX_BOOL avcodecReady;*/ \
  /** @param minBufferLength Field that stores the minimun allowed size for FFmpeg decoder */ \
  OMX_U16 minBufferLength; \
  /** @param inputCurrBuffer Field that stores pointer of the current input buffer position */ \
  OMX_U8* inputCurrBuffer; \
  /** @param inputCurrLength Field that stores current input buffer length in bytes */ \
  OMX_U32 inputCurrLength; \
  /** @param code_type Field that indicate a code type has searched */ \
  OMX_S32 code_type; \
  /** @param nTimeStamp Field that indicate a timestamp of frame has decided */ \
  OMX_TICKS nTimeStamp; \
  /** @param isFirstBuffer Field that the buffer is the first buffer */ \
  OMX_S32 isFirstBuffer; \
  /** @param isNewBuffer Field that indicate a new buffer has arrived*/ \
  OMX_S32 isNewBuffer; \
  /** @param video_coding_type Field that indicate the supported video format of video decoder */ \
  OMX_U32 video_coding_type; \
  /** @param eOutFramePixFmt Field that indicate output frame pixel format */ \
  PIXEL_FORMAT_E eOutFramePixFmt; \
  /** @param extradata pointer to extradata*/ \
  OMX_U8* extradata; \
  /** @param extradata_size extradata size*/ \
  OMX_U32 extradata_size; \
/*  OMX_U8  container_type;*/ \
  OMX_U32  frameSearchOrSkip_flag; \
  OMX_S32  i_skip_scheme_level; \
  OMX_S32  i_skip_count; \
  OMX_S32  i_skip_interval; \
  vdec_input_t gsVDecInput; \
  vdec_output_t gsVDecOutput; \
  vdec_init_t gsVDecInit; \
  vdec_user_info_t gsVDecUserInfo; \
  OMX_U8* pConfigdata; \
  OMX_U32 szConfigdata; \
  /** @param Android Only flag */ \
/*  OMX_BOOL avcodecInited;*/ \
  OMX_BOOL isFirstSyncFrame; \
  OMX_BOOL bUseFrameDefragmentation; \
  OMX_BOOL bDelayedDecodeOut; \
  OMX_U32  extractorType; \
  OMX_U32  frameFilterPTS; \
  OMX_U32 (*SearchCodeType)( \
    OMX_INOUT void* omx_private_type, \
    OMX_OUT   OMX_U32* input_offset, \
    OMX_IN    OMX_U32 search_option); \
  OMX_U32  frameDefragmentationType; \
  OMX_BOOL bDetectFrameDelimiter; \
  OMX_BOOL bSetDecodeOnlyMode; \
  OMX_BOOL bUseDecodeOnlyMode; \
  OMX_U32  skipFrameNum; \
  OMX_U32  numSkipFrame; \
  OMX_U32  decodeOnlyErrNum; \
  OMX_U32  I_frame_search_mode; \
  OMX_U32  AVC_naltype_mask; \
  OMX_U32  IDR_frame_search_count; \
  OMX_U32  start_code_with_type; \
  OMX_U32  start_code_header; \
  OMX_U32  start_code_picture; \
  OMX_U32  start_code_picture1; \
  OMX_BOOL isSplittedStartCode; \
  OMX_U32  splittedStartCodeLen; \
  OMX_U32  frame_delimiter_offset; \
  OMX_U32  appendable_header_offset; \
  OMX_BOOL isFromTCCParser; \
  /** @param thumbnail extraction mode flag */ \
  OMX_BOOL bThumbnailMode; \
  /** @param buffer to store a output frame for thumbnail */ \
  OMX_U8*  pThumbnailBuff; \
/*  OMX_BOOL  isVPUClosed;*/ \
  OMX_BOOL  bPlayDirection; \
/*  cdmx_info_t cdmx_info;*/ \
  OMX_S32 displaying_error_count; \
   OMX_U32 ConsecutiveVdecFailCnt; \
  OMX_U32 maxConsecutiveVdecFailCnt; \
  OMX_S32 seq_header_init_error_count; \
  OMX_S32 vpu_preOpen_fd; \
  /** @param to restore decode error*/ \
  OMX_U8* seqHeader_backup; \
  OMX_U32 seqHeader_len; \
  OMX_U8 cntDecError; \
  /** @param frame counting to update frame-rate */ \
  OMX_S32 frameCount; \
  /** @param store previous timestamp to update frame-rate */ \
  OMX_TICKS prevTimestamp; \
  /** @param average duration of one video frame */ \
  OMX_TICKS frameDuration; \
  /** @param frame-rate update on/off */ \
  OMX_BOOL  bUpdateFPS; \
  /** @param key-frame output only mode on/off for skimming */ \
  OMX_BOOL  bDecIndexOutput; \
  OMX_BOOL  bWaitNewBuffer; \
  /** @param drop output frames after sync frame */ \
  OMX_BOOL  bWaitKeyFrameOut; \
  OMX_S32   dropAfterSyncCount; \
  /** @param for ANDROID_USE_GRALLOC_BUFFER */ \
  gralloc_use_t gralloc_info; \
  OMX_CONFIG_RECTTYPE rectParm; \
  OMX_BOOL bOutputMode; \
  OMX_BOOL blocalPlaybackMode; \
  OMX_S32			mTmem_fd; \
  void* 			mTMapInfo; \
  pmap_t			mUmpReservedPmap; \
  _VIDEO_DECOD_INSTANCE_ pVideoDecodInstance; \
  OMX_BOOL gHDMIOutput; \
  OMX_U32 gVideo_FrameRate; \
  OMX_U32 out_index; \
  OMX_U32 in_index; \
  OMX_U32 frm_clear; \
  OMX_U32 buffer_unique_id; \
  OMX_S32 Display_Buff_ID[VPU_BUFF_COUNT*2]; \
  OMX_U32 Display_index[VPU_BUFF_COUNT*2]; \
  OMX_U32 max_fifo_cnt; \
  OMX_U32 used_fifo_count; \
  OMX_S32 g_hFb; \
  OMX_S32 iVsyncMode; \
  void* sequence_header_only; \
  OMX_U32 sequence_header_size; \
  OMX_BOOL isRemotePlayerPlay; \
  OMX_BOOL need_sequence_header_attachment; \
  OMX_U32 ConsecutiveBufferFullCnt; \
  OMX_BOOL bAllPortsFlushed; \
  OMX_U32  mCodecStart_ms;

ENDCLASS(omx_videodec_component_PrivateType)

/* Component private entry points declaration */
OMX_ERRORTYPE omx_videodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videodec_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_videodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videodec_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videodec_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videodec_component_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);

void omx_videodec_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_videodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_videodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_videodec_component_ComponentRoleEnum(
  OMX_IN OMX_HANDLETYPE hComponent,
  OMX_OUT OMX_U8 *cRole,
  OMX_IN OMX_U32 nIndex);

OMX_ERRORTYPE omx_videodec_component_GetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure);

OMX_ERRORTYPE omx_videodec_component_SetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure);

OMX_ERRORTYPE omx_videodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);

#endif
