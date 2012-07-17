/**
  @file src/components/ffmpeg/omx_videoenc_component.h
  
  This component implements an H.264 / MPEG-4 AVC video decoder. 
  The H.264 / MPEG-4 AVC Video encoder is based on the FFmpeg software library.

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

  Author $Author: ZzaU $
*/

#ifndef _OMX_MPEG4ENC_COMPONENT_H_
#define _OMX_MPEG4ENC_COMPONENT_H_

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
#include <venc.h>

/** Video Encoder component private structure.
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
DERIVEDCLASS(omx_videoenc_component_PrivateType, omx_base_filter_PrivateType)
#define omx_videoenc_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	/** @param semaphore for avcodec access syncrhonization */\
	tsem_t* avCodecSyncSem; \
	/** @param pVideoMpeg4 Referece to OMX_VIDEO_PARAM_MPEG4TYPE structure*/  \
	OMX_VIDEO_PARAM_MPEG4TYPE pVideoMpeg4;  \
	/** @param pVideoAvc Referece to OMX_VIDEO_PARAM_AVCTYPE structure*/  \
	OMX_VIDEO_PARAM_AVCTYPE pVideoAvc;  \
	/** @param pVideoH263 Referece to OMX_VIDEO_PARAM_H263TYPE structure*/  \
	OMX_VIDEO_PARAM_H263TYPE pVideoH263;  \
	/** @param avcodecReady boolean flag that is true when the video coded has been initialized */ \
	OMX_BOOL avcodecReady;  \
	/** @param minBufferLength Field that stores the minimun allowed size for FFmpeg decoder */ \
	OMX_U16 minBufferLength; \
	/** @param inputCurrBuffer Field that stores pointer of the current input buffer position */ \
	OMX_U8* inputCurrBuffer;\
	/** @param inputCurrLength Field that stores current input buffer length in bytes */ \
	OMX_U32 inputCurrLength;\
	/** @param isFirstBuffer Field that the buffer is the first buffer */ \
	OMX_S32 isFirstBuffer;\
	/** @param isNewBuffer Field that indicate a new buffer has arrived*/ \
	OMX_S32 isNewBuffer;  \
	/** @param video_coding_type Field that indicate the supported video format of video decoder */ \
	OMX_U32 video_coding_type;   \
	/** @param eOutFramePixFmt Field that indicate output frame pixel format */ \
	PIXEL_FORMAT_E eOutFramePixFmt; \
	/** @param extradata pointer to extradata*/ \
	OMX_U8* extradata; \
	/** @param extradata_size extradata size*/ \
	OMX_U32 extradata_size; \
	OMX_BOOL iConfigDataFlag;  \
    /*VIDEO ENCODER SPECIFIC PARAMETERS*/  \
	OMX_VIDEO_PARAM_PROFILELEVELTYPE 		sProfileLevel;  \
	OMX_CONFIG_ROTATIONTYPE					VideoOrientationType;  \
	OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE 	VideoErrorCorrection;  \
	OMX_VIDEO_PARAM_BITRATETYPE				VideoRateType;  \
	OMX_VIDEO_PARAM_QUANTIZATIONTYPE		VideoQuantType;  \
	OMX_VIDEO_PARAM_VBSMCTYPE				VideoBlockMotionSize;  \
	OMX_VIDEO_PARAM_MOTIONVECTORTYPE		VideoMotionVector;	  \
	OMX_VIDEO_PARAM_INTRAREFRESHTYPE		VideoIntraRefresh;  \
	OMX_VIDEO_PARAM_AVCSLICEFMO				AvcSliceFMO;  \
	OMX_CONFIG_FRAMERATETYPE				VideoConfigFrameRateType;  \
	OMX_VIDEO_CONFIG_BITRATETYPE			VideoConfigBitRateType;  \
	OMX_CONFIG_INTRAREFRESHVOPTYPE			VideoIFrame; \
	venc_init_t			gsVEncInit; \
	venc_seq_header_t	gsVEncSeqHeader; \
	venc_input_t		gsVEncInput; \
	venc_output_t		gsVEncOutput; \
	OMX_BOOL  isVPUClosed; \
	OMX_BOOL  isEncError;
ENDCLASS(omx_videoenc_component_PrivateType)

/* Component private entry points declaration */
OMX_ERRORTYPE omx_videoenc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videoenc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_videoenc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videoenc_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videoenc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_videoenc_component_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);

void omx_videoenc_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_videoenc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_videoenc_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_videoenc_component_ComponentRoleEnum(
  OMX_IN OMX_HANDLETYPE hComponent,
  OMX_OUT OMX_U8 *cRole,
  OMX_IN OMX_U32 nIndex);

OMX_ERRORTYPE omx_videoenc_component_SetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure);

OMX_ERRORTYPE omx_videoenc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);

#endif
