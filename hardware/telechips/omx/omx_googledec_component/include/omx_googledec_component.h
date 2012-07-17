/**
  @file src/components/ffmpeg/omx_videodec_component.h
  
  This component implements Google Video decoder. (Google codec)
  The VPX Video decoder is based on the google software library.

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

  Android revised by ZzaU.
*/

#ifndef _OMX_GOOGLEDEC_COMPONENT_H_
#define _OMX_GOOGLEDEC_COMPONENT_H_

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


typedef struct BitmapParams 
{
    void *mBits;
    OMX_U32 mWidth; 
	OMX_U32 mHeight;
    OMX_U32 mCropLeft; 
	OMX_U32 mCropTop;
    OMX_U32 mCropRight; 
	OMX_U32 mCropBottom;
}BitmapParams;

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
  /** @param avcodecReady boolean flag that is true when the video coded has been initialized */ \
  OMX_BOOL avcodecReady; \
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
  OMX_BOOL avcodecInited; \
  /** @param for ANDROID_USE_GRALLOC_BUFFER */ \
  gralloc_use_t gralloc_info; \
  OMX_CONFIG_RECTTYPE rectParm; \
  OMX_PTR mCtx; \
  OMX_U32 avgTime; \
  OMX_U8* mClip;

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
