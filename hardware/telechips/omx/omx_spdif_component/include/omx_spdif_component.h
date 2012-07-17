/**

  @file omx_spdif_component.h

  This file is header of spdif component.

  Copyright (C) 2007-2008  STMicroelectronics
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

*/

#ifndef _OMX_SPDIF_COMPONENT_H_
#define _OMX_SPDIF_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <omx_base_filter.h>
#include "TCCMemory.h"

#include "OMX_TCC_Index.h"

#include "tcc_video_common.h"

#include "spdif_parse.h"

// spdif passthrough class
DERIVEDCLASS(omx_spdif_component_PrivateType, omx_base_filter_PrivateType)
#define omx_spdif_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode; \
	OMX_AUDIO_PARAM_AC3TYPE pAudioAc3; \
	OMX_AUDIO_PARAM_DTSTYPE pAudioDts; \
	OMX_U32 decode_ready; \
	int isNewBuffer; \
  	OMX_U32 audio_coding_type; \
	OMX_TICKS iSamples; \
	OMX_TICKS iStartTS; \
	OMX_U32 iSPDIFMode; \
	OMX_U8* spdif_pBuffer; \
	OMX_S32 spdif_nFilledLength; \
	OMX_S32 spdif_nConsumedLength; \
	spdif_header_info_s spdif_info;
ENDCLASS(omx_spdif_component_PrivateType)

// common functions
void omx_spdif_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer);
OMX_ERRORTYPE omx_spdif_component_LibInit(omx_spdif_component_PrivateType* omx_spdif_component_Private);
OMX_ERRORTYPE omx_spdif_component_LibDeinit(omx_spdif_component_PrivateType* omx_spdif_component_Private);
OMX_ERRORTYPE omx_spdif_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp, internalRequestMessageType *message);
OMX_ERRORTYPE omx_spdif_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);
OMX_ERRORTYPE omx_spdif_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);

OMX_ERRORTYPE omx_spdif_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_spdif_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

#endif /* _OMX_SPDIF_COMPONENT_H_ */

