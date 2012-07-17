/**

  @file omx_aacenc_component.h

  This file is header of AAC encoder component.

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

#ifndef _OMX_AACENC_COMPONENT_H_
#define _OMX_AACENC_COMPONENT_H_

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

#include "TCCxxx_AACENC.H"

DERIVEDCLASS(omx_aacenc_component_PrivateType, omx_base_filter_PrivateType)
#define omx_aacenc_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	tAACENC *pAACENC; \
	OMX_AUDIO_PARAM_AACPROFILETYPE pAudioAac; \
	OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode; \
	OMX_U32 aac_encode_ready; \
  	OMX_U32 audio_coding_type; \ 
	int		rec_ramainbufCount;\
	int		eof_flag;\
	int		stop_flag;\
	unsigned char	*rec_tmpbuf;\
	unsigned char	*in_tmpbuf;\
	unsigned char	*out_tmpbuf;\
	unsigned short	*psLeftPCM;\
	unsigned short	*psRightPCM;
ENDCLASS(omx_aacenc_component_PrivateType)

//-------------------------------------------------------------------------------------------------------------------


/* Component private entry points declaration */
OMX_ERRORTYPE omx_aacenc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_aacenc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_aacenc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_aacenc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_aacenc_encoder_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);

void omx_aacenc_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);
  
OMX_ERRORTYPE omx_aacenc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_aacenc_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_aacenc_component_SetConfig
	(OMX_IN  OMX_HANDLETYPE hComponent,
	OMX_IN  OMX_INDEXTYPE nIndex,
	OMX_IN  OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_aacenc_component_GetConfig
	(OMX_IN  OMX_HANDLETYPE hComponent,
	OMX_IN  OMX_INDEXTYPE nIndex,
	OMX_IN  OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_aacenc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);


void omx_aacenc_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp);

void* omx_aacenc_BufferMgmtFunction (void* param);

#endif /* _OMX_AACENC_COMPONENT_H_ */




