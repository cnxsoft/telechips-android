/**

  @file omx_mp3enc_component.h

  This file is header of MP3 encoder component.

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

#ifndef _OMX_MP3ENC_COMPONENT_H_
#define _OMX_MP3ENC_COMPONENT_H_

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

#include "cdmx.h"
#include "cmux.h"
#include "cdk.h"
#include "cdk_audio.h"

DERIVEDCLASS(omx_mp3enc_component_PrivateType, omx_base_filter_PrivateType)
#define omx_mp3enc_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_MP3TYPE pAudioMp3; \
	OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode; \
	OMX_BUFFERHEADERTYPE *temporary_buffer; \
	OMX_U32 mp3_encode_ready; \
	OMX_U8* temp_input_buffer; \
	OMX_U8* temporary_outbuf;\
	int isNewBuffer; \
	OMX_TICKS nTimeStamp; \
	int audio_encode_state; \
  	OMX_U32 audio_coding_type; \
  	int *Power_Spectrum_info; \
	int *Energy_Volume_level; \
	cdk_core_t cdk_core; \
	cmux_info_t cmux_info; \
	cmux_input_t cmux_input; \
	int iAencType; \
	int iCtype; \
	cdk_callback_func_t callback_func; \
	cdk_func_t* cb_function; \
	int isLibInitDone; \
	AENC_VARS gsAEnc;
ENDCLASS(omx_mp3enc_component_PrivateType)



/* Component private entry points enclaration */
OMX_ERRORTYPE omx_mp3enc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_mp3enc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_mp3enc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_mp3enc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_mp3enc_component_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);

void omx_mp3enc_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_mp3enc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_mp3enc_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);


OMX_ERRORTYPE omx_mp3enc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);


void omx_mp3enc_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp);

/*!
 ***********************************************************************
 * \brief
 *		TCC_MP3_DEC		: main api function of mp3 encoder
 * \param
 *		[in]Op			: decoder operation
 * \param
 *		[in,out]pHandle	: handle of MP3 encoder
 * \param
 *		[in]pParam1		: init or input parameter
 * \param
 *		[in]pParam2		: output or information parameter
 * \return
 *		If successful, TCC_MP3_ENC returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
int TCC_MP3_ENC( int iOpCode, int* pHandle, void* pParam1, void* pParam2 );


#endif /* _OMX_MP3ENC_COMPONENT_H_ */

