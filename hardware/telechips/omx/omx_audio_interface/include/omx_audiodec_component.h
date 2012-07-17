/**

  @file omx_audiodec_component.h

  This file is header of audio decoder component.

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

#ifndef _OMX_AUDIODEC_COMPONENT_H_
#define _OMX_AUDIODEC_COMPONENT_H_

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

// audio decoder base class
DERIVEDCLASS(omx_audiodec_component_PrivateType, omx_base_filter_PrivateType)
#define omx_audiodec_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode; \
	OMX_U32 decode_ready; \
	int isNewBuffer; \
  	OMX_U32 audio_coding_type; \
	cdk_core_t cdk_core; \
	cdmx_info_t cdmx_info; \
	cdmx_output_t cdmx_out; \
	int iAdecType; \
	int iCtype; \
	cdk_callback_func_t callback_func; \
	cdk_audio_func_t* cb_function; \
	ADEC_VARS gsADec; \
	OMX_TICKS iSamples; \
	OMX_TICKS iGuardSamples; \
	OMX_U32 iNumOfSeek; \
	OMX_TICKS iStartTS; \
	OMX_TICKS iPrevTS; \
	OMX_TICKS iPrevOriginalTS; \
	OMX_TICKS iNextTS; \
	OMX_BOOL bPrevDecFail; \
	/** for output buffer split*/ \
	OMX_U8* pRest; \
	OMX_U32 iRestSize; \
	OMX_U32 iSplitLength; \
	OMX_U32 iSplitPosition; \
	OMX_BOOL bOutputStarted; \
	OMX_BOOL bBitstreamOut;
ENDCLASS(omx_audiodec_component_PrivateType)

// aac decoder class
DERIVEDCLASS(omx_aacdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_aacdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_AACPROFILETYPE pAudioAac;
ENDCLASS(omx_aacdec_component_PrivateType)

// ac3 decoder class
DERIVEDCLASS(omx_ac3dec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_ac3dec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_AC3TYPE pAudioAc3;
ENDCLASS(omx_ac3dec_component_PrivateType)

// ape decoder class
DERIVEDCLASS(omx_apedec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_apedec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_APETYPE pAudioApe;
ENDCLASS(omx_apedec_component_PrivateType)

// ddp decoder class
DERIVEDCLASS(omx_ddpdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_ddpdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_AC3TYPE pAudioAc3;
ENDCLASS(omx_ddpdec_component_PrivateType)

// dts decoder class
DERIVEDCLASS(omx_dtsdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_dtsdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_DTSTYPE pAudioDts;
ENDCLASS(omx_dtsdec_component_PrivateType)

// flac decoder class
DERIVEDCLASS(omx_flacdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_flacdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_FLACTYPE pAudioFlac;
ENDCLASS(omx_flacdec_component_PrivateType)

// mp2 decoder class
DERIVEDCLASS(omx_mp2dec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_mp2dec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_MP2TYPE pAudioMp2;
ENDCLASS(omx_mp2dec_component_PrivateType)

// mp3 decoder class
DERIVEDCLASS(omx_mp3dec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_mp3dec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_MP3TYPE pAudioMp3; 
ENDCLASS(omx_mp3dec_component_PrivateType)

// real audio decoder class
DERIVEDCLASS(omx_radec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_radec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_RATYPE pAudioRa;
ENDCLASS(omx_radec_component_PrivateType)

// vorbis decoder class
DERIVEDCLASS(omx_vorbisdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_vorbisdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_VORBISTYPE pAudioVorbis;
ENDCLASS(omx_vorbisdec_component_PrivateType)

// wav decoder class
DERIVEDCLASS(omx_wavdec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_wavdec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcm;
ENDCLASS(omx_wavdec_component_PrivateType)

// wma decoder class
DERIVEDCLASS(omx_wmadec_component_PrivateType, omx_audiodec_component_PrivateType)
#define omx_wmadec_component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	OMX_AUDIO_PARAM_WMATYPE pAudioWma;
ENDCLASS(omx_wmadec_component_PrivateType)



// common functions
void AudioInfo_print(cdmx_info_t *info);
OMX_ERRORTYPE omx_audiodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
void omx_audiodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer);
OMX_ERRORTYPE omx_audiodec_component_LibInit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private);
OMX_ERRORTYPE omx_audiodec_component_LibDeinit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private);
OMX_ERRORTYPE omx_audiodec_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp, internalRequestMessageType *message);
OMX_ERRORTYPE omx_audiodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType);
OMX_ERRORTYPE omx_audiodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);

OMX_ERRORTYPE omx_audiodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_audiodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);


void * WRAPPER_malloc(unsigned int size);
void * WRAPPER_calloc(unsigned int size, unsigned int count);
void WRAPPER_free(void * ptr);
void * WRAPPER_realloc(void * ptr, unsigned int size);
void * WRAPPER_memcpy(void* dest, const void* src, unsigned int size);
void WRAPPER_memset(void* ptr, int val, unsigned int size);
void * WRAPPER_memmove(void* dest, const void* src, unsigned int size);

#endif /* _OMX_AUDIODEC_COMPONENT_H_ */

