/**

  @file omx_tp_audiodec_component.h

  This file is header of 3rd-Party audio decoder component.

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

#ifndef _OMX_THIRD_PARTY_DEC_COMPONENT_H_
#define _OMX_THIRD_PARTY_DEC_COMPONENT_H_

#include <omx_audiodec_component.h>
#include "../lib/third_party_predefine.h"

#define OMX_PREFIX		omx_tp

#define OMXCAT1BAR(A, B, C)	OMXCAT2BAR(A, B, C)
#define OMXCAT2BAR(A, B, C)	A ## _ ## B ## _ ## C  

#define OMXCATBAR(func)	OMXCAT1BAR(OMX_PREFIX, CODEC_NAME, func)

#define OMXCAT2TYPE(type)	OMX_AUDIO_PARAM_ ## type ## TYPE
#define OMXCAT2CODING(name)	OMX_AUDIO_Coding ## name
#define OMXCAT2DECNAME(name)	AUDIO_DEC_ ## name ## _NAME

#define OMXCATDECNAME(name) OMXCAT2DECNAME(name)
#define OMXCATCODING(name) OMXCAT2CODING(name)
#define OMXCATTYPE(name) OMXCAT2TYPE(name)

#define component_PrivateType				OMXCATBAR(component_PrivateType)

#define	component_PrivateType_FIELDS		OMXCATBAR(component_PrivateType_FIELDS)
#define audio_param_type					OMXCATTYPE(CODEC_TYPE)
#define audio_dec_name						OMXCATDECNAME(CODEC_TYPE)
#define audio_coding_type					OMXCATCODING(CODEC_TYPE)

DERIVEDCLASS(component_PrivateType, omx_audiodec_component_PrivateType)
#define component_PrivateType_FIELDS omx_audiodec_component_PrivateType_FIELDS \
	audio_param_type pAudio;			/* Do not change */ \
	OMX_U8*		pAudioInputBuffer;		/* Do not change */ \
	OMX_S32		iDataLength;			/* Do not change */ \
	OMX_S32		iAudioInBufferSize;		/* Do not change */ \
	void*		pDecoder;				/* Do not change */
ENDCLASS(component_PrivateType)

#define component_Constructor				OMXCATBAR(component_Constructor)
#define component_Destructor				OMXCATBAR(component_Destructor)
#define	component_BufferMgmtCallback		OMXCATBAR(component_BufferMgmtCallback)

#define MIN_STREM_SIZE	64

/* Component private entry points declaration */
OMX_ERRORTYPE component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
void component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_S32 omx_audiodec_open_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_S32 omx_audiodec_close_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_U8* omx_audiodec_fill_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_S32 *piLength);
OMX_S32 omx_audiodec_update_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp, OMX_S32 iUsedByte);
OMX_S32 omx_audiodec_flush_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp);

#endif /* _OMX_THIRD_PARTY_DEC_COMPONENT_H_ */

