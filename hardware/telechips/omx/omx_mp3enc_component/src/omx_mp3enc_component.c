/**

  @file omx_mp3enc_component.c

  This component implement MP3 encoder.

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

#include <omxcore.h>
#include <omx_base_audio_port.h>

#include <omx_mp3enc_component.h>
#include <tccaudio.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 1
#define LOG_TAG	"OMX_TCC_MP3ENC"
#include <utils/Log.h>

#define MP3_ENC_IN_BUFFER_SIZE 			256 * 1024
#define MP3_ENC_OUT_BUFFER_SIZE 		64 * 1024
#define MP3_ENC_MAX_NUM_OF_IN_BUFFERS 	5
#define MP3_ENC_MAX_NUM_OF_OUT_BUFFERS 	2


static int DEBUG_ON	= 0;
static int DEBUG2_ON	= 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);}
#define DBUG_MSG2(msg...)	if (DEBUG2_ON) { LOGD( ": " msg);}
#endif /* HAVE_ANDROID_OS */

#define modify_buffer_hdh

void omx_mp3enc_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer)
{
	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;

	OMX_S32 ret     		= 0;
	OMX_U8* input_ptr = inputbuffer->pBuffer;

	if (omx_mp3enc_component_Private->isLibInitDone == 0)
	{
		return;
	}

	outputbuffer->nFilledLen = 0;
	outputbuffer->nOffset = 0;

	DBUG_MSG("[DBG_MP3_ENC]  ==> BufferMgmtCallback IN inLen = %ld, Flags = 0x%08x, Offset(%d)", inputbuffer->nFilledLen, inputbuffer->nFlags, inputbuffer->nOffset);


#ifdef modify_buffer_hdh
	if(omx_mp3enc_component_Private->mp3_encode_ready == OMX_FALSE)
	{
		DBUG_MSG("[DBG_MP3_ENC]  ==> Audio Enc init start. AudioInfo Size = %d,", sizeof(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo) );
		memset(&omx_mp3enc_component_Private->gsAEnc, 0, sizeof(AENC_VARS));
		memcpy(&(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo), (void*)inputbuffer->pBuffer, sizeof(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo));
		if( cdk_aenc_init(&omx_mp3enc_component_Private->cdk_core,
							&omx_mp3enc_component_Private->cmux_info,
							omx_mp3enc_component_Private->iAencType, 		 // AUDIO_ID_xxx
							omx_mp3enc_component_Private->iCtype,			 // CONTAINER_TYPE_xxx
							omx_mp3enc_component_Private->cb_function,
							&omx_mp3enc_component_Private->gsAEnc) < 0 ) // TCC_xxx_ENC
		{
			LOGE("Audio Dec init error.");
			return;
		}

		omx_mp3enc_component_Private->mp3_encode_ready  = OMX_TRUE;
	}

	if(omx_mp3enc_component_Private->isNewBuffer == 1)
	{
		DBUG_MSG2("omx_mp3enc_component_Private->isNewBuffer");
	
		memmove (omx_mp3enc_component_Private->temp_input_buffer, omx_mp3enc_component_Private->temporary_buffer->pBuffer, omx_mp3enc_component_Private->temporary_buffer->nFilledLen);
		omx_mp3enc_component_Private->temporary_buffer->pBuffer = omx_mp3enc_component_Private->temp_input_buffer;
		memcpy(omx_mp3enc_component_Private->temporary_buffer->pBuffer+omx_mp3enc_component_Private->temporary_buffer->nFilledLen, inputbuffer->pBuffer, inputbuffer->nFilledLen);
		omx_mp3enc_component_Private->temporary_buffer->nFilledLen += inputbuffer->nFilledLen;

		DBUG_MSG2("nFilledLen(%d), inputbuffer->nFilledLen(%d)", omx_mp3enc_component_Private->temporary_buffer->nFilledLen, inputbuffer->nFilledLen);
		omx_mp3enc_component_Private->nTimeStamp = inputbuffer->nTimeStamp;
	}
	else{
		outputbuffer->nTimeStamp = omx_mp3enc_component_Private->nTimeStamp;
	}

	if (omx_mp3enc_component_Private->temporary_buffer->nFilledLen < 4608)
	{
		DBUG_MSG2("11. omx_mp3enc_component_Private->temporary_buffer->nFilledLen(%d) < 4608", omx_mp3enc_component_Private->temporary_buffer->nFilledLen);
		inputbuffer->nFilledLen = 0;
		omx_mp3enc_component_Private->isNewBuffer = 1;
	}
	else
	{

		while (omx_mp3enc_component_Private->temporary_buffer->nFilledLen >= 4608)
		{
			/* Encode the block */
			omx_mp3enc_component_Private->cdk_core.m_pOutWav 	 = omx_mp3enc_component_Private->temporary_outbuf;

			omx_mp3enc_component_Private->cmux_input.m_pData	 = omx_mp3enc_component_Private->temporary_buffer->pBuffer;
			omx_mp3enc_component_Private->cmux_input.m_iDataSize = 4608;

			ret = cdk_aenc_encode(&omx_mp3enc_component_Private->cdk_core,    //output buffer
								  &omx_mp3enc_component_Private->cmux_info,
								  &omx_mp3enc_component_Private->cmux_input,  //input buffer
								   omx_mp3enc_component_Private->iAencType,
								  &omx_mp3enc_component_Private->gsAEnc);

			omx_mp3enc_component_Private->temporary_buffer->pBuffer += omx_mp3enc_component_Private->cmux_input.m_iDataSize;
			omx_mp3enc_component_Private->temporary_buffer->nFilledLen -= omx_mp3enc_component_Private->cmux_input.m_iDataSize;

//			LOGD( "m_iDataSize : %ld nFilledLen : %ld", omx_mp3enc_component_Private->cmux_input.m_iDataSize, omx_mp3enc_component_Private->temporary_buffer->nFilledLen);

			if (ret >= 0)
			{
				memcpy(outputbuffer->pBuffer+outputbuffer->nFilledLen, omx_mp3enc_component_Private->gsAEnc.gsAEncOutput.m_pcStream, omx_mp3enc_component_Private->gsAEnc.gsAEncOutput.m_iStreamLength);
				outputbuffer->nFilledLen += omx_mp3enc_component_Private->gsAEnc.gsAEncOutput.m_iStreamLength;   // encoded stream length			
//				outputbuffer->pBuffer    = omx_mp3enc_component_Private->cdk_core.m_pOutWav; // encoded data buffer
			}
			else
			{
				LOGE( "cdk_audio_dec_run error: %ld", ret );
			}
		}
		
		if(omx_mp3enc_component_Private->temporary_buffer->nFilledLen < 4608)
		{
//			LOGE("22. omx_mp3enc_component_Private->temporary_buffer->nFilledLen < 4608");
			inputbuffer->nFilledLen = 0;
			omx_mp3enc_component_Private->isNewBuffer = 1;
		}
		else
		{
//			LOGD("22. inputbuffer->nFilledLen = 1");
			inputbuffer->nFilledLen = 1;
			omx_mp3enc_component_Private->isNewBuffer = 0;
		}
	}
#else
//	if((inputbuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) && omx_mp3enc_component_Private->mp3_encode_ready == OMX_FALSE)
	if(omx_mp3enc_component_Private->mp3_encode_ready == OMX_FALSE)
	{
		DBUG_MSG("[DBG_MP3_ENC]  ==> Audio Enc init start. AudioInfo Size = %d,", sizeof(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo) );
		memcpy(&(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo), (void*)inputbuffer->pBuffer, sizeof(omx_mp3enc_component_Private->cmux_info.m_sAudioInfo));

		if( cdk_aenc_init(&omx_mp3enc_component_Private->cdk_core,
							&omx_mp3enc_component_Private->cmux_info,
							omx_mp3enc_component_Private->iAencType, 		 // AUDIO_ID_xxx
							omx_mp3enc_component_Private->iCtype,			 // CONTAINER_TYPE_xxx
							omx_mp3enc_component_Private->cb_function) < 0 ) // TCC_xxx_ENC
		{
			LOGE("Audio Dec init error.");
			return;
		}


		omx_mp3enc_component_Private->mp3_encode_ready  = OMX_TRUE;
#if 0
		omx_mp3enc_component_Private->isNewBuffer = 1;
		outputbuffer->nFilledLen = 0;
		inputbuffer->nFilledLen = 0;
		DBUG_MSG("[DBG_MP3_ENC]  ==>  MP3 ENC initialized.");

		return;
#endif
	}

	if(omx_mp3enc_component_Private->mp3_encode_ready == OMX_FALSE)
	{
		LOGE(" MP3 Encoder not Initialized!!");
		return;
	}


	/* Encode the block */
	omx_mp3enc_component_Private->cdk_core.m_pOutWav 	 = outputbuffer->pBuffer;

	omx_mp3enc_component_Private->cmux_input.m_pData	 = inputbuffer->pBuffer;
	omx_mp3enc_component_Private->cmux_input.m_iDataSize = inputbuffer->nFilledLen;

#if 0
	{
		FILE *dump_fp = NULL;
		dump_fp= fopen("/sdcard/dump.pcm", "a+");
		fwrite(inputbuffer->pBuffer, inputbuffer->nFilledLen, 1, dump_fp);
		fclose(dump_fp);
	}
#endif
#if 0
	{
		char *p= inputbuffer->pBuffer;
		LOGD(" Length (%d)", inputbuffer->nFilledLen);
		LOGD("+++ %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
				*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++,*p++ );
	}
#endif

	ret = cdk_aenc_encode(&omx_mp3enc_component_Private->cdk_core,    //output buffer
						  &omx_mp3enc_component_Private->cmux_info,
						  &omx_mp3enc_component_Private->cmux_input,  //input buffer
						   omx_mp3enc_component_Private->iAencType);

	if (ret >= 0)
	{
		outputbuffer->nFilledLen = omx_mp3enc_component_Private->cdk_core.m_pSize;   // encoded stream length
		outputbuffer->pBuffer    = omx_mp3enc_component_Private->cdk_core.m_pOutWav; // encoded data buffer

	}
	else
	{
		LOGE( "cdk_audio_dec_run error: %ld", ret );
	}

	omx_mp3enc_component_Private->isNewBuffer = 1;
	inputbuffer->nFilledLen = 0;
#endif
}


/** this function sets the parameter values regarding audio format & index */
OMX_ERRORTYPE omx_mp3enc_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
	OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
	OMX_AUDIO_PARAM_MP3TYPE* pAudioMp3;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_AUDIO_CONFIG_SEEKTYPE* gettime;
	OMX_AUDIO_CONFIG_INFOTYPE *info;

	OMX_U32 portIndex;
	OMX_U32 nFileNameLength;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;
	omx_base_audio_PortType *port;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	DBUG_MSG("[DBG_MP3_ENC]  ==>    Setting parameter 0x%x\n", nParamIndex);

	switch(nParamIndex)
	{
		case OMX_IndexParamAudioPortFormat:
			pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			portIndex = pAudioPortFormat->nPortIndex;
			/*Check Structure Header and verify component state*/
			err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
			if(err!=OMX_ErrorNone)
			{
				DBUG_MSG("[DBG_MP3_ENC]  ==> In %s Parameter Check Error=%x\n",__func__,err);
				break;
			}
			if (portIndex <= 1)
			{
				port = (omx_base_audio_PortType *) omx_mp3enc_component_Private->ports[portIndex];
				memcpy(&port->sAudioParam, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
			}
			else
			{
				err = OMX_ErrorBadPortIndex;
			}
		break;

		case OMX_IndexParamAudioPcm:
			pAudioPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
			portIndex = pAudioPcmMode->nPortIndex;
			/*Check Structure Header and verify component state*/
			err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			if(err!=OMX_ErrorNone)
			{
				DBUG_MSG("[DBG_MP3_ENC]  ==> In %s Parameter Check Error=%x\n",__func__,err);
				break;
			}
			memcpy(&omx_mp3enc_component_Private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));

			omx_mp3enc_component_Private->cdk_core.m_iAudioSamplePerSec  = omx_mp3enc_component_Private->pAudioPcmMode.nSamplingRate;
			omx_mp3enc_component_Private->cdk_core.m_iAudioChannels		 = omx_mp3enc_component_Private->pAudioPcmMode.nChannels; 
			omx_mp3enc_component_Private->cdk_core.m_iAudioBitsPerSample = omx_mp3enc_component_Private->pAudioPcmMode.nBitPerSample; 

			LOGI("@@@@ :: nChannels = %d, nBitPerSample = %d, nSamplingRate = %d",
					omx_mp3enc_component_Private->pAudioPcmMode.nChannels, 
					omx_mp3enc_component_Private->pAudioPcmMode.nBitPerSample, 
					omx_mp3enc_component_Private->pAudioPcmMode.nSamplingRate);
		break;

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (!strcmp( (char*) pComponentRole->cRole, AUDIO_ENC_MP3_ROLE))
			{
				omx_mp3enc_component_Private->audio_coding_type = OMX_AUDIO_CodingMP3;
			}
			else
			{
				err = OMX_ErrorBadParameter;
			}
		break;

		case OMX_IndexParamAudioMp3:
			pAudioMp3= (OMX_AUDIO_PARAM_MP3TYPE*) ComponentParameterStructure;
			portIndex = pAudioMp3->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioMp3,sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			if(err!=OMX_ErrorNone)
			{
				DBUG_MSG("[DBG_MP3_ENC]  ==> In %s Parameter Check Error=%x\n",__func__,err);
				break;
			}
			if (pAudioMp3->nPortIndex == 1)
			{
				memcpy(&omx_mp3enc_component_Private->pAudioMp3, pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			}
			else
			{
				err = OMX_ErrorBadPortIndex;
			}
		break;



		case OMX_IndexVendorAudioExtraData:
		{
			OMX_VENDOR_EXTRADATATYPE* pExtradata;
			pExtradata = (OMX_VENDOR_EXTRADATATYPE*)ComponentParameterStructure;

			if (pExtradata->nPortIndex <= 1) {
				/** copy the extradata in the codec context private structure */
//_				memcpy(&omx_mp3enc_component_Private->audioinfo, pExtradata->pData, sizeof(ac3_audio_info));
			} else {
				err = OMX_ErrorBadPortIndex;
			}
		}
		break;

		case OMX_IndexVendorParamMediaInfo:
			info = (OMX_AUDIO_CONFIG_INFOTYPE*) ComponentParameterStructure;
			//omx_mp3enc_component_Private->pAudioMp3.nChannels = info->nChannels;
			//omx_mp3enc_component_Private->pAudioMp3.nBitRate = info->nBitPerSample;
			//omx_mp3enc_component_Private->pAudioMp3.nSampleRate = info->nSamplingRate;
			break;


		default: /*Call the base component function*/
			return omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	if(err != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, err);

	return err;
}





/** this function gets the parameters regarding audio formats and index */
OMX_ERRORTYPE omx_mp3enc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
	OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_AUDIO_PARAM_MP3TYPE *pAudioMp3;
	OMX_AUDIO_CONFIG_GETTIMETYPE *gettime;
    OMX_AUDIO_CONFIG_INFOTYPE *info;
	omx_base_audio_PortType *port;
	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_AUDIO_SPECTRUM_INFOTYPE *power;
	OMX_AUDIO_ENERGY_VOLUME_INFOTYPE *energyvolume;
	int i;

	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}
	DBUG_MSG("[DBG_MP3_ENC]  ==>    Getting parameter 0x%x\n", nParamIndex);
	/* Check which structure we are being fed and fill its header */

	switch(nParamIndex)
	{
		case OMX_IndexParamAudioInit:
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone)
			{
			  break;
			}
			memcpy(ComponentParameterStructure, &omx_mp3enc_component_Private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
			break;

		case OMX_IndexParamAudioPortFormat:
			pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone)
			{
				break;
			}
			if (pAudioPortFormat->nPortIndex <= 1)
			{
				port = (omx_base_audio_PortType *)omx_mp3enc_component_Private->ports[pAudioPortFormat->nPortIndex];
				memcpy(pAudioPortFormat, &port->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
			}
			else
			{
				return OMX_ErrorBadPortIndex;
			}
			break;

		case OMX_IndexParamAudioPcm:
			pAudioPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE))) != OMX_ErrorNone)
			{
				break;
			}
			if (pAudioPcmMode->nPortIndex > 1)
			{
				return OMX_ErrorBadPortIndex;
			}
			memcpy(pAudioPcmMode, &omx_mp3enc_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			break;

		case OMX_IndexParamAudioMp3:
			pAudioMp3 = (OMX_AUDIO_PARAM_MP3TYPE*)ComponentParameterStructure;
			if (pAudioMp3->nPortIndex != 1)//0)
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP3TYPE))) != OMX_ErrorNone)
			{
				break;
			}
			memcpy(pAudioMp3, &omx_mp3enc_component_Private->pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			break;

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone)
			{
				break;
			}
			if (omx_mp3enc_component_Private->audio_coding_type == OMX_AUDIO_CodingMP3)
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_ENC_MP3_ROLE);
			}
			else
			{
				strcpy( (char*) pComponentRole->cRole,"\0");;
			}
			break;

#ifdef HAVE_ANDROID_OS
			case PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX:
			{
				PV_OMXComponentCapabilityFlagsType *pCap_flags =
					(PV_OMXComponentCapabilityFlagsType *) ComponentParameterStructure;
				if (NULL == pCap_flags) {
					return OMX_ErrorBadParameter;
				}

				memset(pCap_flags, 0x00, sizeof(PV_OMXComponentCapabilityFlagsType));
				pCap_flags->iIsOMXComponentMultiThreaded = OMX_TRUE;
				pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc = OMX_TRUE;
				pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc = OMX_TRUE;
				pCap_flags->iOMXComponentSupportsMovableInputBuffers = OMX_TRUE;
				pCap_flags->iOMXComponentSupportsPartialFrames = OMX_TRUE;
//				pCap_flags->iOMXComponentUsesNALStartCodes = OMX_FALSE;
//				pCap_flags->iOMXComponentCanHandleIncompleteFrames = OMX_TRUE;
//				pCap_flags->iOMXComponentUsesFullAVCFrames = OMX_FALSE;
			}
			break;
#endif

		default: /*Call the base component function*/
			return omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	return OMX_ErrorNone;

}



OMX_ERRORTYPE omx_mp3enc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName)
{

	OMX_ERRORTYPE err = OMX_ErrorNone;
	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private;
	omx_base_audio_PortType *inPort,*outPort;
	OMX_U32 i;

#ifdef HAVE_ANDROID_OS
	if (1)
#else
	if (!openmaxStandComp->pComponentPrivate)
#endif
	{
		openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_mp3enc_component_PrivateType));

		if(openmaxStandComp->pComponentPrivate==NULL)
		{
			return OMX_ErrorInsufficientResources;
		}
	}
	else
	{
		DBUG_MSG("[DBG_MP3_ENC]  ==> In %s, Error Component %x Already Allocated\n",
				__func__, (int)openmaxStandComp->pComponentPrivate);
	}

	omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;
	omx_mp3enc_component_Private->ports = NULL;

	/** we could create our own port structures here
	 * fixme maybe the base class could use a "port factory" function pointer?
	 */
	err = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

	DBUG_MSG("[DBG_MP3_ENC]  ==> constructor of mp3 encoder component is called\n");

	/* Domain specific section for the ports. */
	/* first we set the parameter common to both formats */
	/* parameters related to input port which does not depend upon input audio format    */
	/* Allocate Ports and call port constructor. */

	omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
	omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

	if (omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts && !omx_mp3enc_component_Private->ports)
	{
		omx_mp3enc_component_Private->ports = TCC_calloc(omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
		if (!omx_mp3enc_component_Private->ports)
		{
			return OMX_ErrorInsufficientResources;
		}
		for (i=0; i < omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++)
		{
			omx_mp3enc_component_Private->ports[i] = TCC_calloc(1, sizeof(omx_base_audio_PortType));
			if (!omx_mp3enc_component_Private->ports[i])
			{
				return OMX_ErrorInsufficientResources;
			}
		}
	}

	base_audio_port_Constructor(openmaxStandComp, &omx_mp3enc_component_Private->ports[0], 0, OMX_TRUE); // input
	base_audio_port_Constructor(openmaxStandComp, &omx_mp3enc_component_Private->ports[1], 1, OMX_FALSE); // output


	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) omx_mp3enc_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];

#if 0 // test
    inPort->sPortParam.nBufferCountActual = 1152 * 2 * 2;
    inPort->sPortParam.nBufferCountMin = 1;
    inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;
    inPort->sPortParam.bEnabled = OMX_TRUE;
    inPort->sPortParam.bPopulated = OMX_FALSE;

	inPort->sPortParam.format.audio.cMIMEType = (OMX_STRING)"raw";
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;
#else

//    inPort->sPortParam.nBufferCountActual = 1152 * 2 * 2;

	inPort->sPortParam.nBufferSize = MP3_ENC_IN_BUFFER_SIZE;
    inPort->sPortParam.nBufferCountActual = MP3_ENC_MAX_NUM_OF_IN_BUFFERS;
    inPort->sPortParam.nBufferCountMin = 1;
	strcpy(inPort->sPortParam.format.audio.cMIMEType, "raw");
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;
#endif


	/** parameters related to output port */
	outPort = (omx_base_audio_PortType *) omx_mp3enc_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	strcpy(outPort->sPortParam.format.audio.cMIMEType, "audio/mpeg");
	outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingMP3;
	outPort->sPortParam.nBufferSize = MP3_ENC_OUT_BUFFER_SIZE;
    outPort->sPortParam.nBufferCountActual = MP3_ENC_MAX_NUM_OF_OUT_BUFFERS;
    outPort->sPortParam.nBufferCountMin = 1;
	outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingMP3;

	setHeader(&omx_mp3enc_component_Private->pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
    omx_mp3enc_component_Private->pAudioMp3.nPortIndex = 1;
    omx_mp3enc_component_Private->pAudioMp3.nChannels = 1; //2;
    omx_mp3enc_component_Private->pAudioMp3.nBitRate =  0;
    omx_mp3enc_component_Private->pAudioMp3.nSampleRate = 8000; // 44100;
    omx_mp3enc_component_Private->pAudioMp3.eChannelMode = OMX_AUDIO_ChannelModeStereo;
    omx_mp3enc_component_Private->pAudioMp3.nAudioBandWidth = 0;

	/** settings of output port audio format - pcm */
	setHeader(&omx_mp3enc_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	omx_mp3enc_component_Private->pAudioPcmMode.nPortIndex = 0;
	omx_mp3enc_component_Private->pAudioPcmMode.nChannels = 2; // 1;:
	omx_mp3enc_component_Private->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
	omx_mp3enc_component_Private->pAudioPcmMode.eEndian = OMX_EndianLittle;
	omx_mp3enc_component_Private->pAudioPcmMode.bInterleaved = OMX_TRUE;
	omx_mp3enc_component_Private->pAudioPcmMode.nBitPerSample = 16;
	omx_mp3enc_component_Private->pAudioPcmMode.nSamplingRate = 8000; //44100;
	omx_mp3enc_component_Private->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
	omx_mp3enc_component_Private->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
	omx_mp3enc_component_Private->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

	/** now it's time to know the audio coding type of the component */
	if(!strcmp(cComponentName, AUDIO_ENC_MP3_NAME))
	{
		omx_mp3enc_component_Private->audio_coding_type = OMX_AUDIO_CodingMP3;
	}
	else if (!strcmp(cComponentName, AUDIO_ENC_BASE_NAME))
	{
		omx_mp3enc_component_Private->audio_coding_type = OMX_AUDIO_CodingUnused;
	}
	else
	{
		// IL client specified an invalid component name
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
		return OMX_ErrorInvalidComponentName;
	}


	/** general configuration irrespective of any audio formats */
	/**  setting values of other fields of omx_maddec_component_Private structure */

	omx_mp3enc_component_Private->BufferMgmtCallback = omx_mp3enc_component_BufferMgmtCallback;
	omx_mp3enc_component_Private->messageHandler = omx_mp3enc_component_MessageHandler;
	omx_mp3enc_component_Private->destructor = omx_mp3enc_component_Destructor;
	openmaxStandComp->SetParameter = omx_mp3enc_component_SetParameter;
	openmaxStandComp->GetParameter = omx_mp3enc_component_GetParameter;
	openmaxStandComp->GetExtensionIndex = omx_mp3enc_component_GetExtensionIndex;


	omx_mp3enc_component_Private->mp3_encode_ready = OMX_FALSE;

	memset(&omx_mp3enc_component_Private->cdk_core, 0x00, sizeof(cdk_core_t));
	memset(&omx_mp3enc_component_Private->cmux_info, 0x00, sizeof(cmux_info_t));
	memset(&omx_mp3enc_component_Private->cmux_input, 0x00, sizeof(cmux_input_t));

	omx_mp3enc_component_Private->cdk_core.m_iAudioProcessMode = 2; /* encoded pcm mode */

	omx_mp3enc_component_Private->cdk_core.m_psCallback = &(omx_mp3enc_component_Private->callback_func);
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfMalloc   = (void* (*) ( unsigned int ))malloc;
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfRealloc  = (void* (*) ( void*, unsigned int ))realloc;
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfFree	  = (void  (*) ( void* ))free;
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfMemcpy   = (void* (*) ( void*, const void*, unsigned int ))memcpy;
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfMemmove  = (void* (*) ( void*, const void*, unsigned int ))memmove;
	omx_mp3enc_component_Private->cdk_core.m_psCallback->m_pfMemset   = (void  (*) ( void*, int, unsigned int ))memset;


	// setting
	omx_mp3enc_component_Private->iAencType 	= 1; //AUDIO_ID_MP3;
	omx_mp3enc_component_Private->cb_function 	= TCC_MP3_ENC;

	omx_mp3enc_component_Private->cdk_core.m_iAudioBitRates		 = 256000;
#if 0
	omx_mp3enc_component_Private->cdk_core.m_iAudioSamplePerSec  = 44100;
	omx_mp3enc_component_Private->cdk_core.m_iAudioChannels		 = 2;
	omx_mp3enc_component_Private->cdk_core.m_iAudioBitsPerSample = 16;
#endif

	omx_mp3enc_component_Private->isLibInitDone = 0;

	DBUG_MSG("[DBG_MP3_ENC]  ==> constructor of mp3 encoder component is completed ret = %d \n", err);
	return err;

}


/** The destructor */
OMX_ERRORTYPE omx_mp3enc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp)
{

	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

#if 0
	if(omx_mp3enc_component_Private->tccmp3DecSyncSem)
	{
		tsem_deinit(omx_mp3enc_component_Private->tccraDecSyncSem);
		TCC_free(omx_mp3enc_component_Private->tccraDecSyncSem);
		omx_mp3enc_component_Private->tccraDecSyncSem = NULL;
	}
#endif

	/* frees port/s */
	if (omx_mp3enc_component_Private->ports)
	{
		for (i=0; i < omx_mp3enc_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++)
		{
			if(omx_mp3enc_component_Private->ports[i])
				omx_mp3enc_component_Private->ports[i]->PortDestructor(omx_mp3enc_component_Private->ports[i]);
		}

		TCC_free(omx_mp3enc_component_Private->ports);
		omx_mp3enc_component_Private->ports=NULL;
	}

	DBUG_MSG("[DBG_MP3_ENC]  ==> Destructor of MP3 encoder component is called\n");

	omx_mp3enc_component_Private->audio_encode_state = AUDIO_MAX_EVENT;

	//omx_mp3enc_component_Private->audio_valid_info = 0;

	omx_base_filter_Destructor(openmaxStandComp);

	return OMX_ErrorNone;

}

//stMP3EncodeInstance stRAInstance;
//unsigned char RealSDK_Instance_ac3_encode[100*1024];
OMX_ERRORTYPE omx_mp3enc_component_LibInit(omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private)
{
	/*
	* Get the codec 4CC of substream 0. We
	* arbitrarily choose substream 0 here.
	*/

#ifdef modify_buffer_hdh
	/** initializing omx_maddec_component_Private->temporary_buffer with 2k memory space*/
	omx_mp3enc_component_Private->temporary_buffer = TCC_malloc(sizeof(OMX_BUFFERHEADERTYPE));
	omx_mp3enc_component_Private->temporary_buffer->pBuffer = TCC_malloc(MP3_ENC_IN_BUFFER_SIZE*2);
	omx_mp3enc_component_Private->temporary_outbuf = TCC_malloc(MP3_ENC_OUT_BUFFER_SIZE);
	memset(omx_mp3enc_component_Private->temporary_buffer->pBuffer, 0, MP3_ENC_IN_BUFFER_SIZE*2);
	memset(omx_mp3enc_component_Private->temporary_outbuf, 0, MP3_ENC_OUT_BUFFER_SIZE);
	
	omx_mp3enc_component_Private->temp_input_buffer = omx_mp3enc_component_Private->temporary_buffer->pBuffer;
	omx_mp3enc_component_Private->temporary_buffer->nFilledLen=0;
	omx_mp3enc_component_Private->temporary_buffer->nOffset=0;
#endif
	omx_mp3enc_component_Private->isNewBuffer = 1;
	omx_mp3enc_component_Private->isLibInitDone = 1;

	return OMX_ErrorNone;;
}

OMX_ERRORTYPE omx_mp3enc_component_LibDeinit(omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private)
{

	omx_mp3enc_component_Private->mp3_encode_ready = OMX_FALSE;
	omx_mp3enc_component_Private->isLibInitDone = 0;

#ifdef modify_buffer_hdh
	omx_mp3enc_component_Private->temporary_buffer->pBuffer = omx_mp3enc_component_Private->temp_input_buffer;

	/* freeing temporary memory allocation */
	if(omx_mp3enc_component_Private->temporary_buffer->pBuffer) {
		TCC_free(omx_mp3enc_component_Private->temporary_buffer->pBuffer);
		omx_mp3enc_component_Private->temporary_buffer->pBuffer = NULL;
	}
	if(omx_mp3enc_component_Private->temporary_buffer) {
		TCC_free(omx_mp3enc_component_Private->temporary_buffer);
		omx_mp3enc_component_Private->temporary_buffer = NULL;
	}
	if(omx_mp3enc_component_Private->temporary_outbuf)
	{
		TCC_free(omx_mp3enc_component_Private->temporary_outbuf);
		omx_mp3enc_component_Private->temporary_outbuf = NULL;
	}
#endif

	if( cdk_aenc_close(&omx_mp3enc_component_Private->cdk_core,&omx_mp3enc_component_Private->gsAEnc) < 0 )
	{
		LOGE("[DBG_MP3_ENC]  ==> Audio Dec close error\n");
		return OMX_ErrorHardware;
	}

	return OMX_ErrorNone;
}

/** The Initialization function  */
OMX_ERRORTYPE omx_mp3enc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp)
{

	return omx_mp3enc_component_Constructor(openmaxStandComp,AUDIO_ENC_MP3_NAME);
}

/** The Deinitialization function  */
OMX_ERRORTYPE omx_mp3enc_component_DeInit(OMX_COMPONENTTYPE *openmaxStandComp)
{

	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = openmaxStandComp->pComponentPrivate;

	return omx_mp3enc_component_Destructor(openmaxStandComp);
}


OMX_ERRORTYPE omx_mp3enc_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp, internalRequestMessageType *message)
{

	omx_mp3enc_component_PrivateType* omx_mp3enc_component_Private = (omx_mp3enc_component_PrivateType*)openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE err;
	OMX_STATETYPE eCurrentState = omx_mp3enc_component_Private->state;
	DBUG_MSG("[DBG_MP3_ENC]  ==> Start %s\n", __func__);

	/** Execute the base message handling */
	err = omx_base_component_MessageHandler(openmaxStandComp, message);
	DBUG_MSG("[DBG_MP3_ENC]  ==> %d, %s (err = %d)", __LINE__, __func__, err);

	if (message->messageType == OMX_CommandStateSet){
		if ((message->messageParam == OMX_StateExecuting) && (eCurrentState == OMX_StateIdle)) {
			err = omx_mp3enc_component_LibInit(omx_mp3enc_component_Private);
			if(err!=OMX_ErrorNone) {
				LOGE("In %s mp3 encoder Init Failed Error=%x\n",__func__,err);
				return err;
			}
		} else if ((message->messageParam == OMX_StateIdle) && (eCurrentState == OMX_StateExecuting || eCurrentState == OMX_StatePause)) {
			err = omx_mp3enc_component_LibDeinit(omx_mp3enc_component_Private);
			if(err!=OMX_ErrorNone) {
				LOGE("In %s mp3 encoder Deinit Failed Error=%x\n",__func__,err);
				return err;
			}
		}
	}

	return err;

}


OMX_ERRORTYPE omx_mp3enc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType)
{

	DBUG_MSG("[DBG_MP3_ENC]  ==> In  %s \n",__func__);
	if(strcmp(cParameterName,TCC_AUDIO_FILE_OPEN_STRING) == 0) {
		*pIndexType = OMX_IndexVendorParamFileOpen;
	} else if(strcmp(cParameterName,TCC_AUDIO_POWERSPECTUM_STRING) == 0){
		*pIndexType = OMX_IndexVendorConfigPowerSpectrum;
	} else if(strcmp(cParameterName,TCC_AUDIO_MEDIA_INFO_STRING) == 0){
		*pIndexType = OMX_IndexVendorParamMediaInfo;
	}else if(strcmp(cParameterName,TCC_AUDIO_ENERGYVOLUME_STRING) == 0){
		*pIndexType = OMX_IndexVendorConfigEnergyVolume;
	}else{
		return OMX_ErrorBadParameter;
	}

  return OMX_ErrorNone;
}


#ifdef HAVE_ANDROID_OS
OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
  OMX_ERRORTYPE err = OMX_ErrorNone;

	err = omx_mp3enc_component_Constructor(openmaxStandComp,cCompontName);

	return err;
}
#endif

