/**

  @file omx_audiodec_component.c

  This component implement audio decoder's common part.

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

#include <omx_audiodec_component.h>
#include <tccaudio.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_AUDIODEC"
#include <utils/Log.h>
#include <cutils/properties.h>

//#define DEBUG_ON
#ifdef DEBUG_ON
#define DBUG_MSG(x...)	LOGD(x)
#else
#define DBUG_MSG(x...)
#endif

//#define MEM_DEBUG
#ifdef MEM_DEBUG
#define LOGMSG(x...) LOGD(x)
#else
#define LOGMSG(x...)
#endif

// for APE -------------------------------------------
#define CODEC_SPECIFIC_MARKER 0x12345678
#define CODEC_SPECIFIC_MARKER_OFFSET 8
#define CODEC_SPECIFIC_INFO_OFFSET 4

typedef struct ape_dmx_exinput_t
{
	unsigned int m_uiInputBufferByteOffset;
	unsigned int m_uiCurrentFrames;
} ape_dmx_exinput_t;
// for APE -------------------------------------------

#define OUTPUT_SPLIT_TIME_LIMIT 50000 // 50ms
#define OUTPUT_SILENT_TIME_LIMIT 1000000 // 1sec

#endif /* HAVE_ANDROID_OS */

// --------------------------------------------------
// memory management functions
// --------------------------------------------------
void * WRAPPER_malloc(unsigned int size)
{
	LOGMSG("malloc size %d", size);
	void * ptr = malloc(size);
	LOGMSG("malloc ptr %x", ptr);
	return ptr;
}

void * WRAPPER_calloc(unsigned int size, unsigned int count)
{
	LOGMSG("calloc size %d, count %d", size, count);
	void * ptr = calloc(size, count);
	LOGMSG("calloc ptr %x", ptr);
	return ptr;
}
void WRAPPER_free(void * ptr)
{
	LOGMSG("free %x", ptr);
	free(ptr);
	LOGMSG("free out");
}

void * WRAPPER_realloc(void * ptr, unsigned int size)
{
	LOGMSG("realloc ptr %x, size %d", ptr, size);
	return realloc(ptr, size);
}

void * WRAPPER_memcpy(void* dest, const void* src, unsigned int size)
{
	LOGMSG("memcpy dest ptr %x, src ptr %x, size %d", dest, src, size);
	return memcpy(dest, src, size);
}

void WRAPPER_memset(void* ptr, int val, unsigned int size)
{
	LOGMSG("memset ptr %x, val %d, size %d", ptr, val, size);
	memset(ptr, val, size);
}

void * WRAPPER_memmove(void* dest, const void* src, unsigned int size)
{
	LOGMSG("memmove dest %x, src %x, size %d", dest, src, size);
	return memmove(dest, src, size);
}

// ---------------------------------------------------------
// ---------------------------------------------------------

void AudioInfo_print(cdmx_info_t *info)
{
	DBUG_MSG("Audio Info Start=====================================>");
	
	DBUG_MSG("common!!");
	DBUG_MSG("m_iTotalNumber = %d", info->m_sAudioInfo.m_iTotalNumber);
	DBUG_MSG("m_iSamplePerSec = %d", info->m_sAudioInfo.m_iSamplePerSec);
	DBUG_MSG("m_iBitsPerSample = %d", info->m_sAudioInfo.m_iBitsPerSample);
	DBUG_MSG("m_iChannels = %d", info->m_sAudioInfo.m_iChannels);
	DBUG_MSG("m_iFormatId = %d", info->m_sAudioInfo.m_iFormatId);
	DBUG_MSG("--------------------------------------------------------------------");
	
	DBUG_MSG("extra info (common)!!");
	DBUG_MSG("m_pExtraData = 0x%x", info->m_sAudioInfo.m_pExtraData);
	DBUG_MSG("m_iExtraDataLen = %d", info->m_sAudioInfo.m_iExtraDataLen);
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("mp4 info!!");
	DBUG_MSG("m_iFramesPerSample = %d", info->m_sAudioInfo.m_iFramesPerSample);
	DBUG_MSG("m_iTrackTimeScale = %d", info->m_sAudioInfo.m_iTrackTimeScale);
	DBUG_MSG("m_iSamplesPerPacket = 0x%x", info->m_sAudioInfo.m_iSamplesPerPacket);
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("AVI info!!");
	DBUG_MSG("m_iNumAudioStream = %d", info->m_sAudioInfo.m_iNumAudioStream);
	DBUG_MSG("m_iCurrAudioIdx = %d", info->m_sAudioInfo.m_iCurrAudioIdx);
	DBUG_MSG("m_iBlockAlign = %d", info->m_sAudioInfo.m_iBlockAlign);
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("ASF info skip!!");
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("MPG info!!");
	DBUG_MSG("m_iBitRate = %d", info->m_sAudioInfo.m_iBitRate);
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("RM info!!");
	DBUG_MSG("ulActualRate = %d", info->m_sAudioInfo.m_iActualRate);
	DBUG_MSG("usAudioQuality = %d", info->m_sAudioInfo.m_sAudioQuality);
	DBUG_MSG("usFlavorIndex = %d", info->m_sAudioInfo.m_sFlavorIndex);
	DBUG_MSG("ulBitsPerFrame = %d", info->m_sAudioInfo.m_iBitsPerFrame);
	DBUG_MSG("ulGranularity = %d", info->m_sAudioInfo.m_iGranularity);
	DBUG_MSG("ulOpaqueDataSize = %d", info->m_sAudioInfo.m_iOpaqueDataSize);
	DBUG_MSG("pOpaqueData = 0x%x", info->m_sAudioInfo.m_pOpaqueData);
	DBUG_MSG("ulSamplesPerFrame = %d", info->m_sAudioInfo.m_iSamplesPerFrame);
	DBUG_MSG("frameSizeInBits = %d", info->m_sAudioInfo.m_iframeSizeInBits);
	DBUG_MSG("ulSampleRate = %d", info->m_sAudioInfo.m_iSampleRate);
	DBUG_MSG("cplStart = %d", info->m_sAudioInfo.m_scplStart);
	DBUG_MSG("cplQBits = %d", info->m_sAudioInfo.m_scplQBits);
	DBUG_MSG("nRegions = %d", info->m_sAudioInfo.m_snRegions);
	DBUG_MSG("dummy = %d", info->m_sAudioInfo.m_sdummy);
	DBUG_MSG("m_iFormatId = %d", info->m_sAudioInfo.m_iFormatId);
	DBUG_MSG("--------------------------------------------------------------------");

	DBUG_MSG("for Audio Only Path!!");
	DBUG_MSG("m_iMinDataSize = %d", info->m_sAudioInfo.m_iMinDataSize);
	DBUG_MSG("Audio Info End =====================================>");
}

void PrintHexData(OMX_U8* p)
{
	LOGD("------------------------------");
	LOGD("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
	LOGD("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", p[0+16], p[1+16], p[2+16], p[3+16], p[4+16], p[5+16], p[6+16], p[7+16], p[8+16], p[9+16], p[10+16], p[11+16], p[12+16], p[13+16], p[14+16], p[15+16]);
	LOGD("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", p[0+32], p[1+32], p[2+32], p[3+32], p[4+32], p[5+32], p[6+32], p[7+32], p[8+32], p[9+32], p[10+32], p[11+32], p[12+32], p[13+32], p[14+32], p[15+32]);
	LOGD("------------------------------");
}

OMX_ERRORTYPE omx_audiodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp)
{
	omx_audiodec_component_PrivateType* omx_audiodec_component_Private;

	omx_audiodec_component_Private = (omx_audiodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;

	char value[PROPERTY_VALUE_MAX];
	property_get("persist.sys.spdif_setting", value, "");
	if (!strcmp(value, "3"))
	{
		omx_audiodec_component_Private->bBitstreamOut = OMX_TRUE;
	}
	else
	{
		omx_audiodec_component_Private->bBitstreamOut = OMX_FALSE;
	}

	return OMX_ErrorNone;;
}
	
// library init function
OMX_ERRORTYPE omx_audiodec_component_LibInit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private) 
{
	DBUG_MSG("In %s\n", __func__);
	omx_audiodec_component_Private->isNewBuffer = 1;
	omx_audiodec_component_Private->bPrevDecFail = OMX_FALSE;
	omx_audiodec_component_Private->iNumOfSeek = 0;
	omx_audiodec_component_Private->bOutputStarted = OMX_FALSE;
	omx_audiodec_component_Private->iPrevTS = -1;
	omx_audiodec_component_Private->iSamples = 0;
	omx_audiodec_component_Private->iNextTS = 0;
	omx_audiodec_component_Private->iPrevOriginalTS = -1;

	return OMX_ErrorNone;;
}

// library de-init function
OMX_ERRORTYPE omx_audiodec_component_LibDeinit(omx_audiodec_component_PrivateType* omx_audiodec_component_Private) 
{
	DBUG_MSG("In %s\n", __func__);
	omx_audiodec_component_Private->decode_ready = OMX_FALSE;

	if( cdk_adec_close(&omx_audiodec_component_Private->cdk_core, &omx_audiodec_component_Private->gsADec) < 0 )
	{
		DBUG_MSG("Audio DEC close error\n");
		return OMX_ErrorHardware; 	  
	}

	return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_audiodec_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp, internalRequestMessageType *message)  
{
	omx_audiodec_component_PrivateType* omx_audiodec_component_Private = (omx_audiodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;  
	OMX_ERRORTYPE err;
	OMX_STATETYPE eCurrentState = omx_audiodec_component_Private->state;
	DBUG_MSG("In %s\n", __func__);

	/** Execute the base message handling */
	err = omx_base_component_MessageHandler(openmaxStandComp, message);

	if (message->messageType == OMX_CommandStateSet){ 
		if ((message->messageParam == OMX_StateExecuting) && (eCurrentState == OMX_StateIdle)) {
			err = omx_audiodec_component_LibInit(omx_audiodec_component_Private);
			if(err!=OMX_ErrorNone) { 
				LOGE("In %s Audio decoder library Init Failed Error=%x\n",__func__,err); 
				return err;
			}
		} else if ((message->messageParam == OMX_StateIdle) && (eCurrentState == OMX_StateExecuting || eCurrentState == OMX_StatePause)) {
			err = omx_audiodec_component_LibDeinit(omx_audiodec_component_Private);
			if(err!=OMX_ErrorNone) { 
				LOGE("In %s Audio decoder library Deinit Failed Error=%x\n",__func__,err); 
				return err;
			}
		}
	}
	return err;  

}

OMX_ERRORTYPE omx_audiodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType)
{    

	DBUG_MSG("In  %s \n",__func__);
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

/** The destructor */
OMX_ERRORTYPE omx_audiodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) 
{

	omx_audiodec_component_PrivateType* omx_audiodec_component_Private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

	if (omx_audiodec_component_Private->pRest != NULL)
	{
		free(omx_audiodec_component_Private->pRest);
		omx_audiodec_component_Private->pRest = NULL;
	}

	/* frees port/s */
	if (omx_audiodec_component_Private->ports) 
	{
		for (i=0; i < omx_audiodec_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) 
		{
			if(omx_audiodec_component_Private->ports[i])
				omx_audiodec_component_Private->ports[i]->PortDestructor(omx_audiodec_component_Private->ports[i]);
		}

		TCC_free(omx_audiodec_component_Private->ports);
		omx_audiodec_component_Private->ports=NULL;
	}

	DBUG_MSG("Destructor of the Audio decoder component is called\n");

	omx_base_filter_Destructor(openmaxStandComp);

	return OMX_ErrorNone;

}

/** this function sets the parameter values regarding audio format & index */
OMX_ERRORTYPE omx_audiodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
	OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_AUDIO_CONFIG_SEEKTYPE* gettime;
	OMX_AUDIO_CONFIG_INFOTYPE *info;

	omx_base_audio_PortType *port;
	OMX_U32 portIndex;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_audiodec_component_PrivateType* omx_audiodec_component_Private = openmaxStandComp->pComponentPrivate;
	omx_aacdec_component_PrivateType* AACPrivate;
	omx_ac3dec_component_PrivateType* AC3Private;
	omx_apedec_component_PrivateType* APEPrivate;
	omx_dtsdec_component_PrivateType* DTSPrivate;
	omx_flacdec_component_PrivateType* FLACPrivate;
	omx_mp2dec_component_PrivateType* MP2Private;
	omx_mp3dec_component_PrivateType* MP3Private;
	omx_radec_component_PrivateType* RAPrivate;
	omx_vorbisdec_component_PrivateType* VORBISPrivate;
	omx_wmadec_component_PrivateType* WMAPrivate;
	omx_wavdec_component_PrivateType* WAVPrivate;

	OMX_AUDIO_PARAM_AACPROFILETYPE* pAudioAac;
	OMX_AUDIO_PARAM_AC3TYPE * pAudioAc3;
	OMX_AUDIO_PARAM_APETYPE * pAudioApe;
	OMX_AUDIO_PARAM_DTSTYPE * pAudioDts;
	OMX_AUDIO_PARAM_FLACTYPE * pAudioFlac;
	OMX_AUDIO_PARAM_MP2TYPE * pAudioMp2;
	OMX_AUDIO_PARAM_MP3TYPE * pAudioMp3;
	OMX_AUDIO_PARAM_RATYPE * pAudioRa;
	OMX_AUDIO_PARAM_VORBISTYPE * pAudioVorbis;
	OMX_AUDIO_PARAM_WMATYPE * pAudioWma;
	
	if (ComponentParameterStructure == NULL) 
	{
		return OMX_ErrorBadParameter;
	}

	DBUG_MSG("   Setting parameter 0x%x\n", nParamIndex);

	switch(nParamIndex) 
	{
		case OMX_IndexParamAudioPortFormat:
			pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			portIndex = pAudioPortFormat->nPortIndex;
			/*Check Structure Header and verify component state*/
			err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (portIndex <= 1) 
			{
				port = (omx_base_audio_PortType *) omx_audiodec_component_Private->ports[portIndex];
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
			
			if (pAudioPcmMode->nSamplingRate > 352800)
			{
				LOGE("TCC Android doesn't support over 352.8kHz sampling rate!");
				err = OMX_ErrorUnsupportedSetting;
				break;
			}

			/*Check Structure Header and verify component state*/
			err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}

			if (pAudioPcmMode->nPortIndex == 0) // input type is PCM
			{
				WAVPrivate = (omx_wavdec_component_PrivateType *)omx_audiodec_component_Private;
				memcpy(&WAVPrivate->pAudioPcm, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
				WAVPrivate->pAudioPcmMode.nSamplingRate = pAudioPcmMode->nSamplingRate;
				if (omx_audiodec_component_Private->bBitstreamOut == OMX_TRUE) {
					WAVPrivate->pAudioPcmMode.nChannels = pAudioPcmMode->nChannels;
				} else {
					if (pAudioPcmMode->nChannels == 1) {
						WAVPrivate->pAudioPcmMode.nChannels = 1;
					}
				}
			}  
			else // output type is PCM
			{
				LOGI("@@@@ :: nChannels = %d, nBitPerSample = %d, nSamplingRate = %d", pAudioPcmMode->nChannels, pAudioPcmMode->nBitPerSample, pAudioPcmMode->nSamplingRate);
				memcpy(&omx_audiodec_component_Private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
			}
		break;

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_TCC_AAC_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingAAC;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_AC3_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingAC3;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_APE_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingAPE;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_DTS_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingDTS;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_FLAC_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingFLAC;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_MP2_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingMP2;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_MP3_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingMP3;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_RA_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingRA;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_VORBIS_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingVORBIS;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_PCM_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingPCM;
			}  
			else if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_TCC_WMA_ROLE)) 
			{
				omx_audiodec_component_Private->audio_coding_type = OMX_AUDIO_CodingWMA;
			}  
			else
			{
				err = OMX_ErrorBadParameter;
			}
		break;
		
		// --------------------------------------------------------------------------------
		// codec specific parameters -------------------------------------------------------

		case OMX_IndexParamAudioAac:
			pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*) ComponentParameterStructure;
			portIndex = pAudioAac->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioAac,sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			
			AACPrivate = (omx_aacdec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioAac->nPortIndex == 0) 
			{
				memcpy(&AACPrivate->pAudioAac, pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				if (pAudioAac->nSampleRate < 32000) {
					mode->nSamplingRate = pAudioAac->nSampleRate * 2;
				} else {
					mode->nSamplingRate = pAudioAac->nSampleRate;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioAC3:
			pAudioAc3 = (OMX_AUDIO_PARAM_AC3TYPE*) ComponentParameterStructure;
			portIndex = pAudioAc3->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioAc3,sizeof(OMX_AUDIO_PARAM_AC3TYPE));
	
			AC3Private = (omx_ac3dec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("[DBG_AC3]  ==> In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioAc3->nPortIndex == 0) 
			{
				memcpy(&AC3Private->pAudioAc3, pAudioAc3, sizeof(OMX_AUDIO_PARAM_AC3TYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioAc3->nSamplingRate;
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioAPE:
			pAudioApe = (OMX_AUDIO_PARAM_APETYPE*) ComponentParameterStructure;
			portIndex = pAudioApe->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioApe,sizeof(OMX_AUDIO_PARAM_APETYPE));
			
			APEPrivate = (omx_apedec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioApe->nPortIndex == 0) 
			{
				memcpy(&APEPrivate->pAudioApe, pAudioApe, sizeof(OMX_AUDIO_PARAM_APETYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioApe->nSampleRate;
				if (pAudioApe->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;

		case OMX_IndexParamAudioDTS:
			pAudioDts = (OMX_AUDIO_PARAM_DTSTYPE*) ComponentParameterStructure;
			portIndex = pAudioDts->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioDts,sizeof(OMX_AUDIO_PARAM_DTSTYPE));
			
			DTSPrivate = (omx_dtsdec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioDts->nPortIndex == 0) 
			{
				memcpy(&DTSPrivate->pAudioDts, pAudioDts, sizeof(OMX_AUDIO_PARAM_DTSTYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioDts->nSamplingRate;
				if (pAudioDts->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioFLAC:
			pAudioFlac = (OMX_AUDIO_PARAM_FLACTYPE*) ComponentParameterStructure;
			portIndex = pAudioFlac->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioFlac,sizeof(OMX_AUDIO_PARAM_FLACTYPE));
			
			FLACPrivate = (omx_flacdec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioFlac->nPortIndex == 0) 
			{
				memcpy(&FLACPrivate->pAudioFlac, pAudioFlac, sizeof(OMX_AUDIO_PARAM_FLACTYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioFlac->nSampleRate;
				if (pAudioFlac->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioMp2:
			pAudioMp2 = (OMX_AUDIO_PARAM_MP2TYPE*) ComponentParameterStructure;
			portIndex = pAudioMp2->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioMp2,sizeof(OMX_AUDIO_PARAM_MP2TYPE));
			
			MP2Private = (omx_mp2dec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioMp2->nPortIndex == 0) 
			{
				memcpy(&MP2Private->pAudioMp2, pAudioMp2, sizeof(OMX_AUDIO_PARAM_MP2TYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioMp2->nSampleRate;
				if (pAudioMp2->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;

		case OMX_IndexParamAudioMp3:
			pAudioMp3 = (OMX_AUDIO_PARAM_MP3TYPE*) ComponentParameterStructure;
			portIndex = pAudioMp3->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioMp3,sizeof(OMX_AUDIO_PARAM_MP3TYPE));

			MP3Private = (omx_mp3dec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioMp3->nPortIndex == 0) 
			{
				memcpy(&MP3Private->pAudioMp3, pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioMp3->nSampleRate;
				if (pAudioMp3->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioRa:
			pAudioRa = (OMX_AUDIO_PARAM_RATYPE*) ComponentParameterStructure;
			portIndex = pAudioRa->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioRa,sizeof(OMX_AUDIO_PARAM_RATYPE));
			
			RAPrivate = (omx_radec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioRa->nPortIndex == 0) 
			{
				memcpy(&RAPrivate->pAudioRa, pAudioRa, sizeof(OMX_AUDIO_PARAM_RATYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioRa->nSamplingRate;
				if (pAudioRa->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioVorbis:
			pAudioVorbis = (OMX_AUDIO_PARAM_VORBISTYPE*) ComponentParameterStructure;
			portIndex = pAudioVorbis->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioVorbis,sizeof(OMX_AUDIO_PARAM_VORBISTYPE));
			
			VORBISPrivate = (omx_vorbisdec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioVorbis->nPortIndex == 0) 
			{
				memcpy(&VORBISPrivate->pAudioVorbis, pAudioVorbis, sizeof(OMX_AUDIO_PARAM_VORBISTYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioVorbis->nSampleRate;
				if (pAudioVorbis->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;
		
		case OMX_IndexParamAudioWma:
			pAudioWma = (OMX_AUDIO_PARAM_WMATYPE*) ComponentParameterStructure;
			portIndex = pAudioWma->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioWma,sizeof(OMX_AUDIO_PARAM_WMATYPE));
			
			WMAPrivate = (omx_wmadec_component_PrivateType *)omx_audiodec_component_Private;
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioWma->nPortIndex == 0) 
			{
				memcpy(&WMAPrivate->pAudioWma, pAudioWma, sizeof(OMX_AUDIO_PARAM_WMATYPE));
				OMX_AUDIO_PARAM_PCMMODETYPE *mode = &omx_audiodec_component_Private->pAudioPcmMode;
				mode->nSamplingRate = pAudioWma->nSamplingRate;

                if(pAudioWma->nSamplingRate > 48000)
                    mode->nSamplingRate = pAudioWma->nSamplingRate/2;
                else
                    mode->nSamplingRate = pAudioWma->nSamplingRate;

				if (pAudioWma->nChannels == 1) {
					mode->nChannels = 1;
				}
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;

		// codec specific ---------------------------------------------------------------
		// ------------------------------------------------------------------------------

		case OMX_IndexVendorAudioExtraData:
		{
			OMX_VENDOR_EXTRADATATYPE* pExtradata;	
			pExtradata = (OMX_VENDOR_EXTRADATATYPE*)ComponentParameterStructure;
			
			if (pExtradata->nPortIndex <= 1) {
				/** copy the extradata in the codec context private structure */
////				memcpy(&omx_mp3dec_component_Private->audioinfo, pExtradata->pData, sizeof(rm_audio_info));
			} else {
				err = OMX_ErrorBadPortIndex;
			}
		}
		break;

		case OMX_IndexVendorParamMediaInfo:
			info = (OMX_AUDIO_CONFIG_INFOTYPE*) ComponentParameterStructure;
			//omx_mp3dec_component_Private->pAudioMp3.nChannels = info->nChannels;
			//omx_mp3dec_component_Private->pAudioMp3.nBitRate = info->nBitPerSample;
			//omx_mp3dec_component_Private->pAudioMp3.nSampleRate = info->nSamplingRate;
			break;
		

		default: /*Call the base component function*/
			return omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	if(err != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, err);

	return err;
}  

/** this function gets the parameters regarding audio formats and index */
OMX_ERRORTYPE omx_audiodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;  
	OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_AUDIO_CONFIG_GETTIMETYPE *gettime;
    OMX_AUDIO_CONFIG_INFOTYPE *info;
	omx_base_audio_PortType *port;
	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_AUDIO_SPECTRUM_INFOTYPE *power;
	OMX_AUDIO_ENERGY_VOLUME_INFOTYPE *energyvolume;
	int i;

	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_audiodec_component_PrivateType* omx_audiodec_component_Private = openmaxStandComp->pComponentPrivate;
	omx_aacdec_component_PrivateType* AACPrivate;
	omx_ac3dec_component_PrivateType* AC3Private;
	omx_apedec_component_PrivateType* APEPrivate;
	omx_dtsdec_component_PrivateType* DTSPrivate;
	omx_flacdec_component_PrivateType* FLACPrivate;
	omx_mp2dec_component_PrivateType* MP2Private;
	omx_mp3dec_component_PrivateType* MP3Private;
	omx_radec_component_PrivateType* RAPrivate;
	omx_vorbisdec_component_PrivateType* VORBISPrivate;
	omx_wmadec_component_PrivateType* WMAPrivate;
	omx_wavdec_component_PrivateType* WAVPrivate;

	OMX_AUDIO_PARAM_AACPROFILETYPE* pAudioAac;
	OMX_AUDIO_PARAM_AC3TYPE * pAudioAc3;
	OMX_AUDIO_PARAM_APETYPE * pAudioApe;
	OMX_AUDIO_PARAM_DTSTYPE * pAudioDts;
	OMX_AUDIO_PARAM_FLACTYPE * pAudioFlac;
	OMX_AUDIO_PARAM_MP2TYPE * pAudioMp2;
	OMX_AUDIO_PARAM_MP3TYPE * pAudioMp3;
	OMX_AUDIO_PARAM_RATYPE * pAudioRa;
	OMX_AUDIO_PARAM_VORBISTYPE * pAudioVorbis;
	OMX_AUDIO_PARAM_WMATYPE * pAudioWma;

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}
	DBUG_MSG("   Getting parameter 0x%x\n", nParamIndex);
	/* Check which structure we are being fed and fill its header */

	switch(nParamIndex) 
	{
		case OMX_IndexParamAudioInit:
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone) 
			{ 
			  break;
			}
			memcpy(ComponentParameterStructure, &omx_audiodec_component_Private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
			break;    

		case OMX_IndexParamAudioPortFormat:
			pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			if (pAudioPortFormat->nPortIndex <= 1) 
			{
				port = (omx_base_audio_PortType *)omx_audiodec_component_Private->ports[pAudioPortFormat->nPortIndex];
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
			
			if (pAudioPcmMode->nPortIndex == 0) // input is PCM
			{
				WAVPrivate = (omx_wavdec_component_PrivateType *)omx_audiodec_component_Private;
				memcpy(pAudioPcmMode, &WAVPrivate->pAudioPcm, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}  
			else // output is PCM
			{
				memcpy(pAudioPcmMode, &omx_audiodec_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			}
			break;
		
		// --------------------------------------------------------------------------------
		// codec specific parameters -------------------------------------------------------

		case OMX_IndexParamAudioAac:
			pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*)ComponentParameterStructure;
			AACPrivate = (omx_aacdec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioAac->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioAac, &AACPrivate->pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			break;
		
		case OMX_IndexParamAudioAC3: // parameter read(channel,samplerate etc...)
			pAudioAc3 = (OMX_AUDIO_PARAM_AC3TYPE*)ComponentParameterStructure;
			AC3Private = (omx_ac3dec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioAc3->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AC3TYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioAc3, &AC3Private->pAudioAc3, sizeof(OMX_AUDIO_PARAM_AC3TYPE));
			break;
		
		case OMX_IndexParamAudioAPE:
			pAudioApe = (OMX_AUDIO_PARAM_APETYPE*)ComponentParameterStructure;
			APEPrivate = (omx_apedec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioApe->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_APETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioApe, &APEPrivate->pAudioApe, sizeof(OMX_AUDIO_PARAM_APETYPE));
			break;

		case OMX_IndexParamAudioDTS:
			pAudioDts = (OMX_AUDIO_PARAM_DTSTYPE*)ComponentParameterStructure;
			DTSPrivate = (omx_dtsdec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioDts->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_DTSTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioDts, &DTSPrivate->pAudioDts, sizeof(OMX_AUDIO_PARAM_DTSTYPE));
			break;
		
		case OMX_IndexParamAudioFLAC:
			pAudioFlac = (OMX_AUDIO_PARAM_FLACTYPE*)ComponentParameterStructure;
			FLACPrivate = (omx_flacdec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioFlac->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_FLACTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioFlac, &FLACPrivate->pAudioFlac, sizeof(OMX_AUDIO_PARAM_FLACTYPE));
			break;
		
		case OMX_IndexParamAudioMp2:
			pAudioMp2 = (OMX_AUDIO_PARAM_MP2TYPE*)ComponentParameterStructure;
			MP2Private = (omx_mp2dec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioMp2->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP2TYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioMp2, &MP2Private->pAudioMp2, sizeof(OMX_AUDIO_PARAM_MP2TYPE));
			break;

		case OMX_IndexParamAudioMp3:
			pAudioMp3 = (OMX_AUDIO_PARAM_MP3TYPE*)ComponentParameterStructure;
			MP3Private = (omx_mp3dec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioMp3->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP3TYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioMp3, &MP3Private->pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
			break;
		
		case OMX_IndexParamAudioRa: // parameter read(channel,samplerate etc...)
			pAudioRa = (OMX_AUDIO_PARAM_RATYPE*)ComponentParameterStructure;
			RAPrivate = (omx_radec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioRa->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_RATYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioRa, &RAPrivate->pAudioRa, sizeof(OMX_AUDIO_PARAM_RATYPE));
			break;
		
		case OMX_IndexParamAudioVorbis:
			pAudioVorbis = (OMX_AUDIO_PARAM_VORBISTYPE*)ComponentParameterStructure;
			VORBISPrivate = (omx_vorbisdec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioVorbis->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_VORBISTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioVorbis, &VORBISPrivate->pAudioVorbis, sizeof(OMX_AUDIO_PARAM_VORBISTYPE));
			break;
		
		case OMX_IndexParamAudioWma:
			pAudioWma = (OMX_AUDIO_PARAM_WMATYPE*)ComponentParameterStructure;
			WMAPrivate = (omx_wmadec_component_PrivateType *)omx_audiodec_component_Private;
			if (pAudioWma->nPortIndex != 0) 
			{
				return OMX_ErrorBadPortIndex;
			}
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_WMATYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			memcpy(pAudioWma, &WMAPrivate->pAudioWma, sizeof(OMX_AUDIO_PARAM_WMATYPE));
			break;
		
		// codec specific ---------------------------------------------------------------
		// ------------------------------------------------------------------------------

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			
			if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingAAC) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_TCC_AAC_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingAC3) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_AC3_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingAPE) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_APE_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingDTS) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_DTS_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingFLAC) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_FLAC_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingMP2) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_MP2_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingMP3) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_MP3_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingRA) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_RA_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingVORBIS) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_VORBIS_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingPCM) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_PCM_ROLE);
			}
			else if (omx_audiodec_component_Private->audio_coding_type == OMX_AUDIO_CodingWMA) 
			{
				strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_TCC_WMA_ROLE);
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
				pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc 	= OMX_TRUE; //ZzaU::to use MovableInputBuffers
				pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc 	= OMX_FALSE;
				pCap_flags->iOMXComponentSupportsMovableInputBuffers		= OMX_TRUE;
				pCap_flags->iOMXComponentSupportsPartialFrames				= OMX_TRUE; //ZzaU::PartialFrames must enable to use MovableInputBuffers.
//				pCap_flags->iOMXComponentUsesNALStartCodes					= OMX_FALSE;
//				pCap_flags->iOMXComponentUsesFullAVCFrames					= OMX_FALSE;			
//				pCap_flags->iOMXComponentCanHandleIncompleteFrames			= OMX_FALSE;
			}
			break;
#endif

		default: /*Call the base component function*/
			return omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);

	}

	return OMX_ErrorNone;

}

void omx_audiodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer)
{
	omx_audiodec_component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  

	OMX_S32 ret	= 0;
	OMX_U8* input_ptr = inputbuffer->pBuffer;
	ape_dmx_exinput_t *codecSpecific = 0;
	OMX_S16 iSeekFlag = 0;
	
	outputbuffer->nFilledLen = 0;
	outputbuffer->nOffset = 0;

	DBUG_MSG("BufferMgmtCallback IN inLen = %u, Flags = 0x%x, Timestamp = %lld", inputbuffer->nFilledLen, inputbuffer->nFlags, inputbuffer->nTimeStamp);
	if((inputbuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) && pPrivate->decode_ready == OMX_FALSE)
	{
		DBUG_MSG("Audio DEC init start. AudioInfo Size = %d", sizeof(pPrivate->cdmx_info.m_sAudioInfo));
		memcpy(&(pPrivate->cdmx_info.m_sAudioInfo), (void*)inputbuffer->pBuffer, sizeof(pPrivate->cdmx_info.m_sAudioInfo));
		AudioInfo_print(&(pPrivate->cdmx_info));
		memset(&pPrivate->gsADec, 0, sizeof(ADEC_VARS));

		pPrivate->iCtype = (int)(*(inputbuffer->pBuffer + sizeof(pPrivate->cdmx_info.m_sAudioInfo)));
		pPrivate->iGuardSamples = pPrivate->pAudioPcmMode.nSamplingRate >> 3; // number of guard samples are corresponding to 125ms
		OMX_U64 temp_time;
		temp_time = (OMX_U64)OUTPUT_SPLIT_TIME_LIMIT * pPrivate->pAudioPcmMode.nSamplingRate - (pPrivate->pAudioPcmMode.nSamplingRate >> 1);
		pPrivate->iSplitLength = temp_time / 1000000;

		DBUG_MSG("Container type %d, num of guard samples %d, split length %d", pPrivate->iCtype, pPrivate->iGuardSamples, pPrivate->iSplitLength);
		pPrivate->cdk_core.m_pOutWav = outputbuffer->pBuffer;

		if (pPrivate->iAdecType == AUDIO_ID_WAV && pPrivate->bBitstreamOut == OMX_TRUE)
		{
			pPrivate->gsADec.gsADecInit.m_iDownMixMode = -1;
		}

		if(pPrivate->iAdecType == AUDIO_ID_DDP)
		{
		
			char value[PROPERTY_VALUE_MAX];
			property_get("tcc.audio.ddp.drc_high", value, "100");
			int drc_value;
			drc_value = atoi(value);
			
			if(drc_value >=0 && drc_value <=100 ){
				pPrivate->gsADec.gsADecInit.m_unAudioCodecParams.m_unDDPDec.m_iDynScaleHigh = drc_value;
			}
			else{
				pPrivate->gsADec.gsADecInit.m_unAudioCodecParams.m_unDDPDec.m_iDynScaleHigh = 100;
			}
			
			property_get("tcc.audio.ddp.drc_low", value, "100");
			drc_value = atoi(value);
			
			if(drc_value >=0 && drc_value <=100 ){
				pPrivate->gsADec.gsADecInit.m_unAudioCodecParams.m_unDDPDec.m_iDynScaleLow =drc_value;
			}
			else{
				pPrivate->gsADec.gsADecInit.m_unAudioCodecParams.m_unDDPDec.m_iDynScaleLow = 100;
			}
		}

		if( cdk_adec_init(&pPrivate->cdk_core,
							&pPrivate->cdmx_info, 
							pPrivate->iAdecType,		// AUDIO_ID
							pPrivate->iCtype,			// CONTAINER_TYPE
							pPrivate->cb_function,
							&pPrivate->gsADec) < 0 )	// Audio decoder function
		{
			LOGE("Audio DEC init error.");
			inputbuffer->nFlags &= ~OMX_BUFFERFLAG_CODECCONFIG;
			// to skip all audio data
			inputbuffer->nFilledLen = 0;
			return; 	  
		}

		pPrivate->decode_ready  = OMX_TRUE;
		pPrivate->isNewBuffer = 1;
		outputbuffer->nFilledLen = 0;
		inputbuffer->nFilledLen = 0;
		
		if(pPrivate->gsADec.gsADecOutput.m_uiNumberOfChannel != pPrivate->pAudioPcmMode.nChannels )
		{
			pPrivate->pAudioPcmMode.nChannels = pPrivate->gsADec.gsADecOutput.m_uiNumberOfChannel;
		}
		if(pPrivate->gsADec.gsADecOutput.m_eSampleRate != pPrivate->pAudioPcmMode.nSamplingRate )
		{
			pPrivate->pAudioPcmMode.nSamplingRate = pPrivate->gsADec.gsADecOutput.m_eSampleRate;
		}

		DBUG_MSG("Audio DEC initialized.");
		return;
	}

	if(pPrivate->decode_ready == OMX_FALSE)
	{
		DBUG_MSG(" Audio Decoder not Initialized!!");
		// to skip all audio data
		inputbuffer->nFilledLen = 0;
		return;
	}

	// remaining data exist
	// but if inputbuffer has SYNCFRAME flag, it means that the remaining data is useless because it also means seeking was done.
	if(pPrivate->iRestSize > 0 && !(inputbuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME))
	{
		OMX_U32 current_size;
		current_size = pPrivate->iSplitLength * pPrivate->pAudioPcmMode.nChannels * sizeof(short);
		if(pPrivate->iRestSize > current_size)
		{
			outputbuffer->nFilledLen = current_size;
			pPrivate->iRestSize -= outputbuffer->nFilledLen;
			memcpy(outputbuffer->pBuffer, pPrivate->pRest+pPrivate->iSplitPosition, outputbuffer->nFilledLen);
			pPrivate->iSplitPosition += outputbuffer->nFilledLen; 
		}
		else
		{
			outputbuffer->nFilledLen = pPrivate->iRestSize;
			pPrivate->iRestSize = 0;
			memcpy(outputbuffer->pBuffer, pPrivate->pRest+pPrivate->iSplitPosition, outputbuffer->nFilledLen);
			inputbuffer->nFilledLen = 0;
		}

		pPrivate->iPrevTS += OUTPUT_SPLIT_TIME_LIMIT;
		outputbuffer->nTimeStamp = pPrivate->iPrevTS;
		pPrivate->iSamples += outputbuffer->nFilledLen;
		DBUG_MSG("Consume remaining data, nTimeStamp = %lld, output size = %d", ret, outputbuffer->nTimeStamp, outputbuffer->nFilledLen);
		return;
	}

	if(pPrivate->iCtype != CONTAINER_TYPE_AUDIO)
	{
		// if previous decoding failed, silence should be inserted 
		if(pPrivate->bPrevDecFail == OMX_TRUE)
		{
			OMX_TICKS time_diff = outputbuffer->nTimeStamp - pPrivate->iPrevTS;

			if(time_diff > 0 && time_diff < OUTPUT_SILENT_TIME_LIMIT)
			{
				OMX_TICKS samples = (outputbuffer->nTimeStamp - pPrivate->iPrevTS) * pPrivate->pAudioPcmMode.nSamplingRate / 1000000;
				outputbuffer->nFilledLen = pPrivate->pAudioPcmMode.nChannels * samples * sizeof(short);

				if(outputbuffer->nFilledLen > outputbuffer->nAllocLen) 
					outputbuffer->nFilledLen = outputbuffer->nAllocLen;

				memset(outputbuffer->pBuffer, 0, outputbuffer->nFilledLen);
				outputbuffer->nTimeStamp = pPrivate->iPrevTS;
				pPrivate->bPrevDecFail = OMX_FALSE;

				return;
			}
		}
	}

	if(inputbuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
	{
		iSeekFlag = 1;
		pPrivate->iStartTS = inputbuffer->nTimeStamp;
		pPrivate->iSamples = 0;
		pPrivate->iNumOfSeek++;

		// eliminate remaining data
		pPrivate->iRestSize = 0;

		if (pPrivate->iAdecType == AUDIO_ID_APE)
		{
			OMX_U8* p = inputbuffer->pBuffer;

			if ( *((OMX_U32*)(p+inputbuffer->nFilledLen-CODEC_SPECIFIC_MARKER_OFFSET)) == CODEC_SPECIFIC_MARKER)
			{
				codecSpecific = (ape_dmx_exinput_t *)(*((OMX_U32*)(p+inputbuffer->nFilledLen-CODEC_SPECIFIC_INFO_OFFSET)));
				inputbuffer->nFilledLen -= CODEC_SPECIFIC_MARKER_OFFSET;
			}
			else
			{
				LOGE("No codec specific data");
			}
		}
	}
	pPrivate->cdmx_out.m_uiUseCodecSpecific = codecSpecific;

	/* Decode the block */
	pPrivate->cdk_core.m_pOutWav = outputbuffer->pBuffer;

	pPrivate->cdmx_out.m_pPacketData = inputbuffer->pBuffer;
	pPrivate->cdmx_out.m_iPacketSize = inputbuffer->nFilledLen;

	//PrintHexData(inputbuffer->pBuffer);

	ret = cdk_adec_decode(&pPrivate->cdk_core,
			&pPrivate->cdmx_info,
			&pPrivate->cdmx_out,
			pPrivate->iAdecType,
			iSeekFlag,
			0, // alsa_status
			&pPrivate->gsADec);		

	if (ret >= 0)
	{
		if (pPrivate->cdmx_out.m_iDecodedSamples > 0)	//(ret == 0)
		{
			if(pPrivate->gsADec.gsADecOutput.m_uiNumberOfChannel != pPrivate->pAudioPcmMode.nChannels )
			{
				pPrivate->pAudioPcmMode.nChannels = pPrivate->gsADec.gsADecOutput.m_uiNumberOfChannel;
			}
			if(pPrivate->gsADec.gsADecOutput.m_eSampleRate != pPrivate->pAudioPcmMode.nSamplingRate )
			{
				pPrivate->pAudioPcmMode.nSamplingRate = pPrivate->gsADec.gsADecOutput.m_eSampleRate;
			}
		}
		
		outputbuffer->nFilledLen = pPrivate->pAudioPcmMode.nChannels * pPrivate->cdmx_out.m_iDecodedSamples * sizeof(short);
		if (outputbuffer->nFilledLen != 0 && !pPrivate->bOutputStarted)
		{
			pPrivate->bOutputStarted = OMX_TRUE;
		}

		if (pPrivate->iCtype != CONTAINER_TYPE_AUDIO)
		{
			// to reduce peak noise, decoded samples which are corresponding to iGuardSamples are set to 0 after seek
			// 'iNumOfSeek > 1' means that the first trial of seek is done actually
			if (pPrivate->iSamples < pPrivate->iGuardSamples && pPrivate->iNumOfSeek > 1)
			{
				OMX_U32 sample_length;
				sample_length = pPrivate->iSamples + pPrivate->cdmx_out.m_iDecodedSamples;

				if (sample_length > pPrivate->iGuardSamples)
				{
					memset(outputbuffer->pBuffer, 0, (pPrivate->iGuardSamples-pPrivate->iSamples) * pPrivate->pAudioPcmMode.nChannels * sizeof(short));
				}
				else
				{
					memset(outputbuffer->pBuffer, 0, outputbuffer->nFilledLen);
				}
			}

			if (pPrivate->cdmx_out.m_iDecodedSamples > pPrivate->iSplitLength) 
			{
				DBUG_MSG("output split on");

				pPrivate->iRestSize = (pPrivate->cdmx_out.m_iDecodedSamples - pPrivate->iSplitLength) * pPrivate->pAudioPcmMode.nChannels * sizeof(short);
				outputbuffer->nFilledLen = pPrivate->pAudioPcmMode.nChannels * pPrivate->iSplitLength * sizeof(short);
				memcpy(pPrivate->pRest, outputbuffer->pBuffer+outputbuffer->nFilledLen, pPrivate->iRestSize); 
				pPrivate->iSamples += (OMX_TICKS)pPrivate->iSplitLength;
				pPrivate->iPrevTS = outputbuffer->nTimeStamp;
				pPrivate->iSplitPosition = 0;
			}
			else
			{
				pPrivate->iSamples += (OMX_TICKS)pPrivate->cdmx_out.m_iDecodedSamples;
			}
		}
		else
		{
			if (pPrivate->iAdecType == AUDIO_ID_APE)
			{
				outputbuffer->nTimeStamp = pPrivate->iStartTS + ((OMX_TICKS)pPrivate->iSamples * 1000000 + (pPrivate->pAudioPcmMode.nSamplingRate >> 1)) / pPrivate->pAudioPcmMode.nSamplingRate;
				pPrivate->iSamples += (OMX_TICKS)pPrivate->cdmx_out.m_iDecodedSamples;
			}
		}

		if (0)
		{
			FILE* fp;
			fp = fopen("/nand/decout.txt", "ab");
			fwrite(outputbuffer->pBuffer, 1, outputbuffer->nFilledLen, fp);
			fclose(fp);
		}
		if ((outputbuffer->nFilledLen == 0 && ret >= 2)
			|| (!pPrivate->bOutputStarted && ret == 1) )
		{
			// ret = 1 : it's OK. This frame will be decoded next time.
			// ret = 2 : ??? TODO
			// ret > 2 : might fail decoding
			pPrivate->iPrevTS = outputbuffer->nTimeStamp;
			pPrivate->bPrevDecFail = OMX_TRUE;
		}
		else if(pPrivate->iAdecType != AUDIO_ID_APE)
		{
			pPrivate->bPrevDecFail = OMX_FALSE;

			OMX_TICKS duration;
			duration = ((OMX_TICKS)pPrivate->cdmx_out.m_iDecodedSamples * 1000000 ) / pPrivate->pAudioPcmMode.nSamplingRate;
			if (pPrivate->iPrevOriginalTS == inputbuffer->nTimeStamp) {
				outputbuffer->nTimeStamp = pPrivate->iNextTS;
				pPrivate->iNextTS += duration;
			} else {
				pPrivate->iPrevOriginalTS = inputbuffer->nTimeStamp;
				pPrivate->iNextTS = inputbuffer->nTimeStamp + duration;
			}
		}
		DBUG_MSG("Audio DEC Success. ret = %d, nTimeStamp = %lld, output size = %d", ret, outputbuffer->nTimeStamp, outputbuffer->nFilledLen);
	}
	else
	{
		LOGE( "cdk_audio_dec_run error: %ld", ret );
		pPrivate->iPrevTS = outputbuffer->nTimeStamp;
		pPrivate->bPrevDecFail = OMX_TRUE;
	}

	pPrivate->isNewBuffer = 1;
	if (pPrivate->iRestSize > 0)
	{
		// do not set nFilledLen to 0, in order to transfer all the split data to output at once
		inputbuffer->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
	}
	else
	{
		inputbuffer->nFilledLen = 0;
	}
}

