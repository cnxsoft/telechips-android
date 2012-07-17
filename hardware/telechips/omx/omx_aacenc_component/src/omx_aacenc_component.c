/**
  This component implements AAC Audio encoder. 

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

  Android created by ZzaU.
*/ 
#include <omxcore.h>
#include <omx_base_audio_port.h>

#include <omx_aacenc_component.h>
#include <tccaudio.h>
#include <OMX_TCC_Index.h>
#include "TCCFile.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_TCC_AACENC"
#include <utils/Log.h>

static int DEBUG_ON	= 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);}
#endif /* HAVE_ANDROID_OS */

#define DEFAULT_FILENAME_LENGTH 256
#define NUMBER_INPUT_BUFFER_AUDIO_ENC  5
#define NUMBER_OUTPUT_BUFFER_AUDIO_ENC  2

#define AAC_SAMPLES_PER_FRAME			1024
#define	AAC_PCM_BUFFER_SIZE				(AAC_SAMPLES_PER_FRAME*6)//(AAC_SAMPLES_PER_FRAME*3)
#define	HEAP_SIZE_FOR_AMR_AACEnc		(32*1024)			//AmrNB, AACEnc에서 공용으로 사용하는 heap mem area size.
static void * pAAC_ENC_Heap;
static void *pVVAACHandle = NULL;

//static tAACENC gstAACENC;
static int gsRecordTime = 0;
int VIDEOCODEC_AAC_EncOpen(OMX_COMPONENTTYPE *openmaxStandComp, unsigned int channel, unsigned int frequency, unsigned int bitrate)
{
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	tAACENC *pAACEnc = omx_private->pAACENC;
	tMp4AacEncParams stAACParam;
	tInt32 ret;

	DBUG_MSG("In VIDEOCODEC_AAC_EncOpen. ch = %d, freq = %d, BitRate = %d", channel, frequency, bitrate);

//	VV_gStartRec = 0;
	gsRecordTime = 0;//gtCamcorderFileInfo.RecordTime = 0;
//	guiSyncFrameCount = 0;
	
	memset(pAACEnc, 0x00, sizeof(tAACENC) );
	memset(&stAACParam, 0x00, sizeof(tMp4AacEncParams) );

	pAACEnc->ulBitRate 		= bitrate;
	pAACEnc->usSampleRate 	= frequency;
	pAACEnc->ucChannels 	= channel;

	stAACParam.NumberOfChannels 	= pAACEnc->ucChannels;
	stAACParam.OutputBitRate 		= pAACEnc->ulBitRate;

	// added by shmin for M2TS
	if(omx_private->pAudioAac.eAACStreamFormat == OMX_AUDIO_AACStreamFormatMP4ADTS)
		stAACParam.OutputFormat 		= FORMAT_ADTS;
	else if(omx_private->pAudioAac.eAACStreamFormat == OMX_AUDIO_AACStreamFormatADIF)
		stAACParam.OutputFormat 		= FORMAT_ADIF;
	else
		stAACParam.OutputFormat 		= FORMAT_RAW;	

	stAACParam.SamplingFrequency 	= pAACEnc->usSampleRate;
	stAACParam.TurnOnPns 			= E_OFF;
	stAACParam.TurnOnTns 			= E_OFF;
	stAACParam.VersionInfo 			= 1;

	pAAC_ENC_Heap = TCC_malloc(HEAP_SIZE_FOR_AMR_AACEnc); //gstCCDEncHeap.AudioCodecHeapBuffer.pMemAddr ; 
	
	ret = AACEnc_Create(&pVVAACHandle, pAAC_ENC_Heap);
	if( ret < 0)
	{
		LOGE("ERROR:: %d = AACEnc_Create()", ret);
		return	-1;
	}

	ret = AACEnc_Init(pVVAACHandle, &stAACParam);
	if( ret < 0 )
	{
		LOGE("ERROR:: %d = AACEnc_Init()", ret);
		return	0;
	}

	DBUG_MSG("Out VIDEOCODEC_AAC_EncOpen.");
	
	return 0;
}

int VIDEOCODEC_AAC_EncClose(void)
{
	tInt32 ret = 0;

//	VV_gStartRec = 0;

	if( ret < 0 )
	{
		return -1;
	}
	
	TCC_free(pAAC_ENC_Heap);

	return 0;
} 

int VIDEOCODEC_AAC_EncDSI(OMX_COMPONENTTYPE *openmaxStandComp, char *bitstream, unsigned int *size)
{
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	tAACENC *pAACEnc = omx_private->pAACENC;
	const unsigned int usAACSRMap[] =    { 96000, 88200, 64000, 48000,
											44100, 32000, 24000, 22050,
											16000, 12000, 11025, 8000, 7350 };
	unsigned sampling_frequency_index;

	for(sampling_frequency_index = 0; sampling_frequency_index < 13; sampling_frequency_index ++)
	{
		if(pAACEnc->usSampleRate == usAACSRMap[sampling_frequency_index])
		{
			bitstream[0] = 0x10 | ((sampling_frequency_index & 0x0F) >> 1);
			bitstream[1] = ((sampling_frequency_index & 0x01) << 7) | ((pAACEnc->ucChannels & 0x0F) << 3);
			*size = 2;
			return 0;
		}
	}
	
	return -1;
}

static void GetAudioChunk( char nCh, short *left, short *right, short * pSamples, int nSamples)
{
	long j;

	if(nCh == 2)
	{ 
		for(j=0;j<nSamples;j++) 
		{
			left[j] = pSamples[2*j];
			right[j] = pSamples[2*j+1];
		}
	}
	else
	{		
		for(j=0;j<nSamples;j++)
		{
			left[j] = pSamples[j];
			right[j] = 0;
		}
	}
	return;
}


int VIDEOCODEC_AAC_EncFrame(OMX_COMPONENTTYPE *openmaxStandComp, void *bitstream, void *pcm, unsigned int *size)
{
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	tAACENC *pAACEnc = omx_private->pAACENC;
	tInt32 ret;
	long  lLen;
	short *psLeft = (short *)omx_private->psLeftPCM;
	short *psRight = (short *)omx_private->psRightPCM;
	long  lLength;
	int aDstLen;

	lLen = AAC_SAMPLES_PER_FRAME;

//	BufferGetReadPointer(pAACEnc->pOutput, &psLeft, &psRight,&lLength);
	GetAudioChunk(omx_private->pAACENC->ucChannels, psLeft, psRight, pcm, lLen);

	aDstLen = AAC_SAMPLES_PER_FRAME*2;	
	ret = AACEnc_EncodeFrame( pVVAACHandle, psLeft, psRight, (unsigned char*)bitstream, &aDstLen );

	if(ret < 0)
	{
		aDstLen = 0;
		*size = 0 ;
		LOGE("ERROR:: %d = AACEnc_EncodeFrame()", ret);
		
		return -1 ;
	}

	
	*size = aDstLen;
			
//	BufferUpdateReadPointerJS(pAACEnc->pOutput,lLen);

	pAACEnc->ulTimePos += lLen;

	gsRecordTime = ((pAACEnc->ulTimePos / pAACEnc->usSampleRate) * 1000) +
					(((pAACEnc->ulTimePos % pAACEnc->usSampleRate) * 1000) /
					pAACEnc->usSampleRate);
	
	DBUG_MSG("gsRecordTime = %d", gsRecordTime);

	return 0;
}


void omx_aacenc_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp,OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer)
{
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;  
	OMX_U32 nchannels;
	OMX_S32 write_cnt = 0;
	OMX_S32 total_cnt = 0;
	OMX_S32 encoder_count=0;
	OMX_U8 *inputdata;
	OMX_S32 rec_temp_cnt = 0;
	OMX_U32 out_temp_size = 0;
	OMX_S32 ret;

	DBUG_MSG("In  %s \n",__func__);

	inputdata =(OMX_U8 *)inputbuffer->pBuffer;
	total_cnt = inputbuffer->nFilledLen;
	encoder_count = AAC_SAMPLES_PER_FRAME * omx_private->pAudioAac.nChannels;
	outputbuffer->nFilledLen = 0;

	if(omx_private->aac_encode_ready == OMX_FALSE)
	{
		ret = VIDEOCODEC_AAC_EncOpen(openmaxStandComp, omx_private->pAudioAac.nChannels, 
										omx_private->pAudioAac.nSampleRate, 
										omx_private->pAudioAac.nBitRate);
		if(ret < 0)
		{
			goto ERR_PROCESS;
		}


		VIDEOCODEC_AAC_EncDSI(openmaxStandComp, outputbuffer->pBuffer, &outputbuffer->nFilledLen);
		LOGI(" AAC_EncDSI :: len = %d", outputbuffer->nFilledLen);
		
		omx_private->aac_encode_ready = OMX_TRUE;
	}

	if(omx_private->eof_flag == 0)
	{
		if(!omx_private->rec_ramainbufCount)
		{
			memset(omx_private->rec_tmpbuf , 0,  sizeof(omx_private->rec_tmpbuf )) ;
			memcpy(omx_private->rec_tmpbuf , inputdata, total_cnt) ;
		}
		else
		{
			memcpy(&omx_private->rec_tmpbuf [omx_private->rec_ramainbufCount] , inputdata, inputbuffer->nFilledLen) ;
			total_cnt = total_cnt+omx_private->rec_ramainbufCount;
		}

		omx_private->rec_ramainbufCount = 0;
		
		while(total_cnt>= (encoder_count*2))
		{
			memcpy(omx_private->in_tmpbuf, (short*)&omx_private->rec_tmpbuf [write_cnt*encoder_count*2], encoder_count*2) ;
			ret = VIDEOCODEC_AAC_EncFrame(openmaxStandComp, omx_private->out_tmpbuf, omx_private->in_tmpbuf, &out_temp_size);

			if(!ret)	// encoding에 문제가 있거나 더이상 데이타가 없다.
			{ 
				outputbuffer->nFlags = OMX_BUFFERFLAG_EOS;
				omx_private->eof_flag == TRUE;
			}

			total_cnt  = total_cnt  - encoder_count*2;
			write_cnt ++;

			memcpy(&outputbuffer->pBuffer[outputbuffer->nFilledLen], omx_private->out_tmpbuf, out_temp_size);

			outputbuffer->nFilledLen += out_temp_size;
			omx_private->rec_ramainbufCount = total_cnt;
			rec_temp_cnt = write_cnt*encoder_count*2;
		}	

		if((total_cnt<(encoder_count*2)) && (total_cnt>0))
		{
			if(omx_private->rec_ramainbufCount != 0)
			{
				memmove (omx_private->rec_tmpbuf , &omx_private->rec_tmpbuf [rec_temp_cnt], total_cnt);
				omx_private->rec_ramainbufCount = total_cnt;
			}
			else
			{
				omx_private->rec_ramainbufCount += total_cnt;
			}
		}
		else
		{
			omx_private->rec_ramainbufCount = 0;
		}

		inputbuffer->nFilledLen = 0;
		outputbuffer->nFlags = 0;
		outputbuffer->nTimeStamp = gsRecordTime * 1000;
	}
	else if(omx_private->stop_flag == TRUE)
	{
		inputbuffer->nFilledLen = 0;
		outputbuffer->nFlags = OMX_BUFFERFLAG_EOS;
		omx_private->eof_flag == TRUE;
	}
	else
	{
//		MP3EncodeFunction(SUBFN_CODEC_CLOSE, 0, 0, 0, 0);
		VIDEOCODEC_AAC_EncClose();

		(*(omx_private->callbacks->EventHandler))
		   (openmaxStandComp,
		   omx_private->callbackData,
		   OMX_EventDynamicResourcesAvailable, 
		   0,
		   0, 
		   NULL);

		inputbuffer->nFilledLen = 0;
		outputbuffer->nFlags 	= OMX_BUFFERFLAG_EOS;
		omx_private->eof_flag 	= 0;
		omx_private->stop_flag 	= TRUE;
	}

	return;
	
ERR_PROCESS:
	LOGE( "ERROR !! \n");
	VIDEOCODEC_AAC_EncClose();
	
	(*(omx_private->callbacks->EventHandler))(
						   openmaxStandComp,
						   omx_private->callbackData,
						   OMX_EventError, 
						   OMX_ErrorHardware,
						   0, 
						   NULL);

	
}

/** this function sets the parameter values regarding audio format & index */
OMX_ERRORTYPE omx_aacenc_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)
{

	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
	OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
	OMX_AUDIO_PARAM_AACPROFILETYPE* pAudioAac;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_U32 portIndex;
	OMX_U32 nFileNameLength;
	OMX_U32 *RecSetBitrate;

	/* Check which structure we are being fed and make control its header */
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	omx_base_audio_PortType *port;
	
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
				port = (omx_base_audio_PortType *) omx_private->ports[portIndex];
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
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			memcpy(&omx_private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
		break;

		case OMX_IndexParamAudioAac:
			pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*) ComponentParameterStructure;
			portIndex = pAudioAac->nPortIndex;
			err = omx_base_component_ParameterSanityCheck(hComponent,portIndex,pAudioAac,sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			if(err!=OMX_ErrorNone)
			{ 
				DBUG_MSG("In %s Parameter Check Error=%x\n",__func__,err); 
				break;
			}
			if (pAudioAac->nPortIndex == 1) 
			{
				memcpy(&omx_private->pAudioAac, pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			}  
			else
			{
				err = OMX_ErrorBadPortIndex;
			}			
		break;

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if (!strcmp( (char*) pComponentRole->cRole, AUDIO_ENC_AAC_ROLE)) 
			{
				omx_private->audio_coding_type = OMX_AUDIO_CodingAAC;
			}  
			else
			{
				err = OMX_ErrorBadParameter;
			}
		break;		

		default: /*Call the base component function*/
			err = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
		break;
		
	}

	if(err != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, err);

	return err;
}  





/** this function gets the parameters regarding audio formats and index */
OMX_ERRORTYPE omx_aacenc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)
{

	OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;  
	OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
	OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
	OMX_AUDIO_PARAM_AACPROFILETYPE *pAudioAac;
	omx_base_audio_PortType *port;
	OMX_ERRORTYPE err = OMX_ErrorNone;

	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

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
			memcpy(ComponentParameterStructure, &omx_private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
			break;    

		case OMX_IndexParamAudioPortFormat:
			pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			if (pAudioPortFormat->nPortIndex <= 1) 
			{
			  port = (omx_base_audio_PortType *)omx_private->ports[pAudioPortFormat->nPortIndex];
				memcpy(pAudioPortFormat, &port->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
			} 
			else 
			{
				err = OMX_ErrorBadPortIndex;
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
				err = OMX_ErrorBadPortIndex;
			}
			memcpy(pAudioPcmMode, &omx_private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
			break;

		case OMX_IndexParamAudioAac:
			pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE))) != OMX_ErrorNone) { 
			  break;
			}
			if (pAudioAac->nPortIndex != 1) 
			{
				err = OMX_ErrorBadPortIndex;
			}
			memcpy(pAudioAac, &omx_private->pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
			break;

		case OMX_IndexParamStandardComponentRole:
			pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
			if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) 
			{ 
				break;
			}
			if (omx_private->audio_coding_type == OMX_AUDIO_CodingAAC)
			{
			  strcpy( (char*) pComponentRole->cRole, AUDIO_ENC_AAC_ROLE);
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
					err = OMX_ErrorBadParameter;
				}
		
				memset(pCap_flags, 0x00, sizeof(PV_OMXComponentCapabilityFlagsType));
				pCap_flags->iIsOMXComponentMultiThreaded = OMX_TRUE;
				pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc = OMX_TRUE;//OMX_TRUE;
				pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc = OMX_TRUE;
			//	pCap_flags.iOMXComponentSupportsMovableInputBuffers = OMX_TRUE;
			//	pCap_flags.iOMXComponentSupportsPartialFrames = OMX_TRUE;
			//	pCap_flags.iOMXComponentCanHandleIncompleteFrames = OMX_TRUE;
			}
			break;
#endif

		default: /*Call the base component function*/
			err = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
		break;
	}

	if(err != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, err);

	return OMX_ErrorNone;

}

OMX_ERRORTYPE omx_aacenc_component_SetConfig
	(OMX_IN  OMX_HANDLETYPE hComponent,
	OMX_IN  OMX_INDEXTYPE nIndex,
	OMX_IN  OMX_PTR ComponentParameterStructure)
{
	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

	DBUG_MSG("In  %s \n",__func__);

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	switch(nIndex)
	{
		default:
			return omx_base_component_SetConfig(hComponent, nIndex, ComponentParameterStructure);
	}
	return err;
}


OMX_ERRORTYPE omx_aacenc_component_GetConfig
	(OMX_IN  OMX_HANDLETYPE hComponent,
	OMX_IN  OMX_INDEXTYPE nIndex,
	OMX_IN  OMX_PTR ComponentParameterStructure)
{
	OMX_ERRORTYPE err = OMX_ErrorNone;
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
//	OMX_AUDIO_CONFIG_GETTIMETYPE *gettime;
	OMX_TIME_CONFIG_TIMESTAMPTYPE *gettime; 
	OMX_AUDIO_CONFIG_INFOTYPE *info;

	DBUG_MSG("In  %s \n",__func__);

	if (ComponentParameterStructure == NULL)
	{
		return OMX_ErrorBadParameter;
	}

	switch(nIndex)
	{
		case OMX_IndexConfigTimeCurrentMediaTime:
			 gettime = (OMX_TIME_CONFIG_TIMESTAMPTYPE*)ComponentParameterStructure;
//			 MP3EncodeFunction(SUBFN_CODEC_GETTIME,0, &gettime->nTimestamp, 0, 0);
			 break;
		default:
			return omx_base_component_GetConfig(hComponent, nIndex, ComponentParameterStructure);
	}
	return err;
}


OMX_ERRORTYPE omx_aacenc_encoder_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp, internalRequestMessageType *message)  
{

	omx_aacenc_component_PrivateType* omx_private = (omx_aacenc_component_PrivateType*)openmaxStandComp->pComponentPrivate;  
	OMX_ERRORTYPE err;
	OMX_STATETYPE eCurrentState = omx_private->state;

	DBUG_MSG("In  %s \n",__func__);

	/** Execute the base message handling */
	err = omx_base_component_MessageHandler(openmaxStandComp, message);

	if (message->messageType == OMX_CommandStateSet)
	{
		if ((message->messageParam == OMX_StateLoaded) && (eCurrentState == OMX_StateIdle))
		{
		}
		else if ((message->messageParam == OMX_StateIdle) && (eCurrentState == OMX_StateExecuting))
		{
		}
	}
	return err;  
	
}


OMX_ERRORTYPE omx_aacenc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName) 
{

	OMX_ERRORTYPE err = OMX_ErrorNone;  
	omx_aacenc_component_PrivateType* omx_private;
	omx_base_audio_PortType *outPort, *inPort;
	OMX_U32 i;

#ifdef HAVE_ANDROID_OS
	if (1)
#else
	if (!openmaxStandComp->pComponentPrivate) 
#endif
	{
		openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_aacenc_component_PrivateType));

		if(openmaxStandComp->pComponentPrivate==NULL)  
		{
			return OMX_ErrorInsufficientResources;
		}
	} 
	else 
	{
	    DBUG_MSG("In %s, Error Component %x Already Allocated\n", 
	              __func__, (int)openmaxStandComp->pComponentPrivate);
	}

	  omx_private = openmaxStandComp->pComponentPrivate;
	  omx_private->ports = NULL;

	/** we could create our own port structures here
	* fixme maybe the base class could use a "port factory" function pointer?  
	*/
	err = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

	DBUG_MSG("constructor of aac encoder component is called\n");

	/* Domain specific section for the ports. */  
	/* first we set the parameter common to both formats */
	/* parameters related to input port which does not depend upon input audio format    */
	/* Allocate Ports and call port constructor. */  

	omx_private->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
	omx_private->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

	/** Allocate Ports and call port constructor. */  
	if (omx_private->sPortTypesParam[OMX_PortDomainAudio].nPorts && !omx_private->ports) 
	{
		omx_private->ports = TCC_calloc(omx_private->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
		if (!omx_private->ports) 
		{
			return OMX_ErrorInsufficientResources;
		}
		for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) 
		{
			omx_private->ports[i] = TCC_calloc(1, sizeof(omx_base_audio_PortType));
			if (!omx_private->ports[i]) 
			{
				return OMX_ErrorInsufficientResources;
			}
		}
	}

	base_audio_port_Constructor(openmaxStandComp, &omx_private->ports[0], 0, OMX_TRUE);
	base_audio_port_Constructor(openmaxStandComp, &omx_private->ports[1], 1, OMX_FALSE);

	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	  
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;
	strcpy(inPort->sPortParam.format.audio.cMIMEType, "raw");
	inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
	inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;
	
	/** parameters related to output port */
	outPort = (omx_base_audio_PortType *) omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];	
	strcpy(outPort->sPortParam.format.audio.cMIMEType, "audio/aac");
	outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAAC;
	outPort->sPortParam.nBufferSize = DEFAULT_OUT_BUFFER_SIZE;
	outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAAC;

    //Default values for AAC audio param port
	setHeader(&omx_private->pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
    omx_private->pAudioAac.nPortIndex = 1;
    omx_private->pAudioAac.nChannels = 2;
    omx_private->pAudioAac.nBitRate = 0;
    omx_private->pAudioAac.nSampleRate = 44100;
    omx_private->pAudioAac.nAudioBandWidth = 0;
    omx_private->pAudioAac.nFrameLength = 2048; // use HE_PS frame size as default
    omx_private->pAudioAac.eChannelMode = OMX_AUDIO_ChannelModeStereo;
    omx_private->pAudioAac.eAACProfile = OMX_AUDIO_AACObjectHE_PS;    //OMX_AUDIO_AACObjectLC;
    omx_private->pAudioAac.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP2ADTS;


	/** settings of output port audio format - pcm */
	setHeader(&omx_private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	omx_private->pAudioPcmMode.nPortIndex = 0;
	omx_private->pAudioPcmMode.nChannels = 2;
	omx_private->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
	omx_private->pAudioPcmMode.eEndian = OMX_EndianLittle;
	omx_private->pAudioPcmMode.bInterleaved = OMX_TRUE;
	omx_private->pAudioPcmMode.nBitPerSample = 16;
	omx_private->pAudioPcmMode.nSamplingRate = 44100;
	omx_private->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
	omx_private->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
	omx_private->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;


	/** now it's time to know the audio coding type of the component */
	if(!strcmp(cComponentName, AUDIO_ENC_AAC_NAME))  
	{   
		 omx_private->audio_coding_type = OMX_AUDIO_CodingAAC;
	} 
	else if (!strcmp(cComponentName, AUDIO_ENC_BASE_NAME)) 
	{
		omx_private->audio_coding_type = OMX_AUDIO_CodingUnused;
	}
	else  
	{
	    // IL client specified an invalid component name
	    
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
	    return OMX_ErrorInvalidComponentName;
	}

	omx_private->aac_encode_ready = OMX_FALSE;

	/** general configuration irrespective of any audio formats */
	/**  setting values of other fields of omx_maddec_component_Private structure */
	
	omx_private->BufferMgmtCallback = omx_aacenc_component_BufferMgmtCallback;
	omx_private->messageHandler = omx_aacenc_encoder_MessageHandler;
	omx_private->destructor = omx_aacenc_component_Destructor;
	openmaxStandComp->SetParameter = omx_aacenc_component_SetParameter;
	openmaxStandComp->GetParameter = omx_aacenc_component_GetParameter;
	openmaxStandComp->SetConfig = omx_aacenc_component_SetConfig;
	openmaxStandComp->GetConfig = omx_aacenc_component_GetConfig;
	openmaxStandComp->GetExtensionIndex = omx_aacenc_component_GetExtensionIndex;


	/** initialising aacdec structures */
	omx_private->pAACENC = TCC_malloc (sizeof(tAACENC));
	omx_private->eof_flag = FALSE;
	omx_private->stop_flag = FALSE;

	omx_private->rec_tmpbuf = TCC_malloc(DEFAULT_OUT_BUFFER_SIZE*2);
	omx_private->in_tmpbuf = TCC_malloc(DEFAULT_OUT_BUFFER_SIZE*2);
	omx_private->out_tmpbuf = TCC_malloc(DEFAULT_OUT_BUFFER_SIZE*2);
	omx_private->psLeftPCM = TCC_malloc(AAC_PCM_BUFFER_SIZE);
	omx_private->psRightPCM = TCC_malloc(AAC_PCM_BUFFER_SIZE);

	memset(omx_private->pAACENC, 0, sizeof(tAACENC));
	memset(omx_private->rec_tmpbuf, 0, sizeof(omx_private->rec_tmpbuf));
	memset(omx_private->out_tmpbuf, 0, sizeof(omx_private->out_tmpbuf));

	return err;

}

OMX_ERRORTYPE omx_aacenc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType)
{    
	DBUG_MSG("In  %s \n",__func__);

	if(strcmp(cParameterName,TCC_ENC_SET_BITRATE_STRING) == 0)
	{
		*pIndexType = OMX_IndexVendorParamSetBitrate;  
	}
	else if(strcmp(cParameterName,TCC_ENC_END_STRING) == 0)
	{
		*pIndexType = OMX_IndexVendorParamRecend;  
	}
	else
	{
		return OMX_ErrorBadParameter;
	}
	return OMX_ErrorNone;  
  
}


/** The destructor */
OMX_ERRORTYPE omx_aacenc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) 
{

	omx_aacenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

	DBUG_MSG("In  %s \n",__func__);

	if(omx_private->pAACENC !=NULL)
	{
		TCC_free(omx_private->pAACENC);
		omx_private->pAACENC = NULL;
	}

	if(omx_private->rec_tmpbuf !=NULL)
	{
		TCC_free(omx_private->rec_tmpbuf);
		omx_private->rec_tmpbuf = NULL;
	}

	if(omx_private->in_tmpbuf !=NULL)
	{
		TCC_free(omx_private->in_tmpbuf);
		omx_private->in_tmpbuf = NULL;
	}

	if(omx_private->out_tmpbuf !=NULL)
	{
		TCC_free(omx_private->out_tmpbuf);
		omx_private->out_tmpbuf = NULL;
	}

	if(omx_private->psLeftPCM !=NULL)
	{
		TCC_free(omx_private->psLeftPCM);
		omx_private->psLeftPCM = NULL;
	}

	if(omx_private->psRightPCM !=NULL)
	{
		TCC_free(omx_private->psRightPCM);
		omx_private->psRightPCM = NULL;
	}

	/* frees port/s */
	if (omx_private->ports) 
	{
		for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) 
		{
			if(omx_private->ports[i])
				omx_private->ports[i]->PortDestructor(omx_private->ports[i]);
		}
		TCC_free(omx_private->ports);
		omx_private->ports=NULL;
	}

	DBUG_MSG("Destructor of aac encoder component is called\n");

	omx_private->audio_coding_type = OMX_AUDIO_CodingAAC;
	
	omx_base_filter_Destructor(openmaxStandComp);

	return OMX_ErrorNone;

}


OMX_ERRORTYPE omx_aacenc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) 
{
	DBUG_MSG("In  %s \n",__func__);

	return (omx_aacenc_component_Constructor(openmaxStandComp, AUDIO_ENC_AAC_NAME));
}

OMX_ERRORTYPE omx_aacenc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp)
{
	DBUG_MSG("In  %s \n",__func__);

	return omx_aacenc_component_Destructor(openmaxStandComp);
}

#ifdef HAVE_ANDROID_OS
OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
  OMX_ERRORTYPE err = OMX_ErrorNone;

	err = omx_aacenc_component_Constructor(openmaxStandComp,cCompontName);

	return err;
}
#endif

