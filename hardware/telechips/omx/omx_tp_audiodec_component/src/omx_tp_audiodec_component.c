/**

  @file omx_tp_audiodec_component.c

  This component implement 3rd-party audio decoder.

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

#include <omx_tp_audiodec_component.h>

#include "../lib/third_party_audio_dec_api.h"
#include "TCCMemory.h"

#ifdef HAVE_ANDROID_OS
#define USE_EXTERNAL_BUFFER 0
#define LOG_TAG	"OMX_3rdParty_DEC"
#include <utils/Log.h>

static int DEBUG_ON	= 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);}
#endif /* HAVE_ANDROID_OS */

#ifdef HAVE_ANDROID_OS
OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
  	OMX_ERRORTYPE err = OMX_ErrorNone;
  	
  	DBUG_MSG("In %s \n",__func__); 

	err = component_Constructor(openmaxStandComp,cCompontName);

	return err;
}
#endif

OMX_ERRORTYPE component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName) 
{

	OMX_ERRORTYPE err = OMX_ErrorNone;  
	component_PrivateType* pPrivate;
	omx_base_audio_PortType *inPort,*outPort;
	OMX_U32 i;

	DBUG_MSG("In %s\n", __func__);
#ifdef HAVE_ANDROID_OS
	if (1)
#else
	if (!openmaxStandComp->pComponentPrivate) 
#endif
	{
		openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(component_PrivateType));

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

	pPrivate = openmaxStandComp->pComponentPrivate;
	pPrivate->ports = NULL;

	/** we could create our own port structures here
	* fixme maybe the base class could use a "port factory" function pointer?  
	*/
	err = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

	DBUG_MSG("constructor of ThirdParty decoder component is called\n");

	/* Domain specific section for the ports. */  
	/* first we set the parameter common to both formats */
	/* parameters related to input port which does not depend upon input audio format    */
	/* Allocate Ports and call port constructor. */  

	pPrivate->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
  	pPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

	if (pPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts && !pPrivate->ports) 
	{
	    pPrivate->ports = TCC_calloc(pPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
	    if (!pPrivate->ports) 
	    {
	  	    return OMX_ErrorInsufficientResources;
	    }
	    for (i=0; i < pPrivate->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) 
	    {
		      pPrivate->ports[i] = TCC_calloc(1, sizeof(omx_base_audio_PortType));
		      if (!pPrivate->ports[i]) 
		      {
		        	return OMX_ErrorInsufficientResources;
		      }
	    }
	}

	base_audio_port_Constructor(openmaxStandComp, &pPrivate->ports[0], 0, OMX_TRUE); // input
	base_audio_port_Constructor(openmaxStandComp, &pPrivate->ports[1], 1, OMX_FALSE); // output

	/*======================================================================================================*/
	/*	3rd-Praty Lib Port setting     													 					*/
	/*======================================================================================================*/	
	Port_Setting(pPrivate);
	
	/** now it's time to know the audio coding type of the component */
	if(!strcmp(cComponentName, audio_dec_name))  
	{   
		 pPrivate->audio_coding_type = audio_coding_type;
	} 
	else if (!strcmp(cComponentName, AUDIO_DEC_BASE_NAME)) 
	{
		pPrivate->audio_coding_type = OMX_AUDIO_CodingUnused;
	}
	else  
	{
	    // IL client specified an invalid component name
	    
		LOGE("OMX_ErrorInvalidComponentName %s", cComponentName);
	    return OMX_ErrorInvalidComponentName;
	}

	/** general configuration irrespective of any audio formats */
	/**  setting values of other fields of omx_maddec_component_Private structure */
	
	pPrivate->BufferMgmtCallback =component_BufferMgmtCallback;
	pPrivate->destructor = component_Destructor;
	pPrivate->messageHandler = omx_audiodec_component_MessageHandler;
	openmaxStandComp->SetParameter = omx_audiodec_component_SetParameter;
	openmaxStandComp->GetParameter = omx_audiodec_component_GetParameter;
	openmaxStandComp->GetExtensionIndex = omx_audiodec_component_GetExtensionIndex;

	pPrivate->decode_ready = OMX_FALSE;	
	
	memset(&pPrivate->cdk_core, 0x00, sizeof(cdk_core_t));
	memset(&pPrivate->cdmx_info, 0x00, sizeof(cdmx_info_t));
	memset(&pPrivate->cdmx_out, 0x00, sizeof(cdmx_output_t));
	
	if (pPrivate->pRest == NULL)
	{
		pPrivate->pRest = (OMX_U8*)malloc(DEFAULT_OUT_BUFFER_SIZE);
	}
	
	if( omx_audiodec_open_inbuffer(openmaxStandComp) < 0)
	{
		return OMX_ErrorInsufficientResources;
	}
	
	/*======================================================================================================*/
	/*	3rd-Praty Lib Open             													 					*/
	/*======================================================================================================*/	
	pPrivate->pDecoder = ThirdPartyAudio_OpenDecoder(pPrivate);
	if(pPrivate->pDecoder == NULL)
	{
		return OMX_ErrorInsufficientResources;
	}
	DBUG_MSG("constructor of ThirdParty decoder component is completed %d \n", err);
	
	return err;

}

#if 1
#define OUTPUT_SPLIT_TIME_LIMIT 50000 // 50ms
#define OUTPUT_SILENT_TIME_LIMIT 1000000 // 1sec

void component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;
	OMX_S32 iInputLength, iBytesLeft;
	OMX_S32 iUsedBytes, iTotUsedBytes, iDecodedSampleRate, iDecodedChannels, iRet	= 0;
	OMX_U8* pOutPtr;
	OMX_U8* pInputPtr = inputbuffer->pBuffer;
	OMX_S16 iSeekFlag = 0;
	OMX_S32 iOutFrameSize, iDecodedSamples = 0;
	
	outputbuffer->nFilledLen = 0;
	outputbuffer->nOffset = 0;
	
	DBUG_MSG("! %s (3rd-Party) IN inLen = %u, Flags = 0x%x, Timestamp = %lld", __func__, inputbuffer->nFilledLen, inputbuffer->nFlags, inputbuffer->nTimeStamp);
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
		
		iDecodedSampleRate = pPrivate->cdmx_info.m_sAudioInfo.m_iSamplePerSec;
		iDecodedChannels = pPrivate->cdmx_info.m_sAudioInfo.m_iChannels;
		
        /*======================================================================================================*/
        /*	3rd-Praty Lib Init																 					*/
        /*======================================================================================================*/

		iRet = ThirdPartyAudio_InitDecoder(pPrivate, 
										   inputbuffer,
										   outputbuffer,
										   &iDecodedSampleRate,
										   &iDecodedChannels);	
						  
						  
		if( iRet != 0 )	// Audio decoder function
		{
			LOGE("Audio DEC init error\n");
			inputbuffer->nFlags &= ~OMX_BUFFERFLAG_CODECCONFIG;
			// to skip all audio data
			inputbuffer->nFilledLen = 0;
			return; 	  
		}
		
		
		pPrivate->decode_ready  = OMX_TRUE;
		pPrivate->isNewBuffer = 1;
		outputbuffer->nFilledLen = 0;
		inputbuffer->nFilledLen = 0;
		
		if(iDecodedChannels != pPrivate->pAudioPcmMode.nChannels )
		{
			pPrivate->pAudioPcmMode.nChannels = iDecodedChannels;
		}
		if(iDecodedSampleRate != pPrivate->pAudioPcmMode.nSamplingRate )
		{
			pPrivate->pAudioPcmMode.nSamplingRate = iDecodedSampleRate;
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
		DBUG_MSG("Consume remaining data, nTimeStamp = %lld, output size = %d", iRet, outputbuffer->nTimeStamp, outputbuffer->nFilledLen);
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
		
		// To flush the buffered input
		omx_audiodec_flush_inbuffer(openmaxStandComp);
		
		/*======================================================================================================*/
		/*	3rd-Praty Lib Flush														 							*/
		/*======================================================================================================*/
		ThirdPartyAudio_FlushDecoder(pPrivate);
	}
	
	//pPrivate->cdmx_out.m_uiUseCodecSpecific = codecSpecific;
	pInputPtr = omx_audiodec_fill_inbuffer(openmaxStandComp, inputbuffer, &iBytesLeft);
	if(pInputPtr == NULL)
	{
		LOGE( "omx_audiodec_fill_inbuffer error" );
		pPrivate->iPrevTS = outputbuffer->nTimeStamp;
		pPrivate->bPrevDecFail = OMX_TRUE;
		return;
	}
	
	/* Decode the block */	
	iDecodedChannels = pPrivate->pAudioPcmMode.nChannels;
	iDecodedSampleRate = pPrivate->pAudioPcmMode.nSamplingRate;
			
	iTotUsedBytes = 0;
	pOutPtr = outputbuffer->pBuffer;
	
	// Normally, Decoding operations are performed until the input data is exhausted.
	while( ( iBytesLeft > MIN_STREM_SIZE  ) || ( ( iBytesLeft > 0 ) && ( pPrivate->cdmx_out.m_iEndOfFile == 1 ) ) )
	//while(iBytesLeft > 0)
	{
		iInputLength = iBytesLeft;
		
		/*======================================================================================================*/
		/* 3rd-Praty Lib Decode														 							*/
		/*======================================================================================================*/		
		// The following information is provided as input through the variable on the left::
        //    Variable       Meaning
		// 1. pInputPtr    = input bit-stream buffer pointer
		// 2. iInputLength = amount of input bit-stream in bytes
		// 3. pOutPtr      = output pcm buffer pointer
				
		iRet = ThirdPartyAudio_DecodeFrame(pPrivate, 
										pInputPtr, 
										iInputLength, 
										(OMX_PTR *)pOutPtr, 
										&iUsedBytes, 
										&iOutFrameSize,
										&iDecodedSampleRate,
										&iDecodedChannels);
						  				  
		// The following information must be provided through the variable on the left::
        //    Variable                Meaning
		// 1. iUsedBytes      		= How many bytes the decoder has read (used bytes of input buffer)
		// 2. iDecodedSamples 		= How many samples were decoded (number of total samples, not bytes)
		// 3. iDecodedSampleRate	= Samplerate of Decoded PCM
		// 4. iDecodedChannels		= Number of channels of Decoded PCM
		// 5. iRet             		= Decoding result
		//    iRet == 0  : Decoding success
		//    iRet == 1 	: Need more bitstream data
		//    iRet == 2 	: This is recoverable, just ignore the current frame
		//    else		: Unrecoverable error, the remaining data in the input bit-stream buffer will be discarded
				
		/* input buffer update */
		iTotUsedBytes += iUsedBytes;		
		pInputPtr += iUsedBytes;
		iBytesLeft -= iUsedBytes;
		
		/* output buffer update */
		iDecodedSamples += iOutFrameSize;
		pOutPtr = outputbuffer->pBuffer + iDecodedSamples * sizeof(short);
		
		if(iRet)
		{
			if(iRet < 0 || iRet > 2)
				iTotUsedBytes += iBytesLeft;	// Unrecoverable, Discarding the remaining data
			break;
		}
	}
	
	// move the existing data to the beginning of the buffer
	omx_audiodec_update_inbuffer(openmaxStandComp, iTotUsedBytes);	
	
	iDecodedSamples = iDecodedSamples / iDecodedChannels;
	pPrivate->cdmx_out.m_iDecodedSamples = iDecodedSamples;
	pPrivate->cdmx_out.m_iPacketSize = iBytesLeft;
	
	if (iRet >= 0)
	{
		if (iDecodedSamples > 0)
		{
			if(iDecodedChannels != pPrivate->pAudioPcmMode.nChannels )
			{
				pPrivate->pAudioPcmMode.nChannels = iDecodedChannels;
			}
			if(iDecodedSampleRate != pPrivate->pAudioPcmMode.nSamplingRate )
			{
				pPrivate->pAudioPcmMode.nSamplingRate = iDecodedSampleRate;
			}
		}
		
		outputbuffer->nFilledLen = pPrivate->pAudioPcmMode.nChannels * iDecodedSamples * sizeof(short);
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
				sample_length = pPrivate->iSamples + iDecodedSamples;

				if (sample_length > pPrivate->iGuardSamples)
				{
					memset(outputbuffer->pBuffer, 0, (pPrivate->iGuardSamples-pPrivate->iSamples) * pPrivate->pAudioPcmMode.nChannels * sizeof(short));
				}
				else
				{
					memset(outputbuffer->pBuffer, 0, outputbuffer->nFilledLen);
				}
			}

			if (iDecodedSamples > pPrivate->iSplitLength) 
			{
				DBUG_MSG("output split on: iDecodedSamples %d, iSplitLength %d", iDecodedSamples, pPrivate->iSplitLength);

				pPrivate->iRestSize = (iDecodedSamples - pPrivate->iSplitLength) * pPrivate->pAudioPcmMode.nChannels * sizeof(short);
				outputbuffer->nFilledLen = pPrivate->pAudioPcmMode.nChannels * pPrivate->iSplitLength * sizeof(short);
				memcpy(pPrivate->pRest, outputbuffer->pBuffer+outputbuffer->nFilledLen, pPrivate->iRestSize); 
				pPrivate->iSamples += (OMX_TICKS)pPrivate->iSplitLength;
				pPrivate->iPrevTS = outputbuffer->nTimeStamp;
				pPrivate->iSplitPosition = 0;
			}
			else
			{
				pPrivate->iSamples += (OMX_TICKS)iDecodedSamples;
			}
		}

		if ((outputbuffer->nFilledLen == 0 && iRet >= 2)
			|| (!pPrivate->bOutputStarted && iRet == 1) )
		{
			// iRet = 1 : it's OK. This frame will be decoded next time.
			// iRet = 2 : ???
			// iRet > 2 : might fail decoding
			pPrivate->iPrevTS = outputbuffer->nTimeStamp;
			pPrivate->bPrevDecFail = OMX_TRUE;
		}
		else// if(pPrivate->iAdecType != AUDIO_ID_APE)
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
		DBUG_MSG("Audio DEC Success. iRet = %d, nTimeStamp = %lld, output size = %d", iRet, outputbuffer->nTimeStamp, outputbuffer->nFilledLen);
	}
	else
	{
		LOGE( "cdk_audio_dec_run error: %ld", iRet );
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


/** The destructor */
OMX_ERRORTYPE component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) 
{

	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;

	DBUG_MSG("In 3rd_Party %s \n",__func__); 
	
	/*======================================================================================================*/
	/*	3rd-Praty Lib Close														 							*/
	/*======================================================================================================*/
	ThirdPartyAudio_CloseDecoder(pPrivate);
	
	pPrivate->pDecoder = NULL;

	omx_audiodec_close_inbuffer(openmaxStandComp);

	omx_audiodec_component_Destructor(openmaxStandComp);
	
	return OMX_ErrorNone;

}

OMX_S32 omx_audiodec_open_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  
	
	if( pPrivate->pAudioInputBuffer == NULL )
	{
		pPrivate->iAudioInBufferSize = AUDIO_MAX_INPUT_SIZE;
		pPrivate->pAudioInputBuffer = malloc(pPrivate->iAudioInBufferSize);

		if( pPrivate->pAudioInputBuffer == NULL )
		{
			DBUG_MSG( "pAudioInputBuffer allocation fail\n");
			return -1;					
		}
	}
	
	pPrivate->iDataLength = 0;
	return 0;
}

OMX_S32 omx_audiodec_close_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  
	
	pPrivate->iAudioInBufferSize = 0;
	if( pPrivate->pAudioInputBuffer != NULL )
	{
		free(pPrivate->pAudioInputBuffer);
		pPrivate->pAudioInputBuffer = 0;
	}
	
	pPrivate->iDataLength = 0;
	return 0;
}

OMX_U8* omx_audiodec_fill_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_S32 *piLength)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  
	
	//DBUG_MSG( "set inlength %d, datalength %d\n", inputbuffer->nFilledLen, pPrivate->iDataLength);
	
	if( (OMX_S32)(pPrivate->iDataLength + inputbuffer->nFilledLen) > (OMX_S32)pPrivate->iAudioInBufferSize)
	{
		OMX_U8 *pucTmpBuffer = NULL;
		
		DBUG_MSG( "Allocated size of pAudioInputBuffer is over --> realloc buffer!\n");
		
		pPrivate->iAudioInBufferSize = pPrivate->iDataLength + inputbuffer->nFilledLen;
		pucTmpBuffer = malloc(pPrivate->iAudioInBufferSize);	
		if( pucTmpBuffer == NULL )
		{
			DBUG_MSG( "pAudioInputBuffer allocation fail\n");
			return NULL;					
		}
		if( pPrivate->pAudioInputBuffer != NULL )
		{
			memcpy(pucTmpBuffer, pPrivate->pAudioInputBuffer, pPrivate->iDataLength);
			free(pPrivate->pAudioInputBuffer);
		}
		pPrivate->pAudioInputBuffer = pucTmpBuffer;
	}
								
	memcpy(pPrivate->pAudioInputBuffer + pPrivate->iDataLength, inputbuffer->pBuffer, inputbuffer->nFilledLen);
	pPrivate->iDataLength += inputbuffer->nFilledLen;
	
	*piLength = pPrivate->iDataLength;
	
	//DBUG_MSG( "set datalength %d\n", pPrivate->iDataLength);
	
	return pPrivate->pAudioInputBuffer;
}

OMX_S32 omx_audiodec_update_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp, OMX_S32 iUsedByte)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  
	
	//DBUG_MSG( "used bytes %d datalength %d\n", iUsedByte, pPrivate->iDataLength);
	
	if( iUsedByte > pPrivate->iDataLength )
	{
		LOGE( "used bytes %d datalength %d\n", iUsedByte, pPrivate->iDataLength);
		iUsedByte = pPrivate->iDataLength;
	}
		
	if( iUsedByte > 0 )
	{
		memmove(pPrivate->pAudioInputBuffer, pPrivate->pAudioInputBuffer + iUsedByte, pPrivate->iDataLength - iUsedByte);	
		
		pPrivate->iDataLength -= iUsedByte;
		if(pPrivate->iDataLength < 0)
			pPrivate->iDataLength = 0;
	}
	
	//DBUG_MSG( "datalength %d\n", pPrivate->iDataLength);
	return 0;
}

OMX_S32 omx_audiodec_flush_inbuffer(OMX_COMPONENTTYPE *openmaxStandComp)
{
	component_PrivateType* pPrivate = openmaxStandComp->pComponentPrivate;  
	
	pPrivate->iDataLength = 0;

	return 0;
}
#endif