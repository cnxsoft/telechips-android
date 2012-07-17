
/**************************************************************************************
 * ThirdParty Audio Decoder API
 **************************************************************************************/
#include <utils/Log.h>
#include <omxcore.h>
#include <omx_base_audio_port.h>
#include <omx_tp_audiodec_component.h>
#include "third_party_audio_dec_api.h"


// TODO: include header file needed

// declaration third-party decoder structure
typedef struct ThirdPartyAudio_t
{
	// TODO: add member variables needed

	////////////////////////////////////////////////////////////////////////////////////
} ThirdPartyAudio_t;


void* ThirdPartyAudio_OpenDecoder(
    component_PrivateType* pPrivate
)
{
	ThirdPartyAudio_t *pDecoder;
	
	
	pDecoder = (ThirdPartyAudio_t *)malloc(sizeof(ThirdPartyAudio_t));
	if(pDecoder == NULL)
		return NULL;
		
	memset(pDecoder, 0, sizeof(ThirdPartyAudio_t));
	
	LOGD( "ThirdParty Decoder Open\n");
	
	// TODO: add additional opening code here
	
	////////////////////////////////////////////////////////////////////////////////////
		
	return (void *)pDecoder;
}

OMX_S32 ThirdPartyAudio_InitDecoder(
    component_PrivateType* pPrivate, 
    OMX_BUFFERHEADERTYPE* inputbuffer, 
    OMX_BUFFERHEADERTYPE* outputbuffer,
    OMX_S32 *piDecoderSampleRate,
    OMX_S32 *piDecoderChannels
)
{
	OMX_S32 ret = 0;
	ThirdPartyAudio_t *pDecoder = (ThirdPartyAudio_t *)pPrivate->pDecoder;
	
	LOGD( "ThirdParty Decoder Init\n");
	
	//TODO:	add additional initialization code here

			
	////////////////////////////////////////////////////////////////////////////////////
	
	return ret;
}

OMX_S32 ThirdPartyAudio_DecodeFrame(
	component_PrivateType* pPrivate,
    OMX_U8  *pucInput, 		 	 // (i) input stream buffer pointer
    OMX_S32 iInputLength,		 // (i) input stream length (in bytes)
    OMX_PTR *pOutBuff, 			 // (i) output pcm buffer pointer
    OMX_S32 *piUsedBytes, 		 // (o) used bytes (number of bytes used to decode)
    OMX_S32 *piDecodedSamples,	 // (o) number of total samples decoded
    OMX_S32 *piOutSampleRate,	 // (o) samplerate of the current frame decoded
    OMX_S32 *piOutChannels		 // (o) number of channel
)
{
	OMX_S32 ret = 0;
	ThirdPartyAudio_t *pDecoder = (ThirdPartyAudio_t *)pPrivate->pDecoder;
	
	OMX_S32 iUsedBytes, iDecodedSamples = 0;
	
	*piDecodedSamples = 0;	
	
	//LOGD( "ThirdParty Decoder\n");
	
	//TODO: add frame decoding code here

	
	////////////////////////////////////////////////////////////////////////////////////	
	
	iUsedBytes = ;   // iInputLength - iBytesLeft
	*piUsedBytes = ; // iUsedBytes; 
	
	*piDecodedSamples = ; // iDecodedSamples;
	
	*piOutSampleRate = ;
	*piOutChannels = ;
		
	return ret;
}

OMX_S32 ThirdPartyAudio_FlushDecoder(component_PrivateType* pPrivate)
{
	ThirdPartyAudio_t *pDecoder = (ThirdPartyAudio_t *)pPrivate->pDecoder;
	
	LOGD( "ThirdParty Decoder Flush\n");
	//TODO: add flushing code here
		
	
	////////////////////////////////////////////////////////////////////////////////////	
	
	return 0;
}

OMX_S32 ThirdPartyAudio_CloseDecoder(component_PrivateType* pPrivate)
{
	ThirdPartyAudio_t *pDecoder = (ThirdPartyAudio_t *)pPrivate->pDecoder;
	
	LOGD( "Close ThirdParty Decoder\n");
	
	if(pDecoder != NULL)
	{
		//TODO: add additional closing code here
		
		////////////////////////////////////////////////////////////////////////////////////
		
		free(pDecoder);
	}
	
	return 0;
}








///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// component port setting
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
void Port_Setting(component_PrivateType* pPrivate)
{
	omx_base_audio_PortType *inPort,*outPort;
	
	/** parameters related to input port */
	inPort = (omx_base_audio_PortType *) pPrivate->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	  
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE*2;   
	
	//TODO: modify mime-type name below to fit your decoder
	strcpy(inPort->sPortParam.format.audio.cMIMEType, "audio/xxx");
	////////////////////////////////////////////////////////////////////////////////////
	
	inPort->sPortParam.format.audio.eEncoding = audio_coding_type;
	inPort->sAudioParam.eEncoding = audio_coding_type;
	
	/** parameters related to output port */
	outPort = (omx_base_audio_PortType *) pPrivate->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
	outPort->sPortParam.nBufferSize = AUDIO_DEC_OUT_BUFFER_SIZE;
	outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

    //Default values for codec audio param port
	setHeader(&pPrivate->pAudio, sizeof(audio_param_type));
    pPrivate->pAudio.nPortIndex = 0;
    pPrivate->pAudio.nChannels = 2;
    pPrivate->pAudio.nBitRate = 0;    
    pPrivate->pAudio.nSamplingRate = 44100;                     
  	//pPrivate->pAudio.nSampleRate = 44100;
    pPrivate->pAudio.eChannelMode = OMX_AUDIO_ChannelModeStereo;
    
	/** settings of output port audio format - pcm */
	setHeader(&pPrivate->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
	pPrivate->pAudioPcmMode.nPortIndex = 1;
	pPrivate->pAudioPcmMode.nChannels = 2;
	pPrivate->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
	pPrivate->pAudioPcmMode.eEndian = OMX_EndianLittle;
	pPrivate->pAudioPcmMode.bInterleaved = OMX_TRUE;
	pPrivate->pAudioPcmMode.nBitPerSample = 16;
	pPrivate->pAudioPcmMode.nSamplingRate = 44100;
	pPrivate->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
	pPrivate->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
	pPrivate->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;
	
}
