
#ifndef _THIRD_PARTY_AUDIO_DECODER__H_
#define _THIRD_PARTY_AUDIO_DECODER__H_


void* ThirdPartyAudio_OpenDecoder(
    component_PrivateType* pPrivate
);

OMX_S32 ThirdPartyAudio_InitDecoder(
    component_PrivateType* pPrivate, 
    OMX_BUFFERHEADERTYPE* inputbuffer, 
    OMX_BUFFERHEADERTYPE* outputbuffer,
    OMX_S32 *piDecoderSampleRate,
    OMX_S32 *piDecoderChannels
);

OMX_S32 ThirdPartyAudio_DecodeFrame(
	component_PrivateType* pPrivate,
    OMX_U8  *pucInput, 		 	 // (i) input stream buffer pointer
    OMX_S32 iInputLength,		 // (i) input stream length (in bytes)
    OMX_PTR *pOutBuff, 			 // (i) output pcm buffer pointer
    OMX_S32 *piUsedBytes, 		 // (o) used bytes (number of bytes used to decode)
    OMX_S32 *piDecodedSamples,	 // (o) number of total samples decoded
    OMX_S32 *piOutSampleRate,	 // (o) samplerate of the current frame decoded
    OMX_S32 *piOutChannels		 // (o) number of channel
);

OMX_S32 ThirdPartyAudio_FlushDecoder(
    component_PrivateType* pPrivate
);

OMX_S32 ThirdPartyAudio_CloseDecoder(
    component_PrivateType* pPrivate
);

void Port_Setting(component_PrivateType* pPrivate);

#endif /* _THIRD_PARTY_AUDIO_DECODER__H_ */

