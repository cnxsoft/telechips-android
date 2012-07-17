

#if defined(AAC_ENC_INCLUDE) || defined(VIDEO_AAC_ENC_INCLUDE)
#include "main.h"
#include "globals.h"

#include "TCCresource.h"

#include "IO_TCCXXX.h"
#include "buffer/buffer.h"
#include "include/CommonCmd.h"
#include "kernel/telechips.h"

#include "fs/FSAPP.h"
#include "utils.h"
#include "Effect/Visual.h"

//#include "tbench.h"

#include "TCC7xx_Mp4AacEnc_API.h"

#if defined(AAC_ENC_INCLUDE)
#include "filebuffer/simplebuffer.h"
extern SRBUFFER gRecodeBuffer;
#endif

extern unsigned fmemcpy16(void *dest, void *src, unsigned length);
extern int InitializeAudioFIFO(void);

#define AAC_SAMPLES_PER_FRAME 			1024 

#define	AAC_PCM_BUFFER_SIZE				(AAC_SAMPLES_PER_FRAME*6)
#define	AAC_ENCODED_DATA_SIZE			6144
#define	AAC_ENCODED_DATA_WRITE_SIZE		1024

#define	AAC_LEFT_PCM_BUFFER				pAACEnc->psLeft
#define	AAC_RIGHT_PCM_BUFFER			pAACEnc->psRight

//****************************************************************************
// A structure which defines the persistent state of the AAC Encoder.
//****************************************************************************
typedef struct
{

	// A pointer to the buffer containing the persistent internal state of the
	// MP3 encoder library.
	//tAACEncInstance *pAACEncInstance;

	// A buffer to contain the encoded MP3 audio.
	#if 1 
	char pcEncodedData[AAC_ENCODED_DATA_SIZE];
	#endif

	// Buffers to contain the ADC samples
	short psLeft[AAC_PCM_BUFFER_SIZE];
	short psRight[AAC_PCM_BUFFER_SIZE];

	// The buffer to which we write encoded AAC data.
	BufferState *pOutput;

	// The sample rate of the decoded PCM stream.
	unsigned short usSampleRate;

	// The number of channels in the file.
	unsigned char ucChannels;

	// The bit rate of the AAC file.
	unsigned long ulBitRate;

	// The number of samples which have been encoded. //decoded.
	unsigned long ulTimePos;

} tAACENC;

tOutputFormat gTestAACOutputFormat; //FORMAT_ADIF;//FORMAT_ADTS;
unsigned int  gTestAACTurnOnPns; //E_OFF;// E_ON;//
unsigned int  gTestAACTurnOnTns; //E_OFF;// E_ON;//
unsigned int  gTestVersionInfo; 

void *pAACHandle = NULL;

unsigned long gAACEncodedBuffIdx;

short int MergeBuffer[AAC_SAMPLES_PER_FRAME * 2]; // "2" means 2channel

//#define DEBUG_ENCODING_TIME_LOG
//#define DEBUG_PCMDATA_TO_ENCODE_LOG
#if 0//defined(DEBUG_ENCODING_TIME_LOG) || defined(DEBUG_PCMDATA_TO_ENCODE_LOG)
#define MAX_FRAMES_TO_LOG 1024
int EncodeTimeIndex;
#endif

#if 0//def DEBUG_ENCODING_TIME_LOG
unsigned EncodeCurrTime;
unsigned EncodePrevTime;
unsigned EncodeTime[MAX_FRAMES_TO_LOG];
unsigned EncodeDuration[MAX_FRAMES_TO_LOG];
#endif

#if 0//def DEBUG_PCMDATA_TO_ENCODE_LOG
unsigned LOGPCMLeftBufPtr[MAX_FRAMES_TO_LOG];
unsigned LOGPCMRightBufPtr[MAX_FRAMES_TO_LOG];
unsigned LOGPCMLength[MAX_FRAMES_TO_LOG];
#endif

//#define DEBUG_ENCODED_DATA_LOG
#if 0//def DEBUG_ENCODED_DATA_LOG
#define AACBUFFERSIZE (1024*1024)
char AACBUFFER[AACBUFFERSIZE];
int AACBUFFEINDEX=0;
#endif

static long AACENC_MergePcmBuff( short *psLeft, short *psRight, short *pMergeBuff, long lSampleLength)
{
	long lSampleCount;

	short *psLeftReadPtr, *psRightReadPtr;

	psLeftReadPtr  = psLeft;
	psRightReadPtr = psRight;

	lSampleCount = 0L;
	do {
		*(pMergeBuff++) = *psLeftReadPtr++;
		*(pMergeBuff++) = *psRightReadPtr++;

		lSampleCount++;
	} while( (lSampleCount<lSampleLength) );

	return(lSampleCount);
}

//****************************************************************************
//
// The codec plug-in entry point for the AAC decoder.
//
//****************************************************************************
unsigned long AACEncodeFunction(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4)
{
#if	0
	tInt32 ret;
	
	switch(ulSubFn)
	{
		// Prepare the codec to encode a file.
		case SUBFN_CODEC_OPEN_ENC:
		{
			tAACENC *pAACEnc;
			tMp4AacEncParams stAACParam;

			gStartRec	= 0;

			// The first parameter is a pointer to the MP3 persistent state.
			pAACEnc = (tAACENC *)ulParam1;
			pAACEnc->ulTimePos = 0;
			pAACEnc->pOutput = NULL;

			// clear encoded data buffer 
			memset( pAACEnc->pcEncodedData, 0, AAC_ENCODED_DATA_SIZE );

			gAACEncodedBuffIdx = 0L;

			pAACEnc->ulBitRate = 96000;//128000;//REC_GetgBitrate()*1000;
			pAACEnc->usSampleRate = 44100;//REC_GetSamplingFreq();
			pAACEnc->ucChannels = 2;//REC_GetChannel();

			stAACParam.NumberOfChannels  = pAACEnc->ucChannels;
			stAACParam.OutputBitRate     = pAACEnc->ulBitRate;
			stAACParam.OutputFormat      = gTestAACOutputFormat = FORMAT_RAW;//FORMAT_ADTS;//
			stAACParam.SamplingFrequency = pAACEnc->usSampleRate;
			stAACParam.TurnOnPns         = gTestAACTurnOnPns = E_OFF;
			stAACParam.TurnOnTns         = gTestAACTurnOnTns = E_OFF;
			stAACParam.VersionInfo       = 1;

			ret = Mpeg4AacEnc_Create(&pAACHandle);
			if( ret < 0)
			{
				// Error in creating the encoder insance
		        PRINTF("[AACEnc]Error in creation! ret=%d\n",(int)ret);
				return	(0);
			}

			ret = Mpeg4AacEnc_Reset(pAACHandle, &stAACParam);
			if( ret < 0 )
			{
				// Error in resetting the values 
		        PRINTF("[AACEnc]Error in reset! ret=%d\n",(int)ret);
				return	(0);
			}

			#if defined(MP4_ENC_INCLUDE) && (defined(CAM_ENC_INCLUDE) && defined(AAC_ENC_INCLUDE))
			if(Audio_GetCurrentCodec() == CODEC_AAC_ENC)
				InitializeAudioFIFO();
			else
			#endif	//MP4_ENC_INCLUDE
			{
				#if !defined(CAM_ENC_INCLUDE) && defined(AAC_ENC_INCLUDE)
				char 	*pFileBuffer;
				int 	iBufferSize;
				
				pFileBuffer = FSAPP_GetFileBuffer();
				iBufferSize = FSAPP_GetMaxCopySize();
				QInitBuffer(&gRecodeBuffer,iBufferSize,pFileBuffer);
				#endif
			}

			#if 0//def DEBUG_ENCODED_DATA_LOG
			AACBUFFEINDEX = 0;
			memset ( AACBUFFER, 0, AACBUFFERSIZE );
			#endif
			
			#if 0//def DEBUG_ENCODING_TIME_LOG
			EncodeTimeIndex = 0;
			EncodeCurrTime = EncodePrevTime = IO_TMR_Get32bitValue();
			#endif

			return(1);
		}

		// Set the output buffer for the decoder.
		case SUBFN_CODEC_SETBUFFER:
		{
			tAACENC *pAACEnc;

			// The first parameter is a pointer to the AAC persistent state.
			pAACEnc = (tAACENC *)ulParam1;

			// The second parameter is a pointer to the output buffer.
			pAACEnc->pOutput = (BufferState *)ulParam2;

			{
				// Provide the output buffer with our data buffers.
				if(pAACEnc->ucChannels == 2)
				{
					BufferSetBuffer(pAACEnc->pOutput, AAC_LEFT_PCM_BUFFER, AAC_RIGHT_PCM_BUFFER, AAC_PCM_BUFFER_SIZE);
				}
				else
				{
					BufferSetBuffer(pAACEnc->pOutput, AAC_LEFT_PCM_BUFFER, AAC_LEFT_PCM_BUFFER,  AAC_PCM_BUFFER_SIZE);
				}
			}
			ucBufFull	= 0;

			// Success.
			return(1);
		}

		// Encode a frame of data.
		case SUBFN_CODEC_ENCODE:
		{
			tAACENC *pAACEnc;
			short *psLeft,*psRight;
			long  lLength;
			long  lLen;
			unsigned	uCPSR, uBSize;
			#ifdef	MP4_ENC_INCLUDE
			int AudioTimer, iRet;
			unsigned int *DestAddr;

			
			#endif	//MP4_ENC_INCLUDE
			int aDstLen;

                        #ifdef	MP4_ENC_INCLUDE
// airrock-test
//			TC_TimeDly(100);
//			return 1;
                        #endif  //MP4_ENC_INCLUDE

			pAACEnc = (tAACENC *) ulParam1;

			if (!gStartRec)
			{
				// Make Header End
				if ( gTestAACOutputFormat == FORMAT_ADIF )
				{
					// Make Header
					aDstLen = AAC_ENCODED_DATA_SIZE-gAACEncodedBuffIdx;
					ret = Mpeg4AacEnc_GetHeader( pAACHandle, (pAACEnc->pcEncodedData+gAACEncodedBuffIdx), &aDstLen );
					if( ret < 0)
					{
						// Error in creating the encoder insance
				        PRINTF("[AACEnc]Error in GetHeader! ret=%d\n",(int)ret);
						aDstLen = 0;
						return	(0);
					}
					gAACEncodedBuffIdx += aDstLen;
					
				}			

				uCPSR	= IO_INT_DisableINT();
				pAACEnc->pOutput->lReadPtr	= 0;
				pAACEnc->pOutput->lWritePtr	= 0;
				IO_INT_RestoreINT(uCPSR);
				gStartRec	= 1;
				ucBufFull	= 0;
			}

			lLen = AAC_SAMPLES_PER_FRAME;

			while((uBSize = BufferDataAvailable(pAACEnc->pOutput)) < lLen )
			{
				TC_TimeDly(1);
			}

			BufferGetReadPointer(pAACEnc->pOutput, &psLeft, &psRight,&lLength);

			#if 0//def DEBUG_PCMDATA_TO_ENCODE_LOG
			LOGPCMLeftBufPtr[EncodeTimeIndex] = (long)psLeft;
			LOGPCMRightBufPtr[EncodeTimeIndex] = (long)psRight;
			LOGPCMLength[EncodeTimeIndex] = (long)lLength;
			#endif

			AACENC_MergePcmBuff( psLeft, psRight, MergeBuffer, lLen);

			aDstLen = AAC_ENCODED_DATA_SIZE-gAACEncodedBuffIdx;
			
			#if 0//def DEBUG_PCMDATA_TO_ENCODE_LOG
			memcpy( &LOGPCMMergedBuffer[EncodeTimeIndex*(AAC_SAMPLES_PER_FRAME*4)], MergeBuffer, (AAC_SAMPLES_PER_FRAME*4) );
			#endif

			#if 0//def DEBUG_ENCODING_TIME_LOG
			EncodeCurrTime = IO_TMR_Get32bitValue();
			EncodeDuration[EncodeTimeIndex] = EncodeCurrTime - EncodePrevTime;
			#endif

			IO_TMR_DisableTIMER(0);

			ret = Mpeg4AacEnc_Encode( pAACHandle, MergeBuffer, (AAC_SAMPLES_PER_FRAME*2)*pAACEnc->ucChannels, (pAACEnc->pcEncodedData+gAACEncodedBuffIdx), &aDstLen );

			IO_TMR_ClearTIREQ(0);
			IO_TMR_EnableTIMER(0);

			#if 0//def DEBUG_ENCODING_TIME_LOG
			EncodeTime[EncodeTimeIndex++] = IO_TMR_Get32bitValue() - EncodeCurrTime;
			EncodePrevTime = EncodeCurrTime;
			#endif
			
			gAACEncodedBuffIdx += aDstLen;
			
			if( ret < 0)
			{
				// Error in Encoding Frame
				PRINTF("[AACEnc]Error in Encode! ret=%d\n",(int)ret);
				aDstLen = 0;
			}
			
			BufferUpdateReadPointerJS(pAACEnc->pOutput,lLen);

			#if defined(MP4_ENC_INCLUDE) && (defined(CAM_ENC_INCLUDE) && defined(AAC_ENC_INCLUDE))
			if(Audio_GetCurrentCodec() == CODEC_AAC_ENC)
			{
				AudioTimer = MAX_NUM_OF_AUDIO_ELEMENT << 2;
				while(--AudioTimer)
				{
					iRet = GetAudioFifoElement(gAudioFIFOBuffer, &pAudioFiFo);
					if(iRet)
					{
						DestAddr = pAudioFiFo->pBuf + (sizeof(ADBFRAMEINFO) >> 2);
						while(--AudioTimer)
						{
							iRet = PushAudioFifo(gAudioFIFOBuffer, pAudioFiFo, (gAACEncodedBuffIdx + HEADER_OF_AV_INDEX));
							if(iRet)
							{
								/* Insert AVI Header for AVI Containder performance */
								mem_cpy((char *)DestAddr, "01wb", 4);
								*(DestAddr+1) = gAACEncodedBuffIdx; // pAudioFiFo->uiRealBufSize
								fmemcpy16((char *)(DestAddr+2), pAACEnc->pcEncodedData, gAACEncodedBuffIdx);

								#if 0//def DEBUG_ENCODED_DATA_LOG
								if ( (AACBUFFEINDEX+gAACEncodedBuffIdx) < AACBUFFERSIZE )
								{
									memcpy( &AACBUFFER[AACBUFFEINDEX], pAACEnc->pcEncodedData, gAACEncodedBuffIdx);
									AACBUFFEINDEX += gAACEncodedBuffIdx;
								}
								else
								{
									if ( AACBUFFEINDEX < AACBUFFERSIZE )
									{
										PRINTF("[ENC] AACBUFFER Full \n");
										PRINTF("[ENC] AACBUFFEINDEX = 0x%X\n",   AACBUFFEINDEX);
										PRINTF("[ENC] AACBUFFER =     0x%08X\n", AACBUFFER);
										AACBUFFEINDEX = AACBUFFERSIZE ;
									}
								}
								#endif
								
								pADBFrameInfo = (PADBFRAMEINFO)pAudioFiFo->pBuf;
								pADBFrameInfo->iFrame = stAudioInfo.uiAudioCount;
								pADBFrameInfo->iPts = pAACEnc->ulTimePos;
								gAACEncodedBuffIdx = 0;
								stAudioInfo.uiAudioCount++;
								break;
							}
							else
							{
								PRINTF("Audio Wait Push\n");
								TC_TimeDly(1);
							}
						}
						break;
					}
					else
					{
						PRINTF("Audio AAC Wait...%d\n", stAudioInfo.uiAudioCount);
						TC_TimeDly(1);
					}
				}

				if(!AudioTimer)
				{
					PRINTF("Audio No Element\n");
					return (-1); /* Error : No Buffer */
				}

				pAACEnc->ulTimePos += lLen; 
			}
			else
			#endif	//MP4_ENC_INCLUDE
			{
				#if !defined(CAM_ENC_INCLUDE) && defined(AAC_ENC_INCLUDE)
				if(gAACEncodedBuffIdx >= AAC_ENCODED_DATA_WRITE_SIZE)
				{
					while(!QPutData(&gRecodeBuffer,pAACEnc->pcEncodedData,AAC_ENCODED_DATA_WRITE_SIZE))
					{
						TC_TimeDly(1);
					}
					gAACEncodedBuffIdx -= AAC_ENCODED_DATA_WRITE_SIZE;
					fmemcpy16(pAACEnc->pcEncodedData, pAACEnc->pcEncodedData+AAC_ENCODED_DATA_WRITE_SIZE, gAACEncodedBuffIdx);
				}
				pAACEnc->ulTimePos += lLen;
				#endif
			}
			return(1);
		}

		// Return the current position (in milliseconds) within the file.
		case SUBFN_CODEC_GETTIME:
		{
			unsigned long *pulTime;
			tAACENC *pAACEnc;

			// The first parameter is a pointer to the MP3 persistent data.
			pAACEnc = (tAACENC *)ulParam1;

			// The second parameter is a pointer for the number of seconds.
			pulTime = (unsigned long *)ulParam2;

			// Determine the time based on the sample rate.
			*pulTime = ((pAACEnc->ulTimePos / pAACEnc->usSampleRate) * 1000) +
						(((pAACEnc->ulTimePos % pAACEnc->usSampleRate) * 1000) /
						pAACEnc->usSampleRate);

			// Success.
			return(1);
		}

		// Return the sample rate at which this file is encoded.
		case SUBFN_CODEC_GETSAMPLERATE:
		{
			unsigned long *pulSampleRate;
			tAACENC *pAACEnc;

			// The first parameter is a pointer to the MP3 persistent data.
			pAACEnc = (tAACENC *)ulParam1;

			pulSampleRate = (unsigned long *)ulParam2;

			// Return the sample rate of the file.
			*pulSampleRate = pAACEnc->usSampleRate;

			// Success.
			return(1);

		}

		// Return the number of channels in the file.
		case SUBFN_CODEC_GETCHANNELS:
		{
			unsigned long *pulChannels;
			tAACENC *pAACEnc;

			// The first parameter is a pointer to the MP3 persistent data.
			pAACEnc = (tAACENC *)ulParam1;

			pulChannels = (unsigned long *)ulParam2;

			// Return the number of channels in the file.
			*pulChannels = pAACEnc->ucChannels;

			// Success.
			return(1);
		}

		// Return the bitrate at which this file is encoded.
		case SUBFN_CODEC_GETBITRATE:
		{
			unsigned long *pulBitRate;
			tAACENC *pAACEnc;

			// The first parameter is a pointer to the MP3 persistent data.
			pAACEnc = (tAACENC *)ulParam1;

			pulBitRate = (unsigned long *)ulParam2;

			// Return the number of channels in the file.
			*pulBitRate = pAACEnc->ulBitRate;

			// Success.
			return(1);

		}

		#if 0 // not needed in encoding mode
		// Return the length (in milliseconds) of the file.
		case SUBFN_CODEC_GETLENGTH:
		{
			unsigned long *pulLength;
			tMP3ENC *pMP3ENC;

			// The first parameter is a pointer to the MP3 persistent data.
			pMP3ENC = (tMP3ENC *)ulParam1;

			pulLength = (unsigned long *)ulParam2;

			// Return the length of the file.
			*pulLength = pMP3ENC->ulTimeLength;

			// Success.
			return(1);
		}
		#endif

		case SUBFN_CODEC_GETCAPTUREBUFFER:
		{
			tAACENC *pAACEnc;
			short **ppsBuffer;
			long *plLength;

			pAACEnc = (tAACENC *) ulParam1;
			
			ppsBuffer = (short **)ulParam2;
			plLength = (long *)ulParam3;

			*ppsBuffer = AAC_RIGHT_PCM_BUFFER;
			*plLength  = AAC_PCM_BUFFER_SIZE;

			//success
			return(1);	
		}

		// Delete codec handle after finish encoding.
		case SUBFN_CODEC_CLOSE:
		{
			ret = Mpeg4AacEnc_Delete(pAACHandle);

			//AAC_Reencoding();

			if( ret < 0 )
			{
				// Error in resetting the values 
		        PRINTF("[AACEnc]Error in delete! ret=%d\n",ret);
				return	(0);
			}
			gStartRec	= 0;
			return(1);
		}

		default:
		{
			// Return a failure.
			return(0);
		}
	} //end of switch(ulSubFn)
#endif	
}


#endif // defined(AAC_ENC_INCLUDE)

/* End of pAACEnc.c */
