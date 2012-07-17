/**
  @file encoder.c
  
  This component implements All Video encoder. (MPEG4/AVC/H.263)

  Copyright (C) 2009-2011 Telechips Inc.

  Date: 2011/02/25 13:33:29
  Author $Author: B070371 (ZzaU)
*/


#include <encoder.h>
#include <venc.h>
#include <mach/TCC_VPU_CODEC.h>
#include <sys/types.h>

#define LOG_TAG	"DIRECT_ENC"
#include <utils/Log.h>
#include <libpmap/pmap.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>

static int DEBUG_ON  = 0;
#define DBug_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}


//#define CHANGE_BITRATE  //to change bitrate.
#ifdef CHANGE_BITRATE
#define REQUEST_INTRAR_EFRESH  //to request I-Frame.
#endif



/***********************************************************/
//INTERNAL VARIABLE
/***********************************************************/
//INTERNAL VARIABLE
typedef struct enc_private_data {
//enc operation
	venc_init_t			gsVEncInit;
	venc_seq_header_t	gsVEncSeqHeader;
	venc_input_t		gsVEncInput;
	venc_output_t		gsVEncOutput;
	unsigned char  		isVPUClosed;

//info
	unsigned char 		iConfigDataFlag;
	unsigned char 		video_coding_type;
	int					curr_frameRate;
	int					curr_targetKbps;	
	int					use_NalStartCode;
	int					qp_value;
	
//error process
	unsigned char		isEncError;
	pmap_t mVideomap;
	int mFd;
	void* mMapInfo;
}tENC_PRIVATE;


static tENC_PRIVATE *enc_private;
static unsigned int total_count = 0;
static unsigned int restred_count = 0;


/***********************************************************/
//INTERNAL FUNCTION
static void mem_prepare() {
	int dev_fd = -1;

	memset(&enc_private->mVideomap, 0, sizeof(pmap_t));

	pmap_get_info("video", &enc_private->mVideomap);

	enc_private->mFd = -1;
	enc_private->mMapInfo = MAP_FAILED;
	enc_private->mFd = open("/dev/tmem", O_RDWR | O_NDELAY);

	if (enc_private->mFd < 0) {
		memset(&enc_private->mVideomap, 0, sizeof(pmap_t));
		return;
	}

	enc_private->mMapInfo = (void*)mmap(0, enc_private->mVideomap.size, PROT_READ | PROT_WRITE, MAP_SHARED, enc_private->mFd, enc_private->mVideomap.base);

	if(MAP_FAILED == enc_private->mMapInfo)
	{
		LOGE("mmap failed. fd(%d), base addr(0x%x), size(%d)", enc_private->mFd, enc_private->mVideomap.base, enc_private->mVideomap.size);
		memset(&enc_private->mVideomap, 0, sizeof(pmap_t));
		close(enc_private->mFd);
		enc_private->mFd = -1;
	}
}

static void mem_destory() {
	int dev_fd = -1;

	if (enc_private->mMapInfo != MAP_FAILED) {
		munmap((void*)enc_private->mMapInfo, enc_private->mVideomap.size);
		enc_private->mMapInfo = MAP_FAILED;
	}

	if (enc_private->mFd >= 0) {
		close(enc_private->mFd);
		enc_private->mFd = -1;
	}

	memset(&enc_private->mVideomap, 0, sizeof(pmap_t));
}

static int mem_get_addr(unsigned int *out_virtaddr, unsigned int in_phyaddr) {
	if (enc_private->mMapInfo != MAP_FAILED) {
		*out_virtaddr = enc_private->mMapInfo + (in_phyaddr - enc_private->mVideomap.base);
		return 1;
	}

	return 0;
}

#ifdef CHANGE_QP_FOR_IFRAME
int getInitQpND(int bitrate, int width, int height)
{
    int picQp;
    int MbNumPic;
    int bit_64 = bitrate;

    MbNumPic = (width/16 * height/16) / 8;
    picQp = 40 - bit_64 / MbNumPic;
    if (picQp < 20)
           picQp = 20;
    else
           picQp =picQp;

    return picQp;
}
#endif


static int enc_init(tENC_PRIVATE *enc_private)
{
	int ret;

	LOGI("ENC Info :: %dx%d, %d fps, %d interval, %d Kbps", enc_private->gsVEncInit.m_iPicWidth, enc_private->gsVEncInit.m_iPicHeight, 
					enc_private->gsVEncInit.m_iFrameRate, enc_private->gsVEncInit.m_iKeyInterval, enc_private->gsVEncInit.m_iTargetKbps);
	
	if( (ret = venc_vpu( VENC_INIT, NULL, &enc_private->gsVEncInit, (void*)&enc_private->use_NalStartCode )) < 0 )
	{
		LOGE( "[Err:%d] VENC_INIT failed", ret );

		if(ret != -VPU_ENV_INIT_ERROR)
			venc_vpu( VENC_CLOSE, NULL, NULL, NULL );
		
		return -1;
	}

	enc_private->isVPUClosed = 0;
	total_count = 0;

	return 0;
}


/***********************************************************/
//EXTERNAL FUNCTION

/*!
 ***********************************************************************
 * \brief
 *		ENCODER_INIT	: initial function of video encoder
 * \param
 *		[in] pInit			: pointer of encoder initial parameters 
 * \return
 *		If successful, ENCODER_INIT returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
int ENCODER_INIT
(
	tENC_INIT_PARAMS *pInit
)
{
	int ret = 0;
	
	enc_private = calloc(1, sizeof(tENC_PRIVATE));
	if(enc_private == NULL)
	{
		return -1;
	}

//variable init
	memset(enc_private, 0x00, sizeof(tENC_PRIVATE));
	enc_private->isVPUClosed = 1;

	switch(pInit->codecFormat)
	{
		case CODEC_FORMAT_H263:	 enc_private->video_coding_type = STD_H263;  	break;
		case CODEC_FORMAT_MPEG4: enc_private->video_coding_type = STD_MPEG4;	break;
		case CODEC_FORMAT_H264:  enc_private->video_coding_type = STD_AVC;		break;
		default: return -1;
	}

	enc_private->gsVEncInit.m_iBitstreamFormat	= enc_private->video_coding_type;
	enc_private->gsVEncInit.m_iPicWidth			= pInit->picWidth;
	enc_private->gsVEncInit.m_iPicHeight 		= pInit->picHeight;
	enc_private->gsVEncInit.m_iFrameRate 		= enc_private->curr_frameRate  = pInit->frameRate;
	enc_private->gsVEncInit.m_iTargetKbps		= enc_private->curr_targetKbps = pInit->targetKbps;
	enc_private->gsVEncInit.m_iKeyInterval		= pInit->keyFrameInterval;
	enc_private->gsVEncInit.m_iAvcFastEncoding	= 0;
	enc_private->gsVEncInit.m_iSliceMode		= pInit->sliceMode;
	enc_private->gsVEncInit.m_iSliceSizeMode	= pInit->sliceSizeMode;
	enc_private->gsVEncInit.m_iSliceSize		= pInit->sliceSize * 8; // to change to bits.

	enc_private->use_NalStartCode 				= pInit->use_NalStartCode;
	enc_private->qp_value 						= 0;
	
	ret = enc_init(enc_private);
	mem_prepare();
	restred_count = 0;

    if( ret < 0) {
        enc_private->isEncError = 1;
    }
	
	return ret;
}

/*!
 ***********************************************************************
 * \brief
 *		ENCODER_CLOSE	: close function of video encoder
 ***********************************************************************
 */
int ENCODER_CLOSE(void)
{
	int ret= 0;
	
	if(	enc_private->isVPUClosed == 0)
	{
		if( (ret = venc_vpu( VENC_CLOSE, NULL, NULL, NULL )) < 0 )
		{
			LOGE( "[Err:%d] VENC_CLOSE failed", ret );
		}
		enc_private->isVPUClosed = 1;
	}

	mem_destory();
	
	if(enc_private) {
		free(enc_private);
		enc_private = NULL;
	}	

	return ret;
}

/*!
 ***********************************************************************
 * \brief
 *		ENCODER_ENC	: encode function of video encoder
 * \param
 *		[in] pInput			: pointer of encoder frame input parameters  
 * \param
 *		[out] pOutput			: pointer of encoder frame output parameters  
 * \return
 *		If successful, ENCODER_ENC returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
 static void save_decoded_frame(unsigned char* Y, unsigned char* U, unsigned char *V, int width, int height)
{
	FILE *pFs = NULL;
	char name[100];
		

		if(!pFs){
			pFs = fopen("/mnt/sdcard/tflash/frame.yuv.raw", "ab+");
			if (!pFs) {
				LOGE("Cannot open '%s'",name);
				return;
			}
		}
		if(pFs){
			fwrite( Y, width*height, 1, pFs);
			fwrite( U, width*height/4, 1, pFs);
			fwrite( V, width*height/4, 1, pFs);
		}
		close(pFs);
}
int ENCODER_ENC
(
	tENC_FRAME_INPUT *pInput,
	tENC_FRAME_OUTPUT *pOutput
)
{	
	int i;
	signed int ret;
	unsigned int *InputCurrBuffer;
	unsigned int addr_cvt;

	memset(pOutput, 0x00, sizeof(tENC_FRAME_OUTPUT));

	if(enc_private->isEncError == 1)
	{
		LOGI("%d'th Restore Encode-Error", restred_count);
		
		if((ret = enc_init(enc_private)) < 0)
			goto ERR_PROCESS;
		
		enc_private->isEncError = 0;
		enc_private->iConfigDataFlag = 0;
	}

	if(enc_private->isVPUClosed == 1)
	{
		LOGE("Vpu already is closed because of Error!!");
		return -1;
	}

//Send the first output buffer such as CodecSpecificData
	if(enc_private->iConfigDataFlag == 0)
	{
		enc_private->gsVEncSeqHeader.m_SeqHeaderBuffer[PA]	= 0;
		enc_private->gsVEncSeqHeader.m_SeqHeaderBuffer[VA]	= 0;
		enc_private->gsVEncSeqHeader.m_iSeqHeaderBufferSize = 0;
		
		if (enc_private->video_coding_type == STD_MPEG4) 
		{		
			ret = venc_vpu( VENC_SEQ_HEADER, NULL, &enc_private->gsVEncSeqHeader, NULL );
			if( ret < 0 )
			{
				LOGE( "[Err:-0x%x] VENC_SEQ_HEADER failed\n", -ret );
				enc_private->isEncError = 1;
				goto ERR_PROCESS;
			}

			pOutput->outputStreamAddr = enc_private->gsVEncSeqHeader.m_pSeqHeaderOut;
			pOutput->headerLen = enc_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
			pOutput->nTimeStamp = 0;
			//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
			//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
			DBug_MSG(" VOL Header :: Length = %d !!", pOutput->headerLen);
		}
		else if (enc_private->video_coding_type == STD_AVC) 
		{
			ret = venc_vpu( VENC_SEQ_HEADER, NULL, &enc_private->gsVEncSeqHeader, NULL );
			if( ret < 0 )
			{
				LOGE( "[Err:-0x%x] VENC_SEQ_HEADER failed\n", -ret );
				enc_private->isEncError = 1;
				goto ERR_PROCESS;
			}
			DBug_MSG(" SPS - %d !!", enc_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);		

			pOutput->outputStreamAddr = enc_private->gsVEncSeqHeader.m_pSeqHeaderOut;
			pOutput->headerLen = enc_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
			pOutput->nTimeStamp = 0;
			//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
			//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
			
			ret = venc_vpu( VENC_SEQ_HEADER, NULL, &enc_private->gsVEncSeqHeader, NULL );
			if( ret < 0 )
			{
				LOGE( "[Err:-0x%x] VENC_SEQ_HEADER failed\n", -ret );
				enc_private->isEncError = 1;
				goto ERR_PROCESS;
			}
			DBug_MSG(" PPS - %d !!", enc_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);		
			pOutput->headerLen +=  enc_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
			DBug_MSG(" CodecConfig Data :: %d !!", pOutput->headerLen);
		}
		
		enc_private->iConfigDataFlag = 1;
	}
	else
	{
		enc_private->gsVEncInput.m_bCbCrInterleaved = 0;

		if(pInput->noIncludePhyAddr == 0)
		{
			InputCurrBuffer = (unsigned int*)pInput->inputStreamAddr;
			enc_private->gsVEncInput.m_pInputY 		= (unsigned char*)InputCurrBuffer[0];
		}
		else
		{
			memcpy(vpu_get_BitstreamBufAddr(VA), pInput->inputStreamAddr, (enc_private->gsVEncInit.m_iPicWidth*enc_private->gsVEncInit.m_iPicHeight*3)/2);
			enc_private->gsVEncInput.m_pInputY 		= vpu_get_BitstreamBufAddr(PA);
		}
		enc_private->gsVEncInput.m_pInputCbCr[0]	= enc_private->gsVEncInput.m_pInputY + (enc_private->gsVEncInit.m_iPicWidth*enc_private->gsVEncInit.m_iPicHeight);
		enc_private->gsVEncInput.m_pInputCbCr[1]	= enc_private->gsVEncInput.m_pInputCbCr[0] + (enc_private->gsVEncInit.m_iPicWidth*enc_private->gsVEncInit.m_iPicHeight/4);
		
#ifdef CHANGE_BITRATE
		unsigned int change_period = 25;
		if(total_count == change_period*10)
			total_count = change_period;

		if(total_count == change_period)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 896*1024;
		}
		else if(total_count == change_period*2)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 768*1024;
		}
		else if(total_count == change_period*3)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 640*1024;
		}
		else if(total_count == change_period*4)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 512*1024;
		}
		else if(total_count == change_period*5)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 384*1024;
		}
		else if(total_count == change_period*6)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 256*1024;
		}
		else if(total_count == change_period*7)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 192*1024;
		}
		else if(total_count == change_period*8)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 128*1024;
		}
		else if(total_count == change_period*9)
		{
			enc_private->VideoConfigBitRateType.nEncodeBitrate = 64*1024;
		}

#ifdef REQUEST_INTRAR_EFRESH
		if(total_count%17 == 0 ){
			enc_private->VideoIFrame.IntraRefreshVOP = 1;
		}
#endif
#endif
		total_count++;	

#ifdef REMOVE_RC_AUTO_SKIP
		enc_private->gsVEncInput.m_iChangeRcParamFlag = (0x20 | 0x1);
#else
		enc_private->gsVEncInput.m_iChangeRcParamFlag = 0;
#endif

#ifdef CHANGE_QP_FOR_IFRAME
		if( enc_private->qp_value == 0 && !(pInput->targetKbps != 0 && pInput->targetKbps != enc_private->curr_targetKbps))
		{
			enc_private->gsVEncInput.m_iQuantParam = getInitQpND(enc_private->curr_targetKbps, enc_private->gsVEncInit.m_iPicWidth, enc_private->gsVEncInit.m_iPicHeight);
            enc_private->gsVEncInput.m_iChangeRcParamFlag = (0x10 | 0x1);
			enc_private->qp_value = enc_private->gsVEncInput.m_iQuantParam;
			LOGE("QP for I-Frame - Initial value(%d)", enc_private->gsVEncInput.m_iQuantParam);
		}
#endif	

		if(pInput->targetKbps != 0 && pInput->targetKbps != enc_private->curr_targetKbps)
		{
			enc_private->curr_targetKbps = pInput->targetKbps;
			
			enc_private->gsVEncInput.m_iChangeRcParamFlag |= (0x2 | 0x1);
			enc_private->gsVEncInput.m_iChangeTargetKbps = enc_private->curr_targetKbps;
			LOGE("Bitrate- Change(%d)", enc_private->gsVEncInput.m_iChangeTargetKbps);
#ifdef CHANGE_QP_FOR_IFRAME
			enc_private->gsVEncInput.m_iQuantParam = getInitQpND(enc_private->curr_targetKbps, enc_private->gsVEncInit.m_iPicWidth, enc_private->gsVEncInit.m_iPicHeight);
            enc_private->gsVEncInput.m_iChangeRcParamFlag = (0x10 | 0x1);
			LOGE("QP for I-Frame - Change(%d)", enc_private->gsVEncInput.m_iQuantParam);
#endif			
		}
		else if(pInput->frameRate != 0 && pInput->frameRate != enc_private->curr_frameRate)
		{
			enc_private->curr_frameRate = pInput->frameRate;
			
			enc_private->gsVEncInput.m_iChangeRcParamFlag |= (0x4 | 0x1);
			enc_private->gsVEncInput.m_iChangeFrameRate = enc_private->curr_frameRate;
			LOGE("FrameRate- Change(%d)", enc_private->gsVEncInput.m_iChangeFrameRate);
		}

		if(pInput->isForceIFrame){
			enc_private->gsVEncInput.request_IntraFrame = 1;
			LOGE("IntraRefreshVOP");
		}
		else{
			enc_private->gsVEncInput.request_IntraFrame = 0;
		}
			
		ret = venc_vpu( VENC_ENCODE, NULL, &enc_private->gsVEncInput, &enc_private->gsVEncOutput );
		if( ret < 0 )
		{
			enc_private->isEncError = 1;						
			
			LOGE( "[Err:-0x%x] VENC_ENCODE failed\n", -ret );
			goto ERR_PROCESS;
		}
		
		pOutput->frameLen = enc_private->gsVEncOutput.m_iBitstreamOutSize;
		pOutput->outputStreamAddr = enc_private->gsVEncOutput.m_pBitstreamOut;
		pOutput->picType = enc_private->gsVEncOutput.m_iPicType;		
		pOutput->nTimeStamp = pInput->nTimeStamp;

		pOutput->m_iSliceCount = enc_private->gsVEncOutput.m_iSliceCount;
		pOutput->m_pSliceInfo = enc_private->gsVEncOutput.m_pSliceInfo;

		if(pOutput->m_iSliceCount > 1 && DEBUG_ON == 1)
		{
			unsigned char *p = pOutput->outputStreamAddr;
			unsigned int *pSliceSize = pOutput->m_pSliceInfo;
			unsigned int total_bytes = 0;
			
			for( i=0; i<pOutput->m_iSliceCount; i++ ) 
			{
				if(total_count < 5 )
				{
					DBug_MSG( "[%2d/%2d] %d bits, %d bytes ", i, pOutput->m_iSliceCount, pSliceSize[i]*8, pSliceSize[i] );
					DBug_MSG( " 	 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", p[total_bytes+0], p[total_bytes+1], p[total_bytes+2], 
							 p[total_bytes+3], p[total_bytes+4], p[total_bytes+5], p[total_bytes+6], p[total_bytes+7]);
					total_bytes += pSliceSize[i];
					DBug_MSG( "  ~	 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", p[total_bytes-8], p[total_bytes-7], p[total_bytes-6], 
							 p[total_bytes-5], p[total_bytes-4], p[total_bytes-3], p[total_bytes-2], p[total_bytes-1]);
				}
			}
		}
	
		 // IFrame (SyncFrame)
		if( (enc_private->gsVEncOutput.m_iPicType == VENC_PIC_TYPE_I) && pOutput->frameLen > 0)
		{
			DBug_MSG(" I-Frame for Sync :: Frm_size = %d !!", pOutput->frameLen);

			//This flag is set when the buffer content contains a coded sync frame 
			//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
		}
		//Attach the end of frame flag while sending out the last piece of output buffer
		//pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

		//In case of vpu's error, composer can not access vpu's address.
		if(1 == mem_get_addr(&addr_cvt, vpu_getStreamOutPhyAddr(pOutput->outputStreamAddr, VA)))
		{
			pOutput->outputStreamAddr = addr_cvt;
		}
	}

	return 0;
	
ERR_PROCESS:
	if(	enc_private->isVPUClosed == 0)
	{
		venc_vpu( VENC_CLOSE, NULL, NULL, NULL );
		enc_private->isVPUClosed = 1;
		pOutput->headerLen = pOutput->frameLen = 0;
	}

	return ret;
}

#ifdef TEMP_FOR_LINPHONE_VENC
void Display_Stream(unsigned char *p, int size)
{
	int i;
	unsigned char* ps = p;
	for(i=0; (i+10 <size) && (i+10 < 100); i += 10){
		LOGE( "[VENC - Stream] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", ps[i], ps[i+1], ps[i+2], ps[i+3], ps[i+4], ps[i+5], ps[i+6], ps[i+7], ps[i+8], ps[i+9] );
	}
}	

int TCC_VENC_Init_H264(unsigned int *pInitParam)
{
	int ret = 0;
	tENC_INIT_PARAMS InitParam;

	InitParam.codecFormat		= CODEC_FORMAT_H264;
	InitParam.picWidth			= pInitParam[0];
	InitParam.picHeight			= pInitParam[1];
	InitParam.frameRate			= pInitParam[3];
	InitParam.targetKbps 		= pInitParam[2] >> 10;		
	InitParam.keyFrameInterval	= pInitParam[4];		
	InitParam.sliceMode			= 0;		
	InitParam.sliceSizeMode		= 0; 		
	InitParam.sliceSize			= 0; 	
	InitParam.use_NalStartCode	= 1;

	ret = ENCODER_INIT(&InitParam);
	if(ret<0)
		return -1;

	return ret;
}

static unsigned int framecnt = 0;
int TCC_VENC_Enc(unsigned int *pInputStream, unsigned int *pOutstream)																				
{
	int ret = 0;
	tENC_FRAME_INPUT InputStream;
	tENC_FRAME_OUTPUT OutputStream;

	
	InputStream.inputStreamAddr = (unsigned char *)pInputStream[0];	/* Memory pointer that has the physical address of input raw data (memory pointer received from camera callback) */

	InputStream.noIncludePhyAddr = pInputStream[2];   /* Whether inputStreamAddr is virtual address including raw data by itself.*/
	                                    /* if this set into 1, raw data will be copied into physical memory region to encode using H/W block. It will be decreased performance.*/
	InputStream.inputStreamSize = pInputStream[1];	/* Bytes : length of input data */
	InputStream.nTimeStamp = 0;			/* TimeStamp of input data, by ms */	
	InputStream.isForceIFrame = pInputStream[4]; 
	InputStream.isSkipFrame = 0;;		/* Whether skip (do not encode) current frame */
	InputStream.frameRate = pInputStream[5];			/* to change FrameRate during encoding, It is ignored if zero  */
	InputStream.targetKbps = pInputStream[3];     /* to change Bitrate for output stream during encoding, by Kbps, It is ignored if zero */	
	InputStream.Qp =0;

	ret = ENCODER_ENC(&InputStream,&OutputStream);

	pOutstream[0] = (unsigned int)OutputStream.outputStreamAddr; 	/* Base address for output bitstream (virtual address)*/
	pOutstream[1] = OutputStream.picType;							/* Picture coding type */
	pOutstream[2] = OutputStream.nTimeStamp;						/* TimeStamp of output bitstream, by ms */	
	pOutstream[3] = OutputStream.headerLen;							/* Bytes of header */
	pOutstream[4] = OutputStream.frameLen;							/* Bytes of frame encoded */
	pOutstream[5] = OutputStream.m_iSliceCount;						/* total slice's count that one encoded frame have */
	pOutstream[6] = (unsigned int)OutputStream.m_pSliceInfo;

#if 0
	if(OutputStream.headerLen)
		Display_Stream(OutputStream.outputStreamAddr,OutputStream.headerLen);
	else
		Display_Stream(OutputStream.outputStreamAddr,OutputStream.frameLen);
#endif

	if(ret<0)
		return -1;
	
	return ret;

}

int TCC_VENC_Close(void)
{
	int ret = 0;
	
	ret = ENCODER_CLOSE();

	return ret;
}
#endif
