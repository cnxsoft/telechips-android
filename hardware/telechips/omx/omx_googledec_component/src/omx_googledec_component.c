/**
  @file src/components/ffmpeg/omx_videodec_component.h
  
  This component implements Google Video decoder. (Google codec)
  The VPX Video decoder is based on the google software library.

  Copyright (C) 2007-2008 STMicroelectronics
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

  Android revised by ZzaU.
*/

#include <omxcore.h>
#include <omx_base_video_port.h>
#include <omx_googledec_component.h>
#include<OMX_Video.h>

/*
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <mach/tccfb_ioctrl.h>
*/
#define LOG_TAG	"OMX_GOOGLE_VPX_DEC"
#include <utils/Log.h>

#include "vpx/vpx_decoder.h"
#include "vpx/vpx_codec.h"
#include "vpx/vp8dx.h"

#include <cutils/properties.h>

static int DEBUG_ON = 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}

#define USE_EXTERNAL_BUFFER 1
#define USE_RB565_OUTPUT

static int GetCPUCoreCount() {
    int cpuCoreCount = 1;
#if defined(_SC_NPROCESSORS_ONLN)
    cpuCoreCount = sysconf(_SC_NPROCESSORS_ONLN);
#else
    // _SC_NPROC_ONLN must be defined...
    cpuCoreCount = sysconf(_SC_NPROC_ONLN);
#endif
    DBUG_MSG("Number of CPU cores: %d", cpuCoreCount);
    return cpuCoreCount;
}

OMX_ERRORTYPE initDecoder(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	
    omx_private->mCtx = TCC_calloc(1, sizeof(vpx_codec_ctx_t));
    vpx_codec_err_t vpx_err;
    vpx_codec_dec_cfg_t cfg;
    memset(&cfg, 0, sizeof(vpx_codec_dec_cfg_t));
    cfg.threads = GetCPUCoreCount();
    if ((vpx_err = vpx_codec_dec_init((vpx_codec_ctx_t *)omx_private->mCtx, &vpx_codec_vp8_dx_algo, &cfg, 0))) 
	{
        LOGE("on2 decoder failed to initialize. (%d)", vpx_err);
        return OMX_ErrorNoMore;
    }

	omx_private->avcodecReady = OMX_TRUE;
	
    return OMX_ErrorNone;
}

/** internal function to set codec related parameters in the private type structure 
  */
static void SetInternalVideoParameters(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private;
	omx_base_video_PortType *inPort ; 

	omx_private = openmaxStandComp->pComponentPrivate;;
	OMX_U32  video_coding_type = omx_private->video_coding_type;

	if (video_coding_type == OMX_VIDEO_CodingVPX) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-vnd.on2.vp8");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingVPX;

		//setHeader(&omx_private->pVideoRv, sizeof(OMX_VIDEO_PARAM_RVTYPE));    

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingVPX;
	} 	
}

OMX_ERRORTYPE omx_videodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32  video_coding_type = omx_private->video_coding_type;
	if(video_coding_type == OMX_VIDEO_CodingVPX) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_VPX_SW_NAME));
	}
	return OMX_ErrorComponentNotFound;
}
/** The Constructor of the video decoder component
  * @param openmaxStandComp the component handle to be constructed
  * @param cComponentName is the name of the constructed component
  */
OMX_ERRORTYPE omx_videodec_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName) {

	OMX_ERRORTYPE eError = OMX_ErrorNone;  
	omx_videodec_component_PrivateType* omx_private;
	omx_base_video_PortType *inPort,*outPort;
	OMX_U32 i;

		DBUG_MSG("In %s, allocating component\n", __func__);
		openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_videodec_component_PrivateType));
		if(openmaxStandComp->pComponentPrivate == NULL)
		{
			return OMX_ErrorInsufficientResources;
		}
		memset(openmaxStandComp->pComponentPrivate, 0x00, sizeof(omx_videodec_component_PrivateType));

		omx_private = openmaxStandComp->pComponentPrivate;
		omx_private->ports = NULL;

		eError = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

		omx_private->sPortTypesParam[OMX_PortDomainVideo].nStartPortNumber = 0;
		omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts = 2;

		/** Allocate Ports and call port constructor. */
		if (omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts && !omx_private->ports) {
			omx_private->ports = TCC_calloc(omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts, sizeof(omx_base_PortType *));
			if (!omx_private->ports) {
				return OMX_ErrorInsufficientResources;
			}
			for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
			omx_private->ports[i] = TCC_calloc(1, sizeof(omx_base_video_PortType));
			if (!omx_private->ports[i]) {
				return OMX_ErrorInsufficientResources;
			}
			}
		}

		base_video_port_Constructor(openmaxStandComp, &omx_private->ports[0], 0, OMX_TRUE);
		base_video_port_Constructor(openmaxStandComp, &omx_private->ports[1], 1, OMX_FALSE);

		/** now it's time to know the video coding type of the component */
		if(!strcmp(cComponentName, VIDEO_DEC_VPX_SW_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingVPX;
		}else {
			// IL client specified an invalid component name 
			return OMX_ErrorInvalidComponentName;
		} 

		/** here we can override whatever defaults the base_component constructor set
		* e.g. we can override the function pointers in the private struct  
		*/

		/** Domain specific section for the ports.   
		* first we set the parameter common to both formats
		*/
		//common parameters related to input port.  
		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sPortParam.nBufferSize = (OMX_U32) VIDEO_DEC_IN_BUFFER_SIZE;
		inPort->sPortParam.format.video.xFramerate = 30;
		inPort->sPortParam.format.video.nFrameWidth = AVAILABLE_MAX_WIDTH;
		inPort->sPortParam.format.video.nFrameHeight = AVAILABLE_MAX_HEIGHT;
		inPort->sPortParam.nBufferCountMin = 4;    
		inPort->sPortParam.nBufferCountActual = inPort->sPortParam.nBufferCountMin; 


		//common parameters related to output port
		outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
		outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
		outPort->sPortParam.format.video.nFrameWidth = AVAILABLE_MAX_WIDTH;
		outPort->sPortParam.format.video.nFrameHeight = AVAILABLE_MAX_HEIGHT;
		outPort->sPortParam.nBufferSize =  (OMX_U32) (AVAILABLE_MAX_WIDTH*AVAILABLE_MAX_HEIGHT*3/2);
		outPort->sPortParam.format.video.xFramerate = 30;
		outPort->sPortParam.nBufferCountMin= 4;
		outPort->sPortParam.nBufferCountActual = outPort->sPortParam.nBufferCountMin + 2;

		/** settings of output port parameter definition */
		outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
		outPort->sVideoParam.eColorFormat = OMX_COLOR_FormatYUV420Planar;

		outPort->sVideoParam.xFramerate = 30;
		
		if(!omx_private->avCodecSyncSem) {
			omx_private->avCodecSyncSem = TCC_malloc(sizeof(tsem_t));
			if(omx_private->avCodecSyncSem == NULL) {
				return OMX_ErrorInsufficientResources;
			}
			tsem_init(omx_private->avCodecSyncSem, 0);
		}

		SetInternalVideoParameters(openmaxStandComp);

		omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;

		if(omx_private->video_coding_type == OMX_VIDEO_CodingVPX) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingVPX;
		}
		
		/** general configuration irrespective of any video formats
		* setting other parameters of omx_videodec_component_private  
		*/
		//  omx_private->avCodec = NULL;
		//  omx_private->avCodecContext= NULL;
		omx_private->avcodecReady = OMX_FALSE;
		omx_private->BufferMgmtCallback = omx_videodec_component_BufferMgmtCallback;

		/** initializing the codec context etc that was done earlier by ffmpeglibinit function */
		omx_private->messageHandler = omx_videodec_component_MessageHandler;
		omx_private->destructor = omx_videodec_component_Destructor;

		openmaxStandComp->SetParameter = omx_videodec_component_SetParameter;
		openmaxStandComp->GetParameter = omx_videodec_component_GetParameter;
		openmaxStandComp->SetConfig    = omx_videodec_component_SetConfig;
		openmaxStandComp->GetConfig    = omx_videodec_component_GetConfig;
		openmaxStandComp->ComponentRoleEnum = omx_videodec_component_ComponentRoleEnum;
		openmaxStandComp->GetExtensionIndex = omx_videodec_component_GetExtensionIndex;

		omx_private->avcodecReady = OMX_FALSE;

		if(omx_private->video_coding_type == OMX_VIDEO_CodingVPX)
			eError = initDecoder(openmaxStandComp);

		omx_private->avgTime = 0;
		
		return eError;
	}


/** The destructor of the video decoder component
  */
OMX_ERRORTYPE omx_videodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

	OMX_S32 ret;

	if(omx_private->avcodecReady)
	{
		if(omx_private->video_coding_type == OMX_VIDEO_CodingVPX) {
		    vpx_codec_destroy((vpx_codec_ctx_t *)omx_private->mCtx);
		    TCC_free(omx_private->mCtx);
		    omx_private->mCtx = NULL;
		}
		omx_private->avcodecReady = OMX_FALSE;
	}
	
#if defined(USE_RB565_OUTPUT)
	if(omx_private->mClip != NULL){
		TCC_free(omx_private->mClip);
	}
#endif
	
	if(omx_private->avCodecSyncSem) {
		tsem_deinit(omx_private->avCodecSyncSem); 
		TCC_free(omx_private->avCodecSyncSem);
		omx_private->avCodecSyncSem = NULL;
	}

	/* frees port/s */   
	if (omx_private->ports) {   
		for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {   
		if(omx_private->ports[i])   
			omx_private->ports[i]->PortDestructor(omx_private->ports[i]);   
		}   
		TCC_free(omx_private->ports);   
		omx_private->ports=NULL;   
	} 

	DBUG_MSG("Destructor of video decoder component is called\n");

	omx_base_filter_Destructor(openmaxStandComp);

	return OMX_ErrorNone;
}


/** It initializates the FFmpeg framework, and opens an FFmpeg videodecoder of type specified by IL client 
  */ 
OMX_ERRORTYPE omx_videodec_component_LibInit(omx_videodec_component_PrivateType* omx_private) {
	tsem_up(omx_private->avCodecSyncSem);

	omx_private->avcodecInited = 0;
	return OMX_ErrorNone;
}

/** It Deinitializates the ffmpeg framework, and close the ffmpeg video decoder of selected coding type
  */
void omx_videodec_component_LibDeinit(omx_videodec_component_PrivateType* omx_private)
{

}

/** The Initialization function of the video decoder
  */
OMX_ERRORTYPE omx_videodec_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	/** Temporary First Output buffer size */
	omx_private->inputCurrBuffer = NULL;
	omx_private->inputCurrLength = 0;
	omx_private->isNewBuffer = 1;
	
	return eError;
}

/** The Deinitialization function of the video decoder  
  */
OMX_ERRORTYPE omx_videodec_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	DBUG_MSG("omx_videodec_component_Deinit is called(%d)\n", omx_private->avcodecReady);

	omx_videodec_component_LibDeinit(omx_private);

	return eError;
} 

/** Executes all the required steps after an output buffer frame-size has changed.
*/
static inline void UpdateFrameSize(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	omx_base_video_PortType *inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	outPort->sPortParam.format.video.nFrameWidth = 
		inPort->sPortParam.format.video.nFrameWidth + (inPort->sPortParam.format.video.nFrameWidth & 1);
	outPort->sPortParam.format.video.nFrameHeight = 
		inPort->sPortParam.format.video.nFrameHeight + (inPort->sPortParam.format.video.nFrameHeight & 1);
	outPort->sPortParam.format.video.xFramerate = inPort->sPortParam.format.video.xFramerate;
	switch(outPort->sVideoParam.eColorFormat) {
		case OMX_COLOR_FormatYUV420Planar:
		case OMX_COLOR_FormatYUV420SemiPlanar:
			if(outPort->sPortParam.format.video.nFrameWidth && outPort->sPortParam.format.video.nFrameHeight) {
				outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;
			}
		break;
		default:
			if(outPort->sPortParam.format.video.nFrameWidth && outPort->sPortParam.format.video.nFrameHeight) {
				outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3;
			}
		break;
	}

}

static int isPortChange(OMX_COMPONENTTYPE *openmaxStandComp, int width, int height)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	OMX_COLOR_FORMATTYPE colorformat;
	OMX_U32 bPortChanged, bCropChanged;

	bPortChanged = OMX_TRUE;
	bCropChanged = OMX_FALSE;
			
	if( 0 != omx_private->rectParm.nLeft ||
		0 != omx_private->rectParm.nTop ||
		width != omx_private->rectParm.nWidth ||
		height != omx_private->rectParm.nHeight)
	{
		omx_private->rectParm.nLeft		= 0;
		omx_private->rectParm.nTop		= 0;
		omx_private->rectParm.nWidth	= width;
		omx_private->rectParm.nHeight	= height;
		bCropChanged = OMX_TRUE;
		LOGI(" CropInfo Changed %ld,%ld - %ldx%ld", omx_private->rectParm.nLeft, omx_private->rectParm.nTop, omx_private->rectParm.nWidth, omx_private->rectParm.nHeight);
	}
	
	if(bPortChanged || bCropChanged)
	{
		outPort->bIsPortChanged = OMX_TRUE;		
		outPort->sPortParam.format.video.nFrameWidth = width;
		outPort->sPortParam.format.video.nFrameHeight = height;	
		
		switch(outPort->sVideoParam.eColorFormat) {
			case OMX_COLOR_FormatYUV420Planar:
			case OMX_COLOR_FormatYUV420SemiPlanar:
				if(outPort->sPortParam.format.video.nFrameWidth && outPort->sPortParam.format.video.nFrameHeight) {
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;
				}
			break;
			default:
				if(outPort->sPortParam.format.video.nFrameWidth && outPort->sPortParam.format.video.nFrameHeight) {
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3;
				}
			break;
		}

		if( bPortChanged ) {
			(*(omx_private->callbacks->EventHandler))(
								   openmaxStandComp,
								   omx_private->callbackData,
								   OMX_EventPortSettingsChanged, 
								   OMX_DirOutput,
								   0, 
								   NULL);
		}

		if( bCropChanged ) {
			(*(omx_private->callbacks->EventHandler))(
								   openmaxStandComp,
								   omx_private->callbackData,
								   OMX_EventPortSettingsChanged, 
								   OMX_DirOutput,
								   OMX_IndexConfigCommonOutputCrop, 
								   NULL);
		}
		
		LOGI( "ReSize Needed!! %ld x %ld -> %ld x %ld \n", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, width, height);		
	}
	
	return 0;
}

#if defined(USE_RB565_OUTPUT)
OMX_U8* initClip(OMX_COMPONENTTYPE *openmaxStandComp) 
{
    static const signed kClipMin = -278;
    static const signed kClipMax = 535;
	signed i;
	
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

    if (omx_private->mClip == NULL) {
        omx_private->mClip = TCC_calloc(1, kClipMax - kClipMin + 1);

        for (i = kClipMin; i <= kClipMax; ++i) {
            omx_private->mClip[i - kClipMin] = (i < 0) ? 0 : (i > 255) ? 255 : (OMX_U8)i;
        }
    }

    return &omx_private->mClip[-kClipMin];
}

OMX_ERRORTYPE convertYUV420Planar(OMX_COMPONENTTYPE *openmaxStandComp,
        const BitmapParams src, const BitmapParams dst, const OMX_U8 *src_y,  const OMX_U8 *src_u, const OMX_U8 *src_v) 
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32 x, y, cropHeight, cropWidth;
	
    OMX_U8 *kAdjustedClip = initClip(openmaxStandComp);
    OMX_U16 *dst_ptr = (OMX_U16 *)dst.mBits + dst.mCropTop * dst.mWidth + dst.mCropLeft;

	cropWidth = src.mCropRight - src.mCropLeft + 1;
	cropHeight = src.mCropBottom - src.mCropTop + 1;

    for ( y = 0; y < cropHeight; ++y) {
        for ( x = 0; x < cropWidth; x += 2) {
            // B = 1.164 * (Y - 16) + 2.018 * (U - 128)
            // G = 1.164 * (Y - 16) - 0.813 * (V - 128) - 0.391 * (U - 128)
            // R = 1.164 * (Y - 16) + 1.596 * (V - 128)

            // B = 298/256 * (Y - 16) + 517/256 * (U - 128)
            // G = .................. - 208/256 * (V - 128) - 100/256 * (U - 128)
            // R = .................. + 409/256 * (V - 128)

            // min_B = (298 * (- 16) + 517 * (- 128)) / 256 = -277
            // min_G = (298 * (- 16) - 208 * (255 - 128) - 100 * (255 - 128)) / 256 = -172
            // min_R = (298 * (- 16) + 409 * (- 128)) / 256 = -223

            // max_B = (298 * (255 - 16) + 517 * (255 - 128)) / 256 = 534
            // max_G = (298 * (255 - 16) - 208 * (- 128) - 100 * (- 128)) / 256 = 432
            // max_R = (298 * (255 - 16) + 409 * (255 - 128)) / 256 = 481

            // clip range -278 .. 535

            signed y1 = (signed)src_y[x] - 16;
            signed y2 = (signed)src_y[x + 1] - 16;

            signed u = (signed)src_u[x / 2] - 128;
            signed v = (signed)src_v[x / 2] - 128;

            signed u_b = u * 517;
            signed u_g = -u * 100;
            signed v_g = -v * 208;
            signed v_r = v * 409;

            signed tmp1 = y1 * 298;
            signed b1 = (tmp1 + u_b) / 256;
            signed g1 = (tmp1 + v_g + u_g) / 256;
            signed r1 = (tmp1 + v_r) / 256;

            signed tmp2 = y2 * 298;
            signed b2 = (tmp2 + u_b) / 256;
            signed g2 = (tmp2 + v_g + u_g) / 256;
            signed r2 = (tmp2 + v_r) / 256;

            OMX_U32 rgb1 =
                ((kAdjustedClip[r1] >> 3) << 11)
                | ((kAdjustedClip[g1] >> 2) << 5)
                | (kAdjustedClip[b1] >> 3);

            OMX_U32 rgb2 =
                ((kAdjustedClip[r2] >> 3) << 11)
                | ((kAdjustedClip[g2] >> 2) << 5)
                | (kAdjustedClip[b2] >> 3);

            if (x + 1 < cropWidth) {
                *(OMX_U32 *)(&dst_ptr[x]) = (rgb2 << 16) | rgb1;
            } else {
                dst_ptr[x] = rgb1;
            }
        }

        src_y += src.mWidth;

        if (y & 1) {
            src_u += src.mWidth / 2;
            src_v += src.mWidth / 2;
        }

        dst_ptr += dst.mWidth;
    }

    return OMX_ErrorNone;
}
#endif

/** This function is used to process the input buffer and provide one output buffer
  */
void omx_videodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer	, OMX_BUFFERHEADERTYPE* pOutputBuffer) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_S32 nOutputFilled = 0;
#ifdef ANDROID_USE_GRALLOC_BUFFER
	buffer_handle_t*  grallocHandle = NULL;
	OMX_U8 *pGrallocAddr;
	void *pGrallocPureAddr;
#endif

    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	if(omx_private->state != 3) {
	    LOGE("=> omx_private->state != 3");
		return;
	}

	pOutputBuffer->nFilledLen = 0;
	pOutputBuffer->nOffset = 0;

	while (!nOutputFilled) {	    
	    if (omx_private->isFirstBuffer) {
	        tsem_down(omx_private->avCodecSyncSem);
	        omx_private->isFirstBuffer = 0;
	    }		

	    if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
			DBUG_MSG("Config data IN!!");
			
			omx_private->isNewBuffer = 1;
			pOutputBuffer->nFilledLen = 0;
			pInputBuffer->nFilledLen = 0;			
			return;
		}    

		if (vpx_codec_decode(
                    (vpx_codec_ctx_t *)omx_private->mCtx,
                    pInputBuffer->pBuffer + pInputBuffer->nOffset,
                    pInputBuffer->nFilledLen,
                    NULL,
                    0)) {
            LOGE("on2 decoder failed to decode frame.");

			(*(omx_private->callbacks->EventHandler))(openmaxStandComp, omx_private->callbackData,
					OMX_EventError, OMX_ErrorUndefined,
					0, NULL);	
            return;
        }

        vpx_codec_iter_t iter = NULL;
        vpx_image_t *img = vpx_codec_get_frame((vpx_codec_ctx_t *)omx_private->mCtx, &iter);

        if (img != NULL) {
            int32_t width = img->d_w;
            int32_t height = img->d_h;

			DBUG_MSG("Res check : %dx%d -> %dx%d", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, width, height);
            if (width != outPort->sPortParam.format.video.nFrameWidth || height != outPort->sPortParam.format.video.nFrameHeight) {
                outPort->sPortParam.format.video.nFrameWidth = width;
                outPort->sPortParam.format.video.nFrameHeight = height;

                isPortChange(openmaxStandComp, width, height);
				UpdateFrameSize(openmaxStandComp);
                return;
            }

            pOutputBuffer->nFilledLen = (width * height * 3) / 2;
            pOutputBuffer->nFlags = 0;
            pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;

#if defined(ANDROID_USE_GRALLOC_BUFFER) 
			if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr) 
			{
				int i = 0;
				int stride_y, stride_uv;
				unsigned int framesize_Y, framesize_C;
				BitmapParams src, dst;
				
				stride_y = ALIGNED_BUFF(outPort->sPortParam.format.video.nFrameWidth, 16);
				stride_uv = ALIGNED_BUFF(stride_y/2, 16);
				framesize_Y = stride_y * outPort->sPortParam.format.video.nFrameHeight;
				framesize_C = stride_uv * outPort->sPortParam.format.video.nFrameHeight/2;
				
				grallocHandle = (buffer_handle_t*)pOutputBuffer->pBuffer;
				omx_private->gralloc_info.grallocModule->lock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule,
												(buffer_handle_t)grallocHandle, GRALLOC_USAGE_SW_WRITE_MASK,
												0,0,outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, &pGrallocPureAddr);

				pGrallocAddr = (OMX_U8 *)pGrallocPureAddr;

				DBUG_MSG("VPX Res(%dx%d), stride(%d, %d)", img->d_w, img->d_h, stride_y, stride_uv);
				DBUG_MSG("VPX stride : %d/%d/%d, Res(%dx%d)", img->stride[PLANE_Y], img->stride[PLANE_U], img->stride[PLANE_V], width, height);

	#if defined(USE_RB565_OUTPUT)
				src.mBits 		= (void *)img->planes[PLANE_Y];
				src.mWidth 		= img->stride[PLANE_Y];
				src.mHeight 	= img->d_h;
				src.mCropLeft 	= 0;
				src.mCropTop 	= 0;
				src.mCropRight 	= img->d_w-1;
				src.mCropBottom = img->d_h-1;

				dst.mBits 		= (void*)pGrallocAddr;
				dst.mWidth 		= img->d_w;
				dst.mHeight 	= img->d_h;
				dst.mCropLeft 	= 0;
				dst.mCropTop 	= 0;
				dst.mCropRight 	= img->d_w-1;
				dst.mCropBottom = img->d_h-1;

				DBUG_MSG("VPX addr : %p/%p/%p -> %p", (const OMX_U8 *)img->planes[PLANE_Y], (const OMX_U8 *)img->planes[PLANE_U], (const OMX_U8 *)img->planes[PLANE_V], pGrallocAddr);
				convertYUV420Planar(openmaxStandComp, src, dst,
										(const OMX_U8 *)img->planes[PLANE_Y],
										(const OMX_U8 *)img->planes[PLANE_U],
										(const OMX_U8 *)img->planes[PLANE_V]);
	#else
				const uint8_t *srcLine = (const uint8_t *)img->planes[PLANE_Y];
				uint8_t *dst = pGrallocAddr;

				if( stride_y == img->stride[PLANE_Y])
				{
					memcpy(dst, srcLine, framesize_Y);
				}
				else
				{
					for (i = 0; i < img->d_h; ++i) {
						memcpy(dst, srcLine, img->d_w);

						srcLine += img->stride[PLANE_Y];
						dst += stride_y;
					}
				}
								
				srcLine = (const uint8_t *)img->planes[PLANE_V];
				dst = pGrallocAddr+framesize_Y;

				if( stride_uv == img->stride[PLANE_V])
				{
					memcpy(dst, srcLine, framesize_C);
				}
				else
				{
					for (i = 0; i < img->d_h / 2; ++i) {
						memcpy(dst, srcLine, img->d_w / 2);

						srcLine += img->stride[PLANE_V];
						dst += stride_uv;
					}
				}

				srcLine = (const uint8_t *)img->planes[PLANE_U];
				dst = pGrallocAddr+framesize_Y+framesize_C;
				if( stride_uv == img->stride[PLANE_U])
				{
					memcpy(dst, srcLine, framesize_C);
				}
				else
				{				
					for (i = 0; i < img->d_h / 2; ++i) {
						memcpy(dst, srcLine, img->d_w / 2);

						srcLine += img->stride[PLANE_U];
						dst += stride_uv;
					}
				}
	#endif
				omx_private->gralloc_info.grallocModule->unlock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule, (buffer_handle_t)grallocHandle);
			}
			else
#endif
			{
				int i = 0;
	            const uint8_t *srcLine = (const uint8_t *)img->planes[PLANE_Y];
	            uint8_t *dst = pOutputBuffer->pBuffer;
	            for (i = 0; i < img->d_h; ++i) {
	                memcpy(dst, srcLine, img->d_w);

	                srcLine += img->stride[PLANE_Y];
	                dst += img->d_w;
	            }

	            srcLine = (const uint8_t *)img->planes[PLANE_U];
	            for (i = 0; i < img->d_h / 2; ++i) {
	                memcpy(dst, srcLine, img->d_w / 2);

	                srcLine += img->stride[PLANE_U];
	                dst += img->d_w / 2;
	            }

	            srcLine = (const uint8_t *)img->planes[PLANE_V];
	            for (i = 0; i < img->d_h / 2; ++i) {
	                memcpy(dst, srcLine, img->d_w / 2);

	                srcLine += img->stride[PLANE_V];
	                dst += img->d_w / 2;
	            }
			}
			pInputBuffer->nFilledLen = 0;
        }
		else
		{
			pInputBuffer->nFilledLen = 0;
			pOutputBuffer->nFilledLen = 0;
		}
		
		nOutputFilled = 1;
	}

	return;
}

OMX_ERRORTYPE omx_videodec_component_SetParameter(
OMX_IN  OMX_HANDLETYPE hComponent,
OMX_IN  OMX_INDEXTYPE nParamIndex,
OMX_IN  OMX_PTR ComponentParameterStructure) {

  OMX_ERRORTYPE eError = OMX_ErrorNone;
  OMX_U32 portIndex;

  /* Check which structure we are being fed and make control its header */
  OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  omx_base_video_PortType *port;
  if (ComponentParameterStructure == NULL) {
    return OMX_ErrorBadParameter;
  }

  DBUG_MSG("   Setting parameter 0x%x", nParamIndex);
  switch(nParamIndex) {
    case OMX_IndexParamPortDefinition:
      {
        eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
        if(eError == OMX_ErrorNone) {
          OMX_PARAM_PORTDEFINITIONTYPE *pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE*)ComponentParameterStructure;
          //UpdateFrameSize (openmaxStandComp);
          portIndex = pPortDef->nPortIndex;
          port = (omx_base_video_PortType *)omx_private->ports[portIndex];
          port->sVideoParam.eColorFormat = port->sPortParam.format.video.eColorFormat;

		  omx_private->rectParm.nLeft 	= 0;
		  omx_private->rectParm.nTop 	= 0;
		  omx_private->rectParm.nWidth	= port->sPortParam.format.video.nFrameWidth;
		  omx_private->rectParm.nHeight	= port->sPortParam.format.video.nFrameHeight;
		  LOGI(" CropInfo %ld,%ld - %ldx%ld", omx_private->rectParm.nLeft, omx_private->rectParm.nTop, omx_private->rectParm.nWidth, omx_private->rectParm.nHeight);
        }
        break;
      }
    case OMX_IndexParamVideoPortFormat:
      {
        OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat;
        pVideoPortFormat = ComponentParameterStructure;
        portIndex = pVideoPortFormat->nPortIndex;
        /*Check Structure Header and verify component state*/
        eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
        if(eError!=OMX_ErrorNone) { 
          LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
          break;
        } 

		if (pVideoPortFormat->nIndex != 0) {
            return OMX_ErrorNoMore;
        }
		
        if (portIndex <= 1) {
          port = (omx_base_video_PortType *)omx_private->ports[portIndex];
          memcpy(&port->sVideoParam, pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
          omx_private->ports[portIndex]->sPortParam.format.video.eColorFormat = port->sVideoParam.eColorFormat;

          if (portIndex == 1) {
            switch(port->sVideoParam.eColorFormat) {
              case OMX_COLOR_FormatYUV420Planar :
                omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;
                break;
              default:
			  case OMX_COLOR_FormatYUV420SemiPlanar :
				omx_private->eOutFramePixFmt = PIX_FMT_NV12;
                break;
            }
            //UpdateFrameSize (openmaxStandComp);
          }
        } else {
          return OMX_ErrorBadPortIndex;
        }
        break;
      }
    case OMX_IndexParamStandardComponentRole:
	{
		OMX_PARAM_COMPONENTROLETYPE *pComponentRole;
		pComponentRole = ComponentParameterStructure;
		if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_VPX_SW_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingVPX;
		}
		SetInternalVideoParameters(openmaxStandComp);
		break;
	}
	case OMX_IndexVendorVideoExtraData:
	{
		OMX_VENDOR_EXTRADATATYPE* pExtradata;
		pExtradata = (OMX_VENDOR_EXTRADATATYPE*)ComponentParameterStructure;
		if (pExtradata->nPortIndex <= 1) {
		} else {
			return OMX_ErrorBadPortIndex;
		}
	}
	break;

	case OMX_IndexParamCommonDeblocking:
	{
		break;
	}
	
#ifdef ANDROID_USE_GRALLOC_BUFFER
	case OMX_IndexUseNativeBuffers:
	{
		OMX_PARAMUSENATIVEBUFFER *pParamNativeBuffer = NULL;

		pParamNativeBuffer = (OMX_PARAMUSENATIVEBUFFER* )ComponentParameterStructure;
		if(pParamNativeBuffer->bEnable == OMX_TRUE)
		{
			LOGI("######################## Use GrallocPtr mode #########################");
			omx_private->gralloc_info.PortBuffers[pParamNativeBuffer->nPortIndex].BufferType = GrallocPtr;
			omx_private->gralloc_info.PortBuffers[pParamNativeBuffer->nPortIndex].IsBuffer2D = OMX_TRUE;
		}
	}
	break;
#endif

    default: /*Call the base component function*/
      return omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
  }
  return eError;
}

OMX_ERRORTYPE omx_videodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure) {

  omx_base_video_PortType *port;
  OMX_ERRORTYPE eError = OMX_ErrorNone;

  OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
  omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  if (ComponentParameterStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
  DBUG_MSG("   Getting parameter 0x%x", nParamIndex);
  /* Check which structure we are being fed and fill its header */
  switch(nParamIndex) {
    case OMX_IndexParamVideoInit:
      if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone) { 
        break;
      }
      memcpy(ComponentParameterStructure, &omx_private->sPortTypesParam[OMX_PortDomainVideo], sizeof(OMX_PORT_PARAM_TYPE));
      break;    
    case OMX_IndexParamVideoPortFormat:
      {
        OMX_VIDEO_PARAM_PORTFORMATTYPE *pVideoPortFormat;  
        pVideoPortFormat = ComponentParameterStructure;
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) { 
          break;
        }
		
		if (pVideoPortFormat->nIndex != 0) {
            return OMX_ErrorNoMore;
        }
		
        if (pVideoPortFormat->nPortIndex <= 1) {
          port = (omx_base_video_PortType *)omx_private->ports[pVideoPortFormat->nPortIndex];
          memcpy(pVideoPortFormat, &port->sVideoParam, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
        } else {
          return OMX_ErrorBadPortIndex;
        }
        break;    
      }
    case OMX_IndexParamStandardComponentRole:
      {
        OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
        pComponentRole = ComponentParameterStructure;
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
          break;
        }
        if (omx_private->video_coding_type == OMX_VIDEO_CodingVPX) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_VPX_SW_ROLE);
        }
        break;
      }
#ifdef HAVE_ANDROID_OS
	case PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX:
	{
		PV_OMXComponentCapabilityFlagsType *pCap_flags =
		(PV_OMXComponentCapabilityFlagsType *) ComponentParameterStructure;
		if (NULL == pCap_flags)
		{
			return OMX_ErrorBadParameter;
		}
	
		memset(pCap_flags, 0, sizeof(PV_OMXComponentCapabilityFlagsType));
		pCap_flags->iIsOMXComponentMultiThreaded = OMX_TRUE;
		pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc 	= OMX_TRUE;
		pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc 	= OMX_TRUE;
		pCap_flags->iOMXComponentSupportsMovableInputBuffers	= OMX_FALSE;
		pCap_flags->iOMXComponentSupportsPartialFrames			= OMX_FALSE;
		pCap_flags->iOMXComponentUsesNALStartCodes 				= OMX_FALSE;
		pCap_flags->iOMXComponentUsesFullAVCFrames 				= OMX_FALSE;			
        pCap_flags->iOMXComponentCanHandleIncompleteFrames 			= OMX_FALSE;

		DBUG_MSG("=== PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX ===");
	}
	break;
#endif

	case OMX_IndexParamVideoProfileLevelQuerySupported:
	{
		OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *) ComponentParameterStructure;
		OMX_U32 nNumberOfProfiles = 0;

		if (profileLevel->nPortIndex != 0) {
		    LOGE("Invalid port index: %ld", profileLevel->nPortIndex);
		    return OMX_ErrorUnsupportedIndex;
		}

		if (profileLevel->nProfileIndex >= nNumberOfProfiles) {
		    return OMX_ErrorNoMore;
		}

		return OMX_ErrorNone;
	}
	break;

    case OMX_IndexParamCommonDeblocking:
      {
        break;
      }
	
#ifdef ANDROID_USE_GRALLOC_BUFFER
	case OMX_IndexAndroidNativeBufferUsage:
	{
		OMX_PARAMNATIVEBUFFERUSAGE *pNativeBuffUsage = NULL;

		pNativeBuffUsage = (OMX_PARAMNATIVEBUFFERUSAGE*)ComponentParameterStructure;
		if(omx_private->gralloc_info.PortBuffers[pNativeBuffUsage->nPortIndex].BufferType == GrallocPtr)
		{
			pNativeBuffUsage->nUsage = GRALLOC_USAGE_HW_RENDER;
			eError = OMX_ErrorNone;
		}
	}
	break;
#endif

    default: /*Call the base component function*/
    {
		eError = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
#ifdef ANDROID_USE_GRALLOC_BUFFER
		if( nParamIndex == OMX_IndexParamPortDefinition )
		{
			OMX_PARAM_PORTDEFINITIONTYPE *pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE*)ComponentParameterStructure;
			
			pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE *)ComponentParameterStructure;
			if(omx_private->gralloc_info.PortBuffers[pPortDef->nPortIndex].BufferType == GrallocPtr)
			{
		#if defined(USE_RB565_OUTPUT)		
				pPortDef->format.video.eColorFormat = HAL_PIXEL_FORMAT_RGB_565;
		#else
				pPortDef->format.video.eColorFormat = HAL_PIXEL_FORMAT_YV12;
		#endif
				DBUG_MSG("pPortDef->format.video.eColorFormat(%d)", pPortDef->format.video.eColorFormat);
			}
			else
			{
				pPortDef->format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
			}
		}
		else if ( nParamIndex == OMX_IndexParamVideoPortFormat )
		{
	        OMX_VIDEO_PARAM_PORTFORMATTYPE *pPortParam = (OMX_VIDEO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
			
			pPortParam = (OMX_VIDEO_PARAM_PORTFORMATTYPE *)ComponentParameterStructure;
			if(omx_private->gralloc_info.PortBuffers[pPortParam->nPortIndex].BufferType == GrallocPtr)
			{
		#if defined(USE_RB565_OUTPUT)		
				pPortParam->eColorFormat = HAL_PIXEL_FORMAT_RGB_565;
		#else		
				pPortParam->eColorFormat = HAL_PIXEL_FORMAT_YV12;
		#endif
				DBUG_MSG("pPortParam->eColorFormat(%d)", pPortParam->eColorFormat);
			}
			else
			{
				pPortParam->eColorFormat = OMX_COLOR_FormatYUV420Planar;
			}
		}
#endif
	}
	break;
  }
  return eError;
}

OMX_ERRORTYPE omx_videodec_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp,internalRequestMessageType *message) {
  omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err;
  OMX_STATETYPE eCurrentState = omx_private->state;

  DBUG_MSG("In %s\n", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateExecuting ) && (omx_private->state == OMX_StateIdle)) {
      if (!omx_private->avcodecReady) {
        err = omx_videodec_component_LibInit(omx_private);
        if (err != OMX_ErrorNone) {
          return OMX_ErrorNotReady;
        }
        omx_private->avcodecReady = OMX_TRUE;
      }
    } 
    else if ((message->messageParam == OMX_StateIdle ) && (omx_private->state == OMX_StateLoaded)) {
      err = omx_videodec_component_Initialize(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        LOGE( "In %s Video Decoder Init Failed Error=%x\n",__func__,err); 
        return err;
      } 
    } else if ((message->messageParam == OMX_StateLoaded) && (omx_private->state == OMX_StateIdle)) {
      err = omx_videodec_component_Deinit(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        LOGE( "In %s Video Decoder Deinit Failed Error=%x\n",__func__,err); 
        return err;
      } 
    }
  }
  // Execute the base message handling
  err =  omx_base_component_MessageHandler(openmaxStandComp,message);

  if (message->messageType == OMX_CommandStateSet){
   if ((message->messageParam == OMX_StateIdle  ) && (eCurrentState == OMX_StateExecuting || eCurrentState == OMX_StatePause)) {
    }
  }
  return err;
}
OMX_ERRORTYPE omx_videodec_component_ComponentRoleEnum(
  OMX_IN OMX_HANDLETYPE hComponent,
  OMX_OUT OMX_U8 *cRole,
  OMX_IN OMX_U32 nIndex) {
  
	OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;

	if (nIndex == 0) {
		if(omx_private->video_coding_type == OMX_VIDEO_CodingVPX) {
			strcpy((char *)cRole, VIDEO_DEC_VPX_SW_ROLE);
		}
	} else{
		return OMX_ErrorUnsupportedIndex;
	}
		
	return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_videodec_component_SetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure) {

  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_VENDOR_EXTRADATATYPE* pExtradata;

  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;

  if (pComponentConfigStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
  DBUG_MSG("   Setting configuration %i\n", nIndex);
  /* Check which structure we are being fed and fill its header */
  switch (nIndex) {
    case OMX_IndexVendorVideoExtraData :	
		pExtradata = (OMX_VENDOR_EXTRADATATYPE*)pComponentConfigStructure;
		if (pExtradata->nPortIndex <= 1) {
		}
		else 
		{
			return OMX_ErrorBadPortIndex;
		}

      break;

    case OMX_IndexConfigVideoPlayDirection :
      break;

  case OMX_IndexConfigVideoOutputKeyFrameOnly:
	  break;

  case OMX_IndexVendorThumbnailMode:
	  break;

    default: // delegate to superclass
      return omx_base_component_SetConfig(hComponent, nIndex, pComponentConfigStructure);
  }
  return err;
}

OMX_ERRORTYPE omx_videodec_component_GetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure) {

  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_VENDOR_EXTRADATATYPE* pExtradata;

  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
	omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;

  if (pComponentConfigStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
  DBUG_MSG("   Getting configuration %i\n", nIndex);
  /* Check which structure we are being fed and fill its header */
  switch (nIndex) {
    case OMX_IndexConfigCommonOutputCrop:
		{
			OMX_CONFIG_RECTTYPE *rectParams = (OMX_CONFIG_RECTTYPE *)pComponentConfigStructure;

			if (rectParams->nPortIndex != 1) {
			    return OMX_ErrorUndefined;
			}

			rectParams->nLeft 	= omx_private->rectParm.nLeft;
			rectParams->nTop 	= omx_private->rectParm.nTop;
			rectParams->nWidth 	= omx_private->rectParm.nWidth;
			rectParams->nHeight = omx_private->rectParm.nHeight;
	    }
		break;

    default: // delegate to superclass
		return omx_base_component_GetConfig(hComponent, nIndex, pComponentConfigStructure);
  }
  return err;
}

OMX_ERRORTYPE omx_videodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType) {

	DBUG_MSG("In  %s - %s \n",__func__, cParameterName);

	if(strcmp(cParameterName,"OMX.tcc.index.config.videoextradata") == 0) {
		*pIndexType = (OMX_INDEXTYPE)OMX_IndexVendorVideoExtraData;
	}else if(strcmp(cParameterName, "OMX.TCC.index.ThumbnailMode") == 0){
		*pIndexType = (OMX_INDEXTYPE)OMX_IndexVendorThumbnailMode;
	}
#ifdef ANDROID_USE_GRALLOC_BUFFER
	else if(strcmp(cParameterName, "OMX.google.android.index.getAndroidNativeBufferUsage") == 0){
		*pIndexType = (OMX_INDEXTYPE)OMX_IndexAndroidNativeBufferUsage;
	}else if(strcmp(cParameterName, "OMX.google.android.index.enableAndroidNativeBuffers") == 0){
		*pIndexType = (OMX_INDEXTYPE) OMX_IndexUseNativeBuffers;
	}else if(strcmp(cParameterName, "OMX.google.android.index.useAndroidNativeBuffer2") == 0){
		*pIndexType = (OMX_INDEXTYPE) NULL;
	}
#endif
	else {
		LOGE("OMX_ErrorBadParameter  %s - %s \n", __func__, cParameterName);
		return OMX_ErrorBadParameter;
	}

	DBUG_MSG("Out(Index = 0x%x)  %s - %s \n", *pIndexType, __func__, cParameterName);
	return OMX_ErrorNone;
}

#ifdef HAVE_ANDROID_OS
OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
	OMX_ERRORTYPE err = OMX_ErrorNone;

	err = omx_videodec_component_Constructor(openmaxStandComp,cCompontName);
#ifdef ANDROID_USE_GRALLOC_BUFFER
	if(err == OMX_ErrorNone)
	{
		hw_module_t const* module;
  		omx_videodec_component_PrivateType* omx_private = ((OMX_COMPONENTTYPE*)openmaxStandComp)->pComponentPrivate;

        err = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &module);
        if (err == 0)
		{
            omx_private->gralloc_info.grallocModule = (gralloc_module_t const *)module;
        }
		else
		{
            LOGE("ERROR: can't load gralloc using hw_get_module(%s) => err[0x%x]", GRALLOC_HARDWARE_MODULE_ID, err);
			err = OMX_ErrorInsufficientResources;
		}
	}
#endif
	return err;
}

#endif
