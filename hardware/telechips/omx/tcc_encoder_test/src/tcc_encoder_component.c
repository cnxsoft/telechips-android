/**
  @file src/components/ffmpeg/omx_videodec_component.c
  
  This component implements MPEG-4 SP video encoder. 
  The MPEG-4 SP Video decoder is based on the FFmpeg software library.

  Copyright (C) 2007-2008 STMicroelectronics
  Copyright (C) 2007-2008 Nokia Corporation and/or its subsidiary(-ies)
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

  Date: 2009/04/6
  Author : ZzaU
*/
#include <omxcore.h>
#include <omx_base_video_port.h>
#include <tcc_encoder_component.h>
#include <encoder.h>
#include<OMX_Video.h>

#include <lcd_resolution.h>
#include <tcc_vpu_encode_interface.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>


#define LOG_TAG	"TCC_VIDEO_ENC"
#include <utils/Log.h>

static int DEBUG_ON  = 0;
#define DBug_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}

/** Maximum Number of Video Component Instance*/
#define MAX_COMPONENT_VIDEOENC 4

static OMX_U32 total_count = 0;
/** Counter of Video Component Instance*/
static OMX_U32 noVideoDecInstance = 0;
/** The output decoded color format */
#define INPUT_ENCODED_COLOR_FMT OMX_COLOR_FormatYUV420Planar

#define NUM_IN_BUFFERS          	4      	 					// Input Buffers   (camera buffer - 2)
#define NUM_OUT_BUFFERS         	VIDEO_ENC_BUFFER_COUNT     	// Output Buffers
#define IN_BUFFER_SIZE         		(4*1024)  					// This buffer only include physical Address!!
#define OUT_BUFFER_SIZE         	(LARGE_STREAM_BUF_SIZE) 

#define USE_OUTPUT_USE_BUFFER 0
#define USE_INPUT_USE_BUFFER 1

//#define CHANGE_BITRATE  //to change bitrate.
#ifdef CHANGE_BITRATE
#define REQUEST_INTRAR_EFRESH  //to request I-Frame.
#endif

typedef struct VIDEO_PROFILE_LEVEL
{
    OMX_S32  nProfile;
    OMX_S32  nLevel;
} VIDEO_PROFILE_LEVEL_TYPE;

/* H.263 Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedH263ProfileLevels[] = {
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level10},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level20},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level30},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level40},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level45},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level50},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level60},
  {OMX_VIDEO_H263ProfileBaseline, OMX_VIDEO_H263Level70},
  {-1, -1}};

/* MPEG4 Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedMPEG4ProfileLevels[] ={
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level0b},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level1},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level2},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level3},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level4a},
  {OMX_VIDEO_MPEG4ProfileSimple, OMX_VIDEO_MPEG4Level5},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0b},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level1},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level2},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level3},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level4},
//  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level5},
  {-1,-1}};

/* AVC Supported Levels & profiles */
VIDEO_PROFILE_LEVEL_TYPE SupportedAVCProfileLevels[] ={
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel1},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel1b},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel11},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel12},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel13},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel2},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel21},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel22},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel3},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel31},
  {-1,-1}};

/* 
 * Initializes a data structure using a pointer to the structure.
 * The initialization of OMX structures always sets up the nSize and nVersion fields 
 *   of the structure.
 */
#define OMX_CONF_INIT_STRUCT_PTR(_s_, _name_)   \
    memset((_s_), 0x0, sizeof(_name_));         \
    (_s_)->nSize = sizeof(_name_);              \
    (_s_)->nVersion.s.nVersionMajor = 0x1;      \
    (_s_)->nVersion.s.nVersionMinor = 0x0;      \
    (_s_)->nVersion.s.nRevision = 0x0;          \
    (_s_)->nVersion.s.nStep = 0x0


/** internal function to set codec related parameters in the private type structure 
  */
static void SetInternalVideoParameters(OMX_COMPONENTTYPE *openmaxStandComp) {

  omx_videoenc_component_PrivateType* omx_private;
  omx_base_video_PortType *inPort ; 

  omx_private = openmaxStandComp->pComponentPrivate;;

  if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
    strcpy(omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/mpeg4");
    omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;

    setHeader(&omx_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));    
    omx_private->pVideoMpeg4.nPortIndex = 0x1;                                                                    
    omx_private->pVideoMpeg4.nSliceHeaderSpacing = 0;
    omx_private->pVideoMpeg4.bSVH = OMX_FALSE;
    omx_private->pVideoMpeg4.bGov = OMX_FALSE;
    omx_private->pVideoMpeg4.nPFrames = 10;
    omx_private->pVideoMpeg4.nBFrames = 0;
    omx_private->pVideoMpeg4.nIDCVLCThreshold = 0;
    omx_private->pVideoMpeg4.bACPred = OMX_FALSE;
    omx_private->pVideoMpeg4.nMaxPacketSize = 256;
    omx_private->pVideoMpeg4.nTimeIncRes = 0;
    omx_private->pVideoMpeg4.eProfile = OMX_VIDEO_MPEG4ProfileSimple; //OMX_VIDEO_MPEG4ProfileCore
    omx_private->pVideoMpeg4.eLevel = OMX_VIDEO_MPEG4Level0;//OMX_VIDEO_MPEG4Level2
    omx_private->pVideoMpeg4.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
    omx_private->pVideoMpeg4.nHeaderExtension = 0;
    omx_private->pVideoMpeg4.bReversibleVLC = OMX_FALSE;

    inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
    inPort->sVideoParam.eCompressionFormat = OMX_COLOR_FormatUnused;

  } 
  else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
    strcpy(omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/avc");
    omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;

    setHeader(&omx_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));    
    omx_private->pVideoAvc.nPortIndex = 0x1;
    omx_private->pVideoAvc.nSliceHeaderSpacing = 0;
    omx_private->pVideoAvc.bUseHadamard = OMX_FALSE;
    omx_private->pVideoAvc.nRefFrames = 2;
    omx_private->pVideoAvc.nPFrames = 0;
    omx_private->pVideoAvc.nBFrames = 0;
    omx_private->pVideoAvc.eProfile = OMX_VIDEO_AVCProfileBaseline;
    omx_private->pVideoAvc.eLevel = OMX_VIDEO_AVCLevel1;
    omx_private->pVideoAvc.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
    omx_private->pVideoAvc.bFrameMBsOnly = OMX_FALSE;
    omx_private->pVideoAvc.nRefIdx10ActiveMinus1 = 0;
    omx_private->pVideoAvc.nRefIdx11ActiveMinus1 = 0;
    omx_private->pVideoAvc.bEnableUEP = OMX_FALSE;  
    omx_private->pVideoAvc.bEnableFMO = OMX_FALSE;  
    omx_private->pVideoAvc.bEnableASO = OMX_FALSE;  
    omx_private->pVideoAvc.bEnableRS = OMX_FALSE;   

    omx_private->pVideoAvc.bMBAFF = OMX_FALSE;               
    omx_private->pVideoAvc.bEntropyCodingCABAC = OMX_FALSE;  
    omx_private->pVideoAvc.bWeightedPPrediction = OMX_FALSE; 
    omx_private->pVideoAvc.nWeightedBipredicitonMode = 0; 
    omx_private->pVideoAvc.bconstIpred = OMX_FALSE;
    omx_private->pVideoAvc.bDirect8x8Inference = OMX_FALSE;  
    omx_private->pVideoAvc.bDirectSpatialTemporal = OMX_FALSE;
    omx_private->pVideoAvc.nCabacInitIdc = 0;
    omx_private->pVideoAvc.eLoopFilterMode = OMX_VIDEO_AVCLoopFilterDisable;

    inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
    inPort->sVideoParam.eCompressionFormat = OMX_COLOR_FormatUnused;

  } 
  else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
    strcpy(omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/h263");
    omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;

    setHeader(&omx_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));    
    omx_private->pVideoH263.nPortIndex = 0x1;      
    omx_private->pVideoH263.eProfile = OMX_VIDEO_H263ProfileBaseline; //OMX_VIDEO_MPEG4ProfileCore
    omx_private->pVideoH263.eLevel = OMX_VIDEO_H263Level45;
    omx_private->pVideoH263.bPLUSPTYPEAllowed = OMX_FALSE;
    omx_private->pVideoH263.nAllowedPictureTypes = OMX_VIDEO_PictureTypeI | OMX_VIDEO_PictureTypeP;
    omx_private->pVideoH263.bForceRoundingTypeToZero = OMX_TRUE;
    omx_private->pVideoH263.nPictureHeaderRepetition = 0;
    omx_private->pVideoH263.nGOBHeaderInterval = 0;
    omx_private->pVideoH263.nPFrames = 10;
    omx_private->pVideoH263.nBFrames = 0;

    inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
    inPort->sVideoParam.eCompressionFormat = OMX_COLOR_FormatUnused;

  } 
}



OMX_ERRORTYPE omx_videoenc_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videoenc_component_PrivateType* omx_private;
	omx_private = openmaxStandComp->pComponentPrivate;;
	
	if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		return (omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_MPEG4_NAME));
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		return (omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_H264_NAME));
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		return (omx_videoenc_component_Constructor(openmaxStandComp, VIDEO_ENC_H263_NAME));
	}

	return OMX_ErrorNone;
}
/** The Constructor of the video decoder component
  * @param openmaxStandComp the component handle to be constructed
  * @param cComponentName is the name of the constructed component
  */
OMX_ERRORTYPE omx_videoenc_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName) {

  OMX_ERRORTYPE eError = OMX_ErrorNone;  
  omx_videoenc_component_PrivateType* omx_private;
  omx_base_video_PortType *inPort,*outPort;
  OMX_U32 i;

#ifdef HAVE_ANDROID_OS
	if (1) {
#else
	if(!openmaxStandComp->pComponentPrivate) {
#endif
		DBug_MSG( "In %s, allocating component\n", __func__);
		openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_videoenc_component_PrivateType));
		if(openmaxStandComp->pComponentPrivate == NULL) {
			return OMX_ErrorInsufficientResources;
		}
	} else {
		DBug_MSG( "In %s, Error Component %x Already Allocated\n", __func__, (int)openmaxStandComp->pComponentPrivate);
	}

	omx_private = openmaxStandComp->pComponentPrivate;
	omx_private->ports = NULL;

	eError = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

	omx_private->sPortTypesParam[OMX_PortDomainVideo].nStartPortNumber = 0;
	omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts = 2;

	/** Allocate Ports and call port constructor. */
	if (omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts && !omx_private->ports) {
		omx_private->ports = TCC_calloc(omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts, sizeof(omx_base_PortType *));
		if (!omx_private->ports) {
		LOGE( "%s, Port allocating error \n", __func__);
		return OMX_ErrorInsufficientResources;
		}
		for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
			omx_private->ports[i] = TCC_calloc(1, sizeof(omx_base_video_PortType));
			if (!omx_private->ports[i]) {
			LOGE( "%s, Port allocating error1 \n", __func__);
			return OMX_ErrorInsufficientResources;
			}
		}
	}

	base_video_port_Constructor(openmaxStandComp, &omx_private->ports[0], 0, OMX_TRUE);
	base_video_port_Constructor(openmaxStandComp, &omx_private->ports[1], 1, OMX_FALSE);

	/** now it's time to know the video coding type of the component */
	if(!strcmp(cComponentName, VIDEO_ENC_MPEG4_NAME)) { 
		omx_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
	}else if(!strcmp(cComponentName, VIDEO_ENC_H264_NAME)) { 
		omx_private->video_coding_type = OMX_VIDEO_CodingAVC;
	}else if(!strcmp(cComponentName, VIDEO_ENC_H263_NAME)) { 
		omx_private->video_coding_type = OMX_VIDEO_CodingH263;
	}else if (!strcmp(cComponentName, VIDEO_ENC_BASE_NAME)) {
		omx_private->video_coding_type = OMX_VIDEO_CodingUnused;
	} else {
		// IL client specified an invalid component name 
		LOGE( "%s, OMX_ErrorInvalidComponentName \n", __func__);
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
	inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE;
	inPort->sPortParam.format.video.xFramerate = (30 << 16);
	inPort->sPortParam.nBufferCountMin = 3;
	inPort->sPortParam.nBufferCountActual = NUM_IN_BUFFERS;
	inPort->sPortParam.format.video.eColorFormat = INPUT_ENCODED_COLOR_FMT;
	inPort->sVideoParam.eColorFormat = INPUT_ENCODED_COLOR_FMT;

	//common parameters related to output port
	outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	outPort->sPortParam.nBufferSize =  OUT_BUFFER_SIZE;
	outPort->sPortParam.format.video.xFramerate = (30 << 16);
	outPort->sPortParam.nBufferCountMin = NUM_OUT_BUFFERS;   
	outPort->sPortParam.nBufferCountActual = NUM_OUT_BUFFERS;

	if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
		outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
		outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingAVC;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		outPort->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;
		outPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingH263;
	}
	
	outPort->sPortParam.format.video.nBitrate = 64000;
	outPort->sPortParam.format.video.xFramerate = (30 << 16);
	
	/** settings of output port parameter definition */
	outPort->sVideoParam.xFramerate = (30 << 16);

	if(!omx_private->avCodecSyncSem) {
		omx_private->avCodecSyncSem = TCC_malloc(sizeof(tsem_t));
		if(omx_private->avCodecSyncSem == NULL) {
			LOGE( "%s, avCodecSyncSem  - OMX_ErrorInsufficientResources\n", __func__);
			return OMX_ErrorInsufficientResources;
		}
		tsem_init(omx_private->avCodecSyncSem, 0);
	}

	SetInternalVideoParameters(openmaxStandComp);

	omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;

	/** general configuration irrespective of any video formats
	* setting other parameters of omx_videodec_component_private  
	*/
	//  omx_private->avCodec = NULL;
	//  omx_private->avCodecContext= NULL;
	omx_private->avcodecReady = OMX_FALSE;
	omx_private->extradata = NULL;
	omx_private->extradata_size = 0;
	omx_private->BufferMgmtCallback = omx_videoenc_component_BufferMgmtCallback;
	omx_private->isVPUClosed = OMX_TRUE;
	omx_private->isEncError = OMX_FALSE;

	/** initializing the codec context etc that was done earlier by ffmpeglibinit function */
	omx_private->messageHandler = omx_videoenc_component_MessageHandler;
	omx_private->destructor = omx_videoenc_component_Destructor;

	openmaxStandComp->SetParameter = omx_videoenc_component_SetParameter;
	openmaxStandComp->GetParameter = omx_videoenc_component_GetParameter;
	openmaxStandComp->SetConfig    = omx_videoenc_component_SetConfig;
	openmaxStandComp->ComponentRoleEnum = omx_videoenc_component_ComponentRoleEnum;
	openmaxStandComp->GetExtensionIndex = omx_videoenc_component_GetExtensionIndex;

#if (!USE_OUTPUT_USE_BUFFER)
	//For reducing needless memory copy.
	openmaxStandComp->AllocateBuffer = omx_base_component_AllocateBuffer2;
	openmaxStandComp->UseBuffer = omx_base_component_UseBuffer2;
	openmaxStandComp->FreeBuffer = omx_base_component_FreeBuffer2;	

#endif

	if (omx_private->video_coding_type == OMX_VIDEO_CodingH263)
		omx_private->iConfigDataFlag = OMX_TRUE;
	else
		omx_private->iConfigDataFlag = OMX_FALSE;

	noVideoDecInstance++;

	if(noVideoDecInstance > MAX_COMPONENT_VIDEOENC) {
		LOGE( "%s, noVideoDecInstance > MAX_COMPONENT_VIDEOENC \n", __func__);
		return OMX_ErrorInsufficientResources;
	}


	//OMX_VIDEO_PARAM_PROFILELEVELTYPE structure
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->sProfileLevel, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
	omx_private->sProfileLevel.nPortIndex = 0x1;
	omx_private->sProfileLevel.nProfileIndex = 0;
	if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		omx_private->sProfileLevel.eProfile = OMX_VIDEO_MPEG4ProfileSimple;
		omx_private->sProfileLevel.eLevel = OMX_VIDEO_MPEG4Level2;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		omx_private->sProfileLevel.eProfile = OMX_VIDEO_AVCProfileBaseline;
		omx_private->sProfileLevel.eLevel = OMX_VIDEO_AVCLevel1;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		omx_private->sProfileLevel.eProfile = OMX_VIDEO_H263ProfileBaseline;
		omx_private->sProfileLevel.eLevel = OMX_VIDEO_H263Level45;
	}


	//OMX_CONFIG_ROTATIONTYPE SETTINGS ON INPUT PORT
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoOrientationType, OMX_CONFIG_ROTATIONTYPE);
	omx_private->VideoOrientationType.nPortIndex = OMX_DirInput;
	omx_private->VideoOrientationType.nRotation = -1;  //For all the YUV formats that are other than RGB


	//OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE settings of output port
	memset(&omx_private->VideoErrorCorrection, 0, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoErrorCorrection, OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE);
	omx_private->VideoErrorCorrection.nPortIndex = OMX_DirOutput;
	omx_private->VideoErrorCorrection.bEnableDataPartitioning = OMX_FALSE;	//As in node default is h263


	//OMX_VIDEO_PARAM_BITRATETYPE settings of output port
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoRateType, OMX_VIDEO_PARAM_BITRATETYPE);
	omx_private->VideoRateType.nPortIndex = OMX_DirOutput;
	omx_private->VideoRateType.eControlRate = OMX_Video_ControlRateVariable;
	omx_private->VideoRateType.nTargetBitrate = 64000;


	//OMX_CONFIG_FRAMERATETYPE default seetings (specified in khronos conformance test)
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoConfigFrameRateType, OMX_CONFIG_FRAMERATETYPE);
	omx_private->VideoConfigFrameRateType.nPortIndex = OMX_DirOutput;
	omx_private->VideoConfigFrameRateType.xEncodeFramerate = (30 << 16);

	//OMX_VIDEO_CONFIG_BITRATETYPE default settings (specified in khronos conformance test)
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoConfigBitRateType, OMX_VIDEO_CONFIG_BITRATETYPE);
	omx_private->VideoConfigBitRateType.nPortIndex = OMX_DirOutput;
	omx_private->VideoConfigBitRateType.nEncodeBitrate = 64000;


	//OMX_VIDEO_PARAM_QUANTIZATIONTYPE settings of output port
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoQuantType, OMX_VIDEO_PARAM_QUANTIZATIONTYPE);
	omx_private->VideoQuantType.nPortIndex = OMX_DirOutput;
	omx_private->VideoQuantType.nQpI = 15;
	omx_private->VideoQuantType.nQpP = 12;
	omx_private->VideoQuantType.nQpB = 12;


	//OMX_VIDEO_PARAM_VBSMCTYPE settings of output port
	memset(&omx_private->VideoBlockMotionSize, 0, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoBlockMotionSize, OMX_VIDEO_PARAM_VBSMCTYPE);
	omx_private->VideoBlockMotionSize.nPortIndex = OMX_DirOutput;
	omx_private->VideoBlockMotionSize.b16x16 = OMX_TRUE;


	//OMX_VIDEO_PARAM_MOTIONVECTORTYPE settings of output port
	memset(&omx_private->VideoMotionVector, 0, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoMotionVector, OMX_VIDEO_PARAM_MOTIONVECTORTYPE);
	omx_private->VideoMotionVector.nPortIndex = OMX_DirOutput;
	omx_private->VideoMotionVector.eAccuracy = OMX_Video_MotionVectorHalfPel;
	omx_private->VideoMotionVector.bUnrestrictedMVs = OMX_TRUE;
	omx_private->VideoMotionVector.sXSearchRange = 16;
	omx_private->VideoMotionVector.sYSearchRange = 16;


	//OMX_VIDEO_PARAM_INTRAREFRESHTYPE settings of output port
	memset(&omx_private->VideoIntraRefresh, 0, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoIntraRefresh, OMX_VIDEO_PARAM_INTRAREFRESHTYPE);
	omx_private->VideoIntraRefresh.nPortIndex = OMX_DirOutput;
	omx_private->VideoIntraRefresh.eRefreshMode = OMX_VIDEO_IntraRefreshCyclic;
	omx_private->VideoIntraRefresh.nCirMBs = 0;


	//OMX_CONFIG_INTRAREFRESHVOPTYPE settings of output port
	memset(&omx_private->VideoIFrame, 0, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
	OMX_CONF_INIT_STRUCT_PTR(&omx_private->VideoIFrame, OMX_CONFIG_INTRAREFRESHVOPTYPE);
	omx_private->VideoIFrame.nPortIndex = OMX_DirOutput;
	omx_private->VideoIFrame.IntraRefreshVOP = OMX_FALSE;

	return eError;
}


/** The destructor of the video decoder component
  */
OMX_ERRORTYPE omx_videoenc_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  if(omx_private->extradata) {
    TCC_free(omx_private->extradata);
    omx_private->extradata=NULL;
  }

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

  DBug_MSG( "Destructor of video decoder component is called\n");

  omx_base_filter_Destructor(openmaxStandComp);

  noVideoDecInstance--;

  return OMX_ErrorNone;
}


/** It initializates the FFmpeg framework, and opens an FFmpeg videodecoder of type specified by IL client 
  */ 
OMX_ERRORTYPE omx_videoenc_component_LibInit(omx_videoenc_component_PrivateType* omx_private) {

	OMX_U32 target_codecID;  
	OMX_S32 ret;
	int bitstream_format = STD_MPEG4;
	int fps, keyInterval, kbps;

	omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	omx_base_video_PortType *inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	
	fps = (inPort->sPortParam.format.video.xFramerate/(1 << 16));
	kbps = (outPort->sPortParam.format.video.nBitrate / 1024);
	
	if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		bitstream_format = STD_MPEG4;
		keyInterval = omx_private->pVideoMpeg4.nPFrames + 1;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		bitstream_format = STD_AVC;
		keyInterval = omx_private->pVideoAvc.nPFrames + 1;
	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		bitstream_format = STD_H263;
		keyInterval = omx_private->pVideoH263.nPFrames + 1;	
	}
	
//	fps = 26;
//	kbps = 8192;
//	keyInterval = fps*2;

	total_count = 0;
	memset(&omx_private->gsVEncInit, 0x00, sizeof(venc_init_t));
	memset(&omx_private->gsVEncSeqHeader, 0x00, sizeof(venc_seq_header_t));
	memset(&omx_private->gsVEncInput, 0x00, sizeof(venc_input_t));
	memset(&omx_private->gsVEncOutput, 0x00, sizeof(venc_output_t));
	memset(&omx_private->VideoConfigFrameRateType, 0x00, sizeof(OMX_CONFIG_FRAMERATETYPE));
	memset(&omx_private->VideoConfigBitRateType, 0x00, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
	
	omx_private->gsVEncInit.m_iBitstreamFormat	= bitstream_format;
	omx_private->gsVEncInit.m_iPicWidth			= outPort->sPortParam.format.video.nFrameWidth;
	omx_private->gsVEncInit.m_iPicHeight 		= outPort->sPortParam.format.video.nFrameHeight;
	omx_private->gsVEncInit.m_iFrameRate 		= fps;
	omx_private->gsVEncInit.m_iTargetKbps		= kbps;
	omx_private->gsVEncInit.m_iKeyInterval		= keyInterval; // only first picture is I
	omx_private->gsVEncInit.m_iAvcFastEncoding	= 0;

	LOGI("ENC Info :: %dx%d, %d bps, %d interval, %d Kbps", omx_private->gsVEncInit.m_iPicWidth, omx_private->gsVEncInit.m_iPicHeight, 
					omx_private->gsVEncInit.m_iFrameRate, omx_private->gsVEncInit.m_iKeyInterval, omx_private->gsVEncInit.m_iTargetKbps);
	
	// modified by shmin for M2TS
	omx_private->pVideoAvc.bUsedNALStart = 1;

#if 1
		{
			tENC_INIT_PARAMS init;
	
			init.codecFormat		= CODEC_FORMAT_H264;
			init.picWidth			= outPort->sPortParam.format.video.nFrameWidth;
			init.picHeight			= outPort->sPortParam.format.video.nFrameHeight;
			init.frameRate			= fps;		
			init.targetKbps 		= kbps; 
			init.keyFrameInterval	= keyInterval;
			init.sliceMode			= 1; // 0:Off, 1: On	
			init.sliceSizeMode		= 0; // 0:bits, 1:MBs 
			init.sliceSize			= 4*1024; //4KB
			init.use_NalStartCode	= 1;
			
			if(ENCODER_INIT(&init) < 0)
				return OMX_ErrorHardware;				
		}
#else
	if( (ret = venc_vpu( VENC_INIT, NULL, &omx_private->gsVEncInit, (void*)&omx_private->pVideoAvc.bUsedNALStart )) < 0 )
	{
		LOGE( "[Err:%ld] VENC_INIT failed", ret );

		if(ret != -VPU_ENV_INIT_ERROR)
			venc_vpu( VENC_CLOSE, NULL, NULL, NULL );
		
		return OMX_ErrorHardware;
	}
#endif

	omx_private->isVPUClosed = OMX_FALSE;

	DBug_MSG( "VENC_INIT Success!! size = %ld x %ld \n", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight);

	if(omx_private->isEncError == OMX_FALSE)
		tsem_up(omx_private->avCodecSyncSem);

  return OMX_ErrorNone;
}

/** It Deinitializates the ffmpeg framework, and close the ffmpeg video decoder of selected coding type
  */
void omx_videoenc_component_LibDeinit(omx_videoenc_component_PrivateType* omx_private) {
	OMX_S32 ret;

	if(	omx_private->isVPUClosed == OMX_FALSE)
	{
#if 1
		if(ENCODER_CLOSE() < 0)
			return OMX_ErrorHardware;				
#else
	
		if( (ret = venc_vpu( VENC_CLOSE, NULL, NULL, NULL )) < 0 )
		{
			LOGE( "[Err:%ld] VENC_CLOSE failed", ret );
			return OMX_ErrorHardware;
		}
#endif
		
		omx_private->isVPUClosed = OMX_TRUE;
	}
}

/** The Initialization function of the video decoder
  */
OMX_ERRORTYPE omx_videoenc_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp) {

  omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE eError = OMX_ErrorNone;

  /** Temporary First Output buffer size */
  omx_private->inputCurrBuffer = NULL;
  omx_private->inputCurrLength = 0;
  omx_private->isFirstBuffer = 1;
  omx_private->isNewBuffer = 1;

  return eError;
}

/** The Deinitialization function of the video decoder  
  */
OMX_ERRORTYPE omx_videoenc_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {

  omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE eError = OMX_ErrorNone;

  if (omx_private->avcodecReady) {
    omx_videoenc_component_LibDeinit(omx_private);
    omx_private->avcodecReady = OMX_FALSE;
  }

  return eError;
} 

/** Executes all the required steps after an output buffer frame-size has changed.
*/
static inline void UpdateFrameSize(OMX_COMPONENTTYPE *openmaxStandComp) {

  omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
  omx_base_video_PortType *inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
  outPort->sPortParam.format.video.nFrameWidth = inPort->sPortParam.format.video.nFrameWidth;
  outPort->sPortParam.format.video.nFrameHeight = inPort->sPortParam.format.video.nFrameHeight;
  
  switch(inPort->sVideoParam.eColorFormat) {
    case OMX_COLOR_FormatYUV420Planar:
      if(inPort->sPortParam.format.video.nFrameWidth && inPort->sPortParam.format.video.nFrameHeight) {
	 inPort->sPortParam.nBufferSize = IN_BUFFER_SIZE; 
        //inPort->sPortParam.nBufferSize = inPort->sPortParam.format.video.nFrameWidth * inPort->sPortParam.format.video.nFrameHeight * 3/2;
      }
      break;
    default:
      if(inPort->sPortParam.format.video.nFrameWidth && inPort->sPortParam.format.video.nFrameHeight) {
        inPort->sPortParam.nBufferSize = inPort->sPortParam.format.video.nFrameWidth * inPort->sPortParam.format.video.nFrameHeight * 3;
      }
      break;
  }

}

/** This function is used to process the input buffer and provide one output buffer
  */
void omx_videoenc_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer	, OMX_BUFFERHEADERTYPE* pOutputBuffer) {

	omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_S32 ret;
	OMX_S32 nOutputFilled = 0;
	OMX_U8* outputCurrBuffer;
	OMX_U32* InputCurrBuffer;
	OMX_S32 nLen = 0;
	int internalOutputFilled=0;
	OMX_U32 output_len;
	
	omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	/** Fill up the current input buffer when a new buffer has arrived */
	if(omx_private->isNewBuffer) {
		omx_private->inputCurrBuffer = pInputBuffer->pBuffer;
		omx_private->inputCurrLength = pInputBuffer->nFilledLen;
		omx_private->isNewBuffer = 0;
		DBug_MSG( "New Buffer FilledLen = %d\n", (int)pInputBuffer->nFilledLen);
		DBug_MSG( "----------------------> new buffer!! \n");	
	}

	outputCurrBuffer = pOutputBuffer->pBuffer;
	pOutputBuffer->nFilledLen = 0;
	pOutputBuffer->nOffset = 0;

#if 0
	if(omx_private->isEncError == OMX_TRUE)
	{
		LOGI("Restore Encode-Error");
		omx_private->isEncError = OMX_FALSE;
		
		if(OMX_ErrorNone != omx_videoenc_component_LibInit(omx_private))
			goto ERR_PROCESS;
		
		omx_private->iConfigDataFlag = OMX_TRUE;
	}

	if(omx_private->isVPUClosed == OMX_TRUE)
	{
		pInputBuffer->nFilledLen = 0;
		LOGE("Vpu already is closed because of Error!!");
		return;
	}
#endif

	while (!nOutputFilled) {
		if (omx_private->isFirstBuffer) {
			tsem_down(omx_private->avCodecSyncSem);
			omx_private->isFirstBuffer = 0;
		}

#if 1
		{
			tENC_FRAME_INPUT Input;
			tENC_FRAME_OUTPUT Output;

			Input.inputStreamAddr	= (OMX_U8*)omx_private->inputCurrBuffer; 
			Input.inputStreamSize	= omx_private->inputCurrLength; 
			Input.nTimeStamp		= pInputBuffer->nTimeStamp / 1000; 
			Input.isForceIFrame 	= 0; 
			Input.isSkipFrame		= 0; 
			Input.frameRate 		= (omx_private->VideoConfigBitRateType.nEncodeBitrate / 1024); 
			Input.targetKbps		= (outPort->sPortParam.format.video.xFramerate/(1 << 16)); 
			Input.Qp				= 0; 
		
			if(omx_private->iConfigDataFlag == OMX_FALSE)
			{
				if(ENCODER_ENC(&Input, &Output) < 0)
					goto ERR_PROCESS;	

				pOutputBuffer->pBuffer = Output.outputStreamAddr;
				pOutputBuffer->nTimeStamp = Output.nTimeStamp * 1000; 
				pOutputBuffer->nFilledLen = Output.headerLen;		
				//pOutputBuffer->nFilledLen = Output.frameLen;
				pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
				pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

				omx_private->iConfigDataFlag = OMX_TRUE;
			}
			else
			{
				if(omx_private->VideoIFrame.IntraRefreshVOP){
					Input.isForceIFrame = 1;
					omx_private->VideoIFrame.IntraRefreshVOP = OMX_FALSE;					
				}
				else{
					Input.isForceIFrame = 0;
				}	
				
				if(ENCODER_ENC(&Input, &Output) < 0)
					goto ERR_PROCESS;	

				pOutputBuffer->pBuffer = Output.outputStreamAddr;
				pOutputBuffer->nTimeStamp = Output.nTimeStamp * 1000; 
				pOutputBuffer->nFilledLen = Output.frameLen;

				if(Output.picType == TYPE_I_FRAME)
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
				
				pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;

				omx_private->isNewBuffer = 1;
				pInputBuffer->nFilledLen = 0;

				if(Output.m_iSliceCount > 1 && DEBUG_ON == 1)
				{
					unsigned char *p = Output.outputStreamAddr;
					unsigned int *pSliceSize = Output.m_pSliceInfo;
					unsigned int total_bytes = 0;
					int i = 0;
					
					for( i=0; i<Output.m_iSliceCount; i++ ) 
					{
						if(total_count < 5 )
						{
							DBug_MSG( "[%2d/%2d] %d bits, %d bytes ", i, Output.m_iSliceCount, pSliceSize[i]*8, pSliceSize[i] );
							DBug_MSG( " 	 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", p[total_bytes+0], p[total_bytes+1], p[total_bytes+2], 
									 p[total_bytes+3], p[total_bytes+4], p[total_bytes+5], p[total_bytes+6], p[total_bytes+7]);
							total_bytes += pSliceSize[i];
							DBug_MSG( "  ~	 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", p[total_bytes-8], p[total_bytes-7], p[total_bytes-6], 
									 p[total_bytes-5], p[total_bytes-4], p[total_bytes-3], p[total_bytes-2], p[total_bytes-1]);
						}
					}
				}
	
				total_count++;
				
			}
		}
#else
		//Send the first output buffer as vol header in case of m4v format
		{
			if(omx_private->iConfigDataFlag == OMX_FALSE)
			{
				omx_private->gsVEncSeqHeader.m_SeqHeaderBuffer[PA]	= NULL;
				omx_private->gsVEncSeqHeader.m_SeqHeaderBuffer[VA]	= NULL;
				omx_private->gsVEncSeqHeader.m_iSeqHeaderBufferSize = 0;
				
				if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) 
				{		
					ret = venc_vpu( VENC_SEQ_HEADER, NULL, &omx_private->gsVEncSeqHeader, NULL );
					if( ret < 0 )
					{
						LOGE( "[Err%d] VENC_SEQ_HEADER failed\n", ret );
						goto ERR_PROCESS;
					}
					
					pOutputBuffer->pBuffer = omx_private->gsVEncSeqHeader.m_pSeqHeaderOut;
					pOutputBuffer->nFilledLen = output_len = omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
					pOutputBuffer->nTimeStamp = 0;//pInputBuffer->nTimeStamp;
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
					DBug_MSG(" VOL Header :: Length = %d !!", output_len);
				}
				else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) 
				{
					ret = venc_vpu( VENC_SEQ_HEADER, NULL, &omx_private->gsVEncSeqHeader, NULL );
					if( ret < 0 )
					{
						LOGE( "[Err%d] VENC_SEQ_HEADER failed\n", ret );
						goto ERR_PROCESS;
					}
					DBug_MSG(" SPS - %d !!", omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);		
					
					pOutputBuffer->pBuffer = omx_private->gsVEncSeqHeader.m_pSeqHeaderOut;
					pOutputBuffer->nFilledLen = omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
					pOutputBuffer->nTimeStamp = 0;//pInputBuffer->nTimeStamp;	
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
					
					ret = venc_vpu( VENC_SEQ_HEADER, NULL, &omx_private->gsVEncSeqHeader, NULL );
					if( ret < 0 )
					{
						LOGE( "[Err%d] VENC_SEQ_HEADER failed\n", ret );
						goto ERR_PROCESS;
					}
					DBug_MSG(" PPS - %d !!", omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize);		
					
					pOutputBuffer->nFilledLen += omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;
					output_len += omx_private->gsVEncSeqHeader.m_iSeqHeaderOutSize;

					DBug_MSG(" CodecConfig Data :: %d !!", pOutputBuffer->nFilledLen);		
				}
				
				omx_private->iConfigDataFlag = OMX_TRUE;
			}
			else
			{
				output_len = pOutputBuffer->nAllocLen;

				omx_private->gsVEncInput.m_bCbCrInterleaved = 0;
				InputCurrBuffer = (OMX_U32*)omx_private->inputCurrBuffer;
				omx_private->gsVEncInput.m_pInputY 		 	= InputCurrBuffer[0];
				omx_private->gsVEncInput.m_pInputCbCr[0] 	= omx_private->gsVEncInput.m_pInputY + (omx_private->gsVEncInit.m_iPicWidth*omx_private->gsVEncInit.m_iPicHeight);
				omx_private->gsVEncInput.m_pInputCbCr[1] 	= omx_private->gsVEncInput.m_pInputCbCr[0] + (omx_private->gsVEncInit.m_iPicWidth*omx_private->gsVEncInit.m_iPicHeight/4);
	
	#ifdef REMOVE_RC_AUTO_SKIP
				omx_private->gsVEncInput.m_iChangeRcParamFlag = (0x20 | 0x1);
	#else
				omx_private->gsVEncInput.m_iChangeRcParamFlag = 0;
	#endif
				if(omx_private->VideoConfigBitRateType.nEncodeBitrate != 0 && omx_private->VideoConfigBitRateType.nEncodeBitrate != outPort->sPortParam.format.video.nBitrate)
				{
					outPort->sPortParam.format.video.nBitrate = omx_private->VideoConfigBitRateType.nEncodeBitrate;
					
					omx_private->gsVEncInput.m_iChangeRcParamFlag |= 0x2;
					omx_private->gsVEncInput.m_iChangeTargetKbps = (outPort->sPortParam.format.video.nBitrate / 1024);
					LOGE("Bitrate- Change(%d)", omx_private->gsVEncInput.m_iChangeTargetKbps);
				}
				else if(omx_private->VideoConfigFrameRateType.xEncodeFramerate != 0 && omx_private->VideoConfigFrameRateType.xEncodeFramerate != outPort->sPortParam.format.video.xFramerate)
				{
					outPort->sPortParam.format.video.xFramerate = omx_private->VideoConfigFrameRateType.xEncodeFramerate;
					
					omx_private->gsVEncInput.m_iChangeRcParamFlag |= 0x4;
					omx_private->gsVEncInput.m_iChangeFrameRate = (outPort->sPortParam.format.video.xFramerate/(1 << 16));
					LOGE("FrameRate- Change(%d)", omx_private->gsVEncInput.m_iChangeFrameRate);
				}

				if(omx_private->VideoIFrame.IntraRefreshVOP){
					omx_private->gsVEncInput.request_IntraFrame = 1;
					omx_private->VideoIFrame.IntraRefreshVOP = OMX_FALSE;					
					LOGE("IntraRefreshVOP");
				}
				else{
					omx_private->gsVEncInput.request_IntraFrame = 0;
				}
					
				ret = venc_vpu( VENC_ENCODE, NULL, &omx_private->gsVEncInput, &omx_private->gsVEncOutput );
				if( ret < 0 )
				{
					pInputBuffer->nFilledLen = 0;
					omx_private->isNewBuffer = 1;
					omx_private->isEncError = OMX_TRUE;						
					
					LOGE( "[Err%d] VENC_ENCODE failed\n", ret );
					goto ERR_PROCESS;
				}
				
				output_len = omx_private->gsVEncOutput.m_iBitstreamOutSize;
				pOutputBuffer->pBuffer = omx_private->gsVEncOutput.m_pBitstreamOut;
				
				 // IFrame (SyncFrame)
				if( (omx_private->gsVEncOutput.m_iPicType == VENC_PIC_TYPE_I) && output_len > 0)
				{
					DBug_MSG(" I-Frame for Sync :: Frm_size = %ld !!", output_len);

					//This flag is set when the buffer content contains a coded sync frame 
					pOutputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
				}
				//Attach the end of frame flag while sending out the last piece of output buffer
				pOutputBuffer->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;


				internalOutputFilled = 1;
				nLen += omx_private->inputCurrLength;

				if ( nLen >= 0 && internalOutputFilled) {
					omx_private->inputCurrBuffer += nLen;
					omx_private->inputCurrLength -= nLen;
					pInputBuffer->nFilledLen -= nLen;

					//Buffer is fully consumed. Request for new Input Buffer
					if(pInputBuffer->nFilledLen == 0) {
						omx_private->isNewBuffer = 1;
					}

					pOutputBuffer->nFilledLen += output_len;
					pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
				} 
				else
				{
					pInputBuffer->nFilledLen = 0;
					omx_private->isNewBuffer = 1; 
				}
			}
		}
#endif
		nOutputFilled = 1;
		
	}

	return;
	
ERR_PROCESS:
	pInputBuffer->nFilledLen = 0;
	if(	omx_private->isVPUClosed == OMX_FALSE)
	{
#if 0
		venc_vpu( VENC_CLOSE, NULL, NULL, NULL );
#endif
		omx_private->isVPUClosed = OMX_TRUE;

		if(omx_private->isEncError != OMX_TRUE)
		{
			(*(omx_private->callbacks->EventHandler))(openmaxStandComp, omx_private->callbackData,
								   OMX_EventError, OMX_ErrorHardware,
								   0, NULL);
		}
	}
	
	return;
}

OMX_ERRORTYPE omx_videoenc_component_SetParameter(
OMX_IN  OMX_HANDLETYPE hComponent,
OMX_IN  OMX_INDEXTYPE nParamIndex,
OMX_IN  OMX_PTR ComponentParameterStructure) {

  OMX_ERRORTYPE eError = OMX_ErrorNone;
  OMX_U32 portIndex;

  /* Check which structure we are being fed and make control its header */
  OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
  omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
  omx_base_video_PortType *port;
  if (ComponentParameterStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
	DBug_MSG( " IN  Setting parameter 0x%x \n", nParamIndex);

  switch(nParamIndex) {
    case OMX_IndexParamPortDefinition:
      {
        eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
        if(eError == OMX_ErrorNone) {
          OMX_PARAM_PORTDEFINITIONTYPE *pPortDef = (OMX_PARAM_PORTDEFINITIONTYPE*)ComponentParameterStructure;
          UpdateFrameSize (openmaxStandComp);
          portIndex = pPortDef->nPortIndex;
          port = (omx_base_video_PortType *)omx_private->ports[portIndex];
          port->sVideoParam.eColorFormat = port->sPortParam.format.video.eColorFormat;		  
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
        if (portIndex <= 1) {
          port = (omx_base_video_PortType *)omx_private->ports[portIndex];
          memcpy(&port->sVideoParam, pVideoPortFormat, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
          omx_private->ports[portIndex]->sPortParam.format.video.eColorFormat = port->sVideoParam.eColorFormat;

          if (portIndex == 1) {
            switch(port->sVideoParam.eColorFormat) {
              case OMX_COLOR_Format24bitRGB888 :
                omx_private->eOutFramePixFmt = PIX_FMT_RGB24;
                break; 
              case OMX_COLOR_Format24bitBGR888 :
                omx_private->eOutFramePixFmt = PIX_FMT_BGR24;
                break;
              case OMX_COLOR_Format32bitBGRA8888 :
                omx_private->eOutFramePixFmt = PIX_FMT_BGR32;
                break;
              case OMX_COLOR_Format32bitARGB8888 :
                omx_private->eOutFramePixFmt = PIX_FMT_RGB32;
                break; 
              case OMX_COLOR_Format16bitARGB1555 :
                omx_private->eOutFramePixFmt = PIX_FMT_RGB555;
                break;
              case OMX_COLOR_Format16bitRGB565 :
                omx_private->eOutFramePixFmt = PIX_FMT_RGB565;
                break; 
              case OMX_COLOR_Format16bitBGR565 :
                omx_private->eOutFramePixFmt = PIX_FMT_BGR565;
                break;
              default:
                omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;
                break;
            }
            UpdateFrameSize (openmaxStandComp);
          }
        } else {
          eError =   OMX_ErrorBadPortIndex;
        }
        break;
      }
    case OMX_IndexParamStandardComponentRole:
      {
        OMX_PARAM_COMPONENTROLETYPE *pComponentRole;
        pComponentRole = ComponentParameterStructure;
        if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_MPEG4_ROLE)) {
          omx_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
        SetInternalVideoParameters(openmaxStandComp);
        }else if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_H264_ROLE)) {
          omx_private->video_coding_type = OMX_VIDEO_CodingAVC;
        SetInternalVideoParameters(openmaxStandComp);
        }else if (!strcmp((char *)pComponentRole->cRole, VIDEO_ENC_H263_ROLE)) {
          omx_private->video_coding_type = OMX_VIDEO_CodingH263;
        SetInternalVideoParameters(openmaxStandComp);
        } else {
          eError =  OMX_ErrorBadParameter;
        }
        break;
      }
    case OMX_IndexParamVideoMpeg4:
      {
        OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4;
		
        pVideoMpeg4 = ComponentParameterStructure;
        memcpy(&omx_private->pVideoMpeg4, pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
        break;
      }
    case OMX_IndexParamVideoAvc:
      {
        OMX_VIDEO_PARAM_AVCTYPE *pVideoAvc;
		
        pVideoAvc = ComponentParameterStructure;
        memcpy(&omx_private->pVideoAvc, pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
        break;
      }
    case OMX_IndexParamVideoH263:
      {
        OMX_VIDEO_PARAM_H263TYPE *pVideoH263;
		
        pVideoH263 = ComponentParameterStructure;
        memcpy(&omx_private->pVideoH263, pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
        break;
      }

	case OMX_IndexParamVideoProfileLevelCurrent:
		memcpy(&omx_private->sProfileLevel, ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
	break;

	case OMX_IndexConfigVideoFramerate:
		memcpy(&omx_private->VideoConfigFrameRateType, ComponentParameterStructure, sizeof(OMX_CONFIG_FRAMERATETYPE));
		LOGI("OMX_IndexConfigVideoFramerate = %d [%d]", omx_private->VideoConfigFrameRateType.xEncodeFramerate, (omx_private->VideoConfigFrameRateType.xEncodeFramerate/(1 << 16)));
	break;

	case OMX_IndexConfigVideoBitrate:
		memcpy(&omx_private->VideoConfigBitRateType, ComponentParameterStructure, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
		LOGI("OMX_IndexConfigVideoBitrate = %d", omx_private->VideoConfigBitRateType.nEncodeBitrate);
	break;

	case OMX_IndexConfigCommonRotate:
	{
		OMX_CONFIG_ROTATIONTYPE*			 pVideoRotation;
		
		pVideoRotation = (OMX_CONFIG_ROTATIONTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoOrientationType, pVideoRotation, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
	}
	break;

	case OMX_IndexParamVideoErrorCorrection:
	{
		OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* pVideoErrCorr;
		
		pVideoErrCorr = (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoErrorCorrection, pVideoErrCorr, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
	}
	break;

	case OMX_IndexParamVideoBitrate:
	break;

	case OMX_IndexParamVideoQuantization:
	{
		OMX_VIDEO_PARAM_QUANTIZATIONTYPE*	 pVideoQuant;

		pVideoQuant = (OMX_VIDEO_PARAM_QUANTIZATIONTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoQuantType, pVideoQuant, sizeof(OMX_VIDEO_PARAM_QUANTIZATIONTYPE));
	}
	break;

	case OMX_IndexParamVideoVBSMC:
	{
		OMX_VIDEO_PARAM_VBSMCTYPE*			 pVideoBlock;

		pVideoBlock = (OMX_VIDEO_PARAM_VBSMCTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoBlockMotionSize, pVideoBlock, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
	}
	break;

	case OMX_IndexParamVideoMotionVector:
	{
		OMX_VIDEO_PARAM_MOTIONVECTORTYPE*	 pVideoMotionVector;

		pVideoMotionVector = (OMX_VIDEO_PARAM_MOTIONVECTORTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoMotionVector, pVideoMotionVector, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
	}
	break;

	case OMX_IndexParamVideoIntraRefresh:
	{
		OMX_VIDEO_PARAM_INTRAREFRESHTYPE*	 pVideoIntraRefresh;

		pVideoIntraRefresh = (OMX_VIDEO_PARAM_INTRAREFRESHTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoIntraRefresh, pVideoIntraRefresh, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
	}
	break;

	case OMX_IndexConfigVideoIntraVOPRefresh:
	{
		OMX_CONFIG_INTRAREFRESHVOPTYPE*	 pVideoIFrame;
	
		pVideoIFrame = (OMX_CONFIG_INTRAREFRESHVOPTYPE*) ComponentParameterStructure;
		memcpy(&omx_private->VideoIFrame, pVideoIFrame, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
	}
	break;
	
    default: /*Call the base component function*/
      eError = omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
	  break;
  }

	if(eError != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, eError);
  
  return eError;
}

OMX_ERRORTYPE omx_videoenc_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure) {

	omx_base_video_PortType *port;
	OMX_ERRORTYPE eError = OMX_ErrorNone;
	
	OMX_COMPONENTTYPE *openmaxStandComp = hComponent;
	omx_videoenc_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	if (ComponentParameterStructure == NULL) {
		return OMX_ErrorBadParameter;
	}
	DBug_MSG( " IN  Getting parameter 0x%x  \n", nParamIndex);
	
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
		if (pVideoPortFormat->nPortIndex <= 1) {
			port = (omx_base_video_PortType *)omx_private->ports[pVideoPortFormat->nPortIndex];
			memcpy(pVideoPortFormat, &port->sVideoParam, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
		} else {
			eError = OMX_ErrorBadPortIndex;
		}
		break;    
	}
	
	case OMX_IndexParamVideoMpeg4:
	{
		OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4;
		
		pVideoMpeg4 = ComponentParameterStructure;
		memcpy(pVideoMpeg4, &omx_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
		break;
	}
	case OMX_IndexParamVideoAvc:
	{
		OMX_VIDEO_PARAM_AVCTYPE *pVideoAvc;
		
		pVideoAvc = ComponentParameterStructure;
		memcpy(pVideoAvc, &omx_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
		break;
	}
	case OMX_IndexParamVideoH263:
	{
		OMX_VIDEO_PARAM_H263TYPE *pVideoH263;
		
		pVideoH263 = ComponentParameterStructure;
		memcpy(pVideoH263, &omx_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
		break;
	}
	
	case OMX_IndexParamStandardComponentRole:
	{
		OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
		pComponentRole = ComponentParameterStructure;
		if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
			break;
		}
		if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
			strcpy((char *)pComponentRole->cRole, VIDEO_ENC_MPEG4_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
			strcpy((char *)pComponentRole->cRole, VIDEO_ENC_H264_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
			strcpy((char *)pComponentRole->cRole, VIDEO_ENC_H263_ROLE);
		} else {
			strcpy((char *)pComponentRole->cRole,"\0");
		}
		break;
	}
	
	case PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX:
	{
		PV_OMXComponentCapabilityFlagsType *pCap_flags =
		(PV_OMXComponentCapabilityFlagsType *) ComponentParameterStructure;
		if (NULL == pCap_flags)
		{
			eError =  OMX_ErrorBadParameter;
		}
		else
		{
			memset(pCap_flags, 0, sizeof(PV_OMXComponentCapabilityFlagsType));
			pCap_flags->iIsOMXComponentMultiThreaded = OMX_TRUE;
#if USE_INPUT_USE_BUFFER
			pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc = OMX_TRUE;
#else
			pCap_flags->iOMXComponentSupportsExternalInputBufferAlloc = OMX_FALSE;
#endif
#if USE_OUTPUT_USE_BUFFER
			pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc = OMX_TRUE;
#else
			pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc = OMX_FALSE;
#endif
			pCap_flags->iOMXComponentSupportsMovableInputBuffers = OMX_TRUE;
			pCap_flags->iOMXComponentUsesNALStartCodes = OMX_FALSE;
			pCap_flags->iOMXComponentUsesFullAVCFrames = OMX_FALSE;
			pCap_flags->iOMXComponentSupportsPartialFrames = OMX_FALSE;
			pCap_flags->iOMXComponentCanHandleIncompleteFrames = OMX_FALSE;
		}

	}
	break;

	case OMX_IndexParamVideoProfileLevelCurrent:
		memcpy(ComponentParameterStructure, &omx_private->sProfileLevel, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
	break;

	case OMX_IndexConfigVideoFramerate:
		memcpy(ComponentParameterStructure, &omx_private->VideoConfigFrameRateType, sizeof(OMX_CONFIG_FRAMERATETYPE));
	break;

	case OMX_IndexConfigCommonRotate:
	{
		OMX_CONFIG_ROTATIONTYPE*			 pVideoRotation;
		
		pVideoRotation = (OMX_CONFIG_ROTATIONTYPE*) ComponentParameterStructure;
		memcpy(pVideoRotation, &omx_private->VideoOrientationType, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
	}
	break;

	case OMX_IndexParamVideoErrorCorrection:
	{
		OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* pVideoErrCorr;
		
		pVideoErrCorr = (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE*) ComponentParameterStructure;
		memcpy(pVideoErrCorr, &omx_private->VideoErrorCorrection, sizeof(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE));
	}
	break;

	case OMX_IndexParamVideoBitrate:
	{
		OMX_VIDEO_PARAM_BITRATETYPE*		 pVideoRateControl;

		pVideoRateControl = (OMX_VIDEO_PARAM_BITRATETYPE*) ComponentParameterStructure;
		memcpy(pVideoRateControl, &omx_private->VideoRateType, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
	}
	break;

	case OMX_IndexParamVideoQuantization:
	{
		OMX_VIDEO_PARAM_QUANTIZATIONTYPE*	 pVideoQuant;

		pVideoQuant = (OMX_VIDEO_PARAM_QUANTIZATIONTYPE*) ComponentParameterStructure;
		memcpy(pVideoQuant, &omx_private->VideoQuantType, sizeof(OMX_VIDEO_PARAM_QUANTIZATIONTYPE));
	}
	break;

	case OMX_IndexParamVideoVBSMC:
	{
		OMX_VIDEO_PARAM_VBSMCTYPE*			 pVideoBlock;

		pVideoBlock = (OMX_VIDEO_PARAM_VBSMCTYPE*) ComponentParameterStructure;
		memcpy(pVideoBlock, &omx_private->VideoBlockMotionSize, sizeof(OMX_VIDEO_PARAM_VBSMCTYPE));
	}
	break;

	case OMX_IndexParamVideoMotionVector:
	{
		OMX_VIDEO_PARAM_MOTIONVECTORTYPE*	 pVideoMotionVector;

		pVideoMotionVector = (OMX_VIDEO_PARAM_MOTIONVECTORTYPE*) ComponentParameterStructure;
		memcpy(pVideoMotionVector, &omx_private->VideoMotionVector, sizeof(OMX_VIDEO_PARAM_MOTIONVECTORTYPE));
	}
	break;

	case OMX_IndexParamVideoIntraRefresh:
	{
		OMX_VIDEO_PARAM_INTRAREFRESHTYPE*	 pVideoIntraRefresh;

		pVideoIntraRefresh = (OMX_VIDEO_PARAM_INTRAREFRESHTYPE*) ComponentParameterStructure;
		memcpy(pVideoIntraRefresh, &omx_private->VideoIntraRefresh, sizeof(OMX_VIDEO_PARAM_INTRAREFRESHTYPE));
	}
	break;

	case OMX_IndexConfigVideoIntraVOPRefresh:
	{
		OMX_CONFIG_INTRAREFRESHVOPTYPE*	 pVideoIFrame;
	
		pVideoIFrame = (OMX_CONFIG_INTRAREFRESHVOPTYPE*) ComponentParameterStructure;
		memcpy(pVideoIFrame, &omx_private->VideoIFrame, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
	}
	break;

	case OMX_IndexParamVideoProfileLevelQuerySupported:
	{
		VIDEO_PROFILE_LEVEL_TYPE* pProfileLevel = NULL;
		OMX_U32 nNumberOfProfiles = 0;
		OMX_VIDEO_PARAM_PROFILELEVELTYPE *pParamProfileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)ComponentParameterStructure;

		/* Choose table based on compression format */
		switch(omx_private->video_coding_type)
		{
			case OMX_VIDEO_CodingH263:
				pProfileLevel = SupportedH263ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedH263ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
			break;
			case OMX_VIDEO_CodingMPEG4:
				pProfileLevel = SupportedMPEG4ProfileLevels;
				nNumberOfProfiles = sizeof(SupportedMPEG4ProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
			break;
			case OMX_VIDEO_CodingAVC:
				pProfileLevel = SupportedAVCProfileLevels;
				nNumberOfProfiles = sizeof(SupportedAVCProfileLevels) / sizeof (VIDEO_PROFILE_LEVEL_TYPE);
			break;
			default:
				return OMX_ErrorBadParameter;
		}

		if((pParamProfileLevel->nProfileIndex < 0) || (pParamProfileLevel->nProfileIndex >= (nNumberOfProfiles - 1)))
			return OMX_ErrorBadParameter;
		/* Point to table entry based on index */
		pProfileLevel += pParamProfileLevel->nProfileIndex;

		/* -1 indicates end of table */
		if(pProfileLevel->nProfile != -1) {
			pParamProfileLevel->eProfile = pProfileLevel->nProfile;
			pParamProfileLevel->eLevel = pProfileLevel->nLevel;
			eError = OMX_ErrorNone;
		}
		else {
			eError = OMX_ErrorNoMore;
		}
	}
	break;
	
	default: /*Call the base component function*/
		eError  = omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
	break;
  }

	if(eError != OMX_ErrorNone)
		LOGE("ERROR %s :: nParamIndex = 0x%x, error(0x%x)", __func__, nParamIndex, eError);

  return OMX_ErrorNone;
  
}

OMX_ERRORTYPE omx_videoenc_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp,internalRequestMessageType *message) {
  omx_videoenc_component_PrivateType* omx_private = (omx_videoenc_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err;
  OMX_STATETYPE eCurrentState = omx_private->state;

  DBug_MSG( "In %s\n", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateExecuting ) && (omx_private->state == OMX_StateIdle)) {
      if (!omx_private->avcodecReady) {
        err = omx_videoenc_component_LibInit(omx_private);
        if (err != OMX_ErrorNone) {
          return OMX_ErrorNotReady;
        }
        omx_private->avcodecReady = OMX_TRUE;
      }
    } 
    else if ((message->messageParam == OMX_StateIdle ) && (omx_private->state == OMX_StateLoaded)) {
      err = omx_videoenc_component_Initialize(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        LOGE( "In %s Video Decoder Init Failed Error=%x\n",__func__,err); 
        return err;
      } 
    } else if ((message->messageParam == OMX_StateLoaded) && (omx_private->state == OMX_StateIdle)) {
      err = omx_videoenc_component_Deinit(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        LOGE( "In %s Video Decoder Deinit Failed Error=%x\n",__func__,err); 
        return err;
      } 
    }
  }
  // Execute the base message handling
  err =  omx_base_component_MessageHandler(openmaxStandComp,message);

  if (message->messageType == OMX_CommandStateSet){
   if ((message->messageParam == OMX_StateIdle  ) && (eCurrentState == OMX_StateExecuting)) {
#ifndef HAVE_ANDROID_OS	// ZzaU:: to sync call-sequence with opencore!!	   	
      if (omx_private->avcodecReady) {
        omx_videoenc_component_LibDeinit(omx_private);
        omx_private->avcodecReady = OMX_FALSE;
      }
#endif
    }
  }
  return err;
}
OMX_ERRORTYPE omx_videoenc_component_ComponentRoleEnum(
  OMX_IN OMX_HANDLETYPE hComponent,
  OMX_OUT OMX_U8 *cRole,
  OMX_IN OMX_U32 nIndex) {

  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_videoenc_component_PrivateType* omx_private = (omx_videoenc_component_PrivateType*)openmaxStandComp->pComponentPrivate;

  if (nIndex == 0) {
		if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		    strcpy((char *)cRole, VIDEO_ENC_MPEG4_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		    strcpy((char *)cRole, VIDEO_ENC_H264_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		    strcpy((char *)cRole, VIDEO_ENC_H263_ROLE);
		}
  } else{
		return OMX_ErrorUnsupportedIndex;
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_videoenc_component_SetConfig(
  OMX_HANDLETYPE hComponent,
  OMX_INDEXTYPE nIndex,
  OMX_PTR pComponentConfigStructure) {

  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_VENDOR_EXTRADATATYPE* pExtradata;

  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_videoenc_component_PrivateType* omx_private = (omx_videoenc_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  if (pComponentConfigStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
  DBug_MSG( "   Setting configuration %i\n", nIndex);
  /* Check which structure we are being fed and fill its header */
  switch (nIndex) {
    case OMX_IndexVendorVideoExtraData :
      pExtradata = (OMX_VENDOR_EXTRADATATYPE*)pComponentConfigStructure;
      if (pExtradata->nPortIndex <= 1) {
        /** copy the extradata in the codec context private structure */
        omx_private->extradata_size = (OMX_U32)pExtradata->nDataSize;
        if(omx_private->extradata_size > 0) {
          if(omx_private->extradata) {
            TCC_free(omx_private->extradata);
          }
          omx_private->extradata = (unsigned char *)TCC_malloc((int)pExtradata->nDataSize*sizeof(char));
         // memcpy(omx_private->extradata,(unsigned char*)(pExtradata->pData),pExtradata->nDataSize);
        } else {
                  DBug_MSG("extradata size is 0 !!!\n");
        }
      } else {
          return OMX_ErrorBadPortIndex;
      }
      break;

    default: // delegate to superclass
      return omx_base_component_SetConfig(hComponent, nIndex, pComponentConfigStructure);
  }
  return err;
}

OMX_ERRORTYPE omx_videoenc_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType) {

  DBug_MSG("In  %s \n",__func__);

  if(strcmp(cParameterName,"OMX.tcc.index.config.videoextradata") == 0) {
    *pIndexType = OMX_IndexVendorVideoExtraData;
  } else {
    return OMX_ErrorBadParameter;
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
  OMX_ERRORTYPE err = OMX_ErrorNone;
	//LOGE("OMX_ComponentInit \n");
	err = omx_videoenc_component_Constructor(openmaxStandComp,cCompontName);
	return err;
}
