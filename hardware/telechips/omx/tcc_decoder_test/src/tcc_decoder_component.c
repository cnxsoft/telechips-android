/**
  @file src/components/ffmpeg/omx_videodec_component.c
  
  This component implements H.264 / MPEG-4 AVC video decoder. 
  The H.264 / MPEG-4 AVC Video decoder is based on the FFmpeg software library.

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

  $Date: 2009/03/10 13:33:28 $
  Revision $Rev: 557 $
  Author $Author: B060934 $
  Android revised by ZzaU.
*/

#include <omxcore.h>
#include <omx_base_video_port.h>
#include <tcc_decoder_component.h>
#include <decoder.h>
#include<OMX_Video.h>

#include <lcd_resolution.h>

#include <mach/TCC_VPU_CODEC.h>
#include <vdec.h>
#include <cdk.h>
#include "cdk_android.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "tcc_video_config_data.h"

#include <mach/tccfb_ioctrl.h>

#ifdef HAVE_ANDROID_OS
#define LOG_TAG	"OMX_TCC_VIDEO_DEC"
#include <utils/Log.h>

#include <cutils/properties.h>

#ifdef DIVX_DRM5
#include <divx_drm5_Ex.h>
#endif

/* Option for TS frame defragmentation */
#if 1
#define FRAME_DEFRAGMENTATION_TYPE_NONE          0
#define LOGMSG(x...)
#define LOGERR(x...)
#define LOGINFO(x...)
#else
#define DEFRAGMENT_INPUT_FRAME
#ifdef DEFRAGMENT_INPUT_FRAME
#define DEFRAGMENT_INPUT_FRAME_AVC
//#define DEFRAGMENT_INPUT_FRAME_VC1

#ifdef DEFRAGMENT_INPUT_FRAME_AVC
#define BYTE_ORDERING(x) (((x) << 24) | (((x) >> 24) & 0xff) | (((x) << 8) & 0xff0000) | (((x) >> 8) & 0xff00)) 
#define FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
#endif 

//#define DEBUG_FRAME_DEFRAGMENTATION
#ifdef DEBUG_FRAME_DEFRAGMENTATION
	#define PEEK_DEFRAGMENTED_FRAME

	#define LOGMSG(x...)    LOGD(x)
	#define LOGERR(x...)    LOGE(x)
	#define LOGINFO(x...)   LOGI(x)
#else
	#define LOGMSG(x...)
	#define LOGERR(x...)
	#define LOGINFO(x...)
#endif

#define FRAME_DEFRAGMENTATION_TYPE_NONE          0
#define FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN    1
#define FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN    2

//#define DEBUG_INPUT_FRAME_FILTER_BY_PTS
#endif // DEFRAGMENT_INPUT_FRAME
#endif

#define TS_TIMESTAMP_CORRECTION

typedef struct TCC_PLATFORM_PRIVATE_PMEM_INFO
{
	/* pmem file descriptor */
	unsigned int pmem_fd;

	unsigned int offset[6];

	unsigned int optional_info[8];
} TCC_PLATFORM_PRIVATE_PMEM_INFO;

static int DEBUG_ON = 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}
#define DPRINTF_DEC(msg...) //LOGI( ": " msg);
#define DPRINTF_DEC_STATUS(msg...) //LOGD( ": " msg);

#define HARDWARE_CODEC	1
#define USE_EXTERNAL_BUFFER 0
#define USE_TCC_PARSER 0 // if you wanna use the TCC MP4 parser, enable this one

#define CONFIG_DATA_SIZE 	(1024*1024)
#define MIN_NAL_STARTCODE_LEN 	    3
#define MAX_NAL_STARTCODE_LEN 	    4

static unsigned int 	Display_index[VPU_BUFF_COUNT] ={0,};
static unsigned int 	out_index, in_index, frm_clear;
static unsigned int 	max_fifo_cnt = VPU_BUFF_COUNT;

#define OMX_BUFF_OFFSET_UNASSIGNED	0xFFFFFFFF
#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
#define MARK_BUFF_NOT_USED	0xFFFFFFFF
#define MAX_CHECK_COUNT_FOR_CLEAR	30

static unsigned int 	Display_Buff_ID[VPU_BUFF_COUNT] ={0,};
static unsigned int 	buffer_unique_id;
static unsigned int		used_fifo_count;
static int g_hFb = -1 ;
//video display mode related with vsync
static int iVsyncMode;
#endif
static unsigned int 	video_dec_idx = 0;

static OMX_BOOL gHDMIOutput = OMX_FALSE;

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
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level10},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level20},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level30},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level40},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level50},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level60},
  {OMX_VIDEO_H263ProfileISWV2, OMX_VIDEO_H263Level70},
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
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level0b},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level1},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level2},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level3},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level4},
  {OMX_VIDEO_MPEG4ProfileAdvancedSimple, OMX_VIDEO_MPEG4Level5},
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
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileBaseline, OMX_VIDEO_AVCLevel51},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel1},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel1b},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel11},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel12},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel13},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel2},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel21},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel22},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel3},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel31},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileMain, OMX_VIDEO_AVCLevel51},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel1},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel1b},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel11},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel12},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel13},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel2},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel21},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel22},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel3},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel31},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel32},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel4},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel41},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel42},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel5},
  {OMX_VIDEO_AVCProfileHigh, OMX_VIDEO_AVCLevel51},
  {-1,-1}};

#if 0
static cdk_func_t* gspfVDec;

static cdk_func_t* gspfVDecList[VCODEC_MAX] =
{
	vdec_vpu, //STD_AVC
	vdec_vpu, //STD_VC1
	vdec_vpu, //STD_MPEG2
	vdec_vpu, //STD_MPEG4
	vdec_vpu, //STD_H263
	vdec_vpu, //STD_DIV3
	vdec_vpu, //STD_RV
	vdec_vpu, //
#ifdef INCLUDE_WMV78_DEC
	vdec_WMV78, //STD_WMV78
#else
	0,
#endif
#ifdef INCLUDE_SORENSON263_DEC
	vdec_sorensonH263dec,	//STD_SORENSON263
#else
	vdec_vpu,
#endif
#ifdef JPEG_DECODE_FOR_MJPEG
	vdec_jpeg
#else
	vdec_vpu
#endif
};
#endif

/** The output decoded color format */
#ifdef HARDWARE_CODEC
#define OUTPUT_DECODED_COLOR_FMT OMX_COLOR_FormatYUV420SemiPlanar //0x7FA30C00     // special value for hardware codec
#else
#define OUTPUT_DECODED_COLOR_FMT OMX_COLOR_FormatYUV420Planar
#endif

#define RMVB_DECODER_TR_TEST
#ifdef RMVB_DECODER_TR_TEST
int gsRvTRDelta;
int gsRvP_frame_cnt = 0;
int gsRvReference_Flag = 1;

typedef struct rmff_frame_t{
	int Current_TR;
	int Previous_TR;
	int Current_time_stamp;
	int Previous_time_stamp;
} rmff_frame_t;

typedef struct rmff_frame_time_t {
	rmff_frame_t  ref_frame;		
	rmff_frame_t  frame_P1;
	rmff_frame_t  frame_P2;
} rmff_frame_time_t;

static rmff_frame_time_t gsRmff_frame_time;
#endif

#define NAL_UNIT_SPS            0x07 
#define NAL_UNIT_PPS            0x08 

#define CDMX_PTS_MODE		0	//! Presentation Timestamp (Display order)
#define CDMX_DTS_MODE		1	//! Decode Timestamp (Decode order)

#define CVDEC_DISP_INFO_INIT	0
#define CVDEC_DISP_INFO_UPDATE	1
#define CVDEC_DISP_INFO_GET		2
#define CVDEC_DISP_INFO_RESET	3

#define TMP_SKIP_OPT_SKIP_INTERVAL  5

#define SEQ_HEADER_INIT_ERROR_COUNT 60 //200 //sync with REPEAT_COUNT_FOR_THUMBNAIL in tcc_mediaffparser_node.cpp
#define DISPLAYING_FAIL_IN_THUMBNAIL_MODE_ERROR_COUNT 20
#define MAX_CONSECUTIVE_VPU_FAIL_COUNT 500

typedef struct dec_disp_info_ctrl_t {
	int		m_iTimeStampType;	//! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS)
	int		m_iStdType;			//! STD type
	int		m_iFmtType;			//! Formater Type

	int		m_iUsedIdxPTS;		//! total number of decoded index for PTS
	int		m_iRegIdxPTS[32];	//! decoded index for PTS
	void	*m_pRegInfoPTS[32];	//! side information of the decoded index for PTS

	int		m_iDecodeIdxDTS;	//! stored DTS index of decoded frame
	int		m_iDispIdxDTS;		//! display DTS index of DTS array
	int		m_iDTS[32];			//! Decode Timestamp (decoding order)
	
	int		m_Reserved;
} dec_disp_info_ctrl_t;

typedef struct dec_disp_info_t {
	int m_iFrameType;			//! Frame Type

	int m_iTimeStamp;			//! Time Stamp
	int m_iRvTimeStamp;			//! TR(RV)

	int m_iPicStructure;		//! PictureStructure
	int m_iM2vFieldSequence;	//! Field sequence(MPEG2) 
	int m_iFrameDuration;		//! MPEG2 Frame Duration
	
	int m_iFrameSize;			//! Frame size
} dec_disp_info_t;

typedef struct dec_disp_info_input_t {
	int m_iFrameIdx;			//! Display frame buffer index for CVDEC_DISP_INFO_UPDATE command
								//! Decoded frame buffer index for CVDEC_DISP_INFO_GET command
	int m_iStdType;				//! STD type for CVDEC_DISP_INFO_INIT
	int m_iTimeStampType;		//! TS(Timestamp) type (0: Presentation TS(default), 1:Decode TS) for CVDEC_DISP_INFO_INIT
	int m_iFmtType;				//! Formater Type specification
	int m_iFrameRate;
} dec_disp_info_input_t;

dec_disp_info_ctrl_t dec_disp_info_ctrl;
dec_disp_info_t dec_disp_info[32];
dec_disp_info_input_t	dec_disp_info_input = {0,};

typedef struct mpeg2_pts_ctrl{
	int m_iLatestPTS;
	int m_iPTSInterval;
	int m_iRamainingDuration;
} mpeg2_pts_ctrl;

#ifdef TS_TIMESTAMP_CORRECTION
typedef struct ts_pts_ctrl{
	int m_iLatestPTS;
	int m_iPTSInterval;
	int m_iRamainingDuration;
} ts_pts_ctrl;
ts_pts_ctrl gsTSPtsInfo;
#endif
mpeg2_pts_ctrl gsMPEG2PtsInfo;
OMX_U32 gVideo_FrameRate = 30;

#ifdef PEEK_DEFRAGMENTED_FRAME
static void PrintHexDataFrontRear(OMX_U8* pBuffer, OMX_U32 nLength, OMX_U8* pTag)
{
	LOGI("[%s:%08d] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ~ 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", pTag, nLength
			, pBuffer[0] , pBuffer[1] , pBuffer[2] , pBuffer[3]
			, pBuffer[4] , pBuffer[5] , pBuffer[6] 
			, pBuffer[7] , pBuffer[8] , pBuffer[9]
			, pBuffer[nLength - 10] , pBuffer[nLength - 9] , pBuffer[nLength - 8]
			, pBuffer[nLength - 7] , pBuffer[nLength - 6] , pBuffer[nLength - 5]
			, pBuffer[nLength - 4] , pBuffer[nLength - 3] , pBuffer[nLength - 2], pBuffer[nLength - 1]
		);
}

static void PrintHexData(OMX_U8* pBuffer, OMX_U32 nOffset, OMX_U8* pTag)
{
	LOGI("[%s] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", pTag
			, pBuffer[nOffset] , pBuffer[nOffset + 1] , pBuffer[nOffset + 2] , pBuffer[nOffset + 3]
			, pBuffer[nOffset + 4] , pBuffer[nOffset + 5] , pBuffer[nOffset + 6] 
			, pBuffer[nOffset + 7] , pBuffer[nOffset + 8] , pBuffer[nOffset + 9]
		);
}
#endif // PEEK_DEFRAGMENTED_FRAME

static OMX_ERRORTYPE omx_videodec_component_AllocateBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** pBuffer,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes);

static OMX_ERRORTYPE omx_videodec_component_FreeBuffer(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_U32 nPortIndex,
    OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer);

static OMX_BOOL isSWCodec(OMX_S32 format);

/** internal function to set codec related parameters in the private type structure 
  */
static void SetInternalVideoParameters(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private;
	omx_base_video_PortType *inPort ; 

	omx_private = openmaxStandComp->pComponentPrivate;;

	if (omx_private->video_coding_type == OMX_VIDEO_CodingRV) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/rv");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingRV;

		setHeader(&omx_private->pVideoRv, sizeof(OMX_VIDEO_PARAM_RVTYPE));    
#if 0
		omx_private->pVideoRv.nSize = 0;      
		omx_private->pVideoRv.nPortIndex = 0;
		omx_private->pVideoRv.nPFrames = 0;
		omx_private->pVideoRv.nBFrames = 0;
		omx_private->pVideoRv.eProfile = OMX_VIDEO_MPEG2ProfileSimple;
		omx_private->pVideoRv.eLevel = OMX_VIDEO_RVLevelLL;   
#endif

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingRV;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingFLV1) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-flv");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingFLV1;

		setHeader(&omx_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));  
		#if 0
		omx_private->pVideoH263.nPortIndex = 0x0;      
		omx_private->pVideoH263.eProfile = OMX_VIDEO_H263ProfileBaseline; //OMX_VIDEO_MPEG4ProfileCore
		omx_private->pVideoH263.eLevel = OMX_VIDEO_H263Level45;
		omx_private->pVideoH263.bPLUSPTYPEAllowed = OMX_FALSE;
		omx_private->pVideoH263.nAllowedPictureTypes = 0;
		omx_private->pVideoH263.bForceRoundingTypeToZero = OMX_TRUE;
		omx_private->pVideoH263.nPictureHeaderRepetition = 0;
		omx_private->pVideoH263.nGOBHeaderInterval = 0;
		omx_private->pVideoH263.nPFrames = 0;
		omx_private->pVideoH263.nBFrames = 0;
	#endif

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingFLV1;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/h263");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;

		setHeader(&omx_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));    
		omx_private->pVideoH263.nPortIndex = 0x0;      
		omx_private->pVideoH263.eProfile = OMX_VIDEO_H263ProfileBaseline; //OMX_VIDEO_MPEG4ProfileCore
		omx_private->pVideoH263.eLevel = OMX_VIDEO_H263Level45;
		omx_private->pVideoH263.bPLUSPTYPEAllowed = OMX_FALSE;
		omx_private->pVideoH263.nAllowedPictureTypes = 0;
		omx_private->pVideoH263.bForceRoundingTypeToZero = OMX_TRUE;
		omx_private->pVideoH263.nPictureHeaderRepetition = 0;
		omx_private->pVideoH263.nGOBHeaderInterval = 0;
		omx_private->pVideoH263.nPFrames = 0;
		omx_private->pVideoH263.nBFrames = 0;


		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingH263;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/avc(h264)");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;

		setHeader(&omx_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
		omx_private->pVideoAvc.nPortIndex = 0;
		omx_private->pVideoAvc.nSliceHeaderSpacing = 0;
		omx_private->pVideoAvc.bUseHadamard = OMX_FALSE;
		omx_private->pVideoAvc.nRefFrames = 2;
		omx_private->pVideoAvc.nPFrames = 0;
		omx_private->pVideoAvc.nBFrames = 0;
		omx_private->pVideoAvc.bUseHadamard = OMX_FALSE;
		omx_private->pVideoAvc.nRefFrames = 2;
		omx_private->pVideoAvc.eProfile = OMX_VIDEO_AVCProfileBaseline;
		omx_private->pVideoAvc.eLevel = OMX_VIDEO_AVCLevel1;
		omx_private->pVideoAvc.nAllowedPictureTypes = 0;
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
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingAVC;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/mpeg4");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;

		setHeader(&omx_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));    
		omx_private->pVideoMpeg4.nPortIndex = 0;                                                                    
		omx_private->pVideoMpeg4.nSliceHeaderSpacing = 0;
		omx_private->pVideoMpeg4.bSVH = OMX_FALSE;
		omx_private->pVideoMpeg4.bGov = OMX_FALSE;
		omx_private->pVideoMpeg4.nPFrames = 0;
		omx_private->pVideoMpeg4.nBFrames = 0;
		omx_private->pVideoMpeg4.nIDCVLCThreshold = 0;
		omx_private->pVideoMpeg4.bACPred = OMX_FALSE;
		omx_private->pVideoMpeg4.nMaxPacketSize = 0;
		omx_private->pVideoMpeg4.nTimeIncRes = 0;
		omx_private->pVideoMpeg4.eProfile = OMX_VIDEO_MPEG4ProfileSimple;
		omx_private->pVideoMpeg4.eLevel = OMX_VIDEO_MPEG4Level0;
		omx_private->pVideoMpeg4.nAllowedPictureTypes = 0;
		omx_private->pVideoMpeg4.nHeaderExtension = 0;
		omx_private->pVideoMpeg4.bReversibleVLC = OMX_FALSE;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingWMV) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/wmv");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV;

		setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));    
		omx_private->pVideoWmv.nPortIndex = 0;  
		omx_private->pVideoWmv.eFormat = OMX_VIDEO_WMVFormat9;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingWMV;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-wmv-1-2");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;

		setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));    
		omx_private->pVideoWmv.nPortIndex = 0;  
		omx_private->pVideoWmv.eFormat = OMX_VIDEO_WMVFormat9;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingDIVX) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/divx");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingDIVX;

		//setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_DIVXTYPE));    
		//omx_private->pVideoWmv.nPortIndex = 0;  
		//omx_private->pVideoWmv.eFormat = ;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingDIVX;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/mpeg2");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG2;

		setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));    
		omx_private->pVideoMpeg4.nPFrames = 0;
		omx_private->pVideoMpeg4.nBFrames = 0;
		omx_private->pVideoMpeg4.eProfile = OMX_VIDEO_MPEG2ProfileSimple;
		omx_private->pVideoMpeg4.eLevel = OMX_VIDEO_MPEG2LevelHL;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMPEG2;
	} 
	else if (omx_private->video_coding_type == OMX_VIDEO_CodingMJPEG) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-jpeg");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMJPEG;

		LOGE("not defined MJPEG");
		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
	} 
	
}

OMX_ERRORTYPE omx_videodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

	if(omx_private->video_coding_type == OMX_VIDEO_CodingRV) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_RV_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_H263_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingFLV1) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_SORENSON_H263_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_H264_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MPEG4_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_WMV_NAME));
	}	
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_WMV12_NAME));
	}	
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingDIVX) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_DIVX_NAME));
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MPEG2_NAME));
	}	
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingMJPEG) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MJPEG_NAME));
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

#ifdef HAVE_ANDROID_OS
		if (1) 
		{
				DBUG_MSG("In %s, allocating component\n", __func__);
				openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_videodec_component_PrivateType));
				if(openmaxStandComp->pComponentPrivate == NULL)
				{
					return OMX_ErrorInsufficientResources;
				}
				memset(openmaxStandComp->pComponentPrivate, 0x00, sizeof(omx_videodec_component_PrivateType));
		}
		else			
#else
		if(!openmaxStandComp->pComponentPrivate) 
		{
				DBUG_MSG("In %s, allocating component\n", __func__);
				openmaxStandComp->pComponentPrivate = TCC_calloc(1, sizeof(omx_videodec_component_PrivateType));
				if(openmaxStandComp->pComponentPrivate == NULL)
				{
					return OMX_ErrorInsufficientResources;
				}
		}
		else			
#endif
		{
			DBUG_MSG("In %s, Error Component %x Already Allocated\n", __func__, (int)openmaxStandComp->pComponentPrivate);
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

		memset(&omx_private->gsVDecInput, 0x00, sizeof(vdec_input_t));
		memset(&omx_private->gsVDecOutput, 0x00, sizeof(vdec_output_t));
		memset(&omx_private->gsVDecInit, 0x00, sizeof(vdec_init_t));
		memset(&omx_private->gsVDecUserInfo, 0x00, sizeof(vdec_user_info_t));

		/** now it's time to know the video coding type of the component */
		if(!strcmp(cComponentName, VIDEO_DEC_RV_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingRV;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_RV;
		}else if(!strcmp(cComponentName, VIDEO_DEC_H263_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingH263;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_H263;
		}
		#ifdef INCLUDE_SORENSON263_DEC
		else if(!strcmp(cComponentName, VIDEO_DEC_SORENSON_H263_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingFLV1;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_SORENSON263;
			DBUG_MSG("VIDEO_DEC_SORENSON_H263_NAME ");
		}
		#endif
		else if(!strcmp(cComponentName, VIDEO_DEC_H264_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingAVC;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_AVC;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MPEG4_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_MPEG4;
		}else if(!strcmp(cComponentName, VIDEO_DEC_WMV_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingWMV;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_VC1;
		}else if(!strcmp(cComponentName, VIDEO_DEC_WMV12_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingWMV_1_2;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_WMV78;
		}else if (!strcmp(cComponentName, VIDEO_DEC_DIVX_NAME)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingDIVX;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_DIV3;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MPEG2_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingMPEG2;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_MPEG2;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MJPEG_NAME)) { 
			omx_private->video_coding_type = OMX_VIDEO_CodingMJPEG;
			omx_private->gsVDecInit.m_iBitstreamFormat	= STD_MJPG;
		}else if (!strcmp(cComponentName, VIDEO_DEC_BASE_NAME)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingUnused;
		} else {
			// IL client specified an invalid component name 
			return OMX_ErrorInvalidComponentName;
		} 

#if 0
		gspfVDec = gspfVDecList[omx_private->gsVDecInit.m_iBitstreamFormat];
#endif
		/** here we can override whatever defaults the base_component constructor set
		* e.g. we can override the function pointers in the private struct  
		*/

		/** Domain specific section for the ports.   
		* first we set the parameter common to both formats
		*/
		//common parameters related to input port.  
		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		//ZzaU:: change input buffer size because DEFAULT_OUT_BUFFER_SIZE increased. 
		inPort->sPortParam.nBufferSize = (OMX_U32) VIDEO_DEC_IN_BUFFER_SIZE;//DEFAULT_OUT_BUFFER_SIZE;
		inPort->sPortParam.format.video.xFramerate = 30;
		inPort->sPortParam.format.video.nFrameWidth = AVAILABLE_MAX_WIDTH;
		inPort->sPortParam.format.video.nFrameHeight = AVAILABLE_MAX_HEIGHT;
		inPort->sPortParam.nBufferCountMin = 1;    
		inPort->sPortParam.nBufferCountActual = 8; 


		//common parameters related to output port
		outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
		outPort->sPortParam.format.video.eColorFormat = OUTPUT_DECODED_COLOR_FMT;
		outPort->sPortParam.format.video.nFrameWidth = AVAILABLE_MAX_WIDTH;
		outPort->sPortParam.format.video.nFrameHeight = AVAILABLE_MAX_HEIGHT;
		outPort->sPortParam.nBufferSize =  (OMX_U32) (AVAILABLE_MAX_WIDTH*AVAILABLE_MAX_HEIGHT*3/2);
		outPort->sPortParam.format.video.xFramerate = 30;

#ifdef HAVE_ANDROID_OS
		outPort->sPortParam.nBufferCountMin = 5;
		outPort->sPortParam.nBufferCountActual = VPU_BUFF_COUNT - 1;
#endif

		/** settings of output port parameter definition */
		if(isSWCodec(omx_private->gsVDecInit.m_iBitstreamFormat) 
#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)	
			|| (omx_private->video_coding_type == OMX_VIDEO_CodingMJPEG)
#endif
		)
		{
			outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
			outPort->sVideoParam.eColorFormat = OMX_COLOR_FormatYUV420Planar;
		}
		else
		{
			outPort->sPortParam.format.video.eColorFormat = OUTPUT_DECODED_COLOR_FMT;
			outPort->sVideoParam.eColorFormat = OUTPUT_DECODED_COLOR_FMT;
		}

		outPort->sVideoParam.xFramerate = 30;

#if 0
		if(omx_private->gsVDecInit.m_iBitstreamFormat >= STD_AVC)
			gspfVDec = gspfVDecList[omx_private->gsVDecInit.m_iBitstreamFormat];
		else
			gspfVDec = 0;
		
		if(gspfVDec == 0)
		{
			return OMX_ErrorComponentNotFound;
		}
#endif

		if(!omx_private->avCodecSyncSem) {
			omx_private->avCodecSyncSem = TCC_malloc(sizeof(tsem_t));
			if(omx_private->avCodecSyncSem == NULL) {
				return OMX_ErrorInsufficientResources;
			}
			tsem_init(omx_private->avCodecSyncSem, 0);
		}

		SetInternalVideoParameters(openmaxStandComp);

		omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;

		if(omx_private->video_coding_type == OMX_VIDEO_CodingRV) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingRV;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingFLV1) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingFLV1;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingDIVX) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingDIVX;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG2;
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMJPEG) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
		}
		
		/** general configuration irrespective of any video formats
		* setting other parameters of omx_videodec_component_private  
		*/
		//  omx_private->avCodec = NULL;
		//  omx_private->avCodecContext= NULL;
		omx_private->isVPUClosed = OMX_TRUE;
		omx_private->avcodecReady = OMX_FALSE;
		omx_private->extradata = NULL;
		omx_private->extradata_size = 0;
		omx_private->BufferMgmtCallback = omx_videodec_component_BufferMgmtCallback;

		/** initializing the codec context etc that was done earlier by ffmpeglibinit function */
		omx_private->messageHandler = omx_videodec_component_MessageHandler;
		omx_private->destructor = omx_videodec_component_Destructor;

		openmaxStandComp->SetParameter = omx_videodec_component_SetParameter;
		openmaxStandComp->GetParameter = omx_videodec_component_GetParameter;
		openmaxStandComp->SetConfig    = omx_videodec_component_SetConfig;
		openmaxStandComp->ComponentRoleEnum = omx_videodec_component_ComponentRoleEnum;
		openmaxStandComp->GetExtensionIndex = omx_videodec_component_GetExtensionIndex;

#ifdef HAVE_ANDROID_OS
#if (!USE_EXTERNAL_BUFFER)
		//For reducing needless memory copy.
		openmaxStandComp->AllocateBuffer = omx_videodec_component_AllocateBuffer;
		openmaxStandComp->UseBuffer = omx_base_component_UseBuffer2;
		openmaxStandComp->FreeBuffer = omx_videodec_component_FreeBuffer;	
#endif
#endif 

		omx_private->pConfigdata = TCC_calloc(1,CONFIG_DATA_SIZE);
		omx_private->szConfigdata = 0;
		omx_private->seq_header_init_error_count = SEQ_HEADER_INIT_ERROR_COUNT;
		omx_private->displaying_error_count = DISPLAYING_FAIL_IN_THUMBNAIL_MODE_ERROR_COUNT;
		
		#ifdef DIVX_DRM5
		if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
		{
			DivxDecryptExInit();
		}
		#endif
		
		
#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
        {
    		char value[PROPERTY_VALUE_MAX];
    		property_get("tcc.video.vsync", value, "0");
    		iVsyncMode = atoi(value);

			LOGE("Vsync Mode : %d", iVsyncMode);

            if(iVsyncMode)
            {
        		if(g_hFb < 0)
        			g_hFb = open("/dev/graphics/fb0", O_RDWR);

        		if(g_hFb < 0)
        		{
        			LOGE("fb driver open fail");
        		}
            }
        }

#endif
		return eError;
	}


/** The destructor of the video decoder component
  */
OMX_ERRORTYPE omx_videodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

	#ifdef DIVX_DRM5
	if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
	{
		DivxDecryptExDeinit();
	}
	#endif

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

	TCC_free(omx_private->thumbnail_buffer);
	TCC_free(omx_private->pConfigdata);

	DBUG_MSG("Destructor of video decoder component is called\n");

	omx_base_filter_Destructor(openmaxStandComp);
	omx_private->isThumbnailMode = OMX_FALSE;

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
    {
        if(iVsyncMode)
        {
    		g_hFb = close( g_hFb ) ;
    		if(g_hFb < 0)
    		{
    			LOGE("fb driver close fail");
    		}
    		g_hFb = -1;
        }
    }
#endif

	return OMX_ErrorNone;
}


/** It initializates the FFmpeg framework, and opens an FFmpeg videodecoder of type specified by IL client 
  */ 
OMX_ERRORTYPE omx_videodec_component_LibInit(omx_videodec_component_PrivateType* omx_private) {

	OMX_U32 target_codecID;  
	OMX_S8 value[PROPERTY_VALUE_MAX];
	OMX_U32 uiHDMIOutputMode = 0;

	tsem_up(omx_private->avCodecSyncSem);

	omx_private->avcodecInited = 0;
	omx_private->container_type = 0;
	omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
	omx_private->i_skip_count = TMP_SKIP_OPT_SKIP_INTERVAL;
	omx_private->i_skip_interval = TMP_SKIP_OPT_SKIP_INTERVAL;
	omx_private->isFirst_Frame = OMX_TRUE;
	omx_private->isFromTCCParser = 0;
	omx_private->isThumbnailMode = OMX_FALSE;
	omx_private->isThumbnailMade = OMX_FALSE;

	omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
  	omx_private->bUseFrameDefragmentation = OMX_FALSE; 
	omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED; 
#ifdef DEFRAGMENT_INPUT_FRAME
	#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
	omx_private->frameFilterPTS = 0; 
	omx_private->isFrameDiscarded = OMX_FALSE; 
	#endif
	omx_private->bDetectFrameDelimiter = OMX_FALSE;
	omx_private->start_code_with_type = 0xFFFFFFFF;
	omx_private->isSplittedStartCode = OMX_FALSE; 
	omx_private->splittedStartCodeLen = 0; 
	omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED; 
#endif

	out_index = in_index = frm_clear = 0;
#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
	used_fifo_count = buffer_unique_id = 0;
#endif

#ifdef _TCC8900_
	property_get("persist.sys.output_mode", value, "");
	uiHDMIOutputMode = atoi(value);
	if( uiHDMIOutputMode == OUTPUT_HDMI)
#else
	property_get("persist.sys.hdmi_output.enable", value, "");
	if (strcmp(value, "1") == 0) 
#endif
	{
		LOGD("HDMI output enabled");
		gHDMIOutput = OMX_TRUE;
	} 

	return OMX_ErrorNone;
}

/** It Deinitializates the ffmpeg framework, and close the ffmpeg video decoder of selected coding type
  */
void omx_videodec_component_LibDeinit(omx_videodec_component_PrivateType* omx_private)
{
	int ret;

	if(omx_private->isVPUClosed == OMX_FALSE)
	{
#if 1
		DECODER_CLOSE();
#else	
		if( (ret = gspfVDec( VDEC_CLOSE, NULL, NULL, &omx_private->gsVDecOutput )) < 0 )
		{
			LOGE( "[VDEC_CLOSE] [Err:%4d] video decoder Deinit", ret );
		}
#endif
		omx_private->isVPUClosed = OMX_TRUE;
	}
}

/** The Initialization function of the video decoder
  */
OMX_ERRORTYPE omx_videodec_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955

	/** Temporary First Output buffer size */
	omx_private->inputCurrBuffer = NULL;
	omx_private->inputCurrLength = 0;
	omx_private->isFirstBuffer = 1;
	omx_private->isNewBuffer = 1;
	
	omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
  	omx_private->bUseFrameDefragmentation = OMX_FALSE; 
	omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED; 
#ifdef DEFRAGMENT_INPUT_FRAME
	#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
	omx_private->frameFilterPTS = 0; 
	omx_private->isFrameDiscarded = OMX_FALSE; 
	#endif
	omx_private->bDetectFrameDelimiter = OMX_FALSE;
	omx_private->start_code_with_type = 0xFFFFFFFF;
	omx_private->isSplittedStartCode = OMX_FALSE; 
	omx_private->splittedStartCodeLen = 0; 
	omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED; 
#endif

	return eError;
}

/** The Deinitialization function of the video decoder  
  */
OMX_ERRORTYPE omx_videodec_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	if (omx_private->avcodecReady) {
		omx_videodec_component_LibDeinit(omx_private);
		omx_private->avcodecReady = OMX_FALSE;
	}

	return eError;
} 

/** Executes all the required steps after an output buffer frame-size has changed.
*/
static inline void UpdateFrameSize(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	omx_base_video_PortType *inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
	outPort->sPortParam.format.video.nFrameWidth = inPort->sPortParam.format.video.nFrameWidth;
	outPort->sPortParam.format.video.nFrameHeight = inPort->sPortParam.format.video.nFrameHeight;
	outPort->sPortParam.format.video.xFramerate = inPort->sPortParam.format.video.xFramerate;
	switch(outPort->sVideoParam.eColorFormat) {
		case OMX_COLOR_FormatYUV420Planar:
		case OUTPUT_DECODED_COLOR_FMT:
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

static unsigned char* MakeThumbFrame(OMX_COMPONENTTYPE *openmaxStandComp, OMX_U32 *lenth, OMX_U8 thumb_black, tDEC_FRAME_OUTPUT* info)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	unsigned int frameSize;
	unsigned char *pThumb;
	unsigned int picWidth, picHeight;
	unsigned int src_picWidth, src_picHeight;
	unsigned char *pYdst, *pUdst, *pVdst, *pYSrc, *pUSrc, *pVSrc;		
	unsigned int src_offset;
	int cnt = 0;
	int i, j;

#if 1
	picWidth = info->picWidth;
	picHeight = info->picHeight;
#else
	if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_AVC)
	{	
		picWidth = (omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth- omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropRight);
		picHeight = (omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropBottom - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop);
	}
	else
	{
		picWidth = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		picHeight = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight;
	}
#endif

	src_picWidth = ((picWidth+15)>>4)<<4;
	src_picHeight = picHeight;

	frameSize = picWidth * picHeight;

	//omx_private->isThumbnailMade == true means that thumbnail_buffer was allocated and has still valid frame data , return after setting lenth
	if(omx_private->isThumbnailMade )
	{
		*lenth = frameSize*3/2;
		DBUG_MSG("already allocate buffer for thumbnail now set isThumbnailMode %d ",*lenth);
		return omx_private->thumbnail_buffer;
	}

	omx_private->thumbnail_buffer = TCC_calloc(1,frameSize*3/2);
	pThumb = omx_private->thumbnail_buffer;

	pYdst = pThumb;
	pUdst = pYdst + frameSize;
	pVdst = pUdst + frameSize/4;
	
	if (thumb_black == 1)
	{
		memset(pYdst, 0x10, frameSize);
		memset(pUdst, 0x80, frameSize/4);
		memset(pVdst, 0x80, frameSize/4);
	}
	else
	{
#ifdef MOVE_VPU_IN_KERNEL	
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC) || defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
		if(
	#if defined(INCLUDE_WMV78_DEC)
			omx_private->gsVDecInit.m_iBitstreamFormat == STD_WMV78
	#endif
	#if defined(INCLUDE_SORENSON263_DEC)		
			|| omx_private->gsVDecInit.m_iBitstreamFormat == STD_SORENSON263
	#endif
	#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)	
			|| omx_private->gsVDecInit.m_iBitstreamFormat == STD_MJPG
	#endif
		)
		{
	#if 1
			pYSrc = info->bufVirtAddr[0];
			pUSrc = info->bufVirtAddr[1];
			pVSrc = info->bufVirtAddr[2];
	#else
			pYSrc = omx_private->gsVDecOutput.m_pDispOut[VA][0];
			pUSrc = omx_private->gsVDecOutput.m_pDispOut[VA][1];
			pVSrc = omx_private->gsVDecOutput.m_pDispOut[VA][2];
	#endif
		}
		else
#endif
		{
	#if 1
			pYSrc = info->bufVirtAddr[0];
			pUSrc = info->bufVirtAddr[1];
			pVSrc = info->bufVirtAddr[2];
	#else		
			pYSrc = vpu_getFrameBufVirtAddr(omx_private->gsVDecOutput.m_pDispOut[VA][0], K_VA);
			pUSrc = vpu_getFrameBufVirtAddr(omx_private->gsVDecOutput.m_pDispOut[VA][1], K_VA);
			pVSrc = vpu_getFrameBufVirtAddr(omx_private->gsVDecOutput.m_pDispOut[VA][2], K_VA);
			
			DBUG_MSG("Convert-Addr : 0x%x-0x%x-0x%x => 0x%x-0x%x-0x%x", omx_private->gsVDecOutput.m_pDispOut[VA][0], 
							omx_private->gsVDecOutput.m_pDispOut[VA][1], omx_private->gsVDecOutput.m_pDispOut[VA][2], pYSrc, pUSrc, pVSrc);
	#endif
		}		
#else
#if 1
		pYSrc = info->bufVirtAddr[0];
		pUSrc = info->bufVirtAddr[1];
		pVSrc = info->bufVirtAddr[2];
#else

		pYSrc = omx_private->gsVDecOutput.m_pDispOut[VA][0];
		pUSrc = omx_private->gsVDecOutput.m_pDispOut[VA][1];
		pVSrc = omx_private->gsVDecOutput.m_pDispOut[VA][2];
#endif
#endif
		LOGD("11: 0x%x - 0x%x - 0x%x", pYSrc, pUSrc, pVSrc);

		if(omx_private->gsVDecInit.m_bCbCrInterleaveMode)
		{
			for(i=0; i<src_picHeight; i++)
			{
				memcpy(pYdst + (i * picWidth), pYSrc + (i * src_picWidth), picWidth);

				src_offset = i*src_picWidth/2;

				for(j=0; j<picWidth/2; j+=2)
				{
					pUdst[cnt] = pUSrc[src_offset+j];
					pVdst[cnt] = pUSrc[src_offset+j+1];
					cnt++;
				}			
			}
		}
		else
		{	
			for(i=0; i<src_picHeight; i++)
			{
				memcpy(pYdst + (i * picWidth), pYSrc + (i * src_picWidth), picWidth);

				if(i < src_picHeight/2)
				{
					memcpy(pUdst + (i * picWidth/2), pUSrc + (i * src_picWidth/2), picWidth/2);
					memcpy(pVdst + (i * picWidth/2), pVSrc + (i * src_picWidth/2), picWidth/2);
				}
			}
		}

		if(0)
		{
			FILE *fp;
			fp = fopen("/sdcard/thumb.yuv", "ab+");		
			fwrite( omx_private->thumbnail_buffer, frameSize*3/2, 1, fp);
			fclose(fp);
		}
	}
	*lenth = frameSize*3/2;

	return omx_private->thumbnail_buffer;
}

static int isPortChange(OMX_COMPONENTTYPE *openmaxStandComp, OMX_U32 w, OMX_U32 h)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	OMX_COLOR_FORMATTYPE colorformat;
	OMX_U32 width, height;
	int ret = 0;

#if 1
	width = w;
	height= h;
#else
	if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_AVC)
	{	
		width = (omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth- omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropRight);
		height = (omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropBottom - omx_private->gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop);

		if((outPort->sPortParam.format.video.nFrameWidth != width) || 
			(outPort->sPortParam.format.video.nFrameHeight != height))
		{					
			ret = 1;
		}
	}
	else
	{
		width = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		height = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight;

		if(outPort->sPortParam.format.video.nFrameWidth != width ||
			outPort->sPortParam.format.video.nFrameHeight != height)
		{			
			ret = 1;
		}
	
	}
#endif

	if(width > AVAILABLE_MAX_WIDTH || ((width *height) > AVAILABLE_MAX_REGION))
	{
		LOGE("%ld x %ld ==> MAX-Resolution(%ld x %ld) over!!", width, height, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT);
		ret = -1;
	}

	if(width < AVAILABLE_MIN_WIDTH || height < AVAILABLE_MIN_HEIGHT)
	{
		LOGE("%ld x %ld ==> MIN-Resolution(%ld x %ld) less!!", width, height, AVAILABLE_MIN_WIDTH, AVAILABLE_MIN_HEIGHT);
		ret = -1;
	}

#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)	
	if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MJPG)
	{	
		if(omx_private->gsVDecInit.m_bCbCrInterleaveMode != 1)
		{
			//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
			if(omx_private->gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
				colorformat = OMX_COLOR_FormatYUV422Planar;
			else
				colorformat = OMX_COLOR_FormatYUV420Planar;

			if(outPort->sPortParam.format.video.eColorFormat != colorformat)
			{
				LOGI( "Change ColorFormat!! %d -> %d", outPort->sPortParam.format.video.eColorFormat, colorformat);		

				if(omx_private->gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 2;
				else
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;

				outPort->sPortParam.format.video.eColorFormat = colorformat;
				ret = 1;				
			}
		}
	}
#endif
	
	if(ret == 1)
	{
		outPort->bIsPortChanged = OMX_TRUE;		
			
		(*(omx_private->callbacks->EventHandler))(
							   openmaxStandComp,
							   omx_private->callbackData,
							   OMX_EventPortSettingsChanged, 
							   OMX_DirOutput,
							   0, 
							   NULL);
		
		LOGI( "ReSize Needed!! %ld x %ld -> %ld x %ld \n", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, width, height);
		
		outPort->sPortParam.format.video.nFrameWidth = width;
		outPort->sPortParam.format.video.nFrameHeight = height;	
		
		switch(outPort->sVideoParam.eColorFormat) {
			case OMX_COLOR_FormatYUV420Planar:
			case OUTPUT_DECODED_COLOR_FMT:
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

#if 0
	if(ret == 1 || gHDMIOutput)
	{
		if(gHDMIOutput) {
			vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
		}
		else{
			vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight);
		}
	}
#endif

	return ret;
}


static OMX_BOOL isSWCodec(OMX_S32 format)
{
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC)
	if(
#ifdef INCLUDE_WMV78_DEC							
		format == STD_WMV78 
#endif			
#ifdef INCLUDE_SORENSON263_DEC				
		|| format == STD_SORENSON263
#endif
	){
		return OMX_TRUE;
}
#endif

	return OMX_FALSE;
}

static void
disp_pic_info (int Opcode, void* pParam1, void *pParam2, void *pParam3, omx_videodec_component_PrivateType* omx_private)
{
	int i;
	dec_disp_info_ctrl_t  *pInfoCtrl = (dec_disp_info_ctrl_t*)pParam1;
	dec_disp_info_t 	  *pInfo = (dec_disp_info_t *)pParam2;
	dec_disp_info_input_t *pInfoInput = (dec_disp_info_input_t*)pParam3;

	switch( Opcode )
	{
	case CVDEC_DISP_INFO_INIT:	//init.
			pInfoCtrl->m_iStdType = pInfoInput->m_iStdType; 					
			pInfoCtrl->m_iFmtType = pInfoInput->m_iFmtType;
			pInfoCtrl->m_iTimeStampType = pInfoInput->m_iTimeStampType;

			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG )
			{
				gsMPEG2PtsInfo.m_iLatestPTS = 0;
				gsMPEG2PtsInfo.m_iRamainingDuration = 0;
				if(omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate!=0)
				gsMPEG2PtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10;
			}

            #ifdef TS_TIMESTAMP_CORRECTION
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS )
			{
				gsTSPtsInfo.m_iLatestPTS = 0;
				gsTSPtsInfo.m_iRamainingDuration = 0;
				if(omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate!=0)
					gsTSPtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10;
			}
            #endif
            
	case CVDEC_DISP_INFO_RESET: //reset
			for( i=0 ; i<32 ; i++ )
			{
				pInfoCtrl->m_iRegIdxPTS[i] = -1;	//unused
				pInfoCtrl->m_pRegInfoPTS[i] = (void*)&pInfo[i];
			}
			pInfoCtrl->m_iUsedIdxPTS = 0;

			if( pInfoCtrl->m_iTimeStampType == CDMX_DTS_MODE )	//Decode Timestamp (Decode order)
			{
				pInfoCtrl->m_iDecodeIdxDTS = 0;
				pInfoCtrl->m_iDispIdxDTS = 0;
				for( i=0 ; i<32 ; i++ )
				{
					pInfoCtrl->m_iDTS[i] = 0;
				}
			}

			memset(&gsRmff_frame_time, 0, sizeof(rmff_frame_time_t));
			gsRvReference_Flag = 1;
			gsRvP_frame_cnt = 0;

			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG )
			{
				gsMPEG2PtsInfo.m_iLatestPTS = 0;
				gsMPEG2PtsInfo.m_iRamainingDuration = 0;
			}
			
            #ifdef TS_TIMESTAMP_CORRECTION
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS )
			{
				gsTSPtsInfo.m_iLatestPTS = 0;
				gsTSPtsInfo.m_iRamainingDuration = 0;
			}
            #endif
		break;

	case CVDEC_DISP_INFO_UPDATE: //update
		{
			int iDecodedIdx;
			int usedIdx, startIdx, regIdx;
			dec_disp_info_t * pdec_disp_info;

			iDecodedIdx = pInfoInput->m_iFrameIdx;

			//In case that frame rate is changed...
			#if 1
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG )
			{
				if(pInfoInput->m_iFrameRate)
				{
					omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate = ((pInfoInput->m_iFrameRate & 0xffff) * 1000) / (((pInfoInput->m_iFrameRate >> 16) + 1)&0xffff);
					gsMPEG2PtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10;

					
					//LOGD("CVDEC_DISP_INFO_UPDATE m_iPTSInterval %d m_iFrameRate %d input FrameRate %x ",gsMPEG2PtsInfo.m_iPTSInterval , omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate,pInfoInput->m_iFrameRate);
				}
			}
			#endif
			//Presentation Timestamp (Display order)
			{
				//sort
				usedIdx=0;
				startIdx = -1;
				for( i=0 ; i<32 ; i++ )
				{
					if( pInfoCtrl->m_iRegIdxPTS[i] > -1 )
					{
						if( startIdx == -1 )
						{
							startIdx = i;
						}
						usedIdx++;
					}
				}

				if( usedIdx > 0 )
				{
					regIdx = 0;
					for( i=startIdx ; i<32 ; i++ )
					{
						if( pInfoCtrl->m_iRegIdxPTS[i] > -1 )
						{
							if( i != regIdx )
							{
								void * pswap;
								int iswap;

								iswap = pInfoCtrl->m_iRegIdxPTS[regIdx];
								pswap = pInfoCtrl->m_pRegInfoPTS[regIdx];
								
								pInfoCtrl->m_iRegIdxPTS[regIdx] = pInfoCtrl->m_iRegIdxPTS[i];
								pInfoCtrl->m_pRegInfoPTS[regIdx] = pInfoCtrl->m_pRegInfoPTS[i];

								pInfoCtrl->m_iRegIdxPTS[i] = iswap;
								pInfoCtrl->m_pRegInfoPTS[i] = pswap;
							}
							regIdx++;
							if( regIdx == usedIdx )
								break;
						}
					}
				}

				//save the side info.
				pInfoCtrl->m_iRegIdxPTS[usedIdx] = iDecodedIdx;
				pdec_disp_info = (dec_disp_info_t*)pInfoCtrl->m_pRegInfoPTS[usedIdx];

				pdec_disp_info->m_iTimeStamp = pInfo->m_iTimeStamp;
				pdec_disp_info->m_iFrameType = pInfo->m_iFrameType;
				pdec_disp_info->m_iPicStructure = pInfo->m_iPicStructure;
				pdec_disp_info->m_iRvTimeStamp = pInfo->m_iRvTimeStamp;
				pdec_disp_info->m_iM2vFieldSequence = pInfo->m_iM2vFieldSequence;
				pdec_disp_info->m_iFrameDuration = pInfo->m_iFrameDuration;				
				pdec_disp_info->m_iFrameSize = pInfo->m_iFrameSize;
				
				if( pInfoCtrl->m_iStdType  == STD_RV )
				{
					int curTimestamp, rvTimestamp, rvFrameType;

					curTimestamp = pInfo->m_iTimeStamp;
					rvTimestamp = pInfo->m_iRvTimeStamp;
					rvFrameType = pInfo->m_iFrameType;
								
					if(gsRvReference_Flag)
					{
						gsRvReference_Flag = 0;
						gsRmff_frame_time.ref_frame.Current_time_stamp = curTimestamp;
						gsRmff_frame_time.ref_frame.Previous_TR = rvTimestamp;
						gsRmff_frame_time.frame_P2.Current_time_stamp = curTimestamp;
						gsRmff_frame_time.frame_P2.Current_TR = rvTimestamp;
					}
					else
					{
						gsRvTRDelta = rvTimestamp - gsRmff_frame_time.ref_frame.Current_TR;
						if(gsRvTRDelta < 0)
						{
							gsRvTRDelta += 8192;
						}

						if(rvFrameType == 2) //B-frame
						{
							curTimestamp = gsRmff_frame_time.ref_frame.Current_time_stamp + gsRvTRDelta;
						}
						else
						{
							gsRvP_frame_cnt++;
						}
					}

					if( gsRvP_frame_cnt == 1)
					{
						gsRmff_frame_time.frame_P1.Current_TR = rvTimestamp;
						gsRmff_frame_time.frame_P1.Current_time_stamp = curTimestamp;

						gsRmff_frame_time.ref_frame.Current_time_stamp = gsRmff_frame_time.frame_P2.Current_time_stamp;
						gsRmff_frame_time.ref_frame.Current_TR = gsRmff_frame_time.frame_P2.Current_TR;
					}
					else if( gsRvP_frame_cnt == 2)
					{
						gsRvP_frame_cnt = 0;
						gsRmff_frame_time.frame_P2.Current_TR = rvTimestamp;
						gsRmff_frame_time.frame_P2.Current_time_stamp = curTimestamp;

						gsRmff_frame_time.ref_frame.Current_time_stamp = gsRmff_frame_time.frame_P1.Current_time_stamp;
						gsRmff_frame_time.ref_frame.Current_TR = gsRmff_frame_time.frame_P1.Current_TR;
					}

					pdec_disp_info->m_iRvTimeStamp = curTimestamp;
				}

				pInfoCtrl->m_iUsedIdxPTS = usedIdx + 1;
				if( pInfoCtrl->m_iUsedIdxPTS > 31 )
				{
					DBUG_MSG( "[CDK_CORE] disp_pic_info index failed\n" );
					for( i=0 ; i<32 ; i++ )
					{
						pInfoCtrl->m_iRegIdxPTS[i] = -1;
					}
				}
			}

			if( pInfoCtrl->m_iTimeStampType == CDMX_DTS_MODE )	//Decode Timestamp (Decode order)
			{
				if( iDecodedIdx >= 0 || ( iDecodedIdx == -2 && pInfoCtrl->m_iStdType  == STD_MPEG4  ) )
				{		
					pInfoCtrl->m_iDTS[pInfoCtrl->m_iDecodeIdxDTS] = pInfo->m_iTimeStamp;
					pInfoCtrl->m_iDecodeIdxDTS = ( pInfoCtrl->m_iDecodeIdxDTS + 1 ) & 31;
				}
			}
		}
		break;
	case CVDEC_DISP_INFO_GET:	//display
		{
			dec_disp_info_t **pInfo = (dec_disp_info_t **)pParam2;
			int dispOutIdx = pInfoInput->m_iFrameIdx;

			//Presentation Timestamp (Display order)
			{
				*pInfo = 0;
			
				for( i=0; i<pInfoCtrl->m_iUsedIdxPTS ; i++ )
				{
					if( dispOutIdx == pInfoCtrl->m_iRegIdxPTS[i] )
					{
						*pInfo = (dec_disp_info_t*)pInfoCtrl->m_pRegInfoPTS[i];

						if( pInfoCtrl->m_iFmtType  == CONTAINER_TYPE_MPG )
						{
						//LOGD("CVDEC_DISP_INFO_GET m_iPTSInterval %d m_iLatestPTS %d input m_iTimeStamp %d m_iRamainingDuration %d ",gsMPEG2PtsInfo.m_iPTSInterval , 
						//gsMPEG2PtsInfo.m_iLatestPTS,(*pInfo)->m_iTimeStamp,gsMPEG2PtsInfo.m_iRamainingDuration);
							if( (*pInfo)->m_iTimeStamp <= gsMPEG2PtsInfo.m_iLatestPTS )
								(*pInfo)->m_iTimeStamp = gsMPEG2PtsInfo.m_iLatestPTS + ((gsMPEG2PtsInfo.m_iPTSInterval * gsMPEG2PtsInfo.m_iRamainingDuration) >> 1);

							gsMPEG2PtsInfo.m_iLatestPTS = (*pInfo)->m_iTimeStamp;
							gsMPEG2PtsInfo.m_iRamainingDuration = (*pInfo)->m_iFrameDuration;
						}

                        #ifdef TS_TIMESTAMP_CORRECTION
						if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS )
						{
							if( (*pInfo)->m_iTimeStamp <= gsTSPtsInfo.m_iLatestPTS )
								(*pInfo)->m_iTimeStamp = gsTSPtsInfo.m_iLatestPTS + ((gsTSPtsInfo.m_iPTSInterval * gsTSPtsInfo.m_iRamainingDuration) >> 1);

							gsTSPtsInfo.m_iLatestPTS = (*pInfo)->m_iTimeStamp;
							gsTSPtsInfo.m_iRamainingDuration = (*pInfo)->m_iFrameDuration;
						}
                        #endif

						pInfoCtrl->m_iRegIdxPTS[i] = -1; //unused
						pInfoCtrl->m_iUsedIdxPTS--;
						break;
					}
				}
			}
			
			if( pInfoCtrl->m_iTimeStampType == CDMX_DTS_MODE )	//Decode Timestamp (Decode order)
			{
				if( *pInfo != 0 )
				{
					(*pInfo)->m_iTimeStamp =
					(*pInfo)->m_iRvTimeStamp = pInfoCtrl->m_iDTS[pInfoCtrl->m_iDispIdxDTS];
					pInfoCtrl->m_iDispIdxDTS = ( pInfoCtrl->m_iDispIdxDTS + 1 ) & 31;
				}
			}
		}
		break;
	}

	return;
}

static int
get_frame_type_for_frame_skipping(int iStdType, int iPicType, int iPicStructure)
{
	int frameType = 0; //unknown

	switch ( iStdType )
	{
	case STD_VC1 :
		switch( (iPicType>>3) ) //Frame or // FIELD_INTERLACED(TOP FIELD)
		{
		case PIC_TYPE_I:	frameType = 1; break;//I
		case PIC_TYPE_P:	frameType = 2; break;//P
		case 2:				frameType = 3; break;//B //DSTATUS( "BI  :" );
		case 3:				frameType = 3; break;//B //DSTATUS( "B   :" );
		case 4:				frameType = 3; break;//B //DSTATUS( "SKIP:" );
		}
		if( iPicStructure == 3) 
		{
			switch( (iPicType&0x7) ) // FIELD_INTERLACED(BOTTOM FIELD)
			{
			case PIC_TYPE_I:	frameType = 1; break;//I
			case PIC_TYPE_P:	frameType = 2; break;//P
			case 2:				frameType = 3; break;//B //DSTATUS( "BI  :" );
			case 3:				frameType = 3; break;//B //DSTATUS( "B   :" );
			case 4:				frameType = 3; break;//B //DSTATUS( "SKIP:" );
			}
		}
		break;
	case STD_MPEG4 :
		switch( iPicType )
		{
		case PIC_TYPE_I:	frameType = 1;	break;//I
		case PIC_TYPE_P:	frameType = 2;	break;//P
		case PIC_TYPE_B:	frameType = 3;	break;//B
		case PIC_TYPE_B_PB: frameType = 4;	break;//B of Packed PB-frame
		}
		break;
	case STD_MPEG2 :
	default:
		switch( iPicType )
		{
		case PIC_TYPE_I:	frameType = 1;	break;//I
		case PIC_TYPE_P:	frameType = 2;	break;//P
		case PIC_TYPE_B:	frameType = 3;	break;//B
		}
	}
	return frameType;
}


void print_user_data(unsigned char * pUserData)
{
	int i, j;
	unsigned char * pTmpPTR;
	unsigned char * pRealData;
	unsigned int nNumUserData;
	unsigned int nTotalSize;
	unsigned int nDataSize;

	pTmpPTR = pUserData;
	nNumUserData = (pTmpPTR[0] << 8) | pTmpPTR[1];
	nTotalSize = (pTmpPTR[2] << 8) | pTmpPTR[3];

	pTmpPTR = pUserData + 8;
	pRealData = pUserData + (8 * 17);

	DBUG_MSG( "\n***User Data Print***\n");
	for(i = 0;i < nNumUserData;i++)
	{
		nDataSize = (pTmpPTR[2] << 8) | pTmpPTR[3];
		DBUG_MSG( "[User Data][Idx : %02d][Size : %05d]", i, nDataSize);
		for(j = 0;j < nDataSize;j++)
		{
			DBUG_MSG( "%02x ", pRealData[j]);
		}
		pTmpPTR += 8;
		pRealData += nDataSize;
	}
}


char*
print_pic_type( int iVideoType, int iPicType, int iPictureStructure )
{
	switch ( iVideoType )
	{
	case STD_MPEG2 :
		if( iPicType == PIC_TYPE_I )
			return "I :";
		else if( iPicType == PIC_TYPE_P )
			return "P :";
		else if( iPicType == PIC_TYPE_B )
			return "B :";
		else
			return "D :"; //D_TYPE
		break;

	case STD_MPEG4 :
		if( iPicType == PIC_TYPE_I )
			return "I :";
		else if( iPicType == PIC_TYPE_P )
			return "P :";
		else if( iPicType == PIC_TYPE_B )
			return "B :";
		else if( iPicType == PIC_TYPE_B_PB ) //MPEG-4 Packed PB-frame
			return "pB:";
		else
			return "S :"; //S_TYPE
		break;

	case STD_VC1 :
		if( iPictureStructure == 3) 
		{
			// FIELD_INTERLACED
			if( (iPicType>>3) == PIC_TYPE_I )
				return "TF_I   :";	//TOP_FIELD = I	
			else if( (iPicType>>3) == PIC_TYPE_P )
				return "TF_P   :";	//TOP_FIELD = P
			else if( (iPicType>>3) == 2 )
				return "TF_BI  :";	//TOP_FIELD = BI_TYPE
			else if( (iPicType>>3) == 3 )
				return "TF_B   :";	//TOP_FIELD = B_TYPE
			else if( (iPicType>>3) == 4 )
				return "TF_SKIP:";	//TOP_FIELD = SKIP_TYPE
			else
				return "TF_FORBIDDEN :"; //TOP_FIELD = FORBIDDEN

			if( (iPicType&0x7) == PIC_TYPE_I )
				return "BF_I   :";	//BOTTOM_FIELD = I
			else if( (iPicType&0x7) == PIC_TYPE_P )
				return "BF_P   :";	//BOTTOM_FIELD = P
			else if( (iPicType&0x7) == 2 )
				return "BF_BI  :";	//BOTTOM_FIELD = BI_TYPE
			else if( (iPicType&0x7) == 3 )
				return "BF_B   :";	//BOTTOM_FIELD = B_TYPE
			else if( (iPicType&0x7) == 4 )
				return "BF_SKIP:";	//BOTTOM_FIELD = SKIP_TYPE
			else
				return "BF_FORBIDDEN :"; //BOTTOM_FIELD = FORBIDDEN
		}
		else 
		{
			iPicType = iPicType>>3;
			if( iPicType == PIC_TYPE_I )
				return "I   :";
			else if( iPicType == PIC_TYPE_P )
				return "P   :";
			else if( iPicType == 2 )
				return "BI  :";
			else if( iPicType == 3 )
				return "B   :";
			else if( iPicType == 4 )
				return "SKIP:";
			else
				return "FORBIDDEN :"; //FORBIDDEN
		}
		break;
	default:
		if( iPicType == PIC_TYPE_I )
			return "I :";
		else if( iPicType == PIC_TYPE_P )
			return "P :";
		else if( iPicType == PIC_TYPE_B )
			return "B :";
		else
			return "U :"; //Unknown
	}
}


OMX_ERRORTYPE omx_videodec_component_AllocateBuffer(
    OMX_IN OMX_HANDLETYPE hComponent,
    OMX_INOUT OMX_BUFFERHEADERTYPE** pBuffer,
    OMX_IN OMX_U32 nPortIndex,
    OMX_IN OMX_PTR pAppPrivate,
    OMX_IN OMX_U32 nSizeBytes)
{
    OMX_ERRORTYPE eError;

    eError = omx_base_component_AllocateBuffer2(hComponent, pBuffer, nPortIndex, pAppPrivate, nSizeBytes);
    if (eError == OMX_ErrorNone && nPortIndex == OMX_DirOutput) {
        (*pBuffer)->pPlatformPrivate = TCC_calloc(1, sizeof(TCC_PLATFORM_PRIVATE_PMEM_INFO));
        (*pBuffer)->pOutputPortPrivate = TCC_malloc(sizeof(OMX_U8)*9);
		sprintf((*pBuffer)->pOutputPortPrivate, "TCCVIDEO"); 
		*(OMX_U8*)((*pBuffer)->pOutputPortPrivate+8) = 0;
    }

    return eError;
}

OMX_ERRORTYPE omx_videodec_component_FreeBuffer(
    OMX_IN  OMX_HANDLETYPE hComponent,
    OMX_IN  OMX_U32 nPortIndex,
    OMX_IN  OMX_BUFFERHEADERTYPE* pBuffer)
{
    if (pBuffer->pPlatformPrivate && nPortIndex == OMX_DirOutput)
	{
        TCC_free(pBuffer->pPlatformPrivate);
        TCC_free(pBuffer->pOutputPortPrivate);
	}
    return omx_base_component_FreeBuffer2(hComponent, nPortIndex, pBuffer);
}

void ExtractConfigData(omx_videodec_component_PrivateType* omx_private, OMX_U32 input_offset)
{
	OMX_U8* p = omx_private->inputCurrBuffer;
	unsigned int szInfo_video = sizeof(omx_private->cdmx_info.m_sVideoInfo);
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	if(omx_private->inputCurrLength >= sizeof(TCCVideoConfigData))
	{
		TCCVideoConfigData *config_data;
		config_data = (TCCVideoConfigData*)(p+omx_private->inputCurrLength-sizeof(TCCVideoConfigData));

		if (strncmp(config_data->id, TCC_VIDEO_CONFIG_ID, 9) == 0)
		{
			omx_private->isFromTCCParser = 1;		
			omx_private->container_type = config_data->iContainerType;
			omx_private->gsVDecUserInfo.bitrate_mbps = config_data->iBitRate;
			omx_private->gsVDecUserInfo.frame_rate = config_data->iFrameRate;

			memset(&(omx_private->cdmx_info.m_sVideoInfo), 0x00, szInfo_video);
			//sync with parser!!
			if (omx_private->gsVDecInit.m_iBitstreamFormat == STD_RV || omx_private->gsVDecInit.m_iBitstreamFormat == STD_DIV3 || 
					omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG2 || omx_private->gsVDecInit.m_iBitstreamFormat == STD_VC1 ||
					omx_private->gsVDecInit.m_iBitstreamFormat == STD_WMV78
#ifdef INCLUDE_SORENSON263_DEC
					|| omx_private->gsVDecInit.m_iBitstreamFormat == STD_SORENSON263
#endif	
			   )
			{		
				memcpy(&(omx_private->cdmx_info.m_sVideoInfo), (char*)(p+omx_private->inputCurrLength-sizeof(TCCVideoConfigData)-szInfo_video), szInfo_video);
#ifdef INCLUDE_WMV78_DEC		
				omx_private->gsVDecInit.m_iFourCC = omx_private->cdmx_info.m_sVideoInfo.m_iFourCC;
#endif
			}

			if(omx_private->cdmx_info.m_sVideoInfo.m_iExtraDataLen
					&& omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2)
			{
				DBUG_MSG("ExtraData = %d", omx_private->cdmx_info.m_sVideoInfo.m_iExtraDataLen);
				omx_private->extradata_size = omx_private->cdmx_info.m_sVideoInfo.m_iExtraDataLen; 
				omx_private->extradata = TCC_calloc(1, omx_private->extradata_size);
				memcpy(omx_private->extradata, omx_private->cdmx_info.m_sVideoInfo.m_pExtraData, omx_private->cdmx_info.m_sVideoInfo.m_iExtraDataLen);
			}
			else
			{
				omx_private->extradata_size = 0; 
				omx_private->extradata = NULL;
			}

			LOGI("Resolution = %d x %d - %d Mbps - %d fps, Container Type = %d, FourCC = %08x", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, 
					omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, omx_private->container_type, omx_private->gsVDecInit.m_iFourCC);
		}
		else
		{
			omx_private->isFromTCCParser = 0;
			omx_private->gsVDecUserInfo.bitrate_mbps = 0;
			omx_private->gsVDecUserInfo.frame_rate = 0;
		}
	}
	else
	{
		omx_private->isFromTCCParser = 0;
		omx_private->gsVDecUserInfo.bitrate_mbps = 0;
		omx_private->gsVDecUserInfo.frame_rate = 0;
	}

	if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
	{
		if (CONTAINER_TYPE_TS != omx_private->container_type)
		{
		OMX_U8* p = omx_private->inputCurrBuffer;
			OMX_U8 cNalUnitType = p[MIN_NAL_STARTCODE_LEN + 1] & 0x1F;

			if(cNalUnitType == NAL_UNIT_SPS || cNalUnitType == NAL_UNIT_PPS)
			{
				memcpy(&omx_private->pConfigdata[omx_private->szConfigdata], omx_private->inputCurrBuffer+input_offset, omx_private->inputCurrLength-input_offset);
				omx_private->szConfigdata +=  (omx_private->inputCurrLength-input_offset);
			}
		}
	}
	else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4)
		{
		if( !omx_private->isFromTCCParser )
		{
			memcpy(&omx_private->pConfigdata[omx_private->szConfigdata], omx_private->inputCurrBuffer+input_offset, omx_private->inputCurrLength-input_offset);
			omx_private->szConfigdata +=  (omx_private->inputCurrLength-input_offset);
		}
		}

#ifdef DEFRAGMENT_INPUT_FRAME
	/* Determine the use of frame defragmentation for VPU. */
	if (CONTAINER_TYPE_TS == omx_private->container_type)
	{
		char value[PROPERTY_VALUE_MAX];
		if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2)
		{
			omx_private->bUseFrameDefragmentation = OMX_TRUE;
		}
		else
		{
			memset(value, 0, PROPERTY_VALUE_MAX);
			property_get("tcc.video.defragment", value, "2");

			omx_private->frameDefragmentationType = (OMX_U32)atoi(value);
			if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN || 
					omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
			{
				omx_private->bUseFrameDefragmentation = OMX_TRUE;
			}

		#ifndef DEFRAGMENT_INPUT_FRAME_AVC
			if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
			{
				if(omx_private->bUseFrameDefragmentation == OMX_TRUE)
				{
					omx_private->bUseFrameDefragmentation = OMX_FALSE;
				}
			}
		#endif
		#ifndef DEFRAGMENT_INPUT_FRAME_VC1
			if (omx_private->video_coding_type == OMX_VIDEO_CodingWMV )
			{
				if(omx_private->bUseFrameDefragmentation == OMX_TRUE)
				{
					omx_private->bUseFrameDefragmentation = OMX_FALSE;
				}
			}
		#endif
		}

	#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
		memset(value, 0, PROPERTY_VALUE_MAX);
		property_get("tcc.dbg.pts", value, "0");

		omx_private->frameFilterPTS = (OMX_U32)atoi(value);
	#endif
	}
#endif
}

#define CODETYPE_NONE		(0x00000000)
#define CODETYPE_HEADER		(0x00000001)
#define CODETYPE_PICTURE	(0x00000002)
#define CODETYPE_ALL		(CODETYPE_HEADER | CODETYPE_PICTURE)

/* H.264/AVC NAL unit type codes for start code and type mask */
#define AVC_NAL_FORBIDDEN_ZERO_BIT_MASK      0x80
#define AVC_NAL_STARTCODE_WITH_TYPE_MASK     0xFFFFFF1F
#define AVC_NAL_STARTCODE_WITH_TYPE_SLICE    0x00000101  // I-frame
#define AVC_NAL_STARTCODE_WITH_TYPE_DPA      0x00000102
#define AVC_NAL_STARTCODE_WITH_TYPE_DPB      0x00000103
#define AVC_NAL_STARTCODE_WITH_TYPE_DPC      0x00000104
#define AVC_NAL_STARTCODE_WITH_TYPE_IDR      0x00000105  // P-frame
#define AVC_NAL_STARTCODE_WITH_TYPE_SEI      0x00000106
#define AVC_NAL_STARTCODE_WITH_TYPE_SPS      0x00000107
#define AVC_NAL_STARTCODE_WITH_TYPE_PPS      0x00000108
#define AVC_NAL_STARTCODE_WITH_TYPE_AUD      0x00000109
#define AVC_NAL_STARTCODE_WITH_TYPE_EOSEQ    0x0000010A 
#define AVC_NAL_STARTCODE_WITH_TYPE_EOSTREAM 0x0000010B 
#define AVC_NAL_STARTCODE_WITH_TYPE_FILL     0x0000010C 

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
#define AVC_NAL_STARTCODE_MASK               0x00FFFFFF
#define AVC_NAL_STARTCODE                    0x00000001
#define AVC_NAL_STARTCODE_MAX_VALUE          0x0000017F

#define COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED()\
{\
	OMX_U32 ii;\
	for(ii = (MAX_NAL_STARTCODE_LEN - 1); omx_private->splittedStartCodeLen > 0; --ii, --omx_private->splittedStartCodeLen)\
	{\
		*(OMX_U8 *)(omx_private->gsVDecInput.m_pInp[VA] + omx_private->gsVDecInput.m_iInpLen++) \
			= (OMX_U8)((omx_private->start_code_with_type >> (8*ii)) & 0x000000FF);\
	}\
}

#define DETERMINE_APPROPRIATE_NEXT_START_CODE_OFFSET(OFFSET)\
{\
	OMX_U32 i;\
	for(i = 1; i < MAX_NAL_STARTCODE_LEN; i++)\
	{\
		if((omx_private->start_code_with_type << 8*i) == 0)\
		{\
			(OFFSET) = i;\
			break;\
		}\
	}\
}
#endif

OMX_U32 SearchCodeType(omx_videodec_component_PrivateType* omx_private, OMX_U32 *input_offset, OMX_U32 search_option)
{
	OMX_U32 temp_input_offset = *input_offset;
	OMX_U32 code_type = CODETYPE_NONE;

	if (OMX_VIDEO_CodingMPEG2 == omx_private->video_coding_type)
	{
		unsigned int SEQUENCE_HEADER = 0x000001B3;
		unsigned int PICTURE_START = 0x00000100;
		omx_private->start_code_with_type = 0xFFFFFFFF;

		for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
		{
			omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
			if ((search_option & CODETYPE_HEADER) && (omx_private->start_code_with_type == SEQUENCE_HEADER))
			{
				code_type = CODETYPE_HEADER;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
				break;
			}
			if ((search_option & CODETYPE_PICTURE) && (omx_private->start_code_with_type == PICTURE_START))
			{
				code_type = CODETYPE_PICTURE;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
				break;
			}
		}
	}
	else if (OMX_VIDEO_CodingAVC == omx_private->video_coding_type)
	{
		OMX_U32 uiMask;
	#ifdef DEFRAGMENT_INPUT_FRAME_AVC
		if(omx_private->bUseFrameDefragmentation == OMX_TRUE)
		{
		#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
			if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
			{
				while (temp_input_offset + MAX_NAL_STARTCODE_LEN <= omx_private->inputCurrLength)
				{
					OMX_U32 offset_count = 0;
					if(omx_private->splittedStartCodeLen)
					{
						// merge start bytes of curr. input with last bytes of prev. one.
						for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
						{
							if(omx_private->splittedStartCodeLen + offset_count++ >= MAX_NAL_STARTCODE_LEN)
								break;

							omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
						}
					}
					else
					{
						omx_private->start_code_with_type = BYTE_ORDERING(*(OMX_U32*)(omx_private->inputCurrBuffer + temp_input_offset));
					}

					// start code without type ?
					if((omx_private->start_code_with_type & AVC_NAL_STARTCODE_MASK) == AVC_NAL_STARTCODE)
					{
						COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
						temp_input_offset = (offset_count == 0) ? temp_input_offset + 1 : temp_input_offset - offset_count; 
						continue;
					}

					// beyond the possibility of start code with type ?
					if(omx_private->start_code_with_type > AVC_NAL_STARTCODE_MAX_VALUE)
					{
						COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();

						// Search how many consecutive 0 bytes exist and then pack them into start_code_with_type, if any.
						OMX_U32 offset = MAX_NAL_STARTCODE_LEN;
						DETERMINE_APPROPRIATE_NEXT_START_CODE_OFFSET(offset);
						temp_input_offset = (offset_count == 0) ? temp_input_offset + offset : temp_input_offset - offset_count; 
						continue;
					}

					uiMask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

					if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_AUD) 
					{
						// first time to detect frame delimiter ?
						if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
						{
							omx_private->bDetectFrameDelimiter = OMX_TRUE;
						}

						omx_private->frame_delimiter_offset = temp_input_offset;
					}
					else if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
					{
						omx_private->appendable_header_offset = temp_input_offset;
					}

					// In case of the element stream with frame delimiter, Keep searching until next delimiter is detected.
					if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
					{
						COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
						temp_input_offset += MAX_NAL_STARTCODE_LEN;
						continue; 
					}

					if (search_option & CODETYPE_HEADER && uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SPS) 
					{
						code_type = CODETYPE_HEADER;
						if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
						{
							temp_input_offset = omx_private->frame_delimiter_offset;
							omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
						}
						else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
						{
							if(omx_private->appendable_header_offset < temp_input_offset)
							{
								temp_input_offset = omx_private->appendable_header_offset;
								omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
							}
						}
					#ifdef PEEK_DEFRAGMENTED_FRAME
						PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "HEADER");
					#endif
						COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
						omx_private->start_code_with_type = 0xFFFFFFFF;
						break;
					}

					if ((search_option & CODETYPE_PICTURE) && 
							((uiMask >= AVC_NAL_STARTCODE_WITH_TYPE_SLICE) && (uiMask <= AVC_NAL_STARTCODE_WITH_TYPE_IDR)))
					{
						code_type = CODETYPE_PICTURE;
						if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
						{
							temp_input_offset = omx_private->frame_delimiter_offset;
							omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
						}
						else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
						{
							if(omx_private->appendable_header_offset < temp_input_offset)
							{
								temp_input_offset = omx_private->appendable_header_offset;
								omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
							}
						}
					#ifdef PEEK_DEFRAGMENTED_FRAME
						PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "PICTURE");
					#endif
						COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
						omx_private->start_code_with_type = 0xFFFFFFFF;
						break;
					}

					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					temp_input_offset += MAX_NAL_STARTCODE_LEN;
				}

				// If any bytes remains unsearched, we should use it with next new buffer. 
				if(omx_private->start_code_with_type != 0xFFFFFFFF && temp_input_offset < omx_private->inputCurrLength)
				{
					for (omx_private->splittedStartCodeLen = 0; temp_input_offset + omx_private->splittedStartCodeLen < omx_private->inputCurrLength; omx_private->splittedStartCodeLen++)
					{
						omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset + omx_private->splittedStartCodeLen];

					}

					// sanity check the possibility of start code with type. 
					if((omx_private->start_code_with_type & 0x000000FF) > 0x01)
					{
						// Don't need to fill these bytes...
						temp_input_offset += omx_private->splittedStartCodeLen;
						omx_private->splittedStartCodeLen = 0;
					}
				}
			}
			else 
		#endif // FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
			{
				// Clean up the flags
				omx_private->isSplittedStartCode = OMX_FALSE;

				for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
				{
					omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];

					// Sanity check the NAL type.
					if (omx_private->inputCurrBuffer[temp_input_offset] & AVC_NAL_FORBIDDEN_ZERO_BIT_MASK)
					{
						continue;
					}

					uiMask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

					if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_AUD) 
					{
						// Is it first time to detect frame delimiter ?
						if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
						{
							omx_private->bDetectFrameDelimiter = OMX_TRUE;
						}

						omx_private->frame_delimiter_offset = temp_input_offset;
					}
					else if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
					{
						omx_private->appendable_header_offset = temp_input_offset;
					}

					if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
					{
						// Keep searching until AUD is detected.
						continue;
					}

					if (search_option & CODETYPE_HEADER && uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SPS) 
					{
						code_type = CODETYPE_HEADER;

						if(temp_input_offset < MIN_NAL_STARTCODE_LEN)
						{
							omx_private->isSplittedStartCode = OMX_TRUE;
						}
						else
						{
							if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
							{
								temp_input_offset = omx_private->frame_delimiter_offset;
								omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
							}
							else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
							{
								if(omx_private->appendable_header_offset < temp_input_offset)
								{
									// assign the target as min. buffer offset
									temp_input_offset = omx_private->appendable_header_offset;
									omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
								}
							}
							temp_input_offset -= MIN_NAL_STARTCODE_LEN;
						}
#ifdef PEEK_DEFRAGMENTED_FRAME
						PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "HEADER");
#endif
						omx_private->start_code_with_type = 0xFFFFFFFF;
						break;
					}

					if ((search_option & CODETYPE_PICTURE) && 
							((uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SLICE) 
							 || (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_IDR) 
							 || (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_DPA)
							 || (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_DPB)
							 || (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_DPC)))
					{
						LOGINFO("SearchCodeType: PICTURE - temp_input_offset(%d)", temp_input_offset);

						code_type = CODETYPE_PICTURE;

						if(temp_input_offset < MIN_NAL_STARTCODE_LEN)
						{
							omx_private->isSplittedStartCode = OMX_TRUE;
						}
						else
						{
							if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
							{
								temp_input_offset = omx_private->frame_delimiter_offset;
								omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
							}
							else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
							{
								if(omx_private->appendable_header_offset < temp_input_offset)
								{
									// assign the target as min. buffer offset
									temp_input_offset = omx_private->appendable_header_offset;
									omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
								}
							}

							temp_input_offset -= MIN_NAL_STARTCODE_LEN;
						}
					#ifdef PEEK_DEFRAGMENTED_FRAME
						PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "PICTURE");
					#endif
						omx_private->start_code_with_type = 0xFFFFFFFF;
						break;
					}
				}
			}
		}
		else
	#endif
		{
			omx_private->start_code_with_type = 0xFFFFFFFF;

			for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
			{
				omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
				uiMask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

				if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_AUD) 
				{
					// Is it first time to detect frame delimiter ?
					if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
					{
						omx_private->bDetectFrameDelimiter = OMX_TRUE;
					}

					omx_private->frame_delimiter_offset = temp_input_offset;
				}
				else if (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
				{
					omx_private->appendable_header_offset = temp_input_offset;
				}

				if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
				{
					// Keep searching until AUD is detected.
					continue;
				}

				if ((search_option & CODETYPE_HEADER) && (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SPS))
				{
					if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
					{
						temp_input_offset = omx_private->frame_delimiter_offset;
						omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
					}
					else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
					{
						if(omx_private->appendable_header_offset < temp_input_offset)
						{
							// assign the target as min. buffer offset
							temp_input_offset = omx_private->appendable_header_offset;
							omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
						}
					}
					code_type = CODETYPE_HEADER;
					temp_input_offset -= MIN_NAL_STARTCODE_LEN;
					break;
				}

				if ((search_option & CODETYPE_PICTURE) && 
						((uiMask == AVC_NAL_STARTCODE_WITH_TYPE_SLICE) || (uiMask == AVC_NAL_STARTCODE_WITH_TYPE_IDR)))
				{
					if (omx_private->frame_delimiter_offset != OMX_BUFF_OFFSET_UNASSIGNED)
					{
						temp_input_offset = omx_private->frame_delimiter_offset;
						omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
					}
					else if (omx_private->appendable_header_offset != OMX_BUFF_OFFSET_UNASSIGNED)
					{
						if(omx_private->appendable_header_offset < temp_input_offset)
						{
							// assign the target as min. buffer offset
							temp_input_offset = omx_private->appendable_header_offset;
							omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
						}
					}
					code_type = CODETYPE_PICTURE;
					temp_input_offset -= MIN_NAL_STARTCODE_LEN;
					break;
				}
			}
		}
	}
	else if (OMX_VIDEO_CodingWMV == omx_private->video_coding_type)
	{
		unsigned int WVC1_VIDEO_HEADER = 0x0000010F;
		unsigned int WVC1_VIDEO_FRAME = 0x0000010D;
		omx_private->start_code_with_type = 0xFFFFFFFF;

		for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
		{
			omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
			if ((search_option & CODETYPE_HEADER) && (omx_private->start_code_with_type == WVC1_VIDEO_HEADER))
			{
				code_type = CODETYPE_HEADER;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
				break;
			}
			if ((search_option & CODETYPE_PICTURE) && (omx_private->start_code_with_type == WVC1_VIDEO_FRAME))
			{
				code_type = CODETYPE_PICTURE;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
				break;
			}
		}
	}

	*input_offset = temp_input_offset;

	return code_type;
}

static void VideoDecErrorProcess(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

	/* explicityly consume input buffer */ 
	pInputBuffer->nFilledLen = 0;

	if(omx_private->isVPUClosed != OMX_TRUE)
	{
#if 0	
		/* close VPU */ 
		gspfVDec( VDEC_CLOSE, NULL, NULL, &omx_private->gsVDecOutput );
#endif
		omx_private->isVPUClosed = OMX_TRUE;

		/* Report error event */ 
		(*(omx_private->callbacks->EventHandler))(openmaxStandComp, omx_private->callbackData,
							   OMX_EventError, OMX_ErrorHardware,
							   0, NULL);	
	}

	return;
}


#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
static int clear_vpu_buffer(int bufferIdx)
{
	int ret =0;
	int cleared_buff_count = 0;
	
	while(bufferIdx >= Display_Buff_ID[out_index] && used_fifo_count>0)
	{
		ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL );

		if(ret  < 0 )
		{
			LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
			return ret;
		}
		///wz// LOGE("### FLAG CLEAR idx(%d), displayed(%d)", out_index, tmp ) ;
		out_index = (out_index + 1) % max_fifo_cnt;
		used_fifo_count--;
		cleared_buff_count++;
	}

	if(cleared_buff_count == 0)
	{
		ioctl(g_hFb, TCC_LCDC_VIDEO_CLEAR_FRAME, Display_Buff_ID[out_index]);
		LOGE("Video Buffer Clear Sync Fail : %d %d\n", bufferIdx, Display_Buff_ID[out_index]);
		ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL );

		if(ret  < 0 )
		{
			LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
			return ret;
		}
		///wz// LOGE("### FLAG CLEAR idx(%d), displayed(%d)", out_index, tmp ) ;
		out_index = (out_index + 1) % max_fifo_cnt;
		used_fifo_count--;
	}

	return 0;
}
#endif

/** This function is used to process the input buffer and provide one output buffer
  */
void omx_videodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer	, OMX_BUFFERHEADERTYPE* pOutputBuffer) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_S32 ret;
	OMX_S32 nOutputFilled = 0;
	OMX_S32 nLen = 0;
	int internalOutputFilled=0;
	OMX_U32 nSize;
	OMX_U32 output_len;
	OMX_U32 input_offset = 0;
	OMX_U32 code_type = CODETYPE_NONE;
	int decode_result;
	int FourCC, i;
	dec_disp_info_t dec_disp_info_tmp;
	OMX_U8* p;
	tDEC_FRAME_INPUT Input;
	tDEC_FRAME_OUTPUT Output;
	tDEC_RESULT Result;

    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	memset(&Input, 0x00, sizeof(tDEC_FRAME_INPUT));
	
	if(omx_private->state != 3){
	    LOGE("=> omx_private->state != 3");
		return;
	}
	#ifdef DIVX_DRM5
	if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
	{
		DivxDecryptEx(pInputBuffer->pBuffer,pInputBuffer->nFilledLen);
	}
	#endif

    /** Fill up the current input buffer when a new buffer has arrived */
    if(omx_private->isNewBuffer) {
		//SSG:: check whether the play mode is normal or thumbnail mode
		if ( omx_private->isFirst_Frame && !(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) )
		{
			if ( pInputBuffer->nFlags & OMX_BUFFERFLAG_THUMBNAIL_MODE )
			{
				DBUG_MSG("this is thumbnail mode");
				omx_private->isThumbnailMode = OMX_TRUE;
			}
			omx_private->isFirst_Frame = OMX_FALSE;
		}

        omx_private->inputCurrBuffer = pInputBuffer->pBuffer;
        omx_private->inputCurrLength = pInputBuffer->nFilledLen;
        omx_private->isNewBuffer = 0;

		if (CONTAINER_TYPE_TS == omx_private->container_type && omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
		{
			// Remove additional NAL start code inserted by PV omx base dec node not to confuse 
			// the detection of appropriate AVC NAL type in case input buffer contains partial/multiple frame(s). 
			p = omx_private->inputCurrBuffer;
			if(p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01)
			{
	#if 0 // opencore 
				// Skip input buffer offset to the start position of 3-byte nal start code.
				if(p[4] == 0x00 && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x01)
				{
					omx_private->inputCurrBuffer += MAX_NAL_STARTCODE_LEN + 1;
					omx_private->inputCurrLength -= MAX_NAL_STARTCODE_LEN + 1;
					pInputBuffer->nFilledLen -= MAX_NAL_STARTCODE_LEN + 1;
				}
				else
				{
					omx_private->inputCurrBuffer += MIN_NAL_STARTCODE_LEN + 1;
					omx_private->inputCurrLength -= MIN_NAL_STARTCODE_LEN + 1;
					pInputBuffer->nFilledLen -= MIN_NAL_STARTCODE_LEN + 1;
				}
	#else // stagefright 
				omx_private->inputCurrBuffer += 1;
				omx_private->inputCurrLength -= 1;
				pInputBuffer->nFilledLen -= 1;
	#endif
			}
		}

		LOGERR("New Buffer -----> inputCurrLen:%d, PTS:%d, offset:%d, flag:%d", omx_private->inputCurrLength, pInputBuffer->nTimeStamp/1000, pInputBuffer->nOffset, pInputBuffer->nFlags);
	#ifdef PEEK_DEFRAGMENTED_FRAME
		PrintHexDataFrontRear(omx_private->inputCurrBuffer, omx_private->inputCurrLength, "Buffer");
	#endif
    }

	pOutputBuffer->nFilledLen = 0;
	pOutputBuffer->nOffset = 0;

	// SSG : extract extra info from the end of input data
	if (pInputBuffer->nFilledLen >= sizeof(CDK_ANDROID_VIDEO_EXTRA_INFO))
	{
		CDK_ANDROID_VIDEO_EXTRA_INFO *ext_info = (CDK_ANDROID_VIDEO_EXTRA_INFO*)(pInputBuffer->pBuffer + pInputBuffer->nFilledLen - sizeof(CDK_ANDROID_VIDEO_EXTRA_INFO));
		if (ext_info->id == CDK_ANDROID_VIDEO_EXTRA_DATA_IDENTIFIER)
		{
#if 0
			switch(ext_info->mode)
			{
				case CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE:
					LOGD("normal mode set");
					omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
					vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight);
					break;
				case CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE_STEPUP1:
					LOGD("normal mode x1.0 ~ x1.5 set");
					omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
					vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
					break;
				case CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE_STEPUP2:
					LOGD("normal mode x1.5 ~ x2.0 set");
					omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
					vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
					break;
				case CDK_ANDROID_VIDEO_EXTRA_FRAME_SKIP_MODE:
					LOGD("b frame skip mode set");
					omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_ONLY_B;
					vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
					break;
				default:
					break;
			}
#endif
			pInputBuffer->nFilledLen -= sizeof(CDK_ANDROID_VIDEO_EXTRA_INFO);
        	omx_private->inputCurrLength = pInputBuffer->nFilledLen;
		}
	}

	while (!nOutputFilled) {
	    
	    if (omx_private->isFirstBuffer) {
	        tsem_down(omx_private->avCodecSyncSem);
	        omx_private->isFirstBuffer = 0;
	    }

	#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
		if(omx_private->frameFilterPTS && pInputBuffer->nTimeStamp/1000 >= omx_private->frameFilterPTS)
			omx_private->isFrameDiscarded = OMX_TRUE; 

		if(omx_private->isFrameDiscarded == OMX_TRUE)
		{
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
			return;
		}
	#endif

	    //////////////////////////////////////////////////////////////////////////////////////////
	        /*ZzaU :: remove NAL-Start Code when there are double codes. ex) AVI container */
#if (USE_TCC_PARSER == 0) // SSG
		if (CONTAINER_TYPE_TS != omx_private->container_type && omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
		{
			p = omx_private->inputCurrBuffer;
			
			if(p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01 && p[4] == 0x00 && p[5] == 0x00 && p[6] == 0x00 && p[7] == 0x01)
			{
				input_offset = 4;
				DBUG_MSG("Double NAL-Start Code!!");
			}
			else if(p[0] == 0x00 && p[1] == 0x00 && p[2] == 0x00 && p[3] == 0x01 && p[4] == 0x00 && p[5] == 0x00 && p[6] == 0x01)
			{
				input_offset = 3;
				DBUG_MSG("remove 00 00 01 behind NAL-Start Code!!");
				pInputBuffer->pBuffer[3] = 0x00;
			}
		}
#endif
	    if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
			DBUG_MSG("Config data IN!!");
			
			ExtractConfigData(omx_private, input_offset);
			
			omx_private->isNewBuffer = 1;
			pOutputBuffer->nFilledLen = 0;
			pInputBuffer->nFilledLen = 0;	
			
			omx_private->bDetectFrameDelimiter = OMX_FALSE;
			omx_private->start_code_with_type = 0xFFFFFFFF;

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
			ioctl(g_hFb, TCC_LCDC_VIDEO_SET_FRAMERATE, omx_private->gsVDecUserInfo.frame_rate ) ;  // TCC_LCDC_HDMI_GET_DISPLAYED 
#endif
			return;
		}    

#if 1		
		Input.inputStreamAddr = omx_private->inputCurrBuffer;
		Input.inputStreamSize = omx_private->inputCurrLength;
		Input.nTimeStamp	= (int)(pInputBuffer->nTimeStamp/1000);				
#endif
		
		if(!omx_private->avcodecInited) 
		{
			if (CONTAINER_TYPE_TS == omx_private->container_type || CONTAINER_TYPE_MPG== omx_private->container_type)
			{
				if (0 == omx_private->szConfigdata)
				{
					LOGINFO("[BufMgmtCB] Call CODETYPE_HEADER");
					SearchCodeType(omx_private, &input_offset, CODETYPE_HEADER);
					if (input_offset >= omx_private->inputCurrLength)
					{
						omx_private->isNewBuffer = 1;
						pInputBuffer->nFilledLen = 0;
						LOGERR("[BufMgmtCB] HEADER is fragmented !!");
						return;
					}

					omx_private->inputCurrBuffer += input_offset;
					omx_private->inputCurrLength -= input_offset;
					pInputBuffer->nFilledLen -= input_offset;
					input_offset = 0;
				}
			}

			DBUG_MSG("VPU Format = %d, size = %d x %d", omx_private->gsVDecInit.m_iBitstreamFormat, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight);

			omx_private->frameSearchOrSkip_flag = 0;
			omx_private->gsVDecInit.m_iPicWidth				= outPort->sPortParam.format.video.nFrameWidth;
			omx_private->gsVDecInit.m_iPicHeight			= outPort->sPortParam.format.video.nFrameHeight;
			omx_private->gsVDecInit.m_bEnableVideoCache		= 0;//1;	// Richard_20100507 Don't use video cache 
			omx_private->gsVDecInit.m_bEnableUserData 		= 0;
			omx_private->gsVDecInit.m_pExtraData			= omx_private->extradata;
			omx_private->gsVDecInit.m_iExtraDataLen			= omx_private->extradata_size;	
			
			omx_private->gsVDecInit.m_bM4vDeblk 			= 0;//pCdk->m_bM4vDeblk;
			omx_private->gsVDecInit.m_uiDecOptFlags			= 0;
			omx_private->gsVDecInit.m_uiMaxResolution 		= 0;//pCdk->m_uiVideoMaxResolution;			
			omx_private->gsVDecInit.m_bFilePlayEnable 		= 1;

			if(isSWCodec(omx_private->gsVDecInit.m_iBitstreamFormat) 
#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)	
				|| (omx_private->video_coding_type == OMX_VIDEO_CodingMJPEG)
#endif
			)
				omx_private->gsVDecInit.m_bCbCrInterleaveMode	= 0;
			else
				omx_private->gsVDecInit.m_bCbCrInterleaveMode	= 1;
			
			{			
				dec_disp_info_input.m_iStdType 			= omx_private->gsVDecInit.m_iBitstreamFormat;
				dec_disp_info_input.m_iTimeStampType 	= CDMX_PTS_MODE;	// Presentation Timestamp (Display order)

				if(omx_private->isFromTCCParser)
				{				
					dec_disp_info_input.m_iFmtType 		= omx_private->container_type;

					if(omx_private->container_type == CONTAINER_TYPE_AVI || omx_private->container_type == CONTAINER_TYPE_MP4)
					{
						DBUG_MSG("TimeStampType = CDMX_DTS_MODE");
						dec_disp_info_input.m_iTimeStampType = CDMX_DTS_MODE;	// Decode Timestamp (Decode order)
					}
					else
					{
						DBUG_MSG("TimeStampType = CDMX_PTS_MODE");
					}
				}
				else
				{
					DBUG_MSG("This file comes from PV MP4 parser node. TimeStampType = CDMX_DTS_MODE");
					dec_disp_info_input.m_iTimeStampType = CDMX_DTS_MODE;	// Decode Timestamp (Decode order)
					dec_disp_info_input.m_iFmtType 		 = CONTAINER_TYPE_MP4;
				}
				disp_pic_info ( CVDEC_DISP_INFO_INIT, (void*)&dec_disp_info_ctrl, (void*)dec_disp_info,(void*)&dec_disp_info_input, omx_private);
			}

			if(omx_private->seq_header_init_error_count == SEQ_HEADER_INIT_ERROR_COUNT)
			{
#if 1
				{
					tDEC_INIT_PARAMS init;
			
					init.container_type		= omx_private->container_type;
					init.picWidth			= outPort->sPortParam.format.video.nFrameWidth;
					init.picHeight			= outPort->sPortParam.format.video.nFrameHeight;

					switch(omx_private->gsVDecInit.m_iBitstreamFormat)
					{
						case STD_RV: 	init.codecFormat = CODEC_FORMAT_RV; 	break;
						case STD_H263: 	init.codecFormat = CODEC_FORMAT_H263; 	break;
						case STD_AVC: 	init.codecFormat = CODEC_FORMAT_H264; 	break;
						case STD_MPEG4: init.codecFormat = CODEC_FORMAT_MPEG4; 	break;
						case STD_VC1: 	init.codecFormat = CODEC_FORMAT_VC1; 	break;
						case STD_DIV3: 	init.codecFormat = CODEC_FORMAT_DIV3; 	break;
						case STD_MPEG2: init.codecFormat = CODEC_FORMAT_MPEG2; 	break;
						case STD_MJPG: 	init.codecFormat = CODEC_FORMAT_MJPG; 	break;
						default : 		init.codecFormat = CODEC_FORMAT_H264; 	break;						
					}
					
					if((ret = DECODER_INIT(&init)) < 0)
					{
						LOGE( "[VDEC_INIT] [Err:%ld] video decoder init", ret );
						
						if(ret != -VPU_ENV_INIT_ERROR) //to close vpu!!
							omx_private->isVPUClosed = OMX_FALSE;			
						
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
						return;
					}
				}
#else
				if( (ret = gspfVDec( VDEC_INIT, NULL, &omx_private->gsVDecInit, &omx_private->gsVDecUserInfo )) < 0 )	
				{
					LOGE( "[VDEC_INIT] [Err:%ld] video decoder init", ret );

					if(ret != -VPU_ENV_INIT_ERROR) //to close vpu!!
						omx_private->isVPUClosed = OMX_FALSE;			

					VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
					return;
				}
#endif

				omx_private->isVPUClosed = OMX_FALSE;
			}			

			if(omx_private->szConfigdata != 0)
			{
				if (CONTAINER_TYPE_TS == omx_private->container_type)
				{
					LOGE("BufMgmtCB: TS fileformat stream can't use szConfigdata !!");
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
					return;
				}

				omx_private->gsVDecInput.m_pInp[PA] = omx_private->gsVDecInput.m_pInp[VA] = omx_private->pConfigdata;
				omx_private->gsVDecInput.m_iInpLen = omx_private->szConfigdata;
#if 1
				Input.inputStreamAddr = omx_private->pConfigdata;
				Input.inputStreamSize = omx_private->szConfigdata;
#endif
				
			}
			else
			{
			#ifdef DEFRAGMENT_INPUT_FRAME
				if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
				{
					omx_private->gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA);
					omx_private->gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA);
					memcpy(omx_private->gsVDecInput.m_pInp[VA], omx_private->inputCurrBuffer + input_offset, omx_private->inputCurrLength - input_offset);
				}
				else
			#endif
				{
					omx_private->gsVDecInput.m_pInp[PA] = omx_private->gsVDecInput.m_pInp[VA] = omx_private->inputCurrBuffer + input_offset;
				}
				omx_private->gsVDecInput.m_iInpLen	= omx_private->inputCurrLength - input_offset;

#if 1
				Input.inputStreamAddr = omx_private->inputCurrBuffer + input_offset;
				Input.inputStreamSize = omx_private->inputCurrLength - input_offset;
#endif				
			}

#if 1
			{
				if((ret = DECODER_DEC(&Input, &Output, &Result)) < 0)
				{
					if(omx_private->seq_header_init_error_count != 0)
						omx_private->seq_header_init_error_count--;
					if ( (omx_private->seq_header_init_error_count == 0) ||
						 (ret == -RETCODE_INVALID_STRIDE) )
					{
						LOGE( "[VDEC_DEC_SEQ_HEADER] [Err:%ld]", ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
						return;
					}
					else
					{
						DBUG_MSG("skip seq header frame, data len %d", omx_private->gsVDecInput.m_iInpLen);
						LOGI( "[VDEC_DEC_SEQ_HEADER] retry %d using next frame!",  SEQ_HEADER_INIT_ERROR_COUNT - omx_private->seq_header_init_error_count);
				
						omx_private->isNewBuffer = 1;
						pInputBuffer->nFilledLen = 0;
						pOutputBuffer->nFilledLen = 0;
				
						return;
					}
				}
			}

			omx_private->avcodecInited = 1;

			// set the flag to disable further processing until Client reacts to this by doing dynamic port reconfiguration
			ret = isPortChange(openmaxStandComp, Output.picWidth, Output.picHeight);
			if(ret < 0) {//max-resolution over!!
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
				return;
			}

			if(ret == 1) //port reconfiguration!!
				return; 	
#else

			
			if( (ret = gspfVDec( VDEC_DEC_SEQ_HEADER, NULL, &omx_private->gsVDecInput, &omx_private->gsVDecOutput )) < 0 )
			{
				if(omx_private->seq_header_init_error_count != 0)
					omx_private->seq_header_init_error_count--;
				if ( (omx_private->seq_header_init_error_count == 0) ||
					 (ret == -RETCODE_INVALID_STRIDE) )
				{
					LOGE( "[VDEC_DEC_SEQ_HEADER] [Err:%ld]", ret );
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
					return;
				}
				else
				{
					DBUG_MSG("skip seq header frame, data len %d", omx_private->gsVDecInput.m_iInpLen);
					LOGI( "[VDEC_DEC_SEQ_HEADER] retry %d using next frame!",  SEQ_HEADER_INIT_ERROR_COUNT - omx_private->seq_header_init_error_count);

					omx_private->isNewBuffer = 1;
					pInputBuffer->nFilledLen = 0;
					pOutputBuffer->nFilledLen = 0;

					return;
				}
			}

			video_dec_idx = 0;
			max_fifo_cnt = VPU_BUFF_COUNT;
			if(!isSWCodec(omx_private->gsVDecInit.m_iBitstreamFormat))
			{
				max_fifo_cnt = vpu_get_frame_count(VPU_BUFF_COUNT);// - omx_private->gsVDecOutput.m_pInitialInfo->m_iMinFrameBufferCount;
				if(max_fifo_cnt > VPU_BUFF_COUNT)
					max_fifo_cnt = VPU_BUFF_COUNT;
			}
			
			omx_private->avcodecInited = 1;

		#ifdef DEFRAGMENT_INPUT_FRAME
			if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
			{
				omx_private->code_type = CODETYPE_NONE;
			}
		#endif

			// set the flag to disable further processing until Client reacts to this by doing dynamic port reconfiguration
			ret = isPortChange(openmaxStandComp);
			if(ret < 0) {//max-resolution over!!
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
				return;
			}

			if(ret == 1) //port reconfiguration!!
				return; 
#endif
		}

#if 1
		if(pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
		{
			LOGI("OMX_BUFFERFLAG_SYNCFRAME");
			Input.seek = 1;
		}
#else
		if(pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
		{
			DPRINTF_DEC_STATUS("[SEEK] I-frame Search Mode enable");
			omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955
			omx_private->frameSearchOrSkip_flag = 1;
			
			if(!omx_private->isFromTCCParser && omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
				omx_private->frameSearchOrSkip_flag = 0;			

			disp_pic_info( CVDEC_DISP_INFO_RESET, (void*)&dec_disp_info_ctrl, (void*)dec_disp_info,(void*)&dec_disp_info_input, omx_private);		

#ifdef HAVE_ANDROID_OS 
			/*ZzaU :: Clear all decoded frame-buffer!!*/
			if(max_fifo_cnt != 0)
			{
			#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				LOGD("Used Buffer Count %d", used_fifo_count);
			#endif
				while(in_index != out_index)
				{
					//DPRINTF_DEC_STATUS("DispIdx Clear %d", Display_index[out_index]);
					LOGD("DispIdx Clear %d", Display_index[out_index]);
					if( ( ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL ) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
						return;
					}
					out_index = (out_index + 1) % max_fifo_cnt;
				}
				in_index = out_index = frm_clear = 0;
			#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				used_fifo_count = 0;
			#endif
			}
#endif
		#ifdef DEFRAGMENT_INPUT_FRAME
			if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
			{
				omx_private->code_type = CODETYPE_NONE;
			}
		#endif
		}

	#ifdef DEFRAGMENT_INPUT_FRAME
		if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
		{
			if (CODETYPE_NONE == omx_private->code_type)
			{
				// search current frame code type
				LOGINFO("[BufMgmtCB] Call CODETYPE_ALL - input_offset:%d", input_offset);
				code_type = SearchCodeType(omx_private, &input_offset, CODETYPE_ALL);

			#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
				if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
				{
					if(input_offset + MAX_NAL_STARTCODE_LEN >= omx_private->inputCurrLength)
					{
						omx_private->isNewBuffer = 1;
						pOutputBuffer->nFilledLen = 0;
						pInputBuffer->nFilledLen = 0;

						return;
					}
				}
				else
			#endif
				if (input_offset >= omx_private->inputCurrLength)
				{
					omx_private->isNewBuffer = 1;
					pOutputBuffer->nFilledLen = 0;
					pInputBuffer->nFilledLen = 0;

					return;
				}

				omx_private->gsVDecInput.m_iInpLen = 0;
				omx_private->nTimeStamp = pInputBuffer->nTimeStamp;
				pInputBuffer->nTimeStamp += (((((1000 * 1000) << 10) / omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10) * 1000);

				omx_private->code_type = code_type;
				if (input_offset > 0)
				{
					omx_private->inputCurrBuffer += input_offset;
					omx_private->inputCurrLength -= input_offset;
					pInputBuffer->nFilledLen -= input_offset;
				}

			#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
				if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
					input_offset = MAX_NAL_STARTCODE_LEN;
				else
			#endif
					input_offset = 1;
			}

			if (CODETYPE_HEADER == omx_private->code_type)
			{
				LOGINFO("[BufMgmtCB] CODETYPE_HEADER --> Call CODETYPE_PICTURE");
				code_type = SearchCodeType(omx_private, &input_offset, CODETYPE_PICTURE);

				if(omx_private->isSplittedStartCode == OMX_TRUE)
				{
					LOGD("[BufMgmtCB] NAL Start code is Splitted !!");
					omx_private->gsVDecInput.m_iInpLen -= (MIN_NAL_STARTCODE_LEN - input_offset);
				}
				else if (input_offset > 0)
				{
					LOGINFO("[BufMgmtCB] CODETYPE_HEADER--> Call CODETYPE_PICTURE : Copy (%d) bytes", input_offset);
					memcpy(omx_private->gsVDecInput.m_pInp[VA] + omx_private->gsVDecInput.m_iInpLen, omx_private->inputCurrBuffer, input_offset);
					omx_private->gsVDecInput.m_iInpLen += input_offset;
				#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
					if(omx_private->splittedStartCodeLen)
					{
						input_offset += omx_private->splittedStartCodeLen;
					}
				#endif
				}

				omx_private->code_type = CODETYPE_PICTURE; 

				if (input_offset >= omx_private->inputCurrLength)
				{
					omx_private->isNewBuffer = 1;
					pOutputBuffer->nFilledLen = 0;
					pInputBuffer->nFilledLen = 0;
					return;
				}

				if (input_offset > 0)
				{
					omx_private->inputCurrBuffer += input_offset;
					omx_private->inputCurrLength -= input_offset;
					pInputBuffer->nFilledLen -= input_offset;
				}

			#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
				if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
					input_offset = MAX_NAL_STARTCODE_LEN;
				else
			#endif
					input_offset = 1;
			}

			if (CODETYPE_PICTURE == omx_private->code_type)
			{
				LOGINFO("[BufMgmtCB] CODETYPE_PICTURE --> Call CODETYPE_PICTURE : input_offset = %d", input_offset);
				code_type = SearchCodeType(omx_private, &input_offset, CODETYPE_PICTURE);
				if(omx_private->isSplittedStartCode == OMX_TRUE)
				{
					LOGW("[BufMgmtCB] NAL Start code is Splitted !!");
					omx_private->gsVDecInput.m_iInpLen -= (MIN_NAL_STARTCODE_LEN - input_offset);
				}
				else if (input_offset > 0)
				{
					LOGINFO("[BufMgmtCB] CODETYPE_PICTURE--> Call CODETYPE_PICTURE : Copy (%d) bytes", input_offset);
					memcpy(omx_private->gsVDecInput.m_pInp[VA] + omx_private->gsVDecInput.m_iInpLen, omx_private->inputCurrBuffer, input_offset);
					omx_private->gsVDecInput.m_iInpLen += input_offset;
				#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
					if(omx_private->splittedStartCodeLen)
					{
						input_offset += omx_private->splittedStartCodeLen;
					}
				#endif
				}

				if (input_offset >= omx_private->inputCurrLength)
				{
					omx_private->isNewBuffer = 1;
					pOutputBuffer->nFilledLen = 0;
					pInputBuffer->nFilledLen = 0;
					return;
				}

				omx_private->code_type = CODETYPE_NONE;
				if (input_offset > 0)
				{
					omx_private->inputCurrBuffer += input_offset;
					omx_private->inputCurrLength -= input_offset;
					pInputBuffer->nFilledLen -= input_offset;
					input_offset = 0;
				}
			}
		}
		else
	#endif // DEFRAGMENT_INPUT_FRAME
#endif
		{
			if (CONTAINER_TYPE_TS == omx_private->container_type)
			{
				code_type = SearchCodeType(omx_private, &input_offset, CODETYPE_ALL);
				if (input_offset >= omx_private->inputCurrLength)
				{
					omx_private->isNewBuffer = 1;
					pOutputBuffer->nFilledLen = 0;
					pInputBuffer->nFilledLen = 0;

					return;
				}
			}
			omx_private->gsVDecInput.m_pInp[PA] = omx_private->gsVDecInput.m_pInp[VA] = omx_private->inputCurrBuffer + input_offset;
			omx_private->gsVDecInput.m_iInpLen  = omx_private->inputCurrLength - input_offset;
#if 1
			Input.inputStreamAddr = omx_private->inputCurrBuffer + input_offset;
			Input.inputStreamSize = omx_private->inputCurrLength - input_offset;
#endif				

		}

#if 1
		if(DECODER_DEC(&Input, &Output, &Result) < 0)
		{
			LOGE( "[VDEC_DECODE] [Err:%ld] video decode", ret );
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
			return;
		}

		if(!Result.no_frame_output)
		{
			/* physical address */
			for(i=0;i<3;i++)
				memcpy(pOutputBuffer->pBuffer+i*4, &Output.bufPhyAddr[i], 4);
			
			/* logical address */
			for(i=3;i<6;i++)
				memcpy(pOutputBuffer->pBuffer+i*4, &Output.bufVirtAddr[i-3], 4);
			
			*((OMX_U32*)(pOutputBuffer->pBuffer+24)) = Output.picWidth;
			*((OMX_U32*)(pOutputBuffer->pBuffer+28)) = Output.picHeight;
			
			*((OMX_U32*)(pOutputBuffer->pBuffer+32)) = outPort->sPortParam.format.video.nFrameWidth;
			*((OMX_U32*)(pOutputBuffer->pBuffer+36)) = outPort->sPortParam.format.video.nFrameHeight;

	#ifdef HARDWARE_CODEC
			{
				TCC_PLATFORM_PRIVATE_PMEM_INFO *pmemInfoPtr;
				
				pmemInfoPtr = (TCC_PLATFORM_PRIVATE_PMEM_INFO *) pOutputBuffer->pPlatformPrivate;
			#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, output_len);
			#else
				memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, sizeof(int)*6);
			#endif
			}
	#endif
			pOutputBuffer->nTimeStamp = Output.nTimeStamp * 1000;
			pOutputBuffer->nFilledLen = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2; 

			
			if ( omx_private->isThumbnailMode ) {
				/*ZzaU :: android framework directly make thumbnail for video player using jpeg encoder */
		#if 1
				pOutputBuffer->pBuffer = (OMX_U8 *)MakeThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, 1, &Output);
		#else
				pOutputBuffer->pBuffer = (OMX_U8 *)MakeThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, 0, &Output);
		#endif
				//xelloss thumbnail_debugging, thumbnail frame buffer might be not send to application at one time because of procedure by isPortChange() 
				//so, isThumbnailMode be set component at end part of the component.(but .. guess that already setting at init part may be enough)
				//eclair, do not make 'isThumbnailMode' FALSE to prevent passing 'phy addr' to application
				#if 1//ZzaU:: restore this setting for eclair!!  //def _TCC8900_  use sema 'bPortChangeSem' instead of this flag
				//if(!omx_private->isFromTCCParser)
				//	omx_private->isThumbnailMode = OMX_FALSE;
				#else
				omx_private->isThumbnailMode = OMX_FALSE;					
				#endif
				
				omx_private->isThumbnailMade = OMX_TRUE; // SSG
			}	
	
		}
		
		if(!Result.need_input_retry)
		{
			omx_private->isNewBuffer = 1;
			pInputBuffer->nFilledLen = 0;	
		}

#else
		omx_private->gsVDecInput.m_iSkipFrameNum = 0;
		omx_private->gsVDecInput.m_iFrameSearchEnable = 0;

		switch(omx_private->i_skip_scheme_level)
		{
			case VDEC_SKIP_FRAME_DISABLE:
			case VDEC_SKIP_FRAME_EXCEPT_I:
				omx_private->gsVDecInput.m_iSkipFrameMode = omx_private->i_skip_scheme_level;
				break;
			case VDEC_SKIP_FRAME_ONLY_B:
				if(omx_private->i_skip_count == omx_private->i_skip_interval)
				{
					omx_private->gsVDecInput.m_iSkipFrameMode = omx_private->i_skip_scheme_level;
					omx_private->gsVDecInput.m_iSkipFrameNum = 1000;
				}
				else
				{
					omx_private->gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
				}
				break;
			default:
					omx_private->gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
				break;
		}
		
		if(omx_private->frameSearchOrSkip_flag == 1 )
		{
			omx_private->gsVDecInput.m_iSkipFrameNum = 1;
			//omx_private->gsVDecInput.m_iFrameSearchEnable = 0x001;	//I-frame (IDR-picture for H.264)
			omx_private->gsVDecInput.m_iFrameSearchEnable = 0x201;	//I-frame (I-slice for H.264) : Non IDR-picture
			omx_private->gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;

			if( omx_private->gsVDecInput.m_iFrameSearchEnable == 0x001 )
			{
				DPRINTF_DEC_STATUS( "[SEEK] I-frame Search Mode(IDR-picture for H.264) Enable!!!");
			}
			else if( omx_private->gsVDecInput.m_iFrameSearchEnable == 0x201 )
			{
				DPRINTF_DEC_STATUS( "[SEEK] I-frame Search Mode(I-slice for H.264) Enable!!!");
			}
		}
		
		if( omx_private->frameSearchOrSkip_flag == 2 )
		{
			omx_private->gsVDecInput.m_iSkipFrameNum = 1;
			omx_private->gsVDecInput.m_iFrameSearchEnable = 0;
			omx_private->gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_ONLY_B;	
			DPRINTF_DEC_STATUS( "[SEEK] B-frame Skip Mode Enable!!! \n");
		}

		if(omx_private->isVPUClosed == OMX_TRUE)
		{
			LOGE( "Now VPU has been closed , return " );
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
			return;
		}

		if( (ret = gspfVDec( VDEC_DECODE, NULL, &omx_private->gsVDecInput, &omx_private->gsVDecOutput )) < 0 )
		{
			LOGE( "[VDEC_DECODE] [Err:%ld] video decode", ret );
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
			return;
		}	

		if(omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL) 
		{
			// Current input stream should be used next time.
			LOGE("VPU_DEC_BUF_FULL");
			if (omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS)
				decode_result = 0; // display Index : processed.
			else
				decode_result = 1; // display Index : not processsed.
		} 
		else 
		{
			if(omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS)
				decode_result = 2; // display Index : proceed.
			else
				decode_result = 3; // display Index : not processsed.
		}

		switch(omx_private->i_skip_scheme_level)
		{
			case VDEC_SKIP_FRAME_DISABLE:
				break;
			case VDEC_SKIP_FRAME_EXCEPT_I:
			case VDEC_SKIP_FRAME_ONLY_B:
				if((omx_private->gsVDecInput.m_iSkipFrameMode != omx_private->i_skip_scheme_level) || (omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx == -2))
					omx_private->i_skip_count--;

				if(omx_private->i_skip_count < 0)
					omx_private->i_skip_count = omx_private->i_skip_interval;
				break;
		}

		//Update TimeStamp!!
		if(omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS
			&& omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
		{			
		#ifdef DEFRAGMENT_INPUT_FRAME
			if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
			{
				dec_disp_info_tmp.m_iTimeStamp 		= (int)(omx_private->nTimeStamp/1000);
			}
			else
		#endif
			{
				dec_disp_info_tmp.m_iTimeStamp 		= (int)(pInputBuffer->nTimeStamp/1000);
			}
			dec_disp_info_tmp.m_iFrameType 			= omx_private->gsVDecOutput.m_DecOutInfo.m_iPicType;
			dec_disp_info_tmp.m_iPicStructure 		= omx_private->gsVDecOutput.m_DecOutInfo.m_iPictureStructure;
			dec_disp_info_tmp.m_iRvTimeStamp 		= 0;
			dec_disp_info_tmp.m_iM2vFieldSequence   = 0;			
			dec_disp_info_tmp.m_iFrameSize 			= omx_private->gsVDecOutput.m_DecOutInfo.m_iConsumedBytes;// gsCDmxOutput.m_iPacketSize;
			dec_disp_info_tmp.m_iFrameDuration 		= 2;
			
			switch( omx_private->gsVDecInit.m_iBitstreamFormat)
			{
				case STD_RV:
					dec_disp_info_tmp.m_iRvTimeStamp = omx_private->gsVDecOutput.m_DecOutInfo.m_iRvTimestamp;
					if (omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS ) 
					{
						dec_disp_info_input.m_iFrameIdx = omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
						disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&dec_disp_info_ctrl, (void*)&dec_disp_info_tmp, (void*)&dec_disp_info_input, omx_private);
					}
					
					break;
					
				case STD_AVC:
					if ( ( omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && omx_private->gsVDecOutput.m_DecOutInfo.m_iPictureStructure == 0x3 )
						|| omx_private->gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame )
					{
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
					}

					if( omx_private->gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0)
					{
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_ODD_FIRST_FRAME;
					}

					dec_disp_info_tmp.m_iM2vFieldSequence = 0;
					dec_disp_info_input.m_iFrameIdx = omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&dec_disp_info_ctrl, (void*)&dec_disp_info_tmp, (void*)&dec_disp_info_input, omx_private);
					break;
					
				case STD_MPEG2: 					
					if( dec_disp_info_tmp.m_iPicStructure != 3 )
					{
						dec_disp_info_tmp.m_iFrameDuration = 1;
					}
					else if( omx_private->gsVDecOutput.m_pInitialInfo->m_iInterlace == 0 )
					{
						if( omx_private->gsVDecOutput.m_DecOutInfo.m_iRepeatFirstField == 0 )
							dec_disp_info_tmp.m_iFrameDuration = 2;
						else
							dec_disp_info_tmp.m_iFrameDuration = ( omx_private->gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0 )?4:6;
					}
					else
					{ 
						//pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
						/* interlaced sequence */
						if( omx_private->gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame == 0 )
							dec_disp_info_tmp.m_iFrameDuration = 2;
						else
							dec_disp_info_tmp.m_iFrameDuration = ( omx_private->gsVDecOutput.m_DecOutInfo.m_iRepeatFirstField == 0 )?2:3;
					}

					if ( ( omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && omx_private->gsVDecOutput.m_DecOutInfo.m_iPictureStructure == 0x3 )
						|| omx_private->gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame )
					{
						//LOGD("Interlaced Frame!!!");
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
					}

					if( omx_private->gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0)
					{
						//LOGD("Odd First Frame!!!");
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_ODD_FIRST_FRAME;
					}
					
					dec_disp_info_tmp.m_iM2vFieldSequence = omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence;
					dec_disp_info_input.m_iFrameIdx = omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					dec_disp_info_input.m_iFrameRate = omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vFrameRate;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&dec_disp_info_ctrl, (void*)&dec_disp_info_tmp, (void*)&dec_disp_info_input, omx_private);
					break;
					
				default:
					dec_disp_info_tmp.m_iM2vFieldSequence = 0;
					dec_disp_info_input.m_iFrameIdx = omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&dec_disp_info_ctrl, (void*)&dec_disp_info_tmp, (void*)&dec_disp_info_input, omx_private);
					break;
			}
			DPRINTF_DEC("IN-Buffer :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						pInputBuffer->pBuffer[0], pInputBuffer->pBuffer[1], pInputBuffer->pBuffer[2], pInputBuffer->pBuffer[3], pInputBuffer->pBuffer[4], 
						pInputBuffer->pBuffer[5], pInputBuffer->pBuffer[6], pInputBuffer->pBuffer[7], pInputBuffer->pBuffer[8], pInputBuffer->pBuffer[9]);
			//current decoded frame info
			DPRINTF_DEC( "[In - %s][N:%4d][LEN:%6d][RT:%8d] [DecoIdx:%2d][DecStat:%d][FieldSeq:%d][TR:%8d] ", 
							print_pic_type(omx_private->gsVDecInit.m_iBitstreamFormat, dec_disp_info_tmp.m_iFrameType, dec_disp_info_tmp.m_iPicStructure),
							video_dec_idx, pInputBuffer->nFilledLen, (int)(pInputBuffer->nTimeStamp/1000),
							omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx, omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
							omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence, omx_private->gsVDecOutput.m_DecOutInfo.m_iRvTimestamp);
		}
		else
		{
			DPRINTF_DEC("IN-Buffer :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						pInputBuffer->pBuffer[0], pInputBuffer->pBuffer[1], pInputBuffer->pBuffer[2], pInputBuffer->pBuffer[3], pInputBuffer->pBuffer[4], 
						pInputBuffer->pBuffer[5], pInputBuffer->pBuffer[6], pInputBuffer->pBuffer[7], pInputBuffer->pBuffer[8], pInputBuffer->pBuffer[9]);
			DPRINTF_DEC( "[Err In - %s][N:%4d][LEN:%6d][RT:%8d] [DecoIdx:%2d][DecStat:%d][FieldSeq:%d][TR:%8d] ", 
							print_pic_type(omx_private->gsVDecInit.m_iBitstreamFormat, dec_disp_info_tmp.m_iFrameType, dec_disp_info_tmp.m_iPicStructure),
							video_dec_idx, pInputBuffer->nFilledLen, (int)(pInputBuffer->nTimeStamp/1000),
							omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx, omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
							omx_private->gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence, omx_private->gsVDecOutput.m_DecOutInfo.m_iRvTimestamp);
		}

		//In case that only one field picture is decoded...
		#if 1
		if(omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS_FIELD_PICTURE)
		{
			dec_disp_info_t dec_disp_info_tmp;
			OMX_U32 inTS = pInputBuffer->nTimeStamp/1000;
			
			dec_disp_info_tmp.m_iFrameDuration = 1;

			if( dec_disp_info_ctrl.m_iFmtType  == CONTAINER_TYPE_MPG )
			{
			//LOGD("VPU_DEC_SUCCESS_FIELD_PICTURE m_iPTSInterval %d m_iLatestPTS %d inTS %d m_iRamainingDuration %d ",gsMPEG2PtsInfo.m_iPTSInterval , gsMPEG2PtsInfo.m_iLatestPTS,inTS,gsMPEG2PtsInfo.m_iRamainingDuration);
				if( inTS <= gsMPEG2PtsInfo.m_iLatestPTS )
					inTS = gsMPEG2PtsInfo.m_iLatestPTS + ((gsMPEG2PtsInfo.m_iPTSInterval * gsMPEG2PtsInfo.m_iRamainingDuration) >> 1);
				gsMPEG2PtsInfo.m_iLatestPTS = inTS;
				gsMPEG2PtsInfo.m_iRamainingDuration = 1;
			}
            
            #ifdef TS_TIMESTAMP_CORRECTION
			if( dec_disp_info_ctrl.m_iFmtType == CONTAINER_TYPE_TS)
			{
			    //LOGD("VPU_DEC_SUCCESS_FIELD_PICTURE m_iPTSInterval %d m_iLatestPTS %d inTS %d m_iRamainingDuration %d ", gsTSPtsInfo.m_iPTSInterval, gsTSPtsInfo.m_iLatestPTS, inTS, gsTSPtsInfo.m_iRamainingDuration);
				if( inTS <= gsTSPtsInfo.m_iLatestPTS )
					inTS = gsTSPtsInfo.m_iLatestPTS + ((gsTSPtsInfo.m_iPTSInterval * gsTSPtsInfo.m_iRamainingDuration) >> 1);
				gsTSPtsInfo.m_iLatestPTS = inTS;
				gsTSPtsInfo.m_iRamainingDuration = 1;
			}
            #endif
		}
		#endif

		
		if( omx_private->frameSearchOrSkip_flag 
			&& omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0 
			//&& omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS)
			&& (omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS || omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS_FIELD_PICTURE))
		{
			// frameType - 0: unknown, 1: I-frame, 2: P-frame, 3:B-frame
			int frameType = get_frame_type_for_frame_skipping( omx_private->gsVDecInit.m_iBitstreamFormat, 
															omx_private->gsVDecOutput.m_DecOutInfo.m_iPicType, 
															omx_private->gsVDecOutput.m_DecOutInfo.m_iPictureStructure );

			if( omx_private->gsVDecInput.m_iFrameSearchEnable )
			{
				omx_private->frameSearchOrSkip_flag = 2;//I-frame Search Mode disable and B-frame Skip Mode enable
				DPRINTF_DEC_STATUS("[SEEK] I-frame Search Mode disable and B-frame Skip Mode enable");				
			}
			else if( omx_private->gsVDecInput.m_iSkipFrameMode == VDEC_SKIP_FRAME_ONLY_B )
			{
				omx_private->frameSearchOrSkip_flag = 0; //B-frame Skip Mode disable
				DPRINTF_DEC_STATUS( "[SEEK] B-frame Skip Mode Disable after P-frame decoding!!!");
			}
		}

		if (CONTAINER_TYPE_TS != omx_private->container_type)
		{
			omx_private->inputCurrBuffer = pInputBuffer->pBuffer;
			omx_private->inputCurrLength = pInputBuffer->nFilledLen;
		}
		//////////////////////////////////////////////////////////////////////////////////////////

		if (omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS)
		{
			omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955

			/* physical address */
			for(i=0;i<3;i++)
				memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->gsVDecOutput.m_pDispOut[PA][i], 4);
			
			/* logical address */
			for(i=3;i<6;i++)
				memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->gsVDecOutput.m_pDispOut[VA][i-3], 4);

			*((OMX_U32*)(pOutputBuffer->pBuffer+24)) = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth;
			*((OMX_U32*)(pOutputBuffer->pBuffer+28)) = omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight;

			*((OMX_U32*)(pOutputBuffer->pBuffer+32)) = outPort->sPortParam.format.video.nFrameWidth;
			*((OMX_U32*)(pOutputBuffer->pBuffer+36)) = outPort->sPortParam.format.video.nFrameHeight;
						
			/* physical address 3 ea, logical address 3 ea, width and height of VPU output, width and height of actual frame */
			output_len = 4*6 + 8 + 8;

			//Get TimeStamp!!
			{
				dec_disp_info_t *pdec_disp_info = NULL;
				int dispOutIdx = omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;
				dec_disp_info_input.m_iFrameIdx = dispOutIdx;
				disp_pic_info( CVDEC_DISP_INFO_GET, (void*)&dec_disp_info_ctrl, (void*)&pdec_disp_info, (void*)&dec_disp_info_input, omx_private);

				if( pdec_disp_info != (dec_disp_info_t*)0 )
				{
	#ifdef JPEG_DECODE_FOR_MJPEG
					if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MJPG)
					{
						pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
					}
					else
	#endif
					if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_RV)
					{
						pOutputBuffer->nTimeStamp = (OMX_TICKS)pdec_disp_info->m_iRvTimeStamp * 1000;
					}
					else// if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG2)
					{
						pOutputBuffer->nTimeStamp = (OMX_TICKS)pdec_disp_info->m_iTimeStamp * 1000; //pdec_disp_info->m_iM2vFieldSequence * 1000;
					}

					
					if(omx_private->gsVDecInit.m_bEnableUserData)
					{
						print_user_data(omx_private->gsVDecOutput.m_DecOutInfo.m_UserDataAddress[VA]);
					}

					DPRINTF_DEC( "[Out - %s][N:%4d][LEN:%6d][RT:%8d] [DispIdx:%2d][OutStat:%d][FieldSeq:%d][TR:%8d] ", 
									print_pic_type(omx_private->gsVDecInit.m_iBitstreamFormat, pdec_disp_info->m_iFrameType, pdec_disp_info->m_iPicStructure),
									video_dec_idx, pdec_disp_info->m_iFrameSize, pdec_disp_info->m_iTimeStamp,
									omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus,
									pdec_disp_info->m_iM2vFieldSequence, pdec_disp_info->m_iRvTimeStamp);
				}
				else
				{
					//exception process!! temp!!
					pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
				}
			}

#ifdef HAVE_ANDROID_OS 
	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
			if(max_fifo_cnt != 0)
			{
                if(iVsyncMode)
                {
    				Display_index[in_index] = omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;
    				Display_Buff_ID[in_index] = buffer_unique_id;
    				*(unsigned int*)(&pOutputBuffer->pBuffer[output_len]) = buffer_unique_id;
    	 			output_len += 4;

    				DPRINTF_DEC_STATUS("DispIdx Queue %d", Display_index[in_index]);
    				in_index = (in_index + 1) % max_fifo_cnt;
    				used_fifo_count++;
    				buffer_unique_id++;

    				//LOGD("### in(%d), out(%d), addr(%x), addr(%x)", in_index, out_index, omx_private->gsVDecOutput.m_pDispOut[PA][0], omx_private->gsVDecOutput.m_pDispOut[VA][0] ) ;
    				//LOGE("### in(%d), out(%d), used(%d), max(%d)", in_index, out_index, used_fifo_count, max_fifo_cnt) ;
    				if(used_fifo_count == max_fifo_cnt)
    				{
    					int displayed_num;
    					int dispBuffId[VPU_BUFF_COUNT];
						int loopCount = 0;
    				    //LOGE("### buffer clear start",tmp) ;
    					displayed_num = ioctl(g_hFb, TCC_LCDC_VIDEO_GET_DISPLAYED, dispBuffId ) ;  // TCC_LCDC_HDMI_GET_DISPLAYED 

    					if(displayed_num < 0)
    					{
							if(errno == 1)
							{
	    						frm_clear = 1;
	    			  			//LOGE("### buffer clear by skip mode %d", errno) ;
	    						DPRINTF_DEC_STATUS("DispIdx Clear %d", Display_index[out_index]);
	    						ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL );
	    						if(ret)
	    							LOGE("Error Buffer Clear");

	    						out_index = (out_index + 1) % max_fifo_cnt;
	    						used_fifo_count-- ;
							}
							else
							{
								while(loopCount++ < MAX_CHECK_COUNT_FOR_CLEAR)
								{
									usleep(5000);
			    					displayed_num = ioctl(g_hFb, TCC_LCDC_VIDEO_GET_DISPLAYED, dispBuffId ) ;
									if(displayed_num>0)
									{
#if 0
			    						ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL );
			    						if(ret)
			    							LOGE("Error Buffer Clear");

			    						out_index = (out_index + 1) % max_fifo_cnt;
			    						used_fifo_count-- ;
#else
										ret = clear_vpu_buffer(dispBuffId[displayed_num-1]);
		    							if(ret  < 0 )
		    							{
		    								LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
											VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
											return;
		    							}										
#endif
										break;
									}
								}

								if(loopCount >= MAX_CHECK_COUNT_FOR_CLEAR)
								{
		    			  			LOGE("### clear buffer because waiting long time", errno) ;
		    						ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL );
	    							if(ret  < 0 )
	    							{
	    								LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
										VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
										return;
	    							}

		    						out_index = (out_index + 1) % max_fifo_cnt;	
		    						used_fifo_count-- ;									
								}
							}
    					}
    					else
    					{
    						int cleared_buff_count = 0;

    						frm_clear =0;
    						if(used_fifo_count <= displayed_num)
    						{
    							LOGE("Frame Buffer Sync Fail with FB Driver omx_buff(%d), clear_buff(%d)",used_fifo_count,  displayed_num);
    							displayed_num = used_fifo_count-1;
    						}

    				    	//LOGE("### normal buffer clear ",tmp) ;
							ret = clear_vpu_buffer(dispBuffId[displayed_num-1]);
							if(ret  < 0 )
							{
								LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
								VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
								return;
							}										
    					}
    				}
				}
				else
				{
					Display_index[in_index] = omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;
					DPRINTF_DEC_STATUS("DispIdx Queue %d", Display_index[in_index]);
					in_index = (in_index + 1) % max_fifo_cnt;
					
					if(in_index == 0 && !frm_clear)
						frm_clear = 1;

					if(frm_clear)
					{
	//					LOGE("DispIdx Clear %d", Display_index[out_index]);
						if( ( ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL ) ) < 0 )
						{
							LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
							VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
							return;
						}	
						out_index = (out_index + 1) % max_fifo_cnt;
					}
				}
			}
			else
			{		
				//DPRINTF_DEC_STATUS("@ DispIdx Queue %d", Display_index[in_index]);
				if( ( ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, NULL ) ) < 0 )
				{
					LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, ret );
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
					return;
				}
			}
	#else
			/*ZzaU :: Clear decoded frame-buffer according with sequence-order after it was used!!*/
			if(max_fifo_cnt != 0)
			{				
				Display_index[in_index] = omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;
				DPRINTF_DEC_STATUS("DispIdx Queue %d", Display_index[in_index]);
				in_index = (in_index + 1) % max_fifo_cnt;
				
				if(in_index == 0 && !frm_clear)
					frm_clear = 1;

				if(frm_clear)
				{
					DPRINTF_DEC_STATUS("Normal DispIdx Clear %d", Display_index[out_index]);
					if( ( ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &Display_index[out_index], NULL ) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", Display_index[out_index], ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
						return;
					}
					
					out_index = (out_index + 1) % max_fifo_cnt;
				}
			}
			else
			{		
				DPRINTF_DEC_STATUS("@ DispIdx Queue %d", Display_index[in_index]);
				if( ( ret = gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, NULL ) ) < 0 )
				{
					LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, ret );
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
					return;
				}
			}
	#endif
#endif
			video_dec_idx++;
		}
		else
		{
			LOGE( "[VDEC_DECODE] NO-OUTPUT!! m_iDispOutIdx = %d, m_iDecodedIdx = %d, m_iOutputStatus = %d, m_iDecodingStatus = %d, m_iNumOfErrMBs = %d", 
											omx_private->gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodedIdx,
											omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus, omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
											omx_private->gsVDecOutput.m_DecOutInfo.m_iNumOfErrMBs);
			output_len = 0;
		}

		if(omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus!=VPU_DEC_OUTPUT_SUCCESS&&omx_private->frameSearchOrSkip_flag==1)
		{	
			omx_private->ConsecutiveVdecFailCnt++;		
			if(omx_private->ConsecutiveVdecFailCnt>=MAX_CONSECUTIVE_VPU_FAIL_COUNT)
			{
				LOGE("[VDEC_ERROR]m_iOutputStatus %d %dtimes!!!\n",omx_private->gsVDecOutput.m_DecOutInfo.m_iOutputStatus,omx_private->ConsecutiveVdecFailCnt);
				omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer);
				return;
			}
		}
		internalOutputFilled = 1;
		
		// To process input stream retry or next frame
		switch (decode_result) {
			case 0 :	// Display Output Success, Decode Failed Due to Buffer Full
			case 1 :	// Display Output Failed, Decode Failed Due to Buffer Full
				break;
			case 2 :	// Display Output Success, Decode Success
			case 3 :	// Display Output Failed, Decode Success		
#ifdef HAVE_ANDROID_OS 
				/* ZzaU:: Process exception to make right image because android framework only send one-frame on thumbnail-mode. 
							   We will use this condition Because of mp4/3gp formats using pv-parser.*/
	#if (USE_TCC_PARSER == 0) // SSG
				if (!omx_private->isFromTCCParser && decode_result == 3) // if data comes from the TCC parser, doesn't need this routine
				{
					if(!omx_private->isThumbnailMade && omx_private->isThumbnailMode) // SSG
					{
						omx_private->displaying_error_count--;

						if (omx_private->displaying_error_count == 0)
						{
							// we never display this frame
							// so, have to fill the thumbnail image with black
							pOutputBuffer->pBuffer = (OMX_U8 *)MakeThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, 1);
							omx_private->isThumbnailMade = OMX_TRUE; // SSG
						}
						else
						{
							return;
						}
					}
				}
	#endif
#endif		
			
				//LOGI("Consumed byte = %d, input_offset = %d, currLength = %d", omx_private->gsVDecOutput.m_DecOutInfo.m_iConsumedBytes, 
				//                                                               input_offset, omx_private->inputCurrLength);
			#ifdef DEFRAGMENT_INPUT_FRAME
				if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
					{
						nLen = 0;
					}
					else
				#endif
					{
						if ((CONTAINER_TYPE_TS == omx_private->container_type) 
						&& (omx_private->inputCurrLength > (omx_private->gsVDecOutput.m_DecOutInfo.m_iConsumedBytes + input_offset)))
						{
							nLen += omx_private->gsVDecOutput.m_DecOutInfo.m_iConsumedBytes + input_offset;
						}
						else
						{
							nLen += omx_private->inputCurrLength;
						}
					}
				DBUG_MSG("----------------------> inputCurrLength : %d, nLen : %d\n", omx_private->inputCurrLength, nLen);
				if (nLen < 0) {
					LOGE("----> A general error or simply frame not decoded?\n");
				}
				if ( nLen >= 0 && internalOutputFilled) 
				{
					omx_private->inputCurrBuffer += nLen;
					omx_private->inputCurrLength -= nLen;
					pInputBuffer->nFilledLen -= nLen;

				#if defined(_TCC9300_) || defined(_TCC8800_) // until vpu bug is fixed
					if((CONTAINER_TYPE_TS == omx_private->container_type) 
					&& (omx_private->video_coding_type == OMX_VIDEO_CodingAVC))
					{
						if(omx_private->bUseFrameDefragmentation == OMX_FALSE)
						pInputBuffer->nFilledLen = 0;
					}
				#endif
						
					//Buffer is fully consumed. Request for new Input Buffer
					if(pInputBuffer->nFilledLen == 0) 
					{
						omx_private->isNewBuffer = 1;
						DBUG_MSG("----------------------> New InputBuffer!!");
					}
				} 
				break;
			default :
				break;
		}

		// To process output stream retry or next frame
		switch (decode_result) {
			case 0 :	// Display Output Success, Decode Failed Due to Buffer Full
			case 2 :	// Display Output Success, Decode Success
				if ( nLen >= 0 && internalOutputFilled) 
				{
#ifdef HAVE_ANDROID_OS 
					/*ZzaU :: VPU output is seperated yuv420 include gap.*/
					{
		#ifdef HARDWARE_CODEC
						TCC_PLATFORM_PRIVATE_PMEM_INFO *pmemInfoPtr;
						
						pmemInfoPtr = (TCC_PLATFORM_PRIVATE_PMEM_INFO *) pOutputBuffer->pPlatformPrivate;
	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
						memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, output_len);
	#else
						memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, sizeof(int)*6);
	#endif
		#endif
						if ( omx_private->isThumbnailMode ) {
							/*ZzaU :: android framework directly make thumbnail for video player using jpeg encoder */
							pOutputBuffer->pBuffer = (OMX_U8 *)MakeThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, 0);

							//xelloss thumbnail_debugging, thumbnail frame buffer might be not send to application at one time because of procedure by isPortChange() 
							//so, isThumbnailMode be set component at end part of the component.(but .. guess that already setting at init part may be enough)
							//eclair, do not make 'isThumbnailMode' FALSE to prevent passing 'phy addr' to application
							#if 1//ZzaU:: restore this setting for eclair!!  //def _TCC8900_  use sema 'bPortChangeSem' instead of this flag
							//if(!omx_private->isFromTCCParser)
							//	omx_private->isThumbnailMode = OMX_FALSE;
							#else
							omx_private->isThumbnailMode = OMX_FALSE;					
							#endif
							
							omx_private->isThumbnailMade = OMX_TRUE; // SSG
						}
						else
						{
							pOutputBuffer->nFilledLen = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;
						}
						DBUG_MSG("----------------------> New OutputBuffer!!");
					}
#else 
					nSize = output_len;
		
					if(pOutputBuffer->nAllocLen < nSize) {
						LOGE( "Ouch!!!! Output buffer Alloc Len %ld less than Frame Size %ld\n",(int)pOutputBuffer->nAllocLen,nSize);
						return;
					}
					pOutputBuffer->nFilledLen += nSize; 
#endif
				} 
				else 
				{
					/** Few bytes may be left in the input buffer but can't generate one output frame. 
					*  Request for new Input Buffer
					*/
					if(decode_result == 2){
						omx_private->isNewBuffer = 1;
					}
					pOutputBuffer->nFilledLen = 0;		
				}
				break;
				
			case 3 :	// Display Output Failed, Decode Success
			case 1 :	// Display Output Failed, Decode Failed Due to Buffer Full
			default :
				break;
		}
#endif
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

  DBUG_MSG("   Setting parameter %i\n", nParamIndex);
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
          return OMX_ErrorBadPortIndex;
        }
        break;
      }
	case OMX_IndexParamVideoRv:
	{
		OMX_VIDEO_PARAM_RVTYPE *pVideoRv;
		pVideoRv = ComponentParameterStructure;
		portIndex = pVideoRv->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoRv, sizeof(OMX_VIDEO_PARAM_RVTYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoRv->nPortIndex == 0) {
			memcpy(&omx_private->pVideoRv, pVideoRv, sizeof(OMX_VIDEO_PARAM_RVTYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
		break;
	}
    case OMX_IndexParamVideoH263:
	{
		OMX_VIDEO_PARAM_H263TYPE *pVideoH263;
		pVideoH263 = ComponentParameterStructure;
		portIndex = pVideoH263->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoH263->nPortIndex == 0) {
			memcpy(&omx_private->pVideoH263, pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
	break;
	}
    case OMX_IndexParamVideoAvc:
	{
		OMX_VIDEO_PARAM_AVCTYPE *pVideoAvc;
		pVideoAvc = ComponentParameterStructure;
		portIndex = pVideoAvc->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		memcpy(&omx_private->pVideoAvc, pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
		break;
	}
    case OMX_IndexParamVideoMpeg4:
	{
		OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4;
		pVideoMpeg4 = ComponentParameterStructure;
		portIndex = pVideoMpeg4->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoMpeg4->nPortIndex == 0) {
			memcpy(&omx_private->pVideoMpeg4, pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
		break;
	}
    case OMX_IndexParamVideoWmv:
	{
		OMX_VIDEO_PARAM_WMVTYPE *pVideoWmv;
		pVideoWmv = ComponentParameterStructure;
		portIndex = pVideoWmv->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoWmv->nPortIndex == 0) {
			memcpy(&omx_private->pVideoWmv, pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
		break;
	}
/*
    case OMX_IndexParamVideoDivx:
	{
		OMX_VIDEO_PARAM_DIVXTYPE *pVideoWmv;
		pVideoDivx = ComponentParameterStructure;
		portIndex = pVideoWmv->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoDivx, sizeof(OMX_VIDEO_PARAM_DIVXTYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoDivx->nPortIndex == 0) {
			memcpy(&omx_private->pVideoWmv, pVideoWmv, sizeof(OMX_VIDEO_PARAM_DIVXTYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
		break;
	}
*/
    case OMX_IndexParamVideoMpeg2:
	{
		OMX_VIDEO_PARAM_MPEG2TYPE *pVideoMpeg2;
		pVideoMpeg2 = ComponentParameterStructure;
		portIndex = pVideoMpeg2->nPortIndex;
		eError = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pVideoMpeg2, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));
		if(eError!=OMX_ErrorNone) { 
			LOGE( "In %s Parameter Check Error=%x\n",__func__,eError); 
			break;
		} 
		if (pVideoMpeg2->nPortIndex == 0) {
			memcpy(&omx_private->pVideoWmv, pVideoMpeg2, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));
		} else {
			return OMX_ErrorBadPortIndex;
		}
		break;
	}
    case OMX_IndexParamStandardComponentRole:
	{
		OMX_PARAM_COMPONENTROLETYPE *pComponentRole;
		pComponentRole = ComponentParameterStructure;
		if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_RV_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingRV;
#if USE_TCC_PARSER // SSG
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_H263_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingH263;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_H264_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingAVC;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_MPEG4_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
#else
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_H263_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingH263;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_H264_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingAVC;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MPEG4_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingMPEG4;
#endif
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingWMV;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV12_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingWMV_1_2;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_DIVX_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingDIVX;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MPEG2_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingMPEG2;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_SORENSON_H263_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingFLV1;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MJPEG_ROLE)) {
			omx_private->video_coding_type = OMX_VIDEO_CodingMJPEG;
		} else {
			return OMX_ErrorBadParameter;
		}
		SetInternalVideoParameters(openmaxStandComp);
		break;
	}
	case OMX_IndexVendorVideoExtraData:
	{
		OMX_VENDOR_EXTRADATATYPE* pExtradata;
		pExtradata = (OMX_VENDOR_EXTRADATATYPE*)ComponentParameterStructure;
		//printf("%s %d Extradata[%d]  \n",__func__,__LINE__,pExtradata->nPortIndex);
		if (pExtradata->nPortIndex <= 1) {
			/** copy the extradata in the codec context private structure */
			//				memcpy(&omx_private->videoinfo, pExtradata->pData, sizeof(rm_video_info));
			//report_rm_videoinfo(&omx_private->videoinfo);
		} else {
			return OMX_ErrorBadPortIndex;
		}
	}
	break;

	case OMX_IndexParamCommonDeblocking:
	{
	break;
	}
	
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
  DBUG_MSG("   Getting parameter %i\n", nParamIndex);
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
          return OMX_ErrorBadPortIndex;
        }
        break;    
      }
    case OMX_IndexParamVideoRv:
      {
        OMX_VIDEO_PARAM_RVTYPE *pVideoRv;
        pVideoRv = ComponentParameterStructure;
        if (pVideoRv->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_RVTYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoRv, &omx_private->pVideoRv, sizeof(OMX_VIDEO_PARAM_RVTYPE));
        break;
      }
    case OMX_IndexParamVideoH263:
      {
        OMX_VIDEO_PARAM_H263TYPE *pVideoH263;
        pVideoH263 = ComponentParameterStructure;
        if (pVideoH263->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_H263TYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoH263, &omx_private->pVideoH263, sizeof(OMX_VIDEO_PARAM_H263TYPE));
        break;
      }
    case OMX_IndexParamVideoAvc:
      {
        OMX_VIDEO_PARAM_AVCTYPE * pVideoAvc; 
        pVideoAvc = ComponentParameterStructure;
        if (pVideoAvc->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_AVCTYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoAvc, &omx_private->pVideoAvc, sizeof(OMX_VIDEO_PARAM_AVCTYPE));
        break;
      }
    case OMX_IndexParamVideoMpeg4:
      {
        OMX_VIDEO_PARAM_MPEG4TYPE *pVideoMpeg4;
        pVideoMpeg4 = ComponentParameterStructure;
        if (pVideoMpeg4->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoMpeg4, &omx_private->pVideoMpeg4, sizeof(OMX_VIDEO_PARAM_MPEG4TYPE));
        break;
      }
    case OMX_IndexParamVideoWmv:
      {
        OMX_VIDEO_PARAM_WMVTYPE *pVideoWmv;
        pVideoWmv = ComponentParameterStructure;
        if (pVideoWmv->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_WMVTYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoWmv, &omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
        break;
      }
/*
    case OMX_IndexParamVideoDivx:
      {
        OMX_VIDEO_PARAM_DIVXTYPE *pVideoDivx;
        pVideoDivx = ComponentParameterStructure;
        if (pVideoDivx->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_DIVXTYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoDivx, &omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_DIVXTYPE));
        break;
      }	
*/
    case OMX_IndexParamVideoMpeg2:
      {
        OMX_VIDEO_PARAM_MPEG2TYPE *pVideoMpeg2;
        pVideoMpeg2 = ComponentParameterStructure;
        if (pVideoMpeg2->nPortIndex != 0) {
          return OMX_ErrorBadPortIndex;
        }
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE))) != OMX_ErrorNone) { 
          break;
        }
        memcpy(pVideoMpeg2, &omx_private->pVideoMpeg2, sizeof(OMX_VIDEO_PARAM_MPEG2TYPE));
        break;
      }
    case OMX_IndexParamStandardComponentRole:
      {
        OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
        pComponentRole = ComponentParameterStructure;
        if ((eError = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
          break;
        }
        if (omx_private->video_coding_type == OMX_VIDEO_CodingRV) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_RV_ROLE);
#if USE_TCC_PARSER // SSG
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_H263_ROLE);
	 	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_H264_ROLE);
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_MPEG4_ROLE);
#else
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_H263_ROLE);
	 	}else if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_H264_ROLE);
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG4_ROLE);
#endif
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingWMV) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV_ROLE);
        }else if (omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV12_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingDIVX) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_DIVX_ROLE);
		}else if (omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG2_ROLE);
	    } else if (omx_private->video_coding_type ==OMX_VIDEO_CodingFLV1 ) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_SORENSON_H263_ROLE);
		} else if (omx_private->video_coding_type ==OMX_VIDEO_CodingMJPEG) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MJPEG_ROLE);
		} else {
          strcpy((char *)pComponentRole->cRole,"\0");
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
#if USE_EXTERNAL_BUFFER
		pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc 	= OMX_TRUE;
#else
		pCap_flags->iOMXComponentSupportsExternalOutputBufferAlloc 	= OMX_FALSE;
#endif
		if (omx_private->video_coding_type == OMX_VIDEO_CodingAVC)
		{
	        pCap_flags->iOMXComponentSupportsMovableInputBuffers 	= OMX_FALSE;
			pCap_flags->iOMXComponentSupportsPartialFrames 			= OMX_FALSE;		
	        pCap_flags->iOMXComponentUsesNALStartCodes 				= OMX_TRUE;  // ENABLE NAL START CODE 0X00000001
	        pCap_flags->iOMXComponentUsesFullAVCFrames 				= OMX_TRUE;			
		}
		else
		{
		#if 1 //ZzaU:: Streamming problem!!
			pCap_flags->iOMXComponentSupportsMovableInputBuffers	= OMX_FALSE;
			pCap_flags->iOMXComponentSupportsPartialFrames			= OMX_FALSE;
		#else
	        pCap_flags->iOMXComponentSupportsMovableInputBuffers 	= OMX_TRUE;
			pCap_flags->iOMXComponentSupportsPartialFrames 			= OMX_TRUE;	//ZzaU::PartialFrames must enable to use MovableInputBuffers.
		#endif
			pCap_flags->iOMXComponentUsesNALStartCodes 				= OMX_FALSE;
			pCap_flags->iOMXComponentUsesFullAVCFrames 				= OMX_FALSE;			
		}
        pCap_flags->iOMXComponentCanHandleIncompleteFrames 			= OMX_FALSE;

		DBUG_MSG("=== PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX ===");
	}
	break;
#endif

	case OMX_IndexParamVideoProfileLevelQuerySupported:
	{
		VIDEO_PROFILE_LEVEL_TYPE* pProfileLevel = NULL;
		OMX_U32 nNumberOfProfiles = 0;
		OMX_VIDEO_PARAM_PROFILELEVELTYPE *pParamProfileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)ComponentParameterStructure;
		pParamProfileLevel->nPortIndex = omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.nPortIndex;

		/* Choose table based on compression format */
		switch(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat)
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
			return OMX_ErrorNoMore;
		}
	}
	break;

    case OMX_IndexParamCommonDeblocking:
      {
        break;
      }

    default: /*Call the base component function*/
      return omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
  }
  return OMX_ErrorNone;
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
#ifndef HAVE_ANDROID_OS	// ZzaU:: to sync call-sequence with opencore!!
      if (omx_private->avcodecReady) {
        omx_videodec_component_LibDeinit(omx_private);
        omx_private->avcodecReady = OMX_FALSE;
      }
#endif	  
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
		if(omx_private->video_coding_type == OMX_VIDEO_CodingRV) {
			strcpy((char *)cRole, VIDEO_DEC_RV_ROLE);
		}
#if USE_TCC_PARSER // SSG
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_H263_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_H264_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_MPEG4_ROLE);
		}
#else
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingH263) {
			strcpy((char *)cRole, VIDEO_DEC_H263_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingAVC) {
			strcpy((char *)cRole, VIDEO_DEC_H264_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG4) {
			strcpy((char *)cRole, VIDEO_DEC_MPEG4_ROLE);
		}
#endif
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_WMV_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_WMV12_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingDIVX) {
			strcpy((char *)cRole, VIDEO_DEC_DIVX_ROLE);
		}
		else if(omx_private->video_coding_type == OMX_VIDEO_CodingMPEG2) {
			strcpy((char *)cRole, VIDEO_DEC_MPEG2_ROLE);
		}else if (omx_private->video_coding_type ==OMX_VIDEO_CodingFLV1 ) {
			strcpy((char *)cRole, VIDEO_DEC_SORENSON_H263_ROLE);
		}else if (omx_private->video_coding_type ==OMX_VIDEO_CodingMJPEG) {
			strcpy((char *)cRole, VIDEO_DEC_MJPEG_ROLE);
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
  DBUG_MSG("   Getting configuration %i\n", nIndex);
  /* Check which structure we are being fed and fill its header */
  switch (nIndex) {
    case OMX_IndexVendorVideoExtraData :	
		pExtradata = (OMX_VENDOR_EXTRADATATYPE*)pComponentConfigStructure;
		if (pExtradata->nPortIndex <= 1) {
			/** copy the extradata in the codec context private structure */
			omx_private->extradata_size = (OMX_U32)pExtradata->nDataSize;
			if(omx_private->extradata_size > 0) 
			{
				if(omx_private->extradata) {
					TCC_free(omx_private->extradata);
				}
				omx_private->extradata = (unsigned char *)TCC_malloc((int)pExtradata->nDataSize*sizeof(char));
				//_memcpy(omx_private->extradata,(unsigned char*)(pExtradata->pData),pExtradata->nDataSize);
			}
			else 
			{
				DBUG_MSG("extradata size is 0 !!!\n");
			}
		}
		else 
		{
			return OMX_ErrorBadPortIndex;
		}

      break;

    default: // delegate to superclass
      return omx_base_component_SetConfig(hComponent, nIndex, pComponentConfigStructure);
  }
  return err;
}

OMX_ERRORTYPE omx_videodec_component_GetExtensionIndex(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_STRING cParameterName,
  OMX_OUT OMX_INDEXTYPE* pIndexType) {

	DBUG_MSG("In  %s \n",__func__);

	if(strcmp(cParameterName,"OMX.tcc.index.config.videoextradata") == 0) {
		*pIndexType = OMX_IndexVendorVideoExtraData;
	} else {
		return OMX_ErrorBadParameter;
	}
	return OMX_ErrorNone;
}

#ifdef HAVE_ANDROID_OS
OMX_ERRORTYPE OMX_ComponentInit(OMX_HANDLETYPE openmaxStandComp, OMX_STRING cCompontName)
{
	OMX_ERRORTYPE err = OMX_ErrorNone;

	err = omx_videodec_component_Constructor(openmaxStandComp,cCompontName);

	return err;
}

#endif
