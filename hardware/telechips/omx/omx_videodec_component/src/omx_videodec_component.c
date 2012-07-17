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
#include <omx_videodec_component.h>
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
#include <utils/Timers.h>
#include "tcc_video_config_data.h"

#include <mach/tccfb_ioctrl.h>

#ifdef HAVE_ANDROID_OS
#define LOG_TAG	"OMX_TCC_VIDEO_DEC"
#include <utils/Log.h>

#include <cutils/properties.h>
#include "TCCStagefrightDefine.h"
#ifdef MOVE_HW_OPERATION
#ifdef USE_WMIXER_FOR_COPY
#include <mach/tcc_wmixer_ioctrl.h>
#include <mach/vioc_global.h>
#define COPY_DEVICE         "/dev/wmixer"
#else
#include <mach/tcc_scaler_ioctrl.h>
#define COPY_DEVICE         "/dev/scaler1"
#endif
#include <fcntl.h>
#include <sys/poll.h>
#endif
#define TMEM_DEVICE			"/dev/tmem"
#define FB0_DEVICE          "/dev/graphics/fb0"

#ifdef DIVX_DRM5
#include <divx_drm5_Ex.h>
#endif

#define RESTORE_DECODE_ERR
#define CHECK_SEQHEADER_WITH_SYNCFRAME

/* Option for TS H.264 seek tuning */
#define ENABLE_DECODE_ONLY_MODE_AVC

/* Option for TS frame defragmentation */
#define DEFRAGMENT_INPUT_FRAME
#ifdef DEFRAGMENT_INPUT_FRAME
#define DEFRAGMENT_INPUT_FRAME_MPEG2
#define DEFRAGMENT_INPUT_FRAME_AVC
#define DEFRAGMENT_INPUT_FRAME_VC1
#define DEFRAGMENT_INPUT_FRAME_MPEG4
#define DEFRAGMENT_INPUT_FRAME_AVS

#define FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
#define BYTE_ORDERING(x) (((x) << 24) | (((x) >> 24) & 0xff) | (((x) << 8) & 0xff0000) | (((x) >> 8) & 0xff00))
#endif

//#define DEBUG_FRAME_DEFRAGMENTATION
#ifdef DEBUG_FRAME_DEFRAGMENTATION
	#define PEEK_DEFRAGMENTED_FRAME
#endif
#endif // DEFRAGMENT_INPUT_FRAME

#ifdef DEBUG_FRAME_DEFRAGMENTATION
	#define LOGMSG(x...)    LOGD(x)
	#define LOGERR(x...)    LOGE(x)
	#define LOGINFO(x...)   LOGI(x)
#else
	#define LOGMSG(x...)
	#define LOGERR(x...)
	#define LOGINFO(x...)
#endif


enum {
	FRAME_DEFRAGMENTATION_TYPE_NONE,
#ifdef DEFRAGMENT_INPUT_FRAME
	FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN,
#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
	FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN,
#endif
#endif
	NUM_FRAME_DEFRAGMENTATION_TYPE
};
#include <cutils/properties.h>

#define MAX_DECODE_ONLY_FRAME_NUM	(3 * VPU_BUFF_COUNT)
#define MAX_DEC_ONLY_ERR_THRESHOLD	60

//#define DEBUG_INPUT_FRAME_FILTER_BY_PTS

#define TIMESTAMP_CORRECTION

typedef struct TCC_PLATFORM_PRIVATE_PMEM_INFO
{
	unsigned int width;
	unsigned int height;
	unsigned int format;
	unsigned int offset[3];
	unsigned int optional_info[10];
	unsigned char name[6];
	unsigned int unique_addr;
	unsigned int copied; //to gralloc buffer
} TCC_PLATFORM_PRIVATE_PMEM_INFO;

static int DEBUG_ON = 0;
#define DBUG_MSG(msg...)	if (DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}
#define DPRINTF_DEC(msg...) //LOGI( ": " msg);
#define DPRINTF_DEC_STATUS(msg...) //LOGD( ": " msg);

#define HARDWARE_CODEC	1
#ifdef ANDROID_USE_GRALLOC_BUFFER
#define USE_EXTERNAL_BUFFER 1

static int GRALLOC_DEBUG_ON = 0;
#define GBUG_MSG(msg...)	if (GRALLOC_DEBUG_ON) { LOGD( ": " msg);/* sleep(1);*/}

//#define UMP_COPIED_FRAME_DUMP
//#define COMPARE_TIME_LOG
#ifdef COMPARE_TIME_LOG
#include "time.h"
static unsigned int dec_time[30] = {0,};
static unsigned int time_cnt = 0;
static unsigned int total_dec_time = 0;
static unsigned int total_frm = 0;
#endif
#else
#define USE_EXTERNAL_BUFFER 0
#endif

#define CONFIG_DATA_SIZE 	(1024*1024)
#define MIN_NAL_STARTCODE_LEN 	    3
#define MAX_NAL_STARTCODE_LEN 	    4

#define AVC_IDR_PICTURE_SEARCH_COUNT	20     // 10 EA

//static unsigned int 	Display_index[31] ={0,};
//static unsigned char 	out_index, in_index, frm_clear;
//static unsigned int 	max_fifo_cnt = VPU_BUFF_COUNT;

#define OMX_BUFF_OFFSET_UNASSIGNED	0xFFFFFFFF
#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
#define MARK_BUFF_NOT_USED	0xFFFFFFFF
#define MAX_CHECK_COUNT_FOR_CLEAR	100

//static unsigned int 	Display_Buff_ID[31] ={0,};
//static unsigned int 	buffer_unique_id;
//static unsigned int		used_fifo_count;
//static int g_hFb = -1 ;
//video display mode related with vsync
//static int iVsyncMode;
#endif
//static unsigned int 	video_dec_idx = 0;

//static OMX_BOOL gHDMIOutput = OMX_FALSE;
#endif // HAVE_ANDROID_OS

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

//static cdk_func_t* gspfVDec;
static cdk_func_t* gspfVDecList[VCODEC_MAX] =
{
	vdec_vpu //STD_AVC
	,vdec_vpu //STD_VC1
	,vdec_vpu //STD_MPEG2
	,vdec_vpu //STD_MPEG4
	,vdec_vpu //STD_H263
	,vdec_vpu //STD_DIV3
	,vdec_vpu //STD_RV
	,vdec_vpu //STD_AVS
#ifdef INCLUDE_WMV78_DEC
	,vdec_WMV78 //STD_WMV78
#else
	,0
#endif
#ifdef INCLUDE_SORENSON263_DEC
	,vdec_sorensonH263dec	//STD_SORENSON263
#else
	,vdec_vpu //STD_SH263
#endif
#ifdef JPEG_DECODE_FOR_MJPEG
	,vdec_jpeg
#else
	,vdec_vpu //STD_MJPG
#endif
	,vdec_vpu //STD_VP8
	,vdec_vpu //STD_THEORA
	,vdec_vpu //???
	,vdec_vpu //STD_MVC
};

#define RMVB_DECODER_TR_TEST

/*
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
*/
//dec_disp_info_ctrl_t dec_disp_info_ctrl;
//dec_disp_info_t dec_disp_info[32];
//dec_disp_info_input_t	dec_disp_info_input;
/*
#ifdef TIMESTAMP_CORRECTION
typedef struct pts_ctrl{
	int m_iLatestPTS;
	int m_iPTSInterval;
	int m_iRamainingDuration;
} pts_ctrl;
//pts_ctrl gsPtsInfo;
#endif
*/
//OMX_U32 gVideo_FrameRate = 30;

#ifdef PEEK_DEFRAGMENTED_FRAME
static void PrintHexDataFrontRear(OMX_U8* pBuffer, OMX_U32 nLength, OMX_U8* pTag)
{
	LOGI("[%s:%08d] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x "
	     "~ 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", pTag, nLength
			, pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3]
			, pBuffer[4], pBuffer[5], pBuffer[6]
			, pBuffer[7], pBuffer[8], pBuffer[9]
			, pBuffer[nLength - 10], pBuffer[nLength - 9], pBuffer[nLength - 8]
			, pBuffer[nLength - 7], pBuffer[nLength - 6], pBuffer[nLength - 5]
			, pBuffer[nLength - 4], pBuffer[nLength - 3], pBuffer[nLength - 2], pBuffer[nLength - 1]
		);
}

static void PrintHexData(OMX_U8* pBuffer, OMX_U32 nOffset, OMX_U8* pTag)
{
	LOGI("[%s] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", pTag
			, pBuffer[nOffset], pBuffer[nOffset + 1], pBuffer[nOffset + 2], pBuffer[nOffset + 3]
			, pBuffer[nOffset + 4], pBuffer[nOffset + 5], pBuffer[nOffset + 6]
			, pBuffer[nOffset + 7], pBuffer[nOffset + 8], pBuffer[nOffset + 9]
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
	OMX_U32  video_coding_type = omx_private->pVideoDecodInstance.video_coding_type;

	if (video_coding_type == OMX_VIDEO_CodingRV) {
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
	else if (video_coding_type == OMX_VIDEO_CodingFLV1) {
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
	else if (video_coding_type == OMX_VIDEO_CodingAVS) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/avs-video");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVS;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingAVS;
	}
	else if (video_coding_type == OMX_VIDEO_CodingH263) {
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
	else if (video_coding_type == OMX_VIDEO_CodingAVC) {
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
	else if (video_coding_type == OMX_VIDEO_CodingMPEG4) {
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
	else if (video_coding_type == OMX_VIDEO_CodingWMV) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/wmv");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV;

		setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
		omx_private->pVideoWmv.nPortIndex = 0;
		omx_private->pVideoWmv.eFormat = OMX_VIDEO_WMVFormat9;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingWMV;
	}
	else if (video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-wmv-1-2");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;

		setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_WMVTYPE));
		omx_private->pVideoWmv.nPortIndex = 0;
		omx_private->pVideoWmv.eFormat = OMX_VIDEO_WMVFormat9;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;
	}
	else if (video_coding_type == OMX_VIDEO_CodingDIVX) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/divx");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingDIVX;

		//setHeader(&omx_private->pVideoWmv, sizeof(OMX_VIDEO_PARAM_DIVXTYPE));
		//omx_private->pVideoWmv.nPortIndex = 0;
		//omx_private->pVideoWmv.eFormat = ;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingDIVX;
	}
	else if (video_coding_type == OMX_VIDEO_CodingMPEG2) {
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
	else if (video_coding_type == OMX_VIDEO_CodingMJPEG) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-jpeg");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMJPEG;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
	}
	else if (video_coding_type == OMX_VIDEO_CodingVPX) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-vnd.on2.vp8");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingVPX;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingVPX;
	}
	else if (video_coding_type == OMX_VIDEO_CodingMVC) {
		strcpy(omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.cMIMEType,"video/x-mvc");
		omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMVC;

		inPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
		inPort->sVideoParam.eCompressionFormat = OMX_VIDEO_CodingMVC;
	}

}

OMX_ERRORTYPE omx_videodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32  video_coding_type = omx_private->pVideoDecodInstance.video_coding_type;
	if(video_coding_type == OMX_VIDEO_CodingRV) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_RV_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingH263) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_H263_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingFLV1) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_SORENSON_H263_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingAVC) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_H264_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingMPEG4) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MPEG4_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingWMV) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_WMV_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_WMV12_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingDIVX) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_DIVX_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingMPEG2) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MPEG2_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingMJPEG) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MJPEG_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingAVS) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_AVS_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingVPX) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_VPX_NAME));
	}
	else if(video_coding_type == OMX_VIDEO_CodingMVC) {
		return (omx_videodec_component_Constructor(openmaxStandComp, VIDEO_DEC_MVC_NAME));
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
		omx_private->vpu_preOpen_fd = omx_private->gralloc_info.fd_copy = omx_private->mTmem_fd = -1;
		omx_private->g_hFb = -1;

		eError = omx_base_filter_Constructor(openmaxStandComp, cComponentName);

		omx_private->sPortTypesParam[OMX_PortDomainVideo].nStartPortNumber = 0;
		omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts = 2;

		/** Allocate Ports and call port constructor. */
		if (omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts && !omx_private->ports) {
			omx_private->ports = TCC_calloc(omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts, sizeof(omx_base_PortType *));
			if (!omx_private->ports) {
				eError = OMX_ErrorInsufficientResources;
				goto Error;
			}
			for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
				omx_private->ports[i] = TCC_calloc(1, sizeof(omx_base_video_PortType));
				if (!omx_private->ports[i]) {
					eError = OMX_ErrorInsufficientResources;
					goto Error;
				}
			}
		}

		base_video_port_Constructor(openmaxStandComp, &omx_private->ports[0], 0, OMX_TRUE);
		base_video_port_Constructor(openmaxStandComp, &omx_private->ports[1], 1, OMX_FALSE);

		memset(&omx_private->pVideoDecodInstance.gsVDecUserInfo, 0x00, sizeof(vdec_user_info_t));
		memset(&omx_private->pVideoDecodInstance.gsVDecInput, 0x00, sizeof (vdec_input_t));
		memset(&omx_private->pVideoDecodInstance.gsVDecOutput, 0x00, sizeof (vdec_output_t));
		memset(&omx_private->pVideoDecodInstance.gsVDecInit, 0x00, sizeof (vdec_init_t));

		omx_private->pVideoDecodInstance.pVdec_Instance = vdec_alloc_instance();
		if(omx_private->pVideoDecodInstance.pVdec_Instance == NULL)
		{
			LOGE("vdec_alloc_instance() = null");
			eError = OMX_ErrorComponentNotFound;
			goto Error;
		}
		omx_private->pVideoDecodInstance.gspfVDec = gspfVDecList[omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat];
		if (omx_private->pVideoDecodInstance.gspfVDec == 0)
		{
			eError = OMX_ErrorComponentNotFound;
			goto Error;
		}
		/** now it's time to know the video coding type of the component */
		if(!strcmp(cComponentName, VIDEO_DEC_RV_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingRV;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_RV;
		}else if(!strcmp(cComponentName, VIDEO_DEC_H263_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingH263;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_H263;
		}
		#ifdef INCLUDE_SORENSON263_DEC
		else if(!strcmp(cComponentName, VIDEO_DEC_SORENSON_H263_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingFLV1;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_SORENSON263;
		}
		#endif
		#if defined(SUPPORT_SORENSON_SPARK_H_263)
		else if(!strcmp(cComponentName, VIDEO_DEC_SORENSON_H263_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingFLV1;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_SH263;
		}
		#endif
		else if(!strcmp(cComponentName, VIDEO_DEC_H264_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingAVC;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_AVC;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MPEG4_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMPEG4;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_MPEG4;
		}else if(!strcmp(cComponentName, VIDEO_DEC_WMV_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingWMV;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_VC1;
#ifdef INCLUDE_WMV78_DEC
		}else if(!strcmp(cComponentName, VIDEO_DEC_WMV12_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingWMV_1_2;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_WMV78;
#endif
		}else if (!strcmp(cComponentName, VIDEO_DEC_DIVX_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingDIVX;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_DIV3;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MPEG2_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMPEG2;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_MPEG2;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MJPEG_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMJPEG;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_MJPG;
		}else if(!strcmp(cComponentName, VIDEO_DEC_AVS_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingAVS;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_AVS;
		}else if(!strcmp(cComponentName, VIDEO_DEC_VPX_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingVPX;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_VP8;
		}else if(!strcmp(cComponentName, VIDEO_DEC_MVC_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMVC;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat	= STD_MVC;
		}else if (!strcmp(cComponentName, VIDEO_DEC_BASE_NAME)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingUnused;
		} else {
			// IL client specified an invalid component name
			eError = OMX_ErrorInvalidComponentName;
			goto Error;
		}

		omx_private->pVideoDecodInstance.gspfVDec = gspfVDecList[omx_private->gsVDecInit.m_iBitstreamFormat];

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
		outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
		outPort->sPortParam.format.video.nFrameWidth = AVAILABLE_MAX_WIDTH;
		outPort->sPortParam.format.video.nFrameHeight = AVAILABLE_MAX_HEIGHT;
		outPort->sPortParam.nBufferSize =  (OMX_U32) (AVAILABLE_MAX_WIDTH*AVAILABLE_MAX_HEIGHT*3/2);
		outPort->sPortParam.format.video.xFramerate = 30;

#ifdef HAVE_ANDROID_OS
		{
			char isCopyPriv, value[PROPERTY_VALUE_MAX], Rplayerstream[PROPERTY_VALUE_MAX];

            omx_private->bOutputMode = omx_private->blocalPlaybackMode = OMX_FALSE;
            property_get("tcc.video.lplayback.mode", value, "");
            if( atoi(value) != 0) {
		        omx_private->blocalPlaybackMode = 1;
			}

			property_get("tcc.sys.output_mode_detected", value, "");
			if( atoi(value) != 0) {
				omx_private->bOutputMode = 1;
			}
#ifdef ENABLE_REMOTE_PLAYER
            property_get("tcc.rplayer.stream",Rplayerstream,"");
            if(!strcmp(Rplayerstream,"1"))
            {
               omx_private->isRemotePlayerPlay = OMX_TRUE;
            }
            else
            {
               omx_private->isRemotePlayerPlay = OMX_FALSE;
            }
#endif
           char TrscodingService[PROPERTY_VALUE_MAX];
           property_get("tcc.remoteplayer.control", TrscodingService, "failed");
           if(!strcmp(TrscodingService,"start") )
             omx_private->isRemotePlayerPlay = OMX_TRUE;
           else
             omx_private->isRemotePlayerPlay = OMX_FALSE;

			isCopyPriv = OMX_FALSE;
            if( !omx_private->bOutputMode ){
				isCopyPriv = OMX_FALSE; //LCD mode always copy decoded data into gralloc buffer.
            }
			else //Output display Mode
			{
				if( !omx_private->blocalPlaybackMode )
					isCopyPriv = OMX_FALSE; //HDMI mode copy decoded data into gralloc buffer if not local playback like youtube.
			}

			LOGI("The mode to copy private data: %d", isCopyPriv);
			if( isCopyPriv )
				outPort->sPortParam.nBufferCountMin = 4;
			else
				outPort->sPortParam.nBufferCountMin = 5;
		}

		outPort->sPortParam.nBufferCountActual = outPort->sPortParam.nBufferCountMin + 2;
#endif

		/** settings of output port parameter definition */
		if(isSWCodec(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat)
#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
			|| (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMJPEG)
#endif
		)
		{
			outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
			outPort->sVideoParam.eColorFormat = OMX_COLOR_FormatYUV420Planar;
		}
		else
		{
			outPort->sPortParam.format.video.eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
			outPort->sVideoParam.eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
		}

		outPort->sVideoParam.xFramerate = 30;

		if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat >= STD_AVC)
			omx_private->pVideoDecodInstance.gspfVDec = gspfVDecList[omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat];
		else
			omx_private->pVideoDecodInstance.gspfVDec = 0;

		if(omx_private->pVideoDecodInstance.gspfVDec == 0)
		{
			eError = OMX_ErrorComponentNotFound;
			goto Error;
		}

		if(!omx_private->avCodecSyncSem) {
			omx_private->avCodecSyncSem = TCC_malloc(sizeof(tsem_t));
			if(omx_private->avCodecSyncSem == NULL) {
				eError = OMX_ErrorInsufficientResources;
				goto Error;
			}
			tsem_init(omx_private->avCodecSyncSem, 0);
		}

		SetInternalVideoParameters(openmaxStandComp);

		omx_private->eOutFramePixFmt = PIX_FMT_YUV420P;

		if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingRV) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingRV;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingH263) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingH263;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingFLV1) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingFLV1;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG4;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingWMV_1_2;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingDIVX) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingDIVX;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG2) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMPEG2;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMJPEG) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMJPEG;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVS) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingAVS;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingVPX) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingVPX;
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC) {
			omx_private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.video.eCompressionFormat = OMX_VIDEO_CodingMVC;
		}

		/** general configuration irrespective of any video formats
		* setting other parameters of omx_videodec_component_private
		*/
		//  omx_private->avCodec = NULL;
		//  omx_private->avCodecContext= NULL;
		omx_private->pVideoDecodInstance.isVPUClosed = OMX_TRUE;
		omx_private->pVideoDecodInstance.avcodecReady = OMX_FALSE;
		omx_private->extradata = NULL;
		omx_private->extradata_size = 0;
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

#ifdef HAVE_ANDROID_OS
#if (!USE_EXTERNAL_BUFFER) //For reducing needless memory copy.
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
		if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
		{
			DivxDecryptExInit();
		}
#endif

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
        {
            //if(omx_private->iVsyncMode)
            {
        		//if(omx_private->g_hFb < 0)
        			omx_private->g_hFb = open(FB0_DEVICE, O_RDWR);

        		if(omx_private->g_hFb < 0)
        		{
        			LOGE("fb driver open fail");
        		}
            }
        }
#endif

		omx_private->vpu_preOpen_fd = vpu_preCtrl(0);
		LOGI("VPU PreOpened :: %d", omx_private->vpu_preOpen_fd);

#ifdef RESTORE_DECODE_ERR
		omx_private->seqHeader_backup = NULL;
		omx_private->seqHeader_len = 0;
		omx_private->cntDecError = 0;
#endif

		omx_private->pVideoDecodInstance.avcodecReady = OMX_FALSE;
		omx_private->pVideoDecodInstance.video_dec_idx = 0;
		omx_private->pVideoDecodInstance.bitrate_mbps = 20; //default 20Mbps

		memset(omx_private->pVideoDecodInstance.dec_disp_info, 0x00, sizeof(dec_disp_info_t)*32);
		memset(&omx_private->pVideoDecodInstance.dec_disp_info_input, 0x00, sizeof(dec_disp_info_input_t));

		memset(omx_private->Display_index, 0x00, sizeof(OMX_U32)*VPU_BUFF_COUNT);
		memset(omx_private->Display_Buff_ID, 0x00, sizeof(OMX_U32)*VPU_BUFF_COUNT);

		omx_private->max_fifo_cnt = VPU_BUFF_COUNT;
		omx_private->gHDMIOutput = OMX_FALSE;

		omx_private->buffer_unique_id = 0;
		omx_private->gVideo_FrameRate = 30;
		omx_private->ConsecutiveBufferFullCnt = 0;

#ifdef CHECK_SEQHEADER_WITH_SYNCFRAME
		omx_private->sequence_header_only = NULL;
		omx_private->sequence_header_size = 0;
		omx_private->need_sequence_header_attachment = OMX_FALSE;
#endif

#if defined(ANDROID_USE_GRALLOC_BUFFER) && defined(MOVE_HW_OPERATION)
#ifdef COMPARE_TIME_LOG
	 	time_cnt = total_frm = total_dec_time = 0;
#endif
		omx_private->gralloc_info.m_pDispOut[PA][0] = omx_private->gralloc_info.m_pDispOut[PA][1] = omx_private->gralloc_info.m_pDispOut[PA][2] = NULL;
		omx_private->gralloc_info.m_pDispOut[VA][0] = omx_private->gralloc_info.m_pDispOut[VA][1] = omx_private->gralloc_info.m_pDispOut[VA][2] = NULL;
#endif

		{
			omx_private->mTmem_fd = open(TMEM_DEVICE, O_RDWR);
			if (omx_private->mTmem_fd <= 0) {
				LOGE("can't open[%s] '%s'", strerror(errno), TMEM_DEVICE);
			}
		    else {
		        pmap_get_info("ump_reserved", &omx_private->mUmpReservedPmap);

		        if( ( omx_private->mTMapInfo = (void*)mmap(0, omx_private->mUmpReservedPmap.size, PROT_READ | PROT_WRITE, MAP_SHARED, omx_private->mTmem_fd, omx_private->mUmpReservedPmap.base) ) == MAP_FAILED ) {
		            LOGE("%s device's mmap failed.", TMEM_DEVICE);
		            close(omx_private->mTmem_fd);
		            omx_private->mTmem_fd = -1;
		        }
		    }
		}

		omx_private->bThumbnailMode = OMX_FALSE;
		omx_private->mCodecStart_ms = (OMX_U32)(systemTime(SYSTEM_TIME_MONOTONIC)/1000000);

Error:
		if( eError != OMX_ErrorNone )
			omx_videodec_component_Destructor(openmaxStandComp);

		return eError;
}


/** The destructor of the video decoder component
  */
OMX_ERRORTYPE omx_videodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_U32 i;

	//to make sure that VPU close.
	OMX_S32 ret;

	if(omx_private->vpu_preOpen_fd > 0)
	{
		vpu_preCtrl(omx_private->vpu_preOpen_fd);
		omx_private->vpu_preOpen_fd = -1;
	}

	if(omx_private->pVideoDecodInstance.isVPUClosed == OMX_FALSE)
	{
		if( (ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_CLOSE, NULL, NULL, &(omx_private->pVideoDecodInstance.gsVDecOutput), (omx_private->pVideoDecodInstance.pVdec_Instance))) < 0 )
		{
			LOGE( "[VDEC_CLOSE] [Err:%ld] video decoder Deinit", ret );
		}
		omx_private->pVideoDecodInstance.isVPUClosed = OMX_TRUE;
	}

	#ifdef DIVX_DRM5
	if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
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

#ifdef CHECK_SEQHEADER_WITH_SYNCFRAME
	if(omx_private->sequence_header_only) {
		TCC_free(omx_private->sequence_header_only);
		omx_private->sequence_header_only = NULL;
	}
#endif
	/* frees port/s */
	if (omx_private->ports) {
		for (i=0; i < omx_private->sPortTypesParam[OMX_PortDomainVideo].nPorts; i++) {
		if(omx_private->ports[i])
			omx_private->ports[i]->PortDestructor(omx_private->ports[i]);
		}
		TCC_free(omx_private->ports);
		omx_private->ports=NULL;
	}

	if( omx_private->pThumbnailBuff )
		TCC_free(omx_private->pThumbnailBuff);
	if( omx_private->pConfigdata )
		TCC_free(omx_private->pConfigdata);
#ifdef RESTORE_DECODE_ERR
	if(omx_private->seqHeader_backup != NULL)
		TCC_free(omx_private->seqHeader_backup);
#endif
	DBUG_MSG("Destructor of video decoder component is called\n");

	omx_base_filter_Destructor(openmaxStandComp);
	omx_private->bThumbnailMode = OMX_FALSE;

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
    {
        //if(omx_private->iVsyncMode)
        {
    		if(close( omx_private->g_hFb ) < 0)
    		{
    			LOGE("fb driver close fail");
    		}
    		omx_private->g_hFb = -1;
        }
    }
#endif

#ifdef MOVE_HW_OPERATION
	if(omx_private->gralloc_info.fd_copy != -1)
		close(omx_private->gralloc_info.fd_copy);
#endif

	{
		if(omx_private->mTmem_fd != -1){
	        munmap((void*)omx_private->mTMapInfo, omx_private->mUmpReservedPmap.size);
			if(close(omx_private->mTmem_fd) < 0)
			{
				LOGE("Error: close(omx_private->mTmem_fd)");
			}
	    	omx_private->mTmem_fd = -1;
		}
	}

	if(omx_private->pVideoDecodInstance.pVdec_Instance)
		vdec_release_instance(omx_private->pVideoDecodInstance.pVdec_Instance);

	LOGI("Video decoder component is destoried.");

	return OMX_ErrorNone;
}


/** It initializates the FFmpeg framework, and opens an FFmpeg videodecoder of type specified by IL client
  */
OMX_ERRORTYPE omx_videodec_component_LibInit(omx_videodec_component_PrivateType* omx_private) {

	OMX_U32 target_codecID;
	char value[PROPERTY_VALUE_MAX];
	OMX_U32 uiHDMIOutputMode = 0;

	tsem_up(omx_private->avCodecSyncSem);

	omx_private->pVideoDecodInstance.avcodecInited = 0;
	omx_private->pVideoDecodInstance.container_type = 0;
//	omx_private->avcodecInited = 0;
//	omx_private->pVideoDecodInstance.container_type = 0;
	omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
	omx_private->i_skip_count = TMP_SKIP_OPT_SKIP_INTERVAL;
	omx_private->i_skip_interval = TMP_SKIP_OPT_SKIP_INTERVAL;
//	omx_private->isFirst_Frame = OMX_TRUE;
	omx_private->isFirstSyncFrame = OMX_TRUE;
	omx_private->extractorType = 0;
//	omx_private->bThumbnailMode = OMX_FALSE;

	omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
  	omx_private->bUseFrameDefragmentation = OMX_FALSE;
  	omx_private->bDelayedDecodeOut = OMX_FALSE; 
	omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
	omx_private->bSetDecodeOnlyMode = OMX_FALSE;
	omx_private->bUseDecodeOnlyMode = OMX_FALSE;
	omx_private->skipFrameNum = 0;
	omx_private->decodeOnlyErrNum = 0;
	omx_private->numSkipFrame = MAX_DECODE_ONLY_FRAME_NUM;
	omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
	omx_private->IDR_frame_search_count = 0;
#ifdef DEFRAGMENT_INPUT_FRAME
	omx_private->bDetectFrameDelimiter = OMX_FALSE;
	omx_private->start_code_with_type = 0xFFFFFFFF;
	omx_private->start_code_header = 0;
	omx_private->start_code_picture = 0;
	omx_private->start_code_picture1 = 0;
	omx_private->isSplittedStartCode = OMX_FALSE;
	omx_private->splittedStartCodeLen = 0;
	omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
#endif
#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
	memset(value, 0, PROPERTY_VALUE_MAX);
	property_get("tcc.dbg.pts", value, "0");

	omx_private->frameFilterPTS = (OMX_U32)atoi(value);
#endif

	omx_private->out_index = omx_private->in_index = omx_private->frm_clear = 0;
#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
	omx_private->used_fifo_count = omx_private->buffer_unique_id = 0;
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
		omx_private->gHDMIOutput = OMX_TRUE;
	}

	omx_private->bAllPortsFlushed = OMX_FALSE;

	return OMX_ErrorNone;
}

/** It Deinitializates the ffmpeg framework, and close the ffmpeg video decoder of selected coding type
  */
void omx_videodec_component_LibDeinit(omx_videodec_component_PrivateType* omx_private)
{
	int ret;

	DBUG_MSG("omx_videodec_component_LibDeinit is called(%d)\n", omx_private->pVideoDecodInstance.isVPUClosed);

	if(omx_private->vpu_preOpen_fd > 0)
	{
		vpu_preCtrl(omx_private->vpu_preOpen_fd);
		omx_private->vpu_preOpen_fd = -1;
	}

	if(omx_private->pVideoDecodInstance.isVPUClosed == OMX_FALSE)
	{
		if( (ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_CLOSE, NULL, NULL, &(omx_private->pVideoDecodInstance.gsVDecOutput), (omx_private->pVideoDecodInstance.pVdec_Instance))) < 0 )
		{
			LOGE( "[VDEC_CLOSE] [Err:%4d] video decoder Deinit", ret );
		}
		omx_private->pVideoDecodInstance.isVPUClosed = OMX_TRUE;
	}
}

/** The Initialization function of the video decoder
  */
OMX_ERRORTYPE omx_videodec_component_Initialize(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955
    omx_private->maxConsecutiveVdecFailCnt = MAX_CONSECUTIVE_VPU_FAIL_COUNT;

	/** Temporary First Output buffer size */
	omx_private->inputCurrBuffer = NULL;
	omx_private->inputCurrLength = 0;
	omx_private->isFirstBuffer = 1;
	omx_private->isNewBuffer = 1;
	omx_private->isFirstSyncFrame = OMX_TRUE;

	omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
  	omx_private->bUseFrameDefragmentation = OMX_FALSE;
  	omx_private->bDelayedDecodeOut = OMX_FALSE; 
	omx_private->appendable_header_offset = OMX_BUFF_OFFSET_UNASSIGNED;
	omx_private->bSetDecodeOnlyMode = OMX_FALSE;
	omx_private->bUseDecodeOnlyMode = OMX_FALSE;
	omx_private->skipFrameNum = 0;
	omx_private->decodeOnlyErrNum = 0;
	omx_private->numSkipFrame = MAX_DECODE_ONLY_FRAME_NUM;
	omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
	omx_private->IDR_frame_search_count = 0;
	omx_private->bPlayDirection = OMX_TRUE; //Normal Direction.
#ifdef DEFRAGMENT_INPUT_FRAME
	omx_private->bDetectFrameDelimiter = OMX_FALSE;
	omx_private->start_code_with_type = 0xFFFFFFFF;
	omx_private->start_code_header = 0;
	omx_private->start_code_picture = 0;
	omx_private->start_code_picture1 = 0;
	omx_private->isSplittedStartCode = OMX_FALSE;
	omx_private->splittedStartCodeLen = 0;
	omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
#endif
#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
	{
		char value[PROPERTY_VALUE_MAX];
		memset(value, 0, PROPERTY_VALUE_MAX);
		property_get("tcc.dbg.pts", value, "0");

		omx_private->frameFilterPTS = (OMX_U32)atoi(value);
	}
#endif

	return eError;
}

/** The Deinitialization function of the video decoder
  */
OMX_ERRORTYPE omx_videodec_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
	OMX_ERRORTYPE eError = OMX_ErrorNone;

	DBUG_MSG("omx_videodec_component_Deinit is called(%d)\n", omx_private->pVideoDecodInstance.avcodecReady);

	if (omx_private->pVideoDecodInstance.avcodecReady){
		omx_videodec_component_LibDeinit(omx_private);
		omx_private->pVideoDecodInstance.avcodecReady = OMX_FALSE;
	}

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

#ifdef ICS_THUMBNAIL_CREATION
static OMX_U8* CreateThumbFrame(OMX_COMPONENTTYPE *openmaxStandComp, OMX_U8 *buffer, OMX_U32 *lenth, OMX_BOOL bIsBlackThumb)
#else
static OMX_U8* CreateThumbFrame(OMX_COMPONENTTYPE *openmaxStandComp, OMX_U32 *lenth, OMX_BOOL bIsBlackThumb)
#endif
{
	omx_videodec_component_PrivateType* p_omx_private = openmaxStandComp->pComponentPrivate;
	OMX_S32 frame_size;
	OMX_S32	y_size, cb_size, cr_size;
	OMX_S32	dst_width, dst_height;
	OMX_S32	y_stride, cbcr_stride;
	OMX_S32 h_off = 0, v_off = 0;
	OMX_U8 *pDstY, *pDstCb, *pDstCr;
	OMX_S32 i, j;

	if(p_omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_AVC || p_omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MVC)
	{
		y_stride = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		dst_width = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		dst_width -= p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft;
		dst_width -= p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropRight;

		dst_height = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight;
		dst_height -= p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropBottom;
		dst_height -= p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop;

		h_off = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft;
		h_off -= h_off & 1;
		v_off = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop;
		v_off -= v_off & 1;
	}
	else
	{
		y_stride = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		dst_width = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		dst_height = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight;
	}

	dst_width += dst_width & 1;
	dst_height += dst_height & 1;

	// src size setup
	y_stride = ((y_stride + 15) >> 4) << 4;
	cbcr_stride = y_stride >> 1;

	// dst size setup
	y_size = dst_width * dst_height;
	cb_size = y_size >> 2;
	cr_size = cb_size;
	frame_size = y_size + cb_size + cr_size;

#ifdef ICS_THUMBNAIL_CREATION
	pDstY = buffer;
#else
	if( p_omx_private->pThumbnailBuff )
	{
		*lenth = frame_size;
		return p_omx_private->pThumbnailBuff;
	}

	// thumbnail buffer allocation
	p_omx_private->pThumbnailBuff = TCC_calloc(1, frame_size);
	if( p_omx_private->pThumbnailBuff == 0 )
	{
		LOGE("CreateThumbFrame() - memory allocation failed");
		*lenth = 0;
		return 0;
	}

	// dst pointer setup
	pDstY = p_omx_private->pThumbnailBuff;
#endif
	pDstCb = pDstY + y_size;
	pDstCr = pDstCb + cb_size;

	if (bIsBlackThumb == OMX_TRUE)
	{
		memset(pDstY, 0x10, y_size);
		memset(pDstCb, 0x80, cb_size);
		memset(pDstCr, 0x80, cr_size);
	}
	else
	{
		if( p_omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode )
		{
			OMX_U8 *pSrcY = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][0];
			OMX_U8 *pSrcCbCr = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][1];
			OMX_U8 *pSrcCbCrTmp;
			OMX_S32 dst_width_cbcr = dst_width >> 1;
			OMX_S32 dst_height_cbcr = dst_height >> 1;

			// cropping
			pSrcY += h_off + y_stride * v_off;
			pSrcCbCr += ((h_off >> 1) << 1) + (y_stride * (v_off >> 1));

			// luminance
			for(i = 0; i < dst_height; i++)
			{
				memcpy(pDstY, pSrcY, dst_width);
				pDstY += dst_width;
				pSrcY += y_stride;
			}

		#if 1 // In case of ICS, all format is supported.
			// chrominance
			for(i = 0; i < dst_height_cbcr; i++)
			{
				pSrcCbCrTmp = pSrcCbCr;
				pSrcCbCr += y_stride;
				memcpy(pDstCb, pSrcCbCrTmp, dst_width_cbcr*2);
				pDstCb += dst_width_cbcr*2;
			}
		#else
			// chrominance
			for(i = 0; i < dst_height_cbcr; i++)
			{
				pSrcCbCrTmp = pSrcCbCr;
				pSrcCbCr += y_stride;
				for(j = 0; j < dst_width_cbcr; j++) {
					*pDstCb++ = *pSrcCbCrTmp++;
					*pDstCr++ = *pSrcCbCrTmp++;
				}
			}
		#endif
		}
		else
		{
			OMX_U8 *pSrcY = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][0];
			OMX_U8 *pSrcCb = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][1];
			OMX_U8 *pSrcCr = p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][2];
			OMX_S32 dst_width_cbcr = dst_width >> 1;

			//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
			if( p_omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG &&
				p_omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1 )
			{
				// cropping
				pSrcY += h_off + y_stride * v_off;
				h_off >>= 1;
				pSrcCb += h_off + (cbcr_stride * v_off);
				pSrcCr += h_off + (cbcr_stride * v_off);

				for(i = 0; i < dst_height; i++)
				{
					// luminance
					memcpy(pDstY, pSrcY, dst_width);
					pDstY += dst_width;
					pSrcY += y_stride;

					// chrominance (4:2:2 to 4:2:0)
					if( i & 1 ) {
						memcpy(pDstCr, pSrcCr, dst_width_cbcr);
						pDstCr += dst_width_cbcr;
					}
					else {
						memcpy(pDstCb, pSrcCb, dst_width_cbcr);
						pDstCb += dst_width_cbcr;
					}
					pSrcCb += cbcr_stride;
					pSrcCr += cbcr_stride;
				}
			}
			else
			{
				OMX_S32 dst_height_cbcr = dst_height >> 1;

				// cropping
				pSrcY += h_off + (y_stride * v_off);
				h_off >>= 1;
				v_off >>= 1;
				pSrcCb += h_off + (cbcr_stride * v_off);
				pSrcCr += h_off + (cbcr_stride * v_off);

				// luminance
				for(i = 0; i < dst_height; i++)
				{
					// luminance
					memcpy(pDstY, pSrcY, dst_width);
					pDstY += dst_width;
					pSrcY += y_stride;
				}

				// chrominance
				for(i = 0; i < dst_height_cbcr; i++)
				{
					memcpy(pDstCb, pSrcCb, dst_width_cbcr);
					pDstCb += dst_width_cbcr;
					pSrcCb += cbcr_stride;
					memcpy(pDstCr, pSrcCr, dst_width_cbcr);
					pDstCr += dst_width_cbcr;
					pSrcCr += cbcr_stride;
				}
			}
		}
	}

#if 0
	{
		FILE *fp;
		OMX_S8 path[256];
		LOGE("Thumbnail frame: %d x %d", dst_width, dst_height);
		sprintf(path, "/sdcard/tflash/thumb_%dx%d.yuv", dst_width, dst_height);
		if( fp = fopen(path, "wb") ) {
			fwrite( p_omx_private->pThumbnailBuff, 1, frame_size, fp);
			fclose(fp);
		}
	}
#endif

	*lenth = frame_size;
#ifdef ICS_THUMBNAIL_CREATION
	return NULL;
#else
	return p_omx_private->pThumbnailBuff;
#endif
}

static int isPortChange(OMX_COMPONENTTYPE *openmaxStandComp)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];
	OMX_COLOR_FORMATTYPE colorformat;
	OMX_CONFIG_RECTTYPE rectParm;
	OMX_U32 width, height;
	OMX_U32 bPortChanged, bCropChanged;

	bPortChanged = bCropChanged = OMX_FALSE;
	int ret = 0;

	if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_AVC || omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MVC)
	{
		width = (omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth- omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft - omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropRight);
		height = (omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight - omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropBottom - omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop);

		if((outPort->sPortParam.format.video.nFrameWidth != width) ||
			(outPort->sPortParam.format.video.nFrameHeight != height))
		{
			bPortChanged = OMX_TRUE;
		}

		rectParm.nLeft 		= omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropLeft;
		rectParm.nTop 		= omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iAvcPicCrop.m_iCropTop;
		rectParm.nWidth 	= width;
		rectParm.nHeight 	= height;
	}
	else
	{
		width = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
		height = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight;

		if(outPort->sPortParam.format.video.nFrameWidth != width ||
			outPort->sPortParam.format.video.nFrameHeight != height)
		{
			bPortChanged = OMX_TRUE;
		}

		rectParm.nLeft 		= 0;
		rectParm.nTop 		= 0;
		rectParm.nWidth 	= width;
		rectParm.nHeight 	= height;
	}

	if( rectParm.nLeft != omx_private->rectParm.nLeft ||
		rectParm.nTop != omx_private->rectParm.nTop ||
		rectParm.nWidth != omx_private->rectParm.nWidth ||
		rectParm.nHeight != omx_private->rectParm.nHeight)
	{
		omx_private->rectParm.nLeft		= rectParm.nLeft;
		omx_private->rectParm.nTop		= rectParm.nTop;
		omx_private->rectParm.nWidth	= rectParm.nWidth;
		omx_private->rectParm.nHeight	= rectParm.nHeight;
		bCropChanged = OMX_TRUE;
		LOGI("%dx%d :: CropInfo Changed %ld,%ld - %ldx%ld", width, height, omx_private->rectParm.nLeft, omx_private->rectParm.nTop, omx_private->rectParm.nWidth, omx_private->rectParm.nHeight);
	}

	width += width & 1;
	height += height & 1;

	if(width > AVAILABLE_MAX_WIDTH || ((width *height) > AVAILABLE_MAX_REGION))
	{
		LOGE("%ld x %ld ==> MAX-Resolution(%d x %d) over!!", width, height, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT);
		ret = -1;
	}

	if(width < AVAILABLE_MIN_WIDTH || height < AVAILABLE_MIN_HEIGHT)
	{
		LOGE("%ld x %ld ==> MIN-Resolution(%d x %d) less!!", width, height, AVAILABLE_MIN_WIDTH, AVAILABLE_MIN_HEIGHT);
		ret = -1;
	}

#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
	if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG)
	{
		if(omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode != 1)
		{
			//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
			if(omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
				colorformat = OMX_COLOR_FormatYUV422Planar;
			else
				colorformat = OMX_COLOR_FormatYUV420Planar;

			if(outPort->sPortParam.format.video.eColorFormat != colorformat)
			{
				LOGI( "Change ColorFormat!! %d -> %d", outPort->sPortParam.format.video.eColorFormat, colorformat);

				if(omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 2;
				else
					outPort->sPortParam.nBufferSize = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;

				outPort->sPortParam.format.video.eColorFormat = colorformat;
				bPortChanged = 1;
			}
		}
	}
#endif

	if(bPortChanged || bCropChanged)
	{
		if(bPortChanged)
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

		ret = 1;

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

	if(ret == 1 || omx_private->gHDMIOutput)
	{
		if(omx_private->gHDMIOutput) {
			vpu_update_sizeinfo(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps, omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT, omx_private->pVideoDecodInstance.pVdec_Instance); //max-clock!!
		}
		else{
			vpu_update_sizeinfo(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps,
								omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate, omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth,
								omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight, omx_private->pVideoDecodInstance.pVdec_Instance);
		}
	}

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

			#ifdef TIMESTAMP_CORRECTION
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG
			|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS
			|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MKV )
			{
				omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS = 0;
				omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration = 0;

				if( omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate != 0 )
				{
					omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10;
				}
				else if(omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate != 0)
				{
					omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate) >> 10;
				}
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

			memset(&omx_private->pVideoDecodInstance.gsRmff_frame_time, 0, sizeof(rmff_frame_time_t));
			omx_private->pVideoDecodInstance.gsRvReference_Flag = 1;
			omx_private->pVideoDecodInstance.gsRvP_frame_cnt = 0;

			#ifdef TIMESTAMP_CORRECTION
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG
				|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS
				|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MKV )
			{
				omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS = 0;
				omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration = 0;
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
			#ifdef TIMESTAMP_CORRECTION
			if( pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MPG )
			{
				if(pInfoInput->m_iFrameRate)
				{
					omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate = ((pInfoInput->m_iFrameRate & 0xffff) * 1000) / (((pInfoInput->m_iFrameRate >> 16) + 1)&0xffff);
					if(omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate != 0)
					{
						omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval = (((1000 * 1000) << 10) / omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate) >> 10;
					}
					else if(omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate != 0)
					{
						omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval = ((1000 << 10) / omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate) >> 10;
					}

					//LOGD("CVDEC_DISP_INFO_UPDATE m_iPTSInterval %d m_iFrameRate %d input FrameRate %x ",omx_private->gsMPEG2PtsInfo.m_iPTSInterval , omx_private->cdmx_info.m_sVideoInfo.m_iFrameRate,pInfoInput->m_iFrameRate);
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

					if(omx_private->pVideoDecodInstance.gsRvReference_Flag)
					{
						omx_private->pVideoDecodInstance.gsRvReference_Flag = 0;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_time_stamp = curTimestamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Previous_TR = rvTimestamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_time_stamp = curTimestamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_TR = rvTimestamp;
					}
					else
					{
						omx_private->pVideoDecodInstance.gsRvTRDelta = rvTimestamp - omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_TR;
						if(omx_private->pVideoDecodInstance.gsRvTRDelta < 0)
						{
							omx_private->pVideoDecodInstance.gsRvTRDelta += 8192;
						}

						if(rvFrameType == 2) //B-frame
						{
							curTimestamp = omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_time_stamp + omx_private->pVideoDecodInstance.gsRvTRDelta;
						}
						else
						{
							omx_private->pVideoDecodInstance.gsRvP_frame_cnt++;
						}
					}

					if( omx_private->pVideoDecodInstance.gsRvP_frame_cnt == 1)
					{
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P1.Current_TR = rvTimestamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P1.Current_time_stamp = curTimestamp;

						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_time_stamp = omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_time_stamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_TR = omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_TR;
					}
					else if( omx_private->pVideoDecodInstance.gsRvP_frame_cnt == 2)
					{
						omx_private->pVideoDecodInstance.gsRvP_frame_cnt = 0;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_TR = rvTimestamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P2.Current_time_stamp = curTimestamp;

						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_time_stamp = omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P1.Current_time_stamp;
						omx_private->pVideoDecodInstance.gsRmff_frame_time.ref_frame.Current_TR = omx_private->pVideoDecodInstance.gsRmff_frame_time.frame_P1.Current_TR;
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

						#ifdef TIMESTAMP_CORRECTION
						if( (pInfoCtrl->m_iFmtType  == CONTAINER_TYPE_MPG
							|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_TS
							|| pInfoCtrl->m_iFmtType == CONTAINER_TYPE_MKV)
							&& (omx_private->bPlayDirection))
						{
							if( (*pInfo)->m_iTimeStamp <= omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS )
								(*pInfo)->m_iTimeStamp = omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS + ((omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval * omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration) >> 1);
							omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS = (*pInfo)->m_iTimeStamp;
							omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration = (*pInfo)->m_iFrameDuration;
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
	unsigned int i, j;
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


#if !defined(USE_EXTERNAL_BUFFER)
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
		*(OMX_U8*)(((OMX_U8*)(*pBuffer)->pOutputPortPrivate)+8) = 0;
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
#endif

///////////////////////////////////////////////////////////////////////////////////
#define CODETYPE_NONE		(0x00000000)
#define CODETYPE_HEADER		(0x00000001)
#define CODETYPE_PICTURE	(0x00000002)
#define CODETYPE_ALL		(CODETYPE_HEADER | CODETYPE_PICTURE)

/* MPEG2 start code */
#define MPEG2_PICTURE_START     0x00000100
#define MPEG2_SEQUENCE_HEADER   0x000001B3
#define MPEG2_SEQUENCE_END      0x000001B7
#define MPEG2_GROUP_OF_PICTURES 0x000001B8

/* WVC1 start code */
#define WVC1_VIDEO_HEADER       0x0000010F
#define WVC1_VIDEO_FRAME        0x0000010D

/* MPEG4 start code */
#define MPEG4_VIDEO_HEADER      0x00000100
#define MPEG4_VIDEO_FRAME       0x000001B6

/* AVS start code */
#define AVS_VIDEO_HEADER        0x000001B0
#define AVS_VIDEO_I_FRAME       0x000001B3
#define AVS_VIDEO_PB_FRAME      0x000001B6

/* H.264/AVC NAL unit type codes for start code and type mask */
#define AVC_NAL_FORBIDDEN_ZERO_BIT_MASK      0x80
#define AVC_NAL_STARTCODE_WITH_TYPE_MASK     0xFFFFFF1F
#define AVC_NAL_STARTCODE_WITH_TYPE_SLICE    0x00000101  // P-frame
#define AVC_NAL_STARTCODE_WITH_TYPE_DPA      0x00000102
#define AVC_NAL_STARTCODE_WITH_TYPE_DPB      0x00000103
#define AVC_NAL_STARTCODE_WITH_TYPE_DPC      0x00000104
#define AVC_NAL_STARTCODE_WITH_TYPE_IDR      0x00000105  // I-frame
#define AVC_NAL_STARTCODE_WITH_TYPE_SEI      0x00000106
#define AVC_NAL_STARTCODE_WITH_TYPE_SPS      0x00000107
#define AVC_NAL_STARTCODE_WITH_TYPE_PPS      0x00000108
#define AVC_NAL_STARTCODE_WITH_TYPE_AUD      0x00000109
#define AVC_NAL_STARTCODE_WITH_TYPE_EOSEQ    0x0000010A
#define AVC_NAL_STARTCODE_WITH_TYPE_EOSTREAM 0x0000010B
#define AVC_NAL_STARTCODE_WITH_TYPE_FILL     0x0000010C

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
#define ES_STARTCODE_PREFIX_MASK             0x00FFFFFF
#define ES_STARTCODE_PREFIX_VALUE            0x00000001
#define AVC_NAL_STARTCODE_MAX_VALUE          0x0000017F

#define COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED()\
{\
	OMX_U32 ii;\
	for(ii = MIN_NAL_STARTCODE_LEN; omx_private->splittedStartCodeLen > 0; --ii, --omx_private->splittedStartCodeLen)\
	{\
		*(OMX_U8 *)(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] + omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen++) \
			= (OMX_U8)((omx_private->start_code_with_type >> (8*ii)) & 0xFF);\
	}\
}

// max. available consecutive 0 byte length : 2
#define DETERMINE_APPROPRIATE_NEXT_START_CODE_OFFSET(OFFSET)\
{\
	OMX_U32 i,j = 0;\
	for(i = 0; i < MIN_NAL_STARTCODE_LEN-1; i++)\
	{\
		if(((omx_private->start_code_with_type >> 8*i) & 0xFF) != 0)\
		{\
			break;\
		}\
		j++;\
	}\
	(OFFSET) = j;\
}
#endif

OMX_U32 SearchCodeType_Common(OMX_INOUT void* omx_private_type, OMX_OUT OMX_U32 *input_offset, OMX_IN OMX_U32 search_option)
{
	omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)omx_private_type;
	OMX_U32 temp_input_offset = *input_offset;
	OMX_U32 code_type = CODETYPE_NONE;

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
	if(omx_private->bUseFrameDefragmentation == OMX_TRUE &&
	   omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
	{
		while (temp_input_offset + MAX_NAL_STARTCODE_LEN <= omx_private->inputCurrLength)
		{
			OMX_U32 offset_count = 0;
			if(omx_private->splittedStartCodeLen)
			{
				// merge start bytes of curr. input with last bytes of prev. one.
				for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++, offset_count++)
				{
					if(omx_private->splittedStartCodeLen + offset_count >= MAX_NAL_STARTCODE_LEN)
						break;

					omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
				}

			#ifdef PEEK_DEFRAGMENTED_FRAME
				LOGINFO("Conjunction: start_code_with_type = 0x%08x, temp_input_offset(%d), offset_count(%d)",
							 omx_private->start_code_with_type, temp_input_offset, offset_count);
			#endif
			}
			else
			{
				omx_private->start_code_with_type = BYTE_ORDERING(*(OMX_U32*)(omx_private->inputCurrBuffer + temp_input_offset));
			}

			// start code without type ?
			if((omx_private->start_code_with_type & ES_STARTCODE_PREFIX_MASK) == ES_STARTCODE_PREFIX_VALUE)
			{
				OMX_S32 next_offset;
				next_offset = (offset_count > 0) ? 0 : temp_input_offset - offset_count + 1;

			#ifdef PEEK_DEFRAGMENTED_FRAME
				LOGINFO("SearchCodeType: detect startcode (0x%08x), temp_input_offset(%d), next_offset(%d), %02x %02x %02x %02x",
						omx_private->start_code_with_type, temp_input_offset, next_offset,
						omx_private->inputCurrBuffer[next_offset], omx_private->inputCurrBuffer[next_offset+1],
						omx_private->inputCurrBuffer[next_offset+2], omx_private->inputCurrBuffer[next_offset+3]);
			#endif

				COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
				temp_input_offset = next_offset;
				continue;
			}

			if (omx_private->start_code_with_type == omx_private->start_code_header)
			{
				if(search_option & CODETYPE_HEADER)
				{
					code_type = CODETYPE_HEADER;
				#ifdef PEEK_DEFRAGMENTED_FRAME
					PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "HEADER");
				#endif
					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					omx_private->start_code_with_type = 0xFFFFFFFF;
					break;
				}
			}
			else if (omx_private->start_code_with_type == omx_private->start_code_picture ||
			        (omx_private->start_code_picture1 && omx_private->start_code_with_type == omx_private->start_code_picture1))
			{
				if(search_option & CODETYPE_PICTURE)
				{
					code_type = CODETYPE_PICTURE;
				#ifdef PEEK_DEFRAGMENTED_FRAME
					PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "PICTURE");
				#endif
					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					omx_private->start_code_with_type = 0xFFFFFFFF;
					break;
				}
			}
			else
			{
				// Search how many consecutive 0 bytes exist and then pack them into start_code_with_type.
				OMX_U32 offset = 0;
				OMX_S32 next_offset;
				DETERMINE_APPROPRIATE_NEXT_START_CODE_OFFSET(offset);

				next_offset = MAX_NAL_STARTCODE_LEN - offset - offset_count;

				temp_input_offset = (offset_count > 0) ? 0 : temp_input_offset + next_offset;
				COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
				continue;
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

			// Check if rest bytes are the part of start code or not.
			if( ((omx_private->start_code_with_type & 0x000000FF) != 0x00)
			 || (((omx_private->start_code_with_type >> 8) & 0xFF) != 0x00 && (omx_private->start_code_with_type & 0xFF) != 0x10))
			{
				temp_input_offset += omx_private->splittedStartCodeLen;
				omx_private->splittedStartCodeLen = 0;
			}
		#ifdef PEEK_DEFRAGMENTED_FRAME
			else
			{
				LOGINFO("Split: start_code_with_type = 0x%08x, splittedStartCodeLen(%d)",
						omx_private->start_code_with_type, omx_private->splittedStartCodeLen);
			}
		#endif
		}
	}
	else
#endif // FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
	{
		omx_private->start_code_with_type = 0xFFFFFFFF;

		for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++)
		{
			omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];

			if ((search_option & CODETYPE_HEADER) && (omx_private->start_code_with_type == omx_private->start_code_header))
			{
				code_type = CODETYPE_HEADER;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
			#ifdef PEEK_DEFRAGMENTED_FRAME
				PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "HEADER");
			#endif
				break;
			}
			if ((search_option & CODETYPE_PICTURE) &&
			    ((omx_private->start_code_with_type == omx_private->start_code_picture) ||
				 (omx_private->start_code_picture1 && omx_private->start_code_with_type == omx_private->start_code_picture1)))
			{
				code_type = CODETYPE_PICTURE;
				temp_input_offset -= MIN_NAL_STARTCODE_LEN;
			#ifdef PEEK_DEFRAGMENTED_FRAME
				PrintHexData(omx_private->inputCurrBuffer, temp_input_offset, "PICTURE");
			#endif
				break;
			}
		}
	}

	*input_offset = temp_input_offset;

	return code_type;
}

OMX_U32 SearchCodeType_AVC(OMX_INOUT void* omx_private_type, OMX_OUT OMX_U32 *input_offset, OMX_IN OMX_U32 search_option)
{
	omx_videodec_component_PrivateType* omx_private = (omx_videodec_component_PrivateType*)omx_private_type;
	OMX_U32 temp_input_offset = *input_offset;
	OMX_U32 code_type = CODETYPE_NONE;
	omx_private->AVC_naltype_mask = 0;

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
					for (; temp_input_offset < omx_private->inputCurrLength; temp_input_offset++, offset_count++)
					{
						if(omx_private->splittedStartCodeLen + offset_count >= MAX_NAL_STARTCODE_LEN)
							break;

						omx_private->start_code_with_type = (omx_private->start_code_with_type << 8) | omx_private->inputCurrBuffer[temp_input_offset];
					}

				#ifdef PEEK_DEFRAGMENTED_FRAME
					LOGINFO("Conjunction: start_code_with_type = 0x%08x, temp_input_offset(%d), offset_count(%d)",
											omx_private->start_code_with_type, temp_input_offset, offset_count);
				#endif
				}
				else
				{
					omx_private->start_code_with_type = BYTE_ORDERING(*(OMX_U32*)(omx_private->inputCurrBuffer + temp_input_offset));
				}

				// start code without type ?
				if((omx_private->start_code_with_type & ES_STARTCODE_PREFIX_MASK) == ES_STARTCODE_PREFIX_VALUE)
				{
					OMX_S32 next_offset;
					next_offset = (offset_count > 0) ? 0 : temp_input_offset - offset_count + 1;

				#ifdef PEEK_DEFRAGMENTED_FRAME
					LOGINFO("SearchCodeType: detect startcode (0x%08x), temp_input_offset(%d), next_offset(%d), %02x %02x %02x %02x",
							omx_private->start_code_with_type, temp_input_offset, next_offset,
							omx_private->inputCurrBuffer[next_offset], omx_private->inputCurrBuffer[next_offset+1],
							omx_private->inputCurrBuffer[next_offset+2], omx_private->inputCurrBuffer[next_offset+3]);
				#endif

					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					temp_input_offset = next_offset;
					continue;
				}

				// beyond the possibility of start code with type ?
				if(omx_private->start_code_with_type > AVC_NAL_STARTCODE_MAX_VALUE || omx_private->start_code_with_type == 0)
				{
					// Search how many consecutive 0 bytes exist and then pack them into start_code_with_type.
					OMX_U32 offset = 0;
					OMX_S32 next_offset;
					DETERMINE_APPROPRIATE_NEXT_START_CODE_OFFSET(offset);

					next_offset = MAX_NAL_STARTCODE_LEN - offset - offset_count;

					temp_input_offset = (offset_count > 0) ? 0 : temp_input_offset + next_offset;
					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					continue;
				}

				omx_private->AVC_naltype_mask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

				if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_AUD)
				{
					// first time to detect frame delimiter ?
					if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
					{
						omx_private->bDetectFrameDelimiter = OMX_TRUE;
					}

					omx_private->frame_delimiter_offset = temp_input_offset;
				}
				else if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
				{
					omx_private->appendable_header_offset = temp_input_offset;
				}

				// In case of the element stream with frame delimiter, Keep searching until next delimiter is detected.
				if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
				{
					temp_input_offset += MAX_NAL_STARTCODE_LEN;
					COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
					continue;
				}

				if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SPS)
				{
					if (search_option & CODETYPE_HEADER)
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
				}
				else if ((omx_private->AVC_naltype_mask >= AVC_NAL_STARTCODE_WITH_TYPE_SLICE) &&
				         (omx_private->AVC_naltype_mask <= AVC_NAL_STARTCODE_WITH_TYPE_IDR))
				{
					if (search_option & CODETYPE_PICTURE)
					{
                        // [AVG] for multi-slice
                        if( ( omx_private->inputCurrBuffer[temp_input_offset+4] & 0x80 ) == 0 )
                        {
                            temp_input_offset += MAX_NAL_STARTCODE_LEN;
                            COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
                            continue;
                        }

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

					#ifdef ENABLE_DECODE_ONLY_MODE_AVC
						if(omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_IDR)
						{
							if(omx_private->I_frame_search_mode == AVC_NONIDR_PICTURE_SEARCH_MODE &&
							   omx_private->bSetDecodeOnlyMode == OMX_TRUE)
							{
								LOGMSG("[Seek] Change I-frame search mode : non-IDR -> IDR");
								omx_private->ConsecutiveVdecFailCnt = 0;
								omx_private->frameSearchOrSkip_flag = 1;

								omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
								omx_private->IDR_frame_search_count = 0;
								omx_private->skipFrameNum = 0;
							}
						}
						else
						{
							if((omx_private->frameSearchOrSkip_flag == 1) &&
							   (omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable == AVC_IDR_PICTURE_SEARCH_MODE))
							{
								if(++omx_private->IDR_frame_search_count >= AVC_IDR_PICTURE_SEARCH_COUNT)
								{
									LOGMSG("[Seek] Change I-frame search mode : IDR -> non-IDR");
									omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
									omx_private->skipFrameNum = 0;
								}
							}
						}
					#endif

						break;
					}
				}

				COPY_SPLITTED_BYTE_FROM_PREV_INPUT_INTO_VDEC_IF_REMAINED();
				temp_input_offset += MAX_NAL_STARTCODE_LEN;
			}

			// If any bytes remains unsearched, we should use it with next new buffer.
			if(omx_private->start_code_with_type != 0xFFFFFFFF && temp_input_offset < omx_private->inputCurrLength)
			{
				for ( omx_private->splittedStartCodeLen = 0;
					  temp_input_offset + omx_private->splittedStartCodeLen < omx_private->inputCurrLength;
					  omx_private->splittedStartCodeLen++ )
				{
					omx_private->start_code_with_type = (omx_private->start_code_with_type << 8)
													  | omx_private->inputCurrBuffer[temp_input_offset + omx_private->splittedStartCodeLen];
				}

				// Check if rest bytes are the part of start code or not.
				if( ((omx_private->start_code_with_type & 0x000000FF) != 0x00)
				 || (((omx_private->start_code_with_type >> 8) & 0xFF) != 0x00 && (omx_private->start_code_with_type & 0xFF) != 0x10))
				{
					temp_input_offset += omx_private->splittedStartCodeLen;
					omx_private->splittedStartCodeLen = 0;
				}
			#ifdef PEEK_DEFRAGMENTED_FRAME
				else
				{
					LOGINFO("Split: start_code_with_type = 0x%08x, splittedStartCodeLen(%d)",
							omx_private->start_code_with_type, omx_private->splittedStartCodeLen);
				}
			#endif
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

				omx_private->AVC_naltype_mask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

				if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_AUD)
				{
					// Is it first time to detect frame delimiter ?
					if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
					{
						omx_private->bDetectFrameDelimiter = OMX_TRUE;
					}

					omx_private->frame_delimiter_offset = temp_input_offset;
				}
				else if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
				{
					omx_private->appendable_header_offset = temp_input_offset;
				}

				if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
				{
					// Keep searching until AUD is detected.
					continue;
				}

				if (search_option & CODETYPE_HEADER && omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SPS)
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
					((omx_private->AVC_naltype_mask >= AVC_NAL_STARTCODE_WITH_TYPE_SLICE) && (omx_private->AVC_naltype_mask <= AVC_NAL_STARTCODE_WITH_TYPE_IDR)))
				{
					LOGINFO("SearchCodeType: PICTURE - temp_input_offset(%d)", temp_input_offset);

					code_type = CODETYPE_PICTURE;

					if(temp_input_offset < MIN_NAL_STARTCODE_LEN)
					{
						omx_private->isSplittedStartCode = OMX_TRUE;
					}
					else
					{
                        // [AVG] for multi-slice
                        if( ( omx_private->inputCurrBuffer[temp_input_offset+1] & 0x80 ) == 0 )
                            continue;

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

				#ifdef ENABLE_DECODE_ONLY_MODE_AVC
					if(omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_IDR)
					{
						if(omx_private->I_frame_search_mode == AVC_NONIDR_PICTURE_SEARCH_MODE &&
						   omx_private->bSetDecodeOnlyMode == OMX_TRUE)
						{
							LOGMSG("[Seek] Change I-frame search mode : non-IDR -> IDR");
							omx_private->ConsecutiveVdecFailCnt = 0;
							omx_private->frameSearchOrSkip_flag = 1;

							omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
							omx_private->IDR_frame_search_count = 0;
							omx_private->skipFrameNum = 0;
						}
					}
					else
					{
						if((omx_private->frameSearchOrSkip_flag == 1) &&
						   (omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable == AVC_IDR_PICTURE_SEARCH_MODE))
						{
							if(++omx_private->IDR_frame_search_count >= AVC_IDR_PICTURE_SEARCH_COUNT)
							{
								LOGMSG("[Seek] Change I-frame search mode : IDR -> non-IDR");
								omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
								omx_private->skipFrameNum = 0;
							}
						}
					}
				#endif
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
			omx_private->AVC_naltype_mask = omx_private->start_code_with_type & AVC_NAL_STARTCODE_WITH_TYPE_MASK;

			if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_AUD)
			{
				// Is it first time to detect frame delimiter ?
				if(omx_private->bDetectFrameDelimiter == OMX_FALSE)
				{
					omx_private->bDetectFrameDelimiter = OMX_TRUE;
				}

				omx_private->frame_delimiter_offset = temp_input_offset;
			}
			else if (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SEI)
			{
				omx_private->appendable_header_offset = temp_input_offset;
			}

			if ((omx_private->bDetectFrameDelimiter == OMX_TRUE) && (omx_private->frame_delimiter_offset == OMX_BUFF_OFFSET_UNASSIGNED))
			{
				// Keep searching until AUD is detected.
				continue;
			}

			if ((search_option & CODETYPE_HEADER) && (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SPS))
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
				((omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_SLICE) || (omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_IDR)))
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

			#ifdef ENABLE_DECODE_ONLY_MODE_AVC
				if(omx_private->AVC_naltype_mask == AVC_NAL_STARTCODE_WITH_TYPE_IDR)
				{
					if(omx_private->I_frame_search_mode == AVC_NONIDR_PICTURE_SEARCH_MODE &&
					   omx_private->bSetDecodeOnlyMode == OMX_TRUE)
					{
						LOGMSG("[Seek] Change I-frame search mode : non-IDR -> IDR");
						omx_private->ConsecutiveVdecFailCnt = 0;
						omx_private->frameSearchOrSkip_flag = 1;

						omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
						omx_private->IDR_frame_search_count = 0;
						omx_private->skipFrameNum = 0;
					}
				}
				else
				{
					if((omx_private->frameSearchOrSkip_flag == 1) &&
					   (omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable == AVC_IDR_PICTURE_SEARCH_MODE))
					{
						if(++omx_private->IDR_frame_search_count >= AVC_IDR_PICTURE_SEARCH_COUNT)
						{
							LOGMSG("[Seek] Change I-frame search mode : IDR -> non-IDR");
							omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
							omx_private->skipFrameNum = 0;
						}
					}
				}
			#endif
				break;
			}
		}
	}

	*input_offset = temp_input_offset;

	return code_type;
}
///////////////////////////////////////////////////////////////////////////////////

OMX_BOOL ExtractConfigData(omx_videodec_component_PrivateType* omx_private, OMX_U32 input_offset)
{
	OMX_U8* p = omx_private->inputCurrBuffer;
	unsigned int szInfo_video = sizeof(omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo);
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	if(omx_private->inputCurrLength >= sizeof(TCCVideoConfigData))
	{
		TCCVideoConfigData *config_data = (TCCVideoConfigData*)(p+omx_private->inputCurrLength-sizeof(TCCVideoConfigData));

		if (strncmp(config_data->id, TCC_VIDEO_CONFIG_ID, 9) == 0)
		{
			if(omx_private->extractorType == 0)
			{
				omx_private->extractorType = OMX_BUFFERFLAG_EXTRACTORTYPE_TCC;
			}
			omx_private->pVideoDecodInstance.container_type = config_data->iContainerType;
			omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps = config_data->iBitRate;
			omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate = config_data->iFrameRate;

			memset(&(omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo), 0x00, szInfo_video);
			//sync with parser!!
			if (omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_RV || omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_DIV3 ||
					omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MPEG2 || omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_VC1
#ifdef INCLUDE_WMV78_DEC
					|| omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_WMV78
#endif
#ifdef INCLUDE_SORENSON263_DEC
					|| omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_SORENSON263
#endif
			   )
			{
				memcpy(&(omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo), (char*)(p+omx_private->inputCurrLength-sizeof(TCCVideoConfigData)-szInfo_video), szInfo_video);
#ifdef INCLUDE_WMV78_DEC
				omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC = omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFourCC;
#endif
			}

			if(omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iExtraDataLen
					&& omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV_1_2)
			{
				DBUG_MSG("ExtraData = %d", omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iExtraDataLen);
				omx_private->extradata_size = omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iExtraDataLen;
				omx_private->extradata = TCC_calloc(1, omx_private->extradata_size);
				memcpy(omx_private->extradata, omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_pExtraData, omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iExtraDataLen);
			}
			else
			{
				omx_private->extradata_size = 0;
				omx_private->extradata = NULL;
			}

			LOGI("Resolution = %d x %d - %d Mbps - %d fps, Container Type = %d, FourCC = 0x%08x[%c%c%c%c]", outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
					omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps, omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate/1000, omx_private->pVideoDecodInstance.container_type, omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC,
					(char)(omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC>>0), (char)(omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC>>8), (char)(omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC>>16),(char)(omx_private->pVideoDecodInstance.gsVDecInit.m_iFourCC>>24));
		}
		else
		{
			omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps = 0;
			omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate = 0;
		}
	}
	else
	{
		if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC)
		{
			LOGE("ExtractConfigData : Error - TCCVideoConfigData is truncated");
			return OMX_FALSE;
		}

		omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps = 0;
		omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate = 0;
	}

	/* Determine the use of frame defragmentation for VPU. */
	if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC)
	{
		if ((CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
		/*|| (CONTAINER_TYPE_MPG == omx_private->pVideoDecodInstance.container_type)*/)
		{
		#if defined (DEFRAGMENT_INPUT_FRAME) || defined (ENABLE_DECODE_ONLY_MODE_AVC)
			char value[PROPERTY_VALUE_MAX];
			memset(value, 0, PROPERTY_VALUE_MAX);
		#endif
		#ifdef DEFRAGMENT_INPUT_FRAME
			property_get("tcc.video.defragment", value, "2");

			omx_private->frameDefragmentationType = (OMX_U32)atoi(value);
		#endif
			if(omx_private->frameDefragmentationType != FRAME_DEFRAGMENTATION_TYPE_NONE)
			{
				omx_private->bUseFrameDefragmentation = OMX_TRUE;
			}

			if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG2)
			{
			#ifndef DEFRAGMENT_INPUT_FRAME_MPEG2
				omx_private->bUseFrameDefragmentation = OMX_FALSE;
			#endif
				omx_private->SearchCodeType = SearchCodeType_Common;
				omx_private->start_code_header = MPEG2_SEQUENCE_HEADER;
				omx_private->start_code_picture = MPEG2_PICTURE_START;

			#ifdef DEFRAGMENT_INPUT_FRAME_MPEG2
				// Prevent 4byte leval byte scanning mpeg2 frame defragmentation.....
				omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN;
			#endif
			}
			else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC)
			{
			#ifndef DEFRAGMENT_INPUT_FRAME_AVC
				omx_private->bUseFrameDefragmentation = OMX_FALSE;
				omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
			#endif
				omx_private->SearchCodeType = SearchCodeType_AVC;

			#ifdef ENABLE_DECODE_ONLY_MODE_AVC
				if(CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
				{
					memset(value, 0, PROPERTY_VALUE_MAX);
					property_get("tcc.avc.seektype", value, "0");

					OMX_U32 seektype = (OMX_U32)atoi(value);
					if(seektype)
					{
						omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
						omx_private->maxConsecutiveVdecFailCnt = MAX_CONSECUTIVE_VPU_FAIL_COUNT_FOR_IDR;
					}
					else
					{
						memset(value, 0, PROPERTY_VALUE_MAX);
						property_get("tcc.video.skip.disp", value, "1");
						omx_private->bUseDecodeOnlyMode = (OMX_U32)atoi(value) == 0 ? OMX_FALSE : OMX_TRUE;

						if(omx_private->bUseDecodeOnlyMode)
						{
							memset(value, 0, PROPERTY_VALUE_MAX);
							property_get("tcc.video.skip.disp.num", value, "0");
							OMX_U32 numSkipFrame = (OMX_U32)atoi(value);
							if(numSkipFrame > omx_private->max_fifo_cnt)
							{
								omx_private->numSkipFrame = numSkipFrame;
							}
						}
					}
				}
			#endif
			}
			else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV)
			{
			#ifndef DEFRAGMENT_INPUT_FRAME_VC1
				omx_private->bUseFrameDefragmentation = OMX_FALSE;
				omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_NONE;
			#endif
				omx_private->SearchCodeType = SearchCodeType_Common;
				omx_private->start_code_header = WVC1_VIDEO_HEADER;
				omx_private->start_code_picture = WVC1_VIDEO_FRAME;
			}
			else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4 || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingDIVX)
			{
			#ifndef DEFRAGMENT_INPUT_FRAME_MPEG4
				omx_private->bUseFrameDefragmentation = OMX_FALSE;
			#endif
				omx_private->SearchCodeType = SearchCodeType_Common;
				omx_private->start_code_header = MPEG4_VIDEO_HEADER;
				omx_private->start_code_picture = MPEG4_VIDEO_FRAME;

			#ifdef DEFRAGMENT_INPUT_FRAME_MPEG4
				// Prevent 4byte leval byte scanning mpeg2 frame defragmentation.....
				omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN;
			#endif
			}
			else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVS)
			{
			#ifndef DEFRAGMENT_INPUT_FRAME_AVS
				omx_private->bUseFrameDefragmentation = OMX_FALSE;
			#endif
				omx_private->SearchCodeType = SearchCodeType_Common;
				omx_private->start_code_header = AVS_VIDEO_HEADER;
				omx_private->start_code_picture = AVS_VIDEO_I_FRAME;
				omx_private->start_code_picture1 = AVS_VIDEO_PB_FRAME;

			#ifdef DEFRAGMENT_INPUT_FRAME_AVS
				// Prevent 4byte leval byte scanning mpeg2 frame defragmentation.....
				omx_private->frameDefragmentationType = FRAME_DEFRAGMENTATION_TYPE_1BYTE_SCAN;
			#endif
			}

			LOGI("ExtractConfigData : bUseFrameDefragmentation(%d), frameDefragmentationType(%ld)",
					 omx_private->bUseFrameDefragmentation, omx_private->frameDefragmentationType);
		}
	}
	else
	{
		if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC)
		{
			const OMX_U8 NAL_UNIT_SPS = 0x07;
			const OMX_U8 NAL_UNIT_PPS = 0x08;

			OMX_U8* p = omx_private->inputCurrBuffer;
			OMX_U8 cNalUnitType = p[MIN_NAL_STARTCODE_LEN + 1] & 0x1F;

			if(cNalUnitType == NAL_UNIT_SPS || cNalUnitType == NAL_UNIT_PPS)
			{
				OMX_U32 uiConfigDataSize = omx_private->inputCurrLength - input_offset;
				memcpy(&omx_private->pConfigdata[omx_private->szConfigdata], p + input_offset, uiConfigDataSize);
				omx_private->szConfigdata += uiConfigDataSize;
			}

		#ifdef ENABLE_DECODE_ONLY_MODE_AVC
			if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_MPEG2TS
				|| omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_RTSP)
			{
				char value[PROPERTY_VALUE_MAX];
				memset(value, 0, PROPERTY_VALUE_MAX);
				property_get("tcc.avc.seektype", value, "0");

				OMX_U32 seektype = (OMX_U32)atoi(value);
				if(seektype)
				{
					omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
					omx_private->maxConsecutiveVdecFailCnt = MAX_CONSECUTIVE_VPU_FAIL_COUNT_FOR_IDR;
				}
				else
				{
					memset(value, 0, PROPERTY_VALUE_MAX);
					property_get("tcc.video.skip.disp", value, "1");
					omx_private->bUseDecodeOnlyMode = (OMX_U32)atoi(value) == 0 ? OMX_FALSE : OMX_TRUE;

					if(omx_private->bUseDecodeOnlyMode)
					{
						memset(value, 0, PROPERTY_VALUE_MAX);
						property_get("tcc.video.skip.disp.num", value, "0");
						OMX_U32 numSkipFrame = (OMX_U32)atoi(value);
						if(numSkipFrame > omx_private->max_fifo_cnt)
						{
							omx_private->numSkipFrame = numSkipFrame;
						}
					}
				}
			}
		#endif
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4)
		{
			OMX_U32 uiConfigDataSize = omx_private->inputCurrLength - input_offset;
			memcpy(&omx_private->pConfigdata[omx_private->szConfigdata], omx_private->inputCurrBuffer+input_offset, uiConfigDataSize);
			omx_private->szConfigdata += uiConfigDataSize;
		}
	}

#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
	memset(value, 0, PROPERTY_VALUE_MAX);
	property_get("tcc.dbg.pts", value, "0");

	omx_private->frameFilterPTS = (OMX_U32)atoi(value);
#endif

	return OMX_TRUE;
}

static void VideoDecErrorProcess(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer, OMX_BUFFERHEADERTYPE* pOutputBuffer, int ret)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;

    if(omx_private->cntDecError > MAX_CONSECUTIVE_VPU_FAIL_TO_RESTORE_COUNT)
    {
		LOGE("Consecutive decode-cmd failure is occurred");
    }

	if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_RTSP) {
		omx_private->cntDecError = 0;
	}

#ifdef RESTORE_DECODE_ERR
	if((ret == -RETCODE_CODEC_EXIT || ret == -RETCODE_MULTI_CODEC_EXIT_TIMEOUT) && omx_private->cntDecError <= MAX_CONSECUTIVE_VPU_FAIL_TO_RESTORE_COUNT)
	{
		if(omx_private->pVideoDecodInstance.avcodecInited && omx_private->seqHeader_backup == NULL)
			goto Error_Proc;

		if(!omx_private->pVideoDecodInstance.avcodecInited && omx_private->sequence_header_only != NULL)
			omx_private->need_sequence_header_attachment = OMX_TRUE;

		omx_private->isNewBuffer = 1;
		omx_private->cntDecError++;
		omx_private->pVideoDecodInstance.avcodecInited = 0;
		omx_private->ConsecutiveBufferFullCnt = 0;

	#ifdef DEFRAGMENT_INPUT_FRAME
		omx_private->bDetectFrameDelimiter = OMX_FALSE;
		omx_private->start_code_with_type = 0xFFFFFFFF;
		omx_private->start_code_header = 0;
		omx_private->start_code_picture = 0;
		omx_private->isSplittedStartCode = OMX_FALSE;
		omx_private->splittedStartCodeLen = 0;
		omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED;
	#endif

		omx_private->seq_header_init_error_count = SEQ_HEADER_INIT_ERROR_COUNT;
		if(omx_private->pVideoDecodInstance.isVPUClosed != OMX_TRUE)
		{
			omx_private->pVideoDecodInstance.gspfVDec( VDEC_CLOSE, NULL, NULL, &(omx_private->pVideoDecodInstance.gsVDecOutput), omx_private->pVideoDecodInstance.pVdec_Instance);
			omx_private->pVideoDecodInstance.isVPUClosed = OMX_TRUE;
		}

		omx_private->in_index = omx_private->out_index = omx_private->frm_clear = 0;
		LOGE("try to restore decode error");

		goto Success_Proc;
	}
#endif

Error_Proc:
	{
		if(omx_private->pVideoDecodInstance.isVPUClosed != OMX_TRUE)
		{
			/* close VPU */
			omx_private->pVideoDecodInstance.gspfVDec( VDEC_CLOSE, NULL, NULL, &(omx_private->pVideoDecodInstance.gsVDecOutput), omx_private->pVideoDecodInstance.pVdec_Instance);
			omx_private->pVideoDecodInstance.isVPUClosed = OMX_TRUE;

			/* Report error event */
			(*(omx_private->callbacks->EventHandler))(openmaxStandComp, omx_private->callbackData,
					OMX_EventError, OMX_ErrorHardware,
					0, NULL);
		}
	}

Success_Proc:

	pInputBuffer->nFilledLen = pOutputBuffer->nFilledLen = 0;

#if defined(ANDROID_USE_GRALLOC_BUFFER) && defined(MOVE_HW_OPERATION)
	omx_private->gralloc_info.m_pDispOut[PA][0] = omx_private->gralloc_info.m_pDispOut[PA][1] = omx_private->gralloc_info.m_pDispOut[PA][2] = NULL;
	omx_private->gralloc_info.m_pDispOut[VA][0] = omx_private->gralloc_info.m_pDispOut[VA][1] = omx_private->gralloc_info.m_pDispOut[VA][2] = NULL;
#endif

	return;
}

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
static int clear_vpu_buffer(int buffer_id, omx_videodec_component_PrivateType* omx_private)
{
	int ret =0;
	int cleared_buff_count = 0;
	int loopCount = 0;

	while(buffer_id < omx_private->Display_Buff_ID[omx_private->out_index] && loopCount < MAX_CHECK_COUNT_FOR_CLEAR)
	{
		usleep(10000);
		buffer_id = ioctl(omx_private->g_hFb, TCC_LCDC_VIDEO_GET_DISPLAYED, NULL) ;
		loopCount++;
		if(buffer_id < 0)
			break;
	}

	while(buffer_id >= omx_private->Display_Buff_ID[omx_private->out_index] && omx_private->used_fifo_count>0)
	{
		ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL,  (omx_private->pVideoDecodInstance.pVdec_Instance));

		if(ret  < 0 )
		{
			LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
			return ret;
		}
		///wz// LOGE("### FLAG CLEAR idx(%d), displayed(%d)", omx_private->out_index, tmp ) ;
		omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
		omx_private->used_fifo_count--;
		cleared_buff_count++;
	}

	if(cleared_buff_count == 0)
	{
		ioctl(omx_private->g_hFb, TCC_LCDC_VIDEO_CLEAR_FRAME, omx_private->Display_Buff_ID[omx_private->out_index]);
		LOGW("Video Buffer Clear Sync Fail : %d %d , loopcount(%d)\n", buffer_id, omx_private->Display_Buff_ID[omx_private->out_index], loopCount);
		ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL, (omx_private->pVideoDecodInstance.pVdec_Instance) );

		if(ret  < 0 )
		{
			LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
			return ret;
		}
		///wz// LOGE("### FLAG CLEAR idx(%d), displayed(%d)", omx_private->out_index, tmp ) ;
		omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
		omx_private->used_fifo_count--;
	}

	return 0;
}
#endif

#ifdef MOVE_HW_OPERATION
int move_data_using_scaler(OMX_COMPONENTTYPE *openmaxStandComp, unsigned int width, unsigned int height, unsigned char *YSrc, unsigned char *USrc, unsigned char *VSrc,
							char bSrcYUVInter, OMX_U8 *addrDst, COPY_OPER_MODE cmd)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
#ifdef USE_WMIXER_FOR_COPY
	WMIXER_INFO_TYPE WmixerInfo;
#else
	SCALER_TYPE ScaleInfo;
#endif
	unsigned int stride, stride_c;
	unsigned int framesize_Y, framesize_C;
	struct pollfd poll_event[1];
	int ret_val = 0;

	stride = ALIGNED_BUFF(width, 16);
    framesize_Y = ALIGNED_BUFF(stride * height, 64);

	stride_c = ALIGNED_BUFF(stride/2, 16);
    framesize_C = ALIGNED_BUFF(stride_c * height/2, 64);

	if(cmd & SEND_CMD)
	{
		if( omx_private->gralloc_info.fd_copy < 0 )
		{
			omx_private->gralloc_info.fd_copy = open(COPY_DEVICE, O_RDWR);
			if (omx_private->gralloc_info.fd_copy <= 0) {
				LOGE("can't open[%s] %s", strerror(errno), COPY_DEVICE);
				return -1;
			}
		}

#ifdef USE_WMIXER_FOR_COPY

		memset(&WmixerInfo, 0x00, sizeof(WmixerInfo));
		WmixerInfo.rsp_type			= WMIXER_INTERRUPT;

		WmixerInfo.src_y_addr		= (unsigned int)YSrc;
		WmixerInfo.src_u_addr		= (unsigned int)USrc;
		WmixerInfo.src_v_addr		= (unsigned int)VSrc;

		if(bSrcYUVInter)
			WmixerInfo.src_fmt		= VIOC_IMG_FMT_YUV420IL0;
		else
		{
#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
			if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG)
			{
				//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
				if(omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
					WmixerInfo.src_fmt		= VIOC_IMG_FMT_YUV422SEP;
				else
					WmixerInfo.src_fmt		= VIOC_IMG_FMT_YUV420SEP;
			}
			else
#endif
				WmixerInfo.src_fmt		= VIOC_IMG_FMT_YUV420SEP;
		}

		WmixerInfo.dst_y_addr		= (unsigned int)addrDst;

		if(omx_private->blocalPlaybackMode)
		{
			WmixerInfo.dst_fmt 			= VIOC_IMG_FMT_YUV420IL0;
			WmixerInfo.dst_u_addr		= (unsigned int)((char*)WmixerInfo.dst_y_addr + framesize_Y);
			WmixerInfo.dst_v_addr		= (unsigned int)((char*)WmixerInfo.dst_u_addr + framesize_C);
		}
		else
		{
			WmixerInfo.dst_fmt 			= VIOC_IMG_FMT_YUV420SEP;
			WmixerInfo.dst_v_addr		= (unsigned int)((char*)WmixerInfo.dst_y_addr + framesize_Y);
			WmixerInfo.dst_u_addr		= (unsigned int)((char*)WmixerInfo.dst_v_addr + framesize_C);
		}

		WmixerInfo.img_width 		= stride;
		WmixerInfo.img_height		= height;

		GBUG_MSG("%s copy :: 0x%x-0x%x-0x%x -> 0x%x(0x%x-0x%x-0x%x)", COPY_DEVICE, WmixerInfo.src_y_addr, WmixerInfo.src_u_addr, WmixerInfo.src_v_addr, addrDst, WmixerInfo.dst_y_addr, WmixerInfo.dst_u_addr, WmixerInfo.dst_v_addr);

		if (ioctl(omx_private->gralloc_info.fd_copy, TCC_WMIXER_IOCTRL, &WmixerInfo) < 0)
		{
			LOGE("%s Out Error!", COPY_DEVICE);
			return -1;
		}
#else

		memset(&ScaleInfo, 0x00, sizeof(ScaleInfo));
		ScaleInfo.src_Yaddr			= (char*)YSrc;
		ScaleInfo.src_Uaddr			= (char*)USrc;
		ScaleInfo.src_Vaddr			= (char*)VSrc;
		ScaleInfo.responsetype 		= SCALER_INTERRUPT;
		if(bSrcYUVInter)
			ScaleInfo.src_fmt		= SCALER_YUV420_inter;
		else
		{
#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
			if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG)
			{
				//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
				if(omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
					ScaleInfo.src_fmt		= SCALER_YUV422_sp;
				else
					ScaleInfo.src_fmt		= SCALER_YUV420_sp;
			}
			else
#endif
				ScaleInfo.src_fmt		= SCALER_YUV420_sp;
		}
		ScaleInfo.src_ImgWidth 		= stride;
		ScaleInfo.src_ImgHeight		= height;
		ScaleInfo.src_winLeft		= 0;
		ScaleInfo.src_winTop		= 0;
		ScaleInfo.src_winRight 		= ScaleInfo.src_winLeft + ScaleInfo.src_ImgWidth;
		ScaleInfo.src_winBottom		= ScaleInfo.src_winTop + ScaleInfo.src_ImgHeight;

		ScaleInfo.dest_Yaddr		= (char*)addrDst;

		if(omx_private->blocalPlaybackMode)
		{
			ScaleInfo.dest_fmt 			= SCALER_YUV420_inter;
			ScaleInfo.dest_Uaddr		= (char*)ScaleInfo.dest_Yaddr + framesize_Y;
			ScaleInfo.dest_Vaddr		= (char*)ScaleInfo.dest_Uaddr + framesize_C;
		}
		else
		{
			ScaleInfo.dest_fmt 			= SCALER_YUV420_sp;
			ScaleInfo.dest_Vaddr		= (char*)ScaleInfo.dest_Yaddr + framesize_Y;
			ScaleInfo.dest_Uaddr		= (char*)ScaleInfo.dest_Vaddr + framesize_C;
		}
		GBUG_MSG("%s copy :: 0x%x-0x%x-0x%x -> 0x%x(0x%x-0x%x-0x%x)", COPY_DEVICE, ScaleInfo.src_Yaddr, ScaleInfo.src_Uaddr, ScaleInfo.src_Vaddr, addrDst, ScaleInfo.dest_Yaddr, ScaleInfo.dest_Uaddr, ScaleInfo.dest_Vaddr);

		ScaleInfo.dest_ImgWidth		= stride;
		ScaleInfo.dest_ImgHeight	= height;
		ScaleInfo.dest_winLeft 		= 0;
		ScaleInfo.dest_winTop		= 0;
		ScaleInfo.dest_winRight		= ScaleInfo.dest_winLeft + ScaleInfo.dest_ImgWidth;
		ScaleInfo.dest_winBottom	= ScaleInfo.dest_winTop + ScaleInfo.dest_ImgHeight;

		if (ioctl(omx_private->gralloc_info.fd_copy, TCC_SCALER_IOCTRL, &ScaleInfo) < 0)
		{
			LOGE("%s Out Error!", COPY_DEVICE);
			return -1;
		}
#endif
	}

	if(cmd & WAIT_RESPOND)
	{
		int ret;
		memset(poll_event, 0, sizeof(poll_event));
		poll_event[0].fd = omx_private->gralloc_info.fd_copy;
		poll_event[0].events = POLLIN;
		ret = poll((struct pollfd*)poll_event, 1, 100);

		if (ret < 0) {
			LOGE("%s poll error", COPY_DEVICE);
			ret_val = -1;
		}else if (ret == 0) {
			LOGE("%s poll timeout", COPY_DEVICE);
			ret_val = -1;
		}else if (ret > 0) {
			if (poll_event[0].revents & POLLERR) {
				LOGE("%s poll POLLERR", COPY_DEVICE);
				ret_val = -1;
			}
		}
	}

#ifdef UMP_COPIED_FRAME_DUMP //for frame dump.
	if(total_frm == 300){
		FILE *pFs;
		unsigned int Y, U, V;

#ifdef USE_WMIXER_FOR_COPY
		Y = ((unsigned int)omx_private->mTMapInfo + (WmixerInfo.dst_y_addr - omx_private->mUmpReservedPmap.base));
		U = ((unsigned int)omx_private->mTMapInfo + (WmixerInfo.dst_u_addr - omx_private->mUmpReservedPmap.base));
		V = ((unsigned int)omx_private->mTMapInfo + (WmixerInfo.dst_v_addr - omx_private->mUmpReservedPmap.base));
#else
		Y = ((unsigned int)omx_private->mTMapInfo + (ScaleInfo.dest_Yaddr - omx_private->mUmpReservedPmap.base));
		U = ((unsigned int)omx_private->mTMapInfo + (ScaleInfo.dest_Uaddr - omx_private->mUmpReservedPmap.base));
		V = ((unsigned int)omx_private->mTMapInfo + (ScaleInfo.dest_Vaddr - omx_private->mUmpReservedPmap.base));
#endif

		pFs = fopen("/sdcard/frame.yuv", "ab+");
		if (!pFs) {
			LOGE("Cannot open '/sdcard/frame.yuv'");
			return 0;
		}
		else
		{
			if(pFs){
				fwrite( Y, stride*height, 1, pFs);
				fwrite( U, stride*height/4, 1, pFs);
				fwrite( V, stride*height/4, 1, pFs);
			}
			fclose(pFs);
		}
	}
#endif

	if(ret_val < 0)
	{
		if(omx_private->gralloc_info.fd_copy != -1){
			close(omx_private->gralloc_info.fd_copy);
			omx_private->gralloc_info.fd_copy = -1;
		}
	}

	return ret_val;
}
#endif


void* get_private_addr(OMX_COMPONENTTYPE *openmaxStandComp, int fd_val, int width, int height, OMX_BOOL *bCopyToGrallocBuffer)
{
	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
    int stride, stride_c, frame_len = 0;

	stride = (width + 15) & ~15;
    stride_c = (stride/2 + 15) & ~15;
	frame_len = height * (stride + stride_c);// + 100;

    if( omx_private->mTmem_fd <= 0 ) {
        LOGE("%s device is not opened.", TMEM_DEVICE);
		*bCopyToGrallocBuffer = OMX_TRUE;
        return NULL;
    }

    if( !omx_private->bOutputMode ){
		*bCopyToGrallocBuffer = OMX_TRUE; //LCD mode always copy decoded data into gralloc buffer.
		//return NULL;
    }
	else //Output display Mode
	{
		if( omx_private->blocalPlaybackMode )
			*bCopyToGrallocBuffer = OMX_FALSE;
		else
			*bCopyToGrallocBuffer = OMX_TRUE;
	}

    return (void*)((unsigned int)omx_private->mTMapInfo + (fd_val - omx_private->mUmpReservedPmap.base) + frame_len);
}

#ifdef CHECK_SEQHEADER_WITH_SYNCFRAME
static int extract_seqheader(
		const unsigned char	*pbyStreamData,
		long				lStreamDataSize,
		unsigned char		**ppbySeqHeaderData,
		long				*plSeqHeaderSize,
		int					codec_type
		)
{
	long i;

	if( codec_type == OMX_VIDEO_CodingAVC || codec_type == OMX_VIDEO_CodingMVC)
	{
		long l_seq_start_pos = 0, l_seq_end_pos = 0, l_seq_length = 0; // Start Position, End Position, Length of the sequence header
		long l_sps_found = 0;
		long l_pps_found = 0;

		unsigned long ul_read_word_buff;	   	    	            //4 byte temporary buffer
		unsigned long ul_masking_word_seq          = 0x0FFFFFFF;    //Masking Value for finding H.264 sequence header
		unsigned long ul_masking_word_sync         = 0x00FFFFFF;    //Masking Value for finding sync word of H.264
		unsigned long ul_h264_result_word_seq_SPS  = 0x07010000;    //Masking result should be this value in case of SPS. SPS Sequence header of H.264 must start by "00 00 01 x7"
		unsigned long ul_h264_result_word_seq_PPS  = 0x08010000;    //Masking result should be this value in case of PPS. PPS Sequence header of H.264 must start by "00 00 01 x8"
		unsigned long ul_h264_result_word_sync     = 0x00010000;    //Masking result should be this value. Sequence header of H.264 must start by "00 00 01 x7"

		if ( lStreamDataSize < 4 )
			return 0; // there's no Seq. header in this frame. we need the next frame.

		if ( *plSeqHeaderSize > 0 )
		{
			// we already find the sps, pps in previous frame
			l_sps_found = 1;
			l_pps_found = 1;
			l_seq_start_pos = 0;
		}
		else
		{
			// find the SPS of H.264
			ul_read_word_buff = 0;
			ul_read_word_buff |= (pbyStreamData[0] << 8);
			ul_read_word_buff |= (pbyStreamData[1] << 16);
			ul_read_word_buff |= (pbyStreamData[2] << 24);

			for ( i = 0; i < lStreamDataSize-4; i++ )
			{
				ul_read_word_buff = ul_read_word_buff >> 8;
				ul_read_word_buff &= 0x00FFFFFF;
				ul_read_word_buff |= (pbyStreamData[i+3] << 24);

				if ( (ul_read_word_buff & ul_masking_word_seq) == ul_h264_result_word_seq_SPS )
				{
					// SPS Sequence Header has been detected
					l_sps_found = 1;
					l_seq_start_pos = i;          // save the start position of the sequence header

					break;
				}

				// Continue to find the sps in next loop
			}

			if ( l_sps_found == 1 )
			{
				// Now, let's start to find the PPS of the Seq. header.

				i = i + 4;
				ul_read_word_buff = 0;
				ul_read_word_buff |= (pbyStreamData[i] << 8);
				ul_read_word_buff |= (pbyStreamData[i+1] << 16);
				ul_read_word_buff |= (pbyStreamData[i+2] << 24);

				for (  ; i < lStreamDataSize - 4; i++ )
				{
					ul_read_word_buff = ul_read_word_buff >> 8;
					ul_read_word_buff &= 0x00FFFFFF;
					ul_read_word_buff |= (pbyStreamData[i+3] << 24);

					if ( (ul_read_word_buff & ul_masking_word_seq) == ul_h264_result_word_seq_PPS )
					{
						// PPS has been detected.
						l_pps_found = 1;
						break;
					}

					// Continue to find the pps in next loop
				}
			}
		}

		if ( l_pps_found == 1 )
		{
			// Now, let's start to find the next sync word to find the end position of Seq. Header

			if ( *plSeqHeaderSize > 0 )
				i = 0;     // we already find the sps, pps in previous frame
			else
				i = i + 4;
			ul_read_word_buff = 0;
			ul_read_word_buff |= (pbyStreamData[i] << 8);
			ul_read_word_buff |= (pbyStreamData[i+1] << 16);
			ul_read_word_buff |= (pbyStreamData[i+2] << 24);

			for ( ; i < lStreamDataSize - 4; i++ )
			{
				ul_read_word_buff = ul_read_word_buff >> 8;
				ul_read_word_buff &= 0x00FFFFFF;
				ul_read_word_buff |= (pbyStreamData[i+3] << 24);

				if ( (ul_read_word_buff & ul_masking_word_sync) == ul_h264_result_word_sync )
				{
					long l_cnt_zeros = 0;       // to count extra zeros ahead of "00 00 01"

					// next sync-word has been found.
					l_seq_end_pos = i - 1;      // save the end position of the sequence header (00 00 01 case)

					// any zeros can be added ahead of "00 00 01" sync word by H.264 specification. Count the number of these leading zeros.
					while (1)
					{
						l_cnt_zeros++;

						if(i >= l_cnt_zeros) //ZzaU :: to prevent segmentation fault.
						{
							if ( pbyStreamData[i-l_cnt_zeros] == 0 )
							{
								l_seq_end_pos = l_seq_end_pos -1;    // decrease the end position of Seq. Header by 1.
							}
							else
								break;
						}
						else
							break;
					}

					if ( *plSeqHeaderSize > 0 )
					{
						// we already find the sps, pps in previous frame
						l_seq_length = l_seq_end_pos - l_seq_start_pos + 1;

						if ( l_seq_length > 0 )
						{
							if ( *plSeqHeaderSize + l_seq_length > MAX_SEQ_HEADER_ALLOC_SIZE ) // check the maximum threshold
								return 0;

							*ppbySeqHeaderData = TCC_realloc(*ppbySeqHeaderData , *plSeqHeaderSize + l_seq_length );     // allocation memory for sequence header array (must free this at the CLOSE step)
							memcpy( (unsigned char*) (*ppbySeqHeaderData) + *plSeqHeaderSize , &pbyStreamData[l_seq_start_pos], l_seq_length);   // save the seq. header to array
							*plSeqHeaderSize = *plSeqHeaderSize + l_seq_length;
						}

						return 1;

					}
					else
					{
						// calculate the length of the sequence header
						l_seq_length = l_seq_end_pos - l_seq_start_pos + 1;

						if ( l_seq_length > 0 )
						{
							*ppbySeqHeaderData = TCC_malloc( l_seq_length );     // allocation memory for sequence header array (must free this at the CLOSE step)
							memcpy( (unsigned char*) (*ppbySeqHeaderData), &pbyStreamData[l_seq_start_pos], l_seq_length);   // save the seq. header to array
							*plSeqHeaderSize = l_seq_length;

							return 1;  // We've found the sequence header successfully
						}
					}
				}

				// Continue to find the sync-word in next loop
			}
		}

		if ( l_sps_found == 1 && l_pps_found == 1)
		{
			// we found sps and pps, but we couldn't find the next sync word yet
			l_seq_end_pos = lStreamDataSize - 1;
			l_seq_length = l_seq_end_pos - l_seq_start_pos + 1;        // calculate the length of the sequence header

			if ( *plSeqHeaderSize > 0 )
			{
				// we already saved the sps, pps in previous frame
				if ( l_seq_length > 0 )
				{
					if ( *plSeqHeaderSize + l_seq_length > MAX_SEQ_HEADER_ALLOC_SIZE )     // check the maximum threshold
						return 0;

					*ppbySeqHeaderData = TCC_realloc(*ppbySeqHeaderData , *plSeqHeaderSize + l_seq_length );     // allocate memory for sequence header array (must free this at the CLOSE step)
					memcpy( (unsigned char*) (*ppbySeqHeaderData) + *plSeqHeaderSize , &pbyStreamData[l_seq_start_pos], l_seq_length);   // save the seq. header to array
					*plSeqHeaderSize = *plSeqHeaderSize + l_seq_length;
				}

			}
			else
			{
				*ppbySeqHeaderData = TCC_malloc( l_seq_length );           // allocate memory for sequence header array (must free this at the CLOSE step)
				memcpy( (unsigned char*) (*ppbySeqHeaderData), &pbyStreamData[l_seq_start_pos], l_seq_length);   // save the seq. header to array
				*plSeqHeaderSize = *plSeqHeaderSize + l_seq_length;
			}
		}
	}
	else
	{
		unsigned long syncword = 0xFFFFFFFF;
		int	start_pos = -1;
		int end_pos = -1;
		int i;

		syncword <<= 8;
		syncword |= pbyStreamData[0];
		syncword <<= 8;
		syncword |= pbyStreamData[1];
		syncword <<= 8;
		syncword |= pbyStreamData[2];

		for(i = 3; i < lStreamDataSize; i++) {
			syncword <<= 8;
			syncword |= pbyStreamData[i];

			if( (syncword >> 8) == 1 ) {	// 0x 000001??
				if( syncword >= MPEG4_VOL_STARTCODE_MIN &&
					syncword <= MPEG4_VOL_STARTCODE_MAX )
					start_pos = i-3;
				//else if( start_pos >= 0 || *plSeqHeaderSize > 0 ) {
				else if( start_pos >= 0 && *plSeqHeaderSize > 0 ) {
					if ( syncword == MPEG4_VOP_STARTCODE )
					{
						end_pos = i-3;
						break;
					}
				}
			}
		}

		if (start_pos >= 0 && end_pos == -1) {
			//end_pos = lStreamDataSize - start_pos;
			end_pos = lStreamDataSize;
		}

		if( start_pos >= 0 ) {
			if( end_pos >= 0 ) {
				*plSeqHeaderSize = end_pos-start_pos;
				*ppbySeqHeaderData = TCC_malloc( *plSeqHeaderSize );     // allocate memory for sequence header array
				memcpy(*ppbySeqHeaderData, pbyStreamData + start_pos, *plSeqHeaderSize);
				return 1;
			}
			else {
				*plSeqHeaderSize = lStreamDataSize - start_pos;
				*ppbySeqHeaderData = TCC_malloc( *plSeqHeaderSize );     // allocate memory for sequence header array
				memcpy(*ppbySeqHeaderData, pbyStreamData + start_pos, *plSeqHeaderSize);
				return 0;
			}
		}
		else if( *plSeqHeaderSize > 0 ) {
			if( end_pos < 0 )
				end_pos = lStreamDataSize;

			if ( *plSeqHeaderSize + end_pos > MAX_SEQ_HEADER_ALLOC_SIZE ) // check the maximum threshold
				return 0;

			*ppbySeqHeaderData = TCC_realloc(*ppbySeqHeaderData , *plSeqHeaderSize + end_pos);     // re-allocate memory for sequence header array
			memcpy(*ppbySeqHeaderData + *plSeqHeaderSize, pbyStreamData, end_pos);
			*plSeqHeaderSize += end_pos;
			return 1;
		}
	}

	return 0; // We couldn't find the complete sequence header yet. We need to search the next frame data.
}
#endif

#if defined(_TCC8920_)
#define VP8_BUFF_SIZE 44  // temporary  buffer for saving start code for VP8

static void put_LE32(unsigned char **pp, unsigned int var)
{
	**pp = (unsigned char)((var)>>0);
	*pp = *pp + 1;
	**pp = (unsigned char)((var)>>8);
	*pp = *pp + 1;
	**pp = (unsigned char)((var)>>16);
	*pp = *pp + 1;
	**pp = (unsigned char)((var)>>24);
	*pp = *pp + 1;
}

static void put_LE16(unsigned char **pp, unsigned int var)
{
	**pp = (unsigned char)((var)>>0);
	*pp = *pp + 1;
	**pp = (unsigned char)((var)>>8);
	*pp = *pp + 1;
}

// Function for attaching start code for VP8 codec. This is only for TCC892x(VPU:CODA960).
static int get_startcode_for_VP8( int for_seqHeader, int frameWidth, int frameHeight, int streamDataSize, unsigned char *startCodeBuffer, int *startCodeBufferSize )
{
    unsigned char* puc_buffer_ptr = startCodeBuffer;

    if(startCodeBuffer == NULL)
        return -1;

    if(for_seqHeader)
    {
        // Make Seq Header for VP8
        // Only for the first frame
        *startCodeBufferSize = 44;

        *puc_buffer_ptr++ = (unsigned char)'D';
        *puc_buffer_ptr++ = (unsigned char)'K';
        *puc_buffer_ptr++ = (unsigned char)'I';
        *puc_buffer_ptr++ = (unsigned char)'F';
        put_LE16(&puc_buffer_ptr, 0x00);
        put_LE16(&puc_buffer_ptr, 0x20);
        *puc_buffer_ptr++ = (unsigned char)'V';
        *puc_buffer_ptr++ = (unsigned char)'P';
        *puc_buffer_ptr++ = (unsigned char)'8';
        *puc_buffer_ptr++ = (unsigned char)'0';
        put_LE16(&puc_buffer_ptr, frameWidth);
        put_LE16(&puc_buffer_ptr, frameHeight);
        put_LE32(&puc_buffer_ptr, 30);
        put_LE32(&puc_buffer_ptr, 1);
        put_LE32(&puc_buffer_ptr, 3000);
        put_LE32(&puc_buffer_ptr, 0);
    }
    else
    {
        // Not the first frame.
        *startCodeBufferSize = 12;
    }

    // Make Pic Header for VP8
    // For all frames
    put_LE32(&puc_buffer_ptr, streamDataSize);
    put_LE32(&puc_buffer_ptr, 0);
    put_LE32(&puc_buffer_ptr, 0);

    return 0;
}

static int check_startcode_for_VP8( unsigned char *streamBuffer, int streamBufferSize, int for_seqHeader )
{
    unsigned char* pBuff = streamBuffer;

    if(streamBuffer == NULL)
        return -1;

    if(for_seqHeader)
    {
        if(*pBuff++ == (unsigned char)'D'
	        && *pBuff++ == (unsigned char)'K'
	        && *pBuff++ == (unsigned char)'I'
	        && *pBuff++ == (unsigned char)'F')
        {
			return 1;
        }
    }
    else
    {
		unsigned char tBuff[12+1];
		unsigned char* ptBuff = tBuff;
	    put_LE32(&ptBuff, streamBufferSize);

		if(*pBuff++ == tBuff[0] && *pBuff++ == tBuff[1] && *pBuff++ == tBuff[2] && *pBuff++ == tBuff[3])
			return 1;
    }

    return 0;
}
#endif


/** This function is used to process the input buffer and provide one output buffer
  */
void omx_videodec_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* pInputBuffer	, OMX_BUFFERHEADERTYPE* pOutputBuffer) {

	omx_videodec_component_PrivateType* omx_private = openmaxStandComp->pComponentPrivate;
    char value[PROPERTY_VALUE_MAX];
	OMX_S32 ret;
	OMX_S32 nOutputFilled = 0;
	OMX_S32 nLen = 0;
	int internalOutputFilled = 0;

#ifndef HAVE_ANDROID_OS
	OMX_U32 nSize;
#endif
	OMX_U32 output_len = 0;
	OMX_U32 input_offset = 0;
	OMX_U32 code_type = CODETYPE_NONE;
#ifdef ANDROID_USE_GRALLOC_BUFFER
	TCC_PLATFORM_PRIVATE_PMEM_INFO *plat_priv = NULL;
	buffer_handle_t*  grallocHandle = NULL;
	OMX_U8 *pGrallocAddr;
	void *pGrallocPureAddr;
	#ifdef COMPARE_TIME_LOG
	clock_t start, end;
	#endif
	COPY_MODE gralloc_copy_mode = COPY_NONE;
	OMX_BOOL bCopyToGrallocBuff = OMX_FALSE;
#endif
	int decode_result;
	int i;
	dec_disp_info_t dec_disp_info_tmp;
	dec_disp_info_tmp.m_iTimeStamp = 0;
    omx_base_video_PortType *outPort = (omx_base_video_PortType *)omx_private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

	if(omx_private->state != 3) {
	    LOGE("=> omx_private->state != 3");
		return;
	}
#ifdef DIVX_DRM5
	if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MPEG4)
	{
		DivxDecryptEx(pInputBuffer->pBuffer,pInputBuffer->nFilledLen);
	}
#endif

	if(omx_private->bThumbnailMode && omx_private->pThumbnailBuff)
	{
		VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
		return;
	}

	// stream discontinuity occurred, all ports were flushed, 
	// data in vpu buffer is invalid
	// so, should reset timestamp
	if (omx_private->bAllPortsFlushed == OMX_TRUE) {
		LOGI("Reset timestamp");
		omx_private->bAllPortsFlushed = OMX_FALSE;
		pInputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
	}

    /** Fill up the current input buffer when a new buffer has arrived */
	if(omx_private->isNewBuffer)
	{
		if( !(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) ) {
			if( omx_private->bWaitNewBuffer == OMX_TRUE ) {
				omx_private->bWaitNewBuffer = OMX_FALSE;
				pInputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
			}

			//frame-rate update (AVG)
			if( omx_private->bUpdateFPS == OMX_TRUE ) {
				if( pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME ) {
					omx_private->frameCount = 0;
					omx_private->prevTimestamp = 0;
				}
				else {
					if( omx_private->frameCount ) {
						if( omx_private->prevTimestamp && omx_private->prevTimestamp < pInputBuffer->nTimeStamp ) {
							if ( omx_private->frameCount == 1)
								omx_private->frameDuration = pInputBuffer->nTimeStamp - omx_private->prevTimestamp;
							else {
								OMX_TICKS time_diff = pInputBuffer->nTimeStamp - omx_private->prevTimestamp;
								OMX_TICKS frame_due = time_diff / omx_private->frameCount;
								OMX_S32 frame_rate = (1000000000 / (OMX_S32)frame_due);
								omx_private->frameDuration = frame_due;
								omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate = frame_rate;
								omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameRate = frame_rate;
							}
						}
						omx_private->frameCount = 0;
					}
					omx_private->prevTimestamp = pInputBuffer->nTimeStamp;
				}
			}
		}

		omx_private->inputCurrBuffer = pInputBuffer->pBuffer;
		omx_private->inputCurrLength = pInputBuffer->nFilledLen;
		omx_private->isNewBuffer = 0;

		LOGINFO("New Buffer -----> inputCurrLen:%d, PTS:%d, offset:%d", omx_private->inputCurrLength, pInputBuffer->nTimeStamp/1000, pInputBuffer->nOffset);
#ifdef PEEK_DEFRAGMENTED_FRAME
		PrintHexDataFrontRear(omx_private->inputCurrBuffer, omx_private->inputCurrLength, "Buffer");
#endif
	}
#ifdef DEFRAGMENT_INPUT_FRAME
	else
	{
		if (omx_private->bDelayedDecodeOut == OMX_FALSE) {
			/* 
			 * Case - omx_private->isNewBuffer - FALSE, pInputBuffer->nFlags - SYNCFRAME :
			 * nFlags is current input's but inputCurrBuffer is previous input's during playback in all frame defragmentation mode. 
			 */
			if((pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME) && (omx_private->bUseFrameDefragmentation == OMX_TRUE))
			{
				LOGMSG("SyncFrame - isNewBuffer ? %s", omx_private->isNewBuffer ? "YES" : "NO");
				omx_private->isNewBuffer = 1;
				pOutputBuffer->nFilledLen = 0;
				return;
			}

			if( omx_private->bWaitNewBuffer == OMX_TRUE ) {
				omx_private->isNewBuffer = 1;
				pOutputBuffer->nFilledLen = 0;
				return;
			}
		}
	}
#endif

	pOutputBuffer->nFilledLen = 0;
	pOutputBuffer->nOffset = 0;

#if 1//JS Baek
	if(pInputBuffer->nFlags & OMX_BUFFERFLAG_BFRAME_SKIP)
	{
		//Need to boost up so that decode with maximized speed.
		omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_ONLY_B;
		omx_private->bDecIndexOutput = OMX_FALSE;
		//LOGE("BMC : VDEC_SKIP_FRAME_ONLY_B");
		//vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
	}
	else if(pInputBuffer->nFlags & OMX_BUFFERFLAG_IFRAME_ONLY)
	{
		omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_EXCEPT_I;
		//LOGE("BMC : VDEC_SKIP_FRAME_EXCEPT_I");
		//To display decoded index directly
		omx_private->bDecIndexOutput = OMX_TRUE;
		//vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, AVAILABLE_MAX_WIDTH, AVAILABLE_MAX_HEIGHT); //max-clock!!
		//vpu_update_sizeinfo(omx_private->gsVDecInit.m_iBitstreamFormat, omx_private->gsVDecUserInfo.bitrate_mbps, omx_private->gsVDecUserInfo.frame_rate, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicWidth, omx_private->gsVDecOutput.m_pInitialInfo->m_iPicHeight);
	}
	else
	{
		omx_private->i_skip_scheme_level = VDEC_SKIP_FRAME_DISABLE;
		omx_private->bDecIndexOutput = OMX_FALSE;
	}
#endif

	while (!nOutputFilled) {

	    if (omx_private->isFirstBuffer) {
	        tsem_down(omx_private->avCodecSyncSem);
	        omx_private->isFirstBuffer = 0;
	    }

#ifdef DEBUG_INPUT_FRAME_FILTER_BY_PTS
        if (omx_private->frameFilterPTS && pInputBuffer->nTimeStamp/1000 >= omx_private->frameFilterPTS) {
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
			return;
		}
#endif

	    //////////////////////////////////////////////////////////////////////////////////////////
	    /* ZzaU :: remove NAL-Start Code when there are double codes. ex) AVI container */
        if (CONTAINER_TYPE_TS != omx_private->pVideoDecodInstance.container_type 
            && (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || 
				omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC)) {

			OMX_U8* p = omx_private->inputCurrBuffer;
			if (omx_private->inputCurrLength > 8) {
				if (!memcmp("\x00\x00\x00\x01", p, 4)) {
					if (!memcmp("\x00\x00\x00\x01", p+4, 4)) {
						input_offset = 4;
						DBUG_MSG("Double NAL-Start Code!!");
					} else if (!memcmp("\x00\x00\x01", p+4, 3)) {
						input_offset = 3;
						DBUG_MSG("remove 00 00 01 behind NAL-Start Code!!");
						pInputBuffer->pBuffer[3] = 0x00;
					}
				}
			} else {
				LOGW("WARNING !! video input length is less than 8 bytes.");
				// fall through
			}
		}

	    if(pInputBuffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
			DBUG_MSG("Config data IN!!");

			omx_private->extractorType = (pInputBuffer->nFlags & OMX_BUFFERFLAG_EXTRACTOR_TYPE_FILTER);
			LOGI("omx_private->extractorType = 0x%lx", omx_private->extractorType);

			if(ExtractConfigData(omx_private, input_offset) == OMX_FALSE) {
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
				return;
			}

			omx_private->isNewBuffer = 1;
			pOutputBuffer->nFilledLen = 0;
			pInputBuffer->nFilledLen = 0;

			omx_private->bDetectFrameDelimiter = OMX_FALSE;
			omx_private->start_code_with_type = 0xFFFFFFFF;

#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
			ioctl(omx_private->g_hFb, TCC_LCDC_VIDEO_SET_FRAMERATE, omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate ) ;  // TCC_LCDC_HDMI_GET_DISPLAYED
#endif
			return;
		}

		if(!omx_private->pVideoDecodInstance.avcodecInited)
		{
#ifdef RESTORE_DECODE_ERR
			if(omx_private->cntDecError != 0){
				LOGE("start to restore decode error count(%d)", omx_private->cntDecError);
				if(!(pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME)){
					LOGE("to set forcingly SYNC_FRAME");
					pInputBuffer->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
				}

				if(omx_private->seqHeader_backup != NULL){
					omx_private->need_sequence_header_attachment = OMX_TRUE;
				}
			}
			else
#endif

			if ((CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
			/*|| (CONTAINER_TYPE_MPG == omx_private->pVideoDecodInstance.container_type)*/)
			{
				if (0 == omx_private->szConfigdata)
				{
					LOGINFO("[BufMgmtCB] Call CODETYPE_HEADER");
					omx_private->SearchCodeType(omx_private, &input_offset, CODETYPE_HEADER);
					if (input_offset >= omx_private->inputCurrLength)
					{
						omx_private->isNewBuffer = 1;
						pInputBuffer->nFilledLen = 0;
						LOGW("[BufMgmtCB] header is not included in this input frame");
						return;
					}

					omx_private->inputCurrBuffer += input_offset;
					omx_private->inputCurrLength -= input_offset;
					pInputBuffer->nFilledLen -= input_offset;
					input_offset = 0;
				}
			}

			omx_private->frameSearchOrSkip_flag = 0;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iPicWidth				= outPort->sPortParam.format.video.nFrameWidth;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iPicHeight			= outPort->sPortParam.format.video.nFrameHeight;
			omx_private->pVideoDecodInstance.gsVDecInit.m_bEnableVideoCache		= 0;//1;	// Richard_20100507 Don't use video cache
			omx_private->pVideoDecodInstance.gsVDecInit.m_bEnableUserData 		= 0;
			omx_private->pVideoDecodInstance.gsVDecInit.m_pExtraData			= omx_private->extradata;
			omx_private->pVideoDecodInstance.gsVDecInit.m_iExtraDataLen			= omx_private->extradata_size;

			omx_private->pVideoDecodInstance.gsVDecInit.m_bM4vDeblk 			= 0;//pCdk->m_bM4vDeblk;
			omx_private->pVideoDecodInstance.gsVDecInit.m_uiDecOptFlags			= 0;
			omx_private->pVideoDecodInstance.gsVDecInit.m_uiMaxResolution 		= 0;//pCdk->m_uiVideoMaxResolution;
			omx_private->pVideoDecodInstance.gsVDecInit.m_bFilePlayEnable 		= 1;

			omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode	= 0;
			if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr)
			{
				if(isSWCodec(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat)
		#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
					|| (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMJPEG)
		#endif
				)
					omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode	= 0;
				else
					omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode	= 1;

				LOGI("Gralloc :: VPU Format = %d, HAL_PIXEL_FORMAT_YCbCr_420_SP = %d, size = %ld x %ld, YUVinter(%d)", omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat,
						omx_private->blocalPlaybackMode, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
						omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode);

			}
			else
			{

				if(omx_private->isRemotePlayerPlay == OMX_TRUE)
					omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode	= 1;

				LOGI("VPU Format = %d, size = %ld x %ld, YUVinter(%d)", omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat,
							outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
							omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode);
			}

			{
				omx_private->pVideoDecodInstance.dec_disp_info_input.m_iStdType 			= omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat;
				omx_private->pVideoDecodInstance.dec_disp_info_input.m_iTimeStampType 	= CDMX_PTS_MODE;	// Presentation Timestamp (Display order)
				if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC)
				{
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFmtType 		= omx_private->pVideoDecodInstance.container_type;

					if(omx_private->pVideoDecodInstance.container_type == CONTAINER_TYPE_AVI || omx_private->pVideoDecodInstance.container_type == CONTAINER_TYPE_MP4)
					{
						DBUG_MSG("TimeStampType = CDMX_DTS_MODE");
						omx_private->pVideoDecodInstance.dec_disp_info_input.m_iTimeStampType = CDMX_DTS_MODE;	// Decode Timestamp (Decode order)
					}
					else
					{
						DBUG_MSG("TimeStampType = CDMX_PTS_MODE");
					}
				}
				else if(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_MPEG4)
				{
					DBUG_MSG("This file comes from Android MP4 parser. TimeStampType = CDMX_DTS_MODE");
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iTimeStampType = CDMX_DTS_MODE;	// Decode Timestamp (Decode order)
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFmtType 		 = CONTAINER_TYPE_MP4;
				}
				else
				{
					DBUG_MSG("This file comes from Android proprietry or plug-in parser. extractorType(0x%08x)", omx_private->extractorType);
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iTimeStampType = CDMX_PTS_MODE;	// Decode Timestamp (Decode order)
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFmtType 		 = CONTAINER_TYPE_TS;
				}

				disp_pic_info ( CVDEC_DISP_INFO_INIT, (void*) &(omx_private->pVideoDecodInstance.dec_disp_info_ctrl), (void*)omx_private->pVideoDecodInstance.dec_disp_info,(void*) &(omx_private->pVideoDecodInstance.dec_disp_info_input), omx_private);
			}

			if(omx_private->seq_header_init_error_count == SEQ_HEADER_INIT_ERROR_COUNT)
			{
				if( (ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_INIT, NULL, &(omx_private->pVideoDecodInstance.gsVDecInit), &(omx_private->pVideoDecodInstance.gsVDecUserInfo), (omx_private->pVideoDecodInstance.pVdec_Instance))) < 0 )
				{
					LOGE( "[VDEC_INIT] [Err:%ld] video decoder init", ret );

					if(ret != -VPU_ENV_INIT_ERROR) //to close vpu!!
						omx_private->pVideoDecodInstance.isVPUClosed = OMX_FALSE;

					VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
					return;
				}

				if(omx_private->vpu_preOpen_fd > 0)
				{
					vpu_preCtrl(omx_private->vpu_preOpen_fd);
					omx_private->vpu_preOpen_fd = -1;
				}
				vpu_update_sizeinfo(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, omx_private->pVideoDecodInstance.gsVDecUserInfo.bitrate_mbps,
									omx_private->pVideoDecodInstance.gsVDecUserInfo.frame_rate, omx_private->pVideoDecodInstance.gsVDecInit.m_iPicWidth, omx_private->pVideoDecodInstance.gsVDecInit.m_iPicHeight, omx_private->pVideoDecodInstance.pVdec_Instance);
				omx_private->pVideoDecodInstance.isVPUClosed = OMX_FALSE;
			}

			if(omx_private->pVideoDecodInstance.isVPUClosed == OMX_TRUE)// Following codes should not be worked under vpu-closed status.
			{
				LOGE( "Now VPU has been closed , return " );
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
				return;
			}

			if(omx_private->szConfigdata != 0)
			{
				if (CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
				{
					LOGE("BufMgmtCB: TS fileformat stream can't use szConfigdata !!");
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
					return;
				}

				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = omx_private->pConfigdata;
				omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen = omx_private->szConfigdata;
			}
			else
			{
#ifdef DEFRAGMENT_INPUT_FRAME
				if (omx_private->bUseFrameDefragmentation == OMX_TRUE)
				{
					if (omx_private->bDelayedDecodeOut == OMX_FALSE) {
						omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
						omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);
						memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->inputCurrBuffer + input_offset, omx_private->inputCurrLength - input_offset);
					}
				}
				else
#endif
				{
					omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = omx_private->inputCurrBuffer + input_offset;
				}
				omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen	= omx_private->inputCurrLength - input_offset;
			}

			omx_private->pVideoDecodInstance.gsVDecInput.m_iIsThumbnail = omx_private->bThumbnailMode;

#ifdef RESTORE_DECODE_ERR
			if(omx_private->cntDecError != 0 && omx_private->seqHeader_backup != NULL)
			{
				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);
				memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->seqHeader_backup, omx_private->seqHeader_len);

				//omx_private->gsVDecInput.m_pInp[PA] = omx_private->gsVDecInput.m_pInp[VA] = omx_private->seqHeader_backup;
				omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen	= omx_private->seqHeader_len;
			}
#endif

#ifdef CHECK_SEQHEADER_WITH_SYNCFRAME
			if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC
				|| omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC
				|| omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4)
			{
				OMX_BOOL bFound_frame = OMX_FALSE;
				DBUG_MSG("sequence header: 0x%x - %d bytes, frame: 0x%x - %d bytes", omx_private->sequence_header_only, omx_private->sequence_header_size,
											omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);

				if(0 >= extract_seqheader((const unsigned char*)omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen,
													&(omx_private->sequence_header_only), &(omx_private->sequence_header_size),
													omx_private->pVideoDecodInstance.video_coding_type))
				{
					bFound_frame = OMX_FALSE;

					if( (omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] == omx_private->pConfigdata) && omx_private->sequence_header_size > 0)
					{
						unsigned char *input_addr 	= omx_private->inputCurrBuffer + input_offset;
						unsigned int input_size 	= omx_private->inputCurrLength - input_offset;

						DBUG_MSG("Retry :: sequence header extraction (%d) after checking Configdata", omx_private->sequence_header_size);
						if(0 < extract_seqheader((const unsigned char*)input_addr, input_size,
															&(omx_private->sequence_header_only), &(omx_private->sequence_header_size),
															omx_private->pVideoDecodInstance.video_coding_type))
						{
							bFound_frame = OMX_TRUE;
							omx_private->need_sequence_header_attachment = OMX_TRUE;
							omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
							omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);

							memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->sequence_header_only, omx_private->sequence_header_size);
							memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] + omx_private->sequence_header_size, input_addr, input_size);
							omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen	+= input_size;
						}
					}

					if(!bFound_frame)
					{
						if( omx_private->sequence_header_size > 0 ) {
							omx_private->need_sequence_header_attachment = OMX_TRUE;
						}

						LOGE( "[%d'th frame with only sequence frame (%d bytes)] VPU want sequence_header frame with sync frame!", SEQ_HEADER_INIT_ERROR_COUNT - omx_private->seq_header_init_error_count,
															omx_private->sequence_header_size);

						if(--omx_private->seq_header_init_error_count <= 0)
						{
							LOGE( "This contents can't decode because there are no sequence frame.");
							VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
							return;
						}
						omx_private->isNewBuffer = 1;
						pInputBuffer->nFilledLen = 0;
						pOutputBuffer->nFilledLen = 0;

						return;
					}
				}
				else
				{
					if(omx_private->need_sequence_header_attachment)
					{
						unsigned char *temp_addr = (unsigned char *)omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA];
						omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
						omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);

						memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->sequence_header_only, omx_private->sequence_header_size);

						if(temp_addr == omx_private->pConfigdata)
						{
							memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] + omx_private->sequence_header_size, omx_private->inputCurrBuffer + input_offset, omx_private->inputCurrLength - input_offset);
							omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen	+= (omx_private->inputCurrLength - input_offset);
						}
						else
						{
							memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA]+ omx_private->sequence_header_size, temp_addr, omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
							omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen	+= omx_private->sequence_header_size;
						}
					}
					else
					{
						LOGI("Success at one time :: Input frame has sequence(%d)/%d + sync frame", omx_private->sequence_header_size, omx_private->inputCurrLength - input_offset);
					}
				}
			}
#endif

#if defined(_TCC8920_)
			if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingVPX)
			{
				if(0 >= check_startcode_for_VP8(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, 1))
				{
					unsigned char *InStreamBuff;
					unsigned int InStreamSize;

					InStreamBuff = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);

					get_startcode_for_VP8(1, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
											omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, InStreamBuff, &InStreamSize);

					memcpy(InStreamBuff+InStreamSize, omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
					omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
					omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);
					omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen += InStreamSize;
				}
			}
#endif

			omx_private->pVideoDecodInstance.video_dec_idx = 0;
#ifdef ANDROID_USE_GRALLOC_BUFFER
			if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr)
			{
				omx_private->max_fifo_cnt = 6;//2;
			}
			else
#endif
			{
				omx_private->max_fifo_cnt = outPort->sPortParam.nBufferCountActual;
			}
			vpu_set_additional_refframe_count(omx_private->max_fifo_cnt, omx_private->pVideoDecodInstance.pVdec_Instance);

			if( (ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_DEC_SEQ_HEADER, NULL, &omx_private->pVideoDecodInstance.gsVDecInput, &omx_private->pVideoDecodInstance.gsVDecOutput, (omx_private->pVideoDecodInstance.pVdec_Instance) )) < 0 )
			{
				if ( (--omx_private->seq_header_init_error_count <= 0) ||
					 (ret == -RETCODE_INVALID_STRIDE) || (ret == -RETCODE_CODEC_SPECOUT) || (ret == -RETCODE_CODEC_EXIT) || (ret == -RETCODE_MULTI_CODEC_EXIT_TIMEOUT))
				{
					LOGE( "[VDEC_DEC_SEQ_HEADER] [Err:%ld]", ret );
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
					return;
				}
				else
				{
					DBUG_MSG("skip seq header frame, data len %d", omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
					LOGI( "[VDEC_DEC_SEQ_HEADER] retry %ld using next frame!",  SEQ_HEADER_INIT_ERROR_COUNT - omx_private->seq_header_init_error_count);

					omx_private->isNewBuffer = 1;
					pInputBuffer->nFilledLen = 0;
					pOutputBuffer->nFilledLen = 0;

					return;
				}
			}
#ifdef RESTORE_DECODE_ERR
			else
			{
				if(omx_private->seqHeader_backup == NULL)
				{
					omx_private->seqHeader_backup = TCC_calloc(1,omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
					memcpy(omx_private->seqHeader_backup, omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
					omx_private->seqHeader_len = omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen;
					omx_private->cntDecError = 0;

					LOGI("backup seq_header(%ld) data to restore decode", omx_private->seqHeader_len);
				}
				LOGI("success seq_header(%ld)", omx_private->seqHeader_len);
			}
#endif

			omx_private->pVideoDecodInstance.avcodecInited = 1;

			//frame-rate update (AVG)
			omx_private->frameCount = 0;
			omx_private->prevTimestamp = 0;
			omx_private->frameDuration = 0;
			omx_private->bUpdateFPS = OMX_FALSE;

#ifdef DEFRAGMENT_INPUT_FRAME
			if (omx_private->bUseFrameDefragmentation == OMX_TRUE)
			{
				if (omx_private->bDelayedDecodeOut == OMX_FALSE) 
				{
					omx_private->code_type = CODETYPE_NONE;

					//frame-rate update (AVG)
					if (omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate != 0)
					{
						omx_private->frameDuration = (OMX_TICKS)1000000000 / omx_private->pVideoDecodInstance.cdmx_info.m_sVideoInfo.m_iFrameRate;
						omx_private->bUpdateFPS = OMX_TRUE;
					} 
					else
					{
						omx_private->bUpdateFPS = OMX_FALSE;
					}
				}
			}
#endif

			// set the flag to disable further processing until Client reacts to this by doing dynamic port reconfiguration
			ret = isPortChange(openmaxStandComp);
			if(ret < 0) {//max-resolution over!!
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
				return;
			}

			if(ret == 1) //port reconfiguration!!
				return;
		}

		if(pInputBuffer->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
		{
			pInputBuffer->nFlags &= ~OMX_BUFFERFLAG_SYNCFRAME;

			//AVG: Some problems have found in special conditions. (temporary disabled)
			if( omx_private->isFirstSyncFrame == OMX_FALSE ) {
				//omx_private->bWaitKeyFrameOut = OMX_TRUE;
			}

#ifdef ENABLE_DECODE_ONLY_MODE_AVC
			if(omx_private->bUseDecodeOnlyMode == OMX_TRUE &&
			   omx_private->bDecIndexOutput == OMX_FALSE )
			{
				/* Playback-start should not be affected by decode only mode. */
				if(omx_private->isFirstSyncFrame == OMX_FALSE)
				{
					omx_private->bSetDecodeOnlyMode = OMX_TRUE;
					omx_private->skipFrameNum = 0;
					omx_private->decodeOnlyErrNum = 0;
				}
			}
#endif

			/* Curr. VPU can't support I-frame search mode for AVS encoded stream. */
			if(omx_private->pVideoDecodInstance.video_coding_type != OMX_VIDEO_CodingAVS)
			{
				DPRINTF_DEC_STATUS("[SEEK] I-frame Search Mode enable");
				omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955
				omx_private->frameSearchOrSkip_flag = 1;
			}

			if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC)
			{
#ifdef ENABLE_DECODE_ONLY_MODE_AVC
				if(omx_private->pVideoDecodInstance.container_type == CONTAINER_TYPE_TS
				|| omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_MPEG2TS
				|| omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_RTSP
				)
				{
					if(omx_private->isFirstSyncFrame == OMX_TRUE)
					{
						omx_private->isFirstSyncFrame = OMX_FALSE;
					}
					else
					{
						omx_private->I_frame_search_mode = AVC_IDR_PICTURE_SEARCH_MODE;
						omx_private->IDR_frame_search_count = 0;

						if( omx_private->bDecIndexOutput == OMX_TRUE )
							omx_private->I_frame_search_mode = AVC_NONIDR_PICTURE_SEARCH_MODE;
					}
				}
				else
#endif
				if(!(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC))
				{
					omx_private->frameSearchOrSkip_flag = 0;
				}
			}

			disp_pic_info( CVDEC_DISP_INFO_RESET, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl),
							(void*)omx_private->pVideoDecodInstance.dec_disp_info,(void*)&(omx_private->pVideoDecodInstance.dec_disp_info_input),
							omx_private);

#ifdef HAVE_ANDROID_OS
			/*ZzaU :: Clear all decoded frame-buffer!!*/
			if(omx_private->max_fifo_cnt != 0 && omx_private->bDecIndexOutput == OMX_FALSE)
			{
	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				LOGD("Used Buffer Count %d", omx_private->used_fifo_count);
	#endif
				while(omx_private->in_index != omx_private->out_index)
				{
					//DPRINTF_DEC_STATUS("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
					LOGD("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
					if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL, (omx_private->pVideoDecodInstance.pVdec_Instance)) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
						return;
					}
					omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
				}
				omx_private->in_index = omx_private->out_index = omx_private->frm_clear = 0;
	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				omx_private->used_fifo_count = 0;
	#endif
			}
#endif
#ifdef DEFRAGMENT_INPUT_FRAME
			if (omx_private->bUseFrameDefragmentation == OMX_TRUE)
			{
				if (omx_private->bDelayedDecodeOut == OMX_FALSE) 
				{
					omx_private->code_type = CODETYPE_NONE;
					omx_private->start_code_with_type = 0xFFFFFFFF;
					omx_private->isSplittedStartCode = OMX_FALSE; 
					omx_private->splittedStartCodeLen = 0; 
					omx_private->frame_delimiter_offset = OMX_BUFF_OFFSET_UNASSIGNED; 
				}
			}
#endif

			if( omx_private->isFirstSyncFrame == OMX_TRUE )
				omx_private->isFirstSyncFrame = OMX_FALSE;
		}

#ifdef DEFRAGMENT_INPUT_FRAME
		if (omx_private->bUseFrameDefragmentation == OMX_TRUE) 
		{
			if (omx_private->bDelayedDecodeOut == OMX_FALSE) 
			{	
				if (CODETYPE_NONE == omx_private->code_type)
				{
					// search current frame code type
					LOGMSG("[BufMgmtCB] Call CODETYPE_ALL - input_offset:%d", input_offset);
					code_type = omx_private->SearchCodeType(omx_private, &input_offset, CODETYPE_ALL);

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
					if(input_offset + MAX_NAL_STARTCODE_LEN >= omx_private->inputCurrLength && code_type == CODETYPE_NONE)
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

					omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen = 0;
					omx_private->nTimeStamp = pInputBuffer->nTimeStamp;
					pInputBuffer->nTimeStamp += omx_private->frameDuration; //frame-rate update (AVG)

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
					LOGMSG("[BufMgmtCB] CODETYPE_HEADER --> Call CODETYPE_PICTURE - input_offset:%d", input_offset);
					code_type = omx_private->SearchCodeType(omx_private, &input_offset, CODETYPE_PICTURE);

					if(omx_private->isSplittedStartCode == OMX_TRUE)
					{
						LOGMSG("[BufMgmtCB] NAL Start code is Splitted !!");
						omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen -= (MIN_NAL_STARTCODE_LEN - input_offset);
					}
					else if (input_offset > 0)
					{
						LOGMSG("[BufMgmtCB] CODETYPE_HEADER--> Call CODETYPE_PICTURE : Copy (%d) bytes", input_offset);
						memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] + omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, omx_private->inputCurrBuffer, input_offset);
						omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen += input_offset;
#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
						if(omx_private->splittedStartCodeLen)
						{
							input_offset += omx_private->splittedStartCodeLen;
						}
#endif
					}

					omx_private->code_type = CODETYPE_PICTURE; 

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
					if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
					{
						if(input_offset + MAX_NAL_STARTCODE_LEN >= omx_private->inputCurrLength && code_type == CODETYPE_NONE)
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
					LOGMSG("[BufMgmtCB] CODETYPE_PICTURE --> Call CODETYPE_PICTURE : input_offset = %d", input_offset);
					code_type = omx_private->SearchCodeType(omx_private, &input_offset, CODETYPE_PICTURE);
					if(omx_private->isSplittedStartCode == OMX_TRUE)
					{
						LOGW("[BufMgmtCB] NAL Start code is Splitted !!");
						omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen -= (MIN_NAL_STARTCODE_LEN - input_offset);
					}
					else if (input_offset > 0)
					{
						LOGMSG("[BufMgmtCB] CODETYPE_PICTURE--> Call CODETYPE_PICTURE : Copy (%d) bytes", input_offset);
						memcpy(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] + omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, omx_private->inputCurrBuffer, input_offset);
						omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen += input_offset;
#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
						if(omx_private->splittedStartCodeLen)
						{
							input_offset += omx_private->splittedStartCodeLen;
						}
#endif
					}

#ifdef FAST_SCAN_FOR_STREAM_DEFRAGMENTATION
					if(omx_private->frameDefragmentationType == FRAME_DEFRAGMENTATION_TYPE_4BYTE_SCAN)
					{
						if(input_offset + MAX_NAL_STARTCODE_LEN >= omx_private->inputCurrLength && code_type == CODETYPE_NONE)
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
		}
		else
#endif // DEFRAGMENT_INPUT_FRAME
		{
			if (CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
			{
				code_type = omx_private->SearchCodeType(omx_private, &input_offset, CODETYPE_ALL);
				if (input_offset >= omx_private->inputCurrLength)
				{
					omx_private->isNewBuffer = 1;
					pOutputBuffer->nFilledLen = 0;
					pInputBuffer->nFilledLen = 0;

					return;
				}
			}
#ifdef CHECK_SEQHEADER_WITH_SYNCFRAME
			if(omx_private->need_sequence_header_attachment)
			{
				omx_private->need_sequence_header_attachment = OMX_FALSE;
			}
			else
#endif
			{
				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = omx_private->inputCurrBuffer + input_offset;
				omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen  = omx_private->inputCurrLength - input_offset;
			}
		}

		omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameNum = 0;
		omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable = 0;

		switch(omx_private->i_skip_scheme_level)
		{
			case VDEC_SKIP_FRAME_DISABLE:
			case VDEC_SKIP_FRAME_EXCEPT_I:
				omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = omx_private->i_skip_scheme_level;
				break;
			case VDEC_SKIP_FRAME_ONLY_B:
				if(omx_private->i_skip_count == omx_private->i_skip_interval)
				{
					omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = omx_private->i_skip_scheme_level;
					omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameNum = 1000;
				}
				else
				{
					omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
				}
				break;
			default:
					omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
				break;
		}

		if(omx_private->frameSearchOrSkip_flag == 1 )
		{
			omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameNum = 1;
			omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable = omx_private->I_frame_search_mode;
			omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;

			if( omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable == AVC_IDR_PICTURE_SEARCH_MODE )
			{
				DPRINTF_DEC_STATUS( "[SEEK] I-frame Search Mode(IDR-picture for H.264) Enable!!!");
			}
			else if( omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable == AVC_NONIDR_PICTURE_SEARCH_MODE )
			{
				DPRINTF_DEC_STATUS( "[SEEK] I-frame Search Mode(I-slice for H.264) Enable!!!");
			}
		}
		else if( omx_private->frameSearchOrSkip_flag == 2 )
		{
			omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameNum = 1;
			omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable = 0;
			omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_ONLY_B;
			DPRINTF_DEC_STATUS("[SEEK] B-frame Skip Mode Enable!!!");
		}

// FRAME_SKIP_MODE
		if( omx_private->frameSearchOrSkip_flag == 0 ) // video frame skip
		{
			char value[PROPERTY_VALUE_MAX];
			property_get("tcc.video.skip.flag", value, "");
			if (strcmp(value, "1") == 0)
			{
				omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameNum = 1;
				omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable = 0;
				omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_ONLY_B;	//Fix me...
				LOGE("[SEEK] B-frame Skip Mode Enable!!! - Frame late!!! \n");

				property_set("tcc.video.skip.flag", "0");
			}
		}
		if(omx_private->pVideoDecodInstance.isVPUClosed == OMX_TRUE)
		{
			LOGE( "Now VPU has been closed , return " );
			VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, 0);
			return;
		}
// FRAME_SKIP_MODE
#ifdef COMPARE_TIME_LOG
		start = clock();
#endif

#if defined(ANDROID_USE_GRALLOC_BUFFER)
	#if defined(MOVE_HW_OPERATION)
		gralloc_copy_mode = COPY_NONE;
		grallocHandle = (buffer_handle_t*)pOutputBuffer->pBuffer;
		if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr)
		{
			if(omx_private->gralloc_info.m_pDispOut[PA][0] != NULL && omx_private->gralloc_info.m_pDispOut[PA][1] != NULL && omx_private->gralloc_info.m_pDispOut[PA][2] != NULL)
			{
				omx_private->gralloc_info.grallocModule->lock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule,
													(buffer_handle_t)grallocHandle, GRALLOC_USAGE_HW_RENDER,
													0,0,outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, &pGrallocPureAddr);

				pGrallocAddr = (OMX_U8 *)pGrallocPureAddr;
				GBUG_MSG("handle: %p - addr: %p, 0x%x", grallocHandle, (void*)pGrallocAddr, (int)pGrallocAddr);
				plat_priv = (TCC_PLATFORM_PRIVATE_PMEM_INFO *)get_private_addr(openmaxStandComp, (int)pGrallocAddr, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight, &bCopyToGrallocBuff);
				if( bCopyToGrallocBuff )
				{
					GBUG_MSG("G copy");
					pGrallocAddr = (OMX_U8 *)pGrallocPureAddr;
					if(0 <= move_data_using_scaler(openmaxStandComp, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
												omx_private->gralloc_info.m_pDispOut[PA][0], omx_private->gralloc_info.m_pDispOut[PA][1],
												omx_private->gralloc_info.m_pDispOut[PA][2], omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode, pGrallocAddr, SEND_CMD))
					{
						gralloc_copy_mode = COPY_START;
					}
					else
					{
						gralloc_copy_mode = COPY_FAILED;
						omx_private->gralloc_info.grallocModule->unlock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule, (buffer_handle_t)grallocHandle);
					}
				}
			}
		}
	#else
		grallocHandle = (buffer_handle_t*)pOutputBuffer->pBuffer;
		gralloc_copy_mode = COPY_FAILED;
	#endif
#endif

#if defined(_TCC8920_)
		if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingVPX)
		{
			if(0 >= check_startcode_for_VP8(omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, 0))
			{
				unsigned char *InStreamBuff;
				unsigned int InStreamSize;

				InStreamBuff = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);

				get_startcode_for_VP8(0, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
										omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen, InStreamBuff, &InStreamSize);

				memcpy(InStreamBuff+InStreamSize, omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA], omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen);
				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[PA] = vpu_getBitstreamBufAddr(PA, omx_private->pVideoDecodInstance.pVdec_Instance);
				omx_private->pVideoDecodInstance.gsVDecInput.m_pInp[VA] = vpu_getBitstreamBufAddr(VA, omx_private->pVideoDecodInstance.pVdec_Instance);
				omx_private->pVideoDecodInstance.gsVDecInput.m_iInpLen += InStreamSize;
			}
		}
#endif

		//LOGE("Dec Input %d(%d) ", pInputBuffer->nFilledLen, omx_private->gsVDecInput.m_iInpLen);
		if( (ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_DECODE, NULL, &omx_private->pVideoDecodInstance.gsVDecInput, &omx_private->pVideoDecodInstance.gsVDecOutput, omx_private->pVideoDecodInstance.pVdec_Instance)) < 0 )
		{
			LOGE( "[VDEC_DECODE] [Err:%ld (%d)] video decode", ret, -RETCODE_CODEC_EXIT );

			VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
#if defined(ANDROID_USE_GRALLOC_BUFFER) && defined(MOVE_HW_OPERATION)
			if(gralloc_copy_mode == COPY_START)
				omx_private->gralloc_info.grallocModule->unlock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule, (buffer_handle_t)grallocHandle);
#endif
			return;
		}

#if defined(ANDROID_USE_GRALLOC_BUFFER) && defined(MOVE_HW_OPERATION)
		if(gralloc_copy_mode == COPY_START)
		{
			if(0 <= move_data_using_scaler(openmaxStandComp, outPort->sPortParam.format.video.nFrameWidth, outPort->sPortParam.format.video.nFrameHeight,
											omx_private->gralloc_info.m_pDispOut[PA][0], omx_private->gralloc_info.m_pDispOut[PA][1],
											omx_private->gralloc_info.m_pDispOut[PA][2], omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode, pGrallocAddr, WAIT_RESPOND))
			{
				gralloc_copy_mode = COPY_DONE;
				output_len = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3 / 2;
			}
			else
			{
				gralloc_copy_mode = COPY_FAILED;
			}

			GBUG_MSG("gralloc unlock :: mode = %d, len = %ld", gralloc_copy_mode, output_len);
			omx_private->gralloc_info.grallocModule->unlock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule, (buffer_handle_t)grallocHandle);
		}
#endif

		if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL)
		{
			// Current input stream should be used next time.
			if(omx_private->ConsecutiveBufferFullCnt++ > MAX_CONSECUTIVE_VPU_BUFFER_FULL_COUNT) {
				LOGE("VPU_DEC_BUF_FULL");
				omx_private->ConsecutiveBufferFullCnt = 0;
				VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, -RETCODE_CODEC_EXIT);
				return;
			}

			if (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS) {
				decode_result = 0; // display Index : processed.
				omx_private->bDelayedDecodeOut = OMX_TRUE;
			}
			else
			{
				decode_result = 1; // display Index : not processsed.
				omx_private->bDelayedDecodeOut = OMX_FALSE;
			}
		}
		else
		{
			omx_private->bDelayedDecodeOut = OMX_FALSE;
			omx_private->ConsecutiveBufferFullCnt = 0;

			if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS)
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
				if((omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode != omx_private->i_skip_scheme_level) || (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx == -2))
					omx_private->i_skip_count--;

				if(omx_private->i_skip_count < 0)
					omx_private->i_skip_count = omx_private->i_skip_interval;
				break;
		}

		// resolution change detection
		if(1)
		{
			OMX_U32 width, height;

			width = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iWidth;
			height = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iHeight;

			if( width > 0 && height > 0 && (width*height < 2048*2048) )
			{
				if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_AVC || omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MVC)
				{
					width -= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropLeft;
					width -= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropRight;
					height -= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropBottom;
					height -= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropTop;
				}

				if(outPort->sPortParam.format.video.nFrameWidth != width ||
				   outPort->sPortParam.format.video.nFrameHeight != height)
				{
					LOGI("%d - %d - %d, %d - %d - %d", omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iWidth, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropLeft, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropRight,
											omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iHeight, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropTop, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_CropInfo.m_iCropBottom);
					LOGI("Resolution change detected (%ldx%ld ==> %ldx%ld)",
							outPort->sPortParam.format.video.nFrameWidth,
							outPort->sPortParam.format.video.nFrameHeight,
							width,
							height);

					pInputBuffer->nFilledLen = 0;
					pOutputBuffer->nFilledLen = 0;
					(*(omx_private->callbacks->EventHandler))(
											openmaxStandComp,
											omx_private->callbackData,
										  OMX_EventError,
										  OMX_ErrorStreamCorrupt,
										  0,
										  NULL);
					return ;
				}
			}
		}

		// Update TimeStamp!!
		if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS
			&& omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
		{
			if( omx_private->bDecIndexOutput == OMX_TRUE )
				omx_private->bWaitNewBuffer = OMX_TRUE;

			//frame-rate update (AVG)
			omx_private->frameCount++;

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
			dec_disp_info_tmp.m_iFrameType 			= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPicType;

			if (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPicType == 0 )
				pOutputBuffer->nFlags |= OMX_BUFFERFLAG_DECODED_PIC_TYPE;
			else
				pOutputBuffer->nFlags &= ~OMX_BUFFERFLAG_DECODED_PIC_TYPE;

			dec_disp_info_tmp.m_iPicStructure 		= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPictureStructure;
			dec_disp_info_tmp.m_iRvTimeStamp 		= 0;
			dec_disp_info_tmp.m_iM2vFieldSequence   = 0;
			dec_disp_info_tmp.m_iFrameSize 			= omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iConsumedBytes;// gsCDmxOutput.m_iPacketSize;
			dec_disp_info_tmp.m_iFrameDuration 		= 2;

			switch( omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat)
			{
				case STD_RV:
					dec_disp_info_tmp.m_iRvTimeStamp = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iRvTimestamp;
					if (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS )
					{
						omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
						disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl),
							(void*)&dec_disp_info_tmp, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_input), omx_private);
					}

					break;

				case STD_MVC:
				case STD_AVC:
					if ( ( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPictureStructure == 0x3 )
//						|| omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame )
						|| ((omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPictureStructure  ==1) && (omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iInterlace ==0)) )
					{
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
					}

/*
					if( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0)
					{
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_ODD_FIRST_FRAME;
					}
*/
					dec_disp_info_tmp.m_iM2vFieldSequence = 0;
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl), (void*)&dec_disp_info_tmp, (void*)&omx_private->pVideoDecodInstance.dec_disp_info_input, omx_private);
					break;

				case STD_MPEG2:
					if( dec_disp_info_tmp.m_iPicStructure != 3 )
					{
						dec_disp_info_tmp.m_iFrameDuration = 1;
					}
					else if( omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iInterlace == 0 )
					{
						if( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iRepeatFirstField == 0 )
							dec_disp_info_tmp.m_iFrameDuration = 2;
						else
							dec_disp_info_tmp.m_iFrameDuration = ( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0 )?4:6;
					}
					else
					{
						//pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
						/* interlaced sequence */
						if( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame == 0 )
							dec_disp_info_tmp.m_iFrameDuration = 2;
						else
							dec_disp_info_tmp.m_iFrameDuration = ( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iRepeatFirstField == 0 )?2:3;
					}

					if ( ( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vProgressiveFrame == 0 && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPictureStructure == 0x3 )
						|| omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iInterlacedFrame )
					{
						//LOGD("Interlaced Frame!!!");
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_INTERLACED_FRAME;
					}

					if( omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iTopFieldFirst == 0)
					{
						//LOGD("Odd First Frame!!!");
						pOutputBuffer ->nFlags |= OMX_BUFFERFLAG_ODD_FIRST_FRAME;
					}

					dec_disp_info_tmp.m_iM2vFieldSequence = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence;
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameRate = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vFrameRate;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl), (void*)&dec_disp_info_tmp, (void*)&omx_private->pVideoDecodInstance.dec_disp_info_input, omx_private);
					break;

				default:
					dec_disp_info_tmp.m_iM2vFieldSequence = 0;
					omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
					disp_pic_info( CVDEC_DISP_INFO_UPDATE, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl), (void*)&dec_disp_info_tmp, (void*)&omx_private->pVideoDecodInstance.dec_disp_info_input, omx_private);
					break;
			}
			DPRINTF_DEC("IN-Buffer :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						pInputBuffer->pBuffer[0], pInputBuffer->pBuffer[1], pInputBuffer->pBuffer[2], pInputBuffer->pBuffer[3], pInputBuffer->pBuffer[4],
						pInputBuffer->pBuffer[5], pInputBuffer->pBuffer[6], pInputBuffer->pBuffer[7], pInputBuffer->pBuffer[8], pInputBuffer->pBuffer[9]);
			//current decoded frame info
			DPRINTF_DEC( "[In - %s][N:%4d][LEN:%6d][RT:%8d] [DecoIdx:%2d][DecStat:%d][FieldSeq:%d][TR:%8d] ",
							print_pic_type(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, dec_disp_info_tmp.m_iFrameType, dec_disp_info_tmp.m_iPicStructure),
							omx_private->pVideoDecodInstance.video_dec_idx, pInputBuffer->nFilledLen, (int)(pInputBuffer->nTimeStamp/1000),
							omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
							omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iRvTimestamp);
		}
		else
		{
			if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS_FIELD_PICTURE)
			{
#ifdef TS_TIMESTAMP_CORRECTION
				if( ((omx_private->pVideoDecodInstance.dec_disp_info_ctrl).m_iFmtType  == CONTAINER_TYPE_MPG) ||((omx_private->pVideoDecodInstance.dec_disp_info_ctrl).m_iFmtType  == CONTAINER_TYPE_TS) )
#else//TS_TIMESTAMP_CORRECTION
				if( (omx_private->pVideoDecodInstance.dec_disp_info_ctrl).m_iFmtType  == CONTAINER_TYPE_MPG )
#endif//TS_TIMESTAMP_CORRECTION
				{
					if( dec_disp_info_tmp.m_iTimeStamp <= omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS )
						dec_disp_info_tmp.m_iTimeStamp = omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS + ((omx_private->pVideoDecodInstance.gsPtsInfo.m_iPTSInterval * omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration) >> 1);
					omx_private->pVideoDecodInstance.gsPtsInfo.m_iLatestPTS = dec_disp_info_tmp.m_iTimeStamp;
					omx_private->pVideoDecodInstance.gsPtsInfo.m_iRamainingDuration = 1;
				}
			}
			DPRINTF_DEC("IN-Buffer :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
						pInputBuffer->pBuffer[0], pInputBuffer->pBuffer[1], pInputBuffer->pBuffer[2], pInputBuffer->pBuffer[3], pInputBuffer->pBuffer[4],
						pInputBuffer->pBuffer[5], pInputBuffer->pBuffer[6], pInputBuffer->pBuffer[7], pInputBuffer->pBuffer[8], pInputBuffer->pBuffer[9]);
			DPRINTF_DEC( "[Err In - %s][N:%4d][LEN:%6d][RT:%8d] [DecoIdx:%2d][DecStat:%d][FieldSeq:%d][TR:%8d] ",
							print_pic_type(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, dec_disp_info_tmp.m_iFrameType, dec_disp_info_tmp.m_iPicStructure),
							omx_private->pVideoDecodInstance.video_dec_idx, pInputBuffer->nFilledLen, (int)(pInputBuffer->nTimeStamp/1000),
							omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
							omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iM2vFieldSequence, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iRvTimestamp);
		}

		if( omx_private->frameSearchOrSkip_flag
			&& omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0
			//&& omx_private->gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS)
			&& (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS || omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS_FIELD_PICTURE))
		{
			// frameType - 0: unknown, 1: I-frame, 2: P-frame, 3:B-frame
			int frameType = get_frame_type_for_frame_skipping( omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat,
															omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPicType,
															omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iPictureStructure );

			if( omx_private->pVideoDecodInstance.gsVDecInput.m_iFrameSearchEnable )
			{
				omx_private->frameSearchOrSkip_flag = 2; //I-frame Search Mode disable and B-frame Skip Mode enable
				DPRINTF_DEC_STATUS("[SEEK] I-frame Search Mode disable and B-frame Skip Mode enable");
			}
			else if( omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode == VDEC_SKIP_FRAME_ONLY_B )
			{
				if(frameType != 3)
				{
					DPRINTF_DEC_STATUS( "[SEEK] B-frame Skip Mode Disable after P-frame decoding!!!");
					omx_private->frameSearchOrSkip_flag = 0; //B-frame Skip Mode disable
					omx_private->skipFrameNum = (omx_private->I_frame_search_mode == AVC_IDR_PICTURE_SEARCH_MODE)
					                          ? omx_private->numSkipFrame - 3 : 0;
				}
			}
			else if( omx_private->pVideoDecodInstance.gsVDecInput.m_iSkipFrameMode == VDEC_SKIP_FRAME_EXCEPT_I )
			{
				if(frameType == 1)
				{
					DPRINTF_DEC_STATUS( "[SEEK] B-frame Skip Mode Disable after P-frame decoding!!!");
					omx_private->frameSearchOrSkip_flag = 0; //B-frame Skip Mode disable
					omx_private->skipFrameNum = (omx_private->I_frame_search_mode == AVC_IDR_PICTURE_SEARCH_MODE)
					                          ? omx_private->numSkipFrame - 3 : 0;
				}
			}
		}

		if (CONTAINER_TYPE_TS != omx_private->pVideoDecodInstance.container_type)
		{
			omx_private->inputCurrBuffer = pInputBuffer->pBuffer;
			omx_private->inputCurrLength = pInputBuffer->nFilledLen;
		}
		//////////////////////////////////////////////////////////////////////////////////////////

		if (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS ||
		    (omx_private->bDecIndexOutput == OMX_TRUE &&
			 omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS &&
			 omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0))
		{
			int dispOutIdx;
			int dispMVCOutIdx=0;
			dec_disp_info_t *pdec_disp_info = NULL;

			omx_private->ConsecutiveVdecFailCnt = 0; //Reset Consecutive Vdec Fail Counting B060955

#ifdef ANDROID_USE_GRALLOC_BUFFER
			if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr)
			{
				if( omx_private->bDecIndexOutput == OMX_TRUE && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
					dispOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;
				else
					dispOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;

				if( NULL != plat_priv )
				{
					GBUG_MSG("          P copy");
					plat_priv->unique_addr = omx_private->mCodecStart_ms;
					if(gralloc_copy_mode == COPY_NONE)
						plat_priv->copied = 0;
					else
						plat_priv->copied = 1;
					plat_priv->width = outPort->sPortParam.format.video.nFrameWidth;
					plat_priv->height = outPort->sPortParam.format.video.nFrameHeight;
					if(omx_private->pVideoDecodInstance.gsVDecInit.m_bCbCrInterleaveMode)
						plat_priv->format = OMX_COLOR_FormatYUV420SemiPlanar;
					else
					{
				#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG) || defined(JPEG_DECODE_FOR_MJPEG)
						if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG)
						{
							//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )
							if(omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iMjpg_sourceFormat == 1)
								plat_priv->format = OMX_COLOR_FormatYUV422Planar;
							else
								plat_priv->format = OMX_COLOR_FormatYUV420Planar;
						}
						else
				#endif
							plat_priv->format = OMX_COLOR_FormatYUV420Planar;
					}
					if( omx_private->bDecIndexOutput == OMX_TRUE && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
					{
						plat_priv->offset[0] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][0];
						plat_priv->offset[1] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][1];
						plat_priv->offset[2] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][2];
						decode_result = 2;
						#if defined(_TCC8920_) 
						dispMVCOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDecoded;
						#endif
					}
					else
					{
						plat_priv->offset[0] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][0];
						plat_priv->offset[1] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][1];
						plat_priv->offset[2] = (unsigned int)omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][2];
						#if defined(_TCC8920_) 
						dispMVCOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_MvcPicInfo.m_iViewIdxDisplay;
						#endif
					}
					sprintf(plat_priv->name, "video");
					plat_priv->name[5] = 0;
					plat_priv->optional_info[0] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
					plat_priv->optional_info[1] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight;
					plat_priv->optional_info[2] = outPort->sPortParam.format.video.nFrameWidth;
					plat_priv->optional_info[3] = outPort->sPortParam.format.video.nFrameHeight;
					plat_priv->optional_info[4] = 0; //buffer_id // there is no need to put it.
					plat_priv->optional_info[5] = 0; //timeStamp	// there is no need to put it.
					plat_priv->optional_info[6] = 0; //curTime
					plat_priv->optional_info[7] = 0; //flags		// there is no need to put it.
					plat_priv->optional_info[8] = 0; //framerate	// there is no need to put it.
					plat_priv->optional_info[9] = dispMVCOutIdx; 
					pOutputBuffer->pPlatformPrivate = plat_priv;
					output_len = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3 / 2;
					GBUG_MSG("copied ? private data %p %dx%d, %d, 0x%x-0x%x-0x%x", (void*)plat_priv, plat_priv->width, plat_priv->height, plat_priv->format,
								plat_priv->offset[0], plat_priv->offset[1], plat_priv->offset[2]);
					omx_private->gralloc_info.grallocModule->unlock((gralloc_module_t const *) omx_private->gralloc_info.grallocModule, (buffer_handle_t)grallocHandle);
				}

				{
					if( omx_private->bDecIndexOutput == OMX_TRUE && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
					{
						GBUG_MSG("VPU address copy 2:: 0x%p-0x%p-0x%p - 0x%p-0x%p-0x%p",
										omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][0], omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][1], omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][2],
										omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][0], omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][1], omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][2]);

						omx_private->gralloc_info.m_pDispOut[PA][0] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][0];
						omx_private->gralloc_info.m_pDispOut[PA][1] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][1];
						omx_private->gralloc_info.m_pDispOut[PA][2] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][2];

						omx_private->gralloc_info.m_pDispOut[VA][0] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][0];
						omx_private->gralloc_info.m_pDispOut[VA][1] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][1];
						omx_private->gralloc_info.m_pDispOut[VA][2] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][2];
					}
					else
					{
						GBUG_MSG("VPU address copy 1:: 0x%p-0x%p-0x%p - 0x%p-0x%p-0x%p",
										omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][0], omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][1], omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][2],
										omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][0], omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][1], omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][2]);

						omx_private->gralloc_info.m_pDispOut[PA][0] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][0];
						omx_private->gralloc_info.m_pDispOut[PA][1] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][1];
						omx_private->gralloc_info.m_pDispOut[PA][2] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][2];

						omx_private->gralloc_info.m_pDispOut[VA][0] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][0];
						omx_private->gralloc_info.m_pDispOut[VA][1] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][1];
						omx_private->gralloc_info.m_pDispOut[VA][2] = omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][2];
					}
				}

	#ifdef COMPARE_TIME_LOG
				{
					end = clock();

					dec_time[time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
					total_dec_time += dec_time[time_cnt];
					if(time_cnt != 0 && time_cnt % 29 == 0)
					{
						LOGD("VDEC_TIME %d ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
							total_dec_time/total_frm, dec_time[0], dec_time[1], dec_time[2], dec_time[3], dec_time[4], dec_time[5], dec_time[6], dec_time[7], dec_time[8], dec_time[9],
							dec_time[10], dec_time[11], dec_time[12], dec_time[13], dec_time[14], dec_time[15], dec_time[16], dec_time[17], dec_time[18], dec_time[19],
							dec_time[20], dec_time[21], dec_time[22], dec_time[23], dec_time[24], dec_time[25], dec_time[26], dec_time[27], dec_time[28], dec_time[29]);
						time_cnt = 0;
					}
					else{
						time_cnt++;
					}
					total_frm++;
				}
	#endif
			}
			else
#endif
			{
				if( omx_private->bDecIndexOutput == OMX_TRUE &&
				omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0){
				dispOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx;

					/* physical address */
					for(i=0;i<3;i++)
					memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[PA][i], 4);

					/* logical address */
					for(i=3;i<6;i++)
					memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->pVideoDecodInstance.gsVDecOutput.m_pCurrOut[VA][i-3], 4);

					decode_result = 2;
				}
				else {
				dispOutIdx = omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx;

					/* physical address */
					for(i=0;i<3;i++)
					memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[PA][i], 4);

					/* logical address */
					for(i=3;i<6;i++)
					memcpy(pOutputBuffer->pBuffer+i*4, &omx_private->pVideoDecodInstance.gsVDecOutput.m_pDispOut[VA][i-3], 4);
				}

			*((OMX_U32*)(pOutputBuffer->pBuffer+24)) = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicWidth;
			*((OMX_U32*)(pOutputBuffer->pBuffer+28)) = omx_private->pVideoDecodInstance.gsVDecOutput.m_pInitialInfo->m_iPicHeight;

				*((OMX_U32*)(pOutputBuffer->pBuffer+32)) = outPort->sPortParam.format.video.nFrameWidth;
				*((OMX_U32*)(pOutputBuffer->pBuffer+36)) = outPort->sPortParam.format.video.nFrameHeight;

				/* physical address 3 ea, logical address 3 ea, width and height of VPU output, width and height of actual frame */
				output_len = 4*6 + 8 + 8;
			}

			//Get TimeStamp!!
			{
				omx_private->pVideoDecodInstance.dec_disp_info_input.m_iFrameIdx = dispOutIdx;
				disp_pic_info( CVDEC_DISP_INFO_GET, (void*)&(omx_private->pVideoDecodInstance.dec_disp_info_ctrl), (void*)&pdec_disp_info, (void*)&omx_private->pVideoDecodInstance.dec_disp_info_input, omx_private);

				if( pdec_disp_info != (dec_disp_info_t*)0 )
				{
#ifdef JPEG_DECODE_FOR_MJPEG
					if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_MJPG)
					{
						pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
					}
					else
#endif
					if(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat == STD_RV)
					{
						pOutputBuffer->nTimeStamp = (OMX_TICKS)pdec_disp_info->m_iRvTimeStamp * 1000;
					}
					else// if(omx_private->gsVDecInit.m_iBitstreamFormat == STD_MPEG2)
					{
						pOutputBuffer->nTimeStamp = (OMX_TICKS)pdec_disp_info->m_iTimeStamp * 1000; //pdec_disp_info->m_iM2vFieldSequence * 1000;
					}


					if(omx_private->pVideoDecodInstance.gsVDecInit.m_bEnableUserData)
					{
						print_user_data((unsigned char*)omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_UserDataAddress[VA]);
					}

					DPRINTF_DEC( "[Out - %s][N:%4d][LEN:%6d][RT:%8d] [DispIdx:%2d][OutStat:%d][FieldSeq:%d][TR:%8d] ",
									print_pic_type(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, pdec_disp_info->m_iFrameType, pdec_disp_info->m_iPicStructure),
									omx_private->pVideoDecodInstance.video_dec_idx, pdec_disp_info->m_iFrameSize, pdec_disp_info->m_iTimeStamp,
									omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus,
									pdec_disp_info->m_iM2vFieldSequence, pdec_disp_info->m_iRvTimeStamp);
				}
				else
				{
					//exception process!! temp!!
					pOutputBuffer->nTimeStamp = pInputBuffer->nTimeStamp;
				}
			}

			if( omx_private->bDecIndexOutput == OMX_FALSE && omx_private->bWaitKeyFrameOut == OMX_TRUE )
			{
				int output_type = 0;
				if( pdec_disp_info )
					output_type = get_frame_type_for_frame_skipping(omx_private->pVideoDecodInstance.gsVDecInit.m_iBitstreamFormat, pdec_disp_info->m_iFrameType, pdec_disp_info->m_iPicStructure);

				if( output_type == 1 )
					omx_private->bWaitKeyFrameOut = OMX_FALSE;
#if 0//JS Baek
				else
				{
					if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &dispOutIdx, NULL, omx_private->pVideoDecodInstance.pVdec_Instance) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", dispOutIdx, ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
						return;
					}
					decode_result = 3;

					LOGE("drop frame after seek (idx: %d / type: %d)", dispOutIdx, output_type);
				}
#endif
			}

			if( omx_private->bWaitKeyFrameOut == OMX_FALSE )
			{
#ifdef HAVE_ANDROID_OS
	#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
				if(omx_private->max_fifo_cnt != 0)
				{
					// to change video output device
					int before_bOutputMode	 = omx_private->bOutputMode;
					char value[PROPERTY_VALUE_MAX];
					property_get("tcc.sys.output_mode_detected", value, "");

					if( atoi(value) != 0)
						omx_private->bOutputMode = 1;
					else
						omx_private->bOutputMode = 0;

					property_get("tcc.solution.mbox", value, "");
					if(atoi(value)){
						//always set 'bOutputMode = 1' in case of STB even though output_mode_detected is '0' 
						omx_private->bOutputMode = 1;
					}

					if(before_bOutputMode !=omx_private->bOutputMode)
					{
					// >> vpu buffer all clear
						while(omx_private->in_index != omx_private->out_index)
						{
							//DPRINTF_DEC_STATUS("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
							LOGE("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
							if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL, (omx_private->pVideoDecodInstance.pVdec_Instance)) ) < 0 )
							{
								LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
								VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
								return;
							}
							omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
						}
						omx_private->in_index = omx_private->out_index = omx_private->frm_clear = 0;
						#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
						omx_private->used_fifo_count = 0;
						#endif
					// << vpu buffer all clear
					}

					//property_get("tcc.video.vsync.enable", value, "0");
					if(/*(atoi(value) != 0) &&*/ omx_private->bOutputMode)
					{
						omx_private->Display_index[omx_private->in_index] = dispOutIdx;
						omx_private->Display_Buff_ID[omx_private->in_index] = omx_private->buffer_unique_id;

		#ifdef ANDROID_USE_GRALLOC_BUFFER
						if( plat_priv != NULL) {
							plat_priv->optional_info[4] = omx_private->buffer_unique_id;
						}
		#else
						*(unsigned int*)(&pOutputBuffer->pBuffer[output_len]) = omx_private->buffer_unique_id;
							output_len += 4;
		#endif
						DPRINTF_DEC_STATUS("DispIdx Queue %d", omx_private->Display_index[omx_private->in_index]);
						omx_private->in_index = (omx_private->in_index + 1) % omx_private->max_fifo_cnt;
						omx_private->used_fifo_count++;
						omx_private->buffer_unique_id++;

						//LOGD("### in(%d), out(%d), addr(%x), addr(%x)", omx_private->in_index, omx_private->out_index, omx_private->gsVDecOutput.m_pDispOut[PA][0], omx_private->gsVDecOutput.m_pDispOut[VA][0] ) ;
						//LOGE("### in(%d), out(%d), used(%d), max(%d)", omx_private->in_index, omx_private->out_index, used_fifo_count, omx_private->max_fifo_cnt) ;
						if(omx_private->used_fifo_count == omx_private->max_fifo_cnt)
						{
							int cleared_buff_id;
							int loopCount = 0;
							int cleared_buff_count = 0;
							//LOGE("### buffer clear start",tmp) ;
							cleared_buff_id = ioctl(omx_private->g_hFb, TCC_LCDC_VIDEO_GET_DISPLAYED, NULL) ;  // TCC_LCDC_HDMI_GET_DISPLAYED

							//LOGE("### displayed buffer info: num(%d), buffid(%d)",displayed_num, (displayed_num>0)?dispBuffId[displayed_num-1]:errno) ;
							if(cleared_buff_id < 0)
							{
								//LOGE("### buffer clear by skip mode %d", errno) ;
								DPRINTF_DEC_STATUS("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
								ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &omx_private->Display_index[omx_private->out_index], NULL, omx_private->pVideoDecodInstance.pVdec_Instance);
								if(ret  < 0 )
								{
									LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
									VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
									return;
								}

								omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
								omx_private->used_fifo_count-- ;
							}
							else
							{
								//LOGE("### normal buffer clear ",tmp) ;
								ret = clear_vpu_buffer(cleared_buff_id, omx_private);
								if(ret  < 0 )
								{
									LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
									VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
									return;
								}
							}
						}
					}
					else
					{
						omx_private->Display_index[omx_private->in_index] = dispOutIdx;
						DPRINTF_DEC_STATUS("DispIdx Queue %d", omx_private->Display_index[omx_private->in_index]);
						omx_private->in_index = (omx_private->in_index + 1) % omx_private->max_fifo_cnt;

						if(omx_private->in_index == 0 && !omx_private->frm_clear)
							omx_private->frm_clear = 1;

						if(omx_private->frm_clear)
						{
		//					LOGE("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
							if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &omx_private->Display_index[omx_private->out_index], NULL, omx_private->pVideoDecodInstance.pVdec_Instance) ) < 0 )
							{
								LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
								VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
								return;
							}
							omx_private->out_index = (omx_private->out_index + 1) %omx_private->max_fifo_cnt;
						}
					}
				}
				else
				{
					//DPRINTF_DEC_STATUS("@ DispIdx Queue %d", omx_private->Display_index[omx_private->in_index]);
					if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &dispOutIdx, NULL, omx_private->pVideoDecodInstance.pVdec_Instance) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", dispOutIdx, ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
						return;
					}
				}
	#else
			/*ZzaU :: Clear decoded frame-buffer according with sequence-order after it was used!!*/
			if(omx_private->max_fifo_cnt != 0)
			{
				// to change video output device
				int before_bOutputMode	 = omx_private->bOutputMode;
				char value[PROPERTY_VALUE_MAX];
				property_get("tcc.sys.output_mode_detected", value, "");

				if( atoi(value) != 0)
					omx_private->bOutputMode = 1;
				else
					omx_private->bOutputMode = 0;

				if(before_bOutputMode !=omx_private->bOutputMode)
				{
				// >> vpu buffer all clear
					while(omx_private->in_index != omx_private->out_index)
					{
						//DPRINTF_DEC_STATUS("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
						LOGE("DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
						if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL, (omx_private->pVideoDecodInstance.pVdec_Instance)) ) < 0 )
						{
							LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
							VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
							return;
						}
						omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
					}
					omx_private->in_index = omx_private->out_index = omx_private->frm_clear = 0;
					#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
					omx_private->used_fifo_count = 0;
					#endif
				// << vpu buffer all clear
				}

				omx_private->Display_index[omx_private->in_index] = dispOutIdx;
				DPRINTF_DEC_STATUS("DispIdx Queue %d", omx_private->Display_index[omx_private->in_index]);
				omx_private->in_index = (omx_private->in_index + 1) % omx_private->max_fifo_cnt;

				if(omx_private->in_index == 0 && !omx_private->frm_clear)
					omx_private->frm_clear = 1;

				if(omx_private->frm_clear)
				{
					DPRINTF_DEC_STATUS("Normal DispIdx Clear %d", omx_private->Display_index[omx_private->out_index]);
					if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &(omx_private->Display_index[omx_private->out_index]), NULL, (omx_private->pVideoDecodInstance.pVdec_Instance)) ) < 0 )
					{
						LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", omx_private->Display_index[omx_private->out_index], ret );
						VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
						return;
					}

					omx_private->out_index = (omx_private->out_index + 1) % omx_private->max_fifo_cnt;
				}
			}
			else
			{
				DPRINTF_DEC_STATUS("@ DispIdx Queue %d", omx_private->Display_index[omx_private->in_index]);
				if( ( ret = omx_private->pVideoDecodInstance.gspfVDec( VDEC_BUF_FLAG_CLEAR, NULL, &dispOutIdx, NULL, omx_private->pVideoDecodInstance.pVdec_Instance) ) < 0 )
				{
					LOGE( "[VDEC_BUF_FLAG_CLEAR] Idx = %d, ret = %ld", dispOutIdx, ret );
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
					return;
				}
			}
	#endif
			}
#endif
			omx_private->pVideoDecodInstance.video_dec_idx++;
		}
		else
		{
			DPRINTF_DEC( "[VDEC_DECODE] NO-OUTPUT!! m_iDispOutIdx = %d, m_iDecodedIdx = %d, m_iOutputStatus = %d, m_iDecodingStatus = %d, m_iNumOfErrMBs = %d",
											omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx,
											omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus, omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus,
											omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iNumOfErrMBs);

			if(omx_private->frameSearchOrSkip_flag == 1)
			{
				omx_private->ConsecutiveVdecFailCnt++;
				if(omx_private->ConsecutiveVdecFailCnt >= omx_private->maxConsecutiveVdecFailCnt)
				{
					LOGE("[VDEC_ERROR]m_iOutputStatus %d %dtimes!!!\n",omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iOutputStatus,omx_private->ConsecutiveVdecFailCnt);
					omx_private->ConsecutiveVdecFailCnt = 0; // Reset Consecutive Vdec Fail Counting B060955
					VideoDecErrorProcess(openmaxStandComp, pInputBuffer, pOutputBuffer, ret);
					return;
				}
			}

#ifdef ENABLE_DECODE_ONLY_MODE_AVC
			if(omx_private->bSetDecodeOnlyMode == OMX_TRUE && omx_private->I_frame_search_mode == AVC_NONIDR_PICTURE_SEARCH_MODE)
			{
				if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0 && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
				{
					if(omx_private->skipFrameNum > 3) {
						omx_private->skipFrameNum -= 3;
					}
					omx_private->decodeOnlyErrNum++;
				}
			}
#endif

			output_len = 0;
		}

#ifdef ENABLE_DECODE_ONLY_MODE_AVC
		if(omx_private->bSetDecodeOnlyMode == OMX_TRUE)
		{
			LOGINFO("decodeOnlyErrNum = %d, skipFrameNum = %d", omx_private->decodeOnlyErrNum, omx_private->skipFrameNum);
			if(omx_private->skipFrameNum == omx_private->numSkipFrame || omx_private->decodeOnlyErrNum == MAX_DEC_ONLY_ERR_THRESHOLD)
			{
				// Finish decode only mode.
				omx_private->bSetDecodeOnlyMode = OMX_FALSE;
				omx_private->skipFrameNum = 0;
				omx_private->decodeOnlyErrNum = 0;
				LOGMSG("Finish Decode Only frame");
			}
			else
			{
				if(!(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDispOutIdx < 0 && omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx < 0))
				{
					omx_private->skipFrameNum++;
				}

				pInputBuffer->nFlags |= OMX_BUFFERFLAG_DECODEONLY;
				LOGINFO("Decode Only frame - skipFrameNum(%d)", omx_private->skipFrameNum);
			}
		}
#endif

		internalOutputFilled = 1;

#ifdef ANDROID_USE_GRALLOC_BUFFER
		if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType != GrallocPtr)
#endif
		{
			if(omx_private->bThumbnailMode && omx_private->pThumbnailBuff == 0)
			{
			if(omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_SUCCESS
				&& omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iDecodedIdx >= 0)
				{
		#ifdef ICS_THUMBNAIL_CREATION
					CreateThumbFrame(openmaxStandComp, pOutputBuffer->pBuffer, &pOutputBuffer->nFilledLen, OMX_FALSE);
		#else
					pOutputBuffer->pBuffer = (OMX_U8 *)CreateThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, OMX_FALSE);
		#endif
					return;
				}
			}
		}

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
				// If data comes from the TCC parser, doesn't need this routine
				if (!(omx_private->extractorType == OMX_BUFFERFLAG_EXTRACTORTYPE_TCC) && decode_result == 3)
				{
					if(omx_private->bThumbnailMode && omx_private->pThumbnailBuff==0) // SSG
					{
						omx_private->displaying_error_count--;

						if (omx_private->displaying_error_count == 0)
						{
							// we never display this frame
							// so, have to fill the thumbnail image with black
	#ifdef ICS_THUMBNAIL_CREATION
							CreateThumbFrame(openmaxStandComp, pOutputBuffer->pBuffer, &pOutputBuffer->nFilledLen, OMX_TRUE);
	#else
							pOutputBuffer->pBuffer = (OMX_U8 *)CreateThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, OMX_TRUE);
	#endif
						}
						else
						{
							return;
						}
					}
				}
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
						if ((CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
						&& (omx_private->inputCurrLength > (omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iConsumedBytes + input_offset)))
						{
							nLen += omx_private->pVideoDecodInstance.gsVDecOutput.m_DecOutInfo.m_iConsumedBytes + input_offset;
						}
						else
						{
							nLen += omx_private->inputCurrLength;
						}
					}
				DPRINTF_DEC("----------------------> inputCurrLength : %ld, nLen : %ld\n", omx_private->inputCurrLength, nLen);
				if (nLen < 0) {
					LOGE("----> A general error or simply frame not decoded?\n");
				}
				if ( nLen >= 0 && internalOutputFilled)
				{
					omx_private->inputCurrBuffer += nLen;
					omx_private->inputCurrLength -= nLen;
					pInputBuffer->nFilledLen -= nLen;

#if defined(_TCC9300_) || defined(_TCC8800_) || defined(_TCC8920_) // until vpu bug is fixed
					if((CONTAINER_TYPE_TS == omx_private->pVideoDecodInstance.container_type)
					&& (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC))
					{
						if(omx_private->bUseFrameDefragmentation == OMX_FALSE)
							pInputBuffer->nFilledLen = 0;
					}
#endif

					//Buffer is fully consumed. Request for new Input Buffer
					if(pInputBuffer->nFilledLen == 0)
					{
						omx_private->isNewBuffer = 1;
						DPRINTF_DEC("----------------------> New InputBuffer!!");
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
	#ifdef ANDROID_USE_GRALLOC_BUFFER
					if(omx_private->gralloc_info.PortBuffers[OMX_BASE_FILTER_OUTPUTPORT_INDEX].BufferType == GrallocPtr)
					{
						pOutputBuffer->nFilledLen = output_len;
					}
					else
					{
			#if !defined(USE_EXTERNAL_BUFFER)
				#ifdef HARDWARE_CODEC 	/*ZzaU :: VPU output is seperated yuv420 include gap.*/
						TCC_PLATFORM_PRIVATE_PMEM_INFO *pmemInfoPtr;

						pmemInfoPtr = (TCC_PLATFORM_PRIVATE_PMEM_INFO *) pOutputBuffer->pPlatformPrivate;
					#ifdef TCC_VIDEO_DISPLAY_BY_VSYNC_INT
						memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, output_len);
					#else
						memcpy(pmemInfoPtr->offset, pOutputBuffer->pBuffer, sizeof(int)*6);
					#endif
				#endif
			#endif
						if ( omx_private->bThumbnailMode ) {
			#ifdef ICS_THUMBNAIL_CREATION
							CreateThumbFrame(openmaxStandComp, pOutputBuffer->pBuffer, &pOutputBuffer->nFilledLen, OMX_FALSE);
			#else
							pOutputBuffer->pBuffer = (OMX_U8 *)CreateThumbFrame(openmaxStandComp, &pOutputBuffer->nFilledLen, OMX_FALSE);
			#endif
						}
						else
						{
#if defined(USE_EXTERNAL_BUFFER) && defined(ICS_THUMBNAIL_CREATION)
							//To support video-editor!!
							if(omx_private->isRemotePlayerPlay == OMX_FALSE)
							    CreateThumbFrame(openmaxStandComp, pOutputBuffer->pBuffer, &pOutputBuffer->nFilledLen, OMX_FALSE);

#endif
							if(omx_private->isRemotePlayerPlay == OMX_TRUE)
								pOutputBuffer->nFilledLen = output_len;
							else
							    pOutputBuffer->nFilledLen = outPort->sPortParam.format.video.nFrameWidth * outPort->sPortParam.format.video.nFrameHeight * 3/2;

						}
						DBUG_MSG("----------------------> New OutputBuffer!!");
					}
	#endif
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
          UpdateFrameSize (openmaxStandComp);
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
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingRV;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_H263_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingH263;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_H264_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingAVC;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MPEG4_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMPEG4;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingWMV;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV12_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingWMV_1_2;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_DIVX_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingDIVX;
		}else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MPEG2_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMPEG2;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_SORENSON_H263_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingFLV1;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MJPEG_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMJPEG;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_AVS_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingAVS;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_VPX_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingVPX;
		} else if (!strcmp((char *)pComponentRole->cRole, VIDEO_DEC_MVC_ROLE)) {
			omx_private->pVideoDecodInstance.video_coding_type = OMX_VIDEO_CodingMVC;
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
        if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingRV) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_RV_ROLE);
        }else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingH263) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_H263_ROLE);
	 	}else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_H264_ROLE);
        }else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG4_ROLE);
        }else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV_ROLE);
        }else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
          strcpy((char *)pComponentRole->cRole, VIDEO_DEC_TCC_WMV12_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingDIVX) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_DIVX_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG2) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MPEG2_ROLE);
	    } else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingFLV1 ) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_SORENSON_H263_ROLE);
		} else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingMJPEG) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MJPEG_ROLE);
		} else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingAVS) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_AVS_ROLE);
		} else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingVPX) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_VPX_ROLE);
		} else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingMVC) {
			strcpy((char *)pComponentRole->cRole, VIDEO_DEC_MVC_ROLE);
		}else {
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
		if (omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC || omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMVC)
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
		OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevel = (OMX_VIDEO_PARAM_PROFILELEVELTYPE *) ComponentParameterStructure;
		VIDEO_PROFILE_LEVEL_TYPE* pProfileLevel = NULL;
		OMX_U32 nNumberOfProfiles = 0;

		if (profileLevel->nPortIndex != 0) {
		    LOGE("Invalid port index: %ld", profileLevel->nPortIndex);
		    return OMX_ErrorUnsupportedIndex;
		}

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

		if (profileLevel->nProfileIndex >= nNumberOfProfiles) {
		    return OMX_ErrorNoMore;
		}

		profileLevel->eProfile = pProfileLevel[profileLevel->nProfileIndex].nProfile;
		profileLevel->eLevel = pProfileLevel[profileLevel->nProfileIndex].nLevel;

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
				if(omx_private->blocalPlaybackMode)
					pPortDef->format.video.eColorFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP;
				else
					pPortDef->format.video.eColorFormat = HAL_PIXEL_FORMAT_YV12;
				DBUG_MSG("pPortDef->format.video.eColorFormat(0x%x)", pPortDef->format.video.eColorFormat);
			}
			else
			{
				if(omx_private->isRemotePlayerPlay == OMX_TRUE)
					pPortDef->format.video.eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
				else
					pPortDef->format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
			}
		}
		else if ( nParamIndex == OMX_IndexParamVideoPortFormat )
		{
	        OMX_VIDEO_PARAM_PORTFORMATTYPE *pPortParam = (OMX_VIDEO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;

			pPortParam = (OMX_VIDEO_PARAM_PORTFORMATTYPE *)ComponentParameterStructure;
			if(omx_private->gralloc_info.PortBuffers[pPortParam->nPortIndex].BufferType == GrallocPtr)
			{
				if(omx_private->blocalPlaybackMode)
					pPortParam->eColorFormat = HAL_PIXEL_FORMAT_YCbCr_420_SP;
				else
					pPortParam->eColorFormat = HAL_PIXEL_FORMAT_YV12;
				DBUG_MSG("pPortParam->eColorFormat(0x%x)", pPortParam->eColorFormat);
			}
			else
			{
				if(omx_private->isRemotePlayerPlay == OMX_TRUE)
					pPortParam->eColorFormat = OMX_COLOR_FormatYUV420SemiPlanar;
				else
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
      if (!omx_private->pVideoDecodInstance.avcodecReady) {
        err = omx_videodec_component_LibInit(omx_private);
        if (err != OMX_ErrorNone) {
          return OMX_ErrorNotReady;
        }
        omx_private->pVideoDecodInstance.avcodecReady = OMX_TRUE;
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
      if (omx_private->pVideoDecodInstance.avcodecReady) {
        omx_videodec_component_LibDeinit(omx_private);
        omx_private->pVideoDecodInstance.avcodecReady = OMX_FALSE;
      }
#endif
    }
  }

  // flush all ports to start new stream
  if (message->messageType == OMX_CommandFlush && message->messageParam == OMX_ALL) {
	  omx_private->bAllPortsFlushed = OMX_TRUE;
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
		if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingRV) {
			strcpy((char *)cRole, VIDEO_DEC_RV_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingH263) {
			strcpy((char *)cRole, VIDEO_DEC_H263_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingAVC) {
			strcpy((char *)cRole, VIDEO_DEC_H264_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG4) {
			strcpy((char *)cRole, VIDEO_DEC_MPEG4_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_WMV_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingWMV_1_2) {
			strcpy((char *)cRole, VIDEO_DEC_TCC_WMV12_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingDIVX) {
			strcpy((char *)cRole, VIDEO_DEC_DIVX_ROLE);
		}
		else if(omx_private->pVideoDecodInstance.video_coding_type == OMX_VIDEO_CodingMPEG2) {
			strcpy((char *)cRole, VIDEO_DEC_MPEG2_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingFLV1 ) {
			strcpy((char *)cRole, VIDEO_DEC_SORENSON_H263_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingMJPEG) {
			strcpy((char *)cRole, VIDEO_DEC_MJPEG_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingAVS) {
			strcpy((char *)cRole, VIDEO_DEC_AVS_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingVPX) {
			strcpy((char *)cRole, VIDEO_DEC_VPX_ROLE);
		}else if (omx_private->pVideoDecodInstance.video_coding_type ==OMX_VIDEO_CodingMVC) {
			strcpy((char *)cRole, VIDEO_DEC_MVC_ROLE);
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

    case OMX_IndexConfigVideoPlayDirection :
	  omx_private->bPlayDirection = *(OMX_BOOL*)pComponentConfigStructure;
      break;

  case OMX_IndexConfigVideoOutputKeyFrameOnly:
	  omx_private->bDecIndexOutput = *(OMX_BOOL*)pComponentConfigStructure;
	  if( omx_private->bDecIndexOutput == OMX_FALSE )
		  omx_private->bWaitNewBuffer = OMX_FALSE;
	  break;

  case OMX_IndexVendorThumbnailMode:
	  DBUG_MSG("this is thumbnail mode");
	  omx_private->bThumbnailMode = *((OMX_BOOL *)pComponentConfigStructure);
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
