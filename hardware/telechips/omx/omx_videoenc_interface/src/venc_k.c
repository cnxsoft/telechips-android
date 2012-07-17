/*!
 ***********************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2009 Telechips Inc.
  Korad Bldg, 1000-12 Daechi-dong, Kangnam-Ku, Seoul, Korea					
 \endverbatim
 ***********************************************************************
 */
/*!
 ***********************************************************************
 *
 * \file
 *		venc.c
 * \date
 *		2009/07/23
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		video encoder
 *
 ***********************************************************************
 */
#ifdef HAVE_ANDROID_OS
#define LOG_TAG	"VPU_ENC_K"
#include <utils/Log.h>
	 
#include "cdk_core.h"
#include "venc.h"
	 
#include <sys/mman.h>
#include <string.h>
#include <sys/ioctl.h>
#include <mach/tcc_vpu_ioctl.h>
#include <errno.h>
#include "TCCStagefrightDefine.h"
#ifdef ENABLE_REMOTE_PLAYER
#include <cutils/properties.h>
#endif
#ifdef VPU_CLK_CONTROL
#include "vpu_clk_ctrl.h"
#endif

#define INSERT_SEQ_HEADER_IN_FORCED_IFRAME
#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
#include "TCCMemory.h"
unsigned char *seq_backup = NULL;
unsigned int seq_len = 0;
#endif

#ifdef ENABLE_RATE_CONTROL
//#define CHECK_BITRATE  //to check current output bitrate.
#ifdef CHECK_BITRATE
static unsigned int curr_bps, bps_frames, total_size, curr_fps;
#endif
#endif

#define ALIGN_LEN (4*1024)

/************************************************************************/
/* TEST and Debugging                                               								 */
/************************************************************************/
static int DEBUG_ON = 0;
#define DPRINTF(msg...)	 LOGE( ": " msg);
#define DSTATUS(msg...)	 if (DEBUG_ON) { LOGD( ": " msg);}
#define DPRINTF_FRAME(msg...) //LOGD(": " msg);

//#define VPU_FRAME_DUMP
//#define VPU_OUT_FRAME_DUMP
//#define DEBUG_TIME_LOG
#ifdef DEBUG_TIME_LOG
#include "time.h"
static unsigned int dec_time[30] = {0,};
static unsigned int time_cnt = 0;
static unsigned int total_dec_time = 0;
#endif
static unsigned int total_frm = 0;

#define TCC_VPU_INPUT_BUF_SIZE 		(1024 * 1024)

#define STABILITY_GAP (512)
// delete by shmin for M2TS
//#define REMOVE_NALSTARTCODE //We will remove NAL StartCode(4byte), because VPU output-Data already has it!!

#define VPU_ENC_NAME		"/dev/vpu_enc"
int vpu_enc_fd = -1;

static unsigned char venc_env_opened = 0;

int vpu_mgr_fd = -1;
#ifdef HAVE_ANDROID_OS
#define VPU_MGR_NAME	"/dev/vpu_mgr"
#else
#define VPU_MGR_NAME	"/dev/mem"
#endif

#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>
struct pollfd tcc_event[1];

#else
#include "venc.h"
#include "../cdk/cdk_core.h"
#include "../cdk/cdk_sys.h"
#endif

/************************************************************************/
/* STATIC MEMBERS                                                       */
/************************************************************************/
static int gsBitWorkBufSize = 0;
static codec_addr_t gsBitWorkBufAddr[3] = {0,};
//static int gsBitWorkBufDev;

static codec_addr_t gsFrameBufAddr[3] = {0,};
static unsigned int gsFrameBufSize = 0;
//static int gsFrameBufDev = -1;

static unsigned int gsMESearchBufSize = 0;
static codec_addr_t gsMESearchBufAddr[3] = {0,};
//static int gsMESearchBufDev;

static VENC_INIT_t gsVpuEncInit_Info;
static VENC_PUT_HEADER_t gsVpuEncPutHeader_Info;
static VENC_SET_BUFFER_t gsVpuEncBuffer_Info;
static VENC_ENCODE_t gsVpuEncInOut_Info;

#ifdef HAVE_ANDROID_OS
static codec_addr_t gspSeqHeaderAddr[3] = {0,};//, gspSeqHeader_phyAddr;
static unsigned int /*gsiSeqHeaderBufDev, */gsiSeqHeaderSize = 0;
static unsigned int gsiSeqHeaderCnt = 0, gsiSzSeqHeader[3] = {0,};
#else
static unsigned char* gspSeqHeader = NULL;
static unsigned char* gspPictureData = NULL;
static int gsiPictureDataSize;
#endif
static unsigned int gsiFrameIdx = 0;
static int gsBitstreamBufSize = 0;
static codec_addr_t gsBitstreamBufAddr[3] = {0,};
//static int gsBitStreamBufDev;

//#define MAX_NUM_OF_VIDEO_ELEMENT 	VIDEO_ENC_BUFFER_COUNT
static unsigned int encoded_buf_size = 0;
static codec_addr_t encoded_buf_base_pos[3] = {0, };
static codec_addr_t encoded_buf_end_pos[3] = {0, };
static codec_addr_t encoded_buf_cur_pos[3] = {0, };
static int keyInterval_cnt = 0;

static unsigned int bAvcUsedNALStart = 0; // added by shmin for M2TS

#ifdef MULTI_SLICES_AVC
static int enc_avc_aud_enable = 0;
static codec_addr_t enc_slice_info_addr[3] = { 0, };
static unsigned int enc_slice_info_size = 0;
const unsigned char avcAudData[8] = { 0x00,0x00,0x00,0x01,0x09,0x50,0x00,0x00 };
#endif

static void vpu_env_close(void);

static void *cdk_sys_malloc_physical_addr(unsigned int *remap_addr, int uiSize, Buffer_Type type)
{
	MEM_ALLOC_INFO_t alloc_mem;

	memset(&alloc_mem, 0x00, sizeof(MEM_ALLOC_INFO_t));
	
	alloc_mem.request_size = uiSize;
	alloc_mem.buffer_type = type;
	ioctl(vpu_enc_fd, V_ENC_ALLOC_MEMORY, &alloc_mem);

	if(remap_addr != NULL)
		*remap_addr = alloc_mem.kernel_remap_addr;

	return (void*)( alloc_mem.phy_addr );;
}

static void *cdk_sys_malloc_virtual_addr(int* pDev, codec_addr_t pPtr, int uiSize)
{
	return (void *)mmap(NULL, uiSize, PROT_READ | PROT_WRITE, MAP_SHARED, vpu_enc_fd, pPtr);	
}

static int cdk_sys_free_virtual_addr(int* pDev, void* pPtr, int uiSize)
{
	return munmap((void*)pPtr, uiSize);
}

static unsigned int cdk_sys_remain_memory_size(void)
{
	unsigned int sz_freeed_mem;

	ioctl(vpu_mgr_fd, VPU_GET_FREEMEM_SIZE, &sz_freeed_mem);

	return sz_freeed_mem;
}

static void vpu_update_sizeinfo(unsigned int image_width, unsigned int image_height)
{
	CONTENTS_INFO info;

	memset(&info, 0x00, sizeof(CONTENTS_INFO));	
	info.type = VPU_ENC;
	info.width = image_width;
	info.height = image_height;

#ifdef ENABLE_REMOTE_PLAYER
	{
		char Rplayerstream[PROPERTY_VALUE_MAX];

	    property_get("tcc.rplayer.stream",Rplayerstream,"");
	    if(!strcmp(Rplayerstream,"1"))
	    {
	       info.isSWCodec = 1;
	    }
	}
#endif

	ioctl(vpu_mgr_fd, VPU_SET_CLK, &info);

	return;
}

static int gPFrameCnt = 0, gMaxOutputSize_PFrame = 0;
static int gIFrameCnt = 0, gMaxOutputSize_IFrame = 0;
static int vpu_env_open(unsigned int image_width, unsigned int image_height)
{
	DSTATUS("In  %s \n",__func__);

	if(venc_env_opened)
	{
		//to recover abnormal stop error!!
		LOGE("VENC has been already opened. so have to close!!");
		venc_vpu( VENC_CLOSE, NULL, NULL, NULL );
	}

#ifdef  USE_VPU_INTERRUPT
	vpu_intr_fd = open(TCC_INTR_DEV_NAME, O_RDWR);
	if (vpu_intr_fd < 0) {
		LOGE("%s open error", TCC_INTR_DEV_NAME);
		goto err;	
	}	
#endif

	vpu_enc_fd = open(VPU_ENC_NAME, O_RDWR);
	if(vpu_enc_fd < 0)
	{
		LOGE("%s open error[%s]", VPU_ENC_NAME, strerror(errno));
		goto err;	
	}

	vpu_mgr_fd = open(VPU_MGR_NAME, O_RDWR | O_NDELAY);
	if(vpu_mgr_fd < 0)
	{
		LOGE("%s open error[%s]!!", VPU_MGR_NAME, strerror(errno));
		goto err;	
	}
	vpu_update_sizeinfo(image_width, image_height);

	venc_env_opened = 1;
	gsiFrameIdx = 0;
	gPFrameCnt = gMaxOutputSize_PFrame = 0;
	gIFrameCnt = gMaxOutputSize_IFrame = 0;
	gsiSeqHeaderCnt = 0;

	memset( &gsVpuEncInit_Info.gsVpuEncInit,		0, sizeof(gsVpuEncInit_Info.gsVpuEncInit) );
	memset( &gsVpuEncInOut_Info.gsVpuEncInput,		0, sizeof(gsVpuEncInOut_Info.gsVpuEncInput) );
	memset( &gsVpuEncInit_Info.gsVpuEncInitialInfo,	0, sizeof(gsVpuEncInit_Info.gsVpuEncInitialInfo) );
	memset( &gsVpuEncInOut_Info.gsVpuEncInput,		0, sizeof(gsVpuEncInOut_Info.gsVpuEncInput) );

	memset(gsBitstreamBufAddr, 0x00, sizeof(gsBitstreamBufAddr));
	memset(gsBitWorkBufAddr, 0x00, sizeof(gsBitWorkBufAddr));
	memset(gsMESearchBufAddr, 0x00, sizeof(gsMESearchBufAddr));
	memset(gsFrameBufAddr, 0x00, sizeof(gsFrameBufAddr));
	memset(gspSeqHeaderAddr, 0x00, sizeof(gspSeqHeaderAddr));
	memset(encoded_buf_base_pos, 0x00, sizeof(encoded_buf_base_pos));

	DSTATUS("Out  %s \n",__func__);

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
	seq_backup = NULL;
	seq_len = 0;
#endif

#ifdef DEBUG_TIME_LOG	
	time_cnt = 0;
	total_dec_time = 0;
#endif
	total_frm = 0;

	return 0;

err:	
	LOGE("vpu_env_open error");
	vpu_env_close();
	
	return -1;	
	
}


static void vpu_env_close(void)
{
	DSTATUS("In  %s \n",__func__);

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
	if(seq_backup != NULL)
	{
		TCC_free(seq_backup);
		seq_backup = NULL;
		seq_len = 0;
	}
#endif

#ifdef  USE_VPU_INTERRUPT
	if(vpu_intr_fd > 0)
	{
		if(close(vpu_intr_fd) < 0)
		{
			LOGE("%s close error", TCC_INTR_DEV_NAME);
		}
		vpu_intr_fd = -1;
	}
#endif	

	if(vpu_enc_fd)
	{
		if(close(vpu_enc_fd) < 0)
		{
			LOGE("%s close error[%s]", VPU_ENC_NAME, strerror(errno));
		}
		vpu_enc_fd = -1;
	}

	if(vpu_mgr_fd)
	{
		if(close(vpu_mgr_fd) < 0)
		{
			LOGE("%s close error[%s]", VPU_MGR_NAME, strerror(errno));
		}
		vpu_mgr_fd = -1;
	}
	
	venc_env_opened = 0;

	DSTATUS("Out  %s \n",__func__);

}

static void filewrite_memory(char* name, char* addr, unsigned int size)
{
#ifdef VPU_FRAME_DUMP

	FILE *fp;

	if(!bFirst_frame)
		return;
	
	fp = fopen(name, "ab+");		
	fwrite( addr, size, 1, fp);
	fclose(fp);
#endif

}

static void save_output_stream(char* name, int size, unsigned char* addr)
{
#ifdef VPU_OUT_FRAME_DUMP

	int i;
	unsigned char* ps = (unsigned char*)addr;

	if(1)
	{
		FILE *fp;
		fp = fopen(name, "ab+");		  
		fwrite( ps, size, 1, fp);
		fclose(fp);

		return;
	}

	for(i=0; (i+10 <size) && (i+10 < 100); i += 10){
		DPRINTF_FRAME( "[VENC - Stream] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", ps[i], ps[i+1], ps[i+2], ps[i+3], ps[i+4], ps[i+5], ps[i+6], ps[i+7], ps[i+8], ps[i+9] );
	}
#endif	
}		


#ifdef USE_VPU_INTERRUPT
static void write_reg(unsigned int addr, unsigned int val)
{
	*((volatile unsigned int *)(gsRegisterBase + addr)) = (unsigned int)(val);
}

static unsigned int read_reg(unsigned int addr)
{
	return *(volatile unsigned int *)(gsRegisterBase + addr);
}

static int VpuInterrupt()
{
	int success = 0;
	
	while (1) {
		int ret;
		memset(tcc_event, 0, sizeof(tcc_event));
		tcc_event[0].fd = vpu_intr_fd;
		tcc_event[0].events = POLLIN;
		
		ret = poll((struct pollfd *)&tcc_event, 1, 500); // 500 msec
		if (ret < 0) {
			LOGE("vpu poll error\n");
			break;
		}else if (ret == 0) {
			LOGE("vpu poll timeout\n");		
			break;
		}else if (ret > 0) {
			if (tcc_event[0].revents & POLLERR) {
				LOGE("vpu poll POLLERR\n");
				break;
			} else if (tcc_event[0].revents & POLLIN) {
				success = 1;
				break;
			}
		}
	}
	/* todo */
	
	write_reg(0x174, 0);
	write_reg(0x00C, 1);

	if(success)
		return RETCODE_SUCCESS;
	else
		return RETCODE_CODEC_EXIT;
}
#endif

static int venc_cmd_process(int cmd, void* args)
{
	int ret;
	int success = 0;

	if(ioctl(vpu_enc_fd, cmd, args) < 0)
	{
		LOGE("vpu ioctl err[%s] : cmd = 0x%x", strerror(errno), cmd);
	}
	
	while (1) {
		int ret;
		memset(tcc_event, 0, sizeof(tcc_event));
		tcc_event[0].fd = vpu_enc_fd;
		tcc_event[0].events = POLLIN;
		
		ret = poll((struct pollfd *)&tcc_event, 1, 1000); // 100 msec
		if (ret < 0) {
			LOGE("vpu(0x%x) poll error", cmd);
			break;
		}else if (ret == 0) {
			LOGE("vpu(0x%x) poll timeout", cmd);
			break;
		}else if (ret > 0) {
			if (tcc_event[0].revents & POLLERR) {
				LOGE("vpu(0x%x) poll POLLERR", cmd);
				break;
			} else if (tcc_event[0].revents & POLLIN) {
				success = 1;
				break;
			}
		}
	}

	switch(cmd)
	{
		case V_ENC_INIT:
			{			 
			 	VENC_INIT_t* init_info = args;
				
				ioctl(vpu_enc_fd, V_ENC_INIT_RESULT, args);
				ret = init_info->result;
			}
			break;
			
		case V_ENC_PUT_HEADER: 
			{
			 	VENC_PUT_HEADER_t* buff_info = args;
				
				ioctl(vpu_enc_fd, V_ENC_PUT_HEADER_RESULT, args);
				ret = buff_info->result;
			}
			break;
			
		case V_ENC_ENCODE:
			{
			 	VENC_ENCODE_t* encoded_info = args;
				
				ioctl(vpu_enc_fd, V_ENC_ENCODE_RESULT, args);
				ret = encoded_info->result;
			}
			break;
			
		case V_ENC_REG_FRAME_BUFFER:			
		case V_ENC_CLOSE:
		default:
			ioctl(vpu_enc_fd, V_ENC_GENERAL_RESULT, &ret);
			break;			
	}

	/* todo */
	if(!success)
	{	
		LOGE("VENC command(0x%x) didn't work properly. maybe hangup(no return(0x%x))!!", cmd, ret);

		if(ret != RETCODE_CODEC_EXIT && ret != RETCODE_MULTI_CODEC_EXIT_TIMEOUT){
//			ioctl(vpu_mgr_fd, VPU_HW_RESET, (void*)NULL);
		}

		return RETCODE_CODEC_EXIT;
	}

	return ret;
}

int
venc_vpu( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;

#ifdef DEBUG_TIME_LOG
	clock_t start, end;
	start = clock();
#endif

	if( iOpCode == VENC_INIT )
	{
		venc_init_t* p_init_param = (venc_init_t*)pParam1;

#ifdef HAVE_ANDROID_OS
		if(vpu_env_open(p_init_param->m_iPicWidth, p_init_param->m_iPicHeight ) < 0)
			return -VPU_ENV_INIT_ERROR;
#endif

#if defined(VPU_CLK_CONTROL)
		vpu_clock_init();
#endif

		// added by shmin for M2TS
		bAvcUsedNALStart = *((unsigned int*)pParam2);

#ifdef HAVE_ANDROID_OS
		gsVpuEncInit_Info.gsVpuEncInit.m_RegBaseVirtualAddr	= (unsigned int)NULL;
		//gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr	= p_init_param->m_BitstreamBufferAddr;		
		//gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr_VA = p_init_param->m_BitstreamBufferAddr_VA;
		//gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamBufferSize 	= p_init_param->m_iBitstreamBufferSize;

		gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat 	= p_init_param->m_iBitstreamFormat;
		gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth			= p_init_param->m_iPicWidth;
		gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight			= p_init_param->m_iPicHeight;
		gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate			= p_init_param->m_iFrameRate;
		gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps		= p_init_param->m_iTargetKbps;
		gsVpuEncInit_Info.gsVpuEncInit.m_iKeyInterval		= p_init_param->m_iKeyInterval;// only first picture is I

		gsVpuEncInit_Info.gsVpuEncInit.m_iUseSpecificRcOption = 1;
							
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iAvcFastEncoding 	= p_init_param->m_iAvcFastEncoding;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iPicQpY 			= -1;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iIntraMBRefresh 	= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkDisable 		= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkAlpha 		= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkBeta 			= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iDeblkChQpOffset 	= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iConstrainedIntra 	= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iVbvBufferSize 		= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSearchRange 		= 2;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iPVMDisable 		= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iWeightIntraCost 	= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iRCIntervalMode 	= 1;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iRCIntervalMBNum 	= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iEncQualityLevel	= 11;
		
		gsVpuEncInit_Info.gsVpuEncInit.m_bEnableVideoCache				= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_bCbCrInterleaveMode			= 0;
		gsVpuEncInit_Info.gsVpuEncInit.m_uiEncOptFlags					= 0; //(1 << 10 );
		
#ifdef MULTI_SLICES_AVC		
		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
		{
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode			= p_init_param->m_iSliceMode;		// multi-slices per picture
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode		= p_init_param->m_iSliceSizeMode;
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize			= p_init_param->m_iSliceSize;

			if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
				enc_avc_aud_enable = 1;
			else
				enc_avc_aud_enable = 0;
		}
		else
#endif
		{
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode			= 0;		// 1 slice per picture
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode		= 0;
			gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize			= 0;
#ifdef MULTI_SLICES_AVC					
			enc_avc_aud_enable = 0;
#endif
		}
		DSTATUS( "SliceMode[%d] - SizeMode[%d] - %d", gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode, gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSizeMode, gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceSize );
		
		gsVpuEncInit_Info.gsVpuEncInit.m_Memcpy				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsVpuEncInit_Info.gsVpuEncInit.m_Memset				= (void  (*) ( void*, int, unsigned int ))memset;
		gsVpuEncInit_Info.gsVpuEncInit.m_Interrupt			= (int  (*) ( void ))NULL; 

		keyInterval_cnt = 0;
#else
		gsVpuEncInit.m_RegBaseVirtualAddr				= p_init_param->m_RegBaseVirtualAddr;
		gsVpuEncInit.m_BitstreamBufferAddr				= p_init_param->m_BitstreamBufferAddr;
		gsVpuEncInit.m_iBitstreamBufferSize 			= p_init_param->m_iBitstreamBufferSize;
		gsVpuEncInit.m_BitstreamBufferAddr_VA 			= p_init_param->m_BitstreamBufferAddr_VA;
		gsVpuEncInit.m_iBitstreamFormat					= p_init_param->m_iBitstreamFormat;
		gsVpuEncInit.m_iPicWidth						= p_init_param->m_iPicWidth;
		gsVpuEncInit.m_iPicHeight						= p_init_param->m_iPicHeight;
		gsVpuEncInit.m_iFrameRate						= p_init_param->m_iFrameRate;
		gsVpuEncInit.m_iTargetKbps						= p_init_param->m_iTargetKbps;
		gsVpuEncInit.m_iKeyInterval						= p_init_param->m_iKeyInterval;// only first picture is I
		gsVpuEncInit.m_stRcInit.m_iAvcFastEncoding		= p_init_param->m_iAvcFastEncoding;
		gsVpuEncInit.m_stRcInit.m_iSliceMode			= 0;		// 1 slice per picture
		gsVpuEncInit.m_stRcInit.m_iSliceSizeMode		= 0;
		gsVpuEncInit.m_stRcInit.m_iSliceSize			= 0;
		gsVpuEncInit.m_stRcInit.m_iIntraMBRefresh		= 0;
		gsVpuEncInit.m_stRcInit.m_iPicQpY				= -1;
		gsVpuEncInit.m_bEnableVideoCache				= 1;

		gsVpuEncInit.m_Memcpy				= p_init_param->m_pfMemcpy;
		gsVpuEncInit.m_Memset				= p_init_param->m_pfMemset;
		gsVpuEncInit.m_Interrupt			= p_init_param->m_pfInterrupt;
#endif

#ifdef HAVE_ANDROID_OS
		//------------------------------------------------------------
		//! [x] bitstream buffer for each VPU decoder
		//------------------------------------------------------------
		gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
		gsBitstreamBufSize = ALIGNED_BUFF( gsBitstreamBufSize, ALIGN_LEN );
		gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &gsBitstreamBufAddr[K_VA], gsBitstreamBufSize, 0 );
		if( gsBitstreamBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC] bitstream_buf_addr[PA] malloc() failed \n");
			return -1;
		}
		DSTATUS( "[VENC] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		if( gsBitstreamBufAddr[VA] == 0 ) 
		{
			LOGE( "[VENC] bitstream_buf_addr[VA] malloc() failed \n");
			return -1;
		}
		//memset( (void*)gsBitstreamBufAddr[VA], 0x00 , gsBitstreamBufSize);
		DSTATUS("[VENC] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[VA], gsBitstreamBufSize );
	
		gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr		= gsBitstreamBufAddr[PA];
		gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr_VA 	= gsBitstreamBufAddr[K_VA];
		gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamBufferSize		= gsBitstreamBufSize;
#endif

		//------------------------------------------------------------
		//! [x] code buffer, work buffer and parameter buffer for VPU 
		//------------------------------------------------------------
		gsBitWorkBufSize = WORK_CODE_PARA_BUF_SIZE;
		gsBitWorkBufSize = ALIGNED_BUFF(gsBitWorkBufSize, ALIGN_LEN);
		gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &gsBitWorkBufAddr[K_VA], gsBitWorkBufSize, BUFFER_WORK );
		if( gsBitWorkBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC] gsBitWorkBufAddr[PA] malloc() failed \n");
			return -1;
		}
		DSTATUS("[VENC] gsBitWorkBufAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
		gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, gsBitWorkBufAddr[PA], gsBitWorkBufSize );
		if( gsBitWorkBufAddr[VA] == 0 ) 
		{
			LOGE( "[VENC] gsBitWorkBufAddr[VA] malloc() failed \n");
			return -1;
		}
		DSTATUS("[VENC] gsBitWorkBufAddr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitWorkBufAddr[VA], gsBitWorkBufSize );

		//------------------------------------------------------------
		//! [x] me search buffer for each VPU encoder
		//------------------------------------------------------------
		//! Estimate size
		gsMESearchBufSize = ( ( gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth + 15 ) & ~15 ) * 36 + 2048;  // picWidth of searchram size must be a multiple of 16
		gsMESearchBufSize = ALIGNED_BUFF( gsMESearchBufSize, ALIGN_LEN );
		//DSTATUS( "[CDK_CORE] gsMESearchBufSize = %d\n", gsMESearchBufSize );
		gsMESearchBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &gsMESearchBufAddr[K_VA], gsMESearchBufSize, 0 );
		if( gsMESearchBufAddr[PA] == 0 ) 
		{
			LOGE( "[CDK_CORE] gsMESearchBufAddr[PA] physical malloc() failed \n");
			return CDK_ERR_MALLOC;
		}
		DSTATUS("[CDK_CORE] gsMESearchBufAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsMESearchBufAddr[PA], gsMESearchBufSize );
		gsMESearchBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, gsMESearchBufAddr[PA], gsMESearchBufSize );
		if( gsMESearchBufAddr[VA] == 0 ) 
		{
			LOGE( "[CDK_CORE] gsMESearchBufAddr[VA] virtual malloc() failed \n");
			return CDK_ERR_MALLOC;
		}
		DSTATUS("[CDK_CORE] gsMESearchBufAddr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsMESearchBufAddr[VA], gsMESearchBufSize );

		gsVpuEncInit_Info.gsVpuEncInit.m_BitWorkAddr[PA]		= gsBitWorkBufAddr[PA];
		gsVpuEncInit_Info.gsVpuEncInit.m_BitWorkAddr[VA]		= gsBitWorkBufAddr[K_VA];
		//gsVpuEncInit_Info.gsVpuEncInit.m_MeSearchRamAddr		= gsMESearchBufAddr[PA];
		//gsVpuEncInit_Info.gsVpuEncInit.m_iMeSearchRamSize		= gsMESearchBufSize;

#ifdef MULTI_SLICES_AVC	
		if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
		{
			//------------------------------------------------------------
			//! [x] Slice info. buffers requested by the encoder.
			//------------------------------------------------------------
			int iMbWidth, iMbHeight;
			
			iMbWidth = (gsVpuEncInit_Info.gsVpuEncInit.m_iPicWidth+15)>>4;
			iMbHeight = (gsVpuEncInit_Info.gsVpuEncInit.m_iPicHeight+15)>>4;
			
			enc_slice_info_size = iMbWidth * iMbHeight * 8 + 48;
			enc_slice_info_size = ALIGNED_BUFF(enc_slice_info_size, ALIGN_LEN);
			enc_slice_info_addr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &enc_slice_info_addr[K_VA], enc_slice_info_size, 0 );
			if( enc_slice_info_addr[PA] == 0 ) 
			{
				LOGE( "[CDK_CORE] enc_slice_info_addr[PA] physical malloc() failed \n");
				return CDK_ERR_MALLOC;
			}
			
			DSTATUS("[CDK_CORE] enc_slice_info_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)enc_slice_info_addr[PA], enc_slice_info_size );
			enc_slice_info_addr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, enc_slice_info_addr[PA], enc_slice_info_size );
			if( enc_slice_info_addr[VA] == 0 ) 
			{
				LOGE( "[CDK_CORE] enc_slice_info_addr[VA] virtual malloc() failed \n");
				return CDK_ERR_MALLOC;
			}
			DSTATUS("[CDK_CORE] enc_slice_info_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)enc_slice_info_addr[VA], enc_slice_info_size );
		}
#endif

		{
			unsigned int remained_mem_size;

			encoded_buf_size = (gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps/8 /*KB/s*/) * 1024/*Byte*/ * (VIDEO_ENC_BUFFER_COUNT/gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate +3 /*add 3sec*/ );

			if(encoded_buf_size < (LARGE_STREAM_BUF_SIZE * 2)) //4MB
				encoded_buf_size = (LARGE_STREAM_BUF_SIZE * 2);
			encoded_buf_size = ALIGNED_BUFF(encoded_buf_size, ALIGN_LEN);

			remained_mem_size = cdk_sys_remain_memory_size();
			if(remained_mem_size < encoded_buf_size)
			{
				LOGE( "[VENC,Err] Insufficient memory for streamOut. \n");
				return -1;
			}
			
			encoded_buf_base_pos[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &encoded_buf_base_pos[K_VA], encoded_buf_size, 0 );	
			if( !encoded_buf_base_pos[PA] ) 
			{
				LOGE( "[VENC,Err] venc_vpu encoded_phyAddr[PA] alloc failed \n" );
				return -1;
			}	
			encoded_buf_base_pos[VA] 	= (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, encoded_buf_base_pos[PA], encoded_buf_size );	
			encoded_buf_cur_pos[PA] 	= encoded_buf_base_pos[PA];
			encoded_buf_cur_pos[VA] 	= encoded_buf_base_pos[VA];
			encoded_buf_cur_pos[K_VA] 	= encoded_buf_base_pos[K_VA];
			encoded_buf_end_pos[PA] 	= encoded_buf_base_pos[PA] + encoded_buf_size;
			encoded_buf_end_pos[VA] 	= encoded_buf_base_pos[VA] + encoded_buf_size;
			encoded_buf_end_pos[K_VA] 	= encoded_buf_base_pos[K_VA] + encoded_buf_size;
			
			DSTATUS("Stream out-Buffer ::	%d Kbps, %d fps, %d sec !!\n", gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps, gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate, (VIDEO_ENC_BUFFER_COUNT/gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate +3 /*add 3sec*/ ));
			DSTATUS("               PA = 0x%x, VA = 0x%x, size = 0x%x!!\n", encoded_buf_base_pos[PA], encoded_buf_base_pos[VA],	encoded_buf_size);

			gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr		= encoded_buf_base_pos[PA];//gsBitstreamBufAddr[PA];
			gsVpuEncInit_Info.gsVpuEncInit.m_BitstreamBufferAddr_VA 	= encoded_buf_base_pos[K_VA];//gsBitstreamBufAddr[K_VA];
			gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamBufferSize		= encoded_buf_size;//gsBitstreamBufSize;
		}
		ret = venc_cmd_process(V_ENC_INIT, &gsVpuEncInit_Info);

		if( ret != RETCODE_SUCCESS )
		{
			LOGE( "[VENC,Err:0x%x] venc_vpu VPU_ENC_INIT failed \n", ret );
			return -ret;
		}
		DSTATUS("[VENC] venc_vpu VPU_ENC_INIT ok! \n" );

		//------------------------------------------------------------
		//! [x] Register frame buffers requested by the encoder.
		//------------------------------------------------------------
		gsFrameBufSize = gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferCount * gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferSize;
		gsFrameBufSize = ALIGNED_BUFF(gsFrameBufSize, ALIGN_LEN);
		gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &gsFrameBufAddr[K_VA], gsFrameBufSize, 0 );
		if( gsFrameBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC,Err:0x%x] venc_vpu gsFrameBufAddr[PA](0x%x) alloc failed \n", ret, gsFrameBufSize );
			return -ret;
		}	
		DSTATUS("[VENC] gsFrameBufAddr[PA] = 0x%x, 0x%x((%d min) * %d bytes) \n", (codec_addr_t)gsFrameBufAddr[PA], gsFrameBufSize, gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferCount, gsVpuEncInit_Info.gsVpuEncInitialInfo.m_iMinFrameBufferSize);
		gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, gsFrameBufAddr[PA], gsFrameBufSize );
		if( gsFrameBufAddr[VA] == 0 ) 
		{
			LOGE( "[VENC,Err:0x%x] venc_vpu gsFrameBufAddr[VA] alloc failed \n", ret );
			return -ret;
		}
		DSTATUS("[VENC] gsFrameBufAddr[VA] = 0x%x, gsFrameBufAddr[K_VA] = 0x%x \n", (codec_addr_t)gsFrameBufAddr[VA], gsFrameBufAddr[K_VA] );

		gsVpuEncBuffer_Info.gsVpuEncBuffer.m_FrameBufferStartAddr[PA] = gsFrameBufAddr[PA];
		gsVpuEncBuffer_Info.gsVpuEncBuffer.m_FrameBufferStartAddr[VA] = gsFrameBufAddr[K_VA];

		ret = venc_cmd_process(V_ENC_REG_FRAME_BUFFER, &gsVpuEncBuffer_Info);  // register frame buffer

		if( ret != RETCODE_SUCCESS )
		{
			LOGE( "[VENC,Err:0x%x] venc_vpu VPU_ENC_REG_FRAME_BUFFER failed \n", ret );
			return -ret;
		}

		DSTATUS("[VENC] venc_vpu VPU_ENC_REG_FRAME_BUFFER ok! \n" );
#ifdef CHECK_BITRATE
		curr_bps = gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps;
		curr_fps = gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate;
		bps_frames = total_size = 0;
#endif	
	}
	else if( iOpCode == VENC_SEQ_HEADER )
	{
		venc_seq_header_t* p_seq_param = (venc_seq_header_t*)pParam1;
		unsigned char* p_dest = NULL;
		int i_dest_size = 0;
		codec_addr_t m_SeqHeaderBuffer_VA = 0;

#ifdef HAVE_ANDROID_OS
		if(gspSeqHeaderAddr[PA] == 0)
		{
			gsiSeqHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			
			if(gsiSeqHeaderSize == 0)
			{
				gsiSeqHeaderSize = ALIGNED_BUFF( 100*1024, ALIGN_LEN );
			}

			gspSeqHeaderAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( &gspSeqHeaderAddr[K_VA], gsiSeqHeaderSize, 0 );
			if( gspSeqHeaderAddr[PA] == 0 ) 
			{
				LOGE( "[VENC] gspSeqHeaderAddr[PA] malloc() failed \n");
				return -1;
			}
			DSTATUS( "[VENC] gspSeqHeaderAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gspSeqHeaderAddr[PA], gsiSeqHeaderSize );
			gspSeqHeaderAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, gspSeqHeaderAddr[PA], gsiSeqHeaderSize );
			if( gspSeqHeaderAddr[VA] == 0 ) 
			{
				LOGE( "[VENC] gspSeqHeader_VA malloc() failed \n");
				return -1;
			}
			DSTATUS( "[VENC] gspSeqHeader_VA = 0x%x, 0x%x \n", (codec_addr_t)gspSeqHeaderAddr[VA], gsiSeqHeaderSize );
			
			memset( (void*)gspSeqHeaderAddr[VA], 0x00, gsiSeqHeaderSize );
		}

		if(p_seq_param->m_SeqHeaderBuffer[PA] == 0)
		{
			DSTATUS( "[VENC] gspSeqHeader_Buffer = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
			p_seq_param->m_SeqHeaderBuffer[PA]	=	gsBitstreamBufAddr[PA];
			p_seq_param->m_SeqHeaderBuffer[VA]	=	gsBitstreamBufAddr[K_VA];
			p_seq_param->m_iSeqHeaderBufferSize =	gsBitstreamBufSize;
		}
		m_SeqHeaderBuffer_VA = gsBitstreamBufAddr[VA];
#else
		if( gspSeqHeader == NULL )
		{
			gspSeqHeader = (unsigned char*)TCC_calloc( p_seq_param->m_iSeqHeaderBufferSize );
			if( gspSeqHeader == NULL ) 
			{
				LOGE( "[VENC:Err:0x%x] gspSeqHeader malloc failed \n", CDK_ERR_MALLOC );
				return CDK_ERR_MALLOC;
			}
			memset( gspSeqHeader, 0, p_seq_param->m_iSeqHeaderBufferSize );
		}
#endif

		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 )
		{
			p_dest = (unsigned char*)encoded_buf_cur_pos[VA];//gspSeqHeaderAddr[VA];

			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOS_HEADER;
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:0x%x] venc_vpu MPEG4_VOS_HEADER failed \n", ret );
				return -ret;
			}
			memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
			p_dest += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			i_dest_size += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			LOGD("VOL : %d / %d", gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);

			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VIS_HEADER;
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:0x%x] venc_vpu MPEG4_VIS_HEADER failed \n", ret );
				return -ret;
			}
			memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
			p_dest += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			i_dest_size += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			LOGD("VOS : %d / %d", gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);
			
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOL_HEADER;
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;

			DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for MPEG4_VOL_HEADER \n");
			ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
			
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:0x%x] venc_vpu MPEG4_VOL_HEADER failed \n", ret );
				return -ret;
			}
			filewrite_memory("/data/enc.dat", m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
			memcpy( p_dest, (void*)m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
			i_dest_size += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			LOGD("VIS : %d / %d", gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize, i_dest_size);
		}
		else if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
		{
	#ifdef HAVE_ANDROID_OS
			if(gsiSeqHeaderCnt == 0)
	#endif
			{		
				p_dest = (unsigned char*)encoded_buf_cur_pos[VA];//gspSeqHeaderAddr[VA];
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = AVC_SPS_RBSP;
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;

#ifdef MULTI_SLICES_AVC
				if( enc_avc_aud_enable == 1) {
					memcpy( m_SeqHeaderBuffer_VA, avcAudData, 8 ); //H.264 AUD
					gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr += 8;
				}
#endif

				DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_SPS_RBSP \n");
				ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
				if( ret != RETCODE_SUCCESS )
				{
					LOGE( "[VENC:Err:0x%x] venc_vpu AVC_SPS_RBSP failed \n", ret );
					return -ret;
				}

				// modified by shmin for M2TS
				if(bAvcUsedNALStart)
				{
#ifdef MULTI_SLICES_AVC
					if( enc_avc_aud_enable == 1) {
						gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize += 8;
					}
#endif	
					unsigned char* buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
					LOGD("SPS(%d) :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x - p_dest = 0x%x", gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize,
									buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], (uint32_t)p_dest );

					filewrite_memory("/data/enc.dat", m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
					memcpy( (void*)p_dest, (void*)m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
				}
				else
				{
					unsigned char* buffer;

					buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
					gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize -= 4;
					memcpy( (void*)p_dest, (void*)(buffer+4), gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
				}

		#if 0
				{
					int temp = 0;
					DSTATUS("[SPS:0x");
					for( temp = 0; temp < gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize; temp++ )
					{
						DSTATUS("%02X ", p_dest[temp] );
					}
					DSTATUS("\n");
				}
		#endif

				p_dest += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
				i_dest_size += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
				gsiSzSeqHeader[gsiSeqHeaderCnt] = i_dest_size;
			}
	#ifdef HAVE_ANDROID_OS
			else
	#endif
			{
				p_dest = (unsigned char*)encoded_buf_cur_pos[VA];//gspSeqHeaderAddr[VA];
				p_dest += gsiSzSeqHeader[gsiSeqHeaderCnt-1];
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = AVC_PPS_RBSP;
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
				gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
				
				DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_PPS_RBSP \n");
				ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
				if( ret != RETCODE_SUCCESS )
				{
					LOGE( "[VENC:Err:0x%x] venc_vpu AVC_SPS_RBSP failed \n", ret );
					return -ret;
				}

				// modified by shmin for M2TS
				if(bAvcUsedNALStart)
				{
					unsigned char* buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
					LOGD("PPS(%d) :: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x - p_dest = 0x%x", gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize,
										buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], (uint32_t)p_dest );
					filewrite_memory("/data/enc.dat", m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
					memcpy( (void*)p_dest, (void*)m_SeqHeaderBuffer_VA, gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize );
				}
				else
				{
					unsigned char* buffer;

					buffer = (unsigned char*)m_SeqHeaderBuffer_VA;
					gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize -= 4;
					memcpy( (void*)p_dest, (void*)(buffer+4), gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
				}

				#if 0
				{
					int temp = 0;
					DSTATUS("[PPS:0x");
					for( temp = 0; temp < gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize; temp++ )
					{
						DSTATUS("%02X ", p_dest[temp] );
					}
					DSTATUS("\n");
				}
				#endif
				i_dest_size += gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
				gsiSzSeqHeader[gsiSeqHeaderCnt] = gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;

			}
		}

		// output
#ifdef HAVE_ANDROID_OS
	#if 1
		p_seq_param->m_pSeqHeaderOut = (unsigned char*)encoded_buf_cur_pos[VA];

		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 )
		{
		#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
			if(seq_backup == NULL){
				seq_backup = (unsigned char*)TCC_malloc(i_dest_size + (STABILITY_GAP*2));
			}
			memcpy(seq_backup, encoded_buf_cur_pos[VA], i_dest_size);
			seq_len = i_dest_size;
		#endif
			encoded_buf_cur_pos[PA] += ALIGNED_BUFF(i_dest_size + (STABILITY_GAP*2), ALIGN_LEN);
			encoded_buf_cur_pos[VA] += ALIGNED_BUFF(i_dest_size + (STABILITY_GAP*2), ALIGN_LEN);
			encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(i_dest_size + (STABILITY_GAP*2), ALIGN_LEN);
		}
		else
		{
			if(gsiSeqHeaderCnt == 1)
			{
		#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
				if(seq_backup == NULL){
					seq_backup = (unsigned char*)TCC_malloc(gsiSzSeqHeader[0] + gsiSzSeqHeader[1] + (STABILITY_GAP*2));
				}
				memcpy(seq_backup, encoded_buf_cur_pos[VA], gsiSzSeqHeader[0] + gsiSzSeqHeader[1]);				
				seq_len = gsiSzSeqHeader[0] + gsiSzSeqHeader[1];
				save_output_stream("/sdcard/vpu_outSeq.bin", seq_len, seq_backup);
		#endif
			
				encoded_buf_cur_pos[PA] += ALIGNED_BUFF(gsiSzSeqHeader[0] + gsiSzSeqHeader[1] + (STABILITY_GAP*2), ALIGN_LEN);
				encoded_buf_cur_pos[VA] += ALIGNED_BUFF(gsiSzSeqHeader[0] + gsiSzSeqHeader[1] + (STABILITY_GAP*2), ALIGN_LEN);
				encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(gsiSzSeqHeader[0] + gsiSzSeqHeader[1] + (STABILITY_GAP*2), ALIGN_LEN);
			}
		}
	#else
		if(gsiSeqHeaderCnt == 0)
			p_seq_param->m_pSeqHeaderOut	 = (unsigned char*)gspSeqHeaderAddr[VA];
		else
			p_seq_param->m_pSeqHeaderOut	 = (unsigned char*)gspSeqHeaderAddr[VA] + gsiSzSeqHeader[gsiSeqHeaderCnt-1];
	#endif

#else
		p_seq_param->m_pSeqHeaderOut	 = gspSeqHeader;
#endif
		p_seq_param->m_iSeqHeaderOutSize = i_dest_size;
		
		gsiSeqHeaderCnt++;

	}
	else if( iOpCode == VENC_ENCODE )
	{
		venc_input_t* p_input_param = (venc_input_t*)pParam1;
		venc_output_t* p_output_param = (venc_output_t*)pParam2;
#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME		
		int bChanged_fps = 0; //for only MPEG4.
#endif
		//! Start encoding a frame.
		//Input Buffer Setting
#ifdef ENABLE_RATE_CONTROL
		gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag  =  p_input_param->m_iChangeRcParamFlag;
		gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps   =  p_input_param->m_iChangeTargetKbps;
		gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate    =  p_input_param->m_iChangeFrameRate;
	#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME		
		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4
			&& ((gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x04) == 0x04))
		{
			bChanged_fps = 1;
		}
	#endif

	#ifdef CHECK_BITRATE
		if((gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x04) == 0x04)
			curr_fps = gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate;

		if((gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x02) == 0x02)
		{
			unsigned int calc_bps;

			calc_bps = (total_size*8)/1024;

			if(curr_fps != bps_frames)
			{
				unsigned int temp_bps = calc_bps;
				calc_bps = (temp_bps*curr_fps)/bps_frames;
			}
			
			LOGD("Bitrate- %d kbps  => %d kbps :: %d bytes / %d frames (%d fps)", curr_bps, calc_bps, total_size, bps_frames, curr_fps);

			if(gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x2)
				curr_bps = gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps;
			else if(gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag & 0x4)
				curr_fps = gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate;
			
			total_size = bps_frames = 0;
		}
		bps_frames++;
	#endif
#endif

#ifdef HAVE_ANDROID_OS
		if(p_input_param->request_IntraFrame == 1)
		{
			gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture = 1;//set 1 For IDR-Type I-Frame without P-Frame!!
		}
		else
		{		
			gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture = 0;
		}			
#else
		gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture = 0;
#endif
		
		gsVpuEncInOut_Info.gsVpuEncInput.m_iSkipPicture = 0;
		if( gsVpuEncInit_Info.gsVpuEncInit.m_iTargetKbps == 0 ) // no rate control
		{
			gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam = 23;
		}
		else
		{
#ifdef CHANGE_QP_FOR_IFRAME
			gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam =  p_input_param->m_iQuantParam;
#else
			gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam = 10;
#endif
		}

#ifdef HAVE_ANDROID_OS
		if((encoded_buf_cur_pos[PA] + gsBitstreamBufSize) > encoded_buf_end_pos[PA])
		{
			encoded_buf_cur_pos[PA] = encoded_buf_base_pos[PA];
			encoded_buf_cur_pos[VA] = encoded_buf_base_pos[VA];
			encoded_buf_cur_pos[K_VA] = encoded_buf_base_pos[K_VA];
		}

	#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
		if( !bChanged_fps && (/*gsiFrameIdx != 0 && */((gsiFrameIdx%gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate) == 0)) && gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode != 1)
		{
			encoded_buf_cur_pos[PA] = encoded_buf_base_pos[PA] + ALIGNED_BUFF(seq_len, ALIGN_LEN);
			encoded_buf_cur_pos[VA] = encoded_buf_base_pos[VA] + ALIGNED_BUFF(seq_len, ALIGN_LEN);
			encoded_buf_cur_pos[K_VA] = encoded_buf_base_pos[K_VA] + ALIGNED_BUFF(seq_len, ALIGN_LEN);
		}
	#endif

		gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr =  (codec_addr_t)encoded_buf_cur_pos[PA];
		gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize = gsBitstreamBufSize;
#else
		gsVpuEncInput.m_BitstreamBufferAddr  = p_input_param->m_BitstreamBufferPA;
		gsVpuEncInput.m_iBitstreamBufferSize = p_input_param->m_iBitstreamBufferSize;
#endif
		gsVpuEncInOut_Info.gsVpuEncInput.m_PicYAddr = (codec_addr_t)p_input_param->m_pInputY;
		if( gsVpuEncInit_Info.gsVpuEncInit.m_bCbCrInterleaveMode == 0 )
		{
			gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr = (codec_addr_t)p_input_param->m_pInputCbCr[0];
			gsVpuEncInOut_Info.gsVpuEncInput.m_PicCrAddr = (codec_addr_t)p_input_param->m_pInputCbCr[1];
		}
		else
		{
			//FIXME
		}

#ifdef MULTI_SLICES_AVC	
		//H.264 AUD RBSP
		if( enc_avc_aud_enable == 1 && gsiFrameIdx > 0 ) {
			memcpy( encoded_buf_cur_pos[VA], avcAudData, 8 );
			gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr += 8;
			gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize -= 8;
		}

		// Slice information buffer setting
		if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 ) {
			gsVpuEncInOut_Info.gsVpuEncInput.m_iReportSliceInfoEnable = 1;
			gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[PA] = enc_slice_info_addr[PA];
			gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[VA] = enc_slice_info_addr[K_VA];
		}
		else {
			gsVpuEncInOut_Info.gsVpuEncInput.m_iReportSliceInfoEnable = 0;
			gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[PA] = 0;
			gsVpuEncInOut_Info.gsVpuEncInput.m_SliceInfoAddr[VA] = 0;
		}		
#endif

		//LOGD(" 0x%x-0x%x-0x%x", gsVpuEncInOut_Info.gsVpuEncInput.m_PicYAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_PicCrAddr);

//		LOGD(" 0x%x-0x%x-0x%x, %d-%d-%d, %d-%d-%d, 0x%x-%d", gsVpuEncInOut_Info.gsVpuEncInput.m_PicYAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_PicCbAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_PicCrAddr, 
//				gsVpuEncInOut_Info.gsVpuEncInput.m_iForceIPicture, gsVpuEncInOut_Info.gsVpuEncInput.m_iSkipPicture, gsVpuEncInOut_Info.gsVpuEncInput.m_iQuantParam,
//				gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeRcParamFlag, gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeTargetKbps, gsVpuEncInOut_Info.gsVpuEncInput.m_iChangeFrameRate, 
//				gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize);
		ret = venc_cmd_process(V_ENC_ENCODE, &gsVpuEncInOut_Info);

		total_frm++;
//		LOGD("systemtime:: encoded frame");
		
		if( ret != RETCODE_SUCCESS )
		{
			if( ret == RETCODE_WRAP_AROUND )
			{
				LOGE( "[VENC] Warning!! BitStream buffer wrap arounded. prepare more large buffer = %d \n", ret );
			}
			LOGE( "[VENC:Err:0x%x] %d'th VPU_ENC_ENCODE failed \n", ret, gPFrameCnt );
			LOGE( "[VENC:Err] BitAddr 0x%x - 0x%x \n", gsVpuEncInOut_Info.gsVpuEncInput.m_BitstreamBufferAddr, gsVpuEncInOut_Info.gsVpuEncInput.m_iBitstreamBufferSize );
			return -ret;
		}

#ifdef MULTI_SLICES_AVC	
		if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 ) 
		{
			if( enc_avc_aud_enable == 1 && gsiFrameIdx > 0 ) 
			{
				gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[VA] -= 8;
				gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[PA] -= 8;
				gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize += 8;
			}			
		}
#endif

#ifdef HAVE_ANDROID_OS
		// output
		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_AVC)
		{
			// modified by shmin for M2TS
			if(bAvcUsedNALStart)
			{
				p_output_param->m_pBitstreamOut 	= (unsigned char*)encoded_buf_cur_pos[VA];
				p_output_param->m_iBitstreamOutSize = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
				filewrite_memory("/data/enc.dat", (unsigned char*)encoded_buf_cur_pos[VA], gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize);
				save_output_stream("/sdcard/vpu_outEnc.bin", gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize, (unsigned char*)encoded_buf_cur_pos[VA]);
			}
			else
			{	
				p_output_param->m_pBitstreamOut 	= (unsigned char*)encoded_buf_cur_pos[VA];
				p_output_param->m_pBitstreamOut 	+= 4;
				p_output_param->m_iBitstreamOutSize = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize - 4;
			}
		}
		else
		{
			p_output_param->m_pBitstreamOut 	= (unsigned char*)encoded_buf_cur_pos[VA];
			p_output_param->m_iBitstreamOutSize = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
		}

#ifdef MULTI_SLICES_AVC 
		if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 )
		{
			unsigned char *p = p_output_param->m_pBitstreamOut;
			unsigned int *pSliceSize;
			unsigned int extra_size = gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum * sizeof(unsigned int);
			
			DSTATUS("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);

			pSliceSize = (unsigned int*)(p_output_param->m_pBitstreamOut + p_output_param->m_iBitstreamOutSize + STABILITY_GAP);
			pSliceSize = ALIGNED_BUFF(pSliceSize, 256);
			p_output_param->m_pSliceInfo = (unsigned int*)pSliceSize;
			p_output_param->m_iSliceCount = 0;

			if( gsVpuEncInit_Info.gsVpuEncInit.m_stRcInit.m_iSliceMode == 1 ) 
			{
				p_output_param->m_iSliceCount = gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum;
				if( gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum > 1 && gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr != NULL) 
				{
					int i;
					int iSize;
					unsigned int * pSliceParaBuf;
					unsigned char *pS;
					unsigned int total_bytes = 0;
			
					pS = (unsigned char*)p_output_param->m_pBitstreamOut;
					
					iSize = gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoSize;
					
					DSTATUS("[EncSliceNum:%3d], Addr - 0x%x[0x%x/0x%x]", gsVpuEncInOut_Info.gsVpuEncOutput.m_iSliceInfoNum,
								gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr, enc_slice_info_addr[VA], enc_slice_info_addr[K_VA]);
			
					pSliceParaBuf = (unsigned int *)(enc_slice_info_addr[VA] + (gsVpuEncInOut_Info.gsVpuEncOutput.m_SliceInfoAddr - enc_slice_info_addr[K_VA]));
			
					for( i=0 ; i<iSize/8 ; i++, pSliceParaBuf += 2 ) 
					{
						int nMbAddr, nSliceBits;
						
						nMbAddr = pSliceParaBuf[1]&0x00FFFF;
						nSliceBits = pSliceParaBuf[0];
						pSliceSize[i] = nSliceBits / 8;

						if( enc_avc_aud_enable == 1  && gsiFrameIdx > 0 && i == 0){
							pSliceSize[i] += 8;
						}
#if 0
						if(gsiFrameIdx < 5)
						{
							LOGD(" slice[%d] = %d", i, pSliceSize[i]);
						}
						else
						{
							if(pSliceSize[i] > 1500){
								LOGE("[%d'th frames :: OverSlice[%d] = %d", gsiFrameIdx, i, pSliceSize[i]);
							}
						}
#endif				
						if(gsiFrameIdx < 5 && DEBUG_ON == 1)
						{					
							DSTATUS( "[%2d] mbAddr.%3d, Bits.%d\n", i, nMbAddr, nSliceBits );
							DSTATUS( "		0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", pS[total_bytes+0], pS[total_bytes+1], pS[total_bytes+2], 
													pS[total_bytes+3], pS[total_bytes+4], pS[total_bytes+5], pS[total_bytes+6], pS[total_bytes+7]);
							total_bytes += (nSliceBits/8);
							DSTATUS( "	~	0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", pS[total_bytes-8], pS[total_bytes-7], pS[total_bytes-6], 
													pS[total_bytes-5], pS[total_bytes-4], pS[total_bytes-3], pS[total_bytes-2], pS[total_bytes-1]);
						}
						
					}
				}
				else
				{
					p_output_param->m_iSliceCount = 1;
					pSliceSize[0] = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
				}
			}

			encoded_buf_cur_pos[PA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
			encoded_buf_cur_pos[VA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
			encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, ALIGN_LEN);
		}			
		else
#endif
		{
	#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
			if(!bChanged_fps && /*gsiFrameIdx != 0 && */((gsiFrameIdx%gsVpuEncInit_Info.gsVpuEncInit.m_iFrameRate) == 0))
			{
				DSTATUS("Inserted sequence header prior to Stream.")
				p_output_param->m_pBitstreamOut -= seq_len;
				memcpy(p_output_param->m_pBitstreamOut, seq_backup, seq_len);
				p_output_param->m_iBitstreamOutSize += seq_len;
			}
	#endif
			encoded_buf_cur_pos[PA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
			encoded_buf_cur_pos[VA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
			encoded_buf_cur_pos[K_VA] += ALIGNED_BUFF(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), ALIGN_LEN);
		}
				
		p_output_param->m_iPicType = gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType;
#else
		// 인코딩 결과 write 하는 부분
		if( gsiPictureDataSize < gsVpuEncOutput.m_iBitstreamOutSize )
		{
			void* new_ptr;
			gsiPictureDataSize = gsVpuEncOutput.m_iBitstreamOutSize;
			new_ptr = cdk_realloc( gspPictureData, gsiPictureDataSize );
			if( new_ptr == NULL ) 
			{
				LOGE( "[VENC:Err:0x%x] gspPictureData realloc failed \n", CDK_ERROR_REALLOC );
				return CDK_ERROR_REALLOC;
			}

			memset( new_ptr, 0, gsiPictureDataSize );
			gspPictureData = new_ptr;
		}

		memcpy( gspPictureData, gsVpuEncOutput.m_BitstreamOut[VA], gsVpuEncOutput.m_iBitstreamOutSize );

		// output
		p_output_param->m_pBitstreamOut		= gspPictureData;
		p_output_param->m_iBitstreamOutSize = gsVpuEncOutput.m_iBitstreamOutSize;
#endif

		if( gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType == PIC_TYPE_I )
		{		
#ifdef HAVE_ANDROID_OS
			keyInterval_cnt = 1;
#endif
			if(gMaxOutputSize_IFrame < gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize)
				gMaxOutputSize_IFrame = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
			
			LOGI("[I:%4d/%4d] [interval %d] = %d/%d, P=%d!", gIFrameCnt, gsiFrameIdx, gsVpuEncInit_Info.gsVpuEncInit.m_iKeyInterval, gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize, gMaxOutputSize_IFrame, gMaxOutputSize_PFrame);
			DSTATUS( "[I:%4d/%4d] Byte:%7d(%5.1lfK) ", gIFrameCnt, gsiFrameIdx, gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize, gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
			gIFrameCnt++;
		}
		else if( gsVpuEncInOut_Info.gsVpuEncOutput.m_iPicType == PIC_TYPE_P )
		{
#ifdef HAVE_ANDROID_OS
			keyInterval_cnt++;
#endif
			if(gMaxOutputSize_PFrame < gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize)
				gMaxOutputSize_PFrame = gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;

			if(gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize < 20){
//				DSTATUS( "[P:%4d/%4d] Byte:%7d(%5.1lfK) ", gPFrameCnt, gsiFrameIdx, gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize, gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
			}
			gPFrameCnt++;
		}
		gsiFrameIdx++;
#ifdef CHECK_BITRATE		
		total_size += gsVpuEncInOut_Info.gsVpuEncOutput.m_iBitstreamOutSize;
#endif

#ifdef INSERT_SEQ_HEADER_IN_FORCED_IFRAME
		if( gsVpuEncInit_Info.gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 && bChanged_fps)
		{
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderType = MPEG4_VOL_HEADER;
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_HeaderAddr  = gspSeqHeaderAddr[PA];			
			gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize = gsiSeqHeaderSize;
			
			DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for MPEG4_VOL_HEADER \n");
			ret = venc_cmd_process(V_ENC_PUT_HEADER, &gsVpuEncPutHeader_Info);
			
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:0x%x] venc_vpu MPEG4_VOL_HEADER failed \n", ret );
			}
			else
			{
				if(seq_backup == NULL){
					seq_backup = (unsigned char*)TCC_malloc(gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize + (STABILITY_GAP*2));
				}
				memcpy(seq_backup, gspSeqHeaderAddr[VA], gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize);
				seq_len = gsVpuEncPutHeader_Info.gsVpuEncHeader.m_iHeaderSize;
			}
		}
#endif

		#if 0
		DSTATUS( "0x%x ", gsVpuEncInOut_Info.gsVpuEncOutput.m_BitstreamOut[VA] );
		{
			int temp = 0;
			DSTATUS("Data:");
			for( temp = 0; temp < 32; temp+=4 )
			{
				DSTATUS("0x%02X", gspPictureData[temp+0] );
				DSTATUS(  "%02X", gspPictureData[temp+1] );
				DSTATUS(  "%02X", gspPictureData[temp+2] );
				DSTATUS(  "%02X", gspPictureData[temp+3] );
				DSTATUS(  " " );
			}
		}
		#endif
	}
	else if( iOpCode == VENC_CLOSE )
	{
		ret = venc_cmd_process(V_ENC_CLOSE, NULL);
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VENC] VPU_ENC_CLOSE failed Error code is 0x%x \n", ret );
		}

		if( gsBitstreamBufAddr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)gsBitstreamBufAddr[VA], gsBitstreamBufSize ) >= 0)
				gsBitstreamBufAddr[VA] = 0;
		}
	
		if( gsBitWorkBufAddr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)gsBitWorkBufAddr[VA], gsBitWorkBufSize ) >= 0)
				gsBitWorkBufAddr[VA] = 0;
		}
		if( gsMESearchBufAddr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)gsMESearchBufAddr[VA], gsMESearchBufSize ) >= 0)
				gsMESearchBufAddr[VA] = 0;
		}
		if( gsFrameBufAddr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)gsFrameBufAddr[VA], gsFrameBufSize ) >= 0)
				gsFrameBufAddr[VA] = 0;
		}

#ifdef MULTI_SLICES_AVC	
		if( enc_slice_info_addr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)enc_slice_info_addr[VA], enc_slice_info_size ) >= 0)
				enc_slice_info_addr[VA] = 0;
		}
#endif

#ifdef HAVE_ANDROID_OS
		if( gspSeqHeaderAddr[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)gspSeqHeaderAddr[VA], gsiSeqHeaderSize) >= 0)
				gspSeqHeaderAddr[VA] = 0;
		}

		if( encoded_buf_base_pos[VA] )
		{
			if(cdk_sys_free_virtual_addr( NULL, (void*)encoded_buf_base_pos[VA], encoded_buf_size) >= 0)
				encoded_buf_base_pos[VA] = 0;
		}

		vpu_env_close();
#else
		if( gspSeqHeader )
		{
			cdk_free( gspSeqHeader );
			gspSeqHeader = NULL;
		}
		if( gspPictureData )
		{
			cdk_free( gspPictureData );
			gspPictureData = NULL;
		}
#endif

#if defined(VPU_CLK_CONTROL)
		vpu_clock_deinit();
#endif
		gsiFrameIdx = 0;
	}
	else
	{
		LOGE( "[VENC] Invalid Operation!!\n" );
		return -ret;
	}

#ifdef DEBUG_TIME_LOG
	end = clock();

	if( iOpCode == VENC_INIT ){
		LOGD("VENC_INIT_TIME %d ms", (end-start)*1000/CLOCKS_PER_SEC);
	}
	else if( iOpCode == VENC_SEQ_HEADER){
		LOGD("VENC_SEQ_TIME %d ms", (end-start)*1000/CLOCKS_PER_SEC);
	}
	else if( iOpCode == VENC_ENCODE )
	{
		dec_time[time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
		total_dec_time += dec_time[time_cnt];
		if(time_cnt != 0 && time_cnt % 29 == 0)
		{
			LOGD("VENC_TIME %2.1f ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
				total_dec_time/(float)total_frm, dec_time[0], dec_time[1], dec_time[2], dec_time[3], dec_time[4], dec_time[5], dec_time[6], dec_time[7], dec_time[8], dec_time[9], 
				dec_time[10], dec_time[11], dec_time[12], dec_time[13], dec_time[14], dec_time[15], dec_time[16], dec_time[17], dec_time[18], dec_time[19], 
				dec_time[20], dec_time[21], dec_time[22], dec_time[23], dec_time[24], dec_time[25], dec_time[26], dec_time[27], dec_time[28], dec_time[29]);
			time_cnt = 0;
		}
		else{
			time_cnt++;
		}
	}
#endif

	return ret;
}

unsigned char *vpu_get_BitstreamBufAddr(unsigned int index)
{
	unsigned char *pBitstreamBufAddr = NULL;

	if (index == PA)
	{
		pBitstreamBufAddr = (unsigned char *)gsBitstreamBufAddr[PA];
	}
	else if (index == VA)
	{
		pBitstreamBufAddr = (unsigned char *)gsBitstreamBufAddr[VA];
	}
	else /* default : PA */
	{
		pBitstreamBufAddr = (unsigned char *)gsBitstreamBufAddr[PA];
	}

	return pBitstreamBufAddr;
}

unsigned char *vpu_getStreamOutPhyAddr(unsigned char *convert_addr, unsigned int base_index)
{
	unsigned char *pBaseAddr; 
	unsigned char *pTargetBaseAddr = NULL;
	unsigned int szAddrGap = 0;

	pTargetBaseAddr = (unsigned char*)encoded_buf_base_pos[PA];

	if (base_index == K_VA)
	{
		pBaseAddr = (unsigned char*)encoded_buf_base_pos[K_VA];
	}
	else /* default : VA */
	{
		pBaseAddr = (unsigned char*)encoded_buf_base_pos[VA];
	}

	szAddrGap = convert_addr - pBaseAddr;

	return (pTargetBaseAddr+szAddrGap);
}

