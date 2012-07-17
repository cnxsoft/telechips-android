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
 *		vdec.c
 * \date
 *		2009/06/01
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		video decoder
 *
 ***********************************************************************
 */
#ifdef HAVE_ANDROID_OS
#define LOG_TAG	"VPU_DEC"
#include <utils/Log.h>

#include "cdk_core.h"
#include "vdec.h"
#include "TCCMemory.h"
#ifdef VPU_CLK_CONTROL
#include "vpu_clk_ctrl.h"
#endif
#ifdef INCLUDE_WMV78_DEC
#include "TCC_WMV78_DEC.h"
#include "TCC_WMV78_DEC_Huff_table.h"
#endif
#ifdef INCLUDE_SORENSON263_DEC
#include "th263dec.h"
#endif

#include <sys/mman.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <libpmap/pmap.h>
#include <mach/tcc_vpu_ioctl.h>


/************************************************************************/
/* TEST and Debugging                                                   */
/************************************************************************/
static int DEBUG_ON	= 0;
#define DPRINTF(msg...)	 LOGE( ": " msg);
#define DSTATUS(msg...)	 if (DEBUG_ON) { LOGD( ": " msg);}
#define DPRINTF_FRAME(msg...) //LOGD(": " msg);

static unsigned int total_frm = 0;

//#define DEBUG_VPU_DEC_INPUT

//#define VIDEO_DEC_PROFILE
#ifdef VIDEO_DEC_PROFILE
#ifdef _TCC9300_
#define	TEST_GPIO_BASE  			0xB010A000
// TCC9300 => DXB_12pin, GPIO_C29
#define GPIO_BASE_OFFSET 0x80
#define GPIO_NUM		 0x20000000
#else
#define	TEST_GPIO_BASE  			0xF0102000
// TCC9200S => CAM_26pin, GPIO_D20
//#define GPIO_BASE_OFFSET 0xC0
//#define GPIO_NUM		 0x00100000
// TCC8900 => DXB_23pin, GPIO_E11
#define GPIO_BASE_OFFSET 0x100
#define GPIO_NUM		 0x00000800
#endif
static unsigned int 				pTestRegBase = 0;

//#define DEBUG_TIME_LOG
#ifdef DEBUG_TIME_LOG
#include "time.h"
static unsigned int dec_time[30] = {0,};
static unsigned int time_cnt = 0;
static unsigned int total_frame_cnt = 0;
static unsigned int total_dec_time = 0;
#endif
#endif

#if defined(_TCC8800_) || defined(_TCC9300_)// || defined(_TCC8920_)
#define VPU_PERFORMANCE_UP //use sram and vcache
#endif
#define TCC_VPU_INPUT_BUF_SIZE 		(1024 * 1024)
//#define MAX_FRAME_BUFFER_COUNT		31 // move to vdec.h

#define MAX_BITSTREAM_SIZE			(2 * 1024 * 1024)

#define USE_VPU_INTERRUPT
//#define VPU_REGISTER_DUMP
//#define VPU_FRAME_DUMP
#else
#include "vdec.h"

#include "../cdk/cdk_core.h"
#include "../cdk/cdk_sys.h"
#include "vpu/TCC_VPU_CODEC.h"
#ifdef VPU_CLK_CONTROL
#include "vpu_clk_ctrl.h"
#endif

#ifdef INCLUDE_SORENSON263_DEC
#include "sorensonH263dec/th263dec.h"
#endif
#endif

static unsigned char vdec_env_opened = 0, vdec_codec_opened = 0;
int dev_fd = -1;
#ifdef HAVE_ANDROID_OS
#define TCC_VPU_DEV_NAME	"/dev/vpu"
#else
#define TCC_VPU_DEV_NAME	"/dev/mem"
#endif

#ifdef USE_VPU_INTERRUPT
#define TCC_INTR_DEV_NAME		"/dev/tcc_intr"//"/dev/tcc-intr-vc"

#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>
int vpu_intr_fd = -1;
struct pollfd tcc_event[1];
#endif

#ifdef INCLUDE_WMV78_DEC	// for WMV78 video decoder
static int gsWMV78DecodedFrmaeBufferDev = 0;
#ifdef HAVE_ANDROID_OS
unsigned int gsWMV78CurYFrameAddress_PA = 0;
unsigned int gsWMV78Ref0YFrameAddress_PA = 0;
#endif
unsigned int gsWMV78CurYFrameAddress = 0;
unsigned int gsWMV78CurUFrameAddress = 0;
unsigned int gsWMV78CurVFrameAddress = 0;
unsigned int gsWMV78Ref0YFrameAddress = 0;
unsigned int gsWMV78Ref0UFrameAddress = 0;
unsigned int gsWMV78Ref0VFrameAddress = 0;
unsigned int gsWMV78DecodedFrameAddress_NC[2] = {0,};

//! Callback Func
typedef struct vdec_callback_func_t
{
	void* (*m_pfMalloc		 ) ( unsigned int );					//!< malloc
	void* (*m_pfNonCacheMalloc) ( unsigned int );					//!< non-cacheable malloc 
	void  (*m_pfFree			 ) ( void* );							//!< free
	void  (*m_pfNonCacheFree	 ) ( void* );							//!< non-cacheable free
	void* (*m_pfMemcpy		 ) ( void*, const void*, unsigned int );//!< memcpy
	void  (*m_pfMemset		 ) ( void*, int, unsigned int );		//!< memset
	void* (*m_pfRealloc		 ) ( void*, unsigned int );				//!< realloc
	void* (*m_pfMemmove		 ) ( void*, const void*, unsigned int );//!< memmove

	void* (*m_pfPhysicalAlloc)		( unsigned int );
	void  (*m_pfPhysicalFree)		( void*, unsigned int );
	void* (*m_pfVirtualAlloc)		( int*, unsigned int, unsigned int );
	void  (*m_pfVirtualFree)		( int*, unsigned int, unsigned int );
	int m_Reserved1[16-12];

	void*		 (*m_pfFopen	) (const char *, const char *);						//!< fopen
	unsigned int (*m_pfFread	) (void*, unsigned int, unsigned int, void* );		//!< fread
	int			 (*m_pfFseek	) (void*, long, int );								//!< fseek
	long		 (*m_pfFtell	) (void* );											//!< ftell
	unsigned int (*m_pfFwrite) (const void*, unsigned int, unsigned int, void*);	//!< fwrite
	int			 (*m_pfFclose) (void *);											//!< fclose
	int			 (*m_pfUnlink) ( const char *);										//!< _unlink
	unsigned int (*m_pfFeof  ) (void *);											//!< feof
	unsigned int (*m_pfFflush) (void *);											//!< fflush

	int			 (*m_pfFseek64) (void*, int64_t, int );								//!< fseek 64bi io
	int64_t		 (*m_pfFtell64) (void* );											//!< ftell 64bi io
	int m_Reserved2[16-11];
} vdec_callback_func_t;
#endif

/************************************************************************/
/* STATIC MEMBERS                                                       */
/************************************************************************/
#ifdef HAVE_ANDROID_OS
static unsigned int 	gsLogicalBase = 0, gsRegisterBase = 0;
static unsigned int 	gsUsedPhysicalMemSize = 0;
static int gsBitstreamBufSize = 0, gsUserdataBufSize = 0, gsMaxBitstreamSize = 0;
static codec_addr_t gsBitstreamBufAddr[2] = {0,}, gsUserdataBufAddr[2] = {0,};
static int gsBitStreamBufDev = 0, gsUserdataBufDev = 0;
//static int gsAdditional_frame = 0;

static pmap_t pmap_video;
#define VIDEO_PHY_ADDR		pmap_video.base
#define VIDEO_MEM_SIZE		pmap_video.size

#ifdef VPU_CLK_CONTROL
static codec_addr_t gsRegBaseVideoBusAddr = 0, gsRegBaseClkCtrlAddr = 0;
#endif
#endif

static int gsBitWorkBufSize = 0, gsSliceBufSize = 0, gsSpsPpsSize = 0, gsFrameBufSize = 0;
static codec_addr_t gsBitWorkBufAddr[2] = {0,}, gsSliceBufAddr = 0, gsSpsPpsAddr = 0, gsFrameBufAddr[2] = {0,};
static int gsBitWorkBufDev = 0, gsFrameBufDev = 0;
//static int gsBitWorkBufDev = 0, gsSliceBufDev = 0, gsSpsPpsDev = 0, gsFrameBufDev = 0;

static codec_handle_t gsVpuDecHandle = 0;
static dec_init_t gsVpuDecInit;
#ifdef HAVE_ANDROID_OS
static dec_user_info_t gsVpuDecUserInfo;
#endif
static dec_initial_info_t gsVpuDecInitialInfo;
static dec_buffer_t gsVpuDecBuffer;
static dec_input_t gsVpuDecInput;
static dec_output_t gsVpuDecOutput;

#ifdef INCLUDE_WMV78_DEC	// for WMV78 video decoder
static int gsWMV78FrameSize = 0, gsWMV78NCFrameSize = 0;
static WMV78_HANDLE			gsWMV78DecHandle = 0;
static WMV78_INIT			gsWMV78DecInit;
static WMV78_INPUT			gsWMV78DecInput;
static WMV78_OUTPUT			gsWMV78DecOutput;
static dec_initial_info_t	gsWMV78DecInitialInfo;
static int gsFirstFrame = 0;
static int gsIsINITdone = 0;
#endif

#ifdef INCLUDE_SORENSON263_DEC
static codec_handle_t gsS263DecHandle = 0;
static dec_init_t gsS263DecInit;
static dec_initial_info_t gsS263DecInitialInfo;
static h263dec_stats_t gsS263DecInitialStats;
static unsigned char *gsS263DecHeapAddr;
#endif

#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC) || defined(JPEG_DECODE_FOR_MJPEG)
#define MAX_NUM_OF_VIDEO_ELEMENT 6
static codec_addr_t decoded_phyAddr[10] = {0,};
static codec_addr_t decoded_virtAddr[10] = {0,};
static unsigned int decoded_buf_maxcnt = 0;
static unsigned int decoded_buf_size = 0;
static unsigned int decoded_buf_curIdx = 0;
#endif

#ifdef JPEG_DECODE_FOR_MJPEG
#include <mach/tcc_jpeg_ioctl.h>

#define JPEGDEC_DEVICE		"/dev/jpegdec"
#define JPEGDEC_STREAM_MAX  (4*1024*1024)
#define HW_DECODE_OVER_MAX_SIZE 4096
#define MAX_NUM_OF_JPEG_ELEMENT 10

static codec_handle_t gsJPEGDecHandle = 0;
#endif

static int gsAdditionalFrameCount = VPU_BUFF_COUNT;
static int gsbHasSeqHeader;
#define LEVEL_MAX		11
#define PROFILE_MAX		6

static const char *strProfile[VCODEC_MAX][PROFILE_MAX] =
{
	//STD_AVC
	{ "Base Profile", "Main Profile", "Extended Profile", "High Profile", "Reserved Profile", "Reserved Profile" },
	//STD_VC1
	{ "Simple Profile", "Main Profile", "Advance Profile", "Reserved Profile", "Reserved Profile", "Reserved Profile" },
	//STD_MPEG2
	{ "High Profile", "Spatial Scalable Profile", "SNR Scalable Profile", "Main Profile", "Simple Profile", "Reserved Profile" },
	//STD_MPEG4
	{ "Simple Profile", "Advanced Simple Profile", "Advanced Code Efficiency Profile", "Reserved Profile", "Reserved Profile", "Reserved Profile" },
	//STD_H263
	{ " ", " ", " ", " ", " ", " " },
	//STD_DIV3
	{ " ", " ", " ", " ", " ", " " },
	//STD_RV
	{ "Real video Version 8", "Real video Version 9", "Real video Version 10", " ", " ", " " },
	//STD_AVS
	{ "Jizhun Profile", " ", " ", " ", " ", " " },
	//STD_WMV78
	{ " ", " ", " ", " ", " ", " " },
	//STD_SORENSON263
	{ " ", " ", " ", " ", " ", " " }
};

static const char *strLevel[VCODEC_MAX][LEVEL_MAX] =
{
	//STD_AVC
	{ "Level_1B", "Level_", "Reserved Level", "Reserved Level", "Reserved Level", "Reserved Level", "Reserved Level", "Reserved Level" },
	//STD_VC1
	{ "Level_L0(LOW)", "Level_L1", "Level_L2(MEDIUM)", "Level_L3", "Level_L4(HIGH)", "Reserved Level", "Reserved Level", "Reserved Level" },
	//STD_MPEG2
	{ "High Level", "High 1440 Level", "Main Level", "Low Level", "Reserved Level", "Reserved Level", "Reserved Level", "Reserved Level" },
	//STD_MPEG4
	{ "Level_L0", "Level_L1", "Level_L2", "Level_L3", "Level_L4", "Level_L5", "Level_L6", "Reserved Level" },
	//STD_H263
	{ "", "", "", "", "", "", "", "" },
	//STD_DIV3
	{ "", "", "", "", "", "", "", "" },
	//STD_RV
	{ "", "", "", "", "", "", "", "" },
	//STD_AVS
	{"2.0 Level", "4.0 Level", "4.2 Level", "6.0 Level", "6.2 Level", "", "", ""},
	//STD_WMV78
	{ "", "", "", "", "", "", "", "" },
	//STD_SORENSON263
	{ "", "", "", "", "", "", "", "" }
};

static void vpu_hw_reset();
static void vpu_env_close(void);

#ifdef DEBUG_VPU_DEC_INPUT
static void PrintVPUHexData(unsigned char* pBuffer, unsigned int length, unsigned char* pTag)
{
	LOGI("[%s:%08d] 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x ~ 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", pTag, length
			, pBuffer[0] , pBuffer[1] , pBuffer[2] , pBuffer[3]
			, pBuffer[4] , pBuffer[5] , pBuffer[6] 
			, pBuffer[7] , pBuffer[8] , pBuffer[9]
			, pBuffer[length - 10] , pBuffer[length - 9] , pBuffer[length - 8]
			, pBuffer[length - 7] , pBuffer[length - 6] , pBuffer[length - 5]
			, pBuffer[length - 4] , pBuffer[length - 3] , pBuffer[length - 2], pBuffer[length - 1]
		);
}
#endif

#ifdef HAVE_ANDROID_OS
static unsigned char * sPhysicalMemSetting(unsigned int nPhysicalAddr, unsigned int size)
{	
	void *map_ptr = MAP_FAILED;

	map_ptr = (void *)mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, nPhysicalAddr);
	
	if(MAP_FAILED == map_ptr)
	{
		LOGE("mmap failed. fd(%d), addr(0x%x), size(%d)", dev_fd, nPhysicalAddr, size);
		return NULL;
	}
	
	return (unsigned char *)map_ptr;
}

static void sPhysicalMemFree(unsigned int LogicalAddr, unsigned int size)
{
	int ret = 0;
	
	if((ret = munmap((void*)LogicalAddr, size)) < 0)
	{
		LOGE("munmap failed. addr(0x%x), size(%d)", LogicalAddr, size);
	}
}

static void *vdec_malloc(unsigned int size)
{
	void * ptr = TCC_malloc(size);
	return ptr;
}

static void vdec_free(void * ptr )
{
	TCC_free(ptr);
}

static void *cdk_sys_malloc_physical_addr(int uiSize)
{
	codec_addr_t phsical_addr = VIDEO_PHY_ADDR + gsUsedPhysicalMemSize;
	gsUsedPhysicalMemSize += uiSize;
	
	if( gsUsedPhysicalMemSize > VIDEO_MEM_SIZE ){
		LOGE("sys_malloc_physical_addr error! %d bytes \n", gsUsedPhysicalMemSize);
	}
	
	return (void*)( phsical_addr );
}

static void cdk_sys_free_physical_addr(void* pPtr, int uiSize)
{
	gsUsedPhysicalMemSize -= uiSize;
	return;
}

static void *cdk_sys_malloc_virtual_addr(int* pDev, codec_addr_t pPtr, int uiSize)
{
	codec_addr_t used_size = pPtr - VIDEO_PHY_ADDR;
	
	return (void*)( gsLogicalBase + used_size);
}

static void cdk_sys_free_virtual_addr(int* pDev, codec_addr_t pPtr, int uiSize)
{
	
}

static unsigned int cdk_sys_remain_memory_size(void)
{
	return (VIDEO_MEM_SIZE - gsUsedPhysicalMemSize);
}

void vpu_update_sizeinfo(unsigned int format, unsigned int bps, unsigned int fps, unsigned int image_width, unsigned int image_height)
{
	CONTENTS_INFO info;

	info.type = VPU_DEC;

#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC)
	if(
	#ifdef INCLUDE_WMV78_DEC							
		format == STD_WMV78 
	#endif			
	#ifdef INCLUDE_SORENSON263_DEC				
		|| format == STD_SORENSON263
	#endif
	)
		info.isSWCodec = 1;
	else
#endif
		info.isSWCodec = 0;
	
	if( format == STD_MJPG )
	{
		info.width = AVAILABLE_MAX_WIDTH;
		info.height = AVAILABLE_MAX_HEIGHT;
	}
	else
	{
		info.width = image_width;
		info.height = image_height;
	}
	info.bitrate = bps;
	info.framerate = fps/1000;
	ioctl(dev_fd, VPU_SET_CLK, &info);

	return;
}

static int vpu_check_for_video(unsigned char open_status, int *ret_vpu_reset)
{
//return value : if hw reset need or not!!
	int vpu_wait;
	OPENED_gINFO gInfo;
	OPENED_sINFO sInfo;

	if(open_status)
	{
		*ret_vpu_reset = 0;

// 1. check open in vdec in case so library was loaded status in the same region!!
		if(vdec_env_opened || vdec_codec_opened)
		{
			//to recover abnormal stop error!!
			LOGE("VPU has been already opened. so have to close!! ");

			if(vdec_codec_opened)
			{
				if(RETCODE_SUCCESS != TCC_VPU_DEC( VPU_DEC_CLOSE, &gsVpuDecHandle, NULL, &gsVpuDecOutput ))
					*ret_vpu_reset = 1;
			}

			if(vdec_env_opened)
				vpu_env_close();
		}

		dev_fd = open(TCC_VPU_DEV_NAME, O_RDWR | O_NDELAY);
		if(!dev_fd)
		{
			LOGE("Err[%s] :: vpu(%s) open failed!", strerror(errno), TCC_VPU_DEV_NAME);
			return -1;
		}

// 2. check open for 3 second in vpu device driver!!
		ioctl(dev_fd, VPU_GET_OPEN_INFO, &gInfo);

		if(gInfo.count != 1)
		{			
			if(gInfo.dmb_opened)
			{
				LOGD("VPU for DMB has been opened. So wait for 3 sec and check per 100ms!!");
				vpu_wait = 30;
				while(vpu_wait > 0)
				{
					usleep(100 * 1000);//100ms				
					ioctl(dev_fd, VPU_GET_OPEN_INFO, &gInfo);
					vpu_wait--;

					if(gInfo.dmb_opened == 0)
					{
						LOGI("VPU for DMB stopped(within %d ms).", (30 - vpu_wait)*100);
						break;
					}
					
					if(vpu_wait <= 0)
					{
						LOGE("VPU for DMB didn't close!!. so assume to close!!");
						
						sInfo.type = OPEN_DMB;
						sInfo.opened_cnt = 0;
						ioctl(dev_fd, VPU_SET_OPEN_INFO, &sInfo);
						
						*ret_vpu_reset = 1;
						break;
					}
				}
			}
			
			if(gInfo.vid_opened)
			{
				//in case video open!!
				LOGD("VPU for VIDEO has been opened. So wait for 3 sec and check per 100ms!!");
				vpu_wait = 30;
				while(vpu_wait > 0)
				{				
					usleep(100 * 1000);//100ms
					ioctl(dev_fd, VPU_GET_OPEN_INFO, &gInfo);
					vpu_wait--;

					if(gInfo.vid_opened == 0)
					{
						LOGI("VPU for VIDEO stopped(within %d ms).", (30 - vpu_wait)*100);
						break;
					}
					
					if(vpu_wait <= 0)
					{
						LOGE("VPU for VIDEO didn't close!!. so assume to close!!");
						
						sInfo.type = OPEN_VIDEO;
						sInfo.opened_cnt = 0;
						ioctl(dev_fd, VPU_SET_OPEN_INFO, &sInfo);
						
						*ret_vpu_reset = 1;
						break;
					}
				}
			}
		}
	}

	sInfo.type = OPEN_VIDEO;
	if(open_status)
		sInfo.opened_cnt = 1;
	else
		sInfo.opened_cnt = 0;
	ioctl(dev_fd, VPU_SET_OPEN_INFO, &sInfo);

	return 0;
}

static int vpu_env_open(unsigned int format, unsigned int bps, unsigned int fps, unsigned int image_width, unsigned int image_height)
{
	int vpu_reset = 0;
	
	DSTATUS("In  %s \n",__func__);

	pmap_get_info("video", &pmap_video);

	if(vpu_check_for_video(1, &vpu_reset) < 0)
		goto err;
	
	vpu_update_sizeinfo(format, bps, fps, image_width, image_height);
	//reset vpu after clock enable in vpu_update_sizeinfo function!!
#if !defined(_TCC8800_) //to init vpu codec!!
	vpu_reset = 1;
#endif
	if(vpu_reset)
	{
		vpu_hw_reset();
	}

#ifdef  USE_VPU_INTERRUPT
	vpu_intr_fd = open(TCC_INTR_DEV_NAME, O_RDWR);
	if (vpu_intr_fd < 0) {
		LOGE("%s open error", TCC_INTR_DEV_NAME);
		goto err;
	}	
#endif

	gsLogicalBase = (unsigned int)sPhysicalMemSetting(VIDEO_PHY_ADDR, VIDEO_MEM_SIZE);
	if(!gsLogicalBase)
		goto err;

	gsRegisterBase	= (unsigned int)sPhysicalMemSetting(VPU_REG_BASE_ADDR, 100*1024);
	if(!gsRegisterBase)
		goto err;	

#if defined(VPU_CLK_CONTROL)
	gsRegBaseVideoBusAddr	= (unsigned int)sPhysicalMemSetting(0xF0702000, 4*1024);
	if(!gsRegBaseVideoBusAddr)
		goto err;	

	gsRegBaseClkCtrlAddr	= (unsigned int)sPhysicalMemSetting(0xF0700000, 4*1024);
	if(!gsRegBaseClkCtrlAddr)
		goto err;	
#endif

	DSTATUS("[VDEC] gsLogicalBase = 0x%x, gsRegisterBase = 0x%x", (codec_addr_t)gsLogicalBase, (codec_addr_t)gsRegisterBase);
	gsUsedPhysicalMemSize = 0;

#ifdef VIDEO_DEC_PROFILE
	pTestRegBase	= sPhysicalMemSetting(TEST_GPIO_BASE, 1024);
	if(!pTestRegBase)
		goto err;	
#ifdef DEBUG_TIME_LOG	
	time_cnt = 0;
	total_frame_cnt = 0;
	total_dec_time = 0;
#endif
#endif

	vdec_env_opened = 1;

	memset(&gsVpuDecInit, 0x00, sizeof(dec_init_t));
	memset(&gsVpuDecBuffer, 0x00, sizeof(dec_buffer_t));
	memset(&gsVpuDecInput, 0x00, sizeof(dec_input_t));
//	memset(&gsVpuDecUserInfo, 0x00, sizeof(dec_user_info_t));
//	memset(&gsVpuDecOutput, 0x00, sizeof(dec_output_t));

	memset(&gsBitstreamBufAddr, 0x00, sizeof(gsBitstreamBufAddr));
	memset(&gsUserdataBufAddr, 0x00, sizeof(gsUserdataBufAddr));
	memset(&gsBitWorkBufAddr, 0x00, sizeof(gsBitWorkBufAddr));
	memset(&gsFrameBufAddr, 0x00, sizeof(gsFrameBufAddr));
	gsSliceBufAddr = gsSpsPpsAddr = 0;

#ifdef INCLUDE_WMV78_DEC
	memset(&decoded_phyAddr, 0x00, sizeof(decoded_phyAddr));
	memset(&decoded_virtAddr, 0x00, sizeof(decoded_virtAddr));
#endif
	gsAdditionalFrameCount = VPU_BUFF_COUNT;

	DSTATUS("Out  %s \n",__func__);
	total_frm = 0;

	return 0;

err:	
	LOGE("vpu_env_open error");
	vpu_env_close();
	
	return -1;	
}


static void vpu_env_close(void)
{
	int vpu_reset;
	
	DSTATUS("In  %s \n",__func__);

	if(gsLogicalBase)
		sPhysicalMemFree(gsLogicalBase, VIDEO_MEM_SIZE);

	if(gsRegisterBase)
		sPhysicalMemFree(gsRegisterBase, 100*1024);

	gsLogicalBase = 0;
	gsRegisterBase = 0;

#if defined(VPU_CLK_CONTROL)
	if(gsRegBaseVideoBusAddr)
		sPhysicalMemFree(gsRegBaseVideoBusAddr, 4*1024);

	if(gsRegBaseClkCtrlAddr)
		sPhysicalMemFree(gsRegBaseClkCtrlAddr, 4*1024);

	gsRegBaseVideoBusAddr = 0;
	gsRegBaseClkCtrlAddr = 0;	
#endif

#ifdef VIDEO_DEC_PROFILE
	if(pTestRegBase)
		sPhysicalMemFree(pTestRegBase, 1024);	

	pTestRegBase = 0;
#endif

#ifdef  USE_VPU_INTERRUPT
	if(vpu_intr_fd > 0)
	{
		if(close(vpu_intr_fd) < 0)
		{
			LOGE("Err[%s] :: %s close error", strerror(errno), TCC_INTR_DEV_NAME);
		}
		vpu_intr_fd = -1;
	}
#endif	

	if(dev_fd)
	{		
		OPENED_sINFO sInfo;
		
		sInfo.type = OPEN_VIDEO;
		sInfo.opened_cnt = 0;
		ioctl(dev_fd, VPU_SET_OPEN_INFO, &sInfo);

		if(close(dev_fd) < 0)
		{
			LOGE("Err[%s] :: vpu(%s) close failed!", strerror(errno), TCC_VPU_DEV_NAME);
		}
		dev_fd = -1;
	}

	vdec_env_opened = 0;

#ifdef DEBUG_TIME_LOG		
	LOGD("========== Average decode time : %d ==========",total_dec_time/total_frame_cnt);
#endif

	DSTATUS("Out  %s \n",__func__);

}

#ifdef VPU_REGISTER_DUMP
static unsigned char bFirst_frame = 1;
static void filewrite_memory(char* name, char* addr, unsigned int size)
{
	FILE *fp;

	if(!bFirst_frame)
		return;
	
	fp = fopen(name, "ab+");		
	fwrite( addr, size, 1, fp);
	fclose(fp);
}
#endif

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
			LOGE("vpu poll timeout: %d'th frames", total_frm);
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

void vpu_set_additional_refframe_count(int count)
{
	gsAdditionalFrameCount = count;
}

static void print_frame_data(char* name, int size)
{
#ifdef VPU_FRAME_DUMP
	int i;
	unsigned char* ps = (unsigned char*)gsBitstreamBufAddr[VA];

	if(0)
	{
		FILE *fp;
		fp = fopen(name, "ab+");		  
		fwrite( ps, size, 1, fp);
		fclose(fp);
	}

	for(i=0; (i+10 <size) && (i+10 < 100); i += 10){
		DPRINTF_FRAME( "[VDEC - Stream] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", ps[i], ps[i+1], ps[i+2], ps[i+3], ps[i+4], ps[i+5], ps[i+6], ps[i+7], ps[i+8], ps[i+9] );
	}
#endif	
}		

unsigned char *vpu_getBitstreamBufAddr(unsigned int index)
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
#endif

static void vpu_hw_reset()
{
	codec_addr_t ckc_swrst, vidbus_swrst;
	unsigned int ckc_swrst_offset, vidbus_swrst_offset;

#ifdef _TCC9300_
	ckc_swrst_offset = 0x5C;
 	vidbus_swrst_offset = 0x04;
#else
	ckc_swrst_offset = 0x44;
	vidbus_swrst_offset = 0x04;
#endif

	ckc_swrst   = (unsigned int)sPhysicalMemSetting(VIDEOCODECBUS_CLK_BASE_ADDR, 100);   
	if(!ckc_swrst)
	   return;
	
	vidbus_swrst   = (unsigned int)sPhysicalMemSetting(VIDEOBUSCONF_BASE_ADDR, 100);
	if(!vidbus_swrst)
	   return;

	LOGI(" **************** addr:: ckc 0x%x, vid 0x%x ", ckc_swrst, vidbus_swrst);
	// reset
#if !defined(_TCC9300_) && !defined(_TCC8800_) // to prevent before chip-revision!!
	*((volatile unsigned int *)(ckc_swrst + ckc_swrst_offset))   |= (unsigned int)0x00000040;
#endif
	*((volatile unsigned int *)(vidbus_swrst + vidbus_swrst_offset)) |= (unsigned int)0x00000004;
	
	//wait until reset for 1 ms
	usleep( 1 * 1000 );

	// not-reset
#if !defined(_TCC9300_) && !defined(_TCC8800_) // to prevent before chip-revision!!
	*((volatile unsigned int *)(ckc_swrst + ckc_swrst_offset))   &= (unsigned int)(~(0x00000040));
#endif
	*((volatile unsigned int *)(vidbus_swrst + vidbus_swrst_offset)) &= (unsigned int)(~(0x00000004));

	//wait until VPU reboot for 10 ms
	usleep( 10 * 1000 );

	sPhysicalMemFree(ckc_swrst, 100);
	sPhysicalMemFree(vidbus_swrst, 100);

	LOGI(" **************** VPU H/W Reset ****************");
    //restart VPU process..
}

static void print_dec_initial_info( dec_init_t* pDecInit, dec_initial_info_t* pInitialInfo )
{
	unsigned int fRateInfoRes = pInitialInfo->m_uiFrameRateRes;
	unsigned int fRateInfoDiv = pInitialInfo->m_uiFrameRateDiv;
	int userDataEnable = 0;
	int profile = 0;
	int level =0;

	DSTATUS("---------------VIDEO INITIAL INFO-----------------\n");
	if (pDecInit->m_iBitstreamFormat == STD_MPEG4) {
		DSTATUS("[VDEC]Data Partition Enable Flag [%1d]\n", pInitialInfo->m_iM4vDataPartitionEnable);
		DSTATUS("[VDEC]Reversible VLC Enable Flag [%1d]\n", pInitialInfo->m_iM4vReversibleVlcEnable);
		if (pInitialInfo->m_iM4vShortVideoHeader) {			
			DSTATUS("[VDEC]Short Video Header\n");
			DSTATUS("[VDEC]AnnexJ Enable Flag [%1d]\n", pInitialInfo->m_iM4vH263AnnexJEnable);
		} else
			DSTATUS("[VDEC]Not Short Video\n");		
	}

	switch(pDecInit->m_iBitstreamFormat) {
	case STD_MPEG2:
		profile = (pInitialInfo->m_iProfile==0 || pInitialInfo->m_iProfile>5) ? 5 : (pInitialInfo->m_iProfile-1);
		level = pInitialInfo->m_iLevel==4 ? 0 : pInitialInfo->m_iLevel==6 ? 1 : pInitialInfo->m_iLevel==8 ? 2 : pInitialInfo->m_iLevel==10 ? 3 : 4;
		break;
	case STD_MPEG4:
		if (pInitialInfo->m_iLevel & 0x80) 
		{
			// if VOS Header 

			if (pInitialInfo->m_iLevel == 8 && pInitialInfo->m_iProfile == 0) {
				level = 0; profile = 0; // Simple, Level_L0
			} else {
				switch(pInitialInfo->m_iProfile) {
					case 0xB:	profile = 2; break;
					case 0xF:	if( (pInitialInfo->m_iLevel&8) == 0) 
									profile = 1; 
								else
									profile = 5;
								break;
					case 0x0:	profile = 0; break;
					default :	profile = 5; break;
				}
				level = pInitialInfo->m_iLevel;
			}
			
			DSTATUS("[VDEC]VOS Header:%d, %d\n", profile, level);
		} 
		else 
		{ 
			// Vol Header Only
			level = 7; // reserved level
			switch(pInitialInfo->m_iProfile) {
				case  0x1: profile = 0; break; // simple object
				case  0xC: profile = 2; break; // advanced coding efficiency object
				case 0x11: profile = 1; break; // advanced simple object
				default  : profile = 5; break; // reserved
			}
			DSTATUS("[VDEC]VOL Header:%d, %d\n", profile, level);
		}

		if( level > 7 )
			level = 0;
		break;
	case STD_VC1:
		profile = pInitialInfo->m_iProfile;
		level = pInitialInfo->m_iLevel;
		break;
	case STD_AVC:
		profile = (pInitialInfo->m_iProfile==66) ? 0 : (pInitialInfo->m_iProfile==77) ? 1 : (pInitialInfo->m_iProfile==88) ? 2 : (pInitialInfo->m_iProfile==100) ? 3 : 4;
		if(profile<3) {
			// BP, MP, EP
			level = (pInitialInfo->m_iLevel==11 && pInitialInfo->m_iAvcConstraintSetFlag[3] == 1) ? 0 /*1b*/ 
				: (pInitialInfo->m_iLevel>=10 && pInitialInfo->m_iLevel <= 51) ? 1 : 2;
		} else {
			// HP
			level = (pInitialInfo->m_iLevel==9) ? 0 : (pInitialInfo->m_iLevel>=10 && pInitialInfo->m_iLevel <= 51) ? 1 : 2;
		}
		
		break;
	case STD_RV:
		profile = pInitialInfo->m_iProfile - 8;
		level = pInitialInfo->m_iLevel;
		break;
	case STD_H263:
		profile = pInitialInfo->m_iProfile;
		level = pInitialInfo->m_iLevel;
		break;
	case STD_DIV3:
		profile = pInitialInfo->m_iProfile;
		level = pInitialInfo->m_iLevel;
		break;
#if defined(INCLUDE_WMV78_DEC) || defined(INCLUDE_SORENSON263_DEC) 
#ifdef INCLUDE_SORENSON263_DEC
	case STD_SORENSON263:
#endif
#ifdef INCLUDE_WMV78_DEC
	case STD_WMV78:
#endif
		profile = 0;
		level = 0;
		break;
#endif
	default: // STD_MJPG
		;
	}

	if( level >= LEVEL_MAX )
	{
		DSTATUS("[VDEC]Invalid \"level\" value: %d", level);
		level = 0;
	}
	if( profile >= PROFILE_MAX )
	{
		DSTATUS("[VDEC]Invalid \"profile\" value: %d", profile);
		profile = 0;
	}
	if( pDecInit->m_iBitstreamFormat >= VCODEC_MAX )
	{
		DSTATUS("[VDEC]Invalid \"m_iBitstreamFormat\" value: %d", pDecInit->m_iBitstreamFormat);
		pDecInit->m_iBitstreamFormat = 0;
	}

	// No Profile and Level information in WMV78
	if( (pDecInit->m_iBitstreamFormat != STD_WMV78) &&
		(pDecInit->m_iBitstreamFormat != STD_MJPG)
	)
	{
		DSTATUS("[VDEC]%s\n\r", strProfile[pDecInit->m_iBitstreamFormat][profile]);
		if (pDecInit->m_iBitstreamFormat != STD_RV) { // No level information in Rv.
			if (pDecInit->m_iBitstreamFormat == STD_AVC && level != 0 && level != 2){
				DSTATUS("[VDEC]%s%.1f\n\r", strLevel[pDecInit->m_iBitstreamFormat][level], (float)pInitialInfo->m_iLevel/10);
			}
			else{
				DSTATUS("[VDEC]%s\n\r", strLevel[pDecInit->m_iBitstreamFormat][level]);
			}
		}
	}
	
	if(pDecInit->m_iBitstreamFormat == STD_AVC) {
		DSTATUS("[VDEC]frame_mbs_only_flag : %d\n", pInitialInfo->m_iInterlace);
	} else if (pDecInit->m_iBitstreamFormat != STD_RV) {// No interlace information in Rv.
		if (pInitialInfo->m_iInterlace){
			DSTATUS("[VDEC]%s\n", "Interlaced Sequence");
		}
		else{
			DSTATUS("[VDEC]%s\n", "Progressive Sequence");
		}
	}

	if (pDecInit->m_iBitstreamFormat == STD_VC1) {
		if (pInitialInfo->m_iVc1Psf){
			DSTATUS("[VDEC]%s\n", "VC1 - Progressive Segmented Frame");
		}
		else{
			DSTATUS("[VDEC]%s\n", "VC1 - Not Progressive Segmented Frame");
		}
	}

	DSTATUS("[VDEC]Aspect Ratio [%1d]\n", pInitialInfo->m_iAspectRateInfo);
				
	switch (pDecInit->m_iBitstreamFormat) {
	case STD_AVC:
        	DSTATUS("[VDEC]H.264 Profile:%d Level:%d FrameMbsOnlyFlag:%d\n",
			pInitialInfo->m_iProfile, pInitialInfo->m_iLevel, pInitialInfo->m_iInterlace);
		
		if(pInitialInfo->m_iAspectRateInfo) {
			int aspect_ratio_idc;
			int sar_width, sar_height;

			if( (pInitialInfo->m_iAspectRateInfo>>16)==0 ) {
				aspect_ratio_idc = (pInitialInfo->m_iAspectRateInfo & 0xFF);
				DSTATUS("[VDEC]aspect_ratio_idc :%d\n", aspect_ratio_idc);
			}
			else {
				sar_width  = (pInitialInfo->m_iAspectRateInfo >> 16);
				sar_height  = (pInitialInfo->m_iAspectRateInfo & 0xFFFF);
				DSTATUS("[VDEC]sar_width  : %d\nsar_height : %d", sar_width, sar_height);				
			}
		} else {
			DSTATUS("[VDEC]Aspect Ratio is not present");
		}

		break;
	case STD_VC1:
		if(pInitialInfo->m_iProfile == 0){
			DSTATUS("[VDEC]VC1 Profile: Simple\n");
		}
		else if(pInitialInfo->m_iProfile == 1){
			DSTATUS("[VDEC]VC1 Profile: Main\n");
		}
		else if(pInitialInfo->m_iProfile == 2){
			DSTATUS("[VDEC]VC1 Profile: Advanced\n");
		}
		
		DSTATUS("[VDEC]Level: %d Interlace: %d PSF: %d\n", 
			pInitialInfo->m_iLevel, pInitialInfo->m_iInterlace, pInitialInfo->m_iVc1Psf);

		if(pInitialInfo->m_iAspectRateInfo){
			DSTATUS("[VDEC]Aspect Ratio [X, Y]:[%3d, %3d]\n", (pInitialInfo->m_iAspectRateInfo>>8)&0xff,
					(pInitialInfo->m_iAspectRateInfo)&0xff);
		}
		else{
			DSTATUS("[VDEC]Aspect Ratio is not present");
		}


		break;
	case STD_MPEG2:
        	DSTATUS("[VDEC]Mpeg2 Profile:%d Level:%d Progressive Sequence Flag:%d\n",
			pInitialInfo->m_iProfile, pInitialInfo->m_iLevel, pInitialInfo->m_iInterlace);
		// Profile: 3'b101: Simple, 3'b100: Main, 3'b011: SNR Scalable, 
		// 3'b10: Spatially Scalable, 3'b001: High
		// Level: 4'b1010: Low, 4'b1000: Main, 4'b0110: High 1440, 4'b0100: High
		if(pInitialInfo->m_iAspectRateInfo){
			DSTATUS("[VDEC]Aspect Ratio Table index :%d\n", pInitialInfo->m_iAspectRateInfo);
		}
		else{
			DSTATUS("[VDEC]Aspect Ratio is not present");
		}
        	break;

	case STD_MPEG4:
        	DSTATUS("[VDEC]Mpeg4 Profile: %d Level: %d Interlaced: %d\n",
			pInitialInfo->m_iProfile, pInitialInfo->m_iLevel, pInitialInfo->m_iInterlace);
		// Profile: 8'b00000000: SP, 8'b00010001: ASP
		// Level: 4'b0000: L0, 4'b0001: L1, 4'b0010: L2, 4'b0011: L3, ...
		// SP: 1/2/3/4a/5/6, ASP: 0/1/2/3/4/5
		
		if(pInitialInfo->m_iAspectRateInfo){
			DSTATUS("[VDEC]Aspect Ratio Table index :%d\n", pInitialInfo->m_iAspectRateInfo);
		}
		else{
			DSTATUS("[VDEC]Aspect Ratio is not present");
		}
		break;

	case STD_RV:
        	DSTATUS("[VDEC]Real Video Version %d\n",	pInitialInfo->m_iProfile);
        	break;

#ifdef INCLUDE_SORENSON263_DEC
	case STD_SORENSON263:
			DSTATUS("[VDEC]Sorenson's H.263 \n");
        	break;
#endif

	case STD_MJPG:
		{
			const char mjpegFormatList[5][50] = { {"4:2:0"}, {"4:2:2"}, {"4:2:2 vertical"},{"4:4:4"},{"4:0:0"}};
			DSTATUS("[VDEC] MJPEG \n");
			DSTATUS("[VDEC] WIDTH: %d, HEIGHT: %d, SOURCE_FORMAT:%d %s\n", 
			pInitialInfo->m_iPicWidth,
			pInitialInfo->m_iPicHeight,
			pInitialInfo->m_iMjpg_sourceFormat,
			mjpegFormatList[pInitialInfo->m_iMjpg_sourceFormat] );
		}
		break;

   	}

	if (pDecInit->m_iBitstreamFormat == STD_RV) // RV has no user data
		userDataEnable = 0;


	DSTATUS("[VDEC]Dec InitialInfo =>\n m_iPicWidth: %u\n m_iPicHeight: %u\n frameRate: %.2f\n frRes: %u\n frDiv: %u\n",
		pInitialInfo->m_iPicWidth, pInitialInfo->m_iPicHeight, (double)fRateInfoRes/fRateInfoDiv, fRateInfoRes, fRateInfoDiv);

#ifdef INCLUDE_SORENSON263_DEC
	if (pDecInit->m_iBitstreamFormat == STD_SORENSON263) {		
		int disp_width = pInitialInfo->m_iPicWidth;
		int disp_height = pInitialInfo->m_iPicHeight;
		int crop_left = pInitialInfo->m_iAvcPicCrop.m_iCropLeft;
		int crop_right = pInitialInfo->m_iAvcPicCrop.m_iCropRight;
		int crop_top = pInitialInfo->m_iAvcPicCrop.m_iCropTop;
		int crop_bottom = pInitialInfo->m_iAvcPicCrop.m_iCropBottom;

		if( crop_left | crop_right | crop_top | crop_bottom ){
			disp_width = disp_width - ( crop_left + crop_right );
			disp_height = disp_height - ( crop_top + crop_bottom );

			DSTATUS("[VDEC]Dec InitialInfo =>\n Display_PicWidth: %u\n Display_PicHeight: %u\n",
				disp_width, disp_height);
		}
	}
#endif

	DSTATUS("---------------------------------------------------\n");
	
}

int
vpu_sorensonH263dec_ready( dec_init_t* psVDecInit )
{
#ifdef HAVE_ANDROID_OS
		//------------------------------------------------------------
		//! [x] bitstream buffer for each VPU decoder
		//------------------------------------------------------------
		gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
		gsBitstreamBufSize = ALIGNED_BUFF( gsBitstreamBufSize, 4*1024 );
		gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitstreamBufSize );
		if( gsBitstreamBufAddr[PA] == 0 ) 
		{
			DPRINTF( "[VDEC] bitstream_buf_addr[PA] malloc() failed \n");
			return -1;
		}
		DSTATUS( "[VDEC] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitStreamBufDev, gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		if( gsBitstreamBufAddr[VA] == 0 ) 
		{
			DPRINTF( "[VDEC] bitstream_buf_addr[VA] malloc() failed \n");
			return -1;
		}
		memset( (void*)gsBitstreamBufAddr[VA], 0x00 , gsBitstreamBufSize);
		DSTATUS("[VDEC] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[VA], gsBitstreamBufSize );
	
		psVDecInit->m_BitstreamBufAddr[PA]	= gsBitstreamBufAddr[PA];
		psVDecInit->m_BitstreamBufAddr[VA]	= gsBitstreamBufAddr[VA];
		psVDecInit->m_iBitstreamBufSize 	= gsBitstreamBufSize;
#endif

		return 0;
}

int
vpu_dec_ready( dec_init_t* psVDecInit )
{
#ifdef HAVE_ANDROID_OS
	//------------------------------------------------------------
	//! [x] bitstream buffer for each VPU decoder
	//------------------------------------------------------------
	if(gsVpuDecUserInfo.m_bJpegOnly && gsVpuDecUserInfo.jpg_length > LARGE_STREAM_BUF_SIZE)
		gsBitstreamBufSize = ALIGNED_BUFF( gsVpuDecUserInfo.jpg_length, 64*1024 );
	else
		gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
	gsBitstreamBufSize = ALIGNED_BUFF( gsBitstreamBufSize, 4*1024 );
	gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitstreamBufSize );
	if( gsBitstreamBufAddr[PA] == 0 ) 
	{
		DPRINTF( "[VDEC] bitstream_buf_addr[PA] malloc() failed \n");
		return -1;
	}
	DSTATUS( "[VDEC] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
	gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitStreamBufDev, gsBitstreamBufAddr[PA], gsBitstreamBufSize );
	if( gsBitstreamBufAddr[VA] == 0 ) 
	{
		DPRINTF( "[VDEC] bitstream_buf_addr[VA] malloc() failed \n");
		return -1;
	}
	memset( (void*)gsBitstreamBufAddr[VA], 0x00 , gsBitstreamBufSize);
	DSTATUS("[VDEC] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[VA], gsBitstreamBufSize );

	psVDecInit->m_BitstreamBufAddr[PA]	= gsBitstreamBufAddr[PA];
	psVDecInit->m_BitstreamBufAddr[VA]	= gsBitstreamBufAddr[VA];
	psVDecInit->m_iBitstreamBufSize 	= gsBitstreamBufSize;

	/* Set the maximum size of input bitstream. */
//	gsMaxBitstreamSize = MAX_BITSTREAM_SIZE;
//	gsMaxBitstreamSize = ALIGNED_BUFF(gsMaxBitstreamSize, (4 * 1024));
//	if (gsMaxBitstreamSize > gsBitstreamBufSize)
//	{
		gsMaxBitstreamSize = gsBitstreamBufSize;
//	}
#endif

	//------------------------------------------------------------
	//! [x] user data buffer for each VPU decoder
	//------------------------------------------------------------
	if(gsVpuDecInit.m_bEnableUserData)
	{
		gsUserdataBufSize = 50 * 1024;
		gsUserdataBufSize = ALIGNED_BUFF( gsUserdataBufSize, 4*1024 );
		gsUserdataBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsUserdataBufSize );
		if( gsUserdataBufAddr[PA] == 0 ) 
		{
			DPRINTF( "[CDK_CORE:Err%d] gsUserdataBufAddr physical alloc failed \n", -1 );
			return -1;
		}
		DSTATUS( "[CDK_CORE] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsUserdataBufAddr[PA], gsUserdataBufSize );
		gsUserdataBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsUserdataBufDev, gsUserdataBufAddr[PA], gsUserdataBufSize );
		if( gsUserdataBufAddr[VA] == 0 ) 
		{
			DPRINTF( "[CDK_CORE:Err%d] gsUserdataBufAddr virtual alloc failed \n", -1 );
			return -1;
		}
		memset( gsUserdataBufAddr[VA], 0 , gsUserdataBufSize);
		DSTATUS("[CDK_CORE] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsUserdataBufAddr[VA], gsUserdataBufSize );
	}
	
	//------------------------------------------------------------
	// [x] code buffer, work buffer and parameter buffer for VPU 
	//------------------------------------------------------------
	gsBitWorkBufSize = WORK_CODE_PARA_BUF_SIZE;
	gsBitWorkBufSize = ALIGNED_BUFF(gsBitWorkBufSize, 4*1024);
	gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitWorkBufSize );
	if( gsBitWorkBufAddr[PA] == 0 ) 
	{
		DPRINTF( "[VDEC] bit_work_buf_addr[PA] malloc() failed \n");
		return -1;
	}
	DSTATUS("[VDEC] bit_work_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
	gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitWorkBufDev, gsBitWorkBufAddr[PA], gsBitWorkBufSize );
	if( gsBitWorkBufAddr[VA] == 0 ) 
	{
		DPRINTF( "[VDEC] bit_work_buf_addr[VA] malloc() failed \n");
		return -1;
	}
	DSTATUS("[VDEC] bit_work_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitWorkBufAddr[VA], gsBitWorkBufSize );

	//------------------------------------------------------------
	// [x] PS(SPS/PPS) buffer for each VPU decoder
	//------------------------------------------------------------
	if( psVDecInit->m_iBitstreamFormat == STD_AVC )
	{
		gsSpsPpsSize = PS_SAVE_SIZE;
		gsSpsPpsSize = ALIGNED_BUFF( gsSpsPpsSize, 4*1024 );
		gsSpsPpsAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( gsSpsPpsSize );
		if( gsSpsPpsAddr == 0 ) 
		{
			DPRINTF( "[VDEC] sps_pps_buf_addr malloc() failed \n");
			return -1;
		}
		DSTATUS("[VDEC] sps_pps_buf_addr = 0x%x, 0x%x \n", (codec_addr_t)gsSpsPpsAddr, gsSpsPpsSize );

		psVDecInit->m_pSpsPpsSaveBuffer = (unsigned char*)gsSpsPpsAddr;
		psVDecInit->m_iSpsPpsSaveBufferSize = gsSpsPpsSize;
	}

	psVDecInit->m_BitWorkAddr[PA] = gsBitWorkBufAddr[PA];
	psVDecInit->m_BitWorkAddr[VA] = gsBitWorkBufAddr[VA];
	if( psVDecInit->m_bEnableVideoCache == 0 ){
		DSTATUS("[VDEC] Cache OFF\n");
	}
	else{
		DSTATUS("[VDEC] Cache ON\n");
	}

	psVDecInit->m_bCbCrInterleaveMode = psVDecInit->m_bCbCrInterleaveMode;
	if( psVDecInit->m_bCbCrInterleaveMode == 0 ){
		DSTATUS("[VDEC] CbCrInterleaveMode OFF\n");
	}
	else{
		DSTATUS("[VDEC] CbCrInterleaveMode ON\n");
	}

	if( psVDecInit->m_uiDecOptFlags&M4V_DEBLK_ENABLE ){
		DSTATUS( TC_YELLOW"[VDEC] MPEG-4 Deblocking ON\n"STYLE_RESET );
	}
	if( psVDecInit->m_uiDecOptFlags&M4V_GMC_FRAME_SKIP ){
		DSTATUS( TC_YELLOW"[VDEC] MPEG-4 GMC Frame Skip\n"STYLE_RESET );
	}

	return 0;
}

int
vpu_dec_seq_header( int iSize , dec_initial_info_t * psInitialInfo, int iIsThumbnail )
{
	int ret = 0;

	DSTATUS("vpu_dec_seq_header in :: size(%d), JpegOnly(%d), format(%d)", iSize, gsVpuDecUserInfo.m_bJpegOnly, gsVpuDecInit.m_iBitstreamFormat);
	ret = TCC_VPU_DEC( VPU_DEC_SEQ_HEADER, &gsVpuDecHandle, (void*)iSize, &gsVpuDecInitialInfo );
	if( ret != RETCODE_SUCCESS )
	{
	#ifndef HAVE_ANDROID_OS
		psInitialInfo->m_iReportErrorReason = gsVpuDecInitialInfo.m_iReportErrorReason;
	#endif
		DPRINTF( "[VDEC] VPU_DEC_SEQ_HEADER failed Error code is 0x%x \n", ret );
		return -ret;
	}
	
#ifndef HAVE_ANDROID_OS
	if(psInitialInfo != NULL)
		cdk_memcpy(psInitialInfo, &gsVpuDecInitialInfo, sizeof(dec_initial_info_t));
#endif
	print_dec_initial_info( &gsVpuDecInit, &gsVpuDecInitialInfo );

	//------------------------------------------------------------
	// [x] slice buffer for VPU
	//------------------------------------------------------------
	if( gsVpuDecInit.m_iBitstreamFormat == STD_AVC )
	{
		gsSliceBufSize = SLICE_SAVE_SIZE;
		gsSliceBufSize = ALIGNED_BUFF( gsSliceBufSize, 4*1024 );
		gsSliceBufAddr = (codec_addr_t)cdk_sys_malloc_physical_addr( gsSliceBufSize );
		if( gsSliceBufAddr == 0 ) 
		{
			DPRINTF( "[VDEC] slice_buf_addr malloc() failed \n");
			return CDK_ERR_MALLOC;
		}
		DSTATUS("[VDEC] slice_buf_addr = 0x%x, 0x%x \n", (codec_addr_t)gsSliceBufAddr, gsSliceBufSize );

		gsVpuDecBuffer.m_AvcSliceSaveBufferAddr  = gsSliceBufAddr;
		gsVpuDecBuffer.m_iAvcSliceSaveBufferSize = gsSliceBufSize;
	}		
	else
	{
		gsSliceBufSize = 0;
		gsSliceBufAddr = 0;
	}

	//------------------------------------------------------------
	// [x] frame buffer for each VPU decoder
	//------------------------------------------------------------
	gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount;
	DSTATUS( "[VDEC] FrameBufDelay %d\n", gsVpuDecInitialInfo.m_iFrameBufDelay );
	DSTATUS( "[VDEC] MinFrameBufferCount %d\n", gsVpuDecInitialInfo.m_iMinFrameBufferCount );
#ifdef HAVE_ANDROID_OS
#ifdef NEED_SPECIFIC_PROCESS_FOR_MJPEG
	if(gsVpuDecInit.m_iBitstreamFormat == STD_MJPG)
	{
		int max_count;

	#if defined(SUPPORT_MANAGE_MJPEG_BUFFER )
		if(!gsVpuDecUserInfo.m_bJpegOnly)
			gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount + gsAdditionalFrameCount;
		else{
			gsVpuDecInitialInfo.m_iMinFrameBufferCount = 1;
			gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount;// + 2;
		}
	#else
		gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount;// + 2;
	#endif

	#ifdef SUPPORT_MJPEG_SCALING
		if(gsVpuDecUserInfo.m_bJpegOnly)
			gsVpuDecBuffer.m_iMJPGScaleRatio = gsVpuDecUserInfo.jpg_ScaleRatio;
		else
			gsVpuDecBuffer.m_iMJPGScaleRatio = 0;

		DSTATUS("[VDEC] jpeg ratio = %d", gsVpuDecBuffer.m_iMJPGScaleRatio);

		if(gsVpuDecBuffer.m_iMJPGScaleRatio == 0)
			max_count = cdk_sys_remain_memory_size() / gsVpuDecInitialInfo.m_iMinFrameBufferSize;
		else
			max_count = cdk_sys_remain_memory_size() / gsVpuDecInitialInfo.m_iMjpg_MinFrameBufferSize[gsVpuDecBuffer.m_iMJPGScaleRatio];
	#else
		max_count = cdk_sys_remain_memory_size() / gsVpuDecInitialInfo.m_iMinFrameBufferSize;
	#endif
		if(gsVpuDecBuffer.m_iFrameBufferCount > max_count)
			gsVpuDecBuffer.m_iFrameBufferCount = max_count;
		
		if(gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
			gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;
		
		if(gsVpuDecBuffer.m_iFrameBufferCount < gsVpuDecInitialInfo.m_iMinFrameBufferCount)
		{
			LOGE( "[VDEC] Not enough memory for VPU frame buffer, Available[%d - %d], Min[%d- %d], Need[%d]", max_count, cdk_sys_remain_memory_size(),
				gsVpuDecInitialInfo.m_iMinFrameBufferCount, gsVpuDecInitialInfo.m_iMinFrameBufferSize, gsVpuDecBuffer.m_iFrameBufferCount);
			return CDK_ERR_MALLOC;
		}
	}
	else
#endif
	{
		int max_count;
		
		gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount + gsAdditionalFrameCount;
		max_count = cdk_sys_remain_memory_size() / gsVpuDecInitialInfo.m_iMinFrameBufferSize;

		if(gsVpuDecBuffer.m_iFrameBufferCount > max_count)
			gsVpuDecBuffer.m_iFrameBufferCount = max_count;
		
		if(gsVpuDecBuffer.m_iFrameBufferCount > MAX_FRAME_BUFFER_COUNT)
			gsVpuDecBuffer.m_iFrameBufferCount = MAX_FRAME_BUFFER_COUNT;

		if(iIsThumbnail)
		{
			if(gsVpuDecBuffer.m_iFrameBufferCount < (gsVpuDecInitialInfo.m_iMinFrameBufferCount))
			{
				LOGE( "[VDEC] Not enough memory for VPU frame buffer while extracting thumbnail, Available[%d], Min[%d], Need[%d]", max_count, gsVpuDecInitialInfo.m_iMinFrameBufferCount, gsVpuDecBuffer.m_iFrameBufferCount);
				return CDK_ERR_MALLOC;
			}
		}
		else
		{
			if(gsVpuDecBuffer.m_iFrameBufferCount < (gsVpuDecInitialInfo.m_iMinFrameBufferCount + gsAdditionalFrameCount))
			{
				LOGE( "[VDEC] Not enough memory for VPU frame buffer, Available[%d], Min[%d], Need[%d]", max_count, gsVpuDecInitialInfo.m_iMinFrameBufferCount, gsVpuDecInitialInfo.m_iMinFrameBufferCount + gsAdditionalFrameCount);
				return CDK_ERR_MALLOC;
			}
		}
	}
#else	
	gsVpuDecBuffer.m_iFrameBufferCount = gsVpuDecInitialInfo.m_iMinFrameBufferCount;
#endif
	DSTATUS( "[VDEC] FrameBufferCount %d [min %d], min_size = %d \n", gsVpuDecBuffer.m_iFrameBufferCount, gsVpuDecInitialInfo.m_iMinFrameBufferCount, gsVpuDecInitialInfo.m_iMinFrameBufferSize);

#ifdef SUPPORT_MJPEG_SCALING
	if(gsVpuDecBuffer.m_iMJPGScaleRatio != 0 && gsVpuDecInit.m_iBitstreamFormat == STD_MJPG){
		gsFrameBufSize = gsVpuDecBuffer.m_iFrameBufferCount * gsVpuDecInitialInfo.m_iMjpg_MinFrameBufferSize[gsVpuDecBuffer.m_iMJPGScaleRatio];
	}
	else
#endif
	{
		gsFrameBufSize = gsVpuDecBuffer.m_iFrameBufferCount * gsVpuDecInitialInfo.m_iMinFrameBufferSize;
	}

	gsFrameBufSize = ALIGNED_BUFF( gsFrameBufSize, 4*1024 );
	gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsFrameBufSize );
	if( gsFrameBufAddr[PA] == 0 ) 
	{
		DPRINTF( "[VDEC] frame_buf_addr[PA] malloc() failed \n");
		return CDK_ERR_MALLOC;
	}	

#ifdef SUPPORT_MJPEG_SCALING
	if(gsVpuDecBuffer.m_iMJPGScaleRatio != 0 && gsVpuDecInit.m_iBitstreamFormat == STD_MJPG){
		DSTATUS( "[VDEC] MinFrameBufferSize %d bytes \n", gsVpuDecInitialInfo.m_iMjpg_MinFrameBufferSize[gsVpuDecBuffer.m_iMJPGScaleRatio] );
	}
	else
#endif
	{
		DSTATUS( "[VDEC] MinFrameBufferSize %d bytes \n", gsVpuDecInitialInfo.m_iMinFrameBufferSize );
	}
	DSTATUS( "[VDEC] frame_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsFrameBufAddr[PA], gsFrameBufSize );
	gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[PA], gsFrameBufSize );
	if( gsFrameBufAddr[VA] == 0 ) 
	{
		DPRINTF( "[VDEC] frame_buf_addr[VA] malloc() failed \n");
		return CDK_ERR_MALLOC;
	}
	DSTATUS("[VDEC] frame_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsFrameBufAddr[VA], gsFrameBufSize );

	gsVpuDecBuffer.m_FrameBufferStartAddr[PA] = gsFrameBufAddr[PA];
	gsVpuDecBuffer.m_FrameBufferStartAddr[VA] = gsFrameBufAddr[VA];

#if 0//def VPU_PERFORMANCE_UP
	{
		 unsigned int regAddr = ((unsigned int)gsRegisterBase + 0x10000); //0xB0910000

		 //VCACHE_CTRL
		 *(volatile unsigned int *)(regAddr+0x00)	= (1<<0);			  //CACHEON

		 //VCACHE_REG
		 *(volatile unsigned int *)(regAddr+0x04)	= (3<<0);			//WR0|RD0
		 *(volatile unsigned int *)(regAddr+0x024)	= gsFrameBufAddr[PA];//VIDEO_PHY_ADDR;							//VCACHE_R0MIN
		 *(volatile unsigned int *)(regAddr+0x028)	= VIDEO_PHY_ADDR+ VIDEO_MEM_SIZE;	  //VCACHE_R0MAX
		 *(volatile unsigned int *)(regAddr+0x02C)	= 0; //VCACHE_R1MIN
		 *(volatile unsigned int *)(regAddr+0x030)	= 0; //VCACHE_R1MAX
		 *(volatile unsigned int *)(regAddr+0x034)	= 0; //VCACHE_R2MIN
		 *(volatile unsigned int *)(regAddr+0x038)	= 0; //VCACHE_R2MAX
		 *(volatile unsigned int *)(regAddr+0x03C)	= 0; //VCACHE_R3MIN
		 *(volatile unsigned int *)(regAddr+0x040)	= 0; //VCACHE_R3MAX
   }
#endif



	ret = TCC_VPU_DEC( VPU_DEC_REG_FRAME_BUFFER, &gsVpuDecHandle, &gsVpuDecBuffer, NULL );
	if( ret != RETCODE_SUCCESS )
	{
		DPRINTF( "[VDEC] DEC_REG_FRAME_BUFFER failed Error code is 0x%x \n", ret );
		return -ret;
	}
	DSTATUS("[VDEC] TCC_VPU_DEC VPU_DEC_REG_FRAME_BUFFER OK!\n");
	return ret;
}

int
vdec_vpu( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;

	if( iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !vdec_codec_opened)
		return -RETCODE_NOT_INITIALIZED;

	if( iOpCode == VDEC_INIT )
	{
		vdec_init_t* p_input_param = (vdec_init_t*)pParam1;
		
#ifdef HAVE_ANDROID_OS
		vdec_user_info_t* p_input_user_param = (vdec_user_info_t*)pParam2;

		gsVpuDecUserInfo.bitrate_mbps = p_input_user_param->bitrate_mbps;
		gsVpuDecUserInfo.frame_rate   = p_input_user_param->frame_rate;
		gsVpuDecUserInfo.m_bJpegOnly  = p_input_user_param->m_bJpegOnly;
		gsVpuDecUserInfo.jpg_ScaleRatio  = p_input_user_param->jpg_ScaleRatio;
		gsVpuDecUserInfo.jpg_length  = p_input_user_param->jpg_length;
		
		if(vpu_env_open(p_input_param->m_iBitstreamFormat, p_input_user_param->bitrate_mbps, p_input_user_param->frame_rate, p_input_param->m_iPicWidth, p_input_param->m_iPicHeight ) < 0)
			return -VPU_ENV_INIT_ERROR;
#endif

#if defined(VPU_CLK_CONTROL)
		vpu_clock_init();
#endif

#ifdef HAVE_ANDROID_OS
		gsVpuDecInit.m_RegBaseVirtualAddr	= gsRegisterBase;
		gsVpuDecInit.m_iBitstreamFormat		= p_input_param->m_iBitstreamFormat;
		gsVpuDecInit.m_iPicWidth			= p_input_param->m_iPicWidth;
		gsVpuDecInit.m_iPicHeight			= p_input_param->m_iPicHeight;
		gsVpuDecInit.m_bEnableUserData		= p_input_param->m_bEnableUserData;
		gsVpuDecInit.m_bEnableVideoCache	= p_input_param->m_bEnableVideoCache;
		gsVpuDecInit.m_bCbCrInterleaveMode  = p_input_param->m_bCbCrInterleaveMode;
		gsVpuDecInit.m_Memcpy				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsVpuDecInit.m_Memset				= (void  (*) ( void*, int, unsigned int ))memset;
		gsVpuDecInit.m_Interrupt			= (int  (*) ( void ))VpuInterrupt;	
#else
		gsVpuDecInit.m_RegBaseVirtualAddr	= p_input_param->m_RegBaseAddr;
		gsVpuDecInit.m_iBitstreamFormat		= p_input_param->m_iBitstreamFormat;
		gsVpuDecInit.m_BitstreamBufAddr[PA] = p_input_param->m_BitstreamBufAddr[PA];
		gsVpuDecInit.m_BitstreamBufAddr[VA] = p_input_param->m_BitstreamBufAddr[VA];
		gsVpuDecInit.m_iBitstreamBufSize	= p_input_param->m_iBitstreamBufSize;
		gsVpuDecInit.m_iPicWidth			= p_input_param->m_iPicWidth;
		gsVpuDecInit.m_iPicHeight			= p_input_param->m_iPicHeight;
		gsVpuDecInit.m_bEnableUserData		= p_input_param->m_bEnableUserData;
		gsVpuDecInit.m_bEnableVideoCache	= p_input_param->m_bEnableVideoCache;
		gsVpuDecInit.m_bCbCrInterleaveMode  = p_input_param->m_bCbCrInterleaveMode;

		gsVpuDecInit.m_uiDecOptFlags = 0;
		if( p_input_param->m_iBitstreamFormat == STD_MPEG4 )
		{
			if( p_input_param->m_bM4vDeblk )
				gsVpuDecInit.m_uiDecOptFlags |= M4V_DEBLK_ENABLE;
		}
		if( p_input_param->m_uiDecOptFlags )
		{
			if( p_input_param->m_uiDecOptFlags & 0x2 )
			{
				gsVpuDecInit.m_uiDecOptFlags |= M4V_GMC_FRAME_SKIP;
			}
			if( p_input_param->m_uiDecOptFlags >> 16 )
			{
				gsVpuDecInit.m_uiDecOptFlags |= ( p_input_param->m_uiDecOptFlags & 0xFFFF0000 );
			}
		}

		if( p_input_param->m_uiMaxResolution == 2 )
		{
			gsVpuDecInit.m_iMaxResolution = RESOLUTION_625_SD;
		}
		else if( p_input_param->m_uiMaxResolution == 1 )
		{
			gsVpuDecInit.m_iMaxResolution = RESOLUTION_720P_HD;
		}
		else
		{
			gsVpuDecInit.m_iMaxResolution = RESOLUTION_1080_HD;
		}

		gsVpuDecInit.m_Memcpy				= p_input_param->m_pfMemcpy;
		gsVpuDecInit.m_Memset				= p_input_param->m_pfMemset;
		gsVpuDecInit.m_Interrupt			= p_input_param->m_pfInterrupt;
#endif

#ifdef VPU_PERFORMANCE_UP
		//gsVpuDecInit.m_bEnableVideoCache	= 1;
	#if defined(_TCC9300_)
		gsVpuDecInit.m_uiDecOptFlags = SEC_AXI_BUS_ENABLE_TCC93XX;		// use secAXI SRAM0 128K
	#elif defined(_TCC8800_)
		if((gsVpuDecInit.m_iPicHeight * gsVpuDecInit.m_iPicWidth) > (1280*720))
			gsVpuDecInit.m_uiDecOptFlags = SEC_AXI_BUS_ENABLE_TCC88XX;	// use secAXI SRAM0 80K
		else
			gsVpuDecInit.m_uiDecOptFlags = SEC_AXI_BUS_DISABLE;

		if(gsVpuDecUserInfo.m_bJpegOnly)
			gsVpuDecInit.m_uiDecOptFlags = SEC_AXI_BUS_ENABLE_TCC88XX;	// use secAXI SRAM0 80K
	#endif
#endif

		gsVpuDecInit.m_iFilePlayEnable		= p_input_param->m_bFilePlayEnable;
		// 현재는 새로운 VPU Firmware가 완벽히 test되지 않음.
		gsbHasSeqHeader = 0;//p_input_param->m_bHasSeqHeader; 

		vpu_dec_ready( &gsVpuDecInit );

		DSTATUS("workbuff 0x%x/0x%x, Reg: 0x%x, format : %d, Stream(0x%x/0x%x, %d), Res: %d x %d", 
					gsVpuDecInit.m_BitWorkAddr[PA], gsVpuDecInit.m_BitWorkAddr[VA], gsVpuDecInit.m_RegBaseVirtualAddr,
					gsVpuDecInit.m_iBitstreamFormat, gsVpuDecInit.m_BitstreamBufAddr[PA], gsVpuDecInit.m_BitstreamBufAddr[VA], gsVpuDecInit.m_iBitstreamBufSize,
					gsVpuDecInit.m_iPicWidth, gsVpuDecInit.m_iPicHeight);
		DSTATUS("optFlag 0x%x, avcBuff: 0x%x- %d, Userdata(%d), VCache: %d, Inter: %d, PlayEn: %d, MaxRes: %d", 
					gsVpuDecInit.m_uiDecOptFlags, gsVpuDecInit.m_pSpsPpsSaveBuffer, gsVpuDecInit.m_iSpsPpsSaveBufferSize,
					gsVpuDecInit.m_bEnableUserData, gsVpuDecInit.m_bEnableVideoCache, gsVpuDecInit.m_bCbCrInterleaveMode,
					gsVpuDecInit.m_iFilePlayEnable, gsVpuDecInit.m_iMaxResolution);
	
		DSTATUS("Format : %d, Stream(0x%x, %d)", gsVpuDecInit.m_iBitstreamFormat, gsVpuDecInit.m_BitstreamBufAddr[PA], gsVpuDecInit.m_iBitstreamBufSize);
		ret = TCC_VPU_DEC( VPU_DEC_INIT, &gsVpuDecHandle, &gsVpuDecInit, NULL );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] VPU_DEC_INIT failed Error code is 0x%x \n", ret );
			return -ret;
		}
		DSTATUS( "[VDEC] VPU_DEC_INIT OK( has seq = %d), handle = 0x%x \n", gsbHasSeqHeader, gsVpuDecHandle );

		vdec_codec_opened = 1;

#ifdef VPU_REGISTER_DUMP
		bFirst_frame = 1;
		filewrite_memory("sdcard/after_init.bin", gsRegisterBase, 0x200);
#endif
	}
	else if( iOpCode == VDEC_DEC_SEQ_HEADER )
	{		
#ifdef HAVE_ANDROID_OS
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
		int seq_stream_size = (p_input_param->m_iInpLen > gsMaxBitstreamSize) ? gsMaxBitstreamSize : p_input_param->m_iInpLen;
		unsigned int iIsThumbnail = p_input_param->m_iIsThumbnail;

		if (    ((codec_addr_t)p_input_param->m_pInp[PA] == gsBitstreamBufAddr[PA])
		     && ((codec_addr_t)p_input_param->m_pInp[VA] == gsBitstreamBufAddr[VA]) )
		{
			gsVpuDecInit.m_BitstreamBufAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
			gsVpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
		}
		else
		{
			gsVpuDecInit.m_BitstreamBufAddr[PA] = gsBitstreamBufAddr[PA];
			gsVpuDecInit.m_BitstreamBufAddr[VA] = gsBitstreamBufAddr[VA];
			memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], seq_stream_size);
		}

		print_frame_data("sdcard/vpu_inSeq.bin", seq_stream_size);		
#else
		int seq_stream_size = (int)pParam1;
#endif

		DSTATUS( "[VDEC] VDEC_DEC_SEQ_HEADER start  :: len = %d / %d , handle = 0x%x \n", seq_stream_size, p_input_param->m_iInpLen, gsVpuDecHandle );
#ifdef HAVE_ANDROID_OS
		ret = vpu_dec_seq_header(seq_stream_size, &gsVpuDecInitialInfo, iIsThumbnail);
#else
		ret = vpu_dec_seq_header(seq_stream_size, pParam2);
#endif
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] VDEC_DEC_SEQ_HEADER failed Error code is 0x%x \n", ret );
			return ret;
		}
#ifdef HAVE_ANDROID_OS
		gsbHasSeqHeader = 1;
		p_output_param->m_pInitialInfo = &gsVpuDecInitialInfo;
#endif
		//check the maximum/minimum video resolution limitation
		if( gsVpuDecInit.m_iBitstreamFormat != STD_MJPG )
		{
#ifdef HAVE_ANDROID_OS
			vdec_info_t * pVdecInfo = (vdec_info_t *)&gsVpuDecInitialInfo;
#else		
			vdec_info_t * pVdecInfo = (vdec_info_t *)pParam2;
#endif
			int max_width, max_height;
			int min_width, min_height;
		 
			max_width  	= ((AVAILABLE_MAX_WIDTH+15)&0xFFF0);
			max_height 	= ((AVAILABLE_MAX_HEIGHT+15)&0xFFF0);
			min_width 	= AVAILABLE_MIN_WIDTH;
			min_height 	= AVAILABLE_MIN_HEIGHT;
			
			if(    (pVdecInfo->m_iPicWidth > max_width)
				|| ((pVdecInfo->m_iPicWidth * pVdecInfo->m_iPicHeight) > AVAILABLE_MAX_REGION)
				|| (pVdecInfo->m_iPicWidth < min_width)
				|| (pVdecInfo->m_iPicHeight < min_height) )
			{
				ret = 0 - RETCODE_INVALID_STRIDE;
				DPRINTF( "[VDEC] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d  \n", 
									pVdecInfo->m_iPicWidth, pVdecInfo->m_iPicHeight);
				return ret;
			}
		}
		else //MJPEG
		{
#ifdef HAVE_ANDROID_OS
			vdec_info_t * pVdecInfo = (vdec_info_t *)&gsVpuDecInitialInfo;
#else		
			vdec_info_t * pVdecInfo = (vdec_info_t *)pParam2;
#endif
			
			if(  (pVdecInfo->m_iPicWidth > 8192)		\
				|| (pVdecInfo->m_iPicHeight > 8192) )
			{
				ret = 0 - RETCODE_INVALID_STRIDE;
				DSTATUS( "[VDEC] VDEC_DEC_SEQ_HEADER - don't support the resolution %dx%d  \n", 
									pVdecInfo->m_iPicWidth, pVdecInfo->m_iPicHeight);
				return ret;
			}

			if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat != 0 )
			{
				LOGI("VPU OutFormat is YUV422SEQ0 ");
			}

	#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG	) && !defined(SUPPORT_MANAGE_MJPEG_BUFFER)
			if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && !gsVpuDecUserInfo.m_bJpegOnly)
			{
				unsigned int buf_idx;
				
				decoded_buf_curIdx = 0;
				decoded_buf_size = pVdecInfo->m_iPicWidth*pVdecInfo->m_iPicHeight*2; //for YUV422
				decoded_buf_size = ALIGNED_BUFF(decoded_buf_size, 4*1024);
				
				for(buf_idx =0; buf_idx < MAX_NUM_OF_VIDEO_ELEMENT; buf_idx++)
				{
					decoded_phyAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_physical_addr( decoded_buf_size );				
					if( decoded_phyAddr[buf_idx] == 0 ) 
					{
						DPRINTF( "[VDEC,Err:%d] vdec_vpu decoded_virtAddr[PA] alloc failed \n", ret );
						decoded_buf_maxcnt = buf_idx;
						break;
					}	
					decoded_virtAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsWMV78DecodedFrmaeBufferDev, decoded_phyAddr[buf_idx], decoded_buf_size ); 
					if( decoded_virtAddr[buf_idx] == 0 ) 
					{
						DPRINTF( "[VDEC,Err:%d] vdec_vpu decoded_virtAddr[VA] alloc failed \n", ret );
						decoded_buf_maxcnt = buf_idx;
						break;
					}
	
					decoded_buf_maxcnt = MAX_NUM_OF_VIDEO_ELEMENT;
					DSTATUS("OUT-Buffer %d ::	PA = 0x%x, VA = 0x%x, size = 0x%x!!\n", buf_idx, decoded_phyAddr[buf_idx], decoded_virtAddr[buf_idx],	decoded_buf_size);
				}			
			}
	#endif

		}
		
		DSTATUS( "[VDEC] VDEC_DEC_SEQ_HEADER - Success \n" );
		DSTATUS( "=======================================================\n\n" );		
	}
	else if( iOpCode == VDEC_DECODE )
	{
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
#ifdef DEBUG_TIME_LOG
		clock_t start, end;
#endif

		#ifdef PRINT_VPU_INPUT_STREAM
		{
			int kkk;
			unsigned char* p_input = p_input_param->m_pInp[VA];
			int input_size = p_input_param->m_iInpLen;
			printf("FS = %7d :", input_size);
			for( kkk = 0; kkk < PRINT_BYTES; kkk++ )
				printf("%02X ", p_input[kkk] );
			printf("\n");
		}
		#endif

#ifdef HAVE_ANDROID_OS
		gsVpuDecInput.m_iBitstreamDataSize = (p_input_param->m_iInpLen > gsMaxBitstreamSize) ? gsMaxBitstreamSize : p_input_param->m_iInpLen;
		if (    ((codec_addr_t)p_input_param->m_pInp[PA] == gsBitstreamBufAddr[PA])
		     && ((codec_addr_t)p_input_param->m_pInp[VA] == gsBitstreamBufAddr[VA]) )
		{
			gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
			gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
		}
		else
		{
			gsVpuDecInput.m_BitstreamDataAddr[PA] = gsBitstreamBufAddr[PA];
			gsVpuDecInput.m_BitstreamDataAddr[VA] = gsBitstreamBufAddr[VA];
			memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], gsVpuDecInput.m_iBitstreamDataSize);
		}

		print_frame_data("sdcard/vpu_inDec.bin", gsVpuDecInput.m_iBitstreamDataSize); 	
#else
		gsVpuDecInput.m_iBitstreamDataSize = p_input_param->m_iInpLen;
		gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
		gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
#endif

		if(gsVpuDecInit.m_bEnableUserData)
		{
			gsVpuDecInput.m_UserDataAddr[PA] = gsUserdataBufAddr[PA];
			gsVpuDecInput.m_UserDataAddr[VA] = gsUserdataBufAddr[VA];
			gsVpuDecInput.m_iUserDataBufferSize = gsUserdataBufSize;
		}
		
		gsVpuDecInput.m_UserDataAddr[PA] = (codec_addr_t)p_input_param->m_UserDataAddr[PA];
		gsVpuDecInput.m_UserDataAddr[VA] = (codec_addr_t)p_input_param->m_UserDataAddr[VA];
		gsVpuDecInput.m_iUserDataBufferSize = p_input_param->m_iUserDataBufferSize;
		
		gsVpuDecInput.m_iSkipFrameMode = p_input_param->m_iSkipFrameMode;
		gsVpuDecInput.m_iFrameSearchEnable = p_input_param->m_iFrameSearchEnable;
		gsVpuDecInput.m_iSkipFrameNum = 0;
		if( gsVpuDecInput.m_iSkipFrameMode > 0 || gsVpuDecInput.m_iFrameSearchEnable > 0 )
		{
			gsVpuDecInput.m_iSkipFrameNum = p_input_param->m_iSkipFrameNum;
		}

#ifdef  VIDEO_DEC_PROFILE
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) |= (unsigned int)(GPIO_NUM);
#ifdef DEBUG_TIME_LOG		
		start = clock();
#endif
#endif
	#ifdef DEBUG_VPU_DEC_INPUT
		PrintVPUHexData((unsigned char*)gsBitstreamBufAddr[VA], gsVpuDecInput.m_iBitstreamDataSize, "VPU");
	#endif

		// Start decoding a frame.
		ret = TCC_VPU_DEC( VPU_DEC_DECODE, &gsVpuDecHandle, &gsVpuDecInput, &gsVpuDecOutput );

#if 0//def VPU_PERFORMANCE_UP
		{
	         unsigned int regAddr = ((unsigned int)gsRegisterBase + 0x10000); //0xB0910000

	         //VCACHE_CTRL
	         *(volatile unsigned int *)(regAddr+0x08) =      (1<<0);           //DRAIN
	   }
#endif
	
//		if( ret == VPU_DEC_FINISH )
//			return ERR_END_OF_FILE;
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] VPU_DEC_DECODE failed Error code is 0x%x \n", ret );

			if(ret == RETCODE_CODEC_EXIT)
			{
				//vpu h/w reset in case of lockup!!
				vpu_hw_reset();
			}
			return -ret;
		}

		total_frm++;

#ifdef NEED_SPECIFIC_PROCESS_FOR_MJPEG
		if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && 
	#if defined(SUPPORT_MANAGE_MJPEG_BUFFER	)
			gsVpuDecUserInfo.m_bJpegOnly && 
	#endif		
			gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus == VPU_DEC_BUF_FULL &&
			gsVpuDecOutput.m_DecOutInfo.m_iOutputStatus == VPU_DEC_OUTPUT_SUCCESS)
		{
			gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus = VPU_DEC_SUCCESS;
		}
#endif

#ifdef  VIDEO_DEC_PROFILE
#ifdef DEBUG_TIME_LOG		
		end = clock();
		dec_time[time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
		total_dec_time += dec_time[time_cnt];
		total_frame_cnt++;
		if(time_cnt != 0 && time_cnt % 29 == 0)
		{
			LOGD("Vdec %d ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
				total_dec_time/total_frame_cnt, dec_time[0], dec_time[1], dec_time[2], dec_time[3], dec_time[4], dec_time[5], dec_time[6], dec_time[7], dec_time[8], dec_time[9], 
				dec_time[10], dec_time[11], dec_time[12], dec_time[13], dec_time[14], dec_time[15], dec_time[16], dec_time[17], dec_time[18], dec_time[19], 
				dec_time[20], dec_time[21], dec_time[22], dec_time[23], dec_time[24], dec_time[25], dec_time[26], dec_time[27], dec_time[28], dec_time[29]);
			time_cnt = 0;
		}
		else{
			time_cnt++;
		}
#endif
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) &= (unsigned int)(~GPIO_NUM);
#endif

#ifdef HAVE_ANDROID_OS
		memcpy((void*)p_output_param, (void*)&gsVpuDecOutput, sizeof(dec_output_t ) );

	#ifdef NEED_SPECIFIC_PROCESS_FOR_MJPEG		
		if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && gsVpuDecUserInfo.m_bJpegOnly && gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 4)
		{
			unsigned int frame_size = gsVpuDecInitialInfo.m_iPicWidth*gsVpuDecInitialInfo.m_iPicHeight;
			
			if(gsVpuDecInit.m_bCbCrInterleaveMode)
			{
				memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/2);
			}
			else
			{
				memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/4);
				memset( (unsigned char *)p_output_param->m_pDispOut[VA][2], 0x80, frame_size/4);
			}
		}
		
		if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && !gsVpuDecUserInfo.m_bJpegOnly)
		{
			unsigned int frame_size = gsVpuDecInitialInfo.m_iPicWidth*gsVpuDecInitialInfo.m_iPicHeight;
			int divide_val = 4; 					
			if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 1 ) //YUV422
				divide_val = 2;
			
		#if !defined(SUPPORT_MANAGE_MJPEG_BUFFER)
			p_output_param->m_pDispOut[PA][0] = (unsigned char*)decoded_phyAddr[decoded_buf_curIdx];
			p_output_param->m_pDispOut[PA][1] = (unsigned char*)p_output_param->m_pDispOut[PA][0] + frame_size;
			p_output_param->m_pDispOut[PA][2] = (unsigned char*)p_output_param->m_pDispOut[PA][1] + frame_size/divide_val;	
			p_output_param->m_pDispOut[VA][0] = (unsigned char*)decoded_virtAddr[decoded_buf_curIdx];
			p_output_param->m_pDispOut[VA][1] = (unsigned char*)p_output_param->m_pDispOut[VA][0] + frame_size;
			p_output_param->m_pDispOut[VA][2] = (unsigned char*)p_output_param->m_pDispOut[VA][1] + frame_size/divide_val;	

			if(gsVpuDecInit.m_bCbCrInterleaveMode)
			{
				memcpy( (unsigned char *)p_output_param->m_pDispOut[VA][0], gsVpuDecOutput.m_pDispOut[VA][0], frame_size);

				if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 4) //YUV400
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/divide_val);
				else
					memcpy( (unsigned char *)p_output_param->m_pDispOut[VA][1], gsVpuDecOutput.m_pDispOut[VA][1], frame_size/divide_val);
			}
			else
			{
				memcpy( (unsigned char *)p_output_param->m_pDispOut[VA][0], gsVpuDecOutput.m_pDispOut[VA][0], frame_size);
				if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 4) //YUV400
				{
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/divide_val);
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][2], 0x80, frame_size/divide_val);
				}
				else
				{
					memcpy( (unsigned char *)p_output_param->m_pDispOut[VA][1], gsVpuDecOutput.m_pDispOut[VA][1], frame_size/divide_val);
					memcpy( (unsigned char *)p_output_param->m_pDispOut[VA][2], gsVpuDecOutput.m_pDispOut[VA][2], frame_size/divide_val);
				}
			}

			decoded_buf_curIdx++;
			if(decoded_buf_curIdx >= decoded_buf_maxcnt)
				decoded_buf_curIdx = 0;
		#else
			if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 4)
			{
				if(gsVpuDecInit.m_bCbCrInterleaveMode)
				{
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/2);
				}
				else
				{
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][1], 0x80, frame_size/divide_val);
					memset( (unsigned char *)p_output_param->m_pDispOut[VA][2], 0x80, frame_size/divide_val);
				}
			}
		#endif
		}
	#endif
#else
		cdk_memcpy( p_output_param, &gsVpuDecOutput, sizeof(dec_output_t ) );
#endif
		p_output_param->m_pInitialInfo = &gsVpuDecInitialInfo;

		print_frame_data("sdcard/vpu_outDec.bin", p_input_param->m_iInpLen); 	
#ifdef VPU_REGISTER_DUMP
		filewrite_memory("sdcard/after_decode.bin", gsRegisterBase, 0x200);
		bFirst_frame = 0;
#endif
	}
	else if( iOpCode == VDEC_GET_RING_BUFFER_STATUS )
	{
		dec_ring_buffer_status_out_t* p_out_param = (dec_ring_buffer_status_out_t*)pParam2;

		ret = TCC_VPU_DEC( GET_RING_BUFFER_STATUS, &gsVpuDecHandle, NULL, p_out_param ); // get the available space in the ring buffer
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] GET_RING_BUFFER_STATUS failed Error code is 0x%x \n", ret );
			return -ret;
		}
	}
	else if( iOpCode == VDEC_FILL_RING_BUFFER )
	{
		dec_ring_buffer_setting_in_t* p_set_param = (dec_ring_buffer_setting_in_t*)pParam1;

		ret = TCC_VPU_DEC( FILL_RING_BUFFER_AUTO, &gsVpuDecHandle, &gsVpuDecInit, p_set_param ); // fille the Ring Buffer 
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] FILL_RING_BUFFER_AUTO failed Error code is 0x%x \n", ret );
			return -ret;
		}

	}
	else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
	{
		int idx_display = *(int*)pParam1;
		ret = TCC_VPU_DEC( VPU_DEC_BUF_FLAG_CLEAR, &gsVpuDecHandle, &idx_display, 0 );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] VPU_DEC_BUF_FLAG_CLEAR failed Error code is 0x%x \n", ret );
			return -ret;
		}
	}
	else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
	{
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		//vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
		gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
		gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
		gsVpuDecInput.m_iBitstreamDataSize = 0;
		gsVpuDecInput.m_iSkipFrameMode = VDEC_SKIP_FRAME_DISABLE;
		gsVpuDecInput.m_iFrameSearchEnable = 0;
		gsVpuDecInput.m_iSkipFrameNum = 0;

		ret = TCC_VPU_DEC( VPU_DEC_FLUSH_OUTPUT, &gsVpuDecHandle, &gsVpuDecInput, &gsVpuDecOutput );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] VPU_DEC_BUF_FLAG_CLEAR failed Error code is 0x%x \n", ret );
			return -ret;
		}
	}
	else if( iOpCode == VDEC_CLOSE )
	{
		if(!vdec_codec_opened && !vdec_env_opened)
			return -RETCODE_NOT_INITIALIZED;

		if(vdec_codec_opened)
		{
			ret = TCC_VPU_DEC( VPU_DEC_CLOSE, &gsVpuDecHandle, NULL, &gsVpuDecOutput );
			if( ret != RETCODE_SUCCESS )
			{
				DPRINTF( "[VDEC] VPU_DEC_CLOSE failed Error code is 0x%x \n", ret );
				ret = -ret;
			}
			
			vdec_codec_opened = 0;
		}

		if(!vdec_env_opened)
			return -RETCODE_NOT_INITIALIZED;
		
		if( gsBitWorkBufAddr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
			gsBitWorkBufAddr[PA]= NULL;
		}
		if( gsBitWorkBufAddr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsBitWorkBufDev, gsBitWorkBufAddr[VA], gsBitWorkBufSize );
			gsBitWorkBufAddr[VA] = NULL;
		}
		gsBitWorkBufSize = 0;
		gsBitWorkBufDev = 0;
		
		if( gsSliceBufAddr )
		{
			cdk_sys_free_physical_addr( (void*)gsSliceBufAddr, gsSliceBufSize );
			gsSliceBufAddr = NULL;
			gsSliceBufSize = 0;
		}
		if( gsSpsPpsAddr )
		{
			cdk_sys_free_physical_addr( (void*)gsSpsPpsAddr, gsSpsPpsSize );
			gsSpsPpsAddr = NULL;
			gsSpsPpsSize = 0;
		}
		if( gsFrameBufAddr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)gsFrameBufAddr[PA], gsFrameBufSize );
			gsFrameBufAddr[PA]= NULL;
		}
		if( gsFrameBufAddr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[VA], gsFrameBufSize );
			gsFrameBufAddr[VA] = NULL;
		}

#if defined(NEED_SPECIFIC_PROCESS_FOR_MJPEG	) && !defined(SUPPORT_MANAGE_MJPEG_BUFFER)
		if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && !gsVpuDecUserInfo.m_bJpegOnly)
		{
			int buf_idx = 0;
			
			for(buf_idx =0; buf_idx < decoded_buf_maxcnt; buf_idx++)
			{
				cdk_sys_free_physical_addr( (void*)decoded_phyAddr[buf_idx], decoded_buf_size );
				decoded_phyAddr[buf_idx] = 0;
				cdk_sys_free_virtual_addr( NULL, decoded_virtAddr[buf_idx], decoded_buf_size );
				decoded_virtAddr[buf_idx] = 0;
			}			
		}
#endif

#if defined(VPU_CLK_CONTROL)
		vpu_clock_deinit();
#endif
#ifdef HAVE_ANDROID_OS
		vpu_env_close();
#endif
	}
	else
	{
		DPRINTF( "Invalid Operation!!\n" );
		return -ret;
	}

	return ret;
}

#ifdef INCLUDE_WMV78_DEC
int
WMV78_dec_seq_header()
{
	int ret = 0;

	memset( &gsWMV78DecInitialInfo, 0, sizeof(gsWMV78DecInitialInfo) );

	gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropBottom = 0;
	gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropLeft = 0;
	gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropRight = 0;
	gsWMV78DecInitialInfo.m_iAvcPicCrop.m_iCropTop = 0;
	gsWMV78DecInitialInfo.m_iMinFrameBufferCount = 1;
	gsWMV78DecInitialInfo.m_iPicWidth = gsWMV78DecInit.m_iWidth;
	gsWMV78DecInitialInfo.m_iPicHeight = gsWMV78DecInit.m_iHeight;
	gsWMV78DecInitialInfo.m_iAspectRateInfo = 0;
	gsWMV78DecInitialInfo.m_iInterlace = 0;
	print_dec_initial_info( &gsVpuDecInit, &gsWMV78DecInitialInfo );

	return ret;
}

int
vdec_WMV78( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;

	if( iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !vdec_codec_opened)
		return -RETCODE_NOT_INITIALIZED;

	if( iOpCode == VDEC_INIT )
	{
		unsigned int ChromaSize;
		vdec_init_t* p_input_param = (vdec_init_t*)pParam1;
		vdec_callback_func_t* pf_callback = (vdec_callback_func_t*)pParam2;

#ifdef HAVE_ANDROID_OS
		if(vpu_env_open(p_input_param->m_iBitstreamFormat, 0, 0, p_input_param->m_iPicWidth, p_input_param->m_iPicHeight ) < 0) // to operate Max-clock for s/w codec!!
			return -1;
#endif

		gsWMV78FrameSize = ( (p_input_param->m_iPicWidth+15)&0xfffffff0 ) * ( (p_input_param->m_iPicHeight+15)&0xfffffff0 );
		ChromaSize = gsWMV78FrameSize>>2;
		gsWMV78NCFrameSize = gsWMV78FrameSize*1.5;
		gsWMV78NCFrameSize = ALIGNED_BUFF( gsWMV78NCFrameSize, 4*1024 );

#ifdef HAVE_ANDROID_OS
		gsWMV78CurYFrameAddress = (unsigned char*)TCC_malloc(gsWMV78NCFrameSize);
		gsWMV78CurUFrameAddress  = (unsigned int)((unsigned char*)gsWMV78CurYFrameAddress + gsWMV78FrameSize);
		gsWMV78CurVFrameAddress  = (unsigned int)((unsigned char*)gsWMV78CurUFrameAddress + ChromaSize);

		gsWMV78Ref0YFrameAddress = (unsigned char*)TCC_malloc(gsWMV78NCFrameSize);
		gsWMV78Ref0UFrameAddress = (unsigned int)((unsigned char*)gsWMV78Ref0YFrameAddress + gsWMV78FrameSize);
		gsWMV78Ref0VFrameAddress = (unsigned int)((unsigned char*)gsWMV78Ref0UFrameAddress + ChromaSize);
#else
		gsWMV78DecodedFrameAddress_NC[PA]  = (codec_addr_t)cdk_sys_malloc_physical_addr( gsWMV78NCFrameSize );
		gsWMV78DecodedFrameAddress_NC[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsWMV78DecodedFrmaeBufferDev, gsWMV78DecodedFrameAddress_NC[PA], gsWMV78NCFrameSize );
		if( gsWMV78DecodedFrameAddress_NC[VA] == 0 ) 
		{
			DPRINTF( "[VDEC_WMV78:Err%d] gsWMV78DecodedFrameAddress_NC virtual alloc failed \n", -1 );
			return -1;
		}

		gsWMV78CurYFrameAddress  = (unsigned int)cdk_malloc(sizeof(unsigned char)*gsWMV78FrameSize*1.5);
		gsWMV78CurUFrameAddress  = (unsigned int)((unsigned char*)gsWMV78CurYFrameAddress + gsWMV78FrameSize);
		gsWMV78CurVFrameAddress  = (unsigned int)((unsigned char*)gsWMV78CurUFrameAddress + ChromaSize);
		gsWMV78Ref0YFrameAddress = (unsigned int)cdk_malloc(sizeof(unsigned char)*gsWMV78FrameSize*1.5);
		gsWMV78Ref0UFrameAddress = (unsigned int)((unsigned char*)gsWMV78Ref0YFrameAddress + gsWMV78FrameSize);
		gsWMV78Ref0VFrameAddress = (unsigned int)((unsigned char*)gsWMV78Ref0UFrameAddress + ChromaSize);
#endif
		gsWMV78DecInit.m_pExtraData			= p_input_param->m_pExtraData;
		gsWMV78DecInit.m_iExtraDataLen		= p_input_param->m_iExtraDataLen;
		gsWMV78DecInit.m_iWidth				= (p_input_param->m_iPicWidth+15)&0xfffffff0;
		gsWMV78DecInit.m_iHeight			= p_input_param->m_iPicHeight;
		gsWMV78DecInit.m_iFourCC			= p_input_param->m_iFourCC;
		gsWMV78DecInit.m_pHeapAddress		= (unsigned char*)TCC_malloc(sizeof(unsigned char)*200*1024);
		gsWMV78DecInit.m_pHuff_tbl_address	= WMV78_Huff_table;

		gsWMV78DecInit.m_pCurFrameAddress  = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));
		gsWMV78DecInit.m_pRef0FrameAddress = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));
		gsWMV78DecInit.m_pCurFrameAddress->m_pucYPlane		= (unsigned char*)gsWMV78CurYFrameAddress;
		gsWMV78DecInit.m_pCurFrameAddress->m_pucUPlane		= (unsigned char*)gsWMV78CurUFrameAddress;
		gsWMV78DecInit.m_pCurFrameAddress->m_pucVPlane		= (unsigned char*)gsWMV78CurVFrameAddress;
		gsWMV78DecInit.m_pRef0FrameAddress->m_pucYPlane		= (unsigned char*)gsWMV78Ref0YFrameAddress;
		gsWMV78DecInit.m_pRef0FrameAddress->m_pucUPlane		= (unsigned char*)gsWMV78Ref0UFrameAddress;
		gsWMV78DecInit.m_pRef0FrameAddress->m_pucVPlane		= (unsigned char*)gsWMV78Ref0VFrameAddress;
		gsWMV78DecOutput.m_pDecodedData = (tYUV420Frame_WMV*)TCC_malloc(sizeof(tYUV420Frame_WMV));

#ifdef HAVE_ANDROID_OS
		gsWMV78DecInit.m_sCallbackFunc.m_pMalloc			= (void  (*) ( unsigned int ))TCC_malloc;
		gsWMV78DecInit.m_sCallbackFunc.m_pFree				= (void  (*) ( void* ))TCC_free;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemcpy			= (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemset			= (void  (*) ( void*, int, unsigned int ))memset;
		gsWMV78DecInit.m_sCallbackFunc.m_pRealloc			= (void  (*) ( void*, unsigned int ))TCC_realloc;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemmove			= (void  (*) ( void*, const void*, unsigned int ))memmove;

		{
			unsigned int buf_idx;
			
			decoded_buf_curIdx = 0;
			decoded_buf_size = gsWMV78FrameSize* 1.5;
			decoded_buf_size = ALIGNED_BUFF(decoded_buf_size, 4*1024);
			
			for(buf_idx =0; buf_idx < MAX_NUM_OF_VIDEO_ELEMENT; buf_idx++)
			{
				decoded_phyAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_physical_addr( decoded_buf_size );				
				if( decoded_phyAddr[buf_idx] == 0 ) 
				{
					DPRINTF( "[VDEC,Err:%d] vdec_vpu decoded_virtAddr[PA] alloc failed \n", ret );
					decoded_buf_maxcnt = buf_idx;
					break;
				}	
				decoded_virtAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsWMV78DecodedFrmaeBufferDev, decoded_phyAddr[buf_idx], decoded_buf_size );	
				if( decoded_virtAddr[buf_idx] == 0 ) 
				{
					DPRINTF( "[VDEC,Err:%d] vdec_vpu decoded_virtAddr[VA] alloc failed \n", ret );
					decoded_buf_maxcnt = buf_idx;
					break;
				}

				decoded_buf_maxcnt = MAX_NUM_OF_VIDEO_ELEMENT;
				DSTATUS("OUT-Buffer %d ::	PA = 0x%x, VA = 0x%x, size = 0x%x!!\n", buf_idx, decoded_phyAddr[buf_idx], decoded_virtAddr[buf_idx],	decoded_buf_size);
			}			
		}		
#else
		gsWMV78DecInit.m_sCallbackFunc.m_pMalloc			= pf_callback->m_pfMalloc;
		gsWMV78DecInit.m_sCallbackFunc.m_pFree				= pf_callback->m_pfFree;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemcpy			= pf_callback->m_pfMemcpy;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemset			= pf_callback->m_pfMemset;
		gsWMV78DecInit.m_sCallbackFunc.m_pRealloc			= pf_callback->m_pfRealloc;
		gsWMV78DecInit.m_sCallbackFunc.m_pMemmove			= pf_callback->m_pfMemmove;
#endif

		gsVpuDecInit.m_iBitstreamFormat	= p_input_param->m_iBitstreamFormat;

		gsFirstFrame = 1;

		DSTATUS( "[VDEC_WMV78] WMV78_DEC_INIT Enter \n" );
		ret = TCC_WMV78_DEC( VDEC_INIT, &gsWMV78DecHandle, &gsWMV78DecInit, NULL );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC_WMV78] WMV78_DEC_INIT failed Error code is 0x%x \n", ret );
			return -ret;
		}
		gsIsINITdone = 1;
		vdec_codec_opened = 1;
		DSTATUS( "[VDEC_WMV78] WMV78_DEC_INIT OK \n" );
	}
	else if( iOpCode == VDEC_DEC_SEQ_HEADER )
	{
#ifdef HAVE_ANDROID_OS
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

		if( gsFirstFrame )
		{
			DSTATUS( "[VDEC_WMV78] VDEC_DEC_SEQ_HEADER start \n" );
			p_output_param->m_pInitialInfo = &gsWMV78DecInitialInfo;
			ret = WMV78_dec_seq_header();
			if( ret != RETCODE_SUCCESS )
			{
				DPRINTF( "[VDEC_WMV78] vpu_dec_seq_header failed Error code is 0x%x \n", ret );
				return ret;
			}
			DSTATUS( "[VDEC_WMV78] VDEC_DEC_SEQ_HEADER - Success \n" );
			gsFirstFrame = 0;
		}
#endif
		return RETCODE_SUCCESS;
	}
	else if( iOpCode == VDEC_DECODE )
	{
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
#ifdef DEBUG_TIME_LOG		
		clock_t start, end;
#endif

		#ifdef PRINT_VPU_INPUT_STREAM
		{
			int kkk;
			unsigned char* p_input = p_input_param->m_pInp[VA];
			int input_size = p_input_param->m_iInpLen;
			printf("FS = %7d :", input_size);
			for( kkk = 0; kkk < PRINT_BYTES; kkk++ )
				printf("%02X ", p_input[kkk] );
			printf("\n");
		}
		#endif

		if( gsFirstFrame )
		{
			DSTATUS( "[VDEC_WMV78] VDEC_DEC_SEQ_HEADER start \n" );
			p_output_param->m_pInitialInfo = &gsWMV78DecInitialInfo;
			ret = WMV78_dec_seq_header();
			if( ret != RETCODE_SUCCESS )
			{
				DPRINTF( "[VDEC_WMV78] vpu_dec_seq_header failed Error code is 0x%x \n", ret );
				return ret;
			}
			DSTATUS( "[VDEC_WMV78] VDEC_DEC_SEQ_HEADER - Success \n" );
			gsFirstFrame = 0;
		}

		gsWMV78DecInput.m_pPacketBuff = (codec_addr_t)p_input_param->m_pInp[VA];
		gsWMV78DecInput.m_iPacketBuffSize = p_input_param->m_iInpLen;

#ifdef HAVE_ANDROID_OS
	#if 0 // temporarily no-use!!
		gsWMV78DecInit.m_pCurFrameAddress->m_pucYPlane		= (unsigned char*)decoded_virtAddr[decoded_buf_curIdx];
		gsWMV78DecInit.m_pCurFrameAddress->m_pucUPlane		= (unsigned char*)gsWMV78DecInit.m_pCurFrameAddress->m_pucYPlane + gsWMV78FrameSize;
		gsWMV78DecInit.m_pCurFrameAddress->m_pucVPlane		= (unsigned char*)gsWMV78DecInit.m_pCurFrameAddress->m_pucUPlane + gsWMV78FrameSize/4;		
	#endif
#endif

#ifdef  VIDEO_DEC_PROFILE
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) |= (unsigned int)(GPIO_NUM);
#ifdef DEBUG_TIME_LOG				
		start = clock();
#endif
#endif

		// Start decoding a frame.
		ret = TCC_WMV78_DEC( VDEC_DECODE, &gsWMV78DecHandle, &gsWMV78DecInput, &gsWMV78DecOutput );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC_WMV78] VDEC_DECODE failed Error code is 0x%x \n", ret );
			return ret;
		}
#ifdef  VIDEO_DEC_PROFILE
#ifdef DEBUG_TIME_LOG				
		end = clock();
		dec_time[time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
		total_dec_time += dec_time[time_cnt];
		total_frame_cnt++;
		if(time_cnt != 0 && time_cnt % 29 == 0)
		{
			LOGD("Vdec   ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
				dec_time[0], dec_time[1], dec_time[2], dec_time[3], dec_time[4], dec_time[5], dec_time[6], dec_time[7], dec_time[8], dec_time[9], 
				dec_time[10], dec_time[11], dec_time[12], dec_time[13], dec_time[14], dec_time[15], dec_time[16], dec_time[17], dec_time[18], dec_time[19], 
				dec_time[20], dec_time[21], dec_time[22], dec_time[23], dec_time[24], dec_time[25], dec_time[26], dec_time[27], dec_time[28], dec_time[29]);
			time_cnt = 0;
		}
		else{
			time_cnt++;
		}
#endif
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) &= (unsigned int)(~GPIO_NUM);
#endif

#ifdef HAVE_ANDROID_OS
	#if 1 // temporarily use!!
		memcpy( (unsigned char *)decoded_virtAddr[decoded_buf_curIdx], gsWMV78DecOutput.m_pDecodedData->m_pucYPlane, gsWMV78FrameSize*1.5*sizeof(unsigned char) );
	#endif
		p_output_param->m_pCurrOut[0][0] = p_output_param->m_pDispOut[0][0] = (unsigned char*)decoded_phyAddr[decoded_buf_curIdx];
		p_output_param->m_pCurrOut[0][1] = p_output_param->m_pDispOut[0][1] = (unsigned char*)p_output_param->m_pDispOut[0][0] + gsWMV78FrameSize;
		p_output_param->m_pCurrOut[0][2] = p_output_param->m_pDispOut[0][2] = (unsigned char*)p_output_param->m_pDispOut[0][1] + gsWMV78FrameSize/4;	
		p_output_param->m_pCurrOut[1][0] = p_output_param->m_pDispOut[1][0] = (unsigned char*)decoded_virtAddr[decoded_buf_curIdx];
		p_output_param->m_pCurrOut[1][1] = p_output_param->m_pDispOut[1][1] = (unsigned char*)p_output_param->m_pDispOut[1][0] + gsWMV78FrameSize;
		p_output_param->m_pCurrOut[1][2] = p_output_param->m_pDispOut[1][2] = (unsigned char*)p_output_param->m_pDispOut[1][1] + gsWMV78FrameSize/4;	

		decoded_buf_curIdx++;
		if(decoded_buf_curIdx >= decoded_buf_maxcnt)
			decoded_buf_curIdx = 0;
#else
		cdk_memcpy( (unsigned char *)gsWMV78DecodedFrameAddress_NC[VA], gsWMV78DecOutput.m_pDecodedData->m_pucYPlane, gsWMV78FrameSize*1.5*sizeof(unsigned char) );

		p_output_param->m_pCurrOut[0][0] = p_output_param->m_pDispOut[0][0] = (unsigned char *)gsWMV78DecodedFrameAddress_NC[PA];
		p_output_param->m_pCurrOut[0][1] = p_output_param->m_pDispOut[0][1] = (unsigned char *)(gsWMV78DecodedFrameAddress_NC[PA]+gsWMV78FrameSize);
		p_output_param->m_pCurrOut[0][2] = p_output_param->m_pDispOut[0][2] = (unsigned char *)(gsWMV78DecodedFrameAddress_NC[PA]+gsWMV78FrameSize+gsWMV78FrameSize/4);
		p_output_param->m_pCurrOut[1][0] = p_output_param->m_pDispOut[1][0] = (unsigned char *)gsWMV78DecodedFrameAddress_NC[VA];
		p_output_param->m_pCurrOut[1][1] = p_output_param->m_pDispOut[1][1] = (unsigned char *)(gsWMV78DecodedFrameAddress_NC[VA]+gsWMV78FrameSize);
		p_output_param->m_pCurrOut[1][2] = p_output_param->m_pDispOut[1][2] = (unsigned char *)(gsWMV78DecodedFrameAddress_NC[VA]+gsWMV78FrameSize+gsWMV78FrameSize/4);
#endif
		p_output_param->m_DecOutInfo.m_iPicType          = gsWMV78DecOutput.m_iPictureType;
		p_output_param->m_DecOutInfo.m_iDecodedIdx       = 0;
		p_output_param->m_DecOutInfo.m_iDispOutIdx       = 0;
		p_output_param->m_DecOutInfo.m_iDecodingStatus   = 1;
		p_output_param->m_DecOutInfo.m_iOutputStatus     = 1;
		p_output_param->m_DecOutInfo.m_iInterlacedFrame  = 0;
		p_output_param->m_DecOutInfo.m_iPictureStructure = 0;

		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC_WMV78] WMV78_DEC_DECODE failed Error code is 0x%x \n", ret );
			return -ret;
		}

		p_output_param->m_pInitialInfo = &gsWMV78DecInitialInfo;
	}
	else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
	{
		return RETCODE_SUCCESS;
	}
	else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
	{
		return RETCODE_SUCCESS;
	}
	else if( iOpCode == VDEC_CLOSE )
	{
		if(!vdec_codec_opened)
			return -RETCODE_NOT_INITIALIZED;
		
		if ( gsIsINITdone )
		{
			ret = TCC_WMV78_DEC( VDEC_CLOSE, &gsWMV78DecHandle, NULL, NULL );
			if( ret != RETCODE_SUCCESS )
			{
				DPRINTF( "[VDEC_WMV78] WMV78_DEC_CLOSE failed Error code is 0x%x \n", ret );
				ret = -ret;
			}
		}

		vdec_codec_opened = 0;
		gsIsINITdone = 0;

		if ( gsWMV78DecInit.m_pHeapAddress )
			TCC_free(gsWMV78DecInit.m_pHeapAddress);
		if ( gsWMV78DecOutput.m_pDecodedData )
			TCC_free(gsWMV78DecOutput.m_pDecodedData);
		if ( gsWMV78DecInit.m_pCurFrameAddress )
			TCC_free(gsWMV78DecInit.m_pCurFrameAddress);
		if ( gsWMV78DecInit.m_pRef0FrameAddress )
			TCC_free(gsWMV78DecInit.m_pRef0FrameAddress);

		if( gsBitWorkBufAddr[PA] )
			cdk_sys_free_physical_addr( (void*)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
		if( gsBitWorkBufAddr[VA] )
			cdk_sys_free_virtual_addr( &gsBitWorkBufDev, gsBitWorkBufAddr[VA], gsBitWorkBufSize );
		if( gsSliceBufAddr )
			cdk_sys_free_physical_addr( (void*)gsSliceBufAddr, gsSliceBufSize );
		if( gsSpsPpsAddr )
			cdk_sys_free_physical_addr( (void*)gsSpsPpsAddr, gsSpsPpsSize );
		if( gsFrameBufAddr[PA] )
			cdk_sys_free_physical_addr( (void*)gsFrameBufAddr[PA], gsFrameBufSize );
		if( gsFrameBufAddr[VA] )
			cdk_sys_free_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[VA], gsFrameBufSize );

		if( gsWMV78DecodedFrameAddress_NC[PA] )
			cdk_sys_free_physical_addr( (void*)gsWMV78DecodedFrameAddress_NC[PA], gsWMV78NCFrameSize );
		if( gsWMV78DecodedFrameAddress_NC[VA] )
			cdk_sys_free_virtual_addr( &gsWMV78DecodedFrmaeBufferDev, gsWMV78DecodedFrameAddress_NC[VA], gsWMV78NCFrameSize );

		if( gsWMV78CurYFrameAddress )
			TCC_free( (void*)gsWMV78CurYFrameAddress );
		if( gsWMV78Ref0YFrameAddress )
			TCC_free( (void*)gsWMV78Ref0YFrameAddress );		

#ifdef HAVE_ANDROID_OS
		vpu_env_close();
#endif
	}
	else
	{
		DPRINTF( "Invaild Operation!!\n" );
		return -ret;
	}

	return ret;
}
#endif

#ifdef INCLUDE_SORENSON263_DEC

cdk_result_t
sorensonH263_dec_seq_header( int iSize , dec_initial_info_t * psInitialInfo )
{
	int ret = 0;
	int width, height;
	h263dec_stats_t s263_dec_stat;
	h263dec_input_t s263_dec_input;

#ifdef HAVE_ANDROID_OS
	memset(&s263_dec_stat, 0, sizeof(h263dec_stats_t));
	memset(&s263_dec_input, 0, sizeof(h263dec_input_t));
#else
	cdk_memset(&s263_dec_stat, 0, sizeof(h263dec_stats_t));
	cdk_memset(&s263_dec_input, 0, sizeof(h263dec_input_t));
#endif

	s263_dec_input.m_pBitstreamStartAddr = (void *)gsS263DecInit.m_BitstreamBufAddr[VA];
	if( iSize == 0 )
	{
		iSize = gsS263DecInit.m_iBitstreamBufSize;
		if( iSize == 0 )
		{
			DPRINTF( "[VDEC] H263_DEC_SEQ_HEADER failed : BitstreamBufSize \n" );
			return CDK_ERR_INVALID_PARAM;
		}
	}
	s263_dec_input.m_iBitstreamSize = iSize;
#ifdef SORENSON263_DEC_OUTPUT_MEM_ALIGN16
	s263_dec_input.m_iDecOption = Th263_SETIMAGEALIGN16;
#else
	s263_dec_input.m_iDecOption = 0;
#endif

	ret = TCC_H263_DEC( H263_DEC_SEQ_HEADER, (void *)gsS263DecHandle, &s263_dec_input, &s263_dec_stat );
	if( ret == Th263_ERR_NOT_SUPPORT )
	{
		DPRINTF( "[VDEC] H263_DEC_SEQ_HEADER failed Error code is 0x%x (not support)\n", ret );
		return ret;
	}
	if( ret < RETCODE_SUCCESS )
	{
		DPRINTF( "[VDEC] H263_DEC_SEQ_HEADER failed Error code is 0x%x \n", ret );
		return ret;
	}

#ifdef HAVE_ANDROID_OS
  memcpy(&gsS263DecInitialStats, &s263_dec_stat, sizeof(h263dec_stats_t));
	memset(&gsS263DecInitialInfo, 0, sizeof(dec_initial_info_t));

#else
	cdk_memcpy(&gsS263DecInitialStats, &s263_dec_stat, sizeof(h263dec_stats_t));
	cdk_memset(&gsS263DecInitialInfo, 0, sizeof(dec_initial_info_t));

#endif
	

	gsS263DecInitialInfo.m_iMinFrameBufferCount = 1;
	gsS263DecInitialInfo.m_iMinFrameBufferSize = s263_dec_stat.m_iNeedFrameBufferSize;		// minimum frame buffer size

	width = s263_dec_stat.m_uInfo.m_sVol.m_iWidth;
	width = ( (width + 1 ) & (~0x1) );
	gsS263DecInitialInfo.m_iPicWidth = width; // {(PicX+1)/2} * 2  (this width  will be used while allocating decoder frame buffers. picWidth  is a multiple of 2)

	height = s263_dec_stat.m_uInfo.m_sVol.m_iHeight;
	height = ( (height+ 1 ) & (~0x1) );
	gsS263DecInitialInfo.m_iPicHeight = height; // {(PicY+1)/2} * 2  (this height will be used while allocating decoder frame buffers. picHeight is a multiple of 2)

	//  represents rectangular window information in a frame 
	gsS263DecInitialInfo.m_iAvcPicCrop.m_iCropLeft = s263_dec_stat.m_uInfo.m_sVol.m_iCropOffset_Left;		
	gsS263DecInitialInfo.m_iAvcPicCrop.m_iCropRight = s263_dec_stat.m_uInfo.m_sVol.m_iCropOffset_Right;
	gsS263DecInitialInfo.m_iAvcPicCrop.m_iCropTop = s263_dec_stat.m_uInfo.m_sVol.m_iCropOffset_Top;
	gsS263DecInitialInfo.m_iAvcPicCrop.m_iCropBottom = s263_dec_stat.m_uInfo.m_sVol.m_iCropOffset_Bottom;

	if( gsS263DecInit.m_iPicWidth != gsS263DecInitialInfo.m_iPicWidth ||
		gsS263DecInit.m_iPicHeight != gsS263DecInitialInfo.m_iPicHeight  )
	{
		gsS263DecInit.m_iPicWidth = gsS263DecInitialInfo.m_iPicWidth;
		gsS263DecInit.m_iPicHeight = gsS263DecInitialInfo.m_iPicHeight;
		DPRINTF( "[VDEC] H263_DEC_SEQ_HEADER : resolution changing %dx%d => %dx%d\n",
			gsS263DecInit.m_iPicWidth, gsS263DecInit.m_iPicHeight,
			gsS263DecInitialInfo.m_iPicWidth, gsS263DecInitialInfo.m_iPicHeight);
	}			

#if defined(SORENSON263_DEC_OUTPUT_MEM_ALIGN16) || defined(SORENSON263_DEC_OUTPUT_MEM_ALIGN16_STRIDE)
	if( 1 )
	{
		int	width_tmp = s263_dec_stat.m_uInfo.m_sVol.m_iWidth;
		if( width_tmp & 0x0F ) 
		{
			width = ( (width_tmp + 15 ) & (~15) ); //multiple of 16 for M2M scaler
		}		
	}
#endif
			
#ifdef HAVE_ANDROID_OS
	if(psInitialInfo != NULL)
		memcpy(psInitialInfo, &gsS263DecInitialInfo, sizeof(dec_initial_info_t));

#else
	if(psInitialInfo != NULL)
	#ifdef HAVE_ANDROID_OS
		memcpy(psInitialInfo, &gsS263DecInitialInfo, sizeof(dec_initial_info_t));
	#else
		cdk_memcpy(psInitialInfo, &gsS263DecInitialInfo, sizeof(dec_initial_info_t));
	#endif
#endif
	
	print_dec_initial_info( &gsS263DecInit, &gsS263DecInitialInfo );

	/* Display output buffer */

	gsFrameBufSize = (int)(width * height * 1.5);
	gsFrameBufSize = ALIGNED_BUFF( gsFrameBufSize, 4*1024 );
	gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsFrameBufSize );
	if( gsFrameBufAddr[PA] == 0 ) 
	{
		DPRINTF( "[VDEC] gsFrameBufAddr[PA] malloc() failed \n");
		return CDK_ERR_MALLOC;
	}	
	gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[PA], gsFrameBufSize );
	if( gsFrameBufAddr[VA] == 0 ) 
	{
		DPRINTF( "[VDEC] gsFrameBufAddr[VA] malloc() failed \n");
		return CDK_ERR_MALLOC;
	}
#ifdef HAVE_ANDROID_OS
	memset( gsFrameBufAddr[VA], 0, gsFrameBufSize);
#else
	cdk_memset( gsFrameBufAddr[VA], 0, gsFrameBufSize);
#endif
	DSTATUS( "[VDEC] gsFrameBufAddr [PA] = 0x%x, [VA] = 0x%x, size = 0x%x \n",		\
												(codec_addr_t)gsFrameBufAddr[PA],	\
												(codec_addr_t)gsFrameBufAddr[VA],	\
												gsFrameBufSize );

	/* decoding instance buffer */

	gsSliceBufSize = s263_dec_stat.m_iNeedFrameBufferSize;
	
	#ifdef HAVE_ANDROID_OS
	gsSliceBufAddr = (codec_addr_t)vdec_malloc( gsSliceBufSize );
	#else
	gsSliceBufAddr = (codec_addr_t)cdk_malloc( gsSliceBufSize );
	#endif
	
	if( gsSliceBufAddr == 0 )
	{
		DPRINTF( "[VDEC] gsSliceBufAddr malloc() failed \n");
		return CDK_ERR_MALLOC;
	}
	DSTATUS( "[VDEC] gsSliceBufAddr = 0x%x, 0x%x \n", (codec_addr_t)gsSliceBufAddr, gsSliceBufSize );

	ret = TCC_H263_DEC( H263_DEC_REG_FRAME_BUFFER, (void *)gsS263DecHandle, (void *)gsSliceBufAddr, (void *)NULL );
	if( ret < RETCODE_SUCCESS )
	{
		DPRINTF( "[VDEC] H263_DEC_REG_FRAME_BUFFER failed Error code is 0x%x \n", ret );
		return ret;
	}
	DSTATUS("[VDEC] TCC_H263_DEC H263_DEC_REG_FRAME_BUFFER OK!\n");
	DSTATUS( "===================================================================\n" );
	return ret;
}

#define SORENSON263_MIN_USEFUL_BYTES 1

cdk_result_t
vdec_sorensonH263dec( int iOpCode, cdk_handle_t* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;

	if( iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !vdec_codec_opened)
		return -RETCODE_NOT_INITIALIZED;

	if( iOpCode == VDEC_INIT )
	{
		int heap_size;
		h263dec_init_t s263_dec_init;

		vdec_init_t* p_input_param = (vdec_init_t*)pParam1;

#ifdef HAVE_ANDROID_OS
		if(vpu_env_open(p_input_param->m_iBitstreamFormat, 0, 0, p_input_param->m_iPicWidth, p_input_param->m_iPicHeight ) < 0)
			return -1;
#endif

#ifdef HAVE_ANDROID_OS
		gsS263DecInit.m_iBitstreamFormat	= p_input_param->m_iBitstreamFormat;
		gsS263DecInit.m_iPicWidth			= p_input_param->m_iPicWidth;
		gsS263DecInit.m_iPicHeight			= p_input_param->m_iPicHeight;
		gsS263DecInit.m_Memcpy				=  (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsS263DecInit.m_Memset				=(void  (*) ( void*, int, unsigned int ))memset;
		vpu_sorensonH263dec_ready(&gsS263DecInit);
#else
		gsS263DecInit.m_iBitstreamFormat	= p_input_param->m_iBitstreamFormat;
		gsS263DecInit.m_iPicWidth			= p_input_param->m_iPicWidth;
		gsS263DecInit.m_iPicHeight			= p_input_param->m_iPicHeight;
		gsS263DecInit.m_Memcpy				= p_input_param->m_pfMemcpy;
		gsS263DecInit.m_Memset				= p_input_param->m_pfMemset;
		gsS263DecInit.m_BitstreamBufAddr[PA] = p_input_param->m_BitstreamBufAddr[PA];
		gsS263DecInit.m_BitstreamBufAddr[VA] = p_input_param->m_BitstreamBufAddr[VA];
		gsS263DecInit.m_iBitstreamBufSize = p_input_param->m_iBitstreamBufSize;
#endif
		gsbHasSeqHeader = 0;//p_input_param->m_bHasSeqHeader; 

		heap_size = H263_DEC_HEAP_SIZE;
		
		#ifdef HAVE_ANDROID_OS
		gsS263DecHeapAddr = (unsigned char *) vdec_malloc( sizeof(unsigned char)*heap_size );
		#else
		gsS263DecHeapAddr = (unsigned char *) cdk_malloc( sizeof(unsigned char)*heap_size );
		#endif
		
		if( gsS263DecHeapAddr == 0 ) 
		{
			DPRINTF( "[VDEC] gsS263DecHeapAddr malloc() failed \n");
			return -1;
		}

#ifdef HAVE_ANDROID_OS
		memset( &s263_dec_init, 0, sizeof(h263dec_init_t) );
#else
		cdk_memset( &s263_dec_init, 0, sizeof(h263dec_init_t) );
#endif

		s263_dec_init.m_iCodecType = Th263_SORENSON_H263;
		s263_dec_init.m_pHeapAddr = gsS263DecHeapAddr;
		s263_dec_init.m_iHeapSize = heap_size;
		s263_dec_init.m_Memcpy = gsS263DecInit.m_Memcpy;
		s263_dec_init.m_Memset = gsS263DecInit.m_Memset;

		ret =  TCC_H263_DEC( H263_DEC_INIT, NULL, (void *)&s263_dec_init, (void *)NULL );
		gsS263DecHandle = (codec_handle_t)s263_dec_init.m_pHandle;
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] H263_DEC_INIT failed Error code is 0x%x \n", ret );
			return -ret;
		}		
		vdec_codec_opened = 1;
		DSTATUS( "[VDEC] H263_DEC_INIT OK( has seq = %d) \n", gsbHasSeqHeader );
	}
	else if( iOpCode == VDEC_DEC_SEQ_HEADER )
	{		
	
	#ifdef HAVE_ANDROID_OS
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
		int seq_stream_size = p_input_param->m_iInpLen;

		gsS263DecInit.m_BitstreamBufAddr[PA] = gsBitstreamBufAddr[PA];
		gsS263DecInit.m_BitstreamBufAddr[VA] = gsBitstreamBufAddr[VA];
		memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], p_input_param->m_iInpLen);

		print_frame_data("sdcard/vpu_inSeq.bin", p_input_param->m_iInpLen);		
	#else
		int seq_stream_size = (int)pParam1;
	#endif

		DSTATUS( "[VDEC] H263_DEC_SEQ_HEADER start seq_stream_size %d \n" ,seq_stream_size);

	#ifdef HAVE_ANDROID_OS
		ret = sorensonH263_dec_seq_header(seq_stream_size, NULL);
	#else
		ret = sorensonH263_dec_seq_header(seq_stream_size, pParam2);
	#endif

		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] H263_DEC_SEQ_HEADER failed Error code is 0x%x \n", ret );
			return ret;
		}
		
		#ifdef HAVE_ANDROID_OS
			gsbHasSeqHeader = 1;
			p_output_param->m_pInitialInfo = &gsS263DecInitialInfo;
		#endif
		DSTATUS( "[VDEC] H263_DEC_SEQ_HEADER - Success \n" );
	}
	else if( iOpCode == VDEC_DECODE )
	{
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;

		h263dec_input_t s263_dec_input;
		h263dec_stats_t s263_dec_stat;

		char *pBitstreamAddr;
		int useful_bytes, used_bytes;

		unsigned char* pDispOut[2][3];
		int dispOutSizeY;
		int stride;
		
		#ifdef HAVE_ANDROID_OS
		memset(&s263_dec_input, 0, sizeof(h263dec_input_t));
		#else
		cdk_memset(&s263_dec_input, 0, sizeof(h263dec_input_t));
		#endif


	#ifdef HAVE_ANDROID_OS
		//memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], p_input_param->m_iInpLen);
		//pBitstreamAddr = (char *)gsBitstreamBufAddr[VA];
		print_frame_data("sdcard/vpu_inDec.bin", p_input_param->m_iInpLen); 	

		pBitstreamAddr = (char *)p_input_param->m_pInp[VA];
	#else
		pBitstreamAddr = (char *)p_input_param->m_pInp[VA];
	#endif
		useful_bytes = p_input_param->m_iInpLen;

		/* Output frame structure */
		s263_dec_input.m_DispOout.m_pPlane[0]  = (void *)gsFrameBufAddr[VA];
		
		stride = gsS263DecInitialInfo.m_iPicWidth;
	#if defined(SORENSON263_DEC_OUTPUT_MEM_ALIGN16) || defined(SORENSON263_DEC_OUTPUT_MEM_ALIGN16_STRIDE)
		if( stride&0x0F )
		{
			stride = (stride+15)&(~15);	//multiple of 16 for M2M scaler
		}
	#endif

		s263_dec_input.m_DispOout.m_iStride[0] = stride;
		s263_dec_input.m_DispOout.m_iCsp = Th263_CSP_I420;

		dispOutSizeY = stride * gsS263DecInitialInfo.m_iPicHeight;
		pDispOut[PA][0] = (unsigned char* )gsFrameBufAddr[PA];
		pDispOut[VA][0] = (unsigned char* )gsFrameBufAddr[VA];
		pDispOut[PA][1] = (unsigned char* )pDispOut[PA][0] + dispOutSizeY;
		pDispOut[VA][1] = (unsigned char* )pDispOut[VA][0] + dispOutSizeY;
		pDispOut[PA][2] = (unsigned char* )pDispOut[PA][1] + dispOutSizeY/4;
		pDispOut[VA][2] = (unsigned char* )pDispOut[VA][1] + dispOutSizeY/4;

		do 
		{
			/* Input stream */
			s263_dec_input.m_pBitstreamStartAddr = pBitstreamAddr;
			s263_dec_input.m_iBitstreamSize      = useful_bytes;
		#ifdef SORENSON263_DEC_OUTPUT_MEM_ALIGN16
			s263_dec_input.m_iDecOption			 = Th263_SETIMAGEALIGN16;
		#else
			s263_dec_input.m_iDecOption = 0;
		#endif

			memset(&s263_dec_stat, 0, sizeof(h263dec_stats_t));
			s263_dec_stat.m_iInterMBinP_CheckFlag = 1;

			/* Decode frame */
			ret = TCC_H263_DEC( H263_DEC_DECODE, (void *)gsS263DecHandle, (void *)&s263_dec_input, (void *)&s263_dec_stat );
			if( ret == RETCODE_SUCCESS )
				break;

			used_bytes = s263_dec_stat.m_iUsedBytes;
			if(used_bytes <= 0) 
			{
				/* Update buffer pointers */
				pBitstreamAddr += 1;
				useful_bytes -= 1;
			}
			else 
			{
				/* Update buffer pointers */
				pBitstreamAddr += used_bytes;
				useful_bytes -= used_bytes;
			}
		} while (s263_dec_stat.m_iDataType <= 0 && useful_bytes > SORENSON263_MIN_USEFUL_BYTES);

		if( ret < RETCODE_SUCCESS )
		{
		#if 0
			DPRINTF( "[VDEC] TCC_H263_DEC failed Error code is 0x%x \n", ret );
			return ret;
		#else
			p_output_param->m_DecOutInfo.m_iDecodedIdx       = -1;
			p_output_param->m_DecOutInfo.m_iDispOutIdx       = -1;
			p_output_param->m_DecOutInfo.m_iDecodingStatus   = 0;
			p_output_param->m_DecOutInfo.m_iOutputStatus     = 0;
			p_output_param->m_DecOutInfo.m_iPicType = 3;
			return RETCODE_SUCCESS;
		#endif
		}

	#ifdef HAVE_ANDROID_OS
		memset( &p_output_param->m_DecOutInfo, 0, sizeof(dec_output_info_t));
	#else
		cdk_memset( &p_output_param->m_DecOutInfo, 0, sizeof(dec_output_info_t));
	#endif

		p_output_param->m_DecOutInfo.m_iDecodedIdx       = 0;
		p_output_param->m_DecOutInfo.m_iDispOutIdx       = 0;
		p_output_param->m_DecOutInfo.m_iDecodingStatus   = 1;
		p_output_param->m_DecOutInfo.m_iOutputStatus     = 1;
		{
			int iPicType = -1;
			if( s263_dec_stat.m_iDataType == Th263_TYPE_IVOP )		/* intra frame */
				iPicType = PIC_TYPE_I;
			else if( s263_dec_stat.m_iDataType == Th263_TYPE_PVOP ) /* predicted frame */
				iPicType = PIC_TYPE_P;
			p_output_param->m_DecOutInfo.m_iPicType      = iPicType;
		}

		p_output_param->m_pDispOut[0][0] =
		p_output_param->m_pCurrOut[0][0] = (unsigned char *) pDispOut[PA][0];
		p_output_param->m_pDispOut[0][1] =
		p_output_param->m_pCurrOut[0][1] = (unsigned char *) pDispOut[PA][1];
		p_output_param->m_pDispOut[0][2] =
		p_output_param->m_pCurrOut[0][2] = (unsigned char *) pDispOut[PA][2];

		p_output_param->m_pDispOut[1][0] =
		p_output_param->m_pCurrOut[1][0] = (unsigned char *) pDispOut[VA][0];
		p_output_param->m_pDispOut[1][1] =
		p_output_param->m_pCurrOut[1][1] = (unsigned char *) pDispOut[VA][1];
		p_output_param->m_pDispOut[1][2] =
		p_output_param->m_pCurrOut[1][2] = (unsigned char *) pDispOut[VA][2];

		/* ignore when display output buffer is only one frame.
		p_output_param->m_pPrevOut[0][0] = (unsigned char *) [PA];
		p_output_param->m_pPrevOut[0][1] = (unsigned char *) [PA];
		p_output_param->m_pPrevOut[0][2] = (unsigned char *) [PA];
		p_output_param->m_pPrevOut[1][0] = (unsigned char *) [VA];
		p_output_param->m_pPrevOut[1][1] = (unsigned char *) [VA];
		p_output_param->m_pPrevOut[1][2] = (unsigned char *) [VA];
		*/

		p_output_param->m_pInitialInfo = &gsS263DecInitialInfo;
	}
	else if( iOpCode == VDEC_BUF_FLAG_CLEAR )
	{
		return RETCODE_SUCCESS;
	}
	else if( iOpCode == VDEC_DEC_FLUSH_OUTPUT)
	{
		return RETCODE_SUCCESS;
	}
	else if( iOpCode == VDEC_CLOSE )
	{
		if(!vdec_codec_opened)
			return -RETCODE_NOT_INITIALIZED;
		
		ret = TCC_H263_DEC( H263_DEC_CLOSE, (void *)gsS263DecHandle, (void *)NULL, (void *)NULL );
		gsS263DecHandle = 0;
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VDEC] H263_DEC_CLOSE failed Error code is 0x%x \n", ret );
			ret = -ret;
		}
		vdec_codec_opened = 0;

		if( gsS263DecHeapAddr )
		{
		#ifdef HAVE_ANDROID_OS
			vdec_free( (void*)gsS263DecHeapAddr );
		#else
			cdk_free( gsS263DecHeapAddr );
		#endif
			gsS263DecHeapAddr = 0;
		}

		if( gsSliceBufAddr )
		{
		#ifdef HAVE_ANDROID_OS
			vdec_free( (void*)gsSliceBufAddr );
		#else
			cdk_free( (void*)gsSliceBufAddr );
		#endif
			gsSliceBufAddr = 0;
		}

		if( gsFrameBufAddr[PA] )
			cdk_sys_free_physical_addr( (void*)gsFrameBufAddr[PA], gsFrameBufSize );
		if( gsFrameBufAddr[VA] )
			cdk_sys_free_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[VA], gsFrameBufSize );

		#ifdef HAVE_ANDROID_OS
			vpu_env_close();
		#endif

	}
	else
	{
		DPRINTF( "Invaild Operation!!\n" );
		return -ret;
	}

	return ret;
}


#endif

#ifdef JPEG_DECODE_FOR_MJPEG
int
jpeg_dec_ready( dec_init_t* psVDecInit )
{
	//------------------------------------------------------------
	//! [x] bitstream buffer for each VPU decoder
	//------------------------------------------------------------
	gsMaxBitstreamSize = gsBitstreamBufSize = JPEGDEC_STREAM_MAX;
	gsBitstreamBufSize = ALIGNED_BUFF( gsBitstreamBufSize, 4*1024 );
	gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitstreamBufSize );
	if( gsBitstreamBufAddr[PA] == 0 ) 
	{
		DPRINTF( "[VDEC] bitstream_buf_addr[PA] malloc() failed \n");
		return -1;
	}
	DSTATUS( "[VDEC] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
	gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitStreamBufDev, gsBitstreamBufAddr[PA], gsBitstreamBufSize );
	if( gsBitstreamBufAddr[VA] == 0 ) 
	{
		DPRINTF( "[VDEC] bitstream_buf_addr[VA] malloc() failed \n");
		return -1;
	}
	memset( (void*)gsBitstreamBufAddr[VA], 0x00 , gsBitstreamBufSize);
	DSTATUS("[VDEC] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[VA], gsBitstreamBufSize );

	psVDecInit->m_BitstreamBufAddr[PA]	= gsBitstreamBufAddr[PA];
	psVDecInit->m_BitstreamBufAddr[VA]	= gsBitstreamBufAddr[VA];
	psVDecInit->m_iBitstreamBufSize 	= gsBitstreamBufSize;

	return 0;
}

int
vdec_jpeg( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;
	TCCXXX_JPEG_DEC_DATA dec_data;

	if( iOpCode != VDEC_INIT && iOpCode != VDEC_CLOSE && !vdec_codec_opened)
		return -RETCODE_NOT_INITIALIZED;

	if( iOpCode == VDEC_INIT )
	{
		vdec_init_t* p_input_param = (vdec_init_t*)pParam1;		
		vdec_user_info_t* p_input_user_param = (vdec_user_info_t*)pParam2;

		gsVpuDecUserInfo.bitrate_mbps = p_input_user_param->bitrate_mbps;
		gsVpuDecUserInfo.frame_rate   = p_input_user_param->frame_rate;
		gsVpuDecUserInfo.m_bJpegOnly  = p_input_user_param->m_bJpegOnly;
		
		if(vpu_env_open(p_input_param->m_iBitstreamFormat, p_input_user_param->bitrate_mbps, p_input_user_param->frame_rate, p_input_param->m_iPicWidth, p_input_param->m_iPicHeight ) < 0)
			return -VPU_ENV_INIT_ERROR;

		gsJPEGDecHandle = open(JPEGDEC_DEVICE, O_RDWR | O_NDELAY);
		if(!gsJPEGDecHandle)
		{
			LOGE("Err[%s] :: jpeg(%s) open failed!", strerror(errno), JPEGDEC_DEVICE);
			return -RETCODE_FAILURE;
		}
		
		gsVpuDecInit.m_RegBaseVirtualAddr	= gsRegisterBase;
		gsVpuDecInit.m_iBitstreamFormat		= p_input_param->m_iBitstreamFormat;
		gsVpuDecInit.m_iPicWidth			= p_input_param->m_iPicWidth;
		gsVpuDecInit.m_iPicHeight			= p_input_param->m_iPicHeight;
		gsVpuDecInit.m_bEnableUserData		= p_input_param->m_bEnableUserData;
		gsVpuDecInit.m_bEnableVideoCache	= p_input_param->m_bEnableVideoCache;
		gsVpuDecInit.m_bCbCrInterleaveMode  = p_input_param->m_bCbCrInterleaveMode;
		gsVpuDecInit.m_Memcpy				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsVpuDecInit.m_Memset				= (void  (*) ( void*, int, unsigned int ))memset;
		gsVpuDecInit.m_Interrupt			= (int  (*) ( void ))VpuInterrupt;	
		gsVpuDecInit.m_iFilePlayEnable		= p_input_param->m_bFilePlayEnable;
		gsbHasSeqHeader = 0;//p_input_param->m_bHasSeqHeader; 

		jpeg_dec_ready( &gsVpuDecInit );
		DSTATUS("Format : %d, Stream(0x%x, %d)", gsVpuDecInit.m_iBitstreamFormat, gsVpuDecInit.m_BitstreamBufAddr[PA], gsVpuDecInit.m_iBitstreamBufSize);

		ret = RETCODE_SUCCESS;
		vdec_codec_opened = 1;
	}
	else if( iOpCode == VDEC_DEC_SEQ_HEADER )
	{		
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
		int seq_stream_size = (p_input_param->m_iInpLen > gsMaxBitstreamSize) ? gsMaxBitstreamSize : p_input_param->m_iInpLen;

		if (    ((codec_addr_t)p_input_param->m_pInp[PA] == gsBitstreamBufAddr[PA])
		     && ((codec_addr_t)p_input_param->m_pInp[VA] == gsBitstreamBufAddr[VA]) )
		{
			gsVpuDecInit.m_BitstreamBufAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
			gsVpuDecInit.m_BitstreamBufAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
		}
		else
		{
			gsVpuDecInit.m_BitstreamBufAddr[PA] = gsBitstreamBufAddr[PA];
			gsVpuDecInit.m_BitstreamBufAddr[VA] = gsBitstreamBufAddr[VA];
			memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], seq_stream_size);
		}

		DSTATUS( "[VDEC_JPG] JPEG_DEC_SEQ_HEADER start  :: len = %d / %d , handle = 0x%x \n", seq_stream_size, p_input_param->m_iInpLen, gsJPEGDecHandle );

		memset(&dec_data, 0x00, sizeof(TCCXXX_JPEG_DEC_DATA));

		dec_data.source_addr = gsVpuDecInit.m_BitstreamBufAddr[PA];
		dec_data.file_length = seq_stream_size;
		dec_data.target_addr = gsVpuDecInit.m_BitstreamBufAddr[PA];
		dec_data.target_size = gsBitstreamBufSize;
		
		ret = ioctl(gsJPEGDecHandle, TCC_JPEGD_IOCTL_HEADER_INFO, &dec_data);
		if( ret < 0 )
		{		
			LOGE("Err[%s] :: jpeg(%d) TCC_JPEGD_IOCTL_HEADER_INFO failed!", strerror(errno), ret);

			if(ret == -1)
				return -RETCODE_INVALID_STRIDE;
			else
				return -ret;
		}
		gsVpuDecInitialInfo.m_iPicWidth = dec_data.pad_width;
		gsVpuDecInitialInfo.m_iPicHeight = dec_data.pad_height;
		DSTATUS( "[VDEC_JPG] Resolution %d x %d, Format : %d \n", gsVpuDecInitialInfo.m_iPicWidth, gsVpuDecInitialInfo.m_iPicHeight, dec_data.image_format);

		//!< MJPEG source chroma format(0 - 4:2:0, 1 - 4:2:2, 2 - 4:2:2 vertical, 3 - 4:4:4, 4 - 4:0:0 )(only for TCC93XX)
		if(dec_data.image_format == IMAGE_CHROMA_420)
			gsVpuDecInitialInfo.m_iMjpg_sourceFormat = 0;
		else
			gsVpuDecInitialInfo.m_iMjpg_sourceFormat = 1;		

		if(dec_data.image_format != IMAGE_CHROMA_420)
		{
			LOGI("MJPEG Format is not YUV420 (%d)", dec_data.image_format); //1:420, 2:422
		}
		
		gsbHasSeqHeader = 1;
		p_output_param->m_pInitialInfo = &gsVpuDecInitialInfo;

		//check the maximum/minimum video resolution limitation
		{
			vdec_info_t * pVdecInfo = (vdec_info_t *)&gsVpuDecInitialInfo;
			unsigned int buf_idx;
			
			decoded_buf_curIdx = 0;
			decoded_buf_size = pVdecInfo->m_iPicWidth*pVdecInfo->m_iPicHeight*2;
			decoded_buf_size = ALIGNED_BUFF(decoded_buf_size, 4*1024);
			
			for(buf_idx =0; buf_idx < MAX_NUM_OF_JPEG_ELEMENT; buf_idx++)
			{
				decoded_phyAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_physical_addr( decoded_buf_size );				
				if( decoded_phyAddr[buf_idx] == 0 ) 
				{
					DPRINTF( "[VDEC_JPG,Err:%d] vdec_vpu decoded_virtAddr[PA] alloc failed \n", ret );
					decoded_buf_maxcnt = buf_idx;
					break;
				}	
				decoded_virtAddr[buf_idx] = (codec_addr_t)cdk_sys_malloc_virtual_addr( NULL, decoded_phyAddr[buf_idx], decoded_buf_size ); 
				if( decoded_virtAddr[buf_idx] == 0 ) 
				{
					DPRINTF( "[VDEC_JPG,Err:%d] vdec_vpu decoded_virtAddr[VA] alloc failed \n", ret );
					decoded_buf_maxcnt = buf_idx;
					break;
				}

				decoded_buf_maxcnt = MAX_NUM_OF_JPEG_ELEMENT;
				DSTATUS("OUT-Buffer %d ::	PA = 0x%x, VA = 0x%x, size = 0x%x!!\n", buf_idx, decoded_phyAddr[buf_idx], decoded_virtAddr[buf_idx],	decoded_buf_size);
			}			
		}
		
		DSTATUS( "[VDEC_JPG] VDEC_DEC_SEQ_HEADER - Success \n" );
		DSTATUS( "=======================================================\n\n" );		
	}
	else if( iOpCode == VDEC_DECODE )
	{
		vdec_input_t* p_input_param = (vdec_input_t*)pParam1;
		vdec_output_t* p_output_param = (vdec_output_t*)pParam2;
#ifdef DEBUG_TIME_LOG
		clock_t start, end;
#endif

		gsVpuDecInput.m_iBitstreamDataSize = (p_input_param->m_iInpLen > gsMaxBitstreamSize) ? gsMaxBitstreamSize : p_input_param->m_iInpLen;
		if (    ((codec_addr_t)p_input_param->m_pInp[PA] == gsBitstreamBufAddr[PA])
		     && ((codec_addr_t)p_input_param->m_pInp[VA] == gsBitstreamBufAddr[VA]) )
		{
			gsVpuDecInput.m_BitstreamDataAddr[PA] = (codec_addr_t)p_input_param->m_pInp[PA];
			gsVpuDecInput.m_BitstreamDataAddr[VA] = (codec_addr_t)p_input_param->m_pInp[VA];
		}
		else
		{
			gsVpuDecInput.m_BitstreamDataAddr[PA] = gsBitstreamBufAddr[PA];
			gsVpuDecInput.m_BitstreamDataAddr[VA] = gsBitstreamBufAddr[VA];
			DSTATUS("memcpy : 0x%x - %d", gsVpuDecInput.m_BitstreamDataAddr[PA], gsVpuDecInput.m_iBitstreamDataSize);
			memcpy( (void*)gsBitstreamBufAddr[VA], (void*)p_input_param->m_pInp[VA], gsVpuDecInput.m_iBitstreamDataSize);
		}
				
#ifdef  VIDEO_DEC_PROFILE
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) |= (unsigned int)(GPIO_NUM);
#ifdef DEBUG_TIME_LOG		
		start = clock();
#endif
#endif

		// Start decoding a frame.
		memset(&dec_data, 0x00, sizeof(TCCXXX_JPEG_DEC_DATA));

		dec_data.source_addr = gsVpuDecInput.m_BitstreamDataAddr[PA];
		dec_data.file_length = gsVpuDecInput.m_iBitstreamDataSize;
		dec_data.target_addr = decoded_phyAddr[decoded_buf_curIdx];
		dec_data.target_size = decoded_buf_size;

		DSTATUS("out[%d] : 0x%x - %d", decoded_buf_curIdx, decoded_phyAddr[decoded_buf_curIdx], decoded_buf_size);		
		ret = ioctl(gsJPEGDecHandle, TCC_JPEGD_IOCTL_DEC, &dec_data);
		if( ret < 0 )
		{
			LOGE("Err[%s] :: jpeg(%d) TCC_JPEGD_IOCTL_DEC failed!", strerror(errno), ret);
			p_output_param->m_DecOutInfo.m_iOutputStatus = VPU_DEC_OUTPUT_FAIL;	
			p_output_param->m_DecOutInfo.m_iDecodingStatus = VPU_DEC_SUCCESS;
			return 0;
		}
		gsVpuDecOutput.m_DecOutInfo.m_iOutputStatus = VPU_DEC_OUTPUT_SUCCESS;
		gsVpuDecOutput.m_DecOutInfo.m_iDecodingStatus = VPU_DEC_SUCCESS;

#ifdef  VIDEO_DEC_PROFILE
#ifdef DEBUG_TIME_LOG		
		end = clock();
		dec_time[time_cnt] = (end-start)*1000/CLOCKS_PER_SEC;
		total_dec_time += dec_time[time_cnt];
		total_frame_cnt++;
		if(time_cnt != 0 && time_cnt % 29 == 0)
		{
			LOGD("VDEC_JPG %d ms: %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d, %3d",
				total_dec_time/total_frame_cnt, dec_time[0], dec_time[1], dec_time[2], dec_time[3], dec_time[4], dec_time[5], dec_time[6], dec_time[7], dec_time[8], dec_time[9], 
				dec_time[10], dec_time[11], dec_time[12], dec_time[13], dec_time[14], dec_time[15], dec_time[16], dec_time[17], dec_time[18], dec_time[19], 
				dec_time[20], dec_time[21], dec_time[22], dec_time[23], dec_time[24], dec_time[25], dec_time[26], dec_time[27], dec_time[28], dec_time[29]);
			time_cnt = 0;
		}
		else{
			time_cnt++;
		}
#endif
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) &= (unsigned int)(~GPIO_NUM);
#endif

		{
			unsigned int frame_size = gsVpuDecInitialInfo.m_iPicWidth*gsVpuDecInitialInfo.m_iPicHeight;
			unsigned char div_val = 4;

			memcpy((void*)p_output_param, (void*)&gsVpuDecOutput, sizeof(dec_output_t ) );

			if(gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 0)
				div_val = 4;
			else
				div_val = 2;
			
			p_output_param->m_pDispOut[PA][0] = (unsigned char*)decoded_phyAddr[decoded_buf_curIdx];
			p_output_param->m_pDispOut[PA][1] = (unsigned char*)p_output_param->m_pDispOut[PA][0] + frame_size;
			p_output_param->m_pDispOut[PA][2] = (unsigned char*)p_output_param->m_pDispOut[PA][1] + frame_size/div_val;
			p_output_param->m_pDispOut[VA][0] = (unsigned char*)decoded_virtAddr[decoded_buf_curIdx];
			p_output_param->m_pDispOut[VA][1] = (unsigned char*)p_output_param->m_pDispOut[VA][0] + frame_size;
			p_output_param->m_pDispOut[VA][2] = (unsigned char*)p_output_param->m_pDispOut[VA][1] + frame_size/div_val;

			if( gsVpuDecInitialInfo.m_iMjpg_sourceFormat == 1) //YUV422
			{
				//To Do::
			}

			decoded_buf_curIdx++;
			if(decoded_buf_curIdx >= decoded_buf_maxcnt)
				decoded_buf_curIdx = 0;
		}
		p_output_param->m_pInitialInfo = &gsVpuDecInitialInfo;
	}
	else if( iOpCode == VDEC_CLOSE )
	{
		if(!vdec_codec_opened && !vdec_env_opened)
			return -RETCODE_NOT_INITIALIZED;

		if(vdec_codec_opened)
		{
			if(close(gsJPEGDecHandle) < 0)
			{
				LOGE("Err[%s] :: jpeg(%s) close failed!", strerror(errno), JPEGDEC_DEVICE);
			}
			gsJPEGDecHandle = -1;
			vdec_codec_opened = 0;
		}
		
		if(!vdec_env_opened)
			return -RETCODE_NOT_INITIALIZED;
		
		if( gsVpuDecInit.m_iBitstreamFormat == STD_MJPG && !gsVpuDecUserInfo.m_bJpegOnly)
		{
			unsigned int buf_idx = 0;
			
			for(buf_idx =0; buf_idx < decoded_buf_maxcnt; buf_idx++)
			{
				cdk_sys_free_physical_addr( (void*)decoded_phyAddr[buf_idx], decoded_buf_size );
				decoded_phyAddr[buf_idx] = 0;
				cdk_sys_free_virtual_addr( NULL, decoded_virtAddr[buf_idx], decoded_buf_size );
				decoded_virtAddr[buf_idx] = 0;
			}			
		}
		vpu_env_close();
	}
	else
	{
		return 0;
	}

	return ret;
}
#endif

int vpu_preCtrl(int curr_fd)
{
	int fd;

	if(curr_fd > 0)
	{
		if(close(curr_fd) < 0)
		{
			LOGE("Err[%s] :: vpu(%s) close failed!", strerror(errno), TCC_VPU_DEV_NAME);
		}	
		LOGI( " VPU PreOpen Closed!! " );

		return 0;
	}
	else
	{
		fd = open(TCC_VPU_DEV_NAME, O_RDWR | O_NDELAY);
		if(!fd)
		{
			LOGE("Err[%s] :: vpu(%s) open failed!", strerror(errno), TCC_VPU_DEV_NAME);
			return -1;
		}
		
		LOGI( " VPU PreOpened!! " );

		return fd;
	}
	
	return 0;
}

