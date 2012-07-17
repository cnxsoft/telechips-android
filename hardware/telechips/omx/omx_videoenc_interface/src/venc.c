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
#define LOG_TAG	"VPU_ENC"
#include <utils/Log.h>
	 
#include "cdk_core.h"
#include "venc.h"
	 
#include <sys/mman.h>
#include <string.h>
#include <sys/ioctl.h>
#include <mach/tcc_vpu_ioctl.h>
#include <errno.h>

#ifdef VPU_CLK_CONTROL
#include "vpu_clk_ctrl.h"
#endif
	 
#include <libpmap/pmap.h>

/************************************************************************/
/* TEST and Debugging                                               								 */
/************************************************************************/
 static int DEBUG_ON = 0;
#define DPRINTF(msg...)	 LOGE( ": " msg);
#define DSTATUS(msg...)	 if (DEBUG_ON) { LOGD( ": " msg);}
#define DPRINTF_FRAME(msg...) //LOGD(": " msg);

//#define VIDEO_ENC_PROFILE
#ifdef VIDEO_ENC_PROFILE
#ifdef _TCC9300_
#define	TEST_GPIO_BASE  			0xB010A000
// TCC9300 => DXB_6pin, GPIO_C28
#define GPIO_BASE_OFFSET 0x80
#define GPIO_NUM		 0x10000000
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
#endif
#define TCC_VPU_INPUT_BUF_SIZE 		(1024 * 1024)

#define STABILITY_GAP (512)
// delete by shmin for M2TS
//#define REMOVE_NALSTARTCODE //We will remove NAL StartCode(4byte), because VPU output-Data already has it!!

#define USE_VPU_INTERRUPT
#ifdef USE_VPU_INTERRUPT
//#define VPU_REGISTER_DUMP
#define TCC_INTR_DEV_NAME		"/dev/tcc_intr"//"/dev/tcc-intr-vc"

#include <fcntl.h>         // O_RDWR
#include <sys/poll.h>
int vpu_intr_fd = -1;
struct pollfd tcc_event[1];
#endif

unsigned char venc_env_opened = 0;

int dev_fd = -1;
#ifdef HAVE_ANDROID_OS
#define TCC_VPU_DEV_NAME	"/dev/vpu"
#else
#define TCC_VPU_DEV_NAME	"/dev/mem"
#endif

#else
#include "venc.h"
#include "../cdk/cdk_core.h"
#include "../cdk/cdk_sys.h"
#endif

/************************************************************************/
/* STATIC MEMBERS                                                       */
/************************************************************************/
static int gsBitWorkBufSize = 0;
static codec_addr_t gsBitWorkBufAddr[2] = {0,};
static int gsBitWorkBufDev = 0;

static codec_addr_t gsFrameBufAddr[2] = {0,};
static unsigned int gsFrameBufSize = 0;
static int gsFrameBufDev = -1;

static unsigned int gsMESearchBufSize = 0;
static codec_addr_t gsMESearchBufAddr[2] = {0,};
static int gsMESearchBufDev = 0;

static codec_handle_t		gsVpuEncHandle = 0;
static enc_init_t			gsVpuEncInit;
static enc_initial_info_t	gsVpuEncInitialInfo;
static enc_buffer_t			gsVpuEncBuffer;
static enc_input_t			gsVpuEncInput;
static enc_output_t			gsVpuEncOutput;

#ifdef HAVE_ANDROID_OS
static codec_addr_t gspSeqHeader = 0, gspSeqHeader_phyAddr = 0;
static unsigned int gsiSeqHeaderBufDev = 0, gsiSeqHeaderSize = 0;
static unsigned int gsiSeqHeaderCnt = 0, gsiSzSeqHeader[2] = {0,};
#else
static unsigned char* gspSeqHeader = NULL;
static unsigned char* gspPictureData = NULL;
static int gsiPictureDataSize = 0;
#endif
static int gsiFrameIdx;

#ifdef HAVE_ANDROID_OS
static unsigned int 	gsLogicalBase = 0, gsRegisterBase = 0;
static unsigned int 	gsUsedPhysicalMemSize = 0;
static int gsBitstreamBufSize = 0;
static codec_addr_t gsBitstreamBufAddr[2] = {0, };
static int gsBitStreamBufDev = 0;

#define MAX_NUM_OF_VIDEO_ELEMENT 	VIDEO_ENC_BUFFER_COUNT
static unsigned int encoded_buf_size = 0;
static codec_addr_t encoded_buf_base_pos[2] = {0, };
static codec_addr_t encoded_buf_end_pos[2] = {0, };
static codec_addr_t encoded_buf_cur_pos[2] = {0, };
static unsigned int keyInterval_cnt = 0;

static unsigned int bAvcUsedNALStart = 0; // added by shmin for M2TS

static pmap_t pmap_video;

#define VIDEO_PHY_ADDR		pmap_video.base
#define VIDEO_MEM_SIZE		pmap_video.size

#ifdef MULTI_SLICES_AVC
static int enc_avc_aud_enable = 0;
static codec_addr_t enc_slice_info_addr[3] = { 0, };
static unsigned int enc_slice_info_size = 0;
const unsigned char avcAudData[8] = { 0x00,0x00,0x00,0x01,0x09,0x50,0x00,0x00 };
#endif

static void vpu_hw_reset();
static void vpu_env_close(void);

static unsigned char * sPhysicalMemSetting(unsigned int nPhysicalAddr, unsigned int size)
{	
	void *mem_ptr = MAP_FAILED;

	return (void *)mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, nPhysicalAddr);
}

static void sPhysicalMemFree(unsigned int LogicalAddr, unsigned int size)
{
	munmap((void*)LogicalAddr, size);
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

static void vpu_update_sizeinfo(unsigned int image_width, unsigned int image_height)
{
	CONTENTS_INFO info;

	memset(&info, 0x00, sizeof(CONTENTS_INFO));
	info.type = VPU_ENC;
	info.width = image_width;
	info.height = image_height;
	if(ioctl(dev_fd, VPU_SET_CLK, &info) < 0)
	{
		LOGE("Err[%s] :: VPU_SET_CLK failed!", strerror(errno));
	}

	return;
}

static int gPFrameCnt = 0, gMaxOutputSize_PFrame = 0;
static int gIFrameCnt = 0, gMaxOutputSize_IFrame = 0;
static int vpu_env_open(unsigned int image_width, unsigned int image_height)
{
	DSTATUS("In  %s \n",__func__);

	pmap_get_info("video", &pmap_video);

	if(venc_env_opened)
	{
		//to recover abnormal stop error!!
		LOGE("VENC has been already opened. so have to close!!");
		vpu_env_close();
	}

	dev_fd = open(TCC_VPU_DEV_NAME, O_RDWR | O_NDELAY);
	if(!dev_fd)
	{
		LOGE("%s open error!!", TCC_VPU_DEV_NAME);
		goto err;
	}
	vpu_update_sizeinfo(image_width, image_height);

#if !defined(_TCC8800_) //to init vpu codec!!
	vpu_hw_reset();
#endif

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

	gsRegisterBase	= (unsigned int)sPhysicalMemSetting(VPU_REG_BASE_ADDR, 8*1024);
	if(!gsRegisterBase)
		goto err;	

	DSTATUS("[VENC] gsLogicalBase = 0x%x, gsRegisterBase = 0x%x", (codec_addr_t)gsLogicalBase, (codec_addr_t)gsRegisterBase);
	gsUsedPhysicalMemSize = 0;

#ifdef VIDEO_ENC_PROFILE
	pTestRegBase	= sPhysicalMemSetting(TEST_GPIO_BASE, 1024);
	if(!pTestRegBase)
		goto err;	
#endif
	venc_env_opened = 1;
	gsiFrameIdx = 0;
	gPFrameCnt = gMaxOutputSize_PFrame = 0;
	gIFrameCnt = gMaxOutputSize_IFrame = 0;
	gspSeqHeader_phyAddr = gspSeqHeader = gsiSeqHeaderCnt = NULL;

	memset( &gsVpuEncInit,		0, sizeof(gsVpuEncInit) );
	memset( &gsVpuEncInput,		0, sizeof(gsVpuEncInput) );
	memset( &gsVpuEncInitialInfo,	0, sizeof(gsVpuEncInitialInfo) );
	memset( &gsVpuEncInput,		0, sizeof(gsVpuEncInput) );

	memset(gsBitstreamBufAddr, 0x00, sizeof(gsBitstreamBufAddr));
	memset(gsBitWorkBufAddr, 0x00, sizeof(gsBitWorkBufAddr));
	memset(gsMESearchBufAddr, 0x00, sizeof(gsMESearchBufAddr));
	memset(gsFrameBufAddr, 0x00, sizeof(gsFrameBufAddr));
	memset(encoded_buf_base_pos, 0x00, sizeof(encoded_buf_base_pos));

	DSTATUS("Out  %s \n",__func__);

	return 0;

err:	
	LOGE("vpu_env_open error");
	vpu_env_close();
	
	return -1;	
}


static void vpu_env_close(void)
{
	DSTATUS("In  %s \n",__func__);

	if(gsLogicalBase)
		sPhysicalMemFree(gsLogicalBase, VIDEO_MEM_SIZE);

	if(gsRegisterBase)
		sPhysicalMemFree(gsRegisterBase, 8*1024);

	gsLogicalBase = 0;
	gsRegisterBase = 0;

#ifdef VIDEO_ENC_PROFILE
	if(pTestRegBase)
		sPhysicalMemFree(pTestRegBase, 1024);	

	pTestRegBase = 0;
#endif

#ifdef  USE_VPU_INTERRUPT
	if(vpu_intr_fd > 0)
	{
		if(close(vpu_intr_fd) < 0)
		{
			LOGE("%s close error[%s]", TCC_INTR_DEV_NAME, strerror(errno));
		}
		vpu_intr_fd = -1;
	}
#endif	

	if(dev_fd)
	{
		if(close(dev_fd) < 0)
		{
			LOGE("%s close error[%s]", TCC_VPU_DEV_NAME, strerror(errno));
		}	
	}
	venc_env_opened = 0;

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

	ckc_swrst	= (unsigned int)sPhysicalMemSetting(VIDEOCODECBUS_CLK_BASE_ADDR, 100);	 
	if(!ckc_swrst)
	   return;
	
	vidbus_swrst   = (unsigned int)sPhysicalMemSetting(VIDEOBUSCONF_BASE_ADDR, 100);
	if(!vidbus_swrst)
	   return;

	LOGI(" **************** addr:: ckc 0x%x, vid 0x%x ", ckc_swrst, vidbus_swrst);
	// reset
#if !defined(_TCC9300_) && !defined(_TCC8800_) // to prevent before chip-revision!!
	*((volatile unsigned int *)(ckc_swrst + ckc_swrst_offset))	 |= (unsigned int)0x00000040;
#endif
	*((volatile unsigned int *)(vidbus_swrst + vidbus_swrst_offset)) |= (unsigned int)0x00000004;
	
	//wait until reset for 1 ms
	usleep( 1 * 1000 );

	// not-reset
#if !defined(_TCC9300_) && !defined(_TCC8800_) // to prevent before chip-revision!!
	*((volatile unsigned int *)(ckc_swrst + ckc_swrst_offset))	 &= (unsigned int)(~(0x00000040));
#endif
	*((volatile unsigned int *)(vidbus_swrst + vidbus_swrst_offset)) &= (unsigned int)(~(0x00000004));

	//wait until VPU reboot for 10 ms
	usleep( 10 * 1000 );

	sPhysicalMemFree(ckc_swrst, 100);
	sPhysicalMemFree(vidbus_swrst, 100);

	LOGI(" **************** VPU H/W Reset ****************");
	//restart VPU process..
}



int
venc_vpu( int iOpCode, int* pHandle, void* pParam1, void* pParam2 )
{
	int ret = 0;

	if( iOpCode == VENC_INIT )
	{
		venc_init_t* p_init_param = (venc_init_t*)pParam1;

#ifdef HAVE_ANDROID_OS
		if(vpu_env_open(p_init_param->m_iPicWidth, p_init_param->m_iPicHeight ) < 0)
			return -VPU_ENV_INIT_ERROR;
		
		//temp - remove Interleaved setting!!
		write_reg(0x110, 0x00);
#endif
#if defined(VPU_CLK_CONTROL)
		vpu_clock_init();
#endif

		// added by shmin for M2TS
		bAvcUsedNALStart = *((unsigned int*)pParam2);

#ifdef HAVE_ANDROID_OS
		gsVpuEncInit.m_RegBaseVirtualAddr	= gsRegisterBase;
		//gsVpuEncInit.m_BitstreamBufferAddr	= p_init_param->m_BitstreamBufferAddr;		
		//gsVpuEncInit.m_BitstreamBufferAddr_VA = p_init_param->m_BitstreamBufferAddr_VA;
		//gsVpuEncInit.m_iBitstreamBufferSize 	= p_init_param->m_iBitstreamBufferSize;

		gsVpuEncInit.m_iBitstreamFormat 	= p_init_param->m_iBitstreamFormat;
		gsVpuEncInit.m_iPicWidth			= p_init_param->m_iPicWidth;
		gsVpuEncInit.m_iPicHeight			= p_init_param->m_iPicHeight;
		gsVpuEncInit.m_iFrameRate			= p_init_param->m_iFrameRate;
		gsVpuEncInit.m_iTargetKbps			= p_init_param->m_iTargetKbps;
		gsVpuEncInit.m_iKeyInterval			= p_init_param->m_iKeyInterval;// only first picture is I
		gsVpuEncInit.m_iAvcFastEncoding		= p_init_param->m_iAvcFastEncoding;
		
		gsVpuEncInit.m_bEnableVideoCache	= 0;
		gsVpuEncInit.m_bCbCrInterleaveMode	= 0;
#ifdef MULTI_SLICES_AVC		
		if( gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
		{
			gsVpuEncInit.m_iSliceMode			= p_init_param->m_iSliceMode;		// multi-slices per picture
			gsVpuEncInit.m_iSliceSizeMode		= p_init_param->m_iSliceSizeMode;
			gsVpuEncInit.m_iSliceSize			= p_init_param->m_iSliceSize;
			if( gsVpuEncInit.m_iSliceMode == 1 )
				enc_avc_aud_enable = 1;
			else
				enc_avc_aud_enable = 0;
		}
		else
#endif
		{
			gsVpuEncInit.m_iSliceMode			= 0;		// 1 slice per picture
			gsVpuEncInit.m_iSliceSizeMode		= 0;
			gsVpuEncInit.m_iSliceSize			= 0;
#ifdef MULTI_SLICES_AVC					
			enc_avc_aud_enable = 0;
#endif
		}
		DSTATUS( "SliceMode[%d] - SizeMode[%d] - %d", gsVpuEncInit.m_iSliceMode, gsVpuEncInit.m_iSliceSizeMode, gsVpuEncInit.m_iSliceSize );

		gsVpuEncInit.m_iIntraMBNumInPFrame	= 0;
		gsVpuEncInit.m_iIFrameQp			= -1;
		gsVpuEncInit.m_bEnableVideoCache	= 0;
		
		gsVpuEncInit.m_Memcpy				= (void* (*) ( void*, const void*, unsigned int ))memcpy;
		gsVpuEncInit.m_Memset				= (void  (*) ( void*, int, unsigned int ))memset;
		gsVpuEncInit.m_Interrupt			= (int  (*) ( void ))VpuInterrupt; 

		keyInterval_cnt = 0;
#else
		gsVpuEncInit.m_RegBaseVirtualAddr	= p_init_param->m_RegBaseVirtualAddr;
		gsVpuEncInit.m_BitstreamBufferAddr	= p_init_param->m_BitstreamBufferAddr;
		gsVpuEncInit.m_iBitstreamBufferSize = p_init_param->m_iBitstreamBufferSize;
		gsVpuEncInit.m_BitstreamBufferAddr_VA = p_init_param->m_BitstreamBufferAddr_VA;
		gsVpuEncInit.m_iBitstreamFormat		= p_init_param->m_iBitstreamFormat;
		gsVpuEncInit.m_iPicWidth			= p_init_param->m_iPicWidth;
		gsVpuEncInit.m_iPicHeight			= p_init_param->m_iPicHeight;
		gsVpuEncInit.m_iFrameRate			= p_init_param->m_iFrameRate;
		gsVpuEncInit.m_iTargetKbps			= p_init_param->m_iTargetKbps;
		gsVpuEncInit.m_iKeyInterval			= p_init_param->m_iKeyInterval;// only first picture is I
		gsVpuEncInit.m_iAvcFastEncoding		= p_init_param->m_iAvcFastEncoding;
		gsVpuEncInit.m_iSliceMode			= 0;		// 1 slice per picture
		gsVpuEncInit.m_iSliceSizeMode		= 0;
		gsVpuEncInit.m_iSliceSize			= 0;
		gsVpuEncInit.m_iIntraMBNumInPFrame	= 0;
		gsVpuEncInit.m_iIFrameQp			= -1;
		gsVpuEncInit.m_bEnableVideoCache	= 1;

		gsVpuEncInit.m_Memcpy				= p_init_param->m_pfMemcpy;
		gsVpuEncInit.m_Memset				= p_init_param->m_pfMemset;
		gsVpuEncInit.m_Interrupt			= p_init_param->m_pfInterrupt;
#endif

#ifdef HAVE_ANDROID_OS
		//------------------------------------------------------------
		//! [x] bitstream buffer for each VPU decoder
		//------------------------------------------------------------
		gsBitstreamBufSize = LARGE_STREAM_BUF_SIZE;
		gsBitstreamBufSize = ALIGNED_BUFF( gsBitstreamBufSize, 4*1024 );
		gsBitstreamBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitstreamBufSize );
		if( gsBitstreamBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC] bitstream_buf_addr[PA] malloc() failed \n");
			return -1;
		}
		DSTATUS( "[VENC] bitstream_buf_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		gsBitstreamBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitStreamBufDev, gsBitstreamBufAddr[PA], gsBitstreamBufSize );
		if( gsBitstreamBufAddr[VA] == 0 ) 
		{
			LOGE( "[VENC] bitstream_buf_addr[VA] malloc() failed \n");
			return -1;
		}
		memset( (void*)gsBitstreamBufAddr[VA], 0x00 , gsBitstreamBufSize);
		DSTATUS("[VENC] bitstream_buf_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[VA], gsBitstreamBufSize );
	
		gsVpuEncInit.m_BitstreamBufferAddr		= gsBitstreamBufAddr[PA];
		gsVpuEncInit.m_BitstreamBufferAddr_VA 	= gsBitstreamBufAddr[VA];
		gsVpuEncInit.m_iBitstreamBufferSize		= gsBitstreamBufSize;
#endif

		//------------------------------------------------------------
		//! [x] code buffer, work buffer and parameter buffer for VPU 
		//------------------------------------------------------------
		gsBitWorkBufSize = WORK_CODE_PARA_BUF_SIZE;
		gsBitWorkBufSize = ALIGNED_BUFF(gsBitWorkBufSize, 4*1024);
		gsBitWorkBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsBitWorkBufSize );
		if( gsBitWorkBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC] gsBitWorkBufAddr[PA] malloc() failed \n");
			return -1;
		}
		DSTATUS("[VENC] gsBitWorkBufAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
		gsBitWorkBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsBitWorkBufDev, gsBitWorkBufAddr[PA], gsBitWorkBufSize );
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
		gsMESearchBufSize = ( ( gsVpuEncInit.m_iPicWidth + 15 ) & ~15 ) * 36 + 2048;  // picWidth of searchram size must be a multiple of 16
		gsMESearchBufSize = ALIGNED_BUFF( gsMESearchBufSize, 4*1024 );
		//DSTATUS( "[CDK_CORE] gsMESearchBufSize = %d\n", gsMESearchBufSize );
		gsMESearchBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsMESearchBufSize );
		if( gsMESearchBufAddr[PA] == 0 ) 
		{
			LOGE( "[CDK_CORE] gsMESearchBufAddr[PA] physical malloc() failed \n");
			return CDK_ERR_MALLOC;
		}
		DSTATUS("[CDK_CORE] gsMESearchBufAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsMESearchBufAddr[PA], gsMESearchBufSize );
		gsMESearchBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsMESearchBufDev, gsMESearchBufAddr[PA], gsMESearchBufSize );
		if( gsMESearchBufAddr[VA] == 0 ) 
		{
			LOGE( "[CDK_CORE] gsMESearchBufAddr[VA] virtual malloc() failed \n");
			return CDK_ERR_MALLOC;
		}
		DSTATUS("[CDK_CORE] gsMESearchBufAddr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsMESearchBufAddr[VA], gsMESearchBufSize );

		gsVpuEncInit.m_BitWorkAddr[PA]		= gsBitWorkBufAddr[PA];
		gsVpuEncInit.m_BitWorkAddr[VA]		= gsBitWorkBufAddr[VA];
		gsVpuEncInit.m_MeSearchRamAddr		= gsMESearchBufAddr[PA];
		gsVpuEncInit.m_iMeSearchRamSize		= gsMESearchBufSize;

#ifdef MULTI_SLICES_AVC	
		if( gsVpuEncInit.m_iSliceMode == 1 )
		{
			//------------------------------------------------------------
			//! [x] Slice info. buffers requested by the encoder.
			//------------------------------------------------------------
			int iMbWidth, iMbHeight;
			
			iMbWidth = (gsVpuEncInit.m_iPicWidth+15)>>4;
			iMbHeight = (gsVpuEncInit.m_iPicHeight+15)>>4;
			
			enc_slice_info_size = iMbWidth * iMbHeight * 8 + 48;
			enc_slice_info_size = ALIGNED_BUFF(enc_slice_info_size, 4*1024);
			enc_slice_info_addr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( enc_slice_info_size );
			if( enc_slice_info_addr[PA] == 0 ) 
			{
				LOGE( "[CDK_CORE] enc_slice_info_addr[PA] physical malloc() failed \n");
				return CDK_ERR_MALLOC;
			}
			
			DSTATUS("[CDK_CORE] enc_slice_info_addr[PA] = 0x%x, 0x%x \n", (codec_addr_t)enc_slice_info_addr[PA], enc_slice_info_size );
			enc_slice_info_addr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsFrameBufDev, enc_slice_info_addr[PA], enc_slice_info_size );
			if( enc_slice_info_addr[VA] == 0 ) 
			{
				LOGE( "[CDK_CORE] enc_slice_info_addr[VA] virtual malloc() failed \n");
				return CDK_ERR_MALLOC;
			}
			DSTATUS("[CDK_CORE] enc_slice_info_addr[VA] = 0x%x, 0x%x \n", (codec_addr_t)enc_slice_info_addr[VA], enc_slice_info_size );
		}
#endif

		ret = TCC_VPU_ENC( VPU_ENC_INIT, &gsVpuEncHandle, &gsVpuEncInit, &gsVpuEncInitialInfo );  // ENCODER INIT
		if( ret != RETCODE_SUCCESS )
		{
			LOGE( "[VENC,Err:%d] venc_vpu VPU_ENC_INIT failed \n", ret );
			return -ret;
		}
		DSTATUS("[VENC] venc_vpu VPU_ENC_INIT ok! \n" );

		//------------------------------------------------------------
		//! [x] Register frame buffers requested by the encoder.
		//------------------------------------------------------------
		gsFrameBufSize = gsVpuEncInitialInfo.m_iMinFrameBufferCount * gsVpuEncInitialInfo.m_iMinFrameBufferSize;
		gsFrameBufSize = ALIGNED_BUFF(gsFrameBufSize, 4*1024);
		gsFrameBufAddr[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( gsFrameBufSize );
		if( gsFrameBufAddr[PA] == 0 ) 
		{
			LOGE( "[VENC,Err:%d] venc_vpu gsFrameBufAddr[PA] alloc failed \n", ret );
			return -ret;
		}	
		DSTATUS("[VENC] gsFrameBufAddr[PA] = 0x%x, 0x%x \n", (codec_addr_t)gsFrameBufAddr[PA], gsFrameBufSize );
		gsFrameBufAddr[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsFrameBufDev, gsFrameBufAddr[PA], gsFrameBufSize );
		if( gsFrameBufAddr[VA] == 0 ) 
		{
			LOGE( "[VENC,Err:%d] venc_vpu gsFrameBufAddr[VA] alloc failed \n", ret );
			return -ret;
		}
		DSTATUS("[VENC] gsFrameBufAddr[VA] = 0x%x, 0x%x \n", (codec_addr_t)gsFrameBufAddr[VA], gsFrameBufSize );

		gsVpuEncBuffer.m_FrameBufferStartAddr[PA] = gsFrameBufAddr[PA];
		gsVpuEncBuffer.m_FrameBufferStartAddr[VA] = gsFrameBufAddr[VA];

		ret = TCC_VPU_ENC( VPU_ENC_REG_FRAME_BUFFER, &gsVpuEncHandle, &gsVpuEncBuffer, 0 );  // register frame buffer
		if( ret != RETCODE_SUCCESS )
		{
			LOGE( "[VENC,Err:%d] venc_vpu VPU_ENC_REG_FRAME_BUFFER failed \n", ret );
			return -ret;
		}

		{
			unsigned int remained_mem_size;

			encoded_buf_size = (gsVpuEncInit.m_iTargetKbps/8 /*KB/s*/) * 1024/*Byte*/ * (VIDEO_ENC_BUFFER_COUNT/gsVpuEncInit.m_iFrameRate +3 /*add 3sec*/ );

			if(encoded_buf_size < (LARGE_STREAM_BUF_SIZE * 5)) //10MB
				encoded_buf_size = (LARGE_STREAM_BUF_SIZE * 5);
			encoded_buf_size = ALIGNED_BUFF(encoded_buf_size, 4*1024);

			remained_mem_size = cdk_sys_remain_memory_size();
			if(remained_mem_size < encoded_buf_size)
			{
				LOGE( "[VENC,Err] Insufficient memory for streamOut. \n");
				return -1;
			}
			
			encoded_buf_base_pos[PA] = (codec_addr_t)cdk_sys_malloc_physical_addr( encoded_buf_size );	
			if( !encoded_buf_base_pos[PA] ) 
			{
				LOGE( "[VENC,Err] venc_vpu encoded_phyAddr[PA] alloc failed \n" );
				return -1;
			}	
			encoded_buf_base_pos[VA] = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsFrameBufDev, encoded_buf_base_pos[PA], encoded_buf_size );	
			encoded_buf_cur_pos[PA] = encoded_buf_base_pos[PA];
			encoded_buf_cur_pos[VA] = encoded_buf_base_pos[VA];
			encoded_buf_end_pos[PA] = encoded_buf_base_pos[PA] + encoded_buf_size;
			encoded_buf_end_pos[VA] = encoded_buf_base_pos[VA] + encoded_buf_size;
			
			DSTATUS("OUT-Buffer ::	%d Kbps, %d fps, %d sec !!\n", gsVpuEncInit.m_iTargetKbps, gsVpuEncInit.m_iFrameRate, (VIDEO_ENC_BUFFER_COUNT/gsVpuEncInit.m_iFrameRate +3 /*add 3sec*/ ));
			DSTATUS("               PA = 0x%x ~ 0x%x, VA = 0x%x ~ 0x%x, size = 0x%x!!\n", encoded_buf_base_pos[PA], encoded_buf_end_pos[PA], 
										encoded_buf_base_pos[VA], encoded_buf_end_pos[VA],encoded_buf_size);
		}

		DSTATUS("[VENC] venc_vpu VPU_ENC_REG_FRAME_BUFFER ok! \n" );
	}
	else if( iOpCode == VENC_SEQ_HEADER )
	{
		venc_seq_header_t* p_seq_param = (venc_seq_header_t*)pParam1;
		enc_header_t enc_header;
		unsigned char* p_dest = NULL;
		int i_dest_size = 0;

#ifdef HAVE_ANDROID_OS
		if(gspSeqHeader == NULL)
		{
			gsiSeqHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			
			if(gsiSeqHeaderSize == 0)
			{
				gsiSeqHeaderSize = p_seq_param->m_iSeqHeaderBufferSize = 100*1024;
			}

			gspSeqHeader_phyAddr = (codec_addr_t)cdk_sys_malloc_physical_addr(gsiSeqHeaderSize);
			if( gspSeqHeader_phyAddr == 0 ) 
			{
				LOGE( "[VENC] gspSeqHeader_phyAddr malloc() failed \n");
				return -1;
			}
			DSTATUS( "[VENC] gspSeqHeader_phyAddr = 0x%x, 0x%x \n", (codec_addr_t)gspSeqHeader_phyAddr, gsiSeqHeaderSize );
			gspSeqHeader = (codec_addr_t)cdk_sys_malloc_virtual_addr( &gsiSeqHeaderBufDev, gspSeqHeader_phyAddr, gsiSeqHeaderSize );
			if( gspSeqHeader == 0 ) 
			{
				LOGE( "[VENC] gspSeqHeader_VA malloc() failed \n");
				return -1;
			}
			DSTATUS( "[VENC] gspSeqHeader_VA = 0x%x, 0x%x \n", (codec_addr_t)gspSeqHeader, gsiSeqHeaderSize );
			
			memset( gspSeqHeader, 0, gsiSeqHeaderSize );
		}
		
		if(p_seq_param->m_SeqHeaderBuffer[PA] == NULL)
		{
			DSTATUS( "[VENC] gspSeqHeader_Buffer = 0x%x, 0x%x \n", (codec_addr_t)gsBitstreamBufAddr[PA], gsBitstreamBufSize );
			p_seq_param->m_SeqHeaderBuffer[PA]	=	gsBitstreamBufAddr[PA];
			p_seq_param->m_SeqHeaderBuffer[VA]  =   gsBitstreamBufAddr[VA];
			p_seq_param->m_iSeqHeaderBufferSize =	gsBitstreamBufSize;
		}
#else
		if( gspSeqHeader == NULL )
		{
			gspSeqHeader = (unsigned char*)TCC_calloc( p_seq_param->m_iSeqHeaderBufferSize );
			if( gspSeqHeader == NULL ) 
			{
				LOGE( "[VENC:Err:%d] gspSeqHeader malloc failed \n", CDK_ERR_MALLOC );
				return CDK_ERR_MALLOC;
			}
			memset( gspSeqHeader, 0, p_seq_param->m_iSeqHeaderBufferSize );
		}
#endif

		if( gsVpuEncInit.m_iBitstreamFormat == STD_MPEG4 )
		{
			p_dest = gspSeqHeader;
	#if 0 // Not Used!!			
			enc_header.m_iHeaderType = MPEG4_VOS_HEADER;
			enc_header.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			enc_header.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			ret = TCC_VPU_ENC( VPU_ENC_PUT_HEADER, &gsVpuEncHandle, &enc_header, 0 );
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VOS_HEADER failed \n", ret );
				return -ret;
			}
			memcpy( p_dest, (void*)p_seq_param->m_SeqHeaderBuffer[VA], enc_header.m_iHeaderSize );
			p_dest += enc_header.m_iHeaderSize;
			i_dest_size += enc_header.m_iHeaderSize;

			enc_header.m_iHeaderType = MPEG4_VIS_HEADER;
			enc_header.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			enc_header.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			ret = TCC_VPU_ENC( VPU_ENC_PUT_HEADER, &gsVpuEncHandle, &enc_header, 0 );
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VIS_HEADER failed \n", ret );
				return -ret;
			}
			memcpy( p_dest, (void*)p_seq_param->m_SeqHeaderBuffer[VA], enc_header.m_iHeaderSize );
			p_dest += enc_header.m_iHeaderSize;
			i_dest_size += enc_header.m_iHeaderSize;
	#endif
			enc_header.m_iHeaderType = MPEG4_VOL_HEADER;
			enc_header.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
			enc_header.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
			
			DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for MPEG4_VOL_HEADER \n");
			ret = TCC_VPU_ENC( VPU_ENC_PUT_HEADER, &gsVpuEncHandle, &enc_header, 0 );
			if( ret != RETCODE_SUCCESS )
			{
				LOGE( "[VENC:Err:%d] venc_vpu MPEG4_VOL_HEADER failed \n", ret );
				return -ret;
			}
			memcpy( p_dest, (void*)p_seq_param->m_SeqHeaderBuffer[VA], enc_header.m_iHeaderSize );
			i_dest_size += enc_header.m_iHeaderSize;
		}
		else if( gsVpuEncInit.m_iBitstreamFormat == STD_AVC )
		{
	#ifdef HAVE_ANDROID_OS
			if(gsiSeqHeaderCnt == 0)
	#endif
			{		
				p_dest = gspSeqHeader;
				enc_header.m_iHeaderType = AVC_SPS_RBSP;
				enc_header.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
				enc_header.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;

#ifdef MULTI_SLICES_AVC
				if( enc_avc_aud_enable == 1) {
					memcpy( p_seq_param->m_SeqHeaderBuffer[VA], avcAudData, 8 ); //H.264 AUD
					enc_header.m_HeaderAddr += 8;
				}
#endif

				DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_SPS_RBSP \n");
				ret = TCC_VPU_ENC( VPU_ENC_PUT_HEADER, &gsVpuEncHandle, &enc_header, 0 );
				if( ret != RETCODE_SUCCESS )
				{
					LOGE( "[VENC:Err:%d] venc_vpu AVC_SPS_RBSP failed \n", ret );
					return -ret;
				}

				// modified by shmin for M2TS
				if(bAvcUsedNALStart)
				{
#ifdef MULTI_SLICES_AVC
					if( enc_avc_aud_enable == 1) {
						enc_header.m_iHeaderSize += 8;
					}
#endif				
					memcpy( p_dest, (void*)p_seq_param->m_SeqHeaderBuffer[VA], enc_header.m_iHeaderSize );	
				}
				else
				{
					unsigned char* buffer;

					buffer = (unsigned char*)p_seq_param->m_SeqHeaderBuffer[VA];
					enc_header.m_iHeaderSize -= 4;
					memcpy( p_dest, (void*)buffer+4, enc_header.m_iHeaderSize);	
				}

		#if 0
				{
					int temp = 0;
					DSTATUS("[SPS:0x");
					for( temp = 0; temp < enc_header.m_iHeaderSize; temp++ )
					{
						DSTATUS("%02X ", p_dest[temp] );
					}
					DSTATUS("\n");
				}
		#endif	
		
				p_dest += enc_header.m_iHeaderSize;
				i_dest_size += enc_header.m_iHeaderSize;
				gsiSzSeqHeader[gsiSeqHeaderCnt] = i_dest_size;
			}
	#ifdef HAVE_ANDROID_OS
			else
	#endif
			{
				p_dest = gspSeqHeader;
				p_dest += gsiSzSeqHeader[gsiSeqHeaderCnt-1];
				enc_header.m_iHeaderType = AVC_PPS_RBSP;
				enc_header.m_HeaderAddr  = p_seq_param->m_SeqHeaderBuffer[PA];
				enc_header.m_iHeaderSize = p_seq_param->m_iSeqHeaderBufferSize;
				
				DSTATUS( "[VENC] VPU_ENC_PUT_HEADER for AVC_PPS_RBSP \n");
				ret = TCC_VPU_ENC( VPU_ENC_PUT_HEADER, &gsVpuEncHandle, &enc_header, 0 );
				if( ret != RETCODE_SUCCESS )
				{
					LOGE( "[VENC:Err:%d] venc_vpu AVC_SPS_RBSP failed \n", ret );
					return -ret;
				}

				// modified by shmin for M2TS
				if(bAvcUsedNALStart)
				{
					memcpy( p_dest, (void*)p_seq_param->m_SeqHeaderBuffer[VA], enc_header.m_iHeaderSize );	
				}
				else
				{
					unsigned char* buffer;

					buffer = (unsigned char*)p_seq_param->m_SeqHeaderBuffer[VA];
					enc_header.m_iHeaderSize -= 4;
					memcpy( p_dest, (void*)buffer+4, enc_header.m_iHeaderSize);	
				}
					
				#if 0
				{
					int temp = 0;
					DSTATUS("[PPS:0x");
					for( temp = 0; temp < enc_header.m_iHeaderSize; temp++ )
					{
						DSTATUS("%02X ", p_dest[temp] );
					}
					DSTATUS("\n");
				}
				#endif
				i_dest_size += enc_header.m_iHeaderSize;
			}
		}

		// output
#ifdef HAVE_ANDROID_OS		
		if(gsiSeqHeaderCnt == 0)
			p_seq_param->m_pSeqHeaderOut	 = gspSeqHeader;
		else
			p_seq_param->m_pSeqHeaderOut	 = gspSeqHeader + gsiSzSeqHeader[gsiSeqHeaderCnt-1];
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

		//! Start encoding a frame.
		//Input Buffer Setting
#ifdef ENABLE_RATE_CONTROL		
		gsVpuEncInput.m_iChangeRcParamFlag  =  p_input_param->m_iChangeRcParamFlag;
		gsVpuEncInput.m_iChangeTargetKbps   =  p_input_param->m_iChangeTargetKbps;
		gsVpuEncInput.m_iChangeFrameRate    =  p_input_param->m_iChangeFrameRate;
#endif

#ifdef HAVE_ANDROID_OS
		if(p_input_param->request_IntraFrame == 1)
			gsVpuEncInput.m_iForceIPicture = 1;//set 1 For IDR-Type I-Frame without P-Frame!!
		else
			gsVpuEncInput.m_iForceIPicture = 0;
#else
		gsVpuEncInput.m_iForceIPicture = 0;
#endif
		
		gsVpuEncInput.m_iSkipPicture = 0;
		if( gsVpuEncInit.m_iTargetKbps == 0 ) // no rate control
		{
			gsVpuEncInput.m_iQuantParam = 23;
		}

#ifdef HAVE_ANDROID_OS
		if((encoded_buf_cur_pos[PA] + gsBitstreamBufSize) > encoded_buf_end_pos[PA])
		{
			encoded_buf_cur_pos[PA] = encoded_buf_base_pos[PA];
			encoded_buf_cur_pos[VA] = encoded_buf_base_pos[VA];
		}
		gsVpuEncInput.m_BitstreamBufferAddr =  (codec_addr_t)encoded_buf_cur_pos[PA];
		gsVpuEncInput.m_iBitstreamBufferSize = gsBitstreamBufSize;
#else
		gsVpuEncInput.m_BitstreamBufferAddr  = p_input_param->m_BitstreamBufferPA;
		gsVpuEncInput.m_iBitstreamBufferSize = p_input_param->m_iBitstreamBufferSize;
#endif
		gsVpuEncInput.m_PicYAddr = p_input_param->m_pInputY;
		if( gsVpuEncInit.m_bCbCrInterleaveMode == 0 )
		{
			gsVpuEncInput.m_PicCbAddr = p_input_param->m_pInputCbCr[0];
			gsVpuEncInput.m_PicCrAddr = p_input_param->m_pInputCbCr[1];
		}
		else
		{
			//FIXME
		}

#ifdef MULTI_SLICES_AVC	
		//H.264 AUD RBSP
		if( enc_avc_aud_enable == 1 && gsiFrameIdx > 0 ) {
			memcpy( encoded_buf_cur_pos[VA], avcAudData, 8 );
			gsVpuEncInput.m_BitstreamBufferAddr += 8;
			gsVpuEncInput.m_iBitstreamBufferSize -= 8;
		}

		// Slice information buffer setting
		if( gsVpuEncInit.m_iSliceMode == 1 ) {
			gsVpuEncInput.m_iReportSliceInfoEnable = 1;
			gsVpuEncInput.m_SliceInfoAddr[PA] = enc_slice_info_addr[PA];
			gsVpuEncInput.m_SliceInfoAddr[VA] = enc_slice_info_addr[VA];
		}
		else {
			gsVpuEncInput.m_iReportSliceInfoEnable = 0;
			gsVpuEncInput.m_SliceInfoAddr[PA] = 0;
			gsVpuEncInput.m_SliceInfoAddr[VA] = 0;
		}		
#endif

#ifdef  VIDEO_ENC_PROFILE
		//(HwGPIOD->GPEN |= Hw20);	(HwGPIOD->GPDAT |= Hw20); //CAM_26pin, GPIO_D20
		//*((volatile unsigned int *)(pTestRegBase + 0xEC)) &= (unsigned int)(~0x000F0000); //0xF0102080
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) |= (unsigned int)(GPIO_NUM);
#endif

		ret = TCC_VPU_ENC( VPU_ENC_ENCODE, &gsVpuEncHandle, &gsVpuEncInput, &gsVpuEncOutput ); 
		if( ret != RETCODE_SUCCESS )
		{
			if( ret == RETCODE_WRAP_AROUND )
			{
				LOGE( "[VENC] Warning!! BitStream buffer wrap arounded. prepare more large buffer = %d \n", ret );
			}

			if(ret == RETCODE_CODEC_EXIT)
			{
				//vpu h/w reset in case of lockup!!
				vpu_hw_reset();
			}

			LOGE( "[VENC:Err:%d] %d'th VPU_ENC_ENCODE failed \n", ret, gPFrameCnt );
			return -ret;
		}
#ifdef  VIDEO_ENC_PROFILE
		//(HwGPIOD->GPEN |= Hw20);	(HwGPIOD->GPDAT |= Hw20);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET + 0x4)) |= (unsigned int)(GPIO_NUM);
		*((volatile unsigned int *)(pTestRegBase + GPIO_BASE_OFFSET)) &= (unsigned int)(~GPIO_NUM);
#endif

#ifdef MULTI_SLICES_AVC	
		if( gsVpuEncInit.m_iSliceMode == 1 ) 
		{
			if( enc_avc_aud_enable == 1 && gsiFrameIdx > 0 ) 
			{
				gsVpuEncOutput.m_BitstreamOut[VA] -= 8;
				gsVpuEncOutput.m_BitstreamOut[PA] -= 8;
				gsVpuEncOutput.m_iBitstreamOutSize += 8;
			}			
		}
#endif

#ifdef HAVE_ANDROID_OS
		// output
		if( gsVpuEncInit.m_iBitstreamFormat == STD_AVC)
		{
			// modified by shmin for M2TS
			if(bAvcUsedNALStart)
			{
				p_output_param->m_pBitstreamOut 	= (codec_addr_t)encoded_buf_cur_pos[VA];
				p_output_param->m_iBitstreamOutSize = gsVpuEncOutput.m_iBitstreamOutSize;
			}
			else
			{
				p_output_param->m_pBitstreamOut 	= (codec_addr_t)encoded_buf_cur_pos[VA];
				p_output_param->m_pBitstreamOut 	+= 4;
				p_output_param->m_iBitstreamOutSize = gsVpuEncOutput.m_iBitstreamOutSize - 4;
			}
		}
		else
		{
			p_output_param->m_pBitstreamOut 	= (codec_addr_t)encoded_buf_cur_pos[VA];
			p_output_param->m_iBitstreamOutSize = gsVpuEncOutput.m_iBitstreamOutSize;
		}

#ifdef MULTI_SLICES_AVC 
		if( gsVpuEncInit.m_iSliceMode == 1 )
		{
			unsigned char *p = p_output_param->m_pBitstreamOut;
			unsigned int *pSliceSize;
			unsigned int extra_size = gsVpuEncOutput.m_iSliceInfoNum * sizeof(unsigned int);
			
			DSTATUS("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
					p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);

			pSliceSize = (unsigned int*)(p_output_param->m_pBitstreamOut + p_output_param->m_iBitstreamOutSize + STABILITY_GAP);
			pSliceSize = ALIGNED_BUFF(pSliceSize, 256);
			p_output_param->m_pSliceInfo = (unsigned int*)pSliceSize;
			p_output_param->m_iSliceCount = 0;

			if( gsVpuEncInit.m_iSliceMode == 1 ) 
			{
				p_output_param->m_iSliceCount = gsVpuEncOutput.m_iSliceInfoNum;
				if( gsVpuEncOutput.m_iSliceInfoNum > 1 && gsVpuEncOutput.m_SliceInfoAddr != NULL) 
				{
					int i;
					int iSize;
					unsigned int * pSliceParaBuf;
					unsigned char *pS;
					unsigned int total_bytes = 0;
			
					pS = (unsigned char*)encoded_buf_cur_pos[VA];
					
					iSize = gsVpuEncOutput.m_iSliceInfoSize;
					
					DSTATUS("[EncSliceNum:%3d], Addr - 0x%x", gsVpuEncOutput.m_iSliceInfoNum, gsVpuEncOutput.m_SliceInfoAddr);
			
					pSliceParaBuf = (unsigned int *)gsVpuEncOutput.m_SliceInfoAddr;
			
					for( i=0 ; i<iSize/8 ; i++, pSliceParaBuf += 2 ) 
					{
						int nMbAddr, nSliceBits;
						
						nMbAddr = pSliceParaBuf[1]&0x00FFFF;
						nSliceBits = pSliceParaBuf[0];
						pSliceSize[i] = nSliceBits / 8;

						if( enc_avc_aud_enable == 1  && gsiFrameIdx > 0 && i == 0){
							pSliceSize[i] += 8;
						}
						
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
					pSliceSize[0] = gsVpuEncOutput.m_iBitstreamOutSize;
				}
			}

			encoded_buf_cur_pos[PA] += ALIGNED_BUFF(gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, 4*1024);
			encoded_buf_cur_pos[VA] += ALIGNED_BUFF(gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2) + extra_size, 4*1024);
		}	
		else
#endif
		{
			encoded_buf_cur_pos[PA] += ALIGNED_BUFF(gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), 4*1024);
			encoded_buf_cur_pos[VA] += ALIGNED_BUFF(gsVpuEncOutput.m_iBitstreamOutSize + (STABILITY_GAP*2), 4*1024);
		}
				
		p_output_param->m_iPicType = gsVpuEncOutput.m_iPicType;
#else
		// 인코딩 결과 write 하는 부분
		if( gsiPictureDataSize < gsVpuEncOutput.m_iBitstreamOutSize )
		{
			void* new_ptr;
			gsiPictureDataSize = gsVpuEncOutput.m_iBitstreamOutSize;
			new_ptr = cdk_realloc( gspPictureData, gsiPictureDataSize );
			if( new_ptr == NULL ) 
			{
				LOGE( "[VENC:Err:%d] gspPictureData realloc failed \n", CDK_ERROR_REALLOC );
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

		if( gsVpuEncOutput.m_iPicType == PIC_TYPE_I )
		{		
#ifdef HAVE_ANDROID_OS
			keyInterval_cnt = 1;
#endif
			if(gMaxOutputSize_IFrame < gsVpuEncOutput.m_iBitstreamOutSize)
				gMaxOutputSize_IFrame = gsVpuEncOutput.m_iBitstreamOutSize;
			
			LOGI("I = %d/%d, P=%d!",  gsVpuEncOutput.m_iBitstreamOutSize, gMaxOutputSize_IFrame, gMaxOutputSize_PFrame);
			DSTATUS( "[I:%4d/%4d] Byte:%7d(%5.1lfK) ", gIFrameCnt, gsiFrameIdx, gsVpuEncOutput.m_iBitstreamOutSize, gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
			gIFrameCnt++;
		}
		else if( gsVpuEncOutput.m_iPicType == PIC_TYPE_P )
		{
#ifdef HAVE_ANDROID_OS
			keyInterval_cnt++;
#endif
			if(gMaxOutputSize_PFrame < gsVpuEncOutput.m_iBitstreamOutSize)
				gMaxOutputSize_PFrame = gsVpuEncOutput.m_iBitstreamOutSize;
			
			DSTATUS( "[P:%4d/%4d] Byte:%7d(%5.1lfK) ", gPFrameCnt, gsiFrameIdx, gsVpuEncOutput.m_iBitstreamOutSize, gsVpuEncOutput.m_iBitstreamOutSize/1024.0 );
			gPFrameCnt++;
		}
		gsiFrameIdx++;
		
		#if 0
		DSTATUS( "0x%x ", gsVpuEncOutput.m_BitstreamOut[VA] );
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
		ret = TCC_VPU_ENC( VPU_ENC_CLOSE, &gsVpuEncHandle, &gsVpuEncInput, &gsVpuEncOutput );
		if( ret != RETCODE_SUCCESS )
		{
			DPRINTF( "[VENC] VPU_ENC_CLOSE failed Error code is 0x%x \n", ret );
			ret = -ret;
		}
	
		if( gsBitWorkBufAddr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)gsBitWorkBufAddr[PA], gsBitWorkBufSize );
			gsBitWorkBufAddr[PA] = 0;
		}
		if( gsBitWorkBufAddr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsBitWorkBufDev, (void*)gsBitWorkBufAddr[VA], gsBitWorkBufSize );
			gsBitWorkBufAddr[VA] = 0;
		}
		if( gsMESearchBufAddr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)gsMESearchBufAddr[PA], gsMESearchBufSize );
			gsMESearchBufAddr[PA] = 0;
		}
		if( gsMESearchBufAddr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsMESearchBufDev, (void*)gsMESearchBufAddr[VA], gsMESearchBufSize );
			gsMESearchBufAddr[VA] = 0;
		}

		if( gsFrameBufAddr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)gsFrameBufAddr[PA], gsFrameBufSize );
			gsFrameBufAddr[PA] = 0;
		}
		if( gsFrameBufAddr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsFrameBufDev, (void*)gsFrameBufAddr[VA], gsFrameBufSize );
			gsFrameBufAddr[VA] = 0;
		}

#ifdef MULTI_SLICES_AVC	
		if( enc_slice_info_addr[PA] )
		{
			cdk_sys_free_physical_addr( (void*)enc_slice_info_addr[PA], enc_slice_info_size );
			enc_slice_info_addr[PA] = 0;
		}

		if( enc_slice_info_addr[VA] )
		{
			cdk_sys_free_virtual_addr( &gsFrameBufDev, (void*)enc_slice_info_addr[VA], enc_slice_info_size );
			enc_slice_info_addr[VA] = 0;
		}
#endif

#ifdef HAVE_ANDROID_OS
		if( gspSeqHeader_phyAddr )
		{
			cdk_sys_free_physical_addr( (void*)gspSeqHeader_phyAddr, gsiSeqHeaderSize);
			gspSeqHeader_phyAddr = 0;
		}
		
		if( gspSeqHeader )
		{
			cdk_sys_free_virtual_addr( &gsiSeqHeaderBufDev, (void*)gspSeqHeader, gsiSeqHeaderSize);
			gspSeqHeader = 0;
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
	pBaseAddr = (unsigned char*)encoded_buf_base_pos[VA];

	szAddrGap = convert_addr - pBaseAddr;

	return (pTargetBaseAddr+szAddrGap);
}
