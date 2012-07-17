/**
  @file encoder.h
  
  This component implements All Video encoder. (MPEG4/AVC/H.263)

  Copyright (C) 2009-2011 Telechips Inc.

  Date: 2011/02/25 13:33:29
  Author $Author: B070371 (ZzaU)
*/

#ifndef _ENCODER_TCC_
#define _ENCODER_TCC_

#include <stdlib.h>

/***********************************************************/
//COMMON PARAMETERS
#ifndef COMMON_CODEC_PARAM
#define COMMON_CODEC_PARAM
typedef enum CODEC_FORMAT {
	CODEC_FORMAT_H263 = 0,
	CODEC_FORMAT_MPEG4,
	CODEC_FORMAT_H264,	
	CODEC_FORMAT_RV,     	// Decoder only
	CODEC_FORMAT_MPEG2,     // Decoder only 
	CODEC_FORMAT_DIV3,      // Decoder only
	CODEC_FORMAT_VC1,       // Decoder only
	CODEC_FORMAT_MJPG,      // Decoder only
	CODEC_FORMAT_ILLEGAL
} tCODEC_FORMAT;

typedef enum pic_type {
	TYPE_I_FRAME	= 0,
	TYPE_P_FRAME,	
	TYPE_B_FRAME,	
	TYPE_ILLEGAL	
} tPIC_TYPE;
#endif


/***********************************************************/
//ENCODER PARAMETERS
typedef enum enc_slice_mode {
	ENC_SLICE_MODE_SINGLE = 0,
	ENC_SLICE_MODE_MULTY
} tENC_SLICE_MODE;

typedef enum enc_slice_size_mode {
	ENC_SLICE_SIZE_MODE_BY_BYTE = 0,		/* bytes : length per one slice */
	ENC_SLICE_SIZE_MODE_BY_MB				/* Macro block counts per one slice. one Macro block is 16x16.*/
} tENC_SLICE_SIZE_MODE;

/*
SliceMode :

In case sliceSizeModeSize is 0 (ENC_SLICE_SIZE_MODE_BY_BYTE).
	If this vaue is set 4KB, one slice length has about 4KB.
	But, slice count is changed per encoded frame.

 
In case sliceSizeModeSize is 0 (ENC_SLICE_SIZE_MODE_BY_MB).
	640x480 resolution has 1200 MBs. (640/16 * 480/16)
	If this value is set 400, one encoded frame has 3 slices. 
	But, Each slice has different length.
*/

typedef struct enc_init_params {
	tCODEC_FORMAT			codecFormat;		/* Encoding output stream format */
	int						picWidth;			/* Frame width, by pixels */
	int						picHeight;			/* Frame height, by pixels */
	int						frameRate;			/* FrameRate, It is used to calculate exact bitrate */
	int						targetKbps;			/* Bitrate for output stream, by Kbps, if zero, no bitrate control used */
	int						keyFrameInterval;	/* Key frame(I-Frame) interval */
	tENC_SLICE_MODE			sliceMode;			/* Slice mode, only can be used in case of AVC(H.264) */
	tENC_SLICE_SIZE_MODE	sliceSizeMode;		/* Slice size mode */
	int 					sliceSize;			/* Slice size */
	int 					use_NalStartCode;	/* Whether NAL Start Code (0x00000001) use or not, In case of only H.264,  */
} tENC_INIT_PARAMS;


typedef struct enc_frame_input {
	unsigned char	*inputStreamAddr;	/* Memory pointer that has the physical address of input raw data (memory pointer received from camera callback) */
	unsigned int	noIncludePhyAddr;   /* Whether inputStreamAddr is virtual address including raw data by itself.*/
	                                    /* if this set into 1, raw data will be copied into physical memory region to encode using H/W block. It will be decreased performance.*/
	unsigned int	inputStreamSize;	/* Bytes : length of input data */
	int				nTimeStamp;			/* TimeStamp of input data, by ms */	
	int				isForceIFrame;		/* Whether force IDC frame is needed */
	int				isSkipFrame;		/* Whether skip (do not encode) current frame */
	int				frameRate;			/* to change FrameRate during encoding, It is ignored if zero  */
	int				targetKbps;			/* to change Bitrate for output stream during encoding, by Kbps, It is ignored if zero */	
	int				Qp;					/* TBD, Qp for current frame */
} tENC_FRAME_INPUT;


/*
 Generally in case of AVC(H.264), encoded output structure is sa below:
	====================================================
	| SPS | PPS | I-Frame | P-Frame | P-Frame | P-Frame | ...
	====================================================

 But, If you use multi-slice mode, encoded output structure is as below:
    - AU(Access Unit) data is { 0x00,0x00,0x00,0x01,0x09,0x50,0x00,0x00 }. 
       You can confirm starting point of frame by AU.
 
 * Whole structure
	====================================================
	| AU | SPS | PPS | I-Frame | AU | P-Frame | AU | P-Frame | ...
	====================================================

 * I/P-Frame structure
	===========================================
	| AU | 1st slice | 2nd slice | 3rd slice ..... | n'th slice |
	===========================================
*/

typedef struct enc_frame_output {
	unsigned char	*outputStreamAddr;	/* Base address for output bitstream (virtual address)*/
	tPIC_TYPE		picType;			/* Picture coding type */
	int				nTimeStamp;			/* TimeStamp of output bitstream, by ms */		
	int				headerLen;			/* Bytes of header */
	int				frameLen;			/* Bytes of frame encoded */
	int 			m_iSliceCount;      /* total slice's count that one encoded frame have */
	unsigned int* 	m_pSliceInfo;		/* bytes : This is the array pointer that has length of each slice. */
} tENC_FRAME_OUTPUT;


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
extern int ENCODER_INIT
(
	tENC_INIT_PARAMS *pInit
);

/*!
 ***********************************************************************
 * \brief
 *		ENCODER_CLOSE	: close function of video encoder
 ***********************************************************************
 */
extern int ENCODER_CLOSE(void);

/*!
 ***********************************************************************
 * \brief
 *		ENCODER_ENC	: encode function of video encoder
 * \param
 *		[in] pInput			: pointer of encoder frame input parameters  
 * \param
 *		[out] pOutput			: pointer of encoder frame output parameters  
 * \param
 *		[out] pResult			: pointer of encoder result patameters 
 * \return
 *		If successful, ENCODER_ENC returns 0 or plus. Otherwise, it returns a minus value.
 ***********************************************************************
 */
extern int ENCODER_ENC
(
	tENC_FRAME_INPUT *pInput,
	tENC_FRAME_OUTPUT *pOutput
);

#define TEMP_FOR_LINPHONE_VENC
#ifdef TEMP_FOR_LINPHONE_VENC
extern void Display_Stream(unsigned char *p, int size);
extern int TCC_VENC_Init_H264(unsigned int *pInitParam);
extern int TCC_VENC_Enc(unsigned int *pInputStream, unsigned int *pOutstream);																			
extern int TCC_VENC_Close(void);
#endif
#endif //_ENCODER_TCC_
