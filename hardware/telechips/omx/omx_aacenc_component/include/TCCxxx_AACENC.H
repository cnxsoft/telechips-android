/****************************************************************************
 *   FileName    : TCC7xx_AACENC_API.H
 *   Description : MPEG 4 AAC audio encoder API implementation.
 ****************************************************************************
 *
 *   TCC Version 2.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
 typedef struct
 {
 
	 // A pointer to the buffer containing the persistent internal state of the
	 //tAACEncInstance *pAACEncInstance;
 
	 // The buffer to which we write encoded AAC data.
//	 BufferState *pOutput;
 
	 // The sample rate of the decoded PCM stream.
	 unsigned short usSampleRate;
 
	 // The number of channels in the file.
	 unsigned char ucChannels;
 
	 // The bit rate of the AAC file.
	 unsigned long ulBitRate;
 
	 // The number of samples which have been encoded. //decoded.
	 unsigned long ulTimePos;
 
 } tAACENC;

 
typedef enum OpFormat
{
	FORMAT_RAW = 0,
	FORMAT_ADIF = 1,
	FORMAT_ADTS = 2
}tOutputFormat;

typedef  struct Mp4AacEncParams tMp4AacEncParams;
struct Mp4AacEncParams
{
	unsigned int NumberOfChannels;
	unsigned int SamplingFrequency;
	unsigned int TurnOnPns;
	unsigned int TurnOnTns;
	unsigned int VersionInfo;
	tOutputFormat OutputFormat;
	unsigned int OutputBitRate;
};
 
/* Data type definitions */
typedef signed char          tInt8;
typedef unsigned char        tUint8;
typedef short int            tInt16;
typedef unsigned short int   tUint16;
typedef int                  tInt32;
typedef unsigned int         tUint32;

typedef float                tFlt32;
typedef double               tFlt64;

typedef unsigned char        tBool;
typedef signed int           tError;


#define E_TRUE               1
#define E_FALSE              0

#define E_ON                 1
#define E_OFF                0

/* Common error code definitions */
#define E_SUCCESS            0
#define E_FAILURE           (-1)
#define E_OUT_OF_MEMORY     (-2)
#define E_INVALID_ARGS      (-3)
#define E_NOT_SUPPORTED     (-4)



/*
 *        AACEnc_Create
 *        ===========================
 *
 * Description
 * -----------
 *   Set instance buffer pointer and create global structures.
 *
 * Arguments
 * ---------
 *   aHandle		Pointer to pointer to AAC stream structure.
 *   p_instance	Pointer to instance buffer for AAC Encoder library.
 *
 * Return value
 * ---------
 *   E_SUCCESS or E_FAILURE 
 *
 */
extern int AACEnc_Create(void **aHandle, unsigned char *p_instance);

/*
 *        AACEnc_Init
 *        ===========================
 *
 * Description
 * -----------
 *   Create the AAC encoder using the specified custom settings for audio.
 *
 * Arguments
 * ---------
 *   pHandle		Pointer to AAC stream structure.
 *   ac			Pointer to user setting structure (tMp4AacEncParams).
 *
 * Return value
 * ---------
 *   E_SUCCESS or E_FAILURE 
 *
 */
extern int AACEnc_Init(void *pHandle,tMp4AacEncParams *ac);
 

/*
 *        AACEnc_EncodeFrame
 *        ===========================
 *
 * Description
 * -----------
 *   Encodes a chunk of samples.
 *
 * Arguments
 * ---------
 *   pHandle		Pointer to AAC stream structure.
 *   Buffer_L		Pointer to Sample data from left audio file
 *   Buffer_R		Pointer to Sample data from right audio file
 *   bitBuffer		Output data that should be written to the AAC file.  (output)
 *   bitBufSize 	Size of bitBuffer in bytes  (output)
 *
 * Return value
 * ---------
 *   E_SUCCESS or E_FAILURE 
 *
 */
extern int AACEnc_EncodeFrame(void *pHandle, short *Buffer_L, short *Buffer_R, unsigned char *bitBuffer, int *bitBufSize);

/*
 *        AACEnc_WriteADIFHeader
 *        ===========================
 *
 * Description
 * -----------
 *   Write ADIF Header to aac stream.
 *
 * Arguments
 * ---------
 *   pHandle		Pointer to AAC stream structure.
 *   bitBuffer		Output data that should be written to the AAC file. 
 *
 * Return value
 * ---------
 *   E_SUCCESS or E_FAILURE 
 *
 */
extern int AACEnc_WriteADIFHeader(void *pHandle, unsigned char *headerBuf);



/* end of file */

