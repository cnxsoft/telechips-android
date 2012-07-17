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
 *		cdmx.h
 * \date
 *		2009/06/01
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		container demuxer
 *
 ***********************************************************************
 */
#ifndef _CDMX_H_
#define _CDMX_H_

#ifdef HAVE_ANDROID_OS
#include "cdk_pre_define.h"
#include "common.h"
#else
#include "../cdk/cdk_pre_define.h"
#include "../cdk/cdk_port.h"
#ifdef INCLUDE_MP4_DMX
	#include "mp4/mp4_demuxer.h"
#endif
#ifdef INCLUDE_MKV_DMX
	#include "mkv/mkv_demuxer.h"
#endif

#ifdef INCLUDE_MPG_DMX
	#include "mpg/mpg_demuxer.h"
#endif

#ifdef INCLUDE_AVI_DMX
	#include "avi/avi_demuxer.h"
#endif

#ifdef INCLUDE_TS_DMX
	#include "ts/TCC_TS_DEMUXER.h"
	#include "ts/TCC_TS_DEMUXER_EXT.h"
#endif

#ifdef INCLUDE_ASF_DMX
	#include "asf/asf_demuxer.h"
#endif
#ifdef INCLUDE_RM_DMX
	#include "rmff/rm_demuxer.h" //! RM Demuxer
#endif
#ifdef INCLUDE_OGG_DMX
	#include "ogg/OGG_demuxer.h" //! OGG Demuxer
#endif

#ifdef INCLUDE_AUDIO_DMX
	#include "audio/audio_demuxer.h" //! Audio only Demuxer
#endif

#ifdef INCLUDE_FLV_DMX
	#include "flv/flv_demuxer.h" //! FLV Demuxer
#endif
#endif

#define CDMX_PTS_MODE		0	//! Presentation Timestamp (Display order)
#define CDMX_DTS_MODE		1	//! Decode Timestamp (Decode order)

#define CDMX_INIT			0
#define CDMX_GET_INFO		1
#define CDMX_GET_SEQ_HEADER	2
#define CDMX_GETSTREAM		3
#define CDMX_CLOSE			4
#define CDMX_SEL_STREAM		5
#define CDMX_SEEK			6
#define CDMX_PRE_INIT       10

#define CDMX_SEEKMODE_RELATIVE_TIME		0
#define CDMX_SEEKMODE_ABSOLUTE_TIME		1

#define SKIMMING_MODE_DIRECTION_FORWARD		0x00000001
#define SKIMMING_MODE_DIRECTION_BACKWARD	0x00000002

#ifdef HAVE_ANDROID_OS
#define CDMX_MARKER_DEFAULT_PLAY_MODE_BIT       0x00000000 
#define CDMX_MARKER_SEQUENTIAL_PLAY_MODE_BIT    0x00000010 
#define CDMX_MARKER_PROGRESSIVE_PLAY_MODE_BIT   0x00000020 
#define CDMX_MARKER_MEDIA_SCAN_MODE_BIT  		0x00000080 

#define ENABLE_MEDIASCAN_OPCODE
#endif

typedef struct cdmx_select_stream_t
{
	long					lSelType;
	long					lVideoID;
	long					lAudioID;
	long					lSubtitleID;
	long					lGraphicsID;
} cdmx_select_stream_t;

typedef struct cdmx_seq_header_t
{
	unsigned char* m_pSeqHeaderData;	//!< Seqeunce header pointer
	int m_iSeqHeaderDataLen;			//!< Seqeunce header len (bytes)
} cdmx_seq_header_t;

typedef struct cdmx_input_t 
{
	unsigned char* m_pPacketBuff;		//!< [in] allocated packet(video or audio) buffer pointer
	int m_iPacketBuffSize;				//!< [in] allocated packet(video or audio) buffer size
	int m_iPacketType;					//!< [in] PACKET_NONE or PACKET_VIDEO or PACKET_AUDIO
	int m_iUsedBytes;					//!< used bytes
	int m_iInterestingCaptionIdx;		//!< caption index
	int m_iSkimmingModeFlags;			//!< infomation for skimming mode such as speed and direction...
	float m_iSkimmingPlaySpeed;			//!< infomation for skimming mode such as speed and direction...
	int m_iSkimmingLateTime;
#ifdef HAVE_ANDROID_OS
	unsigned char* m_pPacketBuff_ext1;	//!< [in] allocated packet buffer pointer for the designated stream type only
	unsigned char* m_pPacketBuff_ext2;	//!< [in] allocated packet buffer pointer for the designated stream type only
#else
	int	m_Reserved[8-6];
#endif
} cdmx_input_t;

typedef struct cdmx_seek_t 
{
	int m_iSeekTime;				//!< seek time in millisec.(could be below "0". That means backward seeking in Relative-Seek-Mode)
	unsigned int m_uiSeekMode;		//!< "0" : Relative time Seek Mode, "1" : Absolute time Seek Mode
} cdmx_seek_t;

typedef struct cdmx_info_t
{
	//! File information : 128 Bytes
	struct 
	{
		/* common */
		char* m_pszOpenFileName;	//!< open file name
		char* m_pszCopyright;		//!< copyright
		char* m_pszCreationTime;	//!< creation time
		int m_iRunningtime;			//!< runing time * 1000
		int64_t m_lFileSize;		//!< total file size

		/* AVI info */
		int m_bHasIndex;
		int m_iSuggestedBufferSize;
		int m_iTotalStreams;

		/* mp4 info */
		int m_iTimeScale;			//!< timescale of file
		//int m_iUserDataPos;			//!< user data position
		i64_t m_lUserDataPos;		//!< user data position
		int m_iUserDataLen;			//!< user data length

		/* asf info */
		int m_iSeekable;			//!< flag of file seekable
//		int m_iIScorruptedindex;	//!< 1 : corrupted index entry
		int m_Reserved[32-14];
	} m_sFileInfo;

	//! Video information : 128 Bytes
	struct 
	{
		/* common */
		int m_iWidth;				//!< width
		int m_iHeight;				//!< height
		int m_iFrameRate;			//!< framerate * 1000;
		int m_iFourCC;				//!< fourcc

		unsigned int m_ulmax_bit_rate;        //!< maximum bit rate of stream 
		unsigned int m_ulavg_bit_rate;        //!< average bit rate of stream

		/* extra info (common) */
		char* m_pszCodecName;		//!< codec name
		char* m_pszCodecVendorName;	//!< codec vendor
		unsigned char* m_pExtraData;//!< extra data
		int m_iExtraDataLen;		//!< extra data length

		/* AVI info */
		int m_iNumVideoStream;		//!< number of video stream
		int m_iCurrVideoIdx;		//!< current video stream index
		int m_iBitsPerSample;		//!< bits per sample

		/* mp4 info */
		int m_iTotalNumber;			//!< total frame number
		int m_iKeyFrameNumber;		//!< key frame number
		int m_bAvcC;				//!< avcC flag for H264
		int m_iTrackTimeScale;		//!< timescale of video
		int m_iLastKeyTime;			//!< time of last key frame
		int m_iRotateDegree;

		/* asf info */
		int m_iVideoStreamID;			//!< video stream ID
		int	m_iHasPayloadAspectRatio;	//!< payload extension pixel aspect ratio
		int m_iAspectRatioX;			//!< pixel aspect ratio X
		int m_iAspectRatioY;			//!< pixel aspect ratio Y
		int m_lAvgBitRate;				//!< average bit rate

		/*mpg info*/
		int m_iAspectRatio;
		int m_iBitRate;

		/* RM info */	
		unsigned int   m_ulLength;				//!< The length of the video
		unsigned int   m_ulMOFTag;				//!< The type of media contained inf the bitstream
		unsigned int   m_ulSubMOFTag;			//!< The video format FOURCC code
		unsigned short int m_usBitCount;		//!< bit count of the video
		unsigned short int   m_usPadWidth;		//!< padded width, in pixels, of a video frame
		unsigned short int   m_usPadHeight;		//!< padded height, in pixels, of a video frame
		unsigned short int   m_sdummy;			//!< dummy byte for 4byte align	
		unsigned int m_ulFramesPerSecond;		//!< Number of frames per second in the video
		unsigned int   m_ulOpaqueDataSize;		//!< OpaqueDataSize
		char*    m_pOpaqueData;					//!< Internal data to be passed on to the next component in the code
		int m_Reserved[32-31];
	} m_sVideoInfo;

	//! Audio information
	struct 
	{
		/* common */
		int m_iTotalNumber;						//!< total audio stream number
		int m_iSamplePerSec;					//!< samples/sec
		int m_iBitsPerSample;					//!< bits/sample
		int m_iChannels;						//!< channels
		int m_iFormatId;						//!< format id

		unsigned int m_ulmax_bit_rate;			//!< maximum bit rate of stream 
		unsigned int m_ulavg_bit_rate;			//!< average bit rate of stream

		/* extra info (common) */
		char* m_pszCodecName;					//!< codec name
		char* m_pszCodecVendorName;				//!< codec vendor
		unsigned char* m_pExtraData;			//!< extra data
		int m_iExtraDataLen;					//!< extra data length

		/* mp4 info */
		int m_iFramesPerSample;					//!< fps
		int m_iTrackTimeScale;					//!< timescale of audio
		int m_iSamplesPerPacket;				//!< samples / packet

		/* AVI info */
		int m_iNumAudioStream;					//!< number of audio stream
		int m_iCurrAudioIdx;					//!< current audio stream index
		int m_iBlockAlign;						//!< block align
		int m_iAvgBytesPerSec;					//!< average bytes per second

		/* ASF info */
//		ASF_AUDIOINFO pAudInfo[1];
		int m_iAudioStreamID;

		/* MPG info & Audio Only Parser */
		int m_iBitRate;

		/* RM info */
		int m_iActualRate;						//!< If the codec flavor supports it, this is the extended sample rate
		short m_sAudioQuality;					//!< This number is obsolete. It always contains a value of 100	
		short m_sFlavorIndex;					//!< Codec flavor used in this substream
		int m_iBitsPerFrame;					//!< Number of bits per audio frame
		int m_iGranularity;						//!< The total size of the block of compressed audio data
		int m_iOpaqueDataSize;					//!< Size, in bytes, of the codec-specific opaque data
		char*  m_pOpaqueData;					//!< Used to initialize the decoder

		int m_iSamplesPerFrame;					//!< Number of samples per audio frame
		int m_iframeSizeInBits;					//!< Sampling Frequency
		int m_iSampleRate;						//!< Sample rate of this substream
		short m_scplStart;						//!< Starting bin coded with coupling
		short m_scplQBits;						//!< allocated bit num for coupling
		short m_snRegions;						//!< number of region
		short m_sdummy;							//!< dummy byte for 4byte align

		/* TS info */
		unsigned long m_ulSubType;

		/* for Audio Only Path */
		int m_iMinDataSize;						//!< minimum data size
		
		//int m_Reserved[32-26];
	} m_sAudioInfo;

} cdmx_info_t;

#ifdef DIVX_DRM5//ndef DIVX_DRM
typedef struct av_dmx_divxdrm_info_t
{
	unsigned short	 m_usIsThisFrameEncrypted;	   // Specifies whether this video frame is encrypted or not. (1 or 0)
	unsigned short	 m_usFrameKeyIndex; 		   // Specifies the index of the frame key that is used to encrypt the video frame in the stream.
	unsigned int	 m_ulEncryptionOffset;		   // Specifies the offset from where the data of the video frame is encrypted.
	unsigned int	 m_ulEncryptionLength;		   // Specifies the length of the portion of the video frame that is encrypted
} av_dmx_divxdrm_info_t;
#endif

typedef struct cdmx_output_t
{
	unsigned char* m_pPacketData;			//!< pointer of output data
	int m_iPacketSize;						//!< length of output data
	int m_iPacketType;						//!< packet type of output data
	int m_iTimeStamp;						//!< timestamp(msec) of output data
	int m_bKeyFrame;						//!< key flag of output data
	int m_iKeyCount;						//!< current key count
	// INTERNAL_SUBTITLE_INCLUDE
	unsigned int	m_iEndTimeStamp ;
	unsigned int	m_iSubtitleType ;

	/* For Audio Only Path */
	int m_iEndOfFile;						//!< end of file
	int m_iDecodedSamples;					//!< Decoded samples
	unsigned int m_uiUseCodecSpecific;		//!< variable use in several codec
	int m_iSkimmingTimeStamp;				//!< DTS is varied since skimming mode is performed

	#ifdef DIVX_DRM5
	av_dmx_divxdrm_info_t	m_divxdrm_info;
	#endif

	int m_Reserved[32-12];					//!< reserved...
} cdmx_output_t;



/*============================================
	Common Demuxer API
============================================*/
//! File information
typedef struct av_dmx_file_info_t
{
	// common info
	char		   *m_pszFileName;
	long			m_lDuration;
	int64_t			m_llFileSize;
} av_dmx_file_info_t;

//! Video stream information
typedef struct av_dmx_video_info_t
{
	// common info
	long			 m_lStreamID;
	long			 m_lWidth;
	long			 m_lHeight;
	long			 m_lFrameRate;
	unsigned long	 m_ulFourCC;
	void			*m_pExtraData;
	long			 m_lExtraLength;
} av_dmx_video_info_t;

//! Audio stream information
typedef struct av_dmx_audio_info_t
{	
	// common info
	long			 m_lStreamID;
	long			 m_lSamplePerSec;
	long			 m_lBitsPerSample;
	long			 m_lChannels;
	long			 m_lFormatTag;
	void		    *m_pExtraData;
	long			 m_lExtraLength;
	char			*m_pszLanguage;
} av_dmx_audio_info_t;

//! Subtitle stream information
typedef struct av_dmx_subtitle_info_t
{	
	// common info
	long			 m_lStreamID;
	char			*m_pszLanguage;
} av_dmx_subtitle_info_t;

//! Graphics stream information
typedef struct av_dmx_graphics_info_t
{	
	// common info
	long			 m_lStreamID;
	char			*m_pszLanguage;
} av_dmx_graphics_info_t;

//! Demuxer Initialize Information
typedef struct av_dmx_info_t
{
	// file info
	av_dmx_file_info_t	   *m_pstFileInfo;				//!< pointer to file information

	// video stream info
	long				 	m_lVideoStreamTotal;		//!< the number of video stream
	av_dmx_video_info_t    *m_pstVideoInfoList;			//!< pointer to video information list ( m_lVideoStreamTotal x sizeof(m_pstVideoInfoList) )
	av_dmx_video_info_t    *m_pstDefaultVideoInfo;		//!< pointer to default video information

	// audio stream info
	long				    m_lAudioStreamTotal;		//!< the number of audio stream
	av_dmx_audio_info_t    *m_pstAudioInfoList;			//!< pointer to audio information list ( m_lAudioStreamTotal x sizeof(m_pstAudioInfoList) )
	av_dmx_audio_info_t    *m_pstDefaultAudioInfo;		//!< pointer to default audio information

	// text subtitle stream info
	long				    m_lSubtitleStreamTotal;		//!< the number of subtitle stream
	av_dmx_subtitle_info_t *m_pstSubtitleInfoList;		//!< pointer to subtitle information list ( m_lSubtitleStreamTotal x sizeof(m_pstSubtitleInfoList) )
	av_dmx_subtitle_info_t *m_pstDefaultSubtitleInfo;	//!< pointer to default audio information

	// graphics stream info
	long				    m_lGraphicsStreamTotal;		//!< the number of graphics stream
	av_dmx_graphics_info_t *m_pstGraphicsInfoList;		//!< pointer to graphics information list ( m_lGraphicsStreamTotal x sizeof(m_pstGraphicsInfoList) )
	av_dmx_graphics_info_t *m_pstDefaultGraphicsInfo;	//!< pointer to default audio information
} av_dmx_info_t;

#ifdef HAVE_ANDROID_OS
void GetStreamIDList(void* , void*, unsigned int* , int** , unsigned int* , int**, unsigned int *, int**);
#endif

int 
cdmx_mp4( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

#ifdef INCLUDE_MKV_DMX
int cdmx_mkv( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);
#endif

int 
cdmx_mpg( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_avi( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_ts( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_asf( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_rmff( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_audio( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

int 
cdmx_ogg( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);

#ifdef INCLUDE_FLV_DMX
int 
cdmx_flv( int iOpCode, int* pHandle, void* pParam1, void* pParam2 , void* pParam3);
#endif

#endif //_CDMX_H_
