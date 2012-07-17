/*!
 ***********************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2008-2009 Telechips Inc.
  Korad Bldg, 1000-12 Daechi-dong, Kangnam-Ku, Seoul, Korea					
 \endverbatim
 ***********************************************************************
 */
/*!
 ***********************************************************************
 *
 * \file
 *		common.h : This file is part of libcommon
 * \date
 *		2008/07/16
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		define common code
 *
 ***********************************************************************
 */
#ifndef _COMMON_COMMON_H_
#define _COMMON_COMMON_H_

#ifndef _COMMON_TYPES_H_
#define _COMMON_TYPES_H_
#if defined(_WIN32_WCE)					// ARM WinCE
	typedef unsigned __int64  ui64_t;
	typedef __int64 i64_t;
#else
	typedef unsigned long long ui64_t;
	typedef signed long long i64_t;
#endif

#endif


/************************************************************************

	Operation Code / Return value

************************************************************************/
// Op Code 
#define INIT		0				//!< initialize
#define CLOSE		1				//!< close
#define DECODE		2				//!< decode
#define SEEK		3				//!< seek -------------------------------------for demuxer
#define GET_STREAM	4				//!< get stream -------------------------------for demuxer
#define PUT_STREAM	4				//!< put stream -------------------------------for muxer
#define INIT_FROM_FILE_HANDLE	5	//!< initialize from file handle --for demuxer


// Return value
#define ERR_NONE				 0
#define ERR_END_OF_FILE			-1
#define ERR_END_OF_VIDEO_FILE	-2
#define ERR_END_OF_AUDIO_FILE	-3
#define ERR_END_OF_SUBTITLE_FILE -4


/************************************************************************

	Seek Options( Demuxer Only )

		- ex1) Seek forward 10 seconds (video-based)
				m_iSeekMode = (SEEK_DIR_FWD|SEEK_BASE_VIDEO);
				m_iSeekTimeMSec = 10000; // 10 seconds

		- ex2) Seek backward 10 seconds (video-based)
				m_iSeekMode = (SEEK_DIR_BWD|SEEK_BASE_VIDEO);
				m_iSeekTimeMSec = 10000; // 10 seconds

		- ex3) Seek to the next key (video-based)
				m_iSeekMode = (SEEK_DIR_FWD|SEEK_BASE_VIDEO|SEEK_OPT_KEY);
				m_iSeekTimeMSec // not used

		- ex4) Seek to time(seconds) (video-based)
				m_iSeekMode = (SEEK_DIR_FWD|SEEK_BASE_VIDEO|SEEK_OPT_TIME);
				m_iSeekTimeMSec = 23000 // 23 seconds from start


************************************************************************/

// Seek Mode 
#define SEEK_RESET				     0x00000000		//!< to start
//=======================================================================
// 1. Direction
#define SEEK_DIR_FWD		         0x00000001		//!< to forward
#define SEEK_DIR_BWD	             0x00000002		//!< to backward
//#define SEEK_DIR_XXXX	             0x00000004
//#define SEEK_DIR_XXXX	             0x00000008
//-----------------------------------------------------------------------
// 2. Packet type
#define SEEK_BASE_VIDEO              0x00000010		//!< seek based on video
#define SEEK_BASE_AUDIO              0x00000020		//!< seek based on audio
#define SEEK_VIDEO_ONLY				 0x00000040		//!< seek video
#define SEEK_AUDIO_ONLY			     0x00000080		//!< seek audio
//-----------------------------------------------------------------------
// 3. Options
#define SEEK_OPT_KEY	             0x00000100		//!< seek based on video key frame(to next key)
#define SEEK_OPT_TIME		         0x00000200		//!< seek based on time
//#define SEEK_						 0x00000400
//#define SEEK_			             0x00000800
//=======================================================================
// 4.examples
#define SEEK_MODE_DEFAULT	(SEEK_DIR_FWD|SEEK_BASE_VIDEO)					//!< current time + m_iSeekTimeMSec
#define SEEK_MODE_NEXT_KEY	(SEEK_DIR_FWD|SEEK_BASE_VIDEO|SEEK_OPT_KEY)		//!< next key frame
#define SEEK_MODE_TIME		(SEEK_BASE_VIDEO|SEEK_OPT_TIME)					//!< to m_iSeekTimeMSec

/************************************************************************

	Common defines

************************************************************************/
// packet(stream) type
#define AV_PACKET_NONE			0			//!< default packet
#define AV_PACKET_VIDEO		1			//!< video packet
#define AV_PACKET_AUDIO		2			//!< audio packet
#define AV_PACKET_SUBTITLE		4			//!< reserved(not used)
#define AV_TICKSPERSEC			1000		//!< ticks/sec

typedef int common_handle_t;
typedef int common_result_t;

//! common callback function : 128 bytes
typedef struct common_callback_func_t
{
	void* (*m_pMalloc		 ) ( unsigned int );					//!< malloc
	void* (*m_pNonCacheMalloc) ( unsigned int );					//!< non-cacheable malloc 
	void  (*m_pFree			 ) ( void* );							//!< free
	void  (*m_pNonCacheFree	 ) ( void* );							//!< non-cacheable free
	void* (*m_pMemcpy		 ) ( void*, const void*, unsigned int );//!< memcpy
	void* (*m_pMemset		 ) ( void*, int, unsigned int );		//!< memset
	void* (*m_pRealloc		 ) ( void*, unsigned int );				//!< realloc
	void* (*m_pMemmove		 ) ( void*, const void*, unsigned int );//!< memmove
	int m_Reserved1[16-8];

	void*		 (*m_pFopen	) ( const char *, const char * );						//!< fopen
	unsigned int (*m_pFread	) ( void*, unsigned int, unsigned int, void* );			//!< fread
	int			 (*m_pFseek	) ( void*, long, int );									//!< fseek
	long		 (*m_pFtell	) ( void* );											//!< ftell
	unsigned int (*m_pFwrite) ( const void*, unsigned int, unsigned int, void* );	//!< fwrite
	int			 (*m_pFclose) ( void* );											//!< fclose
	int			 (*m_pUnlink) ( const char* );										//!< _unlink
	unsigned int (*m_pFeof  ) ( void* );											//!< feof
	unsigned int (*m_pFflush) ( void* );											//!< fflush
	int			 (*m_pFseek64) ( void*, i64_t, int );								//!< fseek 64bit io
	i64_t		 (*m_pFtell64) ( void* );											//!< ftell 64bit io
	int m_Reserved2[16-11];
} common_callback_func_t;

//! common demuxer information : 384 bytes
typedef struct common_dmx_info_t
{
	//! File information : 128 Bytes
	struct 
	{
		/* common */
		char* m_pszOpenFileName;	//!< open file name
		char* m_pszCopyright;		//!< copyright
		char* m_pszCreationTime;	//!< creation time
		int m_iRunningtime;			//!< runing time * 1000
		i64_t m_lFileSize;			//!< total file size

		/* AVI info */
		int m_bHasIndex;
        int m_iSuggestedBufferSize;
		int m_iTotalStreams;

		/* mp4 info */
		int m_iTimeScale;			//!< timescale of file
		i64_t m_lUserDataPos;		//!< user data position
		int m_iUserDataLen;			//!< user data length

		int m_Reserved[32-13];
	} m_sFileInfo;

	//! Video information : 128 Bytes
	struct 
	{
		/* common */
		int m_iWidth;				//!< width
		int m_iHeight;				//!< height
		int m_iFrameRate;			//!< framerate * 1000;
		int m_iFourCC;				//!< fourcc

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

		int	m_iMaxBitrate;			//!< maximum bitrate
		int m_iAvgBitrate;			//!< average bitrate
		int m_Reserved[32-18];			
	} m_sVideoInfo;

	//! Audio information : 128 Bytes
	struct 
	{
		/* common */
		int m_iTotalNumber;			//!< total audio stream number
		int m_iSamplePerSec;		//!< samples/sec
		int m_iBitsPerSample;		//!< bits/sample
		int m_iChannels;			//!< channels
		int m_iFormatId;			//!< format id

		/* extra info (common) */
		char* m_pszCodecName;		//!< codec name
		char* m_pszCodecVendorName;	//!< codec vendor
		unsigned char* m_pExtraData;//!< extra data
		int m_iExtraDataLen;		//!< extra data length

		/* mp4 info */
		int m_iFramesPerSample;		//!< fps
		int m_iTrackTimeScale;		//!< timescale of audio
		int m_iSamplesPerPacket;	//!< samples / packet

		/* AVI info */
		int m_iNumAudioStream;		//!< number of audio stream
		int m_iCurrAudioIdx;		//!< current audio stream index

		int	m_iMaxBitrate;			//!< maximum bitrate
		int m_iAvgBitrate;			//!< average bitrate
		int m_Reserved[32-16];			
	} m_sAudioInfo;

} common_dmx_info_t;

//! Input parameter : 32 bytes
typedef struct common_dmx_input_t
{
	unsigned char* m_pPacketBuff;		//!< [in] allocated packet(video or audio) buffer pointer
	int m_iPacketBuffSize;				//!< [in] allocated packet(video or audio) buffer size
	int m_iPacketType;					//!< [in] PACKET_NONE or PACKET_VIDEO or PACKET_AUDIO
	int m_iUsedBytes;					//!< used bytes
	int	m_Reserved[8-4];
} common_dmx_input_t;

//! Output parameter : 128 bytes
typedef struct common_dmx_output_t
{
	unsigned char* m_pPacketData;	//!< pointer of output data
	int m_iPacketSize;				//!< length of output data
	int m_iPacketType;				//!< packet type of output data
	int m_iTimeStamp;				//!< timestamp(msec) of output data
	int m_bKeyFrame;				//!< key flag of output data
	int m_iKeyCount;				//!< current key count
	int m_Reserved[32-6];			//!< reserved...
} common_dmx_output_t;

//! Seek parameter : 8 bytes
typedef struct common_dmx_seek_t
{
	int m_iSeekTimeMSec;	//!< seek time(second) from current time
	int m_iSeekMode;		//!< 0(default video) 1(video) 2(audio)
} common_dmx_seek_t;


struct creation_time_t
{
	int m_iYear;			//! year
	int m_iMonth;			//! months since January	- [0,11]
	int m_iDayOfMonth;		//! day of the month		- [1,31]
	int m_iDayOfWeek;		//! days since Sunday		- [0, 6]
	int m_iHour;			//! hours since midnight	- [0,23]
	int m_iMin;				//! minutes after the hour	- [0,59]
	int m_iSec;				//! seconds after the minute- [0,59]
};

//! Common muxer information : 384 bytes
typedef struct common_mux_info_t
{
	//! File information : 128 Bytes
	struct 
	{
		struct creation_time_t m_CTime; //! creation time
		int m_Reserved[32-7];		
	} m_sFileInfo;

	//! Video information : 128 Bytes
	struct 
	{
		int m_iWidth;					//!< 1.width
		int m_iHeight;					//!< 2.height
		int m_iFrameRate;				//!< 3.framerate * 1000;
		int m_iFourCC;					//!< 4.fourcc
		unsigned char* m_pExtraData;	//!< 5.extra data
		int m_iExtraDataLen;			//!< 6.extra data length
		int m_bAvcC;					//!< 7.avcC flag for H264 in mp4
		int m_Reserved[32-7];
	} m_sVideoInfo;

	//! Audio information : 128 Bytes
	struct 
	{
		int m_iSamplePerSec;			//!< 1.samples/sec
		int m_iBitsPerSample;			//!< 2.bits/sample
		int m_iChannels;				//!< 3.channels
		int m_iFormatId;				//!< 4.format id
		unsigned char* m_pExtraData;	//!< 5.extra data
		int m_iExtraDataLen;			//!< 6.extra data length
		int m_Reserved[32-6];
	} m_sAudioInfo;

} common_mux_info_t;

//! sps/pps parameter set (demuxer) : 8 bytes
typedef struct avc_parameter_set_t
{
	void* m_pData;		//!< pointer of sps or pps 
	int m_iDataLength;	//!< size of sps or pps 
} avc_parameter_set_t;

//! avcC info(demuxer) : AVCDecoderConfigurationRecord in avc file format(ISO/IEC 14496-15) : 28 bytes
typedef struct avcC_t 
{
	int m_iSpsNum;						//!< numbers of sps
	avc_parameter_set_t* m_pSpsArray;	//!< array of sps data
	int m_iPpsNum;						//!< numbers of pps
	avc_parameter_set_t* m_pPpsArray;	//!< array of pps data
	int m_iNalLenSize;					//!< size of nal length
} avcC_t;

#ifndef FOURCC
#define FOURCC(ch0, ch1, ch2, ch3)                              \
			 ((unsigned long)(unsigned char)(ch0) | ((unsigned long)(unsigned char)(ch1) << 8) |   \
			 ((unsigned long)(unsigned char)(ch2) << 16) | ((unsigned long)(unsigned char)(ch3) << 24 ))
#endif
/************************************************************************

	Video defines

************************************************************************/
#define FOURCC_vide FOURCC('v','i','d','e') // type

// H.264 Family
#define	FOURCC_avc1 FOURCC('a','v','c','1') //0x31637661
#define	FOURCC_AVC1 FOURCC('A','V','C','1') //0x31435641
#define	FOURCC_h264 FOURCC('h','2','6','4') 
#define	FOURCC_H264 FOURCC('H','2','6','4') //0x34363248
#define	FOURCC_x264 FOURCC('x','2','6','4') 
#define	FOURCC_X264 FOURCC('X','2','6','4') 
#define	FOURCC_vssh FOURCC('v','s','s','h') 
#define	FOURCC_VSSH FOURCC('V','S','S','H') 
#define	FOURCC_davc FOURCC('d','a','v','c') //Dicas MPEGable H.264/MPEG-4 AVC base profile codec
#define	FOURCC_DAVC FOURCC('D','A','V','C') //Dicas MPEGable H.264/MPEG-4 AVC base profile codec

// MPEG-4 Family
#define	FOURCC_mp4v FOURCC('m','p','4','v') //0x7634706d
#define	FOURCC_MP4V FOURCC('M','P','4','V') //0x5634504d
#define FOURCC_SEDG FOURCC('S','E','D','G') //0x47444553
#define FOURCC_RMP4 FOURCC('R','M','P','4') //0x34504d52
#define	FOURCC_xvid FOURCC('x','v','i','d')
#define	FOURCC_Xvid FOURCC('X','v','i','d')
#define	FOURCC_XVID FOURCC('X','V','I','D')
#define	FOURCC_divx FOURCC('d','i','v','x')
#define	FOURCC_DIVX FOURCC('D','I','V','X')
#define	FOURCC_MP43 FOURCC('M','P','4','3')
#define	FOURCC_mp43 FOURCC('m','p','4','3')
#define	FOURCC_DIV4 FOURCC('D','I','V','4')
#define	FOURCC_div4 FOURCC('d','i','v','4')
#define	FOURCC_DX50 FOURCC('D','X','5','0')
#define	FOURCC_dx50 FOURCC('d','x','5','0')

#define	FOURCC_FMD4 FOURCC('F','M','D','4')
#define	FOURCC_fmd4 FOURCC('f','m','d','4')

#define	FOURCC_3IV2 FOURCC('3','I','V','2')
#define	FOURCC_3iv2 FOURCC('3','i','v','2')

#define	FOURCC_FVFW FOURCC('F','V','F','W')
#define	FOURCC_fvfw FOURCC('f','v','f','w')

#define	FOURCC_DIVF FOURCC('D','I','V','F')
#define	FOURCC_divf FOURCC('d','i','v','f')

#define	FOURCC_FMP4 FOURCC('F','M','P','4')
#define	FOURCC_fmp4 FOURCC('f','m','p','4')

#define	FOURCC_MPG4 FOURCC('M','P','G','4')
#define	FOURCC_mpg4 FOURCC('m','p','g','4')

#define	FOURCC_COL1 FOURCC('C','O','L','1')
#define	FOURCC_col1 FOURCC('c','o','l','1')

#define	FOURCC_MP42 FOURCC('M','P','4','2')
#define	FOURCC_mp42 FOURCC('m','p','4','2')
#define	FOURCC_MP4S FOURCC('M','P','4','S')
#define	FOURCC_mp4s FOURCC('m','p','4','s')
#define	FOURCC_M4S2 FOURCC('M','4','S','2')
#define	FOURCC_m4s2 FOURCC('m','4','s','2')



#define	FOURCC_rmp4 FOURCC('r','m','p','4')
#define	FOURCC_XviD FOURCC('X','v','i','D')

// H.263 Family
#define	FOURCC_h263 FOURCC('h','2','6','3')
#define	FOURCC_s263 FOURCC('s','2','6','3') //0x33363273 ITU H.263 video (3GPP format)
#define	FOURCC_S263 FOURCC('S','2','6','3') //0x33363253
#define	FOURCC_H263 FOURCC('H','2','6','3') //0x33363248

// Sorenson's H.263 Family
#define	FOURCC_flv1 FOURCC('f','l','v','1')
#define	FOURCC_FLV1 FOURCC('F','L','V','1')

// VC-1 Family
#define FOURCC_WMV3 FOURCC('W','M','V','3')
#define FOURCC_wmv3 FOURCC('w','m','v','3')
#define FOURCC_WVC1 FOURCC('W','V','C','1')
#define FOURCC_wvc1 FOURCC('w','v','c','1')
#define FOURCC_WMVA FOURCC('W','M','V','A')
#define FOURCC_wmva FOURCC('w','m','v','a')
#define FOURCC_VC1	FOURCC('V','C','1',' ')
#define FOURCC_vc1	FOURCC('v','c','1',' ')
#define FOURCC_WMV2 FOURCC('W','M','V','2')
#define FOURCC_wmv2 FOURCC('w','m','v','2')
#define FOURCC_WMV1 FOURCC('W','M','V','1')
#define FOURCC_wmv1 FOURCC('w','m','v','1')

#define	FOURCC_MJPG FOURCC('M','J','P','G')
#define	FOURCC_mjpg FOURCC('m','j','p','g')
#define	FOURCC_IJPG FOURCC('I','J','P','G') //it works

#define	FOURCC_mjpa FOURCC('m','j','p','a') //0x61706a6d
#define	FOURCC_mjpb FOURCC('m','j','p','b')
#define	FOURCC_3IV1 FOURCC('3','I','V','1')
#define	FOURCC_3IV2 FOURCC('3','I','V','2')
#define	FOURCC_SVQ1 FOURCC('S','V','Q','1')
#define	FOURCC_SVQ3 FOURCC('S','V','Q','3')

#define	FOURCC_VP31 FOURCC('V','P','3','1')
#define	FOURCC_cvid FOURCC('c','v','i','d')

#define FOURCC_MPEG FOURCC('M','P','E','G')
#define FOURCC_mpeg FOURCC('m','p','e','g')
#define FOURCC_MPG2 FOURCC('M','P','G','2')
#define FOURCC_mpg2 FOURCC('m','p','g','2')
#define FOURCC_mpg1 FOURCC('m','p','g','1')
#define FOURCC_MP2V FOURCC('M','P','2','V')
#define FOURCC_mp2v FOURCC('m','p','2','v')
#define	FOURCC_m1v1 FOURCC('m','1','v','1')
#define	FOURCC_m2v1 FOURCC('m','2','v','1')
#define	FOURCC_hdv1 FOURCC('h','d','v','1')

#define FOURCC_div3 FOURCC('d','i','v','3')
#define FOURCC_DIV3 FOURCC('D','I','V','3')

#define FOURCC_rv10 FOURCC('r','v','1','0')
#define FOURCC_RV10 FOURCC('R','V','1','0')
#define FOURCC_rv20 FOURCC('r','v','2','0')
#define FOURCC_RV20 FOURCC('R','V','2','0')
#define FOURCC_rv30 FOURCC('r','v','3','0')
#define FOURCC_RV30 FOURCC('R','V','3','0')
#define FOURCC_rv40 FOURCC('r','v','4','0')
#define FOURCC_RV40 FOURCC('R','V','4','0')
#define FOURCC_RV89COMBO FOURCC('T','R','O','M')

/************************************************************************

	Audio defines

************************************************************************/
#define FOURCC_soun FOURCC('s','o','u','n') // type

#define FOURCC_sowt FOURCC('s','o','w','t')
#define FOURCC_twos FOURCC('t','w','o','s')
#define FOURCC_raw  FOURCC('r','a','w',' ')
#define FOURCC_NONE FOURCC('N','O','N','E')
#define FOURCC_ulaw FOURCC('u','l','a','w')
#define FOURCC_alaw FOURCC('a','l','a','w')
#define FOURCC_QDM2 FOURCC('Q','D','M','2')
#define FOURCC_ima4 FOURCC('i','m','a','4')
#define FOURCC_mp4a FOURCC('m','p','4','a')	
#define FOURCC_samr FOURCC('s','a','m','r') // Narrowband AMR voice(3GPP)
#define FOURCC_sawb FOURCC('s','a','w','b') // Wideband AMR voice(3GPP)
#define FOURCC_sawp FOURCC('s','a','w','p') // Extended AMR-WB (AMR-WB+) (3GPP)
#define FOURCC_sqcp FOURCC('s','q','c','p') // 13K(QCELP) Voice(3GPP2) 0xE1 
#define FOURCC_QLCM FOURCC('Q','L','C','M')
#define FOURCC_sevc FOURCC('s','e','v','c') // EVRC Voice(3GPP2) 0xA0
#define FOURCC_EVRC FOURCC('E','V','R','C')
#define FOURCC_secb FOURCC('s','e','c','b') // EVRC-B Voice(3GPP2) 
#define FOURCC_secw FOURCC('s','e','c','w') // EVRC-WB Voice(3GPP2) 
#define FOURCC_ssmv FOURCC('s','s','m','v') // SMV Voice(3GPP2) 0xA1
#define FOURCC_svmr FOURCC('s','v','m','r') // VMR Voice(3GPP2)
#define FOURCC_ec3  FOURCC('e','c','-','3') // ac-3
#define FOURCC_ac3  FOURCC('a','c','-','3') // ac-3
#define FOURCC_alis FOURCC('a','l','i','s')

// rfc2361
// RIFF AudioFormat Tags
// http://www.sno.phy.queensu.ca/~phil/exiftool/TagNames/RIFF.html
#define AV_AUDIO_UNKNOWN			0x0000
#define AV_AUDIO_MS_PCM				0x0001	// Little endian LPCM	
#define AV_AUDIO_MS_EXTENSIBLE		0xFFFE
#define AV_AUDIO_MS_ADPCM			0x0002
#define AV_AUDIO_IEEE_FLOAT			0x0003
#define AV_AUDIO_MS_ALAW			0x0006
#define AV_AUDIO_MS_ULAW			0x0007
#define AV_AUDIO_MS_PCM_SWAP		0x0008	// Big endian LPCM	
#define AV_AUDIO_INTEL_DVI_ADPCM 	0x0011
#define AV_AUDIO_G723_ADPCM 		0x0014

#define AV_AUDIO_DOLBY_AC2	 		0x0030
#define AV_AUDIO_G721_ADPCM 		0x0040
#define AV_AUDIO_G728_CELP	 		0x0041
#define AV_AUDIO_MS_G723	 		0x0042
#define AV_AUDIO_ITUT_G726	 		0x0045
#define AV_AUDIO_MP2		 		0x0050 //MPEG 1 Layer 2 audio codec
#define AV_AUDIO_MP3		 		0x0055 //MPEG-1 Layer 3 (MP3) audio codec
#define AV_AUDIO_APICOM_G726_ADPCM	0x0064
#define AV_AUDIO_APICOM_G722_ADPCM	0x0065
#define AV_AUDIO_ATNT_G729A	 		0x0083
#define AV_AUDIO_SIEMENS_SBC24 		0x0091
#define AV_AUDIO_DOLBY_AC3_SPDIF 	0x0092
#define AV_AUDIO_MS_AAC				0x00ff //AAC
#define AV_AUDIO_MS_AUDIO1		 	0x0160
#define AV_AUDIO_QDESIGN2	 		0x0450 //QDesign Music
#define AV_AUDIO_AC3 				0x2000 //AC3
#define AV_AUDIO_DTS				0x2001 //DTS
#define AV_AUDIO_RA_1_2_144			0x2002 //RealAudio 1 / 2 14.4
#define AV_AUDIO_RA_1_2_288			0x2003 //RealAudio 1 / 2 28.8
#define AV_AUDIO_RA_G2_8_COOK		0x2004 //RealAudio G2 / 8 Cook (low bitrate)
#define AV_AUDIO_RA_3_4_5_MUSIC		0x2005 //RealAudio 3 / 4 / 5 Music (DNET)
#define AV_AUDIO_RA_10_AAC			0x2006 //RealAudio 10 AAC (RAAC)
#define AV_AUDIO_RA_10_AACP			0x2007 //RealAudio 10 AAC+ (RACP)
#define AV_AUDIO_EAC3 				0x2008 //EAC3 Dolby Digital Plus
#define AV_AUDIO_OGG_VORBIS_MODE1	0x674f //Ogg Vorbis (mode 1) 
#define AV_AUDIO_OGG_VORBIS_MODE2	0x6750 //Ogg Vorbis (mode 2) 
#define AV_AUDIO_OGG_VORBIS_MODE3	0x6751 //Ogg Vorbis (mode 3) 
#define AV_AUDIO_OGG_VORBIS_MODE1P	0x676f //Ogg Vorbis (mode 1+) 
#define AV_AUDIO_OGG_VORBIS_MODE2P	0x6770 //Ogg Vorbis (mode 2+) 
#define AV_AUDIO_OGG_VORBIS_MODE3P	0x6771 //Ogg Vorbis (mode 3+) 
#define AV_AUDIO_FAAD_AAC			0x706d //FAAD AAC 
#define AV_AUDIO_FLAC				0xf1ac //Free Lossless Audio Codec FLAC

//http://msdn.microsoft.com/en-us/library/dd443195(VS.85).aspx
#define AV_AUDIO_WMA_STANDARD		0x0161 //Windows Media Audio Standard encoded content
#define AV_AUDIO_WMA_PRO			0x0162 //Windows Media Audio Professional encoded content
#define AV_AUDIO_WMA_LOSSLESS		0x0163 //Windows Media Audio Lossless encoded content
#define AV_AUDIO_WMA_VOICE			0x000a //Windows Media Audio Voice encoded content

// http://www.matroska.org/technical/specs/codecid/index.html
#define AV_AUDIO_TTA1				0x77a1 //The True Audio lossles audio compressor

//Mplayer/etc/codecs.conf
#define AV_AUDIO_AAC				0xAAC0 //Borgtech nonsense tag 

// The others
#define AV_AUDIO_SPEEX				0xA109 //Speex ACM Codec 
#define AV_AUDIO_AMR_WP				0x0056
#define AV_AUDIO_AMR_NB				0x0057
#define AV_AUDIO_AMR_WB				0x0058
#define AV_AUDIO_AAC_706D       	0x0000706d // AAC
#define AV_AUDIO_RAAAC				0x72616163
#define AV_AUDIO_RACP				0x72616370
#define AV_AUDIO_RAAC3			    0x646E6574 //"ra_ac3"
#define AV_AUDIO_QCELP				0xCC00
#define AV_AUDIO_EVRC				0xCD00
#define AV_AUDIO_EVRC_B				0xCD01
#define AV_AUDIO_EVRC_WB			0xCD02
#define AV_AUDIO_VMR				0xCD10
#define AV_AUDIO_SMV				0xCD20
#define AV_AUDIO_MP3HD 				0xCD40
#define AV_AUDIO_MP1 				0xCC01
#define AV_AUDIO_BSAC_MP4			0x0016
#define AV_AUDIO_BSAC_CDK			0xAAC1
#define AV_AUDIO_COOK				0x636F6F6B
#define AV_AUDIO_cook 				0x434F4F4B 
#define AV_AUDIO_kooc 				0x4B4F4F43 
#define AV_AUDIO_APE				0x0a9e
#define AV_AUDIO_QT_IMA_ADPCM 		0x10011
#define AV_AUDIO_OGG_VORBIS_PACKET 			0x2674F
#define AV_AUDIO_OGG_VORBIS_INTERNAL_VIDEO 	0x1674F
#define AV_AUDIO_OGG_VORBIS_INTERNAL_AUDIO 	0x3674F


/************************************************************************

	Image defines (not used)

************************************************************************/
#define	FOURCC_AVDJ FOURCC('A','V','D','J')
#define	FOURCC_jpeg FOURCC('j','p','e','g')

//#define	FOURCC_png$ FOURCC('p','n','g',' ')
#define	FOURCC_png  FOURCC('p','n','g',' ')
#define	FOURCC_tiff FOURCC('t','i','f','f')




/************************************************************************

	Subtitle defines (not used)

************************************************************************/
#define FOURCC_sdsm		FOURCC('s','d','s','m') // type

#define FOURCC_ASCII	FOURCC('A','S','C',' ')
#define FOURCC_OEM		FOURCC('O','E','M',' ')
#define FOURCC_UTF8		FOURCC('U','T','F','8')
#define FOURCC_UTF16	FOURCC('U','T','F','1')
#define FOURCC_SSA		FOURCC('S','S','A',' ')
#define FOURCC_ASS		FOURCC('A','S','S',' ')
#define FOURCC_USF		FOURCC('U','S','F',' ')
#define FOURCC_TEXT		FOURCC('T','E','X','T')

#endif//_COMMON_COMMON_H_
