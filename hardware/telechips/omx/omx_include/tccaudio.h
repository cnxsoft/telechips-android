/****************************************************************************
 *   FileName    : tccaudio.h
 *   Description : 
 ****************************************************************************
 *
 *   TCC Version 1.0
 *   Copyright (c) Telechips, Inc.
 *   ALL RIGHTS RESERVED
 *
 ****************************************************************************/
#ifndef __TCCAUDIO_H__
#define __TCCAUDIO_H__

/****************************************************************************

  Revision History

 ****************************************************************************/

//#include <main.h>
 
#include <pthread.h>
#include <OMX_Core.h>
#include <OMX_Component.h>
#include <OMX_Types.h>
#include <OMX_Audio.h>
#include <tsemaphore.h>
#include <OMX_TCC_Index.h>


#define  SCERR_NO									0
#define  SCERR_NO_PROC_DEVICE                      -1    // /proc/device 가 존재하지 않는다.
#define  SCERR_DRIVER_NO_FIND                      -2    // /proc/device 에서 드라이버를 찾지 못함
#define  SCERR_NO_SOUNDCARD                        -3    // 사운드 카드가 없음
#define  SCERR_NO_MIXER                            -4    // 믹서가 없음
#define  SCERR_OPEN_SOUNDCARD                      -5    // 사운드 카드를 여는데 실패 했다.
#define  SCERR_OPEN_MIXER                          -6    // 사운드 카드를 여는데 실패 했다.
#define  SCERR_PRIVILEGE_SOUNDCARD                 -7    // 사운드 카드를 사용할 권한이 없음
#define  SCERR_PRIVILEGE_MIXER                     -8    // 미서를 사용할 권한이 없음

#define  SCERR_NO_FILE                             -10   // 파일이 없음
#define  SCERR_NOT_OPEN                            -11   // 파일이 열 수 없
#define  SCERR_NOT_WAV_FILE                        -12   // WAV 파일이 아님
#define  SCERR_NO_wav_info                       -13   // WAV 포맷 정보가 없음


#define CODEC_SEEK_FF                       0x00000000
#define CODEC_SEEK_REW                      0x00000001
#define CODEC_SEEK_OTHER                    0x00000002

#define	MONO					1		// Macro for GetEnergyofVolume Function
#define	STEREO					2		// Macro for GetEnergyofVolume Function
#define	SAMPLE_COUNT			10		// count of samples for GetEnergyofVolume Function

#define MAX_FILENAME_SIZE		256
#define  TRUE                 1
#define  FALSE                0


#define INSTANCESIZE 190568


enum
{
    SUBFN_CODEC_GETNAME,
    SUBFN_CODEC_GETARTIST,
    SUBFN_CODEC_GETTITLE,
    SUBFN_CODEC_GETBITRATE,
    SUBFN_CODEC_GETSAMPLERATE,
    SUBFN_CODEC_GETCHANNELS,
    SUBFN_CODEC_GETLENGTH,
    SUBFN_CODEC_GETTIME,
    SUBFN_CODEC_OPEN_DEC,
    SUBFN_CODEC_OPEN_ENC,
    SUBFN_CODEC_GETCAPTUREBUFFER,
    SUBFN_CODEC_SETBUFFER,
    SUBFN_CODEC_DECODE,
    SUBFN_CODEC_ENCODE,
    SUBFN_CODEC_SEEK,
    SUBFN_CODEC_SEEKLAST,
    SUBFN_CODEC_SEEKFIRST,
    SUBFN_CODEC_DECODEONEFRAME,	//VIDEO Decode OneFrame
    SUBFN_CODEC_CLOSE,
    SUBFN_CODEC_MAX
};

enum{
/* System : USB */
	USB_PLUG_IN,					
	USB_PLUG_OUT,

/* EHI TASK Command */
	EHI_STOP,
	EHI_HISR_EVENT,

/* Audio TASK Commands */
	/* Decode */
	AUDIO_OPEN_DECODE = 50,
	AUDIO_FF_SEEK_DECODE,
	AUDIO_REW_SEEK_DECODE,
	AUDIO_SEEK_DECODE,
	AUDIO_PAUSE_DECODE,
	AUDIO_RESUME_DECODE,
	AUDIO_CLOSE_DECODE,
	AUDIO_OPEN_DMB,
	AUDIO_START_DAB,
	AUDIO_DAB_WAIT,
	AUDIO_PLAY_DAB,
	AUDIO_STOP_DAB,
	/* Encode */
	AUDIO_OPEN_RECORD,
	AUDIO_PAUSE_RECORD,			
	AUDIO_RESUME_RECORD,
	AUDIO_STOP_RECORD,
	AUDIO_FINISHED_PLAY,
	AUDIO_OPENPAUSE_DECODE,
	AUDIO_PLAYER_STOP,
	AUDIO_HAL_TEST,
	AUDIO_STOP_TEST,
	AUDIO_MIXER_TEST,
	AUDIO_MAX_EVENT
};


enum
{
	CODEC_MP3 = 0,
	CODEC_WAV,
	CODEC_WMA,
	CODEC_OGG,
	CODEC_AAC,
	CODEC_MP2,
	CODEC_APE,
	CODEC_FLAC,
	CODEC_AC3,	
	CODEC_BSAC,
	CODEC_MIXER,
	NUMCODECS
};


enum
{
	IDLE = 0,
	STOP,
	PLAY,
	PAUSED,
	REC,
	EXIT,
	MAX
};

enum
{
	DECODING_STATE = 0,
	DECODING_START,
	DECODING_END,
	DECODING_MAX
};

enum
{
	ENCODING_STATE = 0,
	ENCODING_START,
	ENCODING_PAUSE,
	ENCODING_END,
	ENCODING_MAX
};


typedef enum Audio_Enc_Codec_Type
{
	AUDIO_ENC_CODEC_NULL =0,
	AUDIO_ENC_CODEC_MP3 ,
	AUDIO_ENC_CODEC_PCM,
	AUDIO_ENC_CODEC_ADPCM,
	AUDIO_ENC_CODEC_WMA,
	AUDIO_ENC_CODEC_NUM
}Audio_Enc_Codec_Type ;


#define MP3_DATA_SIZE 1500
#define MP2_DATA_SIZE 2048
#define APE_DATA_SIZE (7*1024)*2


unsigned long MP3Function(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4);
								
unsigned long FLAC_DEC_Function(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4);

unsigned long MP2Function(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4);

unsigned long APE_Function(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4);

unsigned long WMAFunction(unsigned long ulSubFn, unsigned	long ulParam1, unsigned	long ulParam2,
								unsigned long ulParam3,	unsigned long ulParam4);
								
unsigned long AACPlusFunction(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2,
								unsigned long ulParam3, unsigned long ulParam4);

unsigned long PCMFunction(unsigned long ulSubFn, unsigned long ulParam1, unsigned long ulParam2, 
								unsigned long ulParam3, unsigned long ulParam4);

int mp3_decoder_function(unsigned long ulParam1,unsigned long ulParam2,unsigned long ulParam3,unsigned char* buffer_ptr);
int mp3_open_codec_function(unsigned long ulParam1,unsigned long ulParam2,unsigned long ulParam3,unsigned char* buffer_ptr);
int mp3_seek_function(unsigned long ulParam1,unsigned long ulParam2,unsigned long ulParam3,unsigned long ulParam4);
int mp3_close_codec_function(unsigned long ulParam1,unsigned long ulParam2,unsigned long ulParam3,unsigned char* buffer_ptr);
int mp3_get_time_function(unsigned long ulParam1,unsigned long ulParam2,unsigned long ulParam3,unsigned long ulParam4);


extern void tcc_component_set_state(int component_state);
extern int tcc_component_get_state(void);
extern void GetEnergyofVolume ( short *left, short *right, int buff_length,int LR,unsigned int* left_level, unsigned int* right_level);
extern unsigned long omx_file_pos_get(void);

#endif /* __TCCAUDIO_H__ */
