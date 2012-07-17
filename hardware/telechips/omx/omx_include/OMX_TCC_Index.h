#ifndef __TCC_OMX_INDEX_H__
#define __TCC_OMX_INDEX_H__

#define PV_OMX_COMPONENT_CAPABILITY_TYPE_INDEX 0xFF7A347

typedef enum{
	// telechips vendor index Audio add.
	OMX_IndexVendorParamFileOpen = 0x7F000001,
	OMX_IndexVendorParamMediaInfo,
	OMX_IndexVendorConfigPowerSpectrum,
	OMX_IndexVendorConfigEnergyVolume,
	OMX_IndexVendorParamSetOutputMode,
	OMX_IndexVendorConfigAlsaConfigureRequest,
	OMX_IndexVendorConfigAudioScan,
	OMX_IndexVendorConfigDynamic_Range,
	OMX_IndexVendorConfigDynamic_AlsaOpen,
	OMX_IndexVendorConfigSpdifPathMode,
	OMX_IndexVendorConfigAlsaStatus,
	OMX_IndexVendorConfigExtramodeset,
	OMX_IndexVendorConfigMultiLanguage,

	// 추후 제거할 것임 
  	OMX_IndexVendorOpenCodec,
  	OMX_IndexVendorCloseCodec,	// delete.
	OMX_IndexVendorGetTime,
	OMX_IndexVendorFileOpen,	// delete
  	OMX_IndexVendorFileClose,	// delete

	// for DEQ
	OMX_IndexVendorConfigDeqOnOff,
	OMX_IndexVendorConfigDeqMode,

	// for rec
	OMX_IndexVendorParamSetFilePath,
	OMX_IndexVendorParamSetBitrate,
	OMX_IndexVendorParamSetRecFileFormat,
	OMX_IndexVendorParamRecend,
	OMX_IndexVendorParamRecHeaderData,

	// for video
	OMX_IndexVendorStreamFromMP4, // h264 decoder should know PARSER
	OMX_IndexVendorFBSinkScreenMode, // fbsink screen mode	
	OMX_IndexVendorClockPortsUsed,	
	OMX_IndexVendorFBSinkSclaeroffsetX,
	OMX_IndexVendorFBSinkSclaeroffsetY,
	OMX_IndexVendorGetframesize,
	OMX_IndexVendorMpeg2GetOneFrameMode,
	OMX_IndexVendorGetRMVideoFormatInfo,	
	OMX_IndexVendorSinkCompSeekComplete,
	OMX_IndexVendorClearEOSFlags, //[dvd] mpg parser  : for new start lba (MPG_OP_REFRESH)
	OMX_IndexVendorSetDVDmode, //[dvd] mpg parser  
	OMX_IndexVendorSetDVDSubtitleonoff,
	OMX_IndexVendorSetDVDSubtitle,
	OMX_IndexVendorSetDVDAudioStream,
	OMX_IndexVenderParamSetFbScanMode,
	OMX_IndexVenderTimeBreakSleep,
	OMX_IndexVendorFBSinkSetYUVOutMode,
	OMX_IndexVendorMeasureStart,	
	OMX_IndexVenderResetMinStartTime,
	OMX_IndexVenderNoAudioFrame,
	OMX_IndexVendorSetCDKDMXInfo,
	OMX_IndexVendorSetCDKContainerType,
	OMX_IndexVendorGetDemuxerEOFState,	
	OMX_IndexVendorSetDisplayPosition,
	OMX_IndexVendorChangeResolution, // get VPU resolution and set fbsink comp. resolution.
	OMX_IndexVendorChangeDestinationResolution,	 
	OMX_IndexVendorQueueVpuDisplayIndex,
	OMX_IndexVendorGetOneFrameMode,
	OMX_IndexVendorDemuxStreamType,
	OMX_IndexVendorConfigDisplayIFrameNum,
	OMX_IndexVendorConfigIFrameDispTime,
	OMX_IndexVendorConfigCurrentKeyFramePTS,
	OMX_IndexVendorConfigPlaySpeed,
	OMX_IndexVendorDemuxSetAccurateSeekMode,
	
	OMX_IndexVendorConfigGetCapture,
	
	// for image
	OMX_IndexVendorSetImageInfo, // for image	
	OMX_IndexVendorGetAlphaValid, // 		
	OMX_IndexVendorSetDecOutWidth, // 
	OMX_IndexVendorSetDecOutHeight, // 	
	OMX_IndexVendorSetDecOutFormat,	
	OMX_IndexVendorSetDecOffset,	
	OMX_IndexVendorSetDecBufferPtr,		
	OMX_IndexVendorSetDecForceEnd,		
	OMX_IndexVendorSetDebugFlag,	

	//for DXB
	OMX_IndexVendorParamDxBGetSTCFunction,
	
	//for isdbt
	OMX_IndexVendorParamTunerDeviceSet,	
	OMX_IndexVendorParamTunerChannelSet,
	OMX_IndexVendorParamTunerCountryCodeSet,
	OMX_IndexVendorParamComponentClose,
	OMX_IndexVendorParamDemuxSysInit,
	OMX_IndexVendorParamDemuxSysdeInit,
	OMX_IndexVendorParamDemuxStopRequest,
	OMX_IndexVendorConfigGetEpg,
	
	//for dvbt
	OMX_IndexVendorParamTunerChannelSearchStart,
	OMX_IndexVendorParamTunerChannelSearchStop,
	OMX_IndexVendorParamDemuxSearchStart,
	OMX_IndexVendorParamSetDemuxAudioVideoStreamType,
	OMX_IndexVendorParamSetDemuxAudioVideoPID,
	OMX_IndexVendorParamTunerOpen,
	OMX_IndexVendorParamDemuxTunerInfo,
	OMX_IndexVendorParamAlsaSinkSetVolume,
	OMX_IndexVendorParamAlsaSinkMute,
	
	// for T-DMB
	OMX_IndexVendorParamTunerChanneIDSet,
	OMX_IndexVendorParamTunerServiceIDSet,
	OMX_IndexVendorParamTunerEnsembleFreqSet,
	OMX_IndexVendorConfigGetSignalStrength,
	OMX_IndexVendorParamTDMBBitrateSet,
	OMX_IndexVendorParamSetResync,
	
	OMX_IndexVendorParamTunerFrequencySet,
	OMX_IndexVendorParamNETBERSet,
	
	OMX_IndexVendorParamTunerEnsembleBitRateSet,
	OMX_IndexVendorParamTunerEnsembleRegSet,
	OMX_IndexVendorParamFBCapture,		// DxB_capture
	OMX_IndexVendorParamRecFilenameSet,
	OMX_IndexVendorParamRecStartRequest,
	OMX_IndexVendorParamRecStopRequest,

	// for Icecream sandwich
	OMX_IndexAndroidNativeBufferUsage,
	OMX_IndexUseNativeBuffers,
	OMX_IndexVendorThumbnailMode,
	OMX_IndexEncoderStoreMetadatInBuffers,
	
	OMX_IndexVendorMax = 0x7FFFFFFF
}TC_OMX_INDEXVENDORTYPE;

typedef enum
{
	OMX_ErrorVendorFileOpenFailed = 0x90000001,
	OMX_ErrorVendorFileCloseFailed,
	OMX_ErrorVendorFileReadFailed,  
	OMX_ErrorVendorMemAllocFailed ,
	OMX_ErrorVendorParserInitFailed,
	OMX_ErrorVendorParserDeinitFailed,
	OMX_ErrorVendorDecInitFailed,
	OMX_ErrorVendorEncInitFailed,
	OMX_ErrorVendorUnsupportedFormat,
	OMX_ErrorVendorNoVideoData,
	OMX_ErrorVendorNoAudioData,
	OMX_ErrorVendorNoImageData,	
	OMX_ErrorVendorBufferOverflow,
	OMX_ErrorVendorExtDataFailed,
	OMX_ErrorVendorAudioPortParamSettingFailed,
	OMX_ErrorVendorVideoPortParamSettingFailed,	
	OMX_ErrorVendorAppPrivMemAllocFailed ,
	OMX_ErrorVendorComponentLoadingFailed ,
	OMX_ErrorVendorSetupTunnelingFailed ,
	OMX_ErrorVendorComponentExecutingFailed ,
	OMX_ErrorVendorGetOneFrameFailed ,
	OMX_ErrorVendorGetExtIndexFailed ,
	OMX_ErrorVendorGetExtDataFailed ,
	OMX_ErrorVendorSetExtDataFailed ,
	OMX_ErrorVendorPortEnableFailed ,
	OMX_ErrorVendorPortDisableFailed ,
	OMX_ErrorVendorPortFlushFailed	,
	OMX_ErrorVendorTimeRequestFailed,
	OMX_ErrorVendorSemaTimerExpire,
	OMX_ErrorVendorCodecOpenFailed,
	OMX_ErrorVendorCodecCloseFailed,
	OMX_ErrorVendorInvalidInfo,
	OMX_ErrorVendorInvalidFile,
	// for rec
	OMX_ErrorVendorRecFileOpenFail,
	OMX_ErrorVendorRecFileCloseFail,

	//for image
	OMX_ErrorVendorImageGetOrgSizeFailed,	
	OMX_ErrorVendorImageSetOutWidth,	
	OMX_ErrorVendorImageSetOutHeight,
	OMX_ErrorVendorImageSetThumbOffset,
	OMX_ErrorVendorImageSetOutBufferPointer,
	OMX_ErrorVendorImageSetOutFormat,	
	
	OMX_ErrorVendorMax = 0x9000FFFF
}TC_OMX_ERRORVENDORTYPE;

typedef enum{
	STEREO_OUTPUT_MODE = 0,
	MULTI_OUTPUT_MODE
}AUDIO_OUTPUT_TYPE;

typedef enum{
	ALSA_OPEN_SRC = 0,
	ALSA_OPEN_PCM_SRC,
	ALSA_OPEN_NOTSRC,
}ALSA_OPENMODE_TYPE;

// extention index_name
#define TCC_AUDIO_FILE_OPEN_STRING "OMX.tcc.index.param.dec.inputfilename"
//#define TCC_AUDIO_FILE_CLOSE_INDEX "OMX.tcc.index.param.fileclose"
#define TCC_AUDIO_MEDIA_INFO_STRING "OMX.tcc.index.config.dec.mediainfo"
#define TCC_AUDIO_POWERSPECTUM_STRING "OMX.tcc.index.config.dec.powerspectrum"
#define TCC_AUDIO_ENERGYVOLUME_STRING "OMX.tcc.index.config.dec.energyvolume"
#define TCC_AUDIO_PARSER_MEDIA_STRING "OMX.tcc.index.param.parser.mediainfo"

#define TCC_AUDIO_DEQ_ONOFF_STRING "OMX.tcc.index.config.deq.onoff"
#define TCC_AUDIO_DEQ_MODE_STRING "OMX.tcc.index.config.deq.mode"


#define TCC_ENC_SET_BITRATE_STRING "OMX.TCC.index.param.enc.setbitrate"
#define TCC_ENC_SET_FILE_FORMAT_STRING "OMX.TCC.index.param.enc.setfileformat"
#define TCC_ENC_SETFILE_PATH_STRING "OMX.TCC.index.param.enc.setfilepath"
#define TCC_ENC_WRITE_HEADER_DATA_STRING "OMX.TCC.index.param.enc.writeheader"
#define TCC_ENC_END_STRING "OMX.TCC.index.param.enc.end"

#define TCC_BROADCAST_PID_STRING "OMX.tcc.index.config.tuner.setpid"
#define TCC_BROADCAST_SET_MODE_STRING "OMX.tcc.index.param.dec.setmode"
#define TCC_BROADCAST_SET_PMT_ID_STRING "OMX.tcc.index.param.dec.setpmtid"

#endif /* __TCC_OMX_INDEX_H__ */
