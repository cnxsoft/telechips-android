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
 *		cdk_core.h
 * \date
 *		2009/05/29
 * \author
 *		AV algorithm group(AValgorithm@telechips.com) 
 * \brief
 *		cdk library core header
 * \version
 *		- 0.0.1 : 2009/05/29
 *
 ***********************************************************************
 */
#ifndef _CDK_CORE_H_
#define _CDK_CORE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "cdk_pre_define.h"
#ifdef HAVE_ANDROID_OS
#else
#include "cdk_sys.h"
//! Container demuxer format
#include "../container/cdmx.h"
//! Container muxer format
#include "../container/cmux.h"
//! Video codec Decoder
#include "../video_codec/vdec.h"
//! Video codec Encoder
#include "../video_codec/venc.h"
//! Audio codec Decoder
#include "../audio_codec/adec.h"
//! Audio codec Encoder
#include "../audio_codec/aenc.h"
#endif

#ifdef __cplusplus
}
#endif
#endif //_CDK_CORE_H_
