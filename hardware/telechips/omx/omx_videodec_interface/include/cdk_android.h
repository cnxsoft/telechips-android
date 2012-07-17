/*!
 ***********************************************************************
 \par Copyright
 \verbatim
  ________  _____           _____   _____           ____  ____   ____		
     /     /       /       /       /       /     /   /    /   \ /			
    /     /___    /       /___    /       /____ /   /    /____/ \___			
   /     /       /       /       /       /     /   /    /           \		
  /     /_____  /_____  /_____  /_____  /     / _ /_  _/_      _____/ 		
   																				
  Copyright (c) 2010 Telechips Inc.
  Korad Bldg, 1000-12 Daechi-dong, Kangnam-Ku, Seoul, Korea					
 \endverbatim
 ***********************************************************************
 */
/*!
 ***********************************************************************
 *
 * \file
 *		cdk_android.h
 * \date
 *		2010/04/27
 * \author
 *		Android team(android@telechips.com) 
 * \brief
 *		support Android system 	
 * \version
 *		0.0.1 : 2010/04/27
 *
 ***********************************************************************
 */
// must sync with common/CDK/cdk/cdk_android.h

#ifndef _CDK_ANDROID_H_
#define _CDK_ANDROID_H_

#define CDK_ANDROID_VIDEO_EXTRA_INFO_SIZE 8

#define CDK_ANDROID_VIDEO_EXTRA_DATA_IDENTIFIER 0x54434320 // 'TCC '

#define CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE				0x00000000
#define CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE_STEPUP1		0x00000001
#define CDK_ANDROID_VIDEO_EXTRA_NORMAL_MODE_STEPUP2		0x00000002
#define CDK_ANDROID_VIDEO_EXTRA_FRAME_SKIP_MODE			0x00000003

typedef struct
{
	unsigned long id;
	unsigned long mode; 
} CDK_ANDROID_VIDEO_EXTRA_INFO;

#endif // _CDK_ANDROID_H_

