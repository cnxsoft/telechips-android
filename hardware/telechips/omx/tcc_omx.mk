# Copyright (C) 2011 Telechips, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This file lists the modules that are specific to all TCC devices.
	
PRODUCT_PACKAGES += \
	libOMX.TCC.base

PRODUCT_PACKAGES += \
	libTCC_CDK_AUDIO \
	libTCC_CDK_LIB \
	libTCC_CDK_CONFIG \
	libTCC_CDK_WRAPPER \
	libTCC_RM_DMX \
	libTCC_ASF_DMX
		
PRODUCT_PACKAGES += \
	libOMX.TCC.aacdec \
	libOMX.TCC.aacenc \
	libOMX.TCC.ac3dec \
	libOMX.TCC.apedec \
	libOMX.TCC.audio \
	libOMX.TCC.flacdec \
	libOMX.TCC.mp2dec \
	libOMX.TCC.mp3dec \
	libOMX.TCC.mp3enc \
	libOMX.TCC.pcmdec \
	libOMX.TCC.radec \
	libOMX.TCC.spdif \
	libOMX.TCC.vorbisdec \
	libOMX.TCC.wmadec
	
PRODUCT_PACKAGES += \
	libOMX.TCC.VideoDec \
	libOMX.TCC.VideoEnc \
	libOMX.TCC.VPUDec \
	libOMX.TCC.VPUEnc \
	libTCC_Encoder \
	libTCC_Decoder \
	libOMX.TCC.Google.vpxdec
