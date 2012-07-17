LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_flacdec_component.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_audio_interface/include \
	$(CDK_DIR)/cdk \
	$(CDK_DIR)/audio_codec \
	$(CDK_DIR)/container \
	$(OMX_TOP)/omx_include \
	$(OMX_TOP)/omx_base/include \

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base \
	libOMX.TCC.audio
	
LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-DSYS_LINUX
	
ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/audio_codec/flacdec \
	-lTCC89_92xx_FLACDEC_ANDROID_V2.08.02_CTS
endif
ifeq ($(TARGET_BOARD_PLATFORM),tcc93xx)
LOCAL_LDFLAGS := \
	--whole-archive \
	-L$(CDK_DIR)/audio_codec/flacdec \
	-lTCC93xx_FLACDEC_ANDROID_V2.08.02_CTS
endif
ifeq ($(TARGET_BOARD_PLATFORM),tcc88xx)
LOCAL_LDFLAGS := \
	--whole-archive \
	-L$(CDK_DIR)/audio_codec/flacdec \
	-lTCC88xx_FLACDEC_ANDROID_V2.08.02_CTS
endif
ifeq ($(TARGET_BOARD_PLATFORM),tcc892x)
LOCAL_LDFLAGS := \
	--whole-archive \
	-L$(CDK_DIR)/audio_codec/flacdec \
	-lTCC892x_FLACDEC_ANDROID_V2.08.02
endif
LOCAL_MODULE := libOMX.TCC.flacdec
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

