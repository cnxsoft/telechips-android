LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/pAACEnc.c \
	src/omx_aacenc_component.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_include \
	$(OMX_TOP)/omx_base/include \
	$(TCC_OMX_INCLUDES) \
	$(PV_INCLUDES)

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base
	
LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-DSYS_LINUX

ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_LDFLAGS := \
	-L$(LOCAL_PATH)/lib \
        -lTCC89_92xx_AACENC_ANDROID_V2.02.01_CTS
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc93xx)
LOCAL_LDFLAGS := \
	-L$(LOCAL_PATH)/lib \
        -lTCC93xx_AACENC_ANDROID_V2.02.01_CTS
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc88xx)
LOCAL_LDFLAGS := \
	-L$(LOCAL_PATH)/lib \
        -lTCC88xx_AACENC_ANDROID_V2.02.01_CTS
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc892x)
LOCAL_LDFLAGS := \
        -L$(LOCAL_PATH)/lib \
        -lTCC88xx_AACENC_ANDROID_V2.02.01_CTS
endif
	
LOCAL_MODULE := libOMX.TCC.aacenc
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

