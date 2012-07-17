LOCAL_PATH := $(call my-dir)
MY_PATH := $(LOCAL_PATH)

include $(CLEAR_VARS)

LOCAL_PATH := $(MY_PATH)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_spdif_component.c \

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(CDK_DIR)/cdk \
	$(CDK_DIR)/audio_codec \
	$(CDK_DIR)/container \
	$(OMX_TOP)/omx_include \
	$(OMX_TOP)/omx_base/include \
	$(LOCAL_PATH)/spdif/include \

LOCAL_STATIC_LIBRARIES := \
	libtccspdifparser

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base \

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-D__ANDROID__

LOCAL_MODULE := libOMX.TCC.spdif
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

include $(call first-makefiles-under,$(LOCAL_PATH))
