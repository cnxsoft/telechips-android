LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_base_component.c \
	src/omx_base_port.c \
	src/omx_base_audio_port.c \
	src/omx_base_clock_port.c \
	src/omx_base_filter.c \
	src/omx_base_sink.c \
	src/omx_base_source.c \
	src/omx_base_video_port.c \
	src/omx_base_image_port.c \
	src/TCCFile.c \
	src/TCCMemory.c \
	src/queue.c \
	src/tsemaphore.c 
	
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_include \
	$(TCC_OMX_INCLUDES) \
	$(PV_INCLUDES)

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog

LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS)

#LOCAL_LDFLAGS := \
#	-L$(OMX_TOP)/omx_lib \
#	-lOMX.TCC.common ## test

LOCAL_MODULE := libOMX.TCC.base
LOCAL_MODULE_TAGS := optional

#include $(BUILD_STATIC_LIBRARY)
include $(BUILD_SHARED_LIBRARY)

