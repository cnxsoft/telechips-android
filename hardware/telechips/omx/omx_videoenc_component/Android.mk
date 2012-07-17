LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_videoenc_component.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_include \
	$(OMX_TOP)/omx_base/include \
	$(OMX_TOP)/omx_videoenc_interface/include \
	hardware/telechips/common/libcamera/include \
	hardware/libhardware/include \
	kernel/arch/arm/mach-$(TARGET_BOARD_PLATFORM)/include \
	$(TCC_OMX_INCLUDES) \
	$(PV_INCLUDES)

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base \
	libOMX.TCC.VPUEnc \
	libhardware \
	libpmap
	
LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-DSYS_LINUX \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS) \
	$(BOARD_VPU_IN_KERNEL_FLAGS) \
	$(BOARD_OVERLAY_EXT_FLAGS) \
	$(BOARD_MEM_FLAGS) \
	$(BOARD_MEM_TYPES)

LOCAL_MODULE := libOMX.TCC.VideoEnc
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

