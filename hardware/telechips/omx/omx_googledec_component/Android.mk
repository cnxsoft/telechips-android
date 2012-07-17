LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_googledec_component.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	external/libvpx \
        external/libvpx/vpx_codec \
        external/libvpx/vpx_ports \
	hardware/telechips/common/opencore/opencorehw/main \
	hardware/telechips/common/opencore/opencorehw/tccrenderer \
	$(OMX_TOP)/omx_include \
	$(OMX_TOP)/omx_base/include \
	$(OMX_TOP)/omx_videodec_interface/include \
	$(TCC_OMX_INCLUDES) \
	hardware/telechips/common/cdk_wrapper/main/include \
	kernel/arch/arm/mach-$(TARGET_BOARD_PLATFORM)/include \
	$(PV_INCLUDES) \
	hardware/libhardware/include

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base \
	libhardware

LOCAL_STATIC_LIBRARIES := \
        libvpx
	
LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-DSYS_LINUX \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS) \
	$(BOARD_HDMI_UI_SIZE_FLAGS)

LOCAL_MODULE := libOMX.TCC.Google.vpxdec
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

