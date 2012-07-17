LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/omx_videodec_component.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
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
	libOMX.TCC.VPUDec \
	libhardware \
	libpmap
	
LOCAL_LDLIBS += \
	-lpthread \
	-ldl

LOCAL_ARM_MODE := arm

LOCAL_CFLAGS := \
	$(TCC_OMX_FLAGS) \
	-DSYS_LINUX \
	$(BOARD_MEM_FLAGS) \
	$(BOARD_MEM_TYPES) \
	$(BOARD_REV_FLAGS) \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS) \
	$(BOARD_HDMI_UI_SIZE_FLAGS) \
	$(BOARD_OVERLAY_EXT_FLAGS) \
	$(BOARD_VPU_IN_KERNEL_FLAGS)

ifeq ($(BOARD_VIDEO_DISPLAY_BY_VSYNC_INT_FLAG), true)
LOCAL_CFLAGS += -DTCC_VIDEO_DISPLAY_BY_VSYNC_INT
endif
ifeq ($(BOARD_VIDEO_DEINTERLACE_SUPPORT_FLAG), true)
LOCAL_CFLAGS += -DTCC_VIDEO_DEINTERLACE_SUPPORT
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_CFLAGS += -DTCC_89XX_INCLUDE
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc93xx)
LOCAL_CFLAGS += -DTCC_93XX_INCLUDE
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc88xx)
LOCAL_CFLAGS += -DTCC_88XX_INCLUDE
endif

ifeq ($(TARGET_BOARD_PLATFORM),tcc892x)
LOCAL_CFLAGS += -DTCC_892X_INCLUDE
endif

ifeq ($(BOARD_DIVXDRM_FLAGS),true)
LOCAL_SHARED_LIBRARIES += \
	libpvdivxdrm5plugin \
	
LOCAL_CFLAGS += \
	-DDIVX_DRM5
endif

LOCAL_MODULE := libOMX.TCC.VideoDec
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

