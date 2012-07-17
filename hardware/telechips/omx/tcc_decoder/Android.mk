LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES := \
	src/decoder.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_videodec_interface/include \
	kernel/arch/arm/mach-$(TARGET_BOARD_PLATFORM)/include

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.VPUDec

	
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

ifeq ($(TARGET_BOARD_PLATFORM),tcc93xx)
LOCAL_CFLAGS += -DTCC_93XX_INCLUDE	
endif
ifeq ($(TARGET_BOARD_PLATFORM),tcc88xx)
LOCAL_CFLAGS += -DTCC_88XX_INCLUDE	
endif

LOCAL_MODULE := libTCC_Decoder
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

