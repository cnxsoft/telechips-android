LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

ifeq ($(BOARD_USE_VPU_IN_KERNEL_FLAGS),true)
LOCAL_SRC_FILES := \
	src/venc_k.c
else
LOCAL_SRC_FILES := \
	src/venc.c
endif

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_base/include \
	kernel/arch/arm/mach-$(TARGET_BOARD_PLATFORM)/include

LOCAL_SHARED_LIBRARIES := \
	libc \
	libutils \
	libcutils \
	liblog \
	libOMX.TCC.base \
	libpmap

LOCAL_CFLAGS := -fno-short-enums \
	$(BOARD_MEM_FLAGS) \
	$(BOARD_MEM_TYPES) \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS) \
	$(BOARD_HDMI_UI_SIZE_FLAGS) \
	$(BOARD_OVERLAY_EXT_FLAGS) \
	$(BOARD_VPU_IN_KERNEL_FLAGS)

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

ifeq ($(BOARD_USE_VPU_IN_KERNEL_FLAGS),false)
ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-lTCC89_92xx_VPUCODEC_ANDROID_V1.0.43
endif

ifeq ($(filter-out tcc93xx tcc88xx,$(TARGET_BOARD_PLATFORM)),)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-lTCC93_88xx_VPUCODEC_ANDROID_V0.0.53
endif

ifeq ($(filter-out tcc892x tcc891x,$(TARGET_BOARD_PLATFORM)),)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-lTCC93_88xx_VPUCODEC_ANDROID_V0.0.53
endif
endif

LOCAL_MODULE := libOMX.TCC.VPUEnc
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

