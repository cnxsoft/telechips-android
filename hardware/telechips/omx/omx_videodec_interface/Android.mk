LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_PRELINK_MODULE := false

ifeq ($(BOARD_USE_VPU_IN_KERNEL_FLAGS),true)
LOCAL_SRC_FILES := \
	src/vdec_k.c
else
LOCAL_SRC_FILES := \
	src/vdec.c
endif

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/include \
	$(OMX_TOP)/omx_base/include \
	$(CDK_DIR)/video_codec/wmv78dec \
	kernel/arch/arm/mach-$(TARGET_BOARD_PLATFORM)/include


LOCAL_SHARED_LIBRARIES := \
	libc \
	liblog \
	libOMX.TCC.base \
	libpmap \
	libcutils

LOCAL_CFLAGS := -fno-short-enums \
	$(BOARD_MEM_FLAGS) \
	$(BOARD_MEM_TYPES) \
	$(BOARD_REV_FLAGS) \
	$(TARGET_BOOTLOADER_BOARD_CFLAGS) \
	$(BOARD_HDMI_UI_SIZE_FLAGS) \
	$(BOARD_OVERLAY_EXT_FLAGS) \
	$(BOARD_VPU_IN_KERNEL_FLAGS)

ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_C_INCLUDES += \
	$(CDK_DIR)/video_codec/sorensonH263dec
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

ifeq ($(BOARD_USE_VPU_IN_KERNEL_FLAGS),true)

ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/sorensonH263dec \
	-lTCC89_92xx_H263DEC_ANDROID_V1.2.7_CTS
endif
## RELEASE
#ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
#LOCAL_LDFLAGS := \
#	-L$(CDK_DIR)/video_codec/wmv78dec \
#	-lTCC89xx_WMV78DEC_ANDROID_V01.001.017_CTS
#endif
#ifeq ($(TARGET_BOARD_PLATFORM),tcc93xx)
#LOCAL_LDFLAGS := \
#	-L$(CDK_DIR)/video_codec/wmv78dec \
#	-lTCC93xx_WMV78DEC_ANDROID_V01.001.017_CTS
#endif
#ifeq ($(TARGET_BOARD_PLATFORM),tcc88xx)
#LOCAL_LDFLAGS := \
#	-L$(CDK_DIR)/video_codec/wmv78dec \
#	-lTCC88xx_WMV78DEC_ANDROID_V01.001.017_CTS
#endif
#ifeq ($(TARGET_BOARD_PLATFORM),tcc892x)
#LOCAL_LDFLAGS := \
#	-L$(CDK_DIR)/video_codec/wmv78dec \
#	-lTCC892x_WMV78DEC_ANDROID_V01.001.017
#endif
## RELEASE

else

ifeq ($(TARGET_BOARD_PLATFORM),tcc92xx)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-lTCC89_92xx_VPUCODEC_ANDROID_V1.0.43 \
	-L$(CDK_DIR)/video_codec/wmv78dec \
	-lTCC89xx_WMV78DEC_ANDROID_V01.001.017_CTS \
	-L$(CDK_DIR)/video_codec/sorensonH263dec \
	-lTCC89_92xx_H263DEC_ANDROID_V1.2.7_CTS
endif
ifeq ($(filter-out tcc93xx tcc88xx,$(TARGET_BOARD_PLATFORM)),)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-L$(CDK_DIR)/video_codec/wmv78dec \
	-lTCC93_88xx_VPUCODEC_ANDROID_V0.0.53 \
	-lTCC93xx_WMV78DEC_ANDROID_V01.001.017_CTS
endif
ifeq ($(filter-out tcc892x tcc891x,$(TARGET_BOARD_PLATFORM)),)
LOCAL_LDFLAGS := \
	-L$(CDK_DIR)/video_codec/vpu \
	-L$(CDK_DIR)/video_codec/wmv78dec \
	-lTCC93_88xx_VPUCODEC_ANDROID_V0.0.53 \
	-lTCC88xx_WMV78DEC_ANDROID_V01.001.017_CTS
endif

endif

LOCAL_MODULE := libOMX.TCC.VPUDec
LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

