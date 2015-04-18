LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := eng
LOCAL_MODULE := nanocomm
LOCAL_SRC_FILES := nanocomm.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_SHARED_LIBRARIES := libcutils libm
include $(BUILD_EXECUTABLE)

