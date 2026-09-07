# Glove_DAQ_RV1126B_SDK/RV1126B —— 手套 DAQ 主程序(协议契约V1第四次修订)
# 本目录 = project/app/<仓库名>/RV1126B/, 距 app/ 两层。
# 先自己加载 SDK 参数, 再借 cam_daq/common.mk 的编译规则
# (common.mk 内部的 include 会因 APP_PARAM 已设而跳过, 避免重复加载)。
TARGET := glove_daq_rv
APP_PARAM := ../../Makefile.param
include $(APP_PARAM)
include ../../cam_daq/common.mk
SRCS := main.c daq_fsm.c glove_link.c glove_view.c cam_pipeline.c align.c recorder.c ext_uart.c button.c
