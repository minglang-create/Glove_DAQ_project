# Glove_DAQ_RV1126B_SDK —— 手套 DAQ 主程序(协议契约V1第四次修订)
# 本目录在 app/ 下一层(非 cam_daq/ 下), 先自己加载 SDK 参数,
# 再借 cam_daq/common.mk 的编译规则(其内部 include 会被 APP_PARAM 已设而跳过)。
TARGET := glove_daq_rv
APP_PARAM := ../Makefile.param
include $(APP_PARAM)
include ../cam_daq/common.mk
SRCS := main.c daq_fsm.c glove_link.c glove_view.c cam_pipeline.c align.c recorder.c button.c
