# ---- app.mk: 装甲模块应用层（设计文档 v1.5 第 5.2 节）----
# CubeMX 重生成会覆写 Makefile，需重新在 Makefile 末尾追加 "-include app.mk"。
# 注意：Makefile 的 OBJECTS 按 notdir 扁平化，APP_SOURCES 内文件名必须全局唯一。

APP_SOURCES = \
App/bsp/ads131m04.c \
App/bsp/ws2812_uart.c \
App/bsp/temp_mon.c \
App/detect/hit_detect.c \
App/detect/calibration.c \
App/comm/app_can.c \
App/comm/app_log.c \
App/comm/can_node.c \
App/comm/board_comm.c \
App/app/state_machine.c \
App/app/led_status.c \
App/app/faults.c \
App/app/self_test.c \
App/app/reset_cause.c \
App/app/main_app.c

C_SOURCES += $(APP_SOURCES)
C_INCLUDES += -IApp -IApp/bsp -IApp/detect -IApp/comm \
              -IApp/app
