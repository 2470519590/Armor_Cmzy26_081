TARGET = armor_boot
BUILD_DIR = build
CC = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
ROOT = ..
CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
INC = -I$(ROOT)/Core/Inc -I$(ROOT)/App -I$(ROOT)/Drivers/STM32L4xx_HAL_Driver/Inc -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32L4xx/Include -I$(ROOT)/Drivers/CMSIS/Include
DEFS = -DUSE_HAL_DRIVER -DSTM32L432xx
CFLAGS = $(CPU) $(INC) $(DEFS) -Os -Wall -ffunction-sections -fdata-sections
SRC = main.c $(ROOT)/Core/Src/system_stm32l4xx.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c $(ROOT)/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
OBJ = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC:.c=.o))) $(BUILD_DIR)/startup_stm32l432xx.o
vpath %.c $(sort $(dir $(SRC)))
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
$(BUILD_DIR):
	mkdir $(BUILD_DIR)
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/startup_stm32l432xx.o: $(ROOT)/startup_stm32l432xx.s | $(BUILD_DIR)
	$(CC) -x assembler-with-cpp -c $(CPU) $< -o $@
$(BUILD_DIR)/$(TARGET).elf: $(OBJ) $(ROOT)/STM32L432xx_BOOT.ld
	$(CC) $(OBJ) $(CPU) -specs=nano.specs -T$(ROOT)/STM32L432xx_BOOT.ld -Wl,--gc-sections -lc -lnosys -o $@
	$(SZ) $@
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@
.PHONY: all
