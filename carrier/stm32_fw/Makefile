TARGET   := carrier_fw
BUILD    := build
CC       := arm-none-eabi-gcc
OBJCOPY  := arm-none-eabi-objcopy
SIZE     := arm-none-eabi-size

MCU_FLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

C_SOURCES := \
    Core/Src/main.c \
    BSP/bsp_uart.c \
    Module/carrier_ctrl.c \
    Module/comm.c \
    Middlewares/FreeRTOS/Source/tasks.c \
    Middlewares/FreeRTOS/Source/queue.c \
    Middlewares/FreeRTOS/Source/list.c \
    Middlewares/FreeRTOS/Source/timers.c \
    Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
    Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c \
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

ASM_SOURCES := Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s

C_INCLUDES := \
    -ICore/Inc \
    -IBSP \
    -IDriver \
    -IAlgorithm \
    -IModule \
    -I../../shared/protocol \
    -IMiddlewares/FreeRTOS/Source/include \
    -IMiddlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F \
    -IDrivers/CMSIS/Include \
    -IDrivers/CMSIS/Device/ST/STM32F4xx/Include

C_DEFS := -DSTM32F407xx
CFLAGS := $(MCU_FLAGS) $(C_DEFS) $(C_INCLUDES) -O2 -Wall -fdata-sections -ffunction-sections -std=gnu11
ASFLAGS := $(MCU_FLAGS) -x assembler-with-cpp
LDSCRIPT := STM32F407VGTx_FLASH.ld
LDFLAGS  := $(MCU_FLAGS) -T$(LDSCRIPT) -lm -Wl,--gc-sections

OBJECTS := $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o)) $(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	@$(SIZE) $<

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	@mkdir -p $(dir $@)
	$(CC) $^ $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

flash: $(BUILD)/$(TARGET).bin
	st-flash write $< 0x08000000

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
