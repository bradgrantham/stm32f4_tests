OPT             =       -O2

TOOLROOT        =       $(HOME)/trees/gcc-arm-none-eabi-4_9-2015q3
STM32FCUBEROOT  =       $(HOME)/trees/STM32Cube_FW_F4_V1.9.0
GCC_LIB         =       $(TOOLROOT)/lib/gcc/arm-none-eabi/4.9.3

CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mlittle-endian
TARGET = STM32F415xx
STM32F4XX_CMSIS_INC_PATH        = $(STM32FCUBEROOT)/Drivers/CMSIS/Include/
CORTEX_M4_HWFP_LIB_PATH = $(GCC_LIB)/armv7e-m/fpu

OBJECTS = \
        simple_m4.o \
        startup_stm32f417xx.o \
        system_stm32f4xx.o \
        stm32f4xx_it.o \
        stm32f4xx_hal_gpio.o \
        stm32f4xx_hal_cortex.o \
        stm32f4xx_hal_rcc.o \
        stm32f4xx_hal.o \
        $(NULL)


simple_m4.hex: simple_m4.elf
	$(TOOLROOT)/bin/arm-none-eabi-objcopy -O ihex $< $@

simple_m4.elf: $(OBJECTS)
	$(TOOLROOT)/bin/arm-none-eabi-gcc $(OPT) $(CORTEX_M4_HWFP_CC_FLAGS) -DSTM32F415xx -L$(CORTEX_M4_HWFP_LIB_PATH) -TSTM32F415RG_FLASH.ld -lm -specs=nosys.specs -Wl,--gc-sections $^ -o $@

startup_stm32f417xx.o: startup_stm32f417xx.s
	$(TOOLROOT)/bin/arm-none-eabi-gcc -Wall $(OPT) $(CORTEX_M4_HWFP_CC_FLAGS)  $^ -c -o $@

%.o: %.c
	$(TOOLROOT)/bin/arm-none-eabi-gcc -std=c99 -Wall $(OPT) -I$(STM32F4XX_CMSIS_INC_PATH) -D$(TARGET) $(CORTEX_M4_HWFP_CC_FLAGS)  $^ -c -o $@

clean:
	rm simple_m4.hex *.o
