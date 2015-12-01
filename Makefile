TOOLROOT        =       ~/Downloads/gcc-arm-none-eabi-4_9-2015q3/
GCC_LIB         =       $(TOOLROOT)/lib/gcc/arm-none-eabi/4.9.3
CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CORTEX_M4_HWFP_LIB_PATH = $(GCC_LIB)/armv7e-m/fpu

simple_m4.hex: simple_m4.elf
	$(TOOLROOT)/bin/arm-none-eabi-objcopy -O ihex $< $@

simple_m4.elf: simple_m4.c
	$(TOOLROOT)/bin/arm-none-eabi-gcc $(CORTEX_M4_HWFP_CC_FLAGS) -L$(CORTEX_M4_HWFP_LIB_PATH) -lm -specs=nosys.specs $^ -o $@
