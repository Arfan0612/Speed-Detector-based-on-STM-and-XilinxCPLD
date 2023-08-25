################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Decimal_BCD_converter.c \
../Core/Src/comparator.c \
../Core/Src/display.c \
../Core/Src/lcd16x2_v2.c \
../Core/Src/main.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c 

OBJS += \
./Core/Src/Decimal_BCD_converter.o \
./Core/Src/comparator.o \
./Core/Src/display.o \
./Core/Src/lcd16x2_v2.o \
./Core/Src/main.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o 

C_DEPS += \
./Core/Src/Decimal_BCD_converter.d \
./Core/Src/comparator.d \
./Core/Src/display.d \
./Core/Src/lcd16x2_v2.d \
./Core/Src/main.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I"C:/Users/Arfan Danial/OneDrive - University of Nottingham Malaysia/Year 2/Practical Engineering/Speed_Detector_STM_Xilinx/Core/Inc" -I"C:/Users/Arfan Danial/OneDrive - University of Nottingham Malaysia/Year 2/Practical Engineering/Speed_Detector_STM_Xilinx/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -include"C:/Users/Arfan Danial/OneDrive - University of Nottingham Malaysia/Year 2/Practical Engineering/Speed_Detector_STM_Xilinx/Drivers/CMSIS/DSP/Include/arm_math.h" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Decimal_BCD_converter.d ./Core/Src/Decimal_BCD_converter.o ./Core/Src/Decimal_BCD_converter.su ./Core/Src/comparator.d ./Core/Src/comparator.o ./Core/Src/comparator.su ./Core/Src/display.d ./Core/Src/display.o ./Core/Src/display.su ./Core/Src/lcd16x2_v2.d ./Core/Src/lcd16x2_v2.o ./Core/Src/lcd16x2_v2.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su

.PHONY: clean-Core-2f-Src

