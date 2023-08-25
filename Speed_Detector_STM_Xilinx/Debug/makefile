################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Speed_Detector_STM_Xilinx
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Speed_Detector_STM_Xilinx.elf \

MAP_FILES += \
Speed_Detector_STM_Xilinx.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Speed_Detector_STM_Xilinx.list \


# All Target
all: main-build

# Main-build Target
main-build: Speed_Detector_STM_Xilinx.elf secondary-outputs

# Tool invocations
Speed_Detector_STM_Xilinx.elf Speed_Detector_STM_Xilinx.map: $(OBJS) $(USER_OBJS) C:\Users\Arfan\ Danial\OneDrive\ -\ University\ of\ Nottingham\ Malaysia\Year\ 2\Practical\ Engineering\Speed_Detector_STM_Xilinx\STM32L476RGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Speed_Detector_STM_Xilinx.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Arfan Danial\OneDrive - University of Nottingham Malaysia\Year 2\Practical Engineering\Speed_Detector_STM_Xilinx\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Speed_Detector_STM_Xilinx.map" -Wl,--gc-sections -static -L"C:\Users\Arfan Danial\OneDrive - University of Nottingham Malaysia\Year 2\Practical Engineering\Speed_Detector_STM_Xilinx\Drivers\CMSIS\Lib\GCC" -L"C:\Users\Arfan Danial\OneDrive - University of Nottingham Malaysia\Year 2\Practical Engineering\Speed_Detector_STM_Xilinx\Drivers\CMSIS\DSP\Include" -u _printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Speed_Detector_STM_Xilinx.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Speed_Detector_STM_Xilinx.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Speed_Detector_STM_Xilinx.elf Speed_Detector_STM_Xilinx.list Speed_Detector_STM_Xilinx.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
