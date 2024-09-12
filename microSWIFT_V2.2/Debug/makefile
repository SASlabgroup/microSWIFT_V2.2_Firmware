################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ThreadX/TraceX_Support/subdir.mk
-include Middlewares/ThreadX/Low_Power_Support/subdir.mk
-include Middlewares/ThreadX/Core/subdir.mk
-include Middlewares/LevelX/NAND\ Flash\ Support/subdir.mk
-include Middlewares/FileX/LevelX\ NAND\ interface/subdir.mk
-include Middlewares/FileX/Core/subdir.mk
-include LevelX/Target/subdir.mk
-include FileX/App/subdir.mk
-include Drivers/STM32U5xx_HAL_Driver/subdir.mk
-include Core/ThreadSafe/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/NEDWaves/subdir.mk
-include Core/Src/subdir.mk
-include Core/Components/Src/subdir.mk
-include Core/Components/Drivers/Src/subdir.mk
-include AZURE_RTOS/App/subdir.mk
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


BUILD_ARTIFACT_NAME := microSWIFT_V2.2
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
microSWIFT_V2.2.elf \

MAP_FILES += \
microSWIFT_V2.2.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
microSWIFT_V2.2.list \


# All Target
all: main-build

# Main-build Target
main-build: microSWIFT_V2.2.elf secondary-outputs

# Tool invocations
microSWIFT_V2.2.elf microSWIFT_V2.2.map: $(OBJS) $(USER_OBJS) /Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/microSWIFT.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "microSWIFT_V2.2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m33 -T"/Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/microSWIFT.ld" --specs=nosys.specs -Wl,-Map="microSWIFT_V2.2.map" -Wl,--gc-sections -static -L/Users/philbush/STM32Cube/Repository/Packs/emotas/I-CUBE-CANOPEN/1.3.0/Middlewares/Third_Party/emotas/Library/lib/STM32U5 --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

microSWIFT_V2.2.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "microSWIFT_V2.2.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout microSWIFT_V2.2.elf microSWIFT_V2.2.list microSWIFT_V2.2.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
