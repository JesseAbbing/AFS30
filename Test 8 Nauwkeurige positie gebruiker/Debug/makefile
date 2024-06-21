################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Test 8 Nauwkeurige positie gebruiker
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Test\ 8\ Nauwkeurige\ positie\ gebruiker.elf \

MAP_FILES += \
Test\ 8\ Nauwkeurige\ positie\ gebruiker.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Test\ 8\ Nauwkeurige\ positie\ gebruiker.list \


# All Target
all: main-build

# Main-build Target
main-build: Test\ 8\ Nauwkeurige\ positie\ gebruiker.elf secondary-outputs

# Tool invocations
Test\ 8\ Nauwkeurige\ positie\ gebruiker.elf Test\ 8\ Nauwkeurige\ positie\ gebruiker.map: $(OBJS) $(USER_OBJS) C:\Users\marce\STM32CubeIDE\workspace_1.15.0\Test\ 8\ Nauwkeurige\ positie\ gebruiker\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Test 8 Nauwkeurige positie gebruiker.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\marce\STM32CubeIDE\workspace_1.15.0\Test 8 Nauwkeurige positie gebruiker\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Test 8 Nauwkeurige positie gebruiker.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Test\ 8\ Nauwkeurige\ positie\ gebruiker.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Test 8 Nauwkeurige positie gebruiker.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Test\ 8\ Nauwkeurige\ positie\ gebruiker.elf Test\ 8\ Nauwkeurige\ positie\ gebruiker.list Test\ 8\ Nauwkeurige\ positie\ gebruiker.map Test\\ 8\\ Nauwkeurige\\ positie\\ gebruiker.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
