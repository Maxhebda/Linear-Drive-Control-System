################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: linear_drive_control_system.axf

# Tool invocations
linear_drive_control_system.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\DELL\Desktop\Wbudowane\lpc_board_nxp_lpcxpresso_1347\Debug" -L"C:\Users\DELL\Desktop\Wbudowane\lpc_chip_13xx\Debug" -L"C:\Users\DELL\Desktop\Wbudowane\CMSIS_DSPLIB_CM3\lib" -Xlinker -Map="linear_drive_control_system.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "linear_drive_control_system_Debug.ld" -o "linear_drive_control_system.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) linear_drive_control_system.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "linear_drive_control_system.axf"; # arm-none-eabi-objcopy -v -O binary "linear_drive_control_system.axf" "linear_drive_control_system.bin" ; # checksum -p LPC1347 -d "linear_drive_control_system.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
