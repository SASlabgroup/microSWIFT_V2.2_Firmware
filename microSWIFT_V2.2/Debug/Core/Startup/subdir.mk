################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32u5a5zjtxq.s 

OBJS += \
./Core/Startup/startup_stm32u5a5zjtxq.o 

S_DEPS += \
./Core/Startup/startup_stm32u5a5zjtxq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m33 -g3 -DDEBUG -DTX_SINGLE_MODE_NON_SECURE=1 -c -I../Core/Inc -I../FileX/App -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Middlewares/ST/filex/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Middlewares/ST/filex/ports/generic/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.7.0/Drivers/CMSIS/Include -I../FileX/Target -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32u5a5zjtxq.d ./Core/Startup/startup_stm32u5a5zjtxq.o

.PHONY: clean-Core-2f-Startup

