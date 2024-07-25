################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power/tx_low_power.c 

OBJS += \
./Middlewares/ThreadX/Low_Power_Support/tx_low_power.o 

C_DEPS += \
./Middlewares/ThreadX/Low_Power_Support/tx_low_power.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ThreadX/Low_Power_Support/tx_low_power.o: /Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power/tx_low_power.c Middlewares/ThreadX/Low_Power_Support/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DUSE_HAL_DRIVER -DSTM32U5A5xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../Core/Inc -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Include -I../Core/ThreadSafe -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ThreadX-2f-Low_Power_Support

clean-Middlewares-2f-ThreadX-2f-Low_Power_Support:
	-$(RM) ./Middlewares/ThreadX/Low_Power_Support/tx_low_power.cyclo ./Middlewares/ThreadX/Low_Power_Support/tx_low_power.d ./Middlewares/ThreadX/Low_Power_Support/tx_low_power.o ./Middlewares/ThreadX/Low_Power_Support/tx_low_power.su

.PHONY: clean-Middlewares-2f-ThreadX-2f-Low_Power_Support

