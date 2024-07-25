################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AZURE_RTOS/App/app_azure_rtos.c 

OBJS += \
./AZURE_RTOS/App/app_azure_rtos.o 

C_DEPS += \
./AZURE_RTOS/App/app_azure_rtos.d 


# Each subdirectory must supply rules for building sources it contributes
AZURE_RTOS/App/%.o AZURE_RTOS/App/%.su AZURE_RTOS/App/%.cyclo: ../AZURE_RTOS/App/%.c AZURE_RTOS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DUSE_HAL_DRIVER -DSTM32U5A5xx -DSTM32_THREAD_SAFE_STRATEGY=2 -DFX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Include -I../Core/ThreadSafe -I../FileX/App -I../FileX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/ports/generic/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-AZURE_RTOS-2f-App

clean-AZURE_RTOS-2f-App:
	-$(RM) ./AZURE_RTOS/App/app_azure_rtos.cyclo ./AZURE_RTOS/App/app_azure_rtos.d ./AZURE_RTOS/App/app_azure_rtos.o ./AZURE_RTOS/App/app_azure_rtos.su

.PHONY: clean-AZURE_RTOS-2f-App

