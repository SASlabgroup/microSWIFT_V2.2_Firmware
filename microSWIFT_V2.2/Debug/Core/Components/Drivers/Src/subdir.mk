################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/Drivers/Src/pcf2131_reg.c \
../Core/Components/Drivers/Src/u_ubx_protocol.c 

OBJS += \
./Core/Components/Drivers/Src/pcf2131_reg.o \
./Core/Components/Drivers/Src/u_ubx_protocol.o 

C_DEPS += \
./Core/Components/Drivers/Src/pcf2131_reg.d \
./Core/Components/Drivers/Src/u_ubx_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/Drivers/Src/%.o Core/Components/Drivers/Src/%.su Core/Components/Drivers/Src/%.cyclo: ../Core/Components/Drivers/Src/%.c Core/Components/Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DUSE_HAL_DRIVER -DSTM32U5A5xx -DSTM32_THREAD_SAFE_STRATEGY=2 -DFX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Include -I../Core/ThreadSafe -I../FileX/App -I../FileX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/ports/generic/inc -I"/Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Inc" -I"/Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-Drivers-2f-Src

clean-Core-2f-Components-2f-Drivers-2f-Src:
	-$(RM) ./Core/Components/Drivers/Src/pcf2131_reg.cyclo ./Core/Components/Drivers/Src/pcf2131_reg.d ./Core/Components/Drivers/Src/pcf2131_reg.o ./Core/Components/Drivers/Src/pcf2131_reg.su ./Core/Components/Drivers/Src/u_ubx_protocol.cyclo ./Core/Components/Drivers/Src/u_ubx_protocol.d ./Core/Components/Drivers/Src/u_ubx_protocol.o ./Core/Components/Drivers/Src/u_ubx_protocol.su

.PHONY: clean-Core-2f-Components-2f-Drivers-2f-Src

