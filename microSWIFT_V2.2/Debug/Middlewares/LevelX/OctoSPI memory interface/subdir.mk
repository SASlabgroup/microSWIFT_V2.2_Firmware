################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/levelx/common/drivers/lx_stm32_ospi_driver.c 

OBJS += \
./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.o 

C_DEPS += \
./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.o: /Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/levelx/common/drivers/lx_stm32_ospi_driver.c Middlewares/LevelX/OctoSPI\ memory\ interface/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DUSE_HAL_DRIVER -DSTM32U5A5xx -DSTM32_THREAD_SAFE_STRATEGY=2 -DFX_INCLUDE_USER_DEFINE_FILE -DLX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Include -I../Core/ThreadSafe -I../FileX/App -I../FileX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/ports/generic/inc -I"/Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Inc" -I"/Users/philbush/STM32CubeIDE/workspace_1.14.0/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Drivers/Inc" -I../LevelX/App -I../LevelX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/levelx/common/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/LevelX/OctoSPI memory interface/lx_stm32_ospi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-LevelX-2f-OctoSPI-20-memory-20-interface

clean-Middlewares-2f-LevelX-2f-OctoSPI-20-memory-20-interface:
	-$(RM) ./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.cyclo ./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.d ./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.o ./Middlewares/LevelX/OctoSPI\ memory\ interface/lx_stm32_ospi_driver.su

.PHONY: clean-Middlewares-2f-LevelX-2f-OctoSPI-20-memory-20-interface

