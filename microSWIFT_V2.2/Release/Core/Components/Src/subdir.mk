################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Components/Src/accelerometer_sensor.c \
../Core/Components/Src/battery.c \
../Core/Components/Src/ct_sensor.c \
../Core/Components/Src/ext_rtc.c \
../Core/Components/Src/ext_rtc_api.c \
../Core/Components/Src/gnss.c \
../Core/Components/Src/iridium.c \
../Core/Components/Src/light_sensor.c \
../Core/Components/Src/logger.c \
../Core/Components/Src/rf_switch.c \
../Core/Components/Src/temp_sensor.c \
../Core/Components/Src/turbidity_sensor.c 

OBJS += \
./Core/Components/Src/accelerometer_sensor.o \
./Core/Components/Src/battery.o \
./Core/Components/Src/ct_sensor.o \
./Core/Components/Src/ext_rtc.o \
./Core/Components/Src/ext_rtc_api.o \
./Core/Components/Src/gnss.o \
./Core/Components/Src/iridium.o \
./Core/Components/Src/light_sensor.o \
./Core/Components/Src/logger.o \
./Core/Components/Src/rf_switch.o \
./Core/Components/Src/temp_sensor.o \
./Core/Components/Src/turbidity_sensor.o 

C_DEPS += \
./Core/Components/Src/accelerometer_sensor.d \
./Core/Components/Src/battery.d \
./Core/Components/Src/ct_sensor.d \
./Core/Components/Src/ext_rtc.d \
./Core/Components/Src/ext_rtc_api.d \
./Core/Components/Src/gnss.d \
./Core/Components/Src/iridium.d \
./Core/Components/Src/light_sensor.d \
./Core/Components/Src/logger.d \
./Core/Components/Src/rf_switch.d \
./Core/Components/Src/temp_sensor.d \
./Core/Components/Src/turbidity_sensor.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Components/Src/%.o Core/Components/Src/%.su Core/Components/Src/%.cyclo: ../Core/Components/Src/%.c Core/Components/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DUSE_HAL_DRIVER -DSTM32U5A5xx -DSTM32_THREAD_SAFE_STRATEGY=2 -DFX_INCLUDE_USER_DEFINE_FILE -DLX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I../AZURE_RTOS/App -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Device/ST/STM32U5xx/Include -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/threadx/utility/low_power -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Drivers/CMSIS/Include -I../Core/ThreadSafe -I../FileX/App -I../FileX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/common/inc -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/filex/ports/generic/inc -I"/Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Inc" -I"/microSWIFT_V2.2/Core/Components/Drivers/Inc" -I"/Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Core/Components/Drivers/Inc" -I../LevelX/App -I../LevelX/Target -I/Users/philbush/STM32Cube/Repository/STM32Cube_FW_U5_V1.6.0/Middlewares/ST/levelx/common/inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Components-2f-Src

clean-Core-2f-Components-2f-Src:
	-$(RM) ./Core/Components/Src/accelerometer_sensor.cyclo ./Core/Components/Src/accelerometer_sensor.d ./Core/Components/Src/accelerometer_sensor.o ./Core/Components/Src/accelerometer_sensor.su ./Core/Components/Src/battery.cyclo ./Core/Components/Src/battery.d ./Core/Components/Src/battery.o ./Core/Components/Src/battery.su ./Core/Components/Src/ct_sensor.cyclo ./Core/Components/Src/ct_sensor.d ./Core/Components/Src/ct_sensor.o ./Core/Components/Src/ct_sensor.su ./Core/Components/Src/ext_rtc.cyclo ./Core/Components/Src/ext_rtc.d ./Core/Components/Src/ext_rtc.o ./Core/Components/Src/ext_rtc.su ./Core/Components/Src/ext_rtc_api.cyclo ./Core/Components/Src/ext_rtc_api.d ./Core/Components/Src/ext_rtc_api.o ./Core/Components/Src/ext_rtc_api.su ./Core/Components/Src/gnss.cyclo ./Core/Components/Src/gnss.d ./Core/Components/Src/gnss.o ./Core/Components/Src/gnss.su ./Core/Components/Src/iridium.cyclo ./Core/Components/Src/iridium.d ./Core/Components/Src/iridium.o ./Core/Components/Src/iridium.su ./Core/Components/Src/light_sensor.cyclo ./Core/Components/Src/light_sensor.d ./Core/Components/Src/light_sensor.o ./Core/Components/Src/light_sensor.su ./Core/Components/Src/logger.cyclo ./Core/Components/Src/logger.d ./Core/Components/Src/logger.o ./Core/Components/Src/logger.su ./Core/Components/Src/rf_switch.cyclo ./Core/Components/Src/rf_switch.d ./Core/Components/Src/rf_switch.o ./Core/Components/Src/rf_switch.su ./Core/Components/Src/temp_sensor.cyclo ./Core/Components/Src/temp_sensor.d ./Core/Components/Src/temp_sensor.o ./Core/Components/Src/temp_sensor.su ./Core/Components/Src/turbidity_sensor.cyclo ./Core/Components/Src/turbidity_sensor.d ./Core/Components/Src/turbidity_sensor.o ./Core/Components/Src/turbidity_sensor.su

.PHONY: clean-Core-2f-Components-2f-Src

