################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bms_app.c \
../Core/Src/bms_buttons.c \
../Core/Src/bms_lcd.c \
../Core/Src/bms_telemetry.c \
../Core/Src/i2c-lcd.c \
../Core/Src/ina219.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c 

OBJS += \
./Core/Src/bms_app.o \
./Core/Src/bms_buttons.o \
./Core/Src/bms_lcd.o \
./Core/Src/bms_telemetry.o \
./Core/Src/i2c-lcd.o \
./Core/Src/ina219.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o 

C_DEPS += \
./Core/Src/bms_app.d \
./Core/Src/bms_buttons.d \
./Core/Src/bms_lcd.d \
./Core/Src/bms_telemetry.d \
./Core/Src/i2c-lcd.d \
./Core/Src/ina219.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bms_app.cyclo ./Core/Src/bms_app.d ./Core/Src/bms_app.o ./Core/Src/bms_app.su ./Core/Src/bms_buttons.cyclo ./Core/Src/bms_buttons.d ./Core/Src/bms_buttons.o ./Core/Src/bms_buttons.su ./Core/Src/bms_lcd.cyclo ./Core/Src/bms_lcd.d ./Core/Src/bms_lcd.o ./Core/Src/bms_lcd.su ./Core/Src/bms_telemetry.cyclo ./Core/Src/bms_telemetry.d ./Core/Src/bms_telemetry.o ./Core/Src/bms_telemetry.su ./Core/Src/i2c-lcd.cyclo ./Core/Src/i2c-lcd.d ./Core/Src/i2c-lcd.o ./Core/Src/i2c-lcd.su ./Core/Src/ina219.cyclo ./Core/Src/ina219.d ./Core/Src/ina219.o ./Core/Src/ina219.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su

.PHONY: clean-Core-2f-Src

