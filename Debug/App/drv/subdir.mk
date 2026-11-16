################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/drv/encoder.c \
../App/drv/led.c \
../App/drv/motor.c \
../App/drv/servo.c \
../App/drv/timing.c \
../App/drv/uart_link.c 

OBJS += \
./App/drv/encoder.o \
./App/drv/led.o \
./App/drv/motor.o \
./App/drv/servo.o \
./App/drv/timing.o \
./App/drv/uart_link.o 

C_DEPS += \
./App/drv/encoder.d \
./App/drv/led.d \
./App/drv/motor.d \
./App/drv/servo.d \
./App/drv/timing.d \
./App/drv/uart_link.d 


# Each subdirectory must supply rules for building sources it contributes
App/drv/%.o App/drv/%.su App/drv/%.cyclo: ../App/drv/%.c App/drv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../App -I../App/app -I../App/ctrl -I../App/comm -I../App/drv -I../shared/proto -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-drv

clean-App-2f-drv:
	-$(RM) ./App/drv/encoder.cyclo ./App/drv/encoder.d ./App/drv/encoder.o ./App/drv/encoder.su ./App/drv/led.cyclo ./App/drv/led.d ./App/drv/led.o ./App/drv/led.su ./App/drv/motor.cyclo ./App/drv/motor.d ./App/drv/motor.o ./App/drv/motor.su ./App/drv/servo.cyclo ./App/drv/servo.d ./App/drv/servo.o ./App/drv/servo.su ./App/drv/timing.cyclo ./App/drv/timing.d ./App/drv/timing.o ./App/drv/timing.su ./App/drv/uart_link.cyclo ./App/drv/uart_link.d ./App/drv/uart_link.o ./App/drv/uart_link.su

.PHONY: clean-App-2f-drv

