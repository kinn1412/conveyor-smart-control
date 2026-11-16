################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/app/bringup.c \
../App/app/conveyor.c \
../App/app/safety.c \
../App/app/tasks.c 

OBJS += \
./App/app/bringup.o \
./App/app/conveyor.o \
./App/app/safety.o \
./App/app/tasks.o 

C_DEPS += \
./App/app/bringup.d \
./App/app/conveyor.d \
./App/app/safety.d \
./App/app/tasks.d 


# Each subdirectory must supply rules for building sources it contributes
App/app/%.o App/app/%.su App/app/%.cyclo: ../App/app/%.c App/app/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../App -I../App/app -I../App/ctrl -I../App/comm -I../App/drv -I../shared/proto -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-app

clean-App-2f-app:
	-$(RM) ./App/app/bringup.cyclo ./App/app/bringup.d ./App/app/bringup.o ./App/app/bringup.su ./App/app/conveyor.cyclo ./App/app/conveyor.d ./App/app/conveyor.o ./App/app/conveyor.su ./App/app/safety.cyclo ./App/app/safety.d ./App/app/safety.o ./App/app/safety.su ./App/app/tasks.cyclo ./App/app/tasks.d ./App/app/tasks.o ./App/app/tasks.su

.PHONY: clean-App-2f-app

