################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../shared/proto/frame.c 

OBJS += \
./shared/proto/frame.o 

C_DEPS += \
./shared/proto/frame.d 


# Each subdirectory must supply rules for building sources it contributes
shared/proto/%.o shared/proto/%.su shared/proto/%.cyclo: ../shared/proto/%.c shared/proto/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../App -I../App/app -I../App/ctrl -I../App/comm -I../App/drv -I../shared/proto -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-shared-2f-proto

clean-shared-2f-proto:
	-$(RM) ./shared/proto/frame.cyclo ./shared/proto/frame.d ./shared/proto/frame.o ./shared/proto/frame.su

.PHONY: clean-shared-2f-proto

