################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../App/ctrl/lpf.c \
../App/ctrl/pid.c \
../App/ctrl/traj.c 

OBJS += \
./App/ctrl/lpf.o \
./App/ctrl/pid.o \
./App/ctrl/traj.o 

C_DEPS += \
./App/ctrl/lpf.d \
./App/ctrl/pid.d \
./App/ctrl/traj.d 


# Each subdirectory must supply rules for building sources it contributes
App/ctrl/%.o App/ctrl/%.su App/ctrl/%.cyclo: ../App/ctrl/%.c App/ctrl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../App -I../App/app -I../App/ctrl -I../App/comm -I../App/drv -I../shared/proto -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-App-2f-ctrl

clean-App-2f-ctrl:
	-$(RM) ./App/ctrl/lpf.cyclo ./App/ctrl/lpf.d ./App/ctrl/lpf.o ./App/ctrl/lpf.su ./App/ctrl/pid.cyclo ./App/ctrl/pid.d ./App/ctrl/pid.o ./App/ctrl/pid.su ./App/ctrl/traj.cyclo ./App/ctrl/traj.d ./App/ctrl/traj.o ./App/ctrl/traj.su

.PHONY: clean-App-2f-ctrl

