################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/MotorControl/MotorControl.c 

OBJS += \
./System/MotorControl/MotorControl.o 

C_DEPS += \
./System/MotorControl/MotorControl.d 


# Each subdirectory must supply rules for building sources it contributes
System/MotorControl/%.o System/MotorControl/%.su System/MotorControl/%.cyclo: ../System/MotorControl/%.c System/MotorControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/max/Projects/cube_12_v4/bldc-motor-control/System" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-MotorControl

clean-System-2f-MotorControl:
	-$(RM) ./System/MotorControl/MotorControl.cyclo ./System/MotorControl/MotorControl.d ./System/MotorControl/MotorControl.o ./System/MotorControl/MotorControl.su

.PHONY: clean-System-2f-MotorControl

