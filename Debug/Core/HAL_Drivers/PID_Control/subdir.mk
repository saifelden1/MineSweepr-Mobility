################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HAL_Drivers/PID_Control/PID_Control.c 

OBJS += \
./Core/HAL_Drivers/PID_Control/PID_Control.o 

C_DEPS += \
./Core/HAL_Drivers/PID_Control/PID_Control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HAL_Drivers/PID_Control/%.o Core/HAL_Drivers/PID_Control/%.su Core/HAL_Drivers/PID_Control/%.cyclo: ../Core/HAL_Drivers/PID_Control/%.c Core/HAL_Drivers/PID_Control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Inc/interfaces -I../Core/BSP -I../Core/Middleware/control -I../Core/Middleware/microros -I../Core/Tasks -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HAL_Drivers-2f-PID_Control

clean-Core-2f-HAL_Drivers-2f-PID_Control:
	-$(RM) ./Core/HAL_Drivers/PID_Control/PID_Control.cyclo ./Core/HAL_Drivers/PID_Control/PID_Control.d ./Core/HAL_Drivers/PID_Control/PID_Control.o ./Core/HAL_Drivers/PID_Control/PID_Control.su

.PHONY: clean-Core-2f-HAL_Drivers-2f-PID_Control

