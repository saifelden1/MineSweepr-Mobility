################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Tasks/task_microros.c \
../Core/Tasks/task_motor_control.c \
../Core/Tasks/task_safety_watchdog.c \
../Core/Tasks/task_sensor_acq.c 

OBJS += \
./Core/Tasks/task_microros.o \
./Core/Tasks/task_motor_control.o \
./Core/Tasks/task_safety_watchdog.o \
./Core/Tasks/task_sensor_acq.o 

C_DEPS += \
./Core/Tasks/task_microros.d \
./Core/Tasks/task_motor_control.d \
./Core/Tasks/task_safety_watchdog.d \
./Core/Tasks/task_sensor_acq.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Tasks/%.o Core/Tasks/%.su Core/Tasks/%.cyclo: ../Core/Tasks/%.c Core/Tasks/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Inc/interfaces -I../Core/BSP -I../Core/Middleware/control -I../Core/Middleware/microros -I../Core/Tasks -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Tasks

clean-Core-2f-Tasks:
	-$(RM) ./Core/Tasks/task_microros.cyclo ./Core/Tasks/task_microros.d ./Core/Tasks/task_microros.o ./Core/Tasks/task_microros.su ./Core/Tasks/task_motor_control.cyclo ./Core/Tasks/task_motor_control.d ./Core/Tasks/task_motor_control.o ./Core/Tasks/task_motor_control.su ./Core/Tasks/task_safety_watchdog.cyclo ./Core/Tasks/task_safety_watchdog.d ./Core/Tasks/task_safety_watchdog.o ./Core/Tasks/task_safety_watchdog.su ./Core/Tasks/task_sensor_acq.cyclo ./Core/Tasks/task_sensor_acq.d ./Core/Tasks/task_sensor_acq.o ./Core/Tasks/task_sensor_acq.su

.PHONY: clean-Core-2f-Tasks

