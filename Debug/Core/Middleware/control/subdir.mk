################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Middleware/control/kinematics.c \
../Core/Middleware/control/odometry.c \
../Core/Middleware/control/pid_controller.c 

OBJS += \
./Core/Middleware/control/kinematics.o \
./Core/Middleware/control/odometry.o \
./Core/Middleware/control/pid_controller.o 

C_DEPS += \
./Core/Middleware/control/kinematics.d \
./Core/Middleware/control/odometry.d \
./Core/Middleware/control/pid_controller.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Middleware/control/%.o Core/Middleware/control/%.su Core/Middleware/control/%.cyclo: ../Core/Middleware/control/%.c Core/Middleware/control/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Inc/interfaces -I../Core/BSP -I../Core/Middleware/control -I../Core/Middleware/microros -I../Core/Tasks -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Middleware-2f-control

clean-Core-2f-Middleware-2f-control:
	-$(RM) ./Core/Middleware/control/kinematics.cyclo ./Core/Middleware/control/kinematics.d ./Core/Middleware/control/kinematics.o ./Core/Middleware/control/kinematics.su ./Core/Middleware/control/odometry.cyclo ./Core/Middleware/control/odometry.d ./Core/Middleware/control/odometry.o ./Core/Middleware/control/odometry.su ./Core/Middleware/control/pid_controller.cyclo ./Core/Middleware/control/pid_controller.d ./Core/Middleware/control/pid_controller.o ./Core/Middleware/control/pid_controller.su

.PHONY: clean-Core-2f-Middleware-2f-control

