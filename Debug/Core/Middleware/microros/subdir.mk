################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Middleware/microros/microros_client.c \
../Core/Middleware/microros/microros_transport.c 

OBJS += \
./Core/Middleware/microros/microros_client.o \
./Core/Middleware/microros/microros_transport.o 

C_DEPS += \
./Core/Middleware/microros/microros_client.d \
./Core/Middleware/microros/microros_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Middleware/microros/%.o Core/Middleware/microros/%.su Core/Middleware/microros/%.cyclo: ../Core/Middleware/microros/%.c Core/Middleware/microros/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Inc/interfaces -I../Core/BSP -I../Core/Middleware/control -I../Core/Middleware/microros -I../Core/Tasks -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Middleware-2f-microros

clean-Core-2f-Middleware-2f-microros:
	-$(RM) ./Core/Middleware/microros/microros_client.cyclo ./Core/Middleware/microros/microros_client.d ./Core/Middleware/microros/microros_client.o ./Core/Middleware/microros/microros_client.su ./Core/Middleware/microros/microros_transport.cyclo ./Core/Middleware/microros/microros_transport.d ./Core/Middleware/microros/microros_transport.o ./Core/Middleware/microros/microros_transport.su

.PHONY: clean-Core-2f-Middleware-2f-microros

