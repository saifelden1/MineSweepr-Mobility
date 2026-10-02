################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HAL_Drivers/Encoders/Encoder.c 

OBJS += \
./Core/HAL_Drivers/Encoders/Encoder.o 

C_DEPS += \
./Core/HAL_Drivers/Encoders/Encoder.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HAL_Drivers/Encoders/%.o Core/HAL_Drivers/Encoders/%.su Core/HAL_Drivers/Encoders/%.cyclo: ../Core/HAL_Drivers/Encoders/%.c Core/HAL_Drivers/Encoders/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HAL_Drivers-2f-Encoders

clean-Core-2f-HAL_Drivers-2f-Encoders:
	-$(RM) ./Core/HAL_Drivers/Encoders/Encoder.cyclo ./Core/HAL_Drivers/Encoders/Encoder.d ./Core/HAL_Drivers/Encoders/Encoder.o ./Core/HAL_Drivers/Encoders/Encoder.su

.PHONY: clean-Core-2f-HAL_Drivers-2f-Encoders

