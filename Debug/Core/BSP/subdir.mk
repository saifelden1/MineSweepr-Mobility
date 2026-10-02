################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/BSP/cytron_mdd10a_driver.c \
../Core/BSP/mpu6050_driver.c \
../Core/BSP/neo6m_driver.c \
../Core/BSP/tim_encoder_driver.c 

OBJS += \
./Core/BSP/cytron_mdd10a_driver.o \
./Core/BSP/mpu6050_driver.o \
./Core/BSP/neo6m_driver.o \
./Core/BSP/tim_encoder_driver.o 

C_DEPS += \
./Core/BSP/cytron_mdd10a_driver.d \
./Core/BSP/mpu6050_driver.d \
./Core/BSP/neo6m_driver.d \
./Core/BSP/tim_encoder_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/BSP/%.o Core/BSP/%.su Core/BSP/%.cyclo: ../Core/BSP/%.c Core/BSP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Core/Inc/interfaces -I../Core/BSP -I../Core/Middleware/control -I../Core/Middleware/microros -I../Core/Tasks -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-BSP

clean-Core-2f-BSP:
	-$(RM) ./Core/BSP/cytron_mdd10a_driver.cyclo ./Core/BSP/cytron_mdd10a_driver.d ./Core/BSP/cytron_mdd10a_driver.o ./Core/BSP/cytron_mdd10a_driver.su ./Core/BSP/mpu6050_driver.cyclo ./Core/BSP/mpu6050_driver.d ./Core/BSP/mpu6050_driver.o ./Core/BSP/mpu6050_driver.su ./Core/BSP/neo6m_driver.cyclo ./Core/BSP/neo6m_driver.d ./Core/BSP/neo6m_driver.o ./Core/BSP/neo6m_driver.su ./Core/BSP/tim_encoder_driver.cyclo ./Core/BSP/tim_encoder_driver.d ./Core/BSP/tim_encoder_driver.o ./Core/BSP/tim_encoder_driver.su

.PHONY: clean-Core-2f-BSP

