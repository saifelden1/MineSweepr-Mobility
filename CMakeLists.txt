cmake_minimum_required(VERSION 3.16)
project(STM32_Mobility C ASM)
set(CMAKE_C_STANDARD 11)
set(HAL Drivers/STM32F4xx_HAL_Driver)
set(UROS ${CMAKE_CURRENT_SOURCE_DIR}/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros)
if(NOT EXISTS ${UROS}/libmicroros.a)
    message(FATAL_ERROR "Run scripts/prepare_microros.sh before configuring mobility firmware")
endif()
file(GLOB APP Core/Src/*.c)
file(GLOB HAL_SRC ${HAL}/Src/*.c)
add_executable(${PROJECT_NAME}.elf ${APP} ${HAL_SRC} Core/Startup/startup_stm32f411ceux.s)
target_include_directories(${PROJECT_NAME}.elf PRIVATE Core/Inc ${HAL}/Inc Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include ${UROS}/microros_include)
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE USE_HAL_DRIVER STM32F411xE)
target_compile_options(${PROJECT_NAME}.elf PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16
    -mfloat-abi=hard -Os -g3 -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(${PROJECT_NAME}.elf PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16
    -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F411CEUX_FLASH.ld
    -Wl,--gc-sections,-Map=${PROJECT_NAME}.map,--print-memory-usage)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE ${UROS}/libmicroros.a m)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}.elf>)
