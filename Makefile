# Also supplies ABI flags to the official micro-ROS library builder.
.PHONY: all print_cflags clean
all:
	cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc.cmake
	cmake --build build
print_cflags:
	@echo -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Os -ffunction-sections -fdata-sections -DSTM32F411xE -DUSE_HAL_DRIVER
clean:
	cmake --build build --target clean
