################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Src/debug.cpp \
../System/Src/debug2.cpp \
../System/Src/debug3.cpp \
../System/Src/demo.cpp \
../System/Src/demo2.cpp \
../System/Src/interface_check.cpp \
../System/Src/mode.cpp \
../System/Src/myshell.cpp \
../System/Src/search_time_select.cpp \
../System/Src/system_util.cpp \
../System/Src/wall_histry_check.cpp 

OBJS += \
./System/Src/debug.o \
./System/Src/debug2.o \
./System/Src/debug3.o \
./System/Src/demo.o \
./System/Src/demo2.o \
./System/Src/interface_check.o \
./System/Src/mode.o \
./System/Src/myshell.o \
./System/Src/search_time_select.o \
./System/Src/system_util.o \
./System/Src/wall_histry_check.o 

CPP_DEPS += \
./System/Src/debug.d \
./System/Src/debug2.d \
./System/Src/debug3.d \
./System/Src/demo.d \
./System/Src/demo2.d \
./System/Src/interface_check.d \
./System/Src/mode.d \
./System/Src/myshell.d \
./System/Src/search_time_select.d \
./System/Src/system_util.d \
./System/Src/wall_histry_check.d 


# Each subdirectory must supply rules for building sources it contributes
System/Src/%.o System/Src/%.su System/Src/%.cyclo: ../System/Src/%.cpp System/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Peripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/System/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Params" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Module/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Subsys/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Src

clean-System-2f-Src:
	-$(RM) ./System/Src/debug.cyclo ./System/Src/debug.d ./System/Src/debug.o ./System/Src/debug.su ./System/Src/debug2.cyclo ./System/Src/debug2.d ./System/Src/debug2.o ./System/Src/debug2.su ./System/Src/debug3.cyclo ./System/Src/debug3.d ./System/Src/debug3.o ./System/Src/debug3.su ./System/Src/demo.cyclo ./System/Src/demo.d ./System/Src/demo.o ./System/Src/demo.su ./System/Src/demo2.cyclo ./System/Src/demo2.d ./System/Src/demo2.o ./System/Src/demo2.su ./System/Src/interface_check.cyclo ./System/Src/interface_check.d ./System/Src/interface_check.o ./System/Src/interface_check.su ./System/Src/mode.cyclo ./System/Src/mode.d ./System/Src/mode.o ./System/Src/mode.su ./System/Src/myshell.cyclo ./System/Src/myshell.d ./System/Src/myshell.o ./System/Src/myshell.su ./System/Src/search_time_select.cyclo ./System/Src/search_time_select.d ./System/Src/search_time_select.o ./System/Src/search_time_select.su ./System/Src/system_util.cyclo ./System/Src/system_util.d ./System/Src/system_util.o ./System/Src/system_util.su ./System/Src/wall_histry_check.cyclo ./System/Src/wall_histry_check.d ./System/Src/wall_histry_check.o ./System/Src/wall_histry_check.su

.PHONY: clean-System-2f-Src

