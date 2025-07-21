################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Module/Src/flash.cpp \
../Module/Src/interrupt.cpp \
../Module/Src/log_data.cpp 

OBJS += \
./Module/Src/flash.o \
./Module/Src/interrupt.o \
./Module/Src/log_data.o 

CPP_DEPS += \
./Module/Src/flash.d \
./Module/Src/interrupt.d \
./Module/Src/log_data.d 


# Each subdirectory must supply rules for building sources it contributes
Module/Src/%.o Module/Src/%.su Module/Src/%.cyclo: ../Module/Src/%.cpp Module/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"D:/mouse8/mouse_program/mouse_type8/Component/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Component/ntshell-v0.3.1/src/lib" -I"D:/mouse8/mouse_program/mouse_type8/Peripheral/Inc" -I"D:/mouse8/mouse_program/mouse_type8/System/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Params" -I"D:/mouse8/mouse_program/mouse_type8/Module/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Subsys/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Module-2f-Src

clean-Module-2f-Src:
	-$(RM) ./Module/Src/flash.cyclo ./Module/Src/flash.d ./Module/Src/flash.o ./Module/Src/flash.su ./Module/Src/interrupt.cyclo ./Module/Src/interrupt.d ./Module/Src/interrupt.o ./Module/Src/interrupt.su ./Module/Src/log_data.cyclo ./Module/Src/log_data.d ./Module/Src/log_data.o ./Module/Src/log_data.su

.PHONY: clean-Module-2f-Src

