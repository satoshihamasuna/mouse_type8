################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Component/Src/controller.cpp \
../Component/Src/half_float.cpp 

OBJS += \
./Component/Src/controller.o \
./Component/Src/half_float.o 

CPP_DEPS += \
./Component/Src/controller.d \
./Component/Src/half_float.d 


# Each subdirectory must supply rules for building sources it contributes
Component/Src/%.o Component/Src/%.su Component/Src/%.cyclo: ../Component/Src/%.cpp Component/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"D:/mouse8/mouse_program/mouse_type8/Component/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Component/ntshell-v0.3.1/src/lib" -I"D:/mouse8/mouse_program/mouse_type8/Peripheral/Inc" -I"D:/mouse8/mouse_program/mouse_type8/System/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Params" -I"D:/mouse8/mouse_program/mouse_type8/Module/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Subsys/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Component-2f-Src

clean-Component-2f-Src:
	-$(RM) ./Component/Src/controller.cyclo ./Component/Src/controller.d ./Component/Src/controller.o ./Component/Src/controller.su ./Component/Src/half_float.cyclo ./Component/Src/half_float.d ./Component/Src/half_float.o ./Component/Src/half_float.su

.PHONY: clean-Component-2f-Src

