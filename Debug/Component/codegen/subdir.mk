################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Component/codegen/rtwhalf.cpp 

OBJS += \
./Component/codegen/rtwhalf.o 

CPP_DEPS += \
./Component/codegen/rtwhalf.d 


# Each subdirectory must supply rules for building sources it contributes
Component/codegen/%.o Component/codegen/%.su Component/codegen/%.cyclo: ../Component/codegen/%.cpp Component/codegen/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Peripheral/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Component-2f-codegen

clean-Component-2f-codegen:
	-$(RM) ./Component/codegen/rtwhalf.cyclo ./Component/codegen/rtwhalf.d ./Component/codegen/rtwhalf.o ./Component/codegen/rtwhalf.su

.PHONY: clean-Component-2f-codegen

