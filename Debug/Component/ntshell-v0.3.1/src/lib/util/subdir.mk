################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Component/ntshell-v0.3.1/src/lib/util/ntopt.c \
../Component/ntshell-v0.3.1/src/lib/util/ntstdio.c 

C_DEPS += \
./Component/ntshell-v0.3.1/src/lib/util/ntopt.d \
./Component/ntshell-v0.3.1/src/lib/util/ntstdio.d 

OBJS += \
./Component/ntshell-v0.3.1/src/lib/util/ntopt.o \
./Component/ntshell-v0.3.1/src/lib/util/ntstdio.o 


# Each subdirectory must supply rules for building sources it contributes
Component/ntshell-v0.3.1/src/lib/util/%.o Component/ntshell-v0.3.1/src/lib/util/%.su Component/ntshell-v0.3.1/src/lib/util/%.cyclo: ../Component/ntshell-v0.3.1/src/lib/util/%.c Component/ntshell-v0.3.1/src/lib/util/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Peripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/System/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Task/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Module/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Subsys/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Params" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-util

clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-util:
	-$(RM) ./Component/ntshell-v0.3.1/src/lib/util/ntopt.cyclo ./Component/ntshell-v0.3.1/src/lib/util/ntopt.d ./Component/ntshell-v0.3.1/src/lib/util/ntopt.o ./Component/ntshell-v0.3.1/src/lib/util/ntopt.su ./Component/ntshell-v0.3.1/src/lib/util/ntstdio.cyclo ./Component/ntshell-v0.3.1/src/lib/util/ntstdio.d ./Component/ntshell-v0.3.1/src/lib/util/ntstdio.o ./Component/ntshell-v0.3.1/src/lib/util/ntstdio.su

.PHONY: clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-util

