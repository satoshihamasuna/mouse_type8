################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Component/ntshell-v0.3.1/src/lib/core/ntlibc.c \
../Component/ntshell-v0.3.1/src/lib/core/ntshell.c \
../Component/ntshell-v0.3.1/src/lib/core/text_editor.c \
../Component/ntshell-v0.3.1/src/lib/core/text_history.c \
../Component/ntshell-v0.3.1/src/lib/core/vtrecv.c \
../Component/ntshell-v0.3.1/src/lib/core/vtsend.c 

C_DEPS += \
./Component/ntshell-v0.3.1/src/lib/core/ntlibc.d \
./Component/ntshell-v0.3.1/src/lib/core/ntshell.d \
./Component/ntshell-v0.3.1/src/lib/core/text_editor.d \
./Component/ntshell-v0.3.1/src/lib/core/text_history.d \
./Component/ntshell-v0.3.1/src/lib/core/vtrecv.d \
./Component/ntshell-v0.3.1/src/lib/core/vtsend.d 

OBJS += \
./Component/ntshell-v0.3.1/src/lib/core/ntlibc.o \
./Component/ntshell-v0.3.1/src/lib/core/ntshell.o \
./Component/ntshell-v0.3.1/src/lib/core/text_editor.o \
./Component/ntshell-v0.3.1/src/lib/core/text_history.o \
./Component/ntshell-v0.3.1/src/lib/core/vtrecv.o \
./Component/ntshell-v0.3.1/src/lib/core/vtsend.o 


# Each subdirectory must supply rules for building sources it contributes
Component/ntshell-v0.3.1/src/lib/core/%.o Component/ntshell-v0.3.1/src/lib/core/%.su Component/ntshell-v0.3.1/src/lib/core/%.cyclo: ../Component/ntshell-v0.3.1/src/lib/core/%.c Component/ntshell-v0.3.1/src/lib/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Pheripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-core

clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-core:
	-$(RM) ./Component/ntshell-v0.3.1/src/lib/core/ntlibc.cyclo ./Component/ntshell-v0.3.1/src/lib/core/ntlibc.d ./Component/ntshell-v0.3.1/src/lib/core/ntlibc.o ./Component/ntshell-v0.3.1/src/lib/core/ntlibc.su ./Component/ntshell-v0.3.1/src/lib/core/ntshell.cyclo ./Component/ntshell-v0.3.1/src/lib/core/ntshell.d ./Component/ntshell-v0.3.1/src/lib/core/ntshell.o ./Component/ntshell-v0.3.1/src/lib/core/ntshell.su ./Component/ntshell-v0.3.1/src/lib/core/text_editor.cyclo ./Component/ntshell-v0.3.1/src/lib/core/text_editor.d ./Component/ntshell-v0.3.1/src/lib/core/text_editor.o ./Component/ntshell-v0.3.1/src/lib/core/text_editor.su ./Component/ntshell-v0.3.1/src/lib/core/text_history.cyclo ./Component/ntshell-v0.3.1/src/lib/core/text_history.d ./Component/ntshell-v0.3.1/src/lib/core/text_history.o ./Component/ntshell-v0.3.1/src/lib/core/text_history.su ./Component/ntshell-v0.3.1/src/lib/core/vtrecv.cyclo ./Component/ntshell-v0.3.1/src/lib/core/vtrecv.d ./Component/ntshell-v0.3.1/src/lib/core/vtrecv.o ./Component/ntshell-v0.3.1/src/lib/core/vtrecv.su ./Component/ntshell-v0.3.1/src/lib/core/vtsend.cyclo ./Component/ntshell-v0.3.1/src/lib/core/vtsend.d ./Component/ntshell-v0.3.1/src/lib/core/vtsend.o ./Component/ntshell-v0.3.1/src/lib/core/vtsend.su

.PHONY: clean-Component-2f-ntshell-2d-v0-2e-3-2e-1-2f-src-2f-lib-2f-core

