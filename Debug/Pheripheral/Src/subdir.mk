################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Pheripheral/Src/ir_sensor.c 

C_DEPS += \
./Pheripheral/Src/ir_sensor.d 

OBJS += \
./Pheripheral/Src/ir_sensor.o 


# Each subdirectory must supply rules for building sources it contributes
Pheripheral/Src/%.o Pheripheral/Src/%.su Pheripheral/Src/%.cyclo: ../Pheripheral/Src/%.c Pheripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Pheripheral/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Pheripheral-2f-Src

clean-Pheripheral-2f-Src:
	-$(RM) ./Pheripheral/Src/ir_sensor.cyclo ./Pheripheral/Src/ir_sensor.d ./Pheripheral/Src/ir_sensor.o ./Pheripheral/Src/ir_sensor.su

.PHONY: clean-Pheripheral-2f-Src

