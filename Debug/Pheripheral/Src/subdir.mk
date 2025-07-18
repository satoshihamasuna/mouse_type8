################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Pheripheral/Src/communicate.cpp 

C_SRCS += \
../Pheripheral/Src/encoder.c \
../Pheripheral/Src/imu.c \
../Pheripheral/Src/interface.c \
../Pheripheral/Src/ir_sensor.c \
../Pheripheral/Src/motor.c 

C_DEPS += \
./Pheripheral/Src/encoder.d \
./Pheripheral/Src/imu.d \
./Pheripheral/Src/interface.d \
./Pheripheral/Src/ir_sensor.d \
./Pheripheral/Src/motor.d 

OBJS += \
./Pheripheral/Src/communicate.o \
./Pheripheral/Src/encoder.o \
./Pheripheral/Src/imu.o \
./Pheripheral/Src/interface.o \
./Pheripheral/Src/ir_sensor.o \
./Pheripheral/Src/motor.o 

CPP_DEPS += \
./Pheripheral/Src/communicate.d 


# Each subdirectory must supply rules for building sources it contributes
Pheripheral/Src/%.o Pheripheral/Src/%.su Pheripheral/Src/%.cyclo: ../Pheripheral/Src/%.cpp Pheripheral/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Pheripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Pheripheral/Src/%.o Pheripheral/Src/%.su Pheripheral/Src/%.cyclo: ../Pheripheral/Src/%.c Pheripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Pheripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Pheripheral-2f-Src

clean-Pheripheral-2f-Src:
	-$(RM) ./Pheripheral/Src/communicate.cyclo ./Pheripheral/Src/communicate.d ./Pheripheral/Src/communicate.o ./Pheripheral/Src/communicate.su ./Pheripheral/Src/encoder.cyclo ./Pheripheral/Src/encoder.d ./Pheripheral/Src/encoder.o ./Pheripheral/Src/encoder.su ./Pheripheral/Src/imu.cyclo ./Pheripheral/Src/imu.d ./Pheripheral/Src/imu.o ./Pheripheral/Src/imu.su ./Pheripheral/Src/interface.cyclo ./Pheripheral/Src/interface.d ./Pheripheral/Src/interface.o ./Pheripheral/Src/interface.su ./Pheripheral/Src/ir_sensor.cyclo ./Pheripheral/Src/ir_sensor.d ./Pheripheral/Src/ir_sensor.o ./Pheripheral/Src/ir_sensor.su ./Pheripheral/Src/motor.cyclo ./Pheripheral/Src/motor.d ./Pheripheral/Src/motor.o ./Pheripheral/Src/motor.su

.PHONY: clean-Pheripheral-2f-Src

