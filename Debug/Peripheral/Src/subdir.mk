################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Peripheral/Src/communicate.cpp 

C_SRCS += \
../Peripheral/Src/battery.c \
../Peripheral/Src/current_sens.c \
../Peripheral/Src/encoder.c \
../Peripheral/Src/flash_util.c \
../Peripheral/Src/imu.c \
../Peripheral/Src/interface.c \
../Peripheral/Src/ir_sensor.c \
../Peripheral/Src/motor.c 

C_DEPS += \
./Peripheral/Src/battery.d \
./Peripheral/Src/current_sens.d \
./Peripheral/Src/encoder.d \
./Peripheral/Src/flash_util.d \
./Peripheral/Src/imu.d \
./Peripheral/Src/interface.d \
./Peripheral/Src/ir_sensor.d \
./Peripheral/Src/motor.d 

OBJS += \
./Peripheral/Src/battery.o \
./Peripheral/Src/communicate.o \
./Peripheral/Src/current_sens.o \
./Peripheral/Src/encoder.o \
./Peripheral/Src/flash_util.o \
./Peripheral/Src/imu.o \
./Peripheral/Src/interface.o \
./Peripheral/Src/ir_sensor.o \
./Peripheral/Src/motor.o 

CPP_DEPS += \
./Peripheral/Src/communicate.d 


# Each subdirectory must supply rules for building sources it contributes
Peripheral/Src/%.o Peripheral/Src/%.su Peripheral/Src/%.cyclo: ../Peripheral/Src/%.c Peripheral/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"D:/mouse8/mouse_program/mouse_type8/Component/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Component/ntshell-v0.3.1/src/lib" -I"D:/mouse8/mouse_program/mouse_type8/Peripheral/Inc" -I"D:/mouse8/mouse_program/mouse_type8/System/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Task/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Module/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Subsys/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Params" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Peripheral/Src/%.o Peripheral/Src/%.su Peripheral/Src/%.cyclo: ../Peripheral/Src/%.cpp Peripheral/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"D:/mouse8/mouse_program/mouse_type8/Component/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Component/ntshell-v0.3.1/src/lib" -I"D:/mouse8/mouse_program/mouse_type8/Peripheral/Inc" -I"D:/mouse8/mouse_program/mouse_type8/System/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Params" -I"D:/mouse8/mouse_program/mouse_type8/Module/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Subsys/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripheral-2f-Src

clean-Peripheral-2f-Src:
	-$(RM) ./Peripheral/Src/battery.cyclo ./Peripheral/Src/battery.d ./Peripheral/Src/battery.o ./Peripheral/Src/battery.su ./Peripheral/Src/communicate.cyclo ./Peripheral/Src/communicate.d ./Peripheral/Src/communicate.o ./Peripheral/Src/communicate.su ./Peripheral/Src/current_sens.cyclo ./Peripheral/Src/current_sens.d ./Peripheral/Src/current_sens.o ./Peripheral/Src/current_sens.su ./Peripheral/Src/encoder.cyclo ./Peripheral/Src/encoder.d ./Peripheral/Src/encoder.o ./Peripheral/Src/encoder.su ./Peripheral/Src/flash_util.cyclo ./Peripheral/Src/flash_util.d ./Peripheral/Src/flash_util.o ./Peripheral/Src/flash_util.su ./Peripheral/Src/imu.cyclo ./Peripheral/Src/imu.d ./Peripheral/Src/imu.o ./Peripheral/Src/imu.su ./Peripheral/Src/interface.cyclo ./Peripheral/Src/interface.d ./Peripheral/Src/interface.o ./Peripheral/Src/interface.su ./Peripheral/Src/ir_sensor.cyclo ./Peripheral/Src/ir_sensor.d ./Peripheral/Src/ir_sensor.o ./Peripheral/Src/ir_sensor.su ./Peripheral/Src/motor.cyclo ./Peripheral/Src/motor.d ./Peripheral/Src/motor.o ./Peripheral/Src/motor.su

.PHONY: clean-Peripheral-2f-Src

