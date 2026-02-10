################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Task/Src/ctrl_task.cpp \
../Task/Src/init_motion.cpp \
../Task/Src/init_motion2.cpp \
../Task/Src/ir_sens_task.cpp \
../Task/Src/motion.cpp 

OBJS += \
./Task/Src/ctrl_task.o \
./Task/Src/init_motion.o \
./Task/Src/init_motion2.o \
./Task/Src/ir_sens_task.o \
./Task/Src/motion.o 

CPP_DEPS += \
./Task/Src/ctrl_task.d \
./Task/Src/init_motion.d \
./Task/Src/init_motion2.d \
./Task/Src/ir_sens_task.d \
./Task/Src/motion.d 


# Each subdirectory must supply rules for building sources it contributes
Task/Src/%.o Task/Src/%.su Task/Src/%.cyclo: ../Task/Src/%.cpp Task/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"D:/mouse8/mouse_program/mouse_type8/Component/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Component/ntshell-v0.3.1/src/lib" -I"D:/mouse8/mouse_program/mouse_type8/Peripheral/Inc" -I"D:/mouse8/mouse_program/mouse_type8/System/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Params" -I"D:/mouse8/mouse_program/mouse_type8/Module/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Subsys/Inc" -I"D:/mouse8/mouse_program/mouse_type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Task-2f-Src

clean-Task-2f-Src:
	-$(RM) ./Task/Src/ctrl_task.cyclo ./Task/Src/ctrl_task.d ./Task/Src/ctrl_task.o ./Task/Src/ctrl_task.su ./Task/Src/init_motion.cyclo ./Task/Src/init_motion.d ./Task/Src/init_motion.o ./Task/Src/init_motion.su ./Task/Src/init_motion2.cyclo ./Task/Src/init_motion2.d ./Task/Src/init_motion2.o ./Task/Src/init_motion2.su ./Task/Src/ir_sens_task.cyclo ./Task/Src/ir_sens_task.d ./Task/Src/ir_sens_task.o ./Task/Src/ir_sens_task.su ./Task/Src/motion.cyclo ./Task/Src/motion.d ./Task/Src/motion.o ./Task/Src/motion.su

.PHONY: clean-Task-2f-Src

