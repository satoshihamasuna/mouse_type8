################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Subsys/Src/adachi.cpp \
../Subsys/Src/calcRunTime.cpp \
../Subsys/Src/make_map.cpp \
../Subsys/Src/make_path.cpp \
../Subsys/Src/make_path_expand.cpp \
../Subsys/Src/search.cpp \
../Subsys/Src/wall_class.cpp 

OBJS += \
./Subsys/Src/adachi.o \
./Subsys/Src/calcRunTime.o \
./Subsys/Src/make_map.o \
./Subsys/Src/make_path.o \
./Subsys/Src/make_path_expand.o \
./Subsys/Src/search.o \
./Subsys/Src/wall_class.o 

CPP_DEPS += \
./Subsys/Src/adachi.d \
./Subsys/Src/calcRunTime.d \
./Subsys/Src/make_map.d \
./Subsys/Src/make_path.d \
./Subsys/Src/make_path_expand.d \
./Subsys/Src/search.d \
./Subsys/Src/wall_class.d 


# Each subdirectory must supply rules for building sources it contributes
Subsys/Src/%.o Subsys/Src/%.su Subsys/Src/%.cyclo: ../Subsys/Src/%.cpp Subsys/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U535xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Component/ntshell-v0.3.1/src/lib" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Peripheral/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/System/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Params" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Module/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Subsys/Inc" -I"C:/Users/sato1/Documents/Git/STM32/program_mouse_type8/type8/Task/Inc" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Subsys-2f-Src

clean-Subsys-2f-Src:
	-$(RM) ./Subsys/Src/adachi.cyclo ./Subsys/Src/adachi.d ./Subsys/Src/adachi.o ./Subsys/Src/adachi.su ./Subsys/Src/calcRunTime.cyclo ./Subsys/Src/calcRunTime.d ./Subsys/Src/calcRunTime.o ./Subsys/Src/calcRunTime.su ./Subsys/Src/make_map.cyclo ./Subsys/Src/make_map.d ./Subsys/Src/make_map.o ./Subsys/Src/make_map.su ./Subsys/Src/make_path.cyclo ./Subsys/Src/make_path.d ./Subsys/Src/make_path.o ./Subsys/Src/make_path.su ./Subsys/Src/make_path_expand.cyclo ./Subsys/Src/make_path_expand.d ./Subsys/Src/make_path_expand.o ./Subsys/Src/make_path_expand.su ./Subsys/Src/search.cyclo ./Subsys/Src/search.d ./Subsys/Src/search.o ./Subsys/Src/search.su ./Subsys/Src/wall_class.cyclo ./Subsys/Src/wall_class.d ./Subsys/Src/wall_class.o ./Subsys/Src/wall_class.su

.PHONY: clean-Subsys-2f-Src

