################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Algorithms/maths/kalman_filters.c \
../Core/Src/Algorithms/maths/maths.c 

OBJS += \
./Core/Src/Algorithms/maths/kalman_filters.o \
./Core/Src/Algorithms/maths/maths.o 

C_DEPS += \
./Core/Src/Algorithms/maths/kalman_filters.d \
./Core/Src/Algorithms/maths/maths.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Algorithms/maths/%.o: ../Core/Src/Algorithms/maths/%.c Core/Src/Algorithms/maths/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_PRESENT=1 -D__FPU_USED=1 -c -I../Core/Src/User_Software -I../Drivers/CMSIS/Lib/GCC -I../Core/Src/Device -I../Core/Src/Device/imu -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Core/Src/Algorithms/attitude_resolution -I../Core/Src/Algorithms/comms -I../Core/Src/Algorithms/control -I../Core/Src/Algorithms/maths -I../Core/Src/Applications/Basic_App -I../Core/Src/Applications/Generic_App -I../Drivers/CMSIS/DSP/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Algorithms-2f-maths

clean-Core-2f-Src-2f-Algorithms-2f-maths:
	-$(RM) ./Core/Src/Algorithms/maths/kalman_filters.d ./Core/Src/Algorithms/maths/kalman_filters.o ./Core/Src/Algorithms/maths/maths.d ./Core/Src/Algorithms/maths/maths.o

.PHONY: clean-Core-2f-Src-2f-Algorithms-2f-maths

