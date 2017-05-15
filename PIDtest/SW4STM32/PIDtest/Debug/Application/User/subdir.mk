################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/I2C_Software_Master.c \
C:/Users/user03/Desktop/000/PIDtest/Src/freertos.c \
C:/Users/user03/Desktop/000/PIDtest/Src/main.c \
../Application/User/matrix.c \
C:/Users/user03/Desktop/000/PIDtest/Src/stm32f4xx_hal_msp.c \
C:/Users/user03/Desktop/000/PIDtest/Src/stm32f4xx_it.c 

OBJS += \
./Application/User/I2C_Software_Master.o \
./Application/User/freertos.o \
./Application/User/main.o \
./Application/User/matrix.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o 

C_DEPS += \
./Application/User/I2C_Software_Master.d \
./Application/User/freertos.d \
./Application/User/main.d \
./Application/User/matrix.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/%.o: ../Application/User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/user03/Desktop/000/PIDtest/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/freertos.o: C:/Users/user03/Desktop/000/PIDtest/Src/freertos.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/user03/Desktop/000/PIDtest/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/main.o: C:/Users/user03/Desktop/000/PIDtest/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/user03/Desktop/000/PIDtest/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_hal_msp.o: C:/Users/user03/Desktop/000/PIDtest/Src/stm32f4xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/user03/Desktop/000/PIDtest/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f4xx_it.o: C:/Users/user03/Desktop/000/PIDtest/Src/stm32f4xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/user03/Desktop/000/PIDtest/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/user03/Desktop/000/PIDtest/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/Users/user03/Desktop/000/PIDtest/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


