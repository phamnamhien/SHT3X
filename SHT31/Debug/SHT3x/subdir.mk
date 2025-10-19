################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SHT3x/sht3x.c 

OBJS += \
./SHT3x/sht3x.o 

C_DEPS += \
./SHT3x/sht3x.d 


# Each subdirectory must supply rules for building sources it contributes
SHT3x/%.o SHT3x/%.su SHT3x/%.cyclo: ../SHT3x/%.c SHT3x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/Documents/GitHub/phamnamhien/SHT3X/SHT31/SHT3x" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-SHT3x

clean-SHT3x:
	-$(RM) ./SHT3x/sht3x.cyclo ./SHT3x/sht3x.d ./SHT3x/sht3x.o ./SHT3x/sht3x.su

.PHONY: clean-SHT3x

