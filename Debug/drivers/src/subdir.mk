################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio.c \
../drivers/src/stm32f446xx_spi.c 

OBJS += \
./drivers/src/stm32f446xx_gpio.o \
./drivers/src/stm32f446xx_spi.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio.d \
./drivers/src/stm32f446xx_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f446xx_gpio.o: ../drivers/src/stm32f446xx_gpio.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/Inc" -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_gpio.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/src/stm32f446xx_spi.o: ../drivers/src/stm32f446xx_spi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/Inc" -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_spi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

