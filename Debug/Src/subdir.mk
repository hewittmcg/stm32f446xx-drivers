################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009_spi_message_rcv_it.c \
../Src/sysmem.c 

OBJS += \
./Src/009_spi_message_rcv_it.o \
./Src/sysmem.o 

C_DEPS += \
./Src/009_spi_message_rcv_it.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/009_spi_message_rcv_it.o: ../Src/009_spi_message_rcv_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/Inc" -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/009_spi_message_rcv_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I../Inc -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/Inc" -I"C:/Users/Hewitt McGaughey/Documents/Code/stm32f446xx-drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

