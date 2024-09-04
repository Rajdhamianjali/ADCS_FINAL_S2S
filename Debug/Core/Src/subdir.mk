################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Bdot.c \
../Core/Src/EKF.c \
../Core/Src/IMU.c \
../Core/Src/MT25QL.c \
../Core/Src/OBC_Handshake.c \
../Core/Src/RCFilter.c \
../Core/Src/USER_FUNCTIONS.c \
../Core/Src/estimator_main.c \
../Core/Src/main.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/variables.c 

OBJS += \
./Core/Src/Bdot.o \
./Core/Src/EKF.o \
./Core/Src/IMU.o \
./Core/Src/MT25QL.o \
./Core/Src/OBC_Handshake.o \
./Core/Src/RCFilter.o \
./Core/Src/USER_FUNCTIONS.o \
./Core/Src/estimator_main.o \
./Core/Src/main.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/variables.o 

C_DEPS += \
./Core/Src/Bdot.d \
./Core/Src/EKF.d \
./Core/Src/IMU.d \
./Core/Src/MT25QL.d \
./Core/Src/OBC_Handshake.d \
./Core/Src/RCFilter.d \
./Core/Src/USER_FUNCTIONS.d \
./Core/Src/estimator_main.d \
./Core/Src/main.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/variables.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

