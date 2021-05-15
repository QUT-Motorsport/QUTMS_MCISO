################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Common/Src/AMS_CAN_Messages.c \
../Core/Common/Src/BMS_CAN_Messages.c \
../Core/Common/Src/CC_CAN_Messages.c \
../Core/Common/Src/FSM.c \
../Core/Common/Src/PDM_CAN_Messages.c \
../Core/Common/Src/QUTMS_can.c \
../Core/Common/Src/SHDN_BSPD_CAN_Messages.c \
../Core/Common/Src/SHDN_CAN_Messages.c \
../Core/Common/Src/SHDN_IMD_CAN_Messages.c \
../Core/Common/Src/Timer.c \
../Core/Common/Src/queue.c \
../Core/Common/Src/window_filtering.c 

OBJS += \
./Core/Common/Src/AMS_CAN_Messages.o \
./Core/Common/Src/BMS_CAN_Messages.o \
./Core/Common/Src/CC_CAN_Messages.o \
./Core/Common/Src/FSM.o \
./Core/Common/Src/PDM_CAN_Messages.o \
./Core/Common/Src/QUTMS_can.o \
./Core/Common/Src/SHDN_BSPD_CAN_Messages.o \
./Core/Common/Src/SHDN_CAN_Messages.o \
./Core/Common/Src/SHDN_IMD_CAN_Messages.o \
./Core/Common/Src/Timer.o \
./Core/Common/Src/queue.o \
./Core/Common/Src/window_filtering.o 

C_DEPS += \
./Core/Common/Src/AMS_CAN_Messages.d \
./Core/Common/Src/BMS_CAN_Messages.d \
./Core/Common/Src/CC_CAN_Messages.d \
./Core/Common/Src/FSM.d \
./Core/Common/Src/PDM_CAN_Messages.d \
./Core/Common/Src/QUTMS_can.d \
./Core/Common/Src/SHDN_BSPD_CAN_Messages.d \
./Core/Common/Src/SHDN_CAN_Messages.d \
./Core/Common/Src/SHDN_IMD_CAN_Messages.d \
./Core/Common/Src/Timer.d \
./Core/Common/Src/queue.d \
./Core/Common/Src/window_filtering.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Common/Src/AMS_CAN_Messages.o: ../Core/Common/Src/AMS_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/AMS_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/BMS_CAN_Messages.o: ../Core/Common/Src/BMS_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/BMS_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/CC_CAN_Messages.o: ../Core/Common/Src/CC_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/CC_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/FSM.o: ../Core/Common/Src/FSM.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/FSM.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/PDM_CAN_Messages.o: ../Core/Common/Src/PDM_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/PDM_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/QUTMS_can.o: ../Core/Common/Src/QUTMS_can.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/QUTMS_can.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/SHDN_BSPD_CAN_Messages.o: ../Core/Common/Src/SHDN_BSPD_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_BSPD_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/SHDN_CAN_Messages.o: ../Core/Common/Src/SHDN_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/SHDN_IMD_CAN_Messages.o: ../Core/Common/Src/SHDN_IMD_CAN_Messages.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/SHDN_IMD_CAN_Messages.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/Timer.o: ../Core/Common/Src/Timer.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/Timer.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/queue.o: ../Core/Common/Src/queue.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/queue.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/Common/Src/window_filtering.o: ../Core/Common/Src/window_filtering.c Core/Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -DQUTMS_CAN_AMS -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../Core/Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/Common/Src/window_filtering.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

