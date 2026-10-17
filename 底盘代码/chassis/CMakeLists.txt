CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m4)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_C_COMPILER "D:/App/env-windows-v2.0.0/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_ASM_COMPILER "D:/App/env-windows-v2.0.0/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-gcc.exe")
SET(CMAKE_C_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_ASM_FLAGS " -c -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -x assembler-with-cpp -Wa,-mimplicit-it=thumb  -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_COMPILER "D:/App/env-windows-v2.0.0/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-g++.exe")
SET(CMAKE_CXX_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Dgcc -O0 -gdwarf-2 -g")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_OBJCOPY "D:/App/env-windows-v2.0.0/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-objcopy.exe")
SET(CMAKE_SIZE "D:/App/env-windows-v2.0.0/env-windows/tools/bin/../../tools/gnu_gcc/arm_gcc/mingw/bin/arm-none-eabi-size.exe")

SET(CMAKE_EXE_LINKER_FLAGS " -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Wl,--gc-sections,-Map=rt-thread.map,-cref,-u,Reset_Handler -T ${CMAKE_SOURCE_DIR}/board/linker_scripts/link.lds")

SET(CMAKE_CXX_STANDARD 14)
PROJECT(rtthread C CXX ASM)
INCLUDE_DIRECTORIES(
	applications
	.
	rt-thread/components/libc/compilers/common/include
	rt-thread/components/libc/compilers/newlib
	rt-thread/components/libc/cplusplus
	rt-thread/libcpu/arm/common
	rt-thread/libcpu/arm/cortex-m4
	device
	rt-thread/components/drivers/include
	board
	board/CubeMX_Config/Inc
	board/ports
	libraries/HAL_Drivers
	libraries/HAL_Drivers/config
	libraries/HAL_Drivers/CMSIS/Include
	libraries/HAL_Drivers/drv_flash
	packages/EasyFlash-v4.1.0/inc
	rt-thread/components/fal/inc
	rt-thread/components/finsh
	packages/FlexibleButton-v1.0.0
	packages/i2c-tools-latest
	packages/i2c-tools-latest/src
	rt-thread/include
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Inc
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Include
	rt-thread/components/libc/posix/io/poll
	rt-thread/components/libc/posix/io/stdio
	rt-thread/components/libc/posix/ipc
)

ADD_DEFINITIONS(
	-DHAVE_CCONFIG_H
	-DRT_USING_LIBC
	-DRT_USING_NEWLIB
	-DSTM32F407xx
	-DUSE_HAL_DRIVER
	-D_POSIX_C_SOURCE=1
	-D__RTTHREAD__
)

SET(PROJECT_SOURCES
	applications/drv_seg.c
	applications/main.c
	applications/ef_fal_port.c
	applications/hostpc.c
	applications/battery_info.c
	applications/drv_motor.c
	applications/chassis.c
	rt-thread/components/libc/compilers/common/cctype.c
	rt-thread/components/libc/compilers/common/cstdio.c
	rt-thread/components/libc/compilers/common/cstdlib.c
	rt-thread/components/libc/compilers/common/cstring.c
	rt-thread/components/libc/compilers/common/ctime.c
	rt-thread/components/libc/compilers/common/cwchar.c
	rt-thread/components/libc/compilers/newlib/syscalls.c
	rt-thread/components/libc/cplusplus/cxx_Semaphore.cpp
	rt-thread/components/libc/cplusplus/cxx_crt_init.c
	rt-thread/components/libc/cplusplus/cxx_crt.cpp
	rt-thread/components/libc/cplusplus/cxx_Thread.cpp
	rt-thread/components/libc/cplusplus/cxx_Mutex.cpp
	rt-thread/libcpu/arm/common/backtrace.c
	rt-thread/libcpu/arm/common/div0.c
	rt-thread/libcpu/arm/common/showmem.c
	rt-thread/libcpu/arm/cortex-m4/context_gcc.S
	rt-thread/libcpu/arm/cortex-m4/cpuport.c
	device/button.c
	device/beep.c
	rt-thread/components/drivers/i2c/i2c-bit-ops.c
	rt-thread/components/drivers/i2c/i2c_core.c
	rt-thread/components/drivers/i2c/i2c_dev.c
	rt-thread/components/drivers/ipc/completion.c
	rt-thread/components/drivers/ipc/dataqueue.c
	rt-thread/components/drivers/ipc/pipe.c
	rt-thread/components/drivers/ipc/ringblk_buf.c
	rt-thread/components/drivers/ipc/ringbuffer.c
	rt-thread/components/drivers/ipc/waitqueue.c
	rt-thread/components/drivers/ipc/workqueue.c
	rt-thread/components/drivers/misc/adc.c
	rt-thread/components/drivers/misc/pin.c
	rt-thread/components/drivers/misc/rt_drv_pwm.c
	rt-thread/components/drivers/serial/serial.c
	board/CubeMX_Config/Src/stm32f4xx_hal_msp.c
	board/board.c
	board/drv_flash_f4.c
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
	libraries/HAL_Drivers/drv_adc.c
	libraries/HAL_Drivers/drv_common.c
	libraries/HAL_Drivers/drv_gpio.c
	libraries/HAL_Drivers/drv_pwm.c
	libraries/HAL_Drivers/drv_soft_i2c.c
	libraries/HAL_Drivers/drv_usart.c
	packages/EasyFlash-v4.1.0/src/ef_iap.c
	packages/EasyFlash-v4.1.0/src/easyflash.c
	packages/EasyFlash-v4.1.0/src/ef_cmd.c
	packages/EasyFlash-v4.1.0/src/ef_env_legacy.c
	packages/EasyFlash-v4.1.0/src/ef_env_legacy_wl.c
	packages/EasyFlash-v4.1.0/src/ef_log.c
	packages/EasyFlash-v4.1.0/src/ef_env.c
	packages/EasyFlash-v4.1.0/src/ef_utils.c
	rt-thread/components/fal/src/fal_partition.c
	rt-thread/components/fal/src/fal_rtt.c
	rt-thread/components/fal/src/fal_flash.c
	rt-thread/components/fal/src/fal.c
	rt-thread/components/finsh/msh_parse.c
	rt-thread/components/finsh/cmd.c
	rt-thread/components/finsh/shell.c
	rt-thread/components/finsh/msh.c
	packages/FlexibleButton-v1.0.0/flexible_button.c
	packages/i2c-tools-latest/src/i2c_tools.cpp
	packages/i2c-tools-latest/src/i2c_utils.cpp
	rt-thread/src/clock.c
	rt-thread/src/components.c
	rt-thread/src/device.c
	rt-thread/src/idle.c
	rt-thread/src/ipc.c
	rt-thread/src/irq.c
	rt-thread/src/kservice.c
	rt-thread/src/mem.c
	rt-thread/src/mempool.c
	rt-thread/src/object.c
	rt-thread/src/scheduler.c
	rt-thread/src/thread.c
	rt-thread/src/timer.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cec.c
	libraries/STM32F4xx_HAL/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_lptim.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cryp_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
	libraries/STM32F4xx_HAL/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	packages/rt_vsnprintf_full-latest/rt_vsnprintf.c
)

LINK_DIRECTORIES(
)

LINK_LIBRARIES(
	c
	m
)

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES})
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf)