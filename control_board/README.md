# STM32 control-board firmware

This directory contains the STM32CubeMX project and source code for the
vehicle motor/steering control board.

## Target and interfaces

- MCU: STM32F407VGTx (LQFP100)
- UART: USART1, 115200 baud, 8-N-1
- Steering PWM: TIM1 channel 1
- Left/right motor PWM: TIM8 channels 1 and 2
- Binary protocol frame header: `0x42`
- Address `0`: heartbeat
- Address `1`: vehicle speed and servo command
- Address `4`: buzzer command

The vehicle command frame contains a 32-bit floating-point speed followed by
a 16-bit servo pulse width. See `Core/Src/main.c` for parsing and actuator
limits.

## Projects

- `78678678.ioc`: STM32CubeMX configuration
- `MDK-ARM/78678678.uvprojx`: Keil MDK project
- `EWARM/Project.eww`: IAR Embedded Workbench workspace

Open either IDE project to build and flash the firmware. The required HAL and
CMSIS headers/sources used by the generated project are included under
`Drivers/`.

Local flashing tool bundles, IDE session state, object files, logs, generated
images, and the flash-readback binary are intentionally excluded.
