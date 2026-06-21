# FREE-BMS

FREE-BMS is an STM32F103C8T6 battery management firmware project built around FreeRTOS and the TI BQ769X0 analog front end.

## Platform

- MCU: STM32F103C8T6, Cortex-M3, 72 MHz
- AFE: BQ769X0 / BQ76920, 3-5 cells
- RTOS: FreeRTOS with CMSIS-RTOS v2
- Toolchain: Keil MDK-ARM
- Debug console: USART1, 115200 baud

## Main Features

- Software I2C driver for BQ769X0 register access
- BQ769X0 initialization, CRC, sampling, and ALERT handling
- Monitor layer for voltage, current, temperature, and cell statistics
- Software protection state machine for OV, UV, OCD, and SCD states
- Basic CHG, DSG, and cell-balance control interface
- UART command console for bring-up and diagnostics
- CAN status task for external gateway integration

## Project Structure

```text
Core/          STM32CubeMX generated application and peripheral glue
User/          BMS application, business logic, HAL adapter, and device drivers
Drivers/       STM32 HAL and CMSIS libraries
Middlewares/   FreeRTOS middleware
MDK-ARM/       Keil project files
```

## Build

Open `MDK-ARM/FREE-BMS.uvprojx` with Keil uVision5, then build the target with `F7`.

Expected build result: `0 Error, 0 Warning`.

## Flash and Console

- Flash through ST-Link SWD on PA13/PA14.
- Open USART1 at 115200 baud for logs and the command console.

## Notes

CubeMX-generated files should only be edited inside `USER CODE` blocks. Application code is under `User/`.
