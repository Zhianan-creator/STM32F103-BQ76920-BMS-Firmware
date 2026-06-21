# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

FREE-BMS is a Battery Management System for STM32F103C8T6 (72MHz, Cortex-M3) driving a BQ769X0 (BQ76920 3-5 cell) analog front-end via software I2C. The firmware runs on FreeRTOS (CMSIS-RTOS v2) and is built with Keil MDK-ARM V5.32.

## Build & Flash

The project uses Keil MDK-ARM. There is no Makefile or command-line build.

- **Open**: `MDK-ARM/FREE-BMS.uvprojx` in Keil uVision5
- **Build**: Project → Build Target (F7). Expected: 0 Error, 0 Warning
- **Flash**: Flash → Download (F8) via ST-Link (SWD on PA13/PA14)
- **Console**: USART1 @ 115200 baud (PA9=TX, PA10=RX). Use MicroLIB in Keil options

There are no automated tests — verification is done via serial console output.

## Architecture

```
Core/               STM32 HAL + CubeMX-generated code (DO NOT EDIT outside USER CODE blocks)
  Inc/              main.h (pin defines), FreeRTOSConfig.h
  Src/              main.c, freertos.c (FreeRTOS tasks + init orchestration), usart.c, gpio.c

User/                     Application & driver code (safe to edit freely)
  Bms/
    Config/               Configuration layer
      bms_config.h            Central config: cell count, temp, protection thresholds, balance params
    App/                  FreeRTOS task layer
      bms_init.c/h            BQ769X0 initialization verification flow (UART→I2C→ACK→CRC→Init) + WaitDone
      bms_sample_task.c/h     Periodic sampling task (250ms): Monitor→Analysis→Protect→Energy
      bms_alert_task.c/h      ALERT interrupt processing task: SYS_STAT read, fault print, status clear
      bms_uart_cmd.c/h        Interactive UART command console (help/status/protect/sample/fault/chg/dsg/bal)
    Core/                 Business logic layer
      bms_monitor.c/h         Data validation, max/min/delta/avg stats — single data source for upper layers
      bms_protect.c/h         Software protection state machine (OV/UV/OCD/SCD + hysteresis)
      bms_control.c/h         Low-level CHG/DSG/BAL control with pre-condition safety checks
      bms_energy.c/h          Energy management: CHG/DSG/BAL MOS on/off decision logic
      bms_analysis.c/h        SOC estimation: coulomb counting + OCV correction, capacity tracking
      bms_log.c/h             Unified log service: level control (ERROR/WARN/INFO/DEBUG) + mutex protection
    Hal/                  Hardware adaptation layer
      bms_hal_config.h        Bridge: BQ driver enums → BMS delay macros
      bms_hal_monitor.c/h     BQ driver → integer mV/mA/°C conversion
  Drivers/                Device drivers
    drv_soft_i2c.c/h          Software I2C bit-bang (PB13=SDA, PB14=SCL), FreeRTOS mutex-protected
    drv_softi2c_bq769x0.c/h   BQ769X0 driver: register R/W with CRC8, sampling, ALERT handling

Drivers/            ST HAL & CMSIS libraries (DO NOT EDIT)
Middlewares/        FreeRTOS kernel (DO NOT EDIT)
```

**Layered data flow**:
```
freertos.c creates 4 tasks:
  defaultTask  → BMS_InitRun()                [bms_init.c]
  sampleTask   → BMS_SampleTask()             [bms_sample_task.c]
  alertTask    → BMS_AlertTask()              [bms_alert_task.c]
  uartCmdTask  → BMS_UartCmd_Task()           [bms_uart_cmd.c]

Sample task cycle (1000ms):
  → BMS_MonitorUpdate()           [bms_monitor.c]  — validation, stats
    → BMS_HAL_MonitorSample()     [bms_hal_monitor.c] — unit conversion
      → BQ769X0_UpdateXxx()       [drv_softi2c_bq769x0.c] — I2C register read
  → BMS_ProtectUpdateFromMonitor() [bms_protect.c] — OV/UV/OCD/SCD state machine

ALERT event flow:
  EXTI ISR → BQ769X0_AlertNotifyFromISR() → vTaskNotifyGiveFromISR
  → BMS_AlertTask wakes → BQ769X0_ProcessAlert() → read SYS_STAT → print + clear
```

## Key Design Decisions

**Configuration layering**: `bms_config.h` is the single source of truth for all BMS parameters. It includes `bms_hal_config.h` which maps BQ driver enum names to `BMS_` prefixed macros. BQ driver `drv_softi2c_bq769x0.h` uses `#ifndef` guards so values can be overridden from Keil preprocessor defines.

**ALERT handling (ISR deferred)**: BQ769X0 ALERT pin (PB12, EXTI rising edge) sets a volatile flag in ISR via `BQ769X0_AlertNotifyFromISR()`. The sample task wakes on `xTaskNotifyGive` and calls `BQ769X0_ProcessAlert()` in task context (safe for I2C). Never call I2C functions from ISR.

**FreeRTOS tasks** (created in `Core/Src/freertos.c`):
- `defaultTask` — One-shot: calls `BMS_InitRun()` (in `bms_init.c`). Then idle.
- `sampleTask` — Periodic (1000ms): calls `BMS_SampleTask()` (in `bms_sample_task.c`) — Monitor update → Protect state machine → initial print + control tests once.
- `alertTask` — Event-driven: calls `BMS_AlertTask()` (in `bms_alert_task.c`) — blocks on ISR notification → `BQ769X0_ProcessAlert()`.
- `uartCmdTask` — UART command console via interrupt-driven RX queue. Created only when `BMS_UART_CMD_ENABLE != 0`. Commands: help, status, sample, fault, chg/dsg on/off, bal on/off.

**Protection model** (`bms_protect.c`): Software state machine with hysteresis (OV/UV release delta 50mV, OCD release delta 200mA). Reads data exclusively from `bms_monitor.h`. Currently log-only — no real MOS disconnect actions yet. Hardware protection via BQ769X0 PROTECT registers is configured independently.

**BQ bus mutex**: A FreeRTOS recursive mutex (`BQ769X0_BusLockInit/BusLock/BusUnlock` in `drv_softi2c_bq769x0.c`) protects all BQ769X0 register read/write operations. Initialized in `BMS_InitRun()`. Higher-level functions (ControlDSGOrCHG, CellBalanceControl, ForceSafeOff) also lock for atomicity.

**Monitor data flow**: `bms_monitor.c` is the single data source for upper layers. All business modules (protect, uart cmd, future balance/SOC) read from `BMS_MonitorGetData()` instead of accessing `BQ769X0_SampleData` directly. Only `bms_hal_monitor.c` touches the BQ driver for sampling.

## Hardware Pin Map

| Function | Pin | Notes |
|---|---|---|
| I2C SDA | PB13 | Software I2C, open-drain, 4.7K pull-up to VCC |
| I2C SCL | PB14 | Software I2C, open-drain, 4.7K pull-up to VCC |
| BQ ALERT | PB12 | EXTI rising edge interrupt |
| BQ TS1 (wake) | PA15 | Output high to wake from Ship mode, then input for NTC |
| USART1 TX | PA9 | Debug console, 115200 baud |
| USART1 RX | PA10 | |
| USART2 TX/RX | PA2/PA3 | Secondary UART (reserved) |
| LEDs | PB5-PB9 | Active low (set = off) |

## Coding Conventions

- CubeMX-generated files (`Core/`, `Drivers/`, `Middlewares/`) must never be edited directly. User code goes in `/* USER CODE BEGIN/END */` blocks or under `User/`.
- printf output uses `[TAG]` prefixes: `[UART]`, `[I2C]`, `[BQ]`, `[BQ SAMPLE]`, `[PROTECT]`, `[ALERT]`, `[CTRL TEST]`, `[STAGE SUMMARY]`.
- Protection parameters flow: battery chemistry defaults in `bms_config.h` → `INIT_xxx` macros → `bms_protect.c` reads them via `bms_config.h`.
- Debug logging is macro-controlled (level 0-2) in both `drv_soft_i2c.h` and `drv_softi2c_bq769x0.h`. Default is level 0 (all off).
- Temporary CHG/DSG/Balance driver test macros have been removed. Use the UART commands for manual control during bring-up.
- BQ769X0 runtime diagnostic prints are gated by `BQ769X0_RUNTIME_PRINT_ENABLE` in `drv_softi2c_bq769x0.c`; default is enabled for current console diagnostics.

## Current Development Phase

The driver and sampling are working. Next steps (not yet implemented):
1. Real MOS protection actions (disconnect CHG/DSG on fault)
2. Balance strategy (non-adjacent cell selection, timing)
3. SOC estimation (coulomb counting + OCV correction)
4. CAN/UART communication protocol
5. Data logging and fault history
