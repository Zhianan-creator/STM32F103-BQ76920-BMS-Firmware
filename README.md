# 基于 STM32F103 + BQ76920 的锂电池 BMS 固件开发

本项目是一个基于 STM32F103C8T6、FreeRTOS 和 TI BQ76920/BQ769X0 模拟前端芯片的锂电池 BMS 固件项目，主要用于多串锂电池的数据采样、状态监测、保护判断、控制执行和调试通信。

## 硬件与软件平台

- 主控芯片：STM32F103C8T6，Cortex-M3，72 MHz
- 电池模拟前端：BQ769X0 / BQ76920，支持 3-5 串电芯
- 实时操作系统：FreeRTOS，基于 CMSIS-RTOS v2
- 开发工具链：Keil MDK-ARM
- 调试串口：USART1，115200 baud

## 主要功能

- 基于软件 I2C 实现 BQ769X0 寄存器读写
- 实现 BQ769X0 初始化、CRC 校验、数据采样和 ALERT 中断处理
- 采集并维护电芯电压、电流、温度、最大值、最小值、压差和平均值等监测数据
- 实现过压、欠压、放电过流、短路等软件保护状态机
- 提供 CHG、DSG 和电芯均衡控制接口
- 提供 UART 命令行调试功能，支持状态查看、采样查看、保护状态查看和手动控制
- 提供 CAN 状态任务，用于后续接入外部网关或上位机系统

## 工程结构

```text
Core/          STM32CubeMX 生成的应用入口、外设初始化和 FreeRTOS 任务创建代码
User/          BMS 应用层、业务逻辑层、硬件适配层和设备驱动代码
Drivers/       STM32 HAL 与 CMSIS 库
Middlewares/   FreeRTOS 中间件
MDK-ARM/       Keil 工程文件
```

## 构建方式

使用 Keil uVision5 打开 `MDK-ARM/FREE-BMS.uvprojx`，然后按 `F7` 编译工程。

期望编译结果：`0 Error, 0 Warning`。

## 烧录与调试

- 通过 ST-Link 使用 SWD 接口烧录，SWD 引脚为 PA13/PA14。
- 使用 USART1 作为调试串口，波特率为 115200。
- 串口输出用于查看 BQ 初始化、采样、保护状态、ALERT 事件和 UART 命令行交互。

## 开发说明

- `Core/` 下的 CubeMX 生成文件只建议在 `USER CODE` 区域内修改。
- `User/` 目录是主要应用代码目录，可以安全维护 BMS 业务逻辑、驱动和调试功能。
- 当前项目重点是 BMS 固件功能验证和嵌入式业务链路实现，不代表完整量产级 BMS 硬件方案。
