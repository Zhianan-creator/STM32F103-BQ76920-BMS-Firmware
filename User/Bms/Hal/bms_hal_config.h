/*
 * bms_hal_config.h - BMS 硬件抽象层配最小实
 *
 * 本文件是 bms_config.h 的底层依提供:
 *   1. BQ769X0 驱动头文提供枚举类型定义)
 *   2. BMS 延时映射BQ 驱动枚举
 *
 * 当前阶段: 仅包BQ 初始化所必需的定* 后续阶段: 可扩GPIO、ADC、通信等硬件抽*/
#ifndef __BMS_HAL_CONFIG_H__
#define __BMS_HAL_CONFIG_H__

#include "drv_softi2c_bq769x0.h"

/*
 * BMS 保护延时- 映射BQ769X0 驱动枚举*
 * bms_config.h 中的 INIT_xxx_DELAY 宏使用这BMS_ 前缀的名* BQ 驱动BQ_ 前缀枚举值一一对应(整数值相
 *
 * SCD(短路)延时: 50/100/200/400 us
 * OCD(过流)延时: 10/20/40/80/160/320/640/1280 ms
 * OV (过压)延时: 1/2/4/8 s
 * UV (欠压)延时: 1/4/8/16 s
 */
#define BMS_SCD_DELAY_50us   ((uint8_t)BQ_SCD_DELAY_50us)
#define BMS_SCD_DELAY_100us  ((uint8_t)BQ_SCD_DELAY_100us)
#define BMS_SCD_DELAY_200us  ((uint8_t)BQ_SCD_DELAY_200us)
#define BMS_SCD_DELAY_400us  ((uint8_t)BQ_SCD_DELAY_400us)

#define BMS_OCD_DELAY_10ms   ((uint8_t)BQ_OCD_DEALY_10ms)
#define BMS_OCD_DELAY_20ms   ((uint8_t)BQ_OCD_DELAY_20ms)
#define BMS_OCD_DELAY_40ms   ((uint8_t)BQ_OCD_DELAY_40ms)
#define BMS_OCD_DELAY_80ms   ((uint8_t)BQ_OCD_DELAY_80ms)
#define BMS_OCD_DELAY_160ms  ((uint8_t)BQ_OCD_DELAY_160ms)
#define BMS_OCD_DELAY_320ms  ((uint8_t)BQ_OCD_DELAY_320ms)
#define BMS_OCD_DELAY_640ms  ((uint8_t)BQ_OCD_DELAY_640ms)
#define BMS_OCD_DELAY_1280ms ((uint8_t)BQ_OCD_DELAY_1280ms)

#define BMS_OV_DELAY_1s      ((uint8_t)BQ_OV_DELAY_1s)
#define BMS_OV_DELAY_2s      ((uint8_t)BQ_OV_DELAY_2s)
#define BMS_OV_DELAY_4s      ((uint8_t)BQ_OV_DELAY_4s)
#define BMS_OV_DELAY_8s      ((uint8_t)BQ_OV_DELAY_8s)

#define BMS_UV_DELAY_1s      ((uint8_t)BQ_UV_DELAY_1s)
#define BMS_UV_DELAY_4s      ((uint8_t)BQ_UV_DELAY_4s)
#define BMS_UV_DELAY_8s      ((uint8_t)BQ_UV_DELAY_8s)
#define BMS_UV_DELAY_16s     ((uint8_t)BQ_UV_DELAY_16s)

#endif /* __BMS_HAL_CONFIG_H__ */
