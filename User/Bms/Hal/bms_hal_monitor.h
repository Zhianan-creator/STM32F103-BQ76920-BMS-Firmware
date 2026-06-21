#ifndef BMS_HAL_MONITOR_H
#define BMS_HAL_MONITOR_H

#include "main.h"
#include "bms_config.h"

/* 统一采样监控数据结构*/
typedef struct
{
    uint16_t cell_mv[BMS_CELL_MAX];       /* 各单体电芯电单位: mV) */
    uint16_t pack_mv;                     /* 电池包总电单位: mV) */
    int32_t current_ma;                   /* 电池包充放电电流(单位: mA, 充电为正，放电为*/
    int16_t temp_c[BMS_TEMP_MAX];         /* 温度(单位: °C) */

    uint8_t cell_count;                   /* 实际配置的单体电芯节*/
    uint8_t temp_count;                   /* 实际配置的温度传感器通道*/

    uint8_t valid;                        /* 数据是否有效标志(1=有效, 0=无效) */
    uint32_t sample_tick;                 /* 最新一次采样的系统 Tick 时间*/
} BMS_HAL_MonitorData_t;

/* ==========================================================================
 * BMS HAL Monitor 公开 API 接口
 * ========================================================================== */

/* 初始HAL Monitor 层内部状态与配置*/
int BMS_HAL_MonitorInit(void);

/* 触发底层周期物理采样，并将采样值更新缓存，复制out (若不NULL) */
int BMS_HAL_MonitorSample(BMS_HAL_MonitorData_t *out);

/* 获取最新一次采样的内部缓存数据指针 */
const BMS_HAL_MonitorData_t *BMS_HAL_MonitorGetLatest(void);

/* 读取指定电芯电压(mV) */
uint16_t BMS_HAL_MonitorGetCellMv(uint8_t index);

/* 读取整包电压(mV) */
uint16_t BMS_HAL_MonitorGetPackMv(void);

/* 读取整包电流(mA) */
int32_t BMS_HAL_MonitorGetCurrentMa(void);

/* 读取指定通道温度(°C) */
int16_t BMS_HAL_MonitorGetTempC(uint8_t index);

/* 查询最新采样数据是否有*/
uint8_t BMS_HAL_MonitorIsValid(void);

/* 查询最新采样的系统 Tick 时间*/
uint32_t BMS_HAL_MonitorGetSampleTick(void);

#endif /* BMS_HAL_MONITOR_H */
