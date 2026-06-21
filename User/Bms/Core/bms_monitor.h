#ifndef BMS_MONITOR_H
#define BMS_MONITOR_H

#include "main.h"
#include "bms_config.h"

/* 校验数据合理性范围的宏定*/
#define BMS_MONITOR_CELL_MIN_VALID_MV      1000
#define BMS_MONITOR_CELL_MAX_VALID_MV      4500
#define BMS_MONITOR_PACK_SUM_TOLERANCE_MV  500
#define BMS_MONITOR_TEMP_MIN_VALID_C       -40
#define BMS_MONITOR_TEMP_MAX_VALID_C        100

/* 系统模式与待机电流阈值定*/
#define BMS_MONITOR_CURRENT_STANDBY_MA      20      /* 待机电流界限: ±20mA */
#define BMS_MONITOR_CHARGE_CURRENT_MAX_MA   10000   /* charge valid current upper limit: +10A */
#define BMS_MONITOR_DISCHARGE_CURRENT_MAX_MA 15000  /* discharge valid current lower limit: -15A */
#define BMS_MONITOR_SLEEP_DELAY_MS          10000   /* 调试推荐待机进入休眠延迟: 10s(生产推荐: 300000) */

/* BMS 系统模式枚举 */
typedef enum
{
    BMS_SYS_MODE_STANDBY = 0,
    BMS_SYS_MODE_CHARGE,
    BMS_SYS_MODE_DISCHARGE,
    BMS_SYS_MODE_SLEEP
} BMS_SysMode_t;

/* Monitor 业务层电参状态数据结*/
typedef struct
{
    uint16_t cell_mv[BMS_CELL_MAX];       /* 各单体电芯电单位: mV) */
    uint16_t pack_mv;                     /* 电池包总电单位: mV) */
    int32_t current_ma;                   /* 电池包充放电电流(单位: mA, 充电为正，放电为*/
    int16_t temp_c[BMS_TEMP_MAX];         /* 温度(单位: °C) */

    uint8_t cell_count;                   /* 实际配置的单体电芯节*/
    uint8_t temp_count;                   /* 实际配置的温度传感器通道*/

    uint16_t max_cell_mv;                 /* 最大单体电mV) */
    uint16_t min_cell_mv;                 /* 最小单体电mV) */
    uint16_t delta_cell_mv;               /* 最大与最小单体电压差mV) */
    uint16_t avg_cell_mv;                 /* 所有单体电压平均mV) */
    uint8_t max_cell_index;               /* 最大单体电芯编(1 开*/
    uint8_t min_cell_index;               /* 最小单体电芯编(1 开*/

    uint8_t voltage_valid;                /* 单体/总包电压校验有效(1=有效, 0=无效) */
    uint8_t current_valid;                /* 电流读取校验有效(1=有效, 0=无效) */
    uint8_t temp_valid;                   /* 温度校验有效(1=有效, 0=无效) */
    uint8_t data_valid;                   /* 数据是否完全有效 (voltage_valid && temp_valid && current_valid) */

    uint8_t updated;                      /* 自上ClearUpdated 以来是否有新数据 (1=是0=已清*/
    uint32_t last_sample_tick;            /* 最新一次底层采样的系统 Tick 时间*/
    uint32_t last_update_tick;            /* 最新一Monitor 更新成功的系Tick 时间*/

    /* 内部系统模式状态机字段 */
    BMS_SysMode_t sys_mode;               /* 当前系统模式 */
    BMS_SysMode_t last_sys_mode;          /* 上一次系统模*/
    uint32_t sys_mode_enter_tick;         /* 切入当前模式Tick 时间*/
    uint32_t standby_start_tick;          /* 切入 STANDBY 状态的 Tick 时间(若非该状态则) */
    uint32_t sleep_enter_tick;            /* 切入 SLEEP 状态的 Tick 时间(若非该状态则) */
    uint8_t sleep_request;                /* 系统休眠请求标志 (1=请求休眠, 0=正常工作) */
    uint8_t sleep_allowed;                /* 允许进入休眠状态标*/
} BMS_MonitorData_t;

/* Monitor 运行上下文结构体 */
typedef struct
{
    BMS_MonitorData_t data;               /* 缓存的业务数*/
    uint32_t sample_count;                /* 自启动以来成功的采样统计*/
    uint32_t error_count;                 /* 采样失败统计*/
    int last_error;                       /* 最后一次采样的错误代码 */
} BMS_MonitorContext_t;

/* ==========================================================================
 * BMS Monitor 业务层公开 API 接口
 * ========================================================================== */

/* 初始Monitor 业务状态及计数*/
int BMS_MonitorInit(void);

/* 物理触发适配层采样，并深拷贝、校验及计算业务核心状态统计量 */
int BMS_MonitorUpdate(void);

/* 获取最新一轮业务电参结构体缓存指针 */
const BMS_MonitorData_t *BMS_MonitorGetData(void);

/* 查询当前电参数据是否合法有效 */
uint8_t BMS_MonitorIsValid(void);

/* 查询自上ClearUpdated 以来是否有新数据 */
uint8_t BMS_MonitorIsUpdated(void);

/* 清除 updated 标志（上层处理完新数据后调用*/
void BMS_MonitorClearUpdated(void);

/* 获取成功采样计数 */
uint32_t BMS_MonitorGetSampleCount(void);

/* 读取指定电芯电压(mV) */
uint16_t BMS_MonitorGetCellMv(uint8_t index);

/* 读取电池包总电mV) */
uint16_t BMS_MonitorGetPackMv(void);

/* 读取电池包充放电电流(mA) */
int32_t BMS_MonitorGetCurrentMa(void);

/* 读取指定通道温度(°C) */
int16_t BMS_MonitorGetTempC(uint8_t index);

/* 读取最大单体电芯电mV) */
uint16_t BMS_MonitorGetMaxCellMv(void);

/* 读取最小单体电芯电mV) */
uint16_t BMS_MonitorGetMinCellMv(void);

/* 读取最大与最小单体电压压mV) */
uint16_t BMS_MonitorGetDeltaCellMv(void);

/* 读取电芯平均电压(mV) */
uint16_t BMS_MonitorGetAvgCellMv(void);

/* 读取最大单体电芯编(1 开*/
uint8_t BMS_MonitorGetMaxCellIndex(void);

/* 读取最小单体电芯编(1 开*/
uint8_t BMS_MonitorGetMinCellIndex(void);

/* 获取系统模式的字符串描述 */
const char *BMS_MonitorSysModeToString(BMS_SysMode_t mode);

#endif /* BMS_MONITOR_H */
