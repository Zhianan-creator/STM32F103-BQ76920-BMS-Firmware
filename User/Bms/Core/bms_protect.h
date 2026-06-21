#ifndef BMS_PROTECT_H
#define BMS_PROTECT_H

#include <stdint.h>

/* BMS 保护类型枚举 */
typedef enum
{
    BMS_PROTECT_FAULT_NONE = 0,
    BMS_PROTECT_FAULT_OV,
    BMS_PROTECT_FAULT_UV,
    BMS_PROTECT_FAULT_OCD,
    BMS_PROTECT_FAULT_SCD
} BMS_ProtectFault_t;

/* BMS 保护状态标志结构体 */
typedef struct
{
    /* 1. 单项保护状*/
    uint8_t ov_active;   /* 过压保护激活状(1=触发, 0=未触*/
    uint8_t uv_active;   /* 欠压保护激活状(1=触发, 0=未触*/
    uint8_t ocd_active;  /* 放电过流保护激活状(1=触发, 0=未触*/
    uint8_t scd_active;  /* 短路保护激活状(1=触发, 0=未触*/

    /* 2. 单项挂起(延迟确认)状*/
    uint8_t ov_pending;  /* 过压保护挂起状(1=正在延迟确认, 0=否*/
    uint8_t uv_pending;  /* 欠压保护挂起状(1=正在延迟确认, 0=否*/
    uint8_t ocd_pending; /* 放电过流保护挂起状(1=正在延迟确认, 0=否*/
    uint8_t scd_pending; /* 短路保护挂起状(1=正在延迟确认, 0=否*/

    /* 3. 总保护状*/
    uint8_t any_active;  /* 任何保护是否被激*/

    /* 3. 允许状*/
    uint8_t charge_allowed;    /* 允许充电 */
    uint8_t discharge_allowed; /* 允许放电 */

    /* 4. 最近一次触发信*/
    BMS_ProtectFault_t last_fault; /* 最近一次故障类*/
    uint8_t last_fault_cell;       /* 最近一次故障电芯编(1-indexed, 0表示无或不适用) */
    uint32_t last_fault_value;     /* 最近一次故障触发(mV mA) */

    /* 5. 统计信息 */
    uint32_t ov_trigger_count;  /* 过压触发次数 */
    uint32_t uv_trigger_count;  /* 欠压触发次数 */
    uint32_t ocd_trigger_count; /* 放电过流触发次数 */
    uint32_t scd_trigger_count; /* 短路触发次数 */
} BMS_ProtectState_t;

/* 初始化保护状态变*/
void BMS_ProtectInit(void);

/* 根据 Monitor 电参数据更新保护状态机 */
void BMS_ProtectUpdateFromMonitor(void);

/* 获取当前保护状态结构体常量指针 */
const BMS_ProtectState_t *BMS_ProtectGetState(void);

/* 查询是否有任何保护被激*/
uint8_t BMS_ProtectIsAnyActive(void);

/* 查询过压保护状*/
uint8_t BMS_ProtectIsOvActive(void);

/* 查询欠压保护状*/
uint8_t BMS_ProtectIsUvActive(void);

/* 查询放电过流保护状*/
uint8_t BMS_ProtectIsOcdActive(void);

/* 查询短路保护状*/
uint8_t BMS_ProtectIsScdActive(void);

/* 查询是否允许充电 */
uint8_t BMS_ProtectIsChargeAllowed(void);

/* 查询是否允许放电 */
uint8_t BMS_ProtectIsDischargeAllowed(void);

/* 获取最近一次触发故*/
BMS_ProtectFault_t BMS_ProtectGetLastFault(void);

/* 将保护类型转换为字符*/
const char *BMS_ProtectFaultToString(BMS_ProtectFault_t fault);

/* 动态保护阈值及延时获取/修改函数 */
void BMS_ProtectSetOVMv(int ov_mv);
void BMS_ProtectSetUVMv(int uv_mv);
void BMS_ProtectSetOCDMa(int ocd_ma);
void BMS_ProtectSetSCDMa(int scd_ma);
void BMS_ProtectSetOVDelayMs(int delay_ms);
void BMS_ProtectSetUVDelayMs(int delay_ms);
void BMS_ProtectSetOCDDelayMs(int delay_ms);
void BMS_ProtectSetSCDDelayMs(int delay_ms);

int BMS_ProtectGetOVMv(void);
int BMS_ProtectGetUVMv(void);
int BMS_ProtectGetOCDMa(void);
int BMS_ProtectGetSCDMa(void);
int BMS_ProtectGetOVDelayMs(void);
int BMS_ProtectGetUVDelayMs(void);
int BMS_ProtectGetOCDDelayMs(void);
int BMS_ProtectGetSCDDelayMs(void);

#endif /* BMS_PROTECT_H */
