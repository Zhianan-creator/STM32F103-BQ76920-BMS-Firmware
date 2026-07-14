#ifndef BMS_ENERGY_H
#define BMS_ENERGY_H

#include <stdint.h>
#include "bms_protect.h"
#include "bms_control.h"

/* BMS Energy 业务层状态结构体 */
typedef struct
{
    uint8_t charge_enable_cmd;          /* 全局允许充电使能指令 (1=开0=关闭，默) */
    uint8_t discharge_enable_cmd;       /* 全局允许放电使能指令 (1=开0=关闭，默) */
    uint8_t balance_enable_cmd;         /* 全局被动均衡使能指令 (1=开启自动均衡, 0=关闭) */
    uint16_t soc_permille;              /* 临时 SOC 千分(0~1000，默00表示50%) */
    uint8_t chg_on;                     /* 当前 CHG MOS 开启状(1=开0=关闭) */
    uint8_t dsg_on;                     /* 当前 DSG MOS 开启状(1=开0=关闭) */
    uint8_t balance_active;             /* 均衡是否处于激活运行状(1=是0=否*/
    uint16_t balance_mask;              /* 当前均衡电芯位图 */
    uint32_t balance_start_tick;        /* 均衡启动时间(ms) */
    uint32_t balance_rise_until_tick;   /* 均衡结束后电压回升截止时间戳 (ms) */
    uint32_t last_action_tick;          /* 最近一次执行动作的时间(ms) */
    uint32_t chg_start_count;           /* 充电开启次*/
    uint32_t chg_stop_count;            /* 充电停止/关闭次数 */
    uint32_t dsg_start_count;           /* 放电开启次*/
    uint32_t dsg_stop_count;            /* 放电停止/关闭次数 */
    uint32_t balance_start_count;        /* 均衡启动次数 */
    uint32_t balance_stop_count;         /* 均衡停止次数 */
    BMS_ControlResult_t last_chg_result; /* 最近一CHG 控制操作结果 */
    BMS_ControlResult_t last_dsg_result; /* 最近一DSG 控制操作结果 */
    BMS_ControlResult_t last_balance_result; /* 最近一次均衡控制操作结*/
} BMS_EnergyState_t;

/* 初始Energy 模块 */
void BMS_EnergyInit(void);

/* 更新 Energy 状态（整合充放电和被动均衡管理*/
void BMS_EnergyUpdate(void);

/* 充放电管理决策业*/
void BMS_EnergyChgDsgManage(void);

/* 被动均衡管理决策业务 */
void BMS_EnergyBalanceManage(void);

/* 获取 Energy 当前状态结构体指针 */
const BMS_EnergyState_t *BMS_EnergyGetState(void);

/* 设置/获取 SOC (0~1000) */
void BMS_EnergySetSocPermille(uint16_t soc);
uint16_t BMS_EnergyGetSocPermille(void);

/* 设置全局控制使能指令 */
void BMS_EnergySetChargeEnable(uint8_t enable);
/* Set discharge intent; enabling fails while the explicit recovery gate is closed. */
uint8_t BMS_EnergySetDischargeEnable(uint8_t enable);
void BMS_EnergySetBalanceEnable(uint8_t enable);

#endif /* BMS_ENERGY_H */
