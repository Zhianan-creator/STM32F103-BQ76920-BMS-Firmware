#ifndef BMS_CONTROL_H
#define BMS_CONTROL_H

#include <stdint.h>

/* 控制操作结果枚举 */
typedef enum
{
    BMS_CONTROL_OK = 0,
    BMS_CONTROL_DISABLED,
    BMS_CONTROL_REJECTED_PROTECT,
    BMS_CONTROL_REJECTED_HW_FAULT,
    BMS_CONTROL_REJECTED_VOLTAGE,
    BMS_CONTROL_I2C_ERROR,
    BMS_CONTROL_VERIFY_ERROR
} BMS_ControlResult_t;

/* 在安全条件满足时开启充电 MOS。 */
BMS_ControlResult_t BMS_ControlChgOn(void);

/* 关闭充电 MOS 并确认结果。 */
BMS_ControlResult_t BMS_ControlChgOff(void);

/* 在安全条件满足时开启放电 MOS。 */
BMS_ControlResult_t BMS_ControlDsgOn(void);

/* 关闭放电 MOS 并确认结果。 */
BMS_ControlResult_t BMS_ControlDsgOff(void);

/* 关闭全部电芯均衡并确认结果。 */
BMS_ControlResult_t BMS_ControlBalanceClear(void);

/* 按位图应用电芯均衡通道。 */
BMS_ControlResult_t BMS_ControlBalanceApplyMask(uint16_t cell_mask);

/* 结果转字符串 */
const char *BMS_ControlResultToString(BMS_ControlResult_t result);

/* 获取充电 MOS 的实际状态。 */
uint8_t BMS_ControlIsChgOn(void);

/* 获取放电 MOS 的实际状态。 */
uint8_t BMS_ControlIsDsgOn(void);

#define BMS_CONTROL_AUTO_APPLY_PROTECT 1

#include "bms_protect.h"

/* 按保护状态关闭危险输出，全部回读确认后返回 1。 */
uint8_t BMS_ControlApplyProtectState(const BMS_ProtectState_t *state);

#endif /* BMS_CONTROL_H */
