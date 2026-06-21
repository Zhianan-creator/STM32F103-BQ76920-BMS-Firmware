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

/* CHG MOS 控制 */
BMS_ControlResult_t BMS_ControlChgOn(void);
BMS_ControlResult_t BMS_ControlChgOff(void);

/* DSG MOS 控制 */
BMS_ControlResult_t BMS_ControlDsgOn(void);
BMS_ControlResult_t BMS_ControlDsgOff(void);

/* 均衡寄存器清*/
BMS_ControlResult_t BMS_ControlBalanceClear(void);

/* 按位图开启均*/
BMS_ControlResult_t BMS_ControlBalanceApplyMask(uint16_t cell_mask);

/* 结果转字符串 */
const char *BMS_ControlResultToString(BMS_ControlResult_t result);

/* 获取当前 MOS 实际状*/
uint8_t BMS_ControlIsChgOn(void);
uint8_t BMS_ControlIsDsgOn(void);

#define BMS_CONTROL_AUTO_APPLY_PROTECT 1

#include "bms_protect.h"

/* 联动保护状态，根据软件保护关闭 MOS */
void BMS_ControlApplyProtectState(const BMS_ProtectState_t *state);

#endif /* BMS_CONTROL_H */
