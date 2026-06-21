#include "bms_control.h"
#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_uart_cmd.h"
#include "drv_softi2c_bq769x0.h"
#include <stdio.h>

/* BQ SYS_STAT 故障掩码 */
#define BQ_SYS_STAT_FAULT_MASK \
    (SYS_STAT_OCD_BIT | SYS_STAT_SCD_BIT | SYS_STAT_OV_BIT | \
     SYS_STAT_UV_BIT | SYS_STAT_OVRD_BIT | SYS_STAT_DEVICE_BIT)

/* Pack 电压安全范围 (mV) */
#define CONTROL_PACK_MIN_MV  18000
#define CONTROL_PACK_MAX_MV  21500

/* ==========================================================================
 * 前置安全检*
 * 检查软件保护状态、硬SYS_STAT 故障位、Monitor 数据有效性、Pack 电压范围
 * 所CHG/DSG 开启操作必须先通过此检* ========================================================================== */
static BMS_ControlResult_t CheckPreconditions(void)
{
    /* 1. 软件保护状态检*/
    if (BMS_ProtectIsOvActive() || BMS_ProtectIsUvActive() ||
        BMS_ProtectIsOcdActive() || BMS_ProtectIsScdActive())
    {
        return BMS_CONTROL_REJECTED_PROTECT;
    }

    /* 2. 硬件芯片故障位检*/
    uint8_t sys_stat = 0;
    if (!BQ769X0_ReadRegisterByteWithCRC(SYS_STAT, &sys_stat))
    {
        return BMS_CONTROL_I2C_ERROR;
    }
    if (sys_stat & BQ_SYS_STAT_FAULT_MASK)
    {
        return BMS_CONTROL_REJECTED_HW_FAULT;
    }

    /* 3. Monitor 数据有效性检*/
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    if (mon == NULL || !mon->data_valid)
    {
        return BMS_CONTROL_REJECTED_VOLTAGE;
    }

    /* 4. Pack 电压范围检*/
    int pack_mv = (int)mon->pack_mv;
    if (pack_mv < CONTROL_PACK_MIN_MV || pack_mv > CONTROL_PACK_MAX_MV)
    {
        return BMS_CONTROL_REJECTED_VOLTAGE;
    }

    return BMS_CONTROL_OK;
}

static BMS_ControlResult_t VerifyControlBit(uint8_t control_bit, uint8_t expect_on)
{
    uint8_t ctrl2_val = 0;
    uint8_t sys_stat = 0;

    if (!BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    if (expect_on)
    {
        if ((ctrl2_val & control_bit) != 0)
        {
            return BMS_CONTROL_OK;
        }

        if (!BQ769X0_ReadRegisterByteWithCRC(SYS_STAT, &sys_stat))
        {
            return BMS_CONTROL_I2C_ERROR;
        }
        if ((sys_stat & BQ_SYS_STAT_FAULT_MASK) != 0)
        {
            return BMS_CONTROL_REJECTED_HW_FAULT;
        }
    }
    else if ((ctrl2_val & control_bit) == 0)
    {
        return BMS_CONTROL_OK;
    }

    return BMS_CONTROL_VERIFY_ERROR;
}

/* ==========================================================================
 * CHG MOS 控制
 * ========================================================================== */

BMS_ControlResult_t BMS_ControlChgOn(void)
{
#if (BMS_UART_CMD_ALLOW_CHG_ON == 0)
    return BMS_CONTROL_DISABLED;
#else
    BMS_ControlResult_t ret = CheckPreconditions();
    if (ret != BMS_CONTROL_OK)
    {
        return ret;
    }

    if (!BQ769X0_ControlDSGOrCHG(CHG_CONTROL, BQ_STATE_ENABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    return VerifyControlBit(CHG_CONTROL, 1);
#endif
}

BMS_ControlResult_t BMS_ControlChgOff(void)
{
    if (!BQ769X0_ControlDSGOrCHG(CHG_CONTROL, BQ_STATE_DISABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    return VerifyControlBit(CHG_CONTROL, 0);
}

/* ==========================================================================
 * DSG MOS 控制
 * ========================================================================== */

BMS_ControlResult_t BMS_ControlDsgOn(void)
{
#if (BMS_UART_CMD_ALLOW_DSG_ON == 0)
    return BMS_CONTROL_DISABLED;
#else
    BMS_ControlResult_t ret = CheckPreconditions();
    if (ret != BMS_CONTROL_OK)
    {
        return ret;
    }

    if (!BQ769X0_ControlDSGOrCHG(DSG_CONTROL, BQ_STATE_ENABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    return VerifyControlBit(DSG_CONTROL, 1);
#endif
}

BMS_ControlResult_t BMS_ControlDsgOff(void)
{
    if (!BQ769X0_ControlDSGOrCHG(DSG_CONTROL, BQ_STATE_DISABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    return VerifyControlBit(DSG_CONTROL, 0);
}

/* ==========================================================================
 * 均衡寄存器清* ========================================================================== */

BMS_ControlResult_t BMS_ControlBalanceClear(void)
{
    uint8_t bal_write[3] = {0, 0, 0};
    if (!BQ769X0_WriteBlockWithCRC(CELLBAL1, bal_write, 3))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    uint8_t bal_read[3] = {0, 0, 0};
    if (!BQ769X0_ReadBlockWithCRC(CELLBAL1, bal_read, 3))
    {
        return BMS_CONTROL_I2C_ERROR;
    }

    return (bal_read[0] == 0 && bal_read[1] == 0 && bal_read[2] == 0)
           ? BMS_CONTROL_OK : BMS_CONTROL_I2C_ERROR;
}

BMS_ControlResult_t BMS_ControlBalanceApplyMask(uint16_t cell_mask)
{
    /* 先清除所有电芯均衡通道 */
    BQ769X0_CellBalanceControl(BQ_CELL_ALL, BQ_STATE_DISABLE);
    
    /* 如果有使能的均衡，则开启对应的电芯通道 */
    if (cell_mask != 0)
    {
        BQ769X0_CellBalanceControl((BQ769X0_CellIndexTypedef)cell_mask, BQ_STATE_ENABLE);
    }
    
    return BMS_CONTROL_OK;
}

/* ==========================================================================
 * 结果转字符串
 * ========================================================================== */

const char *BMS_ControlResultToString(BMS_ControlResult_t result)
{
    switch (result)
    {
        case BMS_CONTROL_OK:                return "OK";
        case BMS_CONTROL_DISABLED:          return "rejected: command disabled by macro";
        case BMS_CONTROL_REJECTED_PROTECT:  return "rejected: protection active";
        case BMS_CONTROL_REJECTED_HW_FAULT: return "rejected: HW fault in SYS_STAT";
        case BMS_CONTROL_REJECTED_VOLTAGE:  return "rejected: monitor data invalid or pack voltage out of range";
        case BMS_CONTROL_I2C_ERROR:         return "I2C error";
        case BMS_CONTROL_VERIFY_ERROR:      return "rejected: SYS_CTRL2 readback mismatch";
        default:                            return "unknown error";
    }
}

/* ==========================================================================
 * 预留与保护模块控制联动接* ========================================================================== */
void BMS_ControlApplyProtectState(const BMS_ProtectState_t *state)
{
#if (BMS_CONTROL_AUTO_APPLY_PROTECT == 1)
    if (state == NULL)
    {
        return;
    }

    /* 即使以后宏为 1，本阶段也只允许自动关闭 CHG/DSG，不允许自动打开 */
    /* 本次不要实现自动恢复开 MOS */
    if (!state->charge_allowed)
    {
        if (BMS_ControlIsChgOn())
        {
            BMS_ControlChgOff();
            printf("[PROTECT] Charge forbidden by software protection! CHG forced OFF physically.\r\n");
        }
    }

    if (!state->discharge_allowed)
    {
        if (BMS_ControlIsDsgOn())
        {
            BMS_ControlDsgOff();
            printf("[PROTECT] Discharge forbidden by software protection! DSG forced OFF physically.\r\n");
        }
    }
#else
    (void)state; /* 未开启联动，忽略状态 */
#endif
}

uint8_t BMS_ControlIsChgOn(void)
{
    uint8_t ctrl2_val = 0;
    if (BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
    {
        return (ctrl2_val & CHG_CONTROL) ? 1 : 0;
    }
    return 0;
}

uint8_t BMS_ControlIsDsgOn(void)
{
    uint8_t ctrl2_val = 0;
    if (BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
    {
        return (ctrl2_val & DSG_CONTROL) ? 1 : 0;
    }
    return 0;
}
