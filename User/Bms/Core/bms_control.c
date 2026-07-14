#include "bms_control.h"
#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_uart_cmd.h"
#include "bms_config.h"
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
/* 检查指定 MOS 开启操作所需的软件保护、硬件故障和电压条件。 */
static BMS_ControlResult_t CheckPreconditions(uint8_t control_bit)
{
    if (((control_bit == CHG_CONTROL) && !BMS_ProtectIsChargeAllowed()) ||
        ((control_bit == DSG_CONTROL) && !BMS_ProtectIsDischargeAllowed()))
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

/* 回读 SYS_CTRL2，确认指定 MOS 控制位达到期望状态。 */
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
        if (!BQ769X0_ReadRegisterByteWithCRC(SYS_STAT, &sys_stat))
        {
            return BMS_CONTROL_I2C_ERROR;
        }
        if ((sys_stat & BQ_SYS_STAT_FAULT_MASK) != 0)
        {
            return BMS_CONTROL_REJECTED_HW_FAULT;
        }
        if ((ctrl2_val & control_bit) != 0)
        {
            return BMS_CONTROL_OK;
        }
    }
    else if ((ctrl2_val & control_bit) == 0)
    {
        return BMS_CONTROL_OK;
    }

    return BMS_CONTROL_VERIFY_ERROR;
}

/* 在总线锁内开启指定 MOS，并在写后复核保护许可，避免并发竞态。 */
static BMS_ControlResult_t BMS_ControlTurnOn(BQ769X0_ControlTypedef control)
{
    BMS_ControlResult_t result;
    uint8_t still_allowed;

    BQ769X0_BusLock();
    result = CheckPreconditions((uint8_t)control);
    if (result != BMS_CONTROL_OK)
    {
        goto out;
    }

    if (!BQ769X0_ControlDSGOrCHG(control, BQ_STATE_ENABLE))
    {
        result = BMS_CONTROL_I2C_ERROR;
        goto out;
    }

    result = VerifyControlBit((uint8_t)control, 1U);
    if (result != BMS_CONTROL_OK)
    {
        if (!BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT))
        {
            result = BMS_CONTROL_VERIFY_ERROR;
        }
        goto out;
    }

    still_allowed = (control == CHG_CONTROL) ? BMS_ProtectIsChargeAllowed()
                                             : BMS_ProtectIsDischargeAllowed();
    if (!still_allowed)
    {
        if (!BQ769X0_ControlDSGOrCHG(control, BQ_STATE_DISABLE) ||
            (VerifyControlBit((uint8_t)control, 0U) != BMS_CONTROL_OK))
        {
            (void)BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT);
            result = BMS_CONTROL_VERIFY_ERROR;
        }
        else
        {
            result = BMS_CONTROL_REJECTED_PROTECT;
        }
    }

out:
    BQ769X0_BusUnlock();
    return result;
}

/* ==========================================================================
 * CHG MOS 控制
 * ========================================================================== */

/* 在全部安全条件满足时开启充电 MOS。 */
BMS_ControlResult_t BMS_ControlChgOn(void)
{
#if (BMS_UART_CMD_ALLOW_CHG_ON == 0)
    return BMS_CONTROL_DISABLED;
#else
    return BMS_ControlTurnOn(CHG_CONTROL);
#endif
}

/* 关闭充电 MOS 并确认寄存器状态。 */
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

/* 在全部安全条件满足时开启放电 MOS。 */
BMS_ControlResult_t BMS_ControlDsgOn(void)
{
#if (BMS_UART_CMD_ALLOW_DSG_ON == 0)
    return BMS_CONTROL_DISABLED;
#else
    return BMS_ControlTurnOn(DSG_CONTROL);
#endif
}

/* 关闭放电 MOS 并确认寄存器状态。 */
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

/* 关闭全部电芯均衡并确认寄存器状态。 */
BMS_ControlResult_t BMS_ControlBalanceClear(void)
{
    return BQ769X0_CellBalanceControl(BQ_CELL_ALL, BQ_STATE_DISABLE)
           ? BMS_CONTROL_OK : BMS_CONTROL_I2C_ERROR;
}

/* 先清空旧均衡位，再应用新的电芯均衡掩码。 */
BMS_ControlResult_t BMS_ControlBalanceApplyMask(uint16_t cell_mask)
{
    if (!BQ769X0_CellBalanceControl(BQ_CELL_ALL, BQ_STATE_DISABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
    }
    
    if ((cell_mask != 0U) &&
        !BQ769X0_CellBalanceControl((BQ769X0_CellIndexTypedef)cell_mask,
                                    BQ_STATE_ENABLE))
    {
        return BMS_CONTROL_I2C_ERROR;
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
/* 按保护状态关闭危险输出，全部回读确认后返回 1。 */
uint8_t BMS_ControlApplyProtectState(const BMS_ProtectState_t *state)
{
#if (BMS_CONTROL_AUTO_APPLY_PROTECT == 1)
    uint8_t output_safe = 1U;

    if (state == NULL)
    {
        return 0U;
    }

    if (state->any_active && (BMS_ControlBalanceClear() != BMS_CONTROL_OK))
    {
        output_safe = 0U;
    }

    if (!state->charge_allowed)
    {
        if (BMS_ControlChgOff() != BMS_CONTROL_OK)
        {
            output_safe = 0U;
        }
    }

    if (!state->discharge_allowed)
    {
        if (BMS_ControlDsgOff() != BMS_CONTROL_OK)
        {
            output_safe = 0U;
        }
    }
    return output_safe;
#else
    (void)state;
    return 0U;
#endif
}

/* 获取充电 MOS 的实际状态，读取失败时返回关闭。 */
uint8_t BMS_ControlIsChgOn(void)
{
    uint8_t ctrl2_val = 0;
    if (BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
    {
        return (ctrl2_val & CHG_CONTROL) ? 1 : 0;
    }
    return 0;
}

/* 获取放电 MOS 的实际状态，读取失败时返回关闭。 */
uint8_t BMS_ControlIsDsgOn(void)
{
    uint8_t ctrl2_val = 0;
    if (BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
    {
        return (ctrl2_val & DSG_CONTROL) ? 1 : 0;
    }
    return 0;
}
