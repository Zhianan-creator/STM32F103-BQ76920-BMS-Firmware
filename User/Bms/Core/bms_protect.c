#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_config.h"
#include "bms_log.h"
#include "cmsis_os2.h"
#include "drv_softi2c_bq769x0.h"
#include "bms_control.h"
#include <stdio.h>

/* 保护状态结构体，局部静态维*/
static BMS_ProtectState_t protect_state = {0};

/* 保护阈来自 bms_config.h,转换mV/mA 供整数比较使*/
static int PROT_OV_MV = 4200;   /* 过压阈4200 mV */
static int PROT_UV_MV = 3100;   /* 欠压阈3100 mV */
static int PROT_OCD_MA = 2200;  /* 放电过流阈2200 mA */
static int PROT_SCD_MA = 8800;  /* 短路阈8800 mA */
static int PROT_SHUTDOWN_MV = 3080; /* 自动关机电压阈值 3080 mV */
static uint32_t shutdown_pending_start_tick = 0; /* 严重欠压延迟确认时间戳 */

/* 恢复阈值回(回差释放*/
#define PROTECT_OV_RELEASE_DELTA_MV 50
#define PROTECT_UV_RELEASE_DELTA_MV 50
#define PROTECT_OC_RELEASE_DELTA_MA 200

/* 软件 De-bounce 延时时间 (ms) */
static int SOFT_PROT_OV_DELAY_MS = 2000;
static int SOFT_PROT_UV_DELAY_MS = 4000;
static int SOFT_PROT_OCD_DELAY_MS = 320;
static int SOFT_PROT_SCD_DELAY_MS = 1;

/* 自动恢复延时时间 (ms) */
#define PROTECT_OCD_RECOVERY_DELAY_MS 3000
#define PROTECT_SCD_RECOVERY_DELAY_MS 3000

/* 延迟确认时间戳变*/
static uint32_t ov_pending_start_tick = 0;
static uint32_t uv_pending_start_tick = 0;
static uint32_t ocd_pending_start_tick = 0;
static uint32_t scd_pending_start_tick = 0;

/* OCD 自动恢复延迟挂起变量 */
static uint8_t ocd_recovery_pending = 0;
static uint32_t ocd_recovery_start_tick = 0;

/* SCD 自动恢复延迟挂起变量 */
static uint8_t scd_recovery_pending = 0;
static uint32_t scd_recovery_start_tick = 0;

/* 初始化保护状态变*/
void BMS_ProtectInit(void)
{
    PROT_OV_MV = ((int)(INIT_OV_PROTECT * 1000));
    PROT_UV_MV = ((int)(INIT_UV_PROTECT * 1000));
    PROT_OCD_MA = INIT_OCD_PROTECT_CURRENT;
    PROT_SCD_MA = INIT_SCD_PROTECT_CURRENT;
    PROT_SHUTDOWN_MV = ((int)(INIT_SHUTDOWN_VOLTAGE * 1000));

    SOFT_PROT_OV_DELAY_MS = 2000;
    SOFT_PROT_UV_DELAY_MS = 4000;
    SOFT_PROT_OCD_DELAY_MS = 320;
    SOFT_PROT_SCD_DELAY_MS = 1;

    protect_state.ov_active = 0;
    protect_state.uv_active = 0;
    protect_state.ocd_active = 0;
    protect_state.scd_active = 0;
    
    protect_state.ov_pending = 0;
    protect_state.uv_pending = 0;
    protect_state.ocd_pending = 0;
    protect_state.scd_pending = 0;
    
    protect_state.any_active = 0;
    protect_state.charge_allowed = 1;
    protect_state.discharge_allowed = 1;
    protect_state.last_fault = BMS_PROTECT_FAULT_NONE;
    protect_state.last_fault_cell = 0;
    protect_state.last_fault_value = 0;
    protect_state.ov_trigger_count = 0;
    protect_state.uv_trigger_count = 0;
    protect_state.ocd_trigger_count = 0;
    protect_state.scd_trigger_count = 0;

    ov_pending_start_tick = 0;
    uv_pending_start_tick = 0;
    ocd_pending_start_tick = 0;
    scd_pending_start_tick = 0;
    shutdown_pending_start_tick = 0;

    ocd_recovery_pending = 0;
    ocd_recovery_start_tick = 0;
    scd_recovery_pending = 0;
    scd_recovery_start_tick = 0;
}

/* 根据 Monitor 电参数据更新保护状态机 */
void BMS_ProtectUpdateFromMonitor(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    int32_t discharge_current_ma;
    int i;
    static uint32_t invalid_print_counter = 0;
    uint32_t current_tick = osKernelGetTickCount();

    if (mon == NULL || !mon->data_valid)
    {
        if (invalid_print_counter % 10 == 0)
        {
            BMS_LOGW("PROTECT", "monitor data invalid, skip update");
        }
        invalid_print_counter++;
        return;
    }
    invalid_print_counter = 0; /* Reset counter when valid */

    /* Only negative current is discharge; charge or idle current must not trigger OCD/SCD. */
    discharge_current_ma = (mon->current_ma < 0) ? -mon->current_ma : 0;

    // ==================== 1. OV: 单体过压 ====================
    if (!protect_state.ov_active)
    {
        int fault_cell = -1;
        uint16_t fault_mv = 0;
        for (i = 0; i < mon->cell_count; i++)
        {
            if (mon->cell_mv[i] >= PROT_OV_MV)
            {
                fault_cell = i + 1;
                fault_mv = mon->cell_mv[i];
                break;
            }
        }
        if (fault_cell >= 0)
        {
            if (!protect_state.ov_pending)
            {
                protect_state.ov_pending = 1;
                ov_pending_start_tick = current_tick;
                BMS_LOGW("PROTECT", "OV Pending: Cell%d %dmV", fault_cell, fault_mv);
            }
            else
            {
                if (current_tick - ov_pending_start_tick >= SOFT_PROT_OV_DELAY_MS)
                {
                    protect_state.ov_active = 1;
                    protect_state.ov_pending = 0;
                    protect_state.last_fault = BMS_PROTECT_FAULT_OV;
                    protect_state.last_fault_cell = fault_cell;
                    protect_state.last_fault_value = fault_mv;
                    protect_state.ov_trigger_count++;
                    BMS_LOGE("PROTECT", "OV Triggered: Cell%d %d mV >= %d mV", 
                           fault_cell, fault_mv, PROT_OV_MV);
                }
            }
        }
        else
        {
            protect_state.ov_pending = 0;
        }
    }
    else
    {
        if (mon->max_cell_mv <= (PROT_OV_MV - PROTECT_OV_RELEASE_DELTA_MV))
        {
            protect_state.ov_active = 0;
            BMS_LOGI("PROTECT", "OV Auto recovered");
        }
    }

    // ==================== 2. UV: 单体欠压 ====================
    if (!protect_state.uv_active)
    {
        int fault_cell = -1;
        uint16_t fault_mv = 0;
        for (i = 0; i < mon->cell_count; i++)
        {
            if (mon->cell_mv[i] <= PROT_UV_MV)
            {
                fault_cell = i + 1;
                fault_mv = mon->cell_mv[i];
                break;
            }
        }
        if (fault_cell >= 0)
        {
            if (!protect_state.uv_pending)
            {
                protect_state.uv_pending = 1;
                uv_pending_start_tick = current_tick;
                BMS_LOGW("PROTECT", "UV Pending: Cell%d %dmV", fault_cell, fault_mv);
            }
            else
            {
                if (current_tick - uv_pending_start_tick >= SOFT_PROT_UV_DELAY_MS)
                {
                    protect_state.uv_active = 1;
                    protect_state.uv_pending = 0;
                    protect_state.last_fault = BMS_PROTECT_FAULT_UV;
                    protect_state.last_fault_cell = fault_cell;
                    protect_state.last_fault_value = fault_mv;
                    protect_state.uv_trigger_count++;
                    BMS_LOGE("PROTECT", "UV Triggered: Cell%d %d mV <= %d mV", 
                           fault_cell, fault_mv, PROT_UV_MV);
                }
            }
        }
        else
        {
            protect_state.uv_pending = 0;
        }
    }
    else
    {
        if (mon->min_cell_mv >= (PROT_UV_MV + PROTECT_UV_RELEASE_DELTA_MV))
        {
            protect_state.uv_active = 0;
            BMS_LOGI("PROTECT", "UV Auto recovered");
        }
    }

    // ==================== 3. OCD: 放电过流 ====================
    if (!protect_state.ocd_active)
    {
        if (discharge_current_ma >= PROT_OCD_MA)
        {
            if (!protect_state.ocd_pending)
            {
                protect_state.ocd_pending = 1;
                ocd_pending_start_tick = current_tick;
                BMS_LOGW("PROTECT", "OCD Pending: %dmA", (int)discharge_current_ma);
            }
            else
            {
                if (current_tick - ocd_pending_start_tick >= SOFT_PROT_OCD_DELAY_MS)
                {
                    protect_state.ocd_active = 1;
                    protect_state.ocd_pending = 0;
                    protect_state.last_fault = BMS_PROTECT_FAULT_OCD;
                    protect_state.last_fault_cell = 0;
                    protect_state.last_fault_value = discharge_current_ma;
                    protect_state.ocd_trigger_count++;
                    BMS_LOGE("PROTECT", "OCD Triggered: Current %d mA >= %d mA", 
                           (int)discharge_current_ma, PROT_OCD_MA);
                }
            }
        }
        else
        {
            protect_state.ocd_pending = 0;
        }
    }
    else
    {
        if (discharge_current_ma <= (PROT_OCD_MA - PROTECT_OC_RELEASE_DELTA_MA))
        {
            if (!ocd_recovery_pending)
            {
                ocd_recovery_pending = 1;
                ocd_recovery_start_tick = current_tick;
            }
            else
            {
                if (current_tick - ocd_recovery_start_tick >= PROTECT_OCD_RECOVERY_DELAY_MS)
                {
                    protect_state.ocd_active = 0;
                    ocd_recovery_pending = 0;
                    BMS_LOGI("PROTECT", "OCD Auto recovered");
                }
            }
        }
        else
        {
            ocd_recovery_pending = 0;
        }
    }

    // ==================== 4. SCD: 短路/严重过流 ====================
    if (!protect_state.scd_active)
    {
        if (discharge_current_ma >= PROT_SCD_MA)
        {
            if (!protect_state.scd_pending)
            {
                protect_state.scd_pending = 1;
                scd_pending_start_tick = current_tick;
                BMS_LOGW("PROTECT", "SCD Pending: %dmA", (int)discharge_current_ma);
            }
            else
            {
                if (current_tick - scd_pending_start_tick >= SOFT_PROT_SCD_DELAY_MS)
                {
                    protect_state.scd_active = 1;
                    protect_state.scd_pending = 0;
                    protect_state.last_fault = BMS_PROTECT_FAULT_SCD;
                    protect_state.last_fault_cell = 0;
                    protect_state.last_fault_value = discharge_current_ma;
                    protect_state.scd_trigger_count++;
                    BMS_LOGE("PROTECT", "SCD Triggered: Current %d mA >= %d mA", 
                           (int)discharge_current_ma, PROT_SCD_MA);
                }
            }
        }
        else
        {
            protect_state.scd_pending = 0;
        }
    }
    else
    {
        if (discharge_current_ma <= (PROT_SCD_MA - PROTECT_OC_RELEASE_DELTA_MA))
        {
            if (!scd_recovery_pending)
            {
                scd_recovery_pending = 1;
                scd_recovery_start_tick = current_tick;
            }
            else
            {
                if (current_tick - scd_recovery_start_tick >= PROTECT_SCD_RECOVERY_DELAY_MS)
                {
                    protect_state.scd_active = 0;
                    scd_recovery_pending = 0;
                    BMS_LOGI("PROTECT", "SCD Auto recovered");
                }
            }
        }
        else
        {
            scd_recovery_pending = 0;
        }
    }

    // ==================== 4.5. CRITICAL UV SHUTDOWN (SHIP MODE) ====================
    if (mon->min_cell_mv <= PROT_SHUTDOWN_MV)
    {
        if (shutdown_pending_start_tick == 0)
        {
            shutdown_pending_start_tick = current_tick;
            BMS_LOGW("PROTECT", "CRITICAL UNDER-VOLTAGE PENDING: Min Cell %dmV <= Shutdown %dmV", 
                     (int)mon->min_cell_mv, PROT_SHUTDOWN_MV);
        }
        else
        {
            if (current_tick - shutdown_pending_start_tick >= 5000)
            {
                BMS_LOGE("PROTECT", "CRITICAL UNDER-VOLTAGE TRIGGERED! Entering SHIP mode to prevent battery damage.");
                osDelay(100);
                BQ769X0_ForceSafeOff();
                BQ769X0_EntryShip();
            }
        }
    }
    else
    {
        shutdown_pending_start_tick = 0;
    }

    /* 5. 更新总保护状态和允许状*/
    protect_state.charge_allowed = (protect_state.ov_active == 0);
    protect_state.discharge_allowed = (protect_state.uv_active == 0 &&
                                       protect_state.ocd_active == 0 &&
                                       protect_state.scd_active == 0);
    protect_state.any_active = (protect_state.ov_active || 
                                protect_state.uv_active || 
                                protect_state.ocd_active || 
                                protect_state.scd_active);

    /* 6. 联动底层控制，一有故障，物理瞬间断开 */
    BMS_ControlApplyProtectState(&protect_state);
}

/* 获取当前保护状态结构体常量指针 */
const BMS_ProtectState_t *BMS_ProtectGetState(void)
{
    return &protect_state;
}

/* 查询是否有任何保护被激*/
uint8_t BMS_ProtectIsAnyActive(void)
{
    return protect_state.any_active;
}

/* 查询过压保护状*/
uint8_t BMS_ProtectIsOvActive(void)
{
    return protect_state.ov_active;
}

/* 查询欠压保护状*/
uint8_t BMS_ProtectIsUvActive(void)
{
    return protect_state.uv_active;
}

/* 查询放电过流保护状*/
uint8_t BMS_ProtectIsOcdActive(void)
{
    return protect_state.ocd_active;
}

/* 查询短路保护状*/
uint8_t BMS_ProtectIsScdActive(void)
{
    return protect_state.scd_active;
}

/* 查询是否允许充电 */
uint8_t BMS_ProtectIsChargeAllowed(void)
{
    return protect_state.charge_allowed;
}

/* 查询是否允许放电 */
uint8_t BMS_ProtectIsDischargeAllowed(void)
{
    return protect_state.discharge_allowed;
}

/* 获取最近一次触发故*/
BMS_ProtectFault_t BMS_ProtectGetLastFault(void)
{
    return protect_state.last_fault;
}

/* 将保护类型转换为字符*/
const char *BMS_ProtectFaultToString(BMS_ProtectFault_t fault)
{
    switch (fault)
    {
        case BMS_PROTECT_FAULT_NONE: return "NONE";
        case BMS_PROTECT_FAULT_OV:   return "OV";
        case BMS_PROTECT_FAULT_UV:   return "UV";
        case BMS_PROTECT_FAULT_OCD:  return "OCD";
        case BMS_PROTECT_FAULT_SCD:  return "SCD";
        default:                     return "UNKNOWN";
    }
}

/* 动态保护限额及延时修改器与获取器 */
void BMS_ProtectSetOVMv(int ov_mv) { PROT_OV_MV = ov_mv; }
void BMS_ProtectSetUVMv(int uv_mv) { PROT_UV_MV = uv_mv; }
void BMS_ProtectSetOCDMa(int ocd_ma) { PROT_OCD_MA = ocd_ma; }
void BMS_ProtectSetSCDMa(int scd_ma) { PROT_SCD_MA = scd_ma; }
void BMS_ProtectSetOVDelayMs(int delay_ms) { SOFT_PROT_OV_DELAY_MS = delay_ms; }
void BMS_ProtectSetUVDelayMs(int delay_ms) { SOFT_PROT_UV_DELAY_MS = delay_ms; }
void BMS_ProtectSetOCDDelayMs(int delay_ms) { SOFT_PROT_OCD_DELAY_MS = delay_ms; }
void BMS_ProtectSetSCDDelayMs(int delay_ms) { SOFT_PROT_SCD_DELAY_MS = delay_ms; }

int BMS_ProtectGetOVMv(void) { return PROT_OV_MV; }
int BMS_ProtectGetUVMv(void) { return PROT_UV_MV; }
int BMS_ProtectGetOCDMa(void) { return PROT_OCD_MA; }
int BMS_ProtectGetSCDMa(void) { return PROT_SCD_MA; }
int BMS_ProtectGetOVDelayMs(void) { return SOFT_PROT_OV_DELAY_MS; }
int BMS_ProtectGetUVDelayMs(void) { return SOFT_PROT_UV_DELAY_MS; }
int BMS_ProtectGetOCDDelayMs(void) { return SOFT_PROT_OCD_DELAY_MS; }
int BMS_ProtectGetSCDDelayMs(void) { return SOFT_PROT_SCD_DELAY_MS; }
