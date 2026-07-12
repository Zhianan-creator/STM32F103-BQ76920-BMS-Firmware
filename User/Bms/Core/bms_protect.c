#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_config.h"
#include "bms_log.h"
#include "bms_control.h"
#include "drv_softi2c_bq769x0.h"
#include "cmsis_os2.h"
#include <string.h>

#define PROTECT_OV_RELEASE_DELTA_MV     50
#define PROTECT_UV_RELEASE_DELTA_MV     50
#define PROTECT_OC_RELEASE_DELTA_MA    200
#define PROTECT_OCD_RECOVERY_MS       3000U
#define PROTECT_SCD_RECOVERY_MS       3000U
#define PROTECT_SHUTDOWN_DELAY_MS     5000U

typedef struct
{
    uint8_t active;
    uint8_t pending;
    uint8_t recovery_pending;
    uint32_t pending_tick;
    uint32_t recovery_tick;
} BMS_ProtectGuard_t;

typedef enum
{
    BMS_GUARD_NO_CHANGE = 0,
    BMS_GUARD_TRIGGERED,
    BMS_GUARD_RECOVERED
} BMS_ProtectTransition_t;

static BMS_ProtectState_t protect_state;
static BMS_ProtectGuard_t ov_guard;
static BMS_ProtectGuard_t uv_guard;
static BMS_ProtectGuard_t ocd_guard;
static BMS_ProtectGuard_t scd_guard;
static BMS_ProtectGuard_t occ_guard;
static BMS_ProtectGuard_t otc_guard;
static BMS_ProtectGuard_t otd_guard;
static BMS_ProtectGuard_t ltc_guard;
static BMS_ProtectGuard_t ltd_guard;

static int PROT_OV_MV = 4200;
static int PROT_UV_MV = 3100;
static int PROT_OCD_MA = 2200;
static int PROT_SCD_MA = 8800;
static int PROT_SHUTDOWN_MV = 3080;
static int SOFT_PROT_OV_DELAY_MS = 2000;
static int SOFT_PROT_UV_DELAY_MS = 4000;
static int SOFT_PROT_OCD_DELAY_MS = 320;
static int SOFT_PROT_SCD_DELAY_MS = 1;

static uint8_t invalid_sample_count;
static uint8_t recovery_sample_count;
static uint32_t shutdown_pending_tick;

/* 更新一个带触发延时、恢复阈值和恢复延时的保护条件。 */
static BMS_ProtectTransition_t BMS_ProtectUpdateGuard(BMS_ProtectGuard_t *guard,
                                                       uint8_t trip_condition,
                                                       uint8_t release_condition,
                                                       uint32_t trip_delay_ms,
                                                       uint32_t recovery_delay_ms,
                                                       uint32_t now)
{
    if (!guard->active)
    {
        guard->recovery_pending = 0U;
        if (!trip_condition)
        {
            guard->pending = 0U;
            return BMS_GUARD_NO_CHANGE;
        }

        if (!guard->pending)
        {
            guard->pending = 1U;
            guard->pending_tick = now;
            return BMS_GUARD_NO_CHANGE;
        }

        if ((uint32_t)(now - guard->pending_tick) >= trip_delay_ms)
        {
            guard->active = 1U;
            guard->pending = 0U;
            return BMS_GUARD_TRIGGERED;
        }
        return BMS_GUARD_NO_CHANGE;
    }

    guard->pending = 0U;
    if (!release_condition)
    {
        guard->recovery_pending = 0U;
        return BMS_GUARD_NO_CHANGE;
    }

    if (recovery_delay_ms == 0U)
    {
        guard->active = 0U;
        return BMS_GUARD_RECOVERED;
    }

    if (!guard->recovery_pending)
    {
        guard->recovery_pending = 1U;
        guard->recovery_tick = now;
        return BMS_GUARD_NO_CHANGE;
    }

    if ((uint32_t)(now - guard->recovery_tick) >= recovery_delay_ms)
    {
        guard->active = 0U;
        guard->recovery_pending = 0U;
        return BMS_GUARD_RECOVERED;
    }
    return BMS_GUARD_NO_CHANGE;
}

/* 记录新触发的保护及对应测量值。 */
static void BMS_ProtectRecordFault(BMS_ProtectFault_t fault,
                                   uint8_t cell,
                                   int32_t value,
                                   uint32_t *counter)
{
    protect_state.last_fault = fault;
    protect_state.last_fault_cell = cell;
    protect_state.last_fault_value = value;
    (*counter)++;
    BMS_LOGE("PROTECT", "%s triggered: value=%ld cell=%u",
             BMS_ProtectFaultToString(fault), (long)value, (unsigned int)cell);
}

/* 将内部保护守卫同步到对外状态，并计算充放电许可。 */
static void BMS_ProtectSyncState(void)
{
    protect_state.ov_active = ov_guard.active;
    protect_state.uv_active = uv_guard.active;
    protect_state.ocd_active = ocd_guard.active;
    protect_state.scd_active = scd_guard.active;
    protect_state.occ_active = occ_guard.active;
    protect_state.otc_active = otc_guard.active;
    protect_state.otd_active = otd_guard.active;
    protect_state.ltc_active = ltc_guard.active;
    protect_state.ltd_active = ltd_guard.active;

    protect_state.ov_pending = ov_guard.pending;
    protect_state.uv_pending = uv_guard.pending;
    protect_state.ocd_pending = ocd_guard.pending;
    protect_state.scd_pending = scd_guard.pending;
    protect_state.occ_pending = occ_guard.pending;
    protect_state.otc_pending = otc_guard.pending;
    protect_state.otd_pending = otd_guard.pending;
    protect_state.ltc_pending = ltc_guard.pending;
    protect_state.ltd_pending = ltd_guard.pending;

    protect_state.charge_allowed = !(protect_state.ov_active ||
                                     protect_state.occ_active ||
                                     protect_state.otc_active ||
                                     protect_state.ltc_active ||
                                     protect_state.fail_safe_active ||
                                     protect_state.shutdown_active);
    protect_state.discharge_allowed = !(protect_state.uv_active ||
                                        protect_state.ocd_active ||
                                        protect_state.scd_active ||
                                        protect_state.otd_active ||
                                        protect_state.ltd_active ||
                                        protect_state.fail_safe_active ||
                                        protect_state.shutdown_active);
    protect_state.any_active = !(protect_state.charge_allowed &&
                                 protect_state.discharge_allowed);
}

/* 初始化保护阈值、计时器和状态。 */
void BMS_ProtectInit(void)
{
    PROT_OV_MV = (int)(INIT_OV_PROTECT * 1000.0f);
    PROT_UV_MV = (int)(INIT_UV_PROTECT * 1000.0f);
    PROT_OCD_MA = INIT_OCD_PROTECT_CURRENT;
    PROT_SCD_MA = INIT_SCD_PROTECT_CURRENT;
    PROT_SHUTDOWN_MV = (int)(INIT_SHUTDOWN_VOLTAGE * 1000.0f);

    memset(&protect_state, 0, sizeof(protect_state));
    memset(&ov_guard, 0, sizeof(ov_guard));
    memset(&uv_guard, 0, sizeof(uv_guard));
    memset(&ocd_guard, 0, sizeof(ocd_guard));
    memset(&scd_guard, 0, sizeof(scd_guard));
    memset(&occ_guard, 0, sizeof(occ_guard));
    memset(&otc_guard, 0, sizeof(otc_guard));
    memset(&otd_guard, 0, sizeof(otd_guard));
    memset(&ltc_guard, 0, sizeof(ltc_guard));
    memset(&ltd_guard, 0, sizeof(ltd_guard));

    invalid_sample_count = 0U;
    recovery_sample_count = 0U;
    shutdown_pending_tick = 0U;
    protect_state.charge_allowed = 1U;
    protect_state.discharge_allowed = 1U;
    protect_state.output_safe_confirmed = 1U;
}

/* 根据最新监控数据更新全部软件保护，并执行必要的安全关断。 */
void BMS_ProtectUpdateFromMonitor(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    BMS_ProtectTransition_t transition;
    uint32_t now = osKernelGetTickCount();
    int32_t discharge_current_ma;
    int32_t charge_current_ma;
    int16_t min_temp_c;
    int16_t max_temp_c;
    uint8_t ov_cell = 0U;
    uint8_t uv_cell = 0U;
    uint8_t i;

    if (protect_state.shutdown_active)
    {
        return;
    }

    if ((mon == NULL) || !mon->data_valid)
    {
        recovery_sample_count = 0U;
        if (invalid_sample_count < BMS_MONITOR_FAIL_SAFE_COUNT)
        {
            invalid_sample_count++;
        }

        if (invalid_sample_count >= BMS_MONITOR_FAIL_SAFE_COUNT)
        {
            if (!protect_state.fail_safe_active)
            {
                protect_state.fail_safe_active = 1U;
                BMS_ProtectRecordFault(BMS_PROTECT_FAULT_MONITOR, 0U, -1,
                                       &protect_state.fail_safe_trigger_count);
            }
            if (!protect_state.safe_off_confirmed)
            {
                protect_state.safe_off_confirmed =
                    BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT) ? 1U : 0U;
            }
        }

        BMS_ProtectSyncState();
        protect_state.output_safe_confirmed =
            BMS_ControlApplyProtectState(&protect_state);
        if (protect_state.fail_safe_active && !protect_state.safe_off_confirmed)
        {
            protect_state.output_safe_confirmed = 0U;
        }
        return;
    }

    invalid_sample_count = 0U;
    if (protect_state.fail_safe_active)
    {
        recovery_sample_count++;
        if (recovery_sample_count < BMS_MONITOR_RECOVERY_COUNT)
        {
            BMS_ProtectSyncState();
            protect_state.output_safe_confirmed =
                BMS_ControlApplyProtectState(&protect_state);
            if (!protect_state.safe_off_confirmed)
            {
                protect_state.output_safe_confirmed = 0U;
            }
            return;
        }
        protect_state.fail_safe_active = 0U;
        protect_state.safe_off_confirmed = 0U;
        recovery_sample_count = 0U;
        BMS_LOGI("PROTECT", "monitor fail-safe recovered");
    }

    discharge_current_ma = (mon->current_ma < 0) ? -mon->current_ma : 0;
    charge_current_ma = (mon->current_ma > 0) ? mon->current_ma : 0;
    min_temp_c = mon->temp_c[0];
    max_temp_c = mon->temp_c[0];

    for (i = 0U; i < mon->cell_count; i++)
    {
        if ((ov_cell == 0U) && (mon->cell_mv[i] >= (uint16_t)PROT_OV_MV))
        {
            ov_cell = (uint8_t)(i + 1U);
        }
        if ((uv_cell == 0U) && (mon->cell_mv[i] <= (uint16_t)PROT_UV_MV))
        {
            uv_cell = (uint8_t)(i + 1U);
        }
    }
    for (i = 1U; i < mon->temp_count; i++)
    {
        if (mon->temp_c[i] < min_temp_c)
        {
            min_temp_c = mon->temp_c[i];
        }
        if (mon->temp_c[i] > max_temp_c)
        {
            max_temp_c = mon->temp_c[i];
        }
    }

    transition = BMS_ProtectUpdateGuard(&ov_guard, ov_cell != 0U,
                                        mon->max_cell_mv <= (PROT_OV_MV - PROTECT_OV_RELEASE_DELTA_MV),
                                        (uint32_t)SOFT_PROT_OV_DELAY_MS, 0U, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_OV, ov_cell,
                               mon->cell_mv[ov_cell - 1U], &protect_state.ov_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "OV recovered");
    }

    transition = BMS_ProtectUpdateGuard(&uv_guard, uv_cell != 0U,
                                        mon->min_cell_mv >= (PROT_UV_MV + PROTECT_UV_RELEASE_DELTA_MV),
                                        (uint32_t)SOFT_PROT_UV_DELAY_MS, 0U, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_UV, uv_cell,
                               mon->cell_mv[uv_cell - 1U], &protect_state.uv_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "UV recovered");
    }

    transition = BMS_ProtectUpdateGuard(&ocd_guard,
                                        discharge_current_ma >= PROT_OCD_MA,
                                        discharge_current_ma <= (PROT_OCD_MA - PROTECT_OC_RELEASE_DELTA_MA),
                                        (uint32_t)SOFT_PROT_OCD_DELAY_MS,
                                        PROTECT_OCD_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_OCD, 0U, discharge_current_ma,
                               &protect_state.ocd_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "OCD recovered");
    }

    transition = BMS_ProtectUpdateGuard(&scd_guard,
                                        discharge_current_ma >= PROT_SCD_MA,
                                        discharge_current_ma <= (PROT_SCD_MA - PROTECT_OC_RELEASE_DELTA_MA),
                                        (uint32_t)SOFT_PROT_SCD_DELAY_MS,
                                        PROTECT_SCD_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_SCD, 0U, discharge_current_ma,
                               &protect_state.scd_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "SCD recovered");
    }

    transition = BMS_ProtectUpdateGuard(&occ_guard,
                                        charge_current_ma >= INIT_OCC_PROTECT_MA,
                                        charge_current_ma <= INIT_OCC_RELEASE_MA,
                                        INIT_OCC_DELAY_MS, INIT_OCC_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_OCC, 0U, charge_current_ma,
                               &protect_state.occ_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "OCC recovered");
    }

    transition = BMS_ProtectUpdateGuard(&otc_guard,
                                        max_temp_c >= INIT_OTC_PROTECT_C,
                                        max_temp_c <= INIT_OTC_RELEASE_C,
                                        INIT_TEMP_DELAY_MS, INIT_TEMP_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_OTC, 0U, max_temp_c,
                               &protect_state.otc_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "OTC recovered");
    }

    transition = BMS_ProtectUpdateGuard(&otd_guard,
                                        max_temp_c >= INIT_OTD_PROTECT_C,
                                        max_temp_c <= INIT_OTD_RELEASE_C,
                                        INIT_TEMP_DELAY_MS, INIT_TEMP_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_OTD, 0U, max_temp_c,
                               &protect_state.otd_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "OTD recovered");
    }

    transition = BMS_ProtectUpdateGuard(&ltc_guard,
                                        min_temp_c <= INIT_LTC_PROTECT_C,
                                        min_temp_c >= INIT_LTC_RELEASE_C,
                                        INIT_TEMP_DELAY_MS, INIT_TEMP_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_LTC, 0U, min_temp_c,
                               &protect_state.ltc_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "LTC recovered");
    }

    transition = BMS_ProtectUpdateGuard(&ltd_guard,
                                        min_temp_c <= INIT_LTD_PROTECT_C,
                                        min_temp_c >= INIT_LTD_RELEASE_C,
                                        INIT_TEMP_DELAY_MS, INIT_TEMP_RECOVERY_MS, now);
    if (transition == BMS_GUARD_TRIGGERED)
    {
        BMS_ProtectRecordFault(BMS_PROTECT_FAULT_LTD, 0U, min_temp_c,
                               &protect_state.ltd_trigger_count);
    }
    else if (transition == BMS_GUARD_RECOVERED)
    {
        BMS_LOGI("PROTECT", "LTD recovered");
    }

    BMS_ProtectSyncState();
    protect_state.output_safe_confirmed =
        BMS_ControlApplyProtectState(&protect_state);

    if (mon->min_cell_mv <= (uint16_t)PROT_SHUTDOWN_MV)
    {
        if (shutdown_pending_tick == 0U)
        {
            shutdown_pending_tick = now;
        }
        else if ((uint32_t)(now - shutdown_pending_tick) >= PROTECT_SHUTDOWN_DELAY_MS)
        {
            if (BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT))
            {
                if (BQ769X0_EntryShip())
                {
                    BMS_ProtectLatchShutdown();
                }
            }
        }
    }
    else
    {
        shutdown_pending_tick = 0U;
    }
}

/* 锁存已进入 Ship 模式的终止状态，禁止后续重新开启 MOS。 */
void BMS_ProtectLatchShutdown(void)
{
    protect_state.shutdown_active = 1U;
    protect_state.safe_off_confirmed = 1U;
    BMS_ProtectSyncState();
    protect_state.output_safe_confirmed = 1U;
    BMS_LOGW("PROTECT", "shutdown latched");
}

/* 获取当前保护状态的只读指针。 */
const BMS_ProtectState_t *BMS_ProtectGetState(void)
{
    return &protect_state;
}

/* 查询是否存在任一激活的保护。 */
uint8_t BMS_ProtectIsAnyActive(void) { return protect_state.any_active; }

/* 查询单体过压保护是否激活。 */
uint8_t BMS_ProtectIsOvActive(void) { return protect_state.ov_active; }

/* 查询单体欠压保护是否激活。 */
uint8_t BMS_ProtectIsUvActive(void) { return protect_state.uv_active; }

/* 查询放电过流保护是否激活。 */
uint8_t BMS_ProtectIsOcdActive(void) { return protect_state.ocd_active; }

/* 查询放电短路保护是否激活。 */
uint8_t BMS_ProtectIsScdActive(void) { return protect_state.scd_active; }

/* 查询当前是否允许充电。 */
uint8_t BMS_ProtectIsChargeAllowed(void) { return protect_state.charge_allowed; }

/* 查询当前是否允许放电。 */
uint8_t BMS_ProtectIsDischargeAllowed(void) { return protect_state.discharge_allowed; }

/* 获取最近一次触发的保护类型。 */
BMS_ProtectFault_t BMS_ProtectGetLastFault(void) { return protect_state.last_fault; }

/* 将保护类型转换为便于日志显示的字符串。 */
const char *BMS_ProtectFaultToString(BMS_ProtectFault_t fault)
{
    switch (fault)
    {
        case BMS_PROTECT_FAULT_NONE:    return "NONE";
        case BMS_PROTECT_FAULT_OV:      return "OV";
        case BMS_PROTECT_FAULT_UV:      return "UV";
        case BMS_PROTECT_FAULT_OCD:     return "OCD";
        case BMS_PROTECT_FAULT_SCD:     return "SCD";
        case BMS_PROTECT_FAULT_OCC:     return "OCC";
        case BMS_PROTECT_FAULT_OTC:     return "OTC";
        case BMS_PROTECT_FAULT_OTD:     return "OTD";
        case BMS_PROTECT_FAULT_LTC:     return "LTC";
        case BMS_PROTECT_FAULT_LTD:     return "LTD";
        case BMS_PROTECT_FAULT_MONITOR: return "MONITOR";
        default:                        return "UNKNOWN";
    }
}

/* 设置单体过压阈值，单位为毫伏。 */
void BMS_ProtectSetOVMv(int ov_mv) { PROT_OV_MV = ov_mv; }

/* 设置单体欠压阈值，单位为毫伏。 */
void BMS_ProtectSetUVMv(int uv_mv) { PROT_UV_MV = uv_mv; }

/* 设置放电过流阈值，单位为毫安。 */
void BMS_ProtectSetOCDMa(int ocd_ma) { PROT_OCD_MA = ocd_ma; }

/* 设置放电短路阈值，单位为毫安。 */
void BMS_ProtectSetSCDMa(int scd_ma) { PROT_SCD_MA = scd_ma; }

/* 设置单体过压确认延时，单位为毫秒。 */
void BMS_ProtectSetOVDelayMs(int delay_ms) { SOFT_PROT_OV_DELAY_MS = delay_ms; }

/* 设置单体欠压确认延时，单位为毫秒。 */
void BMS_ProtectSetUVDelayMs(int delay_ms) { SOFT_PROT_UV_DELAY_MS = delay_ms; }

/* 设置放电过流确认延时，单位为毫秒。 */
void BMS_ProtectSetOCDDelayMs(int delay_ms) { SOFT_PROT_OCD_DELAY_MS = delay_ms; }

/* 设置放电短路确认延时，单位为毫秒。 */
void BMS_ProtectSetSCDDelayMs(int delay_ms) { SOFT_PROT_SCD_DELAY_MS = delay_ms; }

/* 获取单体过压阈值，单位为毫伏。 */
int BMS_ProtectGetOVMv(void) { return PROT_OV_MV; }

/* 获取单体欠压阈值，单位为毫伏。 */
int BMS_ProtectGetUVMv(void) { return PROT_UV_MV; }

/* 获取放电过流阈值，单位为毫安。 */
int BMS_ProtectGetOCDMa(void) { return PROT_OCD_MA; }

/* 获取放电短路阈值，单位为毫安。 */
int BMS_ProtectGetSCDMa(void) { return PROT_SCD_MA; }

/* 获取单体过压确认延时，单位为毫秒。 */
int BMS_ProtectGetOVDelayMs(void) { return SOFT_PROT_OV_DELAY_MS; }

/* 获取单体欠压确认延时，单位为毫秒。 */
int BMS_ProtectGetUVDelayMs(void) { return SOFT_PROT_UV_DELAY_MS; }

/* 获取放电过流确认延时，单位为毫秒。 */
int BMS_ProtectGetOCDDelayMs(void) { return SOFT_PROT_OCD_DELAY_MS; }

/* 获取放电短路确认延时，单位为毫秒。 */
int BMS_ProtectGetSCDDelayMs(void) { return SOFT_PROT_SCD_DELAY_MS; }
