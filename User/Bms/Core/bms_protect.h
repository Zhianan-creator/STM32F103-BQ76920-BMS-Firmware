#ifndef BMS_PROTECT_H
#define BMS_PROTECT_H

#include <stdint.h>

/* BMS 软件保护故障类型。 */
typedef enum
{
    BMS_PROTECT_FAULT_NONE = 0,
    BMS_PROTECT_FAULT_OV,
    BMS_PROTECT_FAULT_UV,
    BMS_PROTECT_FAULT_OCD,
    BMS_PROTECT_FAULT_SCD,
    BMS_PROTECT_FAULT_OCC,
    BMS_PROTECT_FAULT_OTC,
    BMS_PROTECT_FAULT_OTD,
    BMS_PROTECT_FAULT_LTC,
    BMS_PROTECT_FAULT_LTD,
    BMS_PROTECT_FAULT_DEVICE,
    BMS_PROTECT_FAULT_OVRD,
    BMS_PROTECT_FAULT_MONITOR
} BMS_ProtectFault_t;

/* BMS 软件保护状态。 */
typedef struct
{
    uint8_t ov_active;
    uint8_t uv_active;
    uint8_t ocd_active;
    uint8_t scd_active;
    uint8_t occ_active;
    uint8_t otc_active;
    uint8_t otd_active;
    uint8_t ltc_active;
    uint8_t ltd_active;
    uint8_t fail_safe_active;
    uint8_t device_fault_active;
    uint8_t ovrd_active;
    uint8_t discharge_rearm_required;
    uint8_t device_restart_required;
    uint8_t safe_off_confirmed;
    uint8_t shutdown_active;
    uint8_t output_safe_confirmed;

    uint8_t ov_pending;
    uint8_t uv_pending;
    uint8_t ocd_pending;
    uint8_t scd_pending;
    uint8_t occ_pending;
    uint8_t otc_pending;
    uint8_t otd_pending;
    uint8_t ltc_pending;
    uint8_t ltd_pending;

    uint8_t any_active;
    uint8_t charge_allowed;
    uint8_t discharge_allowed;

    BMS_ProtectFault_t last_fault;
    uint8_t last_fault_cell;
    int32_t last_fault_value;

    uint32_t ov_trigger_count;
    uint32_t uv_trigger_count;
    uint32_t ocd_trigger_count;
    uint32_t scd_trigger_count;
    uint32_t occ_trigger_count;
    uint32_t otc_trigger_count;
    uint32_t otd_trigger_count;
    uint32_t ltc_trigger_count;
    uint32_t ltd_trigger_count;
    uint32_t device_trigger_count;
    uint32_t ovrd_trigger_count;
    uint32_t fail_safe_trigger_count;
} BMS_ProtectState_t;

/* 初始化保护阈值、计时器和状态。 */
void BMS_ProtectInit(void);

/* 根据最新监控数据更新全部软件保护，并执行必要的安全关断。 */
void BMS_ProtectUpdateFromMonitor(void);

/* 锁存已进入 Ship 模式的终止状态，禁止后续重新开启 MOS。 */
void BMS_ProtectLatchShutdown(void);

/* Persist a BQ hardware fault before its W1C status bit is cleared. */
void BMS_ProtectLatchHardwareFault(BMS_ProtectFault_t fault);

/* Explicitly rearm OCD/SCD only after cooldown and confirmed load removal. */
uint8_t BMS_ProtectTryRearmDischarge(void);

/* 获取当前保护状态的只读指针。 */
const BMS_ProtectState_t *BMS_ProtectGetState(void);

/* 查询是否存在任一激活的保护。 */
uint8_t BMS_ProtectIsAnyActive(void);

/* 查询单体过压保护是否激活。 */
uint8_t BMS_ProtectIsOvActive(void);

/* 查询单体欠压保护是否激活。 */
uint8_t BMS_ProtectIsUvActive(void);

/* 查询放电过流保护是否激活。 */
uint8_t BMS_ProtectIsOcdActive(void);

/* 查询放电短路保护是否激活。 */
uint8_t BMS_ProtectIsScdActive(void);

/* 查询当前是否允许充电。 */
uint8_t BMS_ProtectIsChargeAllowed(void);

/* 查询当前是否允许放电。 */
uint8_t BMS_ProtectIsDischargeAllowed(void);

/* 获取最近一次触发的保护类型。 */
BMS_ProtectFault_t BMS_ProtectGetLastFault(void);

/* 将保护类型转换为便于日志显示的字符串。 */
const char *BMS_ProtectFaultToString(BMS_ProtectFault_t fault);

/* 设置单体过压阈值，单位为毫伏。 */
void BMS_ProtectSetOVMv(int ov_mv);

/* 设置单体欠压阈值，单位为毫伏。 */
void BMS_ProtectSetUVMv(int uv_mv);

/* 设置放电过流阈值，单位为毫安。 */
void BMS_ProtectSetOCDMa(int ocd_ma);

/* 设置放电短路阈值，单位为毫安。 */
void BMS_ProtectSetSCDMa(int scd_ma);

/* 设置单体过压确认延时，单位为毫秒。 */
void BMS_ProtectSetOVDelayMs(int delay_ms);

/* 设置单体欠压确认延时，单位为毫秒。 */
void BMS_ProtectSetUVDelayMs(int delay_ms);

/* 设置放电过流确认延时，单位为毫秒。 */
void BMS_ProtectSetOCDDelayMs(int delay_ms);

/* 设置放电短路确认延时，单位为毫秒。 */
void BMS_ProtectSetSCDDelayMs(int delay_ms);

/* 获取单体过压阈值，单位为毫伏。 */
int BMS_ProtectGetOVMv(void);

/* 获取单体欠压阈值，单位为毫伏。 */
int BMS_ProtectGetUVMv(void);

/* 获取放电过流阈值，单位为毫安。 */
int BMS_ProtectGetOCDMa(void);

/* 获取放电短路阈值，单位为毫安。 */
int BMS_ProtectGetSCDMa(void);

/* 获取单体过压确认延时，单位为毫秒。 */
int BMS_ProtectGetOVDelayMs(void);

/* 获取单体欠压确认延时，单位为毫秒。 */
int BMS_ProtectGetUVDelayMs(void);

/* 获取放电过流确认延时，单位为毫秒。 */
int BMS_ProtectGetOCDDelayMs(void);

/* 获取放电短路确认延时，单位为毫秒。 */
int BMS_ProtectGetSCDDelayMs(void);

#endif /* BMS_PROTECT_H */
