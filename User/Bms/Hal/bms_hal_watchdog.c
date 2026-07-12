#include "bms_hal_watchdog.h"

#include "stm32f1xx.h"

#define BMS_IWDG_LSI_HZ             40000U
#define BMS_IWDG_PRESCALER_DIV        256U
#define BMS_IWDG_PRESCALER_CODE         6U
#define BMS_IWDG_RELOAD_MAX          4095U
#define BMS_IWDG_KEY_ENABLE        0xCCCCU
#define BMS_IWDG_KEY_WRITE_ACCESS  0x5555U
#define BMS_IWDG_KEY_REFRESH       0xAAAAU
#define BMS_IWDG_UPDATE_TIMEOUT  1000000U

/* 根据毫秒参数配置独立看门狗的分频和重装载值。 */
static bool BMS_HAL_WatchdogConfigure(uint32_t timeout_ms)
{
    uint32_t watchdog_ticks;
    uint32_t reload;
    uint32_t wait_count = BMS_IWDG_UPDATE_TIMEOUT;

    watchdog_ticks = (timeout_ms * BMS_IWDG_LSI_HZ) /
                     (BMS_IWDG_PRESCALER_DIV * 1000U);
    if ((watchdog_ticks == 0U) || (watchdog_ticks > (BMS_IWDG_RELOAD_MAX + 1U)))
    {
        return false;
    }
    reload = watchdog_ticks - 1U;

    IWDG->KR = BMS_IWDG_KEY_WRITE_ACCESS;
    IWDG->PR = BMS_IWDG_PRESCALER_CODE;
    IWDG->RLR = reload;

    while ((IWDG->SR != 0U) && (wait_count > 0U))
    {
        wait_count--;
    }

    IWDG->KR = BMS_IWDG_KEY_REFRESH;
    return (wait_count > 0U);
}

/* 启动独立看门狗，并按毫秒参数设置首次超时时间。 */
bool BMS_HAL_WatchdogStart(uint32_t timeout_ms)
{
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
    IWDG->KR = BMS_IWDG_KEY_ENABLE;
    return BMS_HAL_WatchdogConfigure(timeout_ms);
}

/* 更新独立看门狗超时时间。 */
bool BMS_HAL_WatchdogSetTimeout(uint32_t timeout_ms)
{
    return BMS_HAL_WatchdogConfigure(timeout_ms);
}

/* 刷新独立看门狗计数器。 */
void BMS_HAL_WatchdogRefresh(void)
{
    IWDG->KR = BMS_IWDG_KEY_REFRESH;
}
