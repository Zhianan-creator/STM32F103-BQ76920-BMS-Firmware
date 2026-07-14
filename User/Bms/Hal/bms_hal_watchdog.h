#ifndef BMS_HAL_WATCHDOG_H
#define BMS_HAL_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* 启动独立看门狗；timeout_ms 为期望超时时间，配置成功返回 true。 */
bool BMS_HAL_WatchdogStart(uint32_t timeout_ms);

/* 更新独立看门狗超时时间；配置和重装载成功返回 true。 */
bool BMS_HAL_WatchdogSetTimeout(uint32_t timeout_ms);

/* 刷新独立看门狗计数器；仅由证明主任务持续运行的代码调用。 */
void BMS_HAL_WatchdogRefresh(void);

#endif /* BMS_HAL_WATCHDOG_H */
