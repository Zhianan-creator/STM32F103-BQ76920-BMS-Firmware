#include "bms_alert_task.h"
#include "bms_init.h"
#include "bms_config.h"
#include "bms_protect.h"
#include "drv_softi2c_bq769x0.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* 接收 ALERT 中断通知，并通过定时轮询兜底处理保持为高电平的告警。 */
void BMS_AlertTask(void *argument)
{
    (void)argument;

    /* 注册当前任务ALERT 中断的接收目标 */
    TaskHandle_t xTaskHandle = xTaskGetCurrentTaskHandle();
    BQ769X0_RegisterAlertTask(xTaskHandle);

    if (BMS_InitWaitDone() != 0)
    {
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(BMS_ALERT_POLL_MS));
        }
    }

    for (;;)
    {
        const BMS_ProtectState_t *protect = BMS_ProtectGetState();

        if ((protect != NULL) && protect->shutdown_active)
        {
            vTaskDelay(pdMS_TO_TICKS(BMS_ALERT_POLL_MS));
            continue;
        }
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BMS_ALERT_POLL_MS));
        (void)BQ769X0_ProcessAlert();
    }
}
