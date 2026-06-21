#include "bms_alert_task.h"
#include "drv_softi2c_bq769x0.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

void BMS_AlertTask(void *argument)
{
    (void)argument;

    /* 注册当前任务ALERT 中断的接收目标 */
    TaskHandle_t xTaskHandle = xTaskGetCurrentTaskHandle();
    BQ769X0_RegisterAlertTask(xTaskHandle);

    for (;;)
    {
        /*
         * 无限阻塞等待 ISR 任务通知
         * ISR (HAL_GPIO_EXTI_Callback) 中通过 vTaskNotifyGiveFromISR 唤醒
         * 唤醒后在任务上下文中安全读取 SYS_STAT 寄存*/
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        BQ769X0_ProcessAlert();
    }
}
