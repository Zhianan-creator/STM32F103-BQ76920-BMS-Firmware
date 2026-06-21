#ifndef BMS_ALERT_TASK_H
#define BMS_ALERT_TASK_H

/*
 * BMS ALERT 任务入口
 *
 * 功能*   - 注册当前任务BQ769X0 ALERT 中断的接收目*   - 阻塞等待 ISR 任务通知（xTaskNotifyGive） *   - 唤醒后调BQ769X0_ProcessAlert() 读取 SYS_STAT 并处*
 * 注意*   - ISR 中仅设置 volatile 标志 + vTaskNotifyGiveFromISR，不I2C
 *   - 所SYS_STAT 读取、故障打印、状态清除均在任务上下文中执*/
void BMS_AlertTask(void *argument);

#endif /* BMS_ALERT_TASK_H */
