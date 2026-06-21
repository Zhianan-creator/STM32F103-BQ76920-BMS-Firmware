#ifndef BMS_SAMPLE_TASK_H
#define BMS_SAMPLE_TASK_H

/*
 * BMS 周期采样任务入口
 *
 * 功能*   - 注册 ALERT 任务通知
 *   - 周期采样（BMS_MonitorUpdate） *   - ALERT 处理（BQ769X0_ProcessAlert） *   - 保护状态机（BMS_ProtectUpdateFromMonitor） *   - 首次采样打印 + 控制测试 + 安全关闭
 *
 * 参数：FreeRTOS task argument（未使用*/
void BMS_SampleTask(void *argument);

#endif /* BMS_SAMPLE_TASK_H */
