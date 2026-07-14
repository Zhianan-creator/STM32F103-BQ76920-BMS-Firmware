#ifndef BMS_SAMPLE_TASK_H
#define BMS_SAMPLE_TASK_H

#include <stdint.h>

/* 运行周期采样、软件保护、SOC 分析和能量控制任务。 */
void BMS_SampleTask(void *argument);

/* 获取采样任务完成安全周期的累计次数。 */
uint32_t BMS_SampleTaskGetProgress(void);

#endif /* BMS_SAMPLE_TASK_H */
