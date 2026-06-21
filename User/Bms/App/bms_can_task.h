#ifndef BMS_CAN_TASK_H
#define BMS_CAN_TASK_H

#include "cmsis_os2.h"
#include <stdint.h>

extern const osThreadAttr_t bmsCanTask_attributes;

void BMS_CAN_Task(void *argument);

typedef enum
{
    BMS_CAN_POWER_STARTED = 0,
    BMS_CAN_POWER_ALREADY_ON,
    BMS_CAN_POWER_START_FAILED,
    BMS_CAN_POWER_STOPPED,
    BMS_CAN_POWER_ALREADY_OFF
} BMS_CAN_PowerResult_t;

/* Expose functions to control and query CAN transceiver power state */
BMS_CAN_PowerResult_t BMS_CAN_SetPowerState(uint8_t enable);
uint8_t BMS_CAN_GetPowerState(void);

#endif /* BMS_CAN_TASK_H */
