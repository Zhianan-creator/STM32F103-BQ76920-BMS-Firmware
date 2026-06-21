#ifndef BMS_UART_CMD_H
#define BMS_UART_CMD_H

#include "main.h"

/* 串口命令层安全开关定义 */
#ifndef BMS_UART_CMD_ENABLE
#define BMS_UART_CMD_ENABLE          1
#endif

#ifndef BMS_UART_CMD_ALLOW_CHG_ON
#define BMS_UART_CMD_ALLOW_CHG_ON    1
#endif

#ifndef BMS_UART_CMD_ALLOW_DSG_ON
#define BMS_UART_CMD_ALLOW_DSG_ON    1
#endif

#ifndef BMS_UART_CMD_ACTIVE_HOLD_MS
#define BMS_UART_CMD_ACTIVE_HOLD_MS  5000
#endif

#if (BMS_UART_CMD_ENABLE != 0)
void BMS_UartCmd_Init(void);
void BMS_UartCmd_ProcessLine(const char *line);
void BMS_UartCmd_Task(void *argument);
uint8_t BMS_UartCmdIsActive(void);
#else
static inline void BMS_UartCmd_Init(void) {}
static inline void BMS_UartCmd_ProcessLine(const char *line) { (void)line; }
static inline void BMS_UartCmd_Task(void *argument) { (void)argument; }
static inline uint8_t BMS_UartCmdIsActive(void) { return 0; }
#endif

#endif /* BMS_UART_CMD_H */
