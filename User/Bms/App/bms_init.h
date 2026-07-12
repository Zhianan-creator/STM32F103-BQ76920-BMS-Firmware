#ifndef BMS_INIT_H
#define BMS_INIT_H

#include <stdint.h>

/* BQ769X0 初始化结果。 */
typedef struct
{
    uint8_t uart_ok;
    uint8_t i2c_ok;
    uint8_t ack_ok;
    uint8_t crc_ok;
    uint8_t init_ok;
    uint8_t safe_off_ok;
    uint8_t done;
    uint8_t attempts;
} BMS_InitResult_t;

/* 在创建任务前准备可供多个任务等待的初始化事件。 */
uint8_t BMS_InitPrepare(void);

/* 有界重试执行完整的 BQ769X0 初始化流程。 */
int BMS_InitRun(void);

/* 阻塞等待初始化结束，并返回初始化是否成功。 */
int BMS_InitWaitDone(void);

/* 获取初始化结果的只读指针。 */
const BMS_InitResult_t *BMS_InitGetResult(void);

/* 查询 BQ769X0 是否已成功初始化。 */
uint8_t BMS_InitIsOk(void);

#endif /* BMS_INIT_H */
