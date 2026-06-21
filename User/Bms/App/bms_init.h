#ifndef BMS_INIT_H
#define BMS_INIT_H

#include <stdint.h>

/* BQ769X0 初始化结果结构体 */
typedef struct
{
    uint8_t uart_ok;    /* UART 输出测试通过 */
    uint8_t i2c_ok;     /* I2C 总线初始化通过 */
    uint8_t ack_ok;     /* BQ769X0 地址应答通过 */
    uint8_t crc_ok;     /* 寄存CRC 校验全部通过 */
    uint8_t init_ok;    /* BQ769X0 完整初始化通过 */
} BMS_InitResult_t;

/*
 * 执行完整BQ769X0 初始化验证流 *
 * 流程: UART banner I2C 初始ACK 检CRC 校验 ->BQ769X0 初始化 * 每一步失败都会打印错误信息和排查建议
 *
 * 返回: 0=全部通过, -1=有步骤失 */
int BMS_InitRun(void);

/* 获取初始化结果结构体指针 */
const BMS_InitResult_t *BMS_InitGetResult(void);

/* 查询初始化是否全部通过 */
uint8_t BMS_InitIsOk(void);

/*
 * 阻塞等待 BQ 初始化完 *
 * 在采样任务中调用，替代固osDelay()
 * 只有BMS_InitRun() 完成后才会返 */
void BMS_InitWaitDone(void);

#endif /* BMS_INIT_H */
