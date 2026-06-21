#include "bms_init.h"
#include "bms_config.h"
#include "drv_soft_i2c.h"
#include "drv_softi2c_bq769x0.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* 内部保存初始化结果 */
static BMS_InitResult_t init_result = {0};

/* 等待初始化完成的任务通知 */
static TaskHandle_t bms_init_done_task = NULL;

int BMS_InitRun(void)
{
    uint8_t crc_all_ok = 1;
    BQ769X0_InitDataTypedef bq_init_data = {0};

    memset(&init_result, 0, sizeof(BMS_InitResult_t));

    /* 初始化BQ 总线互斥锁（必须在任何I2C 操作前） */
    BQ769X0_BusLockInit();

    init_result.uart_ok = 1;
    osDelay(100);

    /* ============================================================
     * 步骤2: 软件I2C总线初始化
     * ============================================================ */
    if (I2C_BusInitialize() == 0)
    {
        printf("[I2C] Bus init OK (SDA=PB13, SCL=PB14)\r\n");
        init_result.i2c_ok = 1;
    }
    else
    {
        printf("[I2C] Bus init FAILED! Check FreeRTOS heap size.\r\n");
        printf("[I2C] configTOTAL_HEAP_SIZE in FreeRTOSConfig.h\r\n");
        return -1;
    }
    osDelay(100);

    /* ============================================================
     * 步骤3: BQ769X0地址应答检测
     * ============================================================ */
    if (I2C_BusCheckAddress(&i2c1, BQ769X0_I2C_ADDR))
    {
        init_result.ack_ok = 1;
    }
    else
    {
        printf("[BQ] Address NACK! BQ769X0 not responding.\r\n");
        return -1;
    }
    osDelay(100);

    /* ============================================================
     * 步骤4: CRC方式读取基础寄存器并验证
     * ============================================================ */
    crc_all_ok &= BQ769X0_VerifyRegisterCRC(ADCGAIN1,  NULL);
    osDelay(50);
    crc_all_ok &= BQ769X0_VerifyRegisterCRC(ADCOFFSET, NULL);
    osDelay(50);
    crc_all_ok &= BQ769X0_VerifyRegisterCRC(ADCGAIN2,  NULL);
    osDelay(50);
    crc_all_ok &= BQ769X0_VerifyRegisterCRC(SYS_STAT,  NULL);
    osDelay(50);

    init_result.crc_ok = crc_all_ok;
    osDelay(200);

    /* ============================================================
     * 步骤5: BQ769X0完整初始化
     * ============================================================ */
    bq_init_data.AlertOps.ocd    = NULL;
    bq_init_data.AlertOps.scd    = NULL;
    bq_init_data.AlertOps.ov     = NULL;
    bq_init_data.AlertOps.uv     = NULL;
    bq_init_data.AlertOps.ovrd   = NULL;
    bq_init_data.AlertOps.device = NULL;
    bq_init_data.AlertOps.cc     = NULL;

    bq_init_data.ConfigData.SCDDelay     = (BQ769X0_SCDDelayTypedef)INIT_SCD_DELAY;
    bq_init_data.ConfigData.OCDDelay     = (BQ769X0_OCDDelayTypedef)INIT_OCD_DELAY;
    bq_init_data.ConfigData.OVDelay      = (BQ769X0_OVDelayTypedef)INIT_OV_DELAY;
    bq_init_data.ConfigData.UVDelay      = (BQ769X0_UVDelayTypedef)INIT_UV_DELAY;
    bq_init_data.ConfigData.OVPThreshold = (uint16_t)(INIT_OV_PROTECT * 1000);
    bq_init_data.ConfigData.UVPThreshold = (uint16_t)(INIT_UV_PROTECT * 1000);

    BQ769X0_Initialize(&bq_init_data);

    /* 如果执行到这说明初始化成功(否则会在Configuration中死循环) */
    init_result.init_ok = 1;

    /* 通知等待中的任务：BQ 初始化已完成 */
    if (bms_init_done_task != NULL)
    {
        xTaskNotifyGive(bms_init_done_task);
    }

    return 0;
}

void BMS_InitWaitDone(void)
{
    bms_init_done_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

const BMS_InitResult_t *BMS_InitGetResult(void)
{
    return &init_result;
}

uint8_t BMS_InitIsOk(void)
{
    return init_result.init_ok;
}
