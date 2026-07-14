#include "bms_init.h"
#include "bms_config.h"
#include "drv_soft_i2c.h"
#include "drv_softi2c_bq769x0.h"
#include "bms_log.h"
#include "bms_monitor.h"
#include "bms_protect.h"
#include "bms_energy.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdio.h>
#include <string.h>

#define BMS_INIT_EVENT_DONE  (1U << 0)
#define BMS_INIT_EVENT_OK    (1U << 1)

static BMS_InitResult_t init_result;
static StaticEventGroup_t init_event_buffer;
static EventGroupHandle_t init_event;

/* ALERT callbacks run in the alert task, never in the GPIO ISR. */
static void BMS_InitLatchHardwareFault(BMS_ProtectFault_t fault)
{
    BMS_EnergySetChargeEnable(0U);
    (void)BMS_EnergySetDischargeEnable(0U);
    BMS_ProtectLatchHardwareFault(fault);
}

static void BMS_InitHandleOcd(void)    { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_OCD); }
static void BMS_InitHandleScd(void)    { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_SCD); }
static void BMS_InitHandleOv(void)     { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_OV); }
static void BMS_InitHandleUv(void)     { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_UV); }
static void BMS_InitHandleOvrd(void)   { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_OVRD); }
static void BMS_InitHandleDevice(void) { BMS_InitLatchHardwareFault(BMS_PROTECT_FAULT_DEVICE); }

/* 填充 BQ769X0 的硬件保护初始化参数。 */
static void BMS_InitFillBqConfig(BQ769X0_InitDataTypedef *config)
{
    memset(config, 0, sizeof(*config));
    config->ConfigData.SCDDelay = (BQ769X0_SCDDelayTypedef)INIT_SCD_DELAY;
    config->ConfigData.OCDDelay = (BQ769X0_OCDDelayTypedef)INIT_OCD_DELAY;
    config->ConfigData.OVDelay = (BQ769X0_OVDelayTypedef)INIT_OV_DELAY;
    config->ConfigData.UVDelay = (BQ769X0_UVDelayTypedef)INIT_UV_DELAY;
    config->ConfigData.OVPThreshold = (uint16_t)(INIT_OV_PROTECT * 1000.0f);
    config->ConfigData.UVPThreshold = (uint16_t)(INIT_HW_UV_PROTECT * 1000.0f);
    config->AlertOps.ocd = BMS_InitHandleOcd;
    config->AlertOps.scd = BMS_InitHandleScd;
    config->AlertOps.ov = BMS_InitHandleOv;
    config->AlertOps.uv = BMS_InitHandleUv;
    config->AlertOps.ovrd = BMS_InitHandleOvrd;
    config->AlertOps.device = BMS_InitHandleDevice;
}

/* 执行一次 I2C、地址、CRC 和 BQ 配置验证。 */
static int BMS_InitTryOnce(uint8_t wake_first)
{
    BQ769X0_InitDataTypedef config;
    uint8_t crc_ok = 1U;

    init_result.i2c_ok = 0U;
    init_result.ack_ok = 0U;
    init_result.crc_ok = 0U;
    init_result.init_ok = 0U;

    if (wake_first)
    {
        BQ769X0_Wakeup();
    }

    if (I2C_BusInitialize() != 0)
    {
        printf("[I2C] Bus init failed\r\n");
        return -1;
    }
    init_result.i2c_ok = 1U;

    if (!I2C_BusCheckAddress(&i2c1, BQ769X0_I2C_ADDR))
    {
        printf("[BQ] Address NACK\r\n");
        return -1;
    }
    init_result.ack_ok = 1U;

    crc_ok &= BQ769X0_VerifyRegisterCRC(ADCGAIN1, NULL);
    crc_ok &= BQ769X0_VerifyRegisterCRC(ADCOFFSET, NULL);
    crc_ok &= BQ769X0_VerifyRegisterCRC(ADCGAIN2, NULL);
    crc_ok &= BQ769X0_VerifyRegisterCRC(SYS_STAT, NULL);
    if (!crc_ok)
    {
        printf("[BQ] CRC verification failed\r\n");
        return -1;
    }
    init_result.crc_ok = 1U;

    BMS_InitFillBqConfig(&config);
    if (!BQ769X0_Initialize(&config))
    {
        printf("[BQ] Configuration failed\r\n");
        return -1;
    }
    init_result.init_ok = 1U;
    return 0;
}

/* 在创建任务前准备可供多个任务等待的初始化事件。 */
uint8_t BMS_InitPrepare(void)
{
    if (init_event == NULL)
    {
        init_event = xEventGroupCreateStatic(&init_event_buffer);
    }
    return (init_event != NULL) ? 1U : 0U;
}

/* 有界重试执行完整的 BQ769X0 初始化流程。 */
int BMS_InitRun(void)
{
    uint8_t attempt;

    memset(&init_result, 0, sizeof(init_result));
    init_result.uart_ok = 1U;

    if (!BMS_InitPrepare() || !BQ769X0_BusLockInit())
    {
        init_result.done = 1U;
        if (init_event != NULL)
        {
            xEventGroupSetBits(init_event, BMS_INIT_EVENT_DONE);
        }
        return -1;
    }

    xEventGroupClearBits(init_event, BMS_INIT_EVENT_DONE | BMS_INIT_EVENT_OK);
    for (attempt = 1U; attempt <= BMS_INIT_RETRY_COUNT; attempt++)
    {
        init_result.attempts = attempt;
        if (BMS_InitTryOnce(attempt > 1U) == 0)
        {
            init_result.safe_off_ok =
                BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT) ? 1U : 0U;
            if (init_result.safe_off_ok)
            {
                /* Initialize consumers before waking the higher-priority ALERT task. */
                BMS_LogInit();
                BMS_MonitorInit();
                BMS_ProtectInit();
                BMS_EnergyInit();
                init_result.done = 1U;
                xEventGroupSetBits(init_event, BMS_INIT_EVENT_DONE | BMS_INIT_EVENT_OK);
                printf("[BQ] Init OK (attempt %u)\r\n", (unsigned int)attempt);
                return 0;
            }
        }

        printf("[BQ] Init attempt %u/%u failed\r\n",
               (unsigned int)attempt, (unsigned int)BMS_INIT_RETRY_COUNT);
        if (attempt < BMS_INIT_RETRY_COUNT)
        {
            osDelay(BMS_INIT_RETRY_DELAY_MS);
        }
    }

    (void)BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT);
    init_result.done = 1U;
    xEventGroupSetBits(init_event, BMS_INIT_EVENT_DONE);
    return -1;
}

/* 阻塞等待初始化结束，并返回初始化是否成功。 */
int BMS_InitWaitDone(void)
{
    EventBits_t bits;

    if (init_event == NULL)
    {
        return -1;
    }
    bits = xEventGroupWaitBits(init_event, BMS_INIT_EVENT_DONE,
                               pdFALSE, pdTRUE, portMAX_DELAY);
    return ((bits & BMS_INIT_EVENT_OK) != 0U) ? 0 : -1;
}

/* 获取初始化结果的只读指针。 */
const BMS_InitResult_t *BMS_InitGetResult(void)
{
    return &init_result;
}

/* 查询 BQ769X0 是否已成功初始化。 */
uint8_t BMS_InitIsOk(void)
{
    return init_result.done && init_result.init_ok && init_result.safe_off_ok;
}
