#include "bms_can_task.h"

#include "can.h"
#include "bms_analysis.h"
#include "bms_control.h"
#include "bms_energy.h"
#include "bms_init.h"
#include "bms_monitor.h"
#include "bms_protect.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define BMS_CAN_CMD_ID             0x300U
#define BMS_CAN_STATUS_ID          0x180U

#define BMS_CAN_CMD_STATUS_REQ     0x01U
#define BMS_CAN_CMD_DSG_ON         0x10U
#define BMS_CAN_CMD_DSG_OFF        0x11U

#define BMS_CAN_RX_QUEUE_LEN       8U
#define BMS_CAN_TX_TIMEOUT_MS      10U
#define BMS_CAN_INIT_RETRY_MS      1000U
#define BMS_CAN_STATUS_TX_PERIOD_MS 150U

#define BMS_CAN_STATE_MON_VALID    0x01U
#define BMS_CAN_STATE_CHG_ALLOWED  0x02U
#define BMS_CAN_STATE_DSG_ALLOWED  0x04U
#define BMS_CAN_STATE_CHG_ON       0x08U
#define BMS_CAN_STATE_DSG_ON       0x10U
#define BMS_CAN_STATE_BAL_ACTIVE   0x20U
#define BMS_CAN_STATE_SLEEP_REQ    0x40U

#define BMS_CAN_FAULT_OV           0x01U
#define BMS_CAN_FAULT_UV           0x02U
#define BMS_CAN_FAULT_OCD          0x04U
#define BMS_CAN_FAULT_SCD          0x08U
#define BMS_CAN_FAULT_ANY          0x10U
#define BMS_CAN_FAULT_MON_INVALID  0x20U

/*
 * CAN 接收帧缓存。
 * 中断里只把 HAL 收到的帧复制到队列，真正解析命令放到 BMS_CAN_Task() 中执行，
 * 避免在 CAN RX ISR 中调用 BMS 控制、I2C 或发送接口。
 */
typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} BMS_CAN_RxFrame_t;

static QueueHandle_t can_rx_queue = NULL;
static uint8_t can_transceiver_power = 1;

static uint8_t BMS_CAN_RuntimeInit(void);
static void BMS_CAN_HandleFrame(const BMS_CAN_RxFrame_t *frame);
static void BMS_CAN_HandleCommand(uint8_t cmd);
static void BMS_CAN_HandleDsgOn(void);
static void BMS_CAN_HandleDsgOff(void);
static uint8_t BMS_CAN_SendStatus(void);
static uint8_t BMS_CAN_SendFrame(uint32_t std_id, const uint8_t data[8]);
static uint8_t BMS_CAN_BuildStateFlags(const BMS_MonitorData_t *mon,
                                       const BMS_ProtectState_t *prot,
                                       const BMS_EnergyState_t *energy);
static uint8_t BMS_CAN_BuildFaultFlags(const BMS_MonitorData_t *mon,
                                       const BMS_ProtectState_t *prot);
static uint16_t BMS_CAN_U16Scale10(uint16_t value);
static int16_t BMS_CAN_I16Scale10(int32_t value);
static int8_t BMS_CAN_I8Clamp(int16_t value);
static uint8_t BMS_CAN_SocPercent(void);

const osThreadAttr_t bmsCanTask_attributes = {
    .name = "bmsCanTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

/*
 * CAN 应用任务主循环：
 * 1. 延后启动，等待 BMS 初始化和底层外设稳定；
 * 2. 创建 RX 队列并初始化 CAN 滤波器/中断；
 * 3. 阻塞等待 ISR 投递的 CAN 帧，只在任务上下文中解析命令。
 */
void BMS_CAN_Task(void *argument)
{
    (void)argument;

    osDelay(1000);

    can_rx_queue = xQueueCreate(BMS_CAN_RX_QUEUE_LEN, sizeof(BMS_CAN_RxFrame_t));
    if (can_rx_queue == NULL)
    {
        for (;;)
        {
            osDelay(BMS_CAN_INIT_RETRY_MS);
        }
    }

    while (!BMS_CAN_RuntimeInit())
    {
        osDelay(BMS_CAN_INIT_RETRY_MS);
    }

    const TickType_t status_tx_period = pdMS_TO_TICKS(BMS_CAN_STATUS_TX_PERIOD_MS);
    TickType_t last_status_tx_tick = xTaskGetTickCount();

    for (;;)
    {
        BMS_CAN_RxFrame_t frame;
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - last_status_tx_tick;
        TickType_t wait_ticks = (elapsed >= status_tx_period) ? 0U : (status_tx_period - elapsed);

        if (xQueueReceive(can_rx_queue, &frame, wait_ticks) == pdTRUE)
        {
            BMS_CAN_HandleFrame(&frame);
        }

        now = xTaskGetTickCount();
        if ((now - last_status_tx_tick) >= status_tx_period)
        {
            /*
             * Periodically publish the existing 0x180 status frame.
             * data[4] carries the remaining battery charge in percent (0-100).
             */
            (void)BMS_CAN_SendStatus();
            last_status_tx_tick = now;
        }
    }
}

/*
 * CAN RX0 中断回调。
 * 这里不解析命令，只把帧放入 FreeRTOS 队列，然后请求任务切换。
 * 这样可以保持 ISR 短小，也避免在中断中直接打开/关闭 MOS。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_cb)
{
    if (hcan_cb->Instance != CAN1)
    {
        return;
    }

    BMS_CAN_RxFrame_t frame;
    if (HAL_CAN_GetRxMessage(hcan_cb, CAN_RX_FIFO0, &frame.header, frame.data) != HAL_OK)
    {
        return;
    }

    if (can_rx_queue != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(can_rx_queue, &frame, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * CAN 运行时初始化。
 * PB10 为 CAN 收发器电源控制脚，低电平使能；随后配置只接收 BMS_CAN_CMD_ID
 * 的标准数据帧，最后启动 CAN 和 RX FIFO0 消息中断。
 */
static uint8_t BMS_CAN_RuntimeInit(void)
{
    CAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;
    GPIO_InitTypeDef gpio_pwren = {0};

    /* Enable CAN Transceiver Power via PB10 (Active Low) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio_pwren.Pin = GPIO_PIN_10;
    gpio_pwren.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_pwren.Pull = GPIO_NOPULL;
    gpio_pwren.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_pwren);

    /* Pull PB10 low to enable CAN power */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

    /* Give transceiver 10ms to power up and stabilize RX line */
    osDelay(10);

    if (hcan.State != HAL_CAN_STATE_READY)
    {
        status = HAL_CAN_Init(&hcan);
        if (status != HAL_OK)
        {
            return 0;
        }
    }

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(BMS_CAN_CMD_ID << 5);
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    status = HAL_CAN_ConfigFilter(&hcan, &filter);
    if (status != HAL_OK)
    {
        return 0;
    }

    status = HAL_CAN_Start(&hcan);
    if (status != HAL_OK)
    {
        return 0;
    }

    status = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK)
    {
        return 0;
    }

    return 1;
}

/*
 * CAN 帧入口过滤。
 * 只接受标准帧、数据帧、指定命令 ID，并要求至少 1 字节命令码。
 * 通过过滤后，data[0] 才会被当作 BMS 命令处理。
 */
static void BMS_CAN_HandleFrame(const BMS_CAN_RxFrame_t *frame)
{
    if (frame == NULL)
    {
        return;
    }

    if (frame->header.IDE != CAN_ID_STD ||
        frame->header.RTR != CAN_RTR_DATA ||
        frame->header.StdId != BMS_CAN_CMD_ID ||
        frame->header.DLC < 1U)
    {
        return;
    }

    BMS_CAN_HandleCommand(frame->data[0]);
}

/*
 * 命令分发层。
 * 当前协议只实现状态查询、DSG 开启、DSG 关闭；每个控制命令执行后都会回传一帧状态，
 * 上位机可以通过状态位确认实际 MOS 状态和保护状态。
 */
static void BMS_CAN_HandleCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case BMS_CAN_CMD_STATUS_REQ:
            (void)BMS_CAN_SendStatus();
            break;

        case BMS_CAN_CMD_DSG_ON:
            BMS_CAN_HandleDsgOn();
            (void)BMS_CAN_SendStatus();
            break;

        case BMS_CAN_CMD_DSG_OFF:
            BMS_CAN_HandleDsgOff();
            (void)BMS_CAN_SendStatus();
            break;

        default:
            break;
    }
}

/*
 * CAN 请求开启 DSG MOS。
 * 这里先做轻量前置检查：初始化完成、Monitor 数据有效、保护层允许放电。
 * 最终是否真的能打开，由 BMS_ControlDsgOn() 再做完整安全检查后决定。
 */
static void BMS_CAN_HandleDsgOn(void)
{
    const BMS_ProtectState_t *prot = BMS_ProtectGetState();
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();

    if (!BMS_InitIsOk())
    {
        return;
    }

    if (prot == NULL || mon == NULL || !mon->data_valid)
    {
        return;
    }

    if (prot->discharge_allowed == 0)
    {
        return;
    }

    (void)BMS_ControlDsgOn();
}

/*
 * CAN 请求关闭 DSG MOS。
 * 关闭动作是安全方向：先清掉放电使能意图，再调用控制层关闭物理 MOS。
 */
static void BMS_CAN_HandleDsgOff(void)
{
    if (!BMS_InitIsOk())
    {
        return;
    }

    BMS_EnergySetDischargeEnable(0);

    (void)BMS_ControlDsgOff();
}

/*
 * 组包并发送 BMS 状态帧。
 * 数据格式：
 * data[0..1] = Pack 电压 /10，低字节在前；
 * data[2..3] = 电流 /10，int16 低字节在前；
 * data[4]    = SOC 百分比；
 * data[5]    = 状态位；
 * data[6]    = 故障位；
 * data[7]    = 第一温度通道，有符号 8 位整数，单位：摄氏度。
 */
static uint8_t BMS_CAN_SendStatus(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    const BMS_ProtectState_t *prot = BMS_ProtectGetState();
    const BMS_EnergyState_t *energy = BMS_EnergyGetState();
    uint8_t data[8] = {0};

    if (mon != NULL && mon->data_valid)
    {
        uint16_t pack_raw = BMS_CAN_U16Scale10(mon->pack_mv);
        int16_t curr_raw = BMS_CAN_I16Scale10(mon->current_ma);

        data[0] = (uint8_t)(pack_raw & 0xFFU);
        data[1] = (uint8_t)((pack_raw >> 8) & 0xFFU);
        data[2] = (uint8_t)(((uint16_t)curr_raw) & 0xFFU);
        data[3] = (uint8_t)((((uint16_t)curr_raw) >> 8) & 0xFFU);
        data[7] = (uint8_t)BMS_CAN_I8Clamp(mon->temp_c[0]);
    }

    data[4] = BMS_CAN_SocPercent();
    data[5] = BMS_CAN_BuildStateFlags(mon, prot, energy);
    data[6] = BMS_CAN_BuildFaultFlags(mon, prot);

    return BMS_CAN_SendFrame(BMS_CAN_STATUS_ID, data);
}

/*
 * CAN 发送公共函数。
 * 发送前检查收发器电源状态，并最多等待 BMS_CAN_TX_TIMEOUT_MS，
 * 避免邮箱长时间满导致 CAN 任务被永久阻塞。
 */
static uint8_t BMS_CAN_SendFrame(uint32_t std_id, const uint8_t data[8])
{
    if (!can_transceiver_power)
    {
        return 0;
    }

    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t mailbox = 0;
    uint32_t start_tick = osKernelGetTickCount();

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
        if ((osKernelGetTickCount() - start_tick) >= BMS_CAN_TX_TIMEOUT_MS)
        {
            return 0;
        }
        osDelay(1);
    }

    return (HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)data, &mailbox) == HAL_OK) ? 1U : 0U;
}

/*
 * 将当前运行状态压缩成 1 字节状态位。
 * 这里同时包含“允许状态”(保护层给出的 CHG/DSG allowed)和“实际状态”
 * (从 SYS_CTRL2 读取的 CHG/DSG MOS 实际开启位)，两者不要混为一谈。
 */
static uint8_t BMS_CAN_BuildStateFlags(const BMS_MonitorData_t *mon,
                                       const BMS_ProtectState_t *prot,
                                       const BMS_EnergyState_t *energy)
{
    uint8_t flags = 0;

    if (mon != NULL && mon->data_valid)
    {
        flags |= BMS_CAN_STATE_MON_VALID;
        if (mon->sleep_request)
        {
            flags |= BMS_CAN_STATE_SLEEP_REQ;
        }
    }

    if (prot != NULL)
    {
        if (prot->charge_allowed)
        {
            flags |= BMS_CAN_STATE_CHG_ALLOWED;
        }
        if (prot->discharge_allowed)
        {
            flags |= BMS_CAN_STATE_DSG_ALLOWED;
        }
    }

    if (BMS_InitIsOk())
    {
        if (BMS_ControlIsChgOn())
        {
            flags |= BMS_CAN_STATE_CHG_ON;
        }
        if (BMS_ControlIsDsgOn())
        {
            flags |= BMS_CAN_STATE_DSG_ON;
        }
    }

    if (energy != NULL)
    {
        if (energy->balance_active)
        {
            flags |= BMS_CAN_STATE_BAL_ACTIVE;
        }
    }

    return flags;
}

/*
 * 将保护故障压缩成 1 字节故障位。
 * Monitor 无效本身也作为故障位上报，因为此时上位机不能信任电压/电流/SOC 数据。
 */
static uint8_t BMS_CAN_BuildFaultFlags(const BMS_MonitorData_t *mon,
                                       const BMS_ProtectState_t *prot)
{
    uint8_t flags = 0;

    if (prot != NULL)
    {
        if (prot->ov_active)
        {
            flags |= BMS_CAN_FAULT_OV;
        }
        if (prot->uv_active)
        {
            flags |= BMS_CAN_FAULT_UV;
        }
        if (prot->ocd_active)
        {
            flags |= BMS_CAN_FAULT_OCD;
        }
        if (prot->scd_active)
        {
            flags |= BMS_CAN_FAULT_SCD;
        }
        if (prot->any_active)
        {
            flags |= BMS_CAN_FAULT_ANY;
        }
    }

    if (mon == NULL || !mon->data_valid)
    {
        flags |= BMS_CAN_FAULT_MON_INVALID;
    }

    return flags;
}

/* 将无符号物理量缩放为 CAN 上报单位：原始值 / 10。 */
static uint16_t BMS_CAN_U16Scale10(uint16_t value)
{
    return (uint16_t)(value / 10U);
}

/* 将有符号物理量缩放为 CAN 上报单位，并限制到 int16 可表达范围。 */
static int16_t BMS_CAN_I16Scale10(int32_t value)
{
    int32_t scaled = value / 10;

    if (scaled > 32767)
    {
        return 32767;
    }
    if (scaled < -32768)
    {
        return -32768;
    }

    return (int16_t)scaled;
}

static int8_t BMS_CAN_I8Clamp(int16_t value)
{
    /* 将温度限制到 CAN 单字节有符号整数的可表达范围。 */
    if (value > 127)
    {
        return 127;
    }
    if (value < -128)
    {
        return -128;
    }

    return (int8_t)value;
}

/* 将内部千分比 SOC 转换为 0~100 的百分比整数。 */
static uint8_t BMS_CAN_SocPercent(void)
{
    uint16_t soc_permille = BMS_AnalysisGetSocPermille();
    uint16_t soc_percent = (uint16_t)((soc_permille + 5U) / 10U);

    return (soc_percent > 100U) ? 100U : (uint8_t)soc_percent;
}

/*
 * CAN 收发器电源控制。
 * enable=1 时重新初始化并启动 CAN；enable=0 时停止 CAN 外设并拉高 PB10 关闭收发器电源。
 */
BMS_CAN_PowerResult_t BMS_CAN_SetPowerState(uint8_t enable)
{
    if (enable)
    {
        if (can_transceiver_power && hcan.State == HAL_CAN_STATE_LISTENING)
        {
            return BMS_CAN_POWER_ALREADY_ON;
        }

        can_transceiver_power = 1;
        /* Re-initialize transceiver and restart CAN */
        if (BMS_CAN_RuntimeInit())
        {
            return BMS_CAN_POWER_STARTED;
        }

        return BMS_CAN_POWER_START_FAILED;
    }

    if (!can_transceiver_power && hcan.State != HAL_CAN_STATE_LISTENING)
    {
        return BMS_CAN_POWER_ALREADY_OFF;
    }

    can_transceiver_power = 0;
    /* Stop CAN peripheral */
    HAL_CAN_Stop(&hcan);
    /* Disable CAN Transceiver Power via PB10 (Active Low, write HIGH to disable) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    return BMS_CAN_POWER_STOPPED;
}

uint8_t BMS_CAN_GetPowerState(void)
{
    return can_transceiver_power;
}
