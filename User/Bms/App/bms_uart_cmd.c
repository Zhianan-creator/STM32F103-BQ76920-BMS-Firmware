#include "bms_uart_cmd.h"
#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_control.h"
#include "bms_analysis.h"
#include "bms_energy.h"
#include "bms_can_task.h"
#include "bms_config.h"
#include "bms_init.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "drv_softi2c_bq769x0.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#if (BMS_UART_CMD_ENABLE != 0)

#define RX_BUFFER_SIZE 64

/* Redefine BQ_SYS_STAT_FAULT_MASK for command console safety checks */
#define BQ_SYS_STAT_FAULT_MASK \
	(SYS_STAT_OCD_BIT | SYS_STAT_SCD_BIT | SYS_STAT_OV_BIT | \
	 SYS_STAT_UV_BIT | SYS_STAT_OVRD_BIT | SYS_STAT_DEVICE_BIT)

static QueueHandle_t uart_rx_queue = NULL;
static uint8_t rx_char = 0;
static volatile uint32_t uart_last_activity_tick = 0;

static void BMS_UartCmd_MarkActive(void)
{
    uart_last_activity_tick = osKernelGetTickCount();
}

/* CMSIS-RTOS2 任务属性定义 */
const osThreadAttr_t uartCmdTask_attributes = {
  .name = "uartCmdTask",
  .stack_size = 512 * 4, /* 512 栈 = 2048 字节，提供充裕的工作栈 */
  .priority = (osPriority_t) osPriorityNormal,
};

/* 初始化命令控制台硬件配置 */
void BMS_UartCmd_Init(void)
{
    /* 1. 创建 FreeRTOS 队列，用于接收 ISR 传入的字符 */
    uart_rx_queue = xQueueCreate(128, sizeof(uint8_t));
    if (uart_rx_queue == NULL)
    {
        printf("[CMD] ERROR: Create queue failed\r\n");
        return;
    }
    
    /* 2. 配置 USART1 中断优先级（设置为 6，低于 FreeRTOS 最大系统调用限制 5） */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    /* 3. 首次使能单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);
    BMS_UartCmd_MarkActive();
}

/* HAL 库串口接收中断回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        
        if (uart_rx_queue != NULL)
        {
            /* 发送字符到队列中，如果任务被唤醒，设置 xHigherPriorityTaskWoken */
            xQueueSendFromISR(uart_rx_queue, &rx_char, &xHigherPriorityTaskWoken);
        }
        
        /* 重新使能单字节中断接收 */
        HAL_UART_Receive_IT(&huart1, &rx_char, 1);
        
        /* 强制进行上下文切换以确保高实时性 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* 串口命令交互任务主循环 */
void BMS_UartCmd_Task(void *argument)
{
    (void)argument;
    char rx_line[RX_BUFFER_SIZE];
    uint8_t rx_index = 0;
    uint8_t c = 0;
    uint8_t last_char = 0;
    
    /* 初始化硬件与队列，并开启中断 */
    BMS_UartCmd_Init();
    
    for (;;)
    {
        /* 阻塞式读取队列，永不超时 */
        if (xQueueReceive(uart_rx_queue, &c, portMAX_DELAY) == pdTRUE)
        {
            BMS_UartCmd_MarkActive();

            /* 1. 过滤双重换行（如 \r\n 或 \n\r），避免多次触发空回车提示符 */
            if ((last_char == '\r' && c == '\n') || (last_char == '\n' && c == '\r'))
            {
                last_char = c;
                continue;
            }
            last_char = c;
            
            /* 2. 遇到换行或回车则尝试处理整条命令 */
            if (c == '\r' || c == '\n')
            {
                if (rx_index > 0)
                {
                    rx_line[rx_index] = '\0';
                    printf("\r\n");
                    BMS_UartCmd_ProcessLine(rx_line);
                    rx_index = 0;
                }
                else
                {
                    /* 仅仅是敲回车，重新打印提示符 */
                    printf("\r\n> ");
                }
            }
            /* 3. 处理退格键 (Backspace ASCII 8 / 127) */
            else if (c == 8 || c == 127)
            {
                if (rx_index > 0)
                {
                    rx_index--;
                    /* 在终端上回显擦除退格对应的字符 */
                    printf("\b \b");
                }
            }
            /* 4. 处理普通 ASCII 可打印字符 */
            else if (c >= 32 && c <= 126)
            {
                if (rx_index < (RX_BUFFER_SIZE - 1))
                {
                    rx_line[rx_index++] = c;
                    /* 在终端回显字符，向用户提供极佳的交互视觉确认 */
                    putchar(c);
                }
                else
                {
                    /* 溢出保护：丢弃当前命令，重置接收器并重新打印提示符 */
                    printf("\r\n[CMD] ERROR: Command too long, discarded (max %d bytes)\r\n", RX_BUFFER_SIZE - 1);
                    printf("> ");
                    rx_index = 0;
                }
            }
        }
    }
}

/* ==========================================================================
 * 命令分发及执行细节函数
 * ========================================================================== */

static void Cmd_Help(void)
{
    printf("[CMD] help      : Show all supported commands\r\n");
    printf("[CMD] status    : Display system runtime parameters and statuses\r\n");
    printf("[CMD] protect   : Display detailed software protection state\r\n");
    printf("[CMD] sample    : Trigger a fresh sample update and print results\r\n");
    printf("[CMD] fault     : Read and print BQ769X0 hardware faults in SYS_STAT\r\n");
    printf("[CMD] chg on    : Turn on Charge MOS gate (gated by safety rules)\r\n");
    printf("[CMD] chg off   : Turn off Charge MOS gate (always allowed)\r\n");
    printf("[CMD] dsg on    : Turn on Discharge MOS gate (gated by safety rules)\r\n");
    printf("[CMD] dsg off   : Turn off Discharge MOS gate (always allowed)\r\n");
    printf("[CMD] bal on    : Enable automatic passive balancing\r\n");
    printf("[CMD] bal off   : Disable automatic passive balancing\r\n");
    printf("[CMD] can on    : Turn ON CAN transceiver power and report CAN start result\r\n");
    printf("[CMD] can off   : Turn OFF CAN transceiver power and report stop result\r\n");
    printf("[CMD] config    : Display current hardware & software settings\r\n");
    printf("[CMD] set <param> <val> : Set protection param (uvp/ovp/uvdelay/ovdelay/ocddelay/scddelay)\r\n");
    printf("[CMD] ship      : Force BQ769X0 into SHIP mode (shutdown)\r\n");
}

static void Cmd_Config(void)
{
    printf("[CONFIG] Current hardware & software settings:\r\n");
    printf("[CONFIG] OVP: HW=%dmV, SW=%dmV, HW_Delay=%ds, SW_Delay=%dms\r\n",
           (int)BQ769X0_OVPThresholdGet(),
           (int)BMS_ProtectGetOVMv(),
           (int)(1 << BQ769X0_OVDelayGet()), // 00=1s, 01=2s, 10=4s, 11=8s -> 1<<val
           (int)BMS_ProtectGetOVDelayMs());
           
    int uv_delays[] = {1, 4, 8, 16};
    printf("[CONFIG] UVP: HW=%dmV, SW=%dmV, HW_Delay=%ds, SW_Delay=%dms\r\n",
           (int)BQ769X0_UVPThresholdGet(),
           (int)BMS_ProtectGetUVMv(),
           (int)uv_delays[BQ769X0_UVDelayGet()], // 00=1s, 01=4s, 10=8s, 11=16s
           (int)BMS_ProtectGetUVDelayMs());
           
    int ocd_delays[] = {10, 20, 40, 80, 160, 320, 640, 1280};
    printf("[CONFIG] OCD: HW_Delay=%dms, SW_Delay=%dms, SW_Threshold=%dmA\r\n",
           (int)ocd_delays[BQ769X0_OCDDelayGet()],
           (int)BMS_ProtectGetOCDDelayMs(),
           (int)BMS_ProtectGetOCDMa());
           
    int scd_delays[] = {50, 100, 200, 400};
    printf("[CONFIG] SCD: HW_Delay=%dus, SW_Delay=%dms, SW_Threshold=%dmA\r\n",
           (int)scd_delays[BQ769X0_SCDDelayGet()],
           (int)BMS_ProtectGetSCDDelayMs(),
           (int)BMS_ProtectGetSCDMa());
}

typedef struct
{
    int hw_delay_value;
    uint8_t hw_index;
    int soft_delay_ms;
} CmdDelayOption_t;

#define CMD_ARRAY_LEN(a)  ((int)(sizeof(a) / sizeof((a)[0])))

static int Cmd_FindDelayOption(const CmdDelayOption_t *options,
                               int option_count,
                               int value,
                               const CmdDelayOption_t **matched)
{
    for (int i = 0; i < option_count; i++)
    {
        if (value == options[i].hw_delay_value)
        {
            *matched = &options[i];
            return 1;
        }
    }

    if (value >= 0 && value < option_count)
    {
        *matched = &options[value];
        return 1;
    }

    return 0;
}

static void Cmd_SetParam(const char *param, int value)
{
    if (strcmp(param, "uvp") == 0)
    {
        if (value < 1000 || value > 4000)
        {
            printf("[CMD] ERROR: UVP threshold must be between 1000mV and 4000mV\r\n");
            return;
        }
        BQ769X0_UVPThresholdSet((uint16_t)value);
        BMS_ProtectSetUVMv(value);
        printf("[CMD] UVP threshold set to %dmV (HW & SW synchronized)\r\n", value);
    }
    else if (strcmp(param, "ovp") == 0)
    {
        if (value < 2000 || value > 5000)
        {
            printf("[CMD] ERROR: OVP threshold must be between 2000mV and 5000mV\r\n");
            return;
        }
        BQ769X0_OVPThresholdSet((uint16_t)value);
        BMS_ProtectSetOVMv(value);
        printf("[CMD] OVP threshold set to %dmV (HW & SW synchronized)\r\n", value);
    }
    else if (strcmp(param, "uvdelay") == 0)
    {
        static const CmdDelayOption_t uv_delay_options[] = {
            {1,  BQ_UV_DELAY_1s,  1000},
            {4,  BQ_UV_DELAY_4s,  4000},
            {8,  BQ_UV_DELAY_8s,  8000},
            {16, BQ_UV_DELAY_16s, 16000},
        };
        const CmdDelayOption_t *option = NULL;

        if (!Cmd_FindDelayOption(uv_delay_options, CMD_ARRAY_LEN(uv_delay_options), value, &option))
        {
            printf("[CMD] ERROR: Invalid UV delay. Supported: 1, 4, 8, 16 (s) or index 0-3\r\n");
            return;
        }

        BQ769X0_UVDelaySet((BQ769X0_UVDelayTypedef)option->hw_index);
        BMS_ProtectSetUVDelayMs(option->soft_delay_ms);
        printf("[CMD] UV delay set (HW=%ds, SW=%dms)\r\n",
               option->soft_delay_ms / 1000, option->soft_delay_ms);
    }
    else if (strcmp(param, "ovdelay") == 0)
    {
        static const CmdDelayOption_t ov_delay_options[] = {
            {1, BQ_OV_DELAY_1s, 1000},
            {2, BQ_OV_DELAY_2s, 2000},
            {4, BQ_OV_DELAY_4s, 4000},
            {8, BQ_OV_DELAY_8s, 8000},
        };
        const CmdDelayOption_t *option = NULL;

        if (!Cmd_FindDelayOption(ov_delay_options, CMD_ARRAY_LEN(ov_delay_options), value, &option))
        {
            printf("[CMD] ERROR: Invalid OV delay. Supported: 1, 2, 4, 8 (s) or index 0-3\r\n");
            return;
        }

        BQ769X0_OVDelaySet((BQ769X0_OVDelayTypedef)option->hw_index);
        BMS_ProtectSetOVDelayMs(option->soft_delay_ms);
        printf("[CMD] OV delay set (HW=%ds, SW=%dms)\r\n",
               option->soft_delay_ms / 1000, option->soft_delay_ms);
    }
    else if (strcmp(param, "ocddelay") == 0)
    {
        static const CmdDelayOption_t ocd_delay_options[] = {
            {10,   0, 10},
            {20,   1, 20},
            {40,   2, 40},
            {80,   3, 80},
            {160,  4, 160},
            {320,  5, 320},
            {640,  6, 640},
            {1280, 7, 1280},
        };
        const CmdDelayOption_t *option = NULL;

        if (!Cmd_FindDelayOption(ocd_delay_options, CMD_ARRAY_LEN(ocd_delay_options), value, &option))
        {
            printf("[CMD] ERROR: Invalid OCD delay. Supported: 10, 20, 40, 80, 160, 320, 640, 1280 (ms) or index 0-7\r\n");
            return;
        }

        BQ769X0_OCDDelaySet((BQ769X0_OCDDelayTypedef)option->hw_index);
        BMS_ProtectSetOCDDelayMs(option->soft_delay_ms);
        printf("[CMD] OCD delay set (HW=%dms, SW=%dms)\r\n",
               option->hw_delay_value, option->soft_delay_ms);
    }
    else if (strcmp(param, "scddelay") == 0)
    {
        static const CmdDelayOption_t scd_delay_options[] = {
            {50,  0, 1},
            {100, 1, 1},
            {200, 2, 1},
            {400, 3, 1},
        };
        const CmdDelayOption_t *option = NULL;

        if (!Cmd_FindDelayOption(scd_delay_options, CMD_ARRAY_LEN(scd_delay_options), value, &option))
        {
            printf("[CMD] ERROR: Invalid SCD delay. Supported: 50, 100, 200, 400 (us) or index 0-3\r\n");
            return;
        }

        BQ769X0_SCDDelaySet((BQ769X0_SCDDelayTypedef)option->hw_index);
        BMS_ProtectSetSCDDelayMs(option->soft_delay_ms);
        printf("[CMD] SCD delay set (HW=%dus, SW=%dms)\r\n",
               option->hw_delay_value, option->soft_delay_ms);
    }
    else
    {
        printf("[CMD] ERROR: Unknown parameter: '%s'\r\n", param);
        printf("[CMD] Supported params: uvp, ovp, uvdelay, ovdelay, ocddelay, scddelay\r\n");
    }
}

static void Cmd_Protect(void)
{
    const BMS_ProtectState_t *prot = BMS_ProtectGetState();
    printf("[PROTECT] OV=%d UV=%d OCD=%d SCD=%d ANY=%d\r\n",
           (int)prot->ov_active, (int)prot->uv_active,
           (int)prot->ocd_active, (int)prot->scd_active, (int)prot->any_active);
    printf("[PROTECT] Pending: OV=%d UV=%d OCD=%d SCD=%d\r\n",
           (int)prot->ov_pending, (int)prot->uv_pending,
           (int)prot->ocd_pending, (int)prot->scd_pending);
    printf("[PROTECT] OCC=%d OTC=%d OTD=%d LTC=%d LTD=%d FAILSAFE=%d SAFE=%d\r\n",
           (int)prot->occ_active, (int)prot->otc_active,
           (int)prot->otd_active, (int)prot->ltc_active,
           (int)prot->ltd_active, (int)prot->fail_safe_active,
           (int)prot->safe_off_confirmed);
    printf("[PROTECT] Allow: CHG=%d DSG=%d\r\n",
           (int)prot->charge_allowed, (int)prot->discharge_allowed);
    printf("[PROTECT] LastFault: %s Cell=%d Value=%ld\r\n",
           BMS_ProtectFaultToString(prot->last_fault),
           (int)prot->last_fault_cell,
           (long)prot->last_fault_value);
}

static void Cmd_Status(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    if (mon == NULL || !mon->data_valid)
    {
        printf("[CMD][STATUS] monitor data invalid\r\n");
        return;
    }

    const BMS_AnalysisState_t *astate = BMS_AnalysisGetState();

    printf("[STATUS] Pack=%dmV Current=%ldmA Temp1=%dC Power=%ldmW\r\n",
           (int)mon->pack_mv, (long)mon->current_ma, (int)mon->temp_c[0], (long)astate->power_mw);
    printf("[STATUS] Cell1~5: %d, %d, %d, %d, %d mV\r\n",
           (int)mon->cell_mv[0], (int)mon->cell_mv[1], (int)mon->cell_mv[2],
           (int)mon->cell_mv[3], (int)mon->cell_mv[4]);

    printf("[STATUS] OCV Correct: OCV_SOC=%d.%d%%, Filtered_SOC=%d.%d%%, RemainCap=%lu mAh\r\n",
           (int)(astate->last_ocv_soc_permille / 10), (int)(astate->last_ocv_soc_permille % 10),
           (int)(astate->soc_permille / 10), (int)(astate->soc_permille % 10),
           (unsigned long)astate->remain_cap_mah);

    /* 系统工作模式及待机时间输出 */
    uint32_t standby_time_ms = 0;
    if (mon->sys_mode == BMS_SYS_MODE_STANDBY && mon->standby_start_tick > 0)
    {
        standby_time_ms = osKernelGetTickCount() - mon->standby_start_tick;
    }
    printf("[STATUS] Mode=%s | StandbyTime=%lums | SleepReq=%d | Load=%s\r\n",
           BMS_MonitorSysModeToString(mon->sys_mode),
           (unsigned long)standby_time_ms, (int)mon->sleep_request, BQ769X0_LoadDetect() ? "YES" : "NO");
}

static void Cmd_Sample(void)
{
    /* 通过 Monitor 业务层刷新一次采样数据 */
    if (BMS_MonitorUpdate() != 0)
    {
        printf("[CMD][SAMPLE] monitor update failed\r\n");
        return;
    }

    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    if (mon == NULL || !mon->data_valid)
    {
        printf("[CMD][SAMPLE] monitor data invalid\r\n");
        return;
    }

    printf("[SAMPLE] Cell1~5: %d, %d, %d, %d, %d mV | Pack: %dmV | Current: %ldmA | Temp: %dC\r\n",
           (int)mon->cell_mv[0], (int)mon->cell_mv[1], (int)mon->cell_mv[2],
           (int)mon->cell_mv[3], (int)mon->cell_mv[4],
           (int)mon->pack_mv, (long)mon->current_ma, (int)mon->temp_c[0]);
}

static void Cmd_Fault(void)
{
    uint8_t sys_stat = 0;
    if (BQ769X0_ReadRegisterByteWithCRC(SYS_STAT, &sys_stat))
    {
        printf("[FAULT] SYS_STAT=0x%02X\r\n", sys_stat);
        
        if (sys_stat & SYS_STAT_OCD_BIT)    printf("[FAULT]   OCD: Over current in discharge\r\n");
        if (sys_stat & SYS_STAT_SCD_BIT)    printf("[FAULT]   SCD: Short circuit in discharge\r\n");
        if (sys_stat & SYS_STAT_OV_BIT)     printf("[FAULT]   OV: Over voltage\r\n");
        if (sys_stat & SYS_STAT_UV_BIT)     printf("[FAULT]   UV: Under voltage\r\n");
        if (sys_stat & SYS_STAT_OVRD_BIT)   printf("[FAULT]   OVRD_ALERT: Alert override active\r\n");
        if (sys_stat & SYS_STAT_DEVICE_BIT) printf("[FAULT]   DEVICE_XREADY: Chip device xready active\r\n");
        if (sys_stat & SYS_STAT_CC_BIT)     printf("[FAULT]   CC_READY: Coulomb counter completed (normal)\r\n");
        
        if ((sys_stat & BQ_SYS_STAT_FAULT_MASK) == 0)
        {
            printf("[FAULT] No active hardware faults.\r\n");
        }
    }
    else
    {
        printf("[FAULT] ERROR: Read SYS_STAT failed\r\n");
    }
}

static void Cmd_ChgOn(void)
{
    BMS_EnergySetChargeEnable(1);
    printf("[CMD][CHG] %s\r\n", BMS_ControlResultToString(BMS_ControlChgOn()));
}

static void Cmd_ChgOff(void)
{
    BMS_EnergySetChargeEnable(0);
    printf("[CMD][CHG] %s\r\n", BMS_ControlResultToString(BMS_ControlChgOff()));
}

static void Cmd_DsgOn(void)
{
    BMS_EnergySetDischargeEnable(1);
    printf("[CMD][DSG] %s\r\n", BMS_ControlResultToString(BMS_ControlDsgOn()));
}

static void Cmd_DsgOff(void)
{
    BMS_EnergySetDischargeEnable(0);
    printf("[CMD][DSG] %s\r\n", BMS_ControlResultToString(BMS_ControlDsgOff()));
}

static void Cmd_BalOn(void)
{
    BMS_EnergySetBalanceEnable(1);
    printf("[CMD][BAL] automatic balance ON\r\n");
}

static void Cmd_BalOff(void)
{
    BMS_EnergySetBalanceEnable(0);
    const BMS_EnergyState_t *energy = BMS_EnergyGetState();

    printf("[CMD][BAL] automatic balance OFF, outputs=%s\r\n",
           BMS_ControlResultToString(energy->last_balance_result));
}

static void Cmd_CanOn(void)
{
    BMS_CAN_PowerResult_t result = BMS_CAN_SetPowerState(1);

    if (result == BMS_CAN_POWER_STARTED)
    {
        printf("[CMD][CAN] CAN transceiver power ON, CAN started.\r\n");
    }
    else if (result == BMS_CAN_POWER_ALREADY_ON)
    {
        printf("[CMD][CAN] CAN already ON.\r\n");
    }
    else
    {
        printf("[CMD][CAN] CAN start failed.\r\n");
    }
}

static void Cmd_CanOff(void)
{
    BMS_CAN_PowerResult_t result = BMS_CAN_SetPowerState(0);

    if (result == BMS_CAN_POWER_STOPPED)
    {
        printf("[CMD][CAN] CAN stopped, transceiver power OFF.\r\n");
    }
    else
    {
        printf("[CMD][CAN] CAN already OFF.\r\n");
    }
}

/* 解析整行命令的核心逻辑 */
void BMS_UartCmd_ProcessLine(const char *line)
{
    BMS_UartCmd_MarkActive();

    if (strcmp(line, "help") == 0)
    {
        Cmd_Help();
    }
    else if (strcmp(line, "status") == 0)
    {
        Cmd_Status();
    }
    else if (strcmp(line, "protect") == 0)
    {
        Cmd_Protect();
    }
    else if (strcmp(line, "sample") == 0)
    {
        Cmd_Sample();
    }
    else if (strcmp(line, "fault") == 0)
    {
        Cmd_Fault();
    }
    else if (strcmp(line, "chg on") == 0 || strcmp(line, "chg") == 0)
    {
        Cmd_ChgOn();
    }
    else if (strcmp(line, "chg off") == 0)
    {
        Cmd_ChgOff();
    }
    else if (strcmp(line, "dsg on") == 0 || strcmp(line, "dsg") == 0)
    {
        Cmd_DsgOn();
    }
    else if (strcmp(line, "dsg off") == 0)
    {
        Cmd_DsgOff();
    }
    else if (strcmp(line, "bal on") == 0)
    {
        Cmd_BalOn();
    }
    else if (strcmp(line, "bal off") == 0)
    {
        Cmd_BalOff();
    }
    else if (strcmp(line, "can on") == 0)
    {
        Cmd_CanOn();
    }
    else if (strcmp(line, "can off") == 0)
    {
        Cmd_CanOff();
    }
    else if (strcmp(line, "config") == 0)
    {
        Cmd_Config();
    }
    else if (strcmp(line, "ship") == 0 || strcmp(line, "shutdown") == 0)
    {
        if (!BMS_InitIsOk())
        {
            printf("[CMD] ERROR: BMS initialization is not complete.\r\n");
            return;
        }
        printf("[CMD] CRITICAL: Entering BQ769X0 SHIP mode (shutdown).\r\n");
        printf("[CMD] TS1 high pulse on PA15 is required to wake up.\r\n");
        osDelay(100);
        if (!BQ769X0_ForceSafeOff(BMS_SAFE_OFF_RETRY_COUNT))
        {
            printf("[CMD] ERROR: safe-off verification failed; shutdown cancelled.\r\n");
        }
        else if (!BQ769X0_EntryShip())
        {
            printf("[CMD] ERROR: SHIP command failed.\r\n");
        }
        else
        {
            BMS_ProtectLatchShutdown();
        }
    }
    else if (strncmp(line, "set ", 4) == 0)
    {
        char param[20];
        int value = 0;
        if (sscanf(line + 4, "%19s %d", param, &value) == 2)
        {
            Cmd_SetParam(param, value);
        }
        else
        {
            printf("[CMD] Usage: set <param> <value>\r\n");
            printf("[CMD] Supported params: uvp, ovp, uvdelay, ovdelay, ocddelay, scddelay\r\n");
        }
    }
    else
    {
        printf("[CMD] Unknown command: '%s'. Type 'help' for support.\r\n", line);
    }
    
    /* 再次输出提示符，等待下一次输入 */
    printf("> ");
}

uint8_t BMS_UartCmdIsActive(void)
{
    uint32_t now = osKernelGetTickCount();

    if (uart_last_activity_tick == 0)
    {
        return 0;
    }

    return ((now - uart_last_activity_tick) < BMS_UART_CMD_ACTIVE_HOLD_MS) ? 1 : 0;
}

#endif /* BMS_UART_CMD_ENABLE */
