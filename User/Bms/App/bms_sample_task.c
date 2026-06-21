#include "bms_sample_task.h"
#include "main.h"
#include "bms_monitor.h"
#include "bms_protect.h"
#include "bms_energy.h"
#include "bms_analysis.h"
#include "bms_log.h"
#include "bms_init.h"
#include "drv_softi2c_bq769x0.h"
#include "cmsis_os2.h"
#include <stdio.h>

#define SAMPLE_PERIOD_MS  250  /* BQ769X0 采样周期(ms) */
#ifndef BMS_SAMPLE_INITIAL_PRINT_ENABLE
#define BMS_SAMPLE_INITIAL_PRINT_ENABLE  1
#endif

static void BMS_SampleTask_UpdateLEDs(uint16_t soc_permille)
{
    /*
     * LED 指示灯与 SOC 的对应关系（Active Low，低电平 RESET 为亮，高电平 SET 为灭
     * - LED7 (LED1_Pin = PB6): 0%   < SOC <= 100% * - LED6 (LED2_Pin = PB7): 25%  < SOC <= 100% * - LED5 (LED3_Pin = PB8): 50%  < SOC <= 100% * - LED4 (LED4_Pin = PB9): 75%  < SOC <= 100% *
     * 采用标准的“累加式进度条”显示方式（BMS/充电宝通用规范）* 如果 SOC == 0，则全部熄灭*/
    if (soc_permille == 0)
    {
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin, GPIO_PIN_SET);
    }
    else if (soc_permille <= 250) /* 0% < SOC <= 25% (LED7*/
    {
        HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_SET);
    }
    else if (soc_permille <= 500) /* 25% < SOC <= 50% (LED7, LED6*/
    {
        HAL_GPIO_WritePin(GPIOB, LED3_Pin | LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
    }
    else if (soc_permille <= 750) /* 50% < SOC <= 75% (LED7, LED6, LED5*/
    {
        HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED3_Pin | LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    }
    else /* 75% < SOC <= 100% (LED7, LED6, LED5, LED4全亮) */
    {
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin, GPIO_PIN_RESET);
    }
}

void BMS_SampleTask(void *argument)
{
    int monitor_update_ok;
    uint8_t analysis_init_done = 0;
#if BMS_SAMPLE_INITIAL_PRINT_ENABLE
    uint8_t initial_sample_print_done = 0;
#endif
    uint8_t initial_safe_off_done = 0;

    (void)argument;

    /* 阻塞等待 BQ 初始化完成（BMS_InitRun() 通知 */
    BMS_InitWaitDone();

    /* 初始化系统日志、Monitor 业务层、容量与 SOC 分析、以及保护状态机 */
    BMS_LogInit();
    BMS_MonitorInit();
    BMS_ProtectInit();
    BMS_EnergyInit();

    for (;;)
    {
        /* 周期采样：调用 Monitor 更新（内部触发 HAL 采样 + 校验 + 统计 */
        monitor_update_ok = (BMS_MonitorUpdate() == 0 && BMS_MonitorIsValid());

        /* 运行电参分析、可用容量与 SOC 业务结算 */
        if (!analysis_init_done && monitor_update_ok)
        {
            BMS_AnalysisCapAndSocInit();
            analysis_init_done = 1;
        }

        if (analysis_init_done)
        {
            BMS_AnalysisEasy();
            BMS_AnalysisCalCap();
            BMS_AnalysisSocCheck();

        /* 根据最新 SOC 估算值更新 LED 指示灯状态 */
            BMS_SampleTask_UpdateLEDs(BMS_AnalysisGetSocPermille());
        }
        else
        {
            BMS_SampleTask_UpdateLEDs(0);
        }

        /* 运行软件保护状态机（无条件调用，内部处理无效数据） */
        BMS_ProtectUpdateFromMonitor();

        /* 运行能量控制业务层（根据保护状态执行 MOS 通断控制 */
        BMS_EnergyUpdate();

#if BMS_SAMPLE_INITIAL_PRINT_ENABLE
        if (!initial_sample_print_done && monitor_update_ok)
        {
            const BMS_MonitorData_t *mon = BMS_MonitorGetData();

            initial_sample_print_done = 1;
            printf("[BQ SAMPLE] Initial sample:\r\n");
            printf("  Cells: %dmV, %dmV, %dmV, %dmV, %dmV\r\n",
                   (int)mon->cell_mv[0], (int)mon->cell_mv[1], (int)mon->cell_mv[2],
                   (int)mon->cell_mv[3], (int)mon->cell_mv[4]);
            printf("  Pack: %dmV, Current: %ldmA, Temp1: %dC\r\n",
                   (int)mon->pack_mv, (long)mon->current_ma, (int)mon->temp_c[0]);
            printf("[MON] Cell max: Cell%d %dmV, Cell min: Cell%d %dmV\r\n",
                   mon->max_cell_index, (int)mon->max_cell_mv,
                   mon->min_cell_index, (int)mon->min_cell_mv);
        }
#endif

        if (!initial_safe_off_done)
        {
            initial_safe_off_done = 1;
            BQ769X0_ForceSafeOff();
        }

        /* 固定周期延时 */
        osDelay(SAMPLE_PERIOD_MS);
    }
}
