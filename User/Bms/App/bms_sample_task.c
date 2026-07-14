#include "bms_sample_task.h"
#include "main.h"
#include "bms_monitor.h"
#include "bms_protect.h"
#include "bms_energy.h"
#include "bms_analysis.h"
#include "bms_init.h"
#include "cmsis_os2.h"
#include <stdio.h>

#define SAMPLE_PERIOD_MS  250U

#ifndef BMS_SAMPLE_INITIAL_PRINT_ENABLE
#define BMS_SAMPLE_INITIAL_PRINT_ENABLE  1
#endif

static volatile uint32_t sample_progress;

/* 根据 SOC 千分比更新低电平点亮的四级 LED 电量指示。 */
static void BMS_SampleTaskUpdateLeds(uint16_t soc_permille)
{
    if (soc_permille == 0U)
    {
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin,
                          GPIO_PIN_SET);
    }
    else if (soc_permille <= 250U)
    {
        HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin, GPIO_PIN_SET);
    }
    else if (soc_permille <= 500U)
    {
        HAL_GPIO_WritePin(GPIOB, LED3_Pin | LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);
    }
    else if (soc_permille <= 750U)
    {
        HAL_GPIO_WritePin(GPIOB, LED2_Pin | LED3_Pin | LED4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin,
                          GPIO_PIN_RESET);
    }
}

/* 返回采样任务完成安全周期的累计次数，供看门狗监督。 */
uint32_t BMS_SampleTaskGetProgress(void)
{
    return sample_progress;
}

/* 周期执行采样、保护、SOC 分析和能量控制。 */
void BMS_SampleTask(void *argument)
{
    uint8_t analysis_init_done = 0U;
#if BMS_SAMPLE_INITIAL_PRINT_ENABLE
    uint8_t initial_sample_print_done = 0U;
#endif

    (void)argument;
    sample_progress = 0U;

    if (BMS_InitWaitDone() != 0)
    {
        for (;;)
        {
            osDelay(SAMPLE_PERIOD_MS);
        }
    }

    for (;;)
    {
        const BMS_ProtectState_t *protect = BMS_ProtectGetState();

        if ((protect != NULL) && protect->shutdown_active)
        {
            BMS_SampleTaskUpdateLeds(0U);
            sample_progress++;
            osDelay(SAMPLE_PERIOD_MS);
            continue;
        }

        uint8_t monitor_ok = (BMS_MonitorUpdate() == 0) && BMS_MonitorIsValid();

        BMS_ProtectUpdateFromMonitor();
        protect = BMS_ProtectGetState();

        if (monitor_ok)
        {
            if (!analysis_init_done)
            {
                BMS_AnalysisCapAndSocInit();
                analysis_init_done = 1U;
            }

            BMS_AnalysisEasy();
            BMS_AnalysisCalCap();
            BMS_AnalysisSocCheck();
            if ((protect != NULL) && protect->output_safe_confirmed)
            {
                BMS_EnergyUpdate();
            }
            BMS_SampleTaskUpdateLeds(BMS_AnalysisGetSocPermille());

#if BMS_SAMPLE_INITIAL_PRINT_ENABLE
            if (!initial_sample_print_done)
            {
                const BMS_MonitorData_t *mon = BMS_MonitorGetData();

                initial_sample_print_done = 1U;
                printf("[BQ SAMPLE] Cells: %u %u %u %u %u mV\r\n",
                       (unsigned int)mon->cell_mv[0], (unsigned int)mon->cell_mv[1],
                       (unsigned int)mon->cell_mv[2], (unsigned int)mon->cell_mv[3],
                       (unsigned int)mon->cell_mv[4]);
                printf("[BQ SAMPLE] Pack=%u mV Current=%ld mA Temp=%d C\r\n",
                       (unsigned int)mon->pack_mv, (long)mon->current_ma,
                       (int)mon->temp_c[0]);
            }
#endif
            if ((protect != NULL) && protect->output_safe_confirmed)
            {
                sample_progress++;
            }
        }
        else
        {
            BMS_SampleTaskUpdateLeds(0U);
            if ((protect != NULL) && protect->fail_safe_active &&
                protect->safe_off_confirmed && protect->output_safe_confirmed)
            {
                sample_progress++;
            }
        }

        osDelay(SAMPLE_PERIOD_MS);
    }
}
