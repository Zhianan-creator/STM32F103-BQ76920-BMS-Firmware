/*
 * Copyright (C) 2021-2099 PLKJ Development Team
 *
 * SPDX-License-Identifier: CC BY-NC 4.0
 *
 * http://creativecommons.org/licenses/by-nc/4.0/
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "bms_analysis.h"
#include "bms_energy.h"
#include "bms_log.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>

/* OCV-SOC 散点定义 */
typedef struct
{
    uint16_t mv;
    uint16_t soc_permille;
} BMS_OcvSocPoint_t;

/* 三元1 节电芯近似开路电(OCV) 与 SOC (千分映射*/
static const BMS_OcvSocPoint_t ocv_soc_table[] = {
    {3100, 0},
    {3300, 100},
    {3400, 200},
    {3500, 300},
    {3600, 400},
    {3700, 500},
    {3800, 600},
    {3900, 700},
    {4000, 800},
    {4100, 900},
    {4200, 1000}
};

#define OCV_SOC_TABLE_SIZE (sizeof(ocv_soc_table) / sizeof(ocv_soc_table[0]))

/* Analysis 业务状态单*/
static BMS_AnalysisState_t analysis_state = {0};

/* ==========================================================================
 * 内部静态辅助函数声* ========================================================================== */

/* 一、BMS_AnalysisCapAndSocInit() 下调*/
static uint8_t BMS_AnalysisCanInitFromOcv(const BMS_MonitorData_t *mon);
static uint16_t BMS_AnalysisGetInitOcvMv(const BMS_MonitorData_t *mon);
static uint16_t BMS_AnalysisOcvToSocPermille(uint16_t ocv_mv);
static uint32_t BMS_AnalysisGetNominalCapMah(void);
static uint32_t BMS_AnalysisCalcRemainCapFromSoc(uint32_t full_cap_mah, uint16_t soc_permille);

/* 二、BMS_AnalysisEasy() 下调*/
static void BMS_AnalysisCopyMonitorBasic(const BMS_MonitorData_t *mon);
static void BMS_AnalysisCalcCellStat(const BMS_MonitorData_t *mon);
static void BMS_AnalysisCalcPower(const BMS_MonitorData_t *mon);

/* 三、BMS_AnalysisCalCap() 下调*/
static int16_t BMS_AnalysisGetPackTempC(const BMS_MonitorData_t *mon);
static uint16_t BMS_AnalysisTempToCapRatioPermille(int16_t temp_c);
static uint32_t BMS_AnalysisApplyCapRatio(uint32_t nominal_cap_mah, uint16_t ratio_permille);
static void BMS_AnalysisClampRemainCap(void);

/* 四、BMS_AnalysisSocCheck() 下调*/
static void BMS_AnalysisCoulombIntegrate(const BMS_MonitorData_t *mon);
static uint8_t BMS_AnalysisCanDoOcvCorrect(const BMS_MonitorData_t *mon);
static void BMS_AnalysisOcvCorrect(const BMS_MonitorData_t *mon);
static void BMS_AnalysisUpdateSocFromCapacity(void);
static void BMS_AnalysisSyncSocToEnergy(void);

/* ==========================================================================
 * 公开 API 函数实现
 * ========================================================================== */

void BMS_AnalysisCapAndSocInit(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();

    memset(&analysis_state, 0, sizeof(BMS_AnalysisState_t));
    analysis_state.nominal_cap_mah = BMS_AnalysisGetNominalCapMah();

    if (BMS_AnalysisCanInitFromOcv(mon))
    {
        uint16_t ocv_mv = BMS_AnalysisGetInitOcvMv(mon);
        analysis_state.soc_permille = BMS_AnalysisOcvToSocPermille(ocv_mv);
        analysis_state.last_ocv_soc_permille = analysis_state.soc_permille;
        analysis_state.full_cap_mah = analysis_state.nominal_cap_mah;
        analysis_state.remain_cap_mah = BMS_AnalysisCalcRemainCapFromSoc(analysis_state.full_cap_mah, analysis_state.soc_permille);
        analysis_state.data_valid = 1;

        BMS_LOGI("ANALYSIS", "Init from OCV Success: OCV=%dmV, SOC=%d.%d%%, Full=%lu mAh, Remain=%lu mAh",
                 ocv_mv, analysis_state.soc_permille / 10, analysis_state.soc_permille % 10,
                 (unsigned long)analysis_state.full_cap_mah, (unsigned long)analysis_state.remain_cap_mah);
    }
    else
    {
        analysis_state.soc_permille = 500; /* 默认 50% */
        analysis_state.last_ocv_soc_permille = 500;
        analysis_state.full_cap_mah = analysis_state.nominal_cap_mah;
        analysis_state.remain_cap_mah = BMS_AnalysisCalcRemainCapFromSoc(analysis_state.full_cap_mah, analysis_state.soc_permille);
        analysis_state.data_valid = 0;

        BMS_LOGI("ANALYSIS", "Init using Default: SOC=50.0%%, Full=%lu mAh, Remain=%lu mAh (Monitor invalid)",
                 (unsigned long)analysis_state.full_cap_mah, (unsigned long)analysis_state.remain_cap_mah);
    }

    analysis_state.last_update_tick = osKernelGetTickCount();
    analysis_state.last_soc_tick = analysis_state.last_update_tick;
    analysis_state.coulomb_mams_accumulator = 0;
    analysis_state.initialized = 1;

    /* 首次同步 SOC 与 Energy 模块 */
    BMS_AnalysisSyncSocToEnergy();
}

void BMS_AnalysisEasy(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    analysis_state.last_update_tick = osKernelGetTickCount();

    if (mon == NULL || !mon->data_valid)
    {
        analysis_state.data_valid = 0;
        return;
    }

    BMS_AnalysisCopyMonitorBasic(mon);
    BMS_AnalysisCalcCellStat(mon);
    BMS_AnalysisCalcPower(mon);
}

void BMS_AnalysisCalCap(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    int16_t temp_c = BMS_AnalysisGetPackTempC(mon);

    analysis_state.cap_ratio_permille = BMS_AnalysisTempToCapRatioPermille(temp_c);
    analysis_state.full_cap_mah = BMS_AnalysisApplyCapRatio(analysis_state.nominal_cap_mah, analysis_state.cap_ratio_permille);
    BMS_AnalysisClampRemainCap();
}

void BMS_AnalysisSocCheck(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();

    /* 1. 执行高精度安时积*/
    BMS_AnalysisCoulombIntegrate(mon);

    /* 2. 判断并执OCV 校正 */
    if (BMS_AnalysisCanDoOcvCorrect(mon))
    {
        BMS_AnalysisOcvCorrect(mon);
    }
    else
    {
        /* 在 OCV 校正状态下，由最新的容量计算SOC */
        BMS_AnalysisUpdateSocFromCapacity();
    }

    /* 3. 同步最SOC 分数Energy 业务*/
    BMS_AnalysisSyncSocToEnergy();
}

const BMS_AnalysisState_t *BMS_AnalysisGetState(void)
{
    return &analysis_state;
}

uint16_t BMS_AnalysisGetSocPermille(void)
{
    return analysis_state.soc_permille;
}

uint32_t BMS_AnalysisGetRemainCapMah(void)
{
    return analysis_state.remain_cap_mah;
}

uint32_t BMS_AnalysisGetFullCapMah(void)
{
    return analysis_state.full_cap_mah;
}

/* ==========================================================================
 * 内部静态辅助函数实* ========================================================================== */

/* OCV-SOC 线性插值实*/
static uint16_t BMS_AnalysisOcvToSocPermille(uint16_t ocv_mv)
{
    if (ocv_mv <= ocv_soc_table[0].mv)
    {
        return ocv_soc_table[0].soc_permille;
    }
    if (ocv_mv >= ocv_soc_table[OCV_SOC_TABLE_SIZE - 1].mv)
    {
        return ocv_soc_table[OCV_SOC_TABLE_SIZE - 1].soc_permille;
    }

    /* 二分或顺序寻找所属区*/
    for (size_t i = 0; i < OCV_SOC_TABLE_SIZE - 1; ++i)
    {
        if (ocv_mv >= ocv_soc_table[i].mv && ocv_mv <= ocv_soc_table[i + 1].mv)
        {
            uint32_t x0 = ocv_soc_table[i].mv;
            uint32_t x1 = ocv_soc_table[i + 1].mv;
            uint32_t y0 = ocv_soc_table[i].soc_permille;
            uint32_t y1 = ocv_soc_table[i + 1].soc_permille;

            /* y = y0 + (x - x0) * (y1 - y0) / (x1 - x0) */
            uint32_t soc = y0 + (uint32_t)(ocv_mv - x0) * (y1 - y0) / (x1 - x0);
            return (uint16_t)soc;
        }
    }
    return 500;
}

static uint8_t BMS_AnalysisCanInitFromOcv(const BMS_MonitorData_t *mon)
{
    return (mon != NULL && mon->data_valid && mon->voltage_valid) ? 1 : 0;
}

static uint16_t BMS_AnalysisGetInitOcvMv(const BMS_MonitorData_t *mon)
{
    return mon->min_cell_mv;
}

static uint32_t BMS_AnalysisGetNominalCapMah(void)
{
    return (uint32_t)(BMS_BATTERY_CAPACITY * 1000.0f);
}

static uint32_t BMS_AnalysisCalcRemainCapFromSoc(uint32_t full_cap_mah, uint16_t soc_permille)
{
    return (uint32_t)((uint64_t)full_cap_mah * soc_permille / 1000);
}

static void BMS_AnalysisCopyMonitorBasic(const BMS_MonitorData_t *mon)
{
    analysis_state.data_valid = 1;
    analysis_state.current_ma = mon->current_ma;
    analysis_state.pack_mv = mon->pack_mv;
    analysis_state.pack_temp_c = mon->temp_c[0];
}

static void BMS_AnalysisCalcCellStat(const BMS_MonitorData_t *mon)
{
    analysis_state.max_cell_mv = mon->max_cell_mv;
    analysis_state.min_cell_mv = mon->min_cell_mv;
    analysis_state.delta_cell_mv = mon->delta_cell_mv;
    analysis_state.avg_cell_mv = mon->avg_cell_mv;
    analysis_state.max_cell_index = mon->max_cell_index;
    analysis_state.min_cell_index = mon->min_cell_index;
}

static void BMS_AnalysisCalcPower(const BMS_MonitorData_t *mon)
{
    /* 功率(mW) = pack_mv(mV) * current_ma(mA) / 1000 */
    analysis_state.power_mw = (int32_t)((int64_t)mon->pack_mv * mon->current_ma / 1000);
}

static int16_t BMS_AnalysisGetPackTempC(const BMS_MonitorData_t *mon)
{
    return (mon != NULL && mon->data_valid && mon->temp_valid) ? mon->temp_c[0] : 25;
}

static uint16_t BMS_AnalysisTempToCapRatioPermille(int16_t temp_c)
{
    if (temp_c <= -20)
    {
        return 600; /* temp <= -20C: 60% */
    }
    else if (temp_c <= 0)
    {
        return 800; /* -20C < temp <= 0C: 80% */
    }
    else if (temp_c <= 10)
    {
        return 900; /* 0C < temp <= 10C: 90% */
    }
    else if (temp_c <= 45)
    {
        return 1000; /* 10C < temp <= 45C: 100% */
    }
    else if (temp_c <= 60)
    {
        return 950; /* 45C < temp <= 60C: 95% */
    }
    else
    {
        return 900; /* temp > 60C: 90% */
    }
}

static uint32_t BMS_AnalysisApplyCapRatio(uint32_t nominal_cap_mah, uint16_t ratio_permille)
{
    return (uint32_t)((uint64_t)nominal_cap_mah * ratio_permille / 1000);
}

static void BMS_AnalysisClampRemainCap(void)
{
    if (analysis_state.remain_cap_mah > analysis_state.full_cap_mah)
    {
        analysis_state.remain_cap_mah = analysis_state.full_cap_mah;
    }
}

static void BMS_AnalysisCoulombIntegrate(const BMS_MonitorData_t *mon)
{
    uint32_t current_tick = osKernelGetTickCount();
    uint32_t delta_ms = current_tick - analysis_state.last_soc_tick;
    analysis_state.last_soc_tick = current_tick;

    if (mon == NULL || !mon->data_valid || !mon->current_valid)
    {
        return;
    }

    /* 毫安毫秒累加器防止分步整除精度丢(1 mAh = 3,600,000 mA * ms) */
    analysis_state.coulomb_mams_accumulator += mon->current_ma * (int32_t)delta_ms;

    int32_t delta_mah = analysis_state.coulomb_mams_accumulator / 3600000;
    if (delta_mah != 0)
    {
        analysis_state.coulomb_mams_accumulator %= 3600000;

        int32_t new_remain = (int32_t)analysis_state.remain_cap_mah + delta_mah;
        if (new_remain < 0)
        {
            new_remain = 0;
        }
        else if (new_remain > (int32_t)analysis_state.full_cap_mah)
        {
            new_remain = (int32_t)analysis_state.full_cap_mah;
        }

        analysis_state.remain_cap_mah = (uint32_t)new_remain;
        analysis_state.coulomb_update_count++;
    }
}

static uint8_t BMS_AnalysisCanDoOcvCorrect(const BMS_MonitorData_t *mon)
{
    if (mon == NULL || !mon->data_valid || !mon->voltage_valid || !mon->current_valid)
    {
        return 0;
    }

    /* 系统处于 STANDBY 或 SLEEP 且电流绝对值小20mA 时允OCV 估算 */
    uint8_t mode_ok = (mon->sys_mode == BMS_SYS_MODE_STANDBY || mon->sys_mode == BMS_SYS_MODE_SLEEP);
    int32_t abs_curr = mon->current_ma < 0 ? -mon->current_ma : mon->current_ma;
    uint8_t curr_ok = (abs_curr < 20);

    return (mode_ok && curr_ok) ? 1 : 0;
}

static void BMS_AnalysisOcvCorrect(const BMS_MonitorData_t *mon)
{
    uint16_t ocv_mv = mon->min_cell_mv;
    uint16_t ocv_soc = BMS_AnalysisOcvToSocPermille(ocv_mv);

    analysis_state.last_ocv_soc_permille = ocv_soc;

    /* 轻微校正: soc = (soc * 7 + ocv_soc * 3) / 10 */
    analysis_state.soc_permille = (uint16_t)(((uint32_t)analysis_state.soc_permille * 7 + (uint32_t)ocv_soc * 3) / 10);
    analysis_state.remain_cap_mah = BMS_AnalysisCalcRemainCapFromSoc(analysis_state.full_cap_mah, analysis_state.soc_permille);
    analysis_state.ocv_correct_count++;
}

static void BMS_AnalysisUpdateSocFromCapacity(void)
{
    if (analysis_state.full_cap_mah > 0)
    {
        uint32_t soc = (uint32_t)(((uint64_t)analysis_state.remain_cap_mah * 1000) / analysis_state.full_cap_mah);
        if (soc > 1000)
        {
            soc = 1000;
        }
        analysis_state.soc_permille = (uint16_t)soc;
    }
}

static void BMS_AnalysisSyncSocToEnergy(void)
{
    BMS_EnergySetSocPermille(analysis_state.soc_permille);
}
