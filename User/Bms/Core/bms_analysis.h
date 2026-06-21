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
#ifndef BMS_ANALYSIS_H
#define BMS_ANALYSIS_H

#include "main.h"
#include "bms_monitor.h"

/* Analysis 业务层电参分析和容量统计状态数据结*/
typedef struct
{
    uint8_t initialized;
    uint8_t data_valid;

    uint16_t soc_permille;
    uint16_t last_ocv_soc_permille;  /* 上一OCV 查表电量 (千分*/
    uint32_t nominal_cap_mah;
    uint32_t full_cap_mah;
    uint32_t remain_cap_mah;
    uint16_t cap_ratio_permille;

    uint16_t max_cell_mv;
    uint16_t min_cell_mv;
    uint16_t delta_cell_mv;
    uint16_t avg_cell_mv;
    uint8_t max_cell_index;
    uint8_t min_cell_index;

    int32_t current_ma;
    uint16_t pack_mv;
    int32_t power_mw;
    int16_t pack_temp_c;

    uint32_t last_update_tick;
    uint32_t last_soc_tick;
    uint32_t ocv_correct_count;
    uint32_t coulomb_update_count;

    int32_t coulomb_mams_accumulator; /* 高精度安时积分毫安毫秒累加器(mA*ms) */
} BMS_AnalysisState_t;

/* ==========================================================================
 * BMS Analysis 业务层公开 API 接口
 * ========================================================================== */

/* 初始化容量与 SOC */
void BMS_AnalysisCapAndSocInit(void);

/* 电芯基本参数提取与实时功率快照更*/
void BMS_AnalysisEasy(void);

/* 根据温度修正实际满充容量并对剩余容量进行限幅保护 */
void BMS_AnalysisCalCap(void);

/* 执行高精度安时积分、STANDBY下OCV校正以及SOC计算与同*/
void BMS_AnalysisSocCheck(void);

/* 获取 Analysis 模块的状态结构体指针 */
const BMS_AnalysisState_t *BMS_AnalysisGetState(void);

/* 获取当前估算出的 SOC 千分(0~1000) */
uint16_t BMS_AnalysisGetSocPermille(void);

/* 获取当前可用剩余容量 (mAh) */
uint32_t BMS_AnalysisGetRemainCapMah(void);

/* 获取当前经温度修正的实际满充容量 (mAh) */
uint32_t BMS_AnalysisGetFullCapMah(void);

#endif /* BMS_ANALYSIS_H */
