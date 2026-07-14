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
#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H



#include "bms_hal_config.h"




// 最多支持多少节电芯
// BQ76920:3~5
// BQ76930:6~10
// BQ76940:9~15
#define BMS_CELL_MAX	5


// 最多支持几路温
// BQ76920:1
// BQ76930:2
// BQ76940:3
#define BMS_TEMP_MAX	1


// 温度测量范围,具体值需要根据热敏电阻和BQ芯片测量范围决定
#define BMS_TEMP_MEASURE_MAX	125
#define	BMS_TEMP_MEASURE_MIN	-55

// 当测量出来的温度值上面这个范围时,用这个无效值来代替
#define BMS_TEMP_INVALID_VALUE	255

// 默认电池额定容量Ah)
// 这个值没有实际用容量测仪校准是卖家口头说
#define BMS_BATTERY_CAPACITY	2.2

// 在待机模式下静止多少时间进入睡眠模式(Min),睡眠低功耗处理暂未考虑
#define BMS_ENTRY_SLEEP_TIME	60



/***************************** 电池保护相关参数 ***********************************/
// 三元锂电Ternary lithium battery)默认参数
#define TLB_OV_PROTECT			4.20	// 单体过压保护电压
#define TLB_OV_RELIEVE			4.18	// 单体过压恢复电压
#define TLB_UV_PROTECT			3.10	// 单体欠压保护电压
#define TLB_UV_RELIEVE			3.15	// 单体欠压恢复电压
#define TLB_SHUTDOWN_VOLTAGE	3.08	// 自动关机电压
#define TLB_BALANCE_VOLTAGE		3.30	// 均衡起始电压



// 磷酸铁锂电池(lithium iron phosphate battery)默认参数
#define LIPB_OV_PROTECT			3.60	// 单体过压保护电压
#define LIPB_OV_RELIEVE			3.55	// 单体过压恢复电压
#define LIPB_UV_PROTECT			2.60	// 单体欠压保护电压
#define LIPB_UV_RELIEVE			2.65	// 单体欠压恢复电压
#define LIPB_SHUTDOWN_VOLTAGE	2.50 	// 自动关机电压
#define LIPB_BALANCE_VOLTAGE	3.00	// 均衡起始电压


// 钛酸锂电Lithium titanate battery)默认参数
#define LTB_OV_PROTECT			2.70	// 单体过压保护电压
#define LTB_OV_RELIEVE			2.65	// 单体过压恢复电压
#define LTB_UV_PROTECT			1.80	// 单体欠压保护电压
#define LTB_UV_RELIEVE			1.85	// 单体欠压恢复电压
#define LTB_SHUTDOWN_VOLTAGE	1.70	// 自动关机电压
#define LTB_BALANCE_VOLTAGE		2.30	// 均衡起始电压



// 初始默认
#define INIT_OV_PROTECT			TLB_OV_PROTECT			// 单体过压保护电压(V)(注意BQ769X0 OV范围.15~4.70V)
#define INIT_OV_RELIEVE			TLB_OV_RELIEVE			// 单体过压恢复电压(V)
#define INIT_UV_PROTECT			TLB_UV_PROTECT			// 单体欠压保护电压(V)(注意BQ769X0 UV范围.58~3.10V)
#define INIT_UV_RELIEVE			TLB_UV_RELIEVE			// 单体欠压恢复电压(V)

/*
 * BQ769X0 hardware UV is an independent backstop.  Keep it below the
 * 3.10 V software threshold and still validate it against each device's
 * measured ADC gain/offset during initialization.
 */
#define INIT_HW_UV_PROTECT       2.80

#define INIT_SHUTDOWN_VOLTAGE	TLB_SHUTDOWN_VOLTAGE	// 自动关机电压(V),暂未使用,预留
#define INIT_BALANCE_VOLTAGE	TLB_BALANCE_VOLTAGE		// 均衡起始电压(V)

#define INIT_BALANCE_CURRENT_MAX	0.6		// 最大均衡电流(A)，预留

// 放电过流保护阈mA),用于软件层保护判
// 2200mA 对应 2.2A, INIT_OCD_MAX 一
#define INIT_OCD_PROTECT_CURRENT	2200

// 放电短路保护阈mA),用于软件层保护判
// 8800mA 对应 8.8A,由BQ硬件SCD阈4mV/分流电阻5mΩ计算得出
#define INIT_SCD_PROTECT_CURRENT	8800

#define INIT_OV_DELAY		BMS_OV_DELAY_2s		// 充电过压保护延时时间	OV:Over	Voltage
#define INIT_UV_DELAY 		BMS_UV_DELAY_4s		// 放电欠压保护延时时间 UV:Under Voltage

#define INIT_OCD_DELAY		BMS_OCD_DELAY_320ms // 放电过流延时时间(S) OCD:Over Current Discharge
#define INIT_SCD_DELAY		BMS_SCD_DELAY_100us	// 放电短路延时时间(us) SCD:Short Circuit Discharge

/* 软件充电过流保护：电流单位为 mA，时间单位为 ms。 */
#define INIT_OCC_PROTECT_MA          2200
#define INIT_OCC_RELEASE_MA          2000
#define INIT_OCC_DELAY_MS            1000U
#define INIT_OCC_RECOVERY_MS        60000U

/* 三元锂温度保护默认值，量产前应按实际电芯规格复核。 */
#define INIT_OTC_PROTECT_C             45
#define INIT_OTC_RELEASE_C             40
#define INIT_OTD_PROTECT_C             60
#define INIT_OTD_RELEASE_C             55
#define INIT_LTC_PROTECT_C              0
#define INIT_LTC_RELEASE_C              5
#define INIT_LTD_PROTECT_C            (-20)
#define INIT_LTD_RELEASE_C            (-15)
#define INIT_TEMP_DELAY_MS           2000U
#define INIT_TEMP_RECOVERY_MS        5000U

/* 运行安全与恢复参数。 */
#define BMS_MONITOR_FAIL_SAFE_COUNT     3U
#define BMS_MONITOR_RECOVERY_COUNT      3U
#define BMS_SAFE_OFF_RETRY_COUNT        3U
#define BMS_ALERT_POLL_MS             500U
#define BMS_INIT_RETRY_COUNT            3U
#define BMS_INIT_RETRY_DELAY_MS       500U
#define BMS_WATCHDOG_BOOT_TIMEOUT_MS 25000U
#define BMS_WATCHDOG_RUNTIME_TIMEOUT_MS 6000U
#define BMS_WATCHDOG_SUPERVISOR_MS    1000U









#define SOC_STOP_CHG_VALUE		1		// 停止充电SOC
#define SOC_START_CHG_VALUE		0.90	// 启动充电SOC
#define SOC_STOP_DSG_VALUE		0		// 停止放电SOC
#define SOC_START_DSG_VALUE		0.10	// 启动放电SOC

#define BALANCE_DIFFE_VOLTAGE	0.05	// 均衡差异电压(V)
#define BALANCE_CYCLE_TIME		30		// 均衡周期时间(s)
#define BALANCE_VOLT_RISE_DELAY 5000	// 均衡电压回升延时(ms)
/*************************************************************************************/


#endif /* BMS_CONFIG_H */


