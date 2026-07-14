#include "bms_hal_monitor.h"
#include "drv_softi2c_bq769x0.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* 内部静态缓存，保存最近一次完整且有效的采样快照。 */
static BMS_HAL_MonitorData_t latest_monitor_data = {0};

/* 初始化监控适配层的数据缓存。 */
int BMS_HAL_MonitorInit(void)
{
    memset(&latest_monitor_data, 0, sizeof(BMS_HAL_MonitorData_t));
    latest_monitor_data.cell_count = BMS_CELL_MAX;
    latest_monitor_data.temp_count = BMS_TEMP_MAX;
    return 0;
}

/* 原子采集一轮电压、电流和温度，任一底层读取失败则整轮无效。 */
int BMS_HAL_MonitorSample(BMS_HAL_MonitorData_t *out)
{
    BMS_HAL_MonitorData_t next_data = {0};
    bool cell_ok;
    bool pack_ok;
    bool current_ok;
    bool temp_ok;

    BQ769X0_BusLock();
    cell_ok = BQ769X0_UpdateCellVolt();
    pack_ok = BQ769X0_UpadteBatVolt();
    current_ok = BQ769X0_UpdateCurrent();
    temp_ok = BQ769X0_UpdateTsTemp();

    if (cell_ok && pack_ok && current_ok && temp_ok)
    {
        for (uint8_t i = 0; i < BMS_CELL_MAX; i++)
        {
            next_data.cell_mv[i] = BQ769X0_GetCellVoltageMv(i);
        }
        for (uint8_t i = 0; i < BMS_TEMP_MAX; i++)
        {
            next_data.temp_c[i] = BQ769X0_GetTemperatureC(i);
        }
        next_data.pack_mv = BQ769X0_GetPackVoltageMv();
        next_data.current_ma = BQ769X0_GetCurrentMa();
    }
    BQ769X0_BusUnlock();

    if (!(cell_ok && pack_ok && current_ok && temp_ok))
    {
        latest_monitor_data.valid = 0U;
        if (out != NULL)
        {
            memcpy(out, &latest_monitor_data, sizeof(BMS_HAL_MonitorData_t));
        }
        return -1;
    }

    next_data.cell_count = BMS_CELL_MAX;
    next_data.temp_count = BMS_TEMP_MAX;
    next_data.valid = 1U;
    next_data.sample_tick = osKernelGetTickCount();
    latest_monitor_data = next_data;
    if (out != NULL)
    {
        *out = next_data;
    }
    return 0;
}

/* 返回最近一次采样缓存的只读指针。 */
const BMS_HAL_MonitorData_t *BMS_HAL_MonitorGetLatest(void)
{
    return &latest_monitor_data;
}

/* 获取指定电芯的最新电压，单位为毫伏。 */
uint16_t BMS_HAL_MonitorGetCellMv(uint8_t index)
{
    if (!latest_monitor_data.valid || index >= BMS_CELL_MAX)
    {
        return 0;
    }
    return latest_monitor_data.cell_mv[index];
}

/* 获取最新电池包总电压，单位为毫伏。 */
uint16_t BMS_HAL_MonitorGetPackMv(void)
{
    if (!latest_monitor_data.valid)
    {
        return 0;
    }
    return latest_monitor_data.pack_mv;
}

/* 获取最新电池电流，单位为毫安。 */
int32_t BMS_HAL_MonitorGetCurrentMa(void)
{
    if (!latest_monitor_data.valid)
    {
        return 0;
    }
    return latest_monitor_data.current_ma;
}

/* 获取指定温度通道的最新温度，单位为摄氏度。 */
int16_t BMS_HAL_MonitorGetTempC(uint8_t index)
{
    if (!latest_monitor_data.valid || index >= BMS_TEMP_MAX)
    {
        return 0;
    }
    return latest_monitor_data.temp_c[index];
}

/* 查询最近一次采样是否有效。 */
uint8_t BMS_HAL_MonitorIsValid(void)
{
    return latest_monitor_data.valid;
}

/* 获取最近一次有效采样的系统时刻。 */
uint32_t BMS_HAL_MonitorGetSampleTick(void)
{
    return latest_monitor_data.sample_tick;
}
