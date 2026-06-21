#include "bms_hal_monitor.h"
#include "drv_softi2c_bq769x0.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/* 内部全局静态缓存，保存最新采样的电池包物理数*/
static BMS_HAL_MonitorData_t latest_monitor_data = {0};


int BMS_HAL_MonitorInit(void)
{
    /* 1. 清空最新数据缓*/
    memset(&latest_monitor_data, 0, sizeof(BMS_HAL_MonitorData_t));

    /* 2. 写入静态配置数*/
    latest_monitor_data.cell_count = BMS_CELL_MAX;
    latest_monitor_data.temp_count = BMS_TEMP_MAX;
    latest_monitor_data.valid = 0;
    latest_monitor_data.sample_tick = 0;

    return 0;
}

int BMS_HAL_MonitorSample(BMS_HAL_MonitorData_t *out)
{
    /* 1. 触发 BQ 驱动的底层 I2C 周期采样，获取最新原始电参 */
    BQ769X0_UpdateCellVolt();
    BQ769X0_UpadteBatVolt();
    BQ769X0_UpdateCurrent();
    BQ769X0_UpdateTsTemp();

    /* 2. 依次读取单体电芯电压，从浮点 V 转换并填充为 mV */
    for (uint8_t i = 0; i < BMS_CELL_MAX; i++)
    {
        latest_monitor_data.cell_mv[i] = BQ769X0_GetCellVoltageMv(i);
    }

    /* 3. 读取总包电压及电流，进行缩放填充 */
    latest_monitor_data.pack_mv = BQ769X0_GetPackVoltageMv();
    latest_monitor_data.current_ma = BQ769X0_GetCurrentMa();

    /* 4. 读取温度，从浮点 C 转换并填充为 °C 整数 */
    for (uint8_t i = 0; i < BMS_TEMP_MAX; i++)
    {
        latest_monitor_data.temp_c[i] = BQ769X0_GetTemperatureC(i);
    }

    /* 5. 更新数据有效状态及采样 Tick 时间 */
    latest_monitor_data.valid = 1;
    latest_monitor_data.sample_tick = osKernelGetTickCount();

    /* 6. 若外部传入出参缓冲区，则深拷贝一份到外部 */
    if (out != NULL)
    {
        memcpy(out, &latest_monitor_data, sizeof(BMS_HAL_MonitorData_t));
    }

    return 0;
}

const BMS_HAL_MonitorData_t *BMS_HAL_MonitorGetLatest(void)
{
    return &latest_monitor_data;
}

uint16_t BMS_HAL_MonitorGetCellMv(uint8_t index)
{
    if (!latest_monitor_data.valid || index >= BMS_CELL_MAX)
    {
        return 0;
    }
    return latest_monitor_data.cell_mv[index];
}

uint16_t BMS_HAL_MonitorGetPackMv(void)
{
    if (!latest_monitor_data.valid)
    {
        return 0;
    }
    return latest_monitor_data.pack_mv;
}

int32_t BMS_HAL_MonitorGetCurrentMa(void)
{
    if (!latest_monitor_data.valid)
    {
        return 0;
    }
    return latest_monitor_data.current_ma;
}

int16_t BMS_HAL_MonitorGetTempC(uint8_t index)
{
    if (!latest_monitor_data.valid || index >= BMS_TEMP_MAX)
    {
        return 0;
    }
    return latest_monitor_data.temp_c[index];
}

uint8_t BMS_HAL_MonitorIsValid(void)
{
    return latest_monitor_data.valid;
}

uint32_t BMS_HAL_MonitorGetSampleTick(void)
{
    return latest_monitor_data.sample_tick;
}
