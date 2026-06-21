#include "bms_monitor.h"
#include "bms_hal_monitor.h"
#include "bms_protect.h"
#include "bms_energy.h"
#include "bms_uart_cmd.h"
#include "bms_log.h"
#include "cmsis_os2.h"
#include "drv_softi2c_bq769x0.h"
#include <string.h>
#include <stdio.h>

/* 全局静Monitor 业务上下*/
static BMS_MonitorContext_t bms_monitor_ctx = {0};

int BMS_MonitorInit(void)
{
    /* 1. 初始化底层的 HAL Monitor 抽象*/
    BMS_HAL_MonitorInit();
    
    /* 2. 重置整个业务层运行上下文 */
    memset(&bms_monitor_ctx, 0, sizeof(BMS_MonitorContext_t));
    
    /* 3. 设置默认数量配置参数 */
    bms_monitor_ctx.data.cell_count = BMS_CELL_MAX;
    bms_monitor_ctx.data.temp_count = BMS_TEMP_MAX;
    
    /* 4. 初始化系统工作模*/
    bms_monitor_ctx.data.sys_mode = BMS_SYS_MODE_STANDBY;
    bms_monitor_ctx.data.last_sys_mode = BMS_SYS_MODE_STANDBY;
    bms_monitor_ctx.data.sys_mode_enter_tick = osKernelGetTickCount();
    bms_monitor_ctx.data.standby_start_tick = osKernelGetTickCount();
    bms_monitor_ctx.data.sleep_enter_tick = 0;
    bms_monitor_ctx.data.sleep_request = 0;
    bms_monitor_ctx.data.sleep_allowed = 0;
    
    /* 启动单次转移打印 */
    BMS_LOGI("MON", "mode INIT -> STANDBY");
    
    return 0;
}

const char *BMS_MonitorSysModeToString(BMS_SysMode_t mode)
{
    switch (mode)
    {
        case BMS_SYS_MODE_STANDBY:   return "STANDBY";
        case BMS_SYS_MODE_CHARGE:    return "CHARGE";
        case BMS_SYS_MODE_DISCHARGE: return "DISCHARGE";
        case BMS_SYS_MODE_SLEEP:     return "SLEEP";
        default:                     return "UNKNOWN";
    }
}

static uint8_t BMS_MonitorCanEnterSleep(void)
{
    const BMS_EnergyState_t *energy = BMS_EnergyGetState();

    /* 1. 检查是否有软件保护激*/
    if (BMS_ProtectIsAnyActive())
    {
        return 0;
    }

    /* 2. 检ALERT 告警引脚状(PB12 上升沿告警，若引脚为高电平表示有硬件告警挂起) */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET)
    {
        return 0;
    }


    if (energy != NULL && energy->balance_active)
    {
        return 0;
    }

    if (BMS_UartCmdIsActive())
    {
        return 0;
    }

    /* 3. 如果检测到有外部负载接入，则不应进入休眠 */
    if (BQ769X0_LoadDetect())
    {
        return 0;
    }

    return 1;
}

static void BMS_MonitorUpdateSysMode(BMS_MonitorData_t *data)
{
    uint32_t now = osKernelGetTickCount();
    BMS_SysMode_t next_mode = data->sys_mode;
    int32_t current = data->current_ma;

    /* 保存上一次状*/
    data->last_sys_mode = data->sys_mode;

    /* 状态机转移逻辑 */
    switch (data->sys_mode)
    {
        case BMS_SYS_MODE_STANDBY:
            if (current >= BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_CHARGE;
            }
            else if (current <= -BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_DISCHARGE;
            }
            else
            {
                /* 持续在待机状态，判断是否可申请休*/
                if (now - data->standby_start_tick >= BMS_MONITOR_SLEEP_DELAY_MS)
                {
                    if (BMS_MonitorCanEnterSleep())
                    {
                        next_mode = BMS_SYS_MODE_SLEEP;
                    }
                }
            }
            break;

        case BMS_SYS_MODE_CHARGE:
            if (current < BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_STANDBY;
            }
            else if (current <= -BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                /* 充放电电流反向切*/
                next_mode = BMS_SYS_MODE_DISCHARGE;
            }
            break;

        case BMS_SYS_MODE_DISCHARGE:
            if (current > -BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_STANDBY;
            }
            else if (current >= BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                /* 充放电电流反向切*/
                next_mode = BMS_SYS_MODE_CHARGE;
            }
            break;

        case BMS_SYS_MODE_SLEEP:
            /* 睡眠唤醒逻辑：检测到大于阻碍的充放电电流立即退出睡*/
            if (current >= BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_CHARGE;
            }
            else if (current <= -BMS_MONITOR_CURRENT_STANDBY_MA)
            {
                next_mode = BMS_SYS_MODE_DISCHARGE;
            }
            break;

        default:
            next_mode = BMS_SYS_MODE_STANDBY;
            break;
    }

    /* 发生状态转*/
    if (next_mode != data->sys_mode)
    {
        /* 模式切入前的属性清空与时间戳更*/
        if (next_mode == BMS_SYS_MODE_STANDBY)
        {
            data->standby_start_tick = now;
            data->sleep_request = 0;
            data->sleep_allowed = 0;
        }
        else if (next_mode == BMS_SYS_MODE_CHARGE || next_mode == BMS_SYS_MODE_DISCHARGE)
        {
            data->sleep_request = 0;
            data->sleep_allowed = 0;
            data->standby_start_tick = 0;
        }
        else if (next_mode == BMS_SYS_MODE_SLEEP)
        {
            data->sleep_request = 1;
            data->sleep_allowed = 1;
            data->sleep_enter_tick = now;
        }

        /* 状态转移打印一次，防止刷屏 */
        BMS_LOGI("MON", "mode %s -> %s%s", 
               BMS_MonitorSysModeToString(data->sys_mode), 
               BMS_MonitorSysModeToString(next_mode),
               (next_mode == BMS_SYS_MODE_SLEEP) ? " request" : "");

        data->sys_mode = next_mode;
        data->sys_mode_enter_tick = now;
    }
}

int BMS_MonitorUpdate(void)
{
    BMS_HAL_MonitorData_t hal_data;
    
    /* 1. 物理触发底层HAL Monitor 适配层采*/
    if (BMS_HAL_MonitorSample(&hal_data) != 0)
    {
        /* 采样失败，累加计数器并复位所有有效性标*/
        bms_monitor_ctx.error_count++;
        bms_monitor_ctx.last_error = -1;
        bms_monitor_ctx.data.voltage_valid = 0;
        bms_monitor_ctx.data.temp_valid = 0;
        bms_monitor_ctx.data.current_valid = 0;
        bms_monitor_ctx.data.data_valid = 0;
        BMS_LOGE("MON", "HAL sample failed (err=%lu)",
               (unsigned long)bms_monitor_ctx.error_count);
        return -1;
    }
    
    /* 2. 深拷贝底层原始电参至 Monitor 上下*/
    memcpy(bms_monitor_ctx.data.cell_mv, hal_data.cell_mv, sizeof(hal_data.cell_mv));
    bms_monitor_ctx.data.pack_mv = hal_data.pack_mv;
    bms_monitor_ctx.data.current_ma = hal_data.current_ma;
    memcpy(bms_monitor_ctx.data.temp_c, hal_data.temp_c, sizeof(hal_data.temp_c));
    bms_monitor_ctx.data.last_sample_tick = hal_data.sample_tick;
    
    /* 3. 数据合理性校验：校验各电芯单体电*/
    uint8_t volt_ok = 1;
    uint32_t cell_sum_mv = 0;

    for (uint8_t i = 0; i < BMS_CELL_MAX; i++)
    {
        uint16_t mv = bms_monitor_ctx.data.cell_mv[i];
        if (mv < BMS_MONITOR_CELL_MIN_VALID_MV || mv > BMS_MONITOR_CELL_MAX_VALID_MV)
        {
            volt_ok = 0;
            BMS_LOGW("MON", "Cell%d %dmV out of range", i + 1, (int)mv);
        }
        cell_sum_mv += mv;
    }

    /* 校验 Pack 总包电压与所有电芯单体电压求和之间的物理合理偏差 */
    uint32_t diff_mv = (bms_monitor_ctx.data.pack_mv >= cell_sum_mv) ?
                       (bms_monitor_ctx.data.pack_mv - cell_sum_mv) :
                       (cell_sum_mv - bms_monitor_ctx.data.pack_mv);

    if (diff_mv > BMS_MONITOR_PACK_SUM_TOLERANCE_MV)
    {
        volt_ok = 0;
        BMS_LOGW("MON", "Pack-CellSum diff %ldmV", (long)diff_mv);
    }
    bms_monitor_ctx.data.voltage_valid = volt_ok;

    /* 4. 数据合理性校验：温度校验 */
    uint8_t temp_ok = 1;
    for (uint8_t i = 0; i < BMS_TEMP_MAX; i++)
    {
        int16_t tc = bms_monitor_ctx.data.temp_c[i];
        if (tc < BMS_MONITOR_TEMP_MIN_VALID_C || tc > BMS_MONITOR_TEMP_MAX_VALID_C)
        {
            temp_ok = 0;
            BMS_LOGW("MON", "Temp%d %dC out of range", i + 1, (int)tc);
        }
    }
    bms_monitor_ctx.data.temp_valid = temp_ok;
    
    /* 5. 数据合理性校验：电流校验 */
    if (bms_monitor_ctx.data.current_ma > BMS_MONITOR_CHARGE_CURRENT_MAX_MA ||
        bms_monitor_ctx.data.current_ma < -BMS_MONITOR_DISCHARGE_CURRENT_MAX_MA)
    {
        bms_monitor_ctx.data.current_valid = 0;
        BMS_LOGW("MON", "Current %ldmA out of range",
                 (long)bms_monitor_ctx.data.current_ma);
    }
    else
    {
        bms_monitor_ctx.data.current_valid = 1;
    }
    
    /* 6. 更新汇总的整包有效性标data_valid */
    bms_monitor_ctx.data.data_valid = (bms_monitor_ctx.data.voltage_valid &&
                                       bms_monitor_ctx.data.temp_valid &&
                                       bms_monitor_ctx.data.current_valid);
                                       
    /* 7. 数值统计分析：计算最大单体、最小单体、压差、平均单体及单体索引 */
    uint16_t max_val = 0;
    uint16_t min_val = 65535;
    uint8_t max_idx = 0;
    uint8_t min_idx = 0;
    uint32_t sum_val = 0;
    
    for (uint8_t i = 0; i < BMS_CELL_MAX; i++)
    {
        uint16_t mv = bms_monitor_ctx.data.cell_mv[i];
        sum_val += mv;
        
        if (mv > max_val)
        {
            max_val = mv;
            max_idx = i + 1; /* 外部电芯索引编号1 开*/
        }
        if (mv < min_val)
        {
            min_val = mv;
            min_idx = i + 1; /* 外部电芯索引编号1 开*/
        }
    }
    
    bms_monitor_ctx.data.max_cell_mv = max_val;
    bms_monitor_ctx.data.min_cell_mv = min_val;
    bms_monitor_ctx.data.delta_cell_mv = max_val - min_val;
    bms_monitor_ctx.data.avg_cell_mv = (uint16_t)(sum_val / BMS_CELL_MAX);
    bms_monitor_ctx.data.max_cell_index = max_idx;
    bms_monitor_ctx.data.min_cell_index = min_idx;
    
    /* 8. 成功计数累计，更新时间戳和标*/
    bms_monitor_ctx.sample_count++;
    bms_monitor_ctx.last_error = 0;
    bms_monitor_ctx.data.updated = 1;
    bms_monitor_ctx.data.last_update_tick = osKernelGetTickCount();

    /* 9. 运行系统工作模式状态机 */
    BMS_MonitorUpdateSysMode(&bms_monitor_ctx.data);

    return 0;
}

const BMS_MonitorData_t *BMS_MonitorGetData(void)
{
    return &bms_monitor_ctx.data;
}

uint8_t BMS_MonitorIsValid(void)
{
    return bms_monitor_ctx.data.data_valid;
}

uint16_t BMS_MonitorGetCellMv(uint8_t index)
{
    if (index >= BMS_CELL_MAX)
    {
        return 0;
    }
    return bms_monitor_ctx.data.cell_mv[index];
}

uint16_t BMS_MonitorGetPackMv(void)
{
    return bms_monitor_ctx.data.pack_mv;
}

int32_t BMS_MonitorGetCurrentMa(void)
{
    return bms_monitor_ctx.data.current_ma;
}

int16_t BMS_MonitorGetTempC(uint8_t index)
{
    if (index >= BMS_TEMP_MAX)
    {
        return 0;
    }
    return bms_monitor_ctx.data.temp_c[index];
}

uint16_t BMS_MonitorGetMaxCellMv(void)
{
    return bms_monitor_ctx.data.max_cell_mv;
}

uint16_t BMS_MonitorGetMinCellMv(void)
{
    return bms_monitor_ctx.data.min_cell_mv;
}

uint16_t BMS_MonitorGetDeltaCellMv(void)
{
    return bms_monitor_ctx.data.delta_cell_mv;
}

uint16_t BMS_MonitorGetAvgCellMv(void)
{
    return bms_monitor_ctx.data.avg_cell_mv;
}

uint8_t BMS_MonitorGetMaxCellIndex(void)
{
    return bms_monitor_ctx.data.max_cell_index;
}

uint8_t BMS_MonitorGetMinCellIndex(void)
{
    return bms_monitor_ctx.data.min_cell_index;
}

uint8_t BMS_MonitorIsUpdated(void)
{
    return bms_monitor_ctx.data.updated;
}

void BMS_MonitorClearUpdated(void)
{
    bms_monitor_ctx.data.updated = 0;
}

uint32_t BMS_MonitorGetSampleCount(void)
{
    return bms_monitor_ctx.sample_count;
}
