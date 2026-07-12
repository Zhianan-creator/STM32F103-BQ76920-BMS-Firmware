#include "bms_energy.h"
#include "bms_protect.h"
#include "bms_monitor.h"
#include "bms_control.h"
#include "bms_config.h"
#include "bms_log.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* 将 float 宏转换成 permille 比较*/
#define SOC_STOP_CHG_PERMILLE    ((uint16_t)(SOC_STOP_CHG_VALUE * 1000))
#define SOC_START_CHG_PERMILLE   ((uint16_t)(SOC_START_CHG_VALUE * 1000))
#define SOC_STOP_DSG_PERMILLE    ((uint16_t)(SOC_STOP_DSG_VALUE * 1000))
#define SOC_START_DSG_PERMILLE   ((uint16_t)(SOC_START_DSG_VALUE * 1000))

/* 静态的 Energy 业务状态镜*/
static BMS_EnergyState_t energy_state = {0};

void BMS_EnergyInit(void)
{
    energy_state.charge_enable_cmd = 0;       /* 默认不自动开启充电，由用户确*/
    energy_state.discharge_enable_cmd = 0;    /* 默认不自动开启放电，由用户确*/
    energy_state.balance_enable_cmd = 1;      /* 默认开启自动被动均衡策略 */
    energy_state.soc_permille = 500;          /* 默认临时 SOC = 50% */
    
    energy_state.chg_on = 0;
    energy_state.dsg_on = 0;
    energy_state.balance_active = 0;
    energy_state.balance_mask = 0;
    energy_state.balance_start_tick = 0;
    energy_state.balance_rise_until_tick = 0;
    energy_state.last_action_tick = 0;
    
    energy_state.chg_start_count = 0;
    energy_state.chg_stop_count = 0;
    energy_state.dsg_start_count = 0;
    energy_state.dsg_stop_count = 0;
    energy_state.balance_start_count = 0;
    energy_state.balance_stop_count = 0;
    
    energy_state.last_chg_result = BMS_CONTROL_OK;
    energy_state.last_dsg_result = BMS_CONTROL_OK;
    energy_state.last_balance_result = BMS_CONTROL_OK;
}

void BMS_EnergyUpdate(void)
{
    /* 先读取真实的 MOS 通道物理开启状态进行状态同*/
    energy_state.chg_on = BMS_ControlIsChgOn();
    energy_state.dsg_on = BMS_ControlIsDsgOn();

    /* 1. 执行充放MOS 控制管理 */
    BMS_EnergyChgDsgManage();

    /* 2. 执行被动均衡控制管理 */
    BMS_EnergyBalanceManage();
}

void BMS_EnergyChgDsgManage(void)
{
    const BMS_ProtectState_t *prot = BMS_ProtectGetState();
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    uint32_t current_tick = osKernelGetTickCount();

    if (prot == NULL || mon == NULL)
    {
        return;
    }

    /* 1. 安全保护第一：当不允许充/放电时，无条件强制断开对应 MOS */
    if (prot->charge_allowed == 0 && energy_state.chg_on == 1)
    {
        energy_state.last_chg_result = BMS_ControlChgOff();
        energy_state.chg_on = 0;
        energy_state.chg_stop_count++;
        energy_state.last_action_tick = current_tick;
        
        if (prot->ov_active)
        {
            BMS_LOGW("ENERGY", "OV active, CHG forced OFF");

            
        }
        else
        {
            BMS_LOGW("ENERGY", "protection active, CHG forced OFF");
        }
    }

    if (prot->discharge_allowed == 0 && energy_state.dsg_on == 1)
    {
        energy_state.last_dsg_result = BMS_ControlDsgOff();
        energy_state.dsg_on = 0;
        energy_state.dsg_stop_count++;
        energy_state.last_action_tick = current_tick;
        
        if (prot->uv_active)
        {
            BMS_LOGW("ENERGY", "UV active, DSG forced OFF");
        }
        else if (prot->ocd_active)
        {
            BMS_LOGW("ENERGY", "OCD active, DSG forced OFF");
        }
        else if (prot->scd_active)
        {
            BMS_LOGE("ENERGY", "SCD active, DSG forced OFF");
        }
        else
        {
            BMS_LOGW("ENERGY", "protection active, DSG forced OFF");
        }
    }

    /* 2. 正常状态下的充放电管理决策逻辑 */
    if (mon->sys_mode == BMS_SYS_MODE_CHARGE)
    {
        /* 充电模式下检查是否过充达到停止条*/
        if (energy_state.soc_permille >= SOC_STOP_CHG_PERMILLE)
        {
            if (energy_state.chg_on == 1)
            {
                energy_state.last_chg_result = BMS_ControlChgOff();
                if (energy_state.last_chg_result == BMS_CONTROL_OK)
                {
                    energy_state.chg_on = 0;
                    energy_state.chg_stop_count++;
                    energy_state.last_action_tick = current_tick;
                    BMS_LOGI("ENERGY", "Stop Charge");
                }
            }
        }
    }
    else if (mon->sys_mode == BMS_SYS_MODE_DISCHARGE)
    {
        /* 放电模式下检查是否过放达到停止条*/
        if (energy_state.soc_permille <= SOC_STOP_DSG_PERMILLE)
        {
            if (energy_state.dsg_on == 1)
            {
                energy_state.last_dsg_result = BMS_ControlDsgOff();
                if (energy_state.last_dsg_result == BMS_CONTROL_OK)
                {
                    energy_state.dsg_on = 0;
                    energy_state.dsg_stop_count++;
                    energy_state.last_action_tick = current_tick;
                    BMS_LOGI("ENERGY", "Stop Discharge");
                }
            }
        }
    }
    else if (mon->sys_mode == BMS_SYS_MODE_STANDBY)
    {
        /* 待机模式下判定自动恢复充电或放电条件 */
        uint8_t no_chg_fault = (prot->ov_active == 0);
        uint8_t not_in_rise_delay = (energy_state.balance_rise_until_tick == 0 || current_tick >= energy_state.balance_rise_until_tick);

        /* 自动恢复开启充电条*/
        if (energy_state.charge_enable_cmd == 1 &&
            prot->charge_allowed == 1 &&
            no_chg_fault &&
            not_in_rise_delay &&
            energy_state.soc_permille < SOC_START_CHG_PERMILLE)
        {
            if (energy_state.chg_on == 0)
            {
                energy_state.last_chg_result = BMS_ControlChgOn();
                if (energy_state.last_chg_result == BMS_CONTROL_OK)
                {
                    energy_state.chg_on = 1;
                    energy_state.chg_start_count++;
                    energy_state.last_action_tick = current_tick;
                    BMS_LOGI("ENERGY", "Start Charge");
                }
            }
        }

        /* 自动恢复开启放电条*/
        uint8_t no_dsg_fault = (prot->uv_active == 0 && prot->ocd_active == 0 && prot->scd_active == 0);
        if (energy_state.discharge_enable_cmd == 1 &&
            prot->discharge_allowed == 1 &&
            no_dsg_fault &&
            energy_state.soc_permille > SOC_START_DSG_PERMILLE)
        {
            if (energy_state.dsg_on == 0)
            {
                energy_state.last_dsg_result = BMS_ControlDsgOn();
                if (energy_state.last_dsg_result == BMS_CONTROL_OK)
                {
                    energy_state.dsg_on = 1;
                    energy_state.dsg_start_count++;
                    energy_state.last_action_tick = current_tick;
                    BMS_LOGI("ENERGY", "Start Discharge");
                }
            }
        }
    }
}

void BMS_EnergyBalanceManage(void)
{
    const BMS_MonitorData_t *mon = BMS_MonitorGetData();
    const BMS_ProtectState_t *prot = BMS_ProtectGetState();
    uint32_t current_tick = osKernelGetTickCount();

    if (mon == NULL || !mon->data_valid || prot == NULL)
    {
        return;
    }

    if (prot->any_active)
    {
        if (prot->output_safe_confirmed &&
            (energy_state.balance_active || (energy_state.balance_mask != 0U)))
        {
            energy_state.balance_active = 0U;
            energy_state.balance_mask = 0U;
            energy_state.balance_stop_count++;
        }
        return;
    }

    uint16_t start_volt_mv = (uint16_t)(INIT_BALANCE_VOLTAGE * 1000.0f);
    uint16_t diff_mv = (uint16_t)(BALANCE_DIFFE_VOLTAGE * 1000.0f);

    /* 1. 若当前正在运行均衡，检查周期时间是否达到上*/
    if (energy_state.balance_active == 1)
    {
        if (current_tick - energy_state.balance_start_tick >= (uint32_t)BALANCE_CYCLE_TIME * 1000)
        {
            /* 均衡运行时间到达限额0s），安全清零所有通道，并进入电压回升等待*/
            energy_state.last_balance_result = BMS_ControlBalanceClear();
            if (energy_state.last_balance_result == BMS_CONTROL_OK)
            {
                energy_state.balance_active = 0;
                energy_state.balance_mask = 0;
                energy_state.balance_stop_count++;
                energy_state.balance_rise_until_tick = current_tick + BALANCE_VOLT_RISE_DELAY;
                BMS_LOGI("ENERGY", "Balance Timer End");
            }
        }
        return;
    }

    /* 2. 判定电压回升延时状态是否结*/
    if (energy_state.balance_rise_until_tick != 0)
    {
        if (current_tick < energy_state.balance_rise_until_tick)
        {
            return; /* 仍在电压回升干扰排除阶段，不得开启均*/
        }
        else
        {
            energy_state.balance_rise_until_tick = 0; /* 时间已过，清除标*/
        }
    }

    /* 3. 检查启动新一轮被动均衡必须满足的先决条件 */
    if (energy_state.balance_enable_cmd == 1 &&
        (mon->sys_mode == BMS_SYS_MODE_STANDBY || mon->sys_mode == BMS_SYS_MODE_CHARGE) &&
        mon->max_cell_mv >= start_volt_mv &&
        mon->delta_cell_mv >= diff_mv)
    {
        /* 4. 均衡筛选算法：高电压优先筛选，并避开相邻电芯冲突 */
        int indices[BMS_CELL_MAX];
        int cell_count = mon->cell_count;
        uint16_t mask = 0;

        for (int i = 0; i < cell_count; i++)
        {
            indices[i] = i;
        }

        /* 使用简单排序将电芯以当前电压从高到低排*/
        for (int i = 0; i < cell_count - 1; i++)
        {
            for (int j = 0; j < cell_count - 1 - i; j++)
            {
                if (mon->cell_mv[indices[j]] < mon->cell_mv[indices[j + 1]])
                {
                    int temp = indices[j];
                    indices[j] = indices[j + 1];
                    indices[j + 1] = temp;
                }
            }
        }

        /* 逐个检索高电压的电芯，确保压差满足条件，并且避开左右相邻电芯的已选状*/
        for (int i = 0; i < cell_count; i++)
        {
            int idx = indices[i];
            uint16_t cell_voltage = mon->cell_mv[idx];

            /* 高于均衡启动电压，且比最低电芯高出压差设定(50mV) */
            if (cell_voltage >= start_volt_mv && cell_voltage >= (mon->min_cell_mv + diff_mv))
            {
                uint8_t neighbor_selected = 0;

                /* 校验左邻 */
                if (idx > 0 && (mask & (1 << (idx - 1))))
                {
                    neighbor_selected = 1;
                }
                /* 校验右邻 */
                if (idx < (cell_count - 1) && (mask & (1 << (idx + 1))))
                {
                    neighbor_selected = 1;
                }

                if (!neighbor_selected)
                {
                    mask |= (1 << idx);
                    BMS_LOGI("ENERGY", "Balance Cell: %d", idx + 1);
                }
            }
        }

        /* 5. 发现合格筛选出的均衡电芯，下发硬件 mask 控制 */
        if (mask != 0)
        {
            energy_state.balance_mask = mask;
            energy_state.last_balance_result = BMS_ControlBalanceApplyMask(mask);
            if (energy_state.last_balance_result == BMS_CONTROL_OK)
            {
                energy_state.balance_active = 1;
                energy_state.balance_start_tick = current_tick;
                energy_state.balance_start_count++;
                BMS_LOGI("ENERGY", "Balance Start mask=0x%X", (unsigned int)mask);
            }
            else
            {
                energy_state.balance_mask = 0U;
            }
        }
    }
}

const BMS_EnergyState_t *BMS_EnergyGetState(void)
{
    return &energy_state;
}

void BMS_EnergySetSocPermille(uint16_t soc)
{
    if (soc > 1000)
    {
        soc = 1000;
    }
    energy_state.soc_permille = soc;
}

uint16_t BMS_EnergyGetSocPermille(void)
{
    return energy_state.soc_permille;
}

void BMS_EnergySetChargeEnable(uint8_t enable)
{
    energy_state.charge_enable_cmd = enable;
}

void BMS_EnergySetDischargeEnable(uint8_t enable)
{
    energy_state.discharge_enable_cmd = enable;
}

void BMS_EnergySetBalanceEnable(uint8_t enable)
{
    uint32_t current_tick = osKernelGetTickCount();
    uint8_t was_active = energy_state.balance_active;
    uint16_t old_mask = energy_state.balance_mask;

    energy_state.balance_enable_cmd = enable ? 1 : 0;

    if (energy_state.balance_enable_cmd == 0)
    {
        energy_state.last_balance_result = BMS_ControlBalanceClear();
        if (energy_state.last_balance_result == BMS_CONTROL_OK)
        {
            energy_state.balance_active = 0;
            energy_state.balance_mask = 0;
            energy_state.balance_start_tick = 0;
            energy_state.balance_rise_until_tick = 0;
            energy_state.last_action_tick = current_tick;

            if (was_active || old_mask != 0)
            {
                energy_state.balance_stop_count++;
            }

            BMS_LOGI("ENERGY", "Balance Disabled");
        }
    }
}
