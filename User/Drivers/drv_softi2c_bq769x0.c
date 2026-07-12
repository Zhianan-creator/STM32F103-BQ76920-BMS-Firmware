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
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-6-5       逍遥吾皇          the first version
 * 2022-6-18      逍遥吾皇          (1)重新对三元锂电池SOC对应电压数据表排*									(3)utils增加了二分查找算法用于开路SOC计算
 *									(2)修改了开路电压计算函数使之过程更清晰
 * 2022-7-2       逍遥吾皇          去掉查表方法,更新热敏电阻计算公式
 * 2023-1-5       逍遥吾皇          去掉了BQ769X0驱动的动态内存分*/

/*
 * BQ769X0电池管理芯片驱动 - FreeRTOS适配版本
 *
 * 本文件实现了BQ76920/30/40系列电池监控芯片的底层驱包括:
 *   - CRC8校验(BQ769X0专用CRC保护I2C通信)
 *   - 寄存器读单字双字块读带CRC)
 *   - 电芯电压/电流/温度采样
 *   - 充放电MOS控制
 *   - 电芯均衡控制
 *   - ALERT告警处理(从ISR通知到任务处理的适配)
 *
 * 主要改动相比原RT-Thread版本):
 *   1. 删除rtdbg.h日志系统,改用printf(默认关闭)
 *   2. ALERT 处理：ISR 发送任务通知，任务读取并清除状态
 *   3. 删除 bms_config.h 依赖，使用可覆盖的默认配置
 *   4. 延时函数改用 CMSIS-RTOS2 osDelay
 *
 * 硬件连接:
 *   BQ769X0 I2C地址: 0x08 (7位)
 *   TS1唤醒引脚: PA15 (需要在CubeMX中释放JTDI复用)
 *   ALERT告警引脚: PB12 (已配置为外部上升沿中
 *   分流电阻: 5mΩ (RsnsValue)
 */

#include "drv_softi2c_bq769x0.h"

#include <math.h>     /* 用于log()函数,热敏电阻温度计算 */

#include "main.h"
#include "drv_soft_i2c.h"   /* 软件I2C驱动 */
#include "FreeRTOS.h"
#include "semphr.h"

#ifndef BQ769X0_RUNTIME_PRINT_ENABLE
#define BQ769X0_RUNTIME_PRINT_ENABLE  1
#endif

#if (BQ769X0_RUNTIME_PRINT_ENABLE != 0)
#define BQ769X0_RUNTIME_PRINTF(...)   printf(__VA_ARGS__)
#else
#define BQ769X0_RUNTIME_PRINTF(...)   do {} while (0)
#endif


/*==========================================================================
 * BQ 总线互斥*
 * 保护所BQ769X0 寄存器读写操作，防止多任务并发访I2C
 * BQ769X0 公共寄存器读API 内部自动加锁/解锁
 *========================================================================*/
static SemaphoreHandle_t bq_bus_mutex = NULL;
static StaticSemaphore_t bq_bus_mutex_buffer;

/* 创建 BQ 总线递归互斥锁；重复调用不会重复分配。 */
bool BQ769X0_BusLockInit(void)
{
    if (bq_bus_mutex != NULL)
    {
        return true;
    }

    bq_bus_mutex = xSemaphoreCreateRecursiveMutexStatic(&bq_bus_mutex_buffer);
    if (bq_bus_mutex == NULL)
    {
        BQ769X0_RUNTIME_PRINTF("[BQ LOCK] mutex init FAILED\r\n");
        return false;
    }

    BQ769X0_RUNTIME_PRINTF("[BQ LOCK] mutex init OK\r\n");
    return true;
}

void BQ769X0_BusLock(void)
{
    if (bq_bus_mutex != NULL)
    {
        xSemaphoreTakeRecursive(bq_bus_mutex, portMAX_DELAY);
    }
}

void BQ769X0_BusUnlock(void)
{
    if (bq_bus_mutex != NULL)
    {
        xSemaphoreGiveRecursive(bq_bus_mutex);
    }
}


/*==========================================================================
 * 驱动内部全局变量
 *========================================================================*/

/*
 * 报警回调接口(在BQ769X0_Initialize中从InitData复制)
 * 上层BMS通过InitData.AlertOps注册各类告警的处理函* 所有回调在任务上下文中执行(通过BQ769X0_ProcessAlert),不在ISR*/
static BQ769X0_AlertOpsTypedef AlertOps;


/*
 * ADC校准参数(在BQ769X0_GetADCGainOffset中从芯片读取)
 *
 * BQ769X0的ADC不是理想每个芯片的增益和偏移都略有不* 芯片出厂时会在ADCGAIN1/ADCGAIN2/ADCOFFSET寄存器中存储校准* 驱动在初始化时读取这些后续采样时用它们修正ADC读数
 *
 * Gain   - ADC增益(单位: mV/LSB),用于电压换算
 * iGain  - ADC增益的整数形避免浮点精度损失)
 * Adcoffset - ADC偏移(单位: mV),有符号*/
static float Gain = 0;
static int16_t iGain = 0;
static int8_t Adcoffset;

/*
 * 温度采样模式标志
 * BQ769X0的温度采样有两种模式:
 *   0 - 外部NTC热敏电阻(TS1引脚接NTC到GND,用于测量电池温度)
 *   1 - IC内部温度传感用于测量芯片自身温度,未验
 * 两种模式不能同时使用,切换时需要等秒让ADC稳定
 */
static uint8_t TempSampleMode = 0;


/*
 * BQ769X0寄存器组的MCU侧镜* 在内存中维护一份寄存器副本,修改后再批量写入芯片
 * 这样可以减少I2C通信次数,并且方便按位操作
 */
static RegisterGroup Registers = {0};
static uint8_t s_cell_balance_value[3] = {0};


/* ALERT ISR 只读取该句柄并发送任务通知。 */
static volatile TaskHandle_t alert_task_handle = NULL;

/* 注册接收 ALERT 通知的任务句柄。 */
void BQ769X0_RegisterAlertTask(TaskHandle_t task_handle)
{
	alert_task_handle = task_handle;
}


/*
 * 保护阈值配编译时常
 *
 * SCD(短路)阈选择89mV/44mV档位
 *   RSNS=0时实际阈4mV, RSNS=1时阈9mV
 *   当前RSNS=0,分流电阻5mΩ, 保护电流 = 44/5 = 8.8A
 *
 * OCD(过流)阈选择22mV/11mV档位
 *   RSNS=0时实际阈1mV
 *   保护电流 = 11/5 = 2.2A
 */
static const uint8_t SCDThresh = SCD_THRESH_89mV_44mV;
static const uint8_t OCDThresh = OCD_THRESH_22mV_11mV;


/*
 * 分流电阻阻单位: 欧姆)
 * 用于电流换算: I = CC_reading_uV / RsnsValue
 * 当前电路使用5mΩ分流电阻
 */
static float RsnsValue = 0.005;


/*
 * 全局采样数据(上层直接读取此变量获取最新采样
 */
BQ769X0_SampleDataTypedef BQ769X0_SampleData = {0};



/*==========================================================================
 * GPIO操作 - TS1唤醒引脚
 *
 * BQ769X0的TS1引脚有两个用
 *   1. 唤醒: Ship模式TS1输出高电平脉冲唤醒芯*   2. 温度采样: 正常工作TS1作为NTC热敏电阻的采样引*
 * 因此需要在两种模式间动态切
 *   输出模式(推挽): 用于唤醒时驱动高电平
 *   输入模式: 唤醒后切避免干扰温度采样
 *========================================================================*/

/* TS1引脚切换为推挽输出模用于唤醒) */
static void BQ769X0_TS1_SetOutMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = BQ769X0_TS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;   /* 推挽输出 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(BQ769X0_TS1_GPIO_Port, &GPIO_InitStruct);
}

/* TS1引脚切换为输入模用于温度采样) */
static void BQ769X0_TS1_SetInMode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = BQ769X0_TS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       /* 输入模式 */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(BQ769X0_TS1_GPIO_Port, &GPIO_InitStruct);
}

/* 在 HAL EXTI 回调中转发 BQ ALERT 任务通知。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BQ769X0_ALERT_Pin)
    {
        BQ769X0_AlertNotifyFromISR();
    }
}


/*==========================================================================
 * 工具函数
 *========================================================================*/

/*
 * 热敏电阻阻值换算成温度(Steinhart-Hart方程简化版)
 *
 * 原理: NTC热敏电阻的阻值随温度升高而降关系是非线性的
 * 使用B值方程近1/T = 1/T2 + ln(Rt/Rp)/B
 *
 * 参数:
 *   Rt - 热敏电阻当前阻单位:Ω),由ADC采样值换算得*
 * 返回: 温度单位:°C)
 *
 * 硬件参数(需要根据实际热敏电阻修:
 *   Rp = 10000Ω  - 25°C时的标称阻*   T2 = 298.15K  - 25°C对应的开尔文温度
 *   Bx = 3950     - B值热敏电阻数据手册给出)
 */
static float TempChange(float Rt)
{
	float temp = 0;

	float Rp = 10000;           /* 25°C标称阻*/
	float T2 = 273.15 + 25;    /* 25°C = 298.15K */
	float Bx = 3950;            /* B*/
	float Ka = 273.15;          /* 0°C = 273.15K(用于开尔文转摄氏度) */

	/* B值方1/T = 1/T2 + ln(Rt/Rp)/B, 求解T后转为°C */
	temp = 1 / (1 / T2 + log(Rt / Rp) / Bx) - Ka + 0.5;

	return temp;
}


/*
 * CRC8校验函数 - BQ769X0专用
 *
 * BQ769X0的所有I2C通信都需要CRC8校验,防止数据传输错误
 * CRC多项x^8 + x^2 + x + 1 = 0x07 (CRC_KEY)
 *
 * 算法: 逐位处理输入数据,每处理一位就检查CRC最高位
 *   如果CRC最高位: 先左移再异或多项*   如果CRC最高位: 仅左*   然后检查当前数据位,如果则异或多项式
 *
 * 参数:
 *   ptr - 数据指针
 *   len - 数据长度
 *   key - CRC多项0x07)
 * 返回: 8位CRC */
static uint8_t CRC8(uint8_t *ptr, uint8_t len, uint8_t key)
{
	uint8_t i, crc = 0;

	while (len-- != 0)
	{
		for (i = 0x80; i != 0; i /= 2)
		{
			if ((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
			{
				crc *= 2;
			}

			if ((*ptr & i) != 0)
			{
				crc ^= key;
			}
		}
		ptr++;
	}
	return (crc);
}


/*==========================================================================
 * I2C寄存器读写函*
 * BQ769X0的I2C协议要求:
 *   写操[START][ADDR+W][REG][DATA][CRC][STOP]
 *   读操[START][ADDR+W][REG][RESTART][ADDR+R][DATA][CRC][STOP]
 *
 * CRC计算方式:
 *   写单字节: CRC覆盖 [ADDR<<1, REG, DATA]
 *   读单字节: CRC覆盖 [(ADDR<<1)+1, DATA] (返回时带CRC)
 *   写双字节: 低字节CRC覆盖[ADDR<<1, REG, LOW], 高字节CRC单独计算
 *   块读每个数据字节后都跟一个CRC字节
 *========================================================================*/

/* Write one register byte with CRC. */


static bool BQ769X0_WriteRegisterByteWithCRC(uint8_t Register, uint8_t data)
{
    uint8_t dataBuffer[4];
	struct I2C_MessageTypeDef msg = {0};

	dataBuffer[0] = BQ769X0_I2C_ADDR << 1;  /* 地址字节(用于CRC计算) */
	dataBuffer[1] = Register;                 /* 寄存器地址 */
	dataBuffer[2] = data;                     /* 数据 */
	dataBuffer[3] = CRC8(dataBuffer, 3, CRC_KEY);  /* CRC校验 */

	msg.addr = BQ769X0_I2C_ADDR;
	msg.flags = I2C_WR;
	msg.buf = dataBuffer + 1;  /* 跳过地址字节,I2C驱动会自动发地址 */
	msg.tLen = 3;              /* REG + DATA + CRC = 3字节 */

    if (I2C_TransferMessages(&i2c1, &msg, 1) != 1)
    {
		BQ769X0_ERROR("Write Register Byte With CRC Fail");
		return false;
    }

    return true;
}


/*
 * 写连续多个寄存器(带CRC) - 用于批量写入配置(如SYS_CTRL1~CC_CFG)
 *
 * 数据格式: [REG][DATA0][CRC0][DATA1][CRC1]...[DATAn][CRCn]
 * 每个数据字节后都跟一个CRC字节
 *
 * 参数:
 *   startAddress - 起始寄存器地址
 *   buffer       - 待写入的数据数组
 *   length       - 数据长度(字节
 */
/* 连续写入带 CRC 的寄存器数据块。 */
bool BQ769X0_WriteBlockWithCRC(uint8_t startAddress, const uint8_t *buffer, uint8_t length)
{
	uint8_t index;
	uint8_t bufferCRC[32] = {0}, *pointer;
	const uint8_t *source = buffer;
	struct I2C_MessageTypeDef msg = {0};
	bool result = false;

	if ((source == NULL) || (length == 0U) || (length > 15U))
	{
		return false;
	}

	BQ769X0_BusLock();

	pointer = bufferCRC;
	*pointer++ = BQ769X0_I2C_ADDR << 1;
	*pointer++ = startAddress;
	*pointer++ = *source;
	*pointer = CRC8(bufferCRC, 3, CRC_KEY);

	for (index = 1; index < length; index++)
	{
		pointer++;
		source++;
		*pointer = *source;
		*(pointer + 1) = CRC8(pointer, 1, CRC_KEY);
		pointer++;
	}

	msg.addr = BQ769X0_I2C_ADDR;
	msg.flags = I2C_WR;
	msg.buf = bufferCRC + 1;
	msg.tLen = 2 * length + 1;

	if (I2C_TransferMessages(&i2c1, &msg, 1) != 1)
	{
		BQ769X0_ERROR("Write Register Block With CRC Fail");
		goto out;
	}

	result = true;

out:
	BQ769X0_BusUnlock();
	return result;
}



/*
 * 读单字节寄存带CRC) - BQ769X0标准读操*
 * 读取2字节: [DATA][CRC]
 * CRC校验: CRC8({(ADDR<<1)+1, DATA}, 2, 0x07)
 * (ADDR<<1)+1 是因为读操作的地址字节最低位
 */
bool BQ769X0_ReadRegisterByteWithCRC(uint8_t Register, uint8_t *data)
{
	uint8_t readBuffer[2], crcInput[2], crcValue;
	struct I2C_MessageTypeDef msg[2] = {0};
	bool result = false;

	BQ769X0_BusLock();

	msg[0].addr = BQ769X0_I2C_ADDR;
	msg[0].flags = I2C_WR;
	msg[0].buf = &Register;
	msg[0].tLen = 1;

	msg[1].addr = BQ769X0_I2C_ADDR;
	msg[1].flags = I2C_RD;
	msg[1].buf = readBuffer;
	msg[1].tLen = 2;

	if (I2C_TransferMessages(&i2c1, msg, 2) != 2)
	{
		BQ769X0_ERROR("Read Register Byte With CRC Fail");
		goto out;
	}

	crcInput[0] = (BQ769X0_I2C_ADDR << 1) + 1;
	crcInput[1] = readBuffer[0];

	crcValue = CRC8(crcInput, 2, CRC_KEY);
	if (crcValue != readBuffer[1])
	{
		BQ769X0_ERROR("Read Register Byte CRC Check Fail");
		goto out;
	}

	*data = readBuffer[0];
	result = true;

out:
	BQ769X0_BusUnlock();
	return result;
}


/*
 * 读双字节寄存带CRC) - 用于读取电压/电流6位数*
 * 读取4字节: [LOW_BYTE][CRC1][HIGH_BYTE][CRC2]
 * 两个字节各自独立CRC校验
 *
 * 注意: BQ769X0返回的是小端格式(低字节在
 *       组合时需data = (HIGH << 8) | LOW
 */
static bool BQ769X0_ReadRegisterWordWithCRC(uint8_t Register, uint16_t *data)
{
	uint8_t readBuffer[4], crcInput[2], crcValue;
	struct I2C_MessageTypeDef msg[2] = {0};

	msg[0].addr = BQ769X0_I2C_ADDR;
	msg[0].flags = I2C_WR;
	msg[0].buf = &Register;
	msg[0].tLen = 1;

	msg[1].addr = BQ769X0_I2C_ADDR;
	msg[1].flags = I2C_RD;
	msg[1].buf = readBuffer;
	msg[1].tLen = 4;  /* LOW + CRC1 + HIGH + CRC2 = 4字节 */

	if (I2C_TransferMessages(&i2c1, msg, 2) != 2)
	{
		BQ769X0_ERROR("Read Register Word With CRC Fail");
		return false;
	}

	/* 校验低字节CRC */
	crcInput[0] = (BQ769X0_I2C_ADDR << 1) + 1;
	crcInput[1] = readBuffer[0];

	crcValue = CRC8(crcInput, 2, CRC_KEY);
	if (crcValue != readBuffer[1])
	{
		BQ769X0_ERROR("Read Register Word CRC 1 Check Fail");
		return false;
	}

	/* 校验高字节CRC */
	crcValue = CRC8(readBuffer + 2, 1, CRC_KEY);
	if (crcValue != readBuffer[3])
	{
		BQ769X0_ERROR("Read Register Word CRC 2 Check Fail");
		return false;
	}

	/* 组合高低字节(小端格式) */
	*data = (readBuffer[2] << 8) | readBuffer[0];

	return true;
}

/*
 * 读连续多个寄存器(带CRC) - 用于批量读取电芯电压/温度*
 * 读取格式: [DATA0][CRC0][DATA1][CRC1]...[DATAn][CRCn]
 * 每个数据字节后跟一个CRC字节
 *
 * 参数:
 *   Register - 起始寄存器地址
 *   buffer   - 读取数据的存放缓冲区
 *   length   - 要读取的字节*
 * 注意: 实际I2C传输的字节数 = length * 2(每个数据字节配一个CRC字节)
 */
bool BQ769X0_ReadBlockWithCRC(uint8_t Register, uint8_t *buffer, uint8_t length)
{
	uint8_t index, crcValue, crcInput[2];
	uint8_t buf[32] = {0};
	uint8_t *readData = buf;
	struct I2C_MessageTypeDef msg[2] = {0};
	bool result = false;

	BQ769X0_BusLock();

	msg[0].addr = BQ769X0_I2C_ADDR;
	msg[0].flags = I2C_WR;
	msg[0].buf = &Register;
	msg[0].tLen = 1;

	msg[1].addr = BQ769X0_I2C_ADDR;
	msg[1].flags = I2C_RD;
	msg[1].buf = readData;
	msg[1].tLen = length * 2;

	if (I2C_TransferMessages(&i2c1, msg, 2) != 2)
	{
		BQ769X0_ERROR("Read Register Block With CRC Fail");
		goto out;
	}

	crcInput[0] = (BQ769X0_I2C_ADDR << 1) + 1;
	crcInput[1] = readData[0];

	crcValue = CRC8(crcInput, 2, CRC_KEY);
	readData++;
	if (crcValue != *readData)
	{
		BQ769X0_ERROR("Read Register Block CRC 1 Check Fail");
		goto out;
	}
	else
	{
		*buffer = *(readData - 1);
	}

	for (index = 1; index < length; index++)
	{
		readData++;
		crcValue = CRC8(readData, 1, CRC_KEY);
		readData++;
		buffer++;

		if (crcValue != *readData)
		{
			BQ769X0_ERROR("Read Register Block CRC Check Fail");
			goto out;
		}
		else
		{
			*buffer = *(readData - 1);
		}
	}

	result = true;

out:
	BQ769X0_BusUnlock();
	return result;
}


/*==========================================================================
 * 传感数据采集函数
 *
 * 这些函数从BQ769X0读取ADC采样数据,经过换算后存入BQ769X0_SampleData
 * 上层BMS任务周期性调用这些函数获取最新数*
 * 采样周期建议:
 *   电压/电流: 250ms (快速响
 *   温度: 2s (温度变化减少I2C通信开销)
 *========================================================================*/

/*
 * 更新所有电芯电*
 * 从VC1_HI_BYTE寄存器开连续读取BQ769X0_CELL_MAX节电芯的电压数据
 * 每节电芯字节(高字低字,14位ADC *
 * 换算公式:
 *   原始ADC= (高字<< 8) | 低字*   修正= (原始× iGain) / 1000 + Adcoffset
 *   电压(V) = 修正/ 1000.0
 *
 * 建议调用周期: 250ms
 */
/* 更新全部单体电压；通信失败时不覆盖上次有效采样。 */
bool BQ769X0_UpdateCellVolt(void)
{
	uint8_t index = 0;
	uint16_t iTemp = 0;
	uint8_t *pRawADCData = NULL;
	uint32_t lTemp = 0;
	/* 批量读取所有电芯的原始ADC数据(一次I2C事务) */
	bool read_ok = BQ769X0_ReadBlockWithCRC(VC1_HI_BYTE, &(Registers.VCell1.VCell1Byte.VC1_HI), BQ769X0_CELL_MAX << 1);

	if (!read_ok)
	{
		BQ769X0_ERROR("Update Cell Voltage Fail");
		return false;
	}

	/* 逐节换算 */
	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	for (index = 0; index < BQ769X0_CELL_MAX; index++)
	{
		iTemp = (unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);  /* 组合高低字节 */
		lTemp = ((unsigned long)iTemp * iGain) / 1000;  /* 乘以增益修正 */
		lTemp += Adcoffset;                               /* 加上偏移修正 */
		BQ769X0_SampleData.CellVoltage[index] = lTemp / 1000.0;  /* 转为V */
		pRawADCData += 2;  /* 指向下一节电*/
	}

	return true;
}


/*
 * 更新热敏电阻温度
 *
 * BQ769X0的温度采样需要选择模式(SYS_CTRL1.TEMP_SEL):
 *   0 = 外部NTC热敏电阻(默认,用于测量电池温度)
 *   1 = IC内部温度传感用于测量芯片温度)
 *
 * 如果当前模式不是NTC(刚切换过,需要等秒让ADC稳定
 *
 * 温度计算流程:
 *   1. 读取TS1寄存器的ADC原始*   2. 换算为电v_tsx = adc_value × 0.000382 (单位V)
 *   3. 根据分压电路计算热敏电阻阻Rts = (10000 × v_tsx) / (3.3 - v_tsx)
 *      (10KΩ上拉电阻, 3.3V参考电
 *   4. 用B值方程将阻值换算为温度
 *
 * 建议调用周期: 2s
 */
/* 更新外部热敏电阻温度；通信或计算异常时不覆盖上次有效值。 */
bool BQ769X0_UpdateTsTemp(void)
{
	uint8_t index;
	uint16_t iTemp = 0;
	float v_tsx = 0;
	float Rts = 0;
	uint8_t *pRawADCData = NULL;

	/* 如果当前不是NTC模式,需要切换并等待ADC稳定 */
	if (TempSampleMode != 0)
	{
		/* SYS_CTRL1 = 0x18: ADC使能,选择外部NTC */
		if (BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL1, 0x18) != true)
		{
			BQ769X0_ERROR("Update Tsx Temperature Fail");
			return false;
		}
		TempSampleMode = 0;
		BQ769X0_DELAY(2000);  /* 等待ADC稳定 */
	}

	/* 批量读取温度传感器ADC数据 */
	if (BQ769X0_ReadBlockWithCRC(TS1_HI_BYTE, &(Registers.TS1.TS1Byte.TS1_HI), BQ769X0_TMEP_MAX << 1) != true)
	{
		BQ769X0_ERROR("Update Tsx Temperature Fail");
		return false;
	}

	/* 逐通道换算温度 */
	pRawADCData = &Registers.TS1.TS1Byte.TS1_HI;
	for (index = 0; index < BQ769X0_TMEP_MAX; index++, pRawADCData += 2)
	{
		/* 读出ADC原始*/
		iTemp = (uint16_t)(*pRawADCData << 8) | *(pRawADCData + 1);

		/* 换算为电单位V): BQ769X0 ADC分辨率约0.382mV/LSB */
		v_tsx = iTemp * 0.000382;
		if ((v_tsx <= 0.0f) || (v_tsx >= 3.3f))
		{
			BQ769X0_ERROR("TS voltage out of range");
			return false;
		}

		/* 根据分压电路计算热敏电阻阻单位Ω) */
		/* 电路: 3.3V -- 10KΩ上拉 -- ADC采样-- NTC -- GND */
		Rts = (10000 * v_tsx) / (3.3 - v_tsx);

		/* 用B值方程将阻值换算为温度(单位°C) */
		BQ769X0_SampleData.TsxTemperature[index] = TempChange(Rts);
	}

	return true;
}



/*
 * 获取IC内部温度(未验
 *
 * 与热敏电阻温度共用TS1寄存但需要切换TEMP_SEL到内部传感器
 * 切换后需要等秒让ADC稳定
 *
 * 建议调用周期: 2s
 * 注意: 此功能未在实际硬件上验证
 */
void BQ769X0_UpdateDieTemp(void)
{
	uint16_t adc_value = 0;

	if (TempSampleMode != 1)
	{
		TempSampleMode = 1;
		/* SYS_CTRL1 = 0x10: ADC使能,选择内部温度传感*/
		if (BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL1, 0x10) != true)
		{
			BQ769X0_ERROR("Update Die Temperature Fail");
		}
		BQ769X0_DELAY(2000);
	}

	if (BQ769X0_ReadRegisterWordWithCRC(TS1_HI_BYTE, &Registers.TS1.TS1Word) != true)
	{
		BQ769X0_ERROR("Update Die Temperature Fail");
	}

	adc_value = (Registers.TS1.TS1Byte.TS1_HI << 8) | Registers.TS1.TS1Byte.TS1_LO;
	/* 内部温度换算公式(数据手册给出) */
	BQ769X0_SampleData.DieTemperature = adc_value * 382.0 / 1000000.0;
	BQ769X0_SampleData.DieTemperature = 25 - ((BQ769X0_SampleData.DieTemperature - 1.2) / 0.0042);
}


/*
 * 更新电池总电*
 * BQ769X0内置库仑CC),连续采样分流电阻上的压降
 * CC寄存0x32~0x33)存放16位有符号补码)
 *
 * 换算公式:
 *   CC读数(μV) = 16位补码× 8.44μV/LSB
 *   电流(A) = CC读数(μV) / Rsns(μΩ)
 *    Rsns=5mΩ=5000μΩ, CC=1000 -> 电流 = 1000×8.44/5000000 = 1.688mA
 *
 * 建议调用周期: 250ms
 */
/* 更新电池电流；通信失败时不把故障伪装成零电流。 */
bool BQ769X0_UpdateCurrent(void)
{
	int32_t temp;
	bool read_ok = BQ769X0_ReadRegisterWordWithCRC(CC_HI_BYTE, &Registers.CC.CCWord);

	if (!read_ok)
	{
		BQ769X0_ERROR("Update Current Fail");
		return false;
	}

	/* 组合高低字节 */
	temp = Registers.CC.CCByte.CC_HI << 8 | Registers.CC.CCByte.CC_LO;

	/* 16位补码转有符号整*/
	if (temp & 0x8000)
	{
		temp = -((~temp + 1) & 0xFFFF);  /* 负数: 取反*/
	}

	/* 换算为安A) */
	BQ769X0_SampleData.BatteryCurrent = ((temp * 8.44) / RsnsValue) * 0.000001;
	return true;
}


/*
 * 更新电池总电*
 * BAT寄存0x2A~0x2B)存放电池包总电压的ADC * 换算公式:
 *   电压(mV) = 4 × Gain × ADC+ CELL_MAX × Adcoffset
 *   电压(V) = 电压(mV) / 1000
 *
 * 建议调用周期: 250ms
 */
/* 更新电池包总电压；通信失败时不覆盖上次有效采样。 */
bool BQ769X0_UpadteBatVolt(void)
{
	uint16_t adc_value;

	if (BQ769X0_ReadRegisterWordWithCRC(BAT_HI_BYTE, &Registers.VBat.VBatWord) != true)
	{
		BQ769X0_ERROR("Update Battery Voltage Fail");
		return false;
	}

	adc_value = Registers.VBat.VBatByte.BAT_HI << 8 | Registers.VBat.VBatByte.BAT_LO;
	BQ769X0_SampleData.BatteryVoltage = 4 * Gain * adc_value;
	BQ769X0_SampleData.BatteryVoltage += BQ769X0_CELL_MAX * Adcoffset;  /* 单位mV */
	BQ769X0_SampleData.BatteryVoltage /= 1000;  /* 转为V */
	return true;
}


/* 从 EXTI ISR 向 ALERT 任务发送通知，不在中断中执行 I2C。 */
void BQ769X0_AlertNotifyFromISR(void)
{
	TaskHandle_t task_handle = (TaskHandle_t)alert_task_handle;

	if (task_handle != NULL)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

#define BQ_SYS_STAT_FAULT_MASK \
    (SYS_STAT_OVRD_BIT | SYS_STAT_UV_BIT | SYS_STAT_OV_BIT | \
     SYS_STAT_SCD_BIT | SYS_STAT_OCD_BIT | SYS_STAT_DEVICE_BIT)

static void BQ769X0_HandleSysStat(uint8_t sys_stat)
{
	if (sys_stat == 0)
	{
		return;
	}

	if ((sys_stat & BQ_SYS_STAT_FAULT_MASK) == 0)
	{
		BQ769X0_WriteRegisterByteWithCRC(SYS_STAT, sys_stat);
		if ((sys_stat & SYS_STAT_CC_BIT) && AlertOps.cc != NULL)
		{
			AlertOps.cc();
		}
		return;
	}

	BQ769X0_RUNTIME_PRINTF("[ALERT] SYS_STAT=0x%02X\r\n", sys_stat);

	if (sys_stat & SYS_STAT_CC_BIT)
	{
		if (AlertOps.cc != NULL) AlertOps.cc();
	}

	if (sys_stat & SYS_STAT_DEVICE_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][DEVICE] Device fault\r\n");
		if (AlertOps.device != NULL) AlertOps.device();
	}

	if (sys_stat & SYS_STAT_OVRD_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][OVRD] External alert override\r\n");
		if (AlertOps.ovrd != NULL) AlertOps.ovrd();
	}

	if (sys_stat & SYS_STAT_UV_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][UV] Under voltage fault\r\n");
		if (AlertOps.uv != NULL) AlertOps.uv();
	}

	if (sys_stat & SYS_STAT_OV_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][OV] Over voltage fault\r\n");
		if (AlertOps.ov != NULL) AlertOps.ov();
	}

	if (sys_stat & SYS_STAT_SCD_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][SCD] Short circuit discharge fault\r\n");
		if (AlertOps.scd != NULL) AlertOps.scd();
	}

	if (sys_stat & SYS_STAT_OCD_BIT)
	{
		BQ769X0_RUNTIME_PRINTF("[ALERT][OCD] Over current discharge fault\r\n");
		if (AlertOps.ocd != NULL) AlertOps.ocd();
	}

	BQ769X0_WriteRegisterByteWithCRC(SYS_STAT, sys_stat);
	BQ769X0_RUNTIME_PRINTF("[ALERT] SYS_STAT cleared: 0x%02X\r\n", sys_stat);
}

bool BQ769X0_ProcessAlert(void)
{
	uint8_t reg_value = 0;

	/* 每次都读取 SYS_STAT；任务的超时轮询负责兜底丢失的边沿通知。 */
	if (BQ769X0_ReadRegisterByteWithCRC(SYS_STAT, &reg_value) != true)
	{
		return false;
	}

	if (reg_value == 0)
	{
		return true;
	}

	/* Handle the real hardware alert and clear SYS_STAT. */
	BQ769X0_HandleSysStat(reg_value);

	return true;
}

/*==========================================================================
 * ADC校准和配置函*========================================================================*/

/*
 * 读取ADC增益和偏移校准*
 * BQ769X0的ADC增益由ADCGAIN1和ADCGAIN2两个寄存器共同决
 *   GAIN = ADCGAIN_BASE + (ADCGAIN1[3:2] << 1) + ADCGAIN2[7:5]
 *   其中ADCGAIN_BASE = 365 (uV/LSB)
 *
 * 偏移量ADCOFFSET是有符号8位
 *   0x00~0x7F: 正偏0~127)
 *   0x80~0xFF: 负偏-128~-1), 需要减56得到实际*/
/* 读取并计算 ADC 增益与偏移；任一寄存器读取失败时返回 false。 */
static bool BQ769X0_GetADCGainOffset(void)
{
	if (!BQ769X0_ReadRegisterByteWithCRC(ADCGAIN1, &(Registers.ADCGain1.ADCGain1Byte)) ||
	    !BQ769X0_ReadRegisterByteWithCRC(ADCGAIN2, &(Registers.ADCGain2.ADCGain2Byte)) ||
	    !BQ769X0_ReadRegisterByteWithCRC(ADCOFFSET, &(Registers.ADCOffset)))
	{
		return false;
	}

	/* 拼接增益分散在两个寄存器*/
	Gain = (ADCGAIN_BASE + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0) >> 5)) / 1000.0;
	iGain = ADCGAIN_BASE + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0) >> 5);

	/* 处理有符号偏移量 */
	if (Registers.ADCOffset <= 0x7F)
	{
		Adcoffset = Registers.ADCOffset;    /* 正数 */
	}
	else
	{
		Adcoffset = Registers.ADCOffset - 256;  /* 负数 */
	}

	return true;
}


/*
 * 配置BQ769X0寄存器并验证
 *
 * 配置内容:
 *   SYS_CTRL1 = 0x18: ADC使能,选择外部NTC
 *   SYS_CTRL2 = 0x40: 使能连续电流采样,关闭充放电MOS(初始化阶段先
 *   CC_CFG = 0x19: 数据手册要求的固定用于优化库仑计性能
 *
 * 验证方法: 写入后回读全个寄存器,逐字节比* 如果不一致则死循硬件问题,需要检查I2C连接)
 */
/* 写入并回读验证 BQ 关键配置寄存器。 */
static bool BQ769X0_Configuration(void)
{
	unsigned char ReadBuffer[8] = {0};

	/* 设置寄存器镜*/
	Registers.SysCtrl1.SysCtrl1Byte = 0x18;  /* ADC使能+外部NTC */
	Registers.SysCtrl2.SysCtrl2Byte = 0x40;  /* 连续电流采样,关闭MOS */
	Registers.CCCfg = 0x19;                    /* 库仑计配必须0x19) */

	/* 批量写入8个配置寄存器(SYS_CTRL1 ~ CC_CFG) */
	if (!BQ769X0_WriteBlockWithCRC(SYS_CTRL1, &(Registers.SysCtrl1.SysCtrl1Byte), 8))
	{
		return false;
	}

	/* 回读验证 */
	if (!BQ769X0_ReadBlockWithCRC(SYS_CTRL1, ReadBuffer, 8))
	{
		return false;
	}

	/*
	 * 逐字节比ReadBuffer[0]需要屏蔽最高位,因为LOAD_PRESENT可能被置
	 * 如果任何字节不一说明写入失败(硬件问题)
	 */
	if ((ReadBuffer[0] & 0X7F) != Registers.SysCtrl1.SysCtrl1Byte
	|| ReadBuffer[1] != Registers.SysCtrl2.SysCtrl2Byte
	|| ReadBuffer[2] != Registers.Protect1.Protect1Byte
	|| ReadBuffer[3] != Registers.Protect2.Protect2Byte
	|| ReadBuffer[4] != Registers.Protect3.Protect3Byte
	|| ReadBuffer[5] != Registers.OVTrip
	|| ReadBuffer[6] != Registers.UVTrip
	|| ReadBuffer[7] != Registers.CCCfg)
	{
		BQ769X0_ERROR("BQ769X0 config register fail,Please reset BMS board");
		return false;
	}

	return true;
}



/*
 * 负载检*
 * BQ769X0可以检测电池包是否连接了负* 条件: 充电MOS关闭(CHG_ON=0) LOAD_PRESENT标志置位
 * LOAD_PRESENT标志在SYS_CTRL1寄存器中(只读)
 */
bool BQ769X0_LoadDetect(void)
{
	BQ769X0_ReadRegisterWordWithCRC(SYS_CTRL1, (uint16_t *)&Registers.SysCtrl1.SysCtrl1Byte);
	if (Registers.SysCtrl2.SysCtrl2Bit.CHG_ON == 0)  /* 不在充电状*/
	{
		if (Registers.SysCtrl1.SysCtrl1Bit.LOAD_PRESENT)
		{
			return true;  /* 检测到负载 */
		}
	}
	return false;
}

/*
 * 唤醒BQ芯片(从Ship模式)
 *
 * BQ769X0在Ship模式下功耗极.5μA),但所有功能都关闭* 唤醒方法: 在TS1引脚产生一个高电平脉冲(至少500ms)
 *
 * 唤醒流程:
 *   1. TS1切换为推挽输出模*   2. 输出高电保持1 *   3. 输出低电*   4. TS1切换回输入模避免干扰后续温度采样)
 */
void BQ769X0_Wakeup(void)
{
	BQ769X0_TS1_SetOutMode();     /* 切换为输出模*/
	HAL_GPIO_WritePin(BQ769X0_TS1_GPIO_Port, BQ769X0_TS1_Pin, GPIO_PIN_SET);  /* 输出高电*/
	BQ769X0_DELAY(1000);          /* 保持1*/

	HAL_GPIO_WritePin(BQ769X0_TS1_GPIO_Port, BQ769X0_TS1_Pin, GPIO_PIN_RESET);  /* 输出低电*/
	BQ769X0_TS1_SetInMode();      /* 切回输入模式 */
	BQ769X0_DELAY(1000);          /* 等待稳定 */
}

/*
 * 进入Ship模式(超低功
 *
 * 按照数据手册的序连续写入SYS_CTRL1寄存
 *   第一0x00 (清除所有位)
 *   第二0x01 (设置SHUT_A)
 *   第三0x02 (设置SHUT_B, 完成关机序列)
 */
/* 按规定序列进入 Ship 模式；三次写入全部成功时返回 true。 */
bool BQ769X0_EntryShip(void)
{
	bool result;

	BQ769X0_BusLock();
	result = BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL1, 0x00) &&
	         BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL1, 0x01) &&
	         BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL1, 0x02);
	BQ769X0_BusUnlock();

	return result;
}

/*
 * 控制充放电MOS开*
 * 通过修改SYS_CTRL2寄存器的CHG_ON/DSG_ON位来控制MOS * 修改后直接写入芯片生*
 * 参数:
 *   ControlType - CHG_CONTROL(充电) 或 DSG_CONTROL(放电)
 *   NewState    - BQ_STATE_ENABLE(开) 或 BQ_STATE_DISABLE(关)
 */
bool BQ769X0_ControlDSGOrCHG(BQ769X0_ControlTypedef ControlType, BQ769X0_StateTypedef NewState)
{
	uint8_t ctrl2_val = 0;
	bool result = false;

	BQ769X0_BusLock();
	if (!BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_val))
	{
		goto out;
	}

	if (NewState == BQ_STATE_ENABLE)
	{
		ctrl2_val = (uint8_t)(ctrl2_val | (uint8_t)ControlType);
	}
	else
	{
		ctrl2_val = (uint8_t)(ctrl2_val & (uint8_t)(~(uint8_t)ControlType));
	}

	if (!BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL2, ctrl2_val))
	{
		goto out;
	}

	Registers.SysCtrl2.SysCtrl2Byte = ctrl2_val;
	result = true;

out:
	BQ769X0_BusUnlock();
	return result;
}


/*
 * 电芯均衡控制
 *
 * BQ769X0内置均衡MOS可以通过CELLBAL1~3寄存器控* 每个寄存器控节电位有
 *
 * 位图映射:
 *   CellIndex bit0~bit4  -> CELLBAL1 bit0~bit4 (1~5号电
 *   CellIndex bit5~bit9  -> CELLBAL2 bit0~bit4 (6~10号电
 *   CellIndex bit10~bit14 -> CELLBAL3 bit0~bit4 (11~15号电
 *
 * 使用static变量保存均衡状支持增量操作(开关闭指定电芯而不影响其他)
 *
 * 注意: BQ769X0不允许相邻电芯同时均硬件限制)
 *       上层BMS需要做冲突过滤,本函数只负责写寄存器
 */
/* 更新电芯均衡位图；写入并回读一致后才提交软件镜像。 */
bool BQ769X0_CellBalanceControl(BQ769X0_CellIndexTypedef CellIndex, BQ769X0_StateTypedef NewState)
{
	uint8_t next_value[3];
	uint8_t read_value[3] = {0};
	bool result = false;

	BQ769X0_BusLock();
	memcpy(next_value, s_cell_balance_value, sizeof(next_value));
	if (NewState == BQ_STATE_ENABLE)
	{
		next_value[0] |= CellIndex & 0x1F;
		next_value[1] |= (CellIndex >> 5) & 0x1F;
		next_value[2] |= (CellIndex >> 10) & 0x1F;
	}
	else if (NewState == BQ_STATE_DISABLE)
	{
		next_value[0] &= (uint8_t)~(CellIndex & 0x1F);
		next_value[1] &= (uint8_t)~((CellIndex >> 5) & 0x1F);
		next_value[2] &= (uint8_t)~((CellIndex >> 10) & 0x1F);
	}

	if (BQ769X0_WriteBlockWithCRC(CELLBAL1, next_value, 3) &&
	    BQ769X0_ReadBlockWithCRC(CELLBAL1, read_value, 3) &&
	    (memcmp(next_value, read_value, sizeof(next_value)) == 0))
	{
		memcpy(s_cell_balance_value, next_value, sizeof(s_cell_balance_value));
		result = true;
	}
	BQ769X0_BusUnlock();

	return result;
}


/*==========================================================================
 * 保护参数运行时修改函*
 * 这些函数允许在运行时动态修改保护参不需要重新初始化)
 * 修改后直接写入BQ芯片生效
 *========================================================================*/

/* 修改SCD延时 */
void BQ769X0_SCDDelaySet(BQ769X0_SCDDelayTypedef SCDDelay)
{
	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	BQ769X0_WriteRegisterByteWithCRC(PROTECT1, Registers.Protect1.Protect1Byte);
}

/* 修改OCD延时 */
void BQ769X0_OCDDelaySet(BQ769X0_OCDDelayTypedef OCDDelay)
{
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	BQ769X0_WriteRegisterByteWithCRC(PROTECT2, Registers.Protect2.Protect2Byte);
}

/* 修改UV延时 */
void BQ769X0_UVDelaySet(BQ769X0_UVDelayTypedef UVDelay)
{
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;
	BQ769X0_WriteRegisterByteWithCRC(PROTECT3, Registers.Protect3.Protect3Byte);
}

/* 修改OV延时 */
void BQ769X0_OVDelaySet(BQ769X0_OVDelayTypedef OVDelay)
{
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	BQ769X0_WriteRegisterByteWithCRC(PROTECT3, Registers.Protect3.Protect3Byte);
}

/* 修改UV阈mV) */
void BQ769X0_UVPThresholdSet(uint16_t UVPThreshold)
{
	Registers.UVTrip = (uint8_t)((((uint16_t)((UVPThreshold - Adcoffset) / Gain) - UV_THRESH_BASE) >> 4) & 0xFF);
	BQ769X0_WriteRegisterByteWithCRC(UV_TRIP, Registers.UVTrip);
}

/* 修改OV阈mV) */
void BQ769X0_OVPThresholdSet(uint16_t OVPThreshold)
{
	Registers.OVTrip = (uint8_t)((((uint16_t)((OVPThreshold - Adcoffset) / Gain) - OV_THRESH_BASE) >> 4) & 0xFF);
	BQ769X0_WriteRegisterByteWithCRC(OV_TRIP, Registers.OVTrip);
}

/* 获取SCD延时 */
BQ769X0_SCDDelayTypedef BQ769X0_SCDDelayGet(void)
{
	return (BQ769X0_SCDDelayTypedef)Registers.Protect1.Protect1Bit.SCD_DELAY;
}

/* 获取OCD延时 */
BQ769X0_OCDDelayTypedef BQ769X0_OCDDelayGet(void)
{
	return (BQ769X0_OCDDelayTypedef)Registers.Protect2.Protect2Bit.OCD_DELAY;
}

/* 获取UV延时 */
BQ769X0_UVDelayTypedef BQ769X0_UVDelayGet(void)
{
	return (BQ769X0_UVDelayTypedef)Registers.Protect3.Protect3Bit.UV_DELAY;
}

/* 获取OV延时 */
BQ769X0_OVDelayTypedef BQ769X0_OVDelayGet(void)
{
	return (BQ769X0_OVDelayTypedef)Registers.Protect3.Protect3Bit.OV_DELAY;
}

/* 获取UV阈值(mV) */
uint16_t BQ769X0_UVPThresholdGet(void)
{
	return (uint16_t)(((Registers.UVTrip << 4) + UV_THRESH_BASE) * Gain + Adcoffset);
}

/* 获取OV阈值(mV) */
uint16_t BQ769X0_OVPThresholdGet(void)
{
	return (uint16_t)(((Registers.OVTrip << 4) + OV_THRESH_BASE) * Gain + Adcoffset);
}


/*==========================================================================
 * 寄存器CRC验证函数(调试
 *
 * 此函数用于验证I2C通信链路的完整
 *   1. 通过I2C读取指定寄存数据+CRC两个字节)
 *   2. 用CRC8多项x07计算接收到的数据的CRC
 *   3. 与从机返回的CRC字节对比
 *   4. 打印寄存器地址、数据值、CRC校验结果
 *
 * 如果I2C通信失败(无ACK),说明硬件连接有问* 如果CRC校验失败,说明数据在传输过程中被干*
 * 注意: 此函数内部直接操作I2C,不经过BQ769X0_ReadRegisterByteWithCRC
 *       因为后者是static函数,且我们需要分别报告通信失败和CRC失败
 *========================================================================*/
uint8_t BQ769X0_VerifyRegisterCRC(uint8_t reg_addr, const char *reg_name)
{
	uint8_t readBuffer[2];
	uint8_t crcInput[2];
	uint8_t crcValue;
	struct I2C_MessageTypeDef msg[2] = {0};

	(void)reg_name;  /* 避免未使用警*/

	/* 构造I2C读事先写寄存器地址,再读2字节(数据+CRC) */
	msg[0].addr = BQ769X0_I2C_ADDR;
	msg[0].flags = I2C_WR;
	msg[0].buf = &reg_addr;
	msg[0].tLen = 1;

	msg[1].addr = BQ769X0_I2C_ADDR;
	msg[1].flags = I2C_RD;
	msg[1].buf = readBuffer;
	msg[1].tLen = 2;  /* 数据字节 + CRC字节 */

	/* 执行I2C传输 */
	if (I2C_TransferMessages(&i2c1, msg, 2) != 2)
	{
		BQ769X0_RUNTIME_PRINTF("[BQ] read reg=0x%02X comm=failed\r\n", reg_addr);
		return 0;
	}

	/* CRC校验: CRC8({读地址字节, 数据字节}, 2, 0x07) */
	crcInput[0] = (BQ769X0_I2C_ADDR << 1) + 1;  /* 读操作地址字节 */
	crcInput[1] = readBuffer[0];                    /* 数据字节 */
	crcValue = CRC8(crcInput, 2, CRC_KEY);

	if (crcValue == readBuffer[1])
	{
		return 1;
	}
	else
	{
		BQ769X0_RUNTIME_PRINTF("[BQ] read reg=0x%02X data=0x%02X crc=failed (expect=0x%02X got=0x%02X)\r\n",
		       reg_addr, readBuffer[0], crcValue, readBuffer[1]);
		return 0;
	}
}


/* 完成 BQ 初始化；任一步通信或配置验证失败时返回 false。 */
bool BQ769X0_Initialize(const BQ769X0_InitDataTypedef *InitData)
{
	if (InitData == NULL)
	{
		return false;
	}

	/* 步骤1~3: 复位并唤醒BQ芯片 */
	if (!BQ769X0_EntryShip())
	{
		return false;
	}
	BQ769X0_DELAY(500);
	BQ769X0_Wakeup();

	/* 步骤4: 读取ADC校准每个芯片不同,出厂时写*/
	if (!BQ769X0_GetADCGainOffset())
	{
		return false;
	}

	/* 步骤5: 保存报警回调(上层BMS注册的处理函*/
	AlertOps = InitData->AlertOps;

	/* 步骤6: 配置保护参数 */
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect1.Protect1Bit.SCD_DELAY = InitData->ConfigData.SCDDelay;
	Registers.Protect2.Protect2Bit.OCD_DELAY = InitData->ConfigData.OCDDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = InitData->ConfigData.UVDelay;
	Registers.Protect3.Protect3Bit.OV_DELAY = InitData->ConfigData.OVDelay;

	/* 步骤7: 计算OV/UV阈值寄存器*/
	/* OV阈14位寄存器格式 [10-XXXX-XXXX-1000], 取中*/
	Registers.OVTrip = (uint8_t)((((uint16_t)((InitData->ConfigData.OVPThreshold - Adcoffset) / Gain) - OV_THRESH_BASE) >> 4) & 0xFF);
	/* UV阈14位寄存器格式 [01-XXXX-XXXX-0000], 取中*/
	Registers.UVTrip = (uint8_t)((((uint16_t)((InitData->ConfigData.UVPThreshold - Adcoffset) / Gain) - UV_THRESH_BASE) >> 4) & 0xFF);

	/* 步骤8: 写入配置并验*/
	if (!BQ769X0_Configuration())
	{
		return false;
	}

	BQ769X0_INFO("BQ769X0 Initialize successful!");
	return true;
}

/* ==========================================================================
 * 通用安全关闭：断开 CHG/DSG 并清零均衡
 * ========================================================================== */

/* 有界重试关闭充放电 MOS 并清零均衡，回读全部正确时返回 true。 */
bool BQ769X0_ForceSafeOff(uint8_t retry_count)
{
	static const uint8_t safe_ctrl2 = 0x40U;
	static const uint8_t balance_off[3] = {0U, 0U, 0U};
	uint8_t ctrl2_read = 0;
	uint8_t balance_read[3] = {0};
	uint8_t attempt;
	bool result = false;

	if (retry_count == 0U)
	{
		return false;
	}

	BQ769X0_BusLock();
	for (attempt = 0U; attempt < retry_count; attempt++)
	{
		if (BQ769X0_WriteRegisterByteWithCRC(SYS_CTRL2, safe_ctrl2) &&
		    BQ769X0_WriteBlockWithCRC(CELLBAL1, balance_off, 3U) &&
		    BQ769X0_ReadRegisterByteWithCRC(SYS_CTRL2, &ctrl2_read) &&
		    BQ769X0_ReadBlockWithCRC(CELLBAL1, balance_read, 3U) &&
		    ((ctrl2_read & (CHG_CONTROL | DSG_CONTROL)) == 0U) &&
		    (memcmp(balance_read, balance_off, sizeof(balance_off)) == 0))
		{
			Registers.SysCtrl2.SysCtrl2Byte = ctrl2_read;
			memcpy(s_cell_balance_value, balance_off, sizeof(s_cell_balance_value));
			result = true;
			break;
		}
	}
	BQ769X0_BusUnlock();

	return result;
}

/* ==========================================================================
 * BQ769X0 采样参数获取 API (对外公开)
 * ========================================================================== */

uint16_t BQ769X0_GetCellVoltageMv(uint8_t index)
{
	if (index >= BQ769X0_CELL_MAX)
	{
		return 0;
	}
	return (uint16_t)(BQ769X0_SampleData.CellVoltage[index] * 1000.0f);
}

uint16_t BQ769X0_GetPackVoltageMv(void)
{
	return (uint16_t)(BQ769X0_SampleData.BatteryVoltage * 1000.0f);
}

int32_t BQ769X0_GetCurrentMa(void)
{
	return (int32_t)(BQ769X0_SampleData.BatteryCurrent * 1000.0f);
}

int16_t BQ769X0_GetTemperatureC(uint8_t index)
{
	if (index >= BQ769X0_TMEP_MAX)
	{
		return 0;
	}
	return (int16_t)(BQ769X0_SampleData.TsxTemperature[index]);
}
