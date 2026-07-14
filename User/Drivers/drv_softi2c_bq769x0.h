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
#ifndef DRV_SOFTI2C_BQ769X0_H
#define DRV_SOFTI2C_BQ769X0_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"


/*
 * BQ769X0 I2C从机地址(7位)
 *
 * BQ76920/30/40的默认I2C地址x08(7位)
 * 这个地址由芯片的CFETOFF引脚和化学ID决定
 * 如果你的电路中CFETOFF引脚被拉地址可能不同,请查数据手册
 */
#define BQ769X0_I2C_ADDR	0x08

/*
 * BQ769X0 CRC8多项* BQ769X0系列芯片使用CRC8校验保护I2C通信
 * 多项式为 x^8 + x^2 + x + 1 = 0x07
 * 每次读写寄存器都需要带上CRC校验字节
 */
#define CRC_KEY 0x07

/* 6位数据的位和*/
#define LOW_BYTE(Data)			(uint8_t)(0XFF & Data)
#define HIGH_BYTE(Data)			(uint8_t)(0XFF & (Data >> 8))

/*
 * BQ769X0毫秒延时* 原RT-Thread版本使用rt_thread_mdelay(),现改为CMSIS-RTOS2的osDelay()
 * osDelay()会让出CPU给其他任而不是忙等待
 * 用于: 唤醒延时(1000ms)、温度采样模式切换等2000ms)、初始化延时(500ms)
 */
#define BQ769X0_DELAY(ms)		osDelay(ms)


/*==========================================================================
 * 引脚配置
 *
 * 命名规则: BQ769X0_功能_GPIO_Port / BQ769X0_功能_Pin
 * 与HAL库的 GPIOx / GPIO_PIN_x 风格一*
 * TS1引脚(PA15) - 唤醒BQ芯片:
 *   BQ769X0在Ship模式(低功需要在TS1引脚上产生一个高电平脉冲唤醒
 *   唤醒后TS1引脚切回输入模式,用于温度采样(NTC热敏电阻)
 *   注意: PA15默认是JTDI调试引脚,需要在CubeMX中释*
 * ALERT引脚(PB12) - BQ芯片告警输出:
 *   BQ769X0检测到异常(OCD/SCD/OV/UVALERT引脚产生上升*   MCU通过外部中断(EXTI)捕获这个边沿,然后读取SYS_STAT寄存器确认具体告*========================================================================*/

/* BQ769X0唤醒引脚 - PA15 (需要在CubeMX中释放JTDI复用) */
#define BQ769X0_TS1_GPIO_Port	GPIOA
#define BQ769X0_TS1_Pin		GPIO_PIN_15

/* BQ769X0告警引脚 - PB12 (CubeMX中已配置为外部上升沿中断) */
#define BQ769X0_ALERT_GPIO_Port	GPIOB
#define BQ769X0_ALERT_Pin		GPIO_PIN_12
#define BQ769X0_ALERT_EXIT_IRQ	EXTI15_10_IRQn


/*==========================================================================
 * 电芯数量和温度通道配置
 *
 * BQ76920: 3~5串电1路温适合小型电池串电动工
 * BQ76930: 6~10串电2路温* BQ76940: 9~15串电3路温*
 * 这些宏用#ifndef包裹,可以在Keil预编译宏或上层头文件中覆* 例如在Keil的C/C++ -> Define中添BQ769X0_CELL_MAX=10
 *
 * BQ769X0_CELL_MAX - 支持的电芯串联数
 * BQ769X0_TMEP_MAX - 支持的温度通道注意:原作者拼写为TMEP,保持兼容)
 *========================================================================*/
#ifndef BQ769X0_CELL_MAX
#define BQ769X0_CELL_MAX 	5		/* BQ76920默认5串) */
#endif

#ifndef BQ769X0_TMEP_MAX
#define BQ769X0_TMEP_MAX	1		/* BQ76920默认1路温*/
#endif


/*==========================================================================
 * BQ769X0调试日志系统
 *
 * 与软件I2C的调试系统类使用printf输出
 * 需要将printf重定向到UART才能看到输出
 *
 * 等级0 - 全部关闭(默认,正式运行推荐)
 * 等级1 - 仅ERROR,用于定位I2C通信失败等严重问* 等级2 - 带文件名/函数行号,用于精确定位问题位置
 *========================================================================*/
#define BQ769X0_DEBUG_LEVEL	0


#if (BQ769X0_DEBUG_LEVEL == 0)

/* 等级0: 全部关闭,编译器会优化掉这些空*/
#define BQ769X0_ERROR(...)		do{}while(0)
#define BQ769X0_WARNING(...)	do{}while(0)
#define BQ769X0_INFO(...)		do{}while(0)

#elif (BQ769X0_DEBUG_LEVEL == 1)

/* 等级1: 仅显示错误信适合调试I2C通信问题 */
#define BQ769X0_ERROR(fmt, ...)                              \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 ERROR]");                      \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)
#define BQ769X0_WARNING(fmt, ...)                            \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 WARNING]");                    \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)
#define BQ769X0_INFO(fmt, ...)                               \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 INFO]");                       \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)


#elif (BQ769X0_DEBUG_LEVEL == 2)

/* 等级2: 带源码位置信适合精确定位问题 */
#define BQ769X0_ERROR(fmt, ...)                              \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 ERROR][%s:%s:%d] ",            \
                    __FILE__, __FUNCTION__, __LINE__);      \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)
#define BQ769X0_WARNING(fmt, ...)                            \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 WARNING][%s:%s:%d] ",          \
                    __FILE__, __FUNCTION__, __LINE__);      \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)
#define BQ769X0_INFO(fmt, ...)                               \
		do                                                    \
        {                                                   \
            printf("[BQ769X0 INFO][%s:%s:%d] ",             \
                    __FILE__, __FUNCTION__, __LINE__);      \
            printf(fmt"\r\n", ##__VA_ARGS__);               \
        } while(0)

#endif


/*==========================================================================
 * BQ769X0寄存器地址定义
 *
 * BQ769X0的寄存器分为以下几组:
 *   系统控制: SYS_STAT(0x00), SYS_CTRL1(0x04), SYS_CTRL2(0x05)
 *   均衡控制: CELLBAL1~3(0x01~0x03)
 *   保护配置: PROTECT1~3(0x06~0x08), OV_TRIP(0x09), UV_TRIP(0x0A)
 *   电压数据: VC1~VC15(从 0x0C开每节2字节), BAT(0x2A)
 *   温度数据: TS1(0x2C), TS2(0x2E)
 *   电流数据: CC(0x32~0x33,库仑
 *   校准数据: ADCGAIN1(0x50), ADCOFFSET(0x51), ADCGAIN2(0x59)
 *========================================================================*/

#define SYS_STAT 				0x00   /* 系统状态寄存器(告警标志*/
#define CELLBAL1 				0x01   /* 电芯均衡控制1(1~5号电*/
#define CELLBAL2 				0x02   /* 电芯均衡控制2(6~10号电*/
#define CELLBAL3 				0x03   /* 电芯均衡控制3(11~15号电*/
#define SYS_CTRL1 				0x04   /* 系统控制1(ADC使能、温度选择*/
#define SYS_CTRL2 				0x05   /* 系统控制2(CHG/DSG开关、库仑计*/
#define PROTECT1 				0x06   /* 保护配置1(SCD阈值和延时) */
#define PROTECT2 				0x07   /* 保护配置2(OCD阈值和延时) */
#define PROTECT3 				0x08   /* 保护配置3(OV/UV延时) */
#define OV_TRIP 				0x09   /* 过压保护阈*/
#define UV_TRIP 				0x0A   /* 欠压保护阈*/
#define CC_CFG  				0x0B   /* 库仑计配必须x19) */

/* 数据寄存只读) */
#define VC1_HI_BYTE	 			0x0C   /* 电芯1电压高字从这里连续读取所有电*/
#define CC_HI_BYTE				0x32   /* 库仑计高字节(电流采样*/
#define CC_LO_BYTE				0x33   /* 库仑计低字节 */
#define BAT_HI_BYTE				0x2A   /* 电池总电压高字节 */
#define TS1_HI_BYTE				0x2C   /* 温度传感高字*/
#define TS2_HI_BYTE				0x2E   /* 温度传感高字*/

/* 校准寄存只读) */
#define ADCGAIN1 				0x50   /* ADC增益寄存*/
#define ADCOFFSET 				0x51   /* ADC偏移寄存*/
#define ADCGAIN2 				0x59   /* ADC增益寄存*/


/*==========================================================================
 * 保护阈值定*
 * SCD(Short Circuit Detection) - 放电短路保护:
 *   当放电电流超过阈值时触发,阈值单位mV(分流电阻上的压降)
 *   例如: SCD_THRESH_89mV_44mV 表示RSNS=0时阈4mV,RSNS=1时阈9mV
 *   实际保护电流 = 阈mV) / 分流电阻(mΩ)
 *    44mV / 5mΩ = 8.8A
 *
 * OCD(Over Current Detection) - 放电过流保护:
 *   当放电电流持续超过阈值且超过延时时间时触*   例如: OCD_THRESH_22mV_11mV 表示RSNS=0时阈1mV
 *   实际保护电流 = 11mV / 5mΩ = 2.2A
 *========================================================================*/

#define SCD_THRESH_44mV_22mV	0x00
#define SCD_THRESH_67mV_33mV	0x01
#define SCD_THRESH_89mV_44mV	0x02
#define SCD_THRESH_111mV_56mV	0x03
#define SCD_THRESH_133mV_67mV	0x04
#define SCD_TRHESH_155mV_68mV	0x05
#define SCD_THRESH_178mV_89mV	0x06
#define SCD_THRESH_200mV_100mV	0x07

#define OCD_THRESH_17mV_8mV		0x00
#define OCD_THRESH_22mV_11mV	0x01
#define OCD_THRESH_28mV_14mV	0x02
#define OCD_THRESH_33mV_17mV	0x03
#define OCD_THRESH_39mV_19mV	0x04
#define OCD_THRESH_44mV_22mV	0x05
#define OCD_THRESH_50mV_25mV	0x06
#define OCD_THRESH_56mV_28MV	0x07
#define OCD_THRESH_61mV_31mV	0x08
#define OCD_THRESH_67mV_33mV	0x09
#define OCD_THRESH_72mV_36mV	0x0A
#define OCD_THRESH_78mV_39mV	0x0B
#define OCD_THRESH_83mV_42mV	0x0C
#define OCD_THRESH_89mV_44mV	0x0D
#define OCD_THRESH_94mV_47mV	0x0E
#define OCD_THRESH_100mV_50mV	0x0F


/*==========================================================================
 * OV/UV阈值计算常*
 * BQ769X0的OV/UV阈值寄存器位的,但实际比较是14位精* 写入格式: [10-XXXX-XXXX-1000] [01-XXXX-XXXX-0000]
 * OV_THRESH_BASE = 0x2008, UV_THRESH_BASE = 0x1000
 * OV/UV_STEP = 0x10 (每步对应一个ADC LSB)
 *
 * 计算公式:
 *   OV寄存器= ((OVP_mV / GAIN) - OV_THRESH_BASE) >> 4
 *   UV寄存器= ((UVP_mV / GAIN) - UV_THRESH_BASE) >> 4
 *========================================================================*/

#define OV_THRESH_BASE			0x2008
#define UV_THRESH_BASE			0x1000
#define OV_STEP					0x10
#define UV_STEP					0x10

/* ADCGAIN基准uV/LSB),实际增益需要加上寄存器中的修正*/
#define ADCGAIN_BASE			365


/*==========================================================================
 * SYS_STAT寄存器状态位定义
 *
 * SYS_STAT(0x00)是只读寄存器,记录BQ芯片检测到的各类告
 *   OCD   - 放电过流(Over Current Discharge)
 *   SCD   - 放电短路(Short Circuit Discharge)
 *   OV    - 过压(Over Voltage)
 *   UV    - 欠压(Under Voltage)
 *   OVRD  - ALERT引脚被外部电路强制触*   DEVICE - 芯片内部故障
 *   CC    - 库仑计采样完*
 * 写入1到对应位可以清除该告警标清零)
 *========================================================================*/

#define SYS_STAT_OCD_BIT		0X01   /* bit0: 放电过流 */
#define SYS_STAT_SCD_BIT		0X02   /* bit1: 放电短路 */
#define SYS_STAT_OV_BIT			0X04   /* bit2: 过压 */
#define SYS_STAT_UV_BIT			0X08   /* bit3: 欠压 */
#define SYS_STAT_OVRD_BIT		0X10   /* bit4: ALERT被外部覆*/
#define SYS_STAT_DEVICE_BIT		0X20   /* bit5: 设备故障 */
#define SYS_STAT_CC_BIT			0X80   /* bit7: 库仑计就*/


/*==========================================================================
 * 寄存器组结构- BQ769X0所有寄存器的内存镜*
 * 使用union+位域的方可以按字节访问也可以按位访问同一个寄存器
 * 例如: Registers.SysCtrl2.SysCtrl2Byte 可以直接写字节*       Registers.SysCtrl2.SysCtrl2Bit.CHG_ON 可以单独操作充电开关位
 *
 * 这个结构体在MCU侧维护一份寄存器镜像,读写BQ芯片时先修改镜像再批量写*========================================================================*/
typedef struct _Register_Group
{
	/* SYS_STAT(0x00) - 系统状只读,清零) */
	union
	{
		struct
		{
			uint8_t OCD				:1;  /* 放电过流标志 */
			uint8_t SCD				:1;  /* 放电短路标志 */
			uint8_t OV				:1;  /* 过压标志 */
			uint8_t UV				:1;  /* 欠压标志 */
			uint8_t OVRD_ALERT		:1;  /* ALERT被外部覆*/
			uint8_t DEVICE_XREADY	:1;  /* 设备故障 */
			uint8_t WAKE			:1;  /* 唤醒标志 */
			uint8_t CC_READY		:1;  /* 库仑计就*/
		}StatusBit;
		uint8_t StatusByte;
	}SysStatus;

	/* CELLBAL1(0x01) - 电芯均衡控制1(1~5号电*/
	union
	{
		struct
		{
			uint8_t RSVD		:3;  /* 保留*/
			uint8_t CB5			:1;  /* 5号电芯均衡开*/
			uint8_t CB4			:1;  /* 4号电芯均衡开*/
			uint8_t CB3			:1;  /* 3号电芯均衡开*/
			uint8_t CB2			:1;  /* 2号电芯均衡开*/
			uint8_t CB1			:1;  /* 1号电芯均衡开*/
		}CellBal1Bit;
		uint8_t CellBal1Byte;
	}CellBal1;

	/* CELLBAL2(0x02) - 电芯均衡控制2(6~10号电*/
	union
	{
		struct
		{
			uint8_t RSVD		:3;
			uint8_t CB10		:1;
			uint8_t CB9			:1;
			uint8_t CB8			:1;
			uint8_t CB7			:1;
			uint8_t CB6			:1;
		}CellBal2Bit;
		uint8_t CellBal2Byte;
	}CellBal2;

	/* CELLBAL3(0x03) - 电芯均衡控制3(11~15号电*/
	union
	{
		struct
		{
			uint8_t RSVD			:3;
			uint8_t CB15			:1;
			uint8_t CB14			:1;
			uint8_t CB13			:1;
			uint8_t CB12			:1;
			uint8_t CB11			:1;
		}CellBal3Bit;
		uint8_t CellBal3Byte;
	}CellBal3;

	/* SYS_CTRL1(0x04) - 系统控制1 */
	union
	{
		struct
		{
			uint8_t SHUT_B			:1;  /* 关机控制B */
			uint8_t SHUT_A			:1;  /* 关机控制A */
			uint8_t RSVD1			:1;
			uint8_t TEMP_SEL		:1;  /* 温度选择: 0=外部NTC, 1=内部die */
			uint8_t ADC_EN			:1;  /* ADC使能 */
			uint8_t RSVD2			:2;
			uint8_t LOAD_PRESENT	:1;  /* 负载检测标只读) */
		}SysCtrl1Bit;
		uint8_t SysCtrl1Byte;
	}SysCtrl1;

	/* SYS_CTRL2(0x05) - 系统控制2 */
	union
	{
		struct
		{
			uint8_t CHG_ON			:1;  /* 充电MOS开1=开, 0=否*/
			uint8_t DSG_ON			:1;  /* 放电MOS开1=开, 0=否*/
			uint8_t WAKE_T			:2;  /* 唤醒定时器配*/
			uint8_t WAKE_EN			:1;  /* 唤醒使能 */
			uint8_t CC_ONESHOT		:1;  /* 库仑计单次采*/
			uint8_t CC_EN			:1;  /* 库仑计连续采样使*/
			uint8_t DELAY_DIS		:1;  /* 禁用保护延时 */
		}SysCtrl2Bit;
		uint8_t SysCtrl2Byte;
	}SysCtrl2;

	/* PROTECT1(0x06) - 保护配置1(SCD) */
	union
	{
		struct
		{
			uint8_t SCD_THRESH		:3;  /* SCD阈8*/
			uint8_t SCD_DELAY		:2;  /* SCD延时(4*/
			uint8_t RSVD			:2;
			uint8_t RSNS			:1;  /* 分流电阻倍率: 0=不加1=加*/
		}Protect1Bit;
		uint8_t Protect1Byte;
	}Protect1;

	/* PROTECT2(0x07) - 保护配置2(OCD) */
	union
	{
		struct
		{
			uint8_t OCD_THRESH		:4;  /* OCD阈16*/
			uint8_t OCD_DELAY		:3;  /* OCD延时(8*/
			uint8_t RSVD			:1;
		}Protect2Bit;
		uint8_t Protect2Byte;
	}Protect2;

	/* PROTECT3(0x08) - 保护配置3(OV/UV延时) */
	union
	{
		struct
		{
			uint8_t RSVD			:4;
			uint8_t OV_DELAY		:2;  /* 过压延时(4位) */
			uint8_t UV_DELAY		:2;  /* 欠压延时(4位) */
		}Protect3Bit;
		uint8_t Protect3Byte;
	}Protect3;

	uint8_t OVTrip;   /* OV_TRIP(0x09) - 过压阈*/
	uint8_t UVTrip;   /* UV_TRIP(0x0A) - 欠压阈*/
	uint8_t CCCfg;    /* CC_CFG(0x0B) - 库仑计配必须x19 */

	/* 电芯电压寄存VC1~VC15, x0C开始连续排*/
	/* 每节电芯字节(高字低字, 14位ADC) */
	union
	{
		struct
		{
			uint8_t VC1_HI;
			uint8_t VC1_LO;
		}VCell1Byte;
		uint16_t VCell1Word;
	}VCell1;

	union
	{
		struct
		{
			uint8_t VC2_HI;
			uint8_t VC2_LO;
		}VCell2Byte;
		uint16_t VCell2Word;
	}VCell2;

	union
	{
		struct
		{
			uint8_t VC3_HI;
			uint8_t VC3_LO;
		}VCell3Byte;
		uint16_t VCell3Word;
	}VCell3;

	union
	{
		struct
		{
			uint8_t VC4_HI;
			uint8_t VC4_LO;
		}VCell4Byte;
		uint16_t VCell4Word;
	}VCell4;

	union
	{
		struct
		{
			uint8_t VC5_HI;
			uint8_t VC5_LO;
		}VCell5Byte;
		uint16_t VCell5Word;
	}VCell5;

	union
	{
		struct
		{
			uint8_t VC6_HI;
			uint8_t VC6_LO;
		}VCell6Byte;
		uint16_t VCell6Word;
	}VCell6;

	union
	{
		struct
		{
			uint8_t VC7_HI;
			uint8_t VC7_LO;
		}VCell7Byte;
		uint16_t VCell7Word;
	}VCell7;

	union
	{
		struct
		{
			uint8_t VC8_HI;
			uint8_t VC8_LO;
		}VCell8Byte;
		uint16_t VCell8Word;
	}VCell8;

	union
	{
		struct
		{
			uint8_t VC9_HI;
			uint8_t VC9_LO;
		}VCell9Byte;
		uint16_t VCell9Word;
	}VCell9;

	union
	{
		struct
		{
			uint8_t VC10_HI;
			uint8_t VC10_LO;
		}VCell10Byte;
		uint16_t VCell10Word;
	}VCell10;

	union
	{
		struct
		{
			uint8_t VC11_HI;
			uint8_t VC11_LO;
		}VCell11Byte;
		uint16_t VCell11Word;
	}VCell11;

	union
	{
		struct
		{
			uint8_t VC12_HI;
			uint8_t VC12_LO;
		}VCell12Byte;
		uint16_t VCell12Word;
	}VCell12;

	union
	{
		struct
		{
			uint8_t VC13_HI;
			uint8_t VC13_LO;
		}VCell13Byte;
		uint16_t VCell13Word;
	}VCell13;

	union
	{
		struct
		{
			uint8_t VC14_HI;
			uint8_t VC14_LO;
		}VCell14Byte;
		uint16_t VCell14Word;
	}VCell14;

	union
	{
		struct
		{
			uint8_t VC15_HI;
			uint8_t VC15_LO;
		}VCell15Byte;
		uint16_t VCell15Word;
	}VCell15;

	/* BAT(0x2A) - 电池总电*/
	union
	{
		struct
		{
			uint8_t BAT_HI;
			uint8_t BAT_LO;
		}VBatByte;
		uint16_t VBatWord;
	}VBat;

	/* TS1(0x2C) - 温度传感(热敏电阻或内部die温度) */
	union
	{
		struct
		{
			uint8_t TS1_HI;
			uint8_t TS1_LO;
		}TS1Byte;
		uint16_t TS1Word;
	}TS1;

	/* TS2(0x2E) - 温度传感(BQ76930/40才有) */
	union
	{
		struct
		{
			uint8_t TS2_HI;
			uint8_t TS2_LO;
		}TS2Byte;
		uint16_t TS2Word;
	}TS2;

	/* TS3 - 温度传感(BQ76940才有) */
	union
	{
		struct
		{
			uint8_t TS3_HI;
			uint8_t TS3_LO;
		}TS3Byte;
		uint16_t TS3Word;
	}TS3;

	/* CC(0x32) - 库仑计采样16位有符号,补码) */
	union
	{
		struct
		{
			uint8_t CC_HI;
			uint8_t CC_LO;
		}CCByte;
		uint16_t CCWord;
	}CC;

	/* ADCGAIN1(0x50) - ADC增益寄存*/
	union
	{
		struct
		{
			uint8_t RSVD1			:2;
			uint8_t ADCGAIN_4_3		:2;  /* 增益的bit4~bit3 */
			uint8_t RSVD2			:4;
		}ADCGain1Bit;
		uint8_t ADCGain1Byte;
	}ADCGain1;

	uint8_t ADCOffset;  /* ADCOFFSET(0x51) - ADC偏移有符*/

	/* ADCGAIN2(0x59) - ADC增益寄存*/
	union
	{
		struct
		{
			uint8_t RSVD			:5;
			uint8_t ADCGAIN_2_0		:3;  /* 增益的bit2~bit0 */
		}ADCGain2Bit;
		uint8_t ADCGain2Byte;
	}ADCGain2;

}RegisterGroup;


/*==========================================================================
 * 保护延时枚举类型
 *
 * 每种保护都有多档延时可延时越长抗干扰能力越强但响应越慢
 *========================================================================*/

/* SCD(短路)延时: 50us~400us, 延时指超过阈值后多久触发保护 */
typedef enum
{
	BQ_SCD_DELAY_50us  = 0x00,
	BQ_SCD_DELAY_100us = 0x01,
	BQ_SCD_DELAY_200us = 0x02,
	BQ_SCD_DELAY_400us = 0x03,
}BQ769X0_SCDDelayTypedef;

/* OCD(过流)延时: 10ms~1280ms */
typedef enum
{
	BQ_OCD_DEALY_10ms	= 0x00,
	BQ_OCD_DELAY_20ms	= 0x01,
	BQ_OCD_DELAY_40ms	= 0x02,
	BQ_OCD_DELAY_80ms	= 0x03,
	BQ_OCD_DELAY_160ms	= 0x04,
	BQ_OCD_DELAY_320ms	= 0x05,
	BQ_OCD_DELAY_640ms	= 0x06,
	BQ_OCD_DELAY_1280ms	= 0x07,
}BQ769X0_OCDDelayTypedef;


/* OV(过压)延时: 1s~8s */
typedef enum
{
	BQ_OV_DELAY_1s	= 0x00,
	BQ_OV_DELAY_2s	= 0x01,
	BQ_OV_DELAY_4s	= 0x02,
	BQ_OV_DELAY_8s	= 0x03,
}BQ769X0_OVDelayTypedef;

/* UV(欠压)延时: 1s~16s */
typedef enum
{
	BQ_UV_DELAY_1s	= 0x00,
	BQ_UV_DELAY_4s	= 0x01,
	BQ_UV_DELAY_8s	= 0x02,
	BQ_UV_DELAY_16s	= 0x03,
}BQ769X0_UVDelayTypedef;


/*==========================================================================
 * 配置数据结构- 用于BQ769X0_Initialize()的参*
 * 包含所有可配置的保护参
 *   SCDDelay/OCDDelay - 短路/过流延时
 *   UVDelay/OVDelay   - 欠压/过压延时
 *   UVPThreshold/OVPThreshold - 欠压/过压阈mV)
 *========================================================================*/
typedef struct
{
	BQ769X0_SCDDelayTypedef SCDDelay;   /* 短路保护延时 */
	BQ769X0_OCDDelayTypedef OCDDelay;   /* 过流保护延时 */
	BQ769X0_UVDelayTypedef UVDelay;     /* 欠压保护延时 */
	BQ769X0_OVDelayTypedef OVDelay;     /* 过压保护延时 */
	uint16_t UVPThreshold;               /* 欠压阈值(mV), 默认为 900mV */
	uint16_t OVPThreshold;               /* 过压阈值(mV), 默认为 200mV */
}BQ769X0_ConfigDataTypedef;


/*
 * BQ硬件报警回调接口
 *
 * 当BQ769X0检测到异常ALERT引脚产生上升* MCU读取SYS_STAT寄存器后,根据置位的标志调用对应的回调函数
 *
 * 这些回调函数由上层BMS业务注册,例如:
 *   AlertOps.ocd = BMS_OCD_Handler;  // 过流时执行保护动*   AlertOps.ov  = BMS_OV_Handler;   // 过压时断开充电MOS
 *
 * 所有回调都可以传NULL(本阶段不实现完整保护策略)
 * 回调在任务上下文中执不在ISR中,所以可以安全地操作I2C
 */
typedef struct
{
	void (*ocd)(void);		/* 放电过流回调 */
	void (*scd)(void);		/* 放电短路回调 */
	void (*ov)(void);		/* 充电过压回调 */
	void (*uv)(void);		/* 放电欠压回调 */
	void (*ovrd)(void);		/* ALERT被外部覆盖回*/
	void (*device)(void);	/* 设备故障回调 */
	void (*cc)(void);		/* 库仑计采样完成回*/
}BQ769X0_AlertOpsTypedef;

/*
 * 初始化数据结构体 - BQ769X0_Initialize()的参*
 * 包含两部
 *   AlertOps  - 报警回调函数(可以全部传NULL)
 *   ConfigData - 保护配置参数
 */
typedef struct
{
	BQ769X0_AlertOpsTypedef AlertOps;    /* 报警回调 */
	BQ769X0_ConfigDataTypedef ConfigData; /* 保护配置 */
}BQ769X0_InitDataTypedef;


/*
 * 充放电控制类* 用于BQ769X0_ControlDSGOrCHG()函数,控制SYS_CTRL2寄存器中的MOS开*/
typedef enum
{
	CHG_CONTROL = 0x01,  /* 充电MOS控制(SYS_CTRL2.bit0) */
	DSG_CONTROL = 0x02   /* 放电MOS控制(SYS_CTRL2.bit1) */
}BQ769X0_ControlTypedef;


/* 状态枚使能/禁用 */
typedef enum
{
	BQ_STATE_ENABLE,
	BQ_STATE_DISABLE
}BQ769X0_StateTypedef;


/*
 * 电芯均衡位图 - 每个bit对应一节物理电*
 * 使用位图而非数组下标,是因为BQ769X0的CELLBAL寄存器本身就是位图格
 *   CELLBAL1(0x01): bit0=1号电bit1=2号 ..., bit4=5号 *   CELLBAL2(0x02): bit0=6号电bit1=7 ..., bit4=10s*   CELLBAL3(0x03): bit0=11号电bit1=12 ..., bit4=15 *
 * 使用示例:
 *   BQ769X0_CellBalanceControl(BQ_CELL_INDEX1 | BQ_CELL_INDEX3, BQ_STATE_ENABLE);
 *   // 同时开号和3号电芯的均衡
 */
typedef enum
{
	BQ_CELL_INDEX1  = 0x0001,  /* bit0: 1号电*/
	BQ_CELL_INDEX2  = 0x0002,  /* bit1: 2号电*/
	BQ_CELL_INDEX3  = 0x0004,  /* bit2: 3号电*/
	BQ_CELL_INDEX4  = 0x0008,  /* bit3: 4号电*/
	BQ_CELL_INDEX5  = 0x0010,  /* bit4: 5号电*/
	BQ_CELL_INDEX6  = 0x0020,  /* bit5: 6号电*/
	BQ_CELL_INDEX7  = 0x0040,  /* bit6: 7号电*/
	BQ_CELL_INDEX8  = 0x0080,  /* bit7: 8号电*/
	BQ_CELL_INDEX9  = 0x0100,  /* bit8: 9号电*/
	BQ_CELL_INDEX10 = 0x0200,  /* bit9: 10号电*/
	BQ_CELL_INDEX11 = 0x0400,  /* bit10: 11号电*/
	BQ_CELL_INDEX12 = 0x0800,  /* bit11: 12号电*/
	BQ_CELL_INDEX13 = 0x1000,  /* bit12: 13号电*/
	BQ_CELL_INDEX14 = 0x2000,  /* bit13: 14号电*/
	BQ_CELL_INDEX15 = 0x4000,  /* bit14: 15号电*/
	BQ_CELL_ALL		= 0x7FFF,  /* 全部电芯 */
}BQ769X0_CellIndexTypedef;


/*==========================================================================
 * 采样数据结构- 存放BQ769X0的实时采样结*
 * 由周期性调用的Update函数填充:
 *   UpdateCellVolt()  -> CellVoltage[]  (每 250ms)
 *   UpdateCurrent()   -> BatteryCurrent (每 250ms)
 *   UpadteBatVolt()   -> BatteryVoltage (每 250ms)
 *   UpdateTsTemp()    -> TsxTemperature[] (每 2s)
 *   UpdateDieTemp()   -> DieTemperature ((每 2s, 未验证
 *
 * 所有电压单V(伏特)
 * 电流单位: A(安培)
 * 温度单位: °C(摄氏
 *========================================================================*/
typedef struct
{
	float CellVoltage[BQ769X0_CELL_MAX];		/* 单节电芯电压数组(单位V) */
	float TsxTemperature[BQ769X0_TMEP_MAX];	/* 温度传感器数单位°C) */
	float BatteryCurrent;	/* 电池包总电流(单位A, 正为充电, 负为放电) */
	float BatteryVoltage;	/* 电池包总电单位V) */
	float DieTemperature;	/* BQ芯片内部温度(单位°C,未验*/
}BQ769X0_SampleDataTypedef;

/* 全局采样数据实例(在drv_softi2c_bq769x0.c中定*/
extern BQ769X0_SampleDataTypedef BQ769X0_SampleData;


/*==========================================================================
 * BQ769X0公开接口函数声明
 *========================================================================*/

/* 连续写入带 CRC 的寄存器数据块。 */
bool BQ769X0_WriteBlockWithCRC(uint8_t startAddress, const uint8_t *buffer, uint8_t length);

/* 读取一个带 CRC 的寄存器字节。 */
bool BQ769X0_ReadRegisterByteWithCRC(uint8_t Register, uint8_t *data);

/* 连续读取带 CRC 的寄存器数据块。 */
bool BQ769X0_ReadBlockWithCRC(uint8_t Register, uint8_t *buffer, uint8_t length);

/* 初始化 BQ769X0，并完成唤醒、校准参数读取和保护寄存器配置。 */
bool BQ769X0_Initialize(const BQ769X0_InitDataTypedef *InitData);

/*
 * 唤醒BQ芯片(从Ship模式)
 * 原理: 在TS1引脚输出高电平脉冲(至少1s), 然后切回输入模式
 */
/* 通过 TS1 引脚唤醒处于 Ship 模式的 BQ769X0。 */
void BQ769X0_Wakeup(void);

/*
 * 进入Ship模式(超低功
 * 功能: 按照数据手册的序列写入SYS_CTRL1寄存* 注意: 进入Ship后需要通过TS1引脚唤醒
 */
/* 按数据手册规定的写入序列使 BQ769X0 进入 Ship 模式。 */
bool BQ769X0_EntryShip(void);

/*
 * 负载检* 返回: true=检测到负载, false=未检测到
 * 条件: 充电MOS关闭(CHG_ON=0)且LOAD_PRESENT标志置位
 */
/* 检测充电 MOS 关闭时是否存在负载。 */
bool BQ769X0_LoadDetect(void);

/* Read LOAD_PRESENT with communication status; valid only with CHG off. */
bool BQ769X0_LoadPresentGet(bool *present);

/*
 * 控制充放电MOS开* 参数:
 *   ControlType - CHG_CONTROL(充电) 或 DSG_CONTROL(放电)
 *   NewState    - BQ_STATE_ENABLE(开) 或 BQ_STATE_DISABLE(关)
 */
/* 控制充电或放电 MOS，并返回寄存器写入及回读结果。 */
bool BQ769X0_ControlDSGOrCHG(BQ769X0_ControlTypedef ControlType, BQ769X0_StateTypedef NewState);

/*
 * 电芯均衡控制
 * 参数:
 *   CellIndex - 电芯位图(可同时指定多BQ_CELL_INDEX1 | BQ_CELL_INDEX3)
 *   NewState  - BQ_STATE_ENABLE(开启均BQ_STATE_DISABLE(关闭均衡)
 * 注意: 芯片不允许相邻电芯同时均上层需要做冲突过滤
 */
/* 控制指定电芯均衡，并通过回读确认控制结果。 */
bool BQ769X0_CellBalanceControl(BQ769X0_CellIndexTypedef CellIndex, BQ769X0_StateTypedef NewState);

/* 更新全部电芯电压，通信失败时保留旧值并返回 false。 */
bool BQ769X0_UpdateCellVolt(void);

/* 更新外部温度传感器数据，通信或数据异常时返回 false。 */
bool BQ769X0_UpdateTsTemp(void);

/* 更新芯片内部温度。 */
void BQ769X0_UpdateDieTemp(void);

/* 更新电池电流，通信失败时保留旧值并返回 false。 */
bool BQ769X0_UpdateCurrent(void);

/* 更新电池包总电压，通信失败时保留旧值并返回 false。 */
bool BQ769X0_UpadteBatVolt(void);

/* 设置硬件放电短路保护延时。 */
void BQ769X0_SCDDelaySet(BQ769X0_SCDDelayTypedef SCDDelay);

/* 设置硬件放电过流保护延时。 */
void BQ769X0_OCDDelaySet(BQ769X0_OCDDelayTypedef OCDDelay);

/* 设置硬件欠压保护延时。 */
void BQ769X0_UVDelaySet(BQ769X0_UVDelayTypedef UVDelay);

/* 设置硬件过压保护延时。 */
void BQ769X0_OVDelaySet(BQ769X0_OVDelayTypedef OVDelay);

/* 设置并回读验证硬件欠压阈值；不可表示或通信失败时返回 false。 */
bool BQ769X0_UVPThresholdSet(uint16_t UVPThreshold);

/* 设置并回读验证硬件过压阈值；不可表示或通信失败时返回 false。 */
bool BQ769X0_OVPThresholdSet(uint16_t OVPThreshold);

/* 获取硬件放电短路保护延时。 */
BQ769X0_SCDDelayTypedef BQ769X0_SCDDelayGet(void);

/* 获取硬件放电过流保护延时。 */
BQ769X0_OCDDelayTypedef BQ769X0_OCDDelayGet(void);

/* 获取硬件欠压保护延时。 */
BQ769X0_UVDelayTypedef BQ769X0_UVDelayGet(void);

/* 获取硬件过压保护延时。 */
BQ769X0_OVDelayTypedef BQ769X0_OVDelayGet(void);

/* 获取硬件欠压保护阈值，单位为毫伏。 */
uint16_t BQ769X0_UVPThresholdGet(void);

/* 获取硬件过压保护阈值，单位为毫伏。 */
uint16_t BQ769X0_OVPThresholdGet(void);


/* 从 EXTI ISR 向 ALERT 任务发送通知，不在中断中执行 I2C。 */
void BQ769X0_AlertNotifyFromISR(void);

/* 在任务上下文锁存并处理告警；DEVICE_XREADY 保留到受控重启。 */
bool BQ769X0_ProcessAlert(void);


/*
 * 寄存器CRC验证(调试- 读取单个寄存器并打印CRC校验结果
 *
 * 功能: 使用带CRC的方式读取指定寄存器,打印数据和CRC状* 用验证I2C通信链路是否正常,CRC校验是否通过
 *
 * 输出格式:
 *   [BQ] read reg=0x50 data=0xXX crc=ok       (CRC校验通过)
 *   [BQ] read reg=0x50 data=0xXX crc=failed    (CRC校验失败)
 *   [BQ] read reg=0x50 comm=failed             (I2C通信失败)
 *
 * 参数:
 *   reg_addr - 寄存器地址(如ADCGAIN1=0x50)
 *   reg_name - 寄存器名称字符串(用于打印, 如 "ADCGAIN1")
 *
 * 返回: 1=CRC通过, 0=CRC失败或通信失败
 *
 * 注意: 此函数仅用于调试验证,正式代码中不需要调*/
/* 调试读取指定寄存器并验证其 CRC。 */
uint8_t BQ769X0_VerifyRegisterCRC(uint8_t reg_addr, const char *reg_name);

/* 注册接收 ALERT 任务通知的 FreeRTOS 任务句柄。 */
void BQ769X0_RegisterAlertTask(TaskHandle_t task_handle);

/* 获取指定电芯的最新电压，单位为毫伏。 */
uint16_t BQ769X0_GetCellVoltageMv(uint8_t index);

/* 获取最新电池包总电压，单位为毫伏。 */
uint16_t BQ769X0_GetPackVoltageMv(void);

/* 获取最新电池电流，单位为毫安，正值表示充电。 */
int32_t BQ769X0_GetCurrentMa(void);

/* 获取指定温度通道的最新温度，单位为摄氏度。 */
int16_t BQ769X0_GetTemperatureC(uint8_t index);

/* 重试关闭 CHG、DSG 和全部均衡，并通过寄存器回读确认安全状态。 */
bool BQ769X0_ForceSafeOff(uint8_t retry_count);

/* 初始化 BQ 总线互斥锁，重复调用不会重复创建。 */
bool BQ769X0_BusLockInit(void);

/* 获取 BQ 总线互斥锁，用于保护一组连续寄存器操作。 */
void BQ769X0_BusLock(void);

/* 释放 BQ 总线互斥锁。 */
void BQ769X0_BusUnlock(void);

#endif
