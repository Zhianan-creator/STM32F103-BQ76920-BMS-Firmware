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

/*
 * 软件I2C驱动 - FreeRTOS适配版本
 *
 * 本文件实现了基于GPIO模拟的软件I2C主机驱动,适配FreeRTOS/CMSIS-RTOS2 *
 * 主要改动相比原RT-Thread版本):
 *   1. 互斥rt_hw_interrupt_disable/enable -> osMutexAcquire/osMutexRelease
 *      原版本通过关中断保护整个I2C事务,这在FreeRTOS中会导致系统心跳丢失*      任务调度延迟等问题。改为互斥锁多个任务可以安全地共享I2C总线*
 *   2. 微秒延时: 空循环延-> DWT CYCCNT硬件计数*      原版本的空循环延时依赖编译器优化等级,不精确。DWT是Cortex-M3内核*      硬件调试组件,其CYCCNT计数器以CPU主频递增(72MHz),可以实现精确*      一个时钟周期的延时*
 *   3. 约束: 软件I2C只能从任务上下文调用,禁止在ISR中调*      因为互斥锁和HAL_GPIO_Init都不能在中断中使用*
 * 硬件连接:
 *   SDA - PB13 (I2C1_SDA_Pin / I2C1_SDA_GPIO_Port, CubeMX main.h)
 *   SCL - PB14 (I2C1_SCL_Pin / I2C1_SCL_GPIO_Port, CubeMX main.h)
 *   两个引脚都配置为开漏输出模外部需要接上拉电阻(通常4.7K到VCC)
 */

#include "drv_soft_i2c.h"

#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"


/*==========================================================================
 * FreeRTOS互斥*
 * 为什么用互斥锁而不是信号量
 *   互斥锁有"优先级继机制:如果低优先级任务持有高优先级任务等待
 *   RTOS会临时提升低优先级任务的优先避免优先级反转问题*   二值信号量没有这个机制,在多任务场景下可能导致高优先级任务长时间阻塞*
 * 为什么用静态变量而不是放在I2C_BusTypeDef结构体里
 *   当前只有一条I2C总线(i2c1),用静态变量更简洁*   如果将来需要多条总线,可以把osMutexId_t加入结构体*========================================================================*/
static osMutexId_t i2c1_mutex = NULL;
static StaticSemaphore_t i2c1_mutex_buffer;
static const osMutexAttr_t i2c1_mutex_attributes = {
	.name = "i2c1Mutex",
	.cb_mem = &i2c1_mutex_buffer,
	.cb_size = sizeof(i2c1_mutex_buffer),
};


/*
 * 互斥锁初始化回调 - 保留接口兼容,实际创建在I2C_BusInitialize中完*/
static void I2C1_LockInit(void)
{
	/* 互斥锁在I2C_BusInitialize()中统一创建,此处为空实现 */
}

/*
 * 获取I2C总线* 调用后当前任务会阻塞直到获取到锁,或永久等osWaitForever)
 * 获取锁后,其他任务的I2C操作会被阻塞,保证总线访问的原子*/
static void I2C1_Lock(void)
{
	if (i2c1_mutex != NULL)
		osMutexAcquire(i2c1_mutex, osWaitForever);
}

/*
 * 释放I2C总线* 在一次完整的I2C事务(从START到STOP)完成后调* 释放后其他等待的任务可以获取锁并开始自己的I2C事务
 */
static void I2C1_Unlock(void)
{
	if (i2c1_mutex != NULL)
		osMutexRelease(i2c1_mutex);
}


/*==========================================================================
 * DWT微秒延时系统
 *
 * DWT (Data Watchpoint and Trace) 是Cortex-M3/M4内核的硬件调试组
 * 其中CYCCNT寄存器是一2位循环计数器,以CPU主频递增*
 * STM32F103主频72MHz:
 *   CYCCNT每秒递增72,000,000 *   1微秒 = 72个计*   1个计= 1/72 微秒 ≈ 13.89纳秒
 *
 * 为什么不用SysTick或TIM定时
 *   SysTick被FreeRTOS占用做系统心通常1ms精度),不够精确*   TIM定时器虽然精确但占用硬件资源。DWT是内核自带的,不占用任何外设*
 * 注意: CYCCNT2位的,2MHz下约59.7秒溢出一次*       delay_us()内部使用无符号减DWT->CYCCNT - start),即使溢出也能正确工作*       但单次延时不能超 59.7秒实际上软件I2C的微秒延时远小于*========================================================================*/

/*
 * 初始化DWT CYCCNT计数* 必须在使用delay_us()之前调用一* 步骤:
 *   1. 使能DWT跟踪(CoreDebug->DEMCR.TRCENA
 *   2. 清零CYCCNT计数*   3. 使能CYCCNT计数DWT->CTRL.CYCCNTENA
 */
static void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  /* 使能DWT跟踪模块 */
	DWT->CYCCNT = 0;                                  /* 清零计数*/
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动计数*/
}

/*
 * 微秒延时函数 - 基于DWT CYCCNT硬件计数*
 * 原理:
 *   记录起始计数计算目标计数=微秒× 每微秒的时钟,
 *   然后循环读取CYCCNT直到差值达到目标*
 * 参数: us - 延时的微秒数
 *
 * 精度: ±1个时钟周72MHz下约±13.89ns),对软件I2C完全够用
 *
 * 注意: 此函数会阻塞当前任务(忙等,只适用于短延时(微秒
 *       禁止在ISR中调虽然功能上可以工但会阻塞中断处理)
 */
static void delay_us(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (SystemCoreClock / 1000000);  /* 计算目标计数*/

	while ((DWT->CYCCNT - start) < ticks)
	{
		/* 忙等直到计数器达到目标*/
	}
}


/*==========================================================================
 * I2C总线实例
 *
 * 这是一个全局变量,描述了I2C1总线的所有属
 *   - GPIO端口: GPIOB (SDA和SCL都在PB
 *   - SDA引脚: PB13 (I2C1_SDA_Pin, 由CubeMX main.h定义)
 *   - SCL引脚: PB14 (I2C1_SCL_Pin, 由CubeMX main.h定义)
 *   - 重试次数: 3地址发送后无ACK时重
 *   - 延时函数: DWT微秒延时
 *   - 锁函FreeRTOS互斥*========================================================================*/
struct I2C_BusTypeDef i2c1 =
{
	.gpiox = I2C1_SCL_GPIO_Port,     /* GPIOB, SDA和SCL在同一端口 */
	.sda_gpio_pin = I2C1_SDA_Pin,     /* PB13 */
	.scl_gpio_pin = I2C1_SCL_Pin,     /* PB14 */
	.retries = 3,                      /* 地址无响应时最多重*/
	.udelay = (void (*)(uint32_t))delay_us,
	.lockInit = I2C1_LockInit,
	.lock = I2C1_Lock,
	.unlock = I2C1_Unlock,
};


/*==========================================================================
 * GPIO底层操作
 *
 * 软件I2C通过直接操作GPIO引脚来模拟I2C协议时序* I2C协议要求:
 *   - SCL为高电平SDA的下降沿表示START,上升沿表示STOP
 *   - SCL为高电平SDA必须稳定(数据有效)
 *   - SCL为低电平SDA可以变化(准备数据)
 *   - 每个SCL高电平期从机在第9个时钟拉低SDA表示ACK
 *
 * GPIO模式说明:
 *   开漏输Open-Drain): 输出0时引脚拉输出1时引脚浮靠外部上
 *   这是I2C协议的标准电气要允许多个设备共享总线(线与逻辑)
 *========================================================================*/

/*
 * GPIO硬件初始- 配置SDA和SCL为开漏输出模并释放为高电* 这是I2C总线的初始状SCL和SDA都为空闲
 */
static void I2C_BusHardwareInitialize(struct I2C_BusTypeDef *bus)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* 同时配置SCL和SDA两个引脚 */
	GPIO_InitStruct.Pin = bus->scl_gpio_pin | bus->sda_gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    /* 开漏输出模*/
	GPIO_InitStruct.Pull = GPIO_NOPULL;             /* 外部接上拉电内部不需*/
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;   /* 高速模*/

	HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

	/* 释放总线为高电平(开漏模式下= 引脚浮空,靠上拉电阻拉*/
	HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_SET);

	/* 调用锁初始化回调(当前为空实现) */
	if (bus->lockInit) bus->lockInit();
}


/*
 * SDA切换为输出模* 在发送数写操时调MCU主动控制SDA电平
 */
static inline void SDA_SetOutMode(struct I2C_BusTypeDef *bus)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = bus->sda_gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);
}

/*
 * SDA切换为输入模* 在读取数读操或等待ACK时调让从机控制SDA电平
 *
 * 注意: STM32的GPIO在输出模式下仍能读取输入寄存IDR),
 *       所以理论上不需要切换模式。但为了移植兼容其他MCU可能不行),
 *       保留了显式切换输输出模式的做法*/
static inline void SDA_SetInMode(struct I2C_BusTypeDef *bus)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = bus->sda_gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);
}

/* 读取SDA引脚电平(用于等待ACK和读取数据位) */
static inline uint8_t GET_SDA(struct I2C_BusTypeDef *bus)
{
	return HAL_GPIO_ReadPin(bus->gpiox, bus->sda_gpio_pin);
}

/* SDA拉低(开漏模式下输出0) */
static inline void SDA_L(struct I2C_BusTypeDef *bus)
{
	HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_RESET);
}

/* SDA释放(开漏模式下输出1 = 浮空,靠上拉拉*/
static inline void SDA_H(struct I2C_BusTypeDef *bus)
{
	HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_SET);
}

/* SCL拉低(时钟低电此时从机可以采样SDA) */
static inline void SCL_L(struct I2C_BusTypeDef *bus)
{
	HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_RESET);
}

/* SCL释放(时钟高电此时SDA数据有效) */
static inline void SCL_H(struct I2C_BusTypeDef *bus)
{
	HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);
}


/*==========================================================================
 * I2C协议时序函数
 *
 * I2C协议基本时序:
 *   START:  SCL高电平时,SDA从高变低
 *   STOP:   SCL高电平时,SDA从低变高
 *   RESTART: 不释放总线的START(用于读操作中切换方向)
 *   ACK:    第 9 个时钟脉冲期从机拉低SDA表示确认
 *   NACK:   第 9 个时钟脉冲期从机保持SDA高电平表示否*========================================================================*/

/*
 * 发送I2C起始条件(START)
 * 时序: SCL=-> SDA=-> SCL= * 在SDA下降沿时SCL必须为高电平,这是START的定*/
static inline void I2C_Start(struct I2C_BusTypeDef *bus)
{
	SDA_L(bus);              /* SDA先拉*/
	bus->udelay(1);          /* 保持tSU;STA = 1us(数据手册要求至少4.7us,1us2MHz下够*/
	SCL_L(bus);              /* 再拉低SCL,完成START */
}

/*
 * 发送I2C重复起始条件(RESTART)
 * 时序: SDA=-> SCL=-> SDA=-> SCL= * 用于读操作中不释放总线就切换方写地址 -> 读数
 */
static inline void I2C_Restart(struct I2C_BusTypeDef *bus)
{
	SDA_H(bus);              /* 先释放SDA */
	SCL_H(bus);              /* 再释放SCL */
	bus->udelay(1);          /* 保持时间 */
	SDA_L(bus);              /* SCL高电平时拉低SDA = RESTART */
	bus->udelay(1);
	SCL_L(bus);              /* 拉低SCL,准备后续数据传输 */
}

/*
 * 发送I2C停止条件(STOP)
 * 时序: SDA=-> SCL=-> SDA= * 在SDA上升沿时SCL必须为高电平,这是STOP的定* STOP后总线回到空闲SCL和SDA都为
 */
static inline void I2C_Stop(struct I2C_BusTypeDef *bus)
{
	SDA_L(bus);              /* 先拉低SDA */
	bus->udelay(1);
	SCL_H(bus);              /* 释放SCL */
	bus->udelay(1);
	SDA_H(bus);              /* SCL高电平时释放SDA = STOP */
	bus->udelay(1);
}

/*
 * 等待从机应答(ACK检
 *
 * 过程:
 *   1. 主机释放SDA(,让从机可以拉
 *   2. 产生一个SCL高电平脉*   3. 在SCL高电平期间读取SDA
 *   4. SDA=0 表示ACK(从机确认),SDA=1 表示NACK(从机否认)
 *
 * 返回: 1=收到ACK, 0=收到NACK
 */
static inline uint8_t I2C_WaitACK(struct I2C_BusTypeDef *bus)
{
	uint8_t ack;

	SDA_H(bus);              /* 主机释放SDA(靠上拉拉*/
	bus->udelay(1);
	SCL_H(bus);              /* SCL高电此时从机可以拉低SDA表示ACK */
	SDA_SetInMode(bus);      /* 切换为输入模读取从机的应*/
	ack = !GET_SDA(bus);     /* SDA=0是ACK,取反后ack=1表示收到ACK */
	SDA_SetOutMode(bus);     /* 切换回输出模*/
	I2C_INFO("%s", ack ? "ACK" : "NACK");
	SCL_L(bus);              /* 拉低SCL,完成个时*/

	return ack;
}

/*
 * 发送主机应用于读操
 * ack=1: 发送ACK(继续读下一个字
 * ack=0: 发送NACK(读取结束,最后一个字
 */
static inline void I2C_SendAckOrNack(struct I2C_BusTypeDef *bus, int ack)
{
	if (ack)
		SDA_L(bus);          /* ACK: 拉低SDA */
	/* 注意: 如果ack=0,SDA保持高电上拉),即NACK */
	bus->udelay(1);
	SCL_H(bus);              /* SCL高电平脉让从机采样SDA */
	SCL_L(bus);
}

/*
 * 写入一个字8位数+ 等待ACK)
 *
 * 过程:
 *   从MSB(bit7)到LSB(bit0)逐位发
 *   每一位的时序: SCL-> 设置SDA -> 延时 -> SCL-> 延时
 *   SCL高电平期间SDA必须稳定,从机在此时采*
 * 参数: data - 待发送的字节
 * 返回: 1=收到ACK, 0=收到NACK
 */
static uint8_t I2C_WriteByte(struct I2C_BusTypeDef *bus, uint8_t data)
{
	uint8_t mask;

	for (mask = 0x80; mask != 0; mask >>= 1)  /* 从bit7到bit0 */
	{
		SCL_L(bus);                            /* SCL低电准备数据 */
		data & mask ? SDA_H(bus) : SDA_L(bus); /* 根据当前位设置SDA */
		bus->udelay(1);                         /* 数据建立时间 */
		SCL_H(bus);                            /* SCL高电从机采样 */
	}
	SCL_L(bus);                                /* 8位发拉低SCL */
	bus->udelay(1);

	return I2C_WaitACK(bus);                   /* 等待从机应答 */
}


/*
 * 读取一个字8位数
 *
 * 过理:
 *   主机释放SDA(让从机控,从MSB到LSB逐位读取:
 *   每一位的时序: SCL-> 读取SDA -> SCL *   SCL高电平期间从机在SDA上放置数据位
 *
 * 返回: 读取到的字节
 */
static uint8_t I2C_ReadByte(struct I2C_BusTypeDef *bus)
{
	uint8_t mask;
	uint8_t data = 0;

	SDA_H(bus);                    /* 释放SDA(靠上拉拉*/
	bus->udelay(1);
	SDA_SetInMode(bus);            /* 切换为输入模读取从机数据 */
	for (mask = 0x80; mask != 0; mask >>= 1)  /* 从bit7到bit0 */
	{
		SCL_H(bus);                /* SCL高电从机放置数据 */
		if (GET_SDA(bus)) data |= mask;  /* 读取SDA并存入对应位 */
		SCL_L(bus);                /* SCL低电准备下一*/
		bus->udelay(1);
	}
	SDA_SetOutMode(bus);           /* 切换回输出模准备发送ACK/NACK) */

	return data;
}


/*==========================================================================
 * I2C消息传输函数
 *
 * 这些是内部辅助函由I2C_TransferMessages()调用
 *========================================================================*/

/*
 * 发送多个字写操
 *
 * 支持两种模式:
 *   普通模从buf缓冲区逐字节发*   SAME_BYTE模式: 重复发送同一个字sByte),用于填充场景
 *   CONTROL_BYTE模式: 每个数据字节前先发一个控制字cByte)
 *
 * 参数: msg - 消息指针
 * 返回: 实际成功发送的字节*/
static uint16_t I2C_SendBytes(struct I2C_BusTypeDef *bus, struct I2C_MessageTypeDef *msg)
{
	uint8_t ret;
	const uint8_t *ptr = msg->buf;
	uint16_t bytes = 0, count = msg->tLen;

	while (count > 0)
	{
		/* 如果需要发送控制字如SSD1306屏驱动的数据/命令区分) */
		if (msg->flags & I2C_CONTROL_BYTE && I2C_WriteByte(bus, msg->cByte) == 0)
		{
			I2C_WARNING("send bytes: NACK.");
			break;
		}

		/* 根据SAME_BYTE标志决定发送相同字节还是从缓冲区取字节 */
		ret = msg->flags & I2C_SAME_BYTE ? I2C_WriteByte(bus, msg->sByte) : I2C_WriteByte(bus, *ptr) , ptr++;

		if ((ret > 0) || (msg->flags & I2C_IGNORE_NACK && (ret == 0)))
		{
			count--;
			bytes++;
		}
		else if (ret == 0)
		{
			I2C_WARNING("send bytes: NACK.");
			break;
		}
	}

	return bytes;
}

/*
 * 接收多个字节(读操
 *
 * 读取过程:
 *   1. 逐字节读8
 *   2. 除了最后一个字每个字节读后发送ACK(告诉从机继续
 *   3. 最后一个字节发送NACK(告诉从机停止
 *
 * 如果flags包含I2C_NO_READ_ACK,则不发送ACK/NACK(特殊场景)
 *
 * 参数: msg - 消息指针
 * 返回: 实际成功读取的字节数
 */
static uint16_t I2C_RecvBytes(struct I2C_BusTypeDef *bus, struct I2C_MessageTypeDef *msg)
{
	uint8_t val;
	uint8_t *ptr = msg->buf;
	uint16_t bytes = 0, count = msg->tLen;

	while (count > 0)
	{
		val = I2C_ReadByte(bus);
		*ptr = val;
		bytes++;

		ptr++;
		count--;

		I2C_INFO("recieve bytes: 0x%02x, %s",
							val, (msg->flags & I2C_NO_READ_ACK) ?
							"(No ACK/NACK)" : (count ? "ACK" : "NACK"));

		if (!(msg->flags & I2C_NO_READ_ACK))
		{
			/* count>0时发ACK(还有字节要读),count=0时发NACK(最后一个字*/
			I2C_SendAckOrNack(bus, count);
		}
	}

	return bytes;
}

/*
 * 发送从机地址(带重试机
 *
 * 7位地址格式: [7位地址][R/W位] = 1字节
 *   写操地址左移1最低位=0
 *   读操地址左移1最低位=1
 *
 * 如果发送后收到NACK,会重试retries默认3
 * 每次重试前会发送STOP释放总线,再重新START
 *
 * 参数:
 *   addr    - 已经包含R/W位的地址字节
 *   retries - 最大重试次* 返回: 1=收到ACK, 0=全部重试失败
 */
static uint8_t I2C_SendAddress(struct I2C_BusTypeDef *bus, uint8_t addr, uint32_t retries)
{
	uint8_t i, ret = 0;

	for (i = 0; i <= retries; i++)
	{
		ret = I2C_WriteByte(bus, addr);
		if (ret == 1)
		{
			I2C_INFO("response ok.");
			break;                     /* 收到ACK,退出重试循*/
		}
		else if (i == retries)
		{
			I2C_WARNING("no response, please check slave device.");
			break;                     /* 最后一次重试也失败*/
		}
		I2C_WARNING("no response, attempt to resend the address. number:%d.", i);
		I2C_Stop(bus);                 /* 发送STOP释放总线 */
		bus->udelay(1);
		I2C_Start(bus);               /* 重新START */
	}

	return ret;
}

/*
 * 地址发送包装函- 处理710位地址和读/写方*
 * 7位地址模式(常用):
 *   直接将地址左移1加上R/W位后发*
 * 10位地址模式(很少:
 *   第一发11110[addr9:8][R/W] (首字
 *   第二发[addr7:0] (次字
 *   读操作时还需要发RESTART再发一次首字节(R/W=1)
 *
 * 返回: 1=成功, 0=失败
 */
static uint8_t I2C_BitSendAddress(struct I2C_BusTypeDef *bus, struct I2C_MessageTypeDef *msg)
{
	uint8_t ret, retries, addr1, addr2;
	uint8_t flags = msg->flags;
	uint8_t ignore_nack = msg->flags & I2C_IGNORE_NACK;

	retries = ignore_nack ? 0 : bus->retries;

	if (flags & I2C_ADDR_10BIT)
	{
		/* 10位地址模式 */
		addr1 = 0xf0 | ((msg->addr >> 7) & 0x06);  /* 首字11110XX */
		addr2 = msg->addr & 0xff;                     /* 次字*/

		I2C_INFO("addr1: %d, addr2: %d", addr1, addr2);

		ret = I2C_SendAddress(bus, addr1, retries);
		if ((ret != 1) && !ignore_nack)
		{
			I2C_WARNING("NACK: sending first addr");
			return 0;
		}

		ret = I2C_WriteByte(bus, addr2);
		if ((ret != 1) && !ignore_nack)
		{
			I2C_WARNING("NACK: sending second addr");
			return 0;
		}
		if (flags & I2C_RD)
		{
			/* 10位地址读操需要RESTART后再发一次首字节(R/W=1) */
			I2C_INFO("send repeated start condition");
			I2C_Restart(bus);
			addr1 |= 0x01;  /* 设置R/W位为*/
			ret = I2C_SendAddress(bus, addr1, retries);
			if ((ret != 1) && !ignore_nack)
			{
				I2C_ERROR("NACK: sending repeated addr");
				return 0;
			}
		}
	}
	else
	{
		/* 7位地址模式(最常用) */
		addr1 = msg->addr << 1;       /* 地址左移1*/
		if (flags & I2C_RD)
				addr1 |= 1;           /* 读操最低位=1 */
		ret = I2C_SendAddress(bus, addr1, retries);
		if ((ret != 1) && !ignore_nack)
				return 0;
	}

	return 1;
}


/*==========================================================================
 * I2C消息传输主函*
 * 这是软件I2C驱动对外的核心接所有I2C操作都通过此函数完成*
 * 工作流程:
 *   1. 获取互斥阻塞其他任务的I2C操作)
 *   2. 遍历消息数组,对每条消
 *      a. 发送START或RESTART(除非flags包含I2C_NO_START)
 *      b. 发送从机地址(7位或10s带R/W方向
 *      c. 发送或接收数据
 *   3. 发送STOP(除非最后一条消息flags包含I2C_NO_STOP)
 *   4. 释放互斥*
 * 返回: 成功传输的消息数失败返回0
 *
 * 典型用法:
 *   // 字节到寄存器0x10
 *   struct I2C_MessageTypeDef msg = {0};
 *   uint8_t buf[3] = {0x10, 0xAA, 0xBB};
 *   msg.addr = 0x08;  msg.flags = I2C_WR;  msg.buf = buf;  msg.tLen = 3;
 *   I2C_TransferMessages(&i2c1, &msg, 1);
 *
 *   // 从寄存器0x10s字节(先写寄存器地址,再读数据)
 *   struct I2C_MessageTypeDef msg[2] = {0};
 *   uint8_t reg = 0x10, data[2];
 *   msg[0] = {addr=0x08, flags=I2C_WR, buf=&reg, tLen=1};
 *   msg[1] = {addr=0x08, flags=I2C_RD, buf=data, tLen=2};
 *   I2C_TransferMessages(&i2c1, msg, 2);
 *========================================================================*/
uint32_t I2C_TransferMessages(struct I2C_BusTypeDef *bus, struct I2C_MessageTypeDef msgs[], uint32_t num)
{
	struct I2C_MessageTypeDef *msg;
	uint32_t i, ret = 0;
	uint8_t ignore_nack;

	if (NULL == bus || NULL == msgs || num == 0) return ret;

	/* 获取互斥保护整个I2C事务(从START到STOP) */
	if (bus->lock) bus->lock();

	for (i = 0; i < num; i++)
	{
		msg = &msgs[i];
		ignore_nack = msg->flags & I2C_IGNORE_NACK;

		/* 处理起始条件 */
		if (!(msg->flags & I2C_NO_START))
		{
			if (i)
			{
				I2C_Restart(bus);    /* 非第一条消发RESTART(不释放总线) */
			}
			else
			{
				I2C_INFO("send start condition");
				I2C_Start(bus);      /* 第一条消发START */
			}
			/* 发送从机地址(包含R/W方向*/
			ret = I2C_BitSendAddress(bus, msg);
			if ((ret != 1) && !ignore_nack)
			{
				I2C_WARNING("receive NACK from device addr 0x%02x msg %d", msgs[i].addr, i);
				goto out;             /* 地址无响提前退*/
			}
		}

		/* 数据传输 */
		if (msg->flags & I2C_RD)
		{
			/* 读操从从机接收数*/
			ret = I2C_RecvBytes(bus, msg);
			msg->rLen = ret;
			I2C_INFO("read %d byte%s", ret, ret == 1 ? "" : "s");
		}
		else
		{
			/* 写操向从机发送数*/
			ret = I2C_SendBytes(bus, msg);
			msg->rLen = ret;
			I2C_INFO("write %d byte%s", ret, ret == 1 ? "" : "s");
			if (msg->rLen != msg->tLen)
			{
				ret = 0;              /* 未全部发送成视为失败 */
				goto out;
			}
		}
	}
	ret = i;  /* 所有消息都成功 */

out:
	/* 处理停止条件 */
	if (!(msg->flags & I2C_NO_STOP))
	{
		I2C_INFO("send stop condition");
		I2C_Stop(bus);
	}

	/* 释放互斥*/
	if (bus->unlock) bus->unlock();

	return ret;
}


/*==========================================================================
 * I2C地址应答检*
 * 功能: 向指定地址发送一个写操作,仅检测从机是否返回ACK
 * 不发送任何实际数只做地址+R/W字节的ACK检*
 * 工作流程:
 *   1. 获取互斥*   2. 发送START条件
 *   3. 发送地址字节(7位地址左移1R/W=0表示
 *   4. 检测ACK/NACK
 *   5. 发送STOP条件释放总线
 *   6. 释放互斥*
 * 用初始化阶段验证I2C从机是否在线
 *========================================================================*/
uint8_t I2C_BusCheckAddress(struct I2C_BusTypeDef *bus, uint8_t addr)
{
	uint8_t ret;
	uint8_t addr_byte = addr << 1;  /* 7位地址左移1最低位=0(写操*/

	if (NULL == bus) return 0;

	if (bus->lock) bus->lock();

	I2C_Start(bus);
	ret = I2C_WriteByte(bus, addr_byte);
	I2C_Stop(bus);

	if (bus->unlock) bus->unlock();

	return ret;  /* 1=ACK, 0=NACK */
}


/*==========================================================================
 * I2C 总线恢复 (9-Clock Bus Recovery)
 *
 * 问题场景:
 *   MCU I2C 事务进行途中复位（调试器复位、看门狗、掉电）*   从机（BQ769X0）以为事务未结束，持续拉SDA 等待主机SCL *   导致 MCU 重启后总线永久锁死，任I2C 操作都无法得ACK *
 * 解决方案 (I2C Specification 推荐):
 *   向总线拨出最9 SCL 脉冲，使从机完成当前字节传输并释SDA *   随后发STOP 条件使总线正式回到空闲态（SCL 高时 SDA 上升沿）*
 * 调用时机:
 *   I2C_BusInitialize() 中，DWT_Init() 之后、GPIO 正式初始化之前*   此时 GPIO 时钟已由 CubeMX MX_GPIO_Init() 开启，可直接操作引脚*========================================================================*/
static void I2C_BusRecover(struct I2C_BusTypeDef *bus)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    int i;

    /* 临时SDA SCL 均配置为开漏输出，释放为高电平（空闲态） */
    GPIO_InitStruct.Pin   = bus->scl_gpio_pin | bus->sda_gpio_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

    HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_SET);
    delay_us(10);

    /* SDA 切换为输入以检测其电平 */
    GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

    if (HAL_GPIO_ReadPin(bus->gpiox, bus->sda_gpio_pin) == GPIO_PIN_SET)
    {
        /* SDA 为高电平，总线正常，无需恢复 */
        GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);
        return;
    }

    /* SDA 被从机拉低，总线锁死，切回输出后开9-clock 恢复 */
    GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

    I2C_WARNING("SDA stuck low! Starting 9-clock bus recovery...");

    for (i = 0; i < 9; i++)
    {
        /* 拨出一SCL 脉冲（高→低），给从机一个时钟让它移出数据位 */
        HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);
        delay_us(5);
        HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_RESET);
        delay_us(5);

        /* 每个脉冲后切SDA 为输入，检测从机是否已释放总线 */
        GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

        if (HAL_GPIO_ReadPin(bus->gpiox, bus->sda_gpio_pin) == GPIO_PIN_SET)
        {
            I2C_WARNING("SDA released after %d clock(s), sending STOP.", i + 1);

            /* SDA 已释放，切回输出，补STOP（SCL 高时 SDA 上升沿） */
            GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);

            HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_RESET); /* SDA 先拉*/
            delay_us(5);
            HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);   /* SCL 拉高 */
            delay_us(5);
            HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_SET);   /* SDA 上升= STOP */
            delay_us(5);
            return;
        }

        /* SDA 仍为低，切回输出继续下一个脉*/
        GPIO_InitStruct.Pin  = bus->sda_gpio_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        HAL_GPIO_Init(bus->gpiox, &GPIO_InitStruct);
    }

    /* 9 个脉冲后 SDA 仍然为低（严重故障），强制发STOP 并记录警*/
    I2C_WARNING("SDA still stuck after 9 clocks! Forcing STOP, check hardware.");
    HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(bus->gpiox, bus->scl_gpio_pin, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(bus->gpiox, bus->sda_gpio_pin, GPIO_PIN_SET);
    delay_us(5);
}


/*==========================================================================
 * I2C总线初始化函*
 * 初始化顺
 *   1. 初始化DWT微秒延时系统(Cortex-M3硬件计数
 *   2. 执行 9-clock 总线恢复(防止上次复位中断I2C事务导致从机锁死SDA)
 *   3. 创建FreeRTOS互斥保护多任务并发访
 *   4. 初始化GPIO硬件(SDA/SCL为开漏输释放为高电平)
 *
 * 调用时机：FreeRTOS 调度器启动后、任何 I2C 操作前调用。
 * 返回：0 表示成功，-1 表示静态互斥锁创建失败。
 *========================================================================*/
/* 初始化并恢复软件 I2C 总线，重复调用时复用已创建的互斥锁。 */
int I2C_BusInitialize(void)
{
	/* 步骤1: 初始化DWT CYCCNT微秒延时(详见DWT_Init注释) */
	DWT_Init();

	/* 步骤2: 9-clock 总线恢复
	 *   防止上次复位/掉电在I2C事务中途打断，导致从机持续拉低SDA锁死总线*   必须在DWT_Init()之后调用(依赖delay_us)，在osMutexNew之前调用(无需*/
	I2C_BusRecover(&i2c1);

	/* 步骤3: 首次调用时创建互斥锁，初始化重试时直接复用。 */
	if (i2c1_mutex == NULL)
	{
		i2c1_mutex = osMutexNew(&i2c1_mutex_attributes);
		if (i2c1_mutex == NULL)
		{
			return -1;
		}
	}

	/* 步骤4: 配置GPIO并释放总线为高电平(空闲*/
	I2C_BusHardwareInitialize(&i2c1);

	return 0;
}
