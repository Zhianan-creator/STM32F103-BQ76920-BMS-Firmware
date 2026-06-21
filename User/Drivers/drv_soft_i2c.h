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
#ifndef __DRV_SOFT_I2C_H__
#define __DRV_SOFT_I2C_H__

#include <stdio.h>

#include "stm32f1xx_hal.h"

/*
 * 软件I2C调试日志系统
 *
 * 原RT-Thread版本使用rt_kprintf输出日志,现改为printf
 * 需要将printf重定向到UART才能看到输出(例如在main.c中重写fputc)
 *
 * 日志等级设计思路:
 *   等级0 - 全部关闭,适合正式运行,不占用任何串口资默认)
 *   等级1 - 仅显示INFO级别,用于调试I2C通信流程
 *   等级2 - 仅显示WARNING级别,用于定位NACK等异*   等级3 - 仅显示ERROR级别,用于定位严重错误
 *
 * 使用方法:
 *   修改下面I2C_DEBUG_LEVEL 值即可切换等*   也可以在Keil工程的预编译宏中定义,优先级更*/
#ifndef I2C_DEBUG_LEVEL
#define I2C_DEBUG_LEVEL 0
#endif

#if I2C_DEBUG_LEVEL == 0
/* 等级0: 全部关闭,编译器会优化掉这些空不产生任何代*/
#define I2C_INFO(fmt, ...)
#define I2C_WARNING(fmt, ...)
#define I2C_ERROR(fmt, ...)

#elif I2C_DEBUG_LEVEL == 1
/* 等级1: 仅INFO,用于观察I2C通信流程(起始/停止/ACK/字节收发) */
#define I2C_INFO(fmt, ...)       printf("<<-I2C-INFO->> " fmt "\r\n", ##__VA_ARGS__)
#define I2C_WARNING(fmt, ...)
#define I2C_ERROR(fmt, ...)

#elif I2C_DEBUG_LEVEL == 2
/* 等级2: 仅WARNING,用于观察NACK重试等异常情*/
#define I2C_INFO(fmt, ...)
#define I2C_WARNING(fmt, ...)    printf("<<-I2C-WARNING->> " fmt "\r\n", ##__VA_ARGS__)
#define I2C_ERROR(fmt, ...)

#elif I2C_DEBUG_LEVEL == 3
/* 等级3: 仅ERROR,用于定位致命错误(地址无响应、传输失败等) */
#define I2C_INFO(fmt, ...)
#define I2C_WARNING(fmt, ...)
#define I2C_ERROR(fmt, ...)      printf("<<-I2C-ERROR->> " fmt "\r\n", ##__VA_ARGS__)
#endif


/*
 * I2C消息标志位定*
 * 这些标志位组合后放入 I2C_MessageTypeDef.flags 字段,
 * 告诉 I2C_TransferMessages() 如何处理这条消息* 可以用按位或组合使用,例如 I2C_WR | I2C_NO_STOP
 */
#define I2C_WR              0x00          /* 写操默认0不需要置*/
#define I2C_RD              (1 << 0)      /* 读操bit0=1表示*/
#define I2C_ADDR_10BIT      (1 << 1)      /* 10位地址模式(很少默认7*/
#define I2C_NO_START        (1 << 2)      /* 不发送起始条用于续传场景) */
#define I2C_IGNORE_NACK     (1 << 3)      /* 忽略从机的NACK响应(调试*/
#define I2C_NO_READ_ACK     (1 << 4)      /* 读取时不发送ACK(最后一个字节场*/
#define I2C_NO_STOP         (1 << 5)      /* 传输结束后不发送停止条用于续传) */

/* 自定义扩展标*/
#define I2C_CONTROL_BYTE    (1 << 6)      /* 发送数据前先发一个控制字如SSD1306屏驱*/
#define I2C_SAME_BYTE       (1 << 7)      /* 连续发送tLen个相同的sByte字节,避免循环调用 */


/*
 * I2C消息结构- 描述一次I2C传输事务
 *
 * 使用示例(字节到地址0x08):
 *   struct I2C_MessageTypeDef msg = {0};
 *   msg.addr  = 0x08;         // 7位从机地址
 *   msg.flags = I2C_WR;       // 写操*   msg.buf   = dataBuffer;   // 待发送的数据指针
 *   msg.tLen  = 2;            // 发字节
 *   // msg.rLen 会被填充为实际发送成功的字节*
 * 使用示例(先写寄存器地址,再读1字节):
 *   struct I2C_MessageTypeDef msg[2] = {0};
 *   msg[0].addr = 0x08;  msg[0].flags = I2C_WR;  msg[0].buf = &reg; msg[0].tLen = 1;
 *   msg[1].addr = 0x08;  msg[1].flags = I2C_RD;  msg[1].buf = &val; msg[1].tLen = 1;
 *   I2C_TransferMessages(&i2c1, msg, 2);  // 两条消息在一个事务中完成
 */
struct I2C_MessageTypeDef
{
	uint8_t   *buf;       /* 数据缓冲区指写操作时为待发送数读操作时为接收缓冲区) */
	uint16_t  addr;       /* 7位或10位从机地址(不含读写驱动会自动左*/
	uint16_t  tLen;       /* 需要传输的字节最5535) */
	uint16_t  rLen;       /* 实际成功传输的字节数(由驱动填*/
	uint8_t   flags;      /* 标志位组I2C_WR | I2C_NO_STOP */
	uint8_t   cByte;      /* 当flags包含I2C_CONTROL_BYTE此值作为控制字节发*/
	uint8_t   sByte;      /* 当flags包含I2C_SAME_BYTE连续发送tLen个此字节 */
};


/*
 * I2C总线结构- 描述一条软件I2C总线的所有信*
 * 当前硬件配置: SDA=PB13, SCL=PB14, 都在GPIOB * 因此只需一个gpiox指针即可操作两个引脚
 *
 * 函数指针说明:
 *   udelay   - 微秒延时函数(已改为DWT CYCCNT实现)
 *   lockInit - 初始化互斥锁(已改为在I2C_BusInitialize中创
 *   lock     - 获取互斥已改为osMutexAcquire)
 *   unlock   - 释放互斥已改为osMutexRelease)
 */
struct I2C_BusTypeDef
{
	GPIO_TypeDef *gpiox;      /* GPIO端口(SDA和SCL都在同一个端口上) */
	uint32_t gpio_rcc;        /* GPIO时钟(当前由CubeMX使能,此处保留兼容) */
	uint16_t sda_gpio_pin;    /* SDA引脚掩码,如GPIO_PIN_13 */
	uint16_t scl_gpio_pin;    /* SCL引脚掩码,如GPIO_PIN_14 */

	uint32_t retries;         /* 发送地址后无响应时的重试次数(最55) */

	void (*udelay)(uint32_t us);    /* 微秒延时函数指针 */
	void (*lockInit)(void);         /* 互斥锁初始化函数指针 */
	void (*lock)(void);             /* 获取互斥锁函数指*/
	void (*unlock)(void);           /* 释放互斥锁函数指*/
};

/* 全局I2C1总线实例(在drv_soft_i2c.c中定*/
extern struct I2C_BusTypeDef i2c1;

/*
 * I2C总线初始* 功能: 初始化DWT微秒延时、创建FreeRTOS互斥锁、配置GPIO为开漏输* 返回: 0=成功, -1=互斥锁创建失* 注意: 必须在FreeRTOS调度器启动后、任何I2C操作前调*/
int I2C_BusInitialize(void);

/*
 * I2C消息传输函数 - 软件I2C的核心函*
 * 功能: 按照msgs数组的顺在一个完整的I2C事务中传输所有消*       支持: 写操作、读操作、续无起无停0位地址、NACK重试*
 * 参数:
 *   bus  - I2C总线指针(i2c1)
 *   msgs - 消息数组指针
 *   num  - 消息数量
 *
 * 返回: 成功传输的消息数失败返回0
 *
 * 线程安全: 内部使用FreeRTOS互斥锁保多个任务可以安全调用
 * 注意: 禁止在ISR(中断服务程序)中调用此函数!
 */
uint32_t I2C_TransferMessages(struct I2C_BusTypeDef *bus, struct I2C_MessageTypeDef msgs[], uint32_t num);

/*
 * I2C地址应答检- 检查指定地址的从机是否存在并能响*
 * 功能: 向指定地址发送一个写操作,检测从机是否返回ACK
 * 用初始化阶段验证芯片是否在不涉及实际数据传*
 * 参数:
 *   bus  - I2C总线指针(i2c1)
 *   addr - 7位从机地址(不含读写如BQ769X0x08)
 *
 * 返回: true=收到ACK(从机在线), false=收到NACK(从机不在线或未响
 *
 * 注意: 禁止在ISR中调内部使用互斥
 */
uint8_t I2C_BusCheckAddress(struct I2C_BusTypeDef *bus, uint8_t addr);

#endif
