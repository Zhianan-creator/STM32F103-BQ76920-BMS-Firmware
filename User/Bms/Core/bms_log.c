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
#include "bms_log.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* 全局日志等级限制，默WARN */
static BMS_LogLevel_t global_log_level = BMS_LOG_LEVEL_WARN;

/* 多任务打印互斥锁 */
static osMutexId_t log_mutex = NULL;

void BMS_LogInit(void)
{
    global_log_level = BMS_LOG_LEVEL_WARN;

    /* 创建 CMSIS-RTOS2 互斥*/
    log_mutex = osMutexNew(NULL);
    if (log_mutex == NULL)
    {
        printf("[LOG] WARN: Mutex creation failed, fallback to unlocked logging\r\n");
    }
}

void BMS_LogSetLevel(BMS_LogLevel_t level)
{
    if (level <= BMS_LOG_LEVEL_DEBUG)
    {
        global_log_level = level;
    }
}

BMS_LogLevel_t BMS_LogGetLevel(void)
{
    return global_log_level;
}

static const char *BMS_LogLevelToString(BMS_LogLevel_t level)
{
    switch (level)
    {
        case BMS_LOG_LEVEL_ERROR: return "ERROR";
        case BMS_LOG_LEVEL_WARN:  return "WARN";
        case BMS_LOG_LEVEL_INFO:  return "INFO";
        case BMS_LOG_LEVEL_DEBUG: return "DEBUG";
        default:                  return "NONE";
    }
}

void BMS_LogPrintf(BMS_LogLevel_t level, const char *tag, const char *fmt, ...)
{
    /* 1. 过滤未达到等级限制的日志 */
    if (level == BMS_LOG_LEVEL_NONE || level > global_log_level)
    {
        return;
    }

    /* 2. 在栈上分配缓冲，防栈溢出控制总大小在 384 字节*/
    char msg_buf[160];
    char log_buf[224];
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);

    /* 3. 按照统一规格进行包装格式[TAG][LEVEL] message\r\n */
    snprintf(log_buf, sizeof(log_buf), "[%s][%s] %s\r\n", tag, BMS_LogLevelToString(level), msg_buf);

    /* 4. 多任务互斥保护输*/
    uint8_t locked = 0;
    if (log_mutex != NULL && osKernelGetState() == osKernelRunning)
    {
        if (osMutexAcquire(log_mutex, osWaitForever) == osOK)
        {
            locked = 1;
        }
    }

    /* 5. 阻塞式一次性将完整字符串发出，避免碎片化与 printf 多核交错 */
    HAL_UART_Transmit(&huart1, (uint8_t *)log_buf, strlen(log_buf), HAL_MAX_DELAY);

    if (locked)
    {
        osMutexRelease(log_mutex);
    }
}
