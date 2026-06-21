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
#ifndef BMS_LOG_H
#define BMS_LOG_H

#include "main.h"

/* BMS 日志等级定义 */
typedef enum
{
    BMS_LOG_LEVEL_NONE = 0,
    BMS_LOG_LEVEL_ERROR,
    BMS_LOG_LEVEL_WARN,
    BMS_LOG_LEVEL_INFO,
    BMS_LOG_LEVEL_DEBUG
} BMS_LogLevel_t;

/* ==========================================================================
 * BMS Log 业务层公开 API 接口
 * ========================================================================== */

/* 初始化串口日志业务层（创建互斥锁*/
void BMS_LogInit(void);

/* 设置全局日志等级限制 */
void BMS_LogSetLevel(BMS_LogLevel_t level);

/* 获取当前全局日志等级 */
BMS_LogLevel_t BMS_LogGetLevel(void);

/* 统一格式化输出的日志服务接口 */
void BMS_LogPrintf(BMS_LogLevel_t level, const char *tag, const char *fmt, ...);

/* ==========================================================================
 * BMS 日志辅助* ========================================================================== */
#define BMS_LOGE(tag, fmt, ...) BMS_LogPrintf(BMS_LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#define BMS_LOGW(tag, fmt, ...) BMS_LogPrintf(BMS_LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define BMS_LOGI(tag, fmt, ...) BMS_LogPrintf(BMS_LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define BMS_LOGD(tag, fmt, ...) BMS_LogPrintf(BMS_LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)

#endif /* BMS_LOG_H */
