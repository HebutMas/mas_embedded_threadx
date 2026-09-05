#include <stdarg.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "ulog.h"
#include "ulog_def.h"
#include "SEGGER_RTT.h"

/* 集成eyalroz/printf */
#define PRINTF_DISABLE_SUPPORT_LONG_LONG   1 // 关闭64位整数支持（减小体积）
#define PRINTF_DISABLE_SUPPORT_EXPONENTIAL 1 // 关闭科学计数法
#define PRINTF_DISABLE_SUPPORT_PTRDIFF_T   1
#define PRINTF_DISABLE_SUPPORT_FLOAT       1 // 关闭浮点数支持
#include "printf.h"                          // eyalroz/printf 头文件

/* HAL_GetTick 外部声明（STM32 HAL库） */
extern uint32_t HAL_GetTick(void);

/* ThreadX 头文件 */
#include "tx_api.h"

/* Hex查表法 */
static const char hex_table[] = "0123456789ABCDEF";
#define HEX_HIGH(val) hex_table[((val) >> 4) & 0x0F] // 取高4位
#define HEX_LOW(val)  hex_table[(val) & 0x0F]        // 取低4位

/* 中断检测 */
static inline bool ulog_in_isr(void)
{
    uint32_t ipsr;
    __asm__ volatile("mrs %0, ipsr" : "=r"(ipsr));
    return ipsr != 0;
}

/* 颜色信息 */
#ifdef ULOG_USING_COLOR
static const char *const color_output_info[] = {
    ULOG_COLOR_ASSERT, NULL, NULL, ULOG_COLOR_ERROR, ULOG_COLOR_WARN, NULL, ULOG_COLOR_INFO, ULOG_COLOR_DEBUG,
};
#endif

/* Ulog 控制块 */
static struct
{
    bool     init_ok;
    uint32_t level;
    char     log_buf_th[ULOG_LINE_BUF_SIZE + 1];
    char     log_buf_isr[ULOG_LINE_BUF_SIZE + 1];
} ulog = {
    .init_ok = false,
    .level   = LOG_LVL_DBG,
};

/* 字符串拼接 */
static inline size_t ulog_snprintf(char *dst, size_t dst_len, const char *fmt, ...)
{
    if (dst == NULL || fmt == NULL || dst_len >= ULOG_LINE_BUF_SIZE)
    {
        return 0;
    }
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf_(dst + dst_len, ULOG_LINE_BUF_SIZE - dst_len, fmt, args);
    va_end(args);
    return (len > 0 && (dst_len + len) < ULOG_LINE_BUF_SIZE) ? (size_t)len : 0;
}

/* 日志头部格式化 */
static size_t ulog_head_formater(char *log_buf, uint32_t level, const char *tag)
{
    size_t log_len = 0;
    if (log_buf == NULL || tag == NULL || level > LOG_LVL_DBG)
    {
        return 0;
    }

    // 颜色
#ifdef ULOG_USING_COLOR
    if (color_output_info[level] != NULL)
    {
        log_len += ulog_snprintf(log_buf, log_len, "%s%s", CSI_START, color_output_info[level]);
    }
#endif

    // 时间戳
#ifdef ULOG_OUTPUT_TIME
    uint32_t tick = HAL_GetTick();
    log_len += ulog_snprintf(log_buf, log_len, "[%lu.%03lu]", tick / 1000, tick % 1000);
#endif

    // 标签
#ifdef ULOG_OUTPUT_TAG
    log_len += ulog_snprintf(log_buf, log_len, "%s%s", (log_len > 0 ? " " : ""), tag);
#endif

    // 线程名
#ifdef ULOG_OUTPUT_THREAD_NAME
    const char *thread_name = "ISR";
    if (!ulog_in_isr())
    {
        TX_THREAD *p_thread = tx_thread_identify();
        thread_name         = (p_thread && p_thread->tx_thread_name[0]) ? p_thread->tx_thread_name : "Pre-Init";
    }
    log_len += ulog_snprintf(log_buf, log_len, "%s%s", (log_len > 0 ? " " : ""), thread_name);
#endif

    // 分隔符
    log_len += ulog_snprintf(log_buf, log_len, ": ");
    return log_len;
}

/* 日志尾部格式化 */
static size_t ulog_tail_formater(char *log_buf, size_t log_len, bool newline, uint32_t level)
{
    if (log_buf == NULL || log_len >= ULOG_LINE_BUF_SIZE)
    {
        return log_len;
    }

    // 预留空间
    size_t reserve = 0;
#ifdef ULOG_USING_COLOR
    reserve = (color_output_info[level] != NULL) ? strlen(CSI_END) : 0;
#endif
    size_t suffix_len = reserve + (newline ? strlen(ULOG_NEWLINE_SIGN) : 0);
    if (log_len + suffix_len >= ULOG_LINE_BUF_SIZE)
    {
        log_len = ULOG_LINE_BUF_SIZE - suffix_len - 1;
    }

    // 换行
    if (newline)
    {
        log_len += ulog_snprintf(log_buf, log_len, "%s", ULOG_NEWLINE_SIGN);
    }

    // 颜色重置
#ifdef ULOG_USING_COLOR
    if (color_output_info[level] != NULL)
    {
        log_len += ulog_snprintf(log_buf, log_len, "%s", CSI_END);
    }
#endif

    log_buf[log_len] = '\0';
    return log_len;
}

/* 完整日志格式化 */
static size_t ulog_formater(char *log_buf, uint32_t level, const char *tag, bool newline, const char *format, va_list args)
{
    if (log_buf == NULL || format == NULL)
    {
        return 0;
    }

    // 头部
    size_t log_len = ulog_head_formater(log_buf, level, tag);
    if (log_len >= ULOG_LINE_BUF_SIZE)
    {
        return ulog_tail_formater(log_buf, ULOG_LINE_BUF_SIZE - 1, newline, level);
    }

    // 日志内容
    va_list args_copy;
    va_copy(args_copy, args);
    int fmt_len = vsnprintf_(log_buf + log_len, ULOG_LINE_BUF_SIZE - log_len, format, args_copy);
    va_end(args_copy);
    log_len += (fmt_len > 0 && (log_len + fmt_len) < ULOG_LINE_BUF_SIZE) ? (size_t)fmt_len : (ULOG_LINE_BUF_SIZE - log_len - 1);

    // 尾部
    return ulog_tail_formater(log_buf, log_len, newline, level);
}

/* Hex转储查表法实现 */
static size_t ulog_hex_formater(char *log_buf, const char *tag, const uint8_t *buf, size_t size, size_t width, uint32_t addr)
{
    if (log_buf == NULL || buf == NULL || tag == NULL)
    {
        return 0;
    }

    // 头部
    size_t log_len = ulog_head_formater(log_buf, LOG_LVL_DBG, tag);
    if (log_len >= ULOG_LINE_BUF_SIZE)
    {
        return ulog_tail_formater(log_buf, ULOG_LINE_BUF_SIZE - 1, true, LOG_LVL_DBG);
    }

    // 地址范围
    log_len += ulog_snprintf(log_buf, log_len, "%04" PRIx32 "-%04" PRIx32 ": ", addr, addr + size);

    // 查表法输出十六进制数据
    for (size_t i = 0; i < width && i < size; i++)
    {
        if (log_len + 3 >= ULOG_LINE_BUF_SIZE) break; // 预留 "XX " 3个字符

        log_buf[log_len++] = HEX_HIGH(buf[i]); // 高位
        log_buf[log_len++] = HEX_LOW(buf[i]);  // 低位
        log_buf[log_len++] = ' ';

        // 每8个字节加一个空格
        if ((i + 1) % 8 == 0)
        {
            if (log_len + 1 < ULOG_LINE_BUF_SIZE)
            {
                log_buf[log_len++] = ' ';
            }
        }
    }

    // 填充空格
    if (size < width)
    {
        int pad_len = (int)(((width - size) * 3) + ((width >= 8 && size < 8) ? 1 : 0));
        if (log_len + pad_len < ULOG_LINE_BUF_SIZE)
        {
            memset(log_buf + log_len, ' ', pad_len);
            log_len += pad_len;
        }
    }

    // 字符显示
    log_len += ulog_snprintf(log_buf, log_len, "  ");
    for (size_t i = 0; i < size; i++)
    {
        if (log_len + 1 >= ULOG_LINE_BUF_SIZE)
        {
            break;
        }
        log_buf[log_len++] = (buf[i] >= ' ' && buf[i] <= '~') ? buf[i] : '.';
    }

    // 尾部
    return ulog_tail_formater(log_buf, log_len, true, LOG_LVL_DBG);
}

/* 输出核心函数 */
static void do_output(const char *log_buf, size_t log_len)
{
    if (log_buf && log_len > 0)
    {
        SEGGER_RTT_Write(0, log_buf, (unsigned)log_len);
    }
}

static inline char *get_log_buf(void) { return ulog_in_isr() ? ulog.log_buf_isr : ulog.log_buf_th; }

/* 公共API */
void ulog_voutput(uint32_t level, const char *tag, bool newline, const char *format, va_list args)
{
    if (!ulog.init_ok || tag == NULL || format == NULL || level > LOG_LVL_DBG || level > ulog.level)
    {
        return;
    }

    char *log_buf = get_log_buf();
    if (log_buf == NULL)
    {
        return;
    }

    SEGGER_RTT_LOCK();
    size_t log_len = ulog_formater(log_buf, level, tag, newline, format, args);
    do_output(log_buf, log_len);
    SEGGER_RTT_UNLOCK();
}

void ulog_output(uint32_t level, const char *tag, bool newline, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    ulog_voutput(level, tag, newline, format, args);
    va_end(args);
}

void ulog_raw(const char *format, ...)
{
    if (!ulog.init_ok || format == NULL)
    {
        return;
    }

    char *log_buf = get_log_buf();
    if (log_buf == NULL)
    {
        return;
    }

    SEGGER_RTT_LOCK();
    va_list args;
    va_start(args, format);
    int fmt_len = vsnprintf_(log_buf, ULOG_LINE_BUF_SIZE, format, args);
    va_end(args);

    size_t log_len = (fmt_len > 0 && fmt_len < ULOG_LINE_BUF_SIZE) ? (size_t)fmt_len : ULOG_LINE_BUF_SIZE;
    do_output(log_buf, log_len);
    SEGGER_RTT_UNLOCK();
}

void ulog_hexdump(const char *tag, uint32_t width, const uint8_t *buf, uint32_t size)
{
    if (!ulog.init_ok || tag == NULL || buf == NULL || size == 0 || width == 0)
    {
        return;
    }

    char *log_buf = get_log_buf();
    if (log_buf == NULL)
    {
        return;
    }

    SEGGER_RTT_LOCK();
    for (uint32_t offset = 0; offset < size; offset += width)
    {
        size_t len     = (offset + width > size) ? (size - offset) : width;
        size_t log_len = ulog_hex_formater(log_buf, tag, buf + offset, len, width, offset);
        do_output(log_buf, log_len);
    }
    SEGGER_RTT_UNLOCK();
}

void ulog_flush(void) {}

void ulog_set_level(uint32_t level)
{
    if (level <= LOG_LVL_DBG)
    {
        ulog.level = level;
    }
}

uint32_t ulog_get_level(void) { return ulog.level; }

int ulog_init(void)
{
    if (ulog.init_ok)
    {
        return 0;
    }

    SEGGER_RTT_Init();
    ulog.init_ok = true;
    return 0;
}

void ulog_deinit(void) { ulog.init_ok = false; }
