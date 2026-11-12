#include "mpmc_queue.h"

#include <string.h>

/* seq 与 target 的差值统一转有符号比较，uint32 回绕 */
static int32_t mpmc_diff(uint32_t seq, uint32_t target) { return (int32_t)(seq - target); }

static uint32_t mpmc_rounddown_pow2(uint32_t n)
{
    uint32_t cap = 2u;
    while (cap * 2u <= n) cap *= 2u;
    return cap;
}

int mpmc_queue_init(mpmc_queue_t *q, void *slot_buf, uint32_t *seq_buf, uint32_t capacity, uint32_t elem_size)
{
    if (q == NULL || slot_buf == NULL || seq_buf == NULL || elem_size == 0u || capacity < 2u) return -1;

    if ((capacity & (capacity - 1u)) != 0u) capacity = mpmc_rounddown_pow2(capacity);

    q->slots     = (uint8_t *)slot_buf;
    q->seq       = seq_buf;
    q->capacity  = capacity;
    q->mask      = capacity - 1u;
    q->elem_size = elem_size;
    q->head      = 0u;
    q->tail      = 0u;
    /* 每个槽位的初始 seq 等于其索引，即位置 i 处于"空闲"状态 */
    for (uint32_t i = 0u; i < capacity; i++) q->seq[i] = i;

    return 0;
}

bool mpmc_queue_pop(mpmc_queue_t *q, void *element)
{
    uint32_t pos = __atomic_load_n(&q->tail, __ATOMIC_RELAXED);

    for (;;)
    {
        uint32_t idx = pos & q->mask;
        uint32_t seq = __atomic_load_n(&q->seq[idx], __ATOMIC_ACQUIRE);
        int32_t  dif = mpmc_diff(seq, pos + 1u);

        if (dif == 0) /* 数据就绪，可抢占 */
        {
            uint32_t next = pos + 1u;
            if (__atomic_compare_exchange_n(&q->tail, &pos, next, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED))
            {
                if (element != NULL) memcpy(element, q->slots + (size_t)idx * q->elem_size, q->elem_size);
                __atomic_store_n(&q->seq[idx], pos + q->capacity, __ATOMIC_RELEASE); /* 归还槽位 */
                return true;
            }
        }
        else if (dif < 0) /* 槽位尚未发布：队列为空（或有生产者正写入本槽） */
        {
            return false;
        }
        else /* 该槽位已被其他消费者取走，重读 tail 追上进度 */
        {
            pos = __atomic_load_n(&q->tail, __ATOMIC_RELAXED);
        }
    }
}

uint32_t mpmc_queue_push(mpmc_queue_t *q, const void *element, void *dropped_oldest)
{
    uint32_t evicted = 0u;

    for (;;)
    {
        uint32_t pos  = __atomic_load_n(&q->head, __ATOMIC_RELAXED);
        bool     full = false;

        /* 单次入队尝试：成功直接返回；队列满则退出内层去驱逐 */
        while (!full)
        {
            uint32_t idx = pos & q->mask;
            uint32_t seq = __atomic_load_n(&q->seq[idx], __ATOMIC_ACQUIRE);
            int32_t  dif = mpmc_diff(seq, pos);

            if (dif == 0) /* 槽位空闲，可抢占 */
            {
                uint32_t next = pos + 1u;
                /* CAS 失败时 pos 被更新为当前 head，直接以新值重试 */
                if (__atomic_compare_exchange_n(&q->head, &pos, next, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED))
                {
                    memcpy(q->slots + (size_t)idx * q->elem_size, element, q->elem_size);
                    __atomic_store_n(&q->seq[idx], next, __ATOMIC_RELEASE); /* 发布数据 */
                    return evicted;
                }
            }
            else if (dif < 0) /* 槽位仍装着上一轮未消费的数据：队列已满 */
            {
                full = true;
            }
            else /* 该槽位已被其他生产者抢占并发布，重读 head 追上进度 */
            {
                pos = __atomic_load_n(&q->head, __ATOMIC_RELAXED);
            }
        }

        /* 满时出队并丢弃最老一帧腾位 */
        if (mpmc_queue_pop(q, evicted == 0u ? dropped_oldest : NULL)) evicted++;
    }
}