#ifndef _MPMC_QUEUE_H
#define _MPMC_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mpmc_queue
{
    uint32_t *seq;       /* 每槽发布序号，与 pos 的有符号差表达"空槽/就绪/未消费" */
    uint8_t  *slots;     /* 元素存储区 */
    uint32_t  capacity;  /* 槽数（2 的幂） */
    uint32_t  mask;      /* capacity - 1 */
    uint32_t  elem_size; /* 单元素字节数 */
    uint32_t  head;      /* 下一个入队位置 */
    uint32_t  tail;      /* 下一个出队位置 */
} mpmc_queue_t;

/**
 * @brief 初始化队列
 * @param q         队列句柄
 * @param slot_buf  元素存储区，大小 >= capacity * elem_size 字节
 * @param seq_buf   槽位序号数组，大小 >= capacity 个 uint32_t
 * @param capacity  槽数；非 2 的幂时向下取整（与 kfifo_init 一致），最小 2
 * @param elem_size 单元素字节数，> 0
 * @return 成功返回 0，参数非法返回 -1
 */
int mpmc_queue_init(mpmc_queue_t *q, void *slot_buf, uint32_t *seq_buf, uint32_t capacity, uint32_t elem_size);

/**
 * @brief 入队，队列满时丢弃最老元素为新元素腾位
 * @param dropped_oldest 输出本次驱逐中最老的一帧，可为 NULL
 * @return 本次调用驱逐的元素个数，0 表示未满、等价于一次普通入队
 */
uint32_t mpmc_queue_push(mpmc_queue_t *q, const void *element, void *dropped_oldest);

/**
 * @brief 出队
 * @param element 输出去处；传 NULL 表示只丢弃
 * @return 成功返回 true；队列空返回 false
 */
bool mpmc_queue_pop(mpmc_queue_t *q, void *element);


#ifdef __cplusplus
}
#endif

#endif /* _MPMC_QUEUE_H */
