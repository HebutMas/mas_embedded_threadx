# 有界无锁 MPMC 队列 (mpmc_queue)

基于 Dmitry Vyukov 算法的**有界无锁多生产者多消费者**队列，基于外部缓冲区实现，容量自动向下取整为 2 的幂。满时驱逐最老元素

## 数据结构

### `struct mpmc_queue` — 队列实例

| 字段 | 类型 | 说明 |
|------|------|------|
| `seq` | `uint32_t *` | 每槽发布序号，与 `pos` 的有符号差表达槽位状态 |
| `slots` | `uint8_t *` | 元素存储区 |
| `capacity` | `uint32_t` | 槽数（2 的幂） |
| `mask` | `uint32_t` | 容量 - 1，`pos & mask` 即当前槽位 |
| `elem_size` | `uint32_t` | 每个元素的字节数 |
| `head` | `uint32_t` | 下一个入队位置（持续递增，永不回退） |
| `tail` | `uint32_t` | 下一个出队位置（持续递增，永不回退） |

> **缓冲区由调用者提供**，`slot_buf` 需 ≥ `capacity * elem_size` 字节，`seq_buf` 需 ≥ `capacity` 个 `uint32_t`。初始化后两者的生命周期必须覆盖整个队列使用期。

### 槽位状态

`seq[i]` 与当前 `pos` 的差值（统一转 `int32_t` 比较）表达三种状态：

| 条件 | 含义 |
|---|---|
| `seq[i] == pos` | 空闲，可入队 |
| `seq[i] == pos + 1` | 就绪，可出队 |
| `seq[i] == pos - capacity`（差值为负） | 上一轮数据未消费，队满 |

```
capacity = 4, 初始 seq = [0,1,2,3]
                   head=tail=0
                    ↓
         ┌─────┬─────┬─────┬─────┐
seq →    │  0  │  1  │  2  │  3  │
slots →  │ [0] │ [1] │ [2] │ [3] │
         └─────┴─────┴─────┴─────┘

入队 1 个: head=1, seq[0]=1     (pos 0 → 就绪)
出队 1 个: tail=1, seq[0]=1+4=5 (归还，下一轮 pos 5 时 seq==5 即空闲)
```

`head`/`tail` 持续递增溢出 32-bit 后自动回绕，配合 `& mask` 定位物理槽位，无需条件分支处理边界。

## API 说明

```c
int      mpmc_queue_init(mpmc_queue_t *q, void *slot_buf, uint32_t *seq_buf,
                         uint32_t capacity, uint32_t elem_size);
uint32_t mpmc_queue_push(mpmc_queue_t *q, const void *element, void *dropped_oldest);
bool     mpmc_queue_pop(mpmc_queue_t *q, void *element);
uint32_t mpmc_queue_count(const mpmc_queue_t *q);
```

## 使用示例

### 基本入队 / 出队

```c
#include "mpmc_queue.h"

#define CAP 8  // 2 的幂，否则自动向下取整

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
} item_t;

void mpmc_demo(void)
{
    uint32_t     seq[CAP];
    uint8_t      slots[CAP * sizeof(item_t)];
    mpmc_queue_t q;

    mpmc_queue_init(&q, slots, seq, CAP, sizeof(item_t));

    item_t v = {.id = 1};
    mpmc_queue_push(&q, &v, NULL);  // 未满，返回 0

    item_t out;
    if (mpmc_queue_pop(&q, &out))
    {
        // out.id == 1
    }
}
```

### 满时驱逐最老

```c
item_t   dropped;
uint32_t evicted = mpmc_queue_push(&q, &v, &dropped);
if (evicted != 0)
{
    // dropped 是最老的一帧，evicted 是本次驱逐总数
}
```

### 只丢弃

```c
/* 出队但不需要数据：推进 tail、归还槽位 */
mpmc_queue_pop(&q, NULL);
```

## 内部实现

### 入队（`mpmc_queue_push`）

```
外层循环:
  pos = head; full = false
  内层 while (!full):
    seq = seq[pos & mask];  dif = seq - pos
      dif == 0  → CAS(head, pos, pos+1)
                    成功: memcpy(slots, element); seq[pos&mask] = pos+1 (release); return evicted
                    失败: pos 已被更新，继续内层
      dif < 0   → 队满，full = true
      dif > 0   → 其他生产者已抢占，重读 head 重试
  队满: mpmc_queue_pop(q, evicted == 0 ? dropped_oldest : NULL)
        evicted++，回到外层重试
```

### 出队（`mpmc_queue_pop`）

```
循环:
  pos = tail
  seq = seq[pos & mask];  dif = seq - (pos + 1)
    dif == 0  → CAS(tail, pos, pos+1)
                  成功: element 非空则 memcpy; seq[pos&mask] = pos + capacity (release); return true
                  失败: 继续重试
    dif < 0   → 队空，return false
    dif > 0   → 其他消费者已取走，重读 tail 重试
```


## 注意事项

1. **缓冲区生命周期** — 不分配内存，`slot_buf` / `seq_buf` 必须在整个队列使用期间保持有效

2. **容量为 2 的幂** — 非 2 的幂会自动向下取整，`seq_buf` 仍需按取整后的容量给足

3. **定长元素** — 所有元素共用 `elem_size`，不支持变长

4. **队列语义** — 始终保持最新的 ≤capacity 帧，最新胜出

## 与 kfifo 的取舍

| 场景 | 选择 |
|---|---|
| 严格单生产者-单消费者（UART / USB-CDC 的 ISR → 任务） | `kfifo` |
| 生产者可并发（多任务，或 ISR 与任务混用、多路 ISR） | `mpmc_queue` |


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-10-24 | 初始版本 |
