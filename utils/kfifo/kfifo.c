#include "kfifo.h"
#include <string.h>
#include "cmsis_gcc.h"

// 内部辅助函数

// 向下取整为 2 的幂
static unsigned int rounddown_pow_of_two(unsigned int n)
{
    if (n == 0) return 0;
    unsigned int highest = 1;
    while (highest <= n) highest <<= 1;
    return highest >> 1;
}

// 内部拷贝辅助（自动处理回绕）
static void fifo_copy_in(struct kfifo *fifo, const void *src, unsigned int len, unsigned int off)
{
    unsigned int size  = fifo->mask + 1;
    unsigned int esize = fifo->esize;
    unsigned int l;

    off &= fifo->mask;
    if (esize != 1)
    {
        off *= esize;
        size *= esize;
        len *= esize;
    }
    l = (len <= (size - off)) ? len : (size - off);

    memcpy(fifo->data + off, src, l);
    memcpy(fifo->data, (const char *)src + l, len - l);
}

static void fifo_copy_out(const struct kfifo *fifo, void *dst, unsigned int len, unsigned int off)
{
    unsigned int size  = fifo->mask + 1;
    unsigned int esize = fifo->esize;
    unsigned int l;

    off &= fifo->mask;
    if (esize != 1)
    {
        off *= esize;
        size *= esize;
        len *= esize;
    }
    l = (len <= (size - off)) ? len : (size - off);

    memcpy(dst, fifo->data + off, l);
    memcpy((char *)dst + l, fifo->data, len - l);
}

// 对外函数
int kfifo_init(struct kfifo *fifo, void *buffer, unsigned int size, unsigned int esize)
{
    if (size < 2) return -1;

    // 如果不是 2 的幂，向下取整，避免传入错误值
    if ((size & (size - 1)) != 0) size = rounddown_pow_of_two(size);

    fifo->in    = 0;
    fifo->out   = 0;
    fifo->esize = esize;
    fifo->data  = (char *)buffer;
    fifo->mask  = size - 1;
    return 0;
}

unsigned int kfifo_len(const struct kfifo *fifo) { return fifo->in - fifo->out; }

unsigned int kfifo_avail(const struct kfifo *fifo) { return (fifo->mask + 1) - (fifo->in - fifo->out); }

int kfifo_is_empty(const struct kfifo *fifo) { return fifo->in == fifo->out; }

int kfifo_is_full(const struct kfifo *fifo) { return kfifo_len(fifo) > fifo->mask; }

unsigned int kfifo_size(const struct kfifo *fifo) { return fifo->mask + 1; }

void kfifo_reset(struct kfifo *fifo)
{
    fifo->in  = 0;
    fifo->out = 0;
}

void kfifo_reset_out(struct kfifo *fifo) { fifo->out = fifo->in; }

unsigned int kfifo_put(struct kfifo *fifo, const void *element)
{
    if (kfifo_is_full(fifo)) return 0;

    unsigned int off = fifo->in & fifo->mask;
    memcpy(fifo->data + off * fifo->esize, element, fifo->esize);

    __DMB();
    fifo->in++;

    return 1;
}

unsigned int kfifo_get(struct kfifo *fifo, void *element)
{
    if (kfifo_is_empty(fifo)) return 0;

    unsigned int off = fifo->out & fifo->mask;
    memcpy(element, fifo->data + off * fifo->esize, fifo->esize);

    __DMB();
    fifo->out++;
    
    return 1;
}

unsigned int kfifo_peek(const struct kfifo *fifo, void *element)
{
    if (kfifo_is_empty(fifo)) return 0;

    unsigned int off = fifo->out & fifo->mask;
    memcpy(element, fifo->data + off * fifo->esize, fifo->esize);
    return 1;
}

unsigned int kfifo_in(struct kfifo *fifo, const void *buf, unsigned int n)
{
    unsigned int l = kfifo_avail(fifo);
    if (n > l) n = l;
    fifo_copy_in(fifo, buf, n, fifo->in);

    __DMB();
    fifo->in += n;
    
    return n;
}

unsigned int kfifo_out(struct kfifo *fifo, void *buf, unsigned int n)
{
    unsigned int l = kfifo_len(fifo);
    if (n > l) n = l;
    fifo_copy_out(fifo, buf, n, fifo->out);

    __DMB();
    fifo->out += n;
    
    return n;
}

unsigned int kfifo_out_peek(const struct kfifo *fifo, void *buf, unsigned int n)
{
    unsigned int l = kfifo_len(fifo);
    if (n > l) n = l;
    fifo_copy_out((struct kfifo *)fifo, buf, n, fifo->out);
    return n;
}
