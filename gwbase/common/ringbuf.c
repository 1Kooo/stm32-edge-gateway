/**
 * @file    ringbuf.c
 * @brief   Single-producer / single-consumer byte ring buffer.
 *
 * Capacity note: one byte is always left unused so that head == tail can
 * unambiguously mean "empty". Effective capacity is size - 1.
 */

#include "ringbuf.h"

int rb_init(ringbuf_t *rb, uint8_t *storage, uint16_t size)
{
    if (rb == 0 || storage == 0 || size < 2)  return -1;
    if ((size & (size - 1)) != 0)             return -1;   /* not a power of two */

    rb->buf      = storage;
    rb->size     = size;
    rb->mask     = (uint16_t)(size - 1);
    rb->head     = 0;
    rb->tail     = 0;
    rb->overflow = 0;
    return 0;
}

uint16_t rb_used(const ringbuf_t *rb)
{
    return (uint16_t)((rb->head - rb->tail) & rb->mask);
}

uint16_t rb_free(const ringbuf_t *rb)
{
    return (uint16_t)(rb->size - rb_used(rb) - 1);
}

uint16_t rb_write(ringbuf_t *rb, const uint8_t *data, uint16_t len)
{
    uint16_t written = 0;
    uint16_t space   = rb_free(rb);

    if (space == 0) {
        rb->overflow += len;
        return 0;
    }
    if (len > space) {
        rb->overflow += (uint32_t)(len - space);
        len = space;
    }

    while (written < len) {
        rb->buf[rb->head] = data[written++];
        rb->head = (uint16_t)((rb->head + 1) & rb->mask);
    }
    return written;
}

uint16_t rb_read(ringbuf_t *rb, uint8_t *dst, uint16_t maxlen)
{
    uint16_t n = 0;
    while (n < maxlen && rb->tail != rb->head) {
        dst[n++] = rb->buf[rb->tail];
        rb->tail = (uint16_t)((rb->tail + 1) & rb->mask);
    }
    return n;
}

void rb_flush(ringbuf_t *rb)
{
    rb->tail = rb->head;
}
