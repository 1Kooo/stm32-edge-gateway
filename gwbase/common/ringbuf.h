/**
 * @file    ringbuf.h
 * @brief   Single-producer / single-consumer byte ring buffer.
 *
 * Design notes:
 *   - writer = UART RX interrupt only
 *   - reader = application task only
 *   - size must be a power of two (mask instead of modulo)
 *   - no critical section needed on Cortex-M4 for 16-bit indices:
 *     a single STRH/LDRH is atomic, and only one side writes each index
 */

#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

typedef struct {
    uint8_t          *buf;    /* caller owned storage          */
    uint16_t          size;   /* power of two                  */
    uint16_t          mask;   /* size - 1                      */
    volatile uint16_t head;   /* write index, only RX ISR touch*/
    volatile uint16_t tail;   /* read index, only reader touch */
    volatile uint32_t overflow; /* bytes dropped when full     */
} ringbuf_t;

/* Returns 0 on success, -1 on bad arguments (size not power of two). */
int      rb_init(ringbuf_t *rb, uint8_t *storage, uint16_t size);

/* Returns number of bytes actually written. */
uint16_t rb_write(ringbuf_t *rb, const uint8_t *data, uint16_t len);

/* Returns number of bytes actually read. */
uint16_t rb_read(ringbuf_t *rb, uint8_t *dst, uint16_t maxlen);

uint16_t rb_used(const ringbuf_t *rb);
uint16_t rb_free(const ringbuf_t *rb);
void     rb_flush(ringbuf_t *rb);

#endif /* __RINGBUF_H */
