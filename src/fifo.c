#include "fifo.h"

void fifo_init(fifo_t *fifo, char *buf, int buf_len)
{
  fifo->head = 0;
  fifo->tail = 0;
  fifo->buf = buf;
  fifo->len = buf_len;
}

int fifo_put(fifo_t *fifo, char c)
{
  int next;

  // check if FIFO has room
  next = (fifo->head + 1) % fifo->len;
  if (next == fifo->tail) {
    // full
    return -1;
  }

  fifo->buf[fifo->head] = c;
  fifo->head = next;

  return 0;
}


int fifo_get(fifo_t *fifo, char *pc)
{
  int next;

  // check if FIFO has data
  if (fifo->head == fifo->tail) {
    return -1;
  }

  next = (fifo->tail + 1) % fifo->len;

  *pc = fifo->buf[fifo->tail];
  fifo->tail = next;

  return 0;
}


int fifo_avail(fifo_t *fifo)
{
  return (fifo->len + fifo->head - fifo->tail) % fifo->len;
}


int fifo_free(fifo_t *fifo)
{
  return (fifo->len - 1 - fifo_avail(fifo));
}

