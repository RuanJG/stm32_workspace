typedef struct {
  int         head;
  int         tail;
  char      *buf;
  int	      len;
} fifo_t;

void fifo_init(fifo_t *fifo, char *buf,int len);
int fifo_put(fifo_t *fifo, char c);
int fifo_get(fifo_t *fifo, char *pc);
int  fifo_avail(fifo_t *fifo);
int  fifo_free(fifo_t *fifo);
