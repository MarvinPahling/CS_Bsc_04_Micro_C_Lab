#ifndef udmon3_h
#define udmon3_h

#include "../lib/byte_fifo_cb.h"        //fuer byte_fifo_t bytefifo_...()

void udmon3_init(void);

void udmon3_term_init(byte_fifo_cb_t *recv,byte_fifo_cb_t *send,byte_fifo_cb *tx_cb);

#endif