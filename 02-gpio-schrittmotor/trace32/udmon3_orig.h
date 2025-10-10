#ifndef udmon3_h
#define udmon3_h

#include "../lib/byte_fifo.h"        //fuer byte_fifo_t bytefifo_...()

void udmon3_init(void);

void udmon3_term_init(byte_fifo_t *recv,byte_fifo_t *send);

#endif