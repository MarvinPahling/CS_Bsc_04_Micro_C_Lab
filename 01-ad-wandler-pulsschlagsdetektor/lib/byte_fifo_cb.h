/* Byte FIFO with CallBack
 *
 * Appends the Byte_Fifo with a callback function, which is called
 * when putting some stuff into the fifo (e.g. Enable transmitting interrupt)
 */

#ifndef __BYTE_FIFO_CB_H__
#define __BYTE_FIFO_CB_H__

#include "byte_fifo.h"

typedef void (*byte_fifo_cb)(void);

typedef struct {
	byte_fifo_cb cb;
	byte_fifo_t  byte_fifo; //Aufgrund des Flexible Array Member hier hinter keine weiteren Strukturelemente!
} byte_fifo_cb_t;

#define BYTE_FIFO_CB_INIT(size,func)  (byte_fifo_cb_t) {.cb=func, \
                                                        .byte_fifo=BYTE_FIFO_INIT(size) \
									    			   }

size_t byte_fifo_cb_putsize(byte_fifo_cb_t *fifo,const unsigned char *start,size_t len);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static int byte_fifo_cb_put_possible(byte_fifo_cb_t *fifo)
{
	return byte_fifo_put_possible(&fifo->byte_fifo);
}
	
static int byte_fifo_cb_put(byte_fifo_cb_t *fifo,unsigned char val)
{
	int ret=byte_fifo_put(&fifo->byte_fifo,val);
	if( !ret && fifo->cb)
		fifo->cb();
	return ret;
}

static int byte_fifo_cb_get_possible(byte_fifo_cb_t *fifo)
{
	return byte_fifo_get_possible(&fifo->byte_fifo);
}

static int byte_fifo_cb_get(byte_fifo_cb_t *fifo,unsigned char *val)
{
	return byte_fifo_get(&fifo->byte_fifo,val);
}

#pragma GCC diagnostic pop
#endif
