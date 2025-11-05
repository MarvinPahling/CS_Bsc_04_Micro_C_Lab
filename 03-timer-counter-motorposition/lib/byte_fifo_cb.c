/* Byte FIFO with CallBack
 *
 * Appends the Byte_Fifo with a callback function, which is called
 * when putting some stuff into the fifo (e.g. Enable transmitting interrupt)
 */
#include <string.h>

#include "byte_fifo_cb.h"

/* Return: Anzahl der nicht verarbeiteten Zeichen */
size_t byte_fifo_cb_putsize(byte_fifo_cb_t *fifo,const unsigned char *start,size_t len)
{
	size_t remain = byte_fifo_remain(&fifo->byte_fifo);
    size_t ret    = len;
    len = len <= remain ? len : remain;
	if(fifo->byte_fifo.wr + len <= fifo->byte_fifo.mask) {
		memcpy(fifo->byte_fifo.buf+fifo->byte_fifo.wr,start,len);
		fifo->byte_fifo.wr+=len;
		if(fifo->cb)
			fifo->cb();
	}
	else {
		size_t len_to_end=fifo->byte_fifo.mask - fifo->byte_fifo.wr + 1;
		memcpy(fifo->byte_fifo.buf+fifo->byte_fifo.wr,start+0         ,    len_to_end);
		memcpy(fifo->byte_fifo.buf+0                 ,start+len_to_end,len-len_to_end);
		fifo->byte_fifo.wr = (fifo->byte_fifo.wr + len) & fifo->byte_fifo.mask;
		if(fifo->cb)
			fifo->cb();
	}
	return ret-len;
}

#if 0
void debug(byte_fifo_cb_t *fifo) {
    printf("rd=%d wr=%d remain=%d used=%d\n",fifo->byte_fifo.rd,
                                             fifo->byte_fifo.wr,
                                              byte_fifo_remain(&fifo->byte_fifo),
                                              byte_fifo_used(&fifo->byte_fifo));
    for(size_t pos=0;pos<=fifo->byte_fifo.mask;pos++)
        printf("%02x%s",fifo->byte_fifo.buf[pos],pos==fifo->byte_fifo.mask?"\n":" ");
    for(size_t pos=0;pos<=fifo->byte_fifo.mask;pos++) {
        if((pos == fifo->byte_fifo.rd) && (pos == fifo->byte_fifo.wr))
            printf("RW ");
        else if(pos == fifo->byte_fifo.rd)
            printf("RD ");
        else if(pos == fifo->byte_fifo.wr)
            printf("WR ");
        else
            printf("   ");
    }
    printf("\n");
}
#endif
