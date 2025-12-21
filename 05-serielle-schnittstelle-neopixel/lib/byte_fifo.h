/* Byte FIFO queue implementation.
 *
 * Implements a simple reader/write save byte FIFO on top of a data buffer, that lets
 * you enqueue and dequeue single bytes.
 */

#ifndef __BYTE_FIFO_H__
#define __BYTE_FIFO_H__


typedef struct
{
	unsigned short rd;
	unsigned short wr;
    unsigned short mask;
	unsigned char  buf[];
} byte_fifo_t;

#define TEST_2erPOTENZ(x) (((x) & ((x) - 1))?0:(x))
#define TEST_64k(x)       ((x)>0x10000?0:(x))
#define TEST_SIZE(x)      (TEST_64k(TEST_2erPOTENZ(x)))

//size muss einer 2er Potenzzahl sein, andernfalls gibt der Compiler
//die Fehlermeldung aus, dass die Array-Größe (hier -1) ungültit ist
#define BYTE_FIFO_INIT(size)  (byte_fifo_t) {.rd=0,\
                                             .wr=0,\
						        	         .mask=TEST_SIZE(size)-1,\
								    		 .buf={[TEST_SIZE(size)-1]=0}\
									    	}

#define BYTE_FIFO_TYPEOF_RDWR typeof (((byte_fifo_t *)0)->rd)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static int byte_fifo_put_possible(byte_fifo_t *fifo)
{
	if(((fifo->wr+1)&(fifo->mask)) == fifo->rd)
		return -1;
	
	return 0;
}

static BYTE_FIFO_TYPEOF_RDWR byte_fifo_remain(byte_fifo_t *fifo)
{
	return fifo->mask - ((fifo->wr-fifo->rd)&fifo->mask);
}
	
static BYTE_FIFO_TYPEOF_RDWR byte_fifo_used(byte_fifo_t *fifo)
{
	return (fifo->wr-fifo->rd)&fifo->mask;
}
	
static int byte_fifo_put(byte_fifo_t *fifo,unsigned char val)
{
	if(((fifo->wr+1)&(fifo->mask)) == fifo->rd)
		return -1;
	
	fifo->buf[fifo->wr]=val;
	fifo->wr=(fifo->wr+1)&(fifo->mask);
	return 0;
}

static int byte_fifo_get_possible(byte_fifo_t *fifo)
{
	if(fifo->rd == fifo->wr)
		return -1;
		
	return 0;
}

static int byte_fifo_get(byte_fifo_t *fifo,unsigned char *val)
{
	if(fifo->rd == fifo->wr)
		return -1;
		
	*val=fifo->buf[fifo->rd];
	fifo->rd=(fifo->rd+1)&(fifo->mask);
	return 0;
}
#pragma GCC diagnostic pop

#endif