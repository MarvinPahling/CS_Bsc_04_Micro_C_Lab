#ifndef  __SYSTICK_H__
#define __SYSTICK_H__

#include <stdint.h>


typedef struct 
{
	uint32_t ms;
	uint32_t cpiv;    //0...((MCK / 16 / BSP_TICKS_PER_SEC)-1)=((47923200/16/1000) --> 0...2994
} SYSTICK_MS_CPIV;

void     systick_init(void);
uint32_t systick_get_ms(void);
void     systick_get_ms_cpiv(SYSTICK_MS_CPIV *ptr);
void     systick_wait_ms(int32_t ms);
void     systick_wait_ns(uint32_t ns);

#endif

