#include <stdlib.h>			//Fuer NULL
#include "../AT91SAM7S64.h"
#include "systick.h"
#include "aic.h"            //Fuer aic_sys_register_pit() aic_sys_vl_t
#include "../main.h"

/*static*/ struct {
	aic_sys_vl_t          aic_sys_vl;
	volatile uint32_t     systick_ms;
} systick;

uint32_t systick_get_ms(void)
{
	//32-Bit Variable kann auf 32-Bit System in einen Rutsch gelesen werden
	return systick.systick_ms;
}

void systick_get_ms_cpiv(SYSTICK_MS_CPIV *ptr)
{
	//2x32-Bit Variablen bedingen beim Lesen besondere Sorgfalt 
	do {
		ptr->ms   =  systick.systick_ms;
		ptr->cpiv = (*AT91C_PITC_PIIR) & AT91C_PITC_CPIV;
	}
	while(ptr->ms != systick.systick_ms);
}

/* Aktives Warten auf Zeitablauf */
void systick_wait_ms(int32_t ms)
{

#if 1
	//Aufgrund des möglichen Überlaufes hier besondere vorsicht
	int32_t start=(int32_t)systick.systick_ms;
	while(((int32_t)systick.systick_ms-(int32_t)start)<ms);
	//sys_ms  start	(sys_ms-start)>=5		(sys_ms-start)>=7
	//	250   	250	(-6)-(-6)=0>=5=false	(-6)-(-6)=0>=7=false
	//	251 	250	(-5)-(-6)=1>=5=false	(-5)-(-6)=1>=7=false
	//	252		250	(-4)-(-6)=2>=5=false	(-4)-(-6)=2>=7=false
	//	253		250	(-3)-(-6)=3>=5=false	(-3)-(-6)=3>=7=false
	//	254		250	(-2)-(-6)=4>=5=false	(-2)-(-6)=4>=7=false
	//	255		250	(-1)-(-6)=5>=5=true		(-1)-(-6)=5>=7=false
	//	0		250	( 0)-(-6)=6>=5=true		( 0)-(-6)=6>=7=false
	//	1		250	( 1)-(-6)=7>=5=true		( 1)-(-6)=7>=7=true
	//	2		250	( 2)-(-6)=8>=5=true		( 2)-(-6)=8>=7=true
		  
#elif 0
	//So bitte nicht, da hier Probleme mit Überlauf vorhanden sind
	volatile uint32_t final = ms + systick.systick_ms;
	while (systick.systick_ms < final);
#elif 0
	volatile int i,k;
	for(k=ms; k>0; k--) {
		for(i=11000; i>0; i--);
	}
#endif
}

void systick_wait_ns(uint32_t ns)
{
	volatile uint32_t x = (ns >> 7) + 1;

	while (x) {
		x--;
	}
}

void systick_isr(void)
{
	uint32_t status;
	
	/* Read status to confirm interrupt */
	status = *AT91C_PITC_PIVR;

	// Update with number of ticks since last time
	systick.systick_ms += (status & AT91C_PITC_PICNT) >> 20;
}

/************************************************************/

void systick_init(void)
{
	uint32_t value=((MCK / 16 / BSP_TICKS_PER_SEC) - 1);
	
	aic_sys_register_pit(&systick.aic_sys_vl,systick_isr);
	aic_sys_init(AT91C_AIC_SRCTYPE_HIGH_LEVEL,AIC_INT_LEVEL_NORMAL);
	
	*AT91C_PITC_PIMR = AT91C_PITC_PITEN | AT91C_PITC_PITIEN | value;

}

