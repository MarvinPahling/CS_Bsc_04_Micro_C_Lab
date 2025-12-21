/* Driver for the AT91SAM7's Advanced Interrupt Controller (AIC).
 *
 * The AIC is responsible for queuing interrupts from other
 * peripherals on the board. It then hands them one by one to the ARM
 * CPU core for handling, according to each peripheral's configured
 * priority.
 */

#include <stdlib.h>    //fuer NULL

#include "../AT91SAM7S64.h"
#include "term.h"
#include "aic.h"
#include "../main.h"   //fuer VT100_VORDERGRUND_ROT

uint8_t aic_intNest =0;  /* start with nesting level of 0 */
uint8_t aic_intMax  =0;  /* Max nexting int */

/* static */ struct {
	void        *sys_vl_head;
	aic_cb_fcn   commxx_cb;
	uint16_t     spurious_cnt;
} aic;


void __attribute__((noreturn)) Q_onAssert(char const * const file, uint32_t addr,int32_t swi_nr) 
{
#if defined(MODE_RAM) || defined(MODE_SIM) 
	interrupts_enable();  //Senderoutine benötigt IRQ, welche andernfalls deaktiviert ist!
#endif	
	(void)term_string(VT100_VORDERGRUND_ROT "!!!!!! ",ASYNCSYNC_BLOCK);
	(void)term_string(file,ASYNCSYNC_BLOCK);
	(void)term_string(" PC=",ASYNCSYNC_BLOCK);
	(void)term_hex(addr,8,ASYNCSYNC_BLOCK);
	if(swi_nr != -1) {
		(void)term_string(" SWI=",ASYNCSYNC_BLOCK);
		(void)term_hex(swi_nr,4,ASYNCSYNC_BLOCK);
	}
	//__builtin_trap();  //nur bei TrOnchip.undef
    while(1);
}

/* Default handler für nicht registierten IRQ-Interrupt */
void  __attribute__((weak,noreturn)) default_isr(void)
{
#if defined(MODE_RAM) || defined(MODE_SIM) 
	interrupts_enable();  //Senderoutine benötigt IRQ, der andernfalls deaktiviert ist!
#endif	
	(void)term_string("AIC-IRQ nicht zugeordnet",ASYNCSYNC_BLOCK);
	while(1);
}

/* Default handler für nicht registierten FIQ-Interrupt */
void  __attribute__((weak,noreturn)) default_fiq(void)
{
#if defined(MODE_RAM) || defined(MODE_SIM) 
	interrupts_enable();  //Senderoutine benötigt IRQ, der andernfalls deaktiviert ist!
#endif	
	(void)term_string("AIC-FIQ-IRQ nicht zugeordnet",ASYNCSYNC_BLOCK);
	while(1);
}

/* Default handler IRQ, für bspw. Level-Aktive IRQ, die zum Eintritt in die ISR
   geführt haben, zum Zeitpunkt der 'Ursachenauswertung' aber nicht mehr aktiv sind */
void  __attribute__((weak /*,noreturn*/)) spurious_isr(void)
{
#if 0
#if defined(MODE_RAM) || defined(MODE_SIM) 
	interrupts_enable();  //Senderoutine benötigt IRQ, der andernfalls deaktiviert ist!
#endif	
	(void)term_string("SPURIOUS-IRQ nicht zugeordnet",ASYNCSYNC_BLOCK);
#else
	aic.spurious_cnt++;
#endif	
}

void aic_init(void)
{
	aic.commxx_cb    = NULL;
	aic.spurious_cnt = 0;
	aic.sys_vl_head  = NULL;
	
	interrupts_get_and_disable();

	extern uint32_t ivt_app[2][8];  //Wird wahlweise über link.ld oder startup.s gesezt
	                                //Zeigt auf die Startadresse der IVT, welche zur 
	                                //Laufzeit beschrieben werden kann

	/* In die vom low_level_init() vorgefertigte Sprungtabelle die endgültigen Ziele eintragen */
	//ivt_app[1][0] = (uint32_t)_reset;  //Bereits gesetzt
	ivt_app[1][1] = (uint32_t)&isr_undef;
    ivt_app[1][2] = (uint32_t)&isr_swi;
    ivt_app[1][3] = (uint32_t)&isr_pAbort;
    ivt_app[1][4] = (uint32_t)&isr_dAbort;
    //ivt_app[1][5] =                    //Reserved
    ivt_app[1][6] = (uint32_t)&isr_irq;
    ivt_app[1][7] = (uint32_t)&isr_fiq;

	/* If we're coming from a warm boot, the AIC may be in a weird state.
	 * Do some cleaning up to bring the AIC back into a known state:
	 *  - All interrupt lines disabled,
	 *  - No interrupt lines handled by the FIQ handler,
	 *  - No pending interrupts,
	 *  - AIC idle, not handling an interrupt.
	*/
	AT91C_BASE_AIC->AIC_IDCR = 0xFFFFFFFF;    /* disable all interrupts */
	AT91C_BASE_AIC->AIC_FFDR = 0xFFFFFFFF;    /* disable fast forcing */
	AT91C_BASE_AIC->AIC_ICCR = 0xFFFFFFFF;    /* clear all interrupts */
	//AT91C_BASE_AIC->AIC_EOICR = 1;
	for(int i = 0; i < 8; ++i) {
		AT91C_BASE_AIC->AIC_EOICR = 0;        /* write AIC_EOICR 8 times */
	}

	/* Enable debug protection. This is necessary for JTAG debugging, so
	 * that the hardware debugger can read AIC registers without
	 * triggering side-effects.
	 * Inside ISR write after read of aic_ivr is now necessary!
	*/
	AT91C_BASE_AIC->AIC_DCR = AT91C_AIC_DCR_PROT;

	/* Set default handlers for all interrupt lines. */
	for(int i = 0; i < 32; i++) {
		AT91C_BASE_AIC->AIC_SMR[i] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | AIC_INT_LEVEL_LOWEST;
		AT91C_BASE_AIC->AIC_SVR[i] = (uint32_t) default_isr;
	}
	/* Macht eigentlich  keinen Sinn, da der FIQ über eigenen 'PIN' reinkommt*/
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_FIQ] = (uint32_t) default_fiq;
	AT91C_BASE_AIC->AIC_SPU               = (uint32_t) spurious_isr;
}

/* Register an fiq service routine */
/* NOTE: Do NOT enable interrupts throughout the whole FIQ processing. */
/* NOTE: FIQ ISR noch nicht getestet !!!!!!!! */
void aic_set_fiq(aic_cb_fcn isr)
{
	AT91C_BASE_AIC->AIC_SVR[AT91C_ID_FIQ] = (uint32_t) isr;
}

/* Register an interrupt service routine for an interrupt line */
void aic_set_vector(uint32_t vector, uint32_t mode, aic_cb_fcn isr)
{
	if (vector < 32) {
		int i_state = interrupts_get_and_disable();

		AT91C_BASE_AIC->AIC_SMR[vector] = mode;
		AT91C_BASE_AIC->AIC_SVR[vector] = (uint32_t)isr;
		
		if(i_state)
			interrupts_enable();
	}
}

/* Enable handling of an interrupt line in the AIC */
void aic_mask_on(uint32_t vector)
{
	if (vector < 32) {
		int i_state = interrupts_get_and_disable();

		AT91C_BASE_AIC->AIC_IECR = (1 << vector);
	
		if(i_state)
			interrupts_enable();
	}
}

/* Disable handling of an interrupt line in the AIC */
void aic_mask_off(uint32_t vector)
{
	if (vector < 32) {
		int i_state = interrupts_get_and_disable();

		AT91C_BASE_AIC->AIC_IDCR = (1 << vector);
		
		if(i_state)
			interrupts_enable();
	}
}

/* Clear an interrupt line in the AIC */
void aic_clear(uint32_t vector)
{
	if (vector < 32) {
		int i_state = interrupts_get_and_disable();

		AT91C_BASE_AIC->AIC_ICCR = (1 << vector);
		
		if(i_state)
			interrupts_enable();
	}
}

/************************************************************/
void aic_sys_register_commxx(aic_cb_fcn fcn)
{
	aic.commxx_cb = fcn;
}

void aic_sys_register_pit(aic_sys_vl_t *append,aic_cb_fcn fcn)
{
	aic_sys_vl_t *ptr;
	
	/* Ende der verketteten Liste suchen */
	for(ptr=(aic_sys_vl_t *)&aic.sys_vl_head;
	    ptr->next!=NULL;
		ptr=ptr->next);
	
	/* Neues Element initialisieren */
	append->fcn =fcn;
	append->next=NULL;
	
	/* Neues Element am Ende anhängen */
	ptr->next=append;
}

__attribute__ ((section (".text.fastcode")))
void aic_sys_isr_entry(void)
{
	//Der System-Interrupt ergibt sich aus der Verorderung von
	// - PIT
	// - RTT (Real Time Timer)
	// - WDT (Watchdog)
	// - DBGU (Debug Unit)
	// - PMC (Power Management Controller)
	// - RSTC (Reset Controller)
	
	if(*AT91C_PITC_PISR & AT91C_PITC_PITS) {
		aic_sys_vl_t *ptr;
		//Callback Funktionen aufrufen
		//Interruptquelle (PIVR) muss in einer der Callback Routinen zurückgesetzt werden!
		for(ptr=(aic_sys_vl_t *)aic.sys_vl_head;
			ptr!=NULL;
			ptr=ptr->next)
			ptr->fcn();
	}
//	if(*AT91C_RTTC_RTSR & (AT91C_RTTC_ALMS | AT91C_RTTC_RTTINC)) {
//	}
//  if(*AT91C_WDTC_WDSR & (AT91C_WDTC_WDUNF | AT91C_WDTC_WDERR)) {
//  }
	if((aic.commxx_cb) && (*AT91C_DBGU_CSR & *AT91C_DBGU_IMR & (AT91C_US_COMM_TX | AT91C_US_COMM_RX)))
		aic.commxx_cb();
//  if(*AT91C_DBGU_CSR & ()) {
//  }
//  if(*AT91C_PMC_SR & ()) {
//  }	
//  if(*AT91C_RSTC_RSR & ()) {
//  }	
}

void aic_sys_init(uint32_t srctype,uint32_t level)
{
	int i_state = interrupts_get_and_disable();

	//Flankengesteuert funktioniert bei SYS nicht, da mehrere Quellen das FF setzen.
	aic_mask_off  (AT91C_ID_SYS);
	aic_set_vector(AT91C_ID_SYS, srctype |  level, aic_sys_isr_entry);
	aic_clear     (AT91C_ID_SYS);
	aic_mask_on   (AT91C_ID_SYS);
	
	if (i_state)
		interrupts_enable();
}
