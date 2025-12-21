#ifndef isr_h
#define isr_h

//Die tatsächlichen Funktionen sind nicht in C sonder in Assembler geschrieben!

int  interrupts_get_and_disable(void);
void interrupts_enable(void);
int  fiq_get_and_disable(void);
void fiq_enable(void);

void isr_undef(void);
void isr_swi(void);
void isr_pAbort(void);
void isr_dAbort(void);
void isr_irq(void);
void isr_fiq(void);

#endif
