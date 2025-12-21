/*****************************************************************************
* Product:  QK port to ARM, QK, with GNU toolset
* Last Updated for Version: 4.3.00
* Date of the Last Update:  Nov 06, 2011
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2002-2011 Quantum Leaps, LLC. All rights reserved.
*
* This software may be distributed and modified under the terms of the GNU
* General Public License version 2 (GPL) as published by the Free Software
* Foundation and appearing in the file GPL.TXT included in the packaging of
* this file. Please note that GPL Section 2[b] requires that all works based
* on this software must also be made publicly available under the terms of
* the GPL ("Copyleft").
*
* Alternatively, this software may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GPL and are specifically designed for licensees interested in
* retaining the proprietary status of their code.
*
* Contact information:
* Quantum Leaps Web site:  http://www.quantum-leaps.com
* e-mail:                  info@quantum-leaps.com
*****************************************************************************/

    .equ    NO_IRQ,      0x80   /* mask to disable interrupts (FIQ and IRQ) */
    .equ    NO_FIQ,      0x40   /* mask to disable interrupts (FIQ and IRQ) */
    .equ    FIQ_MODE,    0x11
    .equ    IRQ_MODE,    0x12
    .equ    SYS_MODE,    0x1F

    .equ    FIQ_PRIO,    0xFF
    .equ    IRQ_PRIO,    (0xFF - 1)

    .text
    .arm

/*****************************************************************************
* int interrupts_get_and_disable(void);
  - Disables interrupts. 
  - Returns zero if interupts were previously off
*/
    .global interrupts_get_and_disable
    .func   interrupts_get_and_disable
interrupts_get_and_disable:
		mrs	    r0, cpsr
		ands    r0, r0, #0x80
		movne   r0, #0
		bxne    lr
		mrs	    r0, cpsr
		orr	    r0,r0,#0x80
		msr	    cpsr_c,r0
		mov     r0,#1
		bx	    lr

    .size   interrupts_get_and_disable, . - interrupts_get_and_disable
    .endfunc

/*****************************************************************************
* void interrupts_enable(void);
  - Enable Interrupts
*/
    .global interrupts_enable
    .func   interrupts_enable
interrupts_enable:
		mrs	r0, cpsr
		bic	r0,r0,#0x80
		msr	cpsr_c,r0
		bx	lr

    .size   interrupts_enable, . - interrupts_enable
    .endfunc

/*****************************************************************************
* int fiq_get_and_disable(void);
  - Disables FIQ-interrupts. 
  - Returns zero if FIQ-interupts were previously off
*/
    .global fiq_get_and_disable
    .func   fiq_get_and_disable
fiq_get_and_disable:
		mrs	    r0, cpsr
		ands    r0, r0, #0x40
		movne   r0, #0
		bxne    lr
		mrs	    r0, cpsr
		orr	    r0,r0,#0x80
		msr	    cpsr_c,r0
		mov     r0,#1
		bx	    lr

    .size   fiq_get_and_disable, . - fiq_get_and_disable
    .endfunc

/*****************************************************************************
* void fiq_enable(void);
  - enable FIQ-Interrupt
*/
    .global fiq_enable
    .func   fiq_enable
fiq_enable:
		mrs	r0, cpsr
		bic	r0,r0,#0x40
		msr	cpsr_c,r0
		bx	lr

    .size   fiq_enable, . - fiq_enable
    .endfunc

/*****************************************************************************
* void isr_undef(void);
*/
    .global isr_undef
    .func   isr_undef
    .align  3
isr_undef:
    LDR     r0,=CString_undef
    SUB     r1,lr,#4            /* set line number to the exception address */
    mvn	    r2, #0x00           /* 0xffffffff */
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) /* SYSTEM mode, IRQ/FIQ disabled */
    LDR     r12,=Q_onAssert
    MOV     lr,pc               /* store the return address */
    BX      r12                 /* call the assertion-handler (ARM/THUMB) */
    /* the assertion handler should not return, but in case it does
     * hang up the machine in this endless loop */
    B       .

CString_undef:       .string  "isr_undef()"

    .size   isr_undef, . - isr_undef
    .endfunc

/*****************************************************************************
* void isr_swi(void);
*/
    .global isr_swi
    .func   isr_swi
    .align  3
isr_swi:
    LDR     r0,=CString_swi
    SUB     r1,lr,#4            /* set line number to the exception address */
    ldr	    r2, [lr,#-4]     
	and     r2,r2,#0xFF
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) /* SYSTEM mode, IRQ/FIQ disabled */
    LDR     r12,=Q_onAssert
    MOV     lr,pc               /* store the return address */
    BX      r12                 /* call the assertion-handler (ARM/THUMB) */
    /* the assertion handler should not return, but in case it does
     * hang up the machine in this endless loop */
    B       .

CString_swi:         .string  "isr_swi()"

    .size   isr_swi, . - isr_swi
    .endfunc

/*****************************************************************************
* void isr_pAbort(void);
*/
    .global isr_pAbort
    .func   isr_pAbort
    .align  3
isr_pAbort:
    LDR     r0,=CString_pAbort
    SUB     r1,lr,#4  /*?8?*/   /* set line number to the exception address */
    mvn	    r2, #0x00           /* 0xffffffff */
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) /* SYSTEM mode, IRQ/FIQ disabled */
    LDR     r12,=Q_onAssert
    MOV     lr,pc               /* store the return address */
    BX      r12                 /* call the assertion-handler (ARM/THUMB) */
    /* the assertion handler should not return, but in case it does
     * hang up the machine in this endless loop */
    B       .

CString_pAbort:      .string  "isr_pAbort()"
	
    .size   isr_pAbort, . - isr_pAbort
    .endfunc

/*****************************************************************************
* void isr_dAbort(void);
*/
    .global isr_dAbort
    .func   isr_dAbort
    .align  3
isr_dAbort:
    LDR     r0,=CString_dAbort
    SUB     r1,lr,#8            /* set line number to the exception address */
    mvn	    r2, #0x00           /* 0xffffffff */
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) /* SYSTEM mode, IRQ/FIQ disabled */
    LDR     r12,=Q_onAssert
    MOV     lr,pc               /* store the return address */
    BX      r12                 /* call the assertion-handler (ARM/THUMB) */
    /* the assertion handler should not return, but in case it does
     * hang up the machine in this endless loop */
    B       .

CString_dAbort:      .string  "isr_dAbort()"
	
    .size   isr_dAbort, . - isr_dAbort
    .endfunc

/*****************************************************************************
* void isr_irq(void);
*/
    .global isr_irq
    .func   isr_irq
    .align  3

	// Funktionen im RAM ablegen, damit diese schneller ausgeführt werden!
    .section .text.fastcode
isr_irq:
	// Register sichern 
	// IRQ-Mode Aktiv, d.h. 
	// - R14_IRQ  enthält Rücksprungadresse
	// - R13_IRQ  frei verfügbar, da IRQ Stack nicht genutzt wird
	// - SPSR_IRQ enthält gesicherte CPSR
    MOV     r13,r0              /* save r0 in r13_IRQ */
    SUB     r0,lr,#4            /* put return address in r0_SYS */
    MOV     lr,r1               /* save r1 in r14_IRQ (lr) */
    MRS     r1,spsr             /* put the SPSR in r1_SYS */

    //Umschalten in SYS-Mode zwecks 
	// - sichern von LR (in R0) und SPSR (in R1) auf Stack
	// - sichern von R2,R3,R12,R14 auf Stack 
	//      R4-R11 werden, sofern nötig, von weiterverarbeitenden C-Funktionen gesichert
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ)
    STMFD   sp!,{r0,r1}         /* save SPSR and PC on SYS stack */
    STMFD   sp!,{r2-r3,r12,lr}  /* save APCS-clobbered regs on SYS stack */
    MOV     r0,sp               /* make the sp_SYS visible to IRQ mode */
    SUB     sp,sp,#(2*4)        /* make room for stacking (r0_SYS, r1_SYS) */

    //Umschalten in den IRQ-Modus zwecks
    // - sichern von R0 (in R13) und R1 (in R14) auf Stack
    MSR     cpsr_c,#(IRQ_MODE | NO_IRQ) 
    STMFD   r0!,{r13,r14}       /* finish saving the context (r0_SYS,r1_SYS)*/

    //Umschalten in den SYS-Mode
    // - da nachfolgende aic_intNest und aic_intMax bearbeitet werden
    //   welche auch vom FIQ 'benutzt' werden, FIQ deaktivieren
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) 

    //aic_intNest++;
    LDR     r0,=aic_intNest     /* load address in already saved r0 */
    LDRB    r12,[r0]            /* load original aic_intNest into saved r12 */
    ADD     r12,r12,#1          /* increment the nesting level */
    STRB    r12,[r0]            /* store the value in aic_intNest */
	
    //if(aic_intNest > aic_intMax) 
    //    aic_intMax=aic_intNest;
    LDR     r0,=aic_intMax
    LDRB    r1,[r0]
    CMP     r1,r12
    strltb  r12,[r0]

    MSR     cpsr_c,#(SYS_MODE | NO_IRQ)

	//Read the IVR (performs following operations within the aic)
	//- Calculate active interrupt (higher than current or spurious)
	//- Determines and returns the vector of the active interrupt
	//- Memorizes the interrupt
    mvn	    r0, #0xf00          /* 0xfffff0ff */
    ldr	    r1, [r0, #(0x100-0xff)]    /* vect=AT91C_BASE_AIC->AIC_IVR; */
    //Write to IVR (necessary, when AT91C_AIC_DCR_PROT in AIC_DCR is set)
	//- Pushes the current priority level onto the internal stack
	//- Acknowledge the interrupt
    str	    r1, [r0, #(0x100-0xff)]    /* AT91C_BASE_AIC->AIC_IVR=vect; */
	
    //IRQ Enable (Nesting Interrupts einschalten)
    MSR     cpsr_c,#(SYS_MODE)

	//if(vect !=spurious_isr)
//  ldr     r0, =spurious_isr
//  cmp     r0, r1
//  beq	    .L25

	//Call the function (*vect)();
//  push	{lr}
    mov     lr, pc
    bx	    r1
//  pop     {lr} 
	
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ)

    //Write AIC_EOICR to clear interrupt
	//- Restoring the previous current level if one exist on the stack
	//- if another interrupt is pending, the nIRQ Line is re-asserted,
	//  but the Interrupt sequence does not immedeatly start because
	//  the I-Bit is set in the core.
    mvn     r0, #0xf00          /* 0xfffff0ff */
    mov     r1, #0
    str     r1, [r0, #(0x130-0xff)]    /* AT91C_BASE_AIC->AIC_EOICR = 0 */

    //aic_intNest--;
    LDR     r0,=aic_intNest     /* load address */
    LDRB    r12,[r0]            /* load original aic_intNest into saved r12 */
    SUBS    r12,r12,#1          /* decrement the nesting level */
    STRB    r12,[r0]            /* store the value in aic_intNest */

    //Register rekonstruieren
    MOV     r0,sp               /* make sp_SYS visible to IRQ mode */
    ADD     sp,sp,#(8*4)        /* fake unstacking 8 registers from sp_SYS */

    MSR     cpsr_c,#(IRQ_MODE | NO_IRQ | NO_FIQ)
    MOV     sp,r0               /* copy sp_SYS to sp_IRQ */
    LDR     r0,[sp,#(7*4)]      /* load the saved SPSR from the stack */
    MSR     spsr_cxsf,r0        /* copy it into spsr_IRQ */

    LDMFD   sp,{r0-r3,r12,lr}^  /* unstack all saved USER/SYSTEM registers */
    NOP                         /* cant access banked reg immediately */
    LDR     lr,[sp,#(6*4)]      /* load return address from the SYS stack */
    MOVS    pc,lr               /* return restoring CPSR from SPSR */

    .size   isr_irq, . - isr_irq
    .endfunc


/*****************************************************************************
* void isr_fiq(void);
*/
    .global isr_fiq
    .func   isr_fiq
    .align  3

	// Funktionen im RAM ablegen, damit diese schneller ausgeführt werden!
    .section .text.fastcode

	// NOTE: In contrast to the IRQ line, the FIQ line is NOT prioritized
	//       by the AIC. Therefore, you must NOT enable interrupts while
	//       processing FIQ. All FIQs should be the highest-priority in
	//       the system. All FIQs run at the same (highest) priority level.
isr_fiq:
	// Register sichern 
	// FIQ-Mode Aktiv, d.h. 
	// - R14_FIQ  enthält Rücksprungadresse
	// - R13_FIQ  frei verfügbar, da FIQ Stack nicht genutzt wird
	// - R12_FIQ..R8_FIQ frei verfügbar
	// - SPSR_FIQ enthält gesicherte CPSR
    MOV     r13,r0              /* save r0 in r13_FIQ */
    SUB     r0,lr,#4            /* put return address in r0_SYS */
    MOV     lr,r1               /* save r1 in r14_FIQ (lr) */
    MRS     r1,spsr             /* put the SPSR in r1_SYS */

    //Umschalten in SYS-Mode zwecks 
	// - sichern von LR (in R0) und SPSR (in R1) auf Stack
	// - sichern von R2,R3,R12,R14 auf Stack 
	//      R4-R11 werden, sofern nötig, von weiterverarbeitenden C-Funktionen gesichert
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ)
    STMFD   sp!,{r0,r1}         /* save SPSR and PC on SYS stack */
    STMFD   sp!,{r2-r3,r12,lr}  /* save APCS-clobbered regs on SYS stack */
    MOV     r0,sp               /* make the sp_SYS visible to FIQ mode */
    SUB     sp,sp,#(2*4)        /* make room for stacking (r0_SYS, SPSR) */

	//Umschalten in den FIQ-Modus zwecks
	// - sichern von R0 (in R13) und R1 (in R14) auf Stack
    MSR     cpsr_c,#(FIQ_MODE | NO_IRQ | NO_FIQ) /* FIQ mode, IRQ/FIQ disabled */
    STMFD   r0!,{r13,r14}       /* finish saving the context (r0_SYS,r1_SYS)*/


	//Umschalten in den SYS-Mode
	// - da nachfolgende aic_intNest und aic_intMax bearbeitet werden
	//   welche auch von IRQ 'benutzt' werden, IRQ deaktivieren
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ | NO_FIQ) 

	/* ISR Zusatzfunktionen */
	//aic_intNest++;
    LDR     r0,=aic_intNest      /* load address in already saved r0 */
    LDRB    r12,[r0]            /* load original aic_intNest into saved r12 */
    ADD     r12,r12,#1          /* increment interrupt nesting */
    STRB    r12,[r0]            /* store the value in aic_intNest */
	
	//if(aic_intNest > aic_intMax) 
	//    aic_intMax=aic_intNest;
    LDR     r0,=aic_intMax
	LDRB    r1,[r0]
	CMP     r1,r12
	STRLTB  r12,[r0]

	//Read the IVR
    mvn	    r0, #0xf00          /* 0xfffff0ff */
    ldr	    r1, [r0, #(0x104-0xff)]    /* vect=AT91C_BASE_AIC->AIC_FVR; */

	//Call the function (*vect)();
//  push	{lr}
    mov     lr, pc
    bx	    r1
//  pop     {lr} 
	
    MSR     cpsr_c,#(SYS_MODE | NO_IRQ| NO_FIQ)

	//ISR Zusatzfunktionen */
	//aic_intNest--;
    LDR     r0,=aic_intNest     /* load address */
    LDRB    r12,[r0]            /* load original aic_intNest into saved r12 */
    SUBS    r12,r12,#1          /* decrement the nesting level */
    STRB    r12,[r0]            /* store the value in aic_intNest */
	
	//Register rekonstruieren
    MOV     r0,sp               /* make sp_SYS visible to FIQ mode */
    ADD     sp,sp,#(8*4)        /* fake unstacking 8 registers from sp_SYS */

    MSR     cpsr_c,#(FIQ_MODE | NO_IRQ | NO_FIQ) /* FIQ mode, IRQ/FIQ disabled */
    MOV     sp,r0               /* copy sp_SYS to sp_FIQ */
    LDR     r0,[sp,#(7*4)]      /* load the saved SPSR from the stack */
    MSR     spsr_cxsf,r0        /* copy it into spsr_FIQ */

    LDMFD   sp,{r0-r3,r12,lr}^  /* unstack all saved USER/SYSTEM registers */
    NOP                         /* cant access banked reg immediately */
    LDR     lr,[sp,#(6*4)]      /* load return address from the SYS stack */
    MOVS    pc,lr               /* return restoring CPSR from SPSR */

    .size   isr_fiq, . - isr_fiq
    .endfunc

    .end
