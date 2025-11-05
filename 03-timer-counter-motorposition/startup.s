/*****************************************************************************
* Product: QDK-ARM
* Last Updated for Version: 4.1.03
* Date of the Last Update:  Mar 11, 2010
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2002-2010 Quantum Leaps, LLC. All rights reserved.
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

/*****************************************************************************
  CPSR=Current Programm Status Word
  SPSR=Saved Programm Status Word
  User mode: The only non-privileged mode. (Kein Zugriff auf CPSR)
  FIQ mode: A privileged mode that is entered whenever the processor accepts a fast interrupt request.
  IRQ mode: A privileged mode that is entered whenever the processor accepts an interrupt.
  Supervisor (svc) mode: A privileged mode entered whenever the CPU is reset or when an SVC instruction is executed.
  Abort mode: A privileged mode that is entered whenever a prefetch abort or data abort exception occurs.
  Undefined mode: A privileged mode that is entered whenever an undefined instruction exception occurs.
  System mode: The only privileged mode that is not entered by an exception. It can only be entered by executing an instruction that explicitly writes to the mode bits of the Current Program Status Register (CPSR) from another privileged mode (not from user mode).
  
  CPSR 31 30 29 28 27 .. 24 .. 19-16 .. 9 8 7 6 5 4-0
        |  |  |  |  |     +-> J Jazelle | | | | |  +--> Mode 10000=User-Mode R0..R14
        |  |  |  |  +-----> Q Underflow | | | | |            10001=FIQ-Mode  R8..R14,SPSR
        |  |  |  |            Saturation| | | | |            10010=IRQ-Mode  R13..R14,SPSR
        |  |  |  +--------> V Overflow  | | | | |            10011=SVC-Mode  R13..R14,SPSR
        |  |  +-----------> C carry     | | | | |            10111=ABORT-Mode R13..R14,SPSR
        |  +--------------> Z Zero      | | | | |            11011=UNDEF-Mode R13..R14,SPSR
        +-----------------> N Negative  | | | | |            11111=Sys-Mode  R0..R14
                                        | | | | +-----> T Thumb-Mode
                                        | | | +-------> F When set, disable FIQ interrupts
                                        | | +---------> I When set, disable IRQ interrupts
                                        | +-----------> A When set, disable imprecise aborts
                                        +-------------> Endianess, 1=Big or 0=Little
  c, x, s and f postifxes refers to the different parts of the status registers:
  cpsr_c = Control  (Mode+Interrupt+Thumb)
  cpsr_x = eXtension
  cpsr_s = Status
  cpsr_f = Flags (NZCVQ Flags)
*******************************************************************************
The ARM architecture (pre-ARMv8) provides a non-intrusive way of extending the instruction set 
using "coprocessors" that can be addressed using MCR, MRC, MRRC, MCRR and similar instructions. 
The coprocessor space is divided logically into 16 coprocessors with numbers from 0 to 15, 
coprocessor 15 (cp15) being reserved for some typical control functions like managing 
the caches and MMU operation on processors that have one.
In ARM-based machines, peripheral devices are usually attached to the processor by mapping 
their physical registers into ARM memory space, into the coprocessor space, or by connecting 
to another device (a bus) that in turn attaches to the processor. Coprocessor accesses have 
lower latency, so some peripherals—for example, an XScale interrupt controller—are accessible
in both ways: through memory and through coprocessors.
******************************************************************************/
/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs */

    .equ    I_BIT,          0x80      /* when I bit is set, IRQ is disabled */
    .equ    F_BIT,          0x40      /* when F bit is set, FIQ is disabled */

    .equ    USR_MODE,       0x10      /* User-Mode */
    .equ    FIQ_MODE,       0x11      /* FIQ-Mode  */
    .equ    IRQ_MODE,       0x12      /* IRQ-Mode  */
    .equ    SVC_MODE,       0x13      /* Supervisor-Mode */
    .equ    ABT_MODE,       0x17      /* Abort-Mode */
    .equ    UND_MODE,       0x1B      /* Undefined-Mode */
    .equ    SYS_MODE,       0x1F      /* SystemMode*/

/* constant to pre-fill the stack */
    .equ    STACK_FILL,     0xAAAAAAAA

/*****************************************************************************/
    .section .text.ivt_boot,"ax",%progbits
    .code 32

    .global ivt_boot    /* Damit dieses Label in der Symboltabelle erscheint */
    .func   ivt_boot
	
ivt_boot:

/* Sprungtabelle / IVT
 * - Sprungtabelle steht bei Programmstart im ROM (ReadOnlyMemory)
 *   - Normalerweise sollten hier folglich alle IVT-Einträge mit dem endügltigen Ziel 
 *     belegt sein (Ausnahme im Simulationsmodus, hier könnte die Sprungtabelle zur Laufzeit
 *     beschrieben werden!)
 *   - Der AT91SAM7 beherrscht Memory Remap, bei welcher 
 *     - der Internal Flash (0x10.0000..0x1F.FFFF) nach 0x00.0000..0x0F.FFFF) eingeblendet wird
 *       Default nach PowerOn
 *     - der Internal SRAM (0x20.0000..0x2F.FFFF) nach 0x00.0000..0x0F.FFFF) eingeblendet wird
 *   - Mit dem Memory Remap wird auch die IVT neu gesetzt. Dies ermöglicht
 *     - dass die IVT zur Laufzeit neu gesetzt werden kann (so dass die Applikation eine
 *       vom Bootloader unabhängige IVT besitzen kann)
 *     - die Einträge zur Laufzeit geändert werden können
 * - In der hiesigen Implementierung wird Memory Remap genutzt, so dass zwischen zwei IVTs
 *   unterschieden wird:
 *   - eine für den Boot-Vorgang, im welchen einzig der Reset-Vektor eingetragen ist 
 *     (vorrangig im ROM platziert und hier über startup.s definiert)
 *   - eine für die Applikation, in welcher zur Laufzeit die Vektoren eingetragen werden 
 *     (vorrangig im RAM platziert und über low_level_init.c und aic.c initialisiert)
 * --------------------------------------------------------------------------------------------
 * Im ROM-Modus erfolgt der Programmstart über den Reset-Vektor. Da diese im ROM liegt
 *   und zur Laufzeit nicht geändert werden kann wird ergänzend im RAM Speicherplatz für die
 *   Appliatkons-IVT vorgehalten
 * IM RAM-Modus erfoglt der Programmstart über den Debugger aus dem RAM heraus (Entry-Eintrag
 *   im LinkerSkript). Da die Boot-IVT folglich im RAM liegt, kann diese für die Applikations-IVT
 *   weiterverwendet werden.
 * Im SIM-Modus erfolgt der Programmstart über die Speicheradresse 0. Da im SIM-Modus nicht zwischen
 *   RAM und ROM unterschieden wird, kann die IVT auch beschrieben werden, so dass auch hier
 *   die Boot-IVT weiterverwendet werden kann (ein Memory-Remap wird hier ebenfalls nicht unterstützt)
 * Der GDB-Modus entsptricht dem RAM-Modus
 * Im Samba-Modus werden die ersten 0x3000 Byte im RAM vom Samba-Loader verwendet, so dass die Boot-IVT
 *   erst ab 0x20.3000 liegen kann. Zum korrekten Start der geladenen Applikation muss die 
 *   Startadresse händisch eingegeben werden (wird über makefile ausgegeben). Mit dem Start der 
 *   Applikation können die ersten 0x3000 Bytes von der Applikation genutzt werden, so dass 
 *   hier wie im ROM-Modus getrennt Speicher für die Boot- und die Applikations-IVT reserviert
 *   werden muss.
*/
 .ifdef MODE_RAM
    .global ivt_app
ivt_app:       //uint32_t ivt_app[2][8]; Gemeinsame Nutzung der IVT als Boot- und Applikations-IVT
.endif
.ifdef MODE_SIM
    .global ivt_app
ivt_app:       //uint32_t ivt_app[2][8]; Gemeinsame Nutzung der IVT als Boot- und Applikations-IVT
.endif
.ifdef MODE_GDBOPENOCD_RAM
    .global ivt_app
ivt_app:       //uint32_t ivt_app[2][8]; Gemeinsame Nutzung der IVT als Boot- und Applikations-IVT
.endif

/* Aufbau der IVT 
 * - Sollte Sprungbefehle enthalten, wahlweise mit absolutem oder relativen Sprungziel
 *   - Relatives Sprungziel 'b offset' -> Since the offset value is a signed 24-bit value, 
 *     the branch target must be within approximately +/-32MB (2^23<<2)
 *   - Absolutes Sprungziel 'lr pc,[pc,#0x20-8]' -> Absolutes Sprungziel steht 0x20 Bytes 
 *     von diesem Befehl entfernt. Daher separate Tabelle mit im Anschluss an IVT nötig
*/
    ldr   pc,[pc,#0x20-8]     /* Reset: Absoluter Sprung */
    B       .                 /* Undefined Instruction */
    B       .                 /* Software Interrupt    */
    B       .                 /* Prefetch Abort        */
    B       .                 /* Data Abort            */
    .word 0xBEEFDEAD          /* Reserved              */
    B       .                 /* IRQ                   */
    B       .                 /* FIQ                   */
	
	.word _reset  /* Absolute Adresse, so das im ROM-Modus zu 0x10.00xx gesprungen wird */
	.word 0x04    /* Wird bei relativen Sprung nicht benötigt */
	.word 0x08    /* Dennoch hier zwecks Speicherplatzreservierung */
	.word 0x0C
	.word 0x10
	.word 0xBEEFDEAD
	.word 0x18
	.word 0x1C

    .size   ivt_boot, . - ivt_boot   /* Tatsächliche Speichergröße für das Symbol */
	                                 /* ivt_boot ermitteln */
    .endfunc	


/*****************************************************************************
* _reset
*/
	.text
	.code 32
	.global _reset  /* Damit dieses Label in der Symboltabelle erscheint */
    .func   _reset  /* damit im MAP-File dargestellt wird                */

	/* Copyright Notice */
    .string "Copyright (c) OSTFALIA Justen "
    .align 2                            /* re-align to the word boundary */

_reset:

    /* Call the platform-specific low-level initialization routine
    * NOTE: The function low_level_init() cannot rely
    * on uninitialized data being cleared and cannot use any initialized
    * data, because the .bss and .data sections have not been initialized yet.
    */
	
    LDR     r0,=_reset         /* Übergabe der Adresse dieser funktion als erster Parameter */
    LDR     lr,=_cstartup      /* Händisches setzen der Rücksprungadresse */
    LDR     sp,=__stack_end__  /* set the temporary stack pointer */
    LDR     r12,=low_level_init 
    BX      r12


_cstartup:

.ifdef MODE_ROM
    /* Relocate .fastcode section (copy from ROM to RAM) */
    LDR     r0,=__fastcode_load
    LDR     r1,=__fastcode_start
    LDR     r2,=__fastcode_end
1:
    CMP     r1,r2
    LDMLTIA r0!,{r3}
    STMLTIA r1!,{r3}
    BLT     1b
.endif

.ifdef MODE_ROM
    /* Relocate the .data section (copy from ROM to RAM) */
    LDR     r0,=__data_load
    LDR     r1,=__data_start
    LDR     r2,=_edata
1:
    CMP     r1,r2
    LDMLTIA r0!,{r3}
    STMLTIA r1!,{r3}
    BLT     1b
.endif

    /* Clear the .bss section (zero init) */
    LDR     r1,=__bss_start__
    LDR     r2,=__bss_end__
    MOV     r3,#0
1:
    CMP     r1,r2
    STMLTIA r1!,{r3}
    BLT     1b

dummy:
    /* Fill the .stack section */
    LDR     r1,=__stack_start__
    LDR     r2,=__stack_end__
    LDR     r3,=STACK_FILL
1:
    CMP     r1,r2
    STMLTIA r1!,{r3}
    BLT     1b
	
    /* Initialize stack pointer for the SYSTEM mode (C-stack)   */
    MSR     CPSR_c,#(SYS_MODE | I_BIT | F_BIT)
    LDR     sp,=__c_stack_top__ 
	
    /* Initialize stack pointer for the Undefined mode          */
	/*   -> Nicht nötig, da UND im System-Mode ausgeführt wird  */
    /* MSR     CPSR_C,#(UND_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */
	
    /* Initialize stack pointer for the Abort mode              */
	/*   -> Nicht nötig, das ABT im System-Mode ausgeführt wird */
    /* MSR     CPSR_C,#(ABT_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */
	
    /* Initialize stack pointer for the Supervsior mode         */
	/*   -> Nicht nötig, das SVR im System-Mode ausgeführt wird */
    /* MSR     CPSR_C,#(SVC_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */
	
    /* Initialize stack pointer for the Interrupt mode          */
	/*   -> Nicht nötig, da IRQ im System-Mode ausgeführt wird  */
    /* MSR     CPSR_C,#(IRQ_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */
	
    /* Initialize stack pointer for the FastInterrupt mode      */
	/*   -> Nicht nötig, das FIQ im System-Mode ausgeführt wird */
    /* MSR     CPSR_C,#(FIQ_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */
	
    /* Initialize stack pointer for the User mode               */
    /*   -> Nicht nötig, da  zu System-Mode ausgeführt wird	    */
    /* MSR     CPSR_C,#(USR_MODE | I_BIT | F_BIT)               */
	/* LDR     sp,=....                                         */

/* extern void (*__preinit_array_start []) (void) __attribute__((weak));*/
/* extern void (*__preinit_array_end []) (void) __attribute__((weak));  */
/* extern void (*__init_array_start []) (void) __attribute__((weak));   */
/* extern void (*__init_array_end []) (void) __attribute__((weak));     */
/* extern void _init (void);                                            */
/* void __libc_init_array (void) {                                      */
/*   size_t count;                                                      */
/*   size_t i;                                                          */
/*   count = __preinit_array_end - __preinit_array_start;               */
/*   for (i = 0; i < count; i++)                                        */
/*     __preinit_array_start[i] ();                                     */
/*   _init ();                                                          */
/*   count = __init_array_end - __init_array_start;                     */
/*   for (i = 0; i < count; i++)                                        */
/*     __init_array_start[i] ();                                        */
/* }                                                                    */
/* In Section init_array wird von newLib der Zeiger auf frame_dummy()   */
/* eingetragen, welches den Stack fürs Exception Handling von C++       */
/*    vorbereitet                                                       */
/* Weiter Eintragungen mit:                                             */
/*    void __attribute__((constructor)) test(void)                      */
/*   oder händisch über                                                 */
/* 	__attribute__((section(".init_array"))) static void(*fcn)(void)     */
/*  __attribute__((unused)) = {newlib_syscalls_init};                   */
    LDR     r12,=__libc_init_array
    MOV     lr,pc
    BX      r12

	/* Set Sytem-Mode + Interrupts Disabled !!!! */
    MSR     CPSR_c,#(SYS_MODE | I_BIT | F_BIT)

    /* Enter the C/C++ code */
	mov     r0,#2           /* argc */
	ldr     r1,PARGV        /* argv */
	mov     r2,#0           /* env  */
    LDR     r12,=main
    MOV     lr,pc           /* set the return address */
    BX      r12             /* the target code can be ARM or THUMB */
    /*b main*/

	LDR     r12,=_exit
	mov     lr,pc
	bx      r12
	
//	LDR   r12,=__libc_fini_array
//	MOV   lr,pc
//	bx    r12
.if 1
	b  .
.else
    SWI     0xFFFFFF        /* cause exception if main() ever returns */
.endif

/************************************************************************/
	.align 2
	.global PARGV
	PARGV: .word ARGV
	
    .size   _reset, . - _reset
    .endfunc

	.section	.rodata
	.align	2
	.global ARGV0
	ARGV0:		.ascii	"argv0\000"
	.align  2
	.global ARGV1
	ARGV1:      .ascii  "ARGV11\000"

	.global ARGV
	.align 2
	ARGV:       .word ARGV0
	            .word ARGV1
				.word 0

/************************************************************************/
    .end
