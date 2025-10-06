/*****************************************************************************
* Product: QDK/C_ARM-GNU_AT91SAM7S-EK, QK-port
* Last Updated for Version: 4.1.02
* Date of the Last Update:  Jan 13, 2010
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
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(ivt_boot)

/*****************************************************************************
ROM->PROGRAMM ist im ROM enthalten
       ROM-Bereich: wird durch Debugger aus ELF-File programmiert
       RAM-Bereich: nicht initialisiert
RAM-> Programm ist nur im RAM enthalten
       ROM-Bereich: Inhalt ist ohne Bedeutung
       RAM-Bereich: Inhalt wurde durch Debugger aus ELF-File korrekt geladen
                    (incl. DATA und FASTCODE)
SIM-> Hier wird nicht zwischen RAM und ROM unterschieden
      da der Anwendung der gesamten Speicherbereich zur Verfügung steht
      Memory-Swap wird nicht unterstützt!
	  ROM-Bereich: wird vom Debugger aus ELF-File geladen
	  RAM-Bereich: wird vom Debugger aus ELF-File geladen
	               (incl. DATA und FASTCODE)
SAMBA-> Der Prozessorinterne Bootloader unterstützt Anweisungen zum 
        Programmieren des internen RAM's.
		Die Anweisungen zum download werden über 'bin2samba' aus dem durch 
		'objcopy' aus dem ELF-File erzeugten BIN-File generiert 
		Der Bootloader belegt den Speicherbereich 0x200000..0x202000, so dass
		dieser Bereich nicht überladen werden sollte, sondern erst zur Laufzeit
		der Anwendung für HEAP/STACK/BSS genutzt werden kann.
		Ein Memory-Remap wurde noch nicht durchgeführt
		Der Prozessor wurde bereits initialisiert (für USB notwendig), jedoch 
		läuft der PIT in dieser Einstellung nicht mit der erwarteten Geschwindigkeit
		Nach Aktivierung von SAM-BA merkt sich der Prozessor diesen Modus, so 
		das beim nächsten Prozessor-Start der SAM-BA erneut aktiviert wird
*******************************************************************************
  Speicher    ROM-Mode     RAM-Mode      SIM-MODE      SAMBA
  +-------+
  |       |
  |  ...  |
  +-------+               
  |       |                                             DATA
  |  RAM  |                                           7)main
  |  RAM  |               7)main                      5)_cstartup
  |  RAM  |               5)_cstartup                 3)ll_init
  |       |               3)ll_init                     FASTCODE
  |       |               2)_reset                    2)_reset
  |       |               1)ivt_boot                  1)ivt_boot
  |       |  STACK          STACK
  |       |  HEAP           HEAP         STACK
  |       |  BSS            BSS          HEAP          STACK
  |       |  6)DATA         DATA         BSS           HEAP
  |       |  6)Fastcode     FASTCODE     DATA          BSS
  +-------+  4)RAMVEC     4)RAMVEC       FASTCODE     4)RAMVEC
  |  ROM  |  6)DATA
  |       |  7)main
  |       |  6)Fastcode
  |       |  5)_cstartup
  +-------+  3)ll_init     
  |RAM/ROM|
  | REMAP |                              7)main
  |       |                              5)_cstartup
  |       |                              3)ll_init
  |       |  2)_reset                    2)_reset
  +-------+  1)ivt_boot   4)RAMVEC       1)ivt_boot   4)RAMVEC
             4)RAMVEC                      RAMVEC
1) startadresse
    - Im ROM-Mode durch im Prozessor hart codierte Reset-Vector-Tabelle vorgegeben
	- In RAM-/SIM-Mode durch ENTRY Anweisung hier im LD-File vorgegeben
	- IM SAMBA-Mode durch Benutzer manuelle einzugegeben
2) _reset (startup.s)
    = Maschinensprachebefehle zum Aufruf von low_level_init()
    - wird über absoluten Sprung in der Reset-Vector-Tabelle angesprungen
	- Im ROM-Mode dadurch möglich, das ROM bei 0x0000 eingeblendet ist
3) low_level_init (low_level_init.c)
	= Prozessor Initialisierung (außer im SIM-Mode) (48MHz, FLASH Zugriff) 
	= Vector-Tabelle vorbereiten, die anschließend auf Adresse 0x0000 zum abgebildet wird
	= Memory-Swap (außer im SIM-MODE), d.h. auf Speicheradresse 
	    0x000000 wird inhalt von 0x200000 eingeblendet
	- Aufruf über Sprung zur absoluten Adresse
4) RAMVEC = Applikations-IVT
    - durch Abbildung im RAM kann diese zur Laufzeit gesetzt werden
	- siehe startup.s
5) _cstartup (startup.s)
	= BSS/STACK/CLIB Initialisieren
	= DATA/FASTCODE  aus ROM in den RAM kopieren (nur ROM-Mode)
	= Main Aufruf
6)  DATA/FASTCODE Initialisierungswerte
	- Initialisierungswerte aus ROM in den RAM kopieren
	- hierfür wurden hier im linker-File über 'RAM AT>ROM' Platz im RAM und ROM vorgehalten
	- Nur im ROM-Mode nötig
	- Im RAM/SIM-Mode wird dieser Bereich direkt durch den Debugger geladen
	- Im SAMBA-Mode dadurch ereicht, dass diese beiden Bereich dem ROM zugeordnet wurden
7)  main
	- Ohne Worte
*****************************************************************************/
NXT_WIDTH = 150;  /* Breite des NXT Graubild Images */
NXT_HIGH  = 250;  /* Höhe des NXT Graubild Images */

MEMORY {                                       /* memory map of AT91SAM7S64 */
         RAM (rwx) : ORIGIN = 0x00200000, LENGTH =  64k-0x100 /*-->(rwx)(rx)*/
         WA  (r)   : ORIGIN = 0x0020FF00, LENGTH =  0x0100  /* Parameter Speicher */
         LCD (rw)  : ORIGIN = 0x10000000, LENGTH =  NXT_WIDTH*NXT_HIGH   /* Virtueller Speicher für Bildschirm */
}


/* The size of the stack used by the application. NOTE: you need to adjust  */
STACK_SIZE = 3000;

/* The size of the heap used by the application. NOTE: you need to adjust   */
/* 1048 ist die Größe des ersten sbrk() Aufrufes, aufgrund der iprint()     */
//HEAP_SIZE = 1024;
HEAP_SIZE = 4096;

SECTIONS {
  

    ivt_boot_seg : {
        KEEP(*startup.o (.text.ivt_boot))  /* startup code (ARM vectors and reset handler) */
		                                   /* Keep, da Code nicht referenziert wird und    */
										   /* Locater diese Section andernfalls entfernt   */
        . = ALIGN(0x4);
        } >RAM

    fastcode_seg : {
        __fastcode_load = LOADADDR (fastcode_seg);
        __fastcode_start = .;

        *(.text.fastcode)   /* all functions explicitly placed in .fastcode */
        /* add other modules here ... */

        . = ALIGN (4);
        __fastcode_end = .;
        } >RAM

    text_seg : {
        . = ALIGN(4);
        *(.text)                                   /* .text sections (code) */
        *(.text*)                        /* Aufgrund von -ffunction-section */
        *(.rodata)           /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)                             /* Aufgrund -fdata-sections */
/*      *(.glue_7)                                        glue arm to thumb */
/*      *(.glue_7t)                                       glue thumb to arm */

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;                         /* global symbol at end of code */
        } >RAM

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
    preinit_array_seg (READONLY) : {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(SORT(.preinit_array.*)))
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        } >RAM

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
    init_array_seg (READONLY) : {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        } >RAM

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
    fini_array_seg (READONLY) : {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(.fini_array*))
        KEEP (*(SORT(.fini_array.*)))
        PROVIDE_HIDDEN (__fini_array_end = .);
        } >RAM

   ARM_exidx_extab_seg : {
		__exidx_start = .;
		*(.ARM.exidx .ARM.exidx.*)  /*sections containing index entries for section unwinding.*/
		__exidx_end = .;
		__extab_start = .;
		*(.ARM.extab .ARM.extab.*)  /*sections containing exception unwinding information.*/
		__extab_end = .;
        } >RAM


/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
        data_seg (READONLY) : {
        __data_load = LOADADDR (data_seg);
        __data_start = .;
        *(.data)                                          /* .data sections */
        *(.data*)                               /* Aufgrund -fdata-sections */
        . = ALIGN(4);
        _edata = .;
        } >RAM

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
        bss_seg (READONLY) : {
        __bss_start__ = . ;
        *(.bss)                                             /* .bss section */
        *(.bss*)                                /* Aufgrund -fdata-sections */
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;                     /* define a global symbol at bss end */
        __bss_end__ = .;
    } >RAM

    PROVIDE ( end = _ebss );
    PROVIDE ( _end = _ebss );
    PROVIDE ( __end__ = _ebss );

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
        heap_seg (READONLY) : {
        __heap_start__ = . ;
        . = . + HEAP_SIZE;
        . = ALIGN(4);
        __heap_end__ = . ;
    } >RAM

/* Section ist Writeable, so das Locater eine Warning ausgibt, wenn dies in ein loadable Segment gepackt wird! */
        stack_seg (READONLY) : {
        __stack_start__ = . ;
        . += STACK_SIZE;
        . = ALIGN (4);
        __c_stack_top__ = . ;
        __stack_end__ = .;
    } >RAM


    /* Remove information from the standard libraries */
    /DISCARD/ : {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

 lcd_seg : {
     /* Bitmap für NXT Bildschirm als Bit Grafik */
     /* Im RAM-Modus durch UDMON3 Schnittstelle passend aus realen Bildschirmspeicher abgebildet */
     . += NXT_WIDTH*NXT_HIGH / 8;
 } > LCD



}
