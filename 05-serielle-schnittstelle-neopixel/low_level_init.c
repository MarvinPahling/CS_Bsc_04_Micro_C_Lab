/*****************************************************************************
* Product: QDK/C_ARM-GNU_AT91SAM7S-EK
* Last Updated for Version: 4.4.00
* Date of the Last Update:  Feb 26, 2012
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2002-2012 Quantum Leaps, LLC. All rights reserved.
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
#include <stdint.h>
#include "AT91SAM7S64.h"                        /* AT91SAMT7S64 definitions */
#include "main.h"

#define LDR_PC_PC  0xE59FF000U   //LDR pc,[pc,#xxx]
#define MAGIC      0xDEADBEEFU

/*..........................................................................*/
/* low_level_init() is invoked by the startup sequence after initializing
* the C stack, but before initializing the segments in RAM.
*
* low_level_init() is invoked in the ARM state. The function gives the
* application a chance to perform early initializations of the hardware.
* This function cannot rely on initialization of any static variables,
* because these have not yet been initialized in RAM.
*/
void low_level_init(void (*reset_addr)()) 
{
	
#ifndef MODE_SIM
/* Bei Samba ist der Prozessor schon auf 48MHz gestellt (andernfalls */
/* würde USB nicht laufen). Allerdings läuft der Timer nicht mit der */
/* erwarteten Frequenz, so dass diese Initialisierung durchgeführt wird */
/* #ifndef MODE_SAMBA */
    AT91PS_PMC pPMC;

    /* Set flash wait sate FWS and FMCN 
	* MCK=47923200 (wird in bsp.h gesetzt)
	* 2 cycles for Read, 3 for Write operations
	*/	
    AT91C_BASE_MC->MC_FMR = ((AT91C_MC_FMCN) & ((MCK + 500000)/1000000 << 16))
                             | AT91C_MC_FWS_1FWS;

    /* Enable External Reset */
    AT91C_BASE_RSTC->RSTC_RMR = 0xA5000001;
	
    /* Enable the Main Oscillator:
    * set OSCOUNT to 6, which gives Start up time = 8 * 6 / SCK = 1.4ms
    * (SCK = 32768Hz)
    */
    pPMC = AT91C_BASE_PMC;
    pPMC->PMC_MOR = ((6 << 8) & AT91C_CKGR_OSCOUNT) | AT91C_CKGR_MOSCEN;
    while ((pPMC->PMC_SR & AT91C_PMC_MOSCS) == 0); /* Wait the startup time */

    /* Set the PLL and Divider:
    * - div by 5 Fin = 3,6864 =(18,432 / 5)
    * - Mul 25+1: Fout = 95,8464 =(3,6864 *26)
    * for 96 MHz the error is 0.16%
    * Field out NOT USED = 0
    * PLLCOUNT pll startup time estimate at : 0.844 ms
    * PLLCOUNT 28 = 0.000844 /(1/32768)
    */
    pPMC->PMC_PLLR = ((AT91C_CKGR_DIV & 0x05)
                      | (AT91C_CKGR_PLLCOUNT & (28 << 8))
                      | (AT91C_CKGR_MUL & (25 << 16)));
    while ((pPMC->PMC_SR & AT91C_PMC_LOCK) == 0); /* Wait the startup time */
    while ((pPMC->PMC_SR & AT91C_PMC_MCKRDY) == 0);

    /* Select Master Clock and CPU Clock select the PLL clock / 2 */
    pPMC->PMC_MCKR =  AT91C_PMC_PRES_CLK_2;
    while ((pPMC->PMC_SR & AT91C_PMC_MCKRDY) == 0);

    pPMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
    while ((pPMC->PMC_SR & AT91C_PMC_MCKRDY) == 0);
	
#endif

    /* Initialsisierung der Applikations-IVT */
	
    extern uint32_t ivt_app[2][8];  //Wird wahlweise über link.ld oder startup.s gesezt
	                                //Zeigt auf die Startadresse der IVT, welche zur 
						    	    //Laufzeit beschrieben werden kann
 
    /* setup the primary vector table in RAM */
    //*((uint32_t *) 0x00) = LDR_PC_PC | 0x18;   Andernfalls baut der Compiler hier 
	//                                           ein 'breapoint' ein
    ivt_app[0][0] = LDR_PC_PC | 0x18;
    ivt_app[0][1] = LDR_PC_PC | 0x18;
    ivt_app[0][2] = LDR_PC_PC | 0x18;
    ivt_app[0][3] = LDR_PC_PC | 0x18;
    ivt_app[0][4] = LDR_PC_PC | 0x18;
    ivt_app[0][5] = MAGIC;
    ivt_app[0][6] = LDR_PC_PC | 0x18;
    ivt_app[0][7] = LDR_PC_PC | 0x18;

    /* setup the secondary vector table in RAM */
    ivt_app[1][0] = (uint32_t)reset_addr;
    ivt_app[1][1] = 0x04U;  //Hier Sprung zu sich selbst <-> while(1)
    ivt_app[1][2] = 0x08U;  //Eigentliche Einträge erfolgen durch aic.c
    ivt_app[1][3] = 0x0CU;  
    ivt_app[1][4] = 0x10U;
    ivt_app[1][5] = MAGIC;
    ivt_app[1][6] = 0x18U;
    ivt_app[1][7] = 0x1CU;

#ifndef MODE_SIM
    /* check if the Memory Controller has been remapped already */
	/* Wird benötigt, da später die Vector-Tabelle auf Adresse 0 überschrieben wird */
    if (MAGIC != (*(uint32_t volatile *)0x14)) {
        AT91C_BASE_MC->MC_RCR = 1;   /* perform Memory Controller remapping */
    }
#endif
}

