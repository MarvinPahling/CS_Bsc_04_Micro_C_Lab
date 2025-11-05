#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> //fuer _exit()

#include "AT91SAM7S64.h"
#include "lib/aic.h"
#include "lib/display.h"
#include "lib/nxt_avr.h"
#include "lib/systick.h"
#include "lib/term.h"
#include "main.h"
// #include "lib/adc.h"
#if defined(MODE_RAM) || defined(MODE_SIM)
#include "trace32/udmon3.h"
#endif

#define ZYKLUS_MS 2
#define IDLE_MS 1

#if IDLE_MS >= ZYKLUS_MS
#error "Idle_ms muss kleiner als zyklus_ms sein"
#endif

/*****************************************************************************/
/*   Globale Variablen                                                       */
/*****************************************************************************/
struct {
  signed char term_status; // 0->false->Alles Bestens   -1->true->Overflow
  unsigned char term_cnt;
  unsigned char lowbat_cnt;
} main_data = {
    .term_status = 0,
    .term_cnt = 0,
    .lowbat_cnt = 0,
};

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*   Standard-C-Library (weitere befinden sich in newlib_syscalls.c)         */
/*****************************************************************************/
// Routine wird von C-Lib aufgerufen (bspw. printf() abort())
void _exit(int status) {
  (void)status;
  // LED-Blinken lassen
  // Breakpoint setzen
  while (1)
    ;
}

/************************************************************************/
/*   Hilfsroutine zur Darstellung eines analogen Verlaufes entsprechend */
/*   einem Oszillosop                                                   */
/************************************************************************/
#define TRACE_SIZE 200
int16_t trace_buf0[TRACE_SIZE];
int16_t trace_buf1[TRACE_SIZE];
// Darstellung des Puffers über 'v.draw %e trace_buf0 trace_buf1'
void trace_scope(int channel, int16_t value) {
  // #define TRACE_ROLL
#ifdef TRACE_ROLL
  if (channel == 0) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf0[lauf] = trace_buf0[lauf + 1];
    trace_buf0[TRACE_SIZE - 1] = value;
  } else {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf1[lauf] = trace_buf1[lauf + 1];
    trace_buf1[TRACE_SIZE - 1] = value;
  }
#else
  static int trace_index = 0;
  if (trace_index < TRACE_SIZE) {
    if (channel == 0)
      trace_buf0[trace_index] = value;
    else
      trace_buf1[trace_index] = value;
  } else {
    // Pause, damit der Debugger in 'Ruhe' den gesamten Speicher auslesen kann
  }
  if (channel == 1) {
    trace_index++;
    trace_index = trace_index >= (3 * TRACE_SIZE) ? 0 : trace_index;
  }
#endif
}

/*****************************************************************************/
/*   Ihr Programm                                                            */
/*****************************************************************************/
// AT91S_PIO, *AT91PS_PIO
#define SPULE1_PWM (1 << 23)
#define SPULE2_PWM (1 << 2)
#define SPULE1_DIR (1 << 18)
#define SPULE2_DIR (1 << 30)
#define NXT_PORT4_ENABLE (1 << 7)
#define ALL                                                                    \
  (SPULE1_DIR | SPULE2_DIR | SPULE1_PWM | SPULE2_PWM | NXT_PORT4_ENABLE)
#define SPULE1_PWM_BASE AT91C_BASE_PWMC_CH2
#define SPULE2_PWM_BASE AT91C_BASE_PWMC_CH0

#define PA30 SPULE2_DIR
#define PA2  SPULE2_PWM
#define PA18 SPULE1_DIR
#define PA23 SPULE1_PWM
#define PA7 NXT_PORT4_ENABLE

#define MASK_FULL1   3
#define MASK_FULL2   3
#define MASK_HALF    7
#define MASK_MICRO   31

#define PWM_FREQ_HZ   2000
#define MCK_HZ        48000000
#define CPRD_2KHZ     (MCK_HZ / PWM_FREQ_HZ) // = 24000 
#define STEPS_MICRO   32



AT91PS_PIO pio = (AT91PS_PIO)AT91C_BASE_PIOA;
AT91PS_PMC pmc = (AT91PS_PMC)AT91C_BASE_PMC;
AT91PS_PWMC pwm_ctl = (AT91PS_PWMC)AT91C_BASE_PWMC;


typedef enum {
  SCHRITT_VOLL_1,
  SCHRITT_VOLL_2,
  SCHRITT_HALB,
  SCHRITT_SINUS,
  SCHRITT_END
} SCHRITT_MODE;
typedef enum {
  POSITION_MANUELL,
  POSITION_KONTINUIERLICH,
  POSITION_END
} POSITION_MODE;
const char *schritt_mode2str[] = {
    [SCHRITT_VOLL_1] = "Vollschritt 1",
    [SCHRITT_VOLL_2] = "Vollschritt 2",
    [SCHRITT_HALB] = "Halbschritt",
    [SCHRITT_SINUS] = "SinusSchritt",
};

struct {
  button_t button_old;
  SCHRITT_MODE schritt_mode;
  POSITION_MODE position_mode;
  int8_t pos;
  uint8_t speed;
  uint32_t mode;
  uint8_t step;
  uint16_t reload_table[5];
  uint16_t current_reload;
  uint16_t counter;
  int32_t micro[32];
  int8_t dir;            

} schrittmotor_data = {
    .schritt_mode = SCHRITT_VOLL_1,
    .position_mode = POSITION_MANUELL,
    .pos = 0,
    .speed = 0,
    .mode = 0,
    .step = 0,
    .current_reload = 0,
    .dir = 1,
    .counter = 0,
    .reload_table = {
      256, // speed 0 -> 256*4ms = 1024 ms 
      128, // speed 1 -> 512 ms 
      64, // speed 2 -> 256 ms 
      32, // speed 3 -> 128 ms 
      1  // speed 4 -> 4 ms 
    },
    .micro = {
      1,
      4682,
      9184,
      13334,
      16971,
      19955,
      22173,
      23539,
      23999,
      23539,
      22173,
      19955,
      16971,
      13334,
      9184,
      4682,
      1,
      -4682,
      -9184,
      -13334,
      -16971,
      -19955,
      -22173,
      -23539,
      -23999,
      -23539,
      -22173,
      -19955,
      -16971,
      -13334,
      -9184,
      -4682

    }
}; 



void schrittmotor_init(SCHRITT_MODE mode) {
  schrittmotor_data.schritt_mode = mode;
  if (mode != SCHRITT_SINUS) {
    pmc->PMC_PCER = ((1 << 2) | (1 << 10));
    pio->PIO_CODR = (1 << 7);
    pio->PIO_MDDR = ALL;
    pio->PIO_PER = ALL;
    pio->PIO_OER = ALL;
    pio->PIO_SODR = (SPULE1_DIR | SPULE2_DIR | SPULE2_PWM | SPULE1_PWM);

  } else {
    pmc->PMC_PCER = ((1 << 2) | (1 << 10));
    
    pio->PIO_MDDR = (PA30 | PA18 | PA7);
    pio->PIO_PER = (PA30 | PA18 | PA7);
    pio->PIO_OER = (PA30 | PA18 | PA7);
    pio->PIO_CODR = PA7;
    pio->PIO_SODR = (PA30 | PA18);

    pio->PIO_PDR = (PA23 | PA2);
    pio->PIO_ASR = PA2;
    pio->PIO_BSR = PA23;

    // Channel 0 PA23 -> PWM0
    pwm_ctl->PWMC_CH[0].PWMC_CMR = AT91C_PWMC_CPRE_MCK; // CPRE = MCK (kein prescaler) 
    pwm_ctl->PWMC_CH[0].PWMC_CPRDR = CPRD_2KHZ;         // Period
    pwm_ctl->PWMC_CH[0].PWMC_CDTYR = (CPRD_2KHZ / 2);   // duty = 50%

    // Channel 2 (PA2 -> PWM2
    pwm_ctl->PWMC_CH[2].PWMC_CMR = AT91C_PWMC_CPRE_MCK; 
    pwm_ctl->PWMC_CH[2].PWMC_CPRDR = CPRD_2KHZ;
    pwm_ctl->PWMC_CH[2].PWMC_CDTYR = (CPRD_2KHZ / 2);
    
    pwm_ctl->PWMC_ENA = (AT91C_PWMC_CHID0 | AT91C_PWMC_CHID2);

  }
}
void schrittmotor_update(void)
{
    uint16_t max_steps;
    switch (schrittmotor_data.schritt_mode) {
        case SCHRITT_VOLL_1: max_steps = MASK_FULL1; break;
        case SCHRITT_VOLL_2: max_steps = MASK_FULL2; break;
        case SCHRITT_HALB:   max_steps = MASK_HALF;  break;
        case SCHRITT_SINUS:  max_steps = MASK_MICRO; break;
        default:             max_steps = MASK_FULL1; break;
    }

	if (schrittmotor_data.pos !=0) {
				if(schrittmotor_data.pos >0){
					schrittmotor_data.step = (schrittmotor_data.step+1) & max_steps;
					schrittmotor_data.pos--;
				}

				if(schrittmotor_data.pos <0){
					schrittmotor_data.step = (schrittmotor_data.step-1) & max_steps;
					schrittmotor_data.pos++;

				}

	}
}


void schrittmotor_process(void) {
    switch (schrittmotor_data.schritt_mode) {

        case SCHRITT_VOLL_1: {
            if ((schrittmotor_data.pos != 0)) {
              schrittmotor_update();
                switch (schrittmotor_data.step) {
                    case 0:
                        pio->PIO_SODR = SPULE1_PWM;
                        pio->PIO_CODR = (SPULE1_DIR | SPULE2_DIR | SPULE2_PWM);
                        break;
                    case 1:
                        pio->PIO_SODR = SPULE2_PWM;
                        pio->PIO_CODR = (SPULE1_DIR | SPULE2_DIR | SPULE1_PWM);
                        break;
                    case 2:
                        pio->PIO_SODR = (SPULE1_DIR | SPULE1_PWM);
                        pio->PIO_CODR = (SPULE2_DIR | SPULE2_PWM);
                        break;
                    case 3:
                        pio->PIO_SODR = (SPULE2_DIR | SPULE2_PWM);
                        pio->PIO_CODR = (SPULE1_DIR | SPULE1_PWM);
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case SCHRITT_VOLL_2: {
            if ((schrittmotor_data.pos != 0)) {
              schrittmotor_update();
                switch (schrittmotor_data.step) {
                    case 0:
                        pio->PIO_SODR = (PA2 | PA23);
                        pio->PIO_CODR = (PA30 | PA18);
                        break;
                    case 1:
                        pio->PIO_SODR = (PA2 | PA23 | PA18);
                        pio->PIO_CODR = (PA30);
                        break;
                    case 2:
                        pio->PIO_SODR = (PA2 | PA23 | PA18 | PA30);
                        break;
                    case 3:
                        pio->PIO_SODR = (PA2 | PA23 | PA30);
                        pio->PIO_CODR = (PA18);
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case SCHRITT_HALB: {
            if ((schrittmotor_data.pos != 0)) {
              schrittmotor_update();
                switch (schrittmotor_data.step) {
                    case 0:
                        pio->PIO_SODR = SPULE1_PWM;
                        pio->PIO_CODR = (SPULE1_DIR | SPULE2_DIR | SPULE2_PWM);
                        break;
                    case 1:
                        pio->PIO_SODR = (PA2 | PA23);
                        pio->PIO_CODR = (PA30 | PA18);
                        break;
                    case 2:
                        pio->PIO_SODR = SPULE2_PWM;
                        pio->PIO_CODR = (SPULE1_DIR | SPULE2_DIR | SPULE1_PWM);
                        break;
                    case 3:
                        pio->PIO_SODR = (PA2 | PA23 | PA18);
                        pio->PIO_CODR = (PA30);
                        break;
                    case 4:
                        pio->PIO_SODR = (SPULE1_DIR | SPULE1_PWM);
                        pio->PIO_CODR = (SPULE2_DIR | SPULE2_PWM);
                        break;
                    case 5:
                        pio->PIO_SODR = (PA2 | PA23 | PA18 | PA30);
                        break;
                    case 6:
                        pio->PIO_SODR = (SPULE2_DIR | SPULE2_PWM);
                        pio->PIO_CODR = (SPULE1_DIR | SPULE1_PWM);
                        break;
                    case 7:
                        pio->PIO_SODR = (PA2 | PA23 | PA30);
                        pio->PIO_CODR = (PA18);
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case SCHRITT_SINUS: {

            schrittmotor_update();

            // A = sin, B = cos = sin + 90deg
            uint8_t idxA = schrittmotor_data.step & MASK_MICRO;
            uint8_t idxB = (idxA + (STEPS_MICRO / 4)) & MASK_MICRO; 

            //Array auslesen
            int32_t sampleA = (int32_t) schrittmotor_data.micro[idxA];
            int32_t sampleB = (int32_t) schrittmotor_data.micro[idxB];

           
            if (sampleB >= 0) {
                pio->PIO_CODR = SPULE1_DIR;
            } else {
                pio->PIO_SODR = SPULE1_DIR;
            }

            if (sampleA >= 0) {
                pio->PIO_CODR = SPULE2_DIR;
            } else {
                pio->PIO_SODR = SPULE2_DIR;
            }

            uint32_t dutyA = (uint32_t)(sampleA >= 0 ? sampleA : -sampleA);
            uint32_t dutyB = (uint32_t)(sampleB >= 0 ? sampleB : -sampleB);

 
            pwm_ctl->PWMC_CH[0].PWMC_CUPDR = dutyA; // PA23 -> channel 0 
            pwm_ctl->PWMC_CH[2].PWMC_CUPDR = dutyB; // PA2  -> channel 2 

            break;
        }

        default: {
            
            break;
        }
    } 
} 


void ui_init(void) {}

void ui_process(void) {
 

  button_t button_new = nxt_avr_get_buttons();

  if ((button_new.orange == 1) && (schrittmotor_data.button_old.orange == 0)) {

    if (schrittmotor_data.schritt_mode + 1 < SCHRITT_END) {
      schrittmotor_data.schritt_mode++;
      schrittmotor_init(schrittmotor_data.schritt_mode);
    } else {
      schrittmotor_data.schritt_mode = SCHRITT_VOLL_1;
      schrittmotor_init(schrittmotor_data.schritt_mode);
    }

  }

  if ((button_new.grey == 1) && (schrittmotor_data.button_old.grey == 0)) {

    if (schrittmotor_data.position_mode + 1 < POSITION_END) {
      schrittmotor_data.position_mode++;
    } else {
      schrittmotor_data.position_mode = POSITION_MANUELL;
    }
  }

if (schrittmotor_data.position_mode == POSITION_KONTINUIERLICH) {
    if (((button_new.left == 1) && (schrittmotor_data.dir == 1)) && (schrittmotor_data.button_old.left == 0)) {
        if (schrittmotor_data.speed > 0) {
            schrittmotor_data.speed--;
        } else {
            schrittmotor_data.dir = (schrittmotor_data.dir == 1) ? -1 : 1;
        }
    }
    if (((button_new.left == 1) && (schrittmotor_data.dir == -1)) && (schrittmotor_data.button_old.left == 0)) {
        if (schrittmotor_data.speed <4 ) {
            schrittmotor_data.speed++;
        }
    }

    if (((button_new.right == 1) && (schrittmotor_data.dir == 1)) && (schrittmotor_data.button_old.right == 0)) {
        if (schrittmotor_data.speed < 4) {
            schrittmotor_data.speed++;
        }
    }
     if (((button_new.right == 1) && (schrittmotor_data.dir == -1)) && (schrittmotor_data.button_old.right == 0)) {
        if (schrittmotor_data.speed > 0) {
            schrittmotor_data.speed--;
        } else {
            schrittmotor_data.dir = (schrittmotor_data.dir == -1) ? 1 : -1;
        }
    }
}

  if (schrittmotor_data.position_mode == POSITION_MANUELL) {

    if ((button_new.left == 1) && (schrittmotor_data.button_old.left == 0)) {
      schrittmotor_data.pos--;
    }
    if ((button_new.right == 1) && (schrittmotor_data.button_old.right == 0)) {
      schrittmotor_data.pos++;
    }
  }

  

  // Beispielanwendung für Display
  static uint32_t count = 0;
  display_goto_xy(0, 1);
  display_unsigned(++count, 4);
  display_goto_xy(0, 2);
  display_string(schritt_mode2str[schrittmotor_data.schritt_mode]);
  display_update();

  schrittmotor_data.button_old = button_new;
}

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*****************************************************************************/

void task_4ms(void) {
    schrittmotor_data.current_reload = schrittmotor_data.reload_table[schrittmotor_data.speed];

    if (schrittmotor_data.position_mode == POSITION_KONTINUIERLICH) {
        schrittmotor_data.counter++;
        if (schrittmotor_data.counter >= schrittmotor_data.current_reload) {
            schrittmotor_data.counter = 0;
            schrittmotor_data.pos += (int32_t)schrittmotor_data.dir;
            schrittmotor_process();
        }
    } else {
        schrittmotor_process();
        schrittmotor_data.counter = 0;
    }
}

void task_8ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_16ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_32ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_64ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
  ui_process();
}

void task_128ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_256ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_512ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_1024ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_idle(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: IDLE_MS
}

/*****************************************************************************/
/*    Pre-Main-Funktion                                                      */
/*****************************************************************************/
// Zur Vermeidung von malloc(1024) bei der ersten Ausgabe über stdout!
//__attribute__(constructor) stellt sicher, dass premain_init() direkt nach
//__sinit() aufgerufen wird (beides innerhalb von __libc_init_array())
void __attribute__((constructor)) premain_init(void) {
#if 0
	//No linebuffering, call stdio_write() immediately
	//-> Langsam, da mit jedem Zeichen __sflush_r()/_write()/stdio_write() aufgerufen wird 
	setvbuf(stdout,NULL,_IONBF,0);
#else
  static char linebuf[10];
  // LineBuffering into global Varialbe (guter Kompromiss)
  setvbuf(stdout, linebuf, _IOLBF, sizeof(linebuf)); // Linebuffering in
#endif
}
/*****************************************************************************/
/*    Main-Funktion                                                          */
/*****************************************************************************/
#if 0
//Variante 1: Deklaration der main() Funktion
//da es keine CLI gibt, über welcher die Anwendung getartet wird
//sondern der start über startup.s erfolgt, macht dies kein Sinn
//und belegt unnötige Speicherplatz auf den Stack
int main(int argc, char *argv[]) 
{
	(void) argc;
	(void) argv;
#else
// Variante 2: Deklaration der main() funktion
int main(void) {
#endif
  /* Interrupts zu diesem Zeitpunkt disabled !!!! */

  /* 'Pflicht' Initialisierung, können nicht ausgelassen werden */
  aic_init();          // Interrupt-Controller initialisieren
  systick_init();      // System-Timer initialisieren
  interrupts_enable(); // Ohne Worte
  nxt_avr_init(8);
#if defined(MODE_RAM) || defined(MODE_SIM)
  udmon3_init(); // Speicherzugriff durch den Debugger
                 // zur Programmlaufzeit mittels der
                 // DCC Schnittstelle ermöglichen
#endif

  /* 'Wahl' Initialisierung, hängt von den benötigten Komponenten ab */
  term_init();
  display_init();

  ui_init();
  schrittmotor_init(schrittmotor_data.schritt_mode);

  display_clear(0);
  display_update();

  // ANSI Escape sequences - VT100 / VT52 (see main.h)
  (void)term_string("hallo\n\r", ASYNCSYNC_BLOCK);
  (void)term_string(
      "\033[2J" VT100_CURSORHOME // Move Cursor to home position (0,0)
          VT100_DEFAULT,
      ASYNCSYNC_BLOCK);
  (void)term_string("Prog: " APP_NAME "\n\rVersion von: " __DATE__ " " __TIME__
                    "\n\r",
                    ASYNCSYNC_BLOCK);
  (void)term_string(
      "'v.view %e schrittmotor_data' zur Variablendarstellung\n\r",
      ASYNCSYNC_BLOCK);
  (void)term_string("Viel Erfolg!\n\r", ASYNCSYNC_BLOCK);

  // Alternativ zu term_xxx() kann auch printf() oder noch besser iprintf
  // genutzt werden beide bedingen jedoch einen großen Speicherbedarf!
  // Alternativ zu term_read() kann auch scanf() genutzt werden. Auch diese
  // Funktion
  //  bedingt einen großen Speicherbedarf

#ifndef MODE_ROM
  /* Watchdog Disable */
  /* Mode-Register kann nur einmal beschrieben werden */
  AT91C_BASE_WDTC->WDTC_WDMR = 0xFFF | AT91C_WDTC_WDDIS | /*WD Disable */
                               AT91C_WDTC_WDDBGHLT |      /*Debug Halt */
                               AT91C_WDTC_WDIDLEHLT;      /*Idle Halt  */
#else
#if 0
	/* Watchdog Enable */
	/* Da in dieser Version kein zyklischer Reset des Watchdogs */
	/* vorhanden ist, wird von einem Watchdog Enable abgesehen  */
	/* Mit Reset wird der Wachdog aktiviert!                    */
#else
/* Watchdog Disable */
/* Mode-Register kann nur einmal beschrieben werden */
AT91C_BASE_WDTC->WDTC_WDMR = 0xFFF | AT91C_WDTC_WDDIS | /*WD Disable */
                             AT91C_WDTC_WDDBGHLT |      /*Debug Halt */
                             AT91C_WDTC_WDIDLEHLT;      /*Idle Halt  */
#endif
#endif
  // Vorangegangenen Stackaufbau 'löschen'
  stack_fill();
  // Label, so das mit 'go start' hierin gesprungen werden kann
start:
  __attribute__((unused));
  uint32_t start_tick = systick_get_ms();
  uint32_t zeitscheibe = 0;
  char *task_aktiv = "";
  while (1) {
    // Warten bis zum nächsten TimeSlot
    while ((int)(start_tick - systick_get_ms()) > 0)
      ;
    start_tick += ZYKLUS_MS;
    // Label, so das mit 'go zyklus' hierhin gesprungen werden kann
  zyklus:
    __attribute__((unused)) if ((zeitscheibe & 0b000000001) == 0b000000001) {
      task_aktiv = "4ms";
      task_4ms();
    }
    else if ((zeitscheibe & 0b000000011) == 0b000000010) {
      task_aktiv = "8ms";
      task_8ms();
    }
    else if ((zeitscheibe & 0b000000111) == 0b000000100) {
      task_aktiv = "16ms";
      task_16ms();
    }
    else if ((zeitscheibe & 0b000001111) == 0b000001000) {
      task_aktiv = "32ms";
      task_32ms();
    }
    else if ((zeitscheibe & 0b000011111) == 0b000010000) {
      task_aktiv = "64ms";
      task_64ms();
    }
    else if ((zeitscheibe & 0b000111111) == 0b000100000) {
      task_aktiv = "128ms";
      task_128ms();
    }
    else if ((zeitscheibe & 0b001111111) == 0b001000000) {
      task_aktiv = "256ms";
      task_256ms();
    }
    else if ((zeitscheibe & 0b011111111) == 0b010000000) {
      task_aktiv = "512ms";
      task_512ms();
    }
    // Zeit für IDLE-Task verfügbar
    if ((int)(start_tick - systick_get_ms()) >= IDLE_MS) {
      task_aktiv = "Idle";
      task_idle();
    }
    // Max. Zeitdauer einer Zeitscheibe überschritten?
    if ((int)(start_tick - systick_get_ms()) <= 0) {
      main_data.term_status |= term_string(
          VT100_VORDERGRUND_ROT "Timing durch '", ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string(task_aktiv, ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string(
          "' verletzt\n\r" VT100_VORDERGRUND_DEFAULT, ASYNCSYNC_NONBLOCK);
    }
    // Zeitscheibe erhöhen
    zeitscheibe++;

    // Stack Testen
    if (stack_check() < (1 * 4)) {
      static uint8_t stack_cnt = 0;
      if (stack_cnt == 0) {
        stack_cnt = 1;
        main_data.term_status |= term_string(
            VT100_VORDERGRUND_ROT "Stack overflow durch '", ASYNCSYNC_NONBLOCK);
        main_data.term_status |= term_string(task_aktiv, ASYNCSYNC_NONBLOCK);
        main_data.term_status |=
            term_string("'\n\r" VT100_VORDERGRUND_DEFAULT, ASYNCSYNC_NONBLOCK);
        while (1)
          ;
      }
    }

    // Batteriespannung überprüfen
    // Ggf. schlägt die Unterspannungsprüfung im Akku zuvor ein!
    if (nxt_avr_get_battery_raw() < ((2 * 3000 /*mV*/) << 10) / 14180) {
      if (main_data.lowbat_cnt++ > 100)
        while (1)
          ;
      else if (main_data.lowbat_cnt == 10)
        main_data.term_status |=
            term_string(VT100_VORDERGRUND_ROT
                        "\n\rLow Battery\n\r" VT100_VORDERGRUND_DEFAULT,
                        ASYNCSYNC_NONBLOCK);
    } else {
      main_data.lowbat_cnt = 0;
    }

    // Termstatus ueberpruefen
    if (main_data.term_cnt == 0 && main_data.term_status != 0) {
      main_data.term_cnt++;
    overflow:
      __attribute__((unused));

      (void)term_string(VT100_VORDERGRUND_ROT
                        "\n\rTerminal Overflow\n\r" VT100_VORDERGRUND_DEFAULT,
                        ASYNCSYNC_BLOCK);
      while (1)
        ;
    }
  }
  // Nachfolgende Funktion wird leider nie aufgerufen!
  nxt_avr_power_down();
  systick_wait_ms(1000);
  while (1)
    ;
}
