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
int16_t trace_buf2[TRACE_SIZE];
// Darstellung des Puffers über 'v.draw %e trace_buf0 trace_buf1'
void trace_scope(int channel, int16_t value) {
#define TRACE_ROLL
#ifdef TRACE_ROLL
  if (channel == 0) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf0[lauf] = trace_buf0[lauf + 1];
    trace_buf0[TRACE_SIZE - 1] = value;
  } else if (channel == 1) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf1[lauf] = trace_buf1[lauf + 1];
    trace_buf1[TRACE_SIZE - 1] = value;
  } else {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf2[lauf] = trace_buf2[lauf + 1];
    trace_buf2[TRACE_SIZE - 1] = value;
  }
#else
  static int trace_index = 0;
  if (trace_index < TRACE_SIZE) {
    if (channel == 0)
      trace_buf0[trace_index] = value;
    else if (channel == 1)
      trace_buf1[trace_index] = value;
    else
      trace_buf2[trace_index] = value;
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

// GPIO-Portzuweisung
#define MA0 15
// PerA=TF    PerB=TioA1   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer1
#define MA1 1
// PerA=PWM1  PerB=TioB0   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 0?

#define MB0 26
// PerA=DCD1  PerB=TioA2   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer2
#define MB1 9
// PerA=DRxD  PerB=NPCs1   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 1?

#define MC0 0
// PerA=PWM0  PerB=TioA0   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer0
#define MC1 8
// PerA=CTS0  PerB=ADTrg   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 2?

// Bitmask for all motor encoder pins (A and B signals)
#define MOTOR_ENCODER_PINS                                                     \
  ((1 << MA0) | (1 << MA1) | (1 << MB0) | (1 << MB1) | (1 << MC0) | (1 << MC1))

// Bitmask for B-signal pins only (used for GPIO interrupts)
#define MOTOR_B_SIGNAL_PINS ((1 << MA1) | (1 << MB1) | (1 << MC1))

// Bitmask for A-signal pins only (used for Timer/Counter)
#define MOTOR_A_SIGNAL_PINS ((1 << MA0) | (1 << MB0) | (1 << MC0))

typedef enum __attribute__((packed)) {
  TV_MANUELL,
  TV_VARIABEL,
  TV_END
} TV_MODE;

typedef enum __attribute__((packed)) {
  POSITION_MANUELL,
  POSITION_KONTINUIERLICH,
  POSITION_END
} POSITION_MODE;

struct {
  button_t button_old;
  motor_t motor_aktiv;
  TV_MODE schritt_mode;
  POSITION_MODE position_mode;
  struct {
    uint32_t pos;
    int8_t dir;
    int32_t speed;
  } motor[3];
} motor_data = {
    .motor_aktiv = MOTOR_A,
    .motor[0] =
        {
            .pos = 0,
            .dir = 0,
            .speed = 0,
        },
    .motor[1] =
        {
            .pos = 0,
            .dir = 0,
            .speed = 0,
        },
    .motor[2] =
        {
            .pos = 0,
            .dir = 0,
            .speed = 0,
        },
};
// Anzeigen über v.view %e %spotlight motor_data

void gpio_isr_entry(void) {
  //  uint32_t i_state = interrupts_get_and_disable();

  uint32_t pinChanges =
      *AT91C_PIOA_ISR; // IRQ-Quelle auslesen + IRQ zurücksetzen
  uint32_t currentPins = *AT91C_PIOA_PDSR; // Read pins

  //...

  //  if (i_state)
  //    interrupts_enable();
}

int motor_init(void) {
  // Motor PWM 'aktivieren'
  // PWM-Signale laufen über den AVR-Prozessor
  // Initialisierung nicht notwendig, da dieser zuvor
  // bereits mittels nxt_avr_init()()ininitialisiert wurde

  // Motoren Stoppen und in Freilauf stzen
  nxt_avr_set_motor(MOTOR_A, 100, MOTOR_FLOAT); // Motor A, PWM=0
  nxt_avr_set_motor(MOTOR_B, 0, MOTOR_FLOAT);   // Motor B, PWM=0
  nxt_avr_set_motor(MOTOR_C, 0, MOTOR_FLOAT);   // Motor C; PWM=0

  // GPIO Clock einschalten (bereits erledigt)
  // AIC  Clock einschalten (bereits erledigt)

  // AIC Initialisieren
  aic_mask_off(AT91C_ID_PIOA);
  aic_set_vector(AT91C_ID_PIOA,
                 AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | // Art
                     AIC_INT_LEVEL_ABOVE_NORMAL,    // Prio
                 gpio_isr_entry);                   // ISR
  aic_mask_on(AT91C_ID_PIOA);

  // Disable Pull up resistors
  *AT91C_PIOA_PPUDR = MOTOR_ENCODER_PINS;
  // Set Input Filter Enable Registe
  *AT91C_PIOA_IFER = MOTOR_ENCODER_PINS;
  // Read the  Interrupt Status Register
  // This also triggers a reset
  (void)*AT91C_PIOA_ISR;
  // Set Interrupt Enable Register
  *AT91C_PIOA_IER = MOTOR_ENCODER_PINS;

  return 0;
}

// Getter-Methode zur Abfrage der Position und Geschwindigkeit
int motor_get(int8_t port, uint32_t *pos, int16_t *speed) {
  if ((port < 0) || (port > 2))
    return -1;
  if ((pos == NULL) || (speed == NULL))
    return -1;

  *pos = motor_data.motor[port].pos;
  // Ob das reicht, ist ihre Entscheidung
  *speed = motor_data.motor[port].speed;

  return 0;
}

// Funktion wird alle 16ms aufgerufen
void motor_process(void) {
  uint32_t pos;
  int16_t speed;

  // Aktuellen Motor aus UI lesen
  motor_get(motor_data.motor_aktiv, &pos, &speed);
  trace_scope(0, (int16_t)(pos & 0xffff));
  trace_scope(1, speed);
  trace_scope(2, (int16_t)pos);

  // Geschwindigkeitsvorgabe aus UI umsetzen
  // nxt_motor_set();
}

// Funktion wird alle 64ms aufgerufen
void ui_process(void) {
  button_t button_new = nxt_avr_get_buttons();

  // Beispiel für Tastenauswertung
  if (!motor_data.button_old.orange && button_new.orange) {
    (void)term_string("Orange pressed\n\r", ASYNCSYNC_NONBLOCK);
  }
  if (motor_data.button_old.orange && !button_new.orange) {
    (void)term_string("Orange released\n\r", ASYNCSYNC_NONBLOCK);
  }
  if (motor_data.button_old.left && !button_new.left) {
    (void)term_string("Left pressed\n\r", ASYNCSYNC_NONBLOCK);
  }
  if (!motor_data.button_old.right && button_new.right) {
    (void)term_string("Right pressed\n\r", ASYNCSYNC_NONBLOCK);
  }
  if (!motor_data.button_old.grey && button_new.grey) {
    (void)term_string("Grey pressed\n\r", ASYNCSYNC_BLOCK);
  }

  motor_data.button_old = button_new;
}

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*****************************************************************************/

void task_4ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_8ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_16ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
  motor_process();
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

  motor_init();

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
