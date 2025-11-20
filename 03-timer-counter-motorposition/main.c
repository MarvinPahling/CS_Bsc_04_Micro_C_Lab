#include <stdatomic.h>
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
#define MA0_MASK (1 << MA0)
// PerA=TF    PerB=TioA1   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer1
#define MA1 1
#define MA1_MASK (1 << MA1)
// PerA=PWM1  PerB=TioB0   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 0?

#define MB0 26
#define MB0_MASK (1 << MB0)
// PerA=DCD1  PerB=TioA2   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer2
#define MB1 9
#define MB1_MASK (1 << MB1)
// PerA=DRxD  PerB=NPCs1   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 1?

#define MC0 0
#define MC0_MASK (1 << MC0)
// PerA=PWM0  PerB=TioA0   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer0
#define MC1 8
#define MC1_MASK (1 << MC1)
// PerA=CTS0  PerB=ADTrg   <-- Pos=Gpio-IRQ
// //Encoder Input Motor 2?

#define PINBIT(x) (1 << (x))
#define MASKA (PINBIT(MA0) | PINBIT(MA1))
#define MASKB (PINBIT(MB0) | PINBIT(MB1))
#define MASKC (PINBIT(MC0) | PINBIT(MC1))
#define ENCODER_PINS (MASKA | MASKB | MASKC)

#define LOG(msg) (void)term_string(msg, ASYNCSYNC_NONBLOCK)
#define LOG_INT(msg) (void)term_int(msg, 1, ASYNCSYNC_NONBLOCK)

// Macro to extract a single bit from a register
#define GET_BIT(reg, bit) (((reg) >> (bit)) & 1)

// Macro to detect rising edge: prev=0 and current=1
#define RISING_EDGE(prev, curr) ((!(prev)) & (curr))

#define MOTOR_INDEX motor_data.motor_aktiv
#define CURRENT_MOTOR motor_data.motor[MOTOR_INDEX]
#define MA motor_data.motor[0]
#define MB motor_data.motor[1]
#define MC motor_data.motor[2]

typedef enum __attribute__((packed)) {
  TV_MANUELL,
  TV_VARIABEL,
  TV_END
} TV_MODE;

// Speed indices
typedef enum __attribute__((packed)) {
  SLOW = 0,
  MEDIUM = 1,
  FAST = 2,
  SPEED_COUNT = 3
} SPEED;

// Speed duration values in milliseconds
static const uint16_t SPEED_DURATION_MS[] = {
    [SLOW] = 16384,
    [MEDIUM] = 8192,
    [FAST] = 4096,
};

typedef enum __attribute__((packed)) {
  POSITION_MANUELL,
  POSITION_KONTINUIERLICH,
  POSITION_END
} POSITION_MODE;

typedef enum __attribute__((packed)) {
  NORMAL = 0,
  OVERFLOW = 1,
} OVERFLOW_STATE;

typedef enum __attribute__((packed)) {
  FORWARDS = 0,
  BACKWARDS = 1,
} DIRECTION;

typedef enum __attribute__((packed)) {
  LOW = 0b00,
  RISING = 0b01,
  FALLING = 0b10,
  HIGH = 0b11
} EDGE_STATE;

struct {
  button_t button_old;
  motor_t motor_aktiv;
  TV_MODE schritt_mode;
  POSITION_MODE position_mode;
  SPEED speed_level;
  uint32_t cycle_progress;
  struct {
    int32_t pos;
    int32_t power;
    uint32_t period;
    DIRECTION dir;
    OVERFLOW_STATE overflow;
    EDGE_STATE edge_c0;
    EDGE_STATE edge_c1;

  } motor[3];
} motor_data = {
    .motor_aktiv = MOTOR_A,
    .speed_level = MEDIUM,
    .cycle_progress = 0,
    .motor[0] =
        {
            .pos = 0,
            .power = 0,
            .period = 0,
            .dir = FORWARDS,
            .overflow = NORMAL,
            .edge_c0 = LOW,
            .edge_c1 = LOW,
        },
    .motor[1] =
        {
            .pos = 0,
            .power = 0,
            .period = 0,
            .dir = FORWARDS,
            .overflow = NORMAL,
            .edge_c0 = LOW,
            .edge_c1 = LOW,
        },
    .motor[2] =
        {
            .pos = 0,
            .power = 0,
            .period = 0,
            .dir = FORWARDS,
            .overflow = NORMAL,
            .edge_c0 = LOW,
            .edge_c1 = LOW,
        },
};
// Timer block and individual channel references
AT91PS_TCB timer = AT91C_BASE_TCB;
AT91PS_TC tc_a = AT91C_BASE_TC0;
AT91PS_TC tc_b = AT91C_BASE_TC1;
AT91PS_TC tc_c = AT91C_BASE_TC2;

// Anzeigen über v.view %e %spotlight motor_data

// Timer overflow motor A => TC0
void timer0_isr_entry(void) {          // 12
  MA.overflow = OVERFLOW;              // 4
  uint32_t pinChanges = *AT91C_TC0_SR; // 5
  MA.period = 0;                       // 4
  (void)pinChanges;                    // 0
}
//     total 25 instructions
//
//     25 * (1/(48 * 10^6 1/s))
// <=> 25 * 20ns = 500ns

// Timer overflow motor B => TC1
void timer1_isr_entry(void) {
  MB.overflow = OVERFLOW;
  uint32_t pinChanges = *AT91C_TC1_SR;
  MB.period = 0;
  (void)pinChanges;
}

// Timer overflow motor C => TC2
void timer2_isr_entry(void) {
  MC.overflow = OVERFLOW;
  uint32_t pinChanges = *AT91C_TC2_SR;
  MC.period = 0;
  (void)pinChanges;
}

// Shift old value to the left and set the new value
#define GET_EDGE_STATE(old, new) ((((old) & 0b01) << 1) | (new))
// GPIO interrupt triggerd by the edge detector
void gpio_isr_entry(void) {
  // Read & clear PIOA ISR - tells us which pins triggered the interrupt
  volatile uint32_t isr = *AT91C_PIOA_ISR; // 5
  uint32_t pin_status = *AT91C_PIOA_PDSR;  // 5

  // MOTOR A has changed
  if (isr & MASKA) {                                                   // 7
    MA.edge_c0 = GET_EDGE_STATE(MA.edge_c0, GET_BIT(pin_status, MA0)); // 16
    MA.edge_c1 = GET_EDGE_STATE(MA.edge_c1, GET_BIT(pin_status, MA1)); // 16
    // -1 for BACKWARDS, 0 for OVERFLOW, 1 for FORWARDS
    MA.pos += (1 - ((MA.dir << 1))) & -(MA.overflow == NORMAL); // 26
    if (MA.edge_c0 == RISING) {                                 // 7
      MA.period = tc_a->TC_CV;                                  // 7
      tc_a->TC_CCR = AT91C_TC_SWTRG;                            // 6
      MA.overflow = NORMAL;                                     // 4
      MA.dir = (MA.edge_c0 & 1) ^ (MA.edge_c1 & 1);             // 16
    }
  }
  // 105 instructions, if new cycle
  // 72 instructions, if no rising edge on c0 (indicates new cycle)
  // 7 instructions, if no change in motor

  // MOTOR B has changed
  if (isr & MASKB) {                                                   // 7
    MB.edge_c0 = GET_EDGE_STATE(MB.edge_c0, GET_BIT(pin_status, MB0)); // 16
    MB.edge_c1 = GET_EDGE_STATE(MB.edge_c1, GET_BIT(pin_status, MB1)); // 16
    MB.pos += (1 - ((MB.dir << 1))) & -(MB.overflow == NORMAL);        // 26
    if (MB.edge_c0 == RISING) {                                        // 7
      MB.period = tc_b->TC_CV;                                         // 7
      tc_b->TC_CCR = AT91C_TC_SWTRG;                                   // 6
      MB.overflow = NORMAL;                                            // 4
      MB.dir = (MB.edge_c0 & 1) ^ (MB.edge_c1 & 1);                    // 16
    } // 105
  }

  // MOTOR C has changed
  if (isr & MASKC) {                                                   // 7
    MC.edge_c0 = GET_EDGE_STATE(MC.edge_c0, GET_BIT(pin_status, MC0)); // 16
    MC.edge_c1 = GET_EDGE_STATE(MC.edge_c1, GET_BIT(pin_status, MC1)); // 16
    MC.pos += (1 - ((MC.dir << 1))) & -(MC.overflow == NORMAL);        // 26
    if (MC.edge_c0 == RISING) {                                        // 7
      MC.period = tc_c->TC_CV;                                         // 7
      tc_c->TC_CCR = AT91C_TC_SWTRG;                                   // 6
      MC.overflow = NORMAL;                                            // 4
      MC.dir = (MC.edge_c0 & 1) ^ (MC.edge_c1 & 1);                    // 16
    } // 105
  }
}
// Worst case: all 3 motors detect rising edge on ch0:
//     instructions: 10 + 3 * 105 = 325
//
//     325 * (1/(48 * 10^6 1/s))
// <=> 325 * 20ns = 6.5ms

int motor_init(void) {
  // Motor PWM 'aktivieren'
  // PWM-Signale laufen über den AVR-Prozessor
  // Initialisierung nicht notwendig, da dieser zuvor
  // bereits mittels nxt_avr_init()()ininitialisiert wurde

  // Motoren Stoppen und in Freilauf stzen
  nxt_avr_set_motor(MOTOR_A, 0, MOTOR_BREAK); // Motor A, PWM=0
  nxt_avr_set_motor(MOTOR_B, 0, MOTOR_BREAK); // Motor B, PWM=0
  nxt_avr_set_motor(MOTOR_C, 0, MOTOR_BREAK); // Motor C; PWM=0

  // GPIO Clock einschalten (bereits erledigt)
  // AIC  Clock einschalten (bereits erledigt)

  *AT91C_PIOA_PER = ENCODER_PINS;

  // Disable Pull up resistors
  *AT91C_PIOA_PPUDR = ENCODER_PINS;
  // Set Input Filter Enable Registe
  *AT91C_PIOA_IFER = ENCODER_PINS;
  // Read the  Interrupt Status Register
  // This also triggers a reset
  (void)*AT91C_PIOA_ISR;
  // Set Interrupt Enable Register
  *AT91C_PIOA_IER = ENCODER_PINS;

  // AIC Initialisieren
  aic_mask_off(AT91C_ID_PIOA);
  aic_set_vector(AT91C_ID_PIOA,
                 AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | // Art
                     AIC_INT_LEVEL_ABOVE_NORMAL,    // Prio
                 gpio_isr_entry);                   // ISR
  aic_mask_on(AT91C_ID_PIOA);

  // rising->rising auf TIOA: daher LDRB_RISING -> AT91C_TC_LDRA_RISING
  // CLKS = AT91C_TC_CLKS_TIMER_DIV3_CLOCK Also wenn das  MCK/32 ist

  // ### Timer-Peripherie Clock aktivieren (PMC PCER)
  *AT91C_PMC_PCER =
      (1 << AT91C_ID_TC0) | (1 << AT91C_ID_TC1) | (1 << AT91C_ID_TC2);

  // TC0 (Motor C)
  *AT91C_TC0_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_LDRA_RISING |
                   AT91C_TC_ABETRG | AT91C_TC_ETRGEDG_RISING;
  (void)*AT91C_PIOA_ISR;
  // Enable RB-load interrupt (LDRBS).
  *AT91C_TC0_IER = AT91C_TC_COVFS;
  // Start counter (CLKEN + SWTRG).
  *AT91C_TC0_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

  // TC1 (Motor A)
  *AT91C_TC1_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_LDRA_RISING |
                   AT91C_TC_ABETRG | AT91C_TC_ETRGEDG_RISING;

  (void)*AT91C_PIOA_ISR;
  *AT91C_TC1_IER = AT91C_TC_COVFS;
  *AT91C_TC1_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

  // TC2 (Motor B)
  *AT91C_TC2_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_LDRA_RISING |
                   AT91C_TC_ABETRG | AT91C_TC_ETRGEDG_RISING;
  (void)*AT91C_PIOA_ISR;
  *AT91C_TC2_IER = AT91C_TC_COVFS;
  *AT91C_TC2_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

  // AIC aus Aufgabe
  aic_mask_off(AT91C_ID_TC0);
  aic_set_vector(AT91C_ID_TC0,
                 AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | // Art
                     AIC_INT_LEVEL_ABOVE_NORMAL,    // Prio
                 timer0_isr_entry);                 // ISR
  aic_mask_on(AT91C_ID_TC0);

  aic_mask_off(AT91C_ID_TC1);
  aic_set_vector(AT91C_ID_TC1,
                 AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | // Art
                     AIC_INT_LEVEL_ABOVE_NORMAL,    // Prio
                 timer1_isr_entry);                 // ISR
  aic_mask_on(AT91C_ID_TC1);

  aic_mask_off(AT91C_ID_TC2);
  aic_set_vector(AT91C_ID_TC2,
                 AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | // Art
                     AIC_INT_LEVEL_ABOVE_NORMAL,    // Prio
                 timer2_isr_entry);                 // ISR
  aic_mask_on(AT91C_ID_TC2);

  return 0;
}

#define TIMER_FREQUENCY ((uint32_t)MCK / (uint32_t)32)
#define PULSES_PER_REV_RISING (uint32_t)180
#define SCALE (uint32_t)1000

#define SPEED_CONSTANT (TIMER_FREQUENCY * SCALE / PULSES_PER_REV_RISING)

int motor_get(motor_t port, int32_t *pos, int16_t *speed) {
  if (port > MOTOR_C)
    return -1;
  if (pos != NULL) {
    *pos = motor_data.motor[port].pos % 720;
  }
  if (speed != NULL) {
    uint32_t period = motor_data.motor[port].period;
    OVERFLOW_STATE state = motor_data.motor[port].overflow;
    if (state == OVERFLOW || period == 0) {
      *speed = 0;
    } else {
      int16_t calculated_speed = (int16_t)(SPEED_CONSTANT / period);
      if (motor_data.motor[port].dir == BACKWARDS) {
        calculated_speed = -calculated_speed;
      }
      *speed = calculated_speed;
    }
  }

  return 0;
}

// Funktion wird alle 16ms aufgerufen
void motor_process(void) {
  int32_t pos;
  int16_t speed;

  // Aktuellen Motor aus UI lesen
  motor_get(motor_data.motor_aktiv, &pos, &speed);

  // Variable mode
  if (motor_data.schritt_mode == TV_VARIABEL) {
    uint32_t cycle_duration_ms;
    cycle_duration_ms = SPEED_DURATION_MS[motor_data.speed_level];
    uint32_t total_steps = cycle_duration_ms >> 4;

    motor_data.cycle_progress++;
    if (motor_data.cycle_progress >= total_steps) {
      motor_data.cycle_progress = 0;
    }

    int32_t power;
    uint32_t half_steps = total_steps >> 1;
    if (motor_data.cycle_progress < half_steps) {
      power = -100 + (200 * motor_data.cycle_progress) / half_steps;
    } else {
      uint32_t progress_from_peak = motor_data.cycle_progress - half_steps;
      power = 100 - (200 * progress_from_peak) / half_steps;
    }

    if (power > 100)
      power = 100;
    if (power < -100)
      power = -100;

    CURRENT_MOTOR.power = power;
    nxt_avr_set_motor(MOTOR_INDEX, -CURRENT_MOTOR.power, MOTOR_BREAK);
  }

  // Trace Scope
  trace_scope(0, (int16_t)(CURRENT_MOTOR.power * 40));
  trace_scope(1, speed);
  trace_scope(2, (int16_t)pos);
}

// Funktion wird alle 64ms aufgerufen

#define ORANGE_PRESSED (!motor_data.button_old.orange && button_new.orange)
#define ORANGE_RELEASED (motor_data.button_old.orange && !button_new.orange)
#define LEFT_RELEASED (motor_data.button_old.left && !button_new.left)
#define RIGHT_RELEASED (motor_data.button_old.right && !button_new.right)
#define GREY_RELEASED (!motor_data.button_old.grey && button_new.grey)
void ui_process(void) {
  button_t button_new = nxt_avr_get_buttons();

  // Beispiel für Tastenauswertung
  // Orange Pressed
  if (ORANGE_PRESSED) {
    // (void)term_string("Orange pressed\n\r", ASYNCSYNC_NONBLOCK);
    //
  }
  // Orange Released
  if (ORANGE_RELEASED) {
    MOTOR_INDEX = (MOTOR_INDEX + 1) % 3;

    LOG("Motor: ");
    LOG_INT(motor_data.motor_aktiv);
    LOG("\n\r");
  }
  if (RIGHT_RELEASED) {
    if (motor_data.schritt_mode == TV_MANUELL) {
      CURRENT_MOTOR.power += 10;
      if (CURRENT_MOTOR.power > 100) {
        CURRENT_MOTOR.power = 100;
      }
      nxt_avr_set_motor(MOTOR_INDEX, -CURRENT_MOTOR.power, MOTOR_BREAK);
      LOG("Power: ");
      LOG_INT(CURRENT_MOTOR.power);
      LOG("\n\r");
    } else {
      if (motor_data.speed_level < SPEED_COUNT - 1) {
        motor_data.speed_level++;
      }
      LOG("Speed level: ");
      LOG_INT(motor_data.speed_level);
      LOG("\n\r");
    }
  }
  if (LEFT_RELEASED) {
    if (motor_data.schritt_mode == TV_MANUELL) {
      CURRENT_MOTOR.power -= 10;
      if (CURRENT_MOTOR.power < -100) {
        CURRENT_MOTOR.power = -100;
      }
      nxt_avr_set_motor(MOTOR_INDEX, -CURRENT_MOTOR.power, MOTOR_BREAK);
      LOG("Power: ");
      LOG_INT(CURRENT_MOTOR.power);
      LOG("\n\r");
    } else {
      if (motor_data.speed_level > 0) {
        motor_data.speed_level--;
      }
      LOG("Speed level: ");
      LOG_INT(motor_data.speed_level);
      LOG("\n\r");
    }
  }
  // Grey Released
  if (GREY_RELEASED) {
    motor_data.schritt_mode =
        motor_data.schritt_mode ? TV_MANUELL : TV_VARIABEL;
    if (motor_data.schritt_mode == TV_VARIABEL) {
      motor_data.cycle_progress = 0;
    }
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

  // Cooler code

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
