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
#define LOG_BIN(msg) (void)term_string(msg, 1, ASYNCSYNC_NONBLOCK)

// Macro to extract a single bit from a register
#define GET_BIT(reg, bit) (((reg) >> (bit)) & 1)

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
  volatile uint32_t pin_status;
  struct {
    volatile int32_t pos;
    volatile int8_t dir;
    volatile uint8_t ch0;
    volatile uint8_t ch1;
    volatile uint8_t ch0_prev;
    volatile uint8_t ch1_prev;
    volatile uint32_t duty;
    volatile uint32_t pin_status;
    volatile int32_t speed;
    volatile int32_t counter;
    volatile int32_t power;
    volatile int32_t timer;
    volatile int32_t prevTimer;
    volatile uint8_t overflow_f;
    volatile uint8_t
        edge_state; // State counter: 0-2 for 3 edges (2 GPIO + 1 Timer)
    volatile int32_t edge_times[3]; // Timer values for each of 4 edges
    volatile uint32_t pio_status;   // Current PIO pin status
    volatile uint32_t isr;

  } motor[3];
} motor_data = {
    .motor_aktiv = MOTOR_A,
    .motor[0] =
        {
            .pos = 0,
            .dir = 0,
            .ch0 = 0,
            .ch1 = 0,
            .ch0_prev = 0,
            .ch1_prev = 0,
            .duty = 0,
            .pin_status = 0,
            .speed = 0,
            .counter = 0,
            .power = 0,
            .timer = 0,
            .prevTimer = 0,
            .overflow_f = 0,
            .edge_state = 0,
            .edge_times = {0, 0, 0},
            .pio_status = 0,
            .isr = 0,
        },
    .motor[1] =
        {
            .pos = 0,
            .dir = 0,
            .ch0 = 0,
            .ch1 = 0,
            .ch0_prev = 0,
            .ch1_prev = 0,
            .duty = 0,
            .pin_status = 0,
            .speed = 0,
            .counter = 0,
            .power = 0,
            .timer = 0,
            .prevTimer = 0,
            .overflow_f = 0,
            .edge_state = 0,
            .edge_times = {0, 0, 0},
            .pio_status = 0,
            .isr = 0,
        },
    .motor[2] =
        {
            .pos = 0,
            .dir = 0,
            .ch0 = 0,
            .ch1 = 0,
            .ch0_prev = 0,
            .ch1_prev = 0,
            .duty = 0,
            .pin_status = 0,
            .speed = 0,
            .counter = 0,
            .power = 0,
            .timer = 0,
            .prevTimer = 0,
            .overflow_f = 0,
            .edge_state = 0,
            .edge_times = {0, 0, 0},
            .pio_status = 0,
            .isr = 0,
        },
};
// Timer block and individual channel references
//

AT91PS_TCB timer = AT91C_BASE_TCB;
AT91PS_TC tc_a = AT91C_BASE_TC0;
AT91PS_TC tc_b = AT91C_BASE_TC1;
AT91PS_TC tc_c = AT91C_BASE_TC2;

// Anzeigen über v.view %e %spotlight motor_data

void timer0_isr_entry(void) { // TC0 => Motor A
  // Only for timer overflow for motor speed == 0
  MA.overflow_f = 1;
  uint32_t pinChanges = *AT91C_TC0_SR;
  motor_data.motor[0].counter++;
  (void)pinChanges;
  motor_data.motor[0].timer = 0;
  motor_data.motor[0].prevTimer = 0;
  motor_data.motor[0].overflow_f = 1;
  motor_data.motor[0].edge_state = 0; // Reset state machine
}

void timer1_isr_entry(void) { // TC1 => Motor B
  // Only for timer overflow for motor speed == 0
  MA.overflow_f = 1;
  uint32_t pinChanges = *AT91C_TC1_SR;
  motor_data.motor[1].counter++;
  (void)pinChanges;
  motor_data.motor[1].timer = 0;
  motor_data.motor[1].prevTimer = 0;
  motor_data.motor[1].overflow_f = 1;
  motor_data.motor[1].edge_state = 0; // Reset state machine
}

void timer2_isr_entry(void) { // TC2 => Motor C
  // Only for timer overflow for motor speed == 0
  MA.overflow_f = 1;
  uint32_t pinChanges = *AT91C_TC2_SR;
  motor_data.motor[2].counter++;
  (void)pinChanges;
  motor_data.motor[2].timer = 0;
  motor_data.motor[2].prevTimer = 0;
  motor_data.motor[2].overflow_f = 1;
  motor_data.motor[2].edge_state = 0; // Reset state machine
}

void gpio_isr_entry(void) {
  // uint32_t i_state = interrupts_get_and_disable();

  // Read & clear PIOA ISR
  CURRENT_MOTOR.isr = *AT91C_PIOA_ISR;
  motor_data.pin_status = *AT91C_PIOA_PDSR;

  // MOTOR A
  MA.ch0_prev = MA.ch0;
  MA.ch1_prev = MA.ch1;
  MA.ch0 = GET_BIT(motor_data.pin_status, MA0);
  MA.ch1 = GET_BIT(motor_data.pin_status, MA1);

  // Rising Edge on CH0
  if ((!MA.ch0_prev) & MA.ch0) {
    MA.dir = MA.ch0 ^ MA.ch1;
  }
  MA.pos += MA.dir ? 1 : -1;
  MA.edge_state++;
  if (MA.edge_state == 2) {
    // Read timer
    MA.duty = tc_a->TC_CV;
    // Reset timer
    tc_a->TC_CCR = AT91C_TC_SWTRG;
    // Reset edge state
    MA.edge_state = 0;
  }

  // MOTOR B
  MB.ch0_prev = MB.ch0;
  MB.ch1_prev = MB.ch1;
  MB.ch0 = GET_BIT(motor_data.pin_status, MB0);
  MB.ch1 = GET_BIT(motor_data.pin_status, MB1);

  // Rising Edge on CH0
  if ((!MB.ch0_prev) & MB.ch0) {
    MB.dir = MB.ch0 ^ MB.ch1;
  }
  MB.pos += MB.dir ? 1 : -1;

  // MOTOR C
  MC.ch0_prev = MC.ch0;
  MC.ch1_prev = MC.ch1;
  MC.ch0 = GET_BIT(motor_data.pin_status, MC0);
  MC.ch1 = GET_BIT(motor_data.pin_status, MC1);

  // Rising Edge on CH0
  if ((!MC.ch0_prev) & MC.ch0) {
    MC.dir = MC.ch0 ^ MC.ch1;
  }
  MC.pos += MC.dir ? 1 : -1;

  // Rest after overflow
  // if (CURRENT_MOTOR.overflow_f && CURRENT_MOTOR.edge_state == 0) {
  //   CURRENT_MOTOR.overflow_f = 0;
  //   CURRENT_MOTOR.edge_times[0] = 0;
  //   CURRENT_MOTOR.edge_times[1] = 0;
  //   CURRENT_MOTOR.edge_times[2] = 0;
  //   CURRENT_MOTOR.edge_times[3] = 0;
  // }

  // Read the current PIO pin status
  // uint32_t currentPins = *AT91C_PIOA_PDSR;
  // CURRENT_MOTOR.pio_status = currentPins;
  // Read the Timer Counter for the selected channel (Only when not on overflow)
  // if (!CURRENT_MOTOR.overflow_f) {
  //   // int32_t timer_value = timer_ch[MOTOR_INDEX]->TC_CV;
  //   CURRENT_MOTOR.edge_times[CURRENT_MOTOR.edge_state] = timer_value;
  // }
  // // Rolling Index for the state (011 <=> modulo 4)
  // CURRENT_MOTOR.edge_state = (CURRENT_MOTOR.edge_state + 1) & 0b011;
  //
  // // Reset the timer counter
  // timer_ch[MOTOR_INDEX]->TC_CCR = AT91C_TC_SWTRG;

  // // Wenn seit dem letzten Lesen keine Veränderung festgestellt wurde wird
  // // abgebrochen
  // if ((pinChanges & ENCODER_PINS) == 0) {
  //   return;
  // }

  // aktiven Motor wählen
  // int a_pin = -1, b_pin = -1, motor_index = -1;

  if (CURRENT_MOTOR.isr & (1 << MA0)) {
    // position+=   EXOR
    // Wahrscheinlich falsch
    motor_data.motor[0].pos += MA0 ^ MB0;

    // AT91S_TC tc0 = (AT91S_TC) * (AT91C_BASE_TC0);
    // uint32_t counter = tc0.TC_RA;

    // LDRA lesen
    // *AT91C_TC_LDRAS
    // status++
  }
  // switch (motor_data.motor_aktiv) {
  // case MOTOR_A:
  //   a_pin = MA0;
  //   b_pin = MA1;
  //   motor_index = 0;
  //   break;
  // case MOTOR_B:
  //   a_pin = MB0;
  //   b_pin = MB1;
  //   motor_index = 1;
  //   break;
  // case MOTOR_C:
  //   a_pin = MC0;
  //   b_pin = MC1;
  //   motor_index = 2;
  //   break;
  // default:
  //   return;
  // }

  // // keine Änderung an diesem Motor
  // uint32_t mask = ((1u << a_pin) | (1u << b_pin));
  // if ((CURRENT_MOTOR.isr & mask) == 0)
  //   return;
  //
  // /* vorheriger 2-Bit-State (A<<1 | B) */
  // uint8_t prev = (uint8_t)(motor_data.motor[motor_index].dir & 0x3);
  //
  // // aktuellen Pegel holen (A<<1 | B)
  // uint8_t curr = (uint8_t)((((currentPins >> a_pin) & 1) << 1) |
  //                          ((currentPins >> b_pin) & 1));

  // /* Einzelbits */
  // uint8_t prevA = (prev >> 1) & 1;
  // uint8_t prevB = prev & 1;
  // uint8_t currA = (curr >> 1) & 1;
  // uint8_t currB = curr & 1;

  // // Erkennung welche Pin-Flanke
  // uint8_t edgeA =
  //     (uint8_t)((CURRENT_MOTOR.isr >> a_pin) & 1); // 1 == A hat gewechselt
  // uint8_t edgeB =
  //     (uint8_t)((CURRENT_MOTOR.isr >> b_pin) & 1); // 1 == B hat gewechselt
  //
  // if (edgeA && !edgeB) {
  //   // A-edge: rising? -> eA
  //   uint8_t eA = currA & (uint8_t)(~prevA & 1); // 1 bei rising A
  //
  //   // delta = +1 oder -1
  //   int32_t delta = 1 - ((int32_t)((eA ^ currB) << 1)); // z.B. 1 -
  //   2*(eA^currB) motor_data.motor[motor_index].pos += delta;
  //   motor_data.motor[motor_index].dir = curr;
  //   return;
  // }
  //
  // if (edgeB && !edgeA) {
  //   uint8_t eB = currB & (uint8_t)(~prevB & 1); // 1 bei rising B
  //   /* Für B-edges verwenden wir currA inverted (a^1) */
  //   uint8_t a_inverted = currA ^ 1;
  //   int32_t delta =
  //       1 - ((int32_t)((eB ^ a_inverted) << 1)); /* 1 - 2*(eB ^ ~currA) */
  //   motor_data.motor[motor_index].pos += delta;
  //   motor_data.motor[motor_index].dir = curr;
  //   return;
  // }
  // return;
}

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

#define ORANGE_PRESSED (!motor_data.button_old.orange && button_new.orange)
#define ORANGE_RELEASED (motor_data.button_old.orange && !button_new.orange)
#define LEFT_RELEASED (motor_data.button_old.left && !button_new.left)
#define RIGHT_RELEASED (motor_data.button_old.right && !button_new.right)
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

    // Print
    LOG("Motor: ");
    LOG_INT(motor_data.motor_aktiv);
    LOG("\n\r");
  }
  if (RIGHT_RELEASED) {
    CURRENT_MOTOR.power += 10;
    if (CURRENT_MOTOR.power > 100) {
      CURRENT_MOTOR.power = 100;
    }
    nxt_avr_set_motor(MOTOR_INDEX, CURRENT_MOTOR.power, MOTOR_BREAK);
    LOG("Power: ");
    LOG_INT(CURRENT_MOTOR.power);
    LOG("\n\r");
  }
  if (LEFT_RELEASED) {
    CURRENT_MOTOR.power -= 10;
    if (CURRENT_MOTOR.power < -100) {
      CURRENT_MOTOR.power = -100;
    }
    nxt_avr_set_motor(MOTOR_INDEX, CURRENT_MOTOR.power, MOTOR_BREAK);
    LOG("Power: ");
    LOG_INT(CURRENT_MOTOR.power);
    LOG("\n\r");
  }
  // Grey Released
  if (!motor_data.button_old.grey && button_new.grey) {
    LOG("Counter: ");
    LOG_INT(CURRENT_MOTOR.counter);
    LOG("\n\r");

    // LOG("State: ");
    // LOG_INT(CURRENT_MOTOR.edge_state);
    // LOG("\n\r");

    LOG("pos: ");
    LOG_INT(CURRENT_MOTOR.pos % 720);
    LOG("\n\r");

    // LOG("T: ");
    // LOG_INT(CURRENT_MOTOR.edge_times[0]);
    // LOG(", ");
    // LOG_INT(CURRENT_MOTOR.edge_times[1]);
    // LOG(", ");
    // LOG_INT(CURRENT_MOTOR.edge_times[2]);
    // LOG(", ");
    // LOG_INT(CURRENT_MOTOR.edge_times[3]);
    // LOG("\n\r");
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
