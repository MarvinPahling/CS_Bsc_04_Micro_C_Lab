#ifndef _TIMER_COUNTER_EVENTS_H
#define _TIMER_COUNTER_EVENTS_H

#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3

/**
 * enum timer_counter_mode - Betriebsmodi des Timer-Counter-Treibers
 *
 * @TIMER_COUNTER_EVENT_COUNT: Zählt externe Events auf Eingangs-Pin TIOA/TIOB
 * @TIMER_COUNTER_INTERVAL_MEASURE: Misst Zeitintervall zwischen Flanken
 * @TIMER_COUNTER_PULSE_GENERATION: Erzeugt getakteten Ausgangspuls auf TIOA/TIOB
 * @TIMER_COUNTER_PWM: Pulsweitenmodulation auf TIOA (Periode) / TIOB (Duty)
 */
enum timer_counter_mode {
  TIMER_COUNTER_EVENT_COUNT = 1,
  TIMER_COUNTER_INTERVAL_MEASURE = 2,
  TIMER_COUNTER_PULSE_GENERATION = 3,
  TIMER_COUNTER_PWM = 4,
};

/**
 * enum timer_counter_event_id - Event-Typen für asynchrone Benachrichtigung
 *
 * @TC_EVENT_COUNTER_OVERFLOW: Zählerüberlauf aufgetreten
 * @TC_EVENT_LOAD_A: Register A geladen (Capture-Modus)
 * @TC_EVENT_LOAD_B: Register B geladen (Capture-Modus)
 * @TC_EVENT_RC_COMPARE: Zähler hat RC-Vergleichswert erreicht
 * @TC_EVENT_EXT_TRIGGER: Externer Trigger aufgetreten
 */
enum timer_counter_event_id {
  TC_EVENT_COUNTER_OVERFLOW = 1,
  TC_EVENT_LOAD_A = 2,
  TC_EVENT_LOAD_B = 3,
  TC_EVENT_RC_COMPARE = 4,
  TC_EVENT_EXT_TRIGGER = 5,
};

/**
 * enum timer_counter_clock_source - Clock-Eingangsauswahl
 *
 * Interne Clocks abgeleitet von MCK (Master Clock):
 * @TC_CLOCK_MCK_DIV2: MCK/2
 * @TC_CLOCK_MCK_DIV8: MCK/8
 * @TC_CLOCK_MCK_DIV32: MCK/32
 * @TC_CLOCK_MCK_DIV128: MCK/128
 * @TC_CLOCK_MCK_DIV1024: MCK/1024 (SLCK bei manchen Varianten)
 *
 * Externe Clocks:
 * @TC_CLOCK_XC0: Externer Clock-Eingang 0
 * @TC_CLOCK_XC1: Externer Clock-Eingang 1
 * @TC_CLOCK_XC2: Externer Clock-Eingang 2
 */
enum timer_counter_clock_source {
  TC_CLOCK_MCK_DIV2 = 0,
  TC_CLOCK_MCK_DIV8 = 1,
  TC_CLOCK_MCK_DIV32 = 2,
  TC_CLOCK_MCK_DIV128 = 3,
  TC_CLOCK_MCK_DIV1024 = 4,
  TC_CLOCK_XC0 = 5,
  TC_CLOCK_XC1 = 6,
  TC_CLOCK_XC2 = 7,
};

/**
 * enum timer_counter_edge - Flankenerkennung
 *
 * @TC_EDGE_NONE: Keine Flankenerkennung
 * @TC_EDGE_RISING: Steigende Flanke erkennen
 * @TC_EDGE_FALLING: Fallende Flanke erkennen
 * @TC_EDGE_BOTH: Beide Flanken erkennen
 */
enum timer_counter_edge {
  TC_EDGE_NONE = 0,
  TC_EDGE_RISING = 1,
  TC_EDGE_FALLING = 2,
  TC_EDGE_BOTH = 3,
};

/**
 * enum timer_counter_io_pin - I/O-Pin-Auswahl (TIOA oder TIOB)
 *
 * @TC_PIN_TIOA: TIOA-Pin für Ein-/Ausgabe verwenden
 * @TC_PIN_TIOB: TIOB-Pin für Ein-/Ausgabe verwenden
 */
enum timer_counter_io_pin {
  TC_PIN_TIOA = 0,
  TC_PIN_TIOB = 1,
};

#endif /* _TIMER_COUNTER_EVENTS_H */
