#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3

/* Betriebsmodi */
enum timer_counter_mode {
  TIMER_COUNTER_EVENT_COUNT = 1,      /* Externe Events zählen */
  TIMER_COUNTER_INTERVAL_MEASURE = 2, /* Zeitintervall messen */
  TIMER_COUNTER_PULSE_GENERATION = 3, /* Einzelpuls erzeugen */
  TIMER_COUNTER_PWM = 4,              /* PWM-Ausgabe */
};

/* Event-Typen für Benachrichtigungen */
enum timer_counter_event_id {
  TC_EVENT_COUNTER_OVERFLOW = 1,
  TC_EVENT_LOAD_A = 2,     /* RA geladen (Capture) */
  TC_EVENT_LOAD_B = 3,     /* RB geladen (Capture) */
  TC_EVENT_RC_COMPARE = 4, /* RC-Vergleich erreicht */
  TC_EVENT_EXT_TRIGGER = 5,
};

/* Clock-Auswahl (Teiler von MCK) */
enum timer_counter_clock_source {
  TC_CLOCK_MCK_DIV2 = 0,
  TC_CLOCK_MCK_DIV8 = 1,
  TC_CLOCK_MCK_DIV32 = 2,
  TC_CLOCK_MCK_DIV128 = 3,
  TC_CLOCK_MCK_DIV1024 = 4,
};

/* Flankenerkennung */
enum timer_counter_edge {
  TC_EDGE_NONE = 0,
  TC_EDGE_RISING = 1,
  TC_EDGE_FALLING = 2,
  TC_EDGE_BOTH = 3,
};

/* I/O-Pin-Auswahl */
enum timer_counter_io_pin {
  TC_PIN_TIOA = 0,
  TC_PIN_TIOB = 1,
};
