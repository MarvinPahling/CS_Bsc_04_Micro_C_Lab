#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3
#define TC_MAX_NAME_SIZE 32

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

/*
 * Modus-spezifische Konfigurationsstrukturen
 */

/* Konfiguration für Event-Zählmodus */
struct tc_event_count_config {
  __u8 input_pin;  /* enum timer_counter_io_pin */
  __u8 edge;       /* enum timer_counter_edge */
  __u16 threshold; /* Event bei Zähler >= threshold */
};

/* Konfiguration für Intervallmessung */
struct tc_interval_measure_config {
  __u8 start_edge; /* enum timer_counter_edge */
  __u8 stop_edge;  /* enum timer_counter_edge */
  __u8 input_pin;  /* enum timer_counter_io_pin */
};

/* Konfiguration für Pulserzeugung */
struct tc_pulse_generation_config {
  __u8 output_pin;   /* enum timer_counter_io_pin */
  __u8 trigger_edge; /* enum timer_counter_edge */
  __u32 delay_ticks; /* Verzögerung vor Puls (RA) */
  __u32 pulse_ticks; /* Pulsbreite (RC-RA) */
};

/* Konfiguration für PWM-Modus */
struct tc_pwm_config {
  __u32 period_ticks; /* PWM-Periode (RC) */
  __u32 duty_ticks;   /* High-Zeit (RA/RB) */
  __u8 output_pin;    /* enum timer_counter_io_pin */
  __u8 invert;        /* Polarität invertieren */
};

/*
 * Kanal-Konfiguration
 */
struct tc_channel_config {
  __u8 channel;       /* 0, 1 oder 2 */
  __u8 mode;          /* enum timer_counter_mode */
  __u8 clock_source;  /* enum timer_counter_clock_source */
  __u8 enable_events; /* Bitmaske für Event-Benachrichtigungen */
  union {
    struct tc_event_count_config event_count;
    struct tc_interval_measure_config interval;
    struct tc_pulse_generation_config pulse;
    struct tc_pwm_config pwm;
  };
};

/*
 * Konfigurationsanfrage (analog zu gpio_v2_line_request)
 */
struct tc_request {
  struct tc_channel_config channels[MAX_TIMER_COUNTER_CH];
  char consumer[TC_MAX_NAME_SIZE];
  __u32 num_channels;
  __s32 fd; /* Output: fd für Events */
};

/*
 * Event-Struktur (gelesen via read() auf req.fd)
 */
struct tc_event {
  __aligned_u64 timestamp_ns; /* CLOCK_MONOTONIC */
  __u32 event_id;             /* enum timer_counter_event_id */
  __u32 channel;
  __u16 counter_value; /* CV-Register */
  __u16 capture_a;     /* RA-Register */
  __u16 capture_b;     /* RB-Register */
};

/*
 * Direktes Zähler-Lesen
 */
struct tc_counter_value {
  __u8 channel;
  __u16 value;
};

/*
 * ioctl-Kommandos
 */
#define TC_IOC_MAGIC 'T'
#define TC_GET_CHANNEL_IOCTL _IOWR(TC_IOC_MAGIC, 0x01, struct tc_request)
#define TC_START_IOCTL _IO(TC_IOC_MAGIC, 0x03)
#define TC_STOP_IOCTL _IO(TC_IOC_MAGIC, 0x04)
