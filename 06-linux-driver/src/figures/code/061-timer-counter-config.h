#include <linux/types.h>

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
