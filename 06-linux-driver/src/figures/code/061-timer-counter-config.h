#ifndef _TIMER_COUNTER_CONFIG_H
#define _TIMER_COUNTER_CONFIG_H

#include <linux/types.h>

/**
 * struct tc_event_count_config - Konfiguration für Event-Zählmodus
 *
 * @input_pin: Eingangs-Pin für Events (TIOA oder TIOB)
 * @edge: Welche Flanke(n) gezählt werden sollen
 * @threshold: Schwellwert für Event-Benachrichtigung (0 = bei Überlauf)
 * @padding: Reserviert für zukünftige Erweiterungen
 *
 * In diesem Modus wird der Zähler bei jeder erkannten Flanke auf dem
 * gewählten Eingangs-Pin inkrementiert. Der Zählerwert kann jederzeit
 * ausgelesen werden.
 */
struct tc_event_count_config {
  __u8 input_pin;    /* enum timer_counter_io_pin */
  __u8 edge;         /* enum timer_counter_edge */
  __u16 threshold;   /* Benachrichtigung wenn Zähler >= threshold */
  __u32 padding;
};

/**
 * struct tc_interval_measure_config - Konfiguration für Intervallmessung
 *
 * @start_edge: Flankentyp der die Messung startet
 * @stop_edge: Flankentyp der die Messung stoppt (lädt Register)
 * @input_pin: Eingangs-Pin für Flankenerkennung
 * @padding: Reserviert
 *
 * Misst das Zeitintervall zwischen Start- und Stop-Flanke. Der
 * gemessene Wert (in Clock-Ticks) wird in das RA- oder RB-Register
 * geladen.
 */
struct tc_interval_measure_config {
  __u8 start_edge;   /* enum timer_counter_edge */
  __u8 stop_edge;    /* enum timer_counter_edge */
  __u8 input_pin;    /* enum timer_counter_io_pin */
  __u8 padding[5];
};

/**
 * struct tc_pulse_generation_config - Konfiguration für Pulserzeugung
 *
 * @output_pin: Ausgangs-Pin für den Puls
 * @trigger_edge: Flanke die die Pulserzeugung auslöst
 * @delay_ticks: Ticks vom Trigger bis Pulsbeginn (RA-Wert)
 * @pulse_ticks: Pulsdauer in Ticks (RC - RA Wert)
 * @padding: Reserviert
 *
 * Erzeugt einen einzelnen Puls nach programmierbarer Verzögerung.
 * Die Pulsbreite wird durch die Differenz zwischen RC- und RA-Register
 * bestimmt.
 */
struct tc_pulse_generation_config {
  __u8 output_pin;       /* enum timer_counter_io_pin */
  __u8 trigger_edge;     /* enum timer_counter_edge */
  __u16 padding;
  __u32 delay_ticks;     /* Verzögerung vor Puls (mapped auf RA) */
  __u32 pulse_ticks;     /* Pulsbreite (mapped auf RC-RA) */
  __u32 padding2;
};

/**
 * struct tc_pwm_config - Konfiguration für PWM-Modus
 *
 * @period_ticks: PWM-Periode in Clock-Ticks (mapped auf RC-Register)
 * @duty_ticks: High-Zeit in Ticks (mapped auf RA für TIOA, RB für TIOB)
 * @output_pin: Ausgangs-Pin für PWM-Signal
 * @invert: Wenn != 0, Ausgangspolarität invertieren
 *
 * Erzeugt kontinuierliches PWM-Signal. Die Periode wird durch RC gesetzt,
 * das Tastverhältnis durch RA (für TIOA) oder RB (für TIOB).
 * Tastverhältnis in Prozent = (duty_ticks / period_ticks) * 100
 */
struct tc_pwm_config {
  __u32 period_ticks;    /* PWM-Periode (RC-Wert) */
  __u32 duty_ticks;      /* Tastverhältnis-Dauer (RA oder RB Wert) */
  __u8 output_pin;       /* enum timer_counter_io_pin */
  __u8 invert;           /* Ausgangspolarität invertieren */
  __u16 padding;
};

#endif /* _TIMER_COUNTER_CONFIG_H */
