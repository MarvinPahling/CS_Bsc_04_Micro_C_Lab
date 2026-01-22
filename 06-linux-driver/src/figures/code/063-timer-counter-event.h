#ifndef _TIMER_COUNTER_EVENT_H
#define _TIMER_COUNTER_EVENT_H

#include <linux/types.h>

/**
 * struct tc_event - Timer-Counter Event an Userspace
 *
 * @timestamp_ns: Zeitstempel des Events in Nanosekunden (CLOCK_MONOTONIC)
 * @event_id: Event-Typ aus enum timer_counter_event_id
 * @channel: Kanal der dieses Event erzeugt hat (0-2)
 * @seqno: Sequenznummer für Event-Reihenfolge
 * @counter_value: Zählerwert zum Zeitpunkt des Events (16-bit)
 * @capture_a: Wert des RA-Registers (Capture-Modus)
 * @capture_b: Wert des RB-Registers (Capture-Modus)
 * @padding: Reserviert für zukünftige Erweiterungen
 *
 * Events werden vom fd gelesen, der von TC_GET_CHANNEL_IOCTL zurückgegeben
 * wird, mittels blockierendem read() Aufruf, analog zu GPIO-Events.
 */
struct tc_event {
  __aligned_u64 timestamp_ns;
  __u32 event_id;        /* enum timer_counter_event_id */
  __u32 channel;
  __u32 seqno;
  __u16 counter_value;   /* Aktueller Zähler (CV-Register) */
  __u16 capture_a;       /* RA-Registerwert */
  __u16 capture_b;       /* RB-Registerwert */
  __u16 padding[3];
};

/**
 * struct tc_counter_value - Für direktes Zähler-Lesen/Schreiben
 *
 * @channel: Kanalnummer
 * @value: Zählerwert zum Setzen oder der gelesen wurde
 * @padding: Reserviert
 *
 * Wird mit read() auf dem Event-fd verwendet um den aktuellen
 * Zählerwert direkt abzufragen ohne auf ein Event zu warten.
 */
struct tc_counter_value {
  __u8 channel;
  __u8 padding;
  __u16 value;
};

#endif /* _TIMER_COUNTER_EVENT_H */
