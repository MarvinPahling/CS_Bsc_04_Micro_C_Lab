#ifndef _TIMER_COUNTER_REQUEST_H
#define _TIMER_COUNTER_REQUEST_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define MAX_TIMER_COUNTER_CH 3
#define TC_MAX_NAME_SIZE 32

/**
 * struct tc_channel_config - Konfiguration für einen Timer-Counter-Kanal
 *
 * @channel: Kanalnummer (0, 1 oder 2)
 * @mode: Betriebsmodus aus enum timer_counter_mode
 * @clock_source: Clock-Auswahl aus enum timer_counter_clock_source
 * @enable_events: Bitmaske für aktivierte Event-Benachrichtigungen
 * @config: Mode-spezifische Konfiguration (Union)
 *
 * Diese Struktur konfiguriert einen einzelnen Kanal. Mehrere Kanäle
 * können atomar über tc_request mit num_channels > 1 konfiguriert werden.
 */
struct tc_channel_config {
  __u8 channel;       /* 0, 1 oder 2 */
  __u8 mode;          /* enum timer_counter_mode */
  __u8 clock_source;  /* enum timer_counter_clock_source */
  __u8 enable_events; /* Bitmaske: welche Events Benachrichtigungen erzeugen */
  __u32 padding;

  /* Mode-spezifische Konfiguration als Union */
  union {
    struct tc_event_count_config event_count;
    struct tc_interval_measure_config interval;
    struct tc_pulse_generation_config pulse;
    struct tc_pwm_config pwm;
    __u8 reserved[24]; /* Einheitliche Größe sicherstellen */
  };
};

/**
 * struct tc_request - Timer-Counter Konfigurationsanfrage
 *
 * @channels: Array von Kanal-Konfigurationen
 * @consumer: Label zur Identifikation des anfragenden Prozesses
 * @num_channels: Anzahl gültiger Einträge im channels-Array (1-3)
 * @event_buffer_size: Vorgeschlagene Event-Puffergröße
 * @padding: Reserviert für zukünftige Erweiterungen
 * @fd: [OUTPUT] File-Deskriptor zum Lesen von Events/Zählerwerten
 *
 * Analog zu gpio_v2_line_request wird diese Struktur an TC_GET_CHANNEL_IOCTL
 * übergeben. Nach erfolgreichem ioctl enthält das fd-Feld einen
 * File-Deskriptor zum Lesen von Events und Zählerwerten.
 */
struct tc_request {
  struct tc_channel_config channels[MAX_TIMER_COUNTER_CH];
  char consumer[TC_MAX_NAME_SIZE];
  __u32 num_channels;
  __u32 event_buffer_size;
  __u32 padding[4];
  __s32 fd;  /* Output: fd für Events/Reads */
};

/* ioctl Kommando-Definitionen */
#define TC_IOC_MAGIC 'T'

/**
 * TC_GET_CHANNEL_IOCTL - Konfiguriere Kanäle und erhalte Event-fd
 *
 * Konfiguriert einen oder mehrere Timer-Counter-Kanäle und gibt einen
 * File-Deskriptor zum Lesen von Events und Zählerwerten zurück.
 * Analog zu GPIO_V2_GET_LINE_IOCTL.
 */
#define TC_GET_CHANNEL_IOCTL _IOWR(TC_IOC_MAGIC, 0x01, struct tc_request)

/**
 * TC_START_IOCTL - Starte den Timer-Counter
 *
 * Löst Software-Start für konfigurierte Kanäle aus.
 */
#define TC_START_IOCTL _IO(TC_IOC_MAGIC, 0x03)

/**
 * TC_STOP_IOCTL - Stoppe den Timer-Counter
 *
 * Stoppt den Zähler ohne die Konfiguration zu löschen.
 */
#define TC_STOP_IOCTL _IO(TC_IOC_MAGIC, 0x04)

#endif /* _TIMER_COUNTER_REQUEST_H */
