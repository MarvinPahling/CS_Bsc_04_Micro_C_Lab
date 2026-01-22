#include "061-timer-counter-config.h"
#include <linux/ioctl.h>
#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3
#define TC_MAX_NAME_SIZE 32

/* Kanal-Konfiguration */
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

/* Konfigurationsanfrage (analog zu gpio_v2_line_request) */
struct tc_request {
  struct tc_channel_config channels[MAX_TIMER_COUNTER_CH];
  char consumer[TC_MAX_NAME_SIZE];
  __u32 num_channels;
  __s32 fd; /* Output: fd für Events */
};

/* ioctl Kommandos */
#define TC_IOC_MAGIC 'T'
#define TC_GET_CHANNEL_IOCTL _IOWR(TC_IOC_MAGIC, 0x01, struct tc_request)
#define TC_START_IOCTL _IO(TC_IOC_MAGIC, 0x03)
#define TC_STOP_IOCTL _IO(TC_IOC_MAGIC, 0x04)
