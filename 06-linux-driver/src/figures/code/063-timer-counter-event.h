#include <linux/types.h>

/* Event-Struktur (gelesen via read() auf req.fd) */
struct tc_event {
  __aligned_u64 timestamp_ns; /* CLOCK_MONOTONIC */
  __u32 event_id;             /* enum timer_counter_event_id */
  __u32 channel;
  __u16 counter_value; /* CV-Register */
  __u16 capture_a;     /* RA-Register */
  __u16 capture_b;     /* RB-Register */
};

/* Direktes Zähler-Lesen */
struct tc_counter_value {
  __u8 channel;
  __u16 value;
};
