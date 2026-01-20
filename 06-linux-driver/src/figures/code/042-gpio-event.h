#include <linux/types.h>

/**
 * enum gpio_v2_line_event_id - &struct gpio_v2_line_event.id values
 * @GPIO_V2_LINE_EVENT_RISING_EDGE: event triggered by a rising edge
 * @GPIO_V2_LINE_EVENT_FALLING_EDGE: event triggered by a falling edge
 */
enum gpio_v2_line_event_id {
  GPIO_V2_LINE_EVENT_RISING_EDGE = 1,
  GPIO_V2_LINE_EVENT_FALLING_EDGE = 2,
};

/**
 * struct gpio_v2_line_event - The actual event being pushed to userspace
 * @timestamp_ns: best estimate of time of event occurrence, in nanoseconds
 * @id: event identifier with value from &enum gpio_v2_line_event_id
 * @offset: the offset of the line that triggered the event
 * @seqno: the sequence number for this event in the sequence of events for
 * all the lines in this line request
 * @line_seqno: the sequence number for this event in the sequence of
 * events on this particular line
 * @padding: reserved for future use
 *
 * By default the @timestamp_ns is read from %CLOCK_MONOTONIC and is
 * intended to allow the accurate measurement of the time between events.
 * It does not provide the wall-clock time.
 *
 * If the %GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME flag is set then the
 * @timestamp_ns is read from %CLOCK_REALTIME.
 *
 * If the %GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE flag is set then the
 * @timestamp_ns is provided by the hardware timestamping engine (HTE)
 * subsystem.
 */
struct gpio_v2_line_event {
  __aligned_u64 timestamp_ns;
  __u32 id;
  __u32 offset;
  __u32 seqno;
  __u32 line_seqno;
  /* Space reserved for future use. */
  __u32 padding[6];
};
