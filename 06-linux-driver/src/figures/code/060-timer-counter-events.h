#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3
/**
 * enum timer_counter_mode - this reprents all 4 modes supported by this driver
 *
 * @TIMER_COUNTER_EVENT_COUNT: TODO,
 * @TIMER_COUNTER_INTERVAL_MEASURE: TODO,
 * @TIMER_COUNTER_PULSE_GENERATION: TODO,
 * @TIMER_COUNTER_PWM: TODO,
 */
enum timer_counter_mode {
  TIMER_COUNTER_EVENT_COUNT = 1,
  TIMER_COUNTER_INTERVAL_MEASURE = 2,
  TIMER_COUNTER_PULSE_GENERATION = 3,
  TIMER_COUNTER_PWM = 4,
};
