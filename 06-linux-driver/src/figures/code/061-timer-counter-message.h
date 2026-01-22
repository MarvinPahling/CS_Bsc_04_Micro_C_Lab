#include <linux/types.h>

#define MAX_TIMER_COUNTER_CH 3

/**
 * The Timer Counter (TC) includes three identical 16-bit Timer Counter
channels.
*
* Each channel can be independently programmed to perform a wide range of
functions including frequency measurement, event counting, interval measurement,
pulse generation, delay timing and pulse width modulation.

Each channel has three external clock inputs, five internal clock inputs and two
multi-purpose input/output signals which can be configured by the user. Each
channel drives an internal inter- rupt signal which can be programmed to
generate processor interrupts
 * */

/**
 * struct timer_counter_channel -
 */
struct timer_counter_channel {
  __u16 value;
};

// TODO select
struct timer_counter_chip {
  __u16 devider;
  struct timer_counter_channel chan[MAX_TIMER_COUNTER_CH];
};

struct timer_counter_msg {
  __u16 devider;
};

struct timer_counter_transaction {
  __u16 devider;
};

struct timer_counter_event {
  __u16 devider;
};
