#include "064-timer-counter.h"

#include <sys/ioctl.h>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>

#define TC_DEVICE "/dev/tc0"
#define PWM_CHANNEL 0
#define PWM_PERIOD 1500
#define PWM_DUTY 750

int main() {
  int fd;

  struct tc_request req = {.num_channels = 1,
                           .consumer = "pwm-beispiel",
                           .channels[0] = {.channel = PWM_CHANNEL,
                                           .mode = TIMER_COUNTER_PWM,
                                           .clock_source = TC_CLOCK_MCK_DIV32,
                                           .pwm = {.period_ticks = PWM_PERIOD,
                                                   .duty_ticks = PWM_DUTY,
                                                   .output_pin = TC_PIN_TIOA}}};

  if ((fd = open(TC_DEVICE, O_RDWR)) < 0)
    return -1;

  if (ioctl(fd, TC_GET_CHANNEL_IOCTL, &req) < 0) {
    close(fd);
    return -1;
  }

  if (ioctl(req.fd, TC_START_IOCTL) < 0) {
    close(req.fd);
    close(fd);
    return -1;
  }

  printf("PWM aktiv: %d Ticks Periode, %d Ticks Duty (%.0f%%)\n", PWM_PERIOD,
         PWM_DUTY, (float)PWM_DUTY / PWM_PERIOD * 100);

  sleep(10);

  ioctl(req.fd, TC_STOP_IOCTL);
  close(req.fd);
  close(fd);
  return 0;
}
