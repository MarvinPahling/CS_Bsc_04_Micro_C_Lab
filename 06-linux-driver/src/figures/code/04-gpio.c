#include <linux/gpio.h>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>

#define GPIO_DEVICE "/dev/gpiochip0"
#define GPIO_LINE 7
#define CONSUMER_LABEL "gpio-monitor"

int main() {
  int fd;
  struct gpio_v2_line_event event;

  struct gpio_v2_line_request req = {
      .offsets = {GPIO_LINE},
      .num_lines = 1,
      .config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING |
                      GPIO_V2_LINE_FLAG_EDGE_FALLING,
      .consumer = CONSUMER_LABEL};

  if ((fd = open(GPIO_DEVICE, O_RDWR)) < 0)
    return -1;

  if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
    close(fd);
    return -1;
  }

  while (1) {
    if (read(req.fd, &event, sizeof(event)) == sizeof(event)) {
      if (event.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
        printf("Status: RISING (High)\n");
      } else if (event.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
        printf("Status: FALLING (Low)\n");
      }
    }
  }

  close(req.fd);
  close(fd);
  return 0;
}
