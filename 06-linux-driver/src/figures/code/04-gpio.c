#include <linux/gpio.h>

#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#define GPIO_DEVICE "/dev/gpiochip0"
#define GPIO_LINE 7

int main() {
  int fd;
  struct gpiochip_info chip_info;
  struct gpio_v2_line_request req;
  struct gpio_v2_line_values values;

  if ((fd = open(GPIO_DEVICE, O_RDWR)) < 0) {
    perror("Fehler beim Öffnen des GPIO-Chips");
    return 1;
  }

  if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &chip_info) < 0) {
    perror("Fehler beim Abrufen der Chip-Informationen");
    close(fd);
    return 1;
  }

  printf("GPIO Chip: %s\n", chip_info.name);
  printf("Label: %s\n", chip_info.label);
  printf("Anzahl Lines: %u\n", chip_info.lines);

  memset(&req, 0, sizeof(req));
  req.offsets[0] = GPIO_LINE;
  req.num_lines = 1;
  req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
  strncpy(req.consumer, "gpio-example", sizeof(req.consumer) - 1);

  if (ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
    perror("Fehler beim Anfordern der GPIO-Line");
    close(fd);
    return 1;
  }

  memset(&values, 0, sizeof(values));
  values.mask = 1;

  if (ioctl(req.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
    perror("Fehler beim Lesen des GPIO-Werts");
    close(req.fd);
    close(fd);
    return 1;
  }

  printf("GPIO Line %d Wert: %s\n", GPIO_LINE,
         (values.bits & 1) ? "HIGH" : "LOW");

  close(req.fd);
  close(fd);
  return 0;
}
