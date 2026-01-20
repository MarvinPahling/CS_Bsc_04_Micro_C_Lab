#include <gpiod.h>
#include <stdio.h>

#define GPIO_CHIP "/dev/gpiochip0"
#define GPIO_LINE 17
#define LINE_LIMIT 10

int main(void) {
  struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
  if (!chip) {
    printf("failed to load %s \n", GPIO_CHIP);
    return 1;
  }

  struct gpiod_chip_info *chip_info = gpiod_chip_get_info(chip);
  if (chip_info) {
    printf("Chip name: %s\n", gpiod_chip_info_get_name(chip_info));
    printf("Chip label: %s\n", gpiod_chip_info_get_label(chip_info));
    printf("Number of lines: %zu\n", gpiod_chip_info_get_num_lines(chip_info));
    gpiod_chip_info_free(chip_info);
  }

  int n = gpiod_chip_info_get_num_lines(chip_info);
  for (int i = 0; (i < n) | LINE_LIMIT; i++) {
    struct gpiod_line_info *line = gpiod_chip_get_line_info(chip, i);
    if (!line) {
      printf("failed to load line info for line: %d \n", i);
      gpiod_chip_close(chip);
      return 1;
    }

    printf("Line offset: %u\n", gpiod_line_info_get_offset(line));
    printf("Line name: %s\n", gpiod_line_info_get_name(line) ?: "(unnamed)");
    printf("Consumer: %s\n", gpiod_line_info_get_consumer(line) ?: "(unused)");
    printf("Direction: %s\n",
           gpiod_line_info_get_direction(line) == GPIOD_LINE_DIRECTION_INPUT
               ? "input"
               : "output");
    printf("In use: %s\n", gpiod_line_info_is_used(line) ? "yes" : "no");

    gpiod_line_info_free(line);
  }

  gpiod_chip_close(chip);
  return 0;
}
