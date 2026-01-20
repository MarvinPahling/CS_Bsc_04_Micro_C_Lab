#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#define PWM_PATH "/sys/class/hwmon/hwmon4/pwm1"

int main(void) {
  int fd = open(PWM_PATH, O_WRONLY);
  if (fd < 0) {
    perror(PWM_PATH);
    return 1;
  }
  write(fd, "255", 3); // 100% fan speed
  close(fd);
  printf("Fan set to 100%%\n");
  return 0;
}
