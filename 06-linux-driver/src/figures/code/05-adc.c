#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>

#define HWMON_BASE "/sys/class/hwmon/hwmon6/temp%d_input"
#define NUM_CORES 5

int read_temperature(const char *path) {
  int fd;
  char buf[32];

  if ((fd = open(path, O_RDONLY)) < 0)
    return -1;

  ssize_t n = read(fd, buf, sizeof(buf) - 1);
  close(fd);

  if (n <= 0)
    return -1;

  buf[n] = '\0';
  return atoi(buf);
}

int main() {
  char path[64];
  int temps[NUM_CORES];
  int sum = 0;

  for (int i = 0; i < NUM_CORES; i++) {
    sprintf(path, HWMON_BASE, i + 1);
    temps[i] = read_temperature(path);

    if (temps[i] < 0) {
      printf("Fehler: temp%d_input\n", i + 1);
      return -1;
    }

    sum += temps[i];
  }

  for (int i = 0; i < NUM_CORES; i++)
    printf("Core %d: %5.1f °C\n", i, temps[i] / 1000.0);

  printf("Avg:    %5.1f °C\n", (sum / NUM_CORES) / 1000.0);

  return 0;
}
