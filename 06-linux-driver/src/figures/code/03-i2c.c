#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>

#define I2C_DEVICE "/dev/i2c-6"
#define I2C_ADDRESS 0x50
#define REGISTER 0x00
#define BYTES_TO_READ 8

int main() {
  int fd;
  unsigned char write_buf[1] = {REGISTER};
  unsigned char read_buf[BYTES_TO_READ];

  if ((fd = open(I2C_DEVICE, O_RDWR)) < 0)
    return -1;

  struct i2c_msg messages[2] = {{.addr = I2C_ADDRESS,
                                 .flags = 0, /* Write Flag */
                                 .len = sizeof(write_buf),
                                 .buf = write_buf},
                                {.addr = I2C_ADDRESS,
                                 .flags = I2C_M_RD, /* Read Flag */
                                 .len = sizeof(read_buf),
                                 .buf = read_buf}};

  struct i2c_rdwr_ioctl_data packets = {.msgs = messages, .nmsgs = 2};

  if (ioctl(fd, I2C_RDWR, &packets) < 0) {
    close(fd);
    return -1;
  }

  printf("Gelesen von Gerät 0x%02x (Register 0x%02x):\n", I2C_ADDRESS,
         REGISTER);
  for (int i = 0; i < BYTES_TO_READ; i++) {
    printf("0x%02x ", read_buf[i]);
  }

  close(fd);
  return 0;
}
