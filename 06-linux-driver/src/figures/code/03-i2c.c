#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>

/* Macros from your original code */
#define I2C_DEVICE "/dev/i2c-6"
#define I2C_ADDRESS 0x50
#define REGISTER 0x00
#define BYTES_TO_READ 8
#define N_MSG 2
#define S_WRITE_BUFFER BYTES_TO_READ / sizeof(char)

int main() {
  int file;
  unsigned char write_buf[S_WRITE_BUFFER] = {REGISTER};
  unsigned char read_buf[BYTES_TO_READ];

  // Open the I2C device
  if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
    perror("Error opening I2C device");
    return 1;
  }

  // Create write and read message
  struct i2c_msg messages[N_MSG] = {{.addr = I2C_ADDRESS,
                                     .flags = 0,
                                     .len = sizeof(write_buf),
                                     .buf = write_buf},
                                    {.addr = I2C_ADDRESS,
                                     .flags = I2C_M_RD,
                                     .len = sizeof(read_buf),
                                     .buf = read_buf}};

  struct i2c_rdwr_ioctl_data packets = {.msgs = messages, .nmsgs = N_MSG};

  if (ioctl(file, I2C_RDWR, &packets) < 0) {
    perror("Error sending I2C transaction");
    close(file);
    return 1;
  }

  printf("Gelesen von Gerät 0x%02x (Register 0x%02x):\n", I2C_ADDRESS,
         REGISTER);
  for (int i = 0; i < BYTES_TO_READ; i++) {
    printf("0x%02x ", read_buf[i]);
  }
  printf("\n");

  close(file);
  return 0;
}
