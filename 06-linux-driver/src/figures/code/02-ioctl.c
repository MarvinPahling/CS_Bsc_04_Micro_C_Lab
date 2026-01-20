#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

struct i2c_config {
  int speed;
  int mode;
};

#define I2C_PATH "/dev/i2c-6"
#define COMMAND 1
#define PAYLOAD &((struct i2c_config){.speed = 400, .mode = 1})

int main(void) {
  int fd;
  if (open(I2C_PATH, O_RDWR) < 0)
    ; /* error handling */

  /* Perform the I/O control operation specified by REQUEST on FD.
     One argument may follow; its presence and type depend on REQUEST.
     Return value depends on REQUEST.  Usually -1 indicates error.  */
  if (ioctl(fd, COMMAND, PAYLOAD) < 0)
    ; /* error handling */
}
