#include <fcntl.h>
#include <unistd.h>

#define I2C_PATH "/dev/i2c-6"

int main(void) {
  int fd;
  /* Open FILE and return a new file descriptor for it, or -1 on error.
     OFLAG determines the type of access used.  If O_CREAT or O_TMPFILE is set
     in OFLAG, the third argument is taken as a `mode_t', the mode of the
     created file.

     This function is a cancellation point and therefore not marked with
     __THROW.  */
  if (open(I2C_PATH, O_RDWR) < 0)
    ; /* error handling */
}
