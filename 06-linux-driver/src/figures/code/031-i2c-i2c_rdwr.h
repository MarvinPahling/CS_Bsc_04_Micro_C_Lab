#include <linux/types.h>

#define I2C_RDWR 0x0707 /* Combined R/W transfer (one STOP only) */

/* This is the structure as used in the I2C_RDWR ioctl call */
struct i2c_rdwr_ioctl_data {
  struct i2c_msg *msgs; /* pointers to i2c_msgs */
  __u32 nmsgs;          /* number of i2c_msgs */
};
