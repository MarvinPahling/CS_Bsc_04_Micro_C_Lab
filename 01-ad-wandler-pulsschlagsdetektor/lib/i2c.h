#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>
#include "../main.h"

#ifndef I2C_BAUDRATE
#define I2C_BAUDRATE 9600
#endif

typedef void(*i2c_cb)(void *arg);

void i2c_init(void);

void i2c_disable   (sensor_t port);
void i2c_enable    (sensor_t port);
int  i2c_busy      (sensor_t port);
int  i2c_wait_ready(sensor_t port, int32_t wait_ms);
int i2c_start_transaction(sensor_t port, 
                          uint8_t  address, 
                          uint8_t  ninternal_address, 
                          uint8_t  internal_address[ninternal_address], 
                          uint16_t nbytes,
                          uint8_t  data[nbytes], 
                          int      write,  // 0->read / 1->write
					      i2c_cb   cb,     // CallBack Function
					      void    *arg);   // CallBack Argument

void i2c_test(void);

#endif
