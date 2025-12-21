#ifndef __NXT_MOTORS_H__
#define __NXT_MOTORS_H__

#include <stdint.h>
#include "../main.h"


int nxt_motor_init(uint32_t zyklus);
int nxt_motor_set(motor_t n, int speed_percent,motor_zustand_t zustand);
int nxt_motor_get(motor_t port,uint32_t *pos,int16_t *speed);
#endif
