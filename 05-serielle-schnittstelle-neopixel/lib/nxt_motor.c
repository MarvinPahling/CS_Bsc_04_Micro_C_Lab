#include <stddef.h>
#include "../AT91SAM7S64.h"     //fuer GPIO / Timer 
#include "../main.h"            //fuer MCK
#include "nxt_motor.h"
#include "pio_pwm.h"
#include "nxt_avr.h"            //fuer nxt_avr_set_motor()

/**********************************************************************************
 *                        Linksrum <-  |  -> Rechtsrum
 *  A(Mx0)         ----+     +------+     +------+     +------+     +---
 *                     |     |      |     |      |     |      |     |
 *                     +-----+      +-----+      +-----+      +-----+   
 *  B(Mx1)          ------+     +-------------+     +------+     +------
 *                        |     |             |     |      |     |
 *                        +-----+             +-----+      +-----+
 *                        |  |   |  |
 *                        |  +-- | -+--> Timer LDRA+Overrun-ISR
 *                        +------+-----> GPIO-ISR
 *                                    
 * pinChanges A     00010000010000001000001000000100000100000010000010000
 * pinChanges B     00000010000010000000000000100000100000010000010000000
 * currentPins A    11100000011111110000001111111000000111111100000011111
 * currentPins B    11111100000011111111111111000000111111100000011111111
 * 
 * Position über Steigende/Fallende Flanke von A(Timer LDRA-ISR) und GPIO-ISR
 * Geschwindigkeit über Periodendauer von A (Pausen- und Pulsdauer)
 *    Mittelwertbildung immer nur bis zur über nxt_motor_init() angegebenen
 *    Zyklusdauer
 **********************************************************************************/

#define MA0 15     //PerA=TF    PerB=TioA1   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer1
#define MA1 1      //PerA=PWM1  PerB=TioB0   <-- Pos=Gpio-IRQ
#define MB0 26     //PerA=DCD1  PerB=TioA2   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer2
#define MB1 9      //PerA=DRxD  PerB=NPCs1   <-- Pos=Gpio-IRQ
#define MC0 0      //PerA=PWM0  PerB=TioA0   <-- Pos=Timer-IRQ  Geschwindigkeit=Timer0
#define MC1 8      //PerA=CTS0  PerB=ADTrg   <-- Pos=Gpio-IRQ

struct {
	struct motor {
		uint32_t   pos;
		 int16_t   speed;
	} motor[3];
	uint64_t timer_zyklus;
} motor;



int nxt_motor_init(uint32_t zyklus)
{
	//PWM-Signale laufen über den AVR-Prozessor
	//Initialisierung nicht notwendig, da dieser zuvor 
	//bereits ininitialisiert wurde
	//nxt_avr_init()
    nxt_avr_set_motor(MOTOR_A,0,MOTOR_BREAK);
	nxt_avr_set_motor(MOTOR_B,0,MOTOR_BREAK);
	nxt_avr_set_motor(MOTOR_C,0,MOTOR_BREAK);
	
	return 0;
}

int nxt_motor_get(motor_t port,uint32_t *pos,int16_t *speed)
{
	if(port > MOTOR_C)
		return -1;

	struct motor *mot=&motor.motor[port];

	if(pos != NULL)
		*pos = mot->pos;
	
	if(speed != NULL)
		*speed = mot->speed;

	return 0;
}

int nxt_motor_set(motor_t port, int speed_percent, motor_zustand_t zustand)
{
	if((port <= MOTOR_C) && (zustand <= MOTOR_FLOAT)) {
		if (speed_percent > 100)
			speed_percent = 100;
		if (speed_percent < -100)
			speed_percent = -100;
		nxt_avr_set_motor(port, speed_percent, zustand);
		return 0;
	}
	return -1;
}
