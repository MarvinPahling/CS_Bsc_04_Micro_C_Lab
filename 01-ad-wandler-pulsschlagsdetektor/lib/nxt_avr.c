#include <stdio.h>
#include <string.h>

#include "nxt_avr.h"
#include "twi.h"
#include "aic.h"			//fuer aic_sys_register_pit() aic_sys_vl_t
#include "../AT91SAM7S64.h"

#define NXT_AVR_ADDRESS 1

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG==1
#define STATS(code) code
#else
#define STATS(code)
#endif

#define NXT_AVR_N_OUTPUTS 4
#define NXT_AVR_N_INPUTS  4

typedef struct{
	uint8_t power;                   /**< Command byte that is used during power down and firmware update mode
	                                      During normal communication, this byte sould be set to zero */
	uint8_t pwm_frequency;           /**< Holds the PWM frequency used by the PWM signal for the three outputs
	                                      Range: 1-32kHz. Units are in kHz. Standard LEGO firmware uses 8kHz  */
	 int8_t output_percent[NXT_AVR_N_OUTPUTS]; /**< Holds the power level for the individual output.
	                                                Range: -100 to +100 */
	uint8_t output_mode;             /**< Holds the output mode that can be either float or break 
                                          PWM pulses. 0x00 means break, and 0x01 means float	*/
	uint8_t input_power;             /**< No Supply / Pulsed 9V / Constant 9V */
	//uint8_t csum;
} __attribute__((packed)) to_avr_t;  // (8+1)Bytes

typedef struct {
	uint16_t adc_value[NXT_AVR_N_INPUTS];  /**< Holds the raw value from the 10 bit A/D converter */
	uint16_t buttonsVal;                   /**< Holds the status of the buttons. Button 1,2,3 are returned
	                                            as a 10bit AD value. Button 0 adds 0x7ff to this. */
	uint16_t extra;                        /**< Holds information about the measured battery level, whether an
                                                Accu pack has been inserted and the AVR firmware version
												Bit 15  0->AA batteries  1->Accu pack inserted
												Bit 13-14->Major Version
												Bit 10-12->Minor Version
												Bit  0-9 ->battery raw value (multiply with 0,013848 to get V */
	//uint8_t csum;
} __attribute__((packed)) from_avr_t;  // (12+1)Bytes


// This string is used to establish communictions with the AVR
const char avr_brainwash_string[] =
  "\xCC" "Let's samba nxt arm in arm, (c)LEGO System A/S";

/* static */ struct {
	aic_sys_vl_t aic_sys_vl;

	enum {STATE_INIT,
		  STATE_WAIT1,STATE_WAIT2,//STATE_WAIT3,STATE_WAIT4,
		  STATE_SEND,STATE_SEND_RETRY,
		  STATE_RECV,STATE_RECV_RETRY } __attribute__((packed)) state;
	uint8_t pwm_frequency;
	button_t buttons;

	// Output data is double buffered via the following (note extra space for csum)
	uint8_t tofrom_avr_buf[(sizeof(to_avr_t)>sizeof(from_avr_t)?sizeof(to_avr_t):sizeof(from_avr_t))+1];

	to_avr_t   to_avr;
	from_avr_t from_avr;

#if ENABLE_DEBUG==1
	uint16_t good_rx;
	uint16_t bad_rx;
	uint16_t resets;
	uint16_t still_busy;
#endif
} avr;

static void buttonsVal2buttons(void);


static void avr_1kHz_update(void)
{
	uint8_t checkByte;
	uint8_t *dst;
	uint8_t *src;
	
	switch(avr.state) {
	case STATE_INIT:
		/**
		* Initialise the link with the AVR by sending the handshake string. 
		* The transmission time ist because of the length of the string 
		* 47Bytes*9/400kHz=1,05ms, so we need to slots!
		*/
		twi_start_write(NXT_AVR_ADDRESS, (const void *) avr_brainwash_string, strlen(avr_brainwash_string));
		//twi_init() Set to Default
		memset(&avr.from_avr,0,sizeof(avr.from_avr));
		memset(&avr.to_avr,0,sizeof(avr.to_avr));
		avr.to_avr.pwm_frequency  = avr.pwm_frequency;
		
		//Prepare avr.tofrom_avr_buf, so the first check creates no error
		memcpy(avr.tofrom_avr_buf,&avr.from_avr,sizeof(avr.from_avr));
		checkByte = 0;
		src       = avr.tofrom_avr_buf;
		while (src < (avr.tofrom_avr_buf + sizeof(avr.from_avr))) {
			checkByte += *src++;
		}
		avr.tofrom_avr_buf[sizeof(avr.from_avr)]=~checkByte;
		
		avr.state++;
		break;
	case STATE_WAIT1:
	case STATE_WAIT2:
//	case STATE_WAIT3:
//	case STATE_WAIT4:
		avr.state++;
		break;
	case STATE_SEND:
	case STATE_SEND_RETRY:
		switch(twi_status()) {
		case 0: {
			//Check and unpack received data
			checkByte = 0;
			src       = avr.tofrom_avr_buf;
			while (src < (avr.tofrom_avr_buf + sizeof(avr.from_avr) + 1)) {
				checkByte += *src++;
			}
			// Wenn der AVR beim Empfangen eine falsche Prüfsumme empfängt, 
			// generiert dieser beim Senden seinerseits eine falsche Prüfsumme, 
			// so dass die Überprüfung hier nie korrekt ist.
			if(checkByte == 0xff) {
				STATS(avr.good_rx++);
				src = avr.tofrom_avr_buf;
				dst = (uint8_t *)&avr.from_avr;
				while (src < (avr.tofrom_avr_buf + sizeof(avr.from_avr))) {
					*dst++ = *src++;		
				}
			}	
			else {
				STATS(avr.bad_rx++);
			}
			
			//Pack and send data
			checkByte = 0;
			src = (uint8_t *)&avr.to_avr;
			dst = avr.tofrom_avr_buf;
			while (src < ((uint8_t *)&avr.to_avr + sizeof(avr.to_avr))) {
				checkByte += *src;
				*dst++ = *src++;
			}
			*dst = ~checkByte;
			twi_start_write(NXT_AVR_ADDRESS, (void *)avr.tofrom_avr_buf,sizeof(to_avr_t)+1);
			avr.state=STATE_RECV;
		break; 
		}
		case 1:  //Busy
			if(avr.state != STATE_SEND_RETRY) {
				avr.state++;
				STATS(avr.still_busy++);
				break;
			}
			__attribute__((fallthrough));
		default:  //Error
			avr.state=STATE_INIT;
			STATS(avr.resets++);
			break;
		}
		break;
	case STATE_RECV:
	case STATE_RECV_RETRY:
		switch(twi_status()) {
		case 0:
			memset(avr.tofrom_avr_buf, 0, sizeof(avr.tofrom_avr_buf));
			twi_start_read(NXT_AVR_ADDRESS, (void *)avr.tofrom_avr_buf, sizeof(from_avr_t)+1);
			buttonsVal2buttons();
			avr.state=STATE_SEND;
			break;
		case 1:  //Busy
			if(avr.state != STATE_RECV_RETRY) {
				avr.state++;
				STATS(avr.still_busy++);
				break;
			}
			__attribute__((fallthrough));
		default:  //Error
			avr.state=STATE_INIT;
			STATS(avr.resets++);
			break;
		}
		break;
	}
}


//            +----+     +----+     +----+
//     5V -+--|4,7k|--+--|2,2k|--+--|1,0k|---- GND
//         |  +----+  |  +----+  |  +----+
//         / SW1      / SW2      / SW4
//         |          |          |  +----+
//         +----------+----------+--|100k|---- GND
//                  BUTT_ADC        +----+
//                  BUTTON0  --------/-------- GND
//   SW1  SW2  SW3 SW4  BUTT_ADC DIG  DIG/16
//    -    -        -    0,00V     0     0
//    1    -        -    5,00V  1023    63 
//    -    1        -    1,98V   405    25
//    1    1        -    5,00V  1023    63
//    -    -        1    0,627   128     8
//    1    -        1    5,00V  1023    63
//    -    1        1    0,87V   178    11
//    1    1        1    5,00V  1023    63
//             -                   0
//             1                2047   127    
// 
//  SW1 -> UNTEN
//  SW2 -> RECHTS
//  SW3 -> MITTE (Orange) SW3/Button0 adds 0x7ff to 10Bit AD-Value
//  SW4 -> LINKS
//
static void buttonsVal2buttons(void)
{
	static uint8_t old[4];
	static uint8_t index=0;

	old[index]=(avr.from_avr.buttonsVal >> 4) & 0xff;
	index=(index+1)&3;
	
	if((old[0] == old[1]) && (old[2]==old[3]) && (old[0]==old[3])) {
		button_t new = {};
		if(old[0] >= 127) {
			new.orange = 1;
			old[0]-=127;
		}
		if((old[0] & 0x7f) >= ((25+63)/2)) {
			new.grey = 1;
		}
		else if((old[0] & 0x7f) >= ((63+25)/2)) {
			new.grey = 1;
		}
		else if((old[0] & 0x7f) >= ((25+11)/2)) {
			new.right = 1;
		}
		else if((old[0] & 0x7f) >= ((11+8)/2)) {
			new.right = 1;
			new.left  = 1;
		}
		else if((old[0] & 0x7f) >= ((8+0)/2)) {
			new.left = 1;
		}
		avr.buttons=new;
	}
}

button_t nxt_avr_get_buttons(void)
{
	return avr.buttons;
}


#if 0
static void buttons_calculate(void)
{
#define BUTTON_DEBOUNCE_CNT 50/2;
// 50ms Debounce time. Button read is called every other 1000Hz tick
         uint16_t buttonsVal;
         uint16_t newState;
  static uint16_t prev_buttons=0;
  static uint16_t button_state=0;
  static uint16_t debounce_state=0;
  static uint16_t debounce_cnt== BUTTON_DEBOUNCE_CNT;
  
  //Display-Tasten entprellen
  buttonsVal = avr.from_avr.buttonsVal;
  if (buttonsVal > 60 || button_state)
  {
    // Process the buttons. First we drop any noisy inputs
    if (buttonsVal != prev_buttons)
      prev_buttons = buttonsVal;
    else
    {
      // Work out which buttons are down. We allow chording of the enter
      // button with other buttons
      newState = 0;
      if (buttonsVal > 1500) {
        newState |= 1;
        buttonsVal -= 0x7ff;
      }
  
      if (buttonsVal > 720)
        newState |= 0x08;
      else if (buttonsVal > 270)
        newState |= 0x04;
      else if (buttonsVal > 60)
        newState |= 0x02;
      // Debounce things...
      if (newState != debounce_state)
      {
        debounce_cnt = BUTTON_DEBOUNCE_CNT;
        debounce_state = newState;
      }
      else if (debounce_cnt > 0)
        debounce_cnt--;
      else
        // Got a good key, make a note of it
        button_state = debounce_state;
    }
  }
}
#endif

uint16_t nxt_avr_get_battery_mv(void)
{
	//Bit  0-9 ->battery raw value (multiply with 0,013848 to get V) */
	// Uq   = 5V/1023
	// Ulsb = Uq * (220k+120) / 120k = 13,848mv
	// Uadc = 9v * 120k / (220k+120k)
	// DIG  = UBAT / Ulsb
	// 0,013848 wird angenähert durch 14180/1024 / 1000
	return ((uint32_t)(avr.from_avr.extra & 0x3ff) * 14180) >> 10;
}

battery_t nxt_avr_get_battery_type(void)
{
	//Bit 15  0->AA batteries  1->Accu pack inserted
	return (avr.from_avr.extra & 0x8000) ? BATTERY_ACCU : BATTERY_AA;
}

uint8_t nxt_avr_get_avr_version(void)
{
	//Bit 13-14->Major Version
	//Bit 10-12->Minor Version
	return (avr.from_avr.extra & 0x7300)>>10; 
}

uint16_t nxt_avr_get_battery_raw(void)
{
	//Bit  0-9 ->battery raw value (multiply with 0,013848 to get V */
	return avr.from_avr.extra & 0x3ff;
}

uint16_t nxt_avr_get_sensor_adc_raw(sensor_t sens)
{
	if ((sens <= SENSOR_4) && (((avr.to_avr.input_power>>sens)&0x11) != SENSOR_9V) )
		return avr.from_avr.adc_value[sens] & 0x3ff;
	else
		return UINT16_MAX;
}

void nxt_avr_set_sensor_power(sensor_t sens, sensor_power_t sensor_power)
{
	//input_power   s3 s2 s1 s0 | s3 s2 s1 s0
	//              -----------   -----------
	//                 |             +-> 9V Pulsed (sensor could read)
	//                 +--> 9V Constantly on
	// - Both clear -> 9V is not supplied
	// - Both on    -> Invalid / Is not supported
	if ((sens <= SENSOR_4) && (sensor_power <= SENSOR_9V)) {
		sensor_power &= 0x11;
		avr.to_avr.input_power &= ~(0x11 << sens);
		avr.to_avr.input_power |= sensor_power << sens;
	}
}

void nxt_avr_power_down(void)
{
	avr.to_avr.power = 0x5a;
	avr.to_avr.pwm_frequency = 0x00;
}

void nxt_avr_firmware_update_enter(void)
{
	avr.to_avr.power = 0xA5;
	avr.to_avr.pwm_frequency = 0x5A;
}

void nxt_avr_set_motor(motor_t motor, int power_percent, motor_zustand_t brake)
{
	if (motor <= MOTOR_C) {
		//output_percent  ist int8_t  hat also nur Wertbereich von -128..+127
		//Werte größer  +100..+127 werden vom AVR auf 100 und 
		//      kleiner -100..-128 werden vom AVR auf -100 begrenzt
		//Da power_percent integer ist, ist hier ein Begrenzung notwendig
		if(power_percent > +100) 
			power_percent=+100;
		if(power_percent < -100) 
			power_percent=-100;
		avr.to_avr.output_percent[motor] = power_percent;
		
		if (brake == MOTOR_BREAK)
			avr.to_avr.output_mode |=  (1 << motor);
		else
			avr.to_avr.output_mode &= ~(1 << motor);
	}
}

void nxt_avr_init(uint8_t pwm_frequency_kHz)
{
	avr.state=STATE_INIT;
	avr.buttons = (button_t){};
	avr.pwm_frequency = pwm_frequency_kHz;
#if ENABLE_DEBUG==1
	avr.good_rx    = 0;
	avr.bad_rx     = 0;
	avr.resets     = 0;
	avr.still_busy = 0;
#endif
	twi_init();
	
	/* Callback Routine einhängen */
	aic_sys_register_pit(&avr.aic_sys_vl,avr_1kHz_update);
}

