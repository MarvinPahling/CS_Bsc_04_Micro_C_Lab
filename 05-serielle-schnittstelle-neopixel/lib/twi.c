/*
 * I2C/TWI comminications driver
 * Provide read/write to an I2C/TWI device (in this case the ATMega
 * co-processor). Uses the hardware TWI device in interrupt mode.
 * NOTES
 * This code does not support single byte read/write operation.
 * This code does not support internal register addressing.
 * Runs at high priority interrupt to minimise chance of early
 * write termination (have never seen this but...).
 * For read operations we do not wait for the complete event before 
 * marking the read as over. We do this because the time window for
 * a read when talking to the ATMega is very tight, so finishing
 * slightly early avoids a data over-run. It is a little iffy though!
 */


#include "twi.h"
#include "../AT91SAM7S64.h"

#include "systick.h"

#include "aic.h"
#include "../main.h"


// Calculate required clock divisor
#define   I2CClk          400000L
#define   CLDIV           (((MCK/I2CClk)/2)-3)
// Pins
#define TWCK (1 << 4)   //PA4-PerA
#define TWD  (1 << 3)   //PA3-Pera


static enum {
  TWI_UNINITIALISED = 0,
  TWI_FAILED,
  TWI_IDLE,
  TWI_DONE,
  TWI_RX_BUSY,
  TWI_TX_BUSY,
} twi_state;

static uint32_t twi_pending;
static uint8_t *twi_ptr;
static uint32_t twi_mask;

#define USE_STATS 

// Accumlate stats
#ifdef USE_STATS
static struct {
  uint16_t rx_done;
  uint16_t tx_done;
  uint16_t bytes_tx;
  uint16_t bytes_rx;
  uint16_t unre;
  uint16_t ovre;
  uint16_t nack;
  uint16_t leer;
} twi_stats;
#define STATS(code) code
#else
#define STATS(code)
#endif



/**
 * Return the status of the twi device.
 *  0 == Ready for use
 *  1 == Busy
 * -1 == Error or closed
 */
int twi_status(void)
{
  return (twi_state > TWI_DONE ? 1 : (twi_state < TWI_IDLE ? -1 : 0));
}

/**
 * Process TWI interrupts.
 * Assumes that only valid interrupts will be enabled and that twi_mask
 * will have been set to only contain the valid bits for the current
 * I/O state. This means that we do not have to test this state at
 * interrupt time.
 */
 
#pragma GCC push_options
#pragma GCC optimize ("O2")
 
__attribute__ ((section (".text.fastcode")))
void twi_isr_C(void)
{
	uint32_t status = *AT91C_TWI_SR & twi_mask;
	if (status & AT91C_TWI_RXRDY) {    //A byte has been received in the TWI_RHR 
	                                   //since the last read
		STATS(twi_stats.bytes_rx++);
		*twi_ptr++ = *AT91C_TWI_RHR;
		twi_pending--;
		if (twi_pending == 1) { // Second last byte -- issue a stop on the next byte 
			*AT91C_TWI_CR = AT91C_TWI_STOP;  //Slave über NACK mitteilen, dass
		}                                    //kein weiteres Byte gesendet werden soll
		if (!twi_pending) {     // All bytes have been sent. Mark operation as complete.
			STATS(twi_stats.rx_done++);
			twi_state = TWI_DONE; 
			*AT91C_TWI_IDR = AT91C_TWI_RXRDY;
		}
	}
	else if (status & AT91C_TWI_TXRDY) {   //As soon as data byte is tranfered from TWI_THR
	                                       //to internal shift register of if a NACK error
										   //is detected, TXRDY is set at the same time as
										   //TXCOMP and NACK. TXRDY ist also set, when 
										   //MSEN is set
		if (twi_pending) {      // Still Stuff to send
			*AT91C_TWI_THR = *twi_ptr++;
			twi_pending--;
			STATS(twi_stats.bytes_tx++);
		} else {                // everything has been sent, now wait for complete 
			STATS(twi_stats.tx_done++);
			*AT91C_TWI_IDR = AT91C_TWI_TXRDY;
			*AT91C_TWI_IER = AT91C_TWI_TXCOMP;
			twi_mask = AT91C_TWI_TXCOMP | 
			           AT91C_TWI_NACK;
		}
	}
	else if (status & AT91C_TWI_TXCOMP) {  //when both holding and shift registers are empty 
	                                       //and STOP condition has been sent, or whenn
										   //MSEN ist set (enable TWI)
		twi_state = TWI_DONE;
		*AT91C_TWI_IDR = AT91C_TWI_TXCOMP;
	}

	if(status & AT91C_TWI_NACK) {         //A data bytes has not been acknowledged by the
	                                       //slave component. Set at the same time as
										   //TXCOMP. Reset after read
//twi_nack: __attribute__((unused));
		STATS(twi_stats.nack++);
		*AT91C_TWI_IDR = ~0;
		twi_state = TWI_UNINITIALISED;
	}
	if(!status) {
		STATS(twi_stats.leer++);
//twi_leer: __attribute__((unused));
	}
}
#pragma GCC pop_options


/**
 * Initialize the device.
 */
int twi_init(void)
{
	int i_state;

	i_state = interrupts_get_and_disable();

//  *AT91C_TWI_IDR = ~0;		/* Disable all interrupt sources */
	aic_mask_off  (AT91C_ID_TWI);
//	aic_set_vector(AT91C_ID_TWI, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | AIC_INT_LEVEL_NORMAL,twi_isr_C);
	aic_set_vector(AT91C_ID_TWI, AT91C_AIC_SRCTYPE_POSITIVE_EDGE | AIC_INT_LEVEL_ABOVE_NORMAL,twi_isr_C);
	aic_clear     (AT91C_ID_TWI);
	aic_mask_on   (AT91C_ID_TWI);

	uint32_t clocks = 9;

	*AT91C_TWI_IDR = ~0;

	*AT91C_PMC_PCER = (1 << AT91C_ID_PIOA) |	/* Need PIO too */
		              (1 << AT91C_ID_TWI);   	/* TWI clock domain */

	/* Set up pin as an IO pin for clocking till clean */
	*AT91C_PIOA_MDER = TWD | TWCK;  //OpenCollector
	*AT91C_PIOA_PER  = TWD | TWCK;
	*AT91C_PIOA_ODR  = TWD;         //TWD  als Input konfigurieren
	*AT91C_PIOA_OER  = TWCK;        //TWCK als Output konfigurieren

	//9 Clockimpulse senden, so dass ggf. noch anstehender Schreib-/Leseprozess beendet wird
	while (clocks > 0 && !(*AT91C_PIOA_PDSR & TWD)) {
		*AT91C_PIOA_CODR = TWCK;   //TWCK=0
		systick_wait_ns(1500);     
		*AT91C_PIOA_SODR = TWCK;   //TWCK=1
		systick_wait_ns(1500);
		clocks--;
	}

	/* IO-Pins auf Peripherie umsetzen */
	*AT91C_PIOA_ASR = TWD | TWCK;   //Peripherie A setzen
	*AT91C_PIOA_PDR = TWD | TWCK;   //Peripherie setzen

	/* TWI Initialisieren */
	*AT91C_TWI_CR = AT91C_TWI_SWRST |  /* TWI Reset */
	                AT91C_TWI_MSDIS;   /* Master Transfer disable */

	*AT91C_TWI_CWGR =  (0     << 16) |  //CKDIV (Clock Divider)
	                   (CLDIV <<  8) |  //CHDIV (SCL Low Period)
	                   (CLDIV <<  0);   //CLDIV (SCL High Period)
						
	*AT91C_TWI_CR  = AT91C_TWI_MSEN;	/* Master Transfer enable */
	*AT91C_TWI_IER = AT91C_TWI_NACK;    /* Enable NACK ISR */
	twi_mask = 0;

	twi_state = TWI_IDLE;

	if(i_state)
		interrupts_enable();

  return 1;
}


/**
 * Start a read operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_read(uint32_t dev_addr, uint8_t *data, uint32_t nBytes)
{
	if (twi_state < TWI_RX_BUSY) {
		twi_state = TWI_RX_BUSY;
		twi_ptr = data;
		twi_pending = nBytes;
		
		*AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO |  //No internal device address
		                 AT91C_TWI_MREAD     |  //Master Read Direction
						 ((dev_addr & 0x7f) << 16);
		twi_mask = AT91C_TWI_RXRDY |            //Receive holding register ReaDY
		           AT91C_TWI_NACK;              //Not Acknowledged
		//In a single data byter master read, the START und STOP must both be set
		if(twi_pending==1)
			*AT91C_TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
		else
			*AT91C_TWI_CR = AT91C_TWI_START;       //Start Read Sequence
		*AT91C_TWI_IER = AT91C_TWI_RXRDY;
	}
}

/**
 * Start a write operation to the device. The operation will complete
 * asynchronously and can be monitored using twi_status. Note that we
 * do not support single byte reads, or internal register addresses.
 */
void twi_start_write(uint32_t dev_addr, const uint8_t *data, uint32_t nBytes)
{
	if (twi_state < TWI_RX_BUSY) {
		twi_state = TWI_TX_BUSY;
		twi_ptr = (uint8_t *)data;
		twi_pending = nBytes;

		*AT91C_TWI_MMR = AT91C_TWI_IADRSZ_NO |   //No internal device address
		                 ((dev_addr & 0x7f) << 16);
		*AT91C_TWI_THR = *twi_ptr++;             //Start Write Sequence
		twi_pending--;
		STATS(twi_stats.bytes_tx++);
		
			
		twi_mask = AT91C_TWI_TXRDY |             //Transmit holding register ReaDY
		           AT91C_TWI_NACK;               //Not Acknowledged
		*AT91C_TWI_IER = AT91C_TWI_TXRDY;
	}
}
