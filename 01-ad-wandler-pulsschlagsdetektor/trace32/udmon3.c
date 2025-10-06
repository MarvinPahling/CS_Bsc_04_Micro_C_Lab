/**************************************************************************
Bereitstellung des UDMON3 Treibers für
 - Im RAM-Modus
      Zugriff auf Speicher über '%e' Befehle
	  Zugriff auf 'Virtuellen' DisplaySpeicher 
	     (Interne Bitmap wird auf GrayScale Bitmap umgemap)
      Zugriff auf MMU/Cache (jedoch in AT91SAM7 nicht vorhanden)
      Terminal Datenaustausch
 - Im Simulations-Modus
      Terminal Datenaustausch

 Aus debugger_arm.pdf
 If SYStem.MemAccess ist not denied, it is possible to read from memory, 
 to wirte to memory and to set software breakpoints while the CPU
 is executing the programm. This requeires one of the follow monitors
 - CERBERUS (Infineon)
 - CPU (Instruction Set simulator)
 - DAP (Debug Access Port)
 - NEXUS
 - TSMON3: A run-time memory access is done via a Time Sharing Monitor
   The application is responsible for calling the monitor code periodically.
   The call ist typically include in a periodic interrupt or int the idle 
   task of the kernel. (runtime_memory_access)
   Besides runtime memory access TSMON3 would allow run mode debugging.
   But manual break is not possible with TSMON 3 and could only be emulated
   by polling the DCC port. 
 - PTMON3: A run-time memory access is done via a PULSE Triggered Monitor
   Whenever the debugger wants to perform a memory access while the program
   is running, the debugger generates a trigger for the trigger bus. If the
   trigger bus in configured appropriate (TrBus), this trigger is output
   via the TRIGGER connector of the TRACE32 development tool. The TRIGGER
   output can be connected to an external interrupt in order to call a 
   monitor.  (runtime_memory_access)
   Besides runtime memory acces PTMON3 would allow run mode debugging.
   But manual break is not possible with PTMON3 and could only be emulated
   by polling the DCC port.
 - UDMON3: A run-time memory access is done via a Usermode Debug Monitor
   The application is responsible for calling the montiro code periodically.
   The call is typically include in a periodic interrupt or in the idle task
   of the kernel. For runtime memory access UDMON3 behaves exactly as TSMON3. 
   (runtime_memory_access)
   Besides runtime memory access UDMON3 allows run mode debugging.
   Handling of interrupts when the application is stopped is possible when
   the background monitor is activated. On-chip breakpoints and manual 
   program break are only possible when the application runs in user (USR)
   mode. (background_monitor)
**************************************************************************/
#include <stddef.h>          //fuer NULL
#include "udmon3.h"
#include "../lib/aic.h"  //fuer aic_sys_register_commxx() 
                         //     aic_sys_register_pit() aic_sys_vl_t

#pragma GCC push_options
#pragma GCC optimize ("O2")

#define UDMON3_ZYKLISCH 0
#define UDMON3_IRQ      1
#define UDMON3_MODE     UDMON3_ZYKLISCH
//#define UDMON3_MODE     UDMON3_IRQ
//Datenübertragung per IRQ funktioniert gut. Dann sollte jedoch kein d.image genutzt werden
//da trace32 zur Darstellung einer hohen Bildwiederholrate annäherend kontinuierlich
//den Bildschirmspeicher ausliest! Damit bleibt der Hauptanwendung nur noch wenig
//Rechenzeit übrig!
  
#ifdef MODE_RAM
  //Fuer den Zugriff des Debuggers auf das Display wird der der Display-Speicher
  //in einen 'virtuellen' Speicherbereich ab LCD_MEMORY_OFFSET abgebildet
  #include "../lib/nxt_lcd.h"  //fuer NXT_LCD_WIDTH  NXT_LCD_DEPTH
  #include "../lib/display.h"  //fuer display_get_buffer()

  //aus sim_NXT/sim_NXT.h
  #define LCD_MEMORY_OFFSET  0x10000000  //in diesem Bereich 
                                         //- tätigt der Simulator eine Kopie des Bildschirmspeichers
										 //- muss im RAM-Modus der Bildschirmspeicher abgebildet werden
  #define NXT_MEMORY_OFFSET  0x20000000  //in diesen Bereich schreibt der Simulator die NXT
                                         //spezfischen Daten (siehe auch link.ld)

  //aus sim_NXT/nxt.c
  #define NXT_WIDTH 150   //Gesamtgröße NXT-Grafik
  #define NXT_HIGH  250   //Gesamtgröße NXT-Grafik
  
  #define LCD_WIDTH 100   //Größe LCD-Display
  #define LCD_HIGH   64   //Größe LCD-Display
  #define LCD_X      25   //Offset für LCD-Display
  #define LCD_Y      50   //Offset für LCD-Display


  //Zur Sicherheit Daten aus sim_NXT mit Daten aus nxt_lcd vergleichen
  #if (NXT_LCD_DEPTH*8 != LCD_HIGH)
    #error "LCD_HIGH != NXT_LCD_DEPTH"
  #endif
  #if (NXT_LCD_WIDTH != LCD_WIDTH)
    #error "LCD_WIDTH != NXT_LCD_WIDTH"
  #endif
#endif


#include "../AT91SAM7S64.h"

#if UDMON3_MODE == UDMON3_IRQ
#define COMMTX_IRQ_ENABLE()    *AT91C_DBGU_IER = AT91C_US_COMM_TX
#define COMMTX_IRQ_DISABLE()   *AT91C_DBGU_IDR = AT91C_US_COMM_TX
#define COMMRX_IRQ_ENABLE()    *AT91C_DBGU_IER = AT91C_US_COMM_RX
#endif
#if UDMON3_MODE == UDMON3_ZYKLISCH
#define COMMTX_IRQ_ENABLE()
#define COMMTX_IRQ_DISABLE()
#define COMMRX_IRQ_ENABLE()
#endif


/**************************************************************************
**************************************************************************/

struct {
	byte_fifo_cb_t *term_send;
	byte_fifo_cb_t *term_recv;
	
#if UDMON3_MODE == UDMON3_ZYKLISCH
    aic_sys_vl_t   aic_sys_vl;
#endif	
#ifdef MODE_RAM
    unsigned char *display_buffer;
#endif

#ifdef MODE_RAM
	unsigned int monitor_address_high;
	unsigned int monitor_address_low;
	unsigned int monitor_buffer[16];
	         int monitor_index;
			 int monitor_count;       //Anzahl zu sendender Zeichen
#endif
#ifdef PTMON3
    #define MONITOR_STACKSIZE 0x40
	unsigned int  monitor_stacksize;
	unsigned int  monitor_registers_and_stack[MONITOR_STACKSIZE+40];
	unsigned int *monitor_stackbase;
#endif	
} udmon3 = {
	.term_send  = NULL,
	.term_recv  = NULL,
#if UDMON3_MODE == UDMON3_ZYKLISCH
	.aic_sys_vl = {.next=NULL,.fcn=NULL},
#endif	
#ifdef MODE_RAM
	.display_buffer=NULL,
#endif
#ifdef PTMON3
	.monitor_stacksize = MONITOR_STACKSIZE;
	.monitor_stackbase = udmon3.monitor_registers_and_stack+MONITOR_STACKSIZE;
#endif	
};


void udmon3_handler(void);

void udmon3_init(void) 
{
#if UDMON3_MODE == UDMON3_ZYKLISCH
  /* Callback Routine einhängen */
  aic_sys_register_pit(&udmon3.aic_sys_vl,udmon3_handler);
#endif  
#if UDMON3_MODE == UDMON3_IRQ
	aic_sys_register_commxx(udmon3_handler);
	COMMRX_IRQ_ENABLE();
#endif
}


#if UDMON3_MODE == UDMON3_IRQ
static void enable_commtx(void)
{
	COMMTX_IRQ_ENABLE();
}
#endif

void udmon3_term_init(byte_fifo_cb_t *recv,byte_fifo_cb_t *send,byte_fifo_cb *tx_cb)
{
	udmon3.term_send=send;
	udmon3.term_recv=recv;
#if UDMON3_MODE == UDMON3_IRQ
	*tx_cb = enable_commtx;
#endif
#if UDMON3_MODE == UDMON3_ZYKLISCH
	*tx_cb = NULL;
#endif
}

/**************************************************************************

  CP15 access  / MMU + Cache

  Does not support all CP15 register. Can be extended appropriate to your need.

  ARM9 requires the CP15 accesses to be done in a privileged mode (not in
  user mode). In this example the Monitor_Handler will be called from the
  monitor (privileged mode), but also from the application (user mode).
  Therefore do not access CP15 register while the application is running
  (e.g. by 'Data.In EC15:0x0001 /Long').

**************************************************************************/
#if 0 
static unsigned int Monitor_ReadCP15 (unsigned int address)
{
  unsigned int data = 0;

  switch (address)
  {
    case 0xf000:
		asm volatile("MRC p15, 0, %0  , c0, c0" : "=r" (data));
		break;
    case 0xf001:
		asm volatile("MRC p15, 0, %0  , c1, c0" : "=r" (data));
		break;
    case 0xf002:
		asm volatile("MRC p15, 0, %0  , c2, c0" : "=r" (data));
		break;
    case 0xf003:
		asm volatile("MRC p15, 0, %0  , c3, c0" : "=r" (data));
		break;
    case 0xf004:
		asm volatile("MRC p15, 0, %0  , c4, c0" : "=r" (data));
		break;
    case 0xf005:
		asm volatile("MRC p15, 0, %0  , c5, c0" : "=r" (data));
		break;
  }
  return data;
}

static void Monitor_WriteCP15 (unsigned int address, unsigned int data)
{
  switch (address)
  {
    case 0xf000:
		asm volatile("mcr p15, 0, %0, c0, c0" : : "r" (data));
		break;
    case 0xf001:
		asm volatile("mcr p15, 0, %0, c1, c0" : : "r" (data));
		break;
    case 0xf002:
		asm volatile("mcr p15, 0, %0, c2, c0" : : "r" (data));
		break;
    case 0xf003:
		asm volatile("mcr p15, 0, %0, c3, c0" : : "r" (data));
		break;
    case 0xf004:
		asm volatile("mcr p15, 0, %0, c4, c0" : : "r" (data));
		break;
    case 0xf005:
		asm volatile("mcr p15, 0, %0, c5, c0" : : "r" (data));
		break;
  }
}
#endif

/**************************************************************************

  CP14 access  / Debug Schnittstelle
  
  ARM family dependent DCC driver functions

**************************************************************************/

#if 0
static inline unsigned int DCC_SendStatus (void)
{
	int status;

	asm volatile("mrc p14, 0, %0, c0, c0" : "=r" (status));

	return (status & 2);
}

static inline void DCC_SendWord (unsigned int data)
{
	asm volatile("mcr p14, 0, %0, c1, c0" : : "r" (data));
}

static inline unsigned int DCC_ReceiveStatus (void)
{
	int  status;

	asm volatile("mrc p14, 0, %0, c0, c0" : "=r" (status));

	return (status & 1);
}

static inline unsigned int DCC_ReceiveWord (void)
{
	unsigned int data;

	asm volatile("mrc p14, 0, %0, c1, c0" : "=r" (data));

	return data;
}

#else

#define DCC_SendStatus()    ({int status; \
                              asm volatile inline("mrc p14, 0, %0, c0, c0" : "=r" (status));\
                              status & 2;})

#define DCC_SendWord(data)    asm inline("mcr p14, 0, %0, c1, c0" : : "r" (data));

#define DCC_ReceiveStatus() ({int status;\
                              asm volatile inline("mrc p14, 0, %0, c0, c0" : "=r" (status));\
							  status & 1;})

#define DCC_ReceiveWord()   ({unsigned int data; \
	                          asm inline("mrc p14, 0, %0, c1, c0" : "=r" (data));\
                              data;})
#endif

/**************************************************************************

  memory access

**************************************************************************/

#ifdef MODE_RAM
static void Monitor_ReadByte (void * buf, void * address, int len)
{
  int i;
  unsigned char *source = (unsigned char *) address;
  unsigned char *target = (unsigned char *) buf;

  //Zur Darstellung des virtuellen Bildschirmsepeichers
  //Debugger greift bei Nutzung von 'd.image 0x10000000 100. 64. /MONO'
  //nur Byteweise auf diesen Sepicher zu.
  if(((int)source & 0xffC00000) == LCD_MEMORY_OFFSET)
  {
	if(udmon3.display_buffer == NULL)
		udmon3.display_buffer = display_get_buffer();

	for (i = 0; i < len; i++) {
		int lcd_y = (((int)&source[i]-LCD_MEMORY_OFFSET)*8) / 104;   //(NXT_WIDTH/8=100/8=12,5->104/8=13);
		int lcd_x = (((int)&source[i]-LCD_MEMORY_OFFSET)*8) - (104*lcd_y);  //Zur Vermeidung von ... % 13
		uint8_t *read=&udmon3.display_buffer[((lcd_y/8)*LCD_WIDTH) + lcd_x];
		uint8_t  mask=1<<(lcd_y%8);
		target[i] = (read[0] & mask ? 0x80 : 0x00) |
		            (read[1] & mask ? 0x40 : 0x00) |
		            (read[2] & mask ? 0x20 : 0x00) |
		            (read[3] & mask ? 0x10 : 0x00) |
		            (read[4] & mask ? 0x08 : 0x00) |
		            (read[5] & mask ? 0x04 : 0x00) |
		            (read[6] & mask ? 0x02 : 0x00) |
		            (read[7] & mask ? 0x01 : 0x00);
	}
  }
  else
	for (i = 0; i < len; i++)
		target[i] = source[i];
}


static void Monitor_ReadHalf (void * buf, void * address, int len)
{
  int i;
  unsigned short *source = (unsigned short *) address;
  unsigned short *target = (unsigned short *) buf;

  if(((int)source & 0xffC00000) == LCD_MEMORY_OFFSET)
  {
	for (i = 0; i < len; i++)
		target[i] = 0x55;
  }
  else
	for (i = 0; i < len; i++)
		target[i] = source[i];
}


static void Monitor_ReadWord (void * buf, void * address, int len)
{
  int i;
  unsigned int *source = (unsigned int *) address;
  unsigned int *target = (unsigned int *) buf;

  if(((int)source & 0xffC00000) == LCD_MEMORY_OFFSET)
  {
	for (i = 0; i < len; i++)
		target[i] = 0xAA;
  }
  else
	for (i = 0; i < len; i++)
		target[i] = source[i];
		
}

static void Monitor_WriteByte (void * buf, void * address, int len)
{
  int i;
  unsigned char *source = (unsigned char *) address;
  unsigned char *target = (unsigned char *) buf;

  if(!((int)target & 0xffC00000))
	for (i = 0; i < len; i++)
		target[i] = source[i];
}

static void Monitor_WriteHalf (void * buf, void * address, int len)
{
  int i;
  unsigned short *source = (unsigned short *) address;
  unsigned short *target = (unsigned short *) buf;

  if(!((int)target & 0xffC00000))
	for (i = 0; i < len; i++)
		target[i] = source[i];
}

static void Monitor_WriteWord (void * buf, void * address, int len)
{
  int i;
  unsigned int *source = (unsigned int *) address;
  unsigned int *target = (unsigned int *) buf;

  if(!((int)target & 0xffC00000))
	for (i = 0; i < len; i++)
		target[i] = source[i];
}
#endif

/**************************************************************************

  Monitor

  When you put in your application make sure the Monitor_Handler() will be
  called periodically.

  Please note that also the monitor itself calls the Monitor_Handler().
  The monitor will fail if you try to debug this function, especially if
  you place software breakpoints there or if you single-step into this
  function or if you try to restart from an on-chip breakpoint in this function.

	TSMON, UDMON:
	The call is typically included in a periodic interrupt or in the idle
	task of the kernel.
	
	PTMON: 
	The call is typically included in the interrupt service routine which 
	will be triggered by the trigger output signal coming from the debugger.

**************************************************************************/

// Receive data from debugger via DCC channel
static inline void udmon3_receive(void)
{
    unsigned int data;
	#ifdef MODE_RAM
			   int index;
			   int len;
	  unsigned int address;
	#endif
    data = DCC_ReceiveWord();

    switch (data >> 28)
    {
      case 0x00:
	    if(udmon3.term_recv) {
          switch (data >> 24) {
            case 0x00: // reserved for terminal input in DCC3 protocol mode
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>> 0)&0xff));
			  break;
            case 0x01:
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>> 0)&0xff));
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>> 8)&0xff));
			  break;
            case 0x02:
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>> 0)&0xff));
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>> 8)&0xff));
			  byte_fifo_cb_put(udmon3.term_recv,(unsigned char)((data>>16)&0xff));
			  break;
            case 0x04:
            case 0x05:
            case 0x06:
            case 0x07: // reserved for FDX transfer to buffer in DCC3 protocol mode
		    case 0x08: // reserved for PTMON3
		    case 0x09: // reserved for PTMON3
		    case 0x0F: // reserved for PTMON3
			  break;
	      }
		}

#ifdef PTMON3
        switch (data >> 24) {
          case 0x08: // get monitor data base
			udmon3.monitor_index = 0;
			udmon3.monitor_count = 4;
			udmon3.monitor_buffer[0] = (unsigned int) udmon3.monitor_stackbase;
			COMMTX_IRQ_ENABLE();
			break;

          case 0x09: // set monitor data base
			((unsigned char *) udmon3.monitor_buffer)[4 - 1] = (data >> 16) & 0xff;
			udmon3.monitor_stackbase = (unsigned int *) udmon3.monitor_buffer[0];
//Wenn das unterste Bit in dieser Adresse gesetzt wird, muss was gesendet werden
//			if(*udmon3.monitor_stackbase & 0x0001)
//				COMMTX_IRQ_ENABLE();
			break;

          case 0x0f: // stop application (on UDMON this is done by a breakpoint range)
			break;
	    }
#endif
	    break;
		
#ifdef MODE_RAM
      case 0x01: // 8-bit read access
		len = ((data >> 24) & 0x0f) + 1;
		address = (udmon3.monitor_address_low & ~0xffffff) | (data & 0xffffff);
		Monitor_ReadByte (udmon3.monitor_buffer, (void *) address, len);
		udmon3.monitor_index = 0;
		udmon3.monitor_count = len;
		COMMTX_IRQ_ENABLE();
		break;
      case 0x02: // 16-bit read access
		len = ((data >> 24) & 0x0f) + 1;
		address = (udmon3.monitor_address_low & ~0xffffff) | (data & 0xffffff);
		Monitor_ReadHalf (udmon3.monitor_buffer, (void *) address, len);
		udmon3.monitor_index = 0;
		udmon3.monitor_count = len * 2;
		COMMTX_IRQ_ENABLE();
		break;

      case 0x03: // 32-bit read access
		len = ((data >> 24) & 0x0f) + 1;
		address = (udmon3.monitor_address_low & ~0xffffff) | (data & 0xffffff);
		Monitor_ReadWord (udmon3.monitor_buffer, (void *) address, len);
		udmon3.monitor_index = 0;
		udmon3.monitor_count = len * 4;
		COMMTX_IRQ_ENABLE();
		break;

      case 0x04: // 8-bit write access
		len = ((data >> 24) & 0x0f) + 1;
		((unsigned char *) udmon3.monitor_buffer)[len - 1] = (data >> 16) & 0xff;
		address = (udmon3.monitor_address_low & ~0xffff) | (data & 0xffff);
		Monitor_WriteByte ((void *) address, udmon3.monitor_buffer, len);
		break;

      case 0x05: // 16-bit write access
		len = ((data >> 24) & 0x0f) + 1;
		((unsigned char *) udmon3.monitor_buffer)[len * 2 - 1] = (data >> 16) & 0xff;
		address = (udmon3.monitor_address_low & ~0xffff) | (data & 0xffff);
		Monitor_WriteHalf ((void *) address, udmon3.monitor_buffer, len);
		break;

      case 0x06: // 32-bit write access
		len = ((data >> 24) & 0x0f) + 1;
		((unsigned char *) udmon3.monitor_buffer)[len * 4 - 1] = (data >> 16) & 0xff;
		address = (udmon3.monitor_address_low & ~0xffff) | (data & 0xffff);
		Monitor_WriteWord ((void *) address, udmon3.monitor_buffer, len);
		break;
	
#if 0
      case 0x07: // 32-bit CP15 read access
		udmon3.monitor_buffer[0] = Monitor_ReadCP15 (data & 0xffff);
		udmon3.monitor_index = 0;
		udmon3.monitor_count = 4;
		COMMTX_IRQ_ENABLE();
		break;
      case 0x08: // 32-bit CP15 write access
		((unsigned char *) udmon3.monitor_buffer)[4 - 1] = (data >> 16) & 0xff;
		Monitor_WriteCP15 (data & 0xffff, udmon3.monitor_buffer[0]);
		break;
#endif
      
	  case 0x0d: // set (part of the) address e.g. for a memory write request
		if ((data & 0x01000000) == 0)
		{
			/* Bits 16..39 */
			udmon3.monitor_address_low = (data << 16);
			udmon3.monitor_address_high = (udmon3.monitor_address_high & ~0xff) | ((data >> 16) & 0xff);
		}
		else
		{
			/* Bits 40..63 */
			udmon3.monitor_address_high = (udmon3.monitor_address_high & ~0xffffff00) | ((data << 8) & 0xffffff00);
		}
		break;
		
      case 0x0e: // set (part of the) data to buffer e.g. for a memory write request
      case 0x0f:
		index = ((data >> 24) & 0x1f) * 3;
		if (index < 21)
		{
			((unsigned char *) udmon3.monitor_buffer)[index + 0] =  data        & 0xff;
			((unsigned char *) udmon3.monitor_buffer)[index + 1] = (data >>  8) & 0xff;
			((unsigned char *) udmon3.monitor_buffer)[index + 2] = (data >> 16) & 0xff;
		}
		break;
#endif
    }
}

static inline void udmon3_transmit(void)
{
	unsigned int data;
	data=0x0;
	
#ifdef MODE_RAM
	// send data e.g. of a memory read request to TRACE32 GUI via DCC channel
	if (udmon3.monitor_index < udmon3.monitor_count)
	{
		data = (((unsigned char *) udmon3.monitor_buffer)[udmon3.monitor_index    ]      ) | 
			   (((unsigned char *) udmon3.monitor_buffer)[udmon3.monitor_index + 1] << 8 ) | 
		       (((unsigned char *) udmon3.monitor_buffer)[udmon3.monitor_index + 2] << 16) | 
		        0x10000000;
		DCC_SendWord(data);
		udmon3.monitor_index += 3;	
	}
	else 
#endif
#ifdef PTMON3
	// send word to host if monitor has been entered due to a debug event
	if(*udmon3.monitor_stackbase & 0x0001)
	{
		*udmon3.monitor_stackbase &= ~0x0001;
		data = (*udmon3.monitor_stackbase & 0xffff) | 0x0f000000;
		DCC_SendWord(data);
	}
	else 
#endif
	if(udmon3.term_send) {
	    if(byte_fifo_cb_get(udmon3.term_send,((unsigned char *)&data)+0) != -1) {
		    if(byte_fifo_cb_get(udmon3.term_send, ((unsigned char *)&data)+1) != -1) {
			    if(byte_fifo_cb_get(udmon3.term_send, ((unsigned char *)&data)+2) != -1) {
				    DCC_SendWord(data | 0x2000000); //3 Bytes
			    }
			    else {
				    DCC_SendWord(data | 0x1000000); //2 Bytes
			    }
		    }
		    else {
		  	  DCC_SendWord(data | 0x0000000); //1 Bytes
		    }
	    }
	}
#if UDMON3_MODE == UDMON3_IRQ
  //Weitere Daten zum Senden vorhanden, wenn nein, dann COMM-TX Deaktivieren
  #ifdef MODE_RAM
	if((udmon3.monitor_index >= udmon3.monitor_count) && (byte_fifo_cb_get_possible(udmon3.term_send) == -1))
	{
		COMMTX_IRQ_DISABLE();
	}
  #else
	if(byte_fifo_cb_get_possible(udmon3.term_send) == -1)
	{
		COMMTX_IRQ_DISABLE();
	}
  #endif
#endif	
}

//The bits COMMRX and COMMTX, which indicates, respectively, 
//- that the read register has been written by the debugger but not yet read by the processor, 
//- and that the write register has been written bei the processor and not yet read by the debugger
//are wired on the two highest bits of the status register DBGU_SR. 

__attribute__ ((section (".text.fastcode")))
void udmon3_handler(void)
{
#if UDMON3_MODE == UDMON3_ZYKLISCH
	if (DCC_ReceiveStatus())   //Zeichen vom Debugger bereitgestellt?
		udmon3_receive();

	if(!DCC_SendStatus())      //Debugger Empfangsbereit?
		udmon3_transmit();
#endif
#if UDMON3_MODE == UDMON3_IRQ
	
//	do {  So schnell ist der Debugger leider nicht
//        Stattdessen wird erst die ISR Verlassen um diese unmittelbar wieder
//        aufzurufen!
		if(*AT91C_DBGU_CSR & *AT91C_DBGU_IMR & AT91C_US_COMM_RX)
			udmon3_receive();
		if(*AT91C_DBGU_CSR & *AT91C_DBGU_IMR & AT91C_US_COMM_TX)
			udmon3_transmit();
//	} while(*AT91C_DBGU_CSR & *AT91C_DBGU_IMR & (AT91C_US_COMM_TX | AT91C_US_COMM_RX));
#endif
}

#ifdef PTMON3
;=================================================================
; Aus background_monitor/monitor_entry.s
;=================================================================
;  Background Monitor for TRACE32 ARM JTAG Debugger
;  PEG, July 2008
;
;  If the debugger is switched from halt mode debugging to monitor
;  mode debugging (Go.MONitor, Break.SETMONITOR ON), a trap
;  (PABORT, DABORT) will happen instead of halting the program execution
;  when a breakpoint hits or when the BREAK button gets pushed.
;  (The BREAK button causes an on-chip breakpoint range 0-0xffffffff
;  for user mode.)
;
;  When UDMON3 is selected, all onchip breakpoints will be specified
;  for user mode only. It assumes the application which shall be debugged 
;  is running in user mode. TSMON3 and PTMON3 can theoretically also 
;  be used with the background monitor, but then the BREAK button can
;  not be imlemented by a breakpoint range, since even the monitor code 
;  would cause a re-entry into the monitor. Therefore the other two modes 
;  shall not be used for background monitor debugging.
;
;  This example has been tested on an ARM966E-S and ARM926EJ-S Integrator
;  Core Module from ARM. An ARM9..E-S derivative is needed since on other
;  ARM9 derivatives the on-chip breakpoints can not be specified for user mode.
;  Theoretically it will also work on ARM11 and Cortex, but this has not
;  been tested. Then a few changes of the monitor code might be required,
;  but at least the DCC_ functions need to be adapted.
;
;  This trap routine saves the processor registers in a data block at
;  Monitor_StackBase where the TRACE32 software will read it.
;  See the order below.
;
;  The halt reason will be stored at the first data block location.
;  Bit 0 will be set to signalize the Monitor_Handler that the TRACE32
;  software shall be informed that a debug event has happened. The 
;  Monitor_Handler sends the message and clears bit 0. At the beginning
;  this bit needs to be cleared (see monitor.cmm).
;
;  The monitor waits in a loop until TRACE32 causes a re-start of
;  the application. In the same loop memory accesses requested by 
;  TRACE32 will be serviced by calling the Monitor_Handler.
;  TRACE32 can modify the register values by writing to the data block.
;
;  If you enter 'DIAG 3800' in the TRACE32 command line, the communication
;  of the TRACE32 GUI with the Monitor_Handler will be printed in the 
;  'AREA' window.
;
;  Note that the monitor uses the DCC channel. Therefore it can not additionally 
;  be used for terminal or semihosting, except it is done based on a memory interface
;  instead of DCC.
;
;  Monitor_StackBase
;  + 0x00 entry reason, 'debug event just happened' flag (bit 0), re-start signal
;  + 0x04 R0
;  + 0x08 R1
;  + 0x0c R2
;  + 0x10 R3
;  + 0x14 R4
;  + 0x18 R5
;  + 0x1c R6
;  + 0x20 R7
;  + 0x24 R8_USR
;  + 0x28 R9_USR
;  + 0x2c R10_USR
;  + 0x30 R11_USR
;  + 0x34 R12_USR
;  + 0x38 R15
;  + 0x3c CPSR
;  + 0x40 R13_USR
;  + 0x44 R14_USR
;  + 0x48 R13_SVC
;  + 0x4c R14_SVC
;  + 0x50 SPSR_SVC
;  + 0x54 R8_FIQ
;  + 0x58 R9_FIQ
;  + 0x5c R10_FIQ
;  + 0x60 R11_FIQ
;  + 0x64 R12_FIQ
;  + 0x68 R13_FIQ
;  + 0x6c R14_FIQ
;  + 0x70 SPSR_FIQ
;  + 0x74 R13_IRQ
;  + 0x78 R14_IRQ
;  + 0x7c SPSR_IRQ
;  + 0x80 R13_ABT
;  + 0x84 R14_ABT
;  + 0x88 SPSR_ABT
;  + 0x8c DACR
;
;  R13_UND, R14_UND, SPSR_UND are used by the monitor and are
;  therefore of no importance for the application debugging.
;
;=================================================================

; entrypoint with information header for compile phase informations

  AREA  Monitor, CODE, READONLY

  EXPORT  Monitor_EntryRES
  EXPORT  Monitor_EntryUND
  EXPORT  Monitor_EntrySWI
  EXPORT  Monitor_EntryPABT
  EXPORT  Monitor_EntryDABT
  EXPORT  Monitor_EntryIRQ
  EXPORT  Monitor_EntryFIQ
  EXPORT  Monitor_Entry
  EXPORT  Monitor_Polling

  IMPORT  Monitor_Handler


;=================================================================
;  Sample interrupt vector table
;=================================================================

InitVectors LDR PC, _res_vec    ;0x00 Reset
            LDR PC, _und_vec    ;0x04 UNDEF
            LDR PC, _swi_vec    ;0x08 SWI
            LDR PC, _pabt_vec   ;0x0C PABORT
            LDR PC, _dabt_vec   ;0x10 DABORT
            LDR PC, _rsv_vec    ;0x14 reserved
            LDR PC, _irq_vec    ;0x18 IRQ
            LDR PC, _fiq_vec    ;0x1c FIQ

_res_vec  DCD Monitor_EntryRES    ;0x20
_und_vec  DCD Monitor_EntryUND    ;0x24
_swi_vec  DCD Monitor_EntrySWI    ;0x28
_pabt_vec DCD Monitor_EntryPABT   ;0x2c
_dabt_vec DCD Monitor_EntryDABT   ;0x30
_rsv_vec  DCD Monitor_EntrySWI    ;0x34
_irq_vec  DCD Monitor_EntryIRQ    ;0x38
_fiq_vec  DCD Monitor_EntryFIQ    ;0x3c



;=================================================================
;  Monitor Entry Points
;=================================================================

  IMPORT  Monitor_StackBase
Monitor_RegistersPtr
  DCD Monitor_StackBase


Monitor_EntryRES
  MRS R14, CPSR
  MSR CPSR_c, #0xdb     ; switch to undefined mode

  LDR R13, Monitor_RegistersPtr
  LDR R13, [R13]

  STR   R12, [SP, #0x34]
  MOV R12, #0x41        ; reason for entry is RESET

  STR R14, [SP, #0x3c]  ; CPSR

  MOV R14, #0
  STR R14, [SP, #0x38]  ; PC (=0)
  STR R7,  [SP, #0x20]

  B Monitor_Entry


Monitor_EntryUND
  STR   R12, [SP, #0x34]
  MOV R12, #0x11

  SUB R14, R14, #0x04
  STR R14, [SP, #0x38]  ; R14_und = PC
  MRS R14, SPSR
  STR R14, [SP, #0x3c]  ; SPSR_und = CPSR
  STR R7,  [SP, #0x20]

  B Monitor_Entry


Monitor_EntrySWI
  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode
  STR R12, [SP, #0x34]  ; save r12
  STR R7,  [SP, #0x20]  ; save r7
  MOV R7, SP            ; load SP_und into R7

  MSR CPSR_c, #0xD3     ; switch back to SVC mode

  SUB R14, R14, #0x04   ; calculate and save PC
  STR R14, [R7, #0x38]

  MRS R12, SPSR         ; save spsr
  STR R12, [R7, #0x3c]

  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  MOV R12, #0x11
  B Monitor_Entry


Monitor_EntryPABT
  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode
  STR R12, [SP, #0x34]  ; save r12
  STR R7,  [SP, #0x20]  ; save r7
  MOV R7, SP            ; load SP_und into R7

  MSR CPSR_c, #0xD7     ; switch back to ABT mode

  SUB R14, R14, #0x04   ; calculate and save PC
  STR R14, [R7, #0x38]

  MRS R12, SPSR         ; save spsr
  STR R12, [R7, #0x3c]

  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  MOV R12, #0x11
  B Monitor_Entry


Monitor_EntryDABT
  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  LDR R13,Monitor_RegistersPtr
  LDR R13, [R13]

  LDR R14, [SP]
  CMP R14,#0            ; executed from inside monitor ?
  MOVNE R12, #0xf1
  STRNE R12, [SP]
  BNE Monitor_Polling   ; enter loop directly

  STR R12, [SP, #0x34]  ; save r12
  STR R7,  [SP, #0x20]  ; save r7
  MOV R7, SP            ; load SP_und into R7

  MSR CPSR_c, #0xD7     ; switch back to ABT mode

  SUB R14, R14, #0x04   ; calculate and save PC
  STR R14, [R7, #0x38]

  MRS R12, SPSR         ; save spsr
  STR R12, [R7, #0x3c]

  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  MOV R12, #0x11
  B Monitor_Entry


Monitor_EntryIRQ
  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode
  STR R12, [SP, #0x34]  ; save r12
  STR R7,  [SP, #0x20]  ; save r7
  MOV R7, SP            ; load SP_und into R7

  MSR CPSR_c, #0xD2     ; switch back to IRQ mode

  SUB R14, R14, #0x04   ; calculate and save PC
  STR R14, [R7, #0x38]

  MRS R12, SPSR         ; save spsr
  STR R12, [R7, #0x3c]

  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  MOV R12, #0x11
  B Monitor_Entry


Monitor_EntryFIQ
  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode
  STR R12, [SP, #0x34]  ; save r12
  STR R7,  [SP, #0x20]  ; save r7
  MOV R7, SP            ; load SP_und into R7

  MSR CPSR_c, #0xD1     ; switch back to FIQ mode

  SUB R14, R14, #0x04   ; calculate and save PC
  STR R14, [R7, #0x38]

  MRS R12, SPSR         ; save spsr
  STR R12, [R7, #0x3c]

  MSR CPSR_c, #0xDB     ; switch to undefined opcode mode

  MOV R12, #0x11
  B Monitor_Entry


;=================================================================
;  Breakpoint Entry
;=================================================================

Monitor_Entry
  STR R11, [SP, #0x30]
  STR R10, [SP, #0x2c]
  STR R9,  [SP, #0x28]
  STR R8,  [SP, #0x24]

  STR R6,  [SP, #0x1c]
  STR R5,  [SP, #0x18]
  STR R4,  [SP, #0x14]
  STR R3,  [SP, #0x10]
  STR R2,  [SP, #0x0c]
  STR R1,  [SP, #0x08]
  STR R0,  [SP, #0x04]


;==============================================================
  MRS R0, CPSR          ; switch to sys mode
  ORR R1, R0, #0x0F
  MSR CPSR_c, R1

  MOV R11, R13
  MOV R4, R14

  MSR CPSR_c, R0        ; switch back

  STR R11, [SP, #0x40]
  STR R4, [SP, #0x44]

;==============================================================
  MRS R0, CPSR          ; switch to svc mode
  ORR R1, R0, #0x03
  AND R1, R1, #0xF3
  MSR CPSR_c, R1

  MOV R11, R13
  MOV R4, R14
  MRS R5, SPSR

  MSR CPSR_c, R0        ; switch back

  STR R11, [SP, #0x48]
  STR R4, [SP, #0x4c]
  STR R5, [SP, #0x50]

;==============================================================
  MRS R0, CPSR          ; switch to fiq mode(1)
  ORR R1, R0, #0x01
  AND R1, R1, #0xf1
  MSR CPSR_c, R1

  MOV R3,  R8
  MOV R4,  R9
  MOV R5,  R10
  MOV R6,  R11

  MSR CPSR_c, R0        ; switch back

  STR R3, [SP, #0x54]
  STR R4, [SP, #0x58]
  STR R5, [SP, #0x5c]
  STR R6, [SP, #0x60]

  MRS R0, CPSR          ; switch to fiq mode(2)
  ORR R1, R0, #0x01
  AND R1, R1, #0xf1
  MSR CPSR_c, R1

  MOV R3,  R12
  MOV R4,  R13
  MOV R5,  R14
  MRS R6,  SPSR

  MSR CPSR_c, R0        ; switch back

  STR R3, [SP, #0x64]
  STR R4, [SP, #0x68]
  STR R5, [SP, #0x6c]
  STR R6, [SP, #0x70]

;==============================================================
  MRS R0, CPSR          ; switch to irq mode
  ORR R1, R0, #0x02
  AND R1, R1, #0xF2
  MSR CPSR_c, R1

  MOV R11, R13
  MOV R4, R14
  MRS R5, SPSR

  MSR CPSR_c, R0        ; switch back

  STR R11, [SP, #0x74]
  STR R4, [SP, #0x78]
  STR R5, [SP, #0x7c]

;==============================================================
  MRS R0, CPSR          ; switch to abt mode
  ORR R1, R0, #0x07
  AND R1, R1, #0xF7
  MSR CPSR_c, R1

  MOV R11, R13
  MOV R4, R14
  MRS R5, SPSR

  MSR CPSR_c, R0        ; switch back

  STR R11, [SP, #0x80]
  STR R4, [SP, #0x84]
  STR R5, [SP, #0x88]

;==============================================================

  IF :DEF: MONITOR_OVERWRITE_PROTCR ; same effect as 'SYStem.Option DACR ON' in halt mode debugging on ARM926EJ-S
  MRC p15, 0, r4, c3, c0
  STR R4, [SP, #0x8c]
  MOV R4, -1
  MCR p15, 0, r4, c3, c0
  ENDIF

;==============================================================

  STR R12, [SP]         ; save monitor entry reason code

Monitor_Polling
  BL  Monitor_Handler   ; wait for go and service memory read/write requests in the meantime
  LDR R12, [SP]
  CMP R12,#0
  BNE Monitor_Polling

;==============================================================

  IF :DEF: MONITOR_OVERWRITE_PROTCR
  LDR R4, [SP, #0x8c]
  MCR p15, 0, r4, c3, c0
  ENDIF


;==============================================================
  LDR R11, [SP, #0x40]
  LDR R4, [SP, #0x44]

  MRS R0, CPSR          ; switch to sys mode
  ORR R1, R0, #0x0F
  MSR CPSR_c, R1

  MOV R13, R11
  MOV R14, R4

  MSR CPSR_c, R0        ; switch back


;==============================================================
  LDR R11, [SP, #0x48]
  LDR R4, [SP, #0x4c]
  LDR R5, [SP, #0x50]

  MRS R0, CPSR          ; switch to svc mode
  ORR R1, R0, #0x03
  AND R1, R1, #0xF3
  MSR CPSR_c, R1

  MOV R13, R11
  MOV R14, R4
  MSR SPSR_cxsf, R5

  MSR CPSR_c, R0        ; switch back

;==============================================================
  LDR R3, [SP, #0x54]
  LDR R4, [SP, #0x58]
  LDR R5, [SP, #0x5c]
  LDR R6, [SP, #0x60]

  MRS R0, CPSR          ; switch to fiq mode(1)
  ORR R1, R0, #0x01
  AND R1, R1, #0xf1
  MSR CPSR_c, R1

  MOV R8, R3
  MOV R9, R4
  MOV R10, R5
  MOV R11, R6

  MSR CPSR_c, R0        ; switch back

  LDR R3, [SP, #0x64]
  LDR R4, [SP, #0x68]
  LDR R5, [SP, #0x6c]
  LDR R6, [SP, #0x70]

  MRS R0, CPSR          ; switch to fiq mode(2)
  ORR R1, R0, #0x01
  AND R1, R1, #0xf1
  MSR CPSR_c, R1

  MOV R12, R3
  MOV R13, R4
  MOV R14, R5
  MSR SPSR_cxsf, R6

  MSR CPSR_c, R0        ; switch back

;==============================================================
  LDR R11, [SP, #0x74]
  LDR R4, [SP, #0x78]
  LDR R5, [SP, #0x7c]

  MRS R0, CPSR          ; switch to irq mode
  ORR R1, R0, #0x02
  AND R1, R1, #0xF2
  MSR CPSR_c, R1

  MOV R13, R11
  MOV R14, R4
  MSR SPSR_cxsf, R5

  MSR CPSR_c, R0        ; switch back

;==============================================================
  LDR R11, [SP, #0x80]
  LDR R4, [SP, #0x84]
  LDR R5, [SP, #0x88]

  MRS R0, CPSR          ; switch to abt mode
  ORR R1, R0, #0x07
  AND R1, R1, #0xF7
  MSR CPSR_c, R1

  MOV R13, R11
  MOV R14, R4
  MSR SPSR_cxsf, R5

  MSR CPSR_c, R0        ; switch back

  LDR R0, [SP, #0x3c]   ; SPSR_und (=CPSR)
  MSR SPSR_cxsf, R0

  LDR R14, [SP, #0x38]  ; R14_und (=PC)
  LDR R0,  [SP, #0x04]  ; R0
  LDR R1,  [SP, #0x08]  ; R1
  LDR R2,  [SP, #0x0c]  ; R2
  LDR R3,  [SP, #0x10]  ; R3
  LDR R4,  [SP, #0x14]  ; R4
  LDR R5,  [SP, #0x18]  ; R5
  LDR R6,  [SP, #0x1c]  ; R6
  LDR R7,  [SP, #0x20]  ; R7
  LDR R8,  [SP, #0x24]  ; R8
  LDR R9,  [SP, #0x28]  ; R9
  LDR R10, [SP, #0x2c]  ; R10
  LDR R11, [SP, #0x30]  ; R11
  LDR R12, [SP, #0x34]  ; R12

  MOVS  PC, R14         ; return to application

;==============================================================

  END
#endif

#pragma GCC pop_options
