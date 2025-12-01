#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> //fuer _exit()

#include "AT91SAM7S64.h"
#include "lib/aic.h"
#include "lib/display.h"
#include "lib/nxt_avr.h"
#include "lib/systick.h"
#include "lib/term.h"
#include "main.h"

#include <stdbool.h>
// #include "lib/adc.h"
#if defined(MODE_RAM) || defined(MODE_SIM)
#include "trace32/udmon3.h"
#endif

// Vorsicht
//- Zyklusdauer von 2ms auf 16ms hochgesetzt
//- Überprüfung der Zeitscheibenüberschreitung deaktiviert
#define ZYKLUS_MS 16
#define IDLE_MS 1

#if IDLE_MS >= ZYKLUS_MS
#error "Idle_ms muss kleiner als zyklus_ms sein"
#endif

/*****************************************************************************/
/*   I2C Variablen                                                           */
/*****************************************************************************/

// NXT-Sensorport Beschreibung
#define P0_SDA 23 // Sensor-1 DIGx0  PA23=SCK1/PWM0
#define P0_SCL 18 // Sensor-1 DIGx1  PA18=RD  /PCK2/AD1
#define P1_SDA 28 // Sensor-2 DIGx0  PA28=DSR1/TCLK1
#define P1_SCL 19 // Sensor-2 DIGx1  PA19=RK  /FIQ /AD2
#define P2_SDA 29 // Sensor-3 DIGx0  PA29=RI1 /TCLK2
#define P2_SCL 20 // Sensor-3 DIGx1  PA20=RF  /IRQ0/AD3
#define P3_SDA 30 // Sensor-4 DIGx0  PA30=IRQ1/NPCS2
#define P3_SCL 2  // Sensor-4 DIGx1  PA2 =PWM2/SCK0

const struct {
  uint32_t i2c_sda;
  uint32_t i2c_scl;
} i2c_mask[SENSOR_MAX] = {{(1 << P0_SDA), (1 << P0_SCL)},
                          {(1 << P1_SDA), (1 << P1_SCL)},
                          {(1 << P2_SDA), (1 << P2_SCL)},
                          {(1 << P3_SDA), (1 << P3_SCL)}};

#define NXT_I2C_PORT SENSOR_1

/*****************************************************************************/
/*   Globale Variablen                                                       */
/*****************************************************************************/
struct {
  signed char term_status; // 0->false->Alles Bestens   -1->true->Overflow
  unsigned char term_cnt;
  unsigned char lowbat_cnt;
} main_data = {
    .term_status = 0,
    .term_cnt = 0,
    .lowbat_cnt = 0,
};

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*   Standard-C-Library (weitere befinden sich in newlib_syscalls.c)         */
/*****************************************************************************/
// Routine wird von C-Lib aufgerufen (bspw. printf() abort())
void _exit(int status) {
  (void)status;
  // LED-Blinken lassen
  // Breakpoint setzen
  while (1)
    ;
}

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*   zur Datenaufnahme der SDA/SCL Leitung, so dass diese als SCOPE          */
/*   im Debugger über 'v.draw %e scope.buf0 scope.buf1' dargestellt          */
/*   werden kann.                                                            */
/*****************************************************************************/

typedef enum { SCOPE_STOP, SCOPE_CONTINUE, SCOPE_SINGLE } SCOPE_MODE;

#define SCOPE_SIZE 500

struct {
  uint8_t buf0[SCOPE_SIZE];
  uint8_t buf1[SCOPE_SIZE];
  SCOPE_MODE mode;
  uint32_t idx;
} scope = {.mode = SCOPE_STOP};

__attribute__((section(".text.fastcode"))) void pwm_isr_entry(void) {
  uint32_t pinChanges = AT91C_BASE_PWMC->PWMC_ISR; // Acknowledge change
  if (pinChanges & (1 << 0)) {
    if (scope.mode == SCOPE_CONTINUE) {
      for (int lauf = 0; lauf < (SCOPE_SIZE - 1); lauf++) {
        scope.buf0[lauf] = scope.buf0[lauf + 1];
        scope.buf1[lauf] = scope.buf1[lauf + 1];
      }
      uint32_t pio = *AT91C_PIOA_PDSR;
      scope.buf0[SCOPE_SIZE - 1] =
          pio & i2c_mask[NXT_I2C_PORT].i2c_sda ? 100 : 0;
      scope.buf1[SCOPE_SIZE - 1] =
          pio & i2c_mask[NXT_I2C_PORT].i2c_scl ? 250 : 150;
    } else if (scope.mode == SCOPE_SINGLE) {
      uint32_t pio = *AT91C_PIOA_PDSR;
      scope.buf0[scope.idx] = pio & i2c_mask[NXT_I2C_PORT].i2c_sda ? 100 : 0;
      scope.buf1[scope.idx] = pio & i2c_mask[NXT_I2C_PORT].i2c_scl ? 250 : 150;

      scope.idx++;
      if (scope.idx == SCOPE_SIZE) {
        scope.mode = SCOPE_STOP;
        AT91C_BASE_PWMC->PWMC_DIS = 1 << 0; // PWM Disable
        AT91C_BASE_PWMC->PWMC_IDR = 1 << 0; // Disable Interrupt
      }
    }
  }
}

int scope_init(SCOPE_MODE mode, uint32_t zyklus_us) {
  // Clock-Delivery
  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PWMC;
  scope.mode = mode;

  AT91C_BASE_PWMC->PWMC_DIS = 1 << 0; // PWM Disable
  AT91C_BASE_PWMC->PWMC_IDR = 1 << 0; // Disable Interrupt

  if (mode != SCOPE_STOP) {
    if ((zyklus_us < 100) || (zyklus_us > 10900)) {
      // 1/48.000.000*8=10,9ms
      scope.mode = SCOPE_STOP;
      return -1;
    }

    AT91C_BASE_PWMC->PWMC_CH[0].PWMC_CMR =
        3 /* =/8 | AT91C_PWMC_CALG | AT91C_PWMC_CPOL | AT91C_PWMC_CPD*/;
    AT91C_BASE_PWMC->PWMC_CH[0].PWMC_CPRDR = (zyklus_us * (MCK / 1000000)) / 8;
    AT91C_BASE_PWMC->PWMC_CH[0].PWMC_CDTYR = 0;

    aic_mask_off(AT91C_ID_PWMC);
    aic_set_vector(AT91C_ID_PWMC, AIC_INT_LEVEL_ABOVE_NORMAL, pwm_isr_entry);
    /*AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL */
    aic_mask_on(AT91C_ID_PWMC);

    scope.idx = 0;

    uint32_t pinChanges = AT91C_BASE_PWMC->PWMC_ISR; // Acknowledge change
    (void)pinChanges;
    AT91C_BASE_PWMC->PWMC_IER = 1 << 0; // Interrupt Enable Register
    AT91C_BASE_PWMC->PWMC_ENA = 1 << 0; // PWM Enalbe
  }
  return 0;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

struct {
  struct i2c {
    int started;
    int arbitration_lost;
  } i2c[4];
  // Debug values for MCP23017
  uint8_t mcp_iodira_readback;
  uint8_t mcp_olata_readback;
  uint8_t mcp_init_complete;
  // I2C scanner results (addresses 0x20-0x27)
  uint8_t scan_results[8]; // 1 = ACK, 0 = NACK
  uint8_t scan_complete;
} i2c_data;

AT91PS_PIO pio_a = AT91C_BASE_PIOA;

void i2c_init(void) {
  // I2C-Datenstruktur initialisieren
  for (int lauf = 0; lauf < 4; lauf++) {
    i2c_data.i2c[lauf] = (struct i2c){.started = 0, .arbitration_lost = 0};

    uint32_t pin_mask = i2c_mask[lauf].i2c_scl | i2c_mask[lauf].i2c_sda;

    //- Enable PIO control of pins (not peripheral)
    pio_a->PIO_PER = pin_mask;
    //- Enable output
    pio_a->PIO_OER = pin_mask;
    //- Als OpenCollector Ausgang
    pio_a->PIO_MDER = pin_mask;
    //- Pull-Up Disable (external pull-ups present)
    pio_a->PIO_PPUDR = pin_mask;
    //- Set pins high (idle state)
    pio_a->PIO_SODR = pin_mask;
  }
  // I2C Treiberroutinen konfigurieren
}

// Grundlegende I2C-Routine aus alten Wikipedia-Artikel über I2C.
//  i2c_start_cond()
//    Zum Senden der Start-Bedingung (Bus Arbitrierung)
//  i2c_stop_cond()
//    Zum Senden der Stop-Bedingung (Bus freigeben)
//  i2c_write_bit()
//    Zum Senden eines einzel Bits (Data oder Acknowledge)
//  i2c_read_bit()
//    Zum Empfangen eines einzelnen Bits (Data oder Acknowledge)
//  i2c_write_byte()
//    Zum Senden eines Bytes incl. Ack-Bit (optional mit Start- und
//    Stop-Bedingung)
//  i2c_read_byte()
//    Zum Emfang eines Bytes incl. Ack-Bit (optional mit Stop-Bedingung)
//  arbitration_lost()
//    Wird aufgerufen, wenn ein I2C Protokollfehler aufgetreten ist
//  I2C_delay()
//    Verzögerungsfunktion, mit dessen Abstand die einzelnen Bits
//    auf die Leitung gesetzt werden. In der Regel wird pro
//    zu übertragenden Bit diese Funktion zweimal aufgerufen
//    Bei einer Delay-Zeit von 100µs ergibt sich folglich eine
//    Taktfrequenz von ~1/200µs=~5kHz

// Anmerkungen
//- arbitration_lost() sollte im Normalfall nicht
//   auftreten/aufgerufen werden, so dass sie hier nur ein Flag setzen
//   brauchen, welches sie über den Debugger im Auge behalten sollten
//- Anstatt die vorgebenen Funktionen als Funktion zu implentieren
//   empfiehlt sich die Implementierung als Makro (#define set_SDA() ...)
//- Über scope_init() kann ein Oszilloskop getriggert werden, welcher
//   die SDA und SCL Leitungen aufnimmt.
//   Mittels 'v.draw %e scope.buf0 scope.buf1' kann diese Aufnahme im
//   Debugger dargestellt werden.
//   Das Oszilloskop unterstüzt zwei Betriebsarten, einstellbar mittels
//   scope_init()
//   - scope_init(SCOPE_SINGLE,250 /* Zeit in us */);
//     Hiermit werden 500 Werte aufgenommen und danach die Aufnahme gestoppt
//     Sinnvoll, um genau einen I2C Datenfluss aufzunehmen. sollte somit
//     immer zu Beginn eines I2C Zyklus aufgerufen werden
//   - scope_init(SCOPE_CONTINUE,250 /* Zeit in us */);
//     Hiermit wird kontinuierlich aufgenommen.
//   Zur Nutzung des Oszilloskop muss die I2C-Taktrate niedrig sein, welches
//     mit systick_wait_ms(1) dargestellt wird
//     Taktrate: 1/(2*1ms)=~500Hz
//     Übertragungsdauer für ein Byte: 9*2ms=18ms
//   Wenn der Datenfluss über das Oszilloskop als korrekt bewertet wurde
//     kann I2C_delay() mit der for-Schleife realisiert werden
//     Taktrate: 1/(2*100µs)=~5kHz
//     Übertragungsdauer für ein Byte: 9*200µs=1,8ms

#define SCL_SLOW_CLOCK

void I2C_delay(void) {
#ifdef SCL_SLOW_CLOCK
  systick_wait_ms(1);
#else
  volatile int v = 0;
  int i;
  // 500  -> 100µs Delay --> 4800/500=10 Befehle pro Durchlauf
  for (i = 0; i < 500 / 2; ++i) {
    v;
  }
#endif
}

#if 1 // Bitte einkommentieren

// Hardware-specific support functions that MUST be customized:
void I2C_delay(void);

bool read_SCL(void);  // Return current level of SCL line, 0 or 1
bool read_SDA(void);  // Return current level of SDA line, 0 or 1
void set_SCL(void);   // Do not drive SCL (set pin high-impedance)
void clear_SCL(void); // Actively drive SCL signal low
void set_SDA(void);   // Do not drive SDA (set pin high-impedance)
void clear_SDA(void); // Actively drive SDA signal low
void arbitration_lost(void);

#define I2C i2c_mask[NXT_I2C_PORT]
bool read_SCL(void) { return pio_a->PIO_PDSR & I2C.i2c_scl; }

bool read_SDA(void) { return pio_a->PIO_PDSR & I2C.i2c_sda; }

void set_SCL(void) { pio_a->PIO_SODR = I2C.i2c_scl; }

void clear_SCL(void) { pio_a->PIO_CODR = I2C.i2c_scl; }

void set_SDA(void) { pio_a->PIO_SODR = I2C.i2c_sda; }

void clear_SDA(void) { pio_a->PIO_CODR = I2C.i2c_sda; }

void arbitration_lost(void) { i2c_data.i2c[NXT_I2C_PORT].arbitration_lost = 1; }

#define started i2c_data.i2c[NXT_I2C_PORT].started

void i2c_start_cond(void) {
  if (started) {
    // if started, do a restart condition
    // set SDA to 1
    set_SDA();
    I2C_delay();
    set_SCL();
    while (read_SCL() == 0) {
      // Clock stretching
      // You should add timeout to this loop
    }

    // Repeated start setup time, minimum 4.7us
    I2C_delay();
  }

  if (read_SDA() == 0) {
    arbitration_lost();
  }

  // SCL is high, set SDA from 1 to 0.
  clear_SDA();
  I2C_delay();
  clear_SCL();
  started = true;
}

void i2c_stop_cond(void) {
  // set SDA to 0
  clear_SDA();
  I2C_delay();

  set_SCL();
  // Clock stretching
  while (read_SCL() == 0) {
    // add timeout to this loop.
  }

  // Stop bit setup time, minimum 4us
  I2C_delay();

  // SCL is high, set SDA from 0 to 1
  set_SDA();
  I2C_delay();

  if (read_SDA() == 0) {
    arbitration_lost();
  }

  started = false;
}

// Write a bit to I2C bus
void i2c_write_bit(bool bit) {
  if (bit) {
    set_SDA();
  } else {
    clear_SDA();
  }

  // SDA change propagation delay
  I2C_delay();

  // Set SCL high to indicate a new valid SDA value is available
  set_SCL();

  // Wait for SDA value to be read by target, minimum of 4us for standard mode
  I2C_delay();

  while (read_SCL() == 0) {
    // Clock stretching
    // You should add timeout to this loop
  }

  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
  if (bit && (read_SDA() == 0)) {
    arbitration_lost();
  }

  // Clear the SCL to low in preparation for next change
  clear_SCL();
}

// Read a bit from I2C bus
bool i2c_read_bit(void) {
  bool bit;

  // Let the target drive data
  set_SDA();

  // Wait for SDA value to be written by target, minimum of 4us for standard
  // mode
  I2C_delay();

  // Set SCL high to indicate a new valid SDA value is available
  set_SCL();

  while (read_SCL() == 0) {
    // Clock stretching
    // You should add timeout to this loop
  }

  // Wait for SDA value to be written by target, minimum of 4us for standard
  // mode
  I2C_delay();

  // SCL is high, read out bit
  bit = read_SDA();

  // Set SCL low in preparation for next operation
  clear_SCL();

  return bit;
}

// Write a byte to I2C bus. Return 0 if ack by the target.
bool i2c_write_byte(bool send_start, bool send_stop, unsigned char byte) {
  unsigned bit;
  bool nack;

  if (send_start) {
    i2c_start_cond();
  }

  for (bit = 0; bit < 8; ++bit) {
    i2c_write_bit((byte & 0x80) != 0);
    byte <<= 1;
  }

  nack = i2c_read_bit();

  if (send_stop) {
    i2c_stop_cond();
  }

  return nack;
}

// Read a byte from I2C bus
unsigned char i2c_read_byte(bool nack, bool send_stop) {
  unsigned char byte = 0;
  unsigned char bit;

  // i2c_read_bit() 1 0
  for (bit = 0; bit < 8; ++bit) {
    byte = (byte << 1) | i2c_read_bit();
  }

  i2c_write_bit(nack);

  if (send_stop) {
    i2c_stop_cond();
  }

  return byte;
}
#endif

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
// DevicdeAdresse des GPIO-Portexanders
// Common addresses: 0x20 (A2/A1/A0 = LOW), 0x27 (A2/A1/A0 = HIGH)
// Scanner found device at 0x27! (A2/A1/A0 all HIGH)
#define MCP2317_ADDR 0x27

// Registerbeschreibung des GPIO-Portexanders
#define MCP2317_0_IODIRA 0x00   // IO Direction (Default: 1)
#define MCP2317_0_IODIRB 0x01   // 0->Output 1->Input
#define MCP2317_0_IPOLA 0x02    // Input Polarity (Default: 0)
#define MCP2317_0_IPOLB 0x03    //--> 0->Same 1->Invertet
#define MCP2317_0_GPINTENA 0x04 // Interrupt On Change (Default: 0)
#define MCP2317_0_GPINTENB 0x05 //--> 0->Disable Interrupt 1->Enable Interrupt
#define MCP2317_0_DEFVALA 0x06  // Default Compare Register for Interrupt
#define MCP2317_0_DEFVALB                                                      \
  0x07 // --> If the Associated pin level is the opposite, an interrupt occurs
#define MCP2317_0_INTCONA 0x08 // Interrupt Control (Default: 0)
#define MCP2317_0_INTCONB                                                      \
  0x09                       // --> 0->Both Edge triggered 1->Opposite to DEFVAL
#define MCP2317_0_GPPUA 0x0C // Pull Up Register (Default: 0)
#define MCP2317_0_GPPUB                                                        \
  0x0D // --> 0->PullUp Enabled         1-> PullUp Disabled
#define MCP2317_0_INTFA 0x0E // Interrupt Flag Register (Default: 0)
#define MCP2317_0_INTFB                                                        \
  0x0F // --> 0->Interrupt not pending  1-> Interrupt pending
#define MCP2317_0_INTCAPA 0x10 // Interrupt Capture Register
#define MCP2317_0_INTCAPB                                                      \
  0x11 // --> These bits reflects the logic level at the time of interrupt
#define MCP2317_0_GPIOA 0x12 // Port Register
#define MCP2317_0_GPIOB 0x13 // --> Reading reads the port Write modiesfies OLAT
#define MCP2317_0_OLATA 0x14 // Output Latch
#define MCP2317_0_OLATB                                                        \
  0x15 // --> Reading reads the OLAT (not the port) Write modiesfies die OLAT
#define MCP2317_0_IOCONA 0x0A // IO configuration Register (Default: 0)
#define MCP2317_0_IOCONB 0x0B

#define MCP2317_IOCON_BANK 0x80 // 0-> Bank 0 Selected       1->Bank 1
#define MCP2317_IOCON_MIRROR                                                   \
  0x40 // 0-> InTA/IntB separat     1->INTA+INTB Connected
#define MCP2317_IOCON_SEQOP                                                    \
  0x20 // 0-> Sequential enabled    1-> Sequential Disabled (Byte-Mode)
#define MCP2317_IOCON_DISSLW                                                   \
  0x10 // 0-> SlewRate SDA Enabled  1->SlewRate disabled
#define MCP2317_IOCON_HAEN                                                     \
  0x08 // 0->Disable AdrressPins    1->Enable AdressPins (nur SPI)
#define MCP2317_IOCON_ODR 0x04 // 0->Int Active Driven      1->Int Pin Open
// Drain
#define MCP2317_IOCON_INTPOL 0x02 // 0->Int Active Low         1->Active High
#define MCP2317_IOCON_NC 0x01     // Unimplemented

// Zuordnung IO-Expander Port/Bit zur Joystick-Taste
#define JOYSTICK_TAST_BIT 7
#define JOYSTICK_TAST_PORT 0
// Zuordnung IO-Expander Port/Bit zur Joystick-Multiplexer (wird hier nicht
// benötigt)
#define JOYSTICK_MUX_BIT 6
#define JOYSTICK_MUX_PORT 0
// Zuordnung IO-Expander Port/Bit zu den beiden LEDS
#define LED_0_BIT 0
#define LED_0_PORT 0
#define LED_1_BIT 1
#define LED_1_PORT 0

/**
 * Write a byte to MCP23017 register
 * @param reg_addr: Register address (0x00-0x15)
 * @param value: Byte value to write
 */
void mcp23017_write_reg(uint8_t reg_addr, uint8_t value) {
  // I2C Write Protocol:
  // START -> [Device Addr + Write] -> ACK -> [Reg Addr] -> ACK -> [Value] ->
  // ACK
  // -> STOP

  i2c_write_byte(1, 0,
                 (MCP2317_ADDR << 1) | 0); // START, Device address + Write
  i2c_write_byte(0, 0, reg_addr);          // Register address
  i2c_write_byte(0, 1, value);             // Data value + STOP
}

/**
 * Read a byte from MCP23017 register
 * @param reg_addr: Register address (0x00-0x15)
 * @return: Byte value read from register
 */
uint8_t mcp23017_read_reg(uint8_t reg_addr) {
  // I2C Read Protocol:
  // START -> [Device Addr + Write] -> ACK -> [Reg Addr] -> ACK ->
  // RESTART -> [Device Addr + Read] -> ACK -> [Data] -> NACK -> STOP

  i2c_write_byte(1, 0,
                 (MCP2317_ADDR << 1) | 0); // START, Device address + Write
  i2c_write_byte(0, 0, reg_addr);          // Register address (no STOP)

  // Restart and read
  i2c_write_byte(1, 0,
                 (MCP2317_ADDR << 1) | 1); // RESTART, Device address + Read
  uint8_t data = i2c_read_byte(1, 1);      // Read with NACK + STOP

  return data;
}

void io_init(void) {
  i2c_init();
  i2c_data.mcp_init_complete = 0;
  i2c_data.scan_complete = 0;

  for (int i = 0; i < 10; i++) {
    clear_SCL();
    clear_SDA();
    systick_wait_ms(50);
    set_SCL();
    set_SDA();
    systick_wait_ms(50);
  }

  // GPIO Portexander initialisieren
  // Configure I/O Direction Register A (0x00)
  // Bit 0 = 0 (GPA0 = OUTPUT for LED 0)
  // All other bits = 0 (all outputs for testing)
  mcp23017_write_reg(MCP2317_0_IODIRA, 0x00);

  // Test: Turn LED ON to verify communication
  // Write 0xFF to OLATA to turn all outputs high
  mcp23017_write_reg(MCP2317_0_OLATA, 0xFF);

  (void)term_string("OLATA=0xFF, Arb: ", ASYNCSYNC_BLOCK);
  (void)term_unsigned(i2c_data.i2c[NXT_I2C_PORT].arbitration_lost, 1,
                      ASYNCSYNC_BLOCK);
  (void)term_string("\r\n", ASYNCSYNC_BLOCK);

  // Read back to verify communication - stored in i2c_data for T32 debugging
  i2c_data.mcp_iodira_readback = mcp23017_read_reg(MCP2317_0_IODIRA);
  i2c_data.mcp_olata_readback = mcp23017_read_reg(MCP2317_0_OLATA);

  i2c_data.mcp_init_complete = 1;

  scope_init(SCOPE_SINGLE, 250);

  // Zum Anschauen der I2C-Leitungen, entweder
  //- nachfolgende Funktion mit jedem Sendevorgang starten
  //   so dass nur eine I2C Zyklus aufgenommen wird
  //   scope_init(SCOPE_SINGLE,250 /* Zeit in us */);
  //- nachfolgende einmalig starten
  //   so dass die Messwertaufnahme dauerhaft läuft
  //   scope_init(SCOPE_CONTINUE,250 /* Zeit in us */);
  // Darstellung v.draw %e scope.buf0 scope.buf1
}

// Funktion wird alle 128ms aufgerufen
//  da jedes I2C-Bit 2ms lang ist und ein I2C-Frame mit 3*9Bits=27*2ms=58ms
//  dauert wenn mehr als ein I2C-Frame hier gesendet/empfangen werden soll pro
//  Aufruf von io_update() nur ein I2C-Frame senden!
void io_update(void) {
  static uint8_t blink_counter = 0;
  static uint8_t led0_state = 0;

  // Blink LED 0 at ~1Hz
  // Called every 128ms, so 128ms × 4 = 512ms ≈ 500ms toggle period = 1Hz
  blink_counter++;
  if (blink_counter >= 4) {
    blink_counter = 0;
    led0_state ^= 1; // Toggle LED state
  }

  // Write LED state to output latch (bit 0 = GPA0)
  mcp23017_write_reg(MCP2317_0_OLATA, led0_state);
}

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*****************************************************************************/

void task_32ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_64ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_128ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
  io_update();
}

void task_256ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_512ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_1024ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_idle(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: IDLE_MS
}

/*****************************************************************************/
/*    Pre-Main-Funktion                                                      */
/*****************************************************************************/
// Zur Vermeidung von malloc(1024) bei der ersten Ausgabe über stdout!
//__attribute__(constructor) stellt sicher, dass premain_init() direkt nach
//__sinit() aufgerufen wird (beides innerhalb von __libc_init_array())
void __attribute__((constructor)) premain_init(void) {
#if 0
  //No linebuffering, call stdio_write() immediately
  //-> Langsam, da mit jedem Zeichen __sflush_r()/_write()/stdio_write() aufgerufen wird
  setvbuf(stdout, NULL, _IONBF, 0);
#else
  static char linebuf[10];
  // LineBuffering into global Varialbe (guter Kompromiss)
  setvbuf(stdout, linebuf, _IOLBF, sizeof(linebuf)); // Linebuffering in
#endif
}

/*****************************************************************************/
/*    Main-Funktion                                                          */
/*****************************************************************************/
#if 0
//Variante 1: Deklaration der main() Funktion
//da es keine CLI gibt, über welcher die Anwendung getartet wird
//sondern der start über startup.s erfolgt, macht dies kein Sinn
//und belegt unnötige Speicherplatz auf den Stack
int main(int argc, char *argv[]) {
  (void) argc;
  (void) argv;
#else
// Variante 2: Deklaration der main() funktion
int main(void) {
#endif
  /* Interrupts zu diesem Zeitpunkt disabled !!!! */

  /* 'Pflicht' Initialisierung, können nicht ausgelassen werden */
  aic_init();          // Interrupt-Controller initialisieren
  systick_init();      // System-Timer initialisieren
  interrupts_enable(); // Ohne Worte
  nxt_avr_init(8);
#if defined(MODE_RAM) || defined(MODE_SIM)
  udmon3_init(); // Speicherzugriff durch den Debugger
  // zur Programmlaufzeit mittels der
  // DCC Schnittstelle ermöglichen
#endif

  /* 'Wahl' Initialisierung, hängt von den benötigten Komponenten ab */
  term_init();
  display_init();

  io_init();

  display_clear(0);
  display_update();

  // ANSI Escape sequences - VT100 / VT52 (see main.h)
  (void)term_string("hallo\n\r", ASYNCSYNC_BLOCK);
  (void)term_string(
      "\033[2J" VT100_CURSORHOME // Move Cursor to home position (0,0)
          VT100_DEFAULT,
      ASYNCSYNC_BLOCK);
  (void)term_string("Prog: " APP_NAME "\n\rVersion von: " __DATE__ " " __TIME__
                    "\n\r",
                    ASYNCSYNC_BLOCK);
  (void)term_string(
      "'Var.Watch %e %spotlight i2c_data' zur Variablendarstellung\n\r",
      ASYNCSYNC_BLOCK);
  (void)term_string(
      "'v.draw %e scope.buf0 scope.buf1' zur Oszilloskopdarstellung\n\r",
      ASYNCSYNC_BLOCK);
  (void)term_string("Viel Erfolg!\n\r", ASYNCSYNC_BLOCK);

  // Alternativ zu term_xxx() kann auch printf() oder noch besser iprintf
  // genutzt werden beide bedingen jedoch einen großen Speicherbedarf!
  // Alternativ zu term_read() kann auch scanf() genutzt werden. Auch diese
  // Funktion
  //  bedingt einen großen Speicherbedarf

#ifndef MODE_ROM
  /* Watchdog Disable */
  /* Mode-Register kann nur einmal beschrieben werden */
  AT91C_BASE_WDTC->WDTC_WDMR = 0xFFF | AT91C_WDTC_WDDIS | /*WD Disable */
                               AT91C_WDTC_WDDBGHLT |      /*Debug Halt */
                               AT91C_WDTC_WDIDLEHLT;      /*Idle Halt  */
#else
#if 0
  /* Watchdog Enable */
  /* Da in dieser Version kein zyklischer Reset des Watchdogs */
  /* vorhanden ist, wird von einem Watchdog Enable abgesehen  */
  /* Mit Reset wird der Wachdog aktiviert!                    */
#else
/* Watchdog Disable */
/* Mode-Register kann nur einmal beschrieben werden */
AT91C_BASE_WDTC->WDTC_WDMR = 0xFFF | AT91C_WDTC_WDDIS | /*WD Disable */
                             AT91C_WDTC_WDDBGHLT |      /*Debug Halt */
                             AT91C_WDTC_WDIDLEHLT;      /*Idle Halt  */
#endif
#endif
  // Vorangegangenen Stackaufbau 'löschen'
  stack_fill();
  // Label, so das mit 'go start' hierin gesprungen werden kann
start:
  __attribute__((unused));
  uint32_t start_tick = systick_get_ms();
  uint32_t zeitscheibe = 0;
  char *task_aktiv = "";
  while (1) {
    // Warten bis zum nächsten TimeSlot
    while ((int)(start_tick - systick_get_ms()) > 0)
      ;
    start_tick += ZYKLUS_MS;
    // Label, so das mit 'go zyklus' hierhin gesprungen werden kann
  zyklus:
    __attribute__((unused)) if ((zeitscheibe & 0b000000001) == 0b000000001) {
      task_aktiv = "32ms";
      task_32ms();
    }
    else if ((zeitscheibe & 0b000000011) == 0b000000010) {
      task_aktiv = "64ms";
      task_64ms();
    }
    else if ((zeitscheibe & 0b000000111) == 0b000000100) {
      task_aktiv = "128ms";
      task_128ms();
    }
    else if ((zeitscheibe & 0b000001111) == 0b000001000) {
      task_aktiv = "256ms";
      task_256ms();
    }
    else if ((zeitscheibe & 0b000011111) == 0b000010000) {
      task_aktiv = "512ms";
      task_512ms();
    }
    // Zeit für IDLE-Task verfügbar
    if ((int)(start_tick - systick_get_ms()) >= IDLE_MS) {
      task_aktiv = "Idle";
      task_idle();
    }
    // Max. Zeitdauer einer Zeitscheibe überschritten?
#if 0
    if ((int) (start_tick - systick_get_ms()) <= 0) {
      main_data.term_status |= term_string(VT100_VORDERGRUND_ROT
                                           "Timing durch '", ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string(task_aktiv, ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string("' verletzt\n\r"
                                           VT100_VORDERGRUND_DEFAULT, ASYNCSYNC_NONBLOCK);
    }
#endif
    // Zeitscheibe erhöhen
    zeitscheibe++;

    // Stack Testen
    if (stack_check() < (1 * 4)) {
      static uint8_t stack_cnt = 0;
      if (stack_cnt == 0) {
        stack_cnt = 1;
        main_data.term_status |= term_string(
            VT100_VORDERGRUND_ROT "Stack overflow durch '", ASYNCSYNC_NONBLOCK);
        main_data.term_status |= term_string(task_aktiv, ASYNCSYNC_NONBLOCK);
        main_data.term_status |=
            term_string("'\n\r" VT100_VORDERGRUND_DEFAULT, ASYNCSYNC_NONBLOCK);
        while (1)
          ;
      }
    }

    // Batteriespannung überprüfen
    // Ggf. schlägt die Unterspannungsprüfung im Akku zuvor ein!
    if (nxt_avr_get_battery_raw() < ((2 * 3000 /*mV*/) << 10) / 14180) {
      if (main_data.lowbat_cnt++ > 100)
        while (1)
          ;
      else if (main_data.lowbat_cnt == 10)
        main_data.term_status |=
            term_string(VT100_VORDERGRUND_ROT
                        "\n\rLow Battery\n\r" VT100_VORDERGRUND_DEFAULT,
                        ASYNCSYNC_NONBLOCK);
    } else {
      main_data.lowbat_cnt = 0;
    }

    // Termstatus ueberpruefen
    if (main_data.term_cnt == 0 && main_data.term_status != 0) {
      main_data.term_cnt++;
    overflow:
      __attribute__((unused));

      (void)term_string(VT100_VORDERGRUND_ROT
                        "\n\rTerminal Overflow\n\r" VT100_VORDERGRUND_DEFAULT,
                        ASYNCSYNC_BLOCK);
      while (1)
        ;
    }
  }
  // Nachfolgende Funktion wird leider nie aufgerufen!
  nxt_avr_power_down();
  systick_wait_ms(1000);
  while (1)
    ;
}
