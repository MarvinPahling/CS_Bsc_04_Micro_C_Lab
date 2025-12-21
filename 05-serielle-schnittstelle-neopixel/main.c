#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> //fuer _exit()

#include "AT91SAM7S64.h"
#include "lib/aic.h"
#include "lib/display.h"
#include "lib/nxt_avr.h" //fuer nxt_avr_get_sensor_adc_raw()
#include "lib/systick.h"
#include "lib/term.h"
#include "main.h"

// #include "lib/adc.h"
#if defined(MODE_RAM) || defined(MODE_SIM)
#include "trace32/udmon3.h"
#endif

#define ZYKLUS_MS 2
#define IDLE_MS 1

#if IDLE_MS >= ZYKLUS_MS
#error "Idle_ms muss kleiner als zyklus_ms sein"
#endif

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

/************************************************************************/
/*   Hilfsroutine zur Darstellung eines analogen Verlaufes entsprechend */
/*   einem Oszillosop                                                   */
/************************************************************************/
#define TRACE_SIZE 200
int16_t trace_buf0[TRACE_SIZE];
int16_t trace_buf1[TRACE_SIZE];
int16_t trace_buf2[TRACE_SIZE];
// Darstellung des Puffers über 'v.draw %e trace_buf0 trace_buf1'
void trace_scope(int channel, int16_t value) {
#define TRACE_ROLL
#ifdef TRACE_ROLL
  if (channel == 0) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf0[lauf] = trace_buf0[lauf + 1];
    trace_buf0[TRACE_SIZE - 1] = value;
  } else if (channel == 1) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf1[lauf] = trace_buf1[lauf + 1];
    trace_buf1[TRACE_SIZE - 1] = value;
  } else {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf2[lauf] = trace_buf2[lauf + 1];
    trace_buf2[TRACE_SIZE - 1] = value;
  }
#else
  static int trace_index = 0;
  if (trace_index < TRACE_SIZE) {
    if (channel == 0)
      trace_buf0[trace_index] = value;
    else if (channel == 1)
      trace_buf1[trace_index] = value;
    else
      trace_buf2[trace_index] = value;
  } else {
    // Pause, damit der Debugger in 'Ruhe' den gesamten Speicher auslesen kann
  }
  if (channel == 1) {
    trace_index++;
    trace_index = trace_index >= (3 * TRACE_SIZE) ? 0 : trace_index;
  }
#endif
}

/*****************************************************************************/
/*   Ihr Programm                                                            */
/*****************************************************************************/
// Datenstruktur
//            +---------+---------+---------+
//            | G7...G0 | R7...R0 | B7...B0 |
//            +---------+---------+---------+
// Union
//  Integer
//  +---------+---------+---------+---------+
//  |31     24 23     16 15      8 7       0|
//  +---------+---------+---------+---------+
//  Struct
//  +---------+
//  | Dummy   |
//  +---------+
//            +---------+
//            | Green   |
//            +---------+
//                          ...............
typedef union {
  struct {
    uint8_t dummy;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
  };
  uint32_t rgb;
} ws2812_composition_t;

#define LEDRING_LEDS 8

#define LEDRING_PORT SENSOR_4
#define LEDRING_TXD_PORT AT91C_PIO_PA6
#define LEDRING_TASTER_PORT AT91C_PIO_PA2
#define LEDRING_RS485_ENABLE AT91C_PIO_PA7
#define SEND_PDC

// Farbdefinitionen
#define COLOR_BLACK                                                            \
  (ws2812_composition_t){.red = 0x00, .green = 0x00, .blue = 0x00}
#define COLOR_WHITE10                                                          \
  (ws2812_composition_t){.red = 0x08, .green = 0x08, .blue = 0x08}
#define COLOR_WHITE25                                                          \
  (ws2812_composition_t){.red = 0x10, .green = 0x10, .blue = 0x10}
#define COLOR_WHITE50                                                          \
  (ws2812_composition_t){.red = 0x20, .green = 0x20, .blue = 0x20}
#define COLOR_WHITE75                                                          \
  (ws2812_composition_t){.red = 0x40, .green = 0x40, .blue = 0x40}
#define COLOR_WHITE                                                            \
  (ws2812_composition_t){.red = 0xff, .green = 0xff, .blue = 0xff}
#define COLOR_RED                                                              \
  (ws2812_composition_t){.red = 0xff, .green = 0x00, .blue = 0x00}
#define COLOR_ROSE                                                             \
  (ws2812_composition_t){.red = 0xff, .green = 0x00, .blue = 0x7f}
#define COLOR_MAGENTA                                                          \
  (ws2812_composition_t){.red = 0xff, .green = 0x00, .blue = 0xff}
#define COLOR_VIOLET                                                           \
  (ws2812_composition_t){.red = 0x7f, .green = 0x00, .blue = 0xff}
#define COLOR_BLUE                                                             \
  (ws2812_composition_t){.red = 0x00, .green = 0x00, .blue = 0xff}
#define COLOR_AZURE                                                            \
  (ws2812_composition_t){.red = 0x00, .green = 0x7f, .blue = 0xff}
#define COLOR_CYAN                                                             \
  (ws2812_composition_t){.red = 0x00, .green = 0xff, .blue = 0xff}
#define COLOR_AUQUAMARINE                                                      \
  (ws2812_composition_t){.red = 0x00, .green = 0xff, .blue = 0x7f}
#define COLOR_GREEN                                                            \
  (ws2812_composition_t){.red = 0x00, .green = 0xff, .blue = 0x00}
#define COLOR_CHARTREUSE                                                       \
  (ws2812_composition_t){.red = 0x7f, .green = 0xff, .blue = 0x00}
#define COLOR_YELLOW                                                           \
  (ws2812_composition_t){.red = 0xff, .green = 0xff, .blue = 0x00}
#define COLOR_ORANGE                                                           \
  (ws2812_composition_t){.red = 0xff, .green = 0x7f, .blue = 0x00}

struct {
  enum {
    MODE_LAUFLICHT,
    MODE_LAUFLICHTSCHATTEN,
    MODE_FARBVERLAUF,
    MODE_MAX
  } mode;
  int speed_max;
  int farbverlauf_idx;
  ws2812_composition_t leds[LEDRING_LEDS];
  const ws2812_composition_t farbverlauf[12];
} ledring_data = {
    .mode = MODE_LAUFLICHT,
    .farbverlauf = {COLOR_RED, COLOR_ROSE, COLOR_MAGENTA, COLOR_VIOLET,
                    COLOR_BLUE, COLOR_AZURE, COLOR_CYAN, COLOR_AUQUAMARINE,
                    COLOR_GREEN, COLOR_CHARTREUSE, COLOR_YELLOW, COLOR_ORANGE},
};
// Anzeigen über v.view %e %spotlight ledring_data

// Buffer for 8 LEDs, 24 bits per LED, 1 byte per bit
//  8 * 24 * 1 = 192 bytes
#define NEOPIXEL_BUFFER_SIZE (LEDRING_LEDS * 24)
uint8_t ws2812_buffer[NEOPIXEL_BUFFER_SIZE];

// Inverted RS485 logic: Start(L->H), Stop(H->L)
// 0: H(375ns) L(875ns) -> CPU 0xFC (00000011 inverted -> 11111100 -> Start(H)
// 11111100 Stop(L)) Correct logic check: Wire: H (Start) D0..D7 L (Stop) We
// need H H H L L L L L L L for '0' CPU:  0 0 0 1 1 1 1 1 1 1? No. CPU Tx:
// Start(0), D0, D1... D7, Stop(1) Inverted Wire: H, ~D0, ~D1... ~D7, L '0'
// Target: H, H, H, L, L, L, L, L, L, L So ~D0=H, ~D1=H -> D0=0, D1=0 ~D2..~D7=L
// -> D2..D7=1 Byte: 0xFC (11111100) -> D0 is LSB. D0=0, D1=0, D2=10... Correct.
// Logik fuer die RS485 Invertierung und das WS2812 Timing:
// Das RS485-Modul invertiert das Signal: UART Start(L)->Bus High, Stop(H)->Bus
// Low. 8MHz Baudrate -> 1 Bit = 125ns. 1 Frame (10 Bits) = 1.25us (WS2812
// Periode). '0': 3xHigh, 7xLow -> UART: Start(L) D0(L) D1(L) D2..D7(H) Stop(H)
// -> 0xFC. '1': 6xHigh, 4xLow -> UART: Start(L) D0..D4(L) D5..D7(H) Stop(H) ->
// 0xE0.
#define WS2812_BYTE_0 0xFC
#define WS2812_BYTE_1 0xE0

void ws2812_init(void) {
  // USART Clock Delivery (Stromversorgung)
  AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US0) | (1 << AT91C_ID_PIOA);

  // GPIO Konfiguration
  //  PA6 (TXD0) als Peripherie A konfigurieren (damit der USART den Pin
  //  steuert) PA7 (RS485 Enable) als Ausgang konfigurieren PA30 (Sensor4 Data)
  //  als Eingang konfigurieren (um Buskonflikte zu vermeiden)

  AT91PS_PIO p_pio = AT91C_BASE_PIOA;

  // PIO Kontrolle fuer PA6 abschalten -> Periphere Kontrolle aktivieren
  p_pio->PIO_PDR = LEDRING_TXD_PORT;
  p_pio->PIO_ASR = LEDRING_TXD_PORT; // Waehle Peripherie A (TXD0)

  // Konfiguriere PA7 (RS485 Enable)
  p_pio->PIO_PER = LEDRING_RS485_ENABLE;  // PIO Enable
  p_pio->PIO_OER = LEDRING_RS485_ENABLE;  // Output Enable
  p_pio->PIO_SODR = LEDRING_RS485_ENABLE; // Set Output Data Register (High) ->
                                          // RS485 Senden aktiv

  // Konfiguriere PA30 als Eingang (Sicherheitsmassnahme)
  p_pio->PIO_PER = AT91C_PIO_PA30;
  p_pio->PIO_ODR = AT91C_PIO_PA30; // Output Disable -> Input Mode

  // USART Konfiguration
  AT91PS_USART p_usart = AT91C_BASE_US0;

  // Reset und Disable Receiver/Transmitter (Definierter Zustand)
  p_usart->US_CR =
      AT91C_US_RSTRX | AT91C_US_RSTTX | AT91C_US_RXDIS | AT91C_US_TXDIS;

  // Konfiguriere Mode Register
  // US_MODE_NORMAL: Normaler UART Betrieb
  // US_SYNC: Synchroner Modus (wir generieren den Takt, fuer praezises Timing
  // notwendig) 8N1: 8 Bits, No Parity, 1 Stop Bit
  p_usart->US_MR = AT91C_US_USMODE_NORMAL | AT91C_US_CLKS_CLOCK |
                   AT91C_US_CHRL_8_BITS | AT91C_US_PAR_NONE |
                   AT91C_US_NBSTOP_1_BIT | AT91C_US_SYNC;

  // Baudrate
  // MCK (Systemtakt) = 47923200 Hz
  // Ziel-Baudrate = 8.000.000 Hz (8 MHz)
  // CD (Clock Identifier) = MCK / Baudrate = 47923200 / 8000000 = 5.9904
  // Wir runden auf 6.
  // Effektive Baudrate: 47923200 / 6 = 7.98 MHz (Fehler < 0.2%, sehr gut)
  p_usart->US_BRGR = 6;

  // Transmitter aktivieren
  p_usart->US_CR = AT91C_US_TXEN;
}

// Volle Compileroptimierung einschalten
#pragma GCC push_options
#pragma GCC optimize("-O3")
// Code aus dem RAM laufen lassen, welchen einen schnellere Zugriff
// als das Flash hat, so dass die Funktion mit max. Geschwindigekit
// ausgeführt wird
__attribute__((section(".text.fastcode"))) void ws2812_send(void) {
  // Konvertiere das `ledring_data.leds` Array in den `ws2812_buffer`
  // Wir muessen jedes Farb-Bit in ein entsprechendes UART-Byte (0xE0 oder 0xFC)
  // umwandeln.
  uint8_t *p_buf = ws2812_buffer;
  for (int i = 0; i < LEDRING_LEDS; i++) {
    // Farben abholen (WS2812 erwartet GRB Format, nicht RGB!)
    uint32_t color = (ledring_data.leds[i].green << 16) |
                     (ledring_data.leds[i].red << 8) |
                     (ledring_data.leds[i].blue << 0);

    for (int bit = 23; bit >= 0; bit--) {
      if (color & (1 << bit)) {
        *p_buf++ = WS2812_BYTE_1;
      } else {
        *p_buf++ = WS2812_BYTE_0;
      }
    }
  }

#if defined SEND_THRREGISTER
  // Sendevorgang durch direktes Schreiben in das THR-Register (Blocking Mode)

  // Interrupts deaktivieren, damit das Timing nicht durch andere Tasks gestoert
  // wird WS2812 ist timing-kritisch! werden kann
  int i_state = interrupts_get_and_disable();

  AT91PS_USART p_usart = AT91C_BASE_US0;
  for (int i = 0; i < NEOPIXEL_BUFFER_SIZE; i++) {
    // Wait for TXRDY
    while (!(p_usart->US_CSR & AT91C_US_TXRDY))
      ;
    p_usart->US_THR = ws2812_buffer[i];
  }
  // Wait for complete
  while (!(p_usart->US_CSR & AT91C_US_TXEMPTY))
    ;

  if (i_state)
    interrupts_enable();

#elif defined SEND_PDC
  // Sendeovrgang  mittels PDC/DMA Datentransfer
  AT91PS_USART p_usart = AT91C_BASE_US0;

  // Setup PDC
  p_usart->US_TPR = (unsigned int)ws2812_buffer;
  p_usart->US_TCR = NEOPIXEL_BUFFER_SIZE;
  p_usart->US_PTCR = AT91C_PDC_TXTEN; // Enable Tx

  // We don't block here?
  // The task says "Keine blockierende Aufrufe" (No blocking calls) in main
  // loop. But ledring_update calls this. However, for PDC, we shouldn't block.
  // But we need to ensure previous transfer is done?
  // Using simple approach: Assume 16ms is enough for transfer (192 bytes
  // * 1.25us = 240us). So next call will be safe.

#else
#error Unbekannter Mode
#endif
}

// Vorherige CompilerOptions wieder herstellen
#pragma GCC pop_options

void ledring_init(void) {
  // Poti-Initialisieren
  // Da durch AVR Prozessor erfolgt, nichts zu tun

  ws2812_init();
}

void ledring_update(void) {
  // Aufruf alle 16ms
  // Max Bearbeitungsdauer: 8ms

  static uint8_t old_orange_button = 0;
  static uint32_t ms_counter = 0;

  { // Taste detektieren und Mode ändern
    //  Wir nutzen hier den Orangen Taster am NXT Baustein selbst (via nxt_avr
    //  Library). Der externe Taster auf der Adapterplatine wird ignoriert, da
    //  er Konflikte mit dem RS485 Bus verursachen koennte.

    button_t buttons = nxt_avr_get_buttons();
    if (buttons.orange && !old_orange_button) {
      // Rising Edge
      ledring_data.mode++;
      if (ledring_data.mode >= MODE_MAX) {
        ledring_data.mode = MODE_LAUFLICHT;
      }
    }
    old_orange_button = buttons.orange;
  }

  { // Farbe in Abhängigkeit des Modes und der Geschwindigkeit setzen

    // Geschwindigkeitsberechnung aus dem Poti-Wert (0..1023).
    // Hoher Poti-Wert -> Grosse Wartezeit -> Langsame Animation.
    // Formel: (ADC * 10000) / 8 / 1024
    // Beispiel bei ADC=1023: 10230000 / 8 / 1024 = ca 1248ms pro LED-Schritt.
    // Das bedeutet ca. 10 Sekunden fuer eine volle Umdrehung (8 LEDs).
    ledring_data.speed_max =
        (nxt_avr_get_sensor_adc_raw(LEDRING_PORT) * 10000) / 8 / 1024;
    if (ledring_data.speed_max < 10)
      ledring_data.speed_max = 10; // Min Safeguard

    ms_counter += 16;

    static int pos = 0;
    static int color_idx = 0;

    if (ms_counter >= (uint32_t)ledring_data.speed_max) {
      ms_counter = 0;

      pos++;
      if (pos >= LEDRING_LEDS)
        pos = 0;

      color_idx++;
      if (color_idx >= 12)
        color_idx = 0;
    }

    // Logic depending on mode
    switch (ledring_data.mode) {
    case MODE_LAUFLICHT:
      for (int i = 0; i < LEDRING_LEDS; i++) {
        if (i == pos) {
          ledring_data.leds[i] =
              ledring_data.farbverlauf[color_idx]; // Or fixed color? Task says
                                                   // "Farbe frei wählbar"
          // constant red for running light for simplicity, or cycle?
          // "Lauflicht, d.h. es leuchtet nur eine LED (Farbe frei wählbar)."
          ledring_data.leds[i] = COLOR_RED;
        } else {
          ledring_data.leds[i] = COLOR_BLACK;
        }
      }
      break;

    case MODE_LAUFLICHTSCHATTEN:
      for (int i = 0; i < LEDRING_LEDS; i++) {
        ledring_data.leds[i] = COLOR_BLACK;
      }
      // Simple trail
      ledring_data.leds[pos] = COLOR_BLUE;
      int p1 = (pos - 1 + 8) % 8;
      ledring_data.leds[p1] =
          (ws2812_composition_t){.red = 0, .green = 0, .blue = 0x80}; // 50%
      int p2 = (pos - 2 + 8) % 8;
      ledring_data.leds[p2] =
          (ws2812_composition_t){.red = 0, .green = 0, .blue = 0x40}; // 25%
      int p3 = (pos - 3 + 8) % 8;
      ledring_data.leds[p3] =
          (ws2812_composition_t){.red = 0, .green = 0, .blue = 0x10};

      break;

    case MODE_FARBVERLAUF:
      // All LEDs same color, changing over time
      for (int i = 0; i < LEDRING_LEDS; i++) {
        ledring_data.leds[i] = ledring_data.farbverlauf[color_idx];
      }
      break;
    default:
      break;
    }
  }

  {
    // Sendevorgang ausloesen.
    // Dies geschieht in jedem Zyklus (alle 16ms), um sicherzustellen, dass
    // die LEDs ihren Zustand halten und Stoerungen ueberschrieben werden.
    ws2812_send();
  }
}

/*****************************************************************************/
/*   Hilfsroutinen */
/*****************************************************************************/

void task_4ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_8ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_16ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
  ledring_update();
}

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
}

void task_256ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_512ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
}

void task_idle(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: IDLE_MS
}

/*****************************************************************************/
/*    Pre-Main-Funktion */
/*****************************************************************************/
// Zur Vermeidung von malloc(1024) bei der ersten Ausgabe über stdout!
//__attribute__(constructor) stellt sicher, dass premain_init() direkt nach
//__sinit() aufgerufen wird (beides innerhalb von __libc_init_array())
void __attribute__((constructor)) premain_init(void) {
#if 0
    //No linebuffering, call stdio_write() immediately
    //-> Langsam, da mit jedem Zeichen __sflush_r()/_write()/stdio_write() aufgerufen wird 
    setvbuf(stdout,NULL,_IONBF,0);
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
int main(int argc, char *argv[]) 
{
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

  ledring_init();

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
      "'v.view %e schrittmotor_data' zur Variablendarstellung\n\r",
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
      task_aktiv = "4ms";
      task_4ms();
    }
    else if ((zeitscheibe & 0b000000011) == 0b000000010) {
      task_aktiv = "8ms";
      task_8ms();
    }
    else if ((zeitscheibe & 0b000000111) == 0b000000100) {
      task_aktiv = "16ms";
      task_16ms();
    }
    else if ((zeitscheibe & 0b000001111) == 0b000001000) {
      task_aktiv = "32ms";
      task_32ms();
    }
    else if ((zeitscheibe & 0b000011111) == 0b000010000) {
      task_aktiv = "64ms";
      task_64ms();
    }
    else if ((zeitscheibe & 0b000111111) == 0b000100000) {
      task_aktiv = "128ms";
      task_128ms();
    }
    else if ((zeitscheibe & 0b001111111) == 0b001000000) {
      task_aktiv = "256ms";
      task_256ms();
    }
    else if ((zeitscheibe & 0b011111111) == 0b010000000) {
      task_aktiv = "512ms";
      task_512ms();
    }
    // Zeit für IDLE-Task verfügbar
    if ((int)(start_tick - systick_get_ms()) >= IDLE_MS) {
      task_aktiv = "Idle";
      task_idle();
    }
    // Max. Zeitdauer einer Zeitscheibe überschritten?
    if ((int)(start_tick - systick_get_ms()) <= 0) {
      main_data.term_status |= term_string(
          VT100_VORDERGRUND_ROT "Timing durch '", ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string(task_aktiv, ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string(
          "' verletzt\n\r" VT100_VORDERGRUND_DEFAULT, ASYNCSYNC_NONBLOCK);
    }
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