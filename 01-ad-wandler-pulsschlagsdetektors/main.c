#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h> //fuer _exit() usleep()

// Ausführungsumgebung 'automatisch' erkennen
#if defined(MODE_RAM) || defined(MODE_ROM) || defined(MODE_SIM)
#define NXT
#else
#define COMPILER_EXPLORER
#endif

#ifdef NXT
#include "AT91SAM7S64.h"
#include "lib/aic.h"
#include "lib/display.h"
#include "lib/nxt_avr.h"
#include "lib/systick.h"
#include "lib/term.h"
#include "main.h"
// #include "lib/adc.h"
#if defined(MODE_RAM) || defined(MODE_SIM)
#include "trace32/udmon3.h"
#endif

#define ZYKLUS_MS 4
#define IDLE_MS 2
#define HERZSCHLAG_PORT SENSOR_2 // SENSOR_1...4
#if IDLE_MS >= ZYKLUS_MS
#error "Idle_ms muss kleiner als zyklus_ms sein"
#endif
#endif
#ifdef COMPILER_EXPLORER
uint32_t systick_get_ms(void);
#endif

/*****************************************************************************/
/*   Globale Variablen                                                       */
/*****************************************************************************/
#ifdef NXT
struct {
  signed char term_status; // 0->false->Alles Bestens   -1->true->Overflow
  unsigned char term_cnt;
  unsigned char lowbat_cnt;
} main_data = {
    .term_status = 0,
    .term_cnt = 0,
    .lowbat_cnt = 0,
};
#endif

/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*   Standard-C-Library (weitere befinden sich in newlib_syscalls.c)         */
/*****************************************************************************/
// Routine wird von C-Lib aufgerufen (bspw. printf() abort())
#ifdef NXT
void _exit(int status) {
  (void)status;
  // LED-Blinken lassen
  // Breakpoint setzen
  while (1)
    ;
}
#endif
/************************************************************************/
/*   Hilfsroutine zur Darstellung eines analogen Verlaufes entsprechend */
/*   einem Oszillosop                                                   */
/************************************************************************/
#ifdef NXT
#define TRACE_SIZE 200
int16_t trace_buf0[TRACE_SIZE];
int16_t trace_buf1[TRACE_SIZE];
// Darstellung des Puffers über 'v.draw %e trace_buf0 trace_buf1'
void trace_scope(int channel, int16_t value) {
  // #define TRACE_ROLL
#ifdef TRACE_ROLL
  if (channel == 0) {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf0[lauf] = trace_buf0[lauf + 1];
    trace_buf0[TRACE_SIZE - 1] = value;
  } else {
    for (int lauf = 0; lauf < (TRACE_SIZE - 1); lauf++)
      trace_buf1[lauf] = trace_buf1[lauf + 1];
    trace_buf1[TRACE_SIZE - 1] = value;
  }
#else
  static int trace_index = 0;
  if (trace_index < TRACE_SIZE) {
    if (channel == 0)
      trace_buf0[trace_index] = value;
    else
      trace_buf1[trace_index] = value;
  } else {
    // Pause, damit der Debugger in 'Ruhe' den gesamten Speicher auslesen kann
  }
  if (channel == 1) {
    trace_index++;
    trace_index = trace_index >= (3 * TRACE_SIZE) ? 0 : trace_index;
  }
#endif
}
#endif
#ifdef COMPILER_EXPLORER
void trace_scope(int mode, uint16_t value) {
  static int pos0 = 0;
  static int pos1 = 0;
  int val = (value - 450) / 4;
  val = val > 80 ? 80 : (val < 0 ? 0 : val);
  if (mode == 0) {
    pos0 = val;
  } else {
    pos1 = val;
    if (pos1 == pos0) {
      printf("%*c\n", pos0, '+');
    } else if (pos1 > pos0) {
      printf("%*c%*c\n", pos0, '*', pos1 - pos0, '-');
    } else {
      printf("%*c%*c\n", pos1, '-', pos0 - pos1, '*');
    }
  }
}
#endif
#if defined MODE_SIM || defined COMPILER_EXPLORER
// Nachfolgendes Const-Array enthält Rohdaten und 'ersetzt' den AD-Wanderl
uint16_t simuli_idx = 0;
const uint16_t simuli[] = {
    595, 596, 594, 586, 578, 568, 555, 545, 532, 524, 516, 507, 499, 493, 487,
    481, 476, 473, 471, 471, 473, 481, 487, 503, 513, 517, 518, 516, 515, 511,
    507, 502, 497, 492, 489, 484, 479, 476, 473, 470, 468, 467, 467, 466, 467,
    466, 465, 465, 465, 465, 465, 466, 466, 446, 440, 439, 440, 442, 447, 452,
    455, 460, 463, 467, 470, 471, 472, 474, 474, 474, 473, 472, 471, 469, 468,
    468, 467, 466, 465, 465, 465, 464, 465, 466, 466, 488, 495, 497, 499, 500,
    505, 510, 518, 531, 543, 560, 572, 582, 591, 596, 598, 595, 591, 582, 572,
    563, 550, 540, 527, 518, 512, 503, 497, 490, 484, 476, 469, 448, 436, 435,
    437, 441, 447, 455, 462, 469, 475, 480, 485, 485, 485, 484, 481, 479, 476,
    471, 467, 465, 461, 459, 457, 455, 455, 453, 454, 455, 456, 457, 461, 467,
    468, 486, 490, 491, 489, 486, 483, 478, 474, 471, 466, 464, 462, 461, 460,
    459, 459, 460, 461, 462, 463, 464, 465, 467, 467, 467, 467, 467, 467, 467,
    462, 445, 440, 439, 440, 443, 446, 449, 456, 462, 471, 480, 490, 508, 524,
    543, 558, 577, 586, 593, 597, 597, 592, 585, 575, 562, 552, 539, 530, 521,
    511, 506, 501, 514, 515, 512, 507, 499, 492, 486, 480, 477, 475, 475, 477,
    480, 482, 483, 484, 486, 485, 484, 483, 480, 478, 474, 471, 469, 467, 464,
    461, 462, 462, 462, 463, 441, 436, 436, 439, 443, 447, 451, 455, 459, 461,
    464, 465, 465, 467, 467, 466, 466, 465, 465, 466, 466, 466, 466, 466, 467,
    467, 466, 467, 467, 466, 466, 467, 473, 477, 488, 493, 493, 492, 489, 486,
    483, 481, 481, 480, 482, 486, 491, 501, 513, 525, 544, 556, 575, 587, 597,
    605, 608, 606, 601, 595, 587, 578, 570, 555, 532, 516, 509, 505, 501, 499,
    496, 493, 492, 490, 490, 490, 493, 494, 498, 501, 504, 507, 508, 508, 506,
    503, 500, 497, 493, 490, 486, 482, 480, 477, 475, 476, 491, 498, 498, 496,
    493, 489, 484, 482, 479, 474, 472, 470, 468, 468, 468, 468, 469, 470, 472,
    472, 472, 473, 474, 473, 475, 474, 475, 475, 475, 475, 474, 469, 451, 443,
    442, 442, 445, 447, 451, 455, 459, 463, 466, 470, 472, 475, 477, 480, 484,
    488, 496, 508, 519, 531, 549, 561, 574, 581, 586, 587, 583, 576, 567, 558,
    552, 544, 549, 544, 536, 529, 520, 510, 502, 492, 486, 481, 475, 473, 474,
    475, 477, 482, 485, 490, 493, 496, 497, 498, 497, 496, 494, 491, 487, 483,
    480, 470, 451, 443, 438, 438, 440, 443, 446, 451, 456, 461, 464, 469, 472,
    474, 476, 477, 478, 478, 478, 478, 478, 478, 479, 479, 478, 479, 479, 478,
    479, 478, 477, 479, 495, 501, 500, 498, 494, 490, 485, 480, 476, 472, 469,
    466, 463, 463, 462, 462, 463, 465, 466, 471, 476, 483, 494, 506, 525, 540,
    554, 571, 580, 588, 590, 588, 559, 547, 533, 523, 516, 507, 504, 499, 494,
    492, 490, 488, 485, 480, 476, 472, 467, 463, 461, 461, 461, 465, 468, 472,
    474, 478, 479, 480, 480, 479, 477, 475, 478, 478, 488, 489, 487, 485, 480,
    477, 472, 469, 465, 460, 457, 456, 454, 453, 452, 452, 451, 451, 453, 452,
    453, 454, 455, 455, 455, 455, 454, 454, 454, 450, 433, 426, 426, 427, 430,
    433, 438, 442, 447, 450, 453, 456, 459, 461, 461, 462, 462, 463, 462, 462,
    463, 463, 465, 467, 472, 477, 490, 501, 520, 536, 556, 572, 601, 622, 628,
    629, 624, 615, 600, 586, 567, 555, 543, 527, 519, 509, 502, 496, 489, 483,
    475, 470, 466, 463, 462, 463, 463, 465, 470, 472, 474, 476, 477, 474, 457,
    449, 445, 444, 445, 447, 448, 449, 451, 452, 454, 456, 457, 459, 461, 461,
    462, 461, 459, 459, 459, 457, 457, 455, 455, 456, 455, 456, 455, 456, 457,
    460, 464, 471, 482, 486, 488, 486, 484, 482, 478, 474, 470, 467, 465, 461,
    459, 456, 453, 452, 451, 449, 449, 449, 449, 451, 454, 455, 459, 461, 465,
    472, 479, 486, 483, 493, 511, 527, 546, 558, 569, 576, 579, 577, 572, 565,
    555, 546, 534, 527, 519, 511, 502, 497, 493, 487, 480, 475, 471, 467, 465,
    464, 466, 467, 472, 475, 500, 508, 512, 512, 509, 504, 499, 494, 487, 483,
    479, 473, 470, 467, 465, 464, 464, 464, 465, 466, 468, 470, 473, 472, 474,
    475, 474, 474, 474, 472, 471, 467, 444, 439, 436, 436, 438, 441, 445, 449,
    454, 458, 463, 466, 468, 470, 471, 472, 472, 471, 470, 469, 468, 467, 465,
    464, 464, 463, 462, 463, 463, 464, 466, 471, 481, 493, 513, 534, 548, 561,
    576, 587, 595, 598, 598, 594, 587, 577, 567, 553, 543, 532, 521, 514, 510,
    504, 500, 494, 491, 487, 482, 478, 474, 470, 469, 466, 450, 445, 447, 452,
    457, 462, 468, 473, 478, 480, 482, 483, 482, 480, 479, 476, 474, 473, 471,
    470, 468, 466, 466, 466, 466, 467, 466, 467, 467, 469, 469, 470, 491, 496,
    497, 494, 491, 486, 482, 478, 472, 469, 465, 463, 461, 459, 458, 458, 458,
    457, 458, 458, 459, 459, 459, 459, 460, 460, 460, 460, 460, 460, 459, 455,
    440, 433, 434, 440, 445, 455, 472, 487, 504, 527, 545, 566, 579, 588, 595,
    596};
#endif
/*****************************************************************************/
/*   Ihr Programm                                                            */
/*****************************************************************************/
struct {
  uint16_t ad_buf[128];
  int ad_idx;
} herzschlag = {
    .ad_buf = {},
    .ad_idx = 0,
};
int herzschlag_process(int32_t ad) {
  // Hinweis: Im Detektor ist eine automatische Verstärkungsregelung (AGC)
  // verbaut bei starken Änderung fährt dieser seine Verstärkung zurück, so dass
  // im ausgegebenen Signal fast kein Herzschlag zu erkennen ist Mit der Zeit
  // erhöht der Verstärker die Verstärkung, so dass erst dann ein Herzschlag im
  // Signal zu erkennen ist. siehe auch Wikipedia: Automatische
  // Verstärkungsregelung
  herzschlag.ad_buf[herzschlag.ad_idx] = ad;
  trace_scope(0, ad);
  // Beispielhaft ein Marker in der Grafik setzen
  trace_scope(1, (herzschlag.ad_idx & 7) == 0 ? 500 : 0);
  // Systemzeit in ms
  uint32_t time = systick_get_ms();
  (void)time;
  herzschlag.ad_idx = (herzschlag.ad_idx + 1) & 127;

  return 0; //>0 ermittelter Herzschlag =0 kein neuer Herzschlag <0 Fehler
}

#ifdef NXT
/*****************************************************************************/
/*   Hilfsroutinen                                                           */
/*****************************************************************************/
void task_8ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
#ifdef MODE_SIM
  (void)herzschlag_process(simuli[simuli_idx++]);
  simuli_idx = simuli_idx % (sizeof(simuli) / sizeof(simuli[0]));
#else
  (void)herzschlag_process(nxt_avr_get_sensor_adc_raw(HERZSCHLAG_PORT));
#endif
}

void task_16ms(void) {
  // Keine blockierende Aufrufe
  // Max. Bearbeitungsdauer: ZYKLUS_MS
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

  // Beispielanwendung für MotorPos,Sensor,Batterie und NXT-Tasten
  // Im Simulationsmodus kann im 'Per'-Fenster Werte vorgegeben werden
  // Beispiel zur Abfrage des anlogen Eingangs am SensorPort 1
  display_string("S0:");
  display_hex((uint32_t)nxt_avr_get_sensor_adc_raw(SENSOR_1),
              3); // PER: NXT_AVR_ADC_1
  // Beispiel zur Abfrage der Batteriespannung
  display_goto_xy(0, 3);
  display_string("Bat:");
  display_unsigned(nxt_avr_get_battery_mv(),
                   5); // über AVR-Proz -> PER: NXT_AVR_BATTERY
  //	display_unsigned(adc_get_usb_mv()         ,5);  //über SAM7-Proz
  display_string("mV");
  // Beispiel zur Abfrage der NXT-Tasten
  display_goto_xy(0, 4);
  display_string("But: ");
  display_char(nxt_avr_get_buttons().orange ? 'O' : '-');
  display_char(nxt_avr_get_buttons().left ? 'L'
                                          : '-'); // Left/Right/Grey generien
  display_char(nxt_avr_get_buttons().right ? 'R'
                                           : '-'); // je eine eigenen Spannung
  display_char(nxt_avr_get_buttons().grey
                   ? 'G'
                   : '-'); // so dass diese 3 nicht gemeinsam
                           // erkannt werden können
  // Beispiel für Zeitdarstellung
  display_goto_xy(0, 5);
  display_string("Time: ");
  display_unsigned(systick_get_ms() / 1000, 4);
  display_string("s");
  display_update();
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

  // Beispielanwendung für Terminal-Schnittstelle
  unsigned char c;
  static unsigned char string[100];
  static uint8_t strpos = 0;

  if (term_read(&c, ASYNCSYNC_NONBLOCK) == 0) {
    main_data.term_status |=
        term_char(c, ASYNCSYNC_NONBLOCK); // Das empfangene Zeichen als Echo an
                                          // das Terminal zurückschicken
    string[strpos] = c;
    string[++strpos] = 0;
    strpos = strpos >= (sizeof(string) - 1) ? (sizeof(string) - 2) : strpos;

    if (c == '\r') {
      string[--strpos] = 0;
      main_data.term_status |= term_string("\n\r==>ENTER gedr\xFC"
                                           "ckt: '",
                                           ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string((char *)string, ASYNCSYNC_NONBLOCK);
      main_data.term_status |= term_string("'\n\r", ASYNCSYNC_NONBLOCK);
      // oberen 128 ASCII zeichen des Terminal-Fensters sind wie folgt codiert
      // https://en.wikipedia.org/wiki/VT100_encoding

      strpos = 0;
      string[strpos] = 0;
    }
  }
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
  display_clear(0);
  display_string(APP_NAME " : " __TIME__);
  display_update();

  // ANSI Escape sequences - VT100 / VT52 (see main.h)
  (void)term_string(
      "\033[2J" VT100_CURSORHOME // Move Cursor to home position (0,0)
          VT100_DEFAULT,
      ASYNCSYNC_BLOCK);
  (void)term_string("Prog: " APP_NAME "\n\rVersion von: " __DATE__ " " __TIME__
                    "\n\r",
                    ASYNCSYNC_BLOCK);
  (void)term_string(
      "'v.draw %e trace_buf0 trace_buf1' zum oeffnen eines 'Oszilloskop'\n\r",
      ASYNCSYNC_BLOCK);
  (void)term_string("'do trace_scope' zum oeffnen eines 'Oszilloskop'\n\r",
                    ASYNCSYNC_BLOCK);
  (void)term_string("NXT Display Fenster am besten schliessen\n\r",
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
      task_aktiv = "8ms";
      task_8ms();
    }
    else if ((zeitscheibe & 0b000000011) == 0b000000010) {
      task_aktiv = "16ms";
      task_16ms();
    }
    else if ((zeitscheibe & 0b000000111) == 0b000000100) {
      task_aktiv = "32ms";
      task_32ms();
    }
    else if ((zeitscheibe & 0b000001111) == 0b000001000) {
      task_aktiv = "64ms";
      task_64ms();
    }
    else if ((zeitscheibe & 0b000011111) == 0b000010000) {
      task_aktiv = "128ms";
      task_128ms();
    }
    else if ((zeitscheibe & 0b000111111) == 0b000100000) {
      task_aktiv = "256ms";
      task_256ms();
    }
    else if ((zeitscheibe & 0b001111111) == 0b001000000) {
      task_aktiv = "512ms";
      task_512ms();
    }
    else if ((zeitscheibe & 0b011111111) == 0b010000000) {
      task_aktiv = "1024ms";
      task_1024ms();
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
#endif
#ifdef COMPILER_EXPLORER
int main(int argc, char *argv[]) {
  while (1) {
    int ret = herzschlag_process(
        simuli[simuli_idx % (sizeof(simuli) / sizeof(simuli[0]))]);
    simuli_idx++;
    if (simuli_idx > 2 * (sizeof(simuli) / sizeof(simuli[0])))
      break;
    if (ret < 0)
      break;
  }
}
uint32_t systick_get_ms(void) { return simuli_idx * 8; }
#endif
