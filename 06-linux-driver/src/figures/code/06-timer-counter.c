#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

/* Normalerweise: #include <linux/timer-counter.h> */
#include "060-timer-counter-events.h"
#include "061-timer-counter-config.h"
#include "062-timer-counter-request.h"

#define TC_DEVICE "/dev/tc0"
#define PWM_CHANNEL 0

/* PWM-Konfiguration: 1kHz Frequenz, 50% Tastverhältnis
 * Annahme: MCK = 48MHz, MCK/32 = 1.5MHz
 * Periode für 1kHz = 1500 Ticks
 * 50% Duty = 750 Ticks
 */
#define PWM_PERIOD 1500
#define PWM_DUTY   750

int main() {
  int fd;
  struct tc_request req;

  /* Timer-Counter Gerät öffnen */
  if ((fd = open(TC_DEVICE, O_RDWR)) < 0) {
    perror("Fehler beim Öffnen des TC-Geräts");
    return -1;
  }

  /* Request-Struktur initialisieren */
  memset(&req, 0, sizeof(req));
  strncpy(req.consumer, "pwm-beispiel", TC_MAX_NAME_SIZE - 1);
  req.num_channels = 1;
  req.event_buffer_size = 16;

  /* Kanal 0 für PWM-Modus konfigurieren */
  req.channels[0].channel = PWM_CHANNEL;
  req.channels[0].mode = TIMER_COUNTER_PWM;
  req.channels[0].clock_source = TC_CLOCK_MCK_DIV32;
  req.channels[0].enable_events = 0; /* Keine Events für PWM nötig */

  /* PWM-spezifische Konfiguration */
  req.channels[0].pwm.period_ticks = PWM_PERIOD;
  req.channels[0].pwm.duty_ticks = PWM_DUTY;
  req.channels[0].pwm.output_pin = TC_PIN_TIOA;
  req.channels[0].pwm.invert = 0;

  /* Kanal über ioctl konfigurieren */
  if (ioctl(fd, TC_GET_CHANNEL_IOCTL, &req) < 0) {
    perror("TC_GET_CHANNEL_IOCTL fehlgeschlagen");
    close(fd);
    return -1;
  }

  printf("PWM konfiguriert auf Kanal %d\n", PWM_CHANNEL);
  printf("Periode: %d Ticks, Duty: %d Ticks (%.1f%%)\n",
         PWM_PERIOD, PWM_DUTY,
         (float)PWM_DUTY / PWM_PERIOD * 100.0);

  /* Timer starten */
  if (ioctl(req.fd, TC_START_IOCTL) < 0) {
    perror("TC_START_IOCTL fehlgeschlagen");
    close(req.fd);
    close(fd);
    return -1;
  }

  printf("PWM-Ausgabe aktiv auf TIOA.\n");

  /* PWM läuft autonom, Hauptthread kann andere Arbeit erledigen */
  sleep(10);

  /* Aufräumen */
  ioctl(req.fd, TC_STOP_IOCTL);
  close(req.fd);
  close(fd);

  return 0;
}
