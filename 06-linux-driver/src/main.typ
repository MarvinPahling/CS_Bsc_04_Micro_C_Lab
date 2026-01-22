#import "@preview/finite:0.5.0" as finite
#import finite: automaton
#import "@preview/glossarium:0.5.9": gls, glspl, make-glossary, print-glossary, register-glossary
#show: make-glossary
#import "@preview/circuiteria:0.2.0"
#import "@preview/zebraw:0.6.1": *
// Glossary
#let entry-list = (
  (
    key: "lorme",
    short: "l",
    long: "lorem",
    description: "lorem",
  ),
)

#register-glossary(entry-list)
// Your document body

// Load code snippets reference
#let code = json("./figures/code/index.json")
#let content = yaml("./config/text-content.yaml")

#show raw.where(block: true): zebraw.with(numbering: true)

#let render-snippet(id) = {
  let snippet = code.find(s => s.id == id)
  if snippet == none {
    panic("Snippet not found: " + id)
  }
  show raw: it => it

  zebraw(
    raw(
      read(snippet.path),
      lang: snippet.lang,
      block: true,
    ),
    numbering: true,
    header: [*#snippet.description*],
  )
}
// ============================================
// Main Document - Embedded Lab Documentation
// ============================================



// Document Settings
#set document(
  title: content.document.title,
  author: content.document.authors.map(a => a.name),
  date: auto,
)

// Page Setup
#set page(
  paper: "a4",
  margin: (x: 2.5cm, y: 2.5cm),
  numbering: "1",
  number-align: center,
)

// Text Settings
#set text(
  font: "FreeSerif",
  size: 11pt,
  lang: "de",
)

// Heading Settings
#set heading(numbering: "1.1")

// Paragraph Settings
#set par(justify: true)

// Link styling
#show link: underline

// ============================================
// TITLE PAGE
// ============================================

#align(center)[
  #v(3cm)

  #text(size: 24pt, weight: "bold")[
    #content.title_page.main_title
  ]

  #v(0.5cm)

  #text(size: 18pt)[
    #content.title_page.subtitle
  ]

  #v(2cm)

  #text(size: 14pt)[
    #content.title_page.course
  ]

  #v(1cm)


  #v(1fr)

  // Dynamically generate author information
  #for author in content.document.authors [
    #text(size: 12pt)[
      *#content.title_page.labels.author* #author.name \
      // *#content.title_page.labels.student_id* #author.student_id \
      *#content.title_page.labels.date* #datetime.today().display("[month repr:long] [day], [year]")
    ]
    #v(0.5cm)
  ]


  #v(1fr)
  #text(size: 11pt)[
    #content.title_page.institution.name \
    #content.title_page.institution.department
  ]
]

#pagebreak()

// ============================================
// TABLE OF CONTENTS
// ============================================

#outline(
  title: content.table_of_contents.title,
  indent: auto,
  depth: 3,
)


// TODO - add code snippets
// #render-snippet("example-1")
#pagebreak()
// ============================================
// Main Content
// ============================================

= Systemarchitektur von Linux
== Aufgaben des Linuxkernel
Die wichtigsten Aufgaben des Linux-Kernels sind:

=== Process Management
Das Betriebssystem ist in der Lage, einzelne Prozesse zu starten und zu beenden. Darüber hinaus werden Input und Output sowie die Kommunikation zwischen den verschiedenen Prozessen von dem Kernel übernommen. Da sich in der Regel mehrere Prozesse eine CPU teilen, ist der Kernel auch für das Scheduling der Rechenzeit der geteilten CPU verantwortlich.

=== Memory Management
Der Kernel stellt darüber hinaus eine Abstraktionsebene über den geteilten Memory bereit. Hierbei wird ein virtueller Adressraum für alle Prozesse bereitgestellt, welcher dann von dem Kernel auf den echten Memory abgebildet wird.

=== Filesystem
Unter Linux kann fast alles als Datei behandelt werden. Die Aufgabe des entsprechenden Filesystem-Treibers ist es dann die Abstraktion über `Pfade` und `Knoten` auf die darunterliegende physische Struktur abzubilden.

=== Netzwerk
Netzwerkvorgänge sind selten spezifisch für einen einzelnen Prozess, weswegen hierfür ein eigenes Sub-System benötigt wird. Für den Kontext dieser Diskussion ist es aber nicht nötig uns mit Netzwerken zu beschäftigen.

=== Peripheriezugriff
#figure(
  quote(
    "Device drivers take on a special role in the Linux kernel. They are distinct “black
boxes” that make a particular piece of hardware respond to a well-defined internal
programming interface; they hide completely the details of how the device works.",
  ),
  caption: [Aus: "Linux Device Drivers", @ldd3],
)

Die Adressierung von Peripheriegeräten erfolgt über Gerätetreiber. Die Aufgabe dieser sind es den Kernel in einer Art zu erweitern, welche es erlaubt die Funktionsweise der Peripherie auf eine oder mehrere Dateien innerhalb eines Filesystems abzubilden. Diese Treiber werden dabei entweder statisch in den Kernel gelinkt oder können als Kernel-Modul dynamisch zur Laufzeit dazugeladen werden.

Die physischen Peripheriegeräte erscheinen durch die Abstraktion wie eine oder mehrere logische Dateien. Dies ermöglicht es auf der Applikationsebene mit diesen mit den bekannten Dateioperationen zu interagieren.

Geräte erhalten dabei von ihrem Treiber sowohl eine `Major`- als auch eine `Minor`-Nummer. Die Major-Nummer bezieht sich hierbei auf den jeweiligen Treiber und bildet eine logische Obergruppe über die einzelnen Geräte. Die Minor-Nummer hingegen referenziert ein spezifisches Gerät.

Wichtig hierbei ist, dass dies nicht zwangsläufig heißen muss, dass es sich bei unterschiedlichen Geräten auch um unterschiedliche physische Peripheriegeräte handeln muss. Das von der Minor-Nummer gekennzeichnete Gerät muss in Wirklichkeit nicht einmal ein physisches Peripheriegerät sein, sondern es kann sich dabei auch um eine logische Abstraktion handeln. Ein gutes Beispiel hierfür wäre das `loopback`-Interface.

Peripheriegeräte können dabei grob in `Character Devices`, `Block Devices` oder `Netzwerk Devices` unterteilt werden.

`Character Devices` lesen oder schreiben fortlaufende Folge von Datenbytes.
Für die einfache serielle Übertragung von Daten, z.B. durch `I2C` sind diese somit prädestiniert und werden Hauptgegenstand der folgenden Auseinandersetzungen sein. Die anderen beiden Typen lassen wir hierbei außer Acht.

#pagebreak()
= Aufgabe I - Peripheriezugriff

== Zugriff/Konfiguration

Für gewöhnlich erscheinen Peripheriegeräte unter dem Pfad `/dev` oder `/sys`, diese beiden Subsysteme unterscheiden sich neben ihrer Verortung auch in ihrer grundlegenden Designphilosophie.

=== sysfs
`/sys` ist ein einfacher Weg, um mit Peripherie über eine logische ASCII-kodierte Datei zu interagieren. Hierbei beinhaltet für gewöhnlich jede Datei nur genau eine Information. Dies ermöglicht es auch auf einfache Weise die Topologie eines Gerätes abzubilden, da hierarchische Beziehungen über den Pfad abgebildet werden können. Je nach Peripheriegerät kann der Gerätetreiber für diese logischen Dateien Lese- und oder Schreibzugriff bereitstellen. Die klassischen Interaktionen sind `open()`, `close()`, `read()`, `write()`.

Diese Art Schnittstelle eignet sich dadurch hervorragend für Konfiguration oder Statusabfragen von Geräten. Durch das logische Auftreten als ASCII-kodierte Textdatei lassen sich Informationen einfach über Tools wie z.B. `cat` auslesen, was es einfach macht Skripte zu erstellen.

Die Schwächen dieses Systems sind jedoch, dass sich komplexere, atomare Transaktionen nur schwer abbilden lassen. Wollen wir beispielsweise über einen I2C-Bus erste eine Schreib-, gefolgt von einer Leseoperation ausführen, ohne den Bus zwischenzeitlich wieder freizugeben, ist dies über diesen Ansatz nicht möglich.

=== device
`/dev` ist der klassische UNIX-Weg, um auf die Peripherie zuzugreifen. Das entsprechende Gerät wird hierbei logisch als eine binäre Datei realisiert, was den entsprechenden Overhead eliminiert, den es braucht, um ASCII-kodierte Dateien in Binärformat zu parsen. Der Inhalt der Datei repräsentiert hierbei den eigentlichen Datenfluss für das Gerät.

Eine Besonderheit bei diesem Subsystem ist es, dass durch `ioctl()` @glibc_ioctl_h, zusätzlich zu den bereits bekannten Operationen `open()`, `close()`, `read()`, `write()` @glibc_fcntl_h,
auch komplexere Informationen für die entsprechende Transaktion mitgeliefert werden können.

#pagebreak()
#render-snippet("open")
#render-snippet("ioctl")

Die Vorteile dieses Ansatzes sind, dass die Funktion `ioctl()` es ermöglicht sowohl ein Command-Code, sowie optional einen Zeiger zu entsprechenden Nutzdaten für die Transaktion zu übergeben. Eine entsprechende Schnittstelle kann somit eine komplexe atomare Transaktion in nur einem Funktionsaufruf abbilden.

== I2C
=== Schnittstellen
Die eben beschriebene Transaktion über den I2C-Bus wollen wir an dieser Stelle exemplarisch durchgehen. Die Schnittstellendefinition für I2C findet sich in `linux/i2c.h` @linux_i2c_h sowie `linux/i2c-dev.h` @linux_i2c_dev_h.

#render-snippet("i2c-msg")

Das gezeigte `struct` enthält hierbei alle Informationen für einen I2C-Zugriff. Hierzu zählen die Zieladresse, die Flags, die Länge der Nachricht, sowie ein Zeiger zu der Nachricht selber.

#render-snippet("i2c-rdwr")


Das Kernel-Modul `i2c-dev` ist häufig nicht standardmäßig geladen und muss per `modprobe` nachgeladen werden.
```bash
sudo modprobe i2c-dev
```

Da es sich bei den von dem I2C bereitgestellten Geräten logisch um Dateien handelt, lassen sich diese einfach per `ls` und `grep` finden.

```bash
ls -la /dev | grep i2c
```
```
crw-------   1 root   root    89,     0 Jan 18 19:52 i2c-0
crw-------   1 root   root    89,     1 Jan 18 19:52 i2c-1
crw-------   1 root   root    89,     2 Jan 18 19:52 i2c-2
crw-------   1 root   root    89,     3 Jan 18 19:52 i2c-3
crw-------   1 root   root    89,     4 Jan 18 19:52 i2c-4
crw-------   1 root   root    89,     5 Jan 18 19:52 i2c-5
crw-------   1 root   root    89,     6 Jan 18 19:52 i2c-6
crw-------   1 root   root    89,     7 Jan 18 19:52 i2c-7
crw-------   1 root   root    89,     8 Jan 18 19:52 i2c-8
crw-------   1 root   root    89,     9 Jan 18 19:52 i2c-9
```
Das nachfolgende C-Programm zeigt nun einen Beispielhaften Aufruf der API, um den EPROM des eines Laptopdisplays auszulesen
#pagebreak()
#render-snippet("i2c")

Hier wird Beispielsweise das Register `0x00` an der Adresse `0x50` gelesen.

Der Output sieht dann beispielsweise wie folgt aus

```
Gelesen von Gerät 0x50 (Register 0x00):
0x00 0xff 0xff 0xff 0xff 0xff 0xff 0x00
```

== GPIO

Die Schnittstellendefinition für GPIO findet sich in `linux/gpio.h` @linux_gpio_h. Die API für GPIO unterteilt ein GPIO-Device in `chip` und `line`.
#render-snippet("gpio-req")
Die API für die Request an eine Line des GPIO-Controllers ist in diesem Fall umfangreicher als die vorherige. Für den Kontext dieses Moduls sind jedoch vor allem die Flags interessant, mit welchen sich beispielsweise der Pull-Up-Widerstand oder der Edge-Detector konfigurieren lassen.
#render-snippet("gpio-event")
Die API für GPIO-Events gibt uns später die Möglichkeit, Informationen über Änderungen am Edge-Detector zu erhalten. Aus dem Event können wir beispielsweise den Zeitstempel sowie die Art des Events auslesen.

Das nun folgende C-Programm zeigt ein Beispiel, wie wir einen softwareseitigen Edge-Detector in Linux realisieren können.
#pagebreak()
#render-snippet("gpio")

Das Programm konfiguriert `line`-7 als Input und konfiguriert den Edge-Detector, sodass er steigende und fallende Taktflanken erkennt. Über `read()` (blockierender Aufruf) können wir nun auf ein neues Event warten. Der Vorteil im Vergleich zu der Laboraufgabe ist dabei, dass in diesem Fall nur der Thread des entsprechenden Prozesses blockiert wird, jedoch der Rest des Systems weiterarbeiten kann.

== ADC

Im Gegensatz zu I2C und GPIO, die über `/dev/`-Gerätedateien mit binären `ioctl()`-Aufrufen angesprochen werden, nutzen ADC- und Sensordaten typischerweise das *sysfs*-Interface. Das `hwmon`-Subsystem (Hardware Monitoring) stellt Sensordaten als ASCII-Textdateien unter `/sys/class/hwmon/` bereit.

Verfügbare Sensoren lassen sich einfach mit `grep` finden, da es in den meisten Fällen eine `name`-Datei gibt:

Ein solcher Aufruf könnte dabei z.B. so aussehen
```
$ grep . /sys/class/hwmon/hwmon*/name
/sys/class/hwmon/hwmon0/name:AC
/sys/class/hwmon/hwmon1/name:acpitz
/sys/class/hwmon/hwmon2/name:BAT0
/sys/class/hwmon/hwmon3/name:nvme
/sys/class/hwmon/hwmon4/name:thinkpad
/sys/class/hwmon/hwmon5/name:pch_cannonlake
/sys/class/hwmon/hwmon6/name:coretemp
/sys/class/hwmon/hwmon7/name:iwlwifi_1
/sys/class/hwmon/hwmon8/name:ucsi_source_psy_USBC000:001
/sys/class/hwmon/hwmon9/name:ucsi_source_psy_USBC000:002
```
Da sich in solchen Fällen die API-Spezifikation häufig aus der Struktur ergibt, kann es ratsam sein, sich die entsprechende Schnittstelle mit `tree` einmal genauer anzugucken. Ein Aufruf könnte z.B. so aussehen.


```
$ tree /sys/class/hwmon/hwmon6
/sys/class/hwmon/hwmon6
├── device -> ../../../coretemp.0
├── name
├── power
│   ├── autosuspend_delay_ms
│   ├── control
│   ├── runtime_active_time
│   ├── runtime_status
│   └── runtime_suspended_time
├── subsystem -> ../../../../../class/hwmon
├── temp1_crit
├── temp1_crit_alarm
├── temp1_input
├── temp1_label
├── temp1_max
├── temp2_crit
├── temp2_crit_alarm
├── temp2_input
├── temp2_label
├── temp2_max
├── temp3_crit
├── temp3_crit_alarm
├── temp3_input
├── temp3_label
├── temp3_max
├── temp4_crit
├── temp4_crit_alarm
├── temp4_input
├── temp4_label
├── temp4_max
├── temp5_crit
├── temp5_crit_alarm
├── temp5_input
├── temp5_label
├── temp5_max
└── uevent

4 directories, 32 files
```
Da es sich hierbei logisch um ASCII-kodierte Dateien handelt, können wir diese einfach mit `cat` auslesen.

Ein solcher Aufruf könnte z.B. so aussehen:
```
cat /sys/class/hwmon/hwmon6/temp1_input
61000
```

Ein entsprechendes C-Programm, welches die Temperaturen aller CPU-Kerne ausliest und den Durchschnitt ausgibt, könnte so z.B. wie folgt aussehen.

#pagebreak()
#render-snippet("adc")
Im Gegensatz zu dem I2C- und GPIO-Beispiel wird hier kein `ioctl()` benötigt, sondern nur die Standard-POSIX-Funktionen `open()` aus `<fcntl.h>` und `read()` aus `<unistd.h>`.

#pagebreak()
= Aufgabe II - AT91SAM7-Timer Schnittstellenspezifikation

Die Timer-Counter-Einheit des AT91SAM7 kann auf jedem seiner 3 Channel eine Vielzahl an verschiedenen Operationen ausführen. Für die meisten dieser Betriebsmodi ist es nötig, den Chip an mehreren Stellen zu konfigurieren. Aus diesem Grund wird in diesem Fall die Schnittstelle über `/dev` für den Zugriff über `ioctl()` modelliert.

Der erste Schritt in dieser Überlegung ist es, die wichtigsten Use-Cases als Commands abzubilden. Für die Timer-Counter-Einheit sind es in diesem Fall:
- Event Counting
- Intervall Measurement
- Pulse Generation
- Pulse Width Modulation

Diese Betriebsmodi werden nun als Enum modelliert

#pagebreak()
#render-snippet("tc-mode")

Nun brauchen wir zusätzlich noch einen Mechanismus mit welchem sich die verschiedenen Betriebsmodie modellieren lassen. Analog zur I2C-Schnittstelle, welche mehrere `i2c_msg` Strukturen in einer `i2c_rdwr_ioctl_data` Struktur bündelt, definieren wir für jeden Betriebsmodus eine spezialisierte Konfigurationsstruktur.

#pagebreak()
== Konfigurationsstrukturen

Für jeden der vier Betriebsmodi existiert eine eigene Konfigurationsstruktur, welche die mode-spezifischen Parameter enthält:

#render-snippet("tc-config")

Das struct `tc_event_count_config` konfiguriert das Zählen von externen Events. Der `threshold`-Wert ermöglicht eine Benachrichtigung, wenn eine bestimmte Anzahl an Events erreicht wurde.

Das struct `tc_interval_measure_config` dient der Messung von Zeitintervallen zwischen Flanken.

Das struct `tc_pulse_generation_config` konfiguriert die Erzeugung eines einzelnen Pulses mit konfigurierbarer Verzögerung und Pulsbreite.

Das struct `tc_pwm_config` ermöglicht die Konfiguration eines kontinuierlichen PWM-Signals mit Periode und Tastverhältnis.

== Request-Struktur und ioctl-Kommandos

Analog zu GPIO's `gpio_v2_line_request` definieren wir eine zentrale Request-Struktur, welche mehrere Kanal-Konfigurationen bündeln kann:

#render-snippet("tc-request")

Der zentrale ioctl-Command `TC_GET_CHANNEL_IOCTL` folgt dem Muster von `GPIO_V2_GET_LINE_IOCTL`. Nach erfolgreichem Aufruf enthält das `fd`-Feld einen File-Deskriptor über welchen Events gelesen werden können.

== Event-Struktur

Für das Auslesen von Events und Messwerten definieren wir analog zu `gpio_v2_line_event`:

#render-snippet("tc-event")

== Beispiel: PWM-Konfiguration

Das folgende Beispiel zeigt die Konfiguration einer PWM-Ausgabe mit 1kHz und 50% Tastverhältnis:

#render-snippet("tc-example")

Der Ablauf entspricht dem GPIO-Beispiel: Nach dem Öffnen des Gerätes wird eine Request-Struktur konfiguriert und über `ioctl()` an den Treiber übergeben. Der zurückgegebene File-Deskriptor kann für weitere Commands und zum Auslesen von Events genutzt werden.

#pagebreak()
= Aufgabe III - I2C-Scheduling

== Anwendungsfall: Starvation durch Display-Treiber
Man stelle sich folgendes Szenario vor: Ein Display und ein Temperatursensor sind beide über einen gemeinamen I2C-Bus verbunden. Je nach Bildschirmwiederholfrequenz und Auflösung(wirkt sich auf die Größe des zu übertragenden Daten aus) kann es nun sein, dass der Display-Treiber den I2C-Bus mehrfach pro Sekunde anfragt und somit die Leitung dauerhaft belegt. Der Treiber für den Temperatursensor könnte nun die währendessen die ganze Zeit darauf warten, dass der Bus wieder frei ist, um den aktuellen Messwert über den Bus zu schicken. In diesem Fall kann es sein, dass Messwerte verzögert oder gar nicht ankommen, was für zeitkritische Anwendungen katastrophale Konsequenzen haben kann.

== Lösungsansatz: Fair Queuing im I2C-Subsystem

*Konzept:* I2C-Adapter verwaltet Warteschlange mit Fairness-Mechanismus

Wie in dem I2C-Beispiel bereits behandelt, gruppiert der I2C-Treiber eine oder mehrere Zugriffe in eine Transaktion. Zu Begin einer jeden Transaktion wird `__i2c_lock_bus_helper()` aufgerufen, um den Bus für diese Transaktion zu blockieren @linux_i2c_core.

#render-snippet("i2c-transfer")

#quote(block: true, attribution: [Kommentar in `i2c_transfer()` @linux_i2c_core])[REVISIT the fault reporting model here is weak: When we get a NAK after transmitting N bytes to a slave, there is no way to report "N".]

=== Mutex und RT-Mutex

#quote(block: true, attribution: [Linux Kernel Documentation @kernel_mutex_design])[In the Linux kernel, mutexes refer to a particular locking primitive that enforces serialization on shared memory systems, and not only to the generic term referring to 'mutual exclusion' found in academia or similar theoretical text books.]

Das I2C-Subsystem verwendet einen *RT-Mutex* (Real-Time Mutex), der Wartende nach Task-Priorität sortiert (via Red-Black Tree) anstatt in FIFO-Reihenfolge. Das Bus-Locking geschieht über `rt_mutex_lock_nested()` @linux_i2c_core:

#render-snippet("i2c-lock")

Der RT-Mutex implementiert zusätzlich *Priority Inheritance*: Wenn ein hochpriorer Task auf einen Mutex wartet, wird die Priorität des haltenden Tasks temporär angehoben. Dies verhindert *Priority Inversion*, bei der ein mittelpriorer Task den hochprioren indirekt blockieren könnte.

==== Bedeutung für I2C

Der I2C-Treiber verwendet keinen eigenen Scheduling-Algorithmus, sondern verlässt sich vollständig auf den RT-Mutex. Die Priorität einer I2C-Transaktion entspricht der Priorität des aufrufenden Kernel-Threads (`task->prio`). Um einem Treiber höhere Priorität zu geben, muss dessen Thread mit entsprechender Scheduling-Policy konfiguriert werden (z.B. `SCHED_FIFO` via `sched_setscheduler()`).

Um jedoch zu verhindern, dass sich API-Aufrufe gegenseitig blockieren, wird ein Queuing-System verwendet. Dieses organisiert Aufrufe während eines blockierten Busses. Sobald der Bus wieder freigegeben wurde, wird die am höchsten positionierte Transaktion in der Queu ausgeführt. Die Positionsvergabe geschieht dabei grundlegend nach dem FIFO-Prinzip, jedoch mit einem zusätzlichen Prioritätssystem, welches es erlaubt dringende Aufgaben anteilig höher in der Warteschlange einzuorden. Nach diesem Prinzip könnte ein Sensor z.B. eine höhere Priorität bekommen, was eine reinkommende Transaktion weiter nach Vorne in die Warteschlange stellt.

- *Round-Robin:*
  - Nach jeder abgeschlossenen Transaktion wechselt der Bus zum nächsten wartenden Treiber
  - Kein Treiber kann den Bus unbegrenzt belegen

- *Zeitscheiben (Time Slicing):*
  - Maximale Bus-Belegungszeit pro Treiber
  - Nach Ablauf wird Transaktion unterbrochen und andere Treiber bedient

- *Prioritätsstufen:*
  - Zeitkritische Geräte (z.B. Sensoren) erhalten höhere Priorität
  - Display-Updates können verzögert werden ohne Funktionsverlust

- *Linux-Implementierung:*
  - I2C-Adapter nutzt Mutex für Bus-Arbitrierung
  - `i2c_transfer()` ist atomar, aber zwischen Transfers kann gewechselt werden
  - Scheduling erfolgt auf Transaktionsebene, nicht auf Byte-Ebene

#pagebreak()
// ============================================
// Appendex
// ============================================
= Research Copy and Pase (TODO: REMOVE LATER)

== Devices – eine einfache Verbindung zur Hardware
Damit das Betriebssystem mit der Hardware kommunizieren kann (vereinfacht ausgedrückt), wird in Linux (wie in den meisten anderen Betriebssystemen auch) ein Gerätetreiber dafür verwendet. Unter Linux ist dieser Treiber immer ein Teil des Kernels und kann entweder statisch hinzugelinkt oder auf Anfrage als Kernel-Modul nachgeladen werden. Da ein Gerätetreiber ein Teil des Kernels ist, kann aus Sicherheitsgründen nicht direkt darauf mit einem normal laufenden Prozess zugegriffen werden. Um aber jetzt dennoch auf Hardware-Komponenten wie u. a. Festplatte, CD-ROM, Soundkarte oder der seriellen bzw. parallelen Schnittstelle zugreifen zu können, wurde unter UNIX ein Mechanismus eingeführt, womit recht problemlos mit einem Gerätetreiber über eine Hardware-Gerätedatei kommuniziert werden kann. Diese »Datei« können Sie mit einem gewöhnlichen Prozess öffnen und auf sie lesend und schreibend zugreifen – sprich, Sie können über normale Datei-E/A-Operationen mit dem Gerätetreiber dank dieser »Datei« so kommunizieren, als handle es sich um eine gewöhnliche Datei.

- Treiber sind Teil des Kernels
  - Entweder statisch gelinkt oder dynamisch als Kernel-Modul nachgeladen

=== Die Gerätedateitypen
Die Einträge der Gerätedateien – auch als Devices bekannt – finden Sie im Verzeichnis `/dev`. Ein `ls -l /dev` verschafft Ihnen einen ersten Überblick und stiftet vielleicht zugleich auch ein wenig Verwirrung über diese Gerätedateien, die es zu entflechten gilt. Es wird hierbei zwischen zwei Devices-Typen unterschieden – zu erkennen am ersten Buchstaben der Auflistung der Gerätedatei(en) (b oder c):



==== character device (c)
hierbei handelt es sich um Hardwaregeräte (… man kann allerdings mit einem Character Device auch eine Software-Lösung realisieren, z. B. ttyrpld (http://linux01.org:2222/prog-ttyrpld.php)), mit der eine fortlaufende Folge von Datenbytes gelesen oder geschrieben wird. Dies sind Hardware-Komponenten wie Soundkarte, Tapes, Terminalgeräte, serielle oder parallele Schnittstelle(n), Maus, USB-Schnittstelle usw.
gp

In the first category, there are slow devices, which manage a small amount of data, and access to data does not require frequent seek queries. Examples are devices such as keyboard, mouse, serial ports, sound card, joystick. In general, operations with these devices (read, write) are performed sequentially byte by byte.

==== block device (b)
Blockgeräte sind Devices, mit denen feste ganze Blöcke (anstatt Bytes) gelesen oder geschrieben werden können. Dabei wird erst ein Puffer aufgefüllt, bevor der ganze Datenblock an den Treiber gesendet wird. Daher sind blockorientierte Geräte auch meistens etwas schneller als zeichenorientierte, da gewartet werden muss, bis ein Block voll ist. Zu den Blockgeräten gehören z. B. die Festplatte, Diskette oder das CD-ROM-/DVD-Laufwerk. Ein zeichenorientiertes Gerät hat mehr Aufwand, weil nach jedem read/write bereits eine Art flush stattfindet.

The second category includes devices where data volume is large, data is organized on blocks, and search is common. Examples of devices that fall into this category are hard drives, cdroms, ram disks, magnetic tape drives. For these devices, reading and writing is done at the data block level.

For the two types of device drivers, the Linux kernel offers different APIs. If for character devices system calls go directly to device drivers, in case of block devices, the drivers do not work directly with system calls. In the case of block devices, communication between the user-space and the block device driver is mediated by the file management subsystem and the block device subsystem. The role of these subsystems is to prepare the device driver's necessary resources (buffers), to keep the recently read data in the cache buffer, and to order the read and write operations for performance reasons.

== Kernel-Modul

== Architektur
Kernel <=> Kernel IO Subsystem <=> Treiber <=> Controller <=> Hardware

== Overview
In UNIX, hardware devices are accessed by the user through special device files.
These files are grouped into the `/dev` directory, and system calls `open`, `read`, `write`, `close`, `lseek`, `mmap` etc. are redirected by the operating system to the device driver associated with the physical device.

The device driver is a kernel component (usually a module) that interacts with a hardware device.

In the UNIX world there are two categories of device files and thus device drivers: character and block. This division is done by the speed, volume and way of organizing the data to be transferred from the device to the system and vice versa.

== Character Devices
== Block devices

== Major and Minor
In UNIX, the devices traditionally had a unique, fixed identifier associated with them. This tradition is preserved in Linux, although identifiers can be dynamically allocated (for compatibility reasons, most drivers still use static identifiers).
The identifier consists of two parts: major and minor.
=== Major
The first part identifies the device type (IDE disk, SCSI disk, serial port, etc.) and
Most times, the major identifies the driver
=== Minor
the second one identifies the device (first disk, second serial port, etc.).  Most of the times the minor identifies each physical device served by the driver. In general, a driver will have a major associate and will be responsible for all minors associated with that major.



= Glossar
#print-glossary(
  entry-list,
)

#pagebreak()

#bibliography(
  "config/bibliography.bib",
  title: "Bibliographie",
  style: "institute-of-electrical-and-electronics-engineers",
)

