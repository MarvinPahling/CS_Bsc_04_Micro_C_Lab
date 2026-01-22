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
Im Vergleich zu der
Die wichtigsten Aufgaben des Linux-Kernels sind:
=== Process Management
Das Betriebssystem ist in der Lage einzelne Prozesse zu starten und zu beenden. Darüber hinaus werden Input und Output sowie die Kommunikation zwischen den verschiedenen Prozessen von dem Kernel übernommen. Da sich in der Regel mehrere Prozesse eine CPU teilen, ist der Kerel auch für das Sheduling der Rechenzeit der geteilten CPU verantwortlich.

=== Memory Management
Der Kernel stellt darüber hinaus eine Abstraktionsebene über den geteilten Memory bereit. Hierbei wird ein virtueller Adressraum für alle Prozesse bereitgestellt, welcher dann von dem Kernel auf den echten Memory abgebildet wird.

=== Filesystem
Unter Linux kann fast alles als Datei behandelt werden. Die Aufgabe des entsprechenden Filesystem-Treibers ist es dann die Abstraktion über _Pfade_ und _Knoten_ auf die darunterliegende phyische Struktur abzubilden.

=== Netzwerk
Netzwerkvorgänge sind selten speziefisch für einen einzelnen Prozess, weswegen hierfür ein eigenes Sub-System benötigt wird. Für den Kontext dieser Diskusion ist es aber nicht nötig uns mit Netzwerken zu beschäftigen.

=== Peripheriezugriff
Die Adressierung von Peripheriegeräten erfolgt über Gerätetreiber. Die Aufgabe dieser sind es den Kernel in einer Art zu erweitern, welche es erlaubt die Funktionsweise der Peripherie auf eine oder mehere Datein innerhalbt eines Filesystems abzubilden. Diese Treiber werden dabei entweder statisch in den Kernel gelinkt oder können als Kernel-Modul dynamisch zur Laufzeit dazugeladen werden.

Die phyischen Peripheriegeräte erscheinen durch die Abtraktion wie eine oder mehrere logische Datein. Dies ermöglicht es auf der Applikationsebene mit diesen mit den bekannten Dateioperationen zu interargieren.


Geräte erhalten dabei von ihrem Treiber sowohl eine _Major_- als auch eine _Minor_-Nummer. Die Major-Nummer bezieht sich hierbei auf die den jeweiligen Treiber und bildet eine logische Obergruppe über die einzelnen Geräte. Die Minor-Nummer hingegen referenziert ein speziefisches Gerät.

Wichtig hierbei ist, dass dies nicht zwangsläufig heißen muss, dass es sich bei unterschiedlichen Geräten auch um unterschiedliche phyisische Peripheriegeräte handeln muss. Das von der Minor-Nummer gekennzeichnete Geräte muss in wirklichkeit nicht einmal ein phyisches Peripheriegerät sein, sondern es kann sich dabei auch um eine logische Abstraktion handeln. Ein gutes Beispiel hierfür wäre das _loopback_-Interface.

Peripheriegeräte können dabei grob in _Character Devices_, _Block Devices_ oder _Netzwerk Devices_ unterteilt werden.

_Character Devices_ lesen oder schreiben fortlaufende Folge von Datenbytes. //TODO Zitat einfügen
Für die einfache serielle Übertragung von Daten, z.B durch _I2C_ sind diese somit prädistiniert und werden Hauptgegenstand der folgendenden Auseinandersetzunge sein. Die anderen beiden Typen lassen wir hierbei außer Acht.

#pagebreak()
= Aufgabe I - Peripheriezugriff

Beschreiben sie für nachfolgende Schnittstellen, wie der Zugriff/Konfiguration dieser Schnittstellen funktioniert (über welche Schnittstelle werden diese bedient, welche Zugriffe bewirken was, wie sieht ein typischer Zugriff aus)

- I2C
- GPIO
- ADC

Timer-Einheiten stellen in der Regel keine Funktionalitäten über das Datei-System bereit, da sie eher ähnlich dem I2C-Bus von anderen internen Treibern intern genutzt werde (bspw. Geschwindigkeitsbestimmung nutzt u.A. TimerEinheit, zyklische Funktionsaufrufe werden über Timer-IRQ's dargestellt, ...). Ähnlich wie beim I2C Bus stellen sie somit ihre Funktionalität nur kernelintern über Funktionsaufrufe bereit.

== Zugriff/Konfiguration

Für gewöhnlich erscheinen Peripheriegeräten unter dem pfad _/dev_ oder _/sys_, dieses beiden Subsysteme unterscheiden sich neben ihrer Verortung auch in ihrer grundlegenden Designphilosophie.

=== sysfs
_/sys_ ein einfacher weg, um mit Peripherie über eine logische ASCII-kodierte Datei zu interargieren.   Hierbei beinhaltee für gewöhnlich jede Datei nur genau eine Information. Dies ermöglicht es auch auf einfache Weise die Topologie eines Gerätes abzubilden, da hierarchische Beziehungen über den Pfad abgebildet werden können. Je nach Peripheriegerät kann der Gerätetreiber für diese logischen Datein Lese- und oder Schreibezugriff bereitstellen. Die klassischen Interaktionen sind `open()`, `close()`, `read()`, `write()`.

Diese Art Schnittstelle eignet sich dadurch hervorragend für Konfiguration oder Statusabfragen von Geräten. Durch das logische Auftreten als ASCII-kodierte Textdatei lassen sich Informationen einfach über Tools wie z.B. `cat` auslesen, was es einfach macht Skripte zu erstellen.

Die Schwächen dieses Systems sind jedoch, dass sich komplexere, atomare Transaktionen nur schwer abbilden lassen. Wollen wir beispielsweise über einen I2C-Bus erste eine Schreib-, gefolgt von einer Leseoperation ausführen, ohne den Bus zwischenzeitlich wieder freizugeben, ist dies über diesen Ansatz nicht möglich.

=== device
_/dev_ ist der klassische UNIX-Weg, um auf die Peripherie zuzugreifen. Das Entsprechende Gerät wird hierbei logisch als eine binäre Datei realisiert, was den entsprechenden Overhead eliminiert, den es braucht, um ASCII-kodierte Datein in Binärformat zu parsen. Der Inhalt der Datei repräsentiert hierbei den eigentlichen Datenfluss für das Gerät.

Eine Besonderheit bei diesem Subsystem ist es, dass durch `ioctl()`, zusätzlich zu den bereits bekannten Operationen `open()`, `close()`, `read()`, `write()`,
auch komplexere Informationen für die Entsprechende Transaktion mitliefern kann.

#pagebreak()
#render-snippet("open")
#render-snippet("ioctl")

Die Vorteile dieses Ansatzes sind, dass die Funktion `ioctl()` es ermöglicht sowohl ein Command-Code, sowie optional einen Zeiger zu entsprechenden Nutzdaten für die Transaktion zu übergeben. Eine entsprechende Schnittstelle kann somit eine komplexe atomare Transaktion in nur einem Funktionsaufruf abbilden.

== I2C
=== Schnittstellen
Die eben beschriebene Transaktion über den I2C-Bus wollen wir an dieser Stelle exemplarisch durchgehen. Die Schnittstellendefinition für I2C findet sich in `linux/i2c.h` sowie `linux/i2c-dev.h`

#render-snippet("i2c-msg")

Das gezeigte `struct` hierbei alle Informationen für einen I2C-Zugriff. Hierzu zählen die Zieladresse, die Flags /*TODO: Fachbegriff*/ die Lände der Nachricht, sowie ein Zeiger zu der Nachricht selber.

#render-snippet("i2c-rdwr")


Das Kernel-Modul `i2c-dev` ist häufig nicht standardmäßig geladen und per `modprobe` nachgeladen werden.
```bash
sudo modprobe i2c-dev
```

Da es sich bei den von dem I2C bereitgestellten Geräte logisch um Datein handelt lassen sich diese einfach per `ls` und `grep` finden.

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

Die Schnittstellendefinition für GPIO findet sich in `linux/giop.h`. Die API für GPIO unterteilt ein GPIO-Device in `chip` und `line`
#render-snippet("gpio-req")
Die API für die Request an eine Line des GPIO-Controllers ist in diesem Fall Umfangreicher als die Vorherige. Für den Kontext dieses Moduls sind jedoch vor allem die Flags interessant mit welchen sich beispielsweise der Pull-Up-Wiederstand oder der Edge-Detector konfigurieren lassen.
#render-snippet("gpio-event")
Die API für GPIO-Events gibt uns die Möglichkeit später die Möglichkeit Informationen über Änderungen am Edge-Detector zu erhalten. Aus dem Event können wir beispielsweise den Zeistempel, sowie die Art des Events auslesen.

Das nun folgende C-Programm, zeigt nun ein Beispiel, wie wir einen softwareseitigen Edge-Detector in Linux zu realisieren können.
#pagebreak()
#render-snippet("gpio")

Das Programm Konfiguriert `line`-7 als Input und Konfiguriert den Edge-Detector, sodass er steigende und fallende Taktflanken erkennt. Über `read()` (blockierender Aufruf) können wir nun auf ein neues Event warten. Der Vorteil im Vergleich zu der Laboraufgabe ist dabei, dass in diesem Fall nur der Thread des entsprechenden Prozesses blockiert wird, jedoch der Rest des Systems weiterarbeiten kann.

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
Da sich in solchen Fällen die API-Spezifikation häufig aus der Struktur ergibt, kann es ratsam sein sich die entsprechende Schnittstelle mit `tree` einmal genauer anzugucken. Ein Aufruf könnte z.B. so aussseh.


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
Da es sich hierbei logisch um ASCII-kodierte Datein handel, können wir diese einfach mit `cat` auslesen.

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

Überlegen sie eine Schnittstellespezifikation, wie möglichst viele Funktionalitäten des AT91SAM7-Timers über eine der beiden Datei-Schnittstellen bereitgestellt werden können.

Die Timer-Couter-Einheit des AT91SAM7 kann auf jedem seiner 3 Channel eine Vielzahl an verschiedenen Operationen ausführen. Für die meisten dieser Betriebsmodie ist es nötig den Chip an mehreren Stellen zu konfigurieren. Aus diesem Grund wird in diesem Fall die Schnittstelle über `/dev` für den Zugriff über `ioctl()` modelliert.

Der erste Schritt in dieser Überlegung ist es die wichtigsten Use-Cases als Commands abzubilden. Für die Timer-Couter-Einheit sind es in diesem Fall:
- Event Counting
- Intervall Measurement
- Pulse Generation
- Pulse Width Modulation

Diese Betriebsmodie werden nun als Enum modelliert

#render-snippet("tc-mode")

Nun brauchen wir zusätzlich noch einen Mechanismus mit welchem sich die verschiedenen Betriebsmodie modellieren lassen. Analog zur I2C-Schnittstelle, welche mehrere `i2c_msg` Strukturen in einer `i2c_rdwr_ioctl_data` Struktur bündelt, definieren wir für jeden Betriebsmodus eine spezialisierte Konfigurationsstruktur.

== Konfigurationsstrukturen

Für jeden der vier Betriebsmodi existiert eine eigene Konfigurationsstruktur, welche die mode-spezifischen Parameter enthält:

#render-snippet("tc-config")

Die Struktur `tc_event_count_config` konfiguriert das Zählen von externen Events auf einem der Eingangs-Pins. Der `threshold`-Wert ermöglicht eine Benachrichtigung, wenn eine bestimmte Anzahl an Events erreicht wurde.

Die Struktur `tc_interval_measure_config` dient der Messung von Zeitintervallen zwischen Flanken. Hierbei wird der interne Zähler zwischen Start- und Stop-Flanke inkrementiert.

Die Struktur `tc_pulse_generation_config` konfiguriert die Erzeugung eines einzelnen Pulses mit konfigurierbarer Verzögerung und Pulsbreite.

Die Struktur `tc_pwm_config` ermöglicht die Konfiguration eines kontinuierlichen PWM-Signals mit Periode und Tastverhältnis.

== Request-Struktur und ioctl-Kommandos

Analog zu GPIO's `gpio_v2_line_request` definieren wir eine zentrale Request-Struktur, welche mehrere Kanal-Konfigurationen bündeln kann:

#render-snippet("tc-request")

Das zentrale ioctl-Kommando `TC_GET_CHANNEL_IOCTL` folgt dem Muster von `GPIO_V2_GET_LINE_IOCTL`. Nach erfolgreichem Aufruf enthält das `fd`-Feld einen File-Deskriptor über welchen Events gelesen werden können.

== Event-Struktur

Für das Auslesen von Events und Messwerten definieren wir analog zu `gpio_v2_line_event`:

#render-snippet("tc-event")

== Beispiel: PWM-Konfiguration

Das folgende Beispiel zeigt die Konfiguration einer PWM-Ausgabe mit 1kHz und 50% Tastverhältnis:

#render-snippet("tc-example")

Der Ablauf entspricht dem GPIO-Beispiel: Nach dem Öffnen des Gerätes wird eine Request-Struktur konfiguriert und über `ioctl()` an den Treiber übergeben. Der zurückgegebene File-Deskriptor kann für weitere Steuerkommandos und das Auslesen von Events genutzt werden.

#pagebreak()
= Aufgabe III - I2C-Scheduling

I2C Scheduling (gilt auch für anderen Bussysteme wie USB,…)

Am I2C Bus können mehrere Devices angeschlossen sein. Aus Sicht des Gerätetreibers sind somit 2 Gerätetreiber zuständig. Einer für den I2C-Bus zum Senden und Empfangen von I2C Nachrichten und einer für das am I2C-Bus angeschlossene Device, wobei der Anwender dann nur mit dem Gerätetreiber des Devices direkt interagiert. Wenn jetzt ein Anwender intensiv mit 'seinem' Device interagiert, so besteht die Gefahr, dass andere Devices 'ausgehungert' werden. Der I2C-Treiber beinhaltet somit nicht nur den Treiber zum Senden und Empfangen von Nachrichten, sondern sorgt auch für eine faire Zuteilung.

Bearbeiten sie folgende Punkte zum Thema I2C-Scheduling


Skizzieren sie einen Anwendungsfall, bei welchen ein Devicetreiber (z.B. Displaytreiber) einen anderen Devicetreiber aushungern kann.
Überlegen und begründen sie eine Möglichkeit, wie dies vermieden werden kann

Hinweis:

Bei Gerätetreibern wird zwischen Block Device und Character Device unterschieden. Die im Rahmen der Vorlesung behandelten Peripherieelemente sind typischerweise Character Devices, so dass sie nur diese betrachten müssen
Für einige Treiber wurden zum vereinfachten Zugriff auf die Gerätedateien Wrapper entwickelt (z.B. i2c_smbus_read_word_data()). Diese max. am Rande beschreiben. Es gilt den direkten Zugriff zu berücksichtigen
Ebenso bitte nicht auf das Thema Treiber Installation (z.B. insmod, modprobe), installierte Treiber überprüfen (z.B. lsmod) oder Hardwareabfrage (z.B. lsusb, lspci, lshw) eingehen


#pagebreak()
// ============================================
// Appendex
// ============================================
= Research Copy and Pase (TODO: REMOVE LATER)

== Devices – eine einfache Verbindung zur Hardware
Damit das Betriebssystem mit der Hardware kommunizieren kann (vereinfacht ausgedrückt), wird in Linux (wie in den meisten anderen Betriebssystemen auch) ein Gerätetreiber dafür verwendet. Unter Linux ist dieser Treiber immer ein Teil des Kernels und kann entweder statisch hinzugelinkt oder auf Anfrage als Kernel-Modul nachgeladen werden. Da ein Gerätetreiber ein Teil des Kernels ist, kann aus Sicherheitsgründen nicht direkt darauf mit einem normal laufenden Prozess zugegriffen werden. Um aber jetzt dennoch auf Hardware-Komponenten wie u. a. Festplatte, CD-ROM, Soundkarte oder der seriellen bzw. parallelen Schnittstelle zugreifen zu können, wurde unter UNIX ein Mechanismus eingeführt, womit recht problemlos mit einem Gerätetreiber über eine Hardware-Gerätedatei kommuniziert werden kann. Diese »Datei« können Sie mit einem gewöhnlichen Prozess öffnen und auf sie lesend und schreibend zugreifen – sprich, Sie können über normale Datei-E/A-Operationen mit dem Gerätetreiber dank dieser »Datei« so kommunizieren, als handle es sich um eine gewöhnliche Datei.

- Treiber sind Teil des Kernels
  - Entweder statisch gelikt oder dynamisch als Kernel-Modul nachgeladen

=== Die Gerätedateitypen
Die Einträge der Gerätedateien – auch als Devices bekannt – finden Sie im Verzeichnis '/dev'. Ein ls -l /dev verschafft Ihnen einen ersten Überblick und stiftet vielleicht zugleich auch ein wenig Verwirrung über diese Gerätedateien, die es zu entflechten gilt. Es wird hierbei zwischen zwei Devices-Typen unterschieden – zu erkennen am ersten Buchstaben der Auflistung der Gerätedatei(en) (b oder c):



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
These files are grouped into the _/dev_ directory, and system calls _open, read, write, close, lseek, mmap_ etc. are redirected by the operating system to the device driver associated with the physical device.

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

