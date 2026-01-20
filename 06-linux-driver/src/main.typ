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
Unter Linux kann fast alles als Datei behandelt werden. Die Aufgabe des entsprechenden Filesystem-Treibers ist es dann die Abstraktion über _Pfade_ und _Namen_ auf die wirkliche Hardware abzubilden. Die wichtigesten hierfür bereitzustellenden Operationen sind: _open_, _close_, _read_ und _write_.

=== Peripheriezugriff
Die Adressierung von Peripheriegeräten erfolgt über Gerätetreiber. Die Aufgabe dieser sind es den Kernel in einer Art zu erweitern, welche es erlaubt die Funktionsweise der Peripherie auf eine oder mehere Datein innerhalbt eines Filesystems abzubilden. Diese Treiber werden dabei entweder statisch in den Kernel gelinkt oder können als Kernel-Modul dynamisch zur Laufzeit dazugeladen werden.

Die phyischen Peripheriegeräte erscheinen durch die Abtraktion wie eine oder mehrere logische Datein. Dies ermöglicht es auf der Applikationsebene mit diesen mit den bekannten Dateioperationen zu interargieren.

Für gewöhnlich erscheinen Peripheriegeräten dann unter _/dev_ oder _/sys_. //TODO: auf Unterschiede zwischen /dev und /sys eingegen

Geräte erhalten dabei von ihrem Treiber sowohl eine _Major_- als auch eine _Minor_-Nummer. Die Major-Nummer bezieht sich hierbei auf die den jeweiligen Treiber und bildet eine logische Obergruppe über die einzelnen Geräte. Die Minor-Nummer hingegen referenziert ein speziefisches Gerät.

Wichtig hierbei ist, dass dies nicht zwangsläufig heißen muss, dass es sich bei unterschiedlichen Geräten auch um unterschiedliche phyisische Peripheriegeräte handeln muss. Das von der Minor-Nummer gekennzeichnete Geräte muss in wirklichkeit nicht einmal ein phyisches Peripheriegerät sein, sondern es kann sich dabei auch um eine logische Abstraktion handeln. Ein gutes Beispiel hierfür wäre das _loopback_-Interface.


Peripheriegeräte können dabei grob in _Character Devices_, _Block Devices_ oder _Netzwerk Devices_ unterteilt werden.

_Character Devices_ lesen oder schreiben fortlaufende Folge von Datenbytes. //TODO Zitat einfügen
Für die einfache serielle Übertragung von Daten, z.B durch _I2C_ sind diese somit prädistiniert und werden Hauptgegenstand der folgendenden Auseinandersetzunge sein. Die anderen beiden Typen lassen wir hierbei außer Acht.
//TODO ausführlicher machen

=== Netzwerk
Netzwerkvorgänge sind selten speziefisch für einen einzelnen Prozess, weswegen hierfür ein eigenes Sub-System benötigt wird. Für den Kontext dieser Diskusion ist es aber nicht nötig uns mit Netzwerken zu beschäftigen.


#pagebreak()
= Aufgabe I - Peripheriezugriff

Beschreiben sie für nachfolgende Schnittstellen, wie der Zugriff/Konfiguration dieser Schnittstellen funktioniert (über welche Schnittstelle werden diese bedient, welche Zugriffe bewirken was, wie sieht ein typischer Zugriff aus)

- I2C
- GPIO
- ADC

Timer-Einheiten stellen in der Regel keine Funktionalitäten über das Datei-System bereit, da sie eher ähnlich dem I2C-Bus von anderen internen Treibern intern genutzt werde (bspw. Geschwindigkeitsbestimmung nutzt u.A. TimerEinheit, zyklische Funktionsaufrufe werden über Timer-IRQ's dargestellt, ...). Ähnlich wie beim I2C Bus stellen sie somit ihre Funktionalität nur kernelintern über Funktionsaufrufe bereit.


== I2C
Vor der Interaktion mit der I2C-Schnittstelle muss zunächst, falls nicht bereits geschehen, das entsprechende Kernel-Modul geladen werde. Für Entwicklungszwecke können wir _i2c-dev_ einfach per _modprobe_ laden.
```bash
sudo modprobe i2c-dev
```

Nun sollten die entsprechenden Nodes, welche der I2C-Treiber bereitstellt über:

```bash
ls -la /dev | grep i2c
```
auffindbar sein. Eine beispielhafte ausgabe kann dabei wie folgt aussehen:

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
/sys beeinhaltet häufig mehr Informationen //TODO

```bash
ls -la /sys/bus/i2c/devices/*/
```
Hierbei erhalten wir auch einen Blick auf möglicher Konfigurationsdatein wie beispielsweise hier an dem Gerät

```
/sys/bus/i2c/devices/i2c-6/:
Permissions Size User Date Modified Name
drwxr-xr-x     - root 18 Jan 20:20  i2c-dev
drwxr-xr-x     - root 18 Jan 20:20  power
lrwxrwxrwx     - root 18 Jan 20:20  subsystem -> ../../../../../../../bus/i2c
.-w-------  4.1k root 18 Jan 20:20  delete_device
.r--r--r--  4.1k root 18 Jan 20:20  name
.-w-------  4.1k root 18 Jan 20:20  new_device
.rw-r--r--  4.1k root 18 Jan 20:20  uevent
```

Mit dem folgenden Befehl können wir nun beispielsweise gucken, welchen Namen das entsprechende Gerät hat

```bash
cat /sys/bus/i2c/devices/i2c-6/name
```
In diesem konkreten Beispiel handelt es sich um die Schnittstelle des im Laptop verbauten Displays
```
AUX A/DDI A/PHY A
```

Das Programm `i2cdetect` aus dem Packet `i2c-tools` kann nun einen mit dem folgenen Befehl einen Überblick über die verfügbaren Adressen geben

```bash
sudo i2cdetect -y 6
```

Für das Gerät mit der Nummer 6 sind in diesem Fall die folgenden Adressen verfügbar

```
    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- 38 -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f
60: 60 61 62 63 64 65 66 67 -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

Nun nutzen wir das folgende C-Programm, um den Inhalt dieser Adresse auszulesen.
#pagebreak()
#render-snippet("i2c")

Hier wird Beispielsweise das Register `0x00` and der Adresse `0x50` gelesen.

Der Output in diesem Fall war

```
Gelesen von Gerät 0x50 (Register 0x00):
0x00 0xff 0xff 0xff 0xff 0xff 0xff 0x00
```

//TODO entsprechenden Imports dokumentieren


== GPIO
Der Zugriff auf GPIO (General Purpose Input/Output) erfolgt unter Linux über das Character Device Interface. Die entsprechenden Gerätedateien befinden sich unter _/dev/gpiochipX_.

Mit dem folgenden Befehl können wir die verfügbaren GPIO-Chips auflisten:

```bash
ls -la /dev | grep gpiochip
```

Eine beispielhafte Ausgabe kann dabei wie folgt aussehen:

```
crw-------   1 root   root   254,     0 Jan 18 19:52 gpiochip0
crw-------   1 root   root   254,     1 Jan 18 19:52 gpiochip1
crw-------   1 root   root   254,     2 Jan 18 19:52 gpiochip2
```

Zusätzliche Informationen zu den GPIO-Chips finden sich im sysfs unter:

```bash
ls -la /sys/class/gpio/
```

Für detaillierte Informationen zu einem bestimmten Chip können wir auch die Kernel-Dokumentation über das debugfs nutzen:

```bash
cat /sys/kernel/debug/gpio
```

Hier erhalten wir eine Übersicht über alle GPIO-Controller und den aktuellen Zustand der einzelnen Lines.

Das Tool `gpioinfo` aus dem Paket `gpiod` bietet einen übersichtlichen Zugang zu den verfügbaren GPIO-Lines:

```bash
sudo gpioinfo gpiochip0
```

Eine beispielhafte Ausgabe:

```
gpiochip0 - 32 lines:
        line   0:      unnamed       unused   input  active-high
        line   1:      unnamed       unused   input  active-high
        ...
        line  17:      unnamed       unused   input  active-high
        ...
```

Nun nutzen wir das folgende C-Programm, um den Wert einer GPIO-Line über das Character Device Interface zu lesen.
#pagebreak()
#render-snippet("gpio")

Das Programm öffnet den GPIO-Chip, fragt Informationen zum Chip ab und liest dann den aktuellen Wert der Line 17.

Der Output kann beispielsweise wie folgt aussehen:

```
GPIO Chip: gpiochip0
Label: INT34C5:00
Anzahl Lines: 312
GPIO Line 17 Wert: LOW
```

Die wichtigsten ioctl-Befehle für GPIO sind:
- `GPIO_GET_CHIPINFO_IOCTL` - Abrufen der Chip-Informationen
- `GPIO_V2_GET_LINE_IOCTL` - Anfordern einer GPIO-Line
- `GPIO_V2_LINE_GET_VALUES_IOCTL` - Lesen des aktuellen Wertes
- `GPIO_V2_LINE_SET_VALUES_IOCTL` - Setzen eines Wertes (bei Output-Konfiguration)

== ADC
//TODO
#pagebreak()
= Aufgabe II - AT91SAM7-Timer Schnittstellenspezifikation

Überlegen sie eine Schnittstellespezifikation, wie möglichst viele Funktionalitäten des AT91SAM7-Timers über eine der beiden Datei-Schnittstellen bereitgestellt werden können.

I2C Scheduling (gilt auch für anderen Bussysteme wie USB,…)

Am I2C Bus können mehrere Devices angeschlossen sein. Aus Sicht des Gerätetreibers sind somit 2 Gerätetreiber zuständig. Einer für den I2C-Bus zum Senden und Empfangen von I2C Nachrichten und einer für das am I2C-Bus angeschlossene Device, wobei der Anwender dann nur mit dem Gerätetreiber des Devices direkt interagiert. Wenn jetzt ein Anwender intensiv mit 'seinem' Device interagiert, so besteht die Gefahr, dass andere Devices 'ausgehungert' werden. Der I2C-Treiber beinhaltet somit nicht nur den Treiber zum Senden und Empfangen von Nachrichten, sondern sorgt auch für eine faire Zuteilung.

#pagebreak()
= Aufgabe III - I2C-Scheduling
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

