#import "@preview/finite:0.5.0" as finite
#import finite: automaton
#import "@preview/glossarium:0.5.9": gls, glspl, make-glossary, print-glossary, register-glossary
#show: make-glossary
#import "@preview/circuiteria:0.2.0"
#import "@preview/zebraw:0.6.1": *
// Glossary
#let entry-list = (
  (
    key: "i2c",
    short: "I2C",
    long: "Inter-Integrated Circuit",
    description: "Ein serieller Zweidraht-Bus, entwickelt von Philips (heute NXP), für die Kommunikation zwischen integrierten Schaltkreisen. Verwendet eine Datenleitung (SDA) und eine Taktleitung (SCL).",
  ),
  (
    key: "gpio",
    short: "GPIO",
    long: "General Purpose Input/Output",
    description: "Universelle digitale Ein-/Ausgabe-Pins eines Mikrocontrollers oder SoCs, die zur Laufzeit als Eingang oder Ausgang konfiguriert werden können.",
  ),
  (
    key: "adc",
    short: "ADC",
    long: "Analog-to-Digital Converter",
    description: "Wandelt analoge Signale (z.B. Spannungen) in digitale Werte um. Die Auflösung wird in Bit angegeben (z.B. 10-Bit ADC).",
  ),
  (
    key: "fifo",
    short: "FIFO",
    long: "First In, First Out",
    description: "Warteschlangen-Prinzip, bei dem das zuerst eingefügte Element auch zuerst entnommen wird. Häufig als Hardware-Puffer in Kommunikationsschnittstellen verwendet.",
  ),
  (
    key: "ioctl",
    short: "ioctl",
    long: "Input/Output Control",
    description: "POSIX-Systemaufruf zur gerätespezifischen Steuerung, der nicht durch Standard-Dateioperationen abgedeckt wird. Ermöglicht komplexe, atomare Transaktionen.",
  ),
  (
    key: "sysfs",
    short: "sysfs",
    long: "System Filesystem",
    description: "Virtuelles Dateisystem unter `/sys`, das Kernel-Objekte als ASCII-kodierte Dateien exportiert. Jede Datei repräsentiert typischerweise einen einzelnen Wert.",
  ),
  (
    key: "devfs",
    short: "devfs",
    long: "Device Filesystem",
    description: "Dateisystem unter `/dev` für Gerätedateien. Ermöglicht Zugriff auf Character- und Block-Devices über binäre Dateioperationen.",
  ),
  (
    key: "pwm",
    short: "PWM",
    long: "Pulse Width Modulation",
    description: "Modulationsverfahren, bei dem die Pulsbreite eines rechteckigen Signals variiert wird, um einen Mittelwert zu erzeugen. Wird häufig zur Motorsteuerung oder LED-Dimmung verwendet.",
  ),
  (
    key: "mutex",
    short: "Mutex",
    long: "Mutual Exclusion",
    description: "Synchronisationsmechanismus, der sicherstellt, dass nur ein Thread gleichzeitig auf eine gemeinsame Ressource zugreifen kann.",
  ),
  (
    key: "rtmutex",
    short: "RT-Mutex",
    long: "Real-Time Mutex",
    description: [Spezielle Mutex-Implementierung im Linux-Kernel mit Priority Inheritance, die Priority Inversion verhindert und Wartende nach Task-Priorität sortiert. @kernel_rtmutex],
  ),
  (
    key: "chardev",
    short: "Character Device",
    description: "Gerätetyp, der Daten als fortlaufende Folge von Bytes liest oder schreibt. Beispiele: serielle Schnittstellen, Terminals, Soundkarten.",
  ),
  (
    key: "blockdev",
    short: "Block Device",
    description: "Gerätetyp, der Daten in festen Blöcken liest oder schreibt. Beispiele: Festplatten, SSDs, CD-ROM-Laufwerke.",
  ),
  (
    key: "major",
    short: "Major Number",
    description: "Identifiziert den Gerätetreiber im Linux-Kernel. Geräte mit gleicher Major-Nummer werden vom selben Treiber verwaltet.",
  ),
  (
    key: "minor",
    short: "Minor Number",
    description: "Identifiziert ein spezifisches Gerät innerhalb eines Treibers. Ermöglicht es einem Treiber, mehrere Geräte zu verwalten.",
  ),
  (
    key: "hwmon",
    short: "hwmon",
    long: "Hardware Monitoring",
    description: "Linux-Kernel-Subsystem zur Überwachung von Hardware-Sensoren (Temperatur, Spannung, Lüftergeschwindigkeit). Daten werden über sysfs bereitgestellt.",
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

Geräte erhalten dabei von ihrem Treiber sowohl eine #gls("major") als auch eine #gls("minor"). Die #gls("major") bezieht sich hierbei auf den jeweiligen Treiber und bildet eine logische Obergruppe über die einzelnen Geräte. Die #gls("minor") hingegen referenziert ein spezifisches Gerät.

Wichtig hierbei ist, dass dies nicht zwangsläufig heißen muss, dass es sich bei unterschiedlichen Geräten auch um unterschiedliche physische Peripheriegeräte handeln muss. Das von der #gls("minor") gekennzeichnete Gerät muss in Wirklichkeit nicht einmal ein physisches Peripheriegerät sein, sondern es kann sich dabei auch um eine logische Abstraktion handeln. Ein gutes Beispiel hierfür wäre das `loopback`-Interface.

Peripheriegeräte können dabei grob in #gls("chardev"), #gls("blockdev") oder Netzwerk Devices unterteilt werden.

#glspl("chardev") lesen oder schreiben fortlaufende Folge von Datenbytes.
Für die einfache serielle Übertragung von Daten, z.B. durch #gls("i2c") sind diese somit prädestiniert und werden Hauptgegenstand der folgenden Auseinandersetzungen sein. Die anderen beiden Typen lassen wir hierbei außer Acht.

#pagebreak()
= Aufgabe I - Peripheriezugriff

== Zugriff/Konfiguration

Für gewöhnlich erscheinen Peripheriegeräte unter dem Pfad `/dev` oder `/sys`, diese beiden Subsysteme unterscheiden sich neben ihrer Verortung auch in ihrer grundlegenden Designphilosophie.

=== sysfs
`/sys` ist ein einfacher Weg, um mit Peripherie über eine logische ASCII-kodierte Datei zu interagieren. Hierbei beinhaltet für gewöhnlich jede Datei nur genau eine Information. Dies ermöglicht es auch auf einfache Weise die Topologie eines Gerätes abzubilden, da hierarchische Beziehungen über den Pfad abgebildet werden können. Je nach Peripheriegerät kann der Gerätetreiber für diese logischen Dateien Lese- und oder Schreibzugriff bereitstellen. Die klassischen Interaktionen sind `open()`, `close()`, `read()`, `write()`.

Diese Art Schnittstelle eignet sich dadurch hervorragend für Konfiguration oder Statusabfragen von Geräten. Durch das logische Auftreten als ASCII-kodierte Textdatei lassen sich Informationen einfach über Tools wie z.B. `cat` auslesen, was es einfach macht Skripte zu erstellen.

Die Schwächen dieses Systems sind jedoch, dass sich komplexere, atomare Transaktionen nur schwer abbilden lassen. Wollen wir beispielsweise über einen #gls("i2c")-Bus erste eine Schreib-, gefolgt von einer Leseoperation ausführen, ohne den Bus zwischenzeitlich wieder freizugeben, ist dies über diesen Ansatz nicht möglich.

=== device
`/dev` ist der klassische UNIX-Weg, um auf die Peripherie zuzugreifen. Das entsprechende Gerät wird hierbei logisch als eine binäre Datei realisiert, was den entsprechenden Overhead eliminiert, den es braucht, um ASCII-kodierte Dateien in Binärformat zu parsen. Der Inhalt der Datei repräsentiert hierbei den eigentlichen Datenfluss für das Gerät.

Eine Besonderheit bei diesem Subsystem ist es, dass durch #gls("ioctl") @glibc_ioctl_h, zusätzlich zu den bereits bekannten Operationen `open()`, `close()`, `read()`, `write()` @glibc_fcntl_h,
auch komplexere Informationen für die entsprechende Transaktion mitgeliefert werden können.

#pagebreak()
#render-snippet("open")
#render-snippet("ioctl")

Die Vorteile dieses Ansatzes sind, dass die Funktion #gls("ioctl") es ermöglicht sowohl ein Command-Code, sowie optional einen Zeiger zu entsprechenden Nutzdaten für die Transaktion zu übergeben. Eine entsprechende Schnittstelle kann somit eine komplexe atomare Transaktion in nur einem Funktionsaufruf abbilden.

== I2C
=== Schnittstellen
Die eben beschriebene Transaktion über den #gls("i2c")-Bus wollen wir an dieser Stelle exemplarisch durchgehen. Die Schnittstellendefinition für #gls("i2c") findet sich in `linux/i2c.h` @linux_i2c_h sowie `linux/i2c-dev.h` @linux_i2c_dev_h.

#render-snippet("i2c-msg")

Das gezeigte `struct` enthält hierbei alle Informationen für einen #gls("i2c")-Zugriff. Hierzu zählen die Zieladresse, die Flags, die Länge der Nachricht, sowie ein Zeiger zu der Nachricht selber.

#render-snippet("i2c-rdwr")


Das Kernel-Modul `i2c-dev` ist häufig nicht standardmäßig geladen und muss per `modprobe` nachgeladen werden.
```bash
sudo modprobe i2c-dev
```

Da es sich bei den von dem #gls("i2c") bereitgestellten Geräten logisch um Dateien handelt, lassen sich diese einfach per `ls` und `grep` finden.

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

Die Schnittstellendefinition für #gls("gpio") findet sich in `linux/gpio.h` @linux_gpio_h. Die API für #gls("gpio") unterteilt ein #gls("gpio")-Device in `chip` und `line`.
#render-snippet("gpio-req")
Die API für die Request an eine Line des #gls("gpio")-Controllers ist in diesem Fall umfangreicher als die vorherige. Für den Kontext dieses Moduls sind jedoch vor allem die Flags interessant, mit welchen sich beispielsweise der Pull-Up-Widerstand oder der Edge-Detector konfigurieren lassen.
#render-snippet("gpio-event")
Die API für #gls("gpio")-Events gibt uns später die Möglichkeit, Informationen über Änderungen am Edge-Detector zu erhalten. Aus dem Event können wir beispielsweise den Zeitstempel sowie die Art des Events auslesen.

Das nun folgende C-Programm zeigt ein Beispiel, wie wir einen softwareseitigen Edge-Detector in Linux realisieren können.
#pagebreak()
#render-snippet("gpio")

Das Programm konfiguriert `line`-7 als Input und konfiguriert den Edge-Detector, sodass er steigende und fallende Taktflanken erkennt. Über `read()` (blockierender Aufruf) können wir nun auf ein neues Event warten. Der Vorteil im Vergleich zu der Laboraufgabe ist dabei, dass in diesem Fall nur der Thread des entsprechenden Prozesses blockiert wird, jedoch der Rest des Systems weiterarbeiten kann.

== ADC

Im Gegensatz zu #gls("i2c") und #gls("gpio"), die über `/dev/`-Gerätedateien mit binären #gls("ioctl")-Aufrufen angesprochen werden, nutzen #gls("adc")- und Sensordaten typischerweise das #gls("sysfs")-Interface. Das #gls("hwmon")-Subsystem stellt Sensordaten als ASCII-Textdateien unter `/sys/class/hwmon/` bereit.

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
Im Gegensatz zu dem #gls("i2c")- und #gls("gpio")-Beispiel wird hier kein #gls("ioctl") benötigt, sondern nur die Standard-POSIX-Funktionen `open()` aus `<fcntl.h>` und `read()` aus `<unistd.h>`.

#pagebreak()
= Aufgabe II - AT91SAM7-Timer Schnittstellenspezifikation

Die Timer-Counter-Einheit des AT91SAM7 kann auf jedem seiner 3 Channel eine Vielzahl an verschiedenen Operationen ausführen. Für die meisten dieser Betriebsmodi ist es nötig, den Chip an mehreren Stellen zu konfigurieren. Aus diesem Grund wird in diesem Fall die Schnittstelle über `/dev` für den Zugriff über #gls("ioctl") modelliert.

Der erste Schritt in dieser Überlegung ist es, die wichtigsten Use-Cases als Commands abzubilden. Für die Timer-Counter-Einheit sind es in diesem Fall:
- Event Counting
- Intervall Measurement
- Pulse Generation
- #gls("pwm")

Diese Betriebsmodi werden nun als Enum modelliert

#pagebreak()
#render-snippet("tc-mode")

Nun brauchen wir zusätzlich noch einen Mechanismus mit welchem sich die verschiedenen Betriebsmodie modellieren lassen. Analog zur #gls("i2c")-Schnittstelle, welche mehrere `i2c_msg` Strukturen in einer `i2c_rdwr_ioctl_data` Struktur bündelt, definieren wir für jeden Betriebsmodus eine spezialisierte Konfigurationsstruktur.

#pagebreak()
== Konfigurationsstrukturen

Für jeden der vier Betriebsmodi existiert eine eigene Konfigurationsstruktur, welche die mode-spezifischen Parameter enthält:

#render-snippet("tc-config")

Das struct `tc_event_count_config` konfiguriert das Zählen von externen Events. Der `threshold`-Wert ermöglicht eine Benachrichtigung, wenn eine bestimmte Anzahl an Events erreicht wurde.

Das struct `tc_interval_measure_config` dient der Messung von Zeitintervallen zwischen Flanken.

Das struct `tc_pulse_generation_config` konfiguriert die Erzeugung eines einzelnen Pulses mit konfigurierbarer Verzögerung und Pulsbreite.

Das struct `tc_pwm_config` ermöglicht die Konfiguration eines kontinuierlichen #gls("pwm")-Signals mit Periode und Tastverhältnis.

== Request-Struktur und ioctl-Kommandos

Analog zu #gls("gpio")'s `gpio_v2_line_request` definieren wir eine zentrale Request-Struktur, welche mehrere Kanal-Konfigurationen bündeln kann:

#render-snippet("tc-request")

Der zentrale #gls("ioctl")-Command `TC_GET_CHANNEL_IOCTL` folgt dem Muster von `GPIO_V2_GET_LINE_IOCTL`. Nach erfolgreichem Aufruf enthält das `fd`-Feld einen File-Deskriptor über welchen Events gelesen werden können.

== Event-Struktur

Für das Auslesen von Events und Messwerten definieren wir analog zu `gpio_v2_line_event`:

#render-snippet("tc-event")

== Beispiel: #gls("pwm")-Konfiguration

Das folgende Beispiel zeigt die Konfiguration einer #gls("pwm")-Ausgabe mit 1kHz und 50% Tastverhältnis:

#render-snippet("tc-example")

Der Ablauf entspricht dem #gls("gpio")-Beispiel: Nach dem Öffnen des Gerätes wird eine Request-Struktur konfiguriert und über #gls("ioctl") an den Treiber übergeben. Der zurückgegebene File-Deskriptor kann für weitere Commands und zum Auslesen von Events genutzt werden.

#pagebreak()
= Aufgabe III - I2C-Scheduling

Mehrere Threads, welche auf beschränke Ressourcen zugreifen wollen, ist ein Problem, welches essentiell für ein Betriebssystem ist. Linux verwendet dafür einen Scheduler, welcher jeder Task, eine Prioriät zuordnet und für alle anderen Tasks mittels eines #gls("mutex") den Zugriff auf diese Ressource verweigert.

#quote(
  block: true,
  attribution: [Linux Kernel Documentation @kernel_mutex_design],
)[In the Linux kernel, mutexes refer to a particular locking primitive that enforces serialization on shared memory systems, and not only to the generic term referring to 'mutual exclusion' found in academia or similar theoretical text books.]

Ein Mutex ist vereinfacht gesagt nur ein Schloss, welches den Zugriff auf eine Ressource einschränkt. Ist eine Task geblockt ist.

== Anwendungsfall: Starvation durch Display-Treiber
Man stelle sich folgendes Szenario vor: Ein Display und ein Temperatursensor sind beide über einen gemeinamen #gls("i2c")-Bus verbunden. Je nach Bildschirmwiederholfrequenz und Auflösung(wirkt sich auf die Größe des zu übertragenden Daten aus) kann es nun sein, dass der Display-Treiber den #gls("i2c")-Bus mehrfach pro Sekunde anfragt und somit die Leitung dauerhaft belegt.

Wenn die Task des Treiber für den Temperatursensor nun eine geringere Priorität hat kann es sein, dass diese Task nie Zugriff auf die Ressource erhält, weil die Tasks des #gls("i2c")-Treibers vorrang bekommen. Der Bus würde so diese Task eventuell nie, oder für eine Echtzeitanwendung zu selten, für die Übertragung des neuen Temperaturmesswertes freigebenen werden.

Dieses Problem heißt "Unbounded Priority Inversion".

Werfen wir nun einen Blick auf die Implementierung des #gls("i2c")-Treibers
#pagebreak()
#render-snippet("i2c-transfer")

Zu Begin einer jeden Transaktion wird `__i2c_lock_bus_helper()` aufgerufen, um den Bus für diese Transaktion zu blockieren @linux_i2c_core.

Werfen wir nun auch einen Blick auf diese Funktion:

#pagebreak()
#render-snippet("i2c-lock")

Das #gls("i2c")-Subsystem verwendet dabei das #gls("rtmutex")-System von Linux.


#quote(
  block: true,
  attribution: [Linux Kernel Documentation @kernel_rtmutex],
)[
  RT-mutexes extend the semantics of simple mutexes by the priority inheritance protocol.

  A low priority owner of a rt-mutex inherits the priority of a higher priority waiter until the rt-mutex is released. If the temporarily boosted owner blocks on a rt-mutex itself it propagates the priority boosting to the owner of the other rt_mutex it gets blocked on. The priority boosting is immediately removed once the rt_mutex has been unlocked.

  This approach allows us to shorten the block of high-prio tasks on mutexes which protect shared resources. Priority inheritance is not a magic bullet for poorly designed applications, but it allows well-designed applications to use userspace locks in critical parts of an high priority thread, without losing determinism.
]

Der #gls("rtmutex") implementiert zusätzlich *Priority Inheritance*: Wenn ein hochpriorer Task auf einen #gls("mutex") wartet, wird die Priorität des haltenden Tasks temporär angehoben. Dies verhindert *Priority Inversion*, bei der ein mittelpriorer Task den hochprioren indirekt blockieren könnte.

Der #gls("i2c")-Treiber verwendet keinen eigenen Scheduling-Algorithmus, sondern verlässt sich vollständig auf den #gls("rtmutex"), um zu verhindern, dass das Tasks mit mittlerer Priorität ausgehungert werden.

#pagebreak()
// ============================================
// Appendex
// ============================================
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

