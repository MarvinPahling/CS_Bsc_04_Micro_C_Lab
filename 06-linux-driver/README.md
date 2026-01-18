# Linux Gerätetreiber

Nicht nur in embedded Systemen, sondern auch in PC's und Servern ist Peripherie enthalten. Im Gegensatz zu embedded Systemen wird hier jedoch die Peripherie nicht direkt angesprochen, sondern für jede Peripherieeinheit existiert ein Gerätetreiber. Damit nicht jeder Gerätetreiber seine eigene Schnittstelle zum Anwender bereitstellt, nutzen die Gerätetreiber als Schnittelle zum Anwender gemäß dem Unix Grundsatz "Alles ist eine Datei" das Dateisystem als Schnittstelle.

Traditionell erfolgt der Zugriff auf die Gerätetreiber mittels Dateien, welche typischerweise im /dev Verzeichnis vorhanden sind. Diese Dateien beinhalten selbst nicht den Gerätetreiber, sondern vorrangig eine major und minor Nummer (siehe auch https://en.wikipedia.org/wiki/Device_file). Die Gerätetreiber selbst registrieren sich beim Kernel auf eine spezifische Major Nummer, so dass der Kernel die Zugriffe auf diese Dateien an diese Gerätetreiber weiterleitet. Über die Minor Nummer kann der Gerätetreiber dann mehrere 'identische' Geräte verwalten.

Ergänzend zu der traditionellen Methode können sich Gerätetreiber auch noch in das virtuelle Dateisystem sysfs einhängen (zu finden unter /sys), in welcher sie für jede Funktionalität eine eigene virtuelle Datei bereitstellen. Für einen intuitiven Zugriff sind diese Dateien String und nicht Binäre basiert aufgebaut, so dass bspw. ein Lesen von solch einer Datei den Status als lesbaren Text zurückgibt.

Zur Darstellung einer Unabhängigkeit von der Realisierung der Peripherie wurden für bestimmte Geräteklassen auch die Aufrufkonventionen vorgeschrieben. Egal wie bspw. der UART realisiert ist, die Schnittstelle zum Anwender ist identisch. Auch ist es vorgesehen, dass am Beispiel von Soundtreibern die identische API gleichermaßen den Sound über die lokale Soundkarte, über eine an den USB angeschlossene Soundkarte oder gar über Ethernet ausgibt.

Einige Peripheriegeräte sind zur exklusiven Nutzung ausgelegt (z.B. PWM-Treiber), so dass die Treiber einen Nutzerzähler mitführen müssen und bei doppelter Nutzung den Zugriff verweigern. Andere Peripheriegeräte sind für die gemeinsame Nutzung ausgelegt (z.B. Festplatte, I2C), so dass der Gerätetreiber für jeden Nutzer die anstehenden Aktionen zwischenspeichern und koordinieren muss.

Bedenken sie, dass wie bei realen Dateien alle Zugriffe synchron/blockierend sind. Die Dateioperationen kehren erst dann wieder zum Aufrufer zurück, wenn der geforderte Funktionalität ausgeführt wurde, resp. ein Fehler erkannt wurde.

## 1. Aufgabe

Beschreiben sie für nachfolgende Schnittstellen, wie der Zugriff/Konfiguration dieser Schnittstellen funktioniert (über welche Schnittstelle werden diese bedient, welche Zugriffe bewirken was, wie sieht ein typischer Zugriff aus)

- I2C
- GPIO
- ADC

Timer-Einheiten stellen in der Regel keine Funktionalitäten über das Datei-System bereit, da sie eher ähnlich dem I2C-Bus von anderen internen Treibern intern genutzt werde (bspw. Geschwindigkeitsbestimmung nutzt u.A. TimerEinheit, zyklische Funktionsaufrufe werden über Timer-IRQ's dargestellt, ...). Ähnlich wie beim I2C Bus stellen sie somit ihre Funktionalität nur kernelintern über Funktionsaufrufe bereit.

## 2. Aufgabe

Überlegen sie eine Schnittstellespezifikation, wie möglichst viele Funktionalitäten des AT91SAM7-Timers über eine der beiden Datei-Schnittstellen bereitgestellt werden können.

I2C Scheduling (gilt auch für anderen Bussysteme wie USB,…)

Am I2C Bus können mehrere Devices angeschlossen sein. Aus Sicht des Gerätetreibers sind somit 2 Gerätetreiber zuständig. Einer für den I2C-Bus zum Senden und Empfangen von I2C Nachrichten und einer für das am I2C-Bus angeschlossene Device, wobei der Anwender dann nur mit dem Gerätetreiber des Devices direkt interagiert. Wenn jetzt ein Anwender intensiv mit 'seinem' Device interagiert, so besteht die Gefahr, dass andere Devices 'ausgehungert' werden. Der I2C-Treiber beinhaltet somit nicht nur den Treiber zum Senden und Empfangen von Nachrichten, sondern sorgt auch für eine faire Zuteilung.

## 3. Aufgabe

Bearbeiten sie folgende Punkte zum Thema I2C-Scheduling

Skizzieren sie einen Anwendungsfall, bei welchen ein Devicetreiber (z.B. Displaytreiber) einen anderen Devicetreiber aushungern kann.
Überlegen und begründen sie eine Möglichkeit, wie dies vermieden werden kann

Hinweis:

Bei Gerätetreibern wird zwischen Block Device und Character Device unterschieden. Die im Rahmen der Vorlesung behandelten Peripherieelemente sind typischerweise Character Devices, so dass sie nur diese betrachten müssen
Für einige Treiber wurden zum vereinfachten Zugriff auf die Gerätedateien Wrapper entwickelt (z.B. i2c_smbus_read_word_data()). Diese max. am Rande beschreiben. Es gilt den direkten Zugriff zu berücksichtigen
Ebenso bitte nicht auf das Thema Treiber Installation (z.B. insmod, modprobe), installierte Treiber überprüfen (z.B. lsmod) oder Hardwareabfrage (z.B. lsusb, lspci, lshw) eingehen

## Empfohlene Literatur:

- [Harshvardhan Mishra; "Peripheral Management on Linux; UART, GPIO, ADC, SPI, I2C", iotbyhvm](https://iotbyhvm.ooo/peripheral-management-on-linux-uart-gpio-adc-spi-i2c)
- [Wolf, Jürgen; "Linux-UNIX-Programmierung, Kapitel 5 Devices"; Rheinwerk Verlag;](https://openbook.rheinwerk-verlag.de/linux_unix_programmierung/Kap05-000.htm#RxxKap05000040001591F023100)
- [Prof. Dr. Margarita Esponda; "Ein-/Ausgabe-Systeme Teil 2"; Freie Universität Berlin](https://www.inf.fu-berlin.de/lehre/WS11/OS/slides/OS_V23_IO_Ger%C3%A4teverwaltung_Teil_2.pdf)
- [Diverse; "The Linux Kernel. Character Device drivers"; The kernel development community;](https://linux-kernel-labs.github.io/refs/heads/master/labs/device_drivers.html)
- [Diverse; "The Linux Kernel. I2C Device Interface"; The kernel development community;](https://www.kernel.org/doc/html/v5.4/i2c/dev-interface.html)
- [Akshay; " Device-Drivers/Linux_char_driver_for_ADC/userspace.c"; GitHub;](https://github.com/axay03/Device-Drivers/blob/master/Linux_char_driver_for_ADC/userspace.c)
- [Unbekannt; "Control GPIO pins using `sysfs`"; linuxbash.sh;](https://www.linuxbash.sh/post/control-gpio-pins-using-sysfs-eg-sysclassgpioexport)

## Weiterführende Literatur, für Interessierte, die selbst einen Linux Gerätetreiber entwickeln wollen:

- [Diverse; "Linux Device Driver Tutorial Part 6"; EmbeTronicX;](https://embetronicx.com/linux-device-driver-tutorials/)
- [U. Graf, A. Kalberer, M. Züger; "Einführung in die Linux Treiberentwicklung"; Ostschweizer Fachhochschule;](https://wiki.bu.ost.ch/infoportal/_media/software/linux/treiber_entwicklung/linuxtreiberentwicklung_v2.0.pdf)
