Korrektur

    USART im Synchronen Mode betreiben
    Auf der Adapterplatine fehlt ein PullDown Widerstand, so dass der Taster nicht genutzt werden kann. Daher dann doch bitte den orangen Taster nutzen

Grundlage

Der NeoPixel (WS2812) ist ein Farb-LED mit eingebauten Mikrocontroller. Die darzustellende Farbe wird über ein serielle Schnittelle 'eingetaktet'. Für jede Farbe sind 8-Bit vorgesehen, so dass ein Datenrahmen aus 3*8-Bit=24-Bit besteht.

NeoPixel

Der Signalfluss zum Ansteuern der 8 NeoPixel sieht wie folgt:
- Das '0' und '1' Signal wird über den USART0 Transmitter erzeugt
- Die GPIO Einheit muss passend konfiguriert werden, so dass diese das TXD Signal an den Ausgang PA6 bereitstellt
- An SensorPort 4 des NXT Bausteins ist ein RS485 Treiber angebunden, welcher das TXD Signal negiert und an SensorPort 4 bereitstellt. Ergänzend ist diese Leitung noch mit PA30 des Prozessors verbunden. Die Aktivierung des RS485-Treibers erfolgt über PA7, welche dementsprechend auf Ausgabe zu setzen ist.
- Dieses negierte Signal wird über die Adapterplatine an den ersten NeoPixel durchgereicht. Ergänzend versorgt die Adapterplatine über das angeschlossene Steckernetzteil die NeoPixel Platine mit Energie.


Die Taste auf der Adapterplatine ist mit den nicht negierten RS485 Signal verbunden. Während der Datenausgabe an die NeoPixel wird folglich  diese Leitung durch den RS485-Treiber getrieben. Zur Nutzung als Eingabe muss zwischen den Datenübertragungen an die NeoPixel der RS485-Treiber deaktivert werden, so dass nun das Tastensignal über PA2 eingelesen werden kann. Da auf der Platine kein PullUp Widerstand für den Taster vorhanden ist, muss der GPIO-Interne PullUp aktiviert werden.
Damit auf der TXD Leitung kein Pegelwechsel durch Deaktivierung des RS485-Treibers entsteht, sollten PA30 passend konfiguriert sein!

Ergänzend ist auf der Platine ein Poti verbaut, welches am analogen Eingang des NXT-Ports liegt (einzulesen mit 'nxt_avr_get_sensor_adc_raw(0..3)). Auflösung des AD-Wandlers: 10-Bit

Da die Stromaufnahme der 8 NeoPixel größer als der vom NXT bereitgestellten Strom ist, ist ein externes Netzteil zur Energieversorgung notwendig.

Aufgabe

Die NeoPixel sollen über den USART angesteuert werden. Dementsprechend ist die Platine am Sensor-Port 4 des NXT anzuschließen.

Erzeugen sie auf Basis der USART Peripherieeinheit das notwendige Signal zum Ansteuern aller 8 NeoPixel.  Die Ansteuerung des USART soll einerseits händisch per Beschreibung des THR Registers und anderseits automatisch über den DMA Datentransfer (hier PDC) erfolgen. Die Auswahl des Mode erfolgt über ein Makros SEND_THRREGISTER/SEND_PDC in der main-Datei.

Über den Taster auf der Adapterplatine sollen folgende Modis 'durchgeschaltet' werden

    Lauflicht, d.h. es leuchtet nur eine LED (Farbe frei wählbar). Mit Ablauf der per Poti einstellbaren Zeit wird im Uhrzeigersinn die nächste LED angesteuert
    Lauflicht mit 'Nachleuchten'. Ergänzend zum vorherigen Mode leuchtet nicht nur eine (Front)LED sondern auch die 4 benachbarten LED's wobei mit dem Abstand zur FrontLED die Helligkeit abnimmt.
    Farbverlauf, d.h. alle LED's haben die identische Farbe. Mit Ablauf der per Poti einstellbaren Zeit wird zur nächsten Farbe weitergeschaltet.

TIPP: Die NeoPixel sind sehr hell. Es empfiehlt sich daher, ein Blatt Papier auf die Pixel zu legen oder diese mit einem Kreppband abzukleben.

Hinweis:

    Benutzen sie zwingend die Makros aus AT91SAM7S64.h für den Zugriff auf die einzelnen Bits der Peripherie