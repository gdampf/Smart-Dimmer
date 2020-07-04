![GitHub Logo](http://www.heise.de/make/icons/make_logo.png)

Guido Dampf basierend (fork) auf einer Veröffentlichung von Make!

Verbessertes Layout ohne Störsignale auf dem Sync, basierend auf ESP-12S (bessere Funktverbindung, weniger Pins) und komplett überarbeiteter Sketch.

***

# Smarter Dimmer

### mit ESP8266 (ESP-12), Make 1/2020 S. 56

![Picture](https://github.com/MakeMagazinDE/Smart-Dimmer/blob/master/iot_dimmer_v02.jpg)

Gerber-Files zur Platinenfertigung; Abmessungen der Platine 71 x 71 mm, 2-lagig 1,55mm FR4, Lötstopplack und Bestückungsdruck beidseitig. Eine Leerplatine wird auch im Heise-Shop verfügbar sein.

**Weitere Hinweise**

Bitte beachten Sie das gegenüber der im Heft abgedruckten Version leicht geänderte Schaltbild/Layout. Hinzugekommen sind R10 und R11, die für einen sicheren Start der Schaltung bei nicht angeschlossener Last sorgen.

Die Ringkerndrossel L1 wird mit doppelseitigem Schaumstoff-Klebeband auf TR1 befestigt. Zum Anschluss an die Platine können einige Wingungen abgewickelt werden, so dass längere Drahtenden (mit Isolierschlauch schützen) entstehen.

Für Q1 ist ein kleiner Kühlkörper erforderlich, der leicht aus einem Reststück Aluminium-Winkelprofil angefertigt werden kann. **Beachten Sie, dass der Kühlkörper Netzspannung führt!**

Weitere Informationen zu ESP-Modulen und deren Programmierung finden Sie hier: http://stefanfrings.de/esp8266/

Beiliegend ein modifiziertes Layout - das Original aus dem Heft hat größte Probleme mit Störimpulsen auf dem Sync-Signal, verursacht durch das Schalten der Last. Der Autor hatte ursprünglich ein anderes Layout getestet, bei dem das Sync-Signal vor dem Kondensator abgegriffen wird. An dieses ältere Layout, das nie veröffentlicht wurde (afaik), lehnt sich dieses hier an. Zusätzlich wurde der ESP12F durch einen ESP12S ersetzt, der mit weniger Anschlusspads auskommt, was die Platine etwas vereinfacht.

### Stückliste für modifiziertes Layout

	Halbleiter
	U1, U2  PC817 Optokoppler, DIL - U1 wahlweise PC815 (besser)
	U3  ESP-12S
	D1..D3  S1J Diode 600V/1A
	GL1 Gleichrichter 2A/400V
	Q1  IRF740
	ZD1 Zener 10V/400mW SMD MELF (Reichelt SMD ZF 10)
 
	Widerstände
	R1  VDR 275VAC, 360VDC, RM 7,5
	R2  220k 1/8W
	R5, R8  4k7 SMD 0805
	R3, R4, R6, R9  10k SMD 0805
	R10, R11 100k SMD 1206
	R12 220k SMD 1206

	Kodensatoren
	C1  220n/275V AC RM22,5 (Entstörkondensator Klasse X2, Reichelt FUNK 220n)
	C2  47µ/25V Elko 
	C3  100µ/10V Elko

	Sonstiges
	L1  Ringkerndrossel 100µH/2A
	FS1 Miniatur-Sicherung 2A flink, axial 3*10mm (z.B. bei Aliexpress)
	TR1 AC/DC-Wandler Meanwell IRM-03 3.3V
	PL1 Stiftleiste 6pol. RM 2,54
	PL2 Anschlussklemme 3pol. RM 5,08 (am besten gerade, d.h. Schrauben a.d. Seite)
	PL3 Stiftleiste 3pol. RM 2,54
	Ext TTP223 als Touch-Sensor

Der veränderte Schaltplan ist als LTSpice XVII Dokument beigelegt, dass so aber nicht simuliert werden kann, da der Trafo für 3,3V nicht vollständig als Modell definiert ist. Zur Simulation kann man ihn aber durch eine 3,3V-Spannungsquelle ersetzen.
