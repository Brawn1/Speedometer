# Hot Wheels Speedometer

Arduino Nano basierter Geschwindigkeitsmesser fuer Hot Wheels Strecken. Zwei IR-Sensoren im Abstand von 10 cm messen die Durchfahrtszeit und ein Servomotor zeigt die Geschwindigkeit auf einer analogen Skala an.

## Features

- Geschwindigkeitsmessung ueber zwei IR-Sensoren (10 cm Abstand)
- Analoge Tacho-Anzeige mit Servomotor
- Einstellbarer Servo-Ausschlag ueber 4 DIP-Switches (Multiplikator 1.0x - 5.0x)
- Batterieueberwachung mit Rot/Gruen LED (3.7V Li-Ion Akku)
- Sensor-Status LEDs (zeigen Sensor 1/2 Ausloesung an)
- Startup-Melodie ueber Piezo-Lautsprecher
- Serial Monitor Ausgabe fuer Debugging (9600 Baud)

## Hardware

### Komponenten

| Komponente | Anzahl |
|---|---|
| Arduino Nano | 1 |
| IR-Sensor (digital) | 2 |
| Servomotor | 1 |
| Piezo-Lautsprecher | 1 |
| 4-fach DIP-Switch | 1 |
| LED gruen (Batterie OK) | 1 |
| LED rot (Batterie niedrig) | 1 |
| LED (Sensor 1 Status) | 1 |
| LED (Sensor 2 Status) | 1 |
| Widerstand 10k Ohm | 2 |
| 3.7V Li-Ion Akku | 1 |

### Pin-Belegung

| Pin | Funktion |
|---|---|
| D2 | IR-Sensor 1 (Start) |
| D3 | IR-Sensor 2 (Ziel) |
| D4 | LED gruen (Batterie OK) |
| D5 | LED rot (Batterie niedrig) |
| D6 | LED Sensor 1 |
| D7 | LED Sensor 2 |
| D8 | Piezo-Lautsprecher |
| D9 | Servomotor |
| D10 | DIP-Switch 1 (Wert 1) |
| D11 | DIP-Switch 2 (Wert 2) |
| D12 | DIP-Switch 3 (Wert 4) |
| D13 | DIP-Switch 4 (Wert 8) |
| A0 | Batteriespannung (Spannungsteiler) |

### Spannungsteiler (Batteriemessung)

Die Batteriespannung wird ueber einen Spannungsteiler mit 2x 10k Ohm Widerstaenden gemessen. Der Spannungsteiler halbiert die Batteriespannung, damit sie im ADC-Messbereich des Arduino liegt.

```
Batterie+ ---[R1 10k]---+---[R2 10k]--- GND
                         |
                         A0
```

### DIP-Switch Anschluss

Die DIP-Switches nutzen die internen Pull-Up Widerstaende des Arduino. Jeder Schalter verbindet den Pin mit GND wenn aktiv.

```
Pin D10-D13 ---[DIP-Switch]--- GND
```

## DIP-Switch Multiplikator-Tabelle

Die 4 DIP-Switches steuern den Servo-Ausschlag mit 16 Stufen:

| DIP 4-3-2-1 | Wert | Multiplikator |
|---|---|---|
| 0000 | 0 | 1.0x |
| 0001 | 1 | 1.27x |
| 0010 | 2 | 1.53x |
| 0011 | 3 | 1.8x |
| 0100 | 4 | 2.07x |
| 0101 | 5 | 2.33x |
| 0110 | 6 | 2.6x |
| 0111 | 7 | 2.87x |
| 1000 | 8 | 3.13x |
| 1001 | 9 | 3.4x |
| 1010 | 10 | 3.67x |
| 1011 | 11 | 3.93x |
| 1100 | 12 | 4.2x |
| 1101 | 13 | 4.47x |
| 1110 | 14 | 4.73x |
| 1111 | 15 | 5.0x |

## Batterieueberwachung

| Spannung | Status | LED |
|---|---|---|
| > 3.6V | OK | Gruen |
| 3.4V - 3.6V | Niedrig | Rot |
| < 3.4V | Schwach | Rot |

Die Batterie wird alle 30 Sekunden geprueft. Die Messung erfolgt mit 16-fachem Oversampling fuer stabile Werte und wird nur durchgefuehrt wenn der Servo nicht aktiv ist.

## Messablauf

1. Einschalten: Startup-Melodie, Servo faehrt auf Startposition, Batterie wird geprueft
2. Auto faehrt durch Sensor 1: LED Sensor 1 leuchtet, Messung startet
3. Auto faehrt durch Sensor 2: LED Sensor 2 leuchtet, Geschwindigkeit wird berechnet
4. Servo zeigt Geschwindigkeit an (2 Sekunden)
5. Servo faehrt zurueck, beide Sensor-LEDs erlischen
6. Bereit fuer naechste Messung

Bei Timeout (Sensor 2 nicht innerhalb von 4 Sekunden ausgeloest) wird die Messung abgebrochen.

## Projektstruktur

```
Speedometer/
├── Code/speedometer/speedometer.ino   # Arduino Sketch
├── Kicad/                             # KiCad Schaltplan
├── Case/                              # Gehaeuse (FreeCAD)
├── LICENSE
└── README.md
```

## Lizenz

Siehe [LICENSE](LICENSE).
