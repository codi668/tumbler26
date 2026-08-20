# Tumbler26

Balancier-Firmware für einen zweirädrigen Roboter auf der eigenen Platine
**tumlerV2** (ESP32-WROOM-32E, TB6612FNG, MPU6050, Radencoder, 8× WS2812).

Der Roboter hält aufrecht die Balance, behält dabei seine Blickrichtung und
kann an seinen Ausgangspunkt zurückkehren. Bedient wird er über eine
Weboberfläche im WLAN, über die serielle Konsole oder den Taster.

## Aufbau

| Datei | Inhalt |
|---|---|
| `include/config.h` | Pinbelegung, Zeitraster, Sturzgrenzen |
| `src/main.cpp` | IMU-Treiber, Filter, die drei Regler, Kommandozeile |
| `src/encoder.cpp` | Radencoder über die PCNT-Hardware (Vierfachauswertung) |
| `src/leds.cpp` | Statusanzeige auf dem WS2812-Ring |
| `src/webui.cpp` | WLAN, Webserver, WebSocket, eingebettete Bedienseite, OTA |

## Regelung

Drei Regler, die übereinander liegen:

```
   Position ──[ PI ]──► Sollwinkel-Versatz (max ±4°)
                              │
   Winkel ────────────────────┴──[ PD 200 Hz ]──► PWM ──┬──► Motor A
   Drehrate ──────────────────────^ (D-Anteil)          │
                                                        │
   Gierwinkel ────────────────────[ PD ]──► ± Differenz ─┴──► Motor B
```

* **Balance (200 Hz)** — PD auf den Neigungswinkel aus einem
  Komplementärfilter. Der D-Anteil kommt direkt aus der gemessenen Drehrate,
  nicht aus einer numerischen Ableitung eines verrauschten Winkels.
* **Richtung** — die beiden Räder laufen nie exakt gleich, das summiert sich
  zu einer langsamen Drehung. Gegenmittel: Drehrate um die Hochachse (Gyro-Z)
  integrieren, Abweichung als *Differenz* auf die Räder. Der Wert läuft mit
  30 s Zeitkonstante gegen null zurück, sonst würde die Integrationsdrift eine
  eingebildete Drehung real wegdrehen.
* **Position** — PI auf Weg und Tempo aus den Encodern, verschiebt den
  *Sollwinkel*: der Roboter lehnt sich in die Richtung, in die er fahren muss.
  Direkt aufs PWM addiert würde derselbe Wert gegen die Balance arbeiten.
  **Ab Werk aus** (`VP=0 VI=0`) — erst den inneren Regler bestätigen, dann
  vorsichtig zuschalten.

`minPwm` spreizt den Stellbereich auf `minPwm..255`: unter etwa 50/255 bewegt
sich der Roboter unter seinem Eigengewicht überhaupt nicht, kleine Korrekturen
blieben sonst wirkungslos.

## Weboberfläche

Erreichbar unter `http://tumbler-mini.local/` bzw. der IP im Heimnetz; ohne
WLAN spannt der Roboter den Accesspoint `Tumbler` (Passwort `balance123`) auf,
dann `http://192.168.4.1/`.

Die Seite steckt als Zeichenkette im Programmspeicher — es muss kein
Dateisystem getrennt hochgeladen werden. Sie zeigt Neigung, Richtung, PWM,
Weg und Sollwinkel-Versatz, dazu einen laufenden Verlauf. Alle Parameter sind
als Schieberegler einstellbar und wirken sofort. Telemetrie läuft mit 10 Hz
über einen WebSocket, Kommandos denselben Weg zurück.

Updates gehen über WLAN: `pio run -e ota -t upload` — beim Balancieren fällt
das USB-Kabel ständig aus oder steht im Weg.

## Erste Inbetriebnahme

```
cp include/secrets.h.example include/secrets.h     # eigenes WLAN eintragen
pio run -t upload && pio device monitor
```

1. Beim Start wird der Gyro genullt — Roboter dabei **ruhig und aufrecht** halten.
2. Aufrecht hinstellen, `ZERO`, dann `START` (oder Taster).
3. Fährt er in Fallrichtung *weg* statt dagegen: `SIGN=1`.
   Dreht er sich beim Korrigieren stärker weg statt zurück: `YSIGN=1`.
   Beides sieht man nur beim Hinschauen — aus der Telemetrie geht die
   Drehrichtung des Rades gegenüber der Kipprichtung nicht hervor.
4. `SAVE` legt die Werte im Flash (NVS) ab.

## Kommandos

`START` `STOP` `ZERO` `HOME` `STATUS` `T` `SAVE` `LOAD` `?`
Balance `P=` `I=` `D=` `MINPWM=` `TRIM=` `SIGN=` ·
Richtung `YP=` `YD=` `YSIGN=` · Position `VP=` `VI=`

## Statusanzeige

| Farbe | Bedeutung |
|---|---|
| blau, langsam atmend | aus |
| gelb pulsierend | wartet darauf, dass der Roboter aufrecht steht |
| grün → rot | balanciert, Farbe zeigt den Winkelfehler |
| rot blinkend | umgefallen, Motoren aus |

## Sicherheit

Ab 35° Neigung gehen die Motoren aus (`STBY` low). Vor dem Start wartet der
Regler, bis der Roboter näher als 5° am Gleichgewicht steht — aus starker
Schräglage kann sich kein Balancierer mehr fangen.
