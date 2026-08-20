# Tumbler26

Balancier-Firmware für einen zweirädrigen Roboter auf der Platine **tumlerV2**
(ESP32-WROOM-32E, TB6612FNG, MPU6050, Radencoder, 8× WS2812).

Er hält aufrecht die Balance, richtet sich aus einer Schräglage selbst auf,
behält seine Blickrichtung, fährt geradeaus und kann an seinen Ausgangspunkt
zurückkehren. Bedient wird er über eine Weboberfläche im WLAN, die serielle
Konsole oder den Taster.

---

# Inbetriebnahme

## 1. Was du brauchst

* [PlatformIO](https://platformio.org/) (VS-Code-Erweiterung oder `pio` als CLI)
* ein USB-Kabel — **nur für das erste Aufspielen**, danach geht alles über WLAN
* unter Linux: dein Benutzer muss in der Gruppe `dialout` sein, sonst ist
  `/dev/ttyUSB0` nicht beschreibbar:

```bash
sudo usermod -aG dialout $USER      # danach ab- und wieder anmelden
```

## 2. WLAN eintragen

Die Zugangsdaten liegen **nicht** im Repository. Lege sie einmalig an:

```bash
cp include/secrets.h.example include/secrets.h
```

Dann `include/secrets.h` öffnen und dein Netz eintragen:

```c
#define WIFI_SSID   "MeinWLAN"
#define WIFI_PASS   "MeinPasswort"
```

Die Datei steht in `.gitignore` und wird nie mitcommittet.

> **Der ESP32 kann nur 2,4 GHz.** Hat dein Router für 5 GHz eine eigene SSID,
> musst du die 2,4-GHz-SSID eintragen.

**Findet er das Netz nicht**, spannt er nach 8 Sekunden einen eigenen
Accesspoint auf:

| | |
|---|---|
| Netz | `Tumbler` |
| Passwort | `balance123` |
| Adresse | http://192.168.4.1/ |

Damit kommst du auch ohne Router an ihn heran. Beides — Netzname des
Accesspoints und Hostname — steht in `include/config.h`.

## 3. Aufspielen

Beim ersten Mal über Kabel:

```bash
pio run -t upload
pio device monitor          # 115200 Baud
```

Im Monitor steht dann, wo er erreichbar ist:

```
WLAN verbunden, IP=192.168.188.95
```

Danach ist er unter dieser Adresse oder unter **http://tumbler-mini.local/**
erreichbar. Alle weiteren Updates gehen kabellos:

```bash
pio run -e ota -t upload
```

Sitzt er nicht mehr im selben Netz oder ändert sich die IP, kannst du sie in
`platformio.ini` unter `[env:ota]` bei `upload_port` direkt eintragen.

## 4. Erster Start

1. **Beim Einschalten ruhig und aufrecht halten** — der Roboter nullt in den
   ersten zwei Sekunden seinen Gyro. Wackelt er dabei, ist der Nullpunkt
   verschoben und er driftet später weg. Mit `ZERO` lässt sich das jederzeit
   wiederholen.
2. Hinstellen, **START** drücken (Knopf auf der Seite oder der Taster am
   Gerät). Steht er schräg, schwingt er sich zuerst selbst auf.
3. Zwei Vorzeichen musst du am lebenden Objekt prüfen — **die stehen in keiner
   Messreihe, das sieht man nur beim Hinschauen:**

   | Beobachtung | Abhilfe |
   |---|---|
   | Er fährt in Fallrichtung *weg* und kippt schneller | Motorrichtung auf `+1` stellen (`SIGN=1`) |
   | Beim Korrigieren dreht er sich stärker weg statt zurück | Drehrichtung umschalten (`YSIGN=1`) |

4. Passt alles: **SAVE** drücken. Ohne das sind alle Werte nach dem nächsten
   Reset wieder weg.

## 5. Einstellen

Am schnellsten geht es über die Schieberegler auf der Seite, mit dem
Verlaufsdiagramm daneben. Reihenfolge, die sich bewährt hat:

1. **Kp** hochziehen, bis er sicher steht, aber noch nicht zittert
2. **Kd** dagegen, bis das Zittern weg ist
3. **minPwm** so klein wie möglich — unter etwa 50/255 bewegt er sich unter
   seinem Eigengewicht sonst gar nicht
4. **trim** so, dass er nicht dauerhaft in eine Richtung driftet
5. erst dann die äußeren Regler (`VP`/`VI` für Position) vorsichtig zuschalten

Oder du drückst **AUTO-ABSTIMMUNG** und lässt ihn das selbst machen — er
arbeitet fünf Stufen durch (Balance → minPwm → Richtung → Position →
Feinschliff), rund 15 Minuten, in denen er balancieren muss. Fällt er, richtet
er sich selbst wieder auf und macht weiter. Danach **SAVE** nicht vergessen.

Dasselbe gibt es als PC-Skript, wenn du mitlesen willst:

```bash
pip install websocket-client
python3 autotune.py --dry                    einmal messen, nichts ändern
python3 autotune.py --rounds 4 --save        vollständig, Ergebnis in den Flash
```

---

# Weboberfläche

Zeigt Neigung, Richtung, PWM, Weg und Sollwinkel-Versatz, dazu einen
laufenden Verlauf über 24 Sekunden. Alle Parameter sind als Schieberegler
einstellbar und wirken sofort.

| Bereich | Inhalt |
|---|---|
| Knöpfe | START · STOP · ZERO · HOME · SAVE |
| Fahren | Tempo, ▼ zurück / HALT / vor ▲, ↺ links / gerade / rechts ↻ |
| Balance | Kp, Ki, Kd, minPwm, trim, Motorrichtung |
| Richtung halten | YP, YD, Drehrichtung |
| Position halten | VP, VI |
| Aufschwingen | UPPWM (Schwung), UPMAX (größter Winkel) |
| Was er tut | Protokoll: empfangene Befehle, Stürze, Gyro-Nullpunkte |

Die Seite steckt als Zeichenkette im Programm — es muss kein Dateisystem
getrennt hochgeladen werden.

## Kommandos

Dieselben Befehle gehen über die serielle Konsole und über die Weboberfläche.
`?` zeigt die Liste.

```
START  STOP  ZERO  HOME  STATUS  T  SAVE  LOAD
Balance     P=  I=  D=  MINPWM=  TRIM=  SIGN=
Richtung    YP=  YD=  YSIGN=
Position    VP=  VI=
Fahren      FWD=<Impulse/s>  TURN=<Grad/s>  HALT  DP=  DI=
Aufschwingen UPPWM=  UPMAX=
Abstimmung  TUNE (starten/beenden)  TUNEOFF (abbrechen)
```

`TURN=` ist eine **Drehrate**, kein fester Winkel: er dreht weiter, bis
`TURN=0` kommt.

## Statusanzeige

| Farbe | Bedeutung |
|---|---|
| blau, langsam atmend | aus |
| gelb pulsierend | wartet darauf, dass er aufrecht steht |
| violett blinkend | schwingt sich gerade auf |
| grün → rot | balanciert, Farbe zeigt den Winkelfehler |
| rot blinkend | umgefallen, Motoren aus |

---

# Wenn etwas nicht geht

**Die Seite lädt nicht.**
`http://` mit angeben — Browser erzwingen sonst oft HTTPS, das kann der ESP32
nicht. Und das Gerät muss im selben Netz sein; über Mobilfunk oder ein
Gast-WLAN kommst du nicht an ihn heran.

**`pio run -e ota -t upload` bricht bei 0 % ab.**
Die Partitionstabelle hat keine zwei App-Bereiche. `platformio.ini` muss
`board_build.partitions = min_spiffs.csv` enthalten — `huge_app.csv` hat nur
einen einzigen und kann grundsätzlich kein OTA. Umstellen und **einmal über
Kabel** flashen.

**Er zittert im Stand.**
Kd zu klein oder Kp zu groß. Im Verlaufsdiagramm wechselt die PWM-Kurve dann
ständig das Vorzeichen.

**Er steht, driftet aber langsam in eine Richtung.**
`trim` nachstellen — das ist der Winkel, bei dem sein Schwerpunkt wirklich
über der Achse liegt, und der ist selten genau null.

**Er reagiert erst gar nicht, kippt dann plötzlich.**
`minPwm` zu klein: kleine Korrekturen überwinden die Standreibung nicht mehr.

**Er dreht sich beim Balancieren im Kreis.**
`YSIGN` steht falsch herum, siehe Tabelle oben.

**Beim Booten startet er von selbst.**
Der Taster an GPIO2 ist **aktiv HIGH** (externer Pulldown, weil GPIO2
Strapping-Pin ist). Mit `INPUT_PULLUP` liest er dauerhaft „gedrückt". In der
Firmware steht deshalb bewusst `pinMode(PIN_BUTTON, INPUT)`.

---

# Wie es innen funktioniert

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
  nicht aus der Ableitung eines verrauschten Winkels.
* **Richtung** — die beiden Räder laufen nie exakt gleich, das summiert sich zu
  einer langsamen Drehung. Die Drehrate um die Hochachse wird integriert und
  als *Differenz* auf die Räder gelegt. Der Wert läuft mit 30 s Zeitkonstante
  gegen null zurück, sonst würde die Integrationsdrift eine eingebildete
  Drehung real wegdrehen.
* **Position / Fahren** — beides verschiebt den *Sollwinkel*: der Roboter lehnt
  sich in die Richtung, in die er fahren soll. Dieselbe Größe direkt aufs PWM
  gegeben würde gegen die Balance arbeiten. Gebremst wird ebenso — er lehnt
  sich gegen die Fahrtrichtung, bis die Räder wirklich stehen.
* **Aufschwingen** — die Räder fahren kräftig in die Kipprichtung, damit die
  Aufstandsfläche unter den Schwerpunkt wandert. Wirkrichtung wie beim
  Balancieren, deshalb aus `SIGN` abgeleitet.

`minPwm` spreizt den Stellbereich auf `minPwm..255`.

## Dateien

| Datei | Inhalt |
|---|---|
| `include/config.h` | Pinbelegung, Zeitraster, Sturz- und Aufschwinggrenzen |
| `include/secrets.h` | WLAN-Zugangsdaten (lokal, nicht im Repo) |
| `src/main.cpp` | IMU-Treiber, Filter, die Regler, Kommandozeile |
| `src/encoder.cpp` | Radencoder über die PCNT-Hardware (Vierfachauswertung) |
| `src/autotune.cpp` | Abstimmung im Roboter, als Zustandsautomat |
| `src/leds.cpp` | Statusanzeige auf dem WS2812-Ring |
| `src/webui.cpp` | WLAN, Webserver, WebSocket, Bedienseite, OTA, Protokoll |
| `autotune.py` | dieselbe Abstimmung als PC-Skript zum Mitlesen |

## Sicherheit

Ab 35° Neigung gehen die Motoren aus — allerdings nur, *während* er
balanciert, sonst könnte er sich nie aufschwingen. Vor der Übernahme durch den
Regler muss er näher als 5° am Gleichgewicht stehen (9°, wenn er gerade
hochschwingt und ohnehin Schwung hat).
