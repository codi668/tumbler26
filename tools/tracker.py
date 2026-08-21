#!/usr/bin/env python3
"""Mitschnitt: haengt sich an den Roboter und schreibt mit, was passiert.

Laeuft dauerhaft im Hintergrund und protokolliert in eine Datei, damit
hinterher nachvollziehbar ist, was eingegeben wurde und was der Roboter
daraufhin getan hat - auch wenn gerade niemand zugesehen hat.

Mitgeschrieben wird bewusst nicht jeder Telemetrierahmen (das waeren bei
10 Hz ueber Stunden Hunderttausende Zeilen), sondern:

  EVT     jedes Ereignis aus dem Roboter: empfangene Befehle, Stuerze,
          Aufschwingversuche, Gyro-Nullpunkte
  STATE   jeder Zustandswechsel (laeuft / wartet / gestuerzt / schwingt auf)
  DRIVE   jede Aenderung der Soll-Fahrt oder Drehrate
  PARAM   jede Parameteraenderung - auch die per Schieberegler
  STAT    alle 5 s eine Zusammenfassung, solange er balanciert
  CONN    Verbindungsauf- und -abbau

Aufruf:
    python3 tools/tracker.py [--host 192.168.188.95] [--out DATEI]
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

try:
    import websocket
except ImportError:
    sys.exit("Bitte 'pip install websocket-client' ausfuehren.")


F = {  # Feldnamen der Telemetrie, siehe webuiSendTelemetry()
    "angle": 1, "rate": 2, "pwm": 3, "run": 4, "req": 5, "fall": 6,
    "Kp": 7, "Ki": 8, "Kd": 9, "minPwm": 10, "trim": 11, "sign": 12,
    "yaw": 13, "steer": 14, "YP": 15, "YD": 16, "ysign": 17,
    "pos": 18, "v": 19, "bias": 20, "VP": 21, "VI": 22,
    "tune": 23, "stage": 24, "trial": 25, "cost": 26, "phase": 27,
    "swing": 28, "UPPWM": 29, "UPMAX": 30,
    "dist": 31, "dsoll": 32, "turn": 33, "DP": 34, "DI": 35,
    "shove": 36, "SHOVE": 37, "RK": 38, "RV": 39, "RMAX": 40,
}

# Diese Werte sind Einstellungen: aendert sich einer, wurde er verstellt.
PARAMS = ["Kp", "Ki", "Kd", "minPwm", "trim", "sign",
          "YP", "YD", "ysign", "VP", "VI", "DP", "DI", "UPPWM", "UPMAX",
          "SHOVE", "RK", "RV", "RMAX"]

STAT_EVERY = 5.0


class Tracker:
    def __init__(self, host, path):
        self.host = host
        self.fh = open(path, "a", buffering=1)   # zeilenweise, ueberlebt Absturz
        self.prev = None
        self.win = []
        self.win_start = time.time()

    def w(self, kind, text):
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.fh.write(f"{stamp}  {kind:6} {text}\n")

    # -- Auswertung eines Telemetrierahmens ---------------------------------
    def on_telemetry(self, p):
        try:
            cur = {k: p[i] for k, i in F.items()}
        except IndexError:
            return                                  # aeltere Firmware
        f = lambda k: float(cur[k])

        if self.prev is None:
            self.w("START", "Mitschnitt beginnt: " + self.fmt_params(cur))
            self.prev = cur
            return

        # Zustand
        keys = ("run", "req", "fall", "swing", "tune", "stage", "phase", "shove")
        if any(cur[k] != self.prev[k] for k in keys):
            self.w("STATE",
                   f"laeuft={cur['run']} angefordert={cur['req']} "
                   f"gestuerzt={cur['fall']} schwingt={cur['swing']} "
                   f"abstimmung={cur['tune']}({cur['phase']}) "
                   f"stoss={cur['shove']} "
                   f"| Winkel {f('angle'):+.2f} Rate {f('rate'):+.0f}")

        # Fahrt
        if cur["dsoll"] != self.prev["dsoll"] or cur["turn"] != self.prev["turn"]:
            self.w("DRIVE", f"Soll {f('dsoll'):+.0f} Impulse/s "
                            f"(ist {f('dist'):+.0f}) Drehrate {f('turn'):+.0f} Grad/s")

        # Einstellungen
        changed = [k for k in PARAMS if cur[k] != self.prev[k]]
        if changed:
            self.w("PARAM", "  ".join(f"{k}={cur[k]} (war {self.prev[k]})"
                                      for k in changed))

        # Laufende Zusammenfassung, nur waehrend er wirklich regelt
        if cur["run"] == "1":
            self.win.append((f("angle") - f("trim") - f("bias"), f("pwm"),
                             f("yaw"), f("pos")))
        now = time.time()
        if now - self.win_start >= STAT_EVERY:
            if len(self.win) > 3:
                errs = [x[0] for x in self.win]
                pwms = [abs(x[1]) for x in self.win]
                rms = (sum(e * e for e in errs) / len(errs)) ** 0.5
                self.w("STAT", f"Fehler eff {rms:.2f} Grad "
                               f"({min(errs):+.1f}..{max(errs):+.1f})  "
                               f"PWM im Mittel {sum(pwms)/len(pwms):.0f}  "
                               f"Richtung {self.win[-1][2]:+.1f} Grad  "
                               f"Weg {self.win[-1][3]:+.0f}")
            self.win = []
            self.win_start = now

        self.prev = cur

    def fmt_params(self, cur):
        return " ".join(f"{k}={cur[k]}" for k in PARAMS)

    # -- Hauptschleife ------------------------------------------------------
    def run(self):
        while True:
            try:
                ws = websocket.create_connection(f"ws://{self.host}/ws", timeout=10)
                ws.settimeout(15)
                self.w("CONN", f"verbunden mit {self.host}")
                while True:
                    parts = ws.recv().split(",")
                    if not parts:
                        continue
                    if parts[0] == "L":
                        # L,<ms seit Start>,<Text> - der Text darf Kommas haben
                        self.w("EVT", ",".join(parts[2:]))
                    elif parts[0] == "M":
                        self.w("EVT", "Meldung: " + ",".join(parts[1:]))
                    elif parts[0] == "T":
                        self.on_telemetry(parts)
            except KeyboardInterrupt:
                self.w("CONN", "Mitschnitt beendet")
                return
            except Exception as e:
                # Roboter neu gestartet, WLAN weg, OTA-Update - einfach warten.
                self.w("CONN", f"Verbindung weg ({type(e).__name__}), neuer Versuch in 3 s")
                self.prev = None
                time.sleep(3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.188.95")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "logs", "tumbler.log"))
    args = ap.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    print(f"schreibe nach {args.out}")
    Tracker(args.host, args.out).run()


if __name__ == "__main__":
    main()
