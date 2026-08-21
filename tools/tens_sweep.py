#!/usr/bin/env python3
"""Vorspannung ausmessen: ab welchem TENS dreht er sich?

Faehrt TENS stufenweise hoch und misst je Stufe, was passiert. Die
entscheidende Groesse ist nicht der Winkel, sondern die LENKUNG: solange die
Raeder halten, bleibt die Richtung stehen und der Gierregler hat nichts zu tun.
Sobald sie durchrutschen, dreht sich der Roboter, der Gierregler sieht einen
Richtungsfehler und haelt mit genau derselben gegengleichen Groesse dagegen -
seine Ausgabe waechst dann mit der Vorspannung mit. Genau dort ist die Grenze,
und ab da ist die Vorspannung nutzlos: sie wird vollstaendig weggeregelt.

Nebenbei wird mitgemessen, ob sie ueberhaupt etwas bringt: Winkelfehler
(Effektivwert) und mittlere Stellgroesse sollten sinken, wenn das
Getriebespiel wirklich herausgenommen wird.

Das Skript wartet von selbst, bis der Roboter balanciert - einfach starten,
dann den Roboter hinstellen und START druecken. Faellt er um, bricht es ab und
stellt TENS auf 0 zurueck.
"""

import argparse
import sys
import time

try:
    import websocket
except ImportError:
    sys.exit("Bitte 'pip install websocket-client' ausfuehren.")

I_ANGLE, I_PWM, I_RUN, I_FALL, I_TRIM = 1, 3, 4, 6, 11
I_YAW, I_STEER, I_BIAS = 13, 14, 20


class Conn:
    """WebSocket, der sich selbst wieder verbindet.

    Waehrend einer Messreihe wird der Roboter zwischendurch aus- und wieder
    eingeschaltet, neu geflasht oder er verliert kurz das WLAN. Das darf die
    Messung nicht abstuerzen lassen - sie soll einfach warten.
    """

    def __init__(self, host):
        self.host = host
        self.ws = None
        self.connect()

    def connect(self):
        while True:
            try:
                self.ws = websocket.create_connection(f"ws://{self.host}/ws", timeout=10)
                self.ws.settimeout(20)
                return
            except Exception:
                print("    warte auf den Roboter ...", flush=True)
                time.sleep(3)

    def send(self, cmd):
        try:
            self.ws.send(cmd)
        except Exception:
            self.connect()

    def frame(self):
        while True:
            try:
                p = self.ws.recv().split(",")
            except Exception:
                print("    Verbindung weg - warte auf den Roboter ...", flush=True)
                self.connect()
                continue
            if p[0] == "T" and len(p) > 41:
                return p


def wait_balancing(ws, first=False):
    """Wartet, bis er (wieder) ruhig balanciert."""
    print("    warte, bis er balanciert"
          + (" (hinstellen und START druecken)" if first else " - weiter geht es dann von selbst"),
          flush=True)
    steady = None
    while True:
        p = ws.frame()
        if p[I_RUN] == "1" and abs(float(p[I_ANGLE]) - float(p[I_TRIM])) < 3.0:
            steady = steady or time.time()
            if time.time() - steady > 2.0:
                return
        else:
            steady = None


def measure(ws, seconds):
    """Ein Zeitfenster mitschreiben und zusammenfassen.

    Bricht der Lauf ab, wird unterschieden, WARUM - ein Sturz ist ein
    Messergebnis, ein Druck auf den Taster nur eine Unterbrechung.
    """
    errs, pwms, steers, yaws = [], [], [], []
    end = time.time() + seconds
    while time.time() < end:
        p = ws.frame()
        if p[I_RUN] != "1":
            gefallen = p[I_FALL] == "1" or abs(float(p[I_ANGLE]) - float(p[I_TRIM])) > 20.0
            return "sturz" if gefallen else "aus"
        errs.append(float(p[I_ANGLE]) - float(p[I_TRIM]) - float(p[I_BIAS]))
        pwms.append(abs(float(p[I_PWM])))
        steers.append(abs(float(p[I_STEER])))
        yaws.append(float(p[I_YAW]))
    n = len(errs)
    return {
        "rms":   (sum(e * e for e in errs) / n) ** 0.5,
        "pwm":   sum(pwms) / n,
        "steer": sum(steers) / n,
        "yaw":   max(yaws) - min(yaws),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.188.95")
    ap.add_argument("--step", type=int, default=3)
    ap.add_argument("--max", type=int, default=45)
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--settle", type=float, default=1.5)
    args = ap.parse_args()

    ws = Conn(args.host)

    wait_balancing(ws, first=True)
    print("balanciert - Messung laeuft\n", flush=True)

    print(f"{'TENS':>5} {'Fehler eff':>11} {'PWM':>7} {'Lenkung':>9} {'Richtung':>10}")
    print("-" * 46, flush=True)

    rows, base = [], None
    try:
        for tens in range(0, args.max + 1, args.step):
            # Bis zu drei Anlaeufe je Stufe: wird er zwischendurch abgeschaltet
            # oder faellt er, wird die Stufe wiederholt statt alles zu verwerfen.
            for versuch in range(3):
                ws.send(f"TENS={tens}")
                time.sleep(args.settle)
                m = measure(ws, args.hold)
                if isinstance(m, dict):
                    break
                if m == "sturz":
                    print(f"{tens:5d}   umgefallen - Stufe wird wiederholt", flush=True)
                else:
                    print(f"{tens:5d}   abgeschaltet - Stufe wird wiederholt", flush=True)
                ws.send("TENS=0")
                wait_balancing(ws)
            if not isinstance(m, dict):
                print(f"\nStufe TENS={tens} dreimal misslungen - Messung beendet.")
                break
            rows.append((tens, m))
            if base is None:
                base = m
            print(f"{tens:5d} {m['rms']:9.2f}Grad {m['pwm']:7.0f} "
                  f"{m['steer']:9.1f} {m['yaw']:8.1f}Grad", flush=True)
    finally:
        ws.send("TENS=0")
        time.sleep(0.3)

    if not rows or base is None:
        return

    print("\n--- Auswertung ---")
    # Grenze: ab hier muss der Gierregler deutlich gegenhalten, die Raeder
    # rutschen also durch.
    limit = None
    for tens, m in rows:
        if tens and (m["steer"] > base["steer"] + 4.0 or m["yaw"] > base["yaw"] + 6.0):
            limit = tens
            break
    if limit:
        print(f"Ab TENS={limit} dreht er sich - brauchbar ist hoechstens "
              f"{limit - args.step}.")
    else:
        print(f"Bis TENS={rows[-1][0]} dreht er sich nicht. "
              f"Die Grenze liegt hoeher, mit --max weiter fahren.")

    usable = [r for r in rows if not limit or r[0] < limit]
    best = min(usable, key=lambda r: r[1]["rms"])
    print(f"Ruhigster Wert: TENS={best[0]} mit {best[1]['rms']:.2f} Grad "
          f"gegenueber {base['rms']:.2f} Grad ohne Vorspannung "
          f"({(best[1]['rms']/base['rms']-1)*100:+.0f} %), "
          f"Stellgroesse {best[1]['pwm']:.0f} statt {base['pwm']:.0f}.")
    print("\nTENS steht wieder auf 0.")


if __name__ == "__main__":
    main()
