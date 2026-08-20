#!/usr/bin/env python3
"""Automatische Reglerabstimmung fuer den Tumbler.

Laeuft auf dem PC, nicht im Roboter: jede Messung ist ein echter
physikalischer Versuch, und den will man mitlesen und abbrechen koennen.

Verfahren: Twiddle (Koordinatenabstieg mit adaptiver Schrittweite) ueber den
WebSocket der Firmware. Bewertet wird ein gewichteter Kostenwert; ein Sturz
kostet pauschal so viel, dass jede stehende Einstellung besser ist als jede
fallende.

Abgestimmt wird in Stufen, nicht alles auf einmal - neun Parameter gleichzeitig
sind physikalisch nicht trennbar, und der aeussere Regler laesst sich ohnehin
erst beurteilen, wenn der innere steht:

  1. balance   Kp, Kd, trim      Winkelregler, alles andere aus
  2. minpwm    minPwm            so klein wie moeglich, solange er steht
  3. yaw       YP, YD            Blickrichtung halten
  4. pos       VP, VI            an den Ausgangspunkt zurueck
  5. feinsc    Kp, Kd            Nachzug, jetzt mit alle Reglern aktiv

Beispiele:
    python3 autotune.py --dry              einmal messen, nichts aendern
    python3 autotune.py                     alle Stufen, Ergebnis anzeigen
    python3 autotune.py --stages balance minpwm
    python3 autotune.py --rounds 4 --save   laenger, Ergebnis in den Flash

Der Roboter muss die ganze Zeit balancieren, und jemand muss ihn nach einem
Sturz wieder aufstellen - von allein steht er nicht auf.
"""

import argparse
import math
import signal
import sys
import time

try:
    import websocket
except ImportError:
    sys.exit("Bitte 'pip install websocket-client' ausfuehren.")


HOST = "tumbler-mini.local"

# Telemetriefelder, wie webuiSendTelemetry() sie schickt.
IDX = {
    "angle": 1, "rate": 2, "pwm": 3, "run": 4, "req": 5, "fall": 6,
    "Kp": 7, "Ki": 8, "Kd": 9, "minPwm": 10, "trim": 11, "sign": 12,
    "yaw": 13, "steer": 14, "YP": 15, "YD": 16, "ysign": 17,
    "pos": 18, "v": 19, "bias": 20, "VP": 21, "VI": 22,
}

# name -> (kommando, minimum, maximum, startschrittweite, nachkommastellen)
SPEC = {
    "Kp":     ("P",      4.0,  80.0,  4.0,    2),
    "Ki":     ("I",      0.0,   2.0,  0.1,    3),
    "Kd":     ("D",      0.0,   3.0,  0.15,   3),
    "minPwm": ("MINPWM", 0.0, 120.0,  6.0,    0),
    "trim":   ("TRIM",  -6.0,   6.0,  0.4,    2),
    "YP":     ("YP",     0.0,  15.0,  1.0,    2),
    "YD":     ("YD",     0.0,   2.0,  0.1,    3),
    "VP":     ("VP",     0.0,  0.01,  0.0008, 4),
    "VI":     ("VI",     0.0, 0.004,  0.0003, 4),
}

STAGES = {
    "balance": ["Kp", "Kd", "trim"],
    "minpwm":  ["minPwm"],
    "yaw":     ["YP", "YD"],
    "pos":     ["VP", "VI"],
    "feinsch": ["Kp", "Kd"],
}

FALL_COST = 100.0


class Robot:
    """Verbindung zum Roboter: Kommandos hin, Telemetrie zurueck."""

    def __init__(self, host):
        self.ws = websocket.create_connection(f"ws://{host}/ws", timeout=5)
        self.state = self._read_frame()

    def _read_frame(self, timeout=5.0):
        """Naechsten Telemetrierahmen holen; Meldungen (M,...) ueberspringen."""
        end = time.time() + timeout
        while time.time() < end:
            raw = self.ws.recv()
            parts = raw.split(",")
            if parts[0] == "T" and len(parts) > max(IDX.values()):
                return {k: float(parts[i]) for k, i in IDX.items()}
        raise TimeoutError("keine Telemetrie erhalten")

    def send(self, cmd):
        self.ws.send(cmd)
        time.sleep(0.05)          # der Firmware Zeit lassen, es zu uebernehmen

    def set(self, name, value):
        cmd, lo, hi, _, dec = SPEC[name]
        value = max(lo, min(hi, value))
        self.send(f"{cmd}={round(value, dec)}")
        return value

    def read(self):
        self.state = self._read_frame()
        return self.state

    def params(self):
        s = self.read()
        return {k: s[k] for k in SPEC}

    def wait_upright(self, quiet=1.0, tol=3.0):
        """Wartet, bis der Roboter aufgestellt wurde und ruhig steht."""
        print("    warte, bis er wieder aufrecht steht ...", end="", flush=True)
        steady_since = None
        while True:
            s = self.read()
            if abs(s["angle"] - s["trim"]) < tol and abs(s["rate"]) < 25:
                steady_since = steady_since or time.time()
                if time.time() - steady_since >= quiet:
                    print(" ok")
                    return
            else:
                steady_since = None

    def start(self):
        """Regler anwerfen; kommt er nicht hoch, erst aufstellen lassen."""
        for _ in range(3):
            self.send("START")
            deadline = time.time() + 3.0
            while time.time() < deadline:
                if self.read()["run"] > 0.5:
                    return True
            self.wait_upright()
        return False


def measure(bot, seconds, settle=1.2):
    """Einen Versuch fahren und bewerten.

    Zurueck kommen die Rohgroessen; der Kostenwert entsteht in cost().
    """
    if bot.read()["run"] < 0.5:
        bot.wait_upright()
        if not bot.start():
            return {"fall": True}

    bot.send("HOME")
    t_end = time.time() + settle
    while time.time() < t_end:            # Einschwingen nicht mitzaehlen
        bot.read()

    errs, pwms, poss, yaws = [], [], [], []
    flips = 0
    last_pwm = 0.0
    t_end = time.time() + seconds
    while time.time() < t_end:
        s = bot.read()
        if s["run"] < 0.5:                # unterwegs umgefallen
            return {"fall": True}
        errs.append(s["angle"] - s["trim"] - s["bias"])
        pwms.append(s["pwm"])
        poss.append(s["pos"])
        yaws.append(s["yaw"])
        if s["pwm"] * last_pwm < 0:       # Vorzeichenwechsel = Zittern
            flips += 1
        last_pwm = s["pwm"]

    n = max(1, len(errs))
    return {
        "fall": False,
        "rms":   math.sqrt(sum(e * e for e in errs) / n),
        "effort": sum(abs(p) for p in pwms) / n,
        "flips": flips / max(1e-6, seconds),
        "drift": abs(poss[-1] - poss[0]) if poss else 0.0,
        "yaw":   max(abs(y) for y in yaws) if yaws else 0.0,
    }


def cost(m, stage, minpwm=0.0):
    """Gewichteter Kostenwert - kleiner ist besser."""
    if m.get("fall"):
        return FALL_COST

    c = (2.0 * m["rms"]              # Winkelfehler: das eigentliche Ziel
         + 0.010 * m["effort"]       # Stellaufwand: leiser, sparsamer
         + 0.020 * m["flips"]        # Zittern der Stellgroesse
         + 0.0015 * m["drift"])      # Wegdrift

    if stage in ("yaw", "pos", "feinsch"):
        c += 0.05 * m["yaw"]         # Verdrehung zaehlt erst, wenn sie geregelt wird

    if stage == "minpwm":
        # Der Wunsch "so klein wie moeglich" steckt hier: ein niedriger
        # Schwellwert wird aktiv belohnt, aber nur solange der Roboter dabei
        # ruhig steht - der Winkelfehler oben wiegt schwerer.
        c += 0.020 * minpwm

    return c


def fmt(name, value):
    return f"{name}={round(value, SPEC[name][4])}"


def twiddle(bot, names, rounds, seconds, stage):
    """Koordinatenabstieg mit adaptiver Schrittweite."""
    best = bot.params()
    steps = {n: SPEC[n][3] for n in names}

    m = measure(bot, seconds)
    best_cost = cost(m, stage, best["minPwm"])
    print(f"  Ausgangswert: {best_cost:.3f}   "
          + "  ".join(fmt(n, best[n]) for n in names))

    for rnd in range(rounds):
        for name in names:
            for direction in (+1, -1):
                trial = dict(best)
                trial[name] = best[name] + direction * steps[name]
                applied = bot.set(name, trial[name])
                if abs(applied - best[name]) < 1e-9:
                    continue                       # Anschlag, nichts zu holen
                trial[name] = applied

                m = measure(bot, seconds)
                c = cost(m, stage, trial["minPwm"])
                tag = "Sturz" if m.get("fall") else f"{c:.3f}"
                print(f"  R{rnd+1} {fmt(name, applied):>16}  {tag}", end="")

                if c < best_cost:
                    best_cost, best = c, trial
                    steps[name] *= 1.3              # Richtung stimmt: mutiger
                    print("   besser")
                    break
                print()
                bot.set(name, best[name])           # zuruecknehmen
            else:
                steps[name] *= 0.6                  # beide Richtungen schlechter

        if all(steps[n] < SPEC[n][3] * 0.12 for n in names):
            print("  Schrittweiten ausgereizt, Stufe fertig.")
            break

    for n in names:
        bot.set(n, best[n])
    return best, best_cost


def shrink_minpwm(bot, seconds):
    """minPwm so weit senken, wie der Roboter es ohne Qualitaetsverlust traegt.

    Bewusst kein Twiddle: hier ist die Aufgabe nicht "finde das Optimum",
    sondern "geh so weit runter wie moeglich, ohne dass er schlechter wird".
    Also von oben herantasten und beim ersten deutlichen Rueckschritt stoppen.
    """
    start = bot.params()["minPwm"]
    m = measure(bot, seconds)
    ref = cost(m, "balance")
    if m.get("fall"):
        print("  faellt schon beim Ausgangswert - Stufe uebersprungen.")
        return start
    print(f"  Ausgangswert: minPwm={start:.0f} -> {ref:.3f}")

    best = start
    # Grenze, ab der ein niedrigerer Schwellwert die Ruhe merklich verschlechtert
    allowed = ref * 1.15 + 0.05

    for step in (8, 4, 2, 1):
        while True:
            trial = best - step
            if trial < 0:
                break
            applied = bot.set("minPwm", trial)
            m = measure(bot, seconds)
            c = cost(m, "balance")
            tag = "Sturz" if m.get("fall") else f"{c:.3f}"
            print(f"  minPwm={applied:>3.0f}  {tag}", end="")
            if m.get("fall") or c > allowed:
                print("   zu wenig")
                bot.set("minPwm", best)
                break
            print("   haelt")
            best = applied
            if c < ref:                 # unterwegs sogar besser geworden
                ref = c
                allowed = ref * 1.15 + 0.05

    bot.set("minPwm", best)
    print(f"  kleinster tragfaehiger Wert: minPwm={best:.0f}")
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--stages", nargs="+", default=list(STAGES),
                    choices=list(STAGES), help="welche Stufen laufen sollen")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=6.0,
                    help="Messdauer je Versuch")
    ap.add_argument("--save", action="store_true",
                    help="Ergebnis im Flash des Roboters ablegen")
    ap.add_argument("--dry", action="store_true",
                    help="nur einmal messen, nichts veraendern")
    args = ap.parse_args()

    print(f"verbinde mit {args.host} ...")
    bot = Robot(args.host)
    original = bot.params()
    print("Ausgangslage: " + "  ".join(fmt(n, v) for n, v in original.items()))

    def restore(*_):
        print("\nAbbruch - setze die Ausgangswerte zurueck.")
        try:
            bot.send("STOP")
            for n, v in original.items():
                bot.set(n, v)
            bot.send("RATE=100")
        finally:
            sys.exit(1)

    signal.signal(signal.SIGINT, restore)

    bot.send("RATE=20")          # 50 Hz: fein genug, um Zittern zu sehen

    if args.dry:
        m = measure(bot, args.seconds)
        print("Messung:", m)
        print("Kosten:", round(cost(m, "balance", original["minPwm"]), 3))
        bot.send("STOP")
        bot.send("RATE=100")
        return

    try:
        for stage in args.stages:
            print(f"\n=== Stufe '{stage}' ===")

            if stage == "balance":
                # Aussenregler still, sonst misst man ihre Wirkung mit.
                for n in ("VP", "VI", "YP", "YD"):
                    bot.set(n, 0.0)
            elif stage == "yaw":
                for n in ("VP", "VI"):
                    bot.set(n, 0.0)

            if stage == "minpwm":
                shrink_minpwm(bot, args.seconds)
            else:
                twiddle(bot, STAGES[stage], args.rounds, args.seconds, stage)

        final = bot.params()
        print("\n=== Ergebnis ===")
        for n, v in final.items():
            arrow = "" if abs(v - original[n]) < 1e-9 else f"   (vorher {round(original[n], SPEC[n][4])})"
            print(f"  {fmt(n, v)}{arrow}")

        bot.send("STOP")
        if args.save:
            bot.send("SAVE")
            print("\nIm Flash gespeichert.")
        else:
            print("\nNicht gespeichert - 'SAVE' in der Weboberflaeche oder "
                  "--save beim naechsten Lauf.")
    finally:
        bot.send("RATE=100")


if __name__ == "__main__":
    main()
