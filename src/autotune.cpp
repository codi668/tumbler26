#include "autotune.h"
#include "config.h"
#include "state.h"
#include "encoder.h"
#include "webui.h"

// ===========================================================================
//  Automatische Reglerabstimmung
//
//  Verfahren: Twiddle (Koordinatenabstieg mit adaptiver Schrittweite). Fuer
//  jeden Parameter wird ein Schritt nach oben und - falls der nichts bringt -
//  einer nach unten probiert. Wird es besser, waechst die Schrittweite;
//  bringt keine Richtung etwas, schrumpft sie.
//
//  Abgestimmt wird in Stufen, nicht alles auf einmal: acht Parameter
//  gleichzeitig sind am realen Aufbau nicht trennbar, und die aeusseren
//  Regler lassen sich ueberhaupt erst beurteilen, wenn der Winkelregler
//  steht. Waehrend einer Stufe liegen die noch nicht abgestimmten Regler
//  still, sonst misst man ihre Wirkung mit.
// ===========================================================================

namespace {

// --- Bewertung -------------------------------------------------------------
// Ein Sturz muss teurer sein als jede noch so unruhige, aber stehende
// Einstellung - sonst optimiert der Automat sich in den Sturz hinein.
constexpr float FALL_COST   = 100.0f;
constexpr uint32_t SETTLE_MS  = 1200;   // Einschwingen, wird nicht gewertet
constexpr uint32_t MEASURE_MS = 6000;   // eigentliche Messdauer
constexpr uint32_t UPRIGHT_HOLD_MS = 800;

struct Spec {
    const char *name;
    float *value;
    float lo, hi, step0;
};

// Die Grenzen sind bewusst weit, aber nicht unbegrenzt: ausserhalb davon
// balanciert der Roboter ohnehin nicht, und jeder Versuch kostet Zeit.
Spec S_KP   = {"Kp",     &Kp,      4.0f, 80.0f,  4.0f};
Spec S_KD   = {"Kd",     &Kd,      0.0f,  3.0f,  0.15f};
Spec S_TRIM = {"trim",   &trim,   -6.0f,  6.0f,  0.4f};
Spec S_MIN  = {"minPwm", &minPwm,  0.0f,120.0f,  6.0f};
Spec S_YP   = {"YP",     &Ykp,     0.0f, 15.0f,  1.0f};
Spec S_YD   = {"YD",     &Ykd,     0.0f,  2.0f,  0.1f};
Spec S_VP   = {"VP",     &Vkp,     0.0f, 0.01f,  0.0008f};
Spec S_VI   = {"VI",     &Vki,     0.0f, 0.004f, 0.0003f};

struct StageDef {
    const char *name;
    Spec *p[3];
    int   n;
    bool  shrink;   // true = "so klein wie moeglich" statt "optimal"
};

StageDef STAGES[] = {
    {"Balance",     {&S_KP, &S_KD, &S_TRIM}, 3, false},
    {"minPwm",      {&S_MIN, nullptr, nullptr}, 1, true},
    {"Richtung",    {&S_YP, &S_YD, nullptr},  2, false},
    {"Position",    {&S_VP, &S_VI, nullptr},  2, false},
    {"Feinschliff", {&S_KP, &S_KD, nullptr},  2, false},
};
constexpr int STAGE_COUNT = sizeof(STAGES) / sizeof(STAGES[0]);

enum Phase { PH_OFF, PH_WAIT_UPRIGHT, PH_SETTLE, PH_MEASURE, PH_NEXT, PH_DONE };

Phase    phase      = PH_OFF;
int      stageIdx   = 0;
int      paramIdx   = 0;
int      direction  = +1;      // erst nach oben, dann nach unten probieren
int      trialCount = 0;
int      roundCount = 0;
uint32_t phaseStart = 0;
uint32_t uprightSince = 0;

float steps[3];
float best[3];                 // beste bekannte Werte der laufenden Stufe
float bestCost = FALL_COST;
float shrinkAllowed = 0.0f;    // Kostengrenze der minPwm-Stufe
float shrinkStep = 8.0f;
bool  haveReference = false;   // Ausgangsmessung der Stufe schon gemacht?

// Messwerte des laufenden Versuchs
double accErr2 = 0.0, accPwm = 0.0;
float  accYawMax = 0.0f, lastPwmSign = 0.0f;
long   posStart = 0, posEnd = 0;
int    accN = 0, flips = 0;
bool   trialFell = false;

// Alle Parameter, wie sie vor dem Start waren - fuer den Abbruch.
float savedAll[8];
Spec *ALL[8] = {&S_KP, &S_KD, &S_TRIM, &S_MIN, &S_YP, &S_YD, &S_VP, &S_VI};

const char *phaseText = "aus";

void resetTrial()
{
    accErr2 = accPwm = 0.0;
    accYawMax = 0.0f;
    accN = flips = 0;
    lastPwmSign = 0.0f;
    trialFell = false;
    posStart = encoderPos();
}

float trialCost()
{
    if (trialFell || accN < 20) return FALL_COST;

    const float rms    = sqrtf((float)(accErr2 / accN));
    const float effort = (float)(accPwm / accN);
    const float flipHz = flips * 1000.0f / (float)MEASURE_MS;
    const float drift  = fabsf((float)(posEnd - posStart));

    float c = 2.0f * rms            // Winkelfehler: das eigentliche Ziel
            + 0.010f * effort       // Stellaufwand: leiser und sparsamer
            + 0.020f * flipHz       // Zittern der Stellgroesse
            + 0.0015f * drift;      // Wegdrift

    // Die Verdrehung zaehlt erst, sobald sie ueberhaupt geregelt wird.
    if (stageIdx >= 2) c += 0.05f * accYawMax;

    // "So klein wie moeglich" steckt hier: ein niedriger Schwellwert wird
    // belohnt - aber der Winkelfehler oben wiegt schwerer, ein zu kleiner
    // minPwm faellt also trotzdem durch.
    if (STAGES[stageIdx].shrink) c += 0.020f * minPwm;

    return c;
}

void applyValue(Spec *s, float v)
{
    *s->value = constrain(v, s->lo, s->hi);
}

void beginStage()
{
    StageDef &st = STAGES[stageIdx];

    // Noch nicht abgestimmte Regler stilllegen, damit ihre Wirkung nicht in
    // die Messung eingeht.
    if (stageIdx == 0) { Vkp = Vki = 0.0f; Ykp = Ykd = 0.0f; }
    else if (stageIdx <= 2) { Vkp = Vki = 0.0f; }

    for (int i = 0; i < st.n; i++)
    {
        steps[i] = st.p[i]->step0;
        best[i]  = *st.p[i]->value;
    }
    paramIdx = 0;
    direction = +1;
    roundCount = 0;
    bestCost = FALL_COST;
    haveReference = false;
    shrinkStep = 8.0f;

    char msg[64];
    snprintf(msg, sizeof(msg), "Stufe %d/%d: %s", stageIdx + 1, STAGE_COUNT, st.name);
    webuiNotify(msg);
    Serial.print(F("[Abstimmung] ")); Serial.println(msg);
}

void finishStage()
{
    StageDef &st = STAGES[stageIdx];
    for (int i = 0; i < st.n; i++) applyValue(st.p[i], best[i]);

    stageIdx++;
    if (stageIdx >= STAGE_COUNT)
    {
        phase = PH_DONE;
        phaseText = "fertig";
        requested = false;
        running = false;
        webuiNotify("Abstimmung fertig - SAVE nicht vergessen");
        Serial.println(F("[Abstimmung] fertig"));
    }
    else
    {
        beginStage();
        phase = PH_SETTLE;
        phaseStart = millis();
    }
}

// Ergebnis eines Versuchs verrechnen und den naechsten Kandidaten setzen.
void evaluate(float cost)
{
    StageDef &st = STAGES[stageIdx];
    trialCount++;

    // --- Sonderweg fuer minPwm: nicht optimieren, sondern absenken ---------
    if (st.shrink)
    {
        if (!haveReference)
        {
            haveReference = true;
            bestCost = cost;
            best[0] = minPwm;
            // Solange die Ruhe nicht merklich leidet, darf der Schwellwert
            // weiter runter. 15 % Spielraum, damit Messrauschen nicht schon
            // als Verschlechterung durchgeht.
            shrinkAllowed = cost * 1.15f + 0.05f;
        }
        else if (cost <= shrinkAllowed)
        {
            best[0] = minPwm;                    // haelt noch, weiter runter
            if (cost < bestCost)
            {
                bestCost = cost;
                shrinkAllowed = cost * 1.15f + 0.05f;
            }
        }
        else
        {
            applyValue(&S_MIN, best[0]);         // zu wenig, Schritt verkleinern
            shrinkStep *= 0.5f;
        }

        if (shrinkStep < 1.0f || best[0] <= 0.0f) { finishStage(); return; }
        applyValue(&S_MIN, best[0] - shrinkStep);
        if (minPwm >= best[0]) { finishStage(); return; }   // Anschlag erreicht
        return;
    }

    // --- Twiddle ----------------------------------------------------------
    if (!haveReference)
    {
        haveReference = true;
        bestCost = cost;
        applyValue(st.p[0], best[0] + steps[0]);
        return;
    }

    if (cost < bestCost)
    {
        bestCost = cost;
        best[paramIdx] = *st.p[paramIdx]->value;
        steps[paramIdx] *= 1.3f;                 // Richtung stimmt: mutiger
        direction = +1;
        paramIdx = (paramIdx + 1) % st.n;
    }
    else if (direction > 0)
    {
        direction = -1;                          // andere Richtung probieren
    }
    else
    {
        applyValue(st.p[paramIdx], best[paramIdx]);
        steps[paramIdx] *= 0.6f;                 // beide Richtungen schlechter
        direction = +1;
        paramIdx = (paramIdx + 1) % st.n;
    }

    if (paramIdx == 0 && direction > 0)
    {
        roundCount++;
        bool exhausted = true;
        for (int i = 0; i < st.n; i++)
            if (steps[i] >= st.p[i]->step0 * 0.12f) exhausted = false;
        if (exhausted || roundCount >= 4) { finishStage(); return; }
    }

    applyValue(st.p[paramIdx], best[paramIdx] + direction * steps[paramIdx]);
}

} // namespace

void autotuneStart()
{
    for (int i = 0; i < 8; i++) savedAll[i] = *ALL[i]->value;
    stageIdx = 0;
    trialCount = 0;
    beginStage();
    phase = PH_WAIT_UPRIGHT;
    phaseText = "aufstellen";
    uprightSince = 0;
    Serial.println(F("[Abstimmung] gestartet"));
}

void autotuneStop(bool keepBest)
{
    if (phase == PH_OFF) return;

    if (!keepBest)
    {
        for (int i = 0; i < 8; i++) *ALL[i]->value = savedAll[i];
        webuiNotify("Abstimmung abgebrochen, alte Werte zurueck");
    }
    else
    {
        StageDef &st = STAGES[min(stageIdx, STAGE_COUNT - 1)];
        for (int i = 0; i < st.n; i++) applyValue(st.p[i], best[i]);
        webuiNotify("Abstimmung beendet");
    }
    phase = PH_OFF;
    phaseText = "aus";
    requested = false;
    running = false;
}

bool  autotuneActive()   { return phase != PH_OFF && phase != PH_DONE; }
int   autotuneStage()    { return autotuneActive() ? stageIdx : -1; }
int   autotuneTrial()    { return trialCount; }
float autotuneBestCost() { return bestCost; }
const char *autotunePhase() { return phaseText; }

void autotuneTick(float dt, float angleErr, int pwm, float yaw, long pos)
{
    if (phase == PH_OFF || phase == PH_DONE) return;

    const uint32_t now = millis();

    // Umgefallen? Dann zaehlt der laufende Versuch als schlechtestes Ergebnis
    // und der Automat wartet, bis der Roboter wieder aufgestellt wurde.
    if (!running && phase != PH_WAIT_UPRIGHT)
    {
        if (phase == PH_MEASURE || phase == PH_SETTLE)
        {
            trialFell = true;
            if (phase == PH_MEASURE) { posEnd = pos; evaluate(FALL_COST); }
        }
        phase = PH_WAIT_UPRIGHT;
        phaseText = "aufstellen";
        uprightSince = 0;
        return;
    }

    switch (phase)
    {
    case PH_WAIT_UPRIGHT:
        // Erst anwerfen, wenn er ruhig genug steht, dass er sich fangen kann.
        if (fabsf(angleDeg - trim) < ARM_ANGLE_DEG && fabsf(gyroRateDs) < 25.0f)
        {
            if (uprightSince == 0) uprightSince = now;
            if (now - uprightSince >= UPRIGHT_HOLD_MS)
            {
                requested = true;
                fallFlag = false;
                phase = PH_SETTLE;
                phaseText = "einschwingen";
                phaseStart = now;
            }
        }
        else uprightSince = 0;
        break;

    case PH_SETTLE:
        if (!running) { phaseStart = now; break; }   // laeuft noch nicht
        if (now - phaseStart >= SETTLE_MS)
        {
            resetTrial();
            posTarget = pos;              // Weg ab hier zaehlen
            phase = PH_MEASURE;
            phaseText = "messen";
            phaseStart = now;
        }
        break;

    case PH_MEASURE:
    {
        accErr2 += (double)angleErr * angleErr;
        accPwm  += fabs((double)pwm);
        accN++;
        if (fabsf(yaw) > accYawMax) accYawMax = fabsf(yaw);
        const float sgn = (pwm > 0) ? 1.0f : (pwm < 0 ? -1.0f : 0.0f);
        if (sgn != 0.0f && lastPwmSign != 0.0f && sgn != lastPwmSign) flips++;
        if (sgn != 0.0f) lastPwmSign = sgn;

        if (now - phaseStart >= MEASURE_MS)
        {
            posEnd = pos;
            evaluate(trialCost());
            if (phase == PH_MEASURE)      // evaluate() kann die Stufe beenden
            {
                phase = PH_SETTLE;
                phaseText = "einschwingen";
                phaseStart = now;
            }
        }
        break;
    }

    default:
        break;
    }
}
