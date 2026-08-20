// Tumbler V2 - minimaler Balancierregler
//
// Eigenstaendiges Programm: nur die Pinbelegung in config.h stammt aus dem
// bestehenden Projekt /home/thomas/tumbler_balance, der Rest (IMU-Treiber,
// Filter, Regler, Motoransteuerung, Kommandozeile) ist neu geschrieben.
//
// Drei Regler, die uebereinander liegen:
//
//   1. Balance (200 Hz)  PD auf den Neigungswinkel aus einem
//      Komplementaerfilter (Beschleunigungssensor + Gyro). Der D-Anteil kommt
//      direkt aus der gemessenen Drehrate, nicht aus einer Ableitung.
//   2. Richtung          PD auf den aus Gyro-Z integrierten Gierwinkel, wirkt
//      als Differenz auf die beiden Raeder.
//   3. Position          PI auf Weg und Tempo aus den Radencodern, verschiebt
//      den Sollwinkel des Balancereglers (ab Werk aus, siehe Vkp/Vki).
//
// Bedienung ueber die serielle Konsole (115200 Baud) oder die Weboberflaeche:
//   START / STOP         Regler ein-/ausschalten (auch per Taster GPIO2)
//   ZERO                 Gyro und Winkel neu nullen (Roboter dabei ruhig halten)
//   HOME                 aktuellen Standort als Sollposition uebernehmen
//   P= I= D= MINPWM= TRIM= SIGN=     Balance
//   YP= YD= YSIGN=                   Richtung
//   VP= VI=                          Position
//   STATUS  T            Werte anzeigen / Telemetrie an/aus
//   SAVE LOAD            Parameter im Flash ablegen bzw. zurueckholen

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "config.h"
#include "state.h"
#include "leds.h"
#include "webui.h"
#include "encoder.h"
#include "autotune.h"

// ---------------------------------------------------------------------------
// MPU6050 - direkter Registerzugriff, keine externe Bibliothek
// ---------------------------------------------------------------------------
namespace mpu {

constexpr uint8_t ADDR          = 0x68;
constexpr uint8_t REG_PWR_MGMT1 = 0x6B;
constexpr uint8_t REG_GYRO_CFG  = 0x1B;
constexpr uint8_t REG_ACCEL_CFG = 0x1C;
constexpr uint8_t REG_ACCEL_XH  = 0x3B;

constexpr float ACCEL_LSB_PER_G     = 16384.0f;  // Bereich +-2g
constexpr float GYRO_LSB_PER_DEG_S  = 131.0f;    // Bereich +-250 deg/s

void writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool begin()
{
    writeReg(REG_PWR_MGMT1, 0x00);   // aus dem Sleep-Modus wecken
    delay(50);
    writeReg(REG_GYRO_CFG, 0x00);    // +-250 deg/s
    writeReg(REG_ACCEL_CFG, 0x00);   // +-2g

    Wire.beginTransmission(ADDR);
    Wire.write(0x75);                // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR, 1);
    if (!Wire.available()) return false;
    uint8_t who = Wire.read();
    return who == 0x68 || who == 0x72 || who == 0x98; // bekannte MPU6050/6500-Varianten
}

// Liest Beschleunigung in g und Drehrate in Grad/s.
void read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACCEL_XH);
    Wire.endTransmission(false);
    Wire.requestFrom((int)ADDR, 14);

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();        // Temperatur, nicht gebraucht
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    ax = rawAx / ACCEL_LSB_PER_G;
    ay = rawAy / ACCEL_LSB_PER_G;
    az = rawAz / ACCEL_LSB_PER_G;
    gx = rawGx / GYRO_LSB_PER_DEG_S;
    gy = rawGy / GYRO_LSB_PER_DEG_S;
    gz = rawGz / GYRO_LSB_PER_DEG_S;
}

} // namespace mpu

// ---------------------------------------------------------------------------
// Motoransteuerung (TB6612FNG per LEDC-Hardware-PWM)
// ---------------------------------------------------------------------------
namespace motor {

constexpr int CH_A = 0;
constexpr int CH_B = 1;
constexpr int PWM_FREQ_HZ = 20000;   // oberhalb des Hoerbereichs
constexpr int PWM_BITS    = 8;       // 0..255

void begin()
{
    pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    digitalWrite(PIN_STBY, LOW);

    ledcSetup(CH_A, PWM_FREQ_HZ, PWM_BITS);
    ledcSetup(CH_B, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(PIN_PWMA, CH_A);
    ledcAttachPin(PIN_PWMB, CH_B);
    ledcWrite(CH_A, 0);
    ledcWrite(CH_B, 0);
}

void enable(bool on)
{
    digitalWrite(PIN_STBY, on ? HIGH : LOW);
}

// pwm: -255..255, Vorzeichen = Richtung. A und B bekommen denselben Befehl,
// MOTOR_B_DIR gleicht die spiegelbildliche Motormontage aus.
void set(int pwmA, int pwmB)
{
    pwmA = constrain(pwmA, -255, 255);
    pwmB = constrain(MOTOR_B_DIR * pwmB, -255, 255);

    digitalWrite(PIN_AIN1, pwmA >= 0 ? HIGH : LOW);
    digitalWrite(PIN_BIN1, pwmB >= 0 ? HIGH : LOW);
    ledcWrite(CH_A, abs(pwmA));
    ledcWrite(CH_B, abs(pwmB));
}

void stop()
{
    set(0, 0);
    enable(false);
}

} // namespace motor

// ---------------------------------------------------------------------------
// Reglerzustand
// ---------------------------------------------------------------------------
// Startwerte aus der Live-Abstimmung des bestehenden Projekts am selben
// Roboter (Kp/Kd/minPwm/outSign/trim) - reine Zahlen, kein uebernommener
// Code. SIGN dreht bei Bedarf die Wirkrichtung um, falls der Regler bei der
// Erstinbetriebnahme in die falsche Richtung gegensteuert.
float Kp      = 28.0f;
float Ki      = 0.0f;
float Kd      = 0.65f;
float minPwm  = 50.0f;   // darunter bewegt sich der Roboter unter Eigengewicht nicht
float trim    = 0.0f;    // Sollwinkel-Offset, mit ZERO bzw. TRIM= einstellbar
float outSign = -1.0f;

// Gierregler (Blickrichtung halten). Beim Vor- und Zurueckkorrigieren laufen
// die beiden Raeder nie exakt gleich - unterschiedliche Reibung, Getriebespiel,
// leicht verschiedene Motoren. Das summiert sich zu einer langsamen Drehung.
// Gegenmittel: die Drehrate um die Hochachse (Gyro-Z) integrieren und die
// Abweichung von der Startrichtung als DIFFERENZ auf die Raeder legen.
float Ykp      = 3.0f;    // je Grad Richtungsfehler
float Ykd      = 0.25f;   // je Grad/s Drehrate - daempft die Drehung sofort
// -1 am lebenden Objekt bestimmt: mit +1 liefen die Raeder gegeneinander und
// der Roboter drehte sich staerker statt zurueck (Thomas' Beobachtung
// 2026-08-20). Welche Drehrichtung zu welchem Vorzeichen gehoert, steht in
// keiner Zahl - das sieht man nur.
float yawSign  = -1.0f;   // dreht er sich weg statt zurueck: YSIGN=1
constexpr float YAW_MAX_STEER = 60.0f;  // Kappung, damit Balance Vorrang behaelt

// Positionsregler (aeussere Schleife). Ein reiner Winkelregler haelt zwar die
// Balance, kennt aber seinen Standort nicht: jede Stoerung schiebt den Roboter
// ein Stueck weiter, und er bleibt dort. Dieser Regler misst Weg und Tempo an
// den Encodern und verschiebt daraufhin den SOLLWINKEL - der Roboter lehnt
// sich also leicht in die Richtung, in die er fahren muss, und der innere
// Regler faehrt ihn dort hin. Direkt aufs PWM zu addieren wuerde dagegen
// gegen die Balance arbeiten.
//
// Vorsicht beim Zuschalten: laeuft der Aussenregler zu kraeftig, klebt sein
// Ausgang dauernd am Anschlag und arbeitet gegen das Auffangen. Deshalb
// startet er ab Werk AUS (VP=0, VI=0) - erst den inneren Regler am Boden
// bestaetigen, dann VP in kleinen Schritten hochziehen.
float Vkp = 0.0f;    // je Impuls/s Geschwindigkeitsfehler  -> Sollwinkel
float Vki = 0.0f;    // je Impuls Wegfehler                 -> Sollwinkel
constexpr float TILT_BIAS_MAX = 4.0f;   // Sollwinkel hoechstens so weit kippen

// Aufschwingen: Stellgroesse und groesster Winkel, aus dem er es versucht.
// Beides live einstellbar, weil es vom Gewicht und vom Untergrund abhaengt -
// auf Teppich braucht er mehr Schwung als auf Parkett.
float upPwm = 200.0f;      // Stellgroesse beim Aufschwingen, 0..255
float upMax = 60.0f;       // darueber gar nicht erst versuchen (Grad)
bool  swingActive = false; // gerade am Aufschwingen
int   swingTries  = 0;
uint32_t swingStartMs = 0;
uint32_t swingPauseUntilMs = 0;

// Fahrbetrieb. Ein Balancierer faehrt, indem er sich in Fahrtrichtung lehnt -
// eine Sollgeschwindigkeit wird also nicht aufs PWM gegeben, sondern ueber den
// SOLLWINKEL gestellt, genau wie beim Positionsregler. Die Spur haelt dabei
// der Gierregler; "geradeaus" heisst schlicht Solldrehrate null.
float Dkp = 0.0020f;       // Grad Neigung je Impuls/s Geschwindigkeitsfehler
float Dki = 0.0008f;       // I-Anteil, faengt Steigung und Rollwiderstand ab
float driveWish   = 0.0f;  // was verlangt wurde, Impulse/s
float driveSpeed  = 0.0f;  // was gerade gestellt wird (angerampt)
float driveAccel  = 2500.0f;  // Impulse/s^2 - ein Sprung wuerde ihn ausheben
float driveInt    = 0.0f;
bool  driveActive = false;    // faehrt oder bremst gerade noch
// Darunter gilt er als stehend. Nicht zu klein waehlen: die Encoder rauschen,
// und ein zu strenger Wert liesse ihn ewig im Bremszustand haengen.
constexpr float STOP_SPEED = 120.0f;   // Impulse/s
float turnRate    = 0.0f;  // Solldrehrate, Grad/s (0 = geradeaus)
float yawTarget   = 0.0f;  // Sollrichtung, laeuft mit turnRate mit

float tiltBias   = 0.0f;   // aktueller Versatz des Sollwinkels, Grad
long  posTarget  = 0;      // Sollposition in Encoder-Impulsen
float wheelSpeed = 0.0f;   // gefiltertes Tempo, Impulse/s

float angleDeg   = 0.0f;   // gefilterter Neigungswinkel
float gyroRateDs = 0.0f;   // Drehrate um die Kippachse, Grad/s
float yawDeg     = 0.0f;   // aufintegrierte Abweichung von der Sollrichtung
float yawRateDs  = 0.0f;   // Drehrate um die Hochachse, Grad/s
float lastSteer  = 0.0f;   // zuletzt ausgegebene Raddifferenz, fuer die Anzeige
float integral   = 0.0f;

bool running   = false;   // Motoren aktiv geregelt
bool requested = false;   // START wurde verlangt, wartet ggf. auf ARM_ANGLE_DEG
bool fallFlag  = false;   // zeigt an, dass zuletzt die Sturzabschaltung ausgeloest hat
bool telemetry = false;
int  lastPwmOut = 0;      // zuletzt ausgegebener PWM-Wert, fuer LEDs/Web-UI

uint32_t lastLoopUs = 0;
uint32_t lastTelemetryPushMs = 0;
// 10 Hz reichen fuers Auge. Die automatische Abstimmung stellt sich per RATE=
// schneller, weil sie aus dem Verlauf auch das Zittern der Stellgroesse misst.
uint32_t telemetryPeriodMs = 100;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

float g_gyroBias  = 0.0f;   // Gyro-Nullrate der Kippachse, per ZERO ermittelt
float g_gyroBiasZ = 0.0f;   // dito Hochachse - ohne den wandert die Richtung von allein

// Verteilt eine Reglerausgabe (etwa -255..255) auf den nutzbaren PWM-Bereich
// minPwm..255, damit auch kleine Korrekturen die Standreibung ueberwinden.
int spreadToPwm(float out)
{
    float mag = fabsf(out);
    if (mag < 0.5f) return 0;
    mag = constrain(mag, 0.0f, 255.0f);
    float pwm = minPwm + (255 - minPwm) * (mag / 255.0f);
    int pwmI = constrain((int)pwm, 0, 255);
    return out >= 0 ? pwmI : -pwmI;
}

void zeroAngle()
{
    // Kurze Mittelung der Gyro-Nullrate und des Startwinkels aus dem
    // Beschleunigungssensor - Roboter dabei ruhig und aufrecht halten.
    float sumGx = 0, sumGz = 0, sumAy = 0, sumAz = 0;
    const int n = 200;
    for (int i = 0; i < n; i++)
    {
        float ax, ay, az, gx, gy, gz;
        mpu::read(ax, ay, az, gx, gy, gz);
        sumGx += gx;
        sumGz += gz;
        sumAy += ay;
        sumAz += az;
        delay(2);
    }
    g_gyroBias  = sumGx / n;
    g_gyroBiasZ = sumGz / n;
    angleDeg = atan2f(sumAy / n, sumAz / n) * 180.0f / PI;
    integral = 0.0f;
    yawDeg   = 0.0f;   // aktuelle Blickrichtung wird die neue Sollrichtung

    webuiLogf("ZERO  Winkel %.2f  Gyro X %.3f  Z %.3f",
              angleDeg, g_gyroBias, g_gyroBiasZ);
}

// --- Parameter dauerhaft ablegen -------------------------------------------
// Ohne das ist jede muehsam erflogene Einstellung nach dem naechsten Reset
// wieder weg. NVS ueberlebt auch ein neues Firmware-Image.
Preferences prefs;

void saveParams()
{
    prefs.begin("tumblermini", false);
    prefs.putFloat("kp", Kp);
    prefs.putFloat("ki", Ki);
    prefs.putFloat("kd", Kd);
    prefs.putFloat("minpwm2", minPwm);
    prefs.putFloat("trim", trim);
    prefs.putFloat("sign", outSign);
    prefs.putFloat("ykp", Ykp);
    prefs.putFloat("ykd", Ykd);
    prefs.putFloat("ysign", yawSign);
    prefs.putFloat("vkp", Vkp);
    prefs.putFloat("vki", Vki);
    prefs.putFloat("uppwm", upPwm);
    prefs.putFloat("upmax", upMax);
    prefs.putFloat("dkp", Dkp);
    prefs.putFloat("dki", Dki);
    prefs.end();
    Serial.println(F("gespeichert"));
    webuiNotify("gespeichert");
}

void loadParams()
{
    prefs.begin("tumblermini", true);
    Kp      = prefs.getFloat("kp", Kp);
    Ki      = prefs.getFloat("ki", Ki);
    Kd      = prefs.getFloat("kd", Kd);
    minPwm  = prefs.getFloat("minpwm2", minPwm);
    trim    = prefs.getFloat("trim", trim);
    outSign = prefs.getFloat("sign", outSign);
    Ykp     = prefs.getFloat("ykp", Ykp);
    Ykd     = prefs.getFloat("ykd", Ykd);
    yawSign = prefs.getFloat("ysign", yawSign);
    Vkp     = prefs.getFloat("vkp", Vkp);
    Vki     = prefs.getFloat("vki", Vki);
    upPwm   = prefs.getFloat("uppwm", upPwm);
    upMax   = prefs.getFloat("upmax", upMax);
    Dkp     = prefs.getFloat("dkp", Dkp);
    Dki     = prefs.getFloat("dki", Dki);
    prefs.end();
}

void printStatus()
{
    Serial.println(F("--- STATUS ---"));
    Serial.print(F("running=")); Serial.print(running);
    Serial.print(F(" angle=")); Serial.print(angleDeg, 2);
    Serial.print(F(" Kp=")); Serial.print(Kp, 2);
    Serial.print(F(" Ki=")); Serial.print(Ki, 3);
    Serial.print(F(" Kd=")); Serial.print(Kd, 2);
    Serial.print(F(" minPwm=")); Serial.print(minPwm, 1);
    Serial.print(F(" trim=")); Serial.print(trim, 2);
    Serial.print(F(" sign=")); Serial.println(outSign, 0);
    Serial.print(F("yaw=")); Serial.print(yawDeg, 2);
    Serial.print(F(" YP=")); Serial.print(Ykp, 2);
    Serial.print(F(" YD=")); Serial.print(Ykd, 2);
    Serial.print(F(" ysign=")); Serial.println(yawSign, 0);
    Serial.print(F("pos=")); Serial.print(encoderPos() - posTarget);
    Serial.print(F(" v=")); Serial.print(wheelSpeed, 0);
    Serial.print(F(" bias=")); Serial.print(tiltBias, 2);
    Serial.print(F(" VP=")); Serial.print(Vkp, 4);
    Serial.print(F(" VI=")); Serial.print(Vki, 4);
    Serial.print(F(" UPPWM=")); Serial.print(upPwm, 0);
    Serial.print(F(" UPMAX=")); Serial.println(upMax, 0);
    Serial.print(F("fahrt=")); Serial.print(driveSpeed, 0);
    Serial.print(F("/")); Serial.print(driveWish, 0);
    Serial.print(F(" turn=")); Serial.print(turnRate, 0);
    Serial.print(F(" DP=")); Serial.print(Dkp, 4);
    Serial.print(F(" DI=")); Serial.println(Dki, 4);
}

void handleCommand(const String &cmdIn)
{
    String cmd = cmdIn;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "START") { requested = true; fallFlag = false; swingTries = 0; swingPauseUntilMs = 0; }
    else if (cmd == "STOP") {
        if (autotuneActive()) autotuneStop(true);
        requested = false; running = false; swingActive = false; swingTries = 0;
        driveWish = driveSpeed = driveInt = 0.0f;
        turnRate = yawTarget = 0.0f;
        motor::stop(); webuiLog("STOP");
    }
    else if (cmd == "ZERO") { zeroAngle(); fallFlag = false; }
    else if (cmd == "STATUS") { printStatus(); }
    else if (cmd == "T") { telemetry = !telemetry; }
    else if (cmd.startsWith("P=")) { Kp = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("I=")) { Ki = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("D=")) { Kd = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("MINPWM=")) { minPwm = cmd.substring(7).toFloat(); }
    else if (cmd.startsWith("TRIM=")) { trim = cmd.substring(5).toFloat(); }
    else if (cmd.startsWith("SIGN=")) { outSign = cmd.substring(5).toFloat() >= 0 ? 1.0f : -1.0f; }
    else if (cmd.startsWith("YP=")) { Ykp = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("YD=")) { Ykd = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("YSIGN=")) { yawSign = cmd.substring(6).toFloat() >= 0 ? 1.0f : -1.0f; }
    else if (cmd.startsWith("FWD="))
    {
        driveWish = constrain(cmd.substring(4).toFloat(), -6000.0f, 6000.0f);
        webuiLogf("Fahrt: Soll %.0f Impulse/s (ist %.0f)", driveWish, wheelSpeed);
    }
    else if (cmd.startsWith("TURN="))
    {
        turnRate = constrain(cmd.substring(5).toFloat(), -180.0f, 180.0f);
        webuiLogf("Drehrate %.0f Grad/s (Richtung %.1f)", turnRate, yawDeg);
    }
    else if (cmd == "HALT")
    {
        driveWish = 0.0f; turnRate = 0.0f;
        webuiLogf("HALT - bremst aus %.0f Impulse/s", wheelSpeed);
    }
    else if (cmd.startsWith("DP=")) { Dkp = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("DI=")) { Dki = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("UPPWM=")) { upPwm = constrain(cmd.substring(6).toFloat(), 0.0f, 255.0f); }
    else if (cmd.startsWith("UPMAX=")) { upMax = constrain(cmd.substring(6).toFloat(), 0.0f, 80.0f); }
    else if (cmd.startsWith("VP=")) { Vkp = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("VI=")) { Vki = cmd.substring(3).toFloat(); }
    else if (cmd == "HOME") { encoderReset(); posTarget = 0; Serial.println(F("Ausgangslage neu gesetzt")); }
    else if (cmd.startsWith("RATE=")) {
        telemetryPeriodMs = constrain(cmd.substring(5).toInt(), 10, 1000);
    }
    else if (cmd == "TUNE") {
        if (autotuneActive()) autotuneStop(true); else autotuneStart();
    }
    else if (cmd == "TUNEOFF") { autotuneStop(false); }
    else if (cmd == "SAVE") { saveParams(); }
    else if (cmd == "LOAD") { loadParams(); printStatus(); }
    else if (cmd == "?") {
        Serial.println(F("START STOP ZERO STATUS T  P= I= D= MINPWM= TRIM= SIGN="));
        Serial.println(F("Gierregler: YP= YD= YSIGN=  (YP=0 YD=0 schaltet ihn ab)"));
        Serial.println(F("Positionsregler: VP= VI= HOME  (VP=0 VI=0 schaltet ihn ab)"));
        Serial.println(F("Aufschwingen: UPPWM= UPMAX=  (UPPWM=0 schaltet es ab)"));
        Serial.println(F("Fahren: FWD=<Impulse/s> TURN=<Grad/s> HALT  DP= DI="));
        Serial.println(F("SAVE LOAD - Parameter im Flash ablegen/zurueckholen"));
        Serial.println(F("TUNE - Abstimmung starten/beenden, TUNEOFF - abbrechen"));
    }
    else {
        Serial.print(F("? unbekannt: ")); Serial.println(cmd);
    }
}

void pollSerial()
{
    static String buf;
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (buf.length() > 0)
            {
                webuiLogf("< %s (seriell)", buf.c_str());
                handleCommand(buf);
                buf = "";
            }
        }
        else
        {
            buf += c;
        }
    }
}

void pollButton()
{
    static bool lastState = false;
    static uint32_t lastChangeMs = 0;

    bool pressed = digitalRead(PIN_BUTTON) == HIGH; // aktiv HIGH, siehe config.h
    uint32_t now = millis();
    if (pressed != lastState && (now - lastChangeMs) > 40) // einfaches Entprellen
    {
        lastChangeMs = now;
        lastState = pressed;
        if (pressed)
        {
            if (running || requested)
            {
                webuiLog("Taster: aus");
                requested = false; running = false; motor::stop();
            }
            else { webuiLog("Taster: START"); requested = true; fallFlag = false; }
        }
    }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println(F("Tumbler V2 - minimaler Balancierregler"));

    pinMode(PIN_BUTTON, INPUT);   // KEIN internen Pullup - externer Pulldown auf der Platine

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    if (!mpu::begin())
    {
        Serial.println(F("MPU6050 nicht gefunden - Verkabelung pruefen!"));
    }

    motor::begin();
    encoderBegin();
    ledsBegin();
    loadParams();     // gespeicherte Werte vor dem Start der Oberflaeche holen
    webuiBegin();

    Serial.println(F("Kalibriere Gyro/Winkel - Roboter ruhig und aufrecht halten..."));
    zeroAngle();

    Serial.println(F("Bereit. START oder Taster zum Balancieren, ? fuer Hilfe."));
    lastLoopUs = micros();
}

void loop()
{
    pollSerial();
    pollButton();
    webuiLoop();

    uint32_t nowUs = micros();
    if (nowUs - lastLoopUs < LOOP_PERIOD_US) return;
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;

    // --- Winkel schaetzen ---
    float ax, ay, az, gx, gy, gz;
    mpu::read(ax, ay, az, gx, gy, gz);

    // Kippachse dieses Roboters: Beschleunigungswinkel aus ay/az, Drehrate um X.
    float accAngle = atan2f(ay, az) * 180.0f / PI;
    gyroRateDs = gx - g_gyroBias;

    angleDeg = 0.98f * (angleDeg + gyroRateDs * dt) + 0.02f * accAngle;

    // --- Positionsregler: verschiebt den Sollwinkel ---
    encoderPoll(dt);
    wheelSpeed = encoderSpeed();

    // Sollgeschwindigkeit anrampen statt springen zu lassen: ein Sprung
    // verlangt sofort vollen Neigungsversatz und hebt ihm die Raeder weg.
    const float dv = driveAccel * dt;
    driveSpeed += constrain(driveWish - driveSpeed, -dv, dv);

    // Bremsen ist ein eigener Zustand, kein Nebeneffekt. Waere die Bedingung
    // nur "es liegt eine Sollgeschwindigkeit an", fiele die Fahrtregelung in
    // dem Moment aus, in dem der Sollwert null erreicht - der Roboter rollt
    // dann aber noch mit vollem Tempo weiter und wuerde einfach austrudeln.
    // Ein Balancierer bremst, indem er sich GEGEN die Fahrtrichtung lehnt,
    // und dafuer muss der Geschwindigkeitsregler bis zum Stillstand laufen.
    if (running && (fabsf(driveWish) > 1.0f || fabsf(driveSpeed) > 1.0f))
        driveActive = true;
    else if (driveActive && fabsf(wheelSpeed) < STOP_SPEED)
    {
        // Wirklich zum Stehen gekommen: ab hier uebernimmt der Positionsregler
        // und haelt die neue Stelle, statt zum Startpunkt zurueckzufahren.
        driveActive = false;
        driveInt = 0.0f;
        posTarget = encoderPos();
        webuiLog("Fahrt beendet, steht");
    }
    if (!running) driveActive = false;

    const bool driving = running && driveActive;

    if (driving)
    {
        // Fahrbetrieb: auf Geschwindigkeit regeln. Schneller werden heisst
        // weiter nach vorne lehnen, deshalb geht der Fehler mit positivem
        // Vorzeichen in den Sollwinkel.
        const float speedErr = driveSpeed - wheelSpeed;
        driveInt = constrain(driveInt + speedErr * dt, -4000.0f, 4000.0f);
        tiltBias = constrain(Dkp * speedErr + Dki * driveInt,
                             -TILT_BIAS_MAX, TILT_BIAS_MAX);
        posTarget = encoderPos();   // im Fahren zieht ihn nichts zum Start zurueck
    }
    else if (running && (Vkp != 0.0f || Vki != 0.0f))
    {
        const float posErr = (float)(encoderPos() - posTarget);
        // Vorzeichen: ist der Roboter zu weit VORNE (posErr positiv), muss er
        // zurueck - also nach hinten lehnen, der Sollwinkel geht ins Negative.
        float bias = -(Vki * posErr + Vkp * wheelSpeed);
        tiltBias = constrain(bias, -TILT_BIAS_MAX, TILT_BIAS_MAX);
        driveInt = 0.0f;
    }
    else
    {
        tiltBias = 0.0f;
        driveInt = 0.0f;
    }

    float angleErr = angleDeg - trim - tiltBias;

    // Blickrichtung mitfuehren. Fuer die Hochachse gibt es keinen zweiten
    // Sensor, der den Nullpunkt nachzieht (wie der Beschleunigungssensor beim
    // Kippwinkel) - die reine Integration driftet also. Ein Restfehler von
    // 0,1 Grad/s waeren schon 6 Grad je Minute, und der Regler wuerde diese
    // eingebildete Drehung real wegdrehen. Deshalb laeuft der Wert langsam
    // gegen null zurueck: echte Stoerungen faengt der Regler in
    // Sekundenbruchteilen ab und merkt vom Leck nichts, nur das Aufsummieren
    // ueber Minuten wird gebremst.
    constexpr float YAW_LEAK_S = 30.0f;
    yawRateDs = gz - g_gyroBiasZ;
    yawDeg += yawRateDs * dt;
    yawDeg -= yawDeg * (dt / YAW_LEAK_S);

    // Sollrichtung: bei turnRate = 0 bleibt sie stehen, der Gierregler haelt
    // die Spur - das ist das Geradeausfahren. Sie leckt mit derselben
    // Zeitkonstante wie der Istwert, sonst liefen beide auseinander.
    yawTarget += turnRate * dt;
    yawTarget -= yawTarget * (dt / YAW_LEAK_S);

    // --- Sicherheit ---
    // Bewusst die echte Neigung, nicht angleErr: der Positionsregler verschiebt
    // den Sollwinkel um bis zu 4 Grad, und um so viel duerfen Sturz- und
    // Startgrenze nicht mitwandern.
    const float tiltFromTrim = angleDeg - trim;

    // Die Sturzabschaltung gilt nur, WAEHREND er balanciert. Sonst koennte er
    // sich nie aus einer Schraeglage aufschwingen - sie wuerde den Versuch
    // sofort wieder abbrechen.
    if (running && fabsf(tiltFromTrim) > FALL_ANGLE_DEG)
    {
        webuiLogf("STURZ bei %.1f Grad, Drehrate %.0f/s - Motoren aus",
                  tiltFromTrim, gyroRateDs);
        fallFlag = true;
        running = false;
        requested = false;
        swingActive = false;
        driveWish = driveSpeed = driveInt = 0.0f;
        turnRate = yawTarget = 0.0f;
        motor::stop();
        integral = 0.0f;
        lastPwmOut = 0;
        lastSteer  = 0.0f;
        tiltBias   = 0.0f;
    }
    // Waehrend des Aufschwingens darf frueher uebernommen werden: der Roboter
    // hat dann Schwung, und der Regler faengt ihn besser ab, solange er noch
    // in Bewegung ist, als wenn man auf die enge Ruhelage wartet.
    else if (requested && !running &&
             fabsf(tiltFromTrim) < (swingActive ? SWINGUP_HANDOVER_DEG : ARM_ANGLE_DEG))
    {
        running = true;
        swingActive = false;
        swingTries  = 0;
        driveWish = driveSpeed = driveInt = 0.0f;   // nie fahrend losstarten
        turnRate  = yawTarget  = 0.0f;
        integral = 0.0f;
        yawDeg   = 0.0f;   // Richtung im Moment des Starts ist die Sollrichtung
        encoderReset();    // und der Standort im Moment des Starts die Sollposition
        posTarget = 0;
        tiltBias  = 0.0f;
        motor::enable(true);
        webuiLogf("Regler uebernimmt bei %.1f Grad, Drehrate %.0f/s",
                  tiltFromTrim, gyroRateDs);
    }
    else if (requested && !running)
    {
        // Steht er schraeg, erst aufschwingen. Mehr als upMax Grad schafft er
        // mit einem einzigen Anlauf nicht - dann muss ihn jemand aufstellen.
        if (fabsf(tiltFromTrim) < upMax && upPwm > 0.0f)
        {
            const uint32_t now = millis();
            if (!swingActive && now >= swingPauseUntilMs)
            {
                if (swingTries >= SWINGUP_MAX_TRIES)
                {
                    requested = false;
                    swingTries = 0;
                    webuiLogf("Aufschwingen misslungen bei %.1f Grad", tiltFromTrim);
                    webuiNotify("Aufschwingen misslungen");
                }
                else
                {
                    swingActive  = true;
                    swingTries++;
                    swingStartMs = now;
                    motor::enable(true);
                    webuiLogf("Aufschwingen %d/%d aus %.1f Grad, PWM %.0f",
                              swingTries, SWINGUP_MAX_TRIES, tiltFromTrim, upPwm);
                }
            }
            else if (swingActive && now - swingStartMs >= SWINGUP_TIMEOUT_MS)
            {
                // Nicht hochgekommen: kurz absetzen, damit er zurueckpendeln
                // kann, und es aus der neuen Lage noch einmal versuchen.
                swingActive = false;
                motor::stop();
                lastPwmOut = 0;
                swingPauseUntilMs = now + SWINGUP_PAUSE_MS;
                webuiLogf("Anlauf zu kurz, noch %.1f Grad - Pause", tiltFromTrim);
            }
        }
        else if (swingActive)
        {
            swingActive = false;
            motor::stop();
            lastPwmOut = 0;
        }
    }

    // --- Regler ---
    if (running)
    {
        integral = constrain(integral + angleErr * dt, -50.0f, 50.0f);
        float out = Kp * angleErr + Ki * integral + Kd * gyroRateDs;
        out *= outSign;

        lastPwmOut = spreadToPwm(out);

        // Gierregler: PD auf Richtungsfehler und Drehrate. Bewusst NACH der
        // Totzonen-Spreizung addiert - die gilt fuer den Vortrieb, die
        // Raddifferenz soll unveraendert durchkommen.
        float steer = yawSign * (Ykp * (yawDeg - yawTarget)
                                 + Ykd * (yawRateDs - turnRate));
        lastSteer = constrain(steer, -YAW_MAX_STEER, YAW_MAX_STEER);

        motor::set(constrain(lastPwmOut + (int)lastSteer, -255, 255),
                   constrain(lastPwmOut - (int)lastSteer, -255, 255));
    }
    else if (swingActive)
    {
        // In die Kipprichtung fahren: die Raeder laufen unter den Schwerpunkt,
        // der Aufbau richtet sich auf. Wirkrichtung wie beim Balancieren, also
        // ueber outSign - so muss dieses Vorzeichen nicht getrennt bestimmt
        // werden. Nahe der Senkrechten wird zurueckgenommen, sonst schiesst er
        // auf der anderen Seite darueber hinaus.
        const float dir   = (tiltFromTrim > 0.0f) ? 1.0f : -1.0f;
        const float scale = constrain(fabsf(tiltFromTrim) / 25.0f, 0.4f, 1.0f);
        lastPwmOut = (int)(outSign * dir * upPwm * scale);
        lastSteer  = 0.0f;
        yawDeg     = 0.0f;
        motor::set(lastPwmOut, lastPwmOut);
    }
    else
    {
        lastPwmOut = 0;
        lastSteer  = 0.0f;
        yawDeg     = 0.0f;   // im Stillstand nicht aufsummieren
    }

    autotuneTick(dt, angleErr, lastPwmOut, yawDeg, encoderPos());

    if (telemetry)
    {
        Serial.print(millis()); Serial.print(',');
        Serial.print(angleDeg, 2); Serial.print(',');
        Serial.print(gyroRateDs, 2); Serial.print(',');
        Serial.print(lastPwmOut); Serial.print(',');
        Serial.print(yawDeg, 2); Serial.print(',');
        Serial.print((int)lastSteer); Serial.print(',');
        Serial.print(encoderPos() - posTarget); Serial.print(',');
        Serial.print(wheelSpeed, 0); Serial.print(',');
        Serial.println(tiltBias, 2);
    }

    ledsUpdate();

    uint32_t nowMs = millis();
    if (nowMs - lastTelemetryPushMs >= telemetryPeriodMs)
    {
        lastTelemetryPushMs = nowMs;
        webuiSendTelemetry();
    }
}
