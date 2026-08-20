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
int   minPwm  = 50;      // darunter bewegt sich der Roboter unter Eigengewicht nicht
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

    Serial.print(F("ZERO: Startwinkel="));
    Serial.print(angleDeg, 2);
    Serial.print(F(" Gyro-Bias="));
    Serial.println(g_gyroBias, 3);
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
    prefs.putInt("minpwm", minPwm);
    prefs.putFloat("trim", trim);
    prefs.putFloat("sign", outSign);
    prefs.putFloat("ykp", Ykp);
    prefs.putFloat("ykd", Ykd);
    prefs.putFloat("ysign", yawSign);
    prefs.putFloat("vkp", Vkp);
    prefs.putFloat("vki", Vki);
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
    minPwm  = prefs.getInt("minpwm", minPwm);
    trim    = prefs.getFloat("trim", trim);
    outSign = prefs.getFloat("sign", outSign);
    Ykp     = prefs.getFloat("ykp", Ykp);
    Ykd     = prefs.getFloat("ykd", Ykd);
    yawSign = prefs.getFloat("ysign", yawSign);
    Vkp     = prefs.getFloat("vkp", Vkp);
    Vki     = prefs.getFloat("vki", Vki);
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
    Serial.print(F(" minPwm=")); Serial.print(minPwm);
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
    Serial.print(F(" VI=")); Serial.println(Vki, 4);
}

void handleCommand(const String &cmdIn)
{
    String cmd = cmdIn;
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "START") { requested = true; fallFlag = false; }
    else if (cmd == "STOP") { requested = false; running = false; motor::stop(); Serial.println(F("STOP")); }
    else if (cmd == "ZERO") { zeroAngle(); fallFlag = false; }
    else if (cmd == "STATUS") { printStatus(); }
    else if (cmd == "T") { telemetry = !telemetry; }
    else if (cmd.startsWith("P=")) { Kp = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("I=")) { Ki = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("D=")) { Kd = cmd.substring(2).toFloat(); }
    else if (cmd.startsWith("MINPWM=")) { minPwm = cmd.substring(7).toInt(); }
    else if (cmd.startsWith("TRIM=")) { trim = cmd.substring(5).toFloat(); }
    else if (cmd.startsWith("SIGN=")) { outSign = cmd.substring(5).toFloat() >= 0 ? 1.0f : -1.0f; }
    else if (cmd.startsWith("YP=")) { Ykp = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("YD=")) { Ykd = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("YSIGN=")) { yawSign = cmd.substring(6).toFloat() >= 0 ? 1.0f : -1.0f; }
    else if (cmd.startsWith("VP=")) { Vkp = cmd.substring(3).toFloat(); }
    else if (cmd.startsWith("VI=")) { Vki = cmd.substring(3).toFloat(); }
    else if (cmd == "HOME") { encoderReset(); posTarget = 0; Serial.println(F("Ausgangslage neu gesetzt")); }
    else if (cmd.startsWith("RATE=")) {
        telemetryPeriodMs = constrain(cmd.substring(5).toInt(), 10, 1000);
    }
    else if (cmd == "SAVE") { saveParams(); }
    else if (cmd == "LOAD") { loadParams(); printStatus(); }
    else if (cmd == "?") {
        Serial.println(F("START STOP ZERO STATUS T  P= I= D= MINPWM= TRIM= SIGN="));
        Serial.println(F("Gierregler: YP= YD= YSIGN=  (YP=0 YD=0 schaltet ihn ab)"));
        Serial.println(F("Positionsregler: VP= VI= HOME  (VP=0 VI=0 schaltet ihn ab)"));
        Serial.println(F("SAVE LOAD - Parameter im Flash ablegen/zurueckholen"));
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
            if (buf.length() > 0) { handleCommand(buf); buf = ""; }
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
            if (running || requested) { requested = false; running = false; motor::stop(); }
            else { requested = true; fallFlag = false; }
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
    if (running && (Vkp != 0.0f || Vki != 0.0f))
    {
        const float posErr = (float)(encoderPos() - posTarget);
        // Vorzeichen: ist der Roboter zu weit VORNE (posErr positiv), muss er
        // zurueck - also nach hinten lehnen, der Sollwinkel geht ins Negative.
        float bias = -(Vki * posErr + Vkp * wheelSpeed);
        tiltBias = constrain(bias, -TILT_BIAS_MAX, TILT_BIAS_MAX);
    }
    else
    {
        tiltBias = 0.0f;
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

    // --- Sicherheit ---
    // Bewusst die echte Neigung, nicht angleErr: der Positionsregler verschiebt
    // den Sollwinkel um bis zu 4 Grad, und um so viel duerfen Sturz- und
    // Startgrenze nicht mitwandern.
    const float tiltFromTrim = angleDeg - trim;
    if (fabsf(tiltFromTrim) > FALL_ANGLE_DEG)
    {
        if (running) { Serial.println(F("Umgefallen - Motoren aus.")); fallFlag = true; }
        running = false;
        requested = false;
        motor::stop();
        integral = 0.0f;
        lastPwmOut = 0;
        lastSteer  = 0.0f;
        tiltBias   = 0.0f;
    }
    else if (requested && !running && fabsf(tiltFromTrim) < ARM_ANGLE_DEG)
    {
        running = true;
        integral = 0.0f;
        yawDeg   = 0.0f;   // Richtung im Moment des Starts ist die Sollrichtung
        encoderReset();    // und der Standort im Moment des Starts die Sollposition
        posTarget = 0;
        tiltBias  = 0.0f;
        motor::enable(true);
        Serial.println(F("START"));
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
        float steer = yawSign * (Ykp * yawDeg + Ykd * yawRateDs);
        lastSteer = constrain(steer, -YAW_MAX_STEER, YAW_MAX_STEER);

        motor::set(constrain(lastPwmOut + (int)lastSteer, -255, 255),
                   constrain(lastPwmOut - (int)lastSteer, -255, 255));
    }
    else
    {
        lastPwmOut = 0;
        lastSteer  = 0.0f;
        yawDeg     = 0.0f;   // im Stillstand nicht aufsummieren
    }

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
