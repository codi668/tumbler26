#pragma once
#include <Arduino.h>

// Automatische Reglerabstimmung, die im Roboter selbst laeuft.
//
// Kein blockierender Ablauf: der Zustandsautomat wird aus dem Regeltakt
// heraus getickt und darf ihn nie aufhalten. Jeder Versuch ist ein echter
// physikalischer Testlauf von einigen Sekunden - faellt der Roboter dabei um,
// zaehlt das als schlechtestes Ergebnis, und der Automat wartet, bis jemand
// ihn wieder aufstellt.

void autotuneStart();
void autotuneStop(bool keepBest);
bool autotuneActive();
void autotuneTick(float dt, float angleErr, int pwm, float yaw, long pos);

// Fuer die Anzeige in der Weboberflaeche
int         autotuneStage();     // laufende Stufe, -1 wenn untaetig
int         autotuneTrial();     // Anzahl gefahrener Versuche
float       autotuneBestCost();  // bester Kostenwert der laufenden Stufe
const char *autotunePhase();     // was er gerade tut, kurzer Text
