#pragma once
#include <Arduino.h>

// Gemeinsamer Zustand des Balancierreglers (main.cpp), von leds.cpp und
// webui.cpp nur gelesen bzw. ueber handleCommand() veraendert.
extern float angleDeg;
extern float gyroRateDs;
extern bool  running;
extern bool  requested;
extern bool  fallFlag;
extern int   lastPwmOut;

extern float Kp, Ki, Kd, trim, outSign;
extern int   minPwm;

// Gierregler: haelt die Blickrichtung ueber Gyro-Z
extern float yawDeg;
extern float lastSteer;
extern float Ykp, Ykd, yawSign;

// Positionsregler: holt den Roboter ueber die Encoder an den Startpunkt zurueck
extern float tiltBias;
extern float wheelSpeed;
extern long  posTarget;
extern float Vkp, Vki;

void handleCommand(const String &cmd);
