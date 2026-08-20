#pragma once
#include <Arduino.h>

// ===========================================================================
//  Pinbelegung der Platine "tumlerV2" (ESP32-WROOM-32E)
//  Uebernommen aus dem bestehenden Projekt /home/thomas/tumbler_balance,
//  dort ermittelt aus Sheet2.SchDoc (Altium-Projekt tumlerV2).
// ===========================================================================

// --- I2C (GY-521 / MPU6050) ---
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// --- Motortreiber TB6612FNG ---
// AIN2/BIN2 werden auf der Platine von einem Inverter aus AIN1/BIN1 erzeugt,
// pro Motor gibt es daher nur einen Richtungspin: HIGH = vorwaerts, LOW = rueckwaerts.
constexpr int PIN_STBY = 17;   // HIGH = Treiber aktiv, LOW = Endstufen aus
constexpr int PIN_PWMA =  4;
constexpr int PIN_AIN1 = 16;
constexpr int PIN_PWMB = 18;
constexpr int PIN_BIN1 = 23;

// Die beiden Motoren sitzen spiegelbildlich am Chassis: dieselbe Ansteuerung
// wuerde die Raeder gegeneinander drehen lassen. Motor B wird deshalb invertiert.
constexpr int MOTOR_B_DIR = -1;

// --- Encoder ---
// Der TB6612 treibt mit seinem Kanal A den Stecker M2 und mit Kanal B den
// Stecker M1. Der Encoder des Motors an PWMA/AIN1 liegt deshalb auf M2A/M2B -
// das ist kein Verdrahtungsfehler, nur die Benennung auf der Platine.
constexpr int PIN_ENC_A1 = 19;   // M2A -> Encoder von Motor A (PWMA/AIN1)
constexpr int PIN_ENC_A2 = 25;   // M2B
constexpr int PIN_ENC_B1 = 27;   // M1A -> Encoder von Motor B (PWMB/BIN1)
constexpr int PIN_ENC_B2 = 26;   // M1B

// --- Bedienung ---
// GPIO2 ist ein Strapping-Pin und liegt ueber einen externen Pulldown an Masse;
// der Taster schaltet gegen 3V3 (aktiv HIGH). Der interne Pullup darf NICHT
// aktiviert werden, sonst liest der Pin dauerhaft "gedrueckt".
constexpr int PIN_BUTTON = 2;

// ===========================================================================
//  Sicherheit
// ===========================================================================
constexpr float FALL_ANGLE_DEG    = 35.0f;  // darueber: Motoren sofort aus
constexpr float ARM_ANGLE_DEG     = 5.0f;   // erst darunter darf gestartet werden
constexpr uint32_t LOOP_PERIOD_US = 5000;   // 200 Hz Regeltakt

// ===========================================================================
//  Statusanzeige: 8x WS2812 (Sheet3 des Schaltplans)
// ===========================================================================
// Achtung Strom: 8 Stueck auf voller Helligkeit ziehen rund 0,5 A und haengen
// an derselben Versorgung wie die Motoren - deshalb ist die Helligkeit begrenzt.
constexpr int PIN_RGB        = 15;
constexpr int LED_COUNT      = 8;
constexpr int LED_MAX_BRIGHT = 120;

// ===========================================================================
//  WLAN / Weboberflaeche
// ===========================================================================
// Die Zugangsdaten stehen in include/secrets.h, das absichtlich NICHT im
// Repository liegt. Vorlage: secrets.h.example - kopieren und eintragen.
#include "secrets.h"

#define AP_SSID     "Tumbler"      // Notfall-Accesspoint, wenn kein WLAN da ist
#define AP_PASS     "balance123"
#define HOSTNAME    "tumbler-mini"
