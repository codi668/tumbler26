#include "leds.h"
#include "config.h"
#include "state.h"
#include <Adafruit_NeoPixel.h>

namespace {

Adafruit_NeoPixel strip(LED_COUNT, PIN_RGB, NEO_GRB + NEO_KHZ800);

uint32_t lastUpdateMs = 0;

uint32_t colorLerp(uint8_t r0, uint8_t g0, uint8_t b0,
                    uint8_t r1, uint8_t g1, uint8_t b1, float t)
{
    t = constrain(t, 0.0f, 1.0f);
    return strip.Color(r0 + (r1 - r0) * t, g0 + (g1 - g0) * t, b0 + (b1 - b0) * t);
}

} // namespace

void ledsBegin()
{
    strip.begin();
    strip.setBrightness(LED_MAX_BRIGHT);
    strip.clear();
    strip.show();
}

void ledsUpdate()
{
    uint32_t now = millis();
    if (now - lastUpdateMs < 30) return;   // ~33 Hz reicht fuers Auge, schont die Leitung
    lastUpdateMs = now;

    uint32_t color;

    if (fallFlag && !running)
    {
        // Sturz: alles rot, schnelles Blinken, bis ZERO/START/Taster zuruecksetzt.
        bool on = (now / 150) % 2 == 0;
        color = on ? strip.Color(255, 0, 0) : 0;
    }
    else if (running)
    {
        // Gruen bei kleinem Winkelfehler, wandert ueber Gelb nach Rot bei groesserem.
        float err = fabsf(angleDeg - trim);
        float t = err / 8.0f;   // ab 8 Grad voll rot
        color = colorLerp(0, 200, 60, 220, 20, 0, t);
    }
    else if (requested)
    {
        // Wartet auf ARM_ANGLE_DEG: pulsierendes Gelb.
        float phase = (sinf(now * 0.006f) + 1.0f) * 0.5f;
        color = colorLerp(40, 25, 0, 200, 140, 0, phase);
    }
    else
    {
        // Idle: langsam atmendes, gedaemptes Blau.
        float phase = (sinf(now * 0.0015f) + 1.0f) * 0.5f;
        color = colorLerp(0, 0, 10, 0, 40, 90, phase);
    }

    for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, color);
    strip.show();
}
