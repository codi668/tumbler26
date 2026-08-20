#include "encoder.h"
#include "config.h"
#include <driver/pcnt.h>

namespace {

constexpr pcnt_unit_t UNIT_A = PCNT_UNIT_0;
constexpr pcnt_unit_t UNIT_B = PCNT_UNIT_1;

long  position = 0;
float speed    = 0.0f;

// Ein PCNT-Zaehler mit beiden Kanaelen ergibt Vierfachauswertung: gezaehlt
// werden beide Flanken der einen Spur, die Richtung gibt die andere vor.
void setupUnit(pcnt_unit_t unit, int pin1, int pin2)
{
    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = pin1;
    ch0.ctrl_gpio_num  = pin2;
    ch0.channel        = PCNT_CHANNEL_0;
    ch0.unit           = unit;
    ch0.pos_mode       = PCNT_COUNT_INC;
    ch0.neg_mode       = PCNT_COUNT_DEC;
    ch0.lctrl_mode     = PCNT_MODE_REVERSE;
    ch0.hctrl_mode     = PCNT_MODE_KEEP;
    ch0.counter_h_lim  = 30000;
    ch0.counter_l_lim  = -30000;
    pcnt_unit_config(&ch0);

    pcnt_config_t ch1 = ch0;
    ch1.pulse_gpio_num = pin2;
    ch1.ctrl_gpio_num  = pin1;
    ch1.channel        = PCNT_CHANNEL_1;
    ch1.pos_mode       = PCNT_COUNT_DEC;
    ch1.neg_mode       = PCNT_COUNT_INC;
    pcnt_unit_config(&ch1);

    pcnt_set_filter_value(unit, 250);   // kurze Stoerimpulse ausblenden
    pcnt_filter_enable(unit);

    gpio_pullup_en((gpio_num_t)pin1);
    gpio_pullup_en((gpio_num_t)pin2);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

// Zaehlerstand abholen und zuruecksetzen - so kann der 16-Bit-Zaehler gar
// nicht erst ueberlaufen, solange oft genug gepollt wird.
long take(pcnt_unit_t unit)
{
    int16_t v = 0;
    pcnt_get_counter_value(unit, &v);
    pcnt_counter_clear(unit);
    return v;
}

} // namespace

void encoderBegin()
{
    setupUnit(UNIT_A, PIN_ENC_A1, PIN_ENC_A2);
    setupUnit(UNIT_B, PIN_ENC_B1, PIN_ENC_B2);
    encoderReset();
}

void encoderPoll(float dt)
{
    const long dA = take(UNIT_A);
    const long dB = MOTOR_B_DIR * take(UNIT_B);
    const float step = 0.5f * (dA + dB);   // Mittelwert = Fahrweg des Roboters

    position += (long)step;

    if (dt > 1e-5f)
    {
        // Kraeftiger Tiefpass: der rohe Wert springt bei den groben Impulsen
        // je Zyklus stark, und ein zappelndes Tempo wuerde der aeussere Regler
        // direkt in den Sollwinkel weiterreichen.
        speed += 0.15f * (step / dt - speed);
    }
}

void encoderReset()
{
    pcnt_counter_clear(UNIT_A);
    pcnt_counter_clear(UNIT_B);
    position = 0;
    speed    = 0.0f;
}

long  encoderPos()   { return position; }
float encoderSpeed() { return speed; }
