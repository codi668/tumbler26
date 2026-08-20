#pragma once

// Radencoder ueber die PCNT-Hardware des ESP32 (Vierfachauswertung).
// "Positiv = vorwaerts" gilt fuer beide Raeder, MOTOR_B_DIR ist eingerechnet.
void  encoderBegin();
void  encoderPoll(float dt);   // einmal je Regelzyklus
void  encoderReset();          // Weg und Geschwindigkeit auf null
long  encoderPos();            // Mittelwert beider Raeder in Impulsen
float encoderSpeed();          // gefiltert, Impulse/s
