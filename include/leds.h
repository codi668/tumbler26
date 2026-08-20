#pragma once

void ledsBegin();
// Einmal pro Regelzyklus aufrufen - spiegelt Idle/Warte/Balance/Sturz-Zustand
// aus state.h auf den WS2812-Ring.
void ledsUpdate();
