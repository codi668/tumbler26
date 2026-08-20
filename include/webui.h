#pragma once

// WLAN + Weboberflaeche: verbindet sich mit WIFI_SSID/WIFI_PASS aus config.h,
// spannt bei Fehlschlag den Accesspoint AP_SSID auf. Erreichbar unter
// http://<HOSTNAME>.local/ bzw. der jeweiligen IP.
void webuiBegin();
void webuiLoop();          // in loop() aufrufen (DNS fuer den AP-Fall)
void webuiSendTelemetry();          // aktuellen Zustand an alle Clients senden
void webuiNotify(const char *msg);  // kurze Rueckmeldung in der Oberflaeche anzeigen

// Ereignisprotokoll: haelt fest, was der Roboter tatsaechlich getan hat -
// empfangene Befehle, Zustandswechsel, Gyro-Nullpunkte, Stuerze. Die letzten
// Zeilen bekommt auch, wer sich erst spaeter verbindet.
void webuiLog(const char *msg);
void webuiLogf(const char *fmt, ...);
