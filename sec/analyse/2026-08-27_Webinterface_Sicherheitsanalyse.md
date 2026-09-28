# Webinterface-Sicherheitsanalyse VentCon2

Rolle: Risiko- und Evidenzanalyse (Detaildokument, kein Umsetzungsplan)
Grundlage: `../CRA_Compliance_Checklist.md`
Risikobezug: R1 bis R9 (R1 bis R3 kritisch, R4 bis R9 weitere Befunde)
Zuordnung: Die Risiko-IDs sind in Abschnitt 5 des Masters den Massnahmen P1.1 bis P3.3 zugeordnet
Datum: 2026-08-27, Stand der Risikobewertung 2026-09-28
Hinweis: Der Umsetzungsplan wird nicht mehr hier gefuehrt, sondern im Master
(`../CRA_Compliance_Checklist.md`, Abschnitt 6). Dieses Dokument liefert die Befunde,
die Datei- und Funktionsstellen und die Evidenz.

## 1. Zusammenfassung

Das Webinterface ist die primäre Steueroberfläche für ein Ventil, das Drücke bis etwa 10 bar regelt.
Für sämtliche HTTP-Endpunkte existiert weder Authentifizierung noch Autorisierung noch Ratenbegrenzung.
Die einzige Hürde ist das WLAN-Passwort des Access Points, das fest im Code verdrahtet und öffentlich dokumentiert ist.

Das schwerwiegendste Risiko: Die Sensorkalibrierung kann über das Webinterface verändert werden.
Dadurch wird sowohl die Druckmessung verfälscht als auch die Notabschaltung bei Überdruck ausgehebelt.

## 2. Kritische Befunde (R1 bis R3)

| ID | Befund | Datei / Stelle | Risiko | Massnahme |
|---|---|---|---|---|
| R1 | Keine Authentifizierung auf der Steueroberfläche | WebHandlers.cpp: setupRoutes() | Kritisch | P1.1 |
| R2 | Schwaches, fest verdrahtetes, öffentlich dokumentiertes AP-Passwort "ventcon12!" | Constants.h: NetworkConfig::AP_PASSWORD | Kritisch | P2.2 |
| R3 | Sensorkalibrierung per HTTP änderbar, bildet den Druck zu niedrig ab | WebHandlers.cpp: handleSet(); SensorManager.cpp: calculatePressure(), isPressureSafe() | Kritisch | P1.3 |

Details:

- **R1:** Alle mutierenden Endpunkte (/set, /resetPID, /api/slider-limits, /api/chart-settings) sind ohne Anmeldung erreichbar.
- **R2:** Das Passwort ist im Quelltext, in der Dokumentation und im Benutzerhandbuch hinterlegt.
- **R3:** Die Druckberechnung leitet sich direkt aus den Kalibrierwerten ab. Ein Angreifer kann sensor_maxP erhöhen, wodurch die Firmware den tatsächlichen Druck unterschätzt. Die Notabschaltung vergleicht den gemeldeten Druck mit min(11,0 bar, sensor_maxP x 1,1) und löst dadurch nie aus, während der reale Druck weit über dem Grenzwert liegt. Die Schwelle selbst bleibt bestehen, die Messtransformation wird verfälscht.

## 3. Weitere Befunde (R4 bis R9)

- **R4 Zustandsänderungen per HTTP GET, dadurch CSRF-anfällig.** /set und /resetPID mutieren per GET, ohne Origin- oder Token-Prüfung. Massnahme P1.2.
- **R5 Beliebiger Dateizugriff und Informationsabfluss.** onNotFound leitet webServer.uri() direkt an handleFileRead() weiter. /settings.json (Kalibrierung, PID-Gains) und /default.json sind direkt abrufbar, eine Pfadnormalisierung fehlt. Massnahme P2.1.
- **R6 Unbegrenzte PID-Gains.** /api/slider-limits erlaubt beliebig hohe kp/ki/kd-Obergrenzen (nur max > min wird geprüft). Dadurch sind aggressive Regler möglich (Schwingungen, Ventilhämmern). Massnahme P1.4.
- **R7 Flash-Abnutzung und DoS über settings.save().** Jeder /set-Aufruf schreibt synchron auf LittleFS. Unbegrenzte Schreibzugriffe verschleißen den Flash und blockieren den Server. Massnahme P2.3.
- **R8 Client-Anzahl-Handler störte die Verfügbarkeit.** Beim dritten verbundenen Client trennte onWiFiEvent() alle Clients (softAPdisconnect) und startete den AP neu. **Behoben in v2.7.3** (passive Ablehnung des neuen Clients; der AP erzwingt das Limit über `max_connection = MAX_CLIENTS`). Massnahme P3.2, kein eigenes Detaildokument.
- **R9 Fehlende Synchronisation der settings-Zugriffe.** settings wurde nebenläufig von Netzwerk- und Regler-Task ohne Mutex gelesen und geschrieben. **Behoben in v2.7.3** (FreeRTOS-Mutex in `SettingsHandler`). Massnahme P3.1, siehe `../plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` und `../grundlagen/2026-08-28_Mutex_How_It_Works.md`.

## 4. Positiv zu bewerten

- Die meisten Skalarwerte werden in handleSet() auf Bereiche geprüft (sp, kp/ki/kd, flt, freq, res, pst).
- Der Sollwert ist hart begrenzt auf min(sp_limits.max, sensor_max_pressure, HARD_SP_MAX_BAR = 10,0).
- JSON- und HTML-Ausgaben nutzen begrenztes snprintf und numerische Ersetzungen, daher keine klassische Zeichenketten-Injektion.

## 5. Umsetzung

Der Implementierungsplan wird nicht mehr in diesem Dokument geführt, damit Massnahmen-IDs und
Status nur an einer Stelle gepflegt werden. Verbindlich sind:

- Massnahmen, Status und Priorisierung: `../CRA_Compliance_Checklist.md`, Abschnitt 6
- Zuordnung Risiko, Pflicht, Massnahme und Detaildokument: `../CRA_Compliance_Checklist.md`, Abschnitt 5

Detaildokumente: `../plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` (P1.1, P1.2, P2.1),
`../plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` (P1.3),
`../plan/2026-08-31_P2-2_AP_Password_Plan.md` (P2.2),
`../plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` (P3.1).
Für P1.4, P2.3 und P3.3 liegen Skelette unter `../plan/` bereit.

## 6. Geklärte Entscheidungen (Eingang in den Master)

- Umfang: Nur Plan, noch nichts umsetzen.
- Authentifizierung: Login-Seite mit Passwort; Erstinbetriebnahme-Provisioning ohne universelles Standardpasswort.
- Sensorkalibrierung: Änderung weiterhin über Web, aber mit hartem Sensorbereich-Limit.
- AP-Passwort: Zur Laufzeit konfigurierbar, über Web oder Settings.
- Erstinbetriebnahme: Kein universelles Standardpasswort; erzwungene Passwortvergabe (Provisioning) vor Freischaltung der Steuerung (CRA-konform, ETSI EN 303 645 5.1-1).
- Codekommentare: Deutsch.
