# Webinterface-Sicherheitsanalyse VentCon2

Datum: 2026-08-27
Status: Analyse und Maßnahmenplan; P3.1 (Mutex) und P3.2 (Client-Limit) umgesetzt in v2.7.3, übrige Punkte offen

## 1. Zusammenfassung

Das Webinterface ist die primäre Steueroberfläche für ein Ventil, das Drücke bis etwa 10 bar regelt.
Für sämtliche HTTP-Endpunkte existiert weder Authentifizierung noch Autorisierung noch Ratenbegrenzung.
Die einzige Hürde ist das WLAN-Passwort des Access Points, das fest im Code verdrahtet und öffentlich dokumentiert ist.

Das schwerwiegendste Risiko: Die Sensorkalibrierung kann über das Webinterface verändert werden.
Dadurch wird sowohl die Druckmessung verfälscht als auch die Notabschaltung bei Überdruck ausgehebelt.

## 2. Kritische Befunde

| Nr. | Befund | Datei / Stelle | Risiko |
|---|---|---|---|
| 1 | Keine Authentifizierung auf der Steueroberfläche | WebHandlers.cpp: setupRoutes() | Kritisch |
| 2 | Schwaches, fest verdrahtetes, öffentlich dokumentiertes AP-Passwort "ventcon12!" | Constants.h: NetworkConfig::AP_PASSWORD | Kritisch |
| 3 | Sensorkalibrierung per HTTP änderbar, hebelt Notabschaltung aus | WebHandlers.cpp: handleSet(); SensorManager.cpp: calculatePressure(), isPressureSafe() | Kritisch |

Details:

1. Alle mutierenden Endpunkte (/set, /resetPID, /api/slider-limits, /api/chart-settings) sind ohne Anmeldung erreichbar.
2. Das Passwort ist im Quelltext, in der Dokumentation und im Benutzerhandbuch hinterlegt.
3. Die Druckberechnung leitet sich direkt aus den Kalibrierwerten ab. Ein Angreifer kann sensor_maxP erhöhen, wodurch die Firmware den tatsächlichen Druck unterschätzt. Die Notabschaltung vergleicht den gemeldeten Druck mit min(11,0 bar, sensor_maxP x 1,1) und löst dadurch nie aus, während der reale Druck weit über dem Grenzwert liegt.

## 3. Weitere Befunde

4. Zustandsänderungen per HTTP GET, dadurch CSRF-anfällig. /set und /resetPID mutieren per GET, ohne Origin- oder Token-Prüfung.
5. Beliebiger Dateizugriff und Informationsabfluss. onNotFound leitet webServer.uri() direkt an handleFileRead() weiter. /settings.json (Kalibrierung, PID-Gains) und /default.json sind direkt abrufbar, eine Pfadnormalisierung fehlt.
6. Unbegrenzte PID-Gains. /api/slider-limits erlaubt beliebig hohe kp/ki/kd-Obergrenzen (nur max > min wird geprüft). Dadurch sind aggressive Regler möglich (Schwingungen, Ventilhämmern).
7. Flash-Abnutzung und DoS über settings.save(). Jeder /set-Aufruf schreibt synchron auf LittleFS. Unbegrenzte Schreibzugriffe verschleißen den Flash und blockieren den Server.
8. ~~Client-Anzahl-Handler stört die Verfügbarkeit. Beim dritten verbundenen Client trennt onWiFiEvent() alle Clients (softAPdisconnect) und startet den AP neu.~~ **Behoben in v2.7.3** (passive Ablehnung des neuen Clients; AP erzwingt das Limit über `max_connection = MAX_CLIENTS`, siehe P3.2).
9. ~~Fehlende Synchronisation. settings wird nebenläufig von Netzwerk- und Regler-Task ohne Mutex gelesen und geschrieben.~~ **Behoben in v2.7.3** (FreeRTOS-Mutex in `SettingsHandler`, siehe P3.1 und `sec/2026-08-27_Mutex_Settings_Zugriff_Plan.md`).

## 4. Positiv zu bewerten

- Die meisten Skalarwerte werden in handleSet() auf Bereiche geprüft (sp, kp/ki/kd, flt, freq, res, pst).
- Der Sollwert ist hart begrenzt auf min(sp_limits.max, sensor_max_pressure, HARD_SP_MAX_BAR = 10,0).
- JSON- und HTML-Ausgaben nutzen begrenztes snprintf und numerische Ersetzungen, daher keine klassische Zeichenketten-Injektion.

## 5. Implementierungsplan

### Phase 1: Sicherheitskritisch, zuerst umsetzen

- P1.1 Authentifizierung für alle mutierenden Endpunkte einführen. Gewählter Ansatz: Login-Seite mit Passwort.
- P1.2 /set und /resetPID auf POST umstellen und CSRF-Schutz ergänzen.
- P1.3 Sensorkalibrierung schützen und auf den physischen Sensorbereich begrenzen (sensor_maxP kleiner oder gleich SENSOR_MAX_BAR).
- P1.4 Harte Obergrenzen für PID-Gains unabhängig von den Slider-Limits einführen.

### Phase 2: Härtung

- P2.1 Pfadbereinigung in handleFileRead() und Auslieferung von JSON-Konfiguration über HTTP unterbinden.
- P2.2 AP-Passwort zur Laufzeit konfigurierbar machen, über Web oder Settings.
- P2.3 Ratenbegrenzung und Save-Debouncing gegen Flash-Verschleiß ergänzen.

### Phase 3: Robustheit

- P3.1 Mutex für den settings-Zugriff einführen. **Erledigt in v2.7.3** (siehe `sec/2026-08-27_Mutex_Settings_Zugriff_Plan.md` und `sec/2026-08-28_Mutex_How_It_Works.md`).
- P3.2 Client-Connect-DoS beheben, neue Clients ablehnen statt alle bestehenden zu trennen. **Erledigt in v2.7.3** (passive Ablehnung in `onWiFiEvent()`, kein `softAPdisconnect` mehr im Verbindungsfall).
- P3.3 Verschlüsselung oder Authentifizierung auf Funkebene prüfen, zum Beispiel WPA2 Enterprise oder ein verschlüsseltes Anwendungsprotokoll.

## 6. Geklärte Entscheidungen

- Umfang: Nur Plan, noch nichts umsetzen.
- Authentifizierung: Login-Seite mit Passwort.
- Sensorkalibrierung: Änderung weiterhin über Web, aber mit hartem Sensorbereich-Limit.
- AP-Passwort: Zur Laufzeit konfigurierbar, über Web oder Settings.
- Codekommentare: Deutsch.
