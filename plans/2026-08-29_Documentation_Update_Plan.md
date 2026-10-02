# VentCon2 LaTeX Documentation Update Plan

Datum: 2026-08-29
Status: Restrukturierung und Inhalts-Updates erledigt (2026-08-29)
Betrifft: `LaTeX/doc/VentCon2_Documentation.tex` (jetzt gesplittet) und `LaTeX/doc/Dependencies_Matrix.tex`

## 0. Erledigt: Restrukturierung

- Gesplittet in `LaTeX/doc/sections/01_executive_summary.tex` bis `11_conclusion.tex`.
- Neuer Abschnitt `12_security_compliance.tex` (Security & Compliance, englisch).
- Version auf 2.7.3 angehoben (Footer und Titel).
- Monolith-Backup: `LaTeX/doc/VentCon2_Documentation_monolith_2026-08-29.tex`.
- Hauptdatei nutzt jetzt `\input{sections/...}` für alle 12 Abschnitte.

## 1. 01_executive_summary.tex

- [ ] "medical ventilator applications" -> "pressure control demonstrator" (VENTREX elektronischer Druckregler).
- [ ] "solenoid valve" -> "proportional valve".
- [ ] Key Features prüfen: dual ADC, PROGMEM streaming, chunked serving, Auto-Tune, Multi-Core FreeRTOS, LittleFS, Serial, WiFi AP + Channel-Wahl sind alle noch vorhanden.
- [ ] Bullet ergänzen: Security & CRA-Compliance (Verweis auf Abschnitt 12).

## 2. 02_system_overview.tex

- [ ] "This cycle repeats continuously at 100 Hz (or user-configured frequency)" -> Control-Loop bis 1000 Hz (default), PID-Berechnung 100 Hz (10 ms Sample Time).
- [ ] "solenoid valve" -> "proportional valve" (zweimal: Closed-Loop und Signal Chain).
- [ ] "medical ventilator applications" umformulieren.
- [ ] External Interfaces: "Analog Output" prüfen (handleAnalogPressureOutput existiert in ControlSystem.cpp).

## 3. 03_hardware_platform.tex

- [ ] Abschnittstitel "Solenoid Valve" -> "Proportional Valve"; Fließtext anpassen.
- [ ] "320 KB RAM" -> "512 KB SRAM" (ESP32-S3).
- [ ] "16MB with custom partition table (6.25MB app, 3.375MB LittleFS)" -> "16MB flash, default_16MB.csv Partitionstabelle" (App-/LittleFS-Größen verifizieren).
- [ ] "medical ventilator" umformulieren.
- [ ] Sensor "±2% accuracy" gegen Datenblatt prüfen (BOSCH MPS1 Ressourcen in `resources/`).

## 4. 04_control_theory.tex

- [ ] VALVE_MIN_DUTY 50.0f -> 20.0f im Constants-Listing.
- [ ] Mapping-Tabelle neu rechnen (PID% -> Valve Duty) für VALVE_MIN_DUTY = 20 (Werte 50.4/60/70/80/90 ändern sich alle).
- [ ] Dead-Zone-Abschnitt: "50%" Referenzen -> "20%".
- [ ] Anti-Windup-Abschnitt: Schwellen "50%/90%" -> "20%/90%".
- [ ] "Parallel form passed directly without conversion" -> Hinweis, dass die Web-UI jetzt Ti/Td-Zeitkonstanten anzeigt (Ki (Ti=...), Kd (Td=...)).
- [ ] PID-Library-Version prüfen ("Arduino-PID-Library by Brett Beauregard, v1.2.1") gegen platformio.ini (imax9000/Arduino-PID-Library Fork).
- [ ] "solenoid valve" -> "proportional valve".

## 5. 05_software_architecture.tex

- [ ] Logger-Klasse ins Architektur-Diagramm (ASCII) aufnehmen und bei "eight specialized classes" berücksichtigen.
- [ ] "Observer Pattern" Behauptung prüfen/entfernen (nicht klar implementiert).
- [ ] Control-Loop-Frequenz (1000 Hz) vs PID (100 Hz) in Task-Distribution vermerken.

## 6. 06_class_reference.tex

- [ ] Logger-Klasse als Unterabschnitt ergänzen (Logger.h: init(), Log-Level).
- [ ] SettingsHandler-Tabelle: printSettings() -> printStoredSettings(); Mutex-API (update/withLock/snapshotTo), Slider-Limits, Sensorkalibrierung, chart_settings ergänzen.
- [ ] SettingsHandler "Actual Implementation"-Listing: sensor_min/max_pressure/voltage, chart_settings, Mutex ergänzen; printSettings() entfernen.
- [ ] SensorManager-Tabelle: getFilteredPtr() -> getLastFilteredPressurePtr(); getRawPressure/getVoltage/getADCValue/isADSFound/isPressureSafe/resetFilterState ergänzen.
- [ ] WebHandler-Tabelle: "DNSServer webDnsServer / Captive Portal" -> ESPmDNS (ventcon.local); handleResetPID, handleSliderLimits, handleChartSettings, connectedMACs, changeWiFiChannel ergänzen.
- [ ] ControlSystem-Tabelle: Methoden gegen ControlSystem.h prüfen (applyHysteresisCompensation etc.).
- [ ] AutoTuner-Tabelle: Attribute gegen AutoTuner.h prüfen.
- [ ] Constants.h-Tabelle: VALVE_MIN_DUTY 50.0% -> 20.0%; OperatingDefaults, ControlConfig, AutoTuneConfig, Esp32AdcConfig, SettingsDefaults, ChartDefaults, SliderDefaults ergänzen.

## 7. 07_communication_interfaces.tex

- [ ] Endpoint-Tabelle: /moment.min.js und /chartjs-adapter-moment.min.js entfernen (nicht mehr in data/); /api/chart-settings, /settings.json, /default.json ergänzen.
- [ ] Network Config: "DNS: Captive portal with wildcard DNS" -> "mDNS: http://ventcon.local".
- [ ] WebHandler-Listing: DNSServer webDnsServer entfernen.
- [ ] "Security and Access Control" Unterabschnitt: falsche Behauptungen entfernen (MAC Filtering, Session Timeout, Audit Logging sind nicht implementiert); Querverweis auf Abschnitt 12.
- [ ] chart.min.js Größenangabe (199KB) und Circular-Buffer-Größe (60 Punkte) verifizieren.

## 8. 08_system_integration.tex

- [ ] Initialisierungssequenz: Logger::init() (als erstes), settings-Mutex, LittleFS begin(false) ergänzen.
- [ ] Timing-Tabelle: Control Loop 1000 Hz, Sensor 1000 Hz, PID 100 Hz, Web 10 Hz bestätigen.
- [ ] Safety Mechanisms: "mutex-protected settings access" und isPressureSafe() ergänzen; Kalibrierungs-Finding (Abschnitt 12) vermerken.

## 9. 09_configuration_deployment.tex

- [ ] Default Parameters: VALVE_MIN_DUTY 50.0% -> 20.0%; Sensor-Kalibrierdefaults (0.5-4.5V, 0-10 bar), Slider-Limits, Chart-Settings ergänzen.
- [ ] Build-Config-Listing: platformio.ini angleichen (upload_protocol esptool, board_upload.before_reset usb_reset, extra_scripts inject_version.py + usb_reset_pre_upload.py, monitor flags).

## 10. 10_performance.tex

- [ ] "RAM ~58 KB of 320 KB (17.7%)" -> gegen 512 KB neu rechnen.
- [ ] "Flash ~956 KB of 6.25 MB" -> gegen default_16MB.csv App-Partition neu rechnen.
- [ ] Boot-Zeit und Netzwerk-Latenz-Angaben verifizieren.

## 11. 11_conclusion.tex

- [ ] "medical-grade applications" -> Demonstrator / industrielle Druckregelung.
- [ ] Security & Compliance in Stärken / Future Enhancements ergänzen.

## 12. 12_security_compliance.tex

- [x] Erstellt (englische Zusammenfassung der sec/-Findings + CRA).
- [ ] Aktualisieren, sobald Phase-1/2-Security-Fixes umgesetzt sind.

## 13. Dependencies_Matrix.tex (separate Datei)

- [ ] Logger-Klasse in Matrix und Abhängigkeitstabellen ergänzen.
- [ ] WebHandler-Abhängigkeit: DNSServer -> ESPmDNS.
- [ ] Versionspinning / Datum ergänzen.

## Verifikation

- [ ] pdflatex VentCon2_Documentation.tex kompiliert sauber (keine fehlenden \input, keine undefined references).
- [ ] 12 Abschnittsdateien, jede mit genau einem \section.
- [ ] Version 2.7.3 in Footer und Titel.
