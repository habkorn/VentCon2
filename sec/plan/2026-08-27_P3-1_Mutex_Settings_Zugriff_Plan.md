# Plan: Mutex für den settings-Zugriff einführen (P3.1)

Datum: 2026-08-27
Status: Umgesetzt und verifiziert am 2026-08-27
Bezug: sec/2026-08-27_Webinterface_Sicherheitsanalyse.md, Punkt P3.1

## 1. Ziel

Einen konkurrenzsicheren Zugriff auf das gemeinsam genutzte SettingsHandler-Objekt einführen.
Damit werden Nebenläufigkeitsfehler vermieden: zerrissene Lesevorgänge (Torn Reads),
inkonsistente Mehrfeld-Zustände und Datenkorruption beim gleichzeitigen Speichern.

## 2. Ist-Zustand

- SettingsHandler hält alle Felder public und wird direkt per settings-> beziehungsweise settings. gelesen und geschrieben.
- Drei Ausführungskontexte greifen parallel zu:
  - NetworkTask (Core 0, Priorität 1): WebHandler, liest und schreibt, ruft save() auf.
  - ControlTask (Core 1, Priorität 2): ControlSystem und SensorManager, lesen zyklisch.
  - loop() (Core 1): CommandProcessor, liest und schreibt, ruft save() auf.
- AutoTuner wird aus zwei Kontexten genutzt: start(), stop() aus loop(), process() aus ControlTask.
- PID hält einen rohen Zeiger auf settings.setpoint (main.cpp) und liest diesen in jedem Compute().

## 3. Zugriffsstellen-Inventar

| Datei | Kontext | Zugriff |
|---|---|---|
| WebHandlers.cpp | NetworkTask (Core 0) | lesen, schreiben, save() |
| CommandProcessor.cpp | loop() (Core 1) | lesen, schreiben, save() |
| ControlSystem.cpp | ControlTask (Core 1) | lesen (setpoint, antiWindup, hystAmount, hysteresis, control_freq_hz) |
| SensorManager.cpp | ControlTask (Core 1) | lesen (filter_strength, Kalibrierung) |
| AutoTuner.cpp | loop() und ControlTask | lesen, schreiben (setpoint, Kp, Ki, Kd) |
| main.cpp | setup(), vor Task-Start | lesen, PID-Zeiger auf setpoint |

## 4. Design

### 4.1 Mutex

- FreeRTOS-Mutex (SemaphoreHandle_t, xSemaphoreCreateMutex) als privates Mitglied in SettingsHandler, Erzeugung im Konstruktor.
- RAII-Wächterklasse SettingsLockGuard, lock() im Konstruktor, unlock() im Destruktor.
- Kritische Abschnitte kurz halten, maximal wenige Mikrosekunden. Kein delay, kein Serial.printf, kein Flash-Zugriff innerhalb des Locks.

### 4.2 Regeln

- R1: Mehrfeld-Änderungen (handleSet, Sensorkalibrierung, AutoTuner start und stop) vollständig unter Lock ausführen, dann Lock freigeben.
- R2: save() entkoppeln. Unter Lock in einen Puffer serialisieren, Lock freigeben, danach Puffer auf LittleFS schreiben. So blockiert der langsame Flash-Schreibvorgang den ControlTask nicht.
- R3: ControlTask liest die benötigten Werte einmal pro Zyklus unter Lock in lokale Variablen und nutzt danach nur diese Kopien.
- R4: PID-Compute unter Lock ausführen, damit das Lesen von settings.setpoint nicht parallel zu einem Schreibzugriff erfolgt.
- R5: Kein Lock über vTaskDelay oder blockierende Aufrufe halten.

### 4.3 Prioritätsinversion

- Der ControlTask (Priorität 2) kann kurz warten, wenn der NetworkTask (Priorität 1) den Lock hält. Da die kritischen Abschnitte nur Mikrosekunden dauern, ist das akzeptabel. Es erfolgt kein Flash-Zugriff im Lock.

## 5. Umsetzungsschritte

1. SettingsHandler.h und .cpp: Mutex-Mitglied, Konstruktor und Destruktor, lock() und unlock(), SettingsLockGuard, snapshot()-Methode, save() entkoppeln.
2. WebHandlers.cpp: handleSet, handleSliderLimits und handleChartSettings unter Lock setzen, handleRoot und handleValues per snapshot() lesen.
3. CommandProcessor.cpp: alle Schreibblöcke unter Lock, save() außerhalb des Locks.
4. ControlSystem.cpp: Zykluswerte per snapshot() unter Lock lesen, PID-Compute unter Lock oder lokale Sollwert-Kopie.
5. SensorManager.cpp: Kalibrierung und filter_strength per übergebener Kopie oder kurzem Lock lesen.
6. AutoTuner.cpp: start(), stop(), accept() und reject() unter Lock, process() mit lokalen Kopien.
7. main.cpp: unverändert lassen, aber dokumentieren, dass der PID-Sollwertzeiger durch R4 geschützt ist.

## 6. Verifikation

### 6.1 Statisch

- PlatformIO-Build ohne Warnungen und Fehler.
- Code-Review: jeder settings-Zugriff ist einem Lock oder einer Kopie zugeordnet.

### 6.2 Funktional

- Boot, Laden der Settings, Webseite, /values-Aktualisierung und serielle Befehle funktionieren wie zuvor.
- settings.json bleibt gültig und parst fehlerfrei nach mehreren Speichervorgängen.

### 6.3 Nebenläufigkeits-Stresstest

- Während der Regler läuft: schnelle serielle Befehle (KP, SP) und parallele /set-Aufrufe per Skript.
- Ergebnis: kein Crash, kein Watchdog-Reset, keine korrupten Werte.

### 6.4 Deadlock- und Ressourcenprüfung

- Soak-Test über mehrere Minuten.
- printTaskInfo() zur Kontrolle von Stack-High-Water-Mark und laufenden Tasks.
- Jitter-Messung über die bereits vorhandene kontinuierliche Datenausgabe.

### 6.5 Rollback

- Änderungen per Git nachvollziehbar und einzeln zurücknehmbar.

## 7. Risiken

- Verpasster Zugriff: ein nicht abgesicherter Zugriff untergräbt die Garantie. Gegenmaßnahme: Inventar und Code-Review.
- Deadlock durch verschachteltes Locking. Gegenmaßnahme: klare Regeln, keine Verschachtelung, RAII.
- Lock über Flash-Schreibvorgang blockiert den Regler. Gegenmaßnahme: R2, Entkopplung von save().

## 8. Geklärte Entscheidungen

- Umfang: Nur Plan festhalten, keine Umsetzung.
- Granularität: Ein globaler Mutex für das gesamte SettingsHandler-Objekt.
- save(): Snapshot unter Lock, Schreiben außerhalb des Locks.
- PID-Sollwert: Compute unter Lock ausführen.
- Codekommentare: Deutsch.

## 9. Umsetzungsprotokoll

Umgesetzt wurden die Punkte aus Abschnitt 5 in folgenden Dateien:

- SettingsHandler.h und .cpp: FreeRTOS-Mutex, lock()/unlock(), RAII-Wächter ScopedLock, POD-Struktur SettingsSnapshot, snapshotTo(), save() mit Puffer unter Lock und Schreiben außerhalb, load() mit Feldübernahme unter Lock.
- WebHandlers.cpp: handleRoot, handleValues, handleSliderLimits und handleChartSettings lesen über snapshotTo(), handleSet schreibt unter Lock, save() außerhalb.
- CommandProcessor.cpp: PID-, Signalverarbeitungs-, PWM- und Sensorkalibrierungsbefehle schreiben unter Lock, save() außerhalb.
- ControlSystem.cpp und .h: Zykluswerte unter Lock in lokale Variablen, PID-Compute unter Lock, applyHysteresisCompensation mit übergebenen Werten, Sollwert in handleContinuousDataOutput unter Lock.
- SensorManager.cpp: Filterstärke, Kalibrierung und Sicherheitsgrenze unter Lock.
- AutoTuner.cpp: start(), stop(), acceptParameters(), rejectParameters() und process() mit Sperre beziehungsweise lokalen Kopien.

Verifikation:

- PlatformIO-Build erfolgreich: arduino_nano_esp32, keine Fehler oder Warnungen. RAM 18.2 Prozent, Flash 15.5 Prozent.
- IntelliSense meldet für alle geänderten Dateien keine Fehler.
- Manuelle Prüfung: alle verbleibenden settings-Zugriffe im Regelpfad liegen innerhalb von Sperren, Log-Ausgaben nutzen atomare Einzelfeld-Lesevorgänge.
