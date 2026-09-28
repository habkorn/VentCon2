# Plan: Rate Limiting and Save Debouncing (P2.3)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P2.3
CRA-Bezug: Annex I, Teil I, 1.3 (Pflicht-Nr. 8)
Risikobezug: R7
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Abhaengigkeiten: P1.1 und P1.2 (Auth und POST) reduzieren die Erreichbarkeit, ersetzen aber keine Ratenbegrenzung
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Protect the LittleFS flash from write wear and the web server from request flooding, without
delaying legitimate parameter changes or losing confirmed settings on a power cut.

## 2. Current State (verified 2026-09-28)

- `settings->save()` runs on every mutation: `WebHandlers.cpp` (`handleSet`, `handleSliderLimits`,
  `handleChartSettings`) and in most handlers of `src/CommandProcessor.cpp`.
- Since v2.7.3 `save()` serialises to a `String` buffer under the settings mutex and writes outside
  the lock, so the control task is no longer blocked. The flash write still happens per request.
- No request rate limiting exists on any endpoint.
- `handleValues()` is polled continuously by the browser (about one request per second per client,
  `src/WebContentScript.h`); it is read-only and writes nothing.

## 3. Design (TODO)

- Dirty flag plus debounce timer: mark the settings dirty and flush once per window (for example 2 s)
  from a periodic task or from `loop()`.
- Forced flush for values that must survive immediately: WiFi credentials (P2.2), provisioning state,
  factory reset.
- Per-endpoint request limiting with a token bucket and a 429 response on excess.
- Keep the v2.7.3 mutex discipline: serialise under the lock, write outside.

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

- Stress test with a request flood, count flash writes and confirm one flush per window.
- Power-cycle inside the debounce window to confirm the forced-flush paths.
- Confirm that normal operation from the browser is not throttled.

## 6. Open Items

- Window length versus the risk of losing settings on a hard power cut.
- Whether wear levelling on the 16 MB LittleFS partition weakens the wear argument.
