# Plan: sec/-Struktur umkehren, CRA-Checkliste als Grundlage

Rolle: Strukturplan (Detaildokument, kein CRA-Bezug)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: keine (Dokumentenstruktur)
Ausfuehrungsstand: umgesetzt am 2026-09-28
Auftrag: Die CRA-Compliance-Checkliste wird zum fuehrenden Dokument (Grundlage).
Alle uebrigen Dateien in `sec/` werden Detailbeschreibungen der Grundlagen und der Umsetzung
und verweisen nach oben auf den Master. Die Richtung der Abhaengigkeit wird invertiert.
Bezug: Regulation (EU) 2024/2847 (Cyber Resilience Act, CRA)
Vorgaenger-Stand: `sec/2026-08-29_CRA_Compliance_Checklist.md` (Arbeitsliste vom 2026-08-29)

Die Migration wurde am 2026-09-28 durchgefuehrt. Dieses Dokument liegt daher bereits am Zielort
`sec/plan/` und dient als Nachweis des Umbaus (Verifikation siehe Abschnitt 4, Phase D).

---

## 1. Entscheidungen (mit dem Nutzer abgestimmt am 2026-09-28)

| Nr. | Entscheidung |
|---|---|
| E1 | Master wird umbenannt in `sec/CRA_Compliance_Checklist.md` (ohne Datum), Datum nur im Kopf |
| E2 | Unterordner: `sec/plan/`, `sec/analyse/`, `sec/grundlagen/`, `sec/sources/`; Master bleibt in `sec/` |
| E3 | Status wird NUR im Master gepflegt. Detaildokumente haben keine Statuszeile, nur `Grundlage: <Master>, Px.y`. Befund- und Nachweisstaende (Evidenz) duerfen in Detaildokumenten bleiben |
| E4 | Strukturumbau UND inhaltliche Aktualisierung auf Stand 2026-09-28 in einem Schritt |
| E5 | Sprachmix bleibt unveraendert (deutsche und englische Dokumente bleiben wie sie sind) |
| E6 | Skelett-Dokument je offener Massnahme anlegen; Implementierungsplan aus der Analyse entfernen und auf den Master verweisen |

---

## 2. Zielstruktur

```
sec/
  CRA_Compliance_Checklist.md                              <- Master (Grundlage)
  analyse/
    2026-08-27_Webinterface_Sicherheitsanalyse.md           (Risiko-/Evidenzanalyse, Befunde R1..R9)
  plan/
    2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md          (vorhanden, umbenannt)
    2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md              (vorhanden, umbenannt)
    2026-08-31_P1-3_Sensor_Calibration_Plan.md              (vorhanden, umbenannt)
    2026-08-31_P2-2_AP_Password_Plan.md                     (vorhanden, umbenannt)
    2026-09-28_P1-4_PID_Gain_Limits_Plan.md                 (neu, Skelett)
    2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md   (neu, Skelett)
    2026-09-28_P3-3_Radio_Encryption_Plan.md                (neu, Skelett)
    2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md        (neu, Skelett)
    2026-09-28_P4-2_SBOM.md                                 (neu, Skelett)
    2026-09-28_P4-3_Signed_OTA.md                           (neu, Skelett)
    2026-09-28_P4-4_Support_Period.md                       (neu, Skelett)
    2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md      (neu, Skelett)
    2026-09-28_P4-6_Technical_Documentation_Annex_VII.md    (neu, Skelett)
    2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md      (neu, Skelett)
    2026-09-28_P4-8_User_Information_Annex_II.md            (neu, Skelett)
    2026-09-28_P5-1_Reporting_Process.md                    (neu, Skelett)
    2026-09-28_P5-2_SRP_CSIRT_Access.md                     (neu, Skelett)
  grundlagen/
    2026-08-28_Mutex_How_It_Works.md                        (vorhanden, verschoben)
    2026-08-29_Web_Auth_How_It_Works.md                     (vorhanden, verschoben)
  sources/
    EU_CRA_Regulation_2024_2847_EN.pdf                      (vorhanden, umbenannt und verschoben)
```

**ID-Schema**

- Massnahmen-IDs bleiben wie etabliert und stabil: `P1.1` bis `P1.4`, `P2.1` bis `P2.3`,
  `P3.1` bis `P3.3`, `P4.1` bis `P4.8`, `P5.1`, `P5.2`. Bestehende Verweise in den Plaenen
  bleiben damit gueltig.
- Pflichten-Nummerierung in Abschnitt 4 des Masters (zum Planungszeitpunkt Abschnitt 3) wird dagegen
  durchgehend von 1 bis 26 korrigiert (heute doppelte Nummern 10 in 3.1 und 3.2).
- Dateinamen der Plaene tragen die Massnahmen-ID mit Bindestrich statt Punkt (`P1-3`),
  um doppelte Dateiendungen zu vermeiden.

---

## 3. Gepruefter Ist-Stand (Code-Stand 2026-09-28, Grundlage fuer Phase C)

Firmware-Version im Arbeitsbaum: `VERSION` = 2.7.4 (Build-Bump durch `inject_version.py`,
kein CHANGELOG-Eintrag). Letzte sicherheitsrelevante Aenderung: v2.7.3 (Mutex, Client-Limit).

| Massnahme | Befund im Code | Status |
|---|---|---|
| P1.1 Auth | Kein Login, keine Session, kein Token, kein Cookie in `src/` | offen |
| P1.2 POST und CSRF | `webServer.on("/set", ...)` und `on("/resetPID", ...)` ohne Methodenfilter, GET wirkt; kein CSRF-Token | offen |
| P1.3 Kalibrierung | `handleSet()` prueft nur `minP < maxP` und `0 <= minV < maxV <= 5.0f`; `SENSOR_MAX_BAR` wird in `WebHandlers.cpp` nicht referenziert | offen |
| P1.4 PID-Gains | Kp/Ki/Kd gegen `settings->kp_limits` geprueft, aber `POST /api/slider-limits` setzt diese Limits ohne Obergrenze (nur `max > min`, `step > 0`) | offen |
| P2.1 Pfadbereinigung | `onNotFound` gibt `webServer.uri()` unveraendert an `handleFileRead()`; `.json` wird mit `application/json` ausgeliefert | offen |
| P2.2 AP-Passwort | `NetworkConfig::AP_PASSWORD = "ventcon12!"` in `Constants.h`, im Konstruktor uebernommen | offen |
| P2.3 Flash-Wear | `settings->save()` bei jeder Aenderung, kein Debouncing, kein Rate-Limit | offen |
| P3.1 Mutex | FreeRTOS-Mutex in `SettingsHandler`, in v2.7.3 umgesetzt | erledigt |
| P3.2 Client-Limit | `MAX_CLIENTS = 2`, passive Ablehnung im Event-Handler, in v2.7.3 umgesetzt | erledigt |
| P3.3 Funkebene | Nur WPA2-PSK ohne konfigurierbares Passwort, keine Verschluesselung der Anwendungsebene | offen |

Praezisierung zu P1.3: Die Notabschaltung verschwindet nicht. `SensorManager::isPressureSafe()`
bildet `min(MAX_OPERATING_BAR * 1.1, sensor_max_pressure * 1.1)`. Der Angriff liegt in der
Messtransformation: frei waehlbare Kalibrierwerte bilden den realen Druck zu niedrig ab,
sodass die Schwelle nie erreicht wird. Diese Formulierung im Master verwenden.

---

## 4. Phasen

### Phase A: Master aufwerten (blockiert Phase B und C)

| Schritt | Inhalt |
|---|---|
| A1 | `git mv sec/2026-08-29_CRA_Compliance_Checklist.md sec/CRA_Compliance_Checklist.md` |
| A2 | Kopfzeile neu: Rolle "Grundlage (Master)", Stand 2026-09-28, Bezug Regulation (EU) 2024/2847, Verweis "Detaildokumente siehe Abschnitt 2", Revisionshistorie |
| A3 | Neuer Abschnitt 2 "Dokumentenregister": Datei, Rolle, Massnahmen-IDs, CRA-Bezug, letzte Pruefung. Rollen: Master, Risikoanalyse, Umsetzungsplan, Grundlagenerklaerung, Normquelle |
| A4 | Abschnitt 3 durchgehend nummerieren (1 bis 26), Status-Spalte als einzige Statusquelle beibehalten |
| A5 | Abschnitt 3.4 von "ab 2026-09-11" auf "in Kraft seit 2026-09-11" umstellen; Zeile fuer Art. 69(3) ergaenzen (Meldepflicht gilt auch fuer vor dem 2027-12-11 in Verkehr gebrachte Produkte) |
| A6 | Abschnitt 4 zur zentralen Verknuepfungstabelle ausbauen: Risiko-ID, Pflicht-Nr., Massnahmen-ID, Detaildokument (relativer Pfad), Nachweis |
| A7 | Abschnitt 5 "Arbeitspakete": je Massnahme Verweis auf ihr Detaildokument, Status als Checkbox |
| A8 | Abschnitte 1 und 2 inhaltlich schaerfen (RED/CRA-Abgrenzung, Modul-A-Vorbehalt), Details siehe Anhang A |
| A9 | Zeitplan ergaenzen (2026-06-11, Aufhebung RED-Cyber-VO zum 2027-12-11), Strafen praezisieren, Quellen erweitern, Normenstatus vermerken, siehe Anhang A |

### Phase B: Detaildokumente anbinden (parallel, abhaengig von A2, A3, A4, A6)

| Schritt | Inhalt |
|---|---|
| B1 | Acht Move-Operationen, siehe Abschnitt 5 |
| B2 | Einheitliches Frontmatter in allen Detaildokumenten: Rolle, `Grundlage: ../CRA_Compliance_Checklist.md`, Massnahme(n), CRA-Bezug, Risikobezug, Abhaengigkeiten. Alle Statuszeilen entfernen (z.B. "Status: Plan; implementation pending", "Umgesetzt und verifiziert am 2026-08-27") |
| B3 | Analyse umbauen: Befunde auf stabile IDs R1 bis R9 umstellen (heute Nr. 1 bis 9, wobei Nr. 8 und 9 gegenueber P3.2 und P3.1 vertauscht sind); Abschnitt 5 (Implementierungsplan) entfernen und durch einen Verweissatz auf den Master ersetzen; Abschnitt 6 als "Geklaerte Entscheidungen (Eingang in den Master)" behalten; defekte Tabellenzeile in Abschnitt 2 (fuehrendes Anfuehrungszeichen) reparieren |
| B4 | Plaene: Querverweise auf neue Pfade umstellen (Companion-Links, Voraussetzungen). Im Web-Auth-Plan den Schritt "Update Sicherheitsanalyse: mark P1.1 and P1.2" umformulieren zu "Status im Master aktualisieren" |
| B5 | Grundlagen-Dokumente: Rolle "Grundlagenerklaerung" plus Massnahmenzuordnung (Mutex zu P3.1, Web zu P1.1, P1.2, P2.1). Inhalt unveraendert |
| B6 | Normquelle im Register fuehren |

### Phase C: Luecken schliessen (parallel, abhaengig von A3 und A4)

| Schritt | Inhalt |
|---|---|
| C1 | 13 Skelett-Dokumente anlegen (P1.4, P2.3, P3.3, P4.1 bis P4.8, P5.1, P5.2), Vorlage siehe Anhang B |
| C2 | Register gegen die Massnahmenliste P1.1 bis P5.2 abgleichen: jede Massnahme hat genau einen Eintrag; P3.1 und P3.2 verweisen auf ihr vorhandenes Detaildokument |

### Phase D: Verifikation

1. `grep -rn "^Status:" sec/` liefert Treffer nur im Master.
2. `grep -rn "^Grundlage:" sec/` liefert in jedem Detaildokument `../CRA_Compliance_Checklist.md`.
3. `grep -rn "2026-08-27_Webinterface_Sicherheitsanalyse" sec/` liefert keinen Verweis mehr,
   der sie als Grundlage fuehrt, nur noch Nachweis- und Pfadverweise.
4. ID-Abgleich in beide Richtungen: jede genannte P-ID existiert im Master, jede P-ID im Register
   hat genau ein Detaildokument.
5. Alle relativen Pfade aufloesbar (Master in `sec/`, Details eine Ebene tiefer, Plan-zu-Plan
   im selben Ordner).
6. VS-Code-Vorschau: Tabellen in Master Abschnitt 3 und 4 sowie Analyse Abschnitt 2 rendern korrekt.
7. `git status` und `git diff --stat`: alle Verschiebungen erscheinen als Rename, keine Loeschungen.
8. Sichtpruefung durch den Nutzer: der Master funktioniert als alleiniger Einstiegspunkt.

---

## 5. Dateioperationen

Ausschliesslich Umbenennen und Verschieben innerhalb von `sec/`. Keine Loeschungen.

| Von | Nach |
|---|---|
| `sec/2026-08-29_CRA_Compliance_Checklist.md` | `sec/CRA_Compliance_Checklist.md` |
| `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md` | `sec/analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` |
| `sec/2026-08-27_Mutex_Settings_Zugriff_Plan.md` | `sec/plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` |
| `sec/2026-08-29_Web_Auth_Plan.md` | `sec/plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` |
| `sec/2026-08-31_Sensor_Calibration_Plan.md` | `sec/plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` |
| `sec/2026-08-31_AP_Password_Plan.md` | `sec/plan/2026-08-31_P2-2_AP_Password_Plan.md` |
| `sec/2026-08-28_Mutex_How_It_Works.md` | `sec/grundlagen/2026-08-28_Mutex_How_It_Works.md` |
| `sec/2026-08-29_Web_Auth_How_It_Works.md` | `sec/grundlagen/2026-08-29_Web_Auth_How_It_Works.md` |
| `sec/EU CRA___OJ_L_202402847_EN_TXT.pdf` | `sec/sources/EU_CRA_Regulation_2024_2847_EN.pdf` |

**Protokoll fuer die Ausfuehrung (Projektregel Bulk-Operationen)**

1. Dry-Run: Liste der Moves zeigen, Anzahl nennen (9 Pfade, davon 8 git-getrackt, 1 untracked PDF).
2. Freigabe des Nutzers abwarten, vorher nichts ausfuehren.
3. `git mv` verwenden, damit die Historie erhalten bleibt und Renames als `R` erscheinen.
4. Ein Pfad pro Kommando, Pfade in Anfuehrungszeichen, keine dynamisch gebauten Pfade.
5. Dropbox-Hinweis: Renames propagieren auf alle synchronisierten Maschinen, Sync vor der
   Massenoperation pausieren empfohlen. Vorher `git status` dokumentieren.
6. Keine `Remove-Item`-Aufrufe, keine Rekursion.

---

## 6. Scope-Grenzen

- Keine Aenderung an `src/` (Firmware) oder `LaTeX/`.
- Keine Umsetzung der Sicherheitsmassnahmen selbst, nur Struktur und Dokumentenkopf.
- Keine inhaltliche Kuerzung der bestehenden Plaene, die Entwuerfe bleiben vollstaendig.
- Keine Sprachumstellung (Entscheidung E5).
- Keine neuen Normquellen, nur Registrierung der vorhandenen.

---

## 7. Offene Punkte

1. Anwendbarkeit: "gewerbliches Inverkehrbringen in der EU" bleibt im Master eine offene Zeile
   (Geschaeftsentscheidung; reine Prototyp-Nutzung waere nicht betroffen).
2. RED gegen CRA bleibt eine zu treffende Entscheidung (Abschnitt 1 des Masters), siehe Anhang A.
3. Massnahmen ohne Firmwarebezug (P4.x, P5.x) bleiben Skelette, bis Inhalte entschieden sind.

---

## Anhang A: Inhaltliche Korrekturen fuer den Master (Stand 2026-09-28)

Rechercheergebnis: Die CRA-Fristen sind unveraendert. Es gibt kein Corrigendum und keine
angenommene Aenderung, die Art. 14 oder Art. 71 verschiebt. Der Digital Omnibus der Kommission
(COM(2025) 837, 19.11.2025) ist Vorschlag, nicht angenommen; der angenommene Teil
(Verordnung (EU) 2026/1744 vom 24.07.2026) betrifft nur den AI Act, nicht die CRA.

| Stelle im Master | Aenderung |
|---|---|
| Kopfzeile | Stand 2026-09-28, Revisionshistorie, Rolle "Grundlage (Master)" |
| Abschnitt 1 | RED/CRA-Abgrenzung praezisieren: Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444 gilt ab 2025-08-01. Neu: Delegierte VO (EU) 2026/339 vom 16.02.2026 (ABl. 29.04.2026) hebt 2022/30 zum 2027-12-11 auf. Die Aussage "greift RED statt CRA fuer die Cyber-Anforderungen" gilt nur bis 2027-12-11; die Meldepflichten gelten bereits jetzt parallel |
| Abschnitt 2 | Modul-A-Aussage (Selbstbewertung) mit Vorbehalt versehen: EN 18031-1/-2/-3 sind durch Durchfuehrungsbeschluss (EU) 2025/138 mit Restriktionen zitiert, unter anderem die Passwortklauseln 6.2.5.1 und 6.2.5.2. Werden ausgenommene Klauseln beruehrt, wechselt die RED-Route auf Modul B+C oder H (notifizierte Stelle) |
| Abschnitt 3 | Durchgehende Nummerierung 1 bis 26; Status-Spalte beibehalten; Spalte "Befund" durch Risiko-ID (R1 bis R9) plus Nachweis/Detaildokument ersetzen |
| Abschnitt 4.1 Nr. 1 und Nr. 2 | Bezug auf EN 18031 Passwortklauseln und auf das Erstinbetriebnahme-Provisioning (kein universelles Standardpasswort) |
| Abschnitt 3.2 | Nummerierung beginnt heute faelschlich wieder bei 10, korrigieren auf 11 bis 15 |
| Abschnitt 3.3 und 3.4 | Nummern 16 bis 22 und 23 bis 26 |
| Abschnitt 3.4 | Ueberschrift "Meldepflichten (Art. 14, in Kraft seit 2026-09-11)"; Zeile ergaenzen: Art. 69(3), Meldepflicht gilt auch fuer vor dem 2027-12-11 in Verkehr gebrachte Produkte |
| Abschnitt 4 | Verknuepfungstabelle: Risiko-ID, Pflicht-Nr., Massnahmen-ID, Detaildokument, Nachweis |
| Abschnitt 5 | Status als Checkbox, Verweis je Massnahme auf das Detaildokument; Phase 5 als "akut" kennzeichnen, da Art. 14 in Kraft ist |
| Abschnitt 6 Zeitplan | Zeile ergaenzen: 2026-06-11 Kapitel IV (Konformitaetsbewertungsstellen) anwendbar; Zeile 2027-12-11 ergaenzen um "inkl. Aufhebung der RED-Cyber-Delegierten-VO" |
| Abschnitt 7 Strafen | Drei Stufen: 15 Mio. EUR oder 2,5 Prozent (Annex I, Art. 13, Art. 14), 10 Mio. EUR oder 2 Prozent, 5 Mio. EUR oder 1 Prozent (falsche Angaben). Art. 64(10): Kleinst- und Kleinunternehmen sind von Geldstrafen nur bei der 24-h-Frist (Art. 14(2)(a) und Art. 14(4)(a)) ausgenommen, nicht generell |
| Abschnitt 8 Quellen | Ergaenzen: Leitlinien C(2026) 5252 vom 27.07.2026 (Abschnitt 9.1 zu Art. 14), Delegierte VO (EU) 2026/339, Durchfuehrungsbeschluss (EU) 2025/138, Durchfuehrungsverordnung (EU) 2025/2392, Delegierte VO (EU) 2026/881, ENISA SRP, CSIRT-Liste, lokale Normquelle `sources/EU_CRA_Regulation_2024_2847_EN.pdf` |
| Neuer Hinweis | Keine CRA-harmonisierte Norm ist im Amtsblatt zitiert (Stand 2026-09-28), die Konformitaetsvermutung nach Art. 27 besteht derzeit nicht. Nachweis ueber Risikobewertung und technische Dokumentation |
| Abschnitt 5 Phase 5 | Operativer Stand ergaenzen: SRP live seit 2026-09-11 unter `portal.cra-srp.enisa.europa.eu`, Zugang ueber EU Login mit MFA und der Rolle "Assigned Representative", kein API im ersten Release, keine freiwilligen Meldungen nach Art. 15 zum Start, kein Durchfuehrungsrechtsakt zu Format und Verfahren (Art. 14(10)) erlassen, CSIRT-Koordinatorenliste seit 2026-09-04 verfuegbar, Zuordnung nach Main-Establishment-Test (Art. 14(7)) |

---

## Anhang B: Vorlage fuer Skelett-Dokumente (Phase C)

```
# Plan: <Titel> (<Massnahmen-ID>)

Datum: 2026-09-28
Grundlage: ../CRA_Compliance_Checklist.md
Massnahme: <Px.y>
CRA-Bezug: <Annex I 1.x beziehungsweise Art. 13(x)>
Risikobezug: <R1 bis R9 aus der Risikoanalyse, oder "keiner">
Abhaengigkeiten: <andere Massnahmen>

## 1. Ziel

## 2. Ist-Zustand
(Code-Referenzen aus Abschnitt 3 dieses Plans uebernehmen)

## 3. Design
(Entwurf; offene Entscheidungen als Liste)

## 4. Umsetzungsschritte

## 5. Verifikation

## 6. Offene Punkte
```

Regeln fuer Skelette: keine Statuszeile (Entscheidung E3), Status wird nur im Master gefuehrt.
Sprache: Englisch fuer die Normpflichten-Dokumente P4.x und P5.x, entsprechend dem Sprachmix
der bereits vorhandenen Plaene (Entscheidung E5); Ausnahmen sind zulaessig, wenn ein Dokument
inhaltlich an einen deutschen Plan anknuepft.

---

## Anhang C: Werkzeug fuer die Verifikation

Die Pruefungen aus Phase D lassen sich mit Textsuche im Ordner `sec/` ausfuehren
(VS-Code-Suche mit den Ausdruecken `^Status:`, `^Grundlage:`, `^Massnahme:`,
`2026-08-27_Webinterface_Sicherheitsanalyse`). Keine Skripte noetig.
