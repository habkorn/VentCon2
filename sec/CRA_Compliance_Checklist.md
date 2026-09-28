# VentCon2 - EU CRA Compliance Checklist

Datum: 2026-08-29
Status: Arbeitsliste, offen
Bezug: Regulation (EU) 2024/2847 (Cyber Resilience Act, CRA)
Grundlage: `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md`

## 1. Anwendbarkeit

| Frage | Antwort | Status |
|---|---|---|
| Ist VentCon2 ein "Produkt mit digitalen Elementen"? | Ja, ESP32-S3 Ventilsteuerung mit Webinterface und Netzwerkverbindung | Geklaert |
| Wird es in der EU gewerblich in Verkehr gebracht? | Zu klaeren; reine Prototyp-Nutzung waere nicht betroffen | Offen |
| Enthaelt es Funk (Wi-Fi/BT) unter RED 2014/53/EU? | Ja, ESP32 mit Wi-Fi. Falls "internetfaehiges Funkanlagen"-Geraet, greift Delegierte VO (EU) 2022/30 statt CRA fuer die Cyber-Anforderungen | Zu klaeren |

**Erste Entscheidung (vor allem anderen):** RED vs. CRA. Feststellen, ob VentCon2 als internetfaehiges Funkanlagengeraet unter Delegierte VO (EU) 2022/30 faellt (anwendbar seit 2025-08-01). Das bestimmt, welche Anforderungen und welcher CE-Weg gelten.

## 2. Produktklassifizierung

| Kategorie | Einordnung |
|---|---|
| Standardprodukt (Annex III/IV nicht betroffen) | Ja |
| Wichtige Produkte Class I/II (Annex III) | Nein |
| Kritische Produkte (Annex IV) | Nein |
| Konformitaetsbewertung | **Modul A (Selbstbewertung)**, kein notifizierte Stelle noetig |

## 3. Pflichten-Checkliste

Legende Status: `[ ]` offen, `[x]` erledigt, `[?]` zu klaeren, `n/a` nicht anwendbar

### 3.1 Sicherheitsanforderungen (Annex I, Teil I)

| Nr. | Pflicht | CRA-Referenz | Befund | Status |
|---|---|---|---|---|
| 1 | Keine universellen Standardpasswoerter | Annex I, Part I, 2(b)/(d) | Befund 2: AP-Passwort "ventcon12!" fest verdrahtet; gilt auch fuer das Web-Login (P1.1) | [ ] |
| 2 | Authentifizierung und Zugriffskontrolle | Annex I 1.2 | Befund 1: keine Auth auf /set, /resetPID, /api/* | [ ] |
| 3 | Minimale Angriffsflaeche | Annex I 1.4 | Befund 5: beliebiger Dateizugriff via unnormalisiertem Pfad | [ ] |
| 4 | Schutz vor unbefugtem Zugriff, Eingabevalidierung | Annex I 1.7 | Befund 6: unbegrenzte PID-Gains | [ ] |
| 5 | Sichere Kommunikation | Annex I 1.8 | P3.3: TLS/WPA2/WPA3 oder verschluesseltes Protokoll | [ ] |
| 6 | Sichere Standardkonfiguration | Annex I 1.10 | Befund 4: Zustandsaenderung per GET, CSRF | [ ] |
| 7 | Sichere Update-Mechanismen (signiert) | Annex I 1.9 | OTA mit Signaturpruefung fehlt | [ ] |
| 8 | Verfuegbarkeit / DoS-Resistenz | Annex I 1.3 | Befund 7: Flash-Verschleiss via settings.save() | [ ] |
| 9 | Produktsicherheit (Notabschaltung) | Annex I 1.7 | Befund 3: Kalibrierung hebelt Ueberdruckabschaltung aus | [ ] |
| 10 | Sichere Speicherung von Zugangsdaten | Annex I, Part I, 2(e); EN 303 645 5.4-1 | AP-Passwort (rekonstruierbar) und Passwort-Hash im Flash; verschluesseltes NVS noetig | [ ] |

### 3.2 Schwachstellenbehandlung (Annex I, Teil II)

| Nr. | Pflicht | Status |
|---|---|---|
| 10 | SBOM (Software Bill of Materials) pflegen | [ ] |
| 11 | Schwachstellen identifizieren und dokumentieren | [ ] |
| 12 | Schwachstellen ohne Verzoegerung beheben (Security-Updates) | [ ] |
| 13 | Vulnerability-Disclosure-Richtlinie und Sicherheitskontakt veroeffentlichen | [ ] |
| 14 | Sicherheitsupdates testen vor Auslieferung | [ ] |

### 3.3 Konformitaet und Markt

| Nr. | Pflicht | Referenz | Status |
|---|---|---|---|
| 15 | Cybersicherheits-Risikobewertung erstellen und dokumentieren | Art. 13(2) | [ ] |
| 16 | Technische Dokumentation erstellen | Annex VII | [ ] |
| 17 | EU-Konformitaetserklaerung ausstellen | Annex V | [ ] |
| 18 | CE-Kennzeichnung anbringen | Art. 28 | [ ] |
| 19 | Supportzeitraum festlegen (min. 5 Jahre) und Enddatum angeben | Art. 13(8) | [ ] |
| 20 | Informationen und Anweisungen fuer den Nutzer bereitstellen | Annex II | [ ] |
| 21 | Sorgfaltspflicht bei Drittkomponenten | Art. 13(5) | [ ] |

### 3.4 Meldepflichten (Art. 14, ab 2026-09-11)

| Nr. | Pflicht | Frist | Status |
|---|---|---|---|
| 22 | Fruehwarnung aktiv ausgenutzter Schwachstellen / schwerer Sicherheitsvorfaelle | 24 h | [ ] |
| 23 | Vollstaendige Meldung an CSIRT und ENISA (Single Reporting Platform) | 72 h | [ ] |
| 24 | Abschlussbericht aktiv ausgenutzter Schwachstellen | 14 Tage | [ ] |
| 25 | Abschlussbericht schwerer Sicherheitsvorfaelle | 1 Monat | [ ] |

## 4. Befund-zu-Pflicht-Mapping

| Befund (Sicherheitsanalyse) | CRA-Pflicht | Massnahme | Prioritaet |
|---|---|---|---|
| 1 Keine Auth auf Steueroberflaeche | 3.1 Nr. 2 | P1.1 Login-Seite mit Passwort, Erstinbetriebnahme-Provisioning | P1 |
| 2 Festes AP-Passwort | 3.1 Nr. 1 | P2.2 Passwort zur Laufzeit konfigurierbar | P2 |
| 3 Kalibrierung hebelt Notabschaltung aus | 3.1 Nr. 9 | P1.3 Kalibrierung auf Sensorbereich begrenzen | P1 |
| 4 Zustandsaenderung per GET, CSRF | 3.1 Nr. 6 | P1.2 POST + CSRF-Schutz | P1 |
| 5 Beliebiger Dateizugriff | 3.1 Nr. 3 | P2.1 Pfadbereinigung, JSON-Auslieferung sperren | P2 |
| 6 Unbegrenzte PID-Gains | 3.1 Nr. 4 | P1.4 harte Obergrenzen | P1 |
| 7 Flash-Verschleiss / DoS | 3.1 Nr. 8 | P2.3 Ratenbegrenzung + Save-Debouncing | P2 |
| P3.1 Mutex settings-Zugriff | (Robustheit) | Erledigt in v2.7.3 | Erledigt |
| P3.2 Client-Connect-DoS | 3.1 Nr. 8 | Erledigt in v2.7.3 | Erledigt |
| P3.3 Verschluesselung Funkebene | 3.1 Nr. 5 | WPA2 Enterprise oder verschluesseltes Protokoll | P3 |

## 5. Arbeitspakete nach Prioritaet

### Phase 1: Sicherheitskritisch (CRA Annex I Kernpflichten)

- [ ] P1.1 Authentifizierung fuer alle mutierenden Endpunkte (Login-Seite + Erstinbetriebnahme-Provisioning)
- [ ] P1.2 POST-Umstellung + CSRF-Schutz fuer /set und /resetPID
- [ ] P1.3 Sensorkalibrierung auf physischen Sensorbereich begrenzen (sensor_maxP <= SENSOR_MAX_BAR)
- [ ] P1.4 Harte Obergrenzen fuer PID-Gains

### Phase 2: Haertung

- [ ] P2.1 Pfadbereinigung in handleFileRead(), JSON-Konfiguration nicht ueber HTTP ausliefern
- [ ] P2.2 AP-Passwort zur Laufzeit konfigurierbar
- [ ] P2.3 Ratenbegrenzung + Save-Debouncing

### Phase 3: Robustheit

- [x] P3.1 Mutex fuer settings-Zugriff (v2.7.3)
- [x] P3.2 Client-Connect-DoS (v2.7.3)
- [ ] P3.3 Verschluesselung oder Authentifizierung auf Funkebene

### Phase 4: Compliance-Paket (Dokumentation)

- [ ] 4.1 Cybersicherheits-Risikobewertung dokumentieren
- [ ] 4.2 SBOM erstellen (Arduino core, Arduino-ESP32, LittleFS, chart.js, Drittbibliotheken)
- [ ] 4.3 Signierte OTA-Update-Mechanik implementieren
- [ ] 4.4 Supportzeitraum (min. 5 Jahre) festlegen und Enddatum dokumentieren
- [ ] 4.5 Vulnerability-Disclosure-Richtlinie + Sicherheitskontakt
- [ ] 4.6 Technische Dokumentation (Annex VII)
- [ ] 4.7 EU-Konformitaetserklaerung (Annex V) + CE-Kennzeichnung
- [ ] 4.8 Nutzerinformationen (Annex II) aktualisieren

### Phase 5: Meldeprozess (ab 2026-09-11)

- [ ] 5.1 Meldeprozess fuer aktiv ausgenutzte Schwachstellen und schwere Sicherheitsvorfaelle einrichten
- [ ] 5.2 Zugang zur ENISA Single Reporting Platform (SRP) und zustaendiges CSIRT klaeren

## 6. Zeitplan (Fristen)

| Datum | Ereignis / Pflicht |
|---|---|
| 2024-12-10 | CRA in Kraft getreten |
| 2025-08-01 | Delegierte VO (EU) 2022/30 (RED Cyber) anwendbar, falls zutreffend |
| **2026-09-11** | Meldepflichten (Art. 14) anwendbar |
| **2027-12-11** | Vollstaendige Anwendung aller uebrigen Pflichten |

## 7. Strafen (Kontext)

- Bis zu EUR 15.000.000 oder 2,5 % des weltweiten Jahresumsatzes (Verstoesse gegen Annex I, Art. 13, 14)
- Kleinst- und Kleinunternehmen: keine Geldstrafe fuer verpasste 24-h-Meldefrist

## 8. Quellen

1. Regulation (EU) 2024/2847, EUR-Lex: https://eur-lex.europa.eu/eli/reg/2024/2847/oj
2. EU-Kommission, Cyber Resilience Act: https://digital-strategy.ec.europa.eu/en/policies/cyber-resilience-act
3. ENISA, Single Reporting Platform (SRP): https://www.enisa.europa.eu/topics/product-security-and-certification/single-reporting-platform-srp
4. Delegierte VO (EU) 2022/30 (RED Cyber): https://eur-lex.europa.eu/eli/reg_del/2022/30/oj
