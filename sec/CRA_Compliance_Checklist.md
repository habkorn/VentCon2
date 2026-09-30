# VentCon2 - EU CRA Compliance Checklist

Rolle: Grundlage (Master) fuer alle Dokumente in `sec/`. Die Detaildokumente sind in Abschnitt 4
registriert und verweisen auf dieses Dokument.
Stand: 2026-09-30
Bezug: Regulation (EU) 2024/2847 (CRA); Radio Equipment Directive 2014/53/EU
Status: Arbeitsliste, offen. Dies ist die einzige Statusquelle; Detaildokumente fuehren keinen Status.
Vorgaenger: Arbeitsliste vom 2026-08-29 (vormals `sec/2026-08-29_CRA_Compliance_Checklist.md`, seit 2026-09-28 dieses Dokument)
Normquelle: `sources/EU_CRA_Regulation_2024_2847_EN.pdf` (CRA-Volltext)
Umbauplan: `plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md`
Rahmenplan: `plan/2026-09-30_RED_CRA_Abgrenzung_Plan.md`

## 1. Regulatorischer Rahmen: RED und CRA

Dieses Produkt trifft zwei Regelwerke, dazu kommen technische Normen ohne eigene Rechtswirkung.
Die drei Ebenen werden getrennt gefuehrt; alle weiteren Abschnitte bauen darauf auf.

| Merkmal | RED-Cyber (gilt bis 2027-12-11) | CRA |
|---|---|---|
| Rechtsgrundlage | Art. 3(3)(d)(e)(f) der Richtlinie 2014/53/EU, ausgestaltet durch Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444 | Verordnung (EU) 2024/2847 |
| Anwendungsbereich | internetfaehige Funkgeraete, hier der ESP32 mit Wi-Fi | Produkte mit digitalen Elementen, hier die Ventilsteuerung |
| Anwendbar seit | 2025-08-01 | Art. 14 seit 2026-09-11, vollstaendig ab 2027-12-11 |
| Ende | Aufhebung der Delegierten VO (EU) 2022/30 durch Delegierte VO (EU) 2026/339, wirksam ab 2027-12-11 | unbefristet, Supportzeitraum von mindestens fuenf Jahren |
| Inhalt der Anforderungen | Netzwerkschutz, Schutz personenbezogener Daten, Betrugsschutz | Annex I Teil I und II, Art. 13 Herstellerpflichten, Art. 14 Meldepflichten, Art. 28 CE-Kennzeichnung |
| Nachweisweg | Selbstbewertung, bei ausgenommenen Klauseln Modul B+C oder Modul H | Modul A, Selbstbewertung, siehe Abschnitt 5 |
| Konformitaetsvermutung | EN 18031-1/-2/-3, im Amtsblatt zitiert mit Restriktionen | keine Norm zitiert, Art. 27 derzeit nicht nutzbar |
| Meldepflicht | keine | Art. 14: 24 h, 72 h, 14 Tage, 1 Monat, siehe Abschnitt 6.4 |
| Sanktionsrahmen | nationales Recht der Mitgliedstaaten zur Richtlinie 2014/53/EU | Art. 64, siehe Abschnitt 10 |

### 1.1 Fuenf Kernaussagen

1. RED-Cyber ist derzeit der verbindliche Nachweisweg und nicht redundant, er gilt bis
   2027-12-11. Wer heute in Verkehr bringt, braucht diesen Nachweis fuer die CE-Kennzeichnung.
2. Am 2027-12-11 wird die Delegierte VO (EU) 2022/30 aufgehoben. Die Cyber-Anforderungen gehen in die
   CRA ueber, danach gibt es weder Luecke noch Dopplung.
3. Die CRA-Meldepflichten gelten schon jetzt, unabhaengig von der RED-Route.
4. EN 303 645 ist technischer Massstab ohne Rechtswirkung. Die Norm ist kein harmonisierter Rechtsakt
   und begruendet keine Konformitaetsvermutung, weder unter dem RED noch unter der CRA.
5. Das CE-Zeichen stammt heute aus dem RED und ab 2027 aus der CRA. Beide Nachweise bleiben im Dossier
   getrennt, siehe `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md`.

### 1.2 Gueltigkeit

Jede Aussage mit RED-Bezug in diesem Dossier gilt bis zum 2027-12-11, sofern sie nicht ausdruecklich
anders datiert ist. Aussagen mit CRA-Bezug gelten unbefristet und greifen gestaffelt.

## 2. Anwendbarkeit

| Frage | Antwort | Status |
|---|---|---|
| Ist VentCon2 ein "Produkt mit digitalen Elementen"? | Ja, ESP32-S3 Ventilsteuerung mit Webinterface und Netzwerkverbindung | Geklaert |
| Wird es in der EU gewerblich in Verkehr gebracht? | Zu klaeren; reine Prototyp-Nutzung waere nicht betroffen | Offen |
| Ist VentCon2 ein internetfaehiges Funkgeraet? | Ja, ESP32 mit Wi-Fi. Damit greifen die Cyber-Anforderungen des RED bis 2027-12-11, siehe Abschnitt 1 und 3 | Geklaert |

**Erste Entscheidung (vor allem anderen):** Feststellen, ob VentCon2 gewerblich in der EU in Verkehr
gebracht wird. Davon haengt ab, ob CE-Weg, technische Dokumentation, Supportzeitraum und Meldepflichten
greifen. Die Abgrenzung RED gegen CRA steht in Abschnitt 1.

## 3. RED-Pflichten (gilt bis 2027-12-11)

Diese Anforderungen gelten zusaetzlich zur CRA und enden mit der Aufhebung der Delegierten
VO (EU) 2022/30. Die Nummerierung ist bewusst von der CRA-Nummerierung 1 bis 26 getrennt.

| ID | Anforderung | Rechtsgrundlage | Harmonisierte Norm | Massnahme | Status |
|---|---|---|---|---|---|
| RED-1 | Netzwerkschutz, Schutz des Netzes vor Angriffen | RED Art. 3(3)(d) | EN 18031-1 | P1.5 Nachweis, P3.3 Funkebene | [ ] |
| RED-2 | Schutz personenbezogener Daten und der Privatsphaere | RED Art. 3(3)(e) | EN 18031-2 | P1.5 Nachweis, P2.2 Passwort | [ ] |
| RED-3 | Betrugsschutz bei Uebertragung von Geld oder Werten | RED Art. 3(3)(f) | EN 18031-3 | voraussichtlich nicht einschlaegig, Begruendung in P1.5 | n/a |

**Folge der EN-18031-Restriktionen:** Die Normen sind durch Durchfuehrungsbeschluss (EU) 2025/138 mit
Restriktionen im Amtsblatt zitiert. Die Passwortklauseln 6.2.5.1 und 6.2.5.2 sind von der
Konformitaetsvermutung ausgenommen. Werden sie beruehrt, gilt fuer die betroffene Anforderung keine
Selbstbewertung, sondern Modul B+C oder Modul H mit notifizierter Stelle. Fuer ein Geraet mit festem
Standardpasswort ist das der Regelfall, siehe P2.2. Der Konformitaetsweg wird in P1.5 geklaert.

## 4. Dokumentenregister

| Datei | Rolle | Massnahmen | Letzte Pruefung |
|---|---|---|---|
| `CRA_Compliance_Checklist.md` | Master (Grundlage), einzige Statusquelle | alle, P1.1 bis P5.2 | 2026-09-30 |
| `analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` | Risiko- und Evidenzanalyse | R1 bis R9 | 2026-09-30 |
| `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | Umsetzungsplan | P1.1, P1.2, P2.1 | 2026-09-30 |
| `plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` | Umsetzungsplan | P1.3 | 2026-08-31 |
| `plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md` | Umsetzungsplan (Skelett) | P1.4 | 2026-09-28 |
| `plan/2026-09-28_P1-5_RED_Conformity_Route_Plan.md` | Umsetzungsplan (Skelett) | P1.5 | 2026-09-30 |
| `plan/2026-08-31_P2-2_AP_Password_Plan.md` | Umsetzungsplan | P2.2 | 2026-09-30 |
| `plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md` | Umsetzungsplan (Skelett) | P2.3 | 2026-09-28 |
| `plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` | Umsetzungsplan (erledigt in v2.7.3) | P3.1 | 2026-08-27 |
| `plan/2026-09-28_P3-3_Radio_Encryption_Plan.md` | Umsetzungsplan (Skelett) | P3.3 | 2026-09-28 |
| `plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md` | Umsetzungsplan (Skelett) | P4.1 | 2026-09-30 |
| `plan/2026-09-28_P4-2_SBOM.md` | Umsetzungsplan (Skelett) | P4.2 | 2026-09-28 |
| `plan/2026-09-28_P4-3_Signed_OTA.md` | Umsetzungsplan (Skelett) | P4.3 | 2026-09-28 |
| `plan/2026-09-28_P4-4_Support_Period.md` | Umsetzungsplan (Skelett) | P4.4 | 2026-09-30 |
| `plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md` | Umsetzungsplan (Skelett) | P4.5 | 2026-09-28 |
| `plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md` | Umsetzungsplan (Skelett) | P4.6 | 2026-09-30 |
| `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md` | Umsetzungsplan (Skelett) | P4.7 | 2026-09-30 |
| `plan/2026-09-28_P4-8_User_Information_Annex_II.md` | Umsetzungsplan (Skelett) | P4.8 | 2026-09-30 |
| `plan/2026-09-28_P5-1_Reporting_Process.md` | Umsetzungsplan (Skelett) | P5.1 | 2026-09-28 |
| `plan/2026-09-28_P5-2_SRP_CSIRT_Access.md` | Umsetzungsplan (Skelett) | P5.2 | 2026-09-28 |
| `grundlagen/2026-08-28_Mutex_How_It_Works.md` | Grundlagenerklaerung | P3.1 | 2026-08-28 |
| `grundlagen/2026-08-29_Web_Auth_How_It_Works.md` | Grundlagenerklaerung | P1.1, P1.2, P2.1 | 2026-09-30 |
| `plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md` | Strukturplan (Dokumentenstruktur, kein CRA-Bezug) | keine | 2026-09-28 |
| `plan/2026-09-30_RED_CRA_Abgrenzung_Plan.md` | Rahmenplan (Abgrenzung RED gegen CRA) | RED-1 bis RED-3, P1.5 | 2026-09-30 |
| `plan/2026-09-30_VENTREX_Praesentation_Plan.md` | Praesentationsplan (Folienaufbau) | keine | 2026-09-30 |
| `sources/EU_CRA_Regulation_2024_2847_EN.pdf` | Normquelle (CRA-Volltext) | keine | 2026-09-30 |
Massnahmen ohne eigenes Detaildokument (P3.2) sind in Abschnitt 7 mit ihrem Nachweis ausgewiesen.
Alle Detaildokumente tragen im Kopf `Grundlage: ../CRA_Compliance_Checklist.md`. Die Analyse traegt kein
eigenes Statusfeld; Befund- und Nachweisstaende sind Evidenz, der Gesamtstatus steht nur hier.

## 5. Produktklassifizierung

| Kategorie | Einordnung |
|---|---|
| Standardprodukt (Annex III/IV nicht betroffen) | Ja |
| Wichtige Produkte Class I/II (Annex III) | Nein |
| Kritische Produkte (Annex IV) | Nein |
| Konformitaetsbewertung CRA | **Modul A (Selbstbewertung)**, keine notifizierte Stelle noetig |

**Vorbehalt aus dem RED:** Die Bewertung nach Modul A gilt fuer die CRA. Fuer die RED-Route kann sie
nicht gelten, weil die Passwortklauseln der EN 18031 von der Konformitaetsvermutung ausgenommen sind.
Rechtsgrundlagen und die Folge fuer den CE-Weg stehen in Abschnitt 1 und 3.

**Normenstand CRA:** Keine CRA-harmonisierte Norm ist im Amtsblatt zitiert (Stand 2026-09-28). Eine Konformitaetsvermutung nach Art. 27 besteht derzeit nicht; der Nachweis erfolgt ueber die Risikobewertung (P4.1) und die technische Dokumentation (P4.6). Der Normungsauftrag M/606 laeuft.

## 6. Pflichten-Checkliste (CRA)

Legende Status: `[ ]` offen, `[x]` erledigt, `[?]` zu klaeren, `n/a` nicht anwendbar

Die Pflichten sind durchgehend von 1 bis 26 nummeriert. Die Spalte "Risiko / Massnahme" nennt die
Risiko-ID aus der Risiko- und Evidenzanalyse und die Massnahmen-ID; beide sind in Abschnitt 7 aufgeloest.
Die RED-Pflichten RED-1 bis RED-3 stehen getrennt in Abschnitt 3 und gelten bis 2027-12-11.

### 6.1 Sicherheitsanforderungen (Annex I, Teil I)

| Nr. | Pflicht | CRA-Referenz | Risiko / Massnahme | Status |
|---|---|---|---|---|
| 1 | Keine universellen Standardpasswoerter | Annex I, Teil I, 2(b)/(d); technischer Massstab EN 303 645 5.1-1 | R2 / P2.2, Erstinbetriebnahme-Provisioning in P1.1 | [ ] |
| 2 | Authentifizierung und Zugriffskontrolle | Annex I, Teil I, 1.2 | R1 / P1.1 | [ ] |
| 3 | Minimale Angriffsflaeche | Annex I, Teil I, 1.4 | R5 / P2.1 | [ ] |
| 4 | Schutz vor unbefugtem Zugriff, Eingabevalidierung | Annex I, Teil I, 1.7 | R6 / P1.4 | [ ] |
| 5 | Sichere Kommunikation | Annex I, Teil I, 1.8 | Funkebene / P3.3 | [ ] |
| 6 | Sichere Standardkonfiguration | Annex I, Teil I, 1.10 | R4 / P1.2 | [ ] |
| 7 | Sichere Update-Mechanismen (signiert) | Annex I, Teil I, 1.9 | P4.3 | [ ] |
| 8 | Verfuegbarkeit / DoS-Resistenz | Annex I, Teil I, 1.3 | R7, R8 / P2.3, P3.2 | [ ] |
| 9 | Produktsicherheit (Notabschaltung) | Annex I, Teil I, 1.7 | R3 / P1.3 | [ ] |
| 10 | Sichere Speicherung von Zugangsdaten | Annex I, Teil I, 2(e); technischer Massstab EN 303 645 5.4-1 | R2 / P2.2 (verschluesseltes NVS) | [ ] |

### 6.2 Schwachstellenbehandlung (Annex I, Teil II)

| Nr. | Pflicht | CRA-Referenz | Massnahme | Status |
|---|---|---|---|---|
| 11 | SBOM (Software Bill of Materials) pflegen | Annex I, Teil II, Nr. 1 | P4.2 | [ ] |
| 12 | Schwachstellen identifizieren und dokumentieren | Annex I, Teil II, Nr. 1 | P4.5 | [ ] |
| 13 | Schwachstellen ohne Verzoegerung beheben (Security-Updates) | Annex I, Teil II, Nr. 2 | P4.3 | [ ] |
| 14 | Vulnerability-Disclosure-Richtlinie und Sicherheitskontakt veroeffentlichen | Annex I, Teil II, Nr. 4 | P4.5 | [ ] |
| 15 | Sicherheitsupdates testen vor Auslieferung | Annex I, Teil II, Nr. 3 | P4.3 | [ ] |

### 6.3 Konformitaet und Markt

| Nr. | Pflicht | Referenz | Massnahme | Status |
|---|---|---|---|---|
| 16 | Cybersicherheits-Risikobewertung erstellen und dokumentieren | Art. 13(2) | P4.1 | [ ] |
| 17 | Technische Dokumentation erstellen | Annex VII | P4.6 | [ ] |
| 18 | EU-Konformitaetserklaerung ausstellen | Annex V | P4.7 | [ ] |
| 19 | CE-Kennzeichnung anbringen | Art. 28 | P4.7 | [ ] |
| 20 | Supportzeitraum festlegen und Enddatum angeben (mindestens 5 Jahre, oder die erwartete Nutzungsdauer falls kuerzer) | Art. 13(8) | P4.4 | [ ] |
| 21 | Informationen und Anweisungen fuer den Nutzer bereitstellen | Annex II | P4.8 | [ ] |
| 22 | Sorgfaltspflicht bei Drittkomponenten | Art. 13(5) | P4.2 | [ ] |

### 6.4 Meldepflichten (Art. 14, in Kraft seit 2026-09-11)

| Nr. | Pflicht | Frist | CRA-Referenz | Massnahme | Status |
|---|---|---|---|---|---|
| 23 | Fruehwarnung aktiv ausgenutzter Schwachstellen oder schwerer Sicherheitsvorfaelle | 24 h | Art. 14(2)(a), 14(4)(a) | P5.1 | [ ] |
| 24 | Vollstaendige Meldung an CSIRT und ENISA (Single Reporting Platform) | 72 h | Art. 14(2)(b), 14(4)(b) | P5.1, P5.2 | [ ] |
| 25 | Abschlussbericht aktiv ausgenutzter Schwachstellen | 14 Tage | Art. 14(2)(c) | P5.1 | [ ] |
| 26 | Abschlussbericht schwerer Sicherheitsvorfaelle | 1 Monat | Art. 14(4)(c) | P5.1 | [ ] |

**Bestandsprodukte (Art. 69(3)):** Die Meldepflicht nach Art. 14 gilt auch fuer Produkte, die vor dem
2027-12-11 in Verkehr gebracht wurden, und zwar ohne dass eine wesentliche Aenderung vorliegen muss.
Fuer alle uebrigen Pflichten greift Art. 69(2): vor dem 2027-12-11 in Verkehr gebrachte Produkte fallen
nur dann unter die Anforderungen, wenn sie eine wesentliche Aenderung erfahren.

**Operativer Stand (2026-09-28):** Die ENISA Single Reporting Platform ist seit 2026-09-11 in Betrieb
(`portal.cra-srp.enisa.europa.eu`), Zugang ueber EU Login mit Multi-Faktor-Authentifizierung und der
Rolle "Assigned Representative". Im ersten Release gibt es kein API und keine freiwilligen Meldungen
nach Art. 15. Ein Durchfuehrungsrechtsakt zu Format und Verfahren (Art. 14(10)) ist noch nicht erlassen;
Arbeitsgrundlage sind die von ENISA veroeffentlichten Pflichtfelder. Die Liste der als Koordinatoren
benannten CSIRTs wurde am 2026-09-04 veroeffentlicht, die Zuordnung erfolgt nach dem
Main-Establishment-Test (Art. 14(7)). Leitlinien der Kommission: C(2026) 5252 vom 2026-07-27,
Abschnitt 9.1 zu Art. 14. Der 72-h-Zaehler im Portal laeuft ab Absenden der 24-h-Meldung, nicht ab
Kenntnisnahme; der eigene Zeitstempel ab Kenntnis ist daher separat zu fuehren.

## 7. Verknuepfung: Risiko, Pflicht, Massnahme, Detaildokument

Jede Massnahme P1.1 bis P5.2 hat genau eine Zeile. Diese Tabelle ist die Bruecke zwischen der
Risiko- und Evidenzanalyse (Risiko-IDs R1 bis R9), den CRA-Pflichten aus Abschnitt 6, den
RED-Pflichten aus Abschnitt 3 und den Detaildokumenten aus Abschnitt 4.

| Risiko (Analyse) | Pflicht-Nr. | Massnahme | Detaildokument | Stand |
|---|---|---|---|---|
| R1 Keine Authentifizierung auf der Steueroberflaeche | 2 | P1.1 Authentifizierung fuer alle mutierenden Endpunkte | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | offen |
| R4 Zustandsaenderung per GET, CSRF | 6 | P1.2 POST-Umstellung und CSRF-Schutz | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | offen |
| R3 Kalibrierung bildet den Druck zu niedrig ab | 9 | P1.3 Sensorkalibrierung auf den Sensorbereich begrenzen | `plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` | offen |
| R6 Unbegrenzte PID-Gains | 4 | P1.4 Harte Obergrenzen fuer PID-Gains | `plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md` | offen |
| RED-1, RED-2 Normpflicht aus dem RED | RED-1, RED-2 | P1.5 EN-18031-Konformitaetsweg und CE-Route klaeren | `plan/2026-09-28_P1-5_RED_Conformity_Route_Plan.md` | offen, gilt bis 2027-12-11 |
| R5 Beliebiger Dateizugriff auf LittleFS | 3 | P2.1 Pfadbereinigung, JSON-Konfiguration nicht ausliefern | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | teilweise entworfen (JSON-Sperre), Pfadbereinigung offen |
| R2 Fest verdrahtetes AP-Passwort | 1, 10 | P2.2 AP-Passwort zur Laufzeit konfigurierbar | `plan/2026-08-31_P2-2_AP_Password_Plan.md` | offen |
| R7 Flash-Verschleiss und DoS ueber settings.save() | 8 | P2.3 Ratenbegrenzung und Save-Debouncing | `plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md` | offen |
| R9 Fehlende Synchronisation der settings-Zugriffe | kein Annex-I-Punkt (Robustheit) | P3.1 Mutex fuer den settings-Zugriff | `plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` | erledigt in v2.7.3 |
| R8 Client-Connect-DoS | 8 | P3.2 Passive Ablehnung neuer Clients statt Trennung aller | kein Detaildokument, Nachweis in `analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` | erledigt in v2.7.3 |
| Funkebene ohne eigene Authentifizierung | 5 | P3.3 Verschluesselung oder Authentifizierung auf Funkebene | `plan/2026-09-28_P3-3_Radio_Encryption_Plan.md` | offen |
| kein Risikobezug (Normpflicht) | 16 | P4.1 Cybersicherheits-Risikobewertung | `plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md` | offen |
| kein Risikobezug (Normpflicht) | 11, 22 | P4.2 SBOM und Sorgfaltspflicht bei Drittkomponenten | `plan/2026-09-28_P4-2_SBOM.md` | offen |
| kein Risikobezug (Normpflicht) | 7, 13, 15 | P4.3 Signierte OTA-Updates und Update-Tests | `plan/2026-09-28_P4-3_Signed_OTA.md` | offen |
| kein Risikobezug (Normpflicht) | 20 | P4.4 Supportzeitraum und Enddatum | `plan/2026-09-28_P4-4_Support_Period.md` | offen |
| kein Risikobezug (Normpflicht) | 12, 14 | P4.5 Schwachstellenmanagement und Disclosure-Richtlinie | `plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md` | offen |
| kein Risikobezug (Normpflicht) | 17 | P4.6 Technische Dokumentation (Annex VII) | `plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md` | offen |
| kein Risikobezug (Normpflicht) | 18, 19 | P4.7 EU-Konformitaetserklaerung und CE-Kennzeichnung | `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md` | offen |
| kein Risikobezug (Normpflicht) | 21 | P4.8 Nutzerinformationen (Annex II) | `plan/2026-09-28_P4-8_User_Information_Annex_II.md` | offen |
| kein Risikobezug (Meldepflicht, akut) | 23, 25, 26 | P5.1 Meldeprozess fuer aktiv ausgenutzte Schwachstellen und schwere Sicherheitsvorfaelle | `plan/2026-09-28_P5-1_Reporting_Process.md` | offen, akut seit 2026-09-11 |
| kein Risikobezug (Meldepflicht, akut) | 24 | P5.2 Zugang zur ENISA Single Reporting Platform und zustaendiges CSIRT | `plan/2026-09-28_P5-2_SRP_CSIRT_Access.md` | offen, akut seit 2026-09-11 |

## 8. Arbeitspakete nach Prioritaet

Diese Statusliste ist die einzige Statusquelle. Jede Massnahme verweist auf ihr Detaildokument.

### Phase 1: Sicherheitskritisch (CRA Annex I Kernpflichten)

- [ ] P1.1 Authentifizierung fuer alle mutierenden Endpunkte (Login-Seite + Erstinbetriebnahme-Provisioning), `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`
- [ ] P1.2 POST-Umstellung + CSRF-Schutz fuer /set und /resetPID, `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`
- [ ] P1.3 Sensorkalibrierung auf physischen Sensorbereich begrenzen (sensor_maxP <= SENSOR_MAX_BAR), `plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md`
- [ ] P1.4 Harte Obergrenzen fuer PID-Gains, `plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md`
- [ ] P1.5 EN-18031-Konformitaetsweg und CE-Route klaeren (RED-1, RED-2, gilt bis 2027-12-11), `plan/2026-09-28_P1-5_RED_Conformity_Route_Plan.md`

### Phase 2: Haertung

- [ ] P2.1 Pfadbereinigung in handleFileRead(), JSON-Konfiguration nicht ueber HTTP ausliefern, `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`
- [ ] P2.2 AP-Passwort zur Laufzeit konfigurierbar, `plan/2026-08-31_P2-2_AP_Password_Plan.md`
- [ ] P2.3 Ratenbegrenzung + Save-Debouncing, `plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md`

### Phase 3: Robustheit

- [x] P3.1 Mutex fuer settings-Zugriff (v2.7.3), `plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md`
- [x] P3.2 Client-Connect-DoS (v2.7.3), Nachweis in der Risiko- und Evidenzanalyse (R8)
- [ ] P3.3 Verschluesselung oder Authentifizierung auf Funkebene, `plan/2026-09-28_P3-3_Radio_Encryption_Plan.md`

### Phase 4: Compliance-Paket (Dokumentation)

- [ ] P4.1 Cybersicherheits-Risikobewertung dokumentieren, `plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md`
- [ ] P4.2 SBOM erstellen (Arduino core, Arduino-ESP32, LittleFS, chart.js, Drittbibliotheken), `plan/2026-09-28_P4-2_SBOM.md`
- [ ] P4.3 Signierte OTA-Update-Mechanik implementieren und Updates testen, `plan/2026-09-28_P4-3_Signed_OTA.md`
- [ ] P4.4 Supportzeitraum (mindestens 5 Jahre, oder erwartete Nutzungsdauer falls kuerzer) festlegen und Enddatum dokumentieren, `plan/2026-09-28_P4-4_Support_Period.md`
- [ ] P4.5 Vulnerability-Disclosure-Richtlinie + Sicherheitskontakt, `plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md`
- [ ] P4.6 Technische Dokumentation (Annex VII), `plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md`
- [ ] P4.7 EU-Konformitaetserklaerung (Annex V) + CE-Kennzeichnung, `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md`
- [ ] P4.8 Nutzerinformationen (Annex II) aktualisieren, `plan/2026-09-28_P4-8_User_Information_Annex_II.md`

### Phase 5: Meldeprozess (akut, Art. 14 in Kraft seit 2026-09-11)

- [ ] P5.1 Meldeprozess fuer aktiv ausgenutzte Schwachstellen und schwere Sicherheitsvorfaelle einrichten, `plan/2026-09-28_P5-1_Reporting_Process.md`
- [ ] P5.2 Zugang zur ENISA Single Reporting Platform (SRP) und zustaendiges CSIRT klaeren, `plan/2026-09-28_P5-2_SRP_CSIRT_Access.md`

## 9. Zeitplan (Fristen)

| Datum | Ereignis / Pflicht |
|---|---|
| 2024-12-10 | CRA in Kraft getreten |
| 2025-01-30 | EN 18031-1/-2/-3 durch Durchfuehrungsbeschluss (EU) 2025/138 im Amtsblatt zitiert, mit Restriktionen bei den Passwortklauseln |
| 2025-08-01 | Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444 (RED Cyber) anwendbar, gilt bis 2027-12-11 |
| 2026-06-11 | Kapitel IV (Konformitaetsbewertungsstellen) anwendbar |
| **2026-09-11** | Meldepflichten (Art. 14) in Kraft, seit diesem Datum verbindlich |
| **2027-12-11** | CRA gilt vollstaendig; RED-Cyber endet, Aufhebung der Delegierten VO (EU) 2022/30 durch Delegierte VO (EU) 2026/339 |

## 10. Strafen (Kontext)

Die Strafen nach Art. 64 gelten fuer die CRA. Verstoesse gegen die RED-Cyber-Anforderungen
sanktionieren die Mitgliedstaaten nach nationalem Recht zur Richtlinie 2014/53/EU.

- Bis zu EUR 15.000.000 oder 2,5 % des weltweiten Jahresumsatzes (Verstoesse gegen Annex I, Art. 13, Art. 14)
- Bis zu EUR 10.000.000 oder 2 % (sonstige Pflichten, unter anderem Art. 28 und die Konformitaetspflichten)
- Bis zu EUR 5.000.000 oder 1 % (falsche, unvollstaendige oder irrefuehrende Informationen)
- Art. 64(10): Kleinst- und Kleinunternehmen sind von Geldstrafen nur fuer die 24-h-Frist des Art. 14(2)(a) und Art. 14(4)(a) ausgenommen, nicht von den uebrigen Fristen der Meldepflicht

## 11. Quellen

1. Regulation (EU) 2024/2847, EUR-Lex: https://eur-lex.europa.eu/eli/reg/2024/2847/oj (Volltext lokal unter `sources/EU_CRA_Regulation_2024_2847_EN.pdf`)
2. EU-Kommission, Cyber Resilience Act: https://digital-strategy.ec.europa.eu/en/policies/cyber-resilience-act
3. EU-Kommission, CRA Reporting: https://digital-strategy.ec.europa.eu/en/policies/cra-reporting
4. EU-Kommission, Leitlinien C(2026) 5252 vom 2026-07-27: https://digital-strategy.ec.europa.eu/en/library/commission-publishes-new-guidance-support-timely-cyber-resilience-act-implementation
5. EU-Kommission, CRA Standardisation (Normungsauftrag M/606): https://digital-strategy.ec.europa.eu/en/policies/cra-standardisation
6. ENISA, Single Reporting Platform (SRP): https://www.enisa.europa.eu/topics/product-security/single-reporting-platform-srp
7. ENISA, Liste der als Koordinatoren benannten CSIRTs (veroeffentlicht 2026-09-04): https://www.enisa.europa.eu/topics/product-security/single-reporting-platform-srp/list-of-csirts-designated-as-coordinators
8. Delegierte VO (EU) 2022/30 (RED Cyber, gilt bis 2027-12-11): https://eur-lex.europa.eu/eli/reg_del/2022/30/oj
9. Delegierte VO (EU) 2023/2444 (Anwendungsdatum 2025-08-01): https://eur-lex.europa.eu/eli/reg_del/2023/2444/oj/eng
10. Delegierte VO (EU) 2026/339 (Aufhebung von 2022/30 ab 2027-12-11): https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX%3A32026R0339
11. Durchfuehrungsbeschluss (EU) 2025/138 (EN 18031-1/-2/-3, mit Restriktionen; RED-Norm, gilt bis 2027-12-11): https://eur-lex.europa.eu/eli/dec_impl/2025/138/oj
12. ETSI EN 303 645 (technischer Massstab, keine Rechtswirkung, keine Konformitaetsvermutung weder unter RED noch unter CRA): https://www.etsi.org/deliver/etsi_en/303600_303699/303645/

## 12. Revisionshistorie

| Datum | Aenderung |
|---|---|
| 2026-08-29 | Erstanlage als Arbeitsliste, abgeleitet aus der Webinterface-Sicherheitsanalyse |
| 2026-08-31 | Verweise auf die Plaene zur Authentifizierung, zum AP-Passwort und zur Kalibrierung ergaenzt |
| 2026-09-28 | Umbau zum Master: Dokumentenregister, durchgehende Nummerierung 1 bis 26, Verknuepfungstabelle mit Risiko-IDs und Detaildokumenten, Art. 14 als in Kraft seit 2026-09-11, Rechtsstand ergaenzt (Delegierte VO (EU) 2026/339, C(2026) 5252, SRP-Betrieb, CSIRT-Liste, Art. 69(3)), Strafstufen und Quellen erweitert |
| 2026-09-30 | RED und CRA getrennt: neuer Abschnitt 1 Regulatorischer Rahmen, Abschnitt 3 RED-Pflichten RED-1 bis RED-3 mit Gueltigkeit bis 2027-12-11, Anwendbarkeit auf VentCon2 reduziert, EN 303 645 als technischen Massstab gekennzeichnet, Massnahme P1.5 ergaenzt, Abschnitte 4 bis 12 neu nummeriert |
