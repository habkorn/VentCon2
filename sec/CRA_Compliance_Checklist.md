# VentCon2 - EU CRA Compliance Checklist

Rolle: Grundlage (Master) für alle Dokumente in `sec/`. Die Detaildokumente sind in Abschnitt 4
registriert und verweisen auf dieses Dokument.
Stand: 2026-10-01
Bezug: Regulation (EU) 2024/2847 (CRA); Radio Equipment Directive 2014/53/EU
Status: zurückgestellt für den Unionsmarkt. Prototypenbetrieb, geplanter Vertrieb Indien, kein
EU-Vertrieb. Wiedervorlage beim ersten Vertrieb, der ersten unentgeltlichen Demo oder der ersten
Auslieferung in die Union. Davon unabhängig bleiben P1.1, P1.2, P1.3 und P2.2 aktiv, weil der
Prototyp am Netz betrieben wird. Dies ist die einzige Statusquelle; Detaildokumente führen keinen
Status.
Vorgänger: Arbeitsliste vom 2026-08-29 (vormals `sec/2026-08-29_CRA_Compliance_Checklist.md`, seit 2026-09-28 dieses Dokument)
Normquelle: `sources/EU_CRA_Regulation_2024_2847_EN.pdf` (CRA-Volltext)
Umbauplan: `plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md`
Rahmenplan: `plan/2026-09-30_RED_CRA_Abgrenzung_Plan.md`

## 1. Regulatorischer Rahmen: RED und CRA

Dieses Produkt trifft zwei Regelwerke, dazu kommen technische Normen ohne eigene Rechtswirkung.
Die drei Ebenen werden getrennt geführt; alle weiteren Abschnitte bauen darauf auf.

| Merkmal | RED-Cyber (gilt bis 2027-12-11) | CRA |
|---|---|---|
| Rechtsgrundlage | Art. 3(3)(d)(e)(f) der Richtlinie 2014/53/EU, ausgestaltet durch Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444 | Verordnung (EU) 2024/2847 |
| Anwendungsbereich | internetfähige Funkgeräte, hier der ESP32 mit Wi-Fi | Produkte mit digitalen Elementen, hier die Ventilsteuerung |
| Geltungsbereich | Bereitstellung auf dem Unionsmarkt, auch unentgeltlich | derselbe Anknüpfungspunkt, die Bereitstellung auf dem Unionsmarkt |
| Anwendbar seit | 2025-08-01 | Art. 14 seit 2026-09-11, vollständig ab 2027-12-11 |
| Ende | Aufhebung der Delegierten VO (EU) 2022/30 durch Delegierte VO (EU) 2026/339, wirksam ab 2027-12-11 | unbefristet, Supportzeitraum von mindestens fünf Jahren |
| Inhalt der Anforderungen | Netzwerkschutz, Schutz personenbezogener Daten, Betrugsschutz | Annex I Teil I und II, Art. 13 Herstellerpflichten, Art. 14 Meldepflichten, Art. 29 und 30 CE-Kennzeichnung |
| Nachweisweg | Selbstbewertung, bei ausgenommenen Klauseln Modul B+C oder Modul H | Modul A, Selbstbewertung, siehe Abschnitt 5 |
| Konformitätsvermutung | EN 18031-1/-2/-3, im Amtsblatt zitiert mit Restriktionen | keine Norm zitiert, Art. 27 derzeit nicht nutzbar |
| Meldepflicht | keine | Art. 14: 24 h, 72 h, 14 Tage, 1 Monat, siehe Abschnitt 6.4 |
| Sanktionsrahmen | nationales Recht der Mitgliedstaaten zur Richtlinie 2014/53/EU | Art. 64, siehe Abschnitt 10 |

### 1.1 Fünf Kernaussagen

1. RED-Cyber ist derzeit der verbindliche Nachweisweg und nicht redundant, er gilt bis
   2027-12-11. Wer heute in Verkehr bringt, braucht diesen Nachweis für die CE-Kennzeichnung.
2. Am 2027-12-11 wird die Delegierte VO (EU) 2022/30 aufgehoben. Die Cyber-Anforderungen gehen in die
   CRA über, danach gibt es weder Lücke noch Dopplung.
3. Die CRA-Meldepflichten gelten schon jetzt, unabhängig von der RED-Route.
4. EN 303 645 ist technischer Maßstab ohne Rechtswirkung. Die Norm ist kein harmonisierter Rechtsakt
   und begründet keine Konformitätsvermutung, weder unter dem RED noch unter der CRA.
5. Das CE-Zeichen stammt heute aus dem RED und ab 2027 aus der CRA. Beide Nachweise bleiben im Dossier
   getrennt, siehe `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md`.

### 1.2 Gültigkeit

Jede Aussage mit RED-Bezug in diesem Dossier gilt bis zum 2027-12-11, sofern sie nicht ausdrücklich
anders datiert ist. Aussagen mit CRA-Bezug gelten unbefristet und greifen gestaffelt.

## 2. Anwendbarkeit

| Frage | Antwort | Status |
|---|---|---|
| Ist VentCon2 ein "Produkt mit digitalen Elementen" mit Datenverbindung? | Ja, ESP32-S3 Ventilsteuerung mit Webinterface, WLAN und Netzwerkverbindung; die Verbindung ist bestimmungsgemäßer Zweck (Art. 2(1), Art. 3(1)) | Geklärt |
| Greift eine Ausnahme nach Art. 2(2) bis (7)? | Nein; kein Medizinprodukt, keine Fahrzeug-Typgenehmigung, keine Luftfahrt-, Marine- oder Verteidigungsausrüstung, kein Ersatzteil | Geklärt |
| Ist VentCon2 ein internetfähiges Funkgerät? | Ja, ESP32 mit Wi-Fi. Damit greifen die Cyber-Anforderungen des RED bis 2027-12-11, siehe Abschnitt 1 und 3 | Geklärt |
| Wer ist Hersteller nach Art. 3(13)? | Wer das Produkt unter eigenem Namen bereitstellt, hier VENTREX; ein Vertrieb über Dritte ist nicht vorgesehen, damit entfallen Importeur und Händler nach Art. 3(16) und (17) | Geklärt |
| Wird es in der EU gewerblich in Verkehr gebracht? | Nein. Prototypenbetrieb im eigenen Aufbau, geplanter Vertrieb Indien, kein EU-Vertrieb | Geklärt |
| Vertrieb nur in Drittländern, zum Beispiel Indien? | Dann greifen weder CRA noch RED-Cyber. Anknüpfungspunkt ist allein der Unionsmarkt (Art. 2(1), Art. 3(21) und (22)) | Geklärt |
| Lieferung an Kunden in der Union, auch unentgeltlich? | Das ist Bereitstellung auf dem Unionsmarkt nach Art. 3(22); CRA und RED-Cyber greifen | Zu beachten |
| Änderung nach der Bereitstellung? | Änderungen, die die Konformität berühren, gelten nach Art. 3(30) als wesentliche Änderung und lösen die Pflichten neu aus; nach Art. 22 gilt auch der, der sie durchführt, als Hersteller | Beobachten |
| Wird die Firmware als Open Source bereitgestellt? | Nein, keine Veröffentlichung unter freier Lizenz; damit keine Pflichten als Open-Source-Steward nach Art. 24 | Geklärt |

**Entschieden:** VentCon2 wird derzeit nicht in der Union in Verkehr gebracht. CE-Weg, technische
Dokumentation, Supportzeitraum und Meldepflichten greifen erst mit dem ersten EU-Kontakt. Ein Vertrieb
über Importeure, Händler oder mandatierte Vertreter ist nicht vorgesehen, VentCon2 betrifft allein
VENTREX. Die Abgrenzung RED gegen CRA steht in Abschnitt 1.

**Bestandsschutz nach Art. 69(2) und (3):** Produkte, die vor dem 2027-12-11 in Verkehr gebracht wurden,
unterliegen ab dann den Pflichten nur bei wesentlicher Änderung; die Meldepflicht nach Art. 14 gilt für
sie seit dem 2026-09-11.

**Auslöser statt Herkunft:** CRA und RED-Cyber knüpfen nicht an den Sitz des Herstellers an, sondern
an die Bereitstellung auf dem Unionsmarkt. Ein Hersteller außerhalb der Union ist bei EU-Vertrieb voll
verpflichtet. Umgekehrt löst ein Vertrieb ausschließlich nach Indien oder in andere Drittländer keine
Pflicht aus. In diesem Fall ruhen P4.1 bis P4.8, P5.1, P5.2 und die RED-Route P1.5. Wiedervorlage ist
der erste Vertrieb in die Union, die erste kostenlose Demo dort oder die erste Auslieferung an einen
Kunden in der Union.

### 2.1 Prototypen und Vorserien

Die CRA kennt keinen Prototypenbegriff. Das Wort steht an genau zwei Stellen, beide Male in derselben
Regel: Art. 4(2) und die zugehörige Begründung Recital 36. Art. 3 enthält 51 Begriffsbestimmungen,
aber keine für Prototyp, Vorführmuster, Testmuster oder Beta. Ein Prototypenprivileg gibt es daher
nicht. Die Abgrenzung läuft über die Marktbegriffe: Art. 2(1) Anwendungsbereich, Art. 3(13)
Hersteller, Art. 3(21) und (22) Inverkehrbringen und Bereitstellung, dazu Recital 15 zur
Erwerbstätigkeit und Recital 16 zur eigenen Nutzung.

| Fall | CRA-Status | Grundlage |
|---|---|---|
| Bleibt im Labor, am Prüfstand oder in eigener Anlage | Nicht erfasst. Keine Bereitstellung, kein Hersteller nach der Verordnung, keine Meldepflicht | Art. 2(1), Art. 3(13), Art. 3(22) |
| Messe, Ausstellung oder Vorführung, auch wenn nicht konform | Zulässig. Bedingung ist ein sichtbarer Hinweis auf die Nichtkonformität und darauf, dass das Gerät bis zur Konformität nicht in Verkehr gebracht wird | Art. 4(2), Recital 36 |
| Firmware als Alpha, Beta oder Release Candidate zum Testen | Befristet zulässig mit sichtbarem Hinweis, nur zum Testen. Der Hersteller soll vorher eine Risikobewertung machen, die Sicherheitsanforderungen und die Schwachstellenbehandlung soweit möglich erfüllen und keinen Upgrade-Zwang auf Testversionen ausüben. Gilt nicht für Sicherheitsbauteile nach anderem Unionsrecht | Art. 4(3) und 4(4), Recital 37 |
| Weitergabe an Dritte, auch unentgeltlich, als Leihgabe oder Teststellung | Bereitstellung auf dem Unionsmarkt. Ab diesem Zeitpunkt volle Herstellerpflichten nach Art. 13 einschließlich der Meldepflicht nach Art. 14 | Art. 2(1), Art. 3(21), Art. 3(22), Art. 13(1), Recital 15 |
| Vertrieb ausschließlich in Drittländer, zum Beispiel Indien | Nicht erfasst, kein Unionsmarkt | Art. 2(1), Art. 3(21) |

**Hinweistext für Vorführungen** (sichtbar am Gerät oder am Stand):

> Prototyp, nicht konform mit Verordnung (EU) 2024/2847. Wird bis zur Konformität nicht in Verkehr
> gebracht.
>
> Prototype, not in conformity with Regulation (EU) 2024/2847. Not to be placed on the market until
> compliant.

**Übergang von der Vorserie zur Serie:** Die Pflichten greifen mit dem Inverkehrbringen, nicht mit der
Fertigung. Ab dem ersten Inverkehrbringen sind Konformitätsbewertung, technische Dokumentation,
EU-Konformitätserklärung, CE-Kennzeichnung und Supportzeitraum vollständig. Art. 69(2) nimmt
Produkte, die vor dem 2027-12-11 in Verkehr gebracht wurden, nur bei wesentlicher Änderung in die
Pflicht; die Meldepflicht nach Art. 69(3) gilt auch dort. Für Geräte, die den Unionsmarkt nie
erreichen, ist beides ohne Bedeutung.

**Erleichterungen statt Prototypenprivileg:** Art. 33(2) erlaubt Regulatory Sandboxes als kontrollierte
Testumgebung vor dem Inverkehrbringen unter Aufsicht der Marktaufsicht. Art. 33(5) lässt für Mikro-
und Kleinunternehmen eine vereinfachte technische Dokumentation nach Annex VII zu, die notifizierte
Stellen annehmen müssen. Art. 33(3) und (4) sehen Leitfaden und Finanzhilfen vor. Art. 64(10)
beschränkt die Strafausnahme auf die 24-h-Frist.

**Bezug zu VentCon2:** Der Prototyp läuft im eigenen Aufbau und wird Dritten nicht überlassen, damit
ist er nicht erfasst. Wird das Gerät in der Union vorgeführt, gilt die Hinweispflicht. Wird es in der
Union an einen Kunden, Testnutzer oder Interessenten übergeben, auch leihweise und unentgeltlich,
greifen die vollen Pflichten.

Aus der fehlenden Pflicht folgt keine Untätigkeit: der Prototyp hängt im Versuchsaufbau am Netz und
steuert Druck, deshalb bleiben P1.1 bis P1.3 und P2.2 in Arbeit. Login, POST mit CSRF, harte
Eingabegrenzen und ein konfigurierbares AP-Passwort schützen den Aufbau selbst und den Betreiber,
unabhängig vom Marktstatus.

## 3. RED-Pflichten (gilt bis 2027-12-11)

Diese Anforderungen gelten zusätzlich zur CRA und enden mit der Aufhebung der Delegierten
VO (EU) 2022/30. Die Nummerierung ist bewusst von der CRA-Nummerierung 1 bis 26 getrennt.

| ID | Anforderung | Rechtsgrundlage | Harmonisierte Norm | Maßnahme | Status |
|---|---|---|---|---|---|
| RED-1 | Netzwerkschutz, Schutz des Netzes vor Angriffen | RED Art. 3(3)(d) | EN 18031-1 | P1.5 Nachweis, P3.3 Funkebene | [ ] |
| RED-2 | Schutz personenbezogener Daten und der Privatsphäre | RED Art. 3(3)(e) | EN 18031-2 | P1.5 Nachweis, P2.2 Passwort | [ ] |
| RED-3 | Betrugsschutz bei Übertragung von Geld oder Werten | RED Art. 3(3)(f) | EN 18031-3 | voraussichtlich nicht einschlägig, Begründung in P1.5 | n/a |

**Folge der EN-18031-Restriktionen:** Die Normen sind durch Durchführungsbeschluss (EU) 2025/138 mit
Restriktionen im Amtsblatt zitiert. Die Passwortklauseln 6.2.5.1 und 6.2.5.2 sind von der
Konformitätsvermutung ausgenommen. Werden sie berührt, gilt für die betroffene Anforderung keine
Selbstbewertung, sondern Modul B+C oder Modul H mit notifizierter Stelle. Für ein Gerät mit festem
Standardpasswort ist das der Regelfall, siehe P2.2. Der Konformitätsweg wird in P1.5 geklärt.

## 4. Dokumentenregister

| Datei | Rolle | Maßnahmen | Letzte Prüfung |
|---|---|---|---|
| `CRA_Compliance_Checklist.md` | Master (Grundlage), einzige Statusquelle | alle, P1.1 bis P5.2 | 2026-10-01 |
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
| `grundlagen/2026-08-28_Mutex_How_It_Works.md` | Grundlagenerklärung | P3.1 | 2026-08-28 |
| `grundlagen/2026-08-29_Web_Auth_How_It_Works.md` | Grundlagenerklärung | P1.1, P1.2, P2.1 | 2026-09-30 |
| `plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md` | Strukturplan (Dokumentenstruktur, kein CRA-Bezug) | keine | 2026-09-28 |
| `plan/2026-09-30_RED_CRA_Abgrenzung_Plan.md` | Rahmenplan (Abgrenzung RED gegen CRA) | RED-1 bis RED-3, P1.5 | 2026-09-30 |
| `plan/2026-09-30_VENTREX_Praesentation_Plan.md` | Präsentationsplan (Folienaufbau) | keine | 2026-09-30 |
| `sources/EU_CRA_Regulation_2024_2847_EN.pdf` | Normquelle (CRA-Volltext) | keine | 2026-09-30 |
| `sources/EU_CRA_Regulation_2024_2847_EN/00_Index.md` | Normquelle als Markdown, 19 Dateien, Artikel 1 bis 71 und Anhänge I bis VIII zitierfähig | keine | 2026-09-30 |
| `plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md` | Strukturplan (Erschließung der Normquelle, kein CRA-Bezug) | keine | 2026-09-30 |
| `analyse/2026-09-30_CRA_Volltext_Verifikation.md` | Verifikationsprotokoll der Normquellen-Erschließung, sieben Gates | keine | 2026-09-30 |
Maßnahmen ohne eigenes Detaildokument (P3.2) sind in Abschnitt 7 mit ihrem Nachweis ausgewiesen.
Alle Detaildokumente tragen im Kopf `Grundlage: ../CRA_Compliance_Checklist.md`. Die Analyse trägt kein
eigenes Statusfeld; Befund- und Nachweißtaende sind Evidenz, der Gesamtstatus steht nur hier.

## 5. Produktklassifizierung

| Kategorie | Einordnung |
|---|---|
| Standardprodukt (Annex III/IV nicht betroffen) | Ja |
| Wichtige Produkte Class I/II (Annex III) | Nein |
| Kritische Produkte (Annex IV) | Nein |
| Konformitätsbewertung CRA | **Modul A (Selbstbewertung)**, keine notifizierte Stelle nötig |

**Vorbehalt aus dem RED:** Die Bewertung nach Modul A gilt für die CRA. Für die RED-Route kann sie
nicht gelten, weil die Passwortklauseln der EN 18031 von der Konformitätsvermutung ausgenommen sind.
Rechtsgrundlagen und die Folge für den CE-Weg stehen in Abschnitt 1 und 3.

**Normenstand CRA:** Keine CRA-harmonisierte Norm ist im Amtsblatt zitiert (Stand 2026-10-01). Eine Konformitätsvermutung nach Art. 27 besteht derzeit nicht; der Nachweis erfolgt über die Risikobewertung (P4.1) und die technische Dokumentation (P4.6). Der Normungsauftrag M/606 läuft.

## 6. Pflichten-Checkliste (CRA)

Legende Status: `[ ]` offen, `[x]` erledigt, `[?]` zu klären, `n/a` nicht anwendbar

Die Pflichten sind durchgehend von 1 bis 26 nummeriert. Die Spalte "Risiko / Maßnahme" nennt die
Risiko-ID aus der Risiko- und Evidenzanalyse und die Maßnahmen-ID; beide sind in Abschnitt 7 aufgelöst.
Die RED-Pflichten RED-1 bis RED-3 stehen getrennt in Abschnitt 3 und gelten bis 2027-12-11.

Die Liste deckt Annex I Teil I und Teil II sowie die formellen Herstellerpflichten ab, die für
VentCon2 bis zur Bereitstellung auf dem Unionsmarkt relevant sind. Sie ist keine abschließende
Aufzählung der Herstellerpflichten der Verordnung. Weitere Pflichten stehen in Art. 13(6) bis (23)
und in Art. 14(6) und (8): Meldung von Schwachstellen in Drittkomponenten (Art. 13(6)), zehnjährige
Verfügbarkeit der Sicherheitsupdates (Art. 13(9)), Aufbewahrung von technischer Dokumentation und
Erklärung (Art. 13(13)), Kennung und Kontaktdaten (Art. 13(15) bis (17)), Enddatum zum Kaufzeitpunkt
(Art. 13(19)), Konformitätserklärung beim Produkt (Art. 13(20)), Korrekturmaßnahmen, Rücknahme oder
Rückruf (Art. 13(21)), Zusammenarbeit mit der Marktüberwachung (Art. 13(22)), Betriebsaufgabe
(Art. 13(23)) sowie Zwischenbericht und Nutzerinformation nach Art. 14(6) und (8).

### 6.1 Sicherheitsanforderungen (Annex I, Teil I)

| Nr. | Pflicht | CRA-Referenz | Risiko / Maßnahme | Status |
|---|---|---|---|---|
| 1 | Keine universellen Standardpasswörter | Annex I, Teil I, (2)(b) und (2)(d); die Verordnung nennt kein wörtliches Passwortverbot, das Wort steht dort nur in Annex III als Produktkategorie; technischer Maßstab EN 303 645 5.1-1 | R2 / P2.2, Erstinbetriebnahme-Provisioning in P1.1 | [ ] |
| 2 | Authentifizierung und Zugriffskontrolle | Annex I, Teil I, (2)(d) | R1 / P1.1 | [ ] |
| 3 | Minimale Angriffsfläche | Annex I, Teil I, (2)(j) | R5 / P2.1 | [ ] |
| 4 | Schutz vor unbefugtem Zugriff, Eingabevalidierung | Annex I, Teil I, (2)(d); Eingabevalidierung ist Projektauslegung | R6 / P1.4 | [ ] |
| 5 | Sichere Kommunikation | Annex I, Teil I, (2)(e) und (f) | Funkebene / P3.3 | [ ] |
| 6 | Sichere Standardkonfiguration | Annex I, Teil I, (2)(b) | R4 / P1.2 | [ ] |
| 7 | Sichere Update-Mechanismen (signiert) | Annex I, Teil I, (2)(c); signierte Verteilung nach Teil II Nr. 7 | P4.3 | [ ] |
| 8 | Verfügbarkeit / DoS-Resistenz | Annex I, Teil I, (2)(h) und (i) | R7, R8 / P2.3, P3.2 | [ ] |
| 9 | Produktsicherheit (Notabschaltung) | kein Punkt in Annex I, Teil I; Sicherheitsfunktion ist Projektauslegung, nächster Punkt (2)(k) | R3 / P1.3 | [ ] |
| 10 | Sichere Speicherung von Zugangsdaten | Annex I, Teil I, (2)(e); technischer Maßstab EN 303 645 5.4-1 | R2 / P2.2 (verschlüsseltes NVS) | [ ] |

### 6.2 Schwachstellenbehandlung (Annex I, Teil II)

| Nr. | Pflicht | CRA-Referenz | Maßnahme | Status |
|---|---|---|---|---|
| 11 | SBOM (Software Bill of Materials) pflegen | Annex I, Teil II, Nr. 1 | P4.2 | [ ] |
| 12 | Schwachstellen identifizieren und dokumentieren | Annex I, Teil II, Nr. 1 | P4.5 | [ ] |
| 13 | Schwachstellen ohne Verzögerung beheben (Security-Updates) | Annex I, Teil II, Nr. 2 | P4.3 | [ ] |
| 14 | Vulnerability-Disclosure-Richtlinie und Sicherheitskontakt veröffentlichen | Annex I, Teil II, Nr. 5 und Nr. 6; Nr. 4 betrifft die Veröffentlichung zu behobenen Schwachstellen | P4.5 | [ ] |
| 15 | Sicherheitsupdates testen vor Auslieferung | Annex I, Teil II, Nr. 3 | P4.3 | [ ] |

### 6.3 Konformität und Markt

| Nr. | Pflicht | Referenz | Maßnahme | Status |
|---|---|---|---|---|
| 16 | Cybersicherheits-Risikobewertung erstellen und dokumentieren | Art. 13(2) und (3), Aufnahme in die technische Dokumentation nach Art. 13(4) | P4.1 | [ ] |
| 17 | Technische Dokumentation erstellen | Art. 31, Annex VII | P4.6 | [ ] |
| 18 | EU-Konformitätserklärung ausstellen | Art. 28, Annex V | P4.7 | [ ] |
| 19 | CE-Kennzeichnung anbringen | Art. 29 und Art. 30 | P4.7 | [ ] |
| 20 | Supportzeitraum festlegen und Enddatum angeben (mindestens 5 Jahre, oder die erwartete Nutzungsdauer falls kürzer) | Art. 13(8) und (19), Angabe in der Nutzerinformation nach Annex II Nr. 7 | P4.4 | [ ] |
| 21 | Informationen und Anweisungen für den Nutzer bereitstellen | Annex II | P4.8 | [ ] |
| 22 | Sorgfaltspflicht bei Drittkomponenten | Art. 13(5) | P4.2 | [ ] |

### 6.4 Meldepflichten (Art. 14, in Kraft seit 2026-09-11)

| Nr. | Pflicht | Frist | CRA-Referenz | Maßnahme | Status |
|---|---|---|---|---|---|
| 23 | Frühwarnung aktiv ausgenutzter Schwachstellen oder schwerer Sicherheitsvorfälle | 24 h | Art. 14(2)(a), 14(4)(a) | P5.1 | [ ] |
| 24 | Vollständige Meldung an CSIRT und ENISA (Single Reporting Platform) | 72 h | Art. 14(2)(b), 14(4)(b) | P5.1, P5.2 | [ ] |
| 25 | Abschlussbericht aktiv ausgenutzter Schwachstellen | 14 Tage nach Verfügbarkeit einer Abhilfe | Art. 14(2)(c) | P5.1 | [ ] |
| 26 | Abschlussbericht schwerer Sicherheitsvorfälle | 1 Monat nach der 72-h-Meldung | Art. 14(4)(c) | P5.1 | [ ] |

**Bestandsprodukte (Art. 69(3)):** Die Meldepflicht nach Art. 14 gilt auch für Produkte, die vor dem
2027-12-11 in Verkehr gebracht wurden, und zwar ohne dass eine wesentliche Änderung vorliegen muss.
Für alle übrigen Pflichten greift Art. 69(2): vor dem 2027-12-11 in Verkehr gebrachte Produkte fallen
nur dann unter die Anforderungen, wenn sie eine wesentliche Änderung erfahren.

**Anknüpfung der Fristen:** Die Fristen knüpfen an die Kenntnis des Herstellers an. Die 24-h- und
die 72-h-Frist laufen ab Kenntnis der aktiv ausgenutzten Schwachstelle oder des schweren
Sicherheitsvorfalls (Art. 14(2)(a) und (b), Art. 14(4)(a) und (b)). Der Abschlussbericht zu einer
aktiv ausgenutzten Schwachstelle ist spätestens 14 Tage nach Verfügbarkeit einer Korrektur- oder
Milderungsmaßnahme fällig (Art. 14(2)(c)), der Abschlussbericht zu einem schweren Sicherheitsvorfall
innerhalb eines Monats nach der 72-h-Meldung (Art. 14(4)(c)).

**Operativer Stand (2026-09-28):** Die ENISA Single Reporting Platform ist seit 2026-09-11 in Betrieb
(`portal.cra-srp.enisa.europa.eu`), Zugang über EU Login mit Multi-Faktor-Authentifizierung und der
Rolle "Assigned Representative". Im ersten Release gibt es kein API und keine freiwilligen Meldungen
nach Art. 15. Ein Durchführungsrechtsakt zu Format und Verfahren (Art. 14(10)) ist noch nicht erlassen;
Arbeitsgrundlage sind die von ENISA veröffentlichten Pflichtfelder. Die Liste der als Koordinatoren
benannten CSIRTs wurde am 2026-09-04 veröffentlicht, die Zuordnung erfolgt nach dem
Main-Establishment-Test (Art. 14(7)). Leitlinien der Kommission: C(2026) 5252 vom 2026-07-27,
Abschnitt 9.1 zu Art. 14. Der 72-h-Zähler im Portal läuft ab Absenden der 24-h-Meldung, nicht ab
Kenntnisnahme; der eigene Zeitstempel ab Kenntnis ist daher separat zu führen.

## 7. Verknüpfung: Risiko, Pflicht, Maßnahme, Detaildokument

Jede Maßnahme P1.1 bis P5.2 hat genau eine Zeile. Diese Tabelle ist die Brücke zwischen der
Risiko- und Evidenzanalyse (Risiko-IDs R1 bis R9), den CRA-Pflichten aus Abschnitt 6, den
RED-Pflichten aus Abschnitt 3 und den Detaildokumenten aus Abschnitt 4.

| Risiko (Analyse) | Pflicht-Nr. | Maßnahme | Detaildokument | Stand |
|---|---|---|---|---|
| R1 Keine Authentifizierung auf der Steueroberfläche | 2 | P1.1 Authentifizierung für alle mutierenden Endpunkte | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | aktiv im Prototypenbetrieb, Umsetzung offen |
| R4 Zustandsänderung per GET, CSRF | 6 | P1.2 POST-Umstellung und CSRF-Schutz | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | aktiv im Prototypenbetrieb, Umsetzung offen |
| R3 Kalibrierung bildet den Druck zu niedrig ab | 9 | P1.3 Sensorkalibrierung auf den Sensorbereich begrenzen | `plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` | aktiv im Prototypenbetrieb, Umsetzung offen |
| R6 Unbegrenzte PID-Gains | 4 | P1.4 Harte Obergrenzen für PID-Gains | `plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md` | offen |
| RED-1, RED-2 Normpflicht aus dem RED | RED-1, RED-2 | P1.5 EN-18031-Konformitätsweg und CE-Route klären | `plan/2026-09-28_P1-5_RED_Conformity_Route_Plan.md` | offen, gilt bis 2027-12-11 |
| R5 Beliebiger Dateizugriff auf LittleFS | 3 | P2.1 Pfadbereinigung, JSON-Konfiguration nicht ausliefern | `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | teilweise entworfen (JSON-Sperre), Pfadbereinigung offen |
| R2 Fest verdrahtetes AP-Passwort | 1, 10 | P2.2 AP-Passwort zur Laufzeit konfigurierbar | `plan/2026-08-31_P2-2_AP_Password_Plan.md` | aktiv im Prototypenbetrieb, Umsetzung offen |
| R7 Flash-Verschleiß und DoS über settings.save() | 8 | P2.3 Ratenbegrenzung und Save-Debouncing | `plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md` | offen |
| R9 Fehlende Synchronisation der settings-Zugriffe | kein Annex-I-Punkt (Robustheit) | P3.1 Mutex für den settings-Zugriff | `plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md` | erledigt in v2.7.3 |
| R8 Client-Connect-DoS | 8 | P3.2 Passive Ablehnung neuer Clients statt Trennung aller | kein Detaildokument, Nachweis in `analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` | erledigt in v2.7.3 |
| Funkebene ohne eigene Authentifizierung | 5 | P3.3 Verschlüsselung oder Authentifizierung auf Funkebene | `plan/2026-09-28_P3-3_Radio_Encryption_Plan.md` | offen |
| kein Risikobezug (Normpflicht) | 16 | P4.1 Cybersicherheits-Risikobewertung | `plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md` | offen |
| kein Risikobezug (Normpflicht) | 11, 22 | P4.2 SBOM und Sorgfaltspflicht bei Drittkomponenten | `plan/2026-09-28_P4-2_SBOM.md` | offen |
| kein Risikobezug (Normpflicht) | 7, 13, 15 | P4.3 Signierte OTA-Updates und Update-Tests | `plan/2026-09-28_P4-3_Signed_OTA.md` | offen |
| kein Risikobezug (Normpflicht) | 20 | P4.4 Supportzeitraum und Enddatum | `plan/2026-09-28_P4-4_Support_Period.md` | offen |
| kein Risikobezug (Normpflicht) | 12, 14 | P4.5 Schwachstellenmanagement und Disclosure-Richtlinie | `plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md` | offen |
| kein Risikobezug (Normpflicht) | 17 | P4.6 Technische Dokumentation (Annex VII) | `plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md` | offen |
| kein Risikobezug (Normpflicht) | 18, 19 | P4.7 EU-Konformitätserklärung und CE-Kennzeichnung | `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md` | offen |
| kein Risikobezug (Normpflicht) | 21 | P4.8 Nutzerinformationen (Annex II) | `plan/2026-09-28_P4-8_User_Information_Annex_II.md` | offen |
| kein Risikobezug (Meldepflicht) | 23, 25, 26 | P5.1 Meldeprozess für aktiv ausgenutzte Schwachstellen und schwere Sicherheitsvorfälle | `plan/2026-09-28_P5-1_Reporting_Process.md` | ruht bis Marktentscheidung |
| kein Risikobezug (Meldepflicht) | 24 | P5.2 Zugang zur ENISA Single Reporting Platform und zuständiges CSIRT | `plan/2026-09-28_P5-2_SRP_CSIRT_Access.md` | ruht bis Marktentscheidung |

**Lesart bei zurückgestelltem Marktstatus:** In den Zeilen P1.5, P4.1 bis P4.8, P5.1 und P5.2 steht
`offen` für `ruht bis zur Marktentscheidung`, siehe Abschnitt 8. Fristen laufen erst mit der
Bereitstellung auf dem Unionsmarkt. Die vier aktiven Zeilen P1.1, P1.2, P1.3 und P2.2 sind dort
gesondert gekennzeichnet.

## 8. Arbeitspakete nach Priorität

Diese Statusliste ist die einzige Statusquelle. Jede Maßnahme verweist auf ihr Detaildokument.
Aktiv bleiben P1.1, P1.2, P1.3 und P2.2, weil der Prototyp am Netz betrieben wird. Zurückgestellt sind
bis zur Marktentscheidung P1.5 sowie P4.1 bis P4.8, P5.1 und P5.2. Die übrigen Maßnahmen behalten
ihren Stand.

**Umzusetzen mit der Bereitstellung auf dem Unionsmarkt:** Die Pflichtmaßnahmen greifen mit der
Bereitstellung auf dem Unionsmarkt (Art. 2(1), Art. 3(21) und (22), Recital 15). Solange
ausschließlich in Drittstaaten vertrieben wird, entstehen aus CRA und RED-Cyber keine Pflichten, die
Umsetzung ist dann freiwillig.
Der Auslöser ist die Lieferung zur Verteilung oder Nutzung in der Union, auch unentgeltlich, und
ebenfalls die Bereitstellung durch einen Importeur oder Händler in der Union (Art. 3(16) und (17),
Art. 18 und 19).

### Phase 1: Sicherheitskritisch (CRA Annex I Kernpflichten)

- [ ] P1.1 Authentifizierung für alle mutierenden Endpunkte (Login-Seite + Erstinbetriebnahme-Provisioning), `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` (aktiv)
- [ ] P1.2 POST-Umstellung + CSRF-Schutz für /set und /resetPID, `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` (aktiv)
- [ ] P1.3 Sensorkalibrierung auf physischen Sensorbereich begrenzen (sensor_maxP <= SENSOR_MAX_BAR), `plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md` (aktiv)
- [ ] P1.4 Harte Obergrenzen für PID-Gains, `plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md`
- [ ] P1.5 EN-18031-Konformitätsweg und CE-Route klären (RED-1, RED-2, gilt bis 2027-12-11), `plan/2026-09-28_P1-5_RED_Conformity_Route_Plan.md` (ruht bis Marktentscheidung)

### Phase 2: Härtung

- [ ] P2.1 Pfadbereinigung in handleFileRead(), JSON-Konfiguration nicht über HTTP ausliefern, `plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md`
- [ ] P2.2 AP-Passwort zur Laufzeit konfigurierbar, `plan/2026-08-31_P2-2_AP_Password_Plan.md` (aktiv)
- [ ] P2.3 Ratenbegrenzung + Save-Debouncing, `plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md`

### Phase 3: Robustheit

- [x] P3.1 Mutex für settings-Zugriff (v2.7.3), `plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md`
- [x] P3.2 Client-Connect-DoS (v2.7.3), Nachweis in der Risiko- und Evidenzanalyse (R8)
- [ ] P3.3 Verschlüsselung oder Authentifizierung auf Funkebene, `plan/2026-09-28_P3-3_Radio_Encryption_Plan.md`

### Phase 4: Compliance-Paket (Dokumentation)

- [ ] P4.1 Cybersicherheits-Risikobewertung dokumentieren, `plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md`
- [ ] P4.2 SBOM erstellen (Arduino core, Arduino-ESP32, LittleFS, chart.js, Drittbibliotheken), `plan/2026-09-28_P4-2_SBOM.md`
- [ ] P4.3 Signierte OTA-Update-Mechanik implementieren und Updates testen, `plan/2026-09-28_P4-3_Signed_OTA.md`
- [ ] P4.4 Supportzeitraum (mindestens 5 Jahre, oder erwartete Nutzungsdauer falls kürzer) festlegen und Enddatum dokumentieren, `plan/2026-09-28_P4-4_Support_Period.md`
- [ ] P4.5 Vulnerability-Disclosure-Richtlinie + Sicherheitskontakt, `plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md`
- [ ] P4.6 Technische Dokumentation (Annex VII), `plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md`
- [ ] P4.7 EU-Konformitätserklärung (Annex V) + CE-Kennzeichnung, `plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md`
- [ ] P4.8 Nutzerinformationen (Annex II) aktualisieren, `plan/2026-09-28_P4-8_User_Information_Annex_II.md`

### Phase 5: Meldeprozess (ruht bis zur Marktentscheidung, Art. 14 in Kraft seit 2026-09-11)

- [ ] P5.1 Meldeprozess für aktiv ausgenutzte Schwachstellen und schwere Sicherheitsvorfälle einrichten, `plan/2026-09-28_P5-1_Reporting_Process.md`
- [ ] P5.2 Zugang zur ENISA Single Reporting Platform (SRP) und zuständiges CSIRT klären, `plan/2026-09-28_P5-2_SRP_CSIRT_Access.md`

## 9. Zeitplan (Fristen)

| Datum | Ereignis / Pflicht |
|---|---|
| 2024-12-10 | CRA in Kraft getreten |
| 2025-01-30 | EN 18031-1/-2/-3 durch Durchführungsbeschluss (EU) 2025/138 im Amtsblatt zitiert, mit Restriktionen bei den Passwortklauseln |
| 2025-08-01 | Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444 (RED Cyber) anwendbar, gilt bis 2027-12-11 |
| 2026-06-11 | Kapitel IV (Konformitätsbewertungsstellen) anwendbar |
| **2026-09-11** | Meldepflichten (Art. 14) in Kraft, seit diesem Datum verbindlich |
| **2027-12-11** | CRA gilt vollständig; RED-Cyber endet, Aufhebung der Delegierten VO (EU) 2022/30 durch Delegierte VO (EU) 2026/339 |

## 10. Strafen (Kontext)

Die Strafen nach Art. 64 gelten für die CRA. Verstöße gegen die RED-Cyber-Anforderungen
sanktionieren die Mitgliedstaaten nach nationalem Recht zur Richtlinie 2014/53/EU.

- Bis zu EUR 15.000.000 oder 2,5 % des weltweiten Jahresumsatzes (Verstöße gegen Annex I, Art. 13, Art. 14)
- Bis zu EUR 10.000.000 oder 2 % (sonstige Pflichten, unter anderem Art. 28 und die Konformitätspflichten)
- Bis zu EUR 5.000.000 oder 1 % (falsche, unvollständige oder irreführende Informationen)
- Art. 64(10): Kleinst- und Kleinunternehmen sind von Geldstrafen nur für die 24-h-Frist des Art. 14(2)(a) und Art. 14(4)(a) ausgenommen, nicht von den übrigen Fristen der Meldepflicht

## 11. Quellen

1. Regulation (EU) 2024/2847, EUR-Lex: https://eur-lex.europa.eu/eli/reg/2024/2847/oj (Volltext lokal unter `sources/EU_CRA_Regulation_2024_2847_EN.pdf`)
2. EU-Kommission, Cyber Resilience Act: https://digital-strategy.ec.europa.eu/en/policies/cyber-resilience-act
3. EU-Kommission, CRA Reporting: https://digital-strategy.ec.europa.eu/en/policies/cra-reporting
4. EU-Kommission, Leitlinien C(2026) 5252 vom 2026-07-27: https://digital-strategy.ec.europa.eu/en/library/commission-publishes-new-guidance-support-timely-cyber-resilience-act-implementation
5. EU-Kommission, CRA Standardisation (Normungsauftrag M/606): https://digital-strategy.ec.europa.eu/en/policies/cra-standardisation
6. ENISA, Single Reporting Platform (SRP): https://www.enisa.europa.eu/topics/product-security/single-reporting-platform-srp
7. ENISA, Liste der als Koordinatoren benannten CSIRTs (veröffentlicht 2026-09-04): https://www.enisa.europa.eu/topics/product-security/single-reporting-platform-srp/list-of-csirts-designated-as-coordinators
8. Delegierte VO (EU) 2022/30 (RED Cyber, gilt bis 2027-12-11): https://eur-lex.europa.eu/eli/reg_del/2022/30/oj
9. Delegierte VO (EU) 2023/2444 (Anwendungsdatum 2025-08-01): https://eur-lex.europa.eu/eli/reg_del/2023/2444/oj/eng
10. Delegierte VO (EU) 2026/339 (Aufhebung von 2022/30 ab 2027-12-11): https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX%3A32026R0339
11. Durchführungsbeschluss (EU) 2025/138 (EN 18031-1/-2/-3, mit Restriktionen; RED-Norm, gilt bis 2027-12-11): https://eur-lex.europa.eu/eli/dec_impl/2025/138/oj
12. ETSI EN 303 645 (technischer Maßstab, keine Rechtswirkung, keine Konformitätsvermutung weder unter RED noch unter CRA): https://www.etsi.org/deliver/etsi_en/303600_303699/303645/
13. Durchführungsverordnung (EU) 2025/2392 (Kategorien wichtiger Produkte mit digitalen Elementen), Fundstelle EUR-Lex, Aktenzeichen 2025/2392
14. Delegierte VO (EU) 2026/881 (Verzögerung der Weitergabe einer Meldung nach Art. 14), Fundstelle EUR-Lex, Aktenzeichen 2026/881

## 12. Revisionshistorie

| Datum | Änderung |
|---|---|
| 2026-08-29 | Erstanlage als Arbeitsliste, abgeleitet aus der Webinterface-Sicherheitsanalyse |
| 2026-08-31 | Verweise auf die Pläne zur Authentifizierung, zum AP-Passwort und zur Kalibrierung ergänzt |
| 2026-09-28 | Umbau zum Master: Dokumentenregister, durchgehende Nummerierung 1 bis 26, Verknüpfungstabelle mit Risiko-IDs und Detaildokumenten, Art. 14 als in Kraft seit 2026-09-11, Rechtsstand ergänzt (Delegierte VO (EU) 2026/339, C(2026) 5252, SRP-Betrieb, CSIRT-Liste, Art. 69(3)), Strafstufen und Quellen erweitert |
| 2026-09-30 | RED und CRA getrennt: neuer Abschnitt 1 Regulatorischer Rahmen, Abschnitt 3 RED-Pflichten RED-1 bis RED-3 mit Gültigkeit bis 2027-12-11, Anwendbarkeit auf VentCon2 reduziert, EN 303 645 als technischen Maßstab gekennzeichnet, Maßnahme P1.5 ergänzt, Abschnitte 4 bis 12 neu nummeriert |
| 2026-10-01 | Geltungsbereich in Abschnitt 2 erweitert: Drittland-Fall, Auslöser Unionsmarkt, unentgeltliche Lieferung, EU-Importeur und Vertreter nach Art. 18. Prototypenregel als Abschnitt 2.1 neu, mit Falltabelle, Hinweistext in zwei Sprachen, Übergang zur Serie und den Erleichterungen nach Art. 33. Status auf zurückgestellt gesetzt: Prototypenbetrieb, geplanter Vertrieb Indien, kein EU-Vertrieb. P1.1 bis P1.3 und P2.2 bleiben als aktiver Strang im Prototypenbetrieb in Arbeit. Quellen um die Durchführungsverordnung (EU) 2025/2392 und die Delegierte VO (EU) 2026/881 ergänzt, Normenstand auf 2026-10-01 datiert. Normzitate gegen die Quelle geprüft und korrigiert: Annex I Teil I auf Buchstabenpunkte (2)(a) bis (m) statt erfundener Ziffern, Teil II Nr. 4 auf Nr. 5 und 6, CE-Kennzeichnung auf Art. 29 und 30 statt Art. 28, Artikel für technische Dokumentation und Konformitätserklärung ergänzt, Bezugspunkte der Fristen aus Art. 14 präzisiert |
| 2026-10-01 | Abschnitt 2 um die Prüfpunkte erweitert, die im Deck auf Folie 8 stehen: Anwendungsbereich mit Datenverbindung nach Art. 2(1) und Art. 3(1) statt reiner Produkteigenschaft, Ausnahmen nach Art. 2(2) bis (7), Herstellerrolle nach Art. 3(13) mit Abgrenzung zu Importeur und Händler, wesentliche Änderung nach Art. 3(30) und Art. 22, Open-Source-Rolle nach Art. 24 sowie ein Absatz zum Bestandsschutz nach Art. 69(2) und (3) |
| 2026-10-01 | Abschnitt 2 auf den entschiedenen Stand gebracht: Die Zeile zum Inverkehrbringen in der Union steht jetzt auf Geklärt (kein EU-Vertrieb, Prototypenbetrieb im eigenen Aufbau, geplanter Vertrieb Indien). Die Zeilen zu Importeuren, Händlern und mandatierten Vertretern sind entfallen, weil ein Vertrieb über Dritte nicht vorgesehen ist und VentCon2 allein VENTREX betrifft. Der Absatz zur ersten Entscheidung ist durch den entschiedenen Stand ersetzt |
| 2026-10-01 | Schreibweise des gesamten Dossiers vereinheitlicht: deutsche Texte mit echten Umlauten statt Transliteration (1.177 Stellen in 49 Dateien), Regel dazu in `.github/copilot-instructions.md` festgehalten. Der englische Normtext der Extraktion bleibt wortgetreu, geändert wurden dort nur die Extraktionskommentare. Em-Dashes nach der Projektregel ersetzt, davon 29 im englischen Grundlagenpapier zum Mutex, zwei im AP-Passwort-Plan und sieben in den Projektanweisungen. Dazu die Prüfwerkzeuge `tools/cra_extract.py` und `tools/verify_cra_extract.py` konsistent nachgezogen, damit Kommentarform und Prüfmuster weiter zusammenpassen |
| 2026-10-01 | Konsistenz der Präsentation gegen den CRA-Volltext geprüft, Protokoll in `sec/analyse/2026-10-01_Praesentation_CRA_Konsistenz.md`, Plan in `sec/plan/2026-10-01_CRA_Konsistenz_Korrektur_Plan.md`. Im Master nachgezogen: Anknüpfung der Meldefristen an die Kenntnis in Abschnitt 6.4 ergänzt, Pflicht 16 auf Art. 13(2) und (3) mit Aufnahme nach Art. 13(4) präzisiert, Pflicht 20 um Art. 13(19) und Annex II Nr. 7 ergänzt, Pflicht 1 um den Hinweis, dass die Verordnung kein wörtliches Passwortverbot enthält, Abschnitt 6 um den Vollständigkeitsvorbehalt mit den Pflichten nach Art. 13(6) bis (23) und Art. 14(6) und (8) erweitert. Der Vorbehalt nach Art. 4(4) in Abschnitt 2.1 bleibt unverändert, er steht dort bereits |
| 2026-10-01 | Abschnitt 8 um den Auslöser der Maßnahmen ergänzt: Die Pflichtmaßnahmen greifen mit der Bereitstellung auf dem Unionsmarkt (Art. 2(1), Art. 3(21) und (22), Recital 15); bei Vertrieb ausschließlich in Drittstaaten entstehen keine Pflichten und die Umsetzung ist freiwillig. Auch unentgeltliche Lieferungen in die Union sowie die Bereitstellung durch Importeur oder Händler lösen die Pflichten aus. Der Satz steht so auch im Deck auf den Maßnahmenfolgen und in der Anwendbarkeitsfolge, Verifier-Regel V7 prüft ihn |
