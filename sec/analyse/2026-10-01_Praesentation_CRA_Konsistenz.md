# Analyse: Konsistenz der Präsentation mit der CRA

Datum: 2026-10-01
Gegenstand: `sec/2026-10-01_CRA_Compliance_Präsentation.pptx` (28 Folien)
Prüfmaßstab: Verordnung (EU) 2024/2847 im Volltext unter `sec/sources/EU_CRA_Regulation_2024_2847_EN/`,
ergänzend der Master `sec/CRA_Compliance_Checklist.md` als projekteigene Statusquelle
Methode: Aussagen des Decks je Folie erfasst und gegen Artikel, Anhänge und Erwägungsgründe geprüft

## Ergebnis in Kürze

Die Kernaussagen der Präsentation decken sich mit der Verordnung. Alle Anwendbarkeitsschwellen,
Fristen, Zeitpunkte, Konformitätswege, Anhangszuordnungen und die Übergangsregel sind belegt.
Gefunden wurden eine irrige Aussage zu den Meldefristen, eine Lücke beim Prototypenfall und
zwei ungenaue Belegstellen. Dazu der Hinweis, dass die Liste "Pflichten 1 bis 26" eine
Projektliste ist und nicht alle Herstellerpflichten der Verordnung enthält.

## Befund 1: Fristen der Meldepflicht (zu korrigieren)

Folie 21 sagt: "Die Fristen laufen ab Bereitstellung auf dem Unionsmarkt, der Prototyp ohne
Weitergabe löst keine Meldepflicht aus."

Die zweite Hälfte trifft zu (kein Inverkehrbringen, keine Pflichten, Art. 2(1) und Art. 3(21)).
Die erste Hälfte trifft nicht zu. Nach Art. 14 laufen die Fristen ab Kenntnis:

| Frist | Anknüpfung laut Verordnung | Frist im Deck |
|---|---|---|
| 24 h | ohne Verzug, spätestens 24 h ab Kenntnis (Art. 14(2)(a), 14(4)(a)) | 24 h, richtig |
| 72 h | ohne Verzug, spätestens 72 h ab Kenntnis (Art. 14(2)(b), 14(4)(b)) | 72 h, richtig |
| 14 Tage | spätestens 14 Tage, nachdem eine Abhilfe verfügbar ist (Art. 14(2)(c)) | richtig |
| 1 Monat | innerhalb eines Monats nach der 72-h-Meldung (Art. 14(4)(c)) | richtig |

Vorschlag für den Satz: "Die Fristen laufen ab Kenntnis der Schwachstelle oder des Vorfalls. Die
Pflicht selbst greift erst mit der Bereitstellung auf dem Unionsmarkt; der Prototyp ohne Weitergabe
löst keine Meldepflicht aus." Der Master führt den Punkt in seinem operativen Stand bereits
zutreffend (Zeitstempel ab Kenntnis separat führen), das Deck verkürzt ihn falsch.

## Befund 2: Prototypenfall ohne die Ausnahme für Sicherheitsbauteile (Lücke)

Folie 9 nennt für "Alpha, Beta oder Release Candidate zum Testen" die Grundlage "Art. 4(3),
Recital 37". Art. 4(3) wird durch **Art. 4(4)** eingeschränkt: "Paragraph 3 does not apply to safety
components as referred to in Union harmonisation legislation other than this Regulation." Der Master
führt diese Einschränkung in Abschnitt 2.1 ausdrücklich mit ("Gilt nicht für Sicherheitsbauteile nach
anderem Unionsrecht", Art. 4(3) und 4(4)). Die Präsentation lässt sie weg, obwohl VentCon2 ein Ventil
mit Notabschaltung steuert und damit ein Sicherheitsbauteil nach anderem Unionsrecht sein kann.
Vorschlag: Grundlage der Zeile zu "Art. 4(3) und 4(4), Recital 37" ergänzen und in der Zelle
"CRA-Status" aufnehmen: "nicht für Sicherheitsbauteile nach anderem Unionsrecht".

## Befund 3: "Pflichten 1 bis 26" ist eine Projektliste (Erwartung richtig stellen)

Die Folien 19 bis 21 und 27 sprechen von den "Pflichten 1 bis 26". Der Master nummeriert seine
Checkliste so, die Verordnung kennt diese Zählung nicht. Die Liste deckt Annex I Teil I und Teil II
vollständig ab, lässt aber Herstellerpflichten aus Kapitel II aus, die für VentCon2 einschlägig
werden, sobald das Produkt den Unionsmarkt erreicht:

| Nicht in der Liste | Inhalt |
|---|---|
| Art. 13(6) | Schwachstelle in einer Drittkomponente an deren Hersteller melden |
| Art. 13(9) | Sicherheitsupdates mindestens zehn Jahre oder für den Rest des Supportzeitraums verfügbar halten, je nachdem was länger ist |
| Art. 13(13) | Technische Dokumentation und Konformitätserklärung zehn Jahre oder für den Supportzeitraum bereithalten |
| Art. 13(15) | Typen-, Chargen- oder Seriennummer oder gleichwertige Kennung |
| Art. 13(16) | Name, Anschrift und Kontaktdaten des Herstellers am Produkt, auf der Verpackung oder in einer Begleitunterlage |
| Art. 13(17) | Einzige Kontaktstelle für Nutzer, auch für Schwachstellenmeldungen |
| Art. 13(18) | Informationen und Anweisungen nach Annex II, zehn Jahre oder für den Supportzeitraum verfügbar |
| Art. 13(19) | Enddatum des Supportzeitraums zum Zeitpunkt des Kaufs, mindestens Monat und Jahr |
| Art. 13(20) | Kopie oder vereinfachte Fassung der Konformitätserklärung beim Produkt |
| Art. 13(21) | Ab Bereitstellung und während des Supportzeitraums Korrekturmaßnahmen, Rücknahme oder Rückruf |
| Art. 13(22) | Zusammenarbeit mit der Marktüberwachung auf Anfrage |
| Art. 13(23) | Bei Betriebsaufgabe Behörden und Nutzer vorab informieren |
| Art. 14(6) | Zwischenbericht auf Anforderung des Koordinators |
| Art. 14(8) | Betroffene Nutzer über Schwachstelle oder Vorfall informieren |

Vorschlag: eine Fußzeile auf Folie 19, "Diese Quelle ist die Projektliste; die weiteren
Herstellerpflichten stehen in Art. 13(6) bis (23) und Art. 14(6) und (8)." Alternativ die Liste im
Master erweitern und beide Dokumente nachziehen.

## Befund 4: Belegstellen präzisieren (kein Widerspruch, aber ungenau)

| Stelle | Heute | Genauer |
|---|---|---|
| Folie 20, Pflicht 16 | "Cybersicherheits-Risikobewertung dokumentieren, Art. 13(2)" | Die Bewertungspflicht ist Art. 13(2), die Dokumentationspflicht Art. 13(3), die Aufnahme in die technische Dokumentation Art. 13(4). Empfehlung "Art. 13(2) und (3)" |
| Folie 20, Pflicht 20 | "Supportzeitraum und Enddatum angeben, Art. 13(8)" | Art. 13(8) regelt den Supportzeitraum (mindestens fünf Jahre, oder die erwartete Nutzungsdauer, wenn kürzer). Das Enddatum zum Zeitpunkt des Kaufs regelt Art. 13(19), die Angabe in der Nutzerinformation Annex II Nr. 7. Empfehlung "Art. 13(8) und (19)" |
| Folie 19, Pflicht 1 | "Keine universellen Standardpasswörter, Annex I, Teil I, (2)(b) und (d)" | Das Wort Passwort kommt in der Verordnung nur einmal vor, in Annex III als Produktkategorie "Password managers". Die Anforderung stützt sich auf die Voreinstellung sicher ab Werk, Punkt (2)(b), und auf (2)(d). Die Kennzeichnung von EN 303 645 5.1-1 als technischer Maßstab ist korrekt. Empfehlung: Klammerzusatz "(2)(b), sicher ab Werk" |
| Folie 27, "Fristen einplanen" | "Dokumentation und Erklärung zehn Jahre verfügbar halten" | Betrifft Art. 13(13). Daneben stehen die zehn Jahre für Sicherheitsupdates (Art. 13(9)) und für die Nutzerinformation (Art. 13(18)). Empfehlung: "Dokumentation, Erklärung und Nutzerinformation zehn Jahre oder für den Supportzeitraum bereithalten, Updates ebenso lang verfügbar halten" |

## Bestätigte Aussagen (Stichproben mit Beleg)

| Aussage im Deck | Beleg |
|---|---|
| Anwendungsbereich, Datenverbindung, kein Ersatzteil, keine Medizin-, Fahrzeug-, Luftfahrt-, Marine- oder Verteidigungsausnahme | Art. 2(1), 2(2)(a) bis (c), 2(3), 2(4), 2(6), 2(7) |
| Herstellerbegriff, Inverkehrbringen, Bereitstellung, wesentliche Änderung, Supportzeitraum | Art. 3(13), 3(21), 3(22), 3(30), 3(20) |
| Vorführung mit sichtbarem Hinweis, Alpha und Beta zum Testen mit Hinweis und Risikobewertung | Art. 4(2), 4(3), Recital 36, Recital 37 |
| Sorgfaltspflicht bei Drittkomponenten | Art. 13(5) |
| Supportzeitraum mindestens fünf Jahre, Enddatum in der Nutzerinformation | Art. 13(8), Art. 13(19), Annex II Nr. 7 |
| Meldungen 24 h, 72 h, 14 Tage, 1 Monat an CSIRT-Koordinator und ENISA über die Single Reporting Platform | Art. 14(1), 14(2)(a) bis (c), 14(3), 14(4)(a) bis (c), Art. 16 |
| Bestandsschutz: alte Produkte nur bei wesentlicher Änderung, Meldepflicht ausgenommen | Art. 69(2), Art. 69(3) |
| Zeitplan: Art. 14 ab 2026-09-11, vollständig ab 2027-12-11 | Art. 71(2) |
| Keine harmonisierte Norm, Vermutung nach Art. 27 nicht nutzbar | Art. 27(1), 27(2) |
| Konformitätserklärung, CE-Zeichen, technische Dokumentation | Art. 28 und Annex V, Art. 29 und 30, Art. 31 und Annex VII |
| Standardprodukt, Modul A, keine Verfahren nach Art. 32(2) und (3), kein Zertifikat nach Art. 8 | Art. 32(1)(a), 32(2), 32(3), Art. 7(1), Art. 8(1) |
| Keine Pflichten als Open-Source-Steward | Art. 24, Art. 3(14) |
| Zuordnung der Pflichten 1 bis 10 zu Annex I Teil I (2)(b) bis (2)(k) | Annex I Teil I (1), (2)(a) bis (m) geprüft, Buchstaben stimmen |
| Zuordnung der Pflichten 11 bis 15 zu Annex I Teil II | Annex I Teil II Nr. 1 bis 7 geprüft, Nummern stimmen |

## Nachträglich geprüfte Aussage: Auslöser der Maßnahmen (2026-10-01)

Geprüft wurde die Aussage, dass die Maßnahmen nur umzusetzen sind, wenn nicht ausschließlich in
Drittstaaten vertrieben wird. Sie trifft zu, mit folgender Genauigkeit:

| Beleg | Inhalt |
|---|---|
| Art. 2(1) | Anwendungsbereich: Produkte mit digitalen Elementen, die auf dem Markt bereitgestellt werden |
| Art. 3(21) und (22) | Inverkehrbringen und Bereitstellung beziehen sich auf den Unionsmarkt, entgeltlich oder unentgeltlich, in der Ausübung einer Geschäftstätigkeit |
| Recital 15 | "This Regulation applies to economic operators only in relation to products with digital elements made available on the market, hence supplied for distribution or use on the Union market in the course of a commercial activity." |
| Art. 6, Art. 13(1) und (12), Art. 28 bis 32 | Jede Herstellerpflicht ist an das Inverkehrbringen oder die Bereitstellung geknüpft ("when placing on the market", "shall be made available on the market only where") |
| Art. 69(2) und (3) | Sonderfall bereits bereitgestellter Produkte, setzt die Bereitstellung ebenfalls voraus |
| kein Treffer | Die Verordnung kennt keinen Auslöser "putting into service"; geprüft im Volltext, es gibt keine Pflicht ohne Bereitstellung auf dem Unionsmarkt |

Zwei Punkte sind dabei zu beachten: Unentgeltliche Lieferungen in die Union sind Bereitstellung
(Art. 3(22)), und eine Bereitstellung durch Importeur oder Händler in der Union löst deren Pflichten
aus (Art. 3(16) und (17), Art. 18 und 19). Eine Tätigkeit außerhalb einer Geschäftstätigkeit ist keine
Bereitstellung (Recital 15 und 16).

Umgesetzt: Folie 11 Fußzeile (Auslöser), Folien 15 und 16 graue Zeile unter der Maßnahmentafel,
Folie 27 (Was der Marktstatus bedeutet), Master Abschnitt 8. Verifier-Regel V7 prüft den Hinweis in
beiden Dokumenten, Gegenprobe bestanden.

## Nicht aus der Verordnung prüfbar

Diese Angaben stammen aus externen Rechtsakten und Betriebsmeldungen. Sie sind im Deck als Quellen
geführt und waren nicht Gegenstand dieser Prüfung, sollten aber vor einer Außenvorlage gegen das
Amtsblatt nachgezogen werden: Delegierte VO (EU) 2022/30, 2023/2444, 2026/339 und 2026/881,
Durchführungsbeschluss (EU) 2025/138, Durchführungsverordnung (EU) 2025/2392, Leitlinien
C(2026) 5252, Normungsauftrag M/606, Betrieb der Single Reporting Platform, CSIRT-Koordinatorenliste
vom 2026-09-04. Die Verordnung stützt die Existenz dieser Rechtsakte an folgenden Stellen:
Delegierte Rechtsakte zur Verzögerung der Weitergabe Art. 14(9) und Art. 16(2), technische
Beschreibung der Kategorien Art. 7(4), Standardisierungsauftrag Art. 27(1), Plattform Art. 16(1).

## Umsetzung der Korrekturen (2026-10-01)

Plan: `sec/plan/2026-10-01_CRA_Konsistenz_Korrektur_Plan.md`, Entscheidungen E35 bis E39.
Alle vier Befunde sind korrigiert, dazu fünf neue Verifier-Regeln.

Nachtrag: Am selben Tag haben die Abschnitte 2 und 3 die Plätze getauscht (E40). Die Belege der
Folie 9 (Alpha, Beta, Release Candidate) stehen damit auf Folie 12 als 3.3; die übrigen Foliennummern
dieses Protokolls bleiben unverändert.

| Befund | Umsetzung | Abnahme |
|---|---|---|
| 1 Fristen | Folie 21: "Die Fristen laufen ab Kenntnis der Schwachstelle oder des Vorfalls. Die Pflicht greift erst mit der Bereitstellung auf dem Unionsmarkt". Master 6.4 mit dem Absatz zur Anknüpfung der Fristen | Verifier-Regel V1, Gegenprobe bestanden |
| 2 Art. 4(4) | Folie 12 (3.3, am 2026-10-01 von Folie 9 hierher getauscht): Halbsatz zum Sicherheitsbauteil, Grundlage "Art. 4(3) und Art. 4(4), Recital 37" | Verifier-Regel V5, Gegenprobe bestanden |
| 3 Pflichtenliste | Folie 19: Hinweistext "Projektliste: die Nummern 1 bis 26 sind die Zählung dieses Dossiers. Weitere Herstellerpflichten stehen in Art. 13(6) bis (23) und Art. 14(6) und (8)." Master Abschnitt 6 mit dem Vollständigkeitsvorbehalt und der Aufzählung der weiteren Pflichten | Verifier-Regel V4, Gegenprobe bestanden |
| 4 Belegstellen | Folie 20 Pflicht 16 "Art. 13(2) bis (4)", Pflicht 20 "Art. 13(8) und (19), Annex II Nr. 7"; Folie 19 Hinweis zum fehlenden wörtlichen Passwortverbot; Folie 27 Zehnjahresfristen vervollständigt | Verifier-Regeln V2 und V3, Gegenproben bestanden |

Neue Verifier-Regeln: V1 Fristen knüpfen an die Kenntnis (Deck und Master), V2 zitierte Absätze
existieren in der Normquelle (Deck und Master), V3 zitierte Punkte von Annex I Teil I und Teil II
existieren, V4 Pflichtenliste 1 bis 26 nur mit Hinweis auf den Charakter als Projektliste,
V5 Art. 4(3) nur zusammen mit Art. 4(4). Gegenproben über `tools/cra_rule_control.py`, sechs Proben,
alle bestanden.

Abnahme: Verifier ohne Fehler und Hinweise, `tools/audit_plan_umsetzung.py` unverändert grün
(keine Nummer und kein Titel wechselt), Sichtprüfung der Folien 9, 19, 20, 21 und 27 ohne Überlauf.
