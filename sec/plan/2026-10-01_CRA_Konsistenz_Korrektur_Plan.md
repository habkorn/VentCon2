# Plan: Korrektur der CRA-Konsistenzbefunde in Präsentation und Master

Datum: 2026-10-01
Grundlage: `sec/analyse/2026-10-01_Praesentation_CRA_Konsistenz.md` (Befunde 1 bis 4)
Betroffene Dateien: `build_ventrex_deck.py`, `verify_ventrex_deck.py`, `sec/CRA_Compliance_Checklist.md`,
`sec/2026-10-01_CRA_Compliance_Praesentation.pptx`
Status: Plan, entschieden am 2026-10-01, umgesetzt und verifiziert am 2026-10-01

## Umsetzung und Abweichungen

Alle Korrekturen sind umgesetzt: Master (M1 bis M5, M7), Deck (K1 bis K6), Verifier (V1 bis V5).
Zwei Umsetzungsdetails weichen von der Textfassung dieses Plans ab, beide aus Platzgründen:

| Punkt | Plan | Umgesetzt |
|---|---|---|
| K3 | Notiz in der Spalte CRA-Referenz der Pflicht 1 | Die Zelle fasst nur zwei Zeilen; eine dritte Zeile würde die Tabelle in die Fußzeile schieben. Die Notiz steht deshalb als zweite Zeile im Hinweistext unter der Tabelle, zusammen mit dem Hinweis aus K6 |
| K4 Zeile 16 | "Art. 13(2) und (3), Aufnahme nach Art. 13(4)" | Die Referenzspalte der Folie 20 fasst rund 44 Zeichen je Zeile, die Langfassung hätte zwei Zeilen gebraucht. Umgesetzt ist die Kurzform "Art. 13(2) bis (4)", die Langfassung steht im Master |

Dazu zwei Werkzeugschritte, die der Plan nur allgemein beschreibt:

- `tools/cra_rule_control.py` führt die Gegenproben für V1 bis V5 aus (fünf Deckvarianten und eine
  Mastervariante), legt je Probe eine Kontrolldatei an und löscht sie sofort wieder.
- Der Verifier nimmt den Master als zweites Argument entgegen. Ohne das läßt sich die Masterregel
  aus V1 nicht gegenprüfen. Dabei fiel auf, dass der Verifier den Masterpfad weiter unten erneut
  fest zugewiesen hat; die zweite Zuweisung ist entfernt.

## Entscheidungen des Nutzers (2026-10-01)

| Nr. | Entscheidung |
|---|---|
| E35 | Umfang: alle vier Befunde werden korrigiert, dazu die fünf Verifier-Regeln |
| E36 | Befund 3 wird über einen Hinweistext auf Folie 19 gelöst. Die Liste bleibt bei 26 Pflichten, keine Nummer verschiebt sich, keine neue Folie |
| E37 | Befund 2 wird nur in der Grundlage der Zeile gelöst (Art. 4(3) und 4(4), Recital 37) plus dem Halbsatz zum Sicherheitsbauteil. Keine neue Zeile in Abschnitt 2.2, kein Prüfauftrag im Dossier |
| E38 | Reihenfolge: Master zuerst, Deck und Verifier im selben Zug |
| E39 | Alle fünf Verifier-Regeln V1 bis V5 werden ergänzt, jede mit Gegenprobe |

## Korrekturen im Deck

### K1 Fristen der Meldepflicht (Befund 1, inhaltlich)

Stelle: `build_meldepflichten`, Folie 21 (5.3), Einleitungstext im linken Textbereich, Zeilen 827 bis 830

Heute:
> Art. 14 ist seit 2026-09-11 in Kraft: 24 h, 72 h, 14 Tage, 1 Monat. Die Fristen laufen ab
> Bereitstellung auf dem Unionsmarkt, der Prototyp ohne Weitergabe löst keine Meldepflicht aus.

Neu:
> Art. 14 ist seit 2026-09-11 in Kraft: 24 h, 72 h, 14 Tage, 1 Monat. Die Fristen laufen ab Kenntnis
> der Schwachstelle oder des Vorfalls. Die Pflicht greift erst mit der Bereitstellung auf dem
> Unionsmarkt; der Prototyp ohne Weitergabe löst keine Meldepflicht aus.

Wirkung: Der Text wächst um etwa 90 Zeichen, also ungefähr eine Zeile. Sichtprüfung der Folie 21
erforderlich, weil dort zwei Textbereiche nebeneinander liegen.

### K2 Prototypenfall mit der Ausnahme für Sicherheitsbauteile (Befund 2)

Stelle: `build_prototypen`, Folie 9 (2.3), Tabellenzeile, Zeilen 600 bis 602

Heute: Fall "Alpha, Beta oder Release Candidate zum Testen", CRA-Status "Befristet zulässig, nur zum
Testen, mit Hinweis und Risikobewertung", Grundlage "Art. 4(3), Recital 37"

Neu: CRA-Status "Befristet zulässig, nur zum Testen, mit Hinweis und Risikobewertung; die Erleichterung
gilt nicht für Sicherheitsbauteile nach anderem Unionsrecht", Grundlage "Art. 4(3) und 4(4), Recital 37"

### K3 Kennzeichnung der Passwortanforderung (Befund 4)

Stelle: `build_pflichten_1_10`, Folie 19 (5.1), Zeile 1, Spalte CRA-Referenz

Heute: "Annex I, Teil I, (2)(b) und (d); EN 303 645 5.1-1 (technischer Maßstab)"

Neu: "Annex I, Teil I, (2)(b), Voreinstellung sicher ab Werk, und (2)(d); kein wörtliches Passwortverbot
in der Verordnung; EN 303 645 5.1-1 (technischer Maßstab)"

Alternative, falls die Zelle zu lang wird: "Annex I, Teil I, (2)(b) und (2)(d); EN 303 645 5.1-1
(technischer Maßstab), die Verordnung selbst nennt kein Passwortverbot"

### K4 Belegstellen der Pflichten 16 und 20 (Befund 4)

Stelle: `build_pflichten_11_22`, Folie 20 (5.2)

| Pflicht | Heute | Neu |
|---|---|---|
| 16 | Art. 13(2) | Art. 13(2) und (3), Aufnahme nach Art. 13(4) |
| 20 | Art. 13(8) | Art. 13(8) und (19), Annex II Nr. 7 |

Alternative, falls die Referenzspalte zu eng wird: nur "Art. 13(2) und (3)" beziehungsweise
"Art. 13(8) und (19)", die Nebenangaben stehen dann im Master.

### K5 Zehnjahresfristen im Fazit (Befund 4)

Stelle: `build_was_folgt`, Folie 27 (7.1), Zeile "Fristen einplanen", Zeile 731

Heute: "Supportzeitraum von mindestens fünf Jahren mit Enddatum in der Nutzerinformation nach
Annex II; Dokumentation und Erklärung zehn Jahre verfügbar halten"

Neu: "Supportzeitraum von mindestens fünf Jahren mit Enddatum in der Nutzerinformation nach
Annex II; Dokumentation, Erklärung und Nutzerinformation zehn Jahre oder für den Supportzeitraum
bereithalten und Sicherheitsupdates ebenso lange verfügbar halten"

Wirkung: etwa eine Zeile länger, Folie 27 hat Platz.

### K6 Hinweis auf den Charakter der Pflichtenliste (Befund 3)

Stelle: `build_pflichten_1_10`, Folie 19 (5.1), neues graues Textfeld unter der Tabelle
(`Inches(LEFT)` und etwa `Inches(5.75)`, Schrift 9 pt), Vorbild ist der Hinweis auf Folie 14

Textvorschlag:
> Projektliste: die Nummern 1 bis 26 sind die Zählung dieses Dossiers. Weitere Herstellerpflichten
> stehen in Art. 13(6) bis (23) sowie Art. 14(6) und (8).

Damit bleibt die Liste unverändert und keine Nummer verschiebt sich. Die Alternative, die Liste zu
erweitern, ist unter Offene Fragen Q2 beschrieben.

## Korrekturen im Master

Der Master ist die einzige Statusquelle, deshalb werden die inhaltlichen Punkte dort zuerst
nachgezogen und danach im Deck gespiegelt.

| Nr. | Stelle | Änderung |
|---|---|---|
| M1 | Abschnitt 6.4, vor dem operativen Stand | Satz zur Anknüpfung der Fristen: "Die Fristen knüpfen an die Kenntnis des Herstellers an: 24 h und 72 h ab Kenntnis (Art. 14(2)(a) und (b), Art. 14(4)(a) und (b)), 14 Tage ab Verfügbarkeit einer Korrektur- oder Milderungsmaßnahme (Art. 14(2)(c)), 1 Monat ab der 72-h-Meldung (Art. 14(4)(c))." |
| M2 | Abschnitt 6.3, Pflicht 16 | Referenz auf "Art. 13(2) und (3), Aufnahme nach Art. 13(4)" |
| M3 | Abschnitt 6.3, Pflicht 20 | Referenz auf "Art. 13(8) und (19), Angabe nach Annex II Nr. 7" |
| M4 | Abschnitt 6.1, Pflicht 1 | Referenz um "kein wörtliches Passwortverbot in der Verordnung, das Wort steht nur in Annex III als Produktkategorie" ergänzen |
| M5 | Abschnitt 6, Einleitung | Absatz zur Vollständigkeit: Liste deckt Annex I Teil I und II sowie die formellen Pflichten ab, die bis zur Bereitstellung relevant sind, ist aber keine abschließende Aufzählung; die weiteren Pflichten nach Art. 13(6) bis (23), Art. 14(6) und (8) sind dort benannt |
| M6 | Abschnitt 2.1, Zeile Alpha und Beta | Der Vorbehalt nach Art. 4(4) steht bereits. Zu ergänzen ist nur der Bezug zu VentCon2, wenn der Nutzer die Einordnung als Sicherheitsbauteil klären will (siehe Q3) |
| M7 | Abschnitt 12, Revisionshistorie | Ein Eintrag mit Datum 2026-10-01 zu M1 bis M6 |

## Verifier-Regeln

Die Korrekturen sollen gegen Rückfall abgesichert werden. Jede Regel bekommt eine Gegenprobe mit
einer künstlich eingebauten Verletzung, damit sie nicht ins Leere greift.

| Nr. | Regel | Prüfung |
|---|---|---|
| V1 | Fristen nur ab Kenntnis | verbotene Wendung "Fristen laufen ab Bereitstellung", gefordert ist "ab Kenntnis" auf der Folie mit Art. 14 |
| V2 | Zitierte Absätze existieren | Für jede Angabe "Art. N(x)" muss in der Normquelle ein Absatz x des Artikels N vorhanden sein, gelesen aus den Marken "### x." der Extraktion |
| V3 | Annex-Buchstaben existieren | Jede Angabe "Teil I, (2)(x)" muss in Annex I Teil I vorkommen, jede Angabe "Teil II, Nr. n" in Annex I Teil II |
| V4 | Projektlisten-Hinweis | Wenn das Deck "Pflichten 1 bis 26" enthält, muss es auch "Projektliste" enthalten |
| V5 | Art. 4(3) nur mit Art. 4(4) | Wenn "Art. 4(3)" im Deck steht, muss auch "Art. 4(4)" vorkommen |

V2 und V3 gelten für Deck und Master und ersetzen die bisherige grobe Prüfung, die nur die Existenz
der Artikelnummer und die Grenzen von Teil II prüft.

## Prüfkette und Abnahme

1. Master nachziehen, danach `sec/analyse` und Abschnitt 6 gegen die Normquelle lesen.
2. Generator ändern, Deck bauen, `verify_ventrex_deck.py` mit den neuen Regeln, keine Fehler und
   keine Hinweise.
3. `tools/audit_plan_umsetzung.py` bleibt grün, weil keine Nummer und kein Titel wechselt.
4. Gegenproben für V1 bis V5 mit künstlich eingebauten Verletzungen, jede Regel muss anschlagen.
5. Sichtprüfung der Folien 9, 19, 20, 21 und 27 per PNG-Export, besonders auf Überlauf in den
   Zellen der Pflichten 1, 16 und 20 und im Einleitungstext der Meldepflichten.
6. Präsentationsplan um einen Nachtrag und die Entscheidungen E35 bis E39 ergänzen, Analyseprotokoll
   um ein Ergebnisfeld, Repo-Gedächtnis nachziehen.

## Entschiedene Fragen

| Nr. | Frage | Entscheidung |
|---|---|---|
| Q1 | Umfang der Korrektur | Alle vier Befunde und die Verifier-Regeln (E35) |
| Q2 | Umgang mit Befund 3 | Hinweistext auf Folie 19, Liste unverändert (E36). Die Varianten mit einer neuen Folie oder einer erweiterten Liste sind damit verworfen |
| Q3 | Einordnung als Sicherheitsbauteil | Nur die Grundlage der Zeile auf Art. 4(3) und 4(4) ergänzen (E37). Ob VentCon2 Sicherheitsbauteil ist, bleibt offen und wird hier nicht festgestellt |
| Q4 | Reihenfolge | Master zuerst, Deck und Verifier im selben Zug (E38) |
| Q5 | Verifier-Regeln | Alle fünf Vorschläge V1 bis V5 (E39) |

## Offen geblieben

- Die Einordnung von VentCon2 als Sicherheitsbauteil nach anderem Unionsrecht ist nicht festgestellt. Der Vorbehalt nach Art. 4(4) steht ab der Korrektur sichtbar im Deck, die Feststellung selbst bleibt ein eigener Vorgang.
- Die externen Rechtsakte wurden nicht gegen das Amtsblatt geprüft (siehe Scope-Grenzen).

## Scope-Grenzen

- Keine Änderung an Nummerierung, Reihenfolge und Titeln, deshalb bleibt das Inhaltsverzeichnis stabil.
- Keine Änderung an `src/`, `LaTeX/` und an der Normextraktion unter `sec/sources/`.
- Die externen Rechtsakte (Delegierte Verordnungen, Durchführungsbeschlüsse, Leitlinien, M/606) werden
  in diesem Plan nicht gegen das Amtsblatt geprüft. Wenn das gewünscht ist, wird es ein eigener
  Prüfschritt mit Belegen aus dem Amtsblatt.
