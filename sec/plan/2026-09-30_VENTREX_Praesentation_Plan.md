# Plan: CRA-Praesentation auf die VENTREX-Vorlage umstellen

Datum: 2026-09-30
Rolle: Umsetzungsplan (Detaildokument, kein CRA-Bezug)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: keine (Dokumentenaufbereitung)
Ausfuehrungsstand: umgesetzt und verifiziert am 2026-09-30
Ergebnis: 20 Folien, 6,84 MB. Struktur- und Inhaltspruefung ohne Befund (26 Pflicht-Nummern,
20 Massnahmen-IDs, Fusszeile und Datum 15 von 15, Foliennummernfeld 15 von 15, nur Markenschriften
und Markenfarben, keine Vorlagenreste). Sichtpruefung per PowerPoint-PNG-Export durchgefuehrt:
Cover, Divider, Tabellenfolien, zweispaltige Folie, Dunkelfolie und Abschlussfolie geprueft.
Eine Korrektur war noetig: die beiden Spalten von CONTENT B_V1 liegen nur 11 EMU auseinander, deshalb
sortiert der Generator die Textplatzhalter mit einer Toleranz von 0,05 Zoll nach Lage. Ohne die
Toleranz waren die Spalten von Folie 5 und 18 vertauscht.

Nachtrag 2026-09-30, Trennung von RED und CRA im Master: Folien 3, 4, 5, 15, 16 und 19 angepasst,
Folienzahl bleibt 20. Der Verifier prueft jetzt 21 Massnahmen-IDs und zwei zusaetzliche Regeln:
eine RED-Aussage braucht den Gueltigkeitsvermerk bis 2027-12-11, und EN 303 645 ist als technischer
Massstab zu kennzeichnen. Die Sichtpruefung wurde fuer die geaenderten Folien wiederholt.

## Auftrag

Die Praesentation aus `sec/CRA_Compliance_Checklist.md` wird aus der Firmenvorlage
`sec/VENTREX PPT Vorlage 2026.pptx` erzeugt: Vorlage oeffnen, Beispielfolien entfernen,
Folien ueber die Layouts der Vorlage anlegen, Platzhalter fuellen.

## Entscheidungen (2026-09-30)

| Nr. | Entscheidung |
|---|---|
| E1 | Alte Praesentation ersetzen und umbenennen: neu `sec/2026-09-30_CRA_Compliance_Praesentation.pptx`, Altdatei vom 2026-09-29 entfernen |
| E2 | Alle 28 Beispielfolien der Vorlage entfernen |
| E3 | Fuenf Kapitel-Divider auf den SUBSECTION-Layouts |
| E4 | Tabellen beibehalten, im VENTREX-Look (Kopfzeile #002F87, Theme-Schriften, Statusspalte farbig) |
| E5 | Fusszeile: Name "VentCon2 - EU CRA Compliance", Datum "30.09.2026", Foliennummer als Feld |
| E6 | Inhalt gestrafft, 20 Folien |
| E7 | Verknuepfungsfolie in die Statusfolien integriert (Streichung der separaten Folie) |
| E8 | Sichtpruefung per PowerPoint-COM mit PNG-Export |
| E9 | Trennung RED und CRA im Master am 2026-09-30 in die Folien uebernommen, Folienzahl bleibt 20 |

## Befunde zur Vorlage (Discovery)

- Format 13,333 x 7,5 Zoll, 8,37 MB, 72 Bilder mit zusammen 7,8 MB.
- 7 Master mit 31 Layouts: COVER, SUBSECTION 1 bis 5, CONTENT A_V1 bis F (in Master 2 und 6),
  CONTENTDARK 1 und 2, END-01/02, CLOSING SLIDE.
- Alle Beispielfolien mit CONTENT-Layouts nutzen Master 2, Divider Master 1, Dunkelfolien Master 3,
  Cover Master 0, Abschluss Master 5. Master 6 ist ein Namensduplikat und bleibt ungenutzt.
- Markenschriften Clash Display (Haupt) und Satoshi (Text) aus Master 0, 2 und 6. Die Divider-Layouts
  setzen Clash Display 44 pt direkt im Layout, obwohl Master 1 als Theme Calibri traegt.
- Markenfarben #002F87, #32C09D, #E96D5A, #535863, #857C48, #EBEBEB.
- Platzhalterrollen wechseln je Layout: in CONTENT A_V1 ist ID 14 das Datum, in CONTENT A_V2 ist
  ID 14 der Untertitel. Deshalb Aufloesung ueber `placeholder_format.type`, nie ueber feste IDs.
- Tabellen sind in der Vorlage Handarbeit ausserhalb der Platzhalter, Kopfzeile in #002F87.
- Der Foliennummern-Platzhalter darf nicht mit Text gefuellt werden, sonst geht das Feld
  `<a:fld type="slidenum">` verloren.

## Zielbild: 20 Folien

| Nr. | Layout | Inhalt |
|---|---|---|
| 1 | COVER | Titel, Untertitel, Stand und Quelle |
| 2 | SUBSECTION 1 | Divider Rahmen und Anwendbarkeit |
| 3 | CONTENT A_V1 | Anwendbarkeit und erste Entscheidung, Tabelle mit 4 Zeilen |
| 4 | CONTENT A_V2 | RED und CRA: zwei Regime, ein Uebergang, Vergleich in einer Spalte |
| 5 | CONTENT B_V1 | Produktklassifizierung links, Normenlage und RED-Abgrenzung rechts |
| 6 | CONTENT A_V1 | Dokumentenregister, gruppiert |
| 7 | SUBSECTION 2 | Divider Pflichten der CRA |
| 8 | CONTENT A_V1 | Pflichten 1 bis 10 |
| 9 | CONTENT A_V1 | Pflichten 11 bis 22 |
| 10 | CONTENT A_V2 | Pflichten 23 bis 26, Art. 69(3) und SRP-Stand |
| 11 | SUBSECTION 3 | Divider Verknuepfung und Status |
| 12 | CONTENT A_V1 | Status Phase 1 bis 3 mit Risikobezug |
| 13 | CONTENT A_V1 | Status Phase 4 und 5 |
| 14 | SUBSECTION 4 | Divider Fristen und Sanktionen |
| 15 | CONTENT A_V2 | Zeitplan mit Handlungsdruck, sechs Zeilen inklusive EN-18031-Zitierung |
| 16 | CONTENTDARK 1 | Sanktionsrahmen als dunkle Akzentfolie |
| 17 | SUBSECTION 5 | Divider Quellen und Revision |
| 18 | CONTENT B_V1 | Quellen zweispaltig |
| 19 | CONTENT A_V2 | Revision und naechste Schritte |
| 20 | CLOSING SLIDE | Abschlussbild der Vorlage |

## Umsetzung

| Schritt | Inhalt |
|---|---|
| U1 | Generator `build_ventrex_deck.py`: Vorlage oeffnen, alle Folien ueber `sldIdLst` und `drop_rel` entfernen |
| U2 | Layoutaufloesung je Master, Helfer fuer Titel, Body, Fusszeile, Tabellen |
| U3 | 20 Folien aufbauen, Inhalte aus dem Master, Tabellen in 7 Folien, Listen in 6 Folien |
| U4 | Speichern nach `sec/2026-09-30_CRA_Compliance_Praesentation.pptx`, Groesse berichten |
| U5 | Verifikation mit `verify_ventrex_deck.py` |
| U6 | Sichtpruefung per PowerPoint-COM, Export nach `%TEMP%\cra_deck\*.png` |
| U7 | Altdatei vom 2026-09-29 entfernen, Eintrag in `deletion_log.md` |
| U8 | Nach der RED-CRA-Trennung im Master: Folien 3, 4, 5, 15, 16 und 19 anpassen, Verifier um zwei Regeln erweitern, erneut bauen und sichten |

## Verifikation

| Pruefung | Kriterium |
|---|---|
| Folieninventar | 20 Folien, 1 COVER, 5 SUBSECTION, 1 CLOSING, Rest CONTENT |
| Vorlagenreste | kein "Hier kommt der Folientitel", "Name Praesentation", "20.05.2026", "STEP", "DIAGRAMM", "Segnaposto" |
| Platzhalter | Titel jeder Folie gefuellt, Fusszeile und Datum gesetzt |
| Foliennummer | Feld `<a:fld type="slidenum">` erhalten |
| Schrift | kein Fremd-Typeface, nur Theme-Verweise und Markenschriften |
| Farben | nur Markenpalette |
| Inhalt | 26 Pflicht-Nummern, 21 Massnahmen-IDs, Zeitplan, Sanktionsstufen, Quellen, kein Em-Dash |
| RED-Abgrenzung | keine RED-Aussage ohne Gueltigkeitsvermerk bis 2027-12-11 |
| Normenhierarchie | EN 303 645 nur mit Kennzeichnung als technischer Massstab |
| Paket | Datei erneut oeffenbar, jede `r:embed` mit gueltiger Relationship |
| Vorlage | SHA256 vor und nach dem Lauf identisch |
| Visuell | PNG-Export geprueft: Cover, Divider, Tabellenfolie, Dunkelfolie, Abschluss |

## Scope-Grenzen

- Vorlage und Master bleiben unveraendert.
- Keine Aenderung an `src/` und `LaTeX/`.

## Hinweise

- Unbenutzte Master behalten ihre Bilder, deshalb bleibt die Datei groesser als noetig.
  Gemessener Wert siehe Bericht; Pruning nur auf Zuruf.
- Der Vorgaenger-Generator `build_cra_presentation.py` und der Vorgaenger-Verifier
  `verify_cra_presentation.py` erzeugen das alte Design und sind damit abgeloest.
