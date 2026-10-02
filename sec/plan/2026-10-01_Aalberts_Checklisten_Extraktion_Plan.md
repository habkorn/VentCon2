# Plan: Aalberts-Vorlagen auswerten und je Dokument eine Markdown-Datei erzeugen

Datum: 2026-10-01
Gegenstand: drei Word-Vorlagen in `sec/AalbertsChecklists`
Status: Plan, entschieden am 2026-10-01, umgesetzt und verifiziert am 2026-10-01

## Ergebnis der Umsetzung

| Ausgabe | Umfang laut Prüfsummenbericht |
|---|---|
| `sec/AalbertsChecklists/CRA_Compliance_Addendum_Supplier_Agreement.md` | 17 Absätze, 0 Tabellen, Klauseln 1 bis 8, SHA256 `5e29a13d…` |
| `sec/AalbertsChecklists/CRA_Product_Maturity_Checklist.md` | 40 Absätze, 2 Tabellen (26x3 und 12x2), nummerierte Anhangsliste 1 bis 10, SHA256 `fab3c5d4…` |
| `sec/AalbertsChecklists/CRA_Supplier_Due_Diligence_Questionnaire.md` | 27 Absätze, 7 Tabellen, Anhang mit dem Kommentar von Niels Ketelaar, SHA256 im Kopfblock |

Werkzeuge: `tools/docx_extract.py` erzeugt die Extrakte, `tools/verify_docx_extract.py` prüft sie
unabhängig vom Erzeuger. Aufruf jeweils
`"C:\venvs\VentCon2\Scripts\python.exe" tools/<Werkzeug>.py` ohne Argument für alle drei Dateien.
Der Verifier meldet 27 Prüfungen, alle bestanden: Textdeckung je Körperabsatz, Texte aus Kopf-,
Fuß- und Kommentarteil, Tabellenform, Absatzzahlen gegen den Kopfblock, Prüfsumme, fortlaufende
Klauselnummern.

Drei Punkte, die beim Bau der Werkzeuge aufgefallen sind und die die Werkzeuge jetzt abfangen:

1. Die Kopfzeile "Aalberts - internal" liegt in zwei Textboxen desselben Absatzes. Wer die Absätze
direkt ausliest, bekommt den Text doppelt. Der Extraktor liest je Textbox, der Verifier ebenso.
2. Die Checkliste enthält weiche Zeilenumbrüche (`w:br`) innerhalb eines Absatzes, zum Beispiel in
der Zeile "Product: … Owner: … Date: …". Sie werden als `<br>` übernommen, sonst verlieren die
Felder ihre Zeilenstruktur.
3. Der Titel der Checkliste ist im Original in Einzelruns zerlegt ("C", "YBER", "RA", ")"). Runs
werden deshalb je Absatz zusammengefügt, sonst entstehen verstümmelte Zeilen.

## Entscheidungen des Nutzers (2026-10-01)

| Nr. | Entscheidung |
|---|---|
| E43 | Ablage neben den Worddateien in `sec/AalbertsChecklists`, Dateiname aus dem Dokumenttitel |
| E44 | Sprache: nur Englisch. Der Extrakt bleibt durchgängig englisch, auch der Kopfblock und die Hinweise. Keine deutsche Zusammenfassung, keine Übersetzung |
| E45 | Klauselnummern des Addendums werden rekonstruiert und als `1.` bis `8.` ausgegeben |
| E46 | Der Kommentar im Fragebogen wird als Anhang aufgenommen |
| E47 | Es werden Extraktor und unabhängiger Verifier gebaut |
| E48 | Anschluss: nur extrahieren. Kein Eintrag in Master oder Deck, kein Abgleich gegen die CRA-Pflichten in diesem Schritt |

## Auftrag

Die drei Worddateien analysieren, ihre Inhalte vollständig extrahieren und je Dokument eine
Markdown-Datei erzeugen. Danach sollen die Inhalte im Dossier nutzbar sein (Register, Quellen,
Abgleich gegen die CRA-Pflichten).

## Analyse der Ausgangsdateien

| Datei | Autor | erstellt / geändert | Umfang | Besonderheit |
|---|---|---|---|---|
| CRA Compliance Addendum (Supplier Agreement) - template Sep 2026 .docx | Mattijs Planken | 2026-09-07 / 2026-09-15 | 26 Absätze, 0 Tabellen, 0 Bilder | Klauseln als nummerierte Liste (9 Absätze mit Nummerierung), Titel ohne Überschriftenformat, Achtung: Leerzeichen vor `.docx` im Dateinamen |
| CRA product maturity checklist - Sep 2026.docx | Un-named | 2026-09-15 | 59 Absätze, 2 Tabellen (26x3 und 12x2), 0 Bilder | Abschnittstitel im Titelformat statt Überschriftenformat, Titel im Original in Einzelruns zerlegt ("C | YBER | RA | )") |
| CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx | Un-named | 2026-09-07 / 2026-09-15 | 43 Absätze, 7 Tabellen (2 Spalten, Frage und Antwortfeld), 0 Bilder | Überschriften 1 und 2 vorhanden, ein Kommentar im Dokument: "Confusing to me. Should it be LIST the type of security testing or describe the testing process?" |

Gemeinsame Merkmale: englischer Text, je ein Firmenlogo (`word/media/image1.jpg`, rund 143 KB),
Kopfzeile "Aalberts - internal" in Textboxen (nicht über die normalen Absätze erreichbar), Fußzeile
mit Seitenzahlen, keine Fußnoten- oder Endnotentexte, Stylesheet und Nummerierungsdefinitionen
umfangreich. Die drei Dateien sind mit rund 190 KB pro Stück fast gleich groß, der Unterschied liegt
im Inhalt von `word/document.xml` (33 KB, 132 KB, 96 KB).

Werkzeug: `python-docx` 1.2.0 ist in `C:\venvs\VentCon2` installiert.

## Zielbild

| Ausgabe | Inhalt |
|---|---|
| `sec/AalbertsChecklists/CRA_Compliance_Addendum_Supplier_Agreement.md` | Kopfblock, Titel, Vertragsparteien-Absatz, acht Klauseln mit rekonstruierter Nummer, Unterschriftenblock |
| `sec/AalbertsChecklists/CRA_Product_Maturity_Checklist.md` | Kopfblock, Anleitung, zwei Tabellen (26x3 und 12x2) als Markdown-Tabellen |
| `sec/AalbertsChecklists/CRA_Supplier_Due_Diligence_Questionnaire.md` | Kopfblock, Zweck, sieben Abschnitte mit je einer Frage-Antwort-Tabelle, Sign-off, Anhang mit dem Kommentar |

Jede Markdown-Datei beginnt mit einem Kopfblock: Quelldatei, Autor, Erstellt- und Änderungsdatum,
SHA256 der Quelldatei, Extraktionsdatum, Werkzeug, Umfang (Absätze, Tabellen, Wörter), Hinweis auf
nicht übernommene Bestandteile (Firmenlogo, Kopf- und Fußzeile).

## Werkzeug

Neu: `tools/docx_extract.py`, Aufruf
`"C:\venvs\VentCon2\Scripts\python.exe" tools/docx_extract.py` (ohne Argument alle drei Dateien).

| Aufgabe | Umsetzung |
|---|---|
| Absätze | Runs je Absatz zusammenfügen, damit zerlegte Titel wieder lesbar sind |
| Formate | `Title` und `Heading 1/2` auf `#`, `##`, `###` abbilden, `List Paragraph` als Listeneintrag, `Normal` als Absatz |
| Nummerierung | `numId` und `ilvl` aus `word/numbering.xml` auflösen und die Zähler fortlaufend rekonstruieren, damit Klauselnummern erhalten bleiben |
| Tabellen | Markdown-Tabellen, mehrzeilige Zellen mit `<br>` verbinden, Kopfzeile aus der ersten Zeile, Spaltenbreiten aus der Anzahl der Spalten |
| Kopf- und Fußzeile | Text aus `w:txbxContent` der Kopf- und Fußzeilenteile lesen, als Hinweiszeile im Kopfblock, Seitenzahlenfelder nur als Vermerk |
| Kommentare | `word/comments.xml` auslesen und als Anhang mit Fundstelle ausgeben |
| Bilder | nicht übernehmen, im Kopfblock zählen und benennen |
| Fußnoten und Endnoten | prüfen, ob Text vorhanden ist, sonst als leer vermerken |

## Verifikation

Neu: `tools/verify_docx_extract.py`, unabhängig vom Erzeuger. Es liest die Worddatei roh aus und
vergleicht sie mit der erzeugten Markdown-Datei:

| Prüfung | Kriterium |
|---|---|
| Textdeckung | jede nicht leere Zeichenkette aus `w:t` in `document.xml`, den Kopf- und Fußzeilenboxen und den Kommentaren kommt im Markdown vor |
| Kein Zusatztext | Absätze außerhalb der erkannten Struktur werden gemeldet |
| Umfang | Anzahl Absätze, Tabellen und Zellen je Tabelle stimmen mit dem Kopfblock überein |
| Nummerierung | die rekonstruierten Klauselnummern sind fortlaufend und vollständig |
| Sichtprüfung | die drei Markdown-Dateien werden gelesen und die Tabellen auf Vollständigkeit geprüft |

## Offene Fragen

| Nr. | Frage | Entscheidung |
|---|---|---|
| Q1 | Ort und Namen | neben den Worddateien, Namen nach Dokumenttitel (E43) |
| Q2 | Sprache | nur Englisch, also kein deutscher Kopfblock und keine Zusammenfassung (E44) |
| Q3 | Klauselnummern | rekonstruieren und ausgeben (E45) |
| Q4 | Kommentar | als Anhang (E46) |
| Q5 | Prüfwerkzeuge | Extraktor und unabhängiger Verifier (E47) |
| Q6 | Anschluss | nur extrahieren (E48) |
|---|---|---|
| Q1 | Ort und Namen der Markdown-Dateien | a) neben den Worddateien in `sec/AalbertsChecklists`, Namen nach Dokumenttitel, b) in `sec/AalbertsChecklists/md`, c) mit Datumspräfix nach Projektregel, d) in einem neuen Ordner `sec/references` |
| Q2 | Sprache | a) englischer Originaltext wortgetreu, deutscher Kopfblock und deutsche Hinweise, b) zusätzlich eine deutsche Zusammenfassung je Dokument, c) vollständige Übersetzung |
| Q3 | Klauselnummern des Addendums | a) rekonstruieren und als `1.` bis `8.` ausgeben, b) als Aufzählungspunkte ohne Nummern |
| Q4 | Kommentar im Fragebogen | a) als Anhang aufnehmen, b) weglassen, c) als Hinweis direkt bei der betroffenen Frage |
| Q5 | Zusätzliche Prüfwerkzeuge | a) Extrakter und unabhängigen Verifier bauen, b) nur den Extrakter |
| Q6 | Anschluss im Dossier | a) nur extrahieren, b) zusätzlich im Master registrieren (Dokumentenregister und Quellen), c) zusätzlich die Anforderungen der Vorlagen gegen die CRA-Pflichten abgleichen (Lückenliste für VENTREX als Lieferant und als Hersteller), d) zusätzlich im Deck erwähnen |

## Scope-Grenzen

- Die Worddateien bleiben unverändert, es wird nichts konvertiert oder umbenannt.
- Keine Übersetzung und keine inhaltliche Bewertung ohne Freigabe.
- Kein Eingriff in Deck, Master oder Normextraktion, solange Q6 nicht entschieden ist.
- Die Markdown-Dateien sind Extrakt, nicht Quelle: bei Änderungen der Worddatei muss neu erzeugt
  und neu verifiziert werden, die Prüfsumme im Kopfblock macht das sichtbar.

## Nachtrag vom 2026-10-01

Der Ordner wurde nach dem Ausfüllen umstrukturiert: Vorlagen in `sec/AalbertsChecklists/templates`,
Extrakte in `sec/AalbertsChecklists/extracts`, befüllte Wordkopien in `sec/AalbertsChecklists/filled`
(Entscheidungen E57 bis E60 in `sec/plan/2026-10-01_Aalberts_Ordnerstruktur_Plan.md`). Die Pfade im
Abschnitt "Ergebnis der Umsetzung" geben den Stand vor dieser Umstrukturierung wieder, die Inhalte
sind unverändert.
