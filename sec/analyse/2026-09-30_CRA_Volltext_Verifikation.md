# Verifikationsprotokoll: CRA-Volltext als zitierfähiges Markdown

Rolle: Verifikationsprotokoll (Nachweis, keine Bewertung)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: keine (Abnahme der Normquellen-Erschließung)
Bezug: Regulation (EU) 2024/2847 (Cyber Resilience Act, CRA)
Stand: 2026-09-30
Prüfer: GitHub Copilot (AP7 des Plans vom 2026-09-30)
Auftrag: Abnahme des Extrakts gegen die sieben Gates aus Abschnitt 5 des Plans.

---

## 1. Prüfgegenstand und Prüfmittel

| Punkt | Wert |
|---|---|
| Quelle | `sources/EU_CRA_Regulation_2024_2847_EN.pdf`, 81 Seiten |
| SHA-256 der Quelle | `e3ecaabddf6e321fa04097d15dfcccc7309d0fe5896240ab7625a27d74ab1b0a` |
| Erzeuger | `tools/cra_extract.py`, PyMuPDF 1.28.2, Interpreter `C:\venvs\VentCon2\Scripts\python.exe` |
| Erzeugnis | 19 Dateien in `sources/EU_CRA_Regulation_2024_2847_EN/` |
| Prüfsumme der 19 Dateien | `A2567900678D4CF6A0ADBC168CDC4E55A0406E1CEDE7B071CB414845135137F6` (SHA-256 über die nach Name sortierte Liste aus Name, Größe und Einzel-Hash) |
| Prüfer | `tools/verify_cra_extract.py`, 21 Prüfungen in drei Gruppen |
| Unabhängigkeit | Der Prüfer liest alle Sollwerte direkt aus der PDF. Er verwendet keinen Code des Erzeugers und kennt dessen Zwischenergebnisse nicht. |
| Wiederholbarkeit | `--write` erzeugt das Erzeugnis aus der PDF neu, `verify_cra_extract.py` prüft es danach erneut. |

Der Prüfer bestimmt die Sollwerte selbst: Lesezeilen werden aus der PDF aufgebaut,
Erwägungsgrund-Marken, Fußnoten und der Satzspiegel werden unabhängig vom Erzeuger
klassifiziert. Erwägungsgrund-Marken und Fußnotenmarken sind beide 8,5 pt groß und nur
über die Lage zu unterscheiden, Erwägungsgrund-Marken stehen links neben ihrem Absatz,
Fußnoten unterhalb der letzten Zeile des Satzspiegels.

## 2. Ergebnis der sieben Gates

| Gate | Prüfung | Ergebnis | Messwert |
|---|---|---|---|
| G1 | Dateien | bestanden | 19 von 19, keine fehlend, keine zusätzlich |
| G1 | Artikel 1 bis 71 | bestanden | 71 von 71, genau einmal, keine Lücke |
| G1 | Anhänge I bis VIII | bestanden | I, II, III, IV, V, VI, VII, VIII |
| G1 | Erwägungsgründe | bestanden | 130 von 130, Nummern 1 bis 130 lückenlos |
| G2 | Massenbilanz | bestanden | Messbasis 295 812 Zeichen, Extrakt 295 812 Zeichen, Abweichung +0,000 Prozent |
| G2 | Randzeilen-Audit | bestanden | 328 entfernte Zeilen, alle nach Amtsblattmuster, 5 463 Zeichen |
| G3 | Seitengrenzen | bestanden | 81 von 81 Seiten zeichengleich |
| G3 | Seitenmarken | bestanden | 81 von 81 Seiten markiert |
| G4 | Anker | bestanden | 38 Verweise, 0 ohne Ziel |
| G4 | Fundstellen | **Befund** | 7 CRA-Fundstellen ohne Beleg, siehe Abschnitt 5 |
| G5 | Kodierung | bestanden | UTF-8 ohne BOM, LF, kein U+FFFD |
| G5 | Überschriften | bestanden | keine leere Überschrift |
| G5 | Nicht-ASCII | bestanden | Quelldatei und Extrakt identisch: Halbgeviertstrich 1, Auslassungspunkte 3, a grave 1 |
| G6 | Wortlauttreue | bestanden | vollständige Wortfolge geprüft plus sechs Seiten Bildsicht, siehe Abschnitt 6 |
| G7 | Geviertstrich | bestanden | kein U+2014 in 19 Dateien |
| G7 | Dateinamen | bestanden | alle Namen nach Konvention |
| G7 | venv | bestanden | kein venv im Projektordner |

Zur Massenbilanz: Die Zahl 295 812 ist die zeichenweise gezählte Fassung ohne jeden Weißraum.
Mit Weißraum zählt der Erzeuger 349 735 Zeichen in 4 379 Zeilen der Textebene, entfernt davon
328 Zeilen mit 5 709 Zeichen, und stellt 348 060 Zeichen Extrakt gegen 348 076 Zeichen Messbasis,
also -0,005 Prozent. Beide Messungen kommen zum selben Befund: es fehlt kein Normtext. Die
Abweichung gegen die rohe Textebene erklärt sich vollständig durch den entfernten Kolumnentitel
und die Fußzeile des Amtsblatts.

Sechs Seiten sind in zwei Dateien markiert (28, 35, 46, 51, 63, 65). Das ist beabsichtigt: ein
Kapitel beginnt mitten auf der Seite, und die Marke steht in jeder Datei, in der die Seite
vorkommt. Die Seitenprüfung ordnet die Inhalte über die Marke zu und ist davon unberührt.

## 3. Zusatzprüfungen

| Nr | Prüfung | Ergebnis | Messwert |
|---|---|---|---|
| Z1 | Absatzstruktur | 1 Abweichung | siehe M5 |
| Z2 | Fußnoten | bestanden | Quelldatei 41 Marken, Extrakt 41 Einträge, Nummern deckungsgleich |
| Z3 | Größen im Index | bestanden | 18 Zeilen, alle Größen stimmen mit der Datei überein |

Z1 prüft die Absatzstruktur nicht über die Zeichen, sondern über die Typografie der Quelle:
Zeilen innerhalb eines Absatzes haben 10,4 bis 10,7 pt Abstand, ein neuer Absatz beginnt bei
15,7 pt oder mehr. Der Prüfer sucht Stellen, an denen der Extrakt trennt, obwohl die Quelle
denselben Absatz fortsetzt.

## 4. Gefundene Mängel und ihr Stand

| Nr | Befund | Wirkung | Stand |
|---|---|---|---|
| M1 | Der gedruckte Titel `TRANSITIONAL AND FINAL PROVISIONS` (Seite 65) wurde als Fußnotenzeile erkannt und an die letzte Fußnote der Seite 64 angehängt. | 30 Zeichen lagen auf der falschen Seite, Fußnote 37 war verfälscht. | behoben am 2026-09-30, neu erzeugt und nachgeprüft |
| M2 | Das Erzeugnis war gegenüber dem Erzeuger veraltet: die Dateien waren geschrieben, bevor die Absatzregel fertig war. | 135 Absatztrennungen mitten im Absatz. | behoben durch Neuzeugung aus dem aktuellen Skript |
| M3 | `00_Index.md` nannte für `18_Quick_Reference_VentCon2.md` die Größe 0. | falsche Metadaten im Index. | behoben am 2026-09-30 |
| M4 | In Anhang VIII steht ein Wort in anderer Reihenfolge als in der Quelle: der Extrakt setzt `cyberattacks` vor die Zeichenfolge aus x, die Quelle danach. | rein kosmetisch, kein Zeichenverlust, Seite 76 bleibt zeichengleich. | offen, nicht entschieden |
| M5 | Artikel 13 Absatz 17 wird in drei Absätze getrennt, obwohl die Quelle sie mit 10,5 pt Zeilenabstand als einen Absatz setzt. | rein kosmetisch, Wortlaut unverändert. | offen |
| M6 | Die Fußnote mit Sternchen auf Seite 65 steht als gewöhnlicher Absatz, nicht als Blockzitat. | rein kosmetisch, Inhalt vorhanden und richtig zugeordnet. | offen |

M4, M5 und M6 berühren keinen Wortlaut. Die Seitenprüfung ist für alle 81 Seiten
zeichengleich, und die Massenbilanz ist exakt.

## 5. Befund zur Zitatfähigkeit (Gate G4), nur gemeldet

Das Gate prüft die Extraktion gegen das Dossier, nicht das Dossier gegen die Verordnung.
Es wurde nichts korrigiert.

**Befund A: sieben Fundstellen der Checkliste sind gegen die Verordnung nicht auflösbar.**
Betroffen sind `Annex I, Teil I` mit den Nummern 1.2, 1.3, 1.4, 1.7, 1.8, 1.9 und 1.10. Anhang I
Teil I trägt in der Verordnung nur die Punkte (1) und (2); Punkt (2) gliedert sich in die
Buchstaben (a) bis (m). Ein Nummernschema 1.1 bis 1.10 kommt im Normtext nicht vor. Belege:

1. Die Bildsichtprüfung der Seite 68 zeigt `(1)`, `(2)` und darunter `(a)` bis `(m)`.
2. Die Verordnung verweist in Artikel 13 Absatz 3 selbst auf `Part I, point (2)` und
   `Part I, point (1)` und stützt damit die Nummerierung (1) und (2).
3. Die drei Fundstellen mit den Buchstaben 2(b), 2(d) und 2(e) sind belegt. Sie zeigen, dass
   die Checkliste die Sache richtig meint, aber die Nummer falsch schreibt.

**Befund B: die statische Liste in Abschnitt 1 des Plans ist unvollständig.** Die Checkliste
nennt 43 verschiedene Referenzen, der Plan 22. Zusätzlich verwendet die Checkliste unter
anderem Art. 13(2), Art. 13(8), Art. 14(2)(a) bis (c), Art. 14(4)(a) bis (c), Art. 14(7),
Art. 14(10), Art. 15, Art. 28, Art. 64 und Art. 69(2) sowie Art. 69(3).

**Befund C: die Referenzen Art. 3(3)(d), Art. 3(3)(e), Art. 3(3)(f), Art. 3(21) und Art. 3(22)
sind als RED-Bezug geführt.** Für Art. 3(21) und Art. 3(22) ist diese Einordnung zu prüfen,
weil es Begriffsbestimmungen der Verordnung sind.

Die Befundliste liegt damit vor, wie es Abnahmekriterium 5 verlangt. Eine Änderung an Zitaten
im Dossier ist eine Entscheidung des Nutzers.

## 6. Bildsichtprüfung (Gate G6)

Sechs Seiten wurden mit 110 dpi gerendert und gegen den Extrakt verglichen.

| Seite | Inhalt der Seite | Ergebnis |
|---|---|---|
| 1 | Titel, Erwägungsformel mit Verweisen (1) und (2), Erwägungsgründe 1 bis 3, Fußnoten 1 bis 4 | deckungsgleich |
| 6 | Erwägungsgründe 23 bis 26, Fußnoten 7 und 10 | deckungsgleich |
| 35 | Titel Kapitel II, Artikel 13 Absätze 1 bis 5 | deckungsgleich |
| 65 | Artikel 64 Absätze 5 bis 10, Artikel 65, Kapitel VIII mit Titel, Artikel 66, Fußnote mit Sternchen | deckungsgleich |
| 68 | Anhang I Teil I und Teil II | deckungsgleich |
| 81 | Schluss des Anhangs VIII mit Punkt 8 | deckungsgleich |

Einzige bewusste Ausnahme: die Kopfdaten des Amtsblatts auf der Titelseite (`L series`,
`Official Journal`, `2024/2847`, `20.11.2024`) stehen nicht im Extrakt. Sie sind Amtsblatt und
kein Normtext; der Index nennt Quelle, Datum und Abrufweg. Die Bildprüfung der Seite 1 bestätigt,
dass sonst nichts fehlt.

## 7. Offene Punkte

1. Entscheidung zu Befund A: die sieben Nummern 1.2 bis 1.10 in der Checkliste entweder auf die
   tatsächlichen Buchstaben von Anhang I Teil I Punkt (2) umstellen oder als eigene Zählung
   kennzeichnen. Ohne Entscheidung bleibt die Fundstelle unbelegt.
2. M4, M5 und M6 sind kosmetisch und können offen bleiben oder nachgezogen werden.
3. Der Auftrag in Abschnitt 9 des Plans nennt das Gedächtnis des Agenten Frankie. Diese Datei
   liegt außerhalb des Projektordners und wurde nicht angefasst.

## 8. Reproduktion

```
C:\venvs\VentCon2\Scripts\python.exe tools\cra_extract.py --write
C:\venvs\VentCon2\Scripts\python.exe tools\cra_extract.py --verify
C:\venvs\VentCon2\Scripts\python.exe tools\verify_cra_extract.py
```

Rückgabewert des Verifikationsskripts: 0, wenn alle Prüfungen bestanden sind. Der Lauf vom
2026-09-30 ergibt 1, weil die sieben Fundstellen aus Abschnitt 5 offen sind.
