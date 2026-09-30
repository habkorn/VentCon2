# Verifikationsprotokoll: CRA-Volltext als zitierfaehiges Markdown

Rolle: Verifikationsprotokoll (Nachweis, keine Bewertung)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: keine (Abnahme der Normquellen-Erschliessung)
Bezug: Regulation (EU) 2024/2847 (Cyber Resilience Act, CRA)
Stand: 2026-09-30
Pruefer: GitHub Copilot (AP7 des Plans vom 2026-09-30)
Auftrag: Abnahme des Extrakts gegen die sieben Gates aus Abschnitt 5 des Plans.

---

## 1. Pruefgegenstand und Pruefmittel

| Punkt | Wert |
|---|---|
| Quelle | `sources/EU_CRA_Regulation_2024_2847_EN.pdf`, 81 Seiten |
| SHA-256 der Quelle | `e3ecaabddf6e321fa04097d15dfcccc7309d0fe5896240ab7625a27d74ab1b0a` |
| Erzeuger | `tools/cra_extract.py`, PyMuPDF 1.28.2, Interpreter `C:\venvs\VentCon2\Scripts\python.exe` |
| Erzeugnis | 19 Dateien in `sources/EU_CRA_Regulation_2024_2847_EN/` |
| Pruefsumme der 19 Dateien | `A2567900678D4CF6A0ADBC168CDC4E55A0406E1CEDE7B071CB414845135137F6` (SHA-256 ueber die nach Name sortierte Liste aus Name, Groesse und Einzel-Hash) |
| Pruefer | `tools/verify_cra_extract.py`, 21 Pruefungen in drei Gruppen |
| Unabhaengigkeit | Der Pruefer liest alle Sollwerte direkt aus der PDF. Er verwendet keinen Code des Erzeugers und kennt dessen Zwischenergebnisse nicht. |
| Wiederholbarkeit | `--write` erzeugt das Erzeugnis aus der PDF neu, `verify_cra_extract.py` prueft es danach erneut. |

Der Pruefer bestimmt die Sollwerte selbst: Lesezeilen werden aus der PDF aufgebaut,
Erwaegungsgrund-Marken, Fussnoten und der Satzspiegel werden unabhaengig vom Erzeuger
klassifiziert. Erwaegungsgrund-Marken und Fussnotenmarken sind beide 8,5 pt gross und nur
ueber die Lage zu unterscheiden, Erwaegungsgrund-Marken stehen links neben ihrem Absatz,
Fussnoten unterhalb der letzten Zeile des Satzspiegels.

## 2. Ergebnis der sieben Gates

| Gate | Pruefung | Ergebnis | Messwert |
|---|---|---|---|
| G1 | Dateien | bestanden | 19 von 19, keine fehlend, keine zusaetzlich |
| G1 | Artikel 1 bis 71 | bestanden | 71 von 71, genau einmal, keine Luecke |
| G1 | Anhaenge I bis VIII | bestanden | I, II, III, IV, V, VI, VII, VIII |
| G1 | Erwaegungsgruende | bestanden | 130 von 130, Nummern 1 bis 130 lueckenlos |
| G2 | Massenbilanz | bestanden | Messbasis 295 812 Zeichen, Extrakt 295 812 Zeichen, Abweichung +0,000 Prozent |
| G2 | Randzeilen-Audit | bestanden | 328 entfernte Zeilen, alle nach Amtsblattmuster, 5 463 Zeichen |
| G3 | Seitengrenzen | bestanden | 81 von 81 Seiten zeichengleich |
| G3 | Seitenmarken | bestanden | 81 von 81 Seiten markiert |
| G4 | Anker | bestanden | 38 Verweise, 0 ohne Ziel |
| G4 | Fundstellen | **Befund** | 7 CRA-Fundstellen ohne Beleg, siehe Abschnitt 5 |
| G5 | Kodierung | bestanden | UTF-8 ohne BOM, LF, kein U+FFFD |
| G5 | Ueberschriften | bestanden | keine leere Ueberschrift |
| G5 | Nicht-ASCII | bestanden | Quelldatei und Extrakt identisch: Halbgeviertstrich 1, Auslassungspunkte 3, a grave 1 |
| G6 | Wortlauttreue | bestanden | vollstaendige Wortfolge geprueft plus sechs Seiten Bildsicht, siehe Abschnitt 6 |
| G7 | Geviertstrich | bestanden | kein U+2014 in 19 Dateien |
| G7 | Dateinamen | bestanden | alle Namen nach Konvention |
| G7 | venv | bestanden | kein venv im Projektordner |

Zur Massenbilanz: Die Zahl 295 812 ist die zeichenweise gezaehlte Fassung ohne jeden Weissraum.
Mit Weissraum zaehlt der Erzeuger 349 735 Zeichen in 4 379 Zeilen der Textebene, entfernt davon
328 Zeilen mit 5 709 Zeichen, und stellt 348 060 Zeichen Extrakt gegen 348 076 Zeichen Messbasis,
also -0,005 Prozent. Beide Messungen kommen zum selben Befund: es fehlt kein Normtext. Die
Abweichung gegen die rohe Textebene erklaert sich vollstaendig durch den entfernten Kolumnentitel
und die Fusszeile des Amtsblatts.

Sechs Seiten sind in zwei Dateien markiert (28, 35, 46, 51, 63, 65). Das ist beabsichtigt: ein
Kapitel beginnt mitten auf der Seite, und die Marke steht in jeder Datei, in der die Seite
vorkommt. Die Seitenpruefung ordnet die Inhalte ueber die Marke zu und ist davon unberuehrt.

## 3. Zusatzpruefungen

| Nr | Pruefung | Ergebnis | Messwert |
|---|---|---|---|
| Z1 | Absatzstruktur | 1 Abweichung | siehe M5 |
| Z2 | Fussnoten | bestanden | Quelldatei 41 Marken, Extrakt 41 Eintraege, Nummern deckungsgleich |
| Z3 | Groessen im Index | bestanden | 18 Zeilen, alle Groessen stimmen mit der Datei ueberein |

Z1 prueft die Absatzstruktur nicht ueber die Zeichen, sondern ueber die Typografie der Quelle:
Zeilen innerhalb eines Absatzes haben 10,4 bis 10,7 pt Abstand, ein neuer Absatz beginnt bei
15,7 pt oder mehr. Der Pruefer sucht Stellen, an denen der Extrakt trennt, obwohl die Quelle
denselben Absatz fortsetzt.

## 4. Gefundene Maengel und ihr Stand

| Nr | Befund | Wirkung | Stand |
|---|---|---|---|
| M1 | Der gedruckte Titel `TRANSITIONAL AND FINAL PROVISIONS` (Seite 65) wurde als Fussnotenzeile erkannt und an die letzte Fussnote der Seite 64 angehaengt. | 30 Zeichen lagen auf der falschen Seite, Fussnote 37 war verfaelscht. | behoben am 2026-09-30, neu erzeugt und nachgeprueft |
| M2 | Das Erzeugnis war gegenueber dem Erzeuger veraltet: die Dateien waren geschrieben, bevor die Absatzregel fertig war. | 135 Absatztrennungen mitten im Absatz. | behoben durch Neuzeugung aus dem aktuellen Skript |
| M3 | `00_Index.md` nannte fuer `18_Quick_Reference_VentCon2.md` die Groesse 0. | falsche Metadaten im Index. | behoben am 2026-09-30 |
| M4 | In Anhang VIII steht ein Wort in anderer Reihenfolge als in der Quelle: der Extrakt setzt `cyberattacks` vor die Zeichenfolge aus x, die Quelle danach. | rein kosmetisch, kein Zeichenverlust, Seite 76 bleibt zeichengleich. | offen, nicht entschieden |
| M5 | Artikel 13 Absatz 17 wird in drei Absaetze getrennt, obwohl die Quelle sie mit 10,5 pt Zeilenabstand als einen Absatz setzt. | rein kosmetisch, Wortlaut unveraendert. | offen |
| M6 | Die Fussnote mit Sternchen auf Seite 65 steht als gewoehnlicher Absatz, nicht als Blockzitat. | rein kosmetisch, Inhalt vorhanden und richtig zugeordnet. | offen |

M4, M5 und M6 beruehren keinen Wortlaut. Die Seitenpruefung ist fuer alle 81 Seiten
zeichengleich, und die Massenbilanz ist exakt.

## 5. Befund zur Zitatfaehigkeit (Gate G4), nur gemeldet

Das Gate prueft die Extraktion gegen das Dossier, nicht das Dossier gegen die Verordnung.
Es wurde nichts korrigiert.

**Befund A: sieben Fundstellen der Checkliste sind gegen die Verordnung nicht aufloesbar.**
Betroffen sind `Annex I, Teil I` mit den Nummern 1.2, 1.3, 1.4, 1.7, 1.8, 1.9 und 1.10. Anhang I
Teil I traegt in der Verordnung nur die Punkte (1) und (2); Punkt (2) gliedert sich in die
Buchstaben (a) bis (m). Ein Nummernschema 1.1 bis 1.10 kommt im Normtext nicht vor. Belege:

1. Die Bildsichtpruefung der Seite 68 zeigt `(1)`, `(2)` und darunter `(a)` bis `(m)`.
2. Die Verordnung verweist in Artikel 13 Absatz 3 selbst auf `Part I, point (2)` und
   `Part I, point (1)` und stuetzt damit die Nummerierung (1) und (2).
3. Die drei Fundstellen mit den Buchstaben 2(b), 2(d) und 2(e) sind belegt. Sie zeigen, dass
   die Checkliste die Sache richtig meint, aber die Nummer falsch schreibt.

**Befund B: die statische Liste in Abschnitt 1 des Plans ist unvollstaendig.** Die Checkliste
nennt 43 verschiedene Referenzen, der Plan 22. Zusaetzlich verwendet die Checkliste unter
anderem Art. 13(2), Art. 13(8), Art. 14(2)(a) bis (c), Art. 14(4)(a) bis (c), Art. 14(7),
Art. 14(10), Art. 15, Art. 28, Art. 64 und Art. 69(2) sowie Art. 69(3).

**Befund C: die Referenzen Art. 3(3)(d), Art. 3(3)(e), Art. 3(3)(f), Art. 3(21) und Art. 3(22)
sind als RED-Bezug gefuehrt.** Fuer Art. 3(21) und Art. 3(22) ist diese Einordnung zu pruefen,
weil es Begriffsbestimmungen der Verordnung sind.

Die Befundliste liegt damit vor, wie es Abnahmekriterium 5 verlangt. Eine Aenderung an Zitaten
im Dossier ist eine Entscheidung des Nutzers.

## 6. Bildsichtpruefung (Gate G6)

Sechs Seiten wurden mit 110 dpi gerendert und gegen den Extrakt verglichen.

| Seite | Inhalt der Seite | Ergebnis |
|---|---|---|
| 1 | Titel, Erwaegungsformel mit Verweisen (1) und (2), Erwaegungsgruende 1 bis 3, Fussnoten 1 bis 4 | deckungsgleich |
| 6 | Erwaegungsgruende 23 bis 26, Fussnoten 7 und 10 | deckungsgleich |
| 35 | Titel Kapitel II, Artikel 13 Absaetze 1 bis 5 | deckungsgleich |
| 65 | Artikel 64 Absaetze 5 bis 10, Artikel 65, Kapitel VIII mit Titel, Artikel 66, Fussnote mit Sternchen | deckungsgleich |
| 68 | Anhang I Teil I und Teil II | deckungsgleich |
| 81 | Schluss des Anhangs VIII mit Punkt 8 | deckungsgleich |

Einzige bewusste Ausnahme: die Kopfdaten des Amtsblatts auf der Titelseite (`L series`,
`Official Journal`, `2024/2847`, `20.11.2024`) stehen nicht im Extrakt. Sie sind Amtsblatt und
kein Normtext; der Index nennt Quelle, Datum und Abrufweg. Die Bildpruefung der Seite 1 bestaetigt,
dass sonst nichts fehlt.

## 7. Offene Punkte

1. Entscheidung zu Befund A: die sieben Nummern 1.2 bis 1.10 in der Checkliste entweder auf die
   tatsaechlichen Buchstaben von Anhang I Teil I Punkt (2) umstellen oder als eigene Zaehlung
   kennzeichnen. Ohne Entscheidung bleibt die Fundstelle unbelegt.
2. M4, M5 und M6 sind kosmetisch und koennen offen bleiben oder nachgezogen werden.
3. Der Auftrag in Abschnitt 9 des Plans nennt das Gedaechtnis des Agenten Frankie. Diese Datei
   liegt ausserhalb des Projektordners und wurde nicht angefasst.

## 8. Reproduktion

```
C:\venvs\VentCon2\Scripts\python.exe tools\cra_extract.py --write
C:\venvs\VentCon2\Scripts\python.exe tools\cra_extract.py --verify
C:\venvs\VentCon2\Scripts\python.exe tools\verify_cra_extract.py
```

Rueckgabewert des Verifikationsskripts: 0, wenn alle Pruefungen bestanden sind. Der Lauf vom
2026-09-30 ergibt 1, weil die sieben Fundstellen aus Abschnitt 5 offen sind.
