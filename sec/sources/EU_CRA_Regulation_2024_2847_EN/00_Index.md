# CRA-Volltext als zitierfaehiges Markdown: Index

Rolle: Erschliessung der Normquelle, keine Bewertung. Erzeugt am 2026-09-30.
Quelle: sec/sources/EU_CRA_Regulation_2024_2847_EN.pdf
Plan: sec/plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md
Werkzeug: tools/cra_extract.py
Bibliothek: PyMuPDF 1.28.2
Extraktionsdatum: 2026-09-30

## 1. Quelle und Massenbilanz

| Merkmal | Wert |
|---|---|
| Datei | sec/sources/EU_CRA_Regulation_2024_2847_EN.pdf |
| SHA-256 | e3ecaabddf6e321fa04097d15dfcccc7309d0fe5896240ab7625a27d74ab1b0a |
| Seiten in der PDF | 81 |
| Format | PDF 1.7, textgeboren (PDFlib plus Arbortext), kein OCR |
| Zeilen in der Textschicht | 4379 |
| Textebene gesamt (Messbasis des Plans) | 349735 Zeichen |
| Laufkopf und Fusszeile entfernt | 328 Zeilen, 5709 Zeichen |
| Normzeilen nach dem Entfernen | 4051 |
| Zeilenumbrueche, die im Markdown zu Leerzeichen werden | 4050 |
| Messbasis der Massenbilanz | 349735 minus 5709 plus 4050 = 348076 Zeichen |
| Extrakt Normtext | 348060 Zeichen |
| Abweichung | -16 Zeichen = -0.005 Prozent |
| Abweichung ohne Leerzeichen | +0 Zeichen |
| Extrakt direkt gegen die 349735 Zeichen | -0.479 Prozent |

Die Messbasis von 349735 Zeichen enthaelt den laufenden Kolumnentitel und die Seitenzahl
des Amtsblatts, naemlich 328 Zeilen mit 5709 Zeichen. Diese Zeilen werden bewusst entfernt. Umgekehrt werden die Zeilenumbrueche
innerhalb eines Absatzes im Markdown zu einzelnen Leerzeichen; das erhoeht die Zeichenzahl
wieder. Beide Seiten der Bilanz werden deshalb gleich behandelt. Der Vergleich direkt
gegen 349735 Zeichen faellt um -0.479 Prozent ab, und zwar genau um die entfernten Laufkopf- und
Fusszeilen. Ein Verlust an Normtext entsteht dabei nicht.

## 2. Die 19 Dateien

| Datei | Inhalt | PDF-Seiten | Bytes |
|---|---|---|---|
| 01_Recitals.md | Titel, Erwaegungsformel und Erwaegungsgruende 1 bis 130 | 1 bis 28 | 155968 |
| 02_Chapter_I_Art_01-12.md | Kapitel I, Artikel 1 bis 12 | 28 bis 35 | 29508 |
| 03_Chapter_II_Art_13-26.md | Kapitel II, Artikel 13 bis 26 | 35 bis 46 | 51475 |
| 04_Chapter_III_Art_27-34.md | Kapitel III, Artikel 27 bis 34 | 46 bis 51 | 20334 |
| 05_Chapter_IV_Art_35-51.md | Kapitel IV, Artikel 35 bis 51 | 51 bis 56 | 20325 |
| 06_Chapter_V_Art_52-60.md | Kapitel V, Artikel 52 bis 60 | 57 bis 63 | 28486 |
| 07_Chapter_VI_Art_61-62.md | Kapitel VI, Artikel 61 bis 62 | 63 bis 63 | 3838 |
| 08_Chapter_VII_Art_63-65.md | Kapitel VII, Artikel 63 bis 65 | 64 bis 65 | 6662 |
| 09_Chapter_VIII_Art_66-71.md | Kapitel VIII, Artikel 66 bis 71 | 65 bis 67 | 4906 |
| 10_Annex_I.md | Anhang I | 68 bis 69 | 6201 |
| 11_Annex_II.md | Anhang II | 70 bis 70 | 3333 |
| 12_Annex_III.md | Anhang III | 71 bis 71 | 2946 |
| 13_Annex_IV.md | Anhang IV | 72 bis 72 | 1334 |
| 14_Annex_V.md | Anhang V | 73 bis 73 | 1901 |
| 15_Annex_VI.md | Anhang VI | 74 bis 74 | 1241 |
| 16_Annex_VII.md | Anhang VII | 75 bis 75 | 4213 |
| 17_Annex_VIII.md | Anhang VIII | 76 bis 81 | 23060 |
| 18_Quick_Reference_VentCon2.md | Schnellindex der Fundstellen fuer VentCon2 | 01 bis 81 (Querverweis) | 10610 |

## 3. Abdeckung

### 3.1 Erwaegungsgruende

| Einheit | Datei | Nummern | Anzahl | Status |
|---|---|---|---|---|
| Erwaegungsgruende | 01_Recitals.md | 1 bis 130 | 130 | vollstaendig |

### 3.2 Kapitel und Artikel

| Einheit | Datei | Artikel | Anzahl |
|---|---|---|---|
| CHAPTER I | 02_Chapter_I_Art_01-12.md | 1 bis 12 | 12 |
| CHAPTER II | 03_Chapter_II_Art_13-26.md | 13 bis 26 | 14 |
| CHAPTER III | 04_Chapter_III_Art_27-34.md | 27 bis 34 | 8 |
| CHAPTER IV | 05_Chapter_IV_Art_35-51.md | 35 bis 51 | 17 |
| CHAPTER V | 06_Chapter_V_Art_52-60.md | 52 bis 60 | 9 |
| CHAPTER VI | 07_Chapter_VI_Art_61-62.md | 61 bis 62 | 2 |
| CHAPTER VII | 08_Chapter_VII_Art_63-65.md | 63 bis 65 | 3 |
| CHAPTER VIII | 09_Chapter_VIII_Art_66-71.md | 66 bis 71 | 6 |

Artikel 1 bis 71 erscheinen genau einmal: ja. Vollstaendige Liste: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71.
Fehlend: keine. Doppelt: keine.

### 3.3 Anhaenge

| Einheit | Datei | Drucktitel im Amtsblatt | Seiten |
|---|---|---|---|
| ANNEX I | 10_Annex_I.md | ESSENTIAL CYBERSECURITY REQUIREMENTS | 68 bis 69 |
| ANNEX II | 11_Annex_II.md | INFORMATION AND INSTRUCTIONS TO THE USER | 70 bis 70 |
| ANNEX III | 12_Annex_III.md | IMPORTANT PRODUCTS WITH DIGITAL ELEMENTS | 71 bis 71 |
| ANNEX IV | 13_Annex_IV.md | CRITICAL PRODUCTS WITH DIGITAL ELEMENTS | 72 bis 72 |
| ANNEX V | 14_Annex_V.md | EU DECLARATION OF CONFORMITY | 73 bis 73 |
| ANNEX VI | 15_Annex_VI.md | SIMPLIFIED EU DECLARATION OF CONFORMITY | 74 bis 74 |
| ANNEX VII | 16_Annex_VII.md | CONTENT OF THE TECHNICAL DOCUMENTATION | 75 bis 75 |
| ANNEX VIII | 17_Annex_VIII.md | CONFORMITY ASSESSMENT PROCEDURES | 76 bis 81 |

Anhaenge I bis VIII erscheinen genau einmal: ja. Gefunden: I, II, III, IV, V, VI, VII, VIII.

## 4. Fussnoten

Der Extrakt fuehrt 41 Fussnoten, verteilt auf 24 Seiten: [1, 2, 3, 6, 7, 8, 10, 11, 12, 13, 17, 18, 19, 20, 21, 25, 26, 27, 28, 64, 66, 71, 72, 74].
Sie stehen als Blockzitat direkt unter dem Text ihrer Seite und sind damit kein Normtext, sondern Standhinweis. Ihre Zeichen sind in der Massenbilanz enthalten.

## 5. Lesekonventionen

- `#` traegt die Einheit so, wie das Amtsblatt sie druckt (`# CHAPTER I`, `# ANNEX I`).
  Die gedruckte Titelzeile (etwa `GENERAL PROVISIONS`) folgt darunter als eigener Absatz.
- `##` traegt `## Article 13` und, in den Anhaengen, `## Part I`. Der gedruckte Titel der
  Einheit folgt jeweils als eigener Absatz, damit der Anker stabil bleibt.
- `###` traegt nummerierte Absaetze (`### 1.`) und in den Erwaegungsgruenden die Marke
  (`### (1)`). Die Nummer stammt unveraendert aus der Quelle.
- `<!-- PDF S. n -->` steht an der Stelle, an der die Seite n beginnt. Jede Seite von 1
  bis 81 ist mindestens einmal markiert.
- Der Normtext steht zwischen `<!-- NORM BEGIN -->` und `<!-- NORM END -->`.
- Fussnoten sind Blockzitate (`>`) und mit `<!-- Fussnoten zu PDF S. n -->` eingeleitet.

## 6. Grenzen und Unsicherheiten

- Wortlaut vor Lesbarkeit: Absaetze werden nicht umformuliert, nicht gekuerzt und nicht
  umsortiert. Zeilenumbrueche der Druckfassung werden zu einem Absatz zusammengezogen.
- Ein Absatz, der ueber eine Seitengrenze laeuft, wird an der Seitenmarke getrennt. Das
  ist die Folge der geforderten Seitenmarken und keine inhaltliche Aenderung.
- Die Anhaenge III, IV, V und VII enthalten im Original keine Tabellen, sondern
  nummerierte Listen. Es wurde daher keine Markdown-Tabelle erzeugt und keine Zelle
  umgeformt. Anhang I Teil I und Teil II tragen die Marken (1), (2) und (a) bis (m) im
  Fliesstext; sie sind nicht in Ueberschriften umgesetzt, weil die Marken dort mitten in
  der Zeile stehen.
- Die Signaturspalte der PDF-Seite 67 (Parlament und Rat nebeneinander) ist in
  Leserichtung zeilenweise aufgeloest.
- Die Erwaegungsgruende 1 bis 130 sind vollstaendig; die Nummern der Fussnoten
  ueberschneiden sich mit ihnen, sind aber positionsbasiert getrennt.
- Der Kolumnentitel und die Seitenzahl des Amtsblatts sind entfernt. Sie werden nur
  entfernt, wenn sie das Zeilenmuster des Amtsblatts treffen und zusaetzlich im oberen
  oder unteren Rand der Seite stehen.
- Die Nummern 1.2 bis 1.10 der Checkliste sind gegen die Verordnung nicht aufloesbar,
  siehe 18_Quick_Reference_VentCon2.md, Abschnitt 3.
