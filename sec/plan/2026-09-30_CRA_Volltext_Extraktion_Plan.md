# Plan: CRA-Volltext als zitierfaehiges Markdown

Rolle: Strukturplan (Detaildokument, kein CRA-Bezug)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: keine (Normquelle, Erschliessung von `sources/EU_CRA_Regulation_2024_2847_EN.pdf`)
Bezug: Regulation (EU) 2024/2847 (Cyber Resilience Act, CRA)
Stand: 2026-09-30
Ausfuehrungsstand: AP1 und AP2 erledigt, AP3 bis AP8 offen (2026-09-30)
Ausfuehrende Agenten: Frankie (Textextraktion und Montage), GitHub Copilot (Integration, Verifikation)

Nachtrag 2026-09-30 nach der Ausfuehrungspruefung: Der Schreibpfad von `tools/cra_extract.py` fehlte
vollstaendig. `--write` war in Docstring und Argumentparser angekuendigt, wurde aber von `main()`
nie ausgewertet, sodass der Aufruf still mit Exit 0 endete und nichts schrieb. AP3 bis AP6 waren
daher nie implementiert, nicht nur nie ausgefuehrt. Entscheidungen E9 bis E12 ergaenzt, Zielstruktur
auf die tatsaechlichen Kapitelgrenzen umgestellt.

Auftrag: Den Volltext der CRA-Verordnung aus der PDF-Normquelle in Markdown ueberfuehren, sodass
jede im Dossier verwendete Fundstelle (Artikel, Absatz, Buchstabe, Anhang-Nummer) direkt belegbar
ist. Die Extraktion ist eine Quelle, kein Gutachten: Wortlaut vor Lesbarkeit.

---

## 1. Ausgangslage (geprueft 2026-09-30)

| Merkmal | Befund | Folge |
|---|---|---|
| Datei | `sources/EU_CRA_Regulation_2024_2847_EN.pdf`, 1.782.351 Bytes | eine Datei, kein OCR-Bedarf |
| Format | PDF 1.7, `/Encrypt` nicht vorhanden | kein Kopierschutz, Textebene frei lesbar |
| Erzeuger | `PDFlib+PDI 9.0.7p3`, Creator `Servigistics Arbortext Publishing Engine` | Amtstext-Pipeline des ABl., saubere Textebene |
| Schriften | 102 Font-Objekte, 3 `/Type0`-CID-Fonts mit `/ToUnicode`-CMaps | Text ist extrahierbar, CID-Abstandsartefakte moeglich |
| Bilder | genau 1 Bildobjekt | Textwerkzeug statt Bildwerkzeug (kein OCR) |
| Werkzeuge | `C:\venvs\VentCon2` hat nur `pillow`, `python-pptx` | PyMuPDF muss installiert werden |

Die PDF ist damit ein textgeborenes Dokument. OCR wuerde nur Fehler hinzufuegen und ist verworfen.

Verbraucher des Ergebnisses: `sec/CRA_Compliance_Checklist.md` und die Detaildokumente. Dort
bereits verwendete Fundstellen, die das Ergebnis belegen muss:

- Annex I, Teil I: 1.2, 1.3, 1.4, 1.7, 1.8, 1.9, 1.10, 2(b), 2(d), 2(e)
- Annex I, Teil II: Nr. 1 bis 4
- Annex II (`User Information`), Annex III (Class I/II), Annex IV (kritische Produkte), Annex VII
- Art. 13(5), Art. 14, Art. 27, Art. 64(10)

---

## 2. Entscheidungen (mit dem Nutzer abgestimmt am 2026-09-30)

| Nr. | Entscheidung |
|---|---|
| E1 | Umfang: **Volltext** (Erwaegungsgruende, Artikel 1 bis 71, Anhaenge I bis VIII) plus ein kurzer Schnellindex der fuer VentCon2 einschlaegigen Fundstellen |
| E2 | Struktur: **Index plus Kapiteldateien** in einem eigenen Ordner, nicht eine Monolithdatei |
| E3 | Sprache und Treue: **wortgetreues Englisch**, keine Paraphrase, keine Zusammenfassung im Normtext |
| E4 | Geviertstrich: im extrahierten Normtext bleibt die Quelltypografie erhalten (Zitatfreiheit). Frankie-eigener Text (Index, Kopfbloecke) folgt der Projektregel ohne U+2014 |
| E5 | Querpruefung: jede CRA-Fundstelle im Dossier wird gegen den Extrakt geprueft, Abweichungen werden gemeldet |
| E6 | Abhaengigkeit: PyMuPDF wird in `C:\venvs\VentCon2` installiert, niemand legt je ein venv in Dropbox an |
| E7 | Sprache der Huelle: Index und Provenienzkopf deutsch (Projektdokument), Normtext englisch (Quelle) |
| E8 | Umlaute: deutsche Prosa in `sec/` bleibt bei der bestehenden Transkription (ae/oe/ue/ss), der englische Normtext enthaelt keine |
| E9 | Dateischnitt: **ein Dokument je echtem Kapitel**, also 19 Dateien (Index, Erwaegungsgruende, 8 Kapitel, 8 Anhaenge, Schnellindex). Der Vorschlag der ersten Fassung mit 10 Artikeln je Datei ist verworfen, weil er Kapitel zerschnitten haette |
| E10 | Ausfuehrung: Frankie implementiert und faehrt AP3 bis AP6; Copilot verantwortet AP1, AP7, AP8 und alle sieben Gates |
| E11 | Gate G6: 20 Absaetze programmatisch gegen die Textebene, zusaetzlich 6 Seiten als Bildsichtpruefung |
| E12 | Schnellindex: Fundstellen werden aus der aktuellen Checkliste automatisch erhoben und gegen die statische Liste aus Abschnitt 1 verglichen; die Differenz geht in die G4-Befundliste ein |

---

## 3. Zielstruktur

```
sec/sources/EU_CRA_Regulation_2024_2847_EN/
  00_Index.md                     Provenienz, SHA-256, Seitenzahl, Abdeckungstabelle, Lesehinweise
  01_Recitals.md                  Titel, Erwaegungsformel, Erwaegungsgruende 1 bis 130 (S. 1 bis 27)
  02_Chapter_I_Art_01-12.md       Kapitel I, Artikel 1 bis 12 (S. 28)
  03_Chapter_II_Art_13-26.md      Kapitel II, Artikel 13 bis 26 (S. 35)
  04_Chapter_III_Art_27-34.md     Kapitel III, Artikel 27 bis 34 (S. 46)
  05_Chapter_IV_Art_35-51.md      Kapitel IV, Artikel 35 bis 51 (S. 51)
  06_Chapter_V_Art_52-60.md       Kapitel V, Artikel 52 bis 60 (S. 57)
  07_Chapter_VI_Art_61-62.md      Kapitel VI, Artikel 61 bis 62 (S. 63)
  08_Chapter_VII_Art_63-65.md     Kapitel VII, Artikel 63 bis 65 (S. 64)
  09_Chapter_VIII_Art_66-71.md    Kapitel VIII, Artikel 66 bis 71 (S. 65)
  10_Annex_I.md                   Anhang I, S. 68, wesentliche Cybersicherheitsanforderungen
  11_Annex_II.md                  Anhang II, S. 70, Nutzerinformation
  12_Annex_III.md                 Anhang III, S. 71, wichtige Produkte Class I/II
  13_Annex_IV.md                  Anhang IV, S. 72, kritische Produkte
  14_Annex_V.md                   Anhang V, S. 73
  15_Annex_VI.md                  Anhang VI, S. 74
  16_Annex_VII.md                 Anhang VII, S. 75, technische Dokumentation
  17_Annex_VIII.md                Anhang VIII, S. 76
  18_Quick_Reference_VentCon2.md  Schnellindex der einschlaegigen Fundstellen (E1, E12)
```

Das sind 19 Dateien. Die Grenzen stammen aus AP2 und folgen den Kapiteln der Verordnung (E9).
Die Dateinamen nennen bewusst keinen Kapiteltitel, weil AP2 die Titelzeilen nicht erfasst hat;
der echte Titel steht im jeweiligen Dateikopf. Der PDF-Dateiname bleibt unveraendert.

---

## 4. Arbeitspakete

| AP | Inhalt | Ergebnis | Ausfuehrung |
|---|---|---|---|
| AP1 | Werkzeug: PyMuPDF in `C:\venvs\VentCon2` installieren, Version festhalten | funktionierende Extraktion | Copilot |
| AP2 | Strukturkarte: Seitenzahl, Artikelanfaenge, Anhanganfaenge, Kopf-/Fusszeilen identifizieren | Kapitelgrenzen, Seiten-Abschnitt-Zuordnung als JSON | Frankie |
| AP3 | Textgewinnung je Abschnitt: Seitentexte, Fussnoten, Tabellen | Rohtext je Einheit mit Seitenmarken | Frankie |
| AP4 | Normalisierung: Laufkopf und Seitenzahl entfernen, Ligaturen aufloesen, Mehrfachleerzeichen bereinigen, Fussnotenband von Koerper trennen, Anhang-Teilueberschriften erkennen | lesbarer Rohtext, Seitenmarken als HTML-Kommentar erhalten | Frankie |
| AP5 | Montage: Markdown mit Inhaltsverzeichnis, Ankern, `## Article 13`, `### 1.`, Tabellen als Markdown-Tabellen, Fussnoten erhalten | 19 Dateien laut Abschnitt 3 | Frankie |
| AP6 | Schnellindex: Fundstellen aus Abschnitt 1 mit Datei und Anker verlinken, zusaetzlich automatisch aus der Checkliste erhoben (E12) | `18_Quick_Reference_VentCon2.md` | Frankie |
| AP7 | Verifikation nach Abschnitt 5 | Verifikationsskript plus Ergebnisprotokoll | Copilot |
| AP8 | Nachlauf: Register im Master und Agenten-Gedaechtnis aktualisieren | konsistentes Dossier | Copilot |

AP3 bis AP6 sind der Frankie-Auftrag (E10). AP1, AP7 und AP8 bleiben beim Hauptagenten, weil sie
Werkzeugumgebung, Abnahme und Dossierkonsistenz betreffen. AP1 und AP2 sind erledigt.

Befunde aus AP2, die AP3 bis AP5 binden:

| Befund | Bedeutung fuer die Ausfuehrung |
|---|---|
| 0 Zeilenenden mit Bindestrich, 0 weiche Trennzeichen | die Trennregel in AP4 entfaellt, `dehyphenate()` ist toter Code und wird entfernt |
| 0 U+FFFD, 0 U+2014, genau 1 U+2013 | Kodierung unversehrt, die Ausnahme in E4 greift praktisch nicht |
| Erwaegungsgrund und Fussnotenmarke haben dieselbe Form `(n)` | das Fussnotenband darf nicht als Erwaegungsgrund gezaehlt werden, sonst nennt der Index 180 statt 130 |
| Anhang I traegt `Part I` und `Part II`, die Recon-Regel `^PART [IVXL]+$` findet 0 Treffer | die Teilueberschriften der Anhaenge brauchen eine eigene Regel |
| Textebene 349.735 Zeichen (Koerper 341.297, Fussnoten 8.438) | Messbasis fuer Gate G2 |

---

## 5. Qualitaetssicherung

Sieben Gates, jedes mit Abbruchkriterium. Ein Gate ohne Ergebnis blockiert die Abnahme.

| Gate | Pruefung | Methode | Abbruchkriterium |
|---|---|---|---|
| G1 | Vollstaendigkeit | Artikel 1 bis 71 genau einmal, Anhaenge I bis VIII vorhanden, Erwaegungsgruende lueckenlos nummeriert | fehlende oder doppelte Einheit |
| G2 | Massenbilanz | Zeichenzahl des Extrakts gegen die Textebene, Messbasis 349.735 Zeichen | Verlust groesser 0,5 Prozent |
| G3 | Seitengrenzen | letzte Woerter der Seite n gehen in erste Woerter der Seite n+1 ueber | abgerissener Satz oder fehlende Zeile |
| G4 | Fundstellen | alle Referenzen aus Abschnitt 1 woertlich auffindbar | Referenz ohne Beleg |
| G5 | Kodierung | keine Ersatzzeichen, keine verlorenen Nicht-ASCII-Glyphen, keine leeren Ueberschriften | U+FFFD oder leere Ueberschrift |
| G6 | Wortlauttreue | 20 Absaetze programmatisch gegen die Textebene, zusaetzlich 6 Seiten als Bildsichtpruefung (E11) | inhaltliche Abweichung, nicht nur Typografie |
| G7 | Hausregeln | kein U+2014 in Frankie-eigenem Text, Dateinamen nach Konvention, keine venv in Dropbox | Regelverstoss |

G4 ist die eigentliche Absicherung des Dossiers: Sie prueft nicht die Extraktion, sondern ob die
bestehende Checkliste die Verordnung richtig zitiert. Abweichungen werden berichtet, nicht still
korrigiert.

---

## 6. Werkzeuge und Umgebung

| Punkt | Festlegung |
|---|---|
| Interpreter | `C:\venvs\VentCon2\Scripts\python.exe` (kein venv in Dropbox) |
| Bibliothek | PyMuPDF, Version im Index dokumentieren |
| Extraktionsskript | `tools/cra_extract.py`, wiederholbar und ohne Handeingriff |
| Verifikationsskript | `tools/verify_cra_extract.py`, im Stil der vorhandenen `verify_*.py` |
| Seitenmarken | HTML-Kommentar `<!-- PDF S. 12 -->` je Seitenanfang, damit jede Aussage auf die PDF-Seite zurueckfuehrbar ist |
| Wiederholbarkeit | Skript und Eingabe-Hash erlauben die Wiederherstellung der Dateien aus der PDF |

---

## 7. Risiken und Gegenmassnahmen

| Risiko | Wirkung | Gegenmassnahme |
|---|---|---|
| CID-Font erzeugt falsche Abstaende oder Ligaturverluste | unlesbare Woerter, falsche Zitate | G5 und G6, Reparaturregeln in AP4 dokumentiert |
| Tabellen in den Anhaengen III, IV, V, VII brechen um | Verlust der Struktur | Anhaenge einzeln pruefen, Tabellenerkennung nur wo noetig |
| Fussnoten zu Aenderungsverordnungen gehen verloren | falscher Rechtsstand | Fussnoten in AP3 gezielt einsammeln, im Index als Standhinweis fuehren |
| Sehr grosse Textmenge im Kontext | lange Laufzeit, hohe Kosten | Aufteilung nach Kapiteln, gezieltes Lesen ueber den Index |
| Abweichung zwischen Extrakt und bestehender Checkliste | Vertrauensverlust im Dossier | G4 liefert Befundliste zur manuellen Entscheidung |
| Monatsende-Aktualitaet der Norm | veralteter Stand | Stand und Quellhash im Index nennen, Aktualisierung als Nachlauf vermerken |

---

## 8. Abnahmekriterien

1. Zielstruktur aus Abschnitt 3 ist vollstaendig vorhanden.
2. Alle sieben Gates aus Abschnitt 5 sind bestanden und protokolliert.
3. `00_Index.md` nennt Datei, SHA-256, Seitenzahl, Extraktionsdatum, Werkzeugversion und Abdeckung.
4. Jede Referenz aus Abschnitt 1 ist im Schnellindex mit Datei und Anker verlinkt.
5. Die Befundliste aus G4 liegt vor, auch wenn sie leer ist.
6. Kein einziges Zeichen U+2014 in den von Frankie verfassten Textteilen.

## 9. Nachlauf

1. `sec/CRA_Compliance_Checklist.md`, Abschnitt 4: Zeile fuer den extrahierten Volltext ergaenzen,
   die PDF-Zeile bleibt als Original erhalten.
2. Frankie-Gedaechtnis: Erkenntnisse zu CID-Fonts, Normtext-Format und Seitenmarken uebernehmen.
3. Befundliste aus G4 dem Nutzer vorlegen, vor jeder Aenderung an Zitaten im Dossier.
