# Plan: Worddatei mit den in den Antworten referenzierten Maßnahmen

Datum: 2026-10-02
Gegenstand: neue Worddatei in `sec/AalbertsChecklists/filled/` als Anlage zu den Aalberts-Antworten
Vorlage: `sec/VENTREX Wordvorlage_ohne Fußzeile.docx`
Status: entschieden und umgesetzt am 2026-10-02, verifiziert

## Auftrag

Die Antworten in Checkliste und Fragebogen verweisen auf Maßnahmenkennungen wie `measure P4.2`,
`measure P4.3` oder `measure P5.1`. Im Aalberts-Paket steht diesen Verweisen keine Erklärung
gegenüber. Die neue Datei beschreibt die Maßnahmen, damit eine Leserin bei Aalberts jede Kennung
auflösen kann, ohne das interne Dossier zu öffnen.

## Was die Antworten heute referenzieren

| Dokument | genannte Maßnahmen |
|---|---|
| Checkliste | P1.1, P1.2, P1.4, P2.1, P2.2, P2.3, P3.2, P3.3, P4.1, P4.2, P4.3, P4.5, P4.8 |
| Fragebogen | P1.5, P4.2, P4.3, P4.4, P4.5, P4.6, P4.8, P5.1 |
| Kopfzeile der Checkliste | Bereichsangabe "measures P1.1 to P5.2" |

In Summe werden 17 der 21 Maßnahmen namentlich genannt. Nicht genannt werden P3.1, P4.7 und P5.2.

## Datenquelle

`sec/CRA_Compliance_Checklist.md` ist die einzige Statusquelle:

| Abschnitt | Inhalt je Maßnahme |
|---|---|
| 7 Verknüpfung | Risiko (R1 bis R9), CRA-Pflicht-Nummer, Kennung mit Titel, Detaildokument, Stand |
| 8 Arbeitspakete | Phase 1 bis 5, Kennzeichnung "aktiv" oder "ruht bis Marktentscheidung", Kurzfassung des Inhalts |

Die Maßnahmen P1.1 bis P5.2 sind vollständig, 21 Zeilen. Es wird nichts ergänzt, was dort nicht
steht.

## Vorlage

`sec/VENTREX Wordvorlage_ohne Fußzeile.docx`, ausgelesen am 2026-10-02:

| Merkmal | Wert |
|---|---|
| Seite | A4, 8,27 x 11,69 Zoll, Ränder 0,98 / 0,79 / 0,98 / 0,98 Zoll |
| Körper | leer, ein leerer Absatz, keine Tabelle |
| Kopf und Fuß | Grafik aus dem VENTREX-Design, Kopf zwei Zeichnungen, Fuß bis zu fünf Zeichnungen und zwei Textboxen |
| Themenschriften | Clash Display für Überschriften, Satoshi für Text |
| Dokumentensprache | de-DE |
| Fehlend | keine Überschriftenformate, Überschriften müssen direkt formatiert werden (Clash Display, fett, Markenblau `#002F87`) |

## Entscheidungen des Nutzers (2026-10-02)

| Nr. | Entscheidung |
|---|---|
| E65 | Umfang: alle 21 Maßnahmen P1.1 bis P5.2, auch die nicht genannten P3.1, P4.7 und P5.2 |
| E66 | Sprache: Englisch, wie die Antworten |
| E67 | Gliederung: eine Tabelle, eine Zeile je Maßnahme |
| E68 | Felder: Kennung und Titel, Phase, Stand, Marktauslöser, Risikobezug, CRA-Pflicht-Nummer, Detaildokument, Kurzbeschreibung |
| E69 | Dateiname `VentCon2_Measures_Reference_2026-10-02.docx`, Ablage in `sec/AalbertsChecklists/filled/` |
| E70 | Erzeugung aus einem Skript mit eigenem Verifier |
| E71 | Marktauslöser als Kasten am Anfang und als Kennzeichnung bei jeder ruhenden Maßnahme |
| E72 | Die beiden Aalberts-Wordkopien verweisen auf die Anlage |
| E73 | Revision am 2026-10-02: Die Spalte `Detail document` entfällt. Die frei werdende Breite geht an `Measure and scope`, `Status` und `Risk / CRA duty`. Die Detaildokumente bleiben im Master verzeichnet, die Anlage nennt sie nicht mehr |
| E74 | Revision am 2026-10-02: Die Spalte `Phase` entfällt ebenfalls. Die Kennung trägt die Phase bereits (`P4.2` gehört zu Phase 4), das Werkzeug prüft diesen Zusammenhang weiterhin gegen die Überschriften in Abschnitt 8 des Masters |

## Umsetzung und Ergebnis

1. `tools/measure_reference.py` liest Abschnitt 7 und 8 des Masters, prüft die eigene Liste gegen den Master und schreibt die Anlage aus der Vorlage. Der Abgleich bricht ab, wenn der Master eine Maßnahme führt, die im Werkzeug fehlt, oder umgekehrt, und wenn ein Detaildokument im Dateisystem fehlt.
2. `tools/verify_measure_reference.py` prüft 300 Punkte: 21 Kennungen in Reihenfolge und ohne Dopplung, jede Kennung gegen den Master belegt, Phase im Master gegen die Kennung, aktive und ruhende Maßnahmen gegen den Vorspann von Abschnitt 8, Standvokabular, Felder gefüllt, Schriftgröße 8 pt, Kopfzeile fett und weiß auf Markenblau, Spaltenbreiten und Satzbreite, Hinweiskasten, Seiteneinrichtung und Kopf- und Fußgrafik der Vorlage, Prüfsumme der Vorlage, keine Platzhalter.

Ergebnis: 21 Zeilen, 4 Spalten, Spaltenbreiten 0,42 / 2,95 / 1,55 / 1,38 Zoll, Summe 6,30 Zoll gleich der Satzbreite. Vier Maßnahmen sind als aktiv im Prototypenbetrieb gekennzeichnet, elf als ruhend bis zur Marktentscheidung, das sind die Zahlen aus dem Master. Prüfsumme der Vorlage unverändert.

Die beiden Wordkopien tragen jetzt eine vierte Hinweiszeile mit dem Verweis auf die Anlage; `tools/verify_aalberts_answers.py` prüft mit 217 Punkten, dass die Anlage existiert und in beiden Kopien genannt wird.

## Scope-Grenzen

- Die neue Datei ist eine Ableitung, der Master bleibt die einzige Statusquelle.
- Keine Bewertung, keine Zusage zu Terminen, keine neuen Angaben.
- Vorlagen und bestehende Antworten bleiben unverändert, außer der Nutzer wünscht einen Verweis
  nach Q8.
