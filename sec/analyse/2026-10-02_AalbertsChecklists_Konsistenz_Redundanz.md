# Analyse: Konsistenz und Redundanz in `sec/AalbertsChecklists`

Datum: 2026-10-02
Gegenstand: neun Dateien in den Ordnern `templates/`, `extracts/` und `filled/`
Methode: Auswertung mit python-docx und SHA256, Textvergleich der Antwortzellen, Ähnlichkeitsvergleich
zwischen den Dokumenten, Abgleich der Maßnahmenverweise gegen die Anlage. Keine Datei geändert.

## 1 Bestand

| Ordner | Datei | Größe | Rolle |
|---|---|---|---|
| `templates` | CRA product maturity checklist - Sep 2026.docx | 191.656 B | Vorlage, unberührt |
| `templates` | CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx | 190.935 B | Vorlage, unberührt |
| `templates` | CRA Compliance Addendum (Supplier Agreement) - template Sep 2026 .docx | 198.015 B | Vorlage, unberührt |
| `extracts` | CRA_Product_Maturity_Checklist.md | 14.981 B | Extrakt mit 31 Antworten und 4 Hinweisen |
| `extracts` | CRA_Supplier_Due_Diligence_Questionnaire.md | 13.326 B | Extrakt mit 36 Antworten und 4 Hinweisen |
| `extracts` | CRA_Compliance_Addendum_Supplier_Agreement.md | 4.095 B | Extrakt ohne Antworten |
| `filled` | CRA_Product_Maturity_Checklist_answered.docx | 149.922 B | Rücksendefassung Checkliste |
| `filled` | CRA_Supplier_Due_Diligence_Questionnaire_answered.docx | 149.251 B | Rücksendefassung Fragebogen |
| `filled` | VentCon2_Measures_Reference_2026-10-02.docx | 112.999 B | Maßnahmen-Anlage, 21 Zeilen |

## 2 Konsistenz

### 2.1 Extrakt gegen Vorlage

| Prüfung | Checkliste | Fragebogen | Addendum |
|---|---|---|---|
| SHA256 im Kopfblock gegen Datei | stimmt | stimmt | stimmt |
| Absätze mit Text | 40 von 59 | 27 von 43 | 17 von 26 |
| Tabellen | 2 von 2 | 7 von 7 | 0 von 0 |

Die niedrigeren Zahlen im Kopfblock sind Absätze mit Text, die Gesamtzahl enthält die leeren
Absätze. Kein Widerspruch.

### 2.2 Extrakt gegen Wordkopie

| Dokument | Antworten im Extrakt | Antwortzellen in der Kopie | Unterschied |
|---|---|---|---|
| Checkliste | 31 | 31 | keiner |
| Fragebogen | 36 | 36 | keiner |

Die Antworttexte stimmen nach Normierung von Markern und Zeilenumbrüchen überein. Die Hinweiszeilen
sind in beiden Darstellungen identisch, einschließlich des Verweises auf die Maßnahmen-Anlage.

### 2.3 Antworten untereinander

- Innerhalb der Checkliste: 31 Antworten, keine doppelt (die eine Dopplung kommt aus der Vorlage,
  siehe 3.3).
- Innerhalb des Fragebogens: 36 Antworten, keine doppelt.
- Zwischen Checkliste und Fragebogen: 720 mögliche Paare, davon zwei über einer Ähnlichkeit von
  0,50, beide inhaltlich harmlos (`no update path in the product yet, measure P4.3 open` gegen
  `No SBOM yet, measure P4.2 open` und gegen `No, measure P4.5 open`). Es gibt also keine
  inhaltliche Redundanz zwischen den beiden Dokumenten.

### 2.4 Maßnahmenverweise

| Prüfung | Ergebnis |
|---|---|
| verschiedene genannte Maßnahmen | 17 von 21 |
| genannte Maßnahmen ohne Eintrag in der Anlage | keine |
| Maßnahmen der Anlage ohne Verweis in den Antworten | P1.3, P3.1, P4.7, P5.2 |
| häufigste Verweise | P4.3 sechsmal, P4.4 viermal, P4.2 und P4.5 je dreimal |

Dass vier Maßnahmen nie genannt werden, ist unkritisch: die Anlage ist auf Wunsch vollständig (E65)
und deckt auch die Maßnahmen ab, die in keiner Antwort auftauchen.

### 2.5 Stand je Maßnahmenverweis

46 Verweise wurden geprüft, ob der in der Antwort genannte Stand zum Stand in der Anlage passt.
45 stimmen überein. Eine Abweichung:

| Kennung | Antwort sagt | Anlage sagt |
|---|---|---|
| P2.1 | `partly done` | `In progress, partly designed (JSON lock)` |

`partly done` liest sich als teilweise umgesetzt, `partly designed` als entworfen aber nicht
umgesetzt. Das ist der einzige inhaltliche Widerspruch im Paket. Der Master schreibt an dieser
Stelle `teilweise entworfen (JSON-Sperre), Pfadbereinigung offen`, die Anlage gibt ihn also richtig
wieder, die Antwort ist unscharf.

### 2.6 Datumsangaben

| Datei | Datum |
|---|---|
| Antworten in Checkliste und Fragebogen | `Date: 2026-10-01` |
| Maßnahmen-Anlage, Vorspann und Dateiname | 2026-10-02 |
| Extrakte, Feld `Extracted` | 2026-10-02 |

Das Paket trägt zwei Daten. Die Anlage erklärt ihren Stand als `as documented on 2026-10-02`, die
Antworten sind auf den 2026-10-01 datiert, obwohl sie am 2026-10-02 noch geändert wurden
(Schließen der offenen Angaben, Urteilstrennung).

### 2.7 Anschluss im Dossier

`sec/CRA_Compliance_Checklist.md` nennt das Aalberts-Paket an keiner Stelle. Weder Vorlagen noch
Extrakte, Kopien oder Anlage stehen im Dokumentenregister in Abschnitt 4. Die Antworten verweisen
auf den Master, der Master kennt die Antworten nicht.

## 3 Redundanzen

### 3.1 Gewollte Redundanz

Je Dokument existieren drei Darstellungen: Vorlage, Markdown-Extrakt und befüllte Wordkopie. Das
ist die vereinbarte Ablage (E52, E57) und wird von den beiden Prüfern abgesichert, die Vorlage
zusätzlich über die Prüfsumme im Kopfblock.

### 3.2 Anlage gegen Master

Die Anlage ist eine kennungsgleiche Kopie von Abschnitt 7 und 8 des Masters, 21 Zeilen mit Titel,
Stand, Risikobezug und Pflicht-Nummern. Diese Redundanz ist gewollt, sie ist der Zweck der Anlage.
Sie ist technisch gekoppelt: `tools/measure_reference.py` bricht ab, wenn der Master eine Maßnahme
führt, die im Werkzeug fehlt oder umgekehrt, und `tools/verify_measure_reference.py` prüft Phase,
aktive und ruhende Maßnahmen gegen den Master. Restrisiko: Wird der Master geändert, ohne die
Anlage neu zu erzeugen, weichen beide auseinander, bis jemand den Prüfer laufen lässt.

### 3.3 Dopplung aus der Vorlage

Die zweite Tabelle der Checkliste, Teil II, führt die Zeile `Separate security updates` zweimal,
identisch in beiden Spalten. Das ist ein Fehler in der Aalberts-Vorlage, nicht in unserem Paket.
Die Antwort wurde zweimal gesetzt, weil zwei Zeilen zu füllen waren, und die Wordkopie erbt die
Dopplung.

### 3.4 Verwaister Extrakt

`extracts/CRA_Compliance_Addendum_Supplier_Agreement.md` ist ein reiner Vorlagenauszug ohne
Antworten. Seit das Addendum mit E62 aus der Antwortstrecke genommen wurde, verweist keine Datei
und kein Werkzeug auf ihn, außer dem Extraktprüfer, der ihn weiter mitprüft. Er ist damit die
einzige Datei des Pakets ohne Aufgabe.

## 4 Befunde

| Nr. | Befund | Art | Empfehlung |
|---|---|---|---|
| K1 | P2.1: Antwort sagt `partly done`, Anlage `partly designed` | Widerspruch | Antwort auf `partly designed, path sanitisation open` angleichen |
| K2 | Antworten datiert 2026-10-01, Anlage 2026-10-02 | Unschärfe | Antwortdatum auf 2026-10-02 ziehen oder Anlage auf 2026-10-01 datieren |
| K3 | Vorlagendopplung `Separate security updates` | Vorlagenfehler | Beim Versand in der Kopie belassen, im Begleittext zur Rücksendung einmal erwähnen |
| K4 | Vier Maßnahmen ohne Verweis in den Antworten | kein Mangel | Unverändert lassen, die Anlage ist vollständig |
| K5 | Master kennt das Aalberts-Paket nicht | Lücke | Paket in Abschnitt 4 des Masters registrieren, spätestens mit Maßnahme P4.6 |
| K6 | Addendum-Extrakt ohne Aufgabe | Redundanz | Behalten, solange die Extraktstrecke drei Vorlagen abdeckt, oder bewusst aussortieren |
| K7 | Zwei Daten im Paket | Folge von K2 | Mit K2 zusammen lösen |

## 5 Umsetzung der Befunde K1 bis K4 (2026-10-02)

| Befund | Umsetzung |
|---|---|
| K1 | Die Antwort zur Vertraulichkeit und zur begrenzten Angriffsfläche sagt jetzt `measure P2.1 partly designed`, passend zur Anlage und zum Master |
| K2 | Alle Datumsangaben des Pakets stehen auf 2026-10-02: die Fußzeile des Produktfelds in der Checkliste, das Sign-off-Datum im Fragebogen und die Zeile `Answers` im Kopfblock beider Extrakte. Die Anlage trägt dieses Datum bereits |
| K3 | Die Checkliste trägt eine fünfte Hinweiszeile über der Überschrift zu Teil II: `Note on the template: Part II lists the safeguard "Separate security updates" twice, so the same answer appears in both rows.` Damit ist die geerbte Dopplung für die Leserin erklärt |
| K4 | Vier Antworten nennen jetzt auch die bisher ungenannten Maßnahmen: die Verfügbarkeitsantwort nennt P3.1 und P3.2 als erledigt, die Antwort zur begrenzten Wirkung nennt P1.3 als aktiv, die CE-Antwort nennt P4.7, die Meldeantwort nennt P5.1 und P5.2 |

Nachweis nach der Umsetzung: 21 von 21 Maßnahmen sind in den Antworten genannt, 50 Maßnahmenverweise
nennen denselben Stand wie die Anlage, im Paket steht kein Datum vom 2026-10-01 mehr. Prüfstand:
27 Prüfungen Extraktstrecke, 254 Prüfungen Antwortstrecke, 300 Prüfungen Anlage, alle bestanden. Die
Antwortspalte der Checkliste wuchs durch die Ergänzungen von 3,17 auf 3,22 Zoll, der Anteil bleibt mit
46 Prozent im vereinbarten Band.

## 6 Was geprüft und in Ordnung ist

- Alle drei Prüfsummen im Kopfblock stimmen mit den Vorlagen überein, die Vorlagen sind unberührt.
- Extrakt und Wordkopie stimmen in allen 67 Antworten und 8 Hinweiszeilen überein.
- Keine Antwort ist innerhalb eines Dokuments doppelt, keine Aussage widerspricht der anderen Datei.
- Alle 17 genannten Maßnahmenkennungen sind in der Anlage auflösbar.
- 45 der 46 Maßnahmenverweise nennen denselben Stand wie die Anlage.
- Das Paket ist maschinell abgesichert: 27 Prüfungen Extraktstrecke, 252 Prüfungen Antwortstrecke,
  300 Prüfungen Anlage.
