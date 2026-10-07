# Plan: Risikoliste in der Maßnahmen-Anlage und Markdown als Quelle der befüllten Worddateien

Datum: 2026-10-06
Gegenstand: `sec/AalbertsChecklists/filled/` mit drei Worddateien, `tools/aalberts_fill.py`,
`tools/measure_reference.py` und ihre Verifier
Grundlage: `sec/CRA_Compliance_Checklist.md` (Abschnitt 6 bis 8),
`sec/analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`,
`sec/plan/2026-10-02_Massnahmen_Referenz_Worddatei_Plan.md`,
`sec/plan/2026-10-01_Aalberts_Vorlagen_Ausfuellen_Plan.md`,
`sec/plan/2026-10-01_Aalberts_Ordnerstruktur_Plan.md`
Status: entschieden und umgesetzt am 2026-10-06, verifiziert (siehe Abschnitt Umsetzung und
Verifikation)

## Anlass

Zwei Aufträge laufen in denselben Dateien zusammen.

1. **Befund vom 2026-10-06, Risikobezug ohne Auflösung.** Die beantwortete Checkliste nennt die
   Risiken `R1` bis `R9` 19 Mal, die Maßnahmen-Anlage nennt sie 9 Mal, je einmal in der Spalte
   `Risk / CRA duty`. Keine der beiden Dateien erklärt, wofür die Kennungen stehen. Die einzige
   Auflösung steht im Master, Abschnitt 7, und in der Sicherheitsanalyse, beide intern und nicht
   Teil des Aalberts-Pakets. Die Anlage verfehlt damit ihren eigenen Auftrag, jede Kennung ohne
   Öffnen des Dossiers auflösbar zu machen.
2. **Auftrag vom 2026-10-06, Markdown als Quelle.** Jede Worddatei in `filled/` soll ihre
   eingetragenen Inhalte aus einer Markdown-Datei beziehen, eine Markdown-Datei je Worddatei. Aus
   diesen Markdown-Dateien werden die Worddateien zusammengesetzt.

## Befund im Einzelnen

| Dokument | Nennungen `R1` bis `R9` | Auflösung im Dokument |
|---|---|---|
| `CRA_Product_Maturity_Checklist_answered.docx` | 19, davon 4 als Bereich `R1 to R9` und einer als `R7 to R9` | keine Liste, einzelne Kennungen nur beiläufig im Satz erklärt (`R1`, `R4`) |
| `CRA_Supplier_Due_Diligence_Questionnaire_answered.docx` | 2, je als Bereich `R1 to R9` | keine |
| `VentCon2_Measures_Reference_2026-10-02.docx` | 9, je einmal als nackte Kennung in der Spalte `Risk / CRA duty` | keine |

Aufteilung der 21 Zeilen der Anlage in der Spalte `Risk / CRA duty`:

| Form | Zeilen | Anzahl |
|---|---|---|
| Nackte interne Kennung | P1.1 `R1`, P1.2 `R4`, P1.3 `R3`, P1.4 `R6`, P2.1 `R5`, P2.2 `R2`, P2.3 `R7`, P3.1 `R9`, P3.2 `R8` | 9 |
| Freitext ohne Kennung | P3.3 `radio layer without own authentication` | 1 |
| Fremde Kennungsfamilie, in beiden Zellhälften wiederholt | P1.5 `RED-1, RED-2 / duty RED-1, RED-2` | 1 |
| Verneinung | P4.1 bis P4.8, P5.1, P5.2 `no risk reference (standard duty)` beziehungsweise `(reporting duty)` | 10 |

Ursache: `tools/measure_reference.py` führt die Risikospalte als festen Text in der Konstante
`MEASURES` (Zeilen 76 bis 130) und schreibt sie in Zeile 343 als `"%s / duty %s"`. Der Master, der
die Beschreibung im Risikofeld trägt, wird nur für Kennung, Phase und Detaildokument gelesen.
`tools/verify_measure_reference.py` prüft in Zeile 227 lediglich, dass die Zelle gefüllt ist und
einen Schrägstrich enthält. Deshalb bestehen 300 Prüfpunkte, obwohl die Spalte inhaltlich leer ist.

Zweiter Befund aus derselben Stelle: Spalte 1 des Masters, `Risiko (Analyse)`, mischt drei
verschiedene Begriffe, Befundkennung, Freitextbefund und Pflichtfamilie, und verneint sich in zehn
Zeilen selbst (`kein Risikobezug`). Die Anlage hat diese Mischung übernommen.

## Zielbild der neuen Ablage und Werkzeugkette

Heute leben die Antworten als Python-Wörterbücher in `tools/aalberts_fill.py`, die Markdown-Extrakte
in `extracts/` sind Ausgabe, und ein erneuter Lauf von `tools/docx_extract.py` überschreibt sie. Die
Anlage ist vollständig im Werkzeug fest verdrahtet. Künftig gilt für alle drei Worddateien in
`filled/` dasselbe Muster:

| Stufe | Rolle | Beispiel |
|---|---|---|
| 1 Vorlage | Aalberts-Original, unverändert, liefert Layout und Vorlagentext | `templates/CRA product maturity checklist - Sep 2026.docx` |
| 2 Markdown | je Worddatei eine Datei mit dem eingetragenen Inhalt, handgepflegt oder aus dem Master erzeugt | `CRA_Product_Maturity_Checklist_answered.md` |
| 3 Werkzeug | setzt oder erzeugt die Worddatei aus Stufe 2, Vorlage liefert nur Layout | `tools/aalberts_fill.py` |
| 4 Worddatei | Abgabedatei für Aalberts | `filled/CRA_Product_Maturity_Checklist_answered.docx` |
| 5 Verifier | prüft Markdown gegen Master und Worddatei gegen Markdown | `tools/verify_aalberts_answers.py` |

Die Anlage ist der Sonderfall ohne Aalberts-Vorlage: sie erhält ebenfalls eine Markdown-Datei, in
der die 21 Maßnahmenzeilen und die neue Risikoliste stehen. Ihre Zeilen bleiben aus dem Master
abgeleitet, damit der Master nach E3 die einzige Statusquelle bleibt.

## Teil A: Risikoliste in der Anlage

1. Die Anlage führt zwei Tabellen: die Maßnahmen mit den Spalten `ID`, `Measure and scope`,
   `Status`, `Finding`, `CRA duty` und die Befunde mit den Spalten `ID`, `Finding`, `Measure`,
   `Status`.
2. Der englische Befundtext je Risiko-ID steht in der Konstante `FINDINGS` des Werkzeugs. Die
   Zuordnung Befund zu Maßnahme und der Stand kommen aus Abschnitt 7 und 8 des Masters, es wird
   nichts doppelt gepflegt. Die Verneinungen des Masters werden zu `none (standard duty)`,
   `none (reporting duty)` und `none (RED norm duty)`.
3. Der Stand einer Befundzeile ist derselbe wie in der zugehörigen Maßnahmenzeile, damit die
   beiden Tabellen nicht auseinanderlaufen können.
4. Beide Aalberts-Kopien und der Anlagentext nennen die Risikoliste.
5. Guards: Das Werkzeug bricht ab, wenn der Master eine Risiko-ID führt, die im Werkzeug fehlt,
   oder umgekehrt, und wenn eine Befundzelle weder Kennung noch eine der vereinbarten Verneinungen
   ist. Damit ist der ursprüngliche Befund, ein Befund ohne Kennung, künftig ein Baufehler.
   Der Verifier prüft zusätzlich, dass jede in den Abgabedateien genannte Risiko-ID in der Anlage
   aufgelöst wird.

## Teil B: Markdown als Quelle

1. Je Worddatei in `filled/` liegt eine Markdown-Datei in `filled/src/` mit demselben Stamm.
2. Ablauf je Worddatei: Werkzeug -> Markdown-Quelle -> Worddatei. `tools/aalberts_fill.py`
   schreibt die Quelle aus dem Extrakt plus den Antworten, den Hinweiszeilen
   (`<!-- note: <Anker> --> <Text>`), den Werten für die Leerzeilen und der Antwortzeile im
   Kopfblock und setzt die Wordkopie anschließend allein aus dieser Quelle.
3. `tools/measure_reference.py` schreibt die Anlagenquelle mit Kopfblock und beiden Tabellen und
   setzt daraus die Anlage.
4. `tools/docx_extract.py` lehnt Quellen unter `filled` ab. Die Extrakte unter `extracts/` tragen
   wieder nur Vorlagentext, ein Extraktlauf kann also keine Eintragungen mehr überschreiben.
5. `tools/verify_aalberts_answers.py` prüft Quelle gegen Wordkopie, Hinweiszeilen,
   Leerzeilenwerte und die Kennungsauflösung, `tools/verify_measure_reference.py` prüft Quelle
   gegen Worddatei und beide gegen den Master.

## Entscheidungen des Nutzers (2026-10-06)

| Nr. | Entscheidung |
|---|---|
| E78 | Ablage und Namen der Markdown-Dateien: eigener Unterordner `sec/AalbertsChecklists/filled/src/`, je Worddatei eine Datei mit gleichem Stamm (`VentCon2_Measures_Reference_2026-10-02.md`, `CRA_Product_Maturity_Checklist_answered.md`, `CRA_Supplier_Due_Diligence_Questionnaire_answered.md`). `filled/` trägt damit nur die Abgabedateien |
| E79 | Rolle der Markdown-Dateien: alle drei sind erzeugte Artefakte, die Antworttexte im Werkzeug `tools/aalberts_fill.py` und die Befundtexte in `tools/measure_reference.py` bleiben die Quelle. Erzeugung: Werkzeug -> Markdown -> Worddatei |
| E80 | Spalten der Risikoliste: `ID`, `Finding`, `Measure`, `Status`. Keine Bewertungsspalte, weil eine Einstufung der Befunde R4 bis R10 neue Information wäre und die Bewertung Aufgabe der offenen Maßnahme P4.1 ist |
| E81 | Die Spalte `Risk / CRA duty` der Anlage wird in `Finding` und `CRA duty` getrennt. Der bisher kennungslose Funkbefund (P3.3) wird als R10 in den Master aufgenommen, damit jede Zeile entweder einen Befund oder ausdrücklich keinen Befundbezug trägt |
| E82 | Beide Aalberts-Wordkopien nennen die Risikoliste neben dem Maßnahmenverweis: "The measures referenced in this document, P1.1 to P5.2, and the findings R1 to R10 are described in the annex ...". Die Bereiche in den Antworten werden von `R1 to R9` auf `R1 to R10` gezogen; die Aussage "firmware 2.7.3 closed R7 to R9" bleibt, weil R10 offen ist |
| E83 | R10 wird überall nachgezogen: Master, Risikoanalyse, Pläne P3.3, P4.1 und P4.5, VENTREX-Deck und Aalberts-Paket |

## Festlegungen des Werkzeugs (2026-10-06)

| Nr. | Festlegung |
|---|---|
| E84 | Die Anlage behält ihren Dateinamen und ihr Bezugsdatum 2026-10-02, weil sie Anlage der Antworten vom 2026-10-02 ist. Der Stand im Text steht auf 2026-10-06, und eine Revisionszeile nennt die Änderung vom 2026-10-06. Das Deck dagegen trägt den Stand im Dateinamen und wurde deshalb `sec/2026-10-06_CRA_Compliance_Praesentation.pptx`; die Vorgängerdatei ist gelöscht, Eintrag im `deletion_log.md` |
| E85 | Hinweiszeilen der Quellen tragen die Marke `<!-- note: <Anker> --> <Hinweistext>`. Der Anker steht damit in der Quelle und die Worddatei wird ohne die Hinweisliste des Werkzeugs gesetzt |
| E86 | Die Extrakte unter `extracts/` tragen wieder nur Vorlagentext. Das Werkzeug `tools/docx_extract.py` lehnt Quellen unter `filled` ab, und `tools/verify_aalberts_answers.py` prüft ausdrücklich, dass kein Extrakt Antworten oder Hinweise enthält |

## Umsetzung

| Nr. | Schritt | Ergebnis |
|---|---|---|
| 1 | Entscheidungen E78 bis E83 und die Festlegungen E84 bis E86 in diesen Plan eintragen | eingetragen |
| 2 | Master, Abschnitt 4, 6, 7 und Revisionshistorie auf R10 ziehen; Risikoanalyse, Pläne P3.3, P4.1 und P4.5 nachziehen | Master und Detaildokumente konsistent |
| 3 | `tools/measure_reference.py` auf zwei Tabellen umstellen: Maßnahmen mit getrennten Spalten `Finding` und `CRA duty`, neue Tabelle `Findings R1 to R10` | Werkzeug erzeugt die Risikoliste |
| 4 | Anlagenquelle `filled/src/VentCon2_Measures_Reference_2026-10-02.md` erzeugen und die Anlage daraus setzen | Quelle und Abgabedatei liegen vor |
| 5 | `tools/verify_measure_reference.py` erweitern: Markdown gegen Master, Worddatei gegen Markdown, Befundliste vollständig, jede Kennung aufgelöst | 503 Prüfungen |
| 6 | `tools/aalberts_fill.py` auf den Ablauf Werkzeug -> Quelle -> Wordkopie umstellen, Hinweiszeilen mit Anker auszeichnen | Werkzeug umgestellt |
| 7 | Extrakte mit `tools/docx_extract.py` neu erzeugen, Quellen in `filled/src` schreiben, beide Wordkopien setzen | Extrakte nur Vorlagentext, Quellen und Kopien neu |
| 8 | `tools/docx_extract.py` lehnt Quellen unter `filled` ab | kein Überschreiben möglich |
| 9 | `tools/verify_aalberts_answers.py` um Quelle, Hinweismarke, Kennungsauflösung und die Extraktregel erweitern | 323 Prüfungen |
| 10 | R10 im VENTREX-Deck nachziehen, Deck als `sec/2026-10-06_CRA_Compliance_Praesentation.pptx` erzeugen, Vorgänger löschen | Deck konsistent, Löschung protokolliert |
| 11 | Gegenproben: Befund ohne Kennung im Master, neuer Befund im Master, fehlende Befundzeile in der Quelle, Antwortzeile im Extrakt | alle vier werden erkannt |

## Verifikation

| Prüfung | Werkzeug | Ergebnis |
|---|---|---|
| Extrakte quellentreu, Prüfsummen der Vorlagen | `tools/verify_docx_extract.py` | 27 Prüfungen, alle bestanden |
| Antworten, Hinweise, Leerzeilenwerte, Kennungsauflösung | `tools/verify_aalberts_answers.py` | 323 Prüfungen, alle bestanden; Checkliste 31 Antworten und 4 Hinweise, Fragebogen 36 Antworten und 2 Hinweise; beide Kopien nennen die Risikoliste, 9 und 2 genannte Befunde sind in der Anlage aufgelöst |
| Anlage gegen Master und gegen ihre Quelle | `tools/verify_measure_reference.py` | 503 Prüfungen, alle bestanden; 21 Maßnahmenzeilen mit 4 aktiven und 11 ruhenden Maßnahmen, 10 Befundzeilen, Spaltensumme 6,30 in |
| Deck gegen Plan und Master | `verify_ventrex_deck.py`, `tools/audit_plan_umsetzung.py`, `tools/cra_rule_control.py` | keine Fehler, 219 Prüfungen, 8 Gegenproben ohne Fehler |
| Keine Eintragung im Extrakt, fehlende Befundzeile | Gegenprobe von Hand | beide Verifier lehnen ab, danach neu erzeugt und wieder grün |
| Idempotenz | `tools/aalberts_fill.py`, `tools/measure_reference.py` | zweiter Lauf ändert Inhalte nicht, nur das Erzeugungsdatum im Extrakt |
| Befehle | Handlauf | `"C:\venvs\VentCon2\Scripts\python.exe" tools/docx_extract.py`, `tools/measure_reference.py`, `tools/aalberts_fill.py`, danach die vier Verifier |

## Scope-Grenzen

- Keine neue Bewertung, kein Terminversprechen, keine Zusage zum Supportzeitraum (E63 gilt weiter).
- Die Aalberts-Vorlagen und ihre Prüfsummen bleiben unberührt.
- Keine Löschung ohne Dry-Run und Freigabe, Löschprotokoll in `deletion_log.md`.
- Kein venv in Dropbox, Arbeitsumgebung bleibt `C:\venvs\VentCon2`.

## Risiken und Fallstricke

| Risiko | Gegenmaßnahme |
|---|---|
| `tools/docx_extract.py` überschreibt die neue Quelle | Sperre in Schritt 9, vorher Dateinamen nach Q1 unverwechselbar |
| Markdown und Worddatei laufen auseinander | Verifier prüft beide Richtungen, Schritt 10 |
| Statusaussagen wandern aus dem Master in Markdown | Verifier vergleicht Statuswörter und Pflichtnummern mit Abschnitt 7 und 8 |
| Bewertung der Befunde R4 bis R9 wäre neue Information | Q3, Voreinstellung ohne neue Bewertung |
| Windows vergleicht Pfade ohne Rücksicht auf Groß- und Kleinschreibung | Zielnamen wörtlich vorgeben, Prüfsummenprüfung nach jedem Lauf |

## Nachtrag 2026-10-06: HTML-Reste in den Worddateien

Befund nach der Umstellung: In der Checklistenkopie stand 31 Mal, in der Fragebogenkopie 2 Mal der
Text `<br>` als sichtbares Markup in der Worddatei, jeweils am Zellenanfang vor der Antwort.

Ursache: In der Markdown-Quelle trennt `<br>` die Teile einer Zelle, den Vorlagenhinweis von der
Antwort und in der Checkliste das Urteil von der Begründung. `tools/aalberts_fill.py` gab diesen
rohen Zelleninhalt an den Setzer weiter. Vorher entstand die Worddatei aus den Antworttexten im
Werkzeug, in denen kein `<br>` steht; mit der Umstellung auf die Quelle wanderte die Marke mit.

Behebung: Die neue Funktion `answer_from_cell` in `tools/aalberts_fill.py` setzt eine Zelle der
Quelle wieder in die Form des Antworttextes um. Führende Vorlagenteile entfallen, weil die
Worddatei sie schon aus der Vorlage trägt; Urteil und Begründung werden wieder getrennt, sodass der
Setzer das Urteil fett setzt, eine echte Zeilenschaltung einfügt und die Begründung mager folgen
lässt.

Verifikation: Zwei neue Prüfgruppen in `tools/verify_aalberts_answers.py`, damit dieser Fehler nicht
zurückkommt: `check_no_html_artifacts` sucht die Marken `<br>`, `<br/>`, `&nbsp;`, `&amp;`, `&lt;`,
`&gt;`, `<p>`, `</p>`, `<b>`, `</b>`, `<div` und `<!--` in allen drei Abgabedateien, und
`check_marker_counts` vergleicht die Anzahl der `**`-Marker in der Wordkopie mit der Wordvorlage.
Ergebnis: 364 Prüfungen, alle bestanden; alle drei Abgabedateien sind frei von HTML-Resten.
Gegenprobe: ein von Hand eingefügtes `<br>` in der Fragebogenkopie wird abgelehnt, danach
neu erzeugt und wieder grün.

Die zwei verbliebenen `**` in der Checklistenkopie stehen im Anhang der Aalberts-Vorlage selbst
(`APPENDIX 1 - Product risk assessment**` und `**Notes on executing a product risk assessment:**`).
Sie sind Vorlagentext, gehören nicht zur Eintragung und bleiben deshalb stehen; die neue Prüfung
vergleicht gegen die Vorlage und schlägt dort nicht an.
