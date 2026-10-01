# Verifikation: Umsetzung des Plans "Präsentation ohne Vorwärtsverweise"

Datum: 2026-10-01
Gegenstand: `sec/2026-10-01_CRA_Compliance_Praesentation.pptx` (28 Folien), Generator
`build_ventrex_deck.py`, Verifier `verify_ventrex_deck.py`, Plan
`sec/plan/2026-09-30_VENTREX_Praesentation_Plan.md` (Entscheidungen E32 bis E34, Schritt U13)

## Prüfmittel

| Mittel | Art | Ergebnis |
|---|---|---|
| `tools/audit_plan_umsetzung.py` | liest die Zielbild-Tabelle des Plans als Sollvorgabe und vergleicht sie mit dem Deck, unabhängig vom Verifier | 219 Prüfungen, alle bestanden |
| `verify_ventrex_deck.py` | bestehende Prüfkette des Projekts, erweitert um die Regel kein Vorwärtsverweis | keine Fehler, keine Hinweise |
| `tools/forward_ref_control.py` | Negativkontrolle mit absichtlich eingebauten Verstößen | beide Verstöße gemeldet, Kontrolldatei danach gelöscht |
| PowerPoint-COM-Export | Sichtprüfung der Folien 2, 4, 5, 6, 7, 9, 10, 12, 13, 14, 15, 16, 17, 19, 23, 25, 26, 27 | keine Überläufe, Kopfzeilen und Unterzeilen korrekt |
| `git hash-object` gegen `HEAD` | Unverändertheit der Vorlage | `b3b374fe5cd3c77f605aa1e2efca0a9a4f441615`, identisch zu HEAD |

## Ergebnis je Planpunkt

| Planpunkt | Status | Beleg |
|---|---|---|
| V1 Abschnittsverweis auf Abschnitt 5 entfernt | erfüllt | Wendung "aus Abschnitt 5 weiter" im Deck nicht mehr vorhanden, audit geprüft |
| V2 Kennungen und Verweis in 1.2 entfernt, Satz selbsttragend | erfüllt | Folie 5 nennt Angriffspunkt und Zweck der Maßnahmen im Klartext |
| V3 Verweis "siehe 2.2 und 2.3" entfernt | erfüllt | Folie 7 endet mit "die RED-Anforderungen enden am 2027-12-11" |
| V4 "die Folgen stehen auf 7.1" und "Entscheidung in P1.5" entfernt | erfüllt | Folie 12 sagt "der Nachweisweg ist noch zu entscheiden, siehe 3.1" (rückwärts) |
| V5 Pflichtentafeln behalten Risiko- und Maßnahmenspalte | erfüllt | Spalten unverändert, Kennungen stehen jetzt nach Risikobild und Maßnahmentafeln |
| V6 Maßnahmenspalte des Risikobilds ohne Kennungen | erfüllt, anders als empfohlen | siehe Abweichung 1 |
| V7 Register verweist auf Maßnahmenbereiche | erfüllt | Folie 23 nennt "P1.1 bis P5.2", Erklärung ab Folie 15 |
| Zielreihenfolge 1 bis 28 | erfüllt | audit vergleicht Reihenfolge, Layout und Nummer jeder Folie gegen die Zielbild-Tabelle |
| Inhaltsverzeichnis mit neuen Nummern | erfüllt | 28 Einträge, jede Nummer und jedes erste Titelwort geprüft |
| Dividerunterzeile sichtbar | erfüllt | alle sieben Divider tragen eine Unterzeile, drei davon über das neue Textfeld |
| Roter Faden neu aufgebaut | teilweise | siehe Abweichung 3 |
| Verifier-Regel kein Vorwärtsverweis | erfüllt | Regel greift, Negativkontrolle bestanden |
| Nummernfolge und Kapiteltexte im Verifier nachgezogen | erfüllt | Folge 1, 1.1, 1.2, 2, 2.1 bis 4.4, 5.1 bis 5.3, 6.1 bis 6.3, 7, 7.1 |
| PNG-Sichtprüfung der geänderten Folien | erfüllt | Folien 4, 5, 7, 12, 14, 15, 17, 23, 25, 27 und Inhaltsverzeichnis gesichtet, dazu weitere Berührstellen |
| Tabellenbindigkeit und Überlaufprüfung | erfüllt | Verifier Abschnitt 9 ohne Hinweis |
| Präsentationsplan und Repo-Gedächtnis nachgezogen | erfüllt | E32 bis E34, Schritt U13, Nachtrag und drei Prüfzeilen im Plan, Repo-Notizen aktualisiert |
| Master, `src/`, `LaTeX/`, Normextraktion unberührt | erfüllt | Änderungen in `sec/sources/**` und `tools/cra_extract.py` stammen aus der Umlautumstellung vom selben Tag, nicht aus diesem Plan |

## Abweichungen

1. **Reihenfolge im Abschnitt Risiken und Maßnahmen.** Der Plan empfahl Variante A (Risikobild
   zuerst, Maßnahmenspalte entfällt). Umgesetzt ist Variante C: das Risikobild steht zuerst und
   behält die Spalte "Maßnahme", nennt darin aber die Gegenmaßnahme im Klartext statt der Kennung
   ("Anmeldung und Sitzungen", "Kennwort und Erstinbetriebnahme", "Kalibriergrenzen",
   "POST und CSRF-Schutz", "Angriffsfläche", "PID-Obergrenzen"). Grund: die Zuordnung Risiko zu
   Maßnahme bleibt schon im Einstieg des Abschnitts sichtbar, ohne dass eine Kennung vor ihrer
   Erklärung steht. Variante A und B bleiben möglich; die Wahl ist in E32 dokumentiert und im
   letzten Bericht offengelegt, in der Planempfehlung aber nicht so vorgesehen.
2. **Zwei Folgeänderungen, die der Plan nicht ausdrücklich nennt.** Auf Folie 9 (2.3 Prototypen)
   entfällt die Kennungsliste im CRA-Status ("Maßnahmen P1.1 bis P1.3 und P2.2 laufen weiter" wird
   zu "Die Härtungsmaßnahmen laufen weiter"), sonst würde die Planregel 2 dort verletzt. Die
   Unterzeile des Dividers 3 nennt "RED-Cyber gilt bis 2027-12-11", weil Unterzeilen jetzt sichtbar
   sind und die bestehende Verifier-Regel einen Gültigkeitsvermerk verlangt.
3. **Roter Faden nur teilweise umgesetzt.** Der Plan verlangt, dass jeder Abschnittskopf den
   vorigen Faden aufnimmt, entweder in der Dividerunterzeile oder als erster Punkt der Folie. Die
   sieben Unterzeilen sind vorhanden, benennen aber den Abschnittsinhalt ("Produkt, Einsatz und die
   Fälle rund um den Prototypen", "Pflichten 1 bis 26 aus Annex I, Art. 13, Art. 14 und Art. 28").
   Einen ausdrücklichen Übergang gibt es nur auf Folie 12 ("Anschluss an 2.2 und 3.1") und Folie 27
   ("Anschluss: Die Pflichten 1 bis 26 stehen in 5.1 bis 5.3, der Umsetzungsstand in 4.2 bis 4.4").
   Offener Vorschlag: die Unterzeilen der Divider 2, 4, 5 und 6 als Übergang formulieren
   ("Aus dem Aufbau folgt die Frage der Anwendbarkeit: ...", "Aus dem Regelwerk folgen die Befunde:
   ...", "Aus den Maßnahmen folgen die Pflichten: ...", "Belege zu den Pflichten: ...").
4. **Zwei Zeilen der Zielbild-Tabelle im Plan waren veraltet.** Der audit hat sie gefunden und sie
   sind korrigiert: Zeile 11 beschrieb die Folie als "Vergleichstabelle" statt mit ihrem Titel
   ("3.1 RED und CRA: zwei Regime, ein Übergang"), Zeile 9 nannte "fünf Fälle", die Folie trägt
   sechs. Reine Dokumentationskorrektur, keine Deckänderung.

## Grenzen der Prüfung

- Die Verweissuche ist heuristisch: eine Nummer gilt nur dann als Verweis, wenn im Fenster davor ein
  Verweiswort steht (siehe, nach, aus, in, auf, an, Abschnitt, Kapitel). Eine Wendung ohne solches
  Wort, zum Beispiel "Die Erläuterung folgt weiter hinten", wird nicht erkannt.
- Nummern aus Zitaten sind ausgenommen: "Art. 7", "Nr. 7", "Teil I, (2)(d)", "EN 303 645 5.1-1",
  "Abschnitt 9.1" der Kommissionsleitlinie (auf Folie 24 tatsächlich verwendet).
- Die Kennungsregel ist an feste Foliennummern gebunden (R ab 14, P1 bis P3 ab 15, P4 und P5 ab 16).
  Wird die Reihenfolge erneut geändert, müssen diese Schwellen mitgezogen werden, sonst meldet der
  Verifier zu Recht oder zu Unrecht.
- Geprüft ist die technische Umsetzung. Ob die neue Reihenfolge auch inhaltlich überzeugt, bleibt
  eine redaktionelle Entscheidung.
