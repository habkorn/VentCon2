# Plan: Umfassende Anpassung zur Abgrenzung RED gegen CRA

Datum: 2026-09-30
Rolle: Umsetzungsplan (Detaildokument, kein CRA-Bezug)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: keine (Dokumentenrahmen und Praesentation)
Ausfuehrungsstand: umgesetzt und verifiziert am 2026-09-30
Gueltigkeit: jede Aussage mit RED-Bezug in diesem Plan gilt bis 2027-12-11, sofern sie nicht
ausdruecklich anders datiert ist.

## Auftrag

RED und CRA deutlich trennen: befristetes RED-Cyber-Regime, dauerhaftes CRA-Regime und technische
Normen ohne Rechtswirkung. Betroffen sind alle Dateien in `sec/`, die dazu Aussagen treffen, sowie
die Praesentation.

## Entscheidungen (2026-09-30)

| Nr. | Entscheidung |
|---|---|
| E1 | Umfang: alle betroffenen Dateien in `sec/` |
| E2 | Master bekommt einen eigenen Rahmenabschnitt plus einen RED-Pflichtenblock |
| E3 | Neues Arbeitspaket P1.5 mit eigenem Skelett-Dokument |
| E4 | Praesentation: eine Gegenueberstellungsfolie mit zwei Spalten |
| E5 | Gueltigkeitsvermerk an jeder RED-Aussage |
| E6 | EN 303 645 wird als technischer Massstab gekennzeichnet |

## Abgrenzungsmodell

| Ebene | Rechtsgrundlage | Wirkung | Zeitraum |
|---|---|---|---|
| RED-Cyber | Art. 3(3)(d)(e)(f) RED 2014/53/EU, ausgestaltet durch Delegierte VO (EU) 2022/30 i.d.F. (EU) 2023/2444 | verbindlich fuer internetfaehige Funkgeraete, Konformitaetsvermutung ueber EN 18031 | seit 2025-08-01, endet am 2027-12-11 |
| CRA | VO (EU) 2024/2847, Annex I, Art. 13, Art. 14, Art. 28 | verbindlich und unbefristet, gestaffelt | Art. 14 seit 2026-09-11, voll ab 2027-12-11 |
| Technische Normen | EN 303 645 (ETSI-Baseline), EN 18031-1/-2/-3 (harmonisiert unter RED) | EN 303 645 ohne Rechtswirkung; EN 18031 gibt RED-Vermutung, mit Restriktionen bei den Passwortklauseln 6.2.5.1 und 6.2.5.2 | EN 18031 seit 2025-01-30 im Amtsblatt zitiert |

Fuenf Kernaussagen, die ueberall gleich lauten:

1. RED-Cyber ist derzeit der verbindliche Nachweisweg, nicht redundant.
2. Am 2027-12-11 wird die Delegierte VO (EU) 2022/30 durch Delegierte VO (EU) 2026/339 aufgehoben,
   die Cyber-Anforderungen gehen in die CRA ueber.
3. Die CRA-Meldepflichten gelten schon jetzt, parallel zur RED-Route.
4. EN 303 645 ist technischer Massstab, keine Rechtsgrundlage. Eine CRA-Konformitaetsvermutung nach
   Art. 27 gibt es nicht, weil keine CRA-Norm zitiert ist.
5. Das CE-Zeichen stammt heute aus dem RED, ab 2027 aus der CRA. Beide Nachweise bleiben getrennt.

## Zielstruktur des Masters

| Nr. | Abschnitt | Aenderung |
|---|---|---|
| 1 | Regulatorischer Rahmen: RED und CRA | neu, mit Gegenueberstellung und fuenf Kernaussagen |
| 2 | Anwendbarkeit | auf VentCon2 reduziert, RED-Zeilen entfallen hier |
| 3 | RED-Pflichten | neu, RED-1 bis RED-3, gilt bis 2027-12-11 |
| 4 | Dokumentenregister | um P1.5 ergaenzt |
| 5 | Produktklassifizierung | RED-Vorbehalt wird zum Verweis auf Abschnitt 1 |
| 6 | Pflichten-Checkliste | CRA-Pflichten unveraendert 1 bis 26, EN 303 645 gekennzeichnet |
| 7 | Verknuepfung | um P1.5 ergaenzt |
| 8 | Arbeitspakete | P1.5 in Phase 1 |
| 9 | Zeitplan | um EN-18031-Zitierung ergaenzt, RED-Zeile praezisiert |
| 10 | Strafen | Hinweis auf unterschiedlichen Sanktionsrahmen RED und CRA |
| 11 | Quellen | EN 18031 als RED-Norm gekennzeichnet |
| 12 | Revisionshistorie | neuer Eintrag |

## RED-Pflichtenblock

| ID | Anforderung | Rechtsgrundlage | Norm | Massnahme |
|---|---|---|---|---|
| RED-1 | Netzwerkschutz | RED Art. 3(3)(d) | EN 18031-1 | P1.5 Nachweis, P3.3 Funkebene |
| RED-2 | Schutz personenbezogener Daten und Privatsphaere | RED Art. 3(3)(e) | EN 18031-2 | P1.5 Nachweis, P2.2 Passwort |
| RED-3 | Betrugsschutz | RED Art. 3(3)(f) | EN 18031-3 | voraussichtlich nicht einschlaegig, n/a begruenden |

Folgezeile: Werden die ausgenommenen Passwortklauseln beruehrt, gilt keine Selbstbewertung, sondern
Modul B+C oder Modul H mit notifizierter Stelle. Fuer VentCon2 mit festem Standardpasswort ist das
der Regelfall, siehe P2.2.

## Betroffene Dateien

| Datei | Aenderung |
|---|---|
| `../CRA_Compliance_Checklist.md` | Zielstruktur siehe oben |
| `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` | EN-303-645-Bezug als technischen Massstab kennzeichnen |
| `../grundlagen/2026-08-29_Web_Auth_How_It_Works.md` | dito |
| `2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md` | vier Bezuege kennzeichnen |
| `2026-08-31_P2-2_AP_Password_Plan.md` | sechs Bezuege kennzeichnen, Querverweis P1.5 |
| `2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md` | RED-Abschnitt erweitern, getrennte Nachweise |
| `2026-09-28_P1-5_RED_Conformity_Route_Plan.md` | neu, Skelett |
| `2026-09-30_VENTREX_Praesentation_Plan.md` | Folienliste anpassen |
| `build_ventrex_deck.py` und die PPTX | Folienumbau, neu erzeugen, neu rendern |

Nicht betroffen: P5.1, P1.4, P2.3, P3.3, P4.1 bis P4.6, P4.8, P5.2 und die Mutex-Grundlagen.

## Praesentation

| Nr. | Aenderung |
|---|---|
| 3 | Anwendbarkeit auf VentCon2 reduzieren, RED-CRA-Verhaeltnis hier entfernen |
| 4 | Ersetzt durch "RED und CRA: zwei Regime, ein Uebergang", Gegenueberstellung in zwei Spalten, darunter RED-1 bis RED-3 |
| 5 | RED-Vorbehalt entfaellt hier, stattdessen Verweis auf die Gegenueberstellung |
| 15 | Zeitplan um EN-18031-Zitierung und RED-Aufhebung ergaenzen |

Folienzahl bleibt 20.

## Phasen

| Phase | Inhalt |
|---|---|
| A | Master: Abschnitte 1 bis 3 neu, Renummerierung 4 bis 12, Gueltigkeitsvermerke, EN-303-645-Kennzeichnung, Register, Verknuepfung, Arbeitspakete, Zeitplan, Strafen, Quellen, Revision |
| B | Detaildokumente angleichen (Analyse, Grundlagen, P1.1, P2.2, P4.7) |
| C | Skelett `2026-09-28_P1-5_RED_Conformity_Route_Plan.md` anlegen |
| D | Praesentation umbauen, neu erzeugen, neu rendern, Sichtpruefung der Folien 3, 4, 5, 15 |
| E | Verifikation |

## Verifikation

| Pruefung | Kriterium |
|---|---|
| Gueltigkeitsvermerke | jede RED-Aussage in `sec/` traegt "2027-12-11" oder den Vermerk |
| EN 303 645 | keine Stelle behauptet eine Rechtsgrundlage, Kennzeichnung als technischer Massstab vorhanden |
| ID-Kollision | RED-1 bis RED-3 gegen R1 bis R9 und P1.1 bis P5.2 kollisionsfrei |
| CRA-Nummerierung | unveraendert 1 bis 26 |
| Verweise | alle Verweise auf Master-Abschnitte zeigen auf die neuen Nummern |
| Verifier | zwei neue Pruefungen im Deck: RED-Aussage ohne Gueltigkeitsvermerk, EN 303 645 ohne Kennzeichnung |
| Deck | 20 Folien, bisherige Pruefungen weiter gruen |
| Sichtpruefung | Folien 3, 4, 5, 15 im PNG-Export begutachtet |

## Hinweise

- Abschnittsnummern verschieben sich. Alle Verweise auf Master-Abschnitte in den Detaildokumenten
  werden gegengeprueft.
- PowerPoint darf beim Rendern nicht laufen, sonst haengt sich der COM-Export an die Nutzersitzung.

## Ergebnis der Umsetzung (2026-09-30)

| Phase | Stand | Bemerkung |
|---|---|---|
| A | erledigt | Master mit 12 Abschnitten, Rahmenabschnitt und RED-Pflichten RED-1 bis RED-3 |
| B | erledigt | Analyse, Web-Auth-Grundlagen, P1.1, P2.2 und P4.7 angeglichen |
| C | erledigt | Skelett P1.5 mit verifiziertem Ist-Zustand angelegt |
| D | erledigt | Folien 3, 4, 5, 15, 16 und 19 angepasst, 20 Folien, Sichtpruefung wiederholt |
| E | erledigt | Tabellen, Gueltigkeitsvermerke, Kennzeichnungen und Verweise geprueft |

Abweichungen vom Plan:

- Folie 4 nutzt eine Spalte mit vier Punkten statt zwei Spalten, weil das VENTREX-Layout
  CONTENT A_V2 zwei Textspalten gleicher Breite vorgibt und die Gegenueberstellung dort als
  Punktpaar klarer lesbar ist. RED-1 bis RED-3 stehen im zweiten Punkt.
- Zusaetzlich angepasst: Folie 16 um den getrennten Sanktionsrahmen der Funkrichtlinie, Folie 19
  um die korrigierte Zahl der Detaildokumente.
- Ueber den Plan hinaus wurden die Querverweise auf Master-Abschnitte in P4.1, P4.4, P4.6 und P4.8
  sowie im Plan zur Volltextextraktion korrigiert, weil die Renummerierung sie ungueltig gemacht hat.
- Der Strukturplan vom 2026-09-28 erhielt einen Hinweis, dass seine Abschnittsnummern den Stand
  vor der Renummerierung beschreiben.
