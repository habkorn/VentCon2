# Plan: CRA-Präsentation auf die VENTREX-Vorlage umstellen

Datum: 2026-09-30
Rolle: Umsetzungsplan (Detaildokument, kein CRA-Bezug)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: keine (Dokumentenaufbereitung)
Ausführungsstand: umgesetzt und verifiziert am 2026-09-30
Ergebnis: 20 Folien, 6,84 MB. Struktur- und Inhaltsprüfung ohne Befund (26 Pflicht-Nummern,
20 Maßnahmen-IDs, Fußzeile und Datum 15 von 15, Foliennummernfeld 15 von 15, nur Markenschriften
und Markenfarben, keine Vorlagenreste). Sichtprüfung per PowerPoint-PNG-Export durchgeführt:
Cover, Divider, Tabellenfolien, zweispaltige Folie, Dunkelfolie und Abschlussfolie geprüft.
Eine Korrektur war nötig: die beiden Spalten von CONTENT B_V1 liegen nur 11 EMU auseinander, deshalb
sortiert der Generator die Textplatzhalter mit einer Toleranz von 0,05 Zoll nach Lage. Ohne die
Toleranz waren die Spalten von Folie 5 und 18 vertauscht.

Nachtrag 2026-09-30, Trennung von RED und CRA im Master: Folien 3, 4, 5, 15, 16 und 19 angepasst,
Folienzahl bleibt 20. Der Verifier prüft jetzt 21 Maßnahmen-IDs und zwei zusätzliche Regeln:
eine RED-Aussage braucht den Gültigkeitsvermerk bis 2027-12-11, und EN 303 645 ist als technischer
Maßstab zu kennzeichnen. Die Sichtprüfung wurde für die geänderten Folien wiederholt.

Nachtrag 2026-09-30, Geltungsbereich: Folie 3 hat acht Zeilen, neu sind der Vertrieb nur in
Drittländern (Beispiel Indien), die unentgeltliche Lieferung an EU-Kunden, der Weiterverkauf durch
EU-Importeur oder Händler und der Vertreter nach Art. 18. Folie 5 nennt den gemeinsamen
Anknüpfungspunkt Unionsmarkt und führt sechs Punkte, darunter der Drittland-Vertrieb und der
EU-Importeur nach Art. 19 bis Art. 21.

Nachtrag 2026-10-01, Prototypen: neue Folie 4 "Prototypen und Vorserien" mit fünf Fallzeilen und dem
Hinweistext in zwei Sprachen. Das Deck hat damit 21 Folien, die bisherigen Folien 4 bis 20 sind um eins
nach hinten gerückt. Der Verifier erwartet nun 21 Folien. Quelle ist Abschnitt 2.1 des Masters; der
Master-Status steht auf zurückgestellt für den Unionsmarkt, Prototypenbetrieb mit geplantem Vertrieb
Indien.

Nachtrag 2026-10-01, Stand und aktiver Strang: Datei, Footer und Datum auf den 01.10.2026 umgestellt,
das Deck liegt jetzt als `sec/2026-10-01_CRA_Compliance_Praesentation.pptx`. Folie 4 hat eine sechste
Zeile für den eigenen Prototyp am Netz. Die Statusfolien zeigen P1.1, P1.2, P1.3 und P2.2 als `läuft`,
P1.5 sowie P5.1 und P5.2 als `ruht`; die Revisionsfolie nennt den laufenden und den ruhenden Teil.

Nachtrag 2026-10-01, Umstrukturierung nach Analyse: Das Deck umfasst jetzt 25 Folien. Neu sind der Überblick nach dem
Cover, die Folie Produkt und Vorhaben, das Risikobild R1 bis R6 und die Folie zum aktiven Strang. Das
Dokumentenregister wechselt von Abschnitt 1 in Abschnitt 3, alle fünf Divider tragen eine Unterzeile.
Die Regel- und die Klassifizierungsfolie wurden entlastet, die Folie 24 heißt jetzt Status und
Revision und nennt den nächsten Schritt. Der Verifier prüft 25 Folien, die Datums-Whitelist, den
Pflichtbegriff zurückgestellt sowie den Abgleich der Quellenliste gegen Master-Abschnitt 11. Generator
und Verifier akzeptieren einen abweichenden Zieldatei-Pfad, damit ein Kandidat geprüft werden kann,
während PowerPoint die Zieldatei geöffnet hat.

## Auftrag

Die Präsentation aus `sec/CRA_Compliance_Checklist.md` wird aus der Firmenvorlage
`sec/VENTREX PPT Vorlage 2026.pptx` erzeugt: Vorlage öffnen, Beispielfolien entfernen,
Folien über die Layouts der Vorlage anlegen, Platzhalter füllen.

## Entscheidungen (2026-09-30)

| Nr. | Entscheidung |
|---|---|
| E1 | Alte Präsentation ersetzen und umbenennen: neu `sec/2026-09-30_CRA_Compliance_Praesentation.pptx`, Altdatei vom 2026-09-29 entfernen. Seit 2026-10-01 lautet der Name `sec/2026-10-01_CRA_Compliance_Praesentation.pptx`, die Fassung vom 30.09. ist gelöscht (Eintrag im Löschprotokoll) |
| E2 | Alle 28 Beispielfolien der Vorlage entfernen |
| E3 | Fünf Kapitel-Divider auf den SUBSECTION-Layouts |
| E4 | Tabellen beibehalten, im VENTREX-Look (Kopfzeile #002F87, Theme-Schriften, Statusspalte farbig) |
| E5 | Fußzeile: Name "VentCon2 - EU CRA Compliance", Datum "30.09.2026" (seit 2026-10-01 "01.10.2026"), Foliennummer als Feld |
| E6 | Inhalt gestrafft, 20 Folien |
| E7 | Verknüpfungsfolie in die Statusfolien integriert (Streichung der separaten Folie) |
| E8 | Sichtprüfung per PowerPoint-COM mit PNG-Export |
| E9 | Trennung RED und CRA im Master am 2026-09-30 in die Folien übernommen, Folienzahl bleibt 20 |
| E10 | Prototypenregel aus Master-Abschnitt 2.1 als eigene Folie, Deck wächst auf 21 Folien |
| E11 | Inhaltsverzeichnis nach dem Cover: zweispaltige Tabelle mit allen Foliennummern und Titeln, darunter die Statuszeile. Die Abschnittszeilen sind in Markenblau mit weißer Schrift hervorgehoben |
| E12 | Dokumentenregister wechselt in Abschnitt 3, damit Abschnitt 1 entlastet wird |
| E13 | Neue Inhaltsfolien: Produkt und Vorhaben, Risikobild R1 bis R6, Aktiver Strang |
| E14 | Verifier um Datums-Whitelist, Pflichtbegriff und Quellenabgleich erweitert |
| E15 | Generator und Verifier akzeptieren einen alternativen Zieldatei-Pfad für die Kandidatenprüfung |
| E16 | Mehrstufige Nummerierung: Divider tragen die Abschnittsnummer (1 bis 5), die Inhaltsfolien die Untergliederung (1.1 bis 5.2), Cover, Inhaltsverzeichnis und Abschluss bleiben unnummeriert. Die Nummer steht im Folientitel, im Inhaltsverzeichnis und in Querverweisen |
| E17 | Kopfzeile auf den Unterfolien: links oberhalb des Titels steht der Kapiteltitel mit Nummer, 10 pt, fett, in Markenblau, auf Dunkelfolien in Weiß. Divider, Cover, Inhaltsverzeichnis und Abschluss bleiben ohne Kopfzeile |
| E18 | Normzitate werden gegen die Extraktion in `sources/` geprüft und korrigiert: Annex I Teil I nur in der Form (2)(a) bis (m), Teil II Nr. 1 bis 8, CE-Kennzeichnung Art. 29 und 30, Konformitätserklärung Art. 28, technische Dokumentation Art. 31. Pflichten ohne 1:1-Punkt werden als Projektauslegung gekennzeichnet |
| E19 | Deutsche Texte im Dossier, in den beiden Skripten und in `deletion_log.md` werden mit echten Umlauten geschrieben (ä, ö, ü, ß). Transliteration bleibt auf Dateinamen und Bezeichner beschränkt, deshalb behalten Generator, Verifier und alle Pfadangaben ihre ASCII-Namen (E19a: die Projektregel steht in `.github/copilot-instructions.md`) |
| E20 | Umstellung in zwei Durchgängen: erst eine Stammliste (979 Stellen), dann Wortstämme für den Rest (198 Stellen), zusammen 1.177 Stellen in 49 Dateien. Der englische Normtext der CRA-Extrakte bleibt wortgetreu, geändert werden dort nur die Extraktionskommentare. Ein Regeldefekt (`Zusaetz`) beschädigte drei Stellen, alle korrigiert. Keine Prüfregel im Verifier, die Umstellung ist eine einmalige Aktion |
| E21 | Der leere Bildplatzhalter der Divider-Layouts wird auf den fünf Divider-Folien entfernt. Er zeigt im PNG- und PDF-Export einen grauen Kasten mit Bildsymbol, obwohl die Divider kein Bild brauchen. Die Titelplatzhalter bleiben unberührt, die Markenbilder der Layouts werden weiter gerendert |
| E22 | Em-Dashes (U+2014) werden nach der Projektregel ersetzt: 29 im englischen Grundlagenpapier zum Mutex, zwei im AP-Passwort-Plan, sieben in `.github/copilot-instructions.md`. In Überschriften steht ein Doppelpunkt, im fortlaufenden Text ein Komma |
| E23 | Neuer Abschnitt Systembeschreibung vor Abschnitt 1: Divider mit Unterzeile sowie zwei Folien mit den Abbildungen Systemübersicht und Bedienoberfläche aus dem Benutzerhandbuch. Der Abschnitt trägt keine Nummer, damit die Nummerierung 1 bis 5 und alle Querverweise unverändert bleiben. Seine zwei Folien tragen den Abschnittsnamen als Kapitelverweis |
| E24 | Bildwandlung: `tools/pdf_to_png.py` rendert die PDF-Abbildungen aus `LaTeX/manual/pic` nach `sec/pic` (lange Kante 1600 Pixel, höchstens 400 dpi). Der Generator setzt sie über `add_figure` in den Bildbereich des Layouts CONTENT E, unbeschnitten und mit Quellenzeile. Der leere Bildplatzhalter des Layouts wird vorher entfernt |
| E25 | Folie 10 trägt statt der Punktliste eine Vergleichstabelle Merkmal gegen RED-Cyber und CRA mit fünf Zeilen (Rechtsgrundlage, Geltungsbereich, Inhalt der Anforderungen, Pflichten im Betrieb, Zeitplan). Inhalt aus Master-Abschnitt 1, damit beide Dokumente dieselben Aussagen tragen. Die Aussage RED-1 und RED-2 einschlägig, RED-3 nicht bleibt in der Zeile Inhalt der Anforderungen erhalten |
| E26 | Folie 11 trägt den Anschluss an 1.2 und 1.4: je fünf Punkte zu Kategorie, Nachweisweg, RED-Vorbehalt und Normenlage |
| E27 | Kapitel 1.5 wird auf zwei Folien erweitert: 1.5 zeigt die Erkenntnisse (Kategorie, Konformitätsbewertung, Normenlage), 1.6 als dunkle Akzentfolie zeigt acht Folgen für VENTREX (Nachweisweg, Unterlagen, Fristen, Normenlage, laufende Maßnahmen, Marktstatus, drei offene Entscheidungen, Anschluss an Abschnitt 2 und 3) |
| E28 | Abschnitt Fristen und Sanktionen entfällt aus der Präsentation, damit entfallen die Folien 4.1 Zeitplan der Fristen und 4.2 Sanktionsrahmen. Zeitplan und Strafrahmen bleiben im Master in den Abschnitten 9 und 10 |
| E29 | Neue Struktur mit sieben nummerierten Abschnitten: 1 Systembeschreibung (1.1 Systemübersicht, 1.2 Bedienoberfläche), 2 Produkt und Anwendbarkeit (2.1 bis 2.3), 3 Regelwerk (3.1 RED und CRA, 3.2 Klassifizierung und Normenlage), 4 Pflichten der CRA (4.1 bis 4.3), 5 Verknüpfung und Status (5.1 Risikobild, 5.2 Dokumentenregister, 5.3 bis 5.5 Status), 6 Quellen und Revision (6.1, 6.2), 7 Fazit und nächste Schritte (7.1). Die Systembeschreibung ist damit kein Sonderfall mehr, das Deck hat durchgehend dieselbe Nummerierungslogik |
| E30 | Die frühere Folie 1.6 wird zu 7.1 Was folgt daraus für VENTREX und steht als Fazit vor dem Abschluss. Sie nutzt das Standardlayout CONTENT A_V1 statt der dunklen Akzentfolie. Der Punkt Was heute schon läuft nennt den aktiven Strang in 5.5 und den Einzelstand in 5.3 direkt |
| E31 | Im Abschnitt 5 steht das Risikobild vor dem Dokumentenregister, damit die Substanz vor der Ablage kommt |
| E32 | Keine Vorwärtsverweise: das Deck darf nur auf früher Gezeigte verweisen. Aufgelöst wird das durch Umsortierung, nicht durch Streichen. Neue Reihenfolge: 1 Systembeschreibung, 2 Produkt und Anwendbarkeit, 3 Regelwerk, 4 Risiken und Maßnahmen (4.1 Risikobild, 4.2 und 4.3 Maßnahmen mit Status, 4.4 Aktiver Strang), 5 Pflichten der CRA (5.1 bis 5.3), 6 Dokumente und Quellen (6.1 Dokumentenregister, 6.2 Quellen, 6.3 Status und Revision), 7 Fazit und nächste Schritte (7.1). Begründung: das Regelwerk setzt den Rahmen, Risiken und Maßnahmen liefern die Kennungen R1 bis R9 und P1.1 bis P5.2, die die Pflichtentafeln und das Register zitieren, das Register gehört zu den Nachweisen |
| E33 | Divider tragen ihre Unterzeile sichtbar. Die Layouts SUBSECTION 1 bis 3 haben keinen Textplatzhalter, deshalb setzt `build_divider` die Unterzeile als Textfeld unter den Titel. Jeder Abschnitt bringt damit seinen roten Faden mit, und der Aufbau ist auch beim Durchblättern erkennbar |
| E34 | Der Verifier prüft Vorwärtsverweise: Verweise auf spätere Folien oder Abschnitte sind ein Fehler, Kennungen R1 bis R9 dürfen erst ab Folie 14, P1.1 bis P3.3 ab Folie 15, P4.1 bis P5.2 ab Folie 16 stehen. Ausgenommen sind das Inhaltsverzeichnis und Zitate aus Normen und Leitlinien. Negativkontrolle über `tools/forward_ref_control.py`: eine Kontrolldatei mit absichtlich eingebautem Vorwärtsverweis wird erkannt und abgelehnt |
| E40 | Abschnitt 2 und Abschnitt 3 tauschen die Plätze: erst das Regelwerk (2.1 RED und CRA, 2.2 Produktklassifizierung und Normenlage), dann Produkt und Anwendbarkeit (3.1 Produkt und Vorhaben, 3.2 Anwendbarkeit, 3.3 Prototypen und Vorserien). Begründung: das Regelwerk setzt den Rahmen, die Einordnung des Produkts folgt daraus. Damit der Tausch ohne Vorwärtsverweis trägt, behauptet die Klassifizierungsfolge die Anwendbarkeit nicht mehr vorweg: ihr erster Punkt heißt jetzt "Anschluss an 2.1" und leitet aus den zwei Regimen die Kategorien und Normen ab. Die Marktfolgen bleiben in 7.1 und verweisen auf 3.2 |
| E41 | Roter Faden: die Unterzeile jedes Dividers nach dem ersten nimmt den vorigen Abschnitt auf und übergibt an den eigenen ("Aus dem Aufbau folgt ...", "Aus dem Regelwerk folgt ...", "Aus der Einordnung folgen ...", "Aus den Maßnahmen folgen ...", "Belege zu den Pflichten ...", "Was aus den Erkenntnissen ..."). Zusätzlich trägt der erste Inhaltsteil des Regelwerks einen Übergangssatz und die erste Folie von Abschnitt 3 einen Anschlusspunkt. Der Verifier prüft, dass jeder Divider nach dem ersten eine Übergangsmarke enthält |
| E42 | Auslöser der Maßnahmen wird sichtbar gemacht: Die Pflichtmaßnahmen sind mit der Bereitstellung auf dem Unionsmarkt umzusetzen, bei Vertrieb ausschließlich in Drittstaaten bleiben sie freiwillig. Belegt durch Art. 2(1), Art. 3(21) und (22) und Recital 15 ("applies to economic operators only in relation to products with digital elements made available on the market, hence supplied for distribution or use on the Union market in the course of a commercial activity"); die unentgeltliche Lieferung in die Union zählt ebenfalls, ebenso die Bereitstellung durch Importeur oder Händler. Umgesetzt auf Folie 11 (Fußzeile mit dem Auslöser), auf Folien 15 und 16 (graue Zeile unter der Tafel, Höhe aus der Tafelhöhe berechnet) und auf Folie 27 (Was der Marktstatus bedeutet). Im Master steht die Aussage in Abschnitt 8. Verifier-Regel V7 prüft den Hinweis in Deck und Master, Gegenprobe in `tools/cra_rule_control.py` |

## Befunde zur Vorlage (Discovery)

- Format 13,333 x 7,5 Zoll, 8,37 MB, 72 Bilder mit zusammen 7,8 MB.
- 7 Master mit 31 Layouts: COVER, SUBSECTION 1 bis 5, CONTENT A_V1 bis F (in Master 2 und 6),
  CONTENTDARK 1 und 2, END-01/02, CLOSING SLIDE.
- Alle Beispielfolien mit CONTENT-Layouts nutzen Master 2, Divider Master 1, Dunkelfolien Master 3,
  Cover Master 0, Abschluss Master 5. Master 6 ist ein Namensduplikat und bleibt ungenutzt.
- Markenschriften Clash Display (Haupt) und Satoshi (Text) aus Master 0, 2 und 6. Die Divider-Layouts
  setzen Clash Display 44 pt direkt im Layout, obwohl Master 1 als Theme Calibri trägt.
- Markenfarben #002F87, #32C09D, #E96D5A, #535863, #857C48, #EBEBEB.
- Platzhalterrollen wechseln je Layout: in CONTENT A_V1 ist ID 14 das Datum, in CONTENT A_V2 ist
  ID 14 der Untertitel. Deshalb Auflösung über `placeholder_format.type`, nie über feste IDs.
- Tabellen sind in der Vorlage Handarbeit außerhalb der Platzhalter, Kopfzeile in #002F87.
- Der Foliennummern-Platzhalter darf nicht mit Text gefüllt werden, sonst geht das Feld
  `<a:fld type="slidenum">` verloren.

## Zielbild: 28 Folien in sieben Abschnitten

| Nr. | Layout | Inhalt |
|---|---|---|
| 1 | COVER | Titel, Untertitel, Stand und Quelle |
| 2 | CONTENT A_V1 | Inhaltsverzeichnis, alle 28 Folien zweispaltig, mit Statuszeile |
| 3 | SUBSECTION 4 | Divider 1 Systembeschreibung |
| 4 | CONTENT E | 1.1 Systemübersicht mit Abbildung des Versuchsaufbaus |
| 5 | CONTENT E | 1.2 Bedienoberfläche im Browser mit annotierter Abbildung |
| 6 | SUBSECTION 1 | Divider 2 Regelwerk |
| 7 | CONTENT A_V2 | 2.1 RED und CRA: zwei Regime, ein Übergang, Vergleichstabelle mit fünf Merkmalen |
| 8 | CONTENT B_V1 | 2.2 Produktklassifizierung und Normenlage: Kategorie, Konformitätsbewertung, Normen |
| 9 | SUBSECTION 2 | Divider 3 Produkt und Anwendbarkeit |
| 10 | CONTENT B_V1 | 3.1 Produkt und Vorhaben, sechzehn Punkte in zwei Spalten: Aufgabe, Sensor, Regelung, Stellglied, Schnittstellen, Werkzeuge, Aufbau, Betrieb, Dateisystem, Datenverarbeitung, Vorhaben, Rechtslage, Entwicklungsstand, Rollen im Dossier |
| 11 | CONTENT A_V1 | 3.2 Anwendbarkeit, zehn Fragen mit Entscheidungsspalte |
| 12 | CONTENT A_V1 | 3.3 Prototypen und Vorserien, sechs Fälle und Hinweistext in zwei Sprachen |
| 13 | SUBSECTION 3 | Divider 4 Risiken und Maßnahmen |
| 14 | CONTENT A_V1 | 4.1 Risikobild: kritische Befunde R1 bis R9 mit Gegenmaßnahme im Klartext und Stand |
| 15 | CONTENT A_V1 | 4.2 Maßnahmen und Status, Phase 1 bis 3 |
| 16 | CONTENT A_V1 | 4.3 Maßnahmen und Status, Phase 4 und 5 |
| 17 | CONTENT A_V1 | 4.4 Aktiver Strang: was jetzt umgesetzt wird |
| 18 | SUBSECTION 5 | Divider 5 Pflichten der CRA |
| 19 | CONTENT A_V1 | 5.1 Pflichten 1 bis 10 |
| 20 | CONTENT A_V1 | 5.2 Pflichten 11 bis 22 |
| 21 | CONTENT A_V2 | 5.3 Pflichten 23 bis 26 mit Geltungshinweis |
| 22 | SUBSECTION 4 | Divider 6 Dokumente und Quellen |
| 23 | CONTENT A_V1 | 6.1 Dokumentenregister, gruppiert |
| 24 | CONTENT B_V1 | 6.2 Quellen, inklusive ETSI EN 303 645 |
| 25 | CONTENT A_V2 | 6.3 Status und Revision mit nächstem Schritt |
| 26 | SUBSECTION 5 | Divider 7 Fazit und nächste Schritte |
| 27 | CONTENT A_V1 | 7.1 Was folgt daraus für VENTREX, acht Folgen mit Verweisen auf 4.2, 4.4, 5.1 und 2.2 |
| 28 | CLOSING SLIDE | Abschlussbild der Vorlage |

Nachtrag 2026-10-01, Systembeschreibung: Vor Abschnitt 1 steht jetzt der Abschnitt Systembeschreibung mit Divider und zwei Bildfolien. Folie 4 zeigt die Systemübersicht des Versuchsaufbaus, Folie 5 die annotierte Bedienoberfläche im Browser, beide aus dem Benutzerhandbuch. Die Bilder liegen als PNG unter `sec/pic` und werden mit `tools/pdf_to_png.py` aus `LaTeX/manual/pic` erzeugt.

Nachtrag 2026-10-01, Kapitel 1 und Abschnitt Fristen: Kapitel 1.5 teilt sich in 1.5 Produktklassifizierung und Normenlage und 1.6 Was folgt daraus für VENTREX (dunkle Akzentfolie). Der Abschnitt Fristen und Sanktionen ist entfallen, die Quellen und die Revision sind Abschnitt 4. Das Deck umfasst damit 26 Folien.

Nachtrag 2026-10-01, Umsortierung ohne Vorwärtsverweise: Die Abschnitte 4 bis 6 sind neu geordnet. Risiken und Maßnahmen stehen jetzt als Abschnitt 4 vor den Pflichten der CRA (Abschnitt 5), weil die Pflichtentafeln und das Register die Kennungen R1 bis R9 und P1.1 bis P5.2 zitieren. Das Dokumentenregister ist von Platz 19 auf Platz 23 gewandert und gehört als Nachweis zu Abschnitt 6 Dokumente und Quellen. Die Maßnahmenspalte des Risikobilds nennt die Gegenmaßnahme im Klartext statt der Kennung, damit auf Folie 14 noch keine Kennung ohne Erklärung steht. Auf Folie 9 entfällt die Kennungsliste im CRA-Status, auf dem Divider 4 die Kennungsliste in der Unterzeile. Die Dividerunterzeilen aller sieben Abschnitte sind jetzt sichtbar.

Nachtrag 2026-10-01, Tausch der Abschnitte 2 und 3: Das Regelwerk steht jetzt vor der Einordnung des Produkts. Abschnitt 2 trägt 2.1 RED und CRA sowie 2.2 Produktklassifizierung und Normenlage, Abschnitt 3 trägt 3.1 Produkt und Vorhaben, 3.2 Anwendbarkeit und 3.3 Prototypen und Vorserien. Die Folienzahl bleibt 28, das Inhaltsverzeichnis ist nachgezogen. Die Unterzeilen aller Divider nach dem ersten sind als Übergang formuliert (E41), der Verifier prüft das.

Nachtrag 2026-10-01, Details auf Folie 10: 3.1 Produkt und Vorhaben trägt jetzt sechzehn Punkte statt acht, je acht in beiden Spalten. Neu sind Aufgabe, Sensor (0,5 bis 4,5 V für 0 bis 10 bar), Regelung (10 ms Abtastzeit, 1 kHz Regelschleife, Sollwert 6 bar), Stellglied (PWM 2 kHz, 14 Bit, 20 bis 90 Prozent, Hysterese), Schnittstellen (Port 80, VENTCON_AP, 192.168.4.1, ventcon.local, serielle Konsole, LittleFS), Werkzeuge (PlatformIO, Version aus VERSION), Aufbau (12 bis 24 V, ohne Weitergabe), Betrieb (zwei Clients, gesammelte Übernahme), Dateisystem und Datenverarbeitung (keine Cloud, keine Fernwartung). Werte aus `src/Constants.h`, `data/default.json` und dem Benutzerhandbuch, als Quellenzeile unter der Folie benannt.

## Umsetzung

| Schritt | Inhalt |
|---|---|
| U1 | Generator `build_ventrex_deck.py`: Vorlage öffnen, alle Folien über `sldIdLst` und `drop_rel` entfernen |
| U2 | Layoutauflösung je Master, Helfer für Titel, Body, Fußzeile, Tabellen |
| U3 | 21 Folien aufbauen, Inhalte aus dem Master, Tabellen in 8 Folien, Listen in 6 Folien |
| U4 | Speichern nach `sec/2026-10-01_CRA_Compliance_Praesentation.pptx`, Größe berichten |
| U5 | Verifikation mit `verify_ventrex_deck.py` |
| U6 | Sichtprüfung per PowerPoint-COM, Export nach `%TEMP%\cra_deck\*.png` |
| U7 | Altdatei vom 2026-09-29 entfernen, Eintrag in `deletion_log.md` |
| U8 | Nach der RED-CRA-Trennung im Master: Folien 3, 4, 5, 15, 16 und 19 anpassen, Verifier um zwei Regeln erweitern, erneut bauen und sichten |
| U9 | Prototypenfolie nach Folie 3 einfügen, Folienzahl im Verifier auf 21 setzen, bauen, sichten |
| U10 | Analyse auf Konsistenz und Redundanz, danach Umbau auf 25 Folien inklusive Agenda, verschobenem Register und drei neuen Inhaltsfolien |
| U11 | Verifier um drei Prüfungen erweitern und den Zieldatei-Pfad als Argument zulassen |
| U12 | Abschnitt Systembeschreibung mit zwei Bildfolien vor Abschnitt 1 einfügen, Bilder über `tools/pdf_to_png.py` erzeugen, Inhaltsverzeichnis und Verifier auf 28 Folien und sechs Divider nachziehen |
| U13 | Vorwärtsverweise beseitigen: Abschnitte 4 bis 6 umsortieren, Folientitel und Kapitelverweise nachziehen, Inhaltsverzeichnis neu schreiben, Dividerunterzeilen sichtbar machen, Verifier um die Verweisprüfung erweitern, Negativkontrolle mit `tools/forward_ref_control.py` |

## Verifikation

| Prüfung | Kriterium |
|---|---|
| Folieninventar | 28 Folien, 1 COVER, 7 SUBSECTION, 1 CLOSING, Rest CONTENT |
| Vorlagenreste | kein "Hier kommt der Folientitel", "Name Präsentation", "20.05.2026", "STEP", "DIAGRAMM", "Segnaposto" |
| Platzhalter | Titel jeder Folie gefüllt, Fußzeile und Datum gesetzt |
| Foliennummer | Feld `<a:fld type="slidenum">` erhalten |
| Schrift | kein Fremd-Typeface, nur Theme-Verweise und Markenschriften |
| Farben | nur Markenpalette |
| Inhalt | 26 Pflicht-Nummern, 21 Maßnahmen-IDs, Quellen, kein Em-Dash |
| RED-Abgrenzung | keine RED-Aussage ohne Gültigkeitsvermerk bis 2027-12-11 |
| Normenhierarchie | EN 303 645 nur mit Kennzeichnung als technischer Maßstab |
| Paket | Datei erneut öffnen, jede `r:embed` mit gültiger Relationship |
| Vorlage | SHA256 vor und nach dem Lauf identisch |
| Visuell | PNG-Export geprüft: Cover, Divider, Tabellenfolie, Dunkelfolie, Abschluss |
| Inhaltsverzeichnis | jede betitelte Folie ist im Inhaltsverzeichnis auffindbar, Prüfung im Verifier |
| Nummerierung | Divider und Inhaltsfolien tragen die erwartete Folge 1, 1.1 bis 7.1 in Folienreihenfolge, Prüfung im Verifier |
| Kopfzeile | jede Unterfolie trägt den Kapitelverweis, keine andere Folie, Prüfung im Verifier |
| Rechtszitate | keine erfundene Ziffernform, CE-Kennzeichnung nie mit Art. 28, Teil II nur Nr. 1 bis 8, jede Artikelnummer existiert in der Normquelle. Geprüft im Deck und im Master |
| Bilder | zwei Abbildungen aus dem Benutzerhandbuch, vollständig und unbeschnitten, jede mit Quellenzeile; kein leerer Bildplatzhalter im Deck |
| Vorwärtsverweise | Verweise auf spätere Folien oder Abschnitte sind ein Fehler, Kennungen erst ab der Folie ihrer Erklärung, Ausnahmen Inhaltsverzeichnis und Normzitate, Prüfung im Verifier |
| Negativkontrolle | `tools/forward_ref_control.py` erzeugt eine Kontrolldatei mit absichtlichem Vorwärtsverweis und einer zu früh genannten Kennung; der Verifier muss beide melden |
| Dividerunterzeile | jeder der sieben Divider zeigt seine Unterzeile, auch auf den Layouts ohne Textplatzhalter |
| Planumsetzung | `tools/audit_plan_umsetzung.py` liest die Zielbild-Tabelle dieses Plans als Sollvorgabe und vergleicht Reihenfolge, Layout, Nummer, Dividerunterzeile, Verweise, Kennungen und Inhaltsverzeichnis mit dem Deck, unabhängig vom Verifier |
| Protokoll | Ergebnis der Umsetzungskontrolle in `sec/analyse/2026-10-01_Praesentation_Umsortierung_Verifikation.md` |

## Scope-Grenzen

- Vorlage und Master bleiben unverändert.
- Keine Änderung an `src/` und `LaTeX/`.

## Hinweise

- Unbenutzte Master behalten ihre Bilder, deshalb bleibt die Datei größer als nötig.
  Gemessener Wert siehe Bericht; Pruning nur auf Zuruf.
- Der Vorgänger-Generator `build_cra_presentation.py` und der Vorgänger-Verifier
  `verify_cra_presentation.py` erzeugen das alte Design und sind damit abgelöst.
