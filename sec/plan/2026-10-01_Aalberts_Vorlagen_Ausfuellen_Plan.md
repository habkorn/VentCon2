# Plan: Aalberts-Vorlagen ausfüllen (Präsentation, Checkliste, Fragebogen, Addendum)

Datum: 2026-10-01
Gegenstand: die drei Vorlagen in `sec/AalbertsChecklists` und ihre Markdown-Extrakte
Grundlage: `sec/CRA_Compliance_Checklist.md`, `sec/analyse/`, `sec/plan/`, `sec/grundlagen/`, Projektstand aus `src/`
Status: Plan, entschieden am 2026-10-01, umgesetzt und verifiziert am 2026-10-01

## Entscheidungen des Nutzers (2026-10-01)

| Nr. | Entscheidung |
|---|---|
| E49 | Rolle: VENTREX ist Teil des Aalberts-Konzerns. Der Fragebogen dient der Einordnung, wie der aktuelle Stand von VentCon2 gegenüber der CRA aussieht. Aalberts ist die anfragende Gesellschaft, VENTREX antwortet für VentCon2 |
| E50 | Ablage: Antworten werden in die vorhandenen Markdown-Dateien eingetragen und dort fett hervorgehoben, Vorlagentext bleibt unverändert |
| E51 | Sprache: Englisch wie die Vorlagen |
| E52 | Zusätzlich werden befüllte Wordkopien erzeugt, die Wordvorlagen bleiben unberührt |
| E53 | Antworttiefe: knapp, Antwortcode plus Kurzbegründung plus Maßnahmenkennung |
| E54 | Entitäten und Personen: Rollenbezeichnungen und Platzhalter, offene Angaben als `[to be completed]` |
| E55 | Ein Verifier für die beantworteten Dateien wird gebaut |
| E56 | In den Wordkopien werden die Antworten und Hinweise fett und im Markenblau `#002F87` gesetzt, dem Blau der VENTREX- und Aalberts-Vorlagen. Im Markdown bleibt die Hervorhebung fett |
| E61 | Im Worddokument entfallen die Markdown-Marker `**`. Dort trägt die fette Schrift im Markenblau die Betonung. Im Markdown bleiben die Marker, weil sie dort die Auszeichnung sind |
| E62 | Das Addendum wird vollständig aus der Antwortstrecke genommen. Es hat keine Sachfragen, nur Vertragsangaben, die VENTREX nicht allein festlegen kann. Vorlage und Extrakt bleiben im Dossier, die Wordkopie entfällt |
| E63 | Offene Angaben werden geschlossen: Owner als Rolle (`VENTREX product owner, VentCon2 team`), Land `Austria`, Modellnummer `no series model number yet`, Serienintegration `not scheduled`, Meldeweg `VENTREX Engineering department`, Sign-off `Product owner, VENTREX (Aalberts group company)`. Keine Zusage beim Supportzeitraum und bei den Reaktionszeiten, der Stand bleibt offen dokumentiert. Der Sicherheitskontakt wird an den Meldeweg angeglichen |
| E75 | In der Antwortspalte der Checkliste steht das Urteil `Y`, `N` oder `N/A` in einer eigenen Zeile, fett im Markenblau. Die Begründung folgt darunter im selben Blau mit magerem Schnitt. Im Markdown trennt `<br>` beide Teile. Der Fragebogen bleibt unverändert, dort gibt es kein Urteil |
| E76 | Die Leerzeilen der Vorlagen werden an Ort und Stelle gefüllt, nicht durch eine zusätzliche Hinweiszeile: `Product`, `Owner` und `Date` in der Checkliste, `Completed by (name, role, entity name)` und `Date` im Fragebogen. Die Werte stehen fett im Markenblau, die Beschriftungen behalten das Vorlagenformat. Die früheren Hinweiszeilen dazu entfallen |
| E77 | Tabellenzeilen werden nicht über Seitenwechsel getrennt (`w:cantSplit`). Eine Zeile, die am Blattende nicht mehr vollständig passt, rutscht ganz auf die nächste Seite. Gilt für beide Wordkopien und die Maßnahmen-Anlage |

## Ergebnis der Umsetzung

| Datei | Umfang |
|---|---|
| `CRA_Product_Maturity_Checklist.md` | 20 Vorgaben in Teil I und 11 in Teil II bewertet, Ergebniszeile und Lückenbegründung ergänzt; 3 Antworten Yes, 22 No, 4 N/A; Wordkopie `CRA_Product_Maturity_Checklist_answered.docx` |
| `CRA_Supplier_Due_Diligence_Questionnaire.md` | 36 Fragen beantwortet, Sign-off mit Ergebnis ergänzt; Wordkopie `CRA_Supplier_Due_Diligence_Questionnaire_answered.docx` |
| `CRA_Compliance_Addendum_Supplier_Agreement.md` | Parteien und Hinweis zu den einzugehenden Verpflichtungen ergänzt, Unterschriftenblock bleibt offen; Wordkopie `CRA_Compliance_Addendum_Supplier_Agreement_answered.docx` |

Werkzeuge: `tools/aalberts_fill.py` enthält alle Antworten als einzige Quelle und schreibt sie in Markdown und Wordkopien; `tools/verify_aalberts_answers.py` prüft 273 Punkte: jede Vorgabe und jede Frage beantwortet, jede Antwort fett und im Markenblau, keine Markdown-Marker in den eingesetzten Stellen, Hinweiszeilen vollständig, Wordkopien gefüllt, Prüfsummen der Wordvorlagen unverändert, Spaltenbreiten der Antwortspalten, eingetragene Werte in den Leerzeilen, Umbruchschutz der Tabellenzeilen, Ausschluss des Addendums, Querprüfung über `tools/verify_docx_extract.py`.

Wichtig für spätere Läufe: Die Antworten leben im Markdown und in den Wordkopien. Wird ein Extrakt mit `tools/docx_extract.py` neu erzeugt, gehen sie verloren; danach `tools/aalberts_fill.py` erneut ausführen. Befüllte Wordkopien sind in beiden Werkzeugen von der Verarbeitung ausgenommen.

## Auftrag

Die Fragen und Leerstellen der drei Vorlagen aus dem Projekt heraus beantworten und ausfüllen,
mit belastbaren Antworten aus dem Dossier, ohne Erfindungen.

## Was beantwortet werden muss

| Dokument | Felder | Art |
|---|---|---|
| `CRA_Product_Maturity_Checklist.md` | 20 Sicherheitsvorgaben in Teil I (Kategorien: Risk-based security, Updates and access, Data and system protection, Secure design and user control, Secure development methodologies), 11 Punkte in Teil II zur Schwachstellenbehandlung, Ergebniszeile "Overall result" und "Key gaps / evidence", Kopfzeile "Product, Owner, Date" | Bewertung je Vorgabe mit Y, N oder N/A und Begründung |
| `CRA_Supplier_Due_Diligence_Questionnaire.md` | 36 Fragen in sechs Abschnitten: Lieferant, Rollenklassifizierung, Sicherheitsverantwortung, Schwachstellenbehandlung, SBOM und Transparenz, Support, Zertifizierung sowie Sign-off mit Name, Rolle, Entität, Datum, Ergebnis | Antworten, teils Sachstand, teils Angaben, die nur ein Lieferant machen kann |
| `CRA_Compliance_Addendum_Supplier_Agreement.md` | Platzhalter `[Aalberts Group Company]` und `[Supplier]`, Unterschriftenblock mit Namen und Datum | Vertragsangaben, keine Sachfragen |

## Faktenbasis im Dossier

| Quelle | Verwendung |
|---|---|
| `sec/CRA_Compliance_Checklist.md` | Anwendbarkeit, Produktklassifizierung, Pflichten 1 bis 26 mit Status, Arbeitspakete P1.1 bis P5.2 mit Phase und Status, Risiken R1 bis R9, Quellen, Revisionshistorie |
| `sec/analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md` | Befunde und Evidenz zu den Risiken R1 bis R9, Grundlage für Y- und N-Bewertungen |
| `sec/plan/` (21 Dokumente) | Ist-Zustand, Umsetzungsschritte und Verifikation je Maßnahme, Grundlage für "in Arbeit" und Zieltermine |
| `sec/grundlagen/` | Mutex und Web-Authentifizierung, technische Begründung |
| Projektstand | Produktdaten wie Firmware 2.7.4, ESP32-S3, Sensor 0 bis 10 bar, PWM 2 kHz mit 14 Bit, Access-Point mit zwei Clients, LittleFS, keine Cloud und keine Fernwartung, Versorgung 12 bis 24 V |
| Marktstatus | Prototypenbetrieb im eigenen Aufbau, geplanter Markt Indien, kein EU-Vertrieb; Pflichten greifen erst mit der Bereitstellung auf dem Unionsmarkt |

Was heute nachweislich **noch nicht** vorhanden ist und deshalb ehrlich als Lücke erscheint: SBOM,
Supportzeitraum mit Enddatum, Schwachstellenmeldeweg und Disclosure-Richtlinie, technische
Dokumentation nach Annex VII, Konformitätserklärung und CE, Zertifizierungen, signierte Updates.
Ausnahme: die vier aktiven Maßnahmen P1.1, P1.2, P1.3 und P2.2 stehen in Arbeit, P3.1 und P3.2 sind
erledigt.

## Rolle als Vorfrage

Die Vorlagen stammen aus dem Aalberts-Konzern und sind durchgängig aus der Sicht der Gesellschaft
geschrieben, die das Produkt verantwortet (Feld "Aalberts company responsible for this integration").
Vor dem Ausfüllen ist zu klären, welche Rolle VENTREX je Dokument hat. Zwei Lesarten:

- **Lesart A, VENTREX als Gesellschaft:** Die Checkliste wird für VentCon2 als eigenes Produkt
  ausgefüllt, der Fragebogen richtet sich an die eigenen Komponentenlieferanten (ESP32-Modul, Sensor,
  Ventil, Softwarebibliotheken), das Addendum ist die Vorlage für diese Lieferantenbeziehungen.
- **Lesart B, VENTREX als Lieferant:** Der Fragebogen wird von VENTREX über die eigene Komponente
  beantwortet, das Addendum schließt zwischen der Aalberts-Gesellschaft und VENTREX.

Die Antworten unterscheiden sich deutlich, deshalb ist das die erste Frage (Q1).

## Umsetzung

| Schritt | Inhalt |
|---|---|
| S1 | Rolle je Dokument festlegen (Q1) und die Entitäts- und Personenangaben einsammeln (Q6) |
| S2 | Antwortraster aufbauen: je Feld Antwortcode (Y, N, N/A, teilweise), Kurzbegründung, Belegquelle im Dossier, betroffene Maßnahmenkennung, offener Punkt mit Ziel |
| S3 | Bewertung der 20 Sicherheitsvorgaben und der 11 Punkte zur Schwachstellenbehandlung aus dem Projektstand, Ergebniszeile und Lückenliste setzen |
| S4 | Beantwortung der 30 Fragebogenfelder, Angaben, die vom Lieferanten kommen müssen, klar als solche kennzeichnen |
| S5 | Addendum-Parteien setzen, Unterschriftenblock leer lassen |
| S6 | Beantwortete Markdown-Dateien schreiben, die Extrakte bleiben unverändert, damit die Quellentreue erhalten bleibt |
| S7 | Beantwortete Worddateien aus den Vorlagen erzeugen (Kopien befüllen, Originale unberührt) |
| S8 | Verifikation: jedes Feld hat eine Antwort, keine leeren Zellen, jede Antwort mit Belegzeile, Marktstatus genannt |
| S9 | Abnahme und Sichtprüfung, Plan und Gedächtnis nachziehen |

## Antwortleitfaden

1. Nur belegte Aussagen, jede Antwort mit Quellenverweis auf Dossier, Analyse oder Detaildokument.
2. Kein "Y", wo nur ein Plan existiert; laufende Maßnahmen werden als "in Arbeit" mit Kennung und
   Ziel genannt, offene als Lücke.
3. Der Marktstatus wird je Dokument einmal benannt: kein Inverkehrbringen in der Union, deshalb
   greifen die Pflichten derzeit nicht, die Bewertung zeigt trotzdem den Reifegrad.
4. Keine Zusagen erfinden, insbesondere nicht zu Supportzeitraum, Meldefristen, Zertifizierungen,
   Auditrechten oder Vertragsstrafen. Zusagen, die vertraglich wären, werden als offene Entscheidung
   markiert.
5. Sprache und Aufbau der Vorlage beibehalten, Texte in der Vorlage nicht umschreiben.

## Offene Fragen

| Nr. | Frage | Optionen |
|---|---|---|
| Q1 | Rolle je Dokument | a) Lesart A, VENTREX als Gesellschaft, b) Lesart B, VENTREX als Lieferant, c) beides, getrennte Fassungen |
| Q2 | Ablage der Antworten | a) neue Dateien neben den Extrakten, Namen mit Zusatz `_answered`, b) in die Extrakte hineinschreiben, c) eigener Unterordner |
| Q3 | Sprache der Antworten | a) Englisch wie die Vorlagen, b) Deutsch mit englischen Fragen, c) Englisch mit deutscher Kurzbegründung |
| Q4 | Worddateien | a) Markdown und befüllte Wordkopien, b) nur Markdown, c) nur Wordkopien |
| Q5 | Antworttiefe und Umgang mit Lücken | a) knapp, ehrlicher Sachstand plus Maßnahmenkennung, b) ausführlich mit Verweis auf Detaildokumente und Zielterminen, c) nur Y, N, N/A ohne Begründung |
| Q6 | Angaben zu Entitäten und Personen | a) nur Rollenbezeichnungen und Platzhalter, b) konkrete Namen und Adressen, die der Nutzer liefert, c) Platzhalter belassen und im Dokument als offen kennzeichnen |
| Q7 | Prüfung und Anschluss | a) Verifier für die beantworteten Dateien bauen, b) zusätzlich im Master registrieren, c) keine weitere Prüfung |

## Scope-Grenzen

- Die Wordvorlagen und die Extrakte bleiben unverändert.
- Keine Rechts- oder Vertragsberatung: Klauseln des Addendums werden nicht bewertet, nur die
  Parteien eingesetzt.
- Keine Zusagen zu Support, Fristen oder Zertifizierungen ohne Freigabe.
- Kein Eingriff in Deck, Master oder Normextraktion, solange Q7 nicht entschieden ist.

## Nachtrag vom 2026-10-01

Die Dateien liegen nach der Umstrukturierung in `sec/AalbertsChecklists/extracts` (Markdown mit den
Antworten) und `sec/AalbertsChecklists/filled` (befüllte Wordkopien), die Vorlagen in
`sec/AalbertsChecklists/templates` (Entscheidungen E57 bis E60 in
`sec/plan/2026-10-01_Aalberts_Ordnerstruktur_Plan.md`). Inhalte und Antworten sind unverändert.

Nachtrag zur Antwortstrecke (E61 bis E63): Die Wordkopien tragen keine Markdown-Marker mehr, weil
dort die fette Schrift im Markenblau die Betonung trägt. Das Addendum ist vollständig aus der
Antwortstrecke genommen, seine Wordkopie ist entfallen, Vorlage und Extrakt bleiben im Dossier. Alle
offenen Angaben sind mit abgestimmten Werten geschlossen. Die Zeile zum Sicherheitskontakt nennt
jetzt den Meldeweg; der benannte Ansprechpartner nach Art. 13(17) CRA bleibt als Maßnahme P4.8 und
P4.5 offen. Supportzeitraum und Reaktionszeiten bleiben ohne Zusage, der Stand ist dort ehrlich als
offen vermerkt.

## Nachtrag vom 2026-10-02

| Nr. | Entscheidung |
|---|---|
| E64 | Die Antwortspalte bekommt Breite nach Textanteil, begrenzt auf 40 bis 55 Prozent der Tabellenbreite. Die Tabellenbreite bleibt unverändert, jede übrige Spalte behält mindestens 0,90 Zoll. Der Anteil wird je Dokument aus dem Textumfang aller Antworten und aller übrigen Zellen gerechnet, damit Antworten und Fragen im Verhältnis ihres Umfangs stehen |

Ergebnis: In der Checkliste wächst die Antwortspalte von 0,89 auf 3,17 Zoll, im Fragebogen von 2,66 auf 3,85 Zoll. Die geschätzten Zeilenzahlen der Antwortzeilen sinken in der Checkliste um 61 Prozent und im Fragebogen um 22 Prozent. Die übrigen Spalten bleiben lesbar: Kategorie 1,02 Zoll, Sachvorgabe 2,81 Zoll in Teil I und 3,83 Zoll in Teil II, Frage 3,15 Zoll.
