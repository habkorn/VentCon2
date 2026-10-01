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

## Ergebnis der Umsetzung

| Datei | Umfang |
|---|---|
| `CRA_Product_Maturity_Checklist.md` | 20 Vorgaben in Teil I und 11 in Teil II bewertet, Ergebniszeile und Lückenbegründung ergänzt; 3 Antworten Yes, 22 No, 4 N/A; Wordkopie `CRA_Product_Maturity_Checklist_answered.docx` |
| `CRA_Supplier_Due_Diligence_Questionnaire.md` | 36 Fragen beantwortet, Sign-off mit Ergebnis ergänzt; Wordkopie `CRA_Supplier_Due_Diligence_Questionnaire_answered.docx` |
| `CRA_Compliance_Addendum_Supplier_Agreement.md` | Parteien und Hinweis zu den einzugehenden Verpflichtungen ergänzt, Unterschriftenblock bleibt offen; Wordkopie `CRA_Compliance_Addendum_Supplier_Agreement_answered.docx` |

Werkzeuge: `tools/aalberts_fill.py` enthält alle Antworten als einzige Quelle und schreibt sie in Markdown und Wordkopien; `tools/verify_aalberts_answers.py` prüft 158 Punkte: jede Vorgabe und jede Frage beantwortet, jede Antwort fett, Hinweiszeilen vollständig, Wordkopien gefüllt, Prüfsummen der Wordvorlagen unverändert, Querprüfung über `tools/verify_docx_extract.py`.

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
