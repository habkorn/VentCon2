# Analyse: E-Mail-Thread zur CRA-Produktinventur und zur Anwendbarkeit auf VENTCOM

Datum: 2026-10-02
Gegenstand: `sec/AW_ Cyber Resilience Act (CRA) Product Inventory – Request for Segment-wide Assessment.msg`
Methode: MSG mit `extract_msg` gelesen, HTML-Körper ausgewertet, die drei Anlagen sind Grafik. Die
zitierten Rechtsgrundlagen wurden gegen den CRA-Volltext in `sec/sources/` geprüft.

## 1 Der Thread

| Datum | Von | An | Inhalt |
|---|---|---|---|
| 04.09.2026 | Nisius, Dr. Tobias (IT-Direktor, Aalberts Surface Technologies) | Niels Ketelaar (Aalberts) | Ursprungsnachricht, Produktliste "products in scope for Industry" als Anhang |
| 16.09.2026 | Kratz, Sebastian (legal counsel Aalberts-ST) | Jäger, Nisius, Smit (VAF), Klautzer, CC Hackenberger | Ankündigung der Anfrage: (1) CRA-Reifegrad je Produkt mit der Maturity-Checkliste bewerten, (2) Lieferanten von Dritt-Digitalelementen einbinden, derzeit VAF, mit Fragebogen und Addendum; Unterlagen in der Aalberts Academy; Webinar und Q&A in der Woche ab dem 05.10. |
| 23.09.2026 | Klautzer, Wolfgang (VENTREX) | Haberkorn | "wir sind ja mit VENTCOM in scope und müssen CRA product maturity check durchführen", Bitte um Review der Checkliste |
| 01.10.2026 | Klautzer | Haberkorn, CC Pfeifer | Notizen aus dem Meeting HAB/KLW: CRA ab 2028, RED bis Ende 2027; nicht in scope, wenn das Produkt nicht auf dem Unionsmarkt in Umlauf gebracht wird, Verkauf und Schenkung in scope, Mietvereinbarungen nicht; nicht in scope bei Vertrieb in Indien oder China; bei Scope sechs Findings mit Hardware- und Softwareanpassungen; nächste Schritte: Fragebogen beantworten (HAB), Markt prüfen (KLW/PAF) |
| 02.10.2026 09:20 | Klautzer | Kratz, CC Pfeifer | VENTREX-Einschätzung: kein Verkauf und keine kostenlose Bereitstellung in der EU, stattdessen Miet- oder Leihkonzept zur temporären Zurverfügungstellung für Kundentests; CRA daher nicht anwendbar; Bitte, Bewertung und Dokumentation auf Aalberts-Seite anzupassen |
| 02.10.2026 10:44 | Kratz | Klautzer, CC Pfeifer | Rechtsauffassung, siehe Abschnitt 2; Angebot der Abstimmung mit der Kanzlei Kienhuis im Q&A am 06.10. |
| 02.10.2026 12:07 | Klautzer | Kratz, Haberkorn, CC Pfeifer | Dank an Sebastian, Bitte an Thomas Haberkorn, die Argumentation zum Mietkonzept für die EU zu verschriftlichen |

Die inhaltlich entscheidende Nachricht stammt von Sebastian Kratz. Der jüngste Beitrag im Thread ist
die Bitte an den Nutzer.

## 2 Die beiden Positionen

| Frage | VENTREX (Klautzer, 01. und 02.10.) | Aalberts Legal (Kratz, 02.10.) |
|---|---|---|
| Wird verkauft? | nein | unbeachtlich |
| Wird kostenlos bereitgestellt? | nein | unbeachtlich, entgeltlich oder unentgeltlich |
| Miet- und Leihkonzept | temporäre Überlassung, kein Inverkehrbringen | Abgabe im Rahmen einer Geschäftstätigkeit an Dritte zur Verwendung auf dem Unionsmarkt |
| Ergebnis | CRA nicht anwendbar | CRA erfasst |
| Belegstelle | keine | Art. 3 Nr. 22 CRA |

Kratz nennt ausdrücklich zwei Fälle, die nicht unter die Abgabe fallen: ausschließlich
unternehmensinterne Verwendung und eine reine Dienstleistung unter Verwendung des Produkts, ohne
dass das Produkt Dritten überlassen wird.

## 3 Prüfung am Verordnungstext

| Stelle | Wortlaut, Auszug | Bedeutung für den Fall |
|---|---|---|
| Art. 3 Nr. 21 | "placing on the market means the first making available of a product with digital elements on the Union market" | Der Auslöser ist die erste Abgabe, nicht der erste Verkauf |
| Art. 3 Nr. 22 | "making available on the market means the supply of a product with digital elements for distribution or use on the Union market in the course of a commercial activity, whether in return for payment or free of charge" | Verlangt weder Eigentumsübergang noch Kaufpreis, die vorübergehende Überlassung ist eine Abgabe |
| Art. 2 Abs. 1 | Der Anwendungsbereich knüpft an "made available on the market" an | Ohne Abgabe kein Anwendungsbereich |
| Art. 71 Abs. 2 | Gilt ab 11.12.2027, Art. 14 ab 11.09.2026, Kapitel IV ab 11.06.2026 | "CRA ab 2028" im Meeting-Protokoll ist ungenau, die Meldepflichten laufen bereits |

Die Auffassung von Kratz trägt der Verordnungstext. Die These aus dem Protokoll vom 01.10.,
Mietvereinbarungen seien nicht in scope, hat im Wortlaut keine Stütze. Sie wäre nur zu halten, wenn
keine Abgabe an Dritte erfolgt, also bei rein interner Nutzung, oder wenn die Verwendung außerhalb
des Unionsmarkts liegt.

## 4 Abgleich mit dem Dossier

- Der Master führt den Auslöser bereits so, wie Kratz argumentiert: "Umzusetzen mit der Bereitstellung
  auf dem Unionsmarkt, auch unentgeltlich; ohne EU-Vertrieb bleibt die Umsetzung freiwillig." Das
  Deck prüft diesen Satz mit der Regel V7.
- Master Abschnitt 8 nennt zusätzlich die Bereitstellung durch Importeur oder Händler und verweist auf
  Art. 3 Nr. 16 und 17 sowie Art. 18 und 19. Die Prüfung im Volltext bestätigt die Nummern: Nr. 16 ist
  der Importeur, Nr. 17 der Händler.
- Bruchstelle: Das Dossier unterstellt "Prototypenbetrieb im eigenen Aufbau, geplanter Markt Indien,
  kein EU-Vertrieb". Das Mietkonzept für die EU widerspricht dem. Betroffen wären die Marktaussage im
  Master, die Notizen und Antworten im Aalberts-Paket ("the product is not placed on the EU market, so
  the obligations rest"), der Marktstatus im Deck und die Fristenlogik.

## 5 Was zu tun ist

1. Die Verschriftlichung für Sebastian Kratz liefern. Sie muss vier Fragen beantworten: Wird ein
   Produkt abgegeben oder nur genutzt? Handelt es sich um eine Geschäftstätigkeit? Findet die
   Verwendung auf dem Unionsmarkt statt? Spielt die Dauer der Überlassung eine Rolle?
2. Fertig vor dem Q&A am 06.10., weil dort der externe Kollege der Kanzlei Kienhuis teilnimmt.
3. Entscheidung vorbereiten: Bleibt das Mietkonzept, greift die CRA ab der ersten Bereitstellung, dann
   brauchen Master, Aalberts-Paket und Deck eine korrigierte Marktaussage. Kommt es nicht, bleibt der
   bisherige Stand.

## 6 Offene Punkte

| Punkt | Frage |
|---|---|
| Produktname | Der Thread spricht von VENTCOM, das Dossier und die Aalberts-Antworten von VentCon2. Dasselbe Produkt mit zwei Namen, oder zwei Produkte? |
| Produktliste | Der Anhang der Ursprungsnachricht vom 04.09. liegt in dieser Datei nicht vor, damit ist offen, ob VENTCOM dort geführt wird |
| Mietkonzept | Liegt eine Beschreibung vor, die belegt, ob das Produkt überlassen wird oder eine Dienstleistung erbracht wird? |
| Six Findings | Die sechs Findings aus dem Protokoll vom 01.10. sind nicht ausgeführt |

## 7 Technischer Hinweis

Die drei Anlagen der Nachricht sind Grafik: zwei Aalberts-Logos und ein Zertifizierungsabzeichen, kein
Dokument. Für das Lesen von MSG-Dateien wurde `extract-msg` in die Benutzerumgebung der
System-Python installiert, nicht in das Projektvenv `C:\venvs\VentCon2`.
