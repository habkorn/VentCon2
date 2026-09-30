# Plan: RED-Konformitaetsweg und CE-Route (P1.5)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P1.5
RED-Bezug: RED Art. 3(3)(d)(e)(f); Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444;
Pflichten RED-1, RED-2, RED-3
Gueltigkeit: gilt bis 2027-12-11. Danach geht die Anforderung in die CRA ueber (Delegierte
VO (EU) 2026/339 hebt 2022/30 auf). Kein CRA-Bezug in diesem Dokument.
Abhaengigkeiten: P2.2 (AP-Passwort), P3.3 (Funkebene), P4.7 (Konformitaetserklaerung und CE)
Datum: 2026-09-28, Stand der Ist-Aufnahme 2026-09-30
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Ziel

Den Konformitaetsweg nach der Richtlinie 2014/53/EU fuer die Cyber-Anforderungen festlegen, damit die
CE-Kennzeichnung bis zum 2027-12-11 tragfaehig ist. Ergebnis ist eine Entscheidung zwischen
Selbstbewertung und notifizierter Stelle, mit Nachweisliste je RED-Anforderung.

Dieser Plan deckt nur die RED-Route ab. Die CRA-Route (Modul A, Selbstbewertung) steht im Master,
Abschnitt 5.

Geltungsbereich: dieser Plan greift nur, wenn VentCon2 auf dem Unionsmarkt bereitgestellt wird. Bei
Vertrieb ausschliesslich in Drittlaender, zum Beispiel Indien, ruht er; Wiedervorlage ist der erste
Vertrieb in die Union, die erste kostenlose Demo dort oder die erste Auslieferung an einen Kunden in
der Union. Die Abgrenzung steht im Master, Abschnitt 1 und 2.

## 2. Ist-Zustand (verifiziert 2026-09-30)

- VentCon2 ist ein internetfaehiges Funkgeraet: ESP32-S3 mit Wi-Fi. Die Cyber-Anforderungen nach
  Art. 3(3)(d)(e)(f) sind seit 2025-08-01 anwendbar.
- Keine Bewertung nach EN 18031-1/-2/-3 vorhanden. Es gibt keine Pruefprotokolle, keinen
  Anforderungskatalog und keinen Nachweis je Klausel.
- Fuer die RED-Route existiert keine technische Dokumentation. Die vorhandenen Dokumente (P4.1, P4.6)
  sind auf die CRA ausgerichtet.
- Fest verdrahtetes AP-Passwort `ventcon12!` in `src/Constants.h`. Damit ist die EN-18031-2-Klausel
  6.2.5.1 (Passwortklausel) beruehrt; diese Klausel ist von der Konformitaetsvermutung ausgenommen
  (Durchfuehrungsbeschluss (EU) 2025/138). Massnahme dagegen laeuft in P2.2.
- Funkstrecke: WPA2-PSK ohne eigene Geraeteauthentifizierung. Massnahme P3.3 ist offen.
- Das Webinterface laeuft ohne TLS.
- Eine Konformitaetserklaerung existiert nicht, ebenso keine CE-Kennzeichnung (P4.7).
- Offen und vorgelagert: ob VentCon2 ueberhaupt gewerblich in der EU in Verkehr gebracht wird
  (Master, Abschnitt 2). Bei reiner Prototyp-Nutzung entfaellt die RED-Route.

## 3. Design (TODO): Entscheidung und Nachweisliste

- Anwendbarkeit je Anforderung feststellen: RED-1 (Netzwerkschutz) und RED-2 (Schutz
  personenbezogener Daten) einschlaegig, RED-3 (Betrugsschutz) voraussichtlich nicht einschlaegig,
  Begruendung dokumentieren.
- Beruehrte EN-18031-Klauseln je Norm auflisten, getrennt nach ausgenommenen und nicht ausgenommenen
  Klauseln.
- Entscheiden: Selbstbewertung, wenn keine ausgenommene Klausel beruehrt bleibt, sonst Modul B+C oder
  Modul H mit notifizierter Stelle. Notifizierte Stellen fuer EN 18031 in der NANDO-Liste pruefen.
- Nachweisliste je Anforderung: Nachweis, Nachweisort, Datum, verantwortliche Rolle.
- Zuschnitt der technischen Dokumentation fuer die RED-Route, unter Wiederverwendung der CRA-Dokumente
  (P4.1, P4.2, P4.6) mit getrennter Kennzeichnung des Rechtsakts.
- CE-Kennzeichnung: benannter Rechtsakt heute RED, ab 2027-12-11 CRA. Anbringungsort und
  Aufbewahrung nach P4.7.
- Zeitplan vor dem 2027-12-11, mit Zwischenstand fuer die Umstellung auf die CRA.

## 4. Umsetzungsschritte (TODO)

## 5. Verifikation (TODO)

## 6. Offene Punkte

- Traegt das geplante NVS-verschluesselte Passwort die Klausel 6.2.5.1 vollstaendig, oder bleibt eine
  ausgenommene Klausel beruehrt?
- Wird eine notifizierte Stelle benoetigt? Kosten und Vorlaufzeit sind fuer die Terminplanung
  entscheidend.
- Ist der Vertriebsweg geklaert (Master, Abschnitt 2)?
- Wie wird die Funkstrecke bewertet, solange P3.3 offen ist?
