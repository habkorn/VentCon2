# Plan: RED-Konformitätsweg und CE-Route (P1.5)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: P1.5
RED-Bezug: RED Art. 3(3)(d)(e)(f); Delegierte VO (EU) 2022/30 i.d.F. Delegierte VO (EU) 2023/2444;
Pflichten RED-1, RED-2, RED-3
Gültigkeit: gilt bis 2027-12-11. Danach geht die Anforderung in die CRA über (Delegierte
VO (EU) 2026/339 hebt 2022/30 auf). Kein CRA-Bezug in diesem Dokument.
Abhängigkeiten: P2.2 (AP-Passwort), P3.3 (Funkebene), P4.7 (Konformitätserklärung und CE)
Datum: 2026-09-28, Stand der Ist-Aufnahme 2026-09-30
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu füllen.

## 1. Ziel

Den Konformitätsweg nach der Richtlinie 2014/53/EU für die Cyber-Anforderungen festlegen, damit die
CE-Kennzeichnung bis zum 2027-12-11 tragfähig ist. Ergebnis ist eine Entscheidung zwischen
Selbstbewertung und notifizierter Stelle, mit Nachweisliste je RED-Anforderung.

Dieser Plan deckt nur die RED-Route ab. Die CRA-Route (Modul A, Selbstbewertung) steht im Master,
Abschnitt 5.

Geltungsbereich: dieser Plan greift nur, wenn VentCon2 auf dem Unionsmarkt bereitgestellt wird. Bei
Vertrieb ausschließlich in Drittländer, zum Beispiel Indien, ruht er; Wiedervorlage ist der erste
Vertrieb in die Union, die erste kostenlose Demo dort oder die erste Auslieferung an einen Kunden in
der Union. Die Abgrenzung steht im Master, Abschnitt 1 und 2.

## 2. Ist-Zustand (verifiziert 2026-09-30)

- VentCon2 ist ein internetfähiges Funkgerät: ESP32-S3 mit Wi-Fi. Die Cyber-Anforderungen nach
  Art. 3(3)(d)(e)(f) sind seit 2025-08-01 anwendbar.
- Keine Bewertung nach EN 18031-1/-2/-3 vorhanden. Es gibt keine Prüfprotokolle, keinen
  Anforderungskatalog und keinen Nachweis je Klausel.
- Für die RED-Route existiert keine technische Dokumentation. Die vorhandenen Dokumente (P4.1, P4.6)
  sind auf die CRA ausgerichtet.
- Fest verdrahtetes AP-Passwort `ventcon12!` in `src/Constants.h`. Damit ist die EN-18031-2-Klausel
  6.2.5.1 (Passwortklausel) berührt; diese Klausel ist von der Konformitätsvermutung ausgenommen
  (Durchführungsbeschluss (EU) 2025/138). Maßnahme dagegen läuft in P2.2.
- Funkstrecke: WPA2-PSK ohne eigene Geräteauthentifizierung. Maßnahme P3.3 ist offen.
- Das Webinterface läuft ohne TLS.
- Eine Konformitätserklärung existiert nicht, ebenso keine CE-Kennzeichnung (P4.7).
- Offen und vorgelagert: ob VentCon2 überhaupt gewerblich in der EU in Verkehr gebracht wird
  (Master, Abschnitt 2). Bei reiner Prototyp-Nutzung entfällt die RED-Route.

## 3. Design (TODO): Entscheidung und Nachweisliste

- Anwendbarkeit je Anforderung feststellen: RED-1 (Netzwerkschutz) und RED-2 (Schutz
  personenbezogener Daten) einschlägig, RED-3 (Betrugsschutz) voraussichtlich nicht einschlägig,
  Begründung dokumentieren.
- Berührte EN-18031-Klauseln je Norm auflisten, getrennt nach ausgenommenen und nicht ausgenommenen
  Klauseln.
- Entscheiden: Selbstbewertung, wenn keine ausgenommene Klausel berührt bleibt, sonst Modul B+C oder
  Modul H mit notifizierter Stelle. Notifizierte Stellen für EN 18031 in der NANDO-Liste prüfen.
- Nachweisliste je Anforderung: Nachweis, Nachweisort, Datum, verantwortliche Rolle.
- Zuschnitt der technischen Dokumentation für die RED-Route, unter Wiederverwendung der CRA-Dokumente
  (P4.1, P4.2, P4.6) mit getrennter Kennzeichnung des Rechtsakts.
- CE-Kennzeichnung: benannter Rechtsakt heute RED, ab 2027-12-11 CRA. Anbringungsort und
  Aufbewahrung nach P4.7.
- Zeitplan vor dem 2027-12-11, mit Zwischenstand für die Umstellung auf die CRA.

## 4. Umsetzungsschritte (TODO)

## 5. Verifikation (TODO)

## 6. Offene Punkte

- Trägt das geplante NVS-verschlüsselte Passwort die Klausel 6.2.5.1 vollständig, oder bleibt eine
  ausgenommene Klausel berührt?
- Wird eine notifizierte Stelle benötigt? Kosten und Vorlaufzeit sind für die Terminplanung
  entscheidend.
- Ist der Vertriebsweg geklärt (Master, Abschnitt 2)?
- Wie wird die Funkstrecke bewertet, solange P3.3 offen ist?
