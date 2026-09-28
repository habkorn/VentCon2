# Plan: Reporting Process under Art. 14 (P5.1)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P5.1
CRA-Bezug: Art. 14(2) und 14(4) (Pflicht-Nr. 23, 25 und 26), Art. 69(3) fuer Bestandsprodukte
Abhaengigkeiten: P5.2 (Plattformzugang), P4.5 (Disclosure-Prozess als Eingangskanal)
Datum: 2026-09-28
Prioritaet: akut, Art. 14 ist seit 2026-09-11 in Kraft
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Set up and staff a process that reports actively exploited vulnerabilities and severe incidents
within the legal deadlines, including for products already on the market.

## 2. Current State (verified 2026-09-28)

- No process, no named responsible person, no accounts on the reporting platform (P5.2).
- The reporting duty applies since 2026-09-11, so the process is already overdue.
- Products placed on the market before 2027-12-11 are covered by the reporting duty even without a
  substantial modification (Art. 69(3)).
- Guidance available: Commission guidance C(2026) 5252 of 2026-07-27, section 9.1.
- No implementing act on format and procedure (Art. 14(10)) has been adopted yet; the ENISA field
  list is the working basis.

## 3. Design (TODO): trigger, cases and clock

- Trigger: "awareness", read by the Commission as the initial assessment reaching reasonable
  certainty, not the moment of technical discovery. Keep an internal timestamp from awareness.
- Cases: actively exploited vulnerability; severe incident, meaning an incident that can affect the
  ability to protect availability, authenticity, integrity or confidentiality of sensitive or
  important data or functions, or that leads to the injection or execution of malicious code.
- Deadlines: 24 h early warning, 72 h full notification to the CSIRT and ENISA, 14 days final report
  for actively exploited vulnerabilities, one month final report for severe incidents.
- Roles: primary and secondary "Assigned Representative" accounts, a deputy, and an escalation path
  to the person authorised to submit.
- Record keeping: what was known when, what was submitted, and the content of each message.
- Interfaces: intake from the disclosure process (P4.5), fix and release through P4.3.

## 4. Implementation Steps (TODO)

- Name two people and one deputy, create the accounts (P5.2), write a one-page runbook,
  run a tabletop test before the next release.

## 5. Verification (TODO)

- Tabletop exercise against the clock with a synthetic report, from awareness to the 24 h message.
- The runbook resolves the deadline arithmetic and names the competent CSIRT contact.
- The record of the exercise is filed with the technical documentation (P4.6).

## 6. Open Items

- The 72 h counter in the platform starts when the 24 h report is submitted, not at awareness, so a
  separate clock must be kept.
- Micro and small enterprises are exempt from fines only for the 24 h deadline (Art. 64(10)), not
  for the remaining deadlines.
- Whether voluntary reporting under Art. 15 becomes available later and should be used.
