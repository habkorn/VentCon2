# Plan: EU Declaration of Conformity and CE Marking (P4.7)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: P4.7
CRA-Bezug: Annex V (Pflicht-Nr. 18) und Art. 28 (Pflicht-Nr. 19)
Abhängigkeiten: P4.1, P4.6 (technische Dokumentation als Grundlage)
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu füllen.

## 1. Objective

Issue the EU declaration of conformity and apply the CE marking in a compliant way, once the
mandatory inputs exist.

## 2. Current State (verified 2026-09-28)

- Neither the declaration nor the CE marking exists. The device carries no marking.
- The inputs are missing: risk assessment (P4.1), technical documentation (P4.6), and the product
  identification (model designation, hardware revision, serial number scheme).
- The RED route interacts with this step: under Delegated Regulation (EU) 2022/30 the product may
  be radio equipment with its own cyber requirements until 2027-12-11 (master, Abschnitt 1).
  Two separate proof strands result.
  1. RED strand, valid until 2027-12-11: declaration with Directive 2014/53/EU as the legal basis,
     cyber requirements from Art. 3(3)(d)(e)(f), standards applied EN 18031-1/-2. Where the
     password clauses 6.2.5.1 and 6.2.5.2 are touched, self-assessment is not available and a
     notified body is required (Module B+C or Module H); the route is decided in P1.5.
  2. CRA strand, from 2027-12-11: declaration with Regulation (EU) 2024/2847 as the legal basis,
     Module A self-assessment per master, Abschnitt 5.
- Until 2027-12-11 both strands run in parallel. Each declaration names the legal act it is issued
  under; the CE marking covers both.
- Scope trigger: the declaration and the CE marking are needed only for units placed on the Union
  market. A device sold in India only leaves this measure dormant. A free demo unit or a delivery to
  a customer in the Union already counts as making available (Art. 3(22)). From that point the EU
  importer carries its own duties (Art. 19 to Art. 21), and Art. 18 covers the representation of a
  manufacturer established outside the Union.

## 3. Design (TODO): declaration content

- Manufacturer and, where applicable, authorised representative, with address.
- Product identification: type, batch or serial number.
- Statement that the declaration is issued under the sole responsibility of the manufacturer.
- Object of the declaration.
- References to the relevant legislation and to the standards or specifications applied.
- Place and date of issue, name and function of the signatory.

## 4. Implementation Steps (TODO)

- Draft the declaration as a LaTeX document so it shares the layout of the technical documentation.
- Define the CE marking placement: housing, type plate or, if the product is too small, packaging
  and accompanying documents.
- Define the process for keeping the declaration available with the technical documentation.

## 5. Verification (TODO)

- Every referenced standard or specification is actually applied and documented.
- The marking is legible and permanent on the chosen surface (adhesion and abrasion check on the label).
- The declaration matches the shipped firmware and hardware revision.

## 6. Open Items

- Which legal entity acts as manufacturer and signs the declaration.
- Whether the demonstration kit is placed on the market at all; if not, no declaration is needed yet
  (master, Abschnitt 2).
- Retention obligations for the declaration and the technical documentation.
