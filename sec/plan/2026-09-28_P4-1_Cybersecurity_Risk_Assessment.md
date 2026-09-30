# Plan: Cybersecurity Risk Assessment (P4.1)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P4.1
CRA-Bezug: Art. 13(2) (Pflicht-Nr. 16)
Risikobezug: R1 bis R9 als Eingang
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Produce the documented cybersecurity risk assessment that Art. 13(2) requires and that the
technical documentation (P4.6) builds on.

## 2. Current State (verified 2026-09-28)

- Inputs exist: the risk and evidence analysis (R1 to R9) and the obligation mapping in
  `../CRA_Compliance_Checklist.md`, Abschnitt 7.
- The assessment itself is not written: no threat model, no asset list, no residual risk statement.
- Interpretation guidance available: Commission guidance C(2026) 5252 of 2026-07-27.

## 3. Design (TODO): structure of the dossier

1. Product and intended use, interfaces, processed data, intended environment.
2. Assets: setpoint, calibration, credentials, availability of pressure regulation, safety function.
3. Threat model and trust boundaries: local AP, browser client, serial console, USB flashing path.
4. Risk register: R1 to R9 plus risks without a code finding (supply chain, physical access).
5. Risk based requirements from Annex I, Part I and how each is treated.
6. Residual and accepted risks with a written rationale.
7. Review trigger: new findings, new dependencies, substantial modification.

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

- Every obligation in Abschnitt 6 of the master has a treatment decision in the dossier.
- Every risk in the register has an owner and a status.
- The dossier is consistent with the SBOM (P4.2) regarding third party components.

## 6. Open Items

- Whether a formal method is required or a structured register suffices.
- How the safety function (over-pressure shutoff) is described in risk terms.
