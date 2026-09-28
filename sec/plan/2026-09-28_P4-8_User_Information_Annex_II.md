# Plan: User Information (Annex II) (P4.8)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P4.8
CRA-Bezug: Annex II (Pflicht-Nr. 21)
Abhaengigkeiten: P1.1 und P2.2 (Provisioning statt festem Passwort), P4.3 (Update-Anleitung), P4.4 (Enddatum)
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Provide the required product information and instructions, including the security relevant parts,
in the user manual and where appropriate on the device or its packaging.

## 2. Current State (verified 2026-09-28)

`LaTeX/manual/VentCon_User_Manual.tex` (about five pages) covers the system overview, wiring, WiFi
connection, browser interface and PID basics. Gaps against Annex II:

- The AP password `ventcon12!` is printed in the manual. With P2.2 this becomes obsolete and the
  printed password must be replaced by provisioning instructions.
- Missing: manufacturer identity and contact, support period end date (P4.4), update instructions
  (P4.3), security contact and disclosure policy (P4.5), advice on secure commissioning and on
  resetting or decommissioning the device.
- The manual is also served by the device itself (`data/`, served through `handleFileRead`).

## 3. Design (TODO)

- Add a security chapter: first commissioning, password assignment, network scope, what the device
  does not do (no internet access), how to report a problem, how to apply updates.
- Replace the printed password with the setup page flow once P1.1 and P2.2 exist.
- Decide the split between manual, device UI and packaging, and note the manufacturer address
  requirement.
- Keep the document language consistent with the target market.

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

- Cross-check the manual against the Annex II information list and against the master, Abschnitt 4.3.
- Read the manual as an operator: the device can be commissioned and secured without external help.
- Confirm the manual version served by the device matches the shipped revision.

## 6. Open Items

- Required language versions.
- Whether the printed password must remain until the provisioning flow is implemented.
