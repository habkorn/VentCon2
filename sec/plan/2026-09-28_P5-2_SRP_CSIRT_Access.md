# Plan: Single Reporting Platform and CSIRT Access (P5.2)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: P5.2
CRA-Bezug: Art. 14 (Pflicht-Nr. 24), Zuordnung nach Art. 14(7)
Abhängigkeiten: P5.1 (Prozess und Rollen)
Datum: 2026-09-28
Priorität: akut, Art. 14 ist seit 2026-09-11 in Kraft
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu füllen.

## 1. Objective

Obtain working access to the European reporting platform and identify the competent CSIRT, so that
a report can be filed within 24 hours of awareness.

## 2. Current State (verified 2026-09-28)

- No accounts, no registration, no established contact.
- Platform status: the ENISA Single Reporting Platform has been operational since 2026-09-11 at
  `portal.cra-srp.enisa.europa.eu`. Access is through EU Login with multi-factor authentication and
  the role "Assigned Representative". One primary and up to 20 secondary representatives are possible;
  the accounts are personal, and shared functional mailboxes are not recommended.
- No reporting API in the first release; submission is manual through the web portal.
- Voluntary reporting under Art. 15 is not available yet; only Art. 14 and Art. 24(3) are supported.
- Up to 20 reports per manufacturer can be pending CSIRT validation.
- The list of CSIRTs designated as coordinators was published on 2026-09-04. Assignment follows the
  main-establishment test of Art. 14(7), that is the place where the cybersecurity decisions for the
  product are predominantly taken, not the registered office.

## 3. Design (TODO)

- Register the EU Login accounts, assign the roles, and store the recovery factors safely.
- Determine the competent CSIRT for the legal entity and record its contact, including out of hours.
- Write a runbook for logon and submission, noting that submission is manual and that the field list
  follows the ENISA working basis.
- Check whether the preferred reporting route changes if the legal entity's establishment moves.

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

- Log in and walk through the form to the last step without submitting.
- Confirm the CSIRT contact responds, or record the expected channel and response time.
- Confirm the deputy can log in with their own account.

## 6. Open Items

- Whether an implementing act on format and procedure (Art. 14(10)) will change the workflow.
- Registration may require company identification documents; the lead time is unknown.
- Whether a reporting API is planned in a later release.
