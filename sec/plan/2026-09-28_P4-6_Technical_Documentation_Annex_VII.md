# Plan: Technical Documentation (Annex VII) (P4.6)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Massnahme: P4.6
CRA-Bezug: Annex VII (Pflicht-Nr. 17)
Abhaengigkeiten: P4.1 (Risikobewertung), P4.2 (SBOM), P4.4 (Supportzeitraum), P4.7 (Konformitaetserklaerung)
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu fuellen.

## 1. Objective

Assemble the technical documentation that the conformity assessment and market surveillance rely on,
and keep it maintainable rather than producing a one-off PDF.

## 2. Current State (verified 2026-09-28)

- Substantial material exists but is not arranged per Annex VII:
  `LaTeX/doc/VentCon2_Documentation.tex` with `sections/`, `LaTeX/doc/Dependencies_Matrix.tex`,
  the user manual `LaTeX/manual/VentCon_User_Manual.tex`, and the whole `sec/` dossier.
- Missing inputs: risk assessment (P4.1), SBOM (P4.2), support period (P4.4),
  declaration of conformity (P4.7).
- The LaTeX documentation is written in English, the master and parts of `sec/` in German.

## 3. Design (TODO): content list per Annex VII

1. General description, intended use, hardware and software versions.
2. Design and development, including architecture and data flows (reference the LaTeX sections).
3. Cybersecurity risk assessment (P4.1 as sub-document).
4. Standards and specifications applied, and an explanation where they are not applied in full.
   Note: no CRA harmonised standard has been cited in the Official Journal yet (master, Abschnitt 5).
5. Conformity assessment procedure and its result (Module A, self-assessment).
6. Support period (P4.4) and the update mechanism (P4.3).
7. Software bill of materials (P4.2).
8. Test and verification records, including the verification steps of the individual plans.
9. EU declaration of conformity (P4.7).

## 4. Implementation Steps (TODO)

- Decide the delivery form: one LaTeX built PDF, or a folder of documents with an index page.
- Add a document control block (version, date, author, revision history) to each part.

## 5. Verification (TODO)

- Every item of the Annex VII list resolves to a document or to a justified "not applicable".
- The documentation set matches the shipped firmware version.

## 6. Open Items

- Retention period and language requirements.
- Whether the existing LaTeX monolith is reused or replaced by the new structure
  (see `plans/2026-08-29_Documentation_Update_Plan.md`).
