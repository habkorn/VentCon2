# Agent Notes: VentCon2

Durable project knowledge promoted from agent memory (Ledger rule, see `.github/agent-instructions.md`). Newest section last.

## VentCon2 (from github-copilot.md, last updated 2026-10-05, promoted 2026-11-08)

- CRA compliance dossier: `sec/CRA_Compliance_Checklist.md` is the only status source; details under `sec/analyse|plan|grundlagen|sources`. Python via `C:\venvs\VentCon2` (machine-local, present on VEN-PC-W11-KAOO; never a venv in Dropbox).
- Normtext: born-digital 81-page CRA PDF extracted to 19 Markdown files by `tools/cra_extract.py --write`, independently verified by `tools/verify_cra_extract.py` (21 checks, every Sollwert read from the PDF). Protocol `sec/analyse/2026-09-30_CRA_Volltext_Verifikation.md`.
- Resolved by 2026-10-04: the checklist now cites Annex I Part I as `Annex I, Teil I, (2)(b)` (points (1) and (2), letters (a) to (m)); the old `Teil I 1.2 ... 1.10` form is gone from `sec/`.
- Both subagent runs here ended with "completed with no output"; re-derive every subagent claim from the artefacts instead of trusting the report.
