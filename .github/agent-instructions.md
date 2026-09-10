# AI Team - Agent Instructions

> **Redirect.** This file is maintained centrally in the AITeam repository and
> synchronized into every registered project. Substantive repository-specific
> guidance lives in the containing repository's `.github/copilot-instructions.md`,
> which covers:
>
> - Agent roster and common task routing
> - Delegation judgment (when to delegate vs skip)
> - Bulk deletion protocol
> - File system scope
> - Workspace conventions (ISO dates, file naming, em dash rule)
> - Venv and Dropbox rule
> - Project index or registry reference

## Agent Memory Protocol (summary)

- Memory files live at `C:\Dropbox\Tom\AITeam\workspace\memory\<agent-name>.md`.
- Read your memory file at session start; create it with section headers if missing.
- Update relevant sections in place at session end; do not append logs.
- Limits: max 5 entries per section, max 80 lines total; prune least relevant entries when full.
- If entries were pruned, notify the user with the removed entries and the reason before saving.

## Governance

- Authoritative agent definitions live in the central AITeam repository at `C:\Dropbox\Tom\AITeam\.github\agents\<name>.agent.md`.
- Project-specific guidance lives in the containing repository's `.github/copilot-instructions.md` and is never overwritten by synchronization tooling.
