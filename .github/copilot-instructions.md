# Project Instructions: VentCon2

## Purpose

Valve control and ventilation simulation.

## Recommended Agents

| Agent | Role in this project |
|---|---|
| **Axiom** | Numerical simulation of valve control dynamics |
| **Pyra** | Python simulation engine |
| **Forge** | Valve design and actuation mechanics |

---

<!-- AITEAM:BEGIN v2 (generated from AITeam .github/templates/project-instructions.block.md; do not edit inside) -->
## Implementation Protocol

### Delegation Judgment

Use the Recommended Agents table above to decide which specialist to invoke. Whether to actually delegate is a judgment call: weigh benefit against overhead.

**Delegate when:**
- The task spans 3+ files and you would otherwise chain multiple manual searches or reads
- The subagent has domain knowledge you lack
- The task involves codebase exploration across unknown files

**Skip delegation when:**
- The change is a single line in a single file with no exploration needed
- You have already read the full context and there are no hidden dependencies
- The overhead of describing the task to a subagent exceeds the work itself

**Anti-pattern**: delegating for delegation's sake. If a one-line Python edit takes 30 seconds directly, do not spend 60 seconds composing a subagent prompt.

### Bulk Deletion Protocol

**CRITICAL: Applies to ALL file-destructive operations regardless of mechanism: terminal commands, Python scripts invoked by agents, subagent actions, and tool calls. Before delegating any task that may involve file deletion, include this protocol in the subagent's instructions.**

1. **Dry-run first**: Always use `-WhatIf` (PowerShell) or `--dry-run` first. Show the complete list of what WOULD be deleted. Never skip this step.
2. **Get explicit approval**: State the count of files/dirs. Wait for user confirmation before executing. Never proceed without approval.
3. **Check git tracking**: For git-tracked files being deleted, warn the user and require explicit `--force` acknowledgment. Consider `git stash --include-untracked` first, instant recovery via `git stash pop` if needed. Untracked and gitignored files (build artifacts, caches, generated files) may proceed normally with dry-run + approval. If unsure, ask.
4. **Quote all paths**: Use `Remove-Item "literal path with spaces"`, never dynamically construct paths from directory listings without quoting.
5. **One path per Remove-Item call** for anything outside gitignored cache dirs. Use explicit hardcoded paths. Do not use `Get-ChildItem | ForEach-Object { Remove-Item }`.
6. **Never use `Remove-Item -Recurse` on dynamically constructed paths**. If the path comes from a variable or loop, validate it against the expected pattern first.
7. **Prefer `C:\Dropbox\Tom\AITeam\scripts\safe_cleanup.py`** for cache/artifact cleanup. It automates dry-run, git-tracking checks, Dropbox warnings, and audit logging. Run without flags for a dry-run review, then re-run with `--execute` to proceed.
8. **Dropbox awareness**: When deleting files in a Dropbox-synced path, warn that deletions propagate to all synced machines. Suggest pausing Dropbox sync before bulk operations. Dropbox retains 30-day version history as a secondary recovery option.
9. **Audit trail**: For any deletion of 3+ files, write a timestamped log entry to `workspace/deletion_log.md` with: date, time, files, reason, and approval status.

### File System Scope

**CRITICAL: Confine all file operations to this project's directory tree.**

1. Create, edit, and delete files ONLY within this project's folder.
2. Do not touch files in other projects, the AITeam workspace, or any external path.
   **Exception:** Updating your own agent memory file at `C:\Dropbox\Tom\AITeam\workspace\memory\<agent-name>.md` is the one allowed cross-project write (see Agent Memory section below). Do NOT create a local `workspace/memory/` inside this project.
3. The AITeam workspace agent may access files across projects per its own scope rules.
4. The user may grant explicit exceptions: confirm the target path before proceeding.

### Pre-Implementation Checklist

Before starting any multi-step or multi-file implementation:

1. Identify: which parts of this task match a subagent domain?
2. Decide: does the benefit of delegating each part outweigh the overhead?
3. Delegate: launch subagents in parallel for independent work streams where beneficial.
4. Coordinate: handle integration, config, and verification steps directly.

---

## Workspace Conventions

- **Dates**: Always ISO 8601 (`YYYY-MM-DD`).
- **File naming**: `YYYY-MM-DD_short-description.ext`.
- **Output routing**: Write files to the currently open workspace folder.
- **Never use the em dash character** (U+2014) in any generated text. Use a comma, colon, or rephrased sentence instead.

## Venv & Dropbox Rule

- **Never create Python virtual environments inside Dropbox-synced folders.**
- Always use one of these patterns:
  a. `C:\venvs\<project-name>\` (central local cache)
  b. `uv` (automatic central cache via `%APPDATA%\uv\venv`)
  c. `python -m venv --copies .venv` (if local path not feasible, avoids hardlinks)
- Add `.venv/` to `.gitignore` for every Python project.
- Hardlinks inside Dropbox folders are prohibited: they corrupt the sync engine.

## Agent Memory

Agent memory files are stored at:
`C:\Dropbox\Tom\AITeam\workspace\memory\<agent-name>.md`

At the start of every session, read your memory file from that path.
Update it at session end.

The memory protocol (update in place, size limits, pruning order, report every removed entry) is in
`.github/agent-instructions.md`; the central copy is `C:\Dropbox\Tom\AITeam\.github\agent-instructions.md`.

Lasting project-specific knowledge (conventions, pitfalls, verified numbers) belongs in
`docs/AGENT_NOTES.md` of this project. Agent memory keeps cross-project technique and short dated
Ledgers; a finished Ledger is promoted into that file (Ledger rule in `.github/agent-instructions.md`).
<!-- AITEAM:END -->

## Project-Specific Rules

<!-- moved here from the standard sections when the managed block was adopted -->
- **Umlaute**: German text is written with real umlauts (`ä`, `ö`, `ü`, `ß`). Transliteration (`ae`, `oe`, `ue`, `ss`) is only allowed in file names, folder names, identifiers and code symbols, never in prose, headings, table cells or presentation text. Applies to `sec/**`, the generator and verifier scripts, `deletion_log.md` and agent reports. Reference norm text in English stays verbatim.
