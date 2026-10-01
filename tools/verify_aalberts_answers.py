# -*- coding: utf-8 -*-
"""Prüft die ausgefüllten Aalberts-Vorlagen in `sec/AalbertsChecklists`.

Geprüft wird, dass jede Frage und jede Sicherheitsvorgabe eine fette Antwort trägt, dass keine
Antwortzelle leer bleibt, dass die Hinweiszeilen vorhanden sind, dass die Wordkopien dieselben
Antworten enthalten und dass die Wordvorlagen selbst unverändert sind. Zusätzlich läuft der
Extraktprüfer `tools/verify_docx_extract.py` als Querprüfung, damit die Quellentreue der
Markdown-Dateien erhalten bleibt.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/verify_aalberts_answers.py
"""

from __future__ import annotations

import hashlib
import re
import subprocess
import sys
from pathlib import Path

from docx import Document
from docx.shared import RGBColor

ANSWER_COLOR = RGBColor(0x00, 0x2F, 0x87)

ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIR = ROOT / "sec" / "AalbertsChecklists"

CHECKLIST = SOURCE_DIR / "CRA_Product_Maturity_Checklist.md"
CHECKLIST_DOCX = SOURCE_DIR / "CRA product maturity checklist - Sep 2026.docx"
CHECKLIST_COPY = SOURCE_DIR / "CRA_Product_Maturity_Checklist_answered.docx"
QUESTIONNAIRE = SOURCE_DIR / "CRA_Supplier_Due_Diligence_Questionnaire.md"
QUESTIONNAIRE_DOCX = SOURCE_DIR / "CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx"
QUESTIONNAIRE_COPY = SOURCE_DIR / "CRA_Supplier_Due_Diligence_Questionnaire_answered.docx"
ADDENDUM = SOURCE_DIR / "CRA_Compliance_Addendum_Supplier_Agreement.md"
ADDENDUM_COPY = SOURCE_DIR / "CRA_Compliance_Addendum_Supplier_Agreement_answered.docx"

EXPECTED_PART_I = 20
EXPECTED_PART_II = 11
EXPECTED_QUESTIONS = 36
EXPECTED_NOTES = {CHECKLIST: 3, QUESTIONNAIRE: 3, ADDENDUM: 2}

problems: list[str] = []
checks = 0


def check(condition: bool, message: str) -> None:
    global checks
    checks += 1
    if not condition:
        problems.append(message)


def normalize(text: str) -> str:
    return " ".join(text.replace("<br>", " ").replace("**", "").split())


def rows(path: Path):
    """Tabellenzeilen als Zellenlisten, Trennzeilen ausgenommen."""
    for number, line in enumerate(path.read_text(encoding="utf-8").split("\n"), start=1):
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if all(set(cell) <= set("-: ") for cell in cells):
            continue
        yield number, cells


def check_checklist() -> None:
    part = None
    part_one = part_two = 0
    for number, cells in rows(CHECKLIST):
        joined = " ".join(cells)
        if "Compliance assessment" in joined:
            part = 1
            continue
        if "Assessment" in joined and "Safeguard (details)" in joined:
            part = 2
            continue
        if part == 1 and len(cells) == 3 and cells[2]:
            part_one += 1
            check(bool(cells[0]), "Checkliste Teil I, Zeile %d ohne Antwort: %s"
                  % (number, cells[2][:60]))
            check("**" in cells[0], "Checkliste Teil I, Zeile %d ohne fette Antwort" % number)
        if part == 2 and len(cells) == 2 and cells[1]:
            part_two += 1
            check(bool(cells[0]), "Checkliste Teil II, Zeile %d ohne Antwort: %s"
                  % (number, cells[1][:60]))
            check("**" in cells[0], "Checkliste Teil II, Zeile %d ohne fette Antwort" % number)
    check(part_one == EXPECTED_PART_I,
          "Checkliste Teil I: %d Zeilen mit Antwort, erwartet %d" % (part_one, EXPECTED_PART_I))
    check(part_two == EXPECTED_PART_II,
          "Checkliste Teil II: %d Zeilen mit Antwort, erwartet %d" % (part_two, EXPECTED_PART_II))
    text = CHECKLIST.read_text(encoding="utf-8")
    check("Overall result: Gaps identified" in text,
          "Checkliste: Ergebniszeile fehlt")
    check("Key gaps / evidence" in text and "**Evidence:" in text,
          "Checkliste: Lückenbegründung fehlt")
    print("  Checkliste: Teil I %d Antworten, Teil II %d Antworten" % (part_one, part_two))


def check_questionnaire() -> None:
    inside = False
    answered = 0
    for number, cells in rows(QUESTIONNAIRE):
        if cells[:2] == ["Question", "Supplier response"]:
            inside = True
            continue
        if not inside:
            continue
        if len(cells) < 2:
            continue
        if not cells[0]:
            continue
        if cells[0].lower().startswith(("section", "overall")):
            inside = False
            continue
        answered += 1
        check(bool(cells[1]), "Fragebogen, Zeile %d ohne Antwort: %s" % (number, cells[0][:60]))
        check("**" in cells[1], "Fragebogen, Zeile %d ohne fette Antwort" % number)
        if cells[0] == "Question":
            inside = False
    check(answered == EXPECTED_QUESTIONS,
          "Fragebogen: %d beantwortete Fragen, erwartet %d" % (answered, EXPECTED_QUESTIONS))
    text = QUESTIONNAIRE.read_text(encoding="utf-8")
    for token in ("Completed by:", "Date: 2026-10-01", "Accepted with conditions"):
        check(token in text, "Fragebogen: Sign-off ohne %s" % token)
    print("  Fragebogen: %d Antworten" % answered)


def check_notes_and_sources() -> None:
    for path, expected in EXPECTED_NOTES.items():
        text = path.read_text(encoding="utf-8")
        head = text.split("<!-- extract:head-end -->")[0]
        source_name = re.search(r"\| Source file \| `([^`]+)` \|", head).group(1).split("/")[-1]
        source = SOURCE_DIR / source_name
        source_texts = {normalize(paragraph.text)
                        for paragraph in Document(str(source)).paragraphs}
        for table in Document(str(source)).tables:
            for row in table.rows:
                for cell in row.cells:
                    source_texts.add(normalize(cell.text))
        notes = [line for line in text.split("\n")
                 if line.startswith("**") and normalize(line) not in source_texts]
        check(len(notes) == expected, "%s: %d Hinweiszeilen, erwartet %d"
              % (path.name, len(notes), expected))
        recorded = re.search(r"\| SHA256 \| `([0-9a-f]{64})` \|", head)
        check(recorded is not None, "%s: SHA256 im Kopfblock fehlt" % path.name)
        if recorded:
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            check(digest == recorded.group(1),
                  "%s: Wordvorlage wurde verändert, SHA256 weicht ab" % source.name)


def check_docx_copies() -> None:
    pairs = [(CHECKLIST_DOCX, CHECKLIST_COPY), (QUESTIONNAIRE_DOCX, QUESTIONNAIRE_COPY)]
    for source, copy in pairs:
        check(copy.is_file(), "Wordkopie fehlt: %s" % copy.name)
        if not copy.is_file():
            continue
        source_doc = Document(str(source))
        copy_doc = Document(str(copy))
        empty_before = 0
        filled = 0
        without_color = []
        without_bold = []
        for index, (table_before, table_after) in enumerate(zip(source_doc.tables, copy_doc.tables),
                                                         start=1):
            for row_before, row_after in zip(table_before.rows, table_after.rows):
                for cell_before, cell_after in zip(row_before.cells, row_after.cells):
                    if normalize(cell_before.text):
                        continue
                    if not normalize(cell_after.text):
                        continue
                    empty_before += 1
                    filled += 1
                    runs = [run for paragraph in cell_after.paragraphs
                            for run in paragraph.runs if run.text.strip()]
                    if not runs:
                        continue
                    if not all(run.bold for run in runs):
                        without_bold.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
                    if not all(run.font.color and run.font.color.rgb == ANSWER_COLOR
                               for run in runs):
                        without_color.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
        check(filled > 0, "%s: keine Zelle gefüllt" % copy.name)
        check(not without_bold, "%s: %d Antworten ohne Fettung, erste: %s"
              % (copy.name, len(without_bold), without_bold[:2]))
        check(not without_color, "%s: %d Antworten ohne Markenblau, erste: %s"
              % (copy.name, len(without_color), without_color[:2]))
        note_runs = [run for paragraph in copy_doc.paragraphs
                     if paragraph.text.startswith("**")
                     for run in paragraph.runs if run.text.strip()]
        for run in note_runs:
            check(bool(run.bold) and run.font.color and run.font.color.rgb == ANSWER_COLOR,
                  "%s: Hinweiszeile ohne Fettung oder Markenblau" % copy.name)
            break
        print("  %-58s %d Zellen gefüllt, alle fett und in Markenblau" % (copy.name, filled))
        check(empty_before >= 30, "%s: nur %d Zellen gefüllt" % (copy.name, empty_before))


def check_extract_verifier() -> None:
    result = subprocess.run([sys.executable, "tools/verify_docx_extract.py"],
                            capture_output=True, text=True, cwd=str(ROOT))
    check(result.returncode == 0, "Extraktprüfer meldet Abweichungen:\n"
          + (result.stdout or "")[-400:])


def main() -> int:
    print("Prüfung der ausgefüllten Aalberts-Vorlagen")
    check_checklist()
    check_questionnaire()
    check_notes_and_sources()
    check_docx_copies()
    check_extract_verifier()
    print("\nPrüfungen: %d" % checks)
    if problems:
        print("ABWEICHUNGEN (%d):" % len(problems))
        for item in problems:
            print("  -", item)
        return 1
    print("Alle Prüfungen bestanden.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
