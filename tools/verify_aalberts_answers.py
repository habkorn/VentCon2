# -*- coding: utf-8 -*-
"""Prüft die ausgefüllten Aalberts-Vorlagen in `sec/AalbertsChecklists`.

Geprüft wird, dass jede Frage und jede Sicherheitsvorgabe in der Markdown-Quelle unter
`filled/src` eine fette Antwort trägt, dass keine Antwortzelle leer bleibt, dass die Hinweiszeilen
vorhanden sind, dass die Wordkopien aus dieser Quelle dieselben Antworten, Hinweise und
Leerzeilenwerte enthalten, dass jede in den Abgabedateien genannte Risiko-ID in der Anlage
aufgelöst wird und dass die Wordvorlagen selbst unverändert sind. Die Extrakte unter `extracts`
tragen nur Vorlagentext; das prüft dieses Werkzeug ausdrücklich mit. Zusätzlich läuft der
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
from docx.oxml.ns import qn
from docx.shared import RGBColor, Twips

ANSWER_COLOR = RGBColor(0x00, 0x2F, 0x87)

ROOT = Path(__file__).resolve().parent.parent
BASE_DIR = ROOT / "sec" / "AalbertsChecklists"
TEMPLATE_DIR = BASE_DIR / "templates"
EXTRACT_DIR = BASE_DIR / "extracts"
FILLED_DIR = BASE_DIR / "filled"

CHECKLIST = EXTRACT_DIR / "CRA_Product_Maturity_Checklist.md"
CHECKLIST_DOCX = TEMPLATE_DIR / "CRA product maturity checklist - Sep 2026.docx"
CHECKLIST_COPY = FILLED_DIR / "CRA_Product_Maturity_Checklist_answered.docx"
QUESTIONNAIRE = EXTRACT_DIR / "CRA_Supplier_Due_Diligence_Questionnaire.md"
QUESTIONNAIRE_DOCX = TEMPLATE_DIR / "CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx"
QUESTIONNAIRE_COPY = FILLED_DIR / "CRA_Supplier_Due_Diligence_Questionnaire_answered.docx"
ADDENDUM = EXTRACT_DIR / "CRA_Compliance_Addendum_Supplier_Agreement.md"
ADDENDUM_COPY = FILLED_DIR / "CRA_Compliance_Addendum_Supplier_Agreement_answered.docx"

# Quellen der Eintragungen: je Worddatei in `filled` eine Markdown-Datei in `filled/src`. Aus ihr
# wird die Worddatei gesetzt, die Extrakte bleiben Vorlagentext.
SOURCE_DIR = FILLED_DIR / "src"
CHECKLIST_SOURCE = SOURCE_DIR / "CRA_Product_Maturity_Checklist_answered.md"
QUESTIONNAIRE_SOURCE = SOURCE_DIR / "CRA_Supplier_Due_Diligence_Questionnaire_answered.md"
ANNEX = FILLED_DIR / "VentCon2_Measures_Reference_2026-10-02.docx"
ANNEX_SOURCE = SOURCE_DIR / "VentCon2_Measures_Reference_2026-10-02.md"

# Marke für Hinweiszeilen in der Markdown-Quelle: `<!-- note: <Anker> --> <Hinweistext>`.
NOTE_MARKER = "<!-- note:"

EXPECTED_PART_I = 20
EXPECTED_PART_II = 11
EXPECTED_QUESTIONS = 36
EXPECTED_NOTES = {CHECKLIST_SOURCE: 4, QUESTIONNAIRE_SOURCE: 2}
EXPECTED_FINDINGS = ["R%d" % number for number in range(1, 11)]

# Werte, die in die Leerzeilen der Vorlage eingetragen werden (Entscheidung E76).
BLANKS = ((CHECKLIST_SOURCE, CHECKLIST_COPY,
           (("Product:", ("VentCon2 valve control unit, firmware 2.7.4 (prototype)",
                           "VENTREX product owner, VentCon2 team", "2026-10-02")),)),
          (QUESTIONNAIRE_SOURCE, QUESTIONNAIRE_COPY,
           (("Completed by", ("Product owner, VENTREX (Aalberts group company)",)),
            ("Date:", ("2026-10-02",)))))

# Vorlage, Wordkopie, Spalte der Antwort und ob die Antwort mit einem Urteil beginnt.
COPIES = ((CHECKLIST_DOCX, CHECKLIST_COPY, 0, True),
          (QUESTIONNAIRE_DOCX, QUESTIONNAIRE_COPY, 1, False))

# Urteil am Anfang einer Antwort der Checkliste.
VERDICT = re.compile(r"^\*\*(Y|N|N/A)\*\*<br>\S")
VERDICTS = ("Y", "N", "N/A")

# Grenzen für die Aufteilung der Tabellenbreite (Entscheidung E64).
MIN_ANSWER_SHARE = 0.40
MAX_ANSWER_SHARE = 0.55
MIN_OTHER_WIDTH = 0.90
TOLERANCE = 0.005

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
    for number, cells in rows(CHECKLIST_SOURCE):
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
            check(bool(VERDICT.match(cells[0])),
                  "Checkliste Teil I, Zeile %d ohne getrenntes Urteil: %s"
                  % (number, cells[0][:40]))
        if part == 2 and len(cells) == 2 and cells[1]:
            part_two += 1
            check(bool(cells[0]), "Checkliste Teil II, Zeile %d ohne Antwort: %s"
                  % (number, cells[1][:60]))
            check("**" in cells[0], "Checkliste Teil II, Zeile %d ohne fette Antwort" % number)
            check(bool(VERDICT.match(cells[0])),
                  "Checkliste Teil II, Zeile %d ohne getrenntes Urteil: %s"
                  % (number, cells[0][:40]))
    check(part_one == EXPECTED_PART_I,
          "Checkliste Teil I: %d Zeilen mit Antwort, erwartet %d" % (part_one, EXPECTED_PART_I))
    check(part_two == EXPECTED_PART_II,
          "Checkliste Teil II: %d Zeilen mit Antwort, erwartet %d" % (part_two, EXPECTED_PART_II))
    text = CHECKLIST_SOURCE.read_text(encoding="utf-8")
    check("Overall result: Gaps identified" in text,
          "Checkliste: Ergebniszeile fehlt")
    check("Key gaps / evidence" in text and "**Evidence:" in text,
          "Checkliste: Lückenbegründung fehlt")
    print("  Checkliste: Teil I %d Antworten, Teil II %d Antworten" % (part_one, part_two))


def check_questionnaire() -> None:
    inside = False
    answered = 0
    for number, cells in rows(QUESTIONNAIRE_SOURCE):
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
    text = QUESTIONNAIRE_SOURCE.read_text(encoding="utf-8")
    for token in ("Accepted with conditions",
                  "**Product owner, VENTREX (Aalberts group company)**"):
        check(token in text, "Fragebogen: Sign-off ohne %s" % token)
    print("  Fragebogen: %d Antworten" % answered)


def source_texts_of(path: Path) -> set[str]:
    """Normalisierte Texte einer Wordvorlage, Absätze und Tabellenzellen."""
    document = Document(str(path))
    texts = {normalize(paragraph.text) for paragraph in document.paragraphs}
    for table in document.tables:
        for row in table.rows:
            for cell in row.cells:
                texts.add(normalize(cell.text))
    return texts


def all_text(path: Path) -> str:
    """Gesamter Text einer Worddatei, Absätze und Tabellenzellen."""
    document = Document(str(path))
    parts = [paragraph.text for paragraph in document.paragraphs]
    parts += [cell.text for table in document.tables for row in table.rows for cell in row.cells]
    return "\n".join(parts)


# HTML-Reste, die aus dem Markdown in die Worddatei geraten können. `<br>` ist die Trennung
# innerhalb einer Zelle der Quelle und muss dort bleiben.
HTML_ARTIFACTS = ("<br>", "<br/>", "<br />", "&nbsp;", "&amp;", "&lt;", "&gt;", "<p>", "</p>",
                  "<b>", "</b>", "<div", "<!--")


def source_of(path: Path) -> Path:
    """Wordvorlage zu einem Extrakt, gelesen aus dem Kopfblock."""
    head = path.read_text(encoding="utf-8").split("<!-- extract:head-end -->")[0]
    name = re.search(r"\| Source file \| `([^`]+)` \|", head).group(1).split("/")[-1]
    return TEMPLATE_DIR / name


def markdown_notes(path: Path) -> list[str]:
    """Hinweistexte der Markdown-Quelle, gelesen aus den Zeilen mit der Hinweismarke."""
    notes = []
    for line in path.read_text(encoding="utf-8").split("\n"):
        stripped = line.strip()
        if not stripped.startswith(NOTE_MARKER):
            continue
        anchor, _, note = stripped[len(NOTE_MARKER):].partition("-->")
        check(bool(anchor.strip()), "%s: Hinweiszeile ohne Anker" % path.name)
        check(bool(note.strip()), "%s: Hinweiszeile ohne Text" % path.name)
        notes.append(note.strip())
    return notes


def check_notes_and_sources() -> None:
    """Hinweiszeilen in den Quellen, Prüfsummen der Wordvorlagen in den Extrakten."""
    for path, expected in EXPECTED_NOTES.items():
        check(path.is_file(), "Quelle fehlt: %s" % path.name)
        if not path.is_file():
            continue
        notes = markdown_notes(path)
        check(len(notes) == expected, "%s: %d Hinweiszeilen, erwartet %d"
              % (path.name, len(notes), expected))
        print("  %-58s %d Hinweiszeilen" % (path.name, len(notes)))
    for path in (CHECKLIST, QUESTIONNAIRE, ADDENDUM):
        head = path.read_text(encoding="utf-8").split("<!-- extract:head-end -->")[0]
        source = source_of(path)
        recorded = re.search(r"\| SHA256 \| `([0-9a-f]{64})` \|", head)
        check(recorded is not None, "%s: SHA256 im Kopfblock fehlt" % path.name)
        if recorded:
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            check(digest == recorded.group(1),
                  "%s: Wordvorlage wurde verändert, SHA256 weicht ab" % source.name)


def check_docx_copies() -> None:
    for source, copy, _, verdict in COPIES:
        check(copy.is_file(), "Wordkopie fehlt: %s" % copy.name)
        if not copy.is_file():
            continue
        source_doc = Document(str(source))
        copy_doc = Document(str(copy))
        empty_before = 0
        filled = 0
        without_color = []
        without_bold = []
        with_marker = []
        without_verdict = []
        without_separation = []
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
                    if verdict:
                        # Erste Zeile ist das Urteil, sie steht fett in eigener Zeile.
                        if not runs[0].bold:
                            without_bold.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
                        if runs[0].text.strip() not in VERDICTS:
                            without_verdict.append("Tabelle %d: %s"
                                                   % (index, runs[0].text[:20]))
                        if len(runs) < 2 or not runs[0].text.endswith("\n"):
                            without_separation.append("Tabelle %d: %s"
                                                      % (index, cell_after.text[:40]))
                    elif not all(run.bold for run in runs):
                        without_bold.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
                    if not all(run.font.color and run.font.color.rgb == ANSWER_COLOR
                               for run in runs):
                        without_color.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
                    if any("**" in run.text for run in runs):
                        with_marker.append("Tabelle %d: %s" % (index, cell_after.text[:40]))
        check(filled > 0, "%s: keine Zelle gefüllt" % copy.name)
        check(not without_bold, "%s: %d Antworten ohne Fettung im Urteil, erste: %s"
              % (copy.name, len(without_bold), without_bold[:2]))
        check(not without_verdict, "%s: %d Zeilen ohne Urteil Y, N oder N/A, erste: %s"
              % (copy.name, len(without_verdict), without_verdict[:2]))
        check(not without_separation, "%s: %d Urteile ohne eigene Zeile, erste: %s"
              % (copy.name, len(without_separation), without_separation[:2]))
        check(not with_marker, "%s: %d Antworten mit Markdown-Marker, erste: %s"
              % (copy.name, len(with_marker), with_marker[:2]))
        check(not without_color, "%s: %d Antworten ohne Markenblau, erste: %s"
              % (copy.name, len(without_color), without_color[:2]))
        print("  %-58s %d Zellen gefüllt, alle fett und in Markenblau" % (copy.name, filled))
        check(empty_before >= 30, "%s: nur %d Zellen gefüllt" % (copy.name, empty_before))


def grid_widths(table) -> list[float]:
    """Spaltenbreiten aus dem Tabellengitter, in Zoll."""
    return [Twips(int(column.get(qn("w:w")))).inches
            for column in table._tbl.tblGrid.findall(qn("w:gridCol"))]


def check_column_widths() -> None:
    """Die Antwortspalte muss nach Textanteil breit genug sein, die Textbreite bleibt gleich."""
    for source_path, copy_path, answer_index, _ in COPIES:
        if not copy_path.is_file():
            check(False, "Wordkopie fehlt: %s" % copy_path.name)
            continue
        source = Document(str(source_path))
        copy = Document(str(copy_path))
        widths = []
        for number, (before, after) in enumerate(zip(source.tables, copy.tables), start=1):
            new = grid_widths(after)
            old = grid_widths(before)
            total = sum(new)
            widths.append(new[answer_index])
            check(abs(total - sum(old)) < 0.01,
                  "%s: Tabelle %d hat die Textbreite geändert (%.2f statt %.2f in)"
                  % (copy_path.name, number, total, sum(old)))
            share = new[answer_index] / total
            check(MIN_ANSWER_SHARE - TOLERANCE <= share <= MAX_ANSWER_SHARE + TOLERANCE,
                  "%s: Tabelle %d, Antwortspalte %.2f in, das sind %.0f %% statt %.0f bis %.0f %%"
                  % (copy_path.name, number, new[answer_index], share * 100,
                     MIN_ANSWER_SHARE * 100, MAX_ANSWER_SHARE * 100))
            for position, width in enumerate(new):
                if position != answer_index:
                    check(width >= MIN_OTHER_WIDTH - TOLERANCE,
                          "%s: Tabelle %d, Spalte %d nur %.2f in breit"
                          % (copy_path.name, number, position + 1, width))
        check(bool(widths) and max(widths) - min(widths) < 0.01,
              "%s: Antwortspalten unterschiedlich breit: %s" % (copy_path.name, widths))
        print("  %-58s Antwortspalte %.2f in, einheitlich über %d Tabellen"
              % (copy_path.name, widths[0], len(widths)))


def check_addendum_excluded() -> None:
    """Das Addendum ist mit E62 aus der Antwortstrecke genommen und darf dort nicht auftauchen."""
    check(not ADDENDUM_COPY.exists(),
          "Addendum liegt wieder als Wordkopie in filled/: %s" % ADDENDUM_COPY.name)
    text = ADDENDUM.read_text(encoding="utf-8")
    check("| Answers |" not in text, "Addendum-Extrakt trägt wieder eine Antwortzeile")
    check(not any(line.startswith("**") for line in text.split("\n")),
          "Addendum-Extrakt enthält wieder fette Antwortzeilen")


def check_copy_notes() -> None:
    """Eingesetzte Hinweise dürfen keine Marker tragen und müssen fett im Markenblau stehen."""
    for md_path, copy in ((CHECKLIST_SOURCE, CHECKLIST_COPY),
                          (QUESTIONNAIRE_SOURCE, QUESTIONNAIRE_COPY)):
        if not copy.is_file():
            check(False, "Wordkopie fehlt: %s" % copy.name)
            continue
        document = Document(str(copy))
        notes = [normalize(line) for line in markdown_notes(md_path)]
        check(bool(notes), "%s: keine Hinweiszeile im Markdown gefunden" % md_path.name)
        paragraphs = {normalize(paragraph.text): paragraph
                      for paragraph in document.paragraphs}
        missing = [note for note in notes if note not in paragraphs]
        check(not missing, "%s: %d Hinweiszeilen fehlen im Worddokument, erste: %s"
              % (copy.name, len(missing), [note[:60] for note in missing[:1]]))
        for note in notes:
            paragraph = paragraphs.get(note)
            if paragraph is None:
                continue
            runs = [run for run in paragraph.runs if run.text.strip()]
            check(not any("**" in run.text for run in runs),
                  "%s: Hinweiszeile trägt Markdown-Marker" % copy.name)
            check(bool(runs) and all(run.bold and run.font.color
                                     and run.font.color.rgb == ANSWER_COLOR for run in runs),
                  "%s: Hinweiszeile ohne Fettung oder Markenblau" % copy.name)
        print("  %-58s %d Hinweiszeilen ohne Marker, fett und in Markenblau"
              % (copy.name, len(notes)))


def check_blank_fills() -> None:
    """Die Leerzeilen der Vorlage müssen gefüllt und die Werte fett im Markenblau sein."""
    for markdown_path, copy_path, entries in BLANKS:
        if not copy_path.is_file():
            check(False, "Wordkopie fehlt: %s" % copy_path.name)
            continue
        text = markdown_path.read_text(encoding="utf-8")
        document = Document(str(copy_path))
        for prefix, values in entries:
            paragraph = next((item for item in document.paragraphs
                              if item.text.strip().startswith(prefix)), None)
            check(paragraph is not None, "%s: Zeile '%s' fehlt" % (copy_path.name, prefix))
            if paragraph is None:
                continue
            check("__" not in paragraph.text,
                  "%s: Zeile '%s' hat noch Leerstellen: %s"
                  % (copy_path.name, prefix, paragraph.text[:60]))
            for value in values:
                check(value in paragraph.text, "%s: '%s' fehlt in der Zeile '%s'"
                      % (copy_path.name, value[:38], prefix))
                check("**%s**" % value in text, "%s: '%s' steht nicht fett in der Quelle"
                      % (markdown_path.name, value[:38]))
            runs = [run for run in paragraph.runs if run.text.strip() in values]
            check(len(runs) == len(values), "%s: %d von %d Werten in '%s' gefunden"
                  % (copy_path.name, len(runs), len(values), prefix))
            for run in runs:
                check(bool(run.bold) and run.font.color
                      and run.font.color.rgb == ANSWER_COLOR,
                      "%s: '%s' nicht fett im Markenblau" % (copy_path.name, run.text[:30]))
            print("  %-58s %s: %d Werte eingetragen"
                  % (copy_path.name, prefix.rstrip(":"), len(runs)))


def check_row_split_protection() -> None:
    """Keine Tabellenzeile darf über einen Seitenwechsel laufen (Entscheidung E77)."""
    tag = qn("w:cantSplit")
    for _, copy_path, _, _ in COPIES:
        if not copy_path.is_file():
            check(False, "Wordkopie fehlt: %s" % copy_path.name)
            continue
        document = Document(str(copy_path))
        rows = [row for table in document.tables for row in table.rows]
        without = [row for row in rows
                   if row._tr.trPr is None or row._tr.trPr.find(tag) is None]
        check(not without, "%s: %d von %d Tabellenzeilen ohne Umbruchschutz"
              % (copy_path.name, len(without), len(rows)))
        print("  %-58s %d Tabellenzeilen ohne Seitenumbruch" % (copy_path.name, len(rows)))


def check_extracts_are_template_only() -> None:
    """Die Extrakte tragen nur Vorlagentext; die Eintragungen stehen in `filled/src`."""
    for path in (CHECKLIST, QUESTIONNAIRE, ADDENDUM):
        text = path.read_text(encoding="utf-8")
        check("| Answers |" not in text, "%s: Antwortzeile im Extrakt" % path.name)
        check(NOTE_MARKER not in text, "%s: Hinweiszeile im Extrakt" % path.name)
        check(re.search(r"\*\*(Y|N|N/A)\*\*<br>", text) is None,
              "%s: Urteil im Extrakt" % path.name)
        print("  %-58s nur Vorlagentext" % path.name)


def check_source_files_exist() -> None:
    """Je Worddatei in `filled` muss eine Markdown-Quelle in `filled/src` liegen."""
    for name in ("CRA_Product_Maturity_Checklist_answered", "CRA_Supplier_Due_Diligence_"
                 "Questionnaire_answered", "VentCon2_Measures_Reference_2026-10-02"):
        check((FILLED_DIR / (name + ".docx")).is_file(),
              "Abgabedatei fehlt: %s.docx" % name)
        check((SOURCE_DIR / (name + ".md")).is_file(),
              "Quelle fehlt: filled/src/%s.md" % name)
    print("  %-58s %d Abgabedateien mit Quelle" % ("filled und filled/src", 3))


def check_no_html_artifacts() -> None:
    """Keine HTML- oder Markdown-Reste in den Abgabedateien.

    Die Quellen sind Markdown, die Abgabedateien sind Word. Was in der Quelle Auszeichnung ist,
    muss beim Setzen der Worddatei verschwinden: `<br>` wird zur Zeilenschaltung, `**` zur fetten
    Schrift. Bleibt eines davon als Text stehen, sieht die Leserin bei Aalberts Markup.
    """
    for path in (CHECKLIST_COPY, QUESTIONNAIRE_COPY, ANNEX):
        if not path.is_file():
            check(False, "Abgabedatei fehlt: %s" % path.name)
            continue
        text = all_text(path)
        for token in HTML_ARTIFACTS:
            check(token not in text, "%s: HTML-Rest %r in der Worddatei" % (path.name, token))
        print("  %-58s keine HTML-Reste" % path.name)


def check_marker_counts() -> None:
    """Markdown-Marker dürfen nur dort stehen, wo die Vorlage sie schon führt.

    Die Aalberts-Checkliste trägt im Anhang eigener Text mit `**`; die Eintragungen dürfen keinen
    weiteren Marker hinzufügen.
    """
    for template, copy in ((CHECKLIST_DOCX, CHECKLIST_COPY),
                           (QUESTIONNAIRE_DOCX, QUESTIONNAIRE_COPY)):
        if not copy.is_file():
            check(False, "Wordkopie fehlt: %s" % copy.name)
            continue
        expected = all_text(template).count("**")
        found = all_text(copy).count("**")
        check(found == expected, "%s: %d Marker '**' in der Worddatei, in der Vorlage %d"
              % (copy.name, found, expected))
        print("  %-58s %d Marker '**' wie in der Vorlage" % (copy.name, found))


def check_annex_reference() -> None:
    """Die Anlage muss vorhanden sein, in beiden Wordkopien genannt werden und jede dort genannte
    Risiko-ID auflösen."""
    check(ANNEX.is_file(), "Maßnahmen-Anlage fehlt: %s" % ANNEX.name)
    check(ANNEX_SOURCE.is_file(), "Anlage-Quelle fehlt: %s" % ANNEX_SOURCE.name)
    if not ANNEX.is_file():
        return
    defined: set[str] = set()
    annex = Document(str(ANNEX))
    finding_tables = 0
    for table in annex.tables:
        header = [cell.text.strip() for cell in table.rows[0].cells]
        if header != ["ID", "Finding", "Measure", "Status"]:
            continue
        finding_tables += 1
        for row in table.rows[1:]:
            identifier = row.cells[0].text.strip()
            if re.fullmatch(r"R\d+", identifier):
                defined.add(identifier)
    check(finding_tables == 1, "Anlage führt %d Befundtabellen, erwartet 1" % finding_tables)
    check(defined == set(EXPECTED_FINDINGS),
          "Anlage löst nicht alle Befunde auf: %s"
          % sorted(set(EXPECTED_FINDINGS) - defined, key=lambda name: int(name[1:])))
    for _, copy in ((CHECKLIST_SOURCE, CHECKLIST_COPY),
                    (QUESTIONNAIRE_SOURCE, QUESTIONNAIRE_COPY)):
        if not copy.is_file():
            continue
        document = Document(str(copy))
        text = all_text(copy)
        check(ANNEX.name in text, "%s: kein Verweis auf %s" % (copy.name, ANNEX.name))
        check("findings R1 to R10" in text, "%s: Hinweis auf die Risikoliste fehlt" % copy.name)
        mentioned = set(re.findall(r"\bR\d+\b", text))
        unresolved = sorted(mentioned - defined, key=lambda name: int(name[1:]))
        check(not unresolved, "%s: genannte Befunde ohne Auflösung in der Anlage: %s"
              % (copy.name, unresolved))
        print("  %-58s %d Befunde genannt, alle aufgelöst" % (copy.name, len(mentioned)))


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
    check_column_widths()
    check_copy_notes()
    check_addendum_excluded()
    check_blank_fills()
    check_row_split_protection()
    check_no_html_artifacts()
    check_marker_counts()
    check_source_files_exist()
    check_extracts_are_template_only()
    check_annex_reference()
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
