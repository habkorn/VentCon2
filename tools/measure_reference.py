# -*- coding: utf-8 -*-
"""Erzeugt die Maßnahmen-Anlage für Aalberts als Worddatei aus der VENTREX-Vorlage.

Auftrag vom 2026-10-02: Die Antworten in Checkliste und Fragebogen verweisen auf Kennungen wie
`measure P4.2`. Diese Anlage löst jede Kennung auf, damit eine Leserin bei Aalberts das interne
Dossier nicht öffnen muss. Kennung, Titel, Kurzbeschreibung, Stand, Risikobezug und CRA-Pflicht-
Nummern stehen in einer Tabelle, eine Zeile je Maßnahme. Die Phase steht nicht in der Tabelle,
weil die Kennung sie trägt; die Detaildokumente stehen bewusst ebenfalls nicht darin.

Quelle der Struktur ist `sec/CRA_Compliance_Checklist.md`, Abschnitt 7 (Risiko, Pflicht-Nummer,
Kennung mit Titel, Detaildokument, Stand) und Abschnitt 8 (Phase, Kennzeichnung aktiv oder ruht).
Der Text der Anlage ist Englisch, wie die Antworten. Die englischen Formulierungen stehen in diesem
Werkzeug; die Kennungen und Detaildokumente werden gegen den Master geprüft, damit eine Änderung
dort sofort auffällt.

Vorlage: `sec/VENTREX Wordvorlage_ohne Fußzeile.docx` (A4, Kopf- und Fußgrafik, Themenschriften
Clash Display und Satoshi, keine Überschriftenformate). Die Vorlage bleibt unberührt.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/measure_reference.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor

ROOT = Path(__file__).resolve().parent.parent
MASTER = ROOT / "sec" / "CRA_Compliance_Checklist.md"
TEMPLATE = ROOT / "sec" / "VENTREX Wordvorlage_ohne Fußzeile.docx"
OUTPUT = ROOT / "sec" / "AalbertsChecklists" / "filled" / "VentCon2_Measures_Reference_2026-10-02.docx"

BRAND = RGBColor(0x00, 0x2F, 0x87)
GREY = RGBColor(0x59, 0x59, 0x59)
HEADING_FONT = "Clash Display"
BODY_FONT = "Satoshi"
BOX_FILL = "F2F5FA"

# Spalten der Tabelle, Breiten in Zoll, Summe wird auf die Satzbreite der Vorlage skaliert.
# Die Phase steht nicht in der Tabelle: die Kennung trägt sie bereits (P4.2 gehört zu Phase 4).
COLUMNS = (("ID", 0.42), ("Measure and scope", 2.95), ("Status", 1.55),
           ("Risk / CRA duty", 1.38))

STATUS_OPEN = "Open"
STATUS_PROGRESS = "In progress"
STATUS_DONE = "Done"
RESTING = "rests until the market decision"
ACTIVE = "active in prototype operation"

# Zusatz zum Stand, wo der Master mehr sagt als das Grundwort.
QUALIFIERS = {"P2.1": "partly designed (JSON lock)",
              "P3.1": "in v2.7.3",
              "P3.2": "in v2.7.3"}

# Eine Zeile je Maßnahme: Kennung, englischer Titel, englische Kurzbeschreibung, Phase,
# Risikobezug, CRA-Pflicht-Nummern, Stand. Kennung und Detaildokument werden gegen den Master
# geprüft, die englischen Texte kommen aus diesem Werkzeug.
MEASURES = (
    ("P1.1", "Authentication for all mutating endpoints",
     "Login page and first-start provisioning for every endpoint that changes state.",
     1, "R1", "2", STATUS_PROGRESS),
    ("P1.2", "POST migration and CSRF protection",
     "Move /set and /resetPID from GET to POST and protect them against cross-site requests.",
     1, "R4", "6", STATUS_PROGRESS),
    ("P1.3", "Limit the sensor calibration to the physical sensor range",
     "Reject calibration values above the sensor range (sensor_maxP <= SENSOR_MAX_BAR).",
     1, "R3", "9", STATUS_PROGRESS),
    ("P1.4", "Hard upper limits for PID gains",
     "Cap the PID gains so a wrong configuration cannot drive the valve to its limits.",
     1, "R6", "4", STATUS_OPEN),
    ("P1.5", "Clarify the EN 18031 conformity route and the CE route",
     "Radio equipment duties of the RED, valid until 2027-12-11.",
     1, "RED-1, RED-2", "RED-1, RED-2", STATUS_OPEN),
    ("P2.1", "Path sanitisation and no configuration over HTTP",
     "Sanitise handleFileRead() and stop serving the JSON configuration over HTTP.",
     2, "R5", "3", STATUS_PROGRESS),
    ("P2.2", "Access point password configurable at runtime",
     "Remove the password that is fixed in the source and make it configurable.",
     2, "R2", "1, 10", STATUS_PROGRESS),
    ("P2.3", "Rate limiting and save debouncing",
     "Limit request rates and debounce settings writes to protect the flash memory.",
     2, "R7", "8", STATUS_OPEN),
    ("P3.1", "Mutex for settings access",
     "Serialise concurrent access to the settings structure.",
     3, "R9", "no Annex I point (robustness)", STATUS_DONE),
    ("P3.2", "Passive rejection of new clients instead of dropping all",
     "A further client is rejected instead of disconnecting the existing connection.",
     3, "R8", "8", STATUS_DONE),
    ("P3.3", "Encryption or authentication at the radio level",
     "Protect the wireless link itself, today the access point has no own authentication.",
     3, "radio layer without own authentication", "5", STATUS_OPEN),
    ("P4.1", "Document the cybersecurity risk assessment",
     "Written risk assessment for the product, the interim basis today is the security analysis.",
     4, "no risk reference (standard duty)", "16", STATUS_OPEN),
    ("P4.2", "SBOM and due diligence for third-party components",
     "Machine-readable bill of materials for Arduino core, Arduino-ESP32, LittleFS, chart.js and "
     "the further libraries.",
     4, "no risk reference (standard duty)", "11, 22", STATUS_OPEN),
    ("P4.3", "Signed OTA updates and update tests",
     "Implement the signed update mechanics and test the update path.",
     4, "no risk reference (standard duty)", "7, 13, 15", STATUS_OPEN),
    ("P4.4", "Support period and end date",
     "Define the support period, at least five years or the expected service life if shorter, and "
     "document the end date.",
     4, "no risk reference (standard duty)", "20", STATUS_OPEN),
    ("P4.5", "Vulnerability management and disclosure policy",
     "Coordinated disclosure policy plus a security contact and the remediation process.",
     4, "no risk reference (standard duty)", "12, 14", STATUS_OPEN),
    ("P4.6", "Technical documentation in the structure of Annex VII",
     "Assemble the technical documentation required by the CRA.",
     4, "no risk reference (standard duty)", "17", STATUS_OPEN),
    ("P4.7", "EU declaration of conformity and CE marking",
     "Declaration of conformity in the structure of Annex V and the CE marking.",
     4, "no risk reference (standard duty)", "18, 19", STATUS_OPEN),
    ("P4.8", "User information in the structure of Annex II",
     "Update the user information with the security relevant content required by the CRA.",
     4, "no risk reference (standard duty)", "21", STATUS_OPEN),
    ("P5.1", "Reporting process for exploited vulnerabilities and severe incidents",
     "Internal process for the reporting duties of 24 hours, 72 hours, 14 days and one month.",
     5, "no risk reference (reporting duty)", "23, 25, 26", STATUS_OPEN),
    ("P5.2", "Access to the ENISA Single Reporting Platform and the competent CSIRT",
     "Register the reporting access required for the reporting duties.",
     5, "no risk reference (reporting duty)", "24", STATUS_OPEN),
)


def master_rows(text: str) -> dict[str, str]:
    """Kennung und Detaildokument aus Abschnitt 7 des Masters."""
    rows = {}
    pattern = re.compile(r"^\| .+? \| .+? \| (P\d\.\d) (.+?) \| (.+?) \| (.+?) \|$")
    for line in text.split("\n"):
        found = pattern.match(line.strip())
        if found:
            document = found.group(3).strip().strip("`")
            rows[found.group(1)] = document if document.startswith("plan/") else ""
    return rows


def identifiers(text: str) -> set[str]:
    """Kennungen im Text, Bereiche wie 'P4.1 bis P4.8' aufgelöst."""
    found: set[str] = set()
    for start_phase, start_number, _, end_number in re.findall(
            r"P(\d)\.(\d) bis P(\d)\.(\d)", text):
        for number in range(int(start_number), int(end_number) + 1):
            found.add("P%s.%d" % (start_phase, number))
    for phase, number in re.findall(r"P(\d)\.(\d)", text):
        found.add("P%s.%s" % (phase, number))
    return found


def master_flags(text: str) -> tuple[set[str], set[str]]:
    """Aktive Maßnahmen aus den Aufgabenzellen, ruhende aus dem Vorspann von Abschnitt 8."""
    active = {match.group(1) for match in
              re.finditer(r"^- \[[ x]\] (P\d\.\d) .*\(aktiv\)\s*$", text, re.M)}
    flat = " ".join(text.split())
    found = re.search(r"Zurückgestellt sind bis zur Marktentscheidung (.*?)\. ", flat)
    resting = identifiers(found.group(1)) if found else set()
    return active, resting


def master_lines(text: str) -> dict[str, str]:
    """Aufgabentext und Kennzeichnung aus Abschnitt 8 des Masters."""
    lines = {}
    for line in text.split("\n"):
        found = re.match(r"^- \[[ x]\] (P\d\.\d) (.+)$", line.strip())
        if found:
            lines[found.group(1)] = found.group(2)
    return lines


def master_phases(text: str) -> dict[str, str]:
    """Phase je Maßnahme aus den Überschriften von Abschnitt 8."""
    phases: dict[str, str] = {}
    phase = ""
    for line in text.split("\n"):
        heading = re.match(r"^### Phase (\d)", line.strip())
        if heading:
            phase = heading.group(1)
        bullet = re.match(r"^- \[[ x]\] (P\d\.\d) ", line.strip())
        if bullet and phase:
            phases[bullet.group(1)] = phase
    return phases


def check_master(rows: dict[str, str], phases: dict[str, str]) -> list[str]:
    """Abgleich der eigenen Liste mit dem Master, damit keine Kennung still verschwindet."""
    problems = []
    known = {entry[0] for entry in MEASURES}
    for identifier in sorted(set(rows) - known):
        problems.append("Maßnahme %s steht im Master, fehlt aber im Werkzeug" % identifier)
    for identifier, _, _, phase, _, _, _ in MEASURES:
        if identifier not in rows:
            problems.append("Maßnahme %s fehlt im Master" % identifier)
        if str(phase) != phases.get(identifier):
            problems.append("Maßnahme %s steht im Master unter Phase %s statt %d"
                            % (identifier, phases.get(identifier), phase))
    return problems


def check_documents(rows: dict[str, str]) -> list[str]:
    """Detaildokumente gegen die eigene Liste und das Dateisystem prüfen."""
    problems = []
    for entry in MEASURES:
        identifier = entry[0]
        document = rows.get(identifier, "")
        if document and not (ROOT / "sec" / document).is_file():
            problems.append("Detaildokument fehlt: %s (%s)" % (document, identifier))
    return problems


def style_run(run, size: float, *, bold: bool = False, colour=BRAND, font: str = BODY_FONT) -> None:
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.color.rgb = colour
    run.font.name = font


def shade(cell, fill: str) -> None:
    properties = cell._tc.get_or_add_tcPr()
    element = properties.find(qn("w:shd"))
    if element is None:
        element = properties.makeelement(qn("w:shd"), {})
        properties.append(element)
    element.set(qn("w:val"), "clear")
    element.set(qn("w:color"), "auto")
    element.set(qn("w:fill"), fill)


def set_widths(table, widths) -> None:
    """Gitter und jede Zelle setzen, damit Word die Breiten übernimmt."""
    for column, width in zip(table._tbl.tblGrid.findall(qn("w:gridCol")), widths):
        column.set(qn("w:w"), str(int(Inches(width).twips)))
    for row in table.rows:
        if len(row.cells) != len(widths):
            continue
        for cell, width in zip(row.cells, widths):
            cell.width = Inches(width)
    table.autofit = False


def add_box(document, text: str, width: float) -> None:
    """Hinweiskasten ohne Rahmen, eine Zelle mit leichtem Hintergrund."""
    table = document.add_table(rows=1, cols=1)
    table.style = document.styles["Normal Table"]
    cell = table.rows[0].cells[0]
    cell.width = Inches(width)
    shade(cell, BOX_FILL)
    paragraph = cell.paragraphs[0]
    paragraph.paragraph_format.space_before = Pt(4)
    paragraph.paragraph_format.space_after = Pt(4)
    run = paragraph.add_run(text)
    style_run(run, 8.5, colour=RGBColor(0x1F, 0x1F, 0x1F))
    set_widths(table, [width])


def keep_rows_together(table) -> int:
    """Tabellenzeilen nicht über Seiten umbrechen, sonst steht die halbe Zeile am Blattende."""
    tag = qn("w:cantSplit")
    count = 0
    for row in table.rows:
        properties = row._tr.get_or_add_trPr()
        if properties.find(tag) is not None:
            continue
        element = properties.makeelement(tag, {})
        for successor in ("w:trHeight", "w:tblHeader"):
            anchor = properties.find(qn(successor))
            if anchor is not None:
                anchor.addprevious(element)
                break
        else:
            properties.append(element)
        count += 1
    return count


def build(active: set[str], resting: set[str]) -> None:
    document = Document(str(TEMPLATE))
    section = document.sections[0]
    usable = (section.page_width.inches - section.left_margin.inches - section.right_margin.inches)
    scale = usable / sum(width for _, width in COLUMNS)
    widths = [round(width * scale, 4) for _, width in COLUMNS]
    body = document.paragraphs[0]
    body.clear()
    body.paragraph_format.space_after = Pt(2)

    title = body.add_run("VentCon2: Measures Referenced in the Aalberts Answers")
    style_run(title, 19, bold=True, font=HEADING_FONT)

    subtitle = document.add_paragraph()
    subtitle.paragraph_format.space_after = Pt(2)
    style_run(subtitle.add_run("Annex to the CRA product maturity checklist and to the supplier "
                               "due diligence questionnaire, reference date 2026-10-02."), 10)

    meta = document.add_paragraph()
    meta.paragraph_format.space_after = Pt(8)
    style_run(meta.add_run("Source: sec/CRA_Compliance_Checklist.md, the single status source in "
                           "the VENTREX dossier (section 7 and section 8). Measures P1.1 to P5.2, "
                           "21 items. Duty numbers refer to the obligation list 1 to 26 in "
                           "section 6 of the same file."),
              9, colour=GREY)

    add_box(document, "Market trigger: the duties arise with the placing of the product on the "
                      "Union market (Art. 2(1), Art. 3(21) and (22) CRA, Recital 15). As long as "
                      "the product is distributed in third countries only, no duties arise from "
                      "the CRA or from the RED cyber provisions; implementation is voluntary then. "
                      "Measures marked with \"rests until the market decision\" are not being "
                      "implemented today, measures marked with \"active in prototype operation\" "
                      "are.", usable)

    spacer = document.add_paragraph()
    spacer.paragraph_format.space_after = Pt(2)

    table = document.add_table(rows=1, cols=len(COLUMNS))
    table.style = document.styles["Table Grid"]
    header = table.rows[0]
    header._tr.get_or_add_trPr().append(header._tr.makeelement(qn("w:tblHeader"), {}))
    for cell, (name, _) in zip(header.cells, COLUMNS):
        cell.paragraphs[0].clear()
        shade(cell, "002F87")
        paragraph = cell.paragraphs[0]
        paragraph.paragraph_format.space_after = Pt(1)
        style_run(paragraph.add_run(name), 8, bold=True, colour=RGBColor(0xFF, 0xFF, 0xFF))

    for identifier, name, description, _phase, risk, duty, status in MEASURES:
        suffix = [QUALIFIERS[identifier]] if identifier in QUALIFIERS else []
        if identifier in active:
            suffix.append(ACTIVE)
        elif identifier in resting:
            suffix.append(RESTING)
        status_text = ", ".join([status] + suffix)
        cells = table.add_row().cells
        for cell in cells:
            cell.paragraphs[0].paragraph_format.space_after = Pt(0)
        style_run(cells[0].paragraphs[0].add_run(identifier), 8, bold=True)
        style_run(cells[1].paragraphs[0].add_run(name + " "), 8, bold=True)
        style_run(cells[1].paragraphs[0].add_run(description), 8)
        style_run(cells[2].paragraphs[0].add_run(status_text), 8)
        style_run(cells[3].paragraphs[0].add_run("%s / duty %s" % (risk, duty)), 8)

    set_widths(table, widths)
    keep_rows_together(table)

    closing = document.add_paragraph()
    closing.paragraph_format.space_before = Pt(8)
    style_run(closing.add_run("Status as documented on 2026-10-02. The master checklist remains the "
                              "single status source; this annex is an extract of it."), 8,
              colour=GREY)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document.save(str(OUTPUT))


def main() -> int:
    text = MASTER.read_text(encoding="utf-8")
    rows = master_rows(text)
    lines = master_lines(text)
    phases = master_phases(text)
    active, resting = master_flags(text)
    problems = check_master(rows, phases) + check_documents(rows)
    if problems:
        print("Abgleich mit dem Master fehlgeschlagen:")
        for problem in problems:
            print("  -", problem)
        return 1
    print("Abgleich mit dem Master: %d Maßnahmen, %d Zeilen in Abschnitt 8, %d mit Phase, "
          "%d aktiv, %d ruhend"
          % (len(rows), len(lines), len(phases), len(active), len(resting)))
    build(active, resting)
    print("Geschrieben: %s" % OUTPUT.relative_to(ROOT).as_posix())
    return 0


if __name__ == "__main__":
    sys.exit(main())
