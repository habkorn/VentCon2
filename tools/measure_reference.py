# -*- coding: utf-8 -*-
"""Erzeugt die Maßnahmen-Anlage für Aalberts aus ihrer Markdown-Quelle.

Auftrag vom 2026-10-02, erweitert am 2026-10-06: Die Anlage löst jede Kennung auf, die in den
Aalberts-Antworten vorkommt, die Maßnahmen `P1.1` bis `P5.2` und die Befunde `R1` bis `R10`,
damit eine Leserin bei Aalberts das interne Dossier nicht öffnen muss. Die Anlage führt deshalb
zwei Tabellen: die Maßnahmen mit Stand, Befund und CRA-Pflicht-Nummern sowie die Befunde mit
Befundtext, Maßnahme und Stand.

Quelle der Struktur ist `sec/CRA_Compliance_Checklist.md`, Abschnitt 7 (Befundspalte, Pflicht-
Nummer, Kennung mit Titel, Detaildokument, Stand) und Abschnitt 8 (Phase, Kennzeichnung aktiv oder
ruht). Der Text der Anlage ist Englisch, wie die Antworten. Die englischen Befundtexte stehen in
diesem Werkzeug; Kennungen, Pflicht-Nummern und Stand werden gegen den Master geprüft, und der
Abgleich bricht ab, wenn der Master einen Befund führt, der hier fehlt, oder umgekehrt.

Ablauf: Das Werkzeug schreibt zuerst die Markdown-Quelle
`sec/AalbertsChecklists/filled/src/VentCon2_Measures_Reference_2026-10-02.md` und setzt daraus die
Worddatei `sec/AalbertsChecklists/filled/VentCon2_Measures_Reference_2026-10-02.docx` zusammen.
Die Worddatei ist die Umsetzung der Markdown-Datei, die Markdown-Datei ist die lesbare und
prüfbare Fassung. Aufbau der Markdown-Datei: Titel als `# `-Zeile, Untertitel als erster Absatz,
ein Kopfblock zwischen `<!-- annex:head-start -->` und `<!-- annex:head-end -->` mit den Feldern
`Source`, `Market trigger`, `Closing` und `Revision`, danach je Tabelle eine `## `-Überschrift und
die Tabelle. Fett gesetzter Text steht zwischen `**`, wie in Markdown üblich.

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
BASE_DIR = ROOT / "sec" / "AalbertsChecklists"
MASTER = ROOT / "sec" / "CRA_Compliance_Checklist.md"
TEMPLATE = ROOT / "sec" / "VENTREX Wordvorlage_ohne Fußzeile.docx"
SOURCE = BASE_DIR / "filled" / "src" / "VentCon2_Measures_Reference_2026-10-02.md"
OUTPUT = BASE_DIR / "filled" / "VentCon2_Measures_Reference_2026-10-02.docx"

HEAD_START = "<!-- annex:head-start -->"
HEAD_END = "<!-- annex:head-end -->"

BRAND = RGBColor(0x00, 0x2F, 0x87)
GREY = RGBColor(0x59, 0x59, 0x59)
HEADING_FONT = "Clash Display"
BODY_FONT = "Satoshi"
BOX_FILL = "F2F5FA"

# Spalten der Maßnahmentabelle, Breiten in Zoll, Summe wird auf die Satzbreite der Vorlage
# skaliert. Die Phase steht nicht in der Tabelle: die Kennung trägt sie bereits (P4.2 gehört zu
# Phase 4). Befund und CRA-Pflicht stehen in zwei Spalten, damit eine Verneinung wie
# "none (standard duty)" nicht wie eine Pflicht-Nummer aussieht.
COLUMNS = (("ID", 0.44), ("Measure and scope", 2.36), ("Status", 1.35), ("Finding", 0.85),
           ("CRA duty", 1.30))

# Spalten der Befundtabelle. Der Stand ist derselbe wie in der Maßnahmenzeile, damit beide
# Tabellen nicht auseinanderlaufen können.
FINDING_COLUMNS = (("ID", 0.44), ("Finding", 3.56), ("Measure", 0.90), ("Status", 1.40))

# Spaltenaufteilung je Tabelle, in der Markdown-Quelle über die Kopfzeile adressiert.
LAYOUTS = {tuple(name for name, _ in COLUMNS): COLUMNS,
           tuple(name for name, _ in FINDING_COLUMNS): FINDING_COLUMNS}

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
# CRA-Pflicht-Nummern, Stand. Kennung, Pflicht-Nummern und Stand werden gegen Abschnitt 7 und 8
# des Masters geprüft, die englischen Texte kommen aus diesem Werkzeug.
MEASURES = (
    ("P1.1", "Authentication for all mutating endpoints",
     "Login page and first-start provisioning for every endpoint that changes state.",
     1, "2", STATUS_PROGRESS),
    ("P1.2", "POST migration and CSRF protection",
     "Move /set and /resetPID from GET to POST and protect them against cross-site requests.",
     1, "6", STATUS_PROGRESS),
    ("P1.3", "Limit the sensor calibration to the physical sensor range",
     "Reject calibration values above the sensor range (sensor_maxP <= SENSOR_MAX_BAR).",
     1, "9", STATUS_PROGRESS),
    ("P1.4", "Hard upper limits for PID gains",
     "Cap the PID gains so a wrong configuration cannot drive the valve to its limits.",
     1, "4", STATUS_OPEN),
    ("P1.5", "Clarify the EN 18031 conformity route and the CE route",
     "Radio equipment duties of the RED, valid until 2027-12-11.",
     1, "RED-1, RED-2", STATUS_OPEN),
    ("P2.1", "Path sanitisation and no configuration over HTTP",
     "Sanitise handleFileRead() and stop serving the JSON configuration over HTTP.",
     2, "3", STATUS_PROGRESS),
    ("P2.2", "Access point password configurable at runtime",
     "Remove the password that is fixed in the source and make it configurable.",
     2, "1, 10", STATUS_PROGRESS),
    ("P2.3", "Rate limiting and save debouncing",
     "Limit request rates and debounce settings writes to protect the flash memory.",
     2, "8", STATUS_OPEN),
    ("P3.1", "Mutex for settings access",
     "Serialise concurrent access to the settings structure.",
     3, "no Annex I point (robustness)", STATUS_DONE),
    ("P3.2", "Passive rejection of new clients instead of dropping all",
     "A further client is rejected instead of disconnecting the existing connection.",
     3, "8", STATUS_DONE),
    ("P3.3", "Encryption or authentication at the radio level",
     "Protect the wireless link itself, today the access point has no own authentication.",
     3, "5", STATUS_OPEN),
    ("P4.1", "Document the cybersecurity risk assessment",
     "Written risk assessment for the product, the interim basis today is the security analysis.",
     4, "16", STATUS_OPEN),
    ("P4.2", "SBOM and due diligence for third-party components",
     "Machine-readable bill of materials for Arduino core, Arduino-ESP32, LittleFS, chart.js and "
     "the further libraries.",
     4, "11, 22", STATUS_OPEN),
    ("P4.3", "Signed OTA updates and update tests",
     "Implement the signed update mechanics and test the update path.",
     4, "7, 13, 15", STATUS_OPEN),
    ("P4.4", "Support period and end date",
     "Define the support period, at least five years or the expected service life if shorter, and "
     "document the end date.",
     4, "20", STATUS_OPEN),
    ("P4.5", "Vulnerability management and disclosure policy",
     "Coordinated disclosure policy plus a security contact and the remediation process.",
     4, "12, 14", STATUS_OPEN),
    ("P4.6", "Technical documentation in the structure of Annex VII",
     "Assemble the technical documentation required by the CRA.",
     4, "17", STATUS_OPEN),
    ("P4.7", "EU declaration of conformity and CE marking",
     "Declaration of conformity in the structure of Annex V and the CE marking.",
     4, "18, 19", STATUS_OPEN),
    ("P4.8", "User information in the structure of Annex II",
     "Update the user information with the security relevant content required by the CRA.",
     4, "21", STATUS_OPEN),
    ("P5.1", "Reporting process for exploited vulnerabilities and severe incidents",
     "Internal process for the reporting duties of 24 hours, 72 hours, 14 days and one month.",
     5, "23, 25, 26", STATUS_OPEN),
    ("P5.2", "Access to the ENISA Single Reporting Platform and the competent CSIRT",
     "Register the reporting access required for the reporting duties.",
     5, "24", STATUS_OPEN),
)

# Englischer Befundtext je Risiko-ID. Zuordnung Befund zu Maßnahme, Stand und Pflicht-Nummer
# kommen aus Abschnitt 7 und 8 des Masters, hier steht nur der Text. Jede Risiko-ID des Masters
# muss hier stehen und umgekehrt, sonst bricht das Werkzeug ab.
FINDINGS = {
    "R1": "No authentication on the control interface, every endpoint that changes state is "
          "reachable without a login.",
    "R2": "Access point password fixed in the source code and published in the documentation.",
    "R3": "Sensor calibration can be changed over the web interface, the firmware then "
          "underestimates the real pressure and the over-pressure shutdown does not trip.",
    "R4": "State changes over HTTP GET, so a foreign page can change control parameters.",
    "R5": "Arbitrary file access on LittleFS, the JSON configuration with calibration and PID "
          "gains is served over HTTP.",
    "R6": "PID gains can be raised without a limit over the web interface.",
    "R7": "Flash wear and denial of service, every settings write goes synchronously to LittleFS.",
    "R8": "Third client connection dropped all clients and restarted the access point.",
    "R9": "Missing synchronisation of the settings accesses between network and control task.",
    "R10": "Radio layer without its own authentication, the control traffic runs as plain HTTP "
           "over the wireless link.",
}

# Befundzellen des Masters ohne Kennung, übersetzt. Die Verneinung benennt im englischen Text die
# Pflichtenklasse, damit die Leserin den Unterschied zum Befundbezug erkennt.
NO_FINDING = {
    "kein Risikobezug (Normpflicht)": "none (standard duty)",
    "kein Risikobezug (Meldepflicht)": "none (reporting duty)",
    "RED-1, RED-2 Normpflicht aus dem RED": "none (RED norm duty)",
}

# Pflichtzellen des Masters, die Prosa statt Nummern tragen, übersetzt.
NO_DUTY = {"kein Annex-I-Punkt (Robustheit)": "no Annex I point (robustness)"}


ROW = re.compile(r"^\| (.+?) \| (.+?) \| (P\d\.\d) (.+?) \| (.+?) \| (.+?) \|$")


def section7(text: str) -> list[dict[str, str]]:
    """Eine Zeile je Maßnahme aus Abschnitt 7 des Masters, in Dateireihenfolge."""
    rows = []
    for line in text.split("\n"):
        found = ROW.match(line.strip())
        if found:
            document = found.group(5).strip().strip("`")
            rows.append({"finding": found.group(1).strip(),
                         "duty": found.group(2).strip(),
                         "id": found.group(3),
                         "title": found.group(4).strip(),
                         "document": document if document.startswith("plan/") else "",
                         "status": found.group(6).strip()})
    return rows


def finding_of(cell: str, problems: list[str]) -> str:
    """Befundspalte aus Abschnitt 7: eine Risiko-ID oder eine der vereinbarten Verneinungen."""
    found = re.match(r"^(R\d+)\b", cell)
    if found:
        return found.group(1)
    if cell in NO_FINDING:
        return NO_FINDING[cell]
    problems.append("Unbekannte Befundspalte in Abschnitt 7 des Masters: %r" % cell)
    return cell


def duty_of(cell: str) -> str:
    """Pflichtzelle aus Abschnitt 7, Prosaformen übersetzt."""
    return NO_DUTY.get(cell, cell)


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


def check_master(rows: list[dict[str, str]], phases: dict[str, str]) -> list[str]:
    """Abgleich mit dem Master, damit keine Kennung und kein Befund still verschwindet."""
    problems: list[str] = []
    by_id = {row["id"]: row for row in rows}
    known = {entry[0] for entry in MEASURES}
    for identifier in sorted(set(by_id) - known):
        problems.append("Maßnahme %s steht im Master, fehlt aber im Werkzeug" % identifier)
    for identifier, _, _, phase, duty, _ in MEASURES:
        row = by_id.get(identifier)
        if row is None:
            problems.append("Maßnahme %s fehlt im Master" % identifier)
            continue
        if str(phase) != phases.get(identifier):
            problems.append("Maßnahme %s steht im Master unter Phase %s statt %d"
                            % (identifier, phases.get(identifier), phase))
        if duty_of(row["duty"]) != duty:
            problems.append("Maßnahme %s: Pflicht-Nummern im Master %r, im Werkzeug %r"
                            % (identifier, row["duty"], duty))
    named = [finding_of(row["finding"], problems) for row in rows]
    referenced = [finding for finding in named if finding.startswith("R")]
    for finding in sorted(set(referenced) - set(FINDINGS), key=lambda name: int(name[1:])):
        problems.append("Befund %s steht im Master, fehlt im Werkzeug" % finding)
    for finding in sorted(set(FINDINGS) - set(referenced), key=lambda name: int(name[1:])):
        problems.append("Befund %s steht im Werkzeug, fehlt im Master" % finding)
    if len(referenced) != len(set(referenced)):
        problems.append("Befund mehrfach einer Maßnahme zugeordnet: %s" % sorted(referenced))
    return problems


def check_documents(rows: list[dict[str, str]]) -> list[str]:
    """Detaildokumente gegen die eigene Liste und das Dateisystem prüfen."""
    problems = []
    documents = {row["id"]: row["document"] for row in rows if row["document"]}
    for entry in MEASURES:
        identifier = entry[0]
        document = documents.get(identifier, "")
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


def annex_content(rows: list[dict[str, str]], active: set[str], resting: set[str]) -> dict:
    """Inhalt der Anlage: Maßnahmenzeilen und Befundzeilen, beide aus dem Master abgeleitet."""
    by_id = {row["id"]: row for row in rows}
    measures = []
    findings = []
    for identifier, name, description, _phase, duty, status in MEASURES:
        row = by_id.get(identifier)
        if row is None:
            continue
        suffix = [QUALIFIERS[identifier]] if identifier in QUALIFIERS else []
        if identifier in active:
            suffix.append(ACTIVE)
        elif identifier in resting:
            suffix.append(RESTING)
        status_text = ", ".join([status] + suffix)
        finding = finding_of(row["finding"], [])
        measures.append([identifier, "**%s** %s" % (name, description), status_text, finding,
                         duty])
        if finding in FINDINGS:
            findings.append([finding, FINDINGS[finding], identifier, status_text])
    findings.sort(key=lambda row: int(row[0][1:]))
    return {"measures": measures, "findings": findings}


def findings_heading(findings: list[list[str]]) -> str:
    """Überschrift der Befundtabelle, als Bereich solange die Nummern lückenlos sind."""
    numbers = [int(row[0][1:]) for row in findings]
    if numbers and numbers == list(range(numbers[0], numbers[-1] + 1)):
        return "Findings R%d to R%d" % (numbers[0], numbers[-1])
    return "Findings " + ", ".join("R%d" % number for number in numbers)


def md_table(columns, rows) -> list[str]:
    """Tabellenzeilen für die Markdown-Quelle."""
    lines = ["| " + " | ".join(name for name, _ in columns) + " |",
             "|" + "|".join(" --- " for _ in columns) + "|"]
    for values in rows:
        lines.append("| " + " | ".join(values) + " |")
    return lines


def markdown(model: dict) -> str:
    """Markdown-Quelle der Anlage.

    Die Felder `Source`, `Market trigger`, `Revision` und `Closing` erscheinen in der Worddatei,
    das Feld `Tool` belegt in der Markdown-Datei, woher der Inhalt kommt.
    """
    head = (
        ("Source",
         "sec/CRA_Compliance_Checklist.md, the single status source in the VENTREX dossier "
         "(section 7 and section 8). Measures P1.1 to P5.2, 21 items, and findings R1 to R10. "
         "Duty numbers refer to the obligation list 1 to 26 in section 6 of the same file."),
        ("Market trigger",
         "the duties arise with the placing of the product on the Union market (Art. 2(1), "
         "Art. 3(21) and (22) CRA, Recital 15). As long as the product is distributed in third "
         "countries only, no duties arise from the CRA or from the RED cyber provisions; "
         "implementation is voluntary then. Measures marked with \"rests until the market "
         "decision\" are not being implemented today, measures marked with \"active in prototype "
         "operation\" are."),
        ("Revision",
         "2026-10-06: findings R1 to R10 added; the risk reference is split into the columns "
         "Finding and CRA duty."),
        ("Closing",
         "Status as documented on 2026-10-06. The master checklist remains the single status "
         "source; this annex is an extract of it."),
        ("Tool",
         "tools/measure_reference.py, generated from section 7 and section 8 of the master and "
         "composed into VentCon2_Measures_Reference_2026-10-02.docx."),
    )
    lines = ["# VentCon2: Measures Referenced in the Aalberts Answers", "",
             "Annex to the CRA product maturity checklist and to the supplier due diligence "
             "questionnaire, reference date 2026-10-02.", "",
             HEAD_START, "| Field | Value |", "| --- | --- |"]
    for name, value in head:
        lines.append("| %s | %s |" % (name, value))
    lines += [HEAD_END, ""]
    lines.append("## Measures P1.1 to P5.2")
    lines.append("")
    lines += md_table(COLUMNS, model["measures"])
    lines.append("")
    lines.append("## " + findings_heading(model["findings"]))
    lines.append("")
    lines += md_table(FINDING_COLUMNS, model["findings"])
    lines.append("")
    return "\n".join(lines)


def parse_markdown(text: str) -> dict:
    """Markdown-Quelle einlesen: Titel, Untertitel, Kopffelder, Abschnitte mit ihren Tabellen."""
    head: dict[str, str] = {}
    body: list[str] = []
    inside = False
    for line in text.split("\n"):
        stripped = line.strip()
        if stripped == HEAD_START:
            inside = True
            continue
        if stripped == HEAD_END:
            inside = False
            continue
        if inside:
            cells = [cell.strip() for cell in stripped.strip("|").split("|")]
            if len(cells) == 2 and cells[0] not in ("Field", "---", ""):
                head[cells[0]] = cells[1]
            continue
        body.append(line)

    source = {"head": head, "title": "", "subtitle": "", "sections": []}
    for line in body:
        stripped = line.strip()
        if stripped.startswith("## "):
            source["sections"].append({"heading": stripped[3:].strip(), "columns": [], "rows": []})
        elif stripped.startswith("# "):
            source["title"] = stripped[2:].strip()
        elif stripped.startswith("|"):
            if not source["sections"]:
                continue
            section = source["sections"][-1]
            cells = [cell.strip() for cell in stripped.strip("|").split("|")]
            if all(cell and set(cell) <= set("-: ") for cell in cells):
                continue
            if not section["columns"]:
                section["columns"] = cells
            elif len(cells) != len(section["columns"]):
                raise ValueError("Markdown-Quelle: Zeile mit %d Zellen, Kopfzeile hat %d: %r"
                                 % (len(cells), len(section["columns"]), cells[0]))
            else:
                section["rows"].append(cells)
        elif stripped and not source["subtitle"]:
            source["subtitle"] = stripped
    return source


def add_runs(paragraph, text: str, size: float) -> None:
    """Text mit `**fett**` in Läufe umsetzen, die Marken selbst entfallen."""
    for index, part in enumerate(re.split(r"\*\*(.+?)\*\*", text)):
        if part:
            style_run(paragraph.add_run(part), size, bold=index % 2 == 1)


def add_table(document, columns, rows, widths) -> None:
    """Tabelle im Anlagestil: Kopfzeile im Markenblau, Zeilentext und Rahmen wie bisher."""
    table = document.add_table(rows=1, cols=len(columns))
    table.style = document.styles["Table Grid"]
    header = table.rows[0]
    header._tr.get_or_add_trPr().append(header._tr.makeelement(qn("w:tblHeader"), {}))
    for cell, (name, _) in zip(header.cells, columns):
        cell.paragraphs[0].clear()
        shade(cell, "002F87")
        paragraph = cell.paragraphs[0]
        paragraph.paragraph_format.space_after = Pt(1)
        style_run(paragraph.add_run(name), 8, bold=True, colour=RGBColor(0xFF, 0xFF, 0xFF))
    for values in rows:
        cells = table.add_row().cells
        for cell, value in zip(cells, values):
            cell.paragraphs[0].paragraph_format.space_after = Pt(0)
            add_runs(cell.paragraphs[0], value, 8)
    set_widths(table, widths)
    keep_rows_together(table)


def compose(source: dict) -> None:
    """Worddatei aus der Markdown-Quelle setzen."""
    document = Document(str(TEMPLATE))
    section = document.sections[0]
    usable = (section.page_width.inches - section.left_margin.inches
              - section.right_margin.inches)
    body = document.paragraphs[0]
    body.clear()
    body.paragraph_format.space_after = Pt(2)
    style_run(body.add_run(source["title"]), 19, bold=True, font=HEADING_FONT)

    subtitle = document.add_paragraph()
    subtitle.paragraph_format.space_after = Pt(2)
    style_run(subtitle.add_run(source["subtitle"]), 10)

    meta = document.add_paragraph()
    meta.paragraph_format.space_after = Pt(8)
    style_run(meta.add_run("Source: " + source["head"]["Source"]), 9, colour=GREY)

    add_box(document, "Market trigger: " + source["head"]["Market trigger"], usable)

    spacer = document.add_paragraph()
    spacer.paragraph_format.space_after = Pt(2)

    for index, section_content in enumerate(source["sections"]):
        columns = LAYOUTS[tuple(section_content["columns"])]
        scale = usable / sum(width for _, width in columns)
        widths = [round(width * scale, 4) for _, width in columns]
        heading = document.add_paragraph()
        heading.paragraph_format.space_before = Pt(10 if index else 4)
        heading.paragraph_format.space_after = Pt(2)
        style_run(heading.add_run(section_content["heading"]), 10, bold=True)
        add_table(document, columns, section_content["rows"], widths)

    revision = document.add_paragraph()
    revision.paragraph_format.space_before = Pt(8)
    style_run(revision.add_run("Revision " + source["head"]["Revision"]), 8, colour=GREY)

    closing = document.add_paragraph()
    style_run(closing.add_run(source["head"]["Closing"]), 8, colour=GREY)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    document.save(str(OUTPUT))


def main() -> int:
    text = MASTER.read_text(encoding="utf-8")
    rows = section7(text)
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
    model = annex_content(rows, active, resting)
    SOURCE.parent.mkdir(parents=True, exist_ok=True)
    SOURCE.write_text(markdown(model), encoding="utf-8", newline="\n")
    print("Geschrieben: %s" % SOURCE.relative_to(ROOT).as_posix())
    compose(parse_markdown(SOURCE.read_text(encoding="utf-8")))
    print("Geschrieben: %s" % OUTPUT.relative_to(ROOT).as_posix())
    print("  %d Maßnahmenzeilen, %d Befundzeilen" % (len(model["measures"]),
                                                     len(model["findings"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
