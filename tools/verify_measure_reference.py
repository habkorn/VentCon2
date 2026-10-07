# -*- coding: utf-8 -*-
"""Prüft die Maßnahmen-Anlage `VentCon2_Measures_Reference_2026-10-02.docx` unabhängig.

Der Prüfer liest die erzeugte Worddatei, ihre Markdown-Quelle und daneben den Master
`sec/CRA_Compliance_Checklist.md`. Geprüft werden Vollständigkeit und Reihenfolge der Kennungen,
Übereinstimmung von aktivem und ruhendem Status mit dem Master, die Phase im Master gegen die
Kennung, der Befundbezug und die Pflicht-Nummern gegen Abschnitt 7, die Befundliste mit einer
Zeile je Risiko-ID, die Auflösung jeder in der Anlage genannten Risiko-ID, die Übereinstimmung von
Markdown-Quelle und Worddatei, die Felder der Tabellen, die Formatierung, die Spaltenbreiten, der
Hinweiskasten zum Marktauslöser und die Unversehrtheit der Vorlage. Es wird nichts aus dem Erzeuger
importiert, die Erwartungen stehen hier eigenständig.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/verify_measure_reference.py
"""

from __future__ import annotations

import hashlib
import re
import sys
import zipfile
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from docx.shared import RGBColor, Twips

ROOT = Path(__file__).resolve().parent.parent
BASE_DIR = ROOT / "sec" / "AalbertsChecklists"
MASTER = ROOT / "sec" / "CRA_Compliance_Checklist.md"
TEMPLATE = ROOT / "sec" / "VENTREX Wordvorlage_ohne Fußzeile.docx"
SOURCE = BASE_DIR / "filled" / "src" / "VentCon2_Measures_Reference_2026-10-02.md"
OUTPUT = BASE_DIR / "filled" / "VentCon2_Measures_Reference_2026-10-02.docx"

W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"

BRAND = RGBColor(0x00, 0x2F, 0x87)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
TEMPLATE_SHA256 = "0bf201d6e2b3cdde336909c4fcb7442ce1717d204e75b545e43f615fd659faef"

EXPECTED_COLUMNS = ("ID", "Measure and scope", "Status", "Finding", "CRA duty")
EXPECTED_FINDING_COLUMNS = ("ID", "Finding", "Measure", "Status")
EXPECTED_MEASURES = ["P1.%d" % number for number in range(1, 6)]
EXPECTED_MEASURES += ["P2.%d" % number for number in range(1, 4)]
EXPECTED_MEASURES += ["P3.%d" % number for number in range(1, 4)]
EXPECTED_MEASURES += ["P4.%d" % number for number in range(1, 9)]
EXPECTED_MEASURES += ["P5.%d" % number for number in range(1, 3)]
EXPECTED_FINDINGS = ["R%d" % number for number in range(1, 11)]
MEASURE_SECTION = "Measures P1.1 to P5.2"
FINDING_SECTION = "Findings R1 to R10"
MIN_COLUMN = 0.40
ROW_FONT_SIZE = 8.0

problems: list[str] = []
checks = 0


def check(condition: bool, message: str) -> None:
    global checks
    checks += 1
    if not condition:
        problems.append(message)


def identifiers(text: str) -> set[str]:
    """Kennungen P<Phase>.<Nummer> im Text, Bereiche wie 'P4.1 bis P4.8' aufgelöst."""
    found: set[str] = set()
    for start_phase, start_number, end_phase, end_number in re.findall(
            r"P(\d)\.(\d) bis P(\d)\.(\d)", text):
        for number in range(int(start_number), int(end_number) + 1):
            found.add("P%s.%d" % (start_phase, number))
    for phase, number in re.findall(r"P(\d)\.(\d)", text):
        found.add("P%s.%s" % (phase, number))
    return found


def master_data(text: str) -> dict[str, dict[str, str]]:
    """Kennung mit Phase, Detaildokument, Titelzeile, Befundspalte und Pflicht-Nummern."""
    data: dict[str, dict[str, str]] = {}
    pattern = re.compile(r"^\| (.+?) \| (.+?) \| (P\d\.\d) (.+?) \| (.+?) \| (.+?) \|$")
    for line in text.split("\n"):
        found = pattern.match(line.strip())
        if found:
            document = found.group(5).strip().strip("`")
            data[found.group(3)] = {
                "finding": found.group(1).strip(),
                "duty": found.group(2).strip(),
                "title": found.group(4).strip(),
                "document": document if document.startswith("plan/") else "",
                "status": found.group(6).strip()}
    phase = ""
    for line in text.split("\n"):
        heading = re.match(r"^### Phase (\d)", line.strip())
        if heading:
            phase = heading.group(1)
        bullet = re.match(r"^- \[[ x]\] (P\d\.\d) (.+)$", line.strip())
        if bullet:
            data.setdefault(bullet.group(1), {})["phase"] = phase
    return data


def master_finding(cell: str) -> str:
    """Erwartung an die Spalte `Finding`, aus der Befundspalte von Abschnitt 7 abgeleitet.

    Die Verneinungen des Masters sind deutsch, die Anlage ist englisch; die Zuordnung steht hier
    eigenständig, damit sie nicht unbemerkt mit dem Erzeuger wandert.
    """
    found = re.match(r"^(R\d+)\b", cell)
    if found:
        return found.group(1)
    translations = {
        "kein Risikobezug (Normpflicht)": "none (standard duty)",
        "kein Risikobezug (Meldepflicht)": "none (reporting duty)",
        "RED-1, RED-2 Normpflicht aus dem RED": "none (RED norm duty)",
    }
    return translations.get(cell, cell)


def master_duty(cell: str) -> str:
    """Erwartung an die Spalte `CRA duty`, aus der Pflichtspalte von Abschnitt 7 abgeleitet."""
    return "no Annex I point (robustness)" if cell == "kein Annex-I-Punkt (Robustheit)" else cell


def markdown_head(text: str) -> dict[str, str]:
    """Felder des Kopfblocks der Markdown-Quelle."""
    head: dict[str, str] = {}
    inside = False
    for line in text.split("\n"):
        stripped = line.strip()
        if stripped == "<!-- annex:head-start -->":
            inside = True
            continue
        if stripped == "<!-- annex:head-end -->":
            break
        if inside and stripped.startswith("|"):
            cells = [cell.strip() for cell in stripped.strip("|").split("|")]
            if len(cells) == 2 and cells[0] not in ("Field", "---", ""):
                head[cells[0]] = cells[1]
    return head


def markdown_tables(text: str) -> dict[str, list[list[str]]]:
    """Tabellen der Markdown-Quelle, nach der Überschrift des Abschnitts benannt."""
    tables: dict[str, list[list[str]]] = {}
    heading = ""
    in_head = False
    for line in text.split("\n"):
        stripped = line.strip()
        if stripped == "<!-- annex:head-start -->":
            in_head = True
            continue
        if stripped == "<!-- annex:head-end -->":
            in_head = False
            continue
        if in_head:
            continue
        if stripped.startswith("## "):
            heading = stripped[3:].strip()
            tables.setdefault(heading, [])
            continue
        if stripped.startswith("|") and heading:
            cells = [cell.strip() for cell in stripped.strip("|").split("|")]
            if all(cell and set(cell) <= set("-: ") for cell in cells):
                continue
            tables[heading].append(cells)
    for heading, rows in tables.items():
        tables[heading] = rows[1:] if rows else rows
    return tables


def sets_from_master(text: str) -> tuple[set[str], set[str]]:
    """Aktive und ruhende Maßnahmen aus dem Vorspann von Abschnitt 8.

    Der Vorspann ist über mehrere Zeilen umbrochen, deshalb wird der Text vor der Auswertung
    zusammengezogen.
    """
    flat = " ".join(text.split())
    found_active = re.search(r"Aktiv bleiben (.*?), weil", flat)
    found_resting = re.search(r"Zurückgestellt sind bis zur Marktentscheidung (.*?)\. ", flat)
    active = identifiers(found_active.group(1)) if found_active else set()
    resting = identifiers(found_resting.group(1)) if found_resting else set()
    return active, resting


def grid_widths(table) -> list[float]:
    return [Twips(int(column.get(qn("w:w")))).inches
            for column in table._tbl.tblGrid.findall(qn("w:gridCol"))]


def drawing_counts(path: Path) -> dict[str, int]:
    """Zeichnungen je Kopf- und Fußteil, zum Vergleich mit der Vorlage."""
    counts: dict[str, int] = {}
    with zipfile.ZipFile(path) as archive:
        for name in archive.namelist():
            if "header" in name or "footer" in name:
                xml = archive.read(name).decode("utf-8")
                counts[name] = xml.count("<w:drawing") + xml.count("<v:shape")
    return counts


def main() -> int:
    check(OUTPUT.is_file(), "Anlage fehlt: %s" % OUTPUT.name)
    if not OUTPUT.is_file():
        return report()
    check(hashlib.sha256(TEMPLATE.read_bytes()).hexdigest() == TEMPLATE_SHA256,
          "Die VENTREX-Vorlage wurde verändert")

    master = MASTER.read_text(encoding="utf-8")
    data = master_data(master)
    active, resting = sets_from_master(master)
    check(len(data) >= 21, "Master liefert nur %d Maßnahmen" % len(data))
    check(len(active) == 4, "Master nennt %d aktive Maßnahmen, erwartet 4" % len(active))
    check(len(resting) == 11, "Master nennt %d ruhende Maßnahmen, erwartet 11" % len(resting))

    document = Document(str(OUTPUT))
    template = Document(str(TEMPLATE))
    print("Prüfung der Maßnahmen-Anlage")
    print("  %s" % OUTPUT.relative_to(ROOT).as_posix())

    for name in ("page_width", "page_height", "left_margin", "right_margin", "top_margin",
                 "bottom_margin"):
        check(getattr(document.sections[0], name) == getattr(template.sections[0], name),
              "Seiteneinrichtung weicht ab: %s" % name)
    usable = (document.sections[0].page_width.inches - document.sections[0].left_margin.inches
              - document.sections[0].right_margin.inches)

    check(drawing_counts(OUTPUT) == drawing_counts(TEMPLATE),
          "Kopf- oder Fußgrafik der Vorlage fehlt: %s" % drawing_counts(OUTPUT))

    title = document.paragraphs[0].runs[0]
    check("VentCon2" in title.text and "Measures" in title.text,
          "Titel fehlt oder lautet anders: %s" % title.text[:60])
    check(title.font.name == "Clash Display", "Titel nicht in Clash Display")
    check(bool(title.font.bold), "Titel nicht fett")
    check(title.font.color.rgb == BRAND, "Titel nicht im Markenblau")

    body_text = "\n".join(paragraph.text for paragraph in document.paragraphs)
    check("2026-10-02" in body_text, "Referenzdatum fehlt")
    check("sec/CRA_Compliance_Checklist.md" in body_text, "Quellenangabe fehlt")
    check("21 items" in body_text, "Angabe zur Anzahl der Maßnahmen fehlt")

    boxes = [table for table in document.tables if len(table.rows) == 1 and len(table.columns) == 1]
    check(len(boxes) == 1, "Hinweiskasten fehlt oder ist mehrfach vorhanden")
    box = ""
    if boxes:
        box = boxes[0].rows[0].cells[0].text
        check(len(grid_widths(boxes[0])) == 1 and abs(grid_widths(boxes[0])[0] - usable) < 0.01,
              "Hinweiskasten entspricht nicht der Satzbreite")
    for phrase in ("placing of the product on the Union market", "third countries only",
                   "rests until the market decision", "active in prototype operation"):
        check(phrase in box, "Hinweiskasten ohne den Satz: %s" % phrase)

    tables = [table for table in document.tables if len(table.rows) > 1]
    check(len(tables) == 2, "Es gibt %d Tabellen, erwartet 2 (Maßnahmen und Befunde)" % len(tables))
    if len(tables) != 2:
        return report()
    measures_table, findings_table = tables

    check(len(measures_table.columns) == len(EXPECTED_COLUMNS),
          "Maßnahmentabelle hat %d Spalten, erwartet %d"
          % (len(measures_table.columns), len(EXPECTED_COLUMNS)))
    measure_header = [cell.text.strip() for cell in measures_table.rows[0].cells]
    check(tuple(measure_header) == EXPECTED_COLUMNS, "Kopfzeile lautet: %s" % measure_header)
    check(len(measures_table.rows) == len(EXPECTED_MEASURES) + 1,
          "Maßnahmentabelle hat %d Zeilen, erwartet %d"
          % (len(measures_table.rows), len(EXPECTED_MEASURES) + 1))

    check(len(findings_table.columns) == len(EXPECTED_FINDING_COLUMNS),
          "Befundtabelle hat %d Spalten, erwartet %d"
          % (len(findings_table.columns), len(EXPECTED_FINDING_COLUMNS)))
    finding_header = [cell.text.strip() for cell in findings_table.rows[0].cells]
    check(tuple(finding_header) == EXPECTED_FINDING_COLUMNS,
          "Kopfzeile der Befundtabelle lautet: %s" % finding_header)
    check(len(findings_table.rows) == len(EXPECTED_FINDINGS) + 1,
          "Befundtabelle hat %d Zeilen, erwartet %d"
          % (len(findings_table.rows), len(EXPECTED_FINDINGS) + 1))

    for table in tables:
        for cell in table.rows[0].cells:
            run = cell.paragraphs[0].runs[0]
            check(bool(run.font.bold) and run.font.color.rgb == WHITE,
                  "Kopfzelle nicht fett und weiß: %s" % cell.text[:20])
            shade = cell._tc.tcPr.find(qn("w:shd"))
            check(shade is not None and shade.get(qn("w:fill")) == "002F87",
                  "Kopfzelle ohne Markenblau: %s" % cell.text[:20])
        widths = grid_widths(table)
        check(abs(sum(widths) - usable) < 0.01,
              "Tabellenbreite %.2f in statt %.2f in" % (sum(widths), usable))
        split_tag = qn("w:cantSplit")
        without_protection = [row for row in table.rows
                              if row._tr.trPr is None or row._tr.trPr.find(split_tag) is None]
        check(not without_protection, "%d von %d Tabellenzeilen ohne Umbruchschutz"
              % (len(without_protection), len(table.rows)))
        header_properties = table.rows[0]._tr.trPr
        order = [child.tag.split("}")[1] for child in header_properties]
        check("cantSplit" in order and "tblHeader" in order
              and order.index("cantSplit") < order.index("tblHeader"),
              "Kopfzeile: Reihenfolge von cantSplit und tblHeader stimmt nicht: %s" % order)
        for index, width in enumerate(widths):
            check(width >= MIN_COLUMN, "Spalte %d nur %.2f in breit" % (index + 1, width))

    widths = grid_widths(measures_table)
    check(widths[1] >= 2.00, "Spalte Measure and scope nur %.2f in breit" % widths[1])
    check(widths[3] >= 0.70, "Spalte Finding nur %.2f in breit" % widths[3])
    check(widths[4] >= 1.00, "Spalte CRA duty nur %.2f in breit" % widths[4])

    for identifier, entry in data.items():
        check(entry.get("phase") == identifier[1],
              "Master führt %s unter Phase %s" % (identifier, entry.get("phase")))

    seen_identifiers: list[str] = []
    resting_rows: set[str] = set()
    active_rows: set[str] = set()
    status_by_measure: dict[str, str] = {}
    referenced: set[str] = set()
    for row in measures_table.rows[1:]:
        cells = [cell.text.strip() for cell in row.cells]
        identifier = cells[0]
        seen_identifiers.append(identifier)
        check(len(cells) == len(EXPECTED_COLUMNS),
              "Zeile %s hat %d Zellen" % (identifier, len(cells)))
        check(identifier in data, "Kennung %s steht nicht im Master" % identifier)
        check(bool(cells[1]), "Zeile %s ohne Titel und Beschreibung" % identifier)
        check(cells[2].startswith(("Open", "In progress", "Done")),
              "Zeile %s hat den Stand '%s'" % (identifier, cells[2][:24]))
        status_by_measure[identifier] = cells[2]
        expected_finding = master_finding(data.get(identifier, {}).get("finding", ""))
        check(cells[3] == expected_finding,
              "Zeile %s: Befund %r statt %r" % (identifier, cells[3], expected_finding))
        expected_duty = master_duty(data.get(identifier, {}).get("duty", ""))
        check(cells[4] == expected_duty,
              "Zeile %s: Pflicht-Nummern %r statt %r" % (identifier, cells[4], expected_duty))
        if re.fullmatch(r"R\d+", cells[3]):
            referenced.add(cells[3])
        if "rests until the market decision" in cells[2]:
            resting_rows.add(identifier)
        if "active in prototype operation" in cells[2]:
            active_rows.add(identifier)
        for cell in row.cells:
            for paragraph in cell.paragraphs:
                for run in paragraph.runs:
                    if run.text.strip():
                        check(run.font.size is not None
                              and abs(run.font.size.pt - ROW_FONT_SIZE) < 0.01,
                              "Zeile %s hat Schriftgröße %s" % (identifier, run.font.size))
        check("**" not in "".join(cells) and "[to be completed]" not in "".join(cells),
              "Zeile %s trägt einen Platzhalter" % identifier)

    check(seen_identifiers == EXPECTED_MEASURES,
          "Kennungen unvollständig oder falsch sortiert: %s" % seen_identifiers)
    check(len(set(seen_identifiers)) == len(seen_identifiers), "Kennung doppelt in der Tabelle")
    check(resting_rows == resting, "Ruhende Maßnahmen weichen ab: %s statt %s"
          % (sorted(resting_rows), sorted(resting)))
    check(active_rows == active, "Aktive Maßnahmen weichen ab: %s statt %s"
          % (sorted(active_rows), sorted(active)))

    seen_findings: list[str] = []
    for row in findings_table.rows[1:]:
        cells = [cell.text.strip() for cell in row.cells]
        finding = cells[0]
        seen_findings.append(finding)
        check(len(cells) == len(EXPECTED_FINDING_COLUMNS),
              "Befundzeile %s hat %d Zellen" % (finding, len(cells)))
        check(re.fullmatch(r"R\d+", finding) is not None,
              "Befundzeile ohne Risiko-ID: %r" % finding)
        check(finding in EXPECTED_FINDINGS, "Unbekannte Risiko-ID %s" % finding)
        check(len(cells[1]) > 20, "Befund %s hat keinen Befundtext" % finding)
        check(cells[2] in data, "Befund %s nennt die unbekannte Maßnahme %s" % (finding, cells[2]))
        check(data.get(cells[2], {}).get("finding", "").startswith(finding),
              "Befund %s und Maßnahme %s gehören laut Master nicht zusammen"
              % (finding, cells[2]))
        check(cells[3] == status_by_measure.get(cells[2]),
              "Befund %s: Stand %r statt %r aus der Maßnahmenzeile %s"
              % (finding, cells[3], status_by_measure.get(cells[2]), cells[2]))
        for cell in row.cells:
            for paragraph in cell.paragraphs:
                for run in paragraph.runs:
                    if run.text.strip():
                        check(run.font.size is not None
                              and abs(run.font.size.pt - ROW_FONT_SIZE) < 0.01,
                              "Befundzeile %s hat Schriftgröße %s" % (finding, run.font.size))
        check("**" not in "".join(cells), "Befundzeile %s trägt einen Platzhalter" % finding)

    check(seen_findings == EXPECTED_FINDINGS,
          "Befund-IDs unvollständig oder falsch sortiert: %s" % seen_findings)
    check(referenced == set(EXPECTED_FINDINGS),
          "Befunde der Maßnahmentabelle ohne Erklärung in der Befundtabelle: %s"
          % sorted(referenced - set(EXPECTED_FINDINGS)))

    check(SOURCE.is_file(), "Markdown-Quelle fehlt: %s" % SOURCE.name)
    if SOURCE.is_file():
        source_text = SOURCE.read_text(encoding="utf-8")
        head = markdown_head(source_text)
        for field in ("Source", "Market trigger", "Revision", "Closing", "Tool"):
            check(field in head, "Markdown-Quelle ohne Feld %s" % field)
        check(head.get("Source", "") in body_text,
              "Quellenangabe der Markdown-Quelle fehlt in der Worddatei")
        check("Market trigger: " + head.get("Market trigger", "") in box,
              "Marktauslöser der Markdown-Quelle fehlt im Hinweiskasten")
        check("Revision " + head.get("Revision", "") in body_text,
              "Revisionszeile der Markdown-Quelle fehlt in der Worddatei")
        check(head.get("Closing", "") in body_text,
              "Schlusssatz der Markdown-Quelle fehlt in der Worddatei")

        source_tables = markdown_tables(source_text)
        check(MEASURE_SECTION in source_tables and FINDING_SECTION in source_tables,
              "Markdown-Quelle ohne die Abschnitte %r und %r"
              % (MEASURE_SECTION, FINDING_SECTION))
        for heading in (MEASURE_SECTION, FINDING_SECTION):
            check(heading in body_text, "Abschnitt %r fehlt in der Worddatei" % heading)
        markdown_measures = [[cell.replace("**", "") for cell in row]
                             for row in source_tables.get(MEASURE_SECTION, [])]
        document_measures = [[cell.text.strip() for cell in row.cells]
                             for row in measures_table.rows[1:]]
        check(markdown_measures == document_measures,
              "Markdown-Quelle und Worddatei weichen in den Maßnahmen ab")
        markdown_findings = source_tables.get(FINDING_SECTION, [])
        document_findings = [[cell.text.strip() for cell in row.cells]
                             for row in findings_table.rows[1:]]
        check(markdown_findings == document_findings,
              "Markdown-Quelle und Worddatei weichen in den Befunden ab")

    print("  %d Zeilen, %d Kennungen, %d aktiv, %d ruhend"
          % (len(measures_table.rows) - 1, len(seen_identifiers), len(active_rows),
             len(resting_rows)))
    print("  %d Befunde, alle in der Maßnahmentabelle referenziert: %s"
          % (len(seen_findings), ", ".join(seen_findings)))
    print("  Spaltenbreiten %s, Summe %.2f in (Satzbreite %.2f in)"
          % ([round(width, 2) for width in grid_widths(measures_table)],
             sum(grid_widths(measures_table)), usable))
    return report()


def report() -> int:
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
