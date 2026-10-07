# -*- coding: utf-8 -*-
"""Füllt die Aalberts-Vorlagen aus: Antworten in die Markdown-Extrakte und in Wordkopien.

Rolle nach Rücksprache vom 2026-10-01: VENTREX ist Teil des Aalberts-Konzerns. Aalberts möchte
wissen, wie der aktuelle Stand von VentCon2 gegenüber der CRA einzuordnen ist. Die Antworten
kommen aus dem Dossier (`sec/CRA_Compliance_Checklist.md`, `sec/analyse/`, `sec/plan/`) und aus
dem Projektstand (`platformio.ini`, `src/`). Antworten stehen fett (`**`) und ersetzen keinen
Vorlagentext, sie werden in leere Zellen gesetzt oder als fette Zeile eingefügt.

Die Antworten in diesem Werkzeug sind die einzige Quelle. Wird ein Extrakt mit
`tools/docx_extract.py` neu erzeugt, gehen die Antworten verloren und müssen mit diesem Werkzeug
erneut eingetragen werden.

Ablage (Entscheidung E57): Vorlagen in `templates/`, Extrakte in `extracts/`, befüllte Wordkopien
in `filled/`. Die Vorlagen selbst bleiben unverändert.

Im Markdown tragen die Antworten die Markdown-Marker `**`, im Worddokument entfallen sie: dort
übernimmt die fette Schrift im Markenblau die Betonung (Entscheidungen E56 und E61). In der
Checkliste steht das Urteil Y, N oder N/A in einer eigenen Zeile fett im Markenblau, die
Begründung folgt darunter im selben Blau mit magerem Schnitt (Entscheidung E75).

Das Addendum ist mit Entscheidung E62 vollständig aus der Antwortstrecke genommen: Es hat keine
Sachfragen, nur Vertragsangaben, die VENTREX nicht allein festlegen kann. Es bleibt als unberührte
Vorlage und als reiner Extrakt im Dossier.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/aalberts_fill.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from docx import Document
from docx.oxml.ns import qn
from docx.shared import Inches, RGBColor, Twips

# Aufteilung der Tabellenbreite nach Textanteil der Antwortspalte (Entscheidung E64).
MIN_ANSWER_SHARE = 0.40
MAX_ANSWER_SHARE = 0.55
MIN_OTHER_WIDTH = 0.90

# Urteil am Anfang einer Antwort der Checkliste, zum Beispiel `**N/A** kein Updateweg`.
VERDICT = re.compile(r"^\*\*(Y|N|N/A)\*\*\s*")

# Werte, die in die Leerzeilen der Vorlage eingetragen werden (Entscheidung E76).
BLANKS = {
    "checklist": (("Product:", ("VentCon2 valve control unit, firmware 2.7.4 (prototype)",
                                 "VENTREX product owner, VentCon2 team",
                                 "2026-10-02")),),
    "questionnaire": (("Completed by", ("Product owner, VENTREX (Aalberts group company)",)),
                      ("Date:", ("2026-10-02",))),
}

ROOT = Path(__file__).resolve().parent.parent
BASE_DIR = ROOT / "sec" / "AalbertsChecklists"
TEMPLATE_DIR = BASE_DIR / "templates"
EXTRACT_DIR = BASE_DIR / "extracts"
FILLED_DIR = BASE_DIR / "filled"
SOURCE_DIR = FILLED_DIR / "src"

# Markenblau der VENTREX- und Aalberts-Vorlagen, Antworten erscheinen fett in dieser Farbe.
ANSWER_COLOR = RGBColor(0x00, 0x2F, 0x87)

CHECKLIST_MD = EXTRACT_DIR / "CRA_Product_Maturity_Checklist.md"
CHECKLIST_DOCX = TEMPLATE_DIR / "CRA product maturity checklist - Sep 2026.docx"
QUESTIONNAIRE_MD = EXTRACT_DIR / "CRA_Supplier_Due_Diligence_Questionnaire.md"
QUESTIONNAIRE_DOCX = TEMPLATE_DIR / "CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx"

ANSWERS_ROW = ("| Answers | Filled in from the VENTREX dossier on 2026-10-02, marked in bold. "
               "Template text is unchanged. This file holds the filled entries of `%s`; the Word "
               "copy is composed from this file by `tools/aalberts_fill.py`. |")

# Marke für Hinweiszeilen in der Markdown-Quelle: Ankerzeile im Worddokument und Hinweistext.
NOTE_MARKER = "<!-- note:"

# Urteil allein in einem Zellteil der Quelle, etwa `**N**` vor dem `<br>`.
VERDICT_CELL = re.compile(r"^\*\*(Y|N|N/A)\*\*$")

# Antwort zu jeder Sicherheitsvorgabe in Teil I der Checkliste, Schlüssel ist der Vorgabentext.
CHECKLIST_PART_I = {
    "Risk-based design: The product’s security measures match its identified cybersecurity risks (see Appendix 1 below).":
        "**N** no documented product risk assessment yet; the risks R1 to R10 from the security "
        "analysis are the interim basis, measure P4.1 open",
    "No known exploitable vulnerabilities: None are known when the product is released or placed on the market.":
        "**N** the security analysis lists open, exploitable findings, above all R1 (no authentication) "
        "and R4 (state change per GET), measures P1.1 and P1.2 in progress",
    "Secure by default: Default settings are secure, and the product can be reset to its original state. Any different setup for a tailor-made business product is expressly agreed with the business customer.":
        "**N** the access point password is fixed in the source (R2), measure P2.2 in progress; factory "
        "reset is available",
    "Security updates: Vulnerabilities can be fixed through security updates.":
        "**N** no update path in the product yet, measure P4.3 open",
    "Automatic updates where appropriate: Updates are installed automatically within an appropriate period and enabled by default, with clear notification and easy options to opt out or temporarily postpone them.":
        "**N/A** no update mechanism exists; the design decision belongs to measure P4.3",
    "Access protection: Authentication, identity and access controls prevent unauthorised access.":
        "**N** mutating endpoints have no authentication (R1), measure P1.1 in progress",
    "Access alerts: Possible unauthorised access is detected and reported.":
        "**N** not implemented and no measure defined yet; to be added to the backlog",
    "Confidentiality: Relevant stored and transmitted data is protected, including through current encryption methods where appropriate.":
        "**N** settings are stored as plain JSON in LittleFS and readable (R5), measure P2.1 partly "
        "designed; radio encryption open (P3.3)",
    "Integrity: Data, commands, software and settings are protected against unauthorised changes, and corruption is detected and reported.":
        "**N** state can be changed by GET without CSRF protection (R4), measure P1.2 in progress",
    "Data minimisation: The product processes only the data needed for its intended purpose.":
        "**Y** pressure, setpoint, PID parameters and valve duty only; no personal data, no cloud, "
        "no remote maintenance",
    "Availability: Essential and basic functions remain available, or can recover, after an incident, including denial-of-service attacks.":
        "**Y** limited, two clients maximum and client DoS handled in firmware 2.7.3 (measures P3.1 "
        "and P3.2 done); rate limiting and save debouncing still open (P2.3, risk R7)",
    "No harmful network impact: The product minimises disruption to services provided by connected devices or networks.":
        "**Y** local access point only, no outbound traffic, no cloud service",
    "Limited attack surface: External interfaces, access points and unnecessary functions are minimised.":
        "**N** file access to LittleFS is possible (R5, P2.1 partly designed) and PID gains are "
        "unlimited (R6, P1.4 open); interfaces are web UI, WLAN, serial console and USB",
    "Incident impact limited: Security controls reduce the possible impact of successful attacks.":
        "**N** no hard upper limits for PID gains (R6, P1.4 open) and the calibration is not yet "
        "bound to the sensor range (R3, P1.3 active); the pressure safety shutdown limits the "
        "physical impact",
    "Security logging and monitoring: Relevant internal activity, including access to or changes to data, services and functions, is recorded and monitored. Users can opt out.":
        "**N** only local serial output for development, no security event logging; no measure defined yet",
    "Secure data removal and transfer: Users can easily and permanently delete all data and settings. Any transfer to another product or system is secure.":
        "**N** reset to defaults exists, permanent deletion of all data and settings does not; no "
        "measure defined yet",
    "Languages are chosen for their secure properties: e.g. memory safety, alternatively bounds checking tools are deployed":
        "**N** C++ on Arduino and ESP-IDF without memory safety and without bounds checking in the "
        "build; no measure defined yet",
    "Program analysis: static and dynamic analysis, type checking, embedded OS allows for reference monitoring, taint checking and similar techniques":
        "**N** no automated static or dynamic analysis in the build; one manual security analysis "
        "exists (2026-08-27); no measure defined yet",
    "Formal verification":
        "**N** not planned, disproportionate for this product class and prototype stage",
    "Compilation certification or verifiable builds: the code as shipped can be verified against the source code":
        "**N** the version string is injected at build time, but there is no verifiable build "
        "attestation; no measure defined yet",
}

# Antwort zu jedem Punkt in Teil II der Checkliste.
CHECKLIST_PART_II = {
    "Vulnerability and component records: Vulnerabilities and product components are identified and documented.":
        "**N** vulnerabilities are documented as risks R1 to R10 in the security analysis, a component "
        "inventory is missing; measure P4.2 open",
    "Software Bill of Materials: An SBOM is maintained in a commonly used, machine-readable format and includes at least top-level dependencies.":
        "**N** no SBOM yet, measure P4.2 open; the dependency list in platformio.ini and the vendored "
        "Chart.js are the current raw input",
    "Prompt remediation: Vulnerabilities are fixed without delay, based on risk.":
        "**N** fixes are made case by case (firmware 2.7.3 closed R7 to R9), no defined remediation "
        "process; measure P4.5 open",
    "Separate security updates: Where technically possible, security updates are provided separately from feature updates.":
        "**N** no update delivery exists yet, measure P4.3 open",
    "Regular security testing: Effective security tests and reviews are performed regularly.":
        "**N** no regular security testing; one analysis exists (2026-08-27), update tests are part of "
        "measure P4.3",
    "Information about fixed vulnerabilities: Once an update is available, information is publicly shared on the vulnerability, affected product, impact, severity and user remediation steps. Publication may be delayed where justified by security risk until users can apply the patch.":
        "**N** no public information about fixed vulnerabilities; no release notes are published; "
        "measure P4.5 open",
    "Coordinated disclosure policy: A coordinated vulnerability disclosure policy is in place and followed.":
        "**N** no policy yet, measure P4.5 open",
    "Reporting channel: A contact address or similar channel is available for reporting vulnerabilities in the product and its third-party components.":
        "**N** no reporting channel yet; the single point of contact required by Art. 13(17) CRA is "
        "planned with measures P4.5 and P4.8",
    "Secure update delivery: Updates are distributed securely and allow vulnerabilities to be fixed or reduced in a timely manner, automatically where applicable.":
        "**N** no update delivery; signed updates are planned in measure P4.3",
    "Fast, free and explained: Security updates are provided without delay, free of charge, and with clear user guidance, unless different terms are agreed for a tailor-made business product.":
        "**N/A** no updates are delivered today; the duty applies from placing the product on the EU market",
}

CHECKLIST_NOTES = [
    ("Part I:",
     "**The measures referenced in this document, P1.1 to P5.2, and the findings R1 to R10 are "
     "described in the annex VentCon2_Measures_Reference_2026-10-02.docx, delivered with this "
     "package.**"),
    ("Vulnerability handling",
     "**Note on the template: Part II lists the safeguard \"Separate security updates\" twice, so "
     "the same answer appears in both rows.**"),
    ("Overall result:",
     "**Overall result: Gaps identified.** 20 safeguards in Part I and 10 points in Part II assessed: "
     "3 answered Yes, 22 No, 4 N/A. Key gaps: SBOM, support period, update path, disclosure policy, "
     "technical documentation, CE marking, authentication and access control. Market status: the "
     "product is a prototype in the own test rig and is not placed on the EU market, so the CRA "
     "obligations rest; the assessment shows the maturity level today."),
    ("Key gaps / evidence:",
     "**Evidence: sec/CRA_Compliance_Checklist.md (obligations 1 to 26, measures P1.1 to P5.2, risks "
     "R1 to R10), sec/analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md, 21 measure plans under "
     "sec/plan/.**"),
]

# Antwort zu jeder Frage des Fragebogens, Schlüssel ist der Fragetext.
QUESTIONNAIRE = {
    "Supplier legal name":
        "**VENTREX, part of the Aalberts group.**",
    "Supplier country of establishment":
        "**Austria.**",
    "Component name and version + model number":
        "**VentCon2 valve control unit, firmware 2.7.4 on ESP32-S3 (Arduino Nano ESP32); prototype "
        "hardware, no series model number yet**",
    "Component type (software, firmware, hardware with embedded software, cloud/SaaS service)":
        "**Hardware with embedded software and a local web interface; no cloud service.**",
    "Can the component be considered to be open source?":
        "**No. The firmware is proprietary, it is not published under a free and open-source licence.**",
    "Intended use of the digital elements of the component, as per the manufacturer":
        "**Regulate the outlet pressure in a fuel line, actuate the shut-off valve, monitor the "
        "pressure and close the valve on implausible readings; operation through a local WLAN "
        "access point.**",
    "Aalberts product(s) / product line(s) into which this component is integrated":
        "**The VentCon2 pressure control system itself; prototype operation in the own test rig, "
        "series not decided.**",
    "Aalberts company responsible for this integration":
        "**VENTREX.**",
    "Brief description of the component's function within the product":
        "**PID control of the outlet pressure with 10 ms sampling and a 1 kHz control loop, PWM "
        "actuation of the solenoid valve at 2 kHz and 14 bit, pressure measurement 0 to 10 bar, "
        "web interface with live chart, settings in LittleFS.**",
    "Date this component was first integrated / expected integration date":
        "**In operation in the own test rig since 2026 (firmware 2.7.4); series integration "
        "not scheduled.**",
    "Committed Support Period (min 5 years!)":
        "**Not committed yet. The CRA baseline is at least five years or the expected use time; "
        "measure P4.4 open, end date to be defined and published with the user information.**",
    "Has supplier assessed its own role designation under the CRA?":
        "**Yes, documented in sec/CRA_Compliance_Checklist.md section 2.**",
    "If so, what role does it consider itself to have:":
        "**Manufacturer within the meaning of Art. 3(13) CRA. No importer, distributor or authorised "
        "representative is involved, because VENTREX places the product under its own name and no "
        "third-party distribution is planned.**",
    "Does the supplier have a designated security contact or team (e.g. PSIRT)?":
        "**Partly: VENTREX Engineering department is the entry point; the single point of contact "
        "required by Art. 13(17) CRA is planned with measures P4.8 and P4.5.**",
    "Contact details for reporting a suspected vulnerability or incident":
        "**VENTREX Engineering department**; today the development contact and the serial console "
        "are the only channels",
    "Committed acknowledgement time upon receipt of a vulnerability report":
        "**Not committed yet. The CRA reporting duties of 24 h, 72 h, 14 days and one month apply "
        "from placing the product on the EU market; the internal process is measure P5.1 and the "
        "reporting access is measure P5.2, both resting.**",
    "Committed time to a substantive response or remediation plan":
        "**Not committed yet; remediation is handled case by case. Measure P4.5 open.**",
    "Does the supplier have a published vulnerability disclosure or coordinated disclosure policy? (attach if yes)":
        "**No, measure P4.5 open.**",
    "Does the supplier conduct regular security testing of the component (e.g. static/dynamic analysis, penetration testing)? Describe.":
        "**No regular security testing. One manual security analysis of the web interface exists "
        "(2026-08-27, findings R1 to R10) and the code is reviewed during development; update tests "
        "are part of measure P4.3.**",
    "Has the component had any publicly disclosed or CVE-registered vulnerabilities in the past 24 months? List, if any.":
        "**None registered. All findings so far are internal and documented as risks R1 to R10.**",
    "How and how quickly will the supplier notify Aalberts if a new vulnerability affecting this component is identified?":
        "**No committed process yet. Notification through the development contact; the formal "
        "process is measure P5.1, resting until the market decision.**",
    "Does the supplier provide security updates separately from feature updates where technically feasible?":
        "**Not established. Firmware and file system are uploaded separately with development "
        "tooling only; the release process is measure P4.3.**",
    "Will the supplier support coordinated fixes if a vulnerability is identified jointly, including sharing fixes upstream for open-source elements?":
        "**In principle yes. Reporting vulnerabilities in third-party components to their "
        "maintainers (Art. 13(6) CRA) is listed as an open duty in the master and not yet a "
        "defined process.**",
    "Can the supplier provide a software bill of materials (SBOM) for this component, in a machine-readable format (e.g. CycloneDX, SPDX)?":
        "**No SBOM yet, measure P4.2 open.**",
    "Does the component include third-party libraries? If so, can a full list be provided?":
        "**Yes, at top level: ArduinoJson 7.4.1, Adafruit ADS1X15 2.5.0, Arduino PID Library, "
        "littlefs, the Arduino core for ESP32 with ESP-IDF and FreeRTOS, and Chart.js vendored as "
        "data/chart.min.js. The list is available from platformio.ini, a full automatically "
        "generated list does not exist yet.**",
    "Is the SBOM kept up to date on each new release, and will Aalberts receive updated versions?":
        "**No SBOM exists yet, so no update cycle; part of measure P4.2.**",
    "Are there any known unsupported or end-of-life dependencies within the component?":
        "**Not assessed yet; to be determined with measure P4.2.**",
    "For how long does the supplier commit to providing security updates for this component version?":
        "**Not committed yet; intended at least five years or the expected use time, measure P4.4.**",
    "What happens to security support when this component version reaches end-of-life — is a supported upgrade path provided?":
        "**Not defined yet; measure P4.4 for the support period and P4.3 for the update path.**",
    "Does the supplier's committed support period meet or exceed Aalberts' required support period for the product it is integrated into?":
        "**Not determined yet. The target is to meet the Aalberts requirement of at least five "
        "years; the end date will be published in the user information as required by Annex II "
        "No. 7 CRA.**",
    "Are security updates provided free of charge for the duration of the support commitment?":
        "**Intended yes, in line with the CRA; no update delivery exists yet, measure P4.3.**",
    "What advance notice will the supplier give before ending support for this component?":
        "**Not defined yet. The end date of the support period has to be stated in the user "
        "information, measure P4.4 and P4.8.**",
    "Is the component already CE marked? If so, under which regulation(s)/directive(s)?":
        "**No. The product is not placed on the EU market (prototype in the own test rig, planned "
        "market India). CE marking is planned for the case of EU placement; the declaration of "
        "conformity and the CE marking are measure P4.7, the CRA route is internal control "
        "(module A), the RED route is open in measure P1.5.**",
    "Has the component been assessed against any recognised standard (e.g. IEC 62443, ETSI EN 303 645, ISO/IEC 27001)? Attach evidence.":
        "**ETSI EN 303 645 is used as a technical benchmark in the dossier (clauses 5.1-1 for "
        "passwords and 5.4-1 for stored credentials) without certification. IEC 62443 and "
        "ISO/IEC 27001 have not been assessed.**",
    "Can the supplier provide technical documentation suitable for inclusion in Aalberts' CRA conformity file upon request?":
        "**Partly. The dossier holds the master checklist, 21 measure plans and the risk and "
        "evidence analysis; the technical documentation in the structure of Annex VII CRA is "
        "measure P4.6 and still open.**",
    "Is the supplier aware of, and does it acknowledge, its own obligations as a manufacturer, importer, or distributor under the CRA?":
        "**Yes. Applicability, product classification as a standard product, the 26 obligations of "
        "the project list and the deadlines are documented in sec/CRA_Compliance_Checklist.md; the "
        "market status is not placed on the EU market, so the obligations rest until the first "
        "supply into the Union.**",
}

QUESTIONNAIRE_NOTES = [
    ("Supplier and Component Identification",
     "**The measures referenced in this document, P1.1 to P5.2, and the findings R1 to R10 are "
     "described in the annex VentCon2_Measures_Reference_2026-10-02.docx, delivered with this "
     "package.**"),
    ("Outcome: (to be verified by legal counsel)",
     "**Outcome: Accepted with conditions / follow-up required.** Gaps identified and listed in the "
     "checklist; no material risk for the current status, because the product is not placed on the "
     "EU market. The formal outcome remains with legal counsel."),
]


def balance_columns(filled_tables: list, answer_index: int) -> tuple[float, float]:
    """Antwortspalte nach Textanteil breiter setzen, die Tabellenbreite bleibt gleich.

    Liefert die neue Breite der Antwortspalte in Zoll und den Anteil an der Textbreite.
    """
    if not filled_tables:
        return 0.0, 0.0
    answer_total = sum(answer for _, answer, _ in filled_tables)
    other_total = sum(other for _, _, other in filled_tables)
    share = answer_total / (answer_total + other_total) if answer_total + other_total else 0.0
    share = min(max(share, MIN_ANSWER_SHARE), MAX_ANSWER_SHARE)
    result = (0.0, share)
    for table, _, _ in filled_tables:
        widths = grid_widths(table)
        total = sum(widths)
        deficit = share * total - widths[answer_index]
        new = list(widths)
        for index in sorted((i for i in range(len(new)) if i != answer_index),
                            key=lambda i: new[i], reverse=True):
            if deficit <= 0.001:
                break
            take = min(deficit, max(0.0, new[index] - MIN_OTHER_WIDTH))
            new[index] = round(new[index] - take, 4)
            deficit -= take
        if deficit > 0.001:
            continue
        new[answer_index] = round(total - sum(w for i, w in enumerate(new)
                                             if i != answer_index), 4)
        apply_widths(table, new)
        result = (new[answer_index], share)
    return result


def grid_widths(table) -> list[float]:
    """Spaltenbreiten aus dem Tabellengitter, in Zoll."""
    return [Twips(int(column.get(qn("w:w")))).inches
            for column in table._tbl.tblGrid.findall(qn("w:gridCol"))]


def apply_widths(table, widths: list[float]) -> None:
    """Breiten im Gitter und in jeder Zelle setzen, damit Word sie übernimmt."""
    for column, width in zip(table._tbl.tblGrid.findall(qn("w:gridCol")), widths):
        column.set(qn("w:w"), str(int(Inches(width).twips)))
    for row in table.rows:
        if len(row.cells) != len(widths):
            continue
        for cell, width in zip(row.cells, widths):
            cell.width = Inches(width)
    table.autofit = False


def markdown_blanks(text: str, entries) -> tuple[str, int]:
    """Unterstriche in Leerzeilen durch die Werte ersetzen, fett markiert."""
    lines = text.split("\n")
    filled = 0
    for prefix, values in entries:
        for index, line in enumerate(lines):
            if not line.startswith(prefix) or "__" not in line:
                continue
            for value in values:
                line = re.sub(r"__+", "**%s**" % value, line, count=1)
                filled += 1
            lines[index] = line
            break
    return "\n".join(lines), filled


def markdown_answers(text: str, table_answers: dict[str, str], cell_index: int) \
        -> tuple[str, int, list[str]]:
    """Antworten in die leeren Tabellenzellen der Markdown-Quelle setzen.

    Eine Antwort kann mehrere Zeilen treffen, weil die Vorlage eine Zeile doppelt führt; alle
    Treffer werden gefüllt.
    """
    used: set[str] = set()
    filled = 0
    output = []
    for line in text.split("\n"):
        if line.startswith("|"):
            cells = line.split("|")
            found = None
            for cell in cells:
                candidate = normalize(cell)
                if candidate in table_answers:
                    found = candidate
                    break
            if found:
                answer = table_answers[found]
                # Beim Aufteilen einer Zeile mit "|" liegt Zelle n auf dem Index n + 1.
                target_cell = cell_index + 1 if len(cells) > cell_index + 1 else 1
                if answer.split(" ")[0] not in cells[target_cell]:
                    rendered = rendered_markdown(answer)
                    if normalize(cells[target_cell]):
                        # Vorlagenzelle enthält schon einen Hinweis, die Antwort folgt darunter.
                        cells[target_cell] = cells[target_cell].rstrip() + " <br>" + rendered + " "
                    else:
                        cells[target_cell] = " " + rendered + " "
                    line = "|".join(cells)
                    filled += 1
                used.add(found)
        output.append(line)
    return "\n".join(output), filled, sorted(set(table_answers) - used)


def markdown_notes(text: str, notes: list[tuple[str, str]]) -> tuple[str, int, list[str]]:
    """Hinweise als ausgezeichnete Zeile hinter ihre Ankerzeile setzen.

    Die Marke trägt den Anker, damit die Worddatei ohne die Hinweisliste des Werkzeugs aus der
    Quelle gesetzt werden kann.
    """
    remaining = list(notes)
    inserted = 0
    output = []
    for line in text.split("\n"):
        output.append(line)
        # Mehrere Hinweise können an derselben Zeile hängen, etwa das Ergebnis und der Nachweis.
        for anchor, note in list(remaining):
            if normalize(anchor) not in normalize(line):
                continue
            output.append("%s %s --> %s" % (NOTE_MARKER, anchor, note))
            inserted += 1
            remaining = [entry for entry in remaining if entry[0] != anchor]
    return "\n".join(output), inserted, [anchor for anchor, _ in remaining]


def with_answers_row(text: str, docx_name: str) -> str:
    """Kopfzeile der Quelle um die Herkunft der Eintragungen ergänzen."""
    lines = text.split("\n")
    if any(line.startswith("| Answers |") for line in lines):
        return text
    for index, line in enumerate(lines):
        if line.startswith("| Note |"):
            lines.insert(index, ANSWERS_ROW % docx_name)
            break
    return "\n".join(lines)


def write_source(extract: Path, source: Path, table_answers: dict[str, str], cell_index: int,
                 notes: list[tuple[str, str]], blanks: tuple, docx_name: str) \
        -> tuple[int, int, int, list[str], list[str]]:
    """Markdown-Quelle der befüllten Worddatei schreiben: Vorlagentext plus Eintragungen."""
    text = extract.read_text(encoding="utf-8")
    text, filled, missing = markdown_answers(text, table_answers, cell_index)
    text, inserted, missing_notes = markdown_notes(text, notes)
    text, blank_count = markdown_blanks(text, blanks)
    text = with_answers_row(text, docx_name)
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text(text, encoding="utf-8", newline="\n")
    return filled, inserted, blank_count, missing, missing_notes


def answer_from_cell(cell: str) -> str:
    """Antwortzelle der Quelle in die Form des Antworttextes bringen.

    In der Quelle trennt `<br>` die Teile einer Zelle: der Vorlagenhinweis steht davor, die Antwort
    dahinter, und in der Checkliste trennt dasselbe `<br>` Urteil und Begründung. Die Worddatei
    bekommt den Vorlagenhinweis aus der Vorlage und die Antwort ohne Marken, das Urteil als eigene
    Zeile. Ohne diese Umsetzung landet `<br>` als Text in der Worddatei.
    """
    parts = [part.strip() for part in cell.split("<br>")]
    for index, part in enumerate(parts):
        if VERDICT_CELL.match(part):
            return " ".join([part] + parts[index + 1:])
    return parts[-1]


def read_source(source: Path, cell_index: int) \
        -> tuple[dict[str, str], list[tuple[str, str]], tuple]:
    """Eintragungen aus der Markdown-Quelle lesen: Antworten, Hinweise, Leerzeilenwerte.

    Das ist die Grundlage der Worddatei: Antworten stehen in den Tabellenzellen des Rumpfes,
    Hinweise in Zeilen mit der Marke `<!-- note: <Anker> -->`, Werte für Leerzeilen fett hinter
    ihrer Beschriftung. Der Kopfblock und der Anhang der Extraktion gehören nicht zum Dokument und
    bleiben außen vor.
    """
    answers: dict[str, str] = {}
    notes: list[tuple[str, str]] = []
    blanks: list[tuple[str, tuple[str, ...]]] = []
    in_head = False
    in_appendix = False
    for line in source.read_text(encoding="utf-8").split("\n"):
        stripped = line.strip()
        if stripped.startswith("<!-- extract:head-start -->"):
            in_head = True
            continue
        if stripped.startswith("<!-- extract:head-end -->"):
            in_head = False
            continue
        if stripped.startswith("<!-- extract:appendix-start -->"):
            in_appendix = True
            continue
        if in_head or in_appendix:
            continue
        if stripped.startswith(NOTE_MARKER):
            anchor, _, note = stripped[len(NOTE_MARKER):].partition("-->")
            notes.append((anchor.strip(), note.strip()))
            continue
        if stripped.startswith("|"):
            cells = line.split("|")
            if len(cells) <= cell_index + 1:
                continue
            if not cells[cell_index + 1].strip():
                continue
            answer = answer_from_cell(cells[cell_index + 1])
            for position, cell in enumerate(cells):
                if position != cell_index + 1 and normalize(cell):
                    answers[normalize(cell)] = answer
            continue
        values = re.findall(r"\*\*(.+?)\*\*", stripped)
        if values and stripped.split("**")[0].strip():
            blanks.append((stripped.split("**")[0], tuple(values)))
    return answers, notes, tuple(blanks)


def fill_docx_blanks(document, entries) -> int:
    """Unterstriche in Leerzeilen der Wordkopie durch Werte ersetzen, fett im Markenblau."""
    filled = 0
    for prefix, values in entries:
        paragraph = next((item for item in document.paragraphs
                          if item.text.strip().startswith(prefix)), None)
        if paragraph is None:
            continue
        for value in values:
            for run in paragraph.runs:
                if "_" not in run.text:
                    continue
                found = re.search(r"_+", run.text)
                head, tail = run.text[:found.start()], run.text[found.end():]
                run.text = head
                value_run = paragraph.add_run(value)
                value_run.bold = True
                value_run.font.color.rgb = ANSWER_COLOR
                run._element.addnext(value_run._element)
                if tail.strip():
                    value_run._element.addnext(paragraph.add_run(tail)._element)
                filled += 1
                break
    return filled


def normalize(text: str) -> str:
    return " ".join(text.replace("<br>", " ").replace("**", "").split())


def plain(text: str) -> str:
    """Text für das Worddokument: ohne Markdown-Marker, die Betonung trägt dort die Schrift."""
    return text.replace("**", "")


def split_verdict(answer: str) -> tuple[str, str]:
    """Antwort in Urteil und Begründung trennen, wenn sie mit Y, N oder N/A beginnt.

    Das Urteil steht in der ersten Zeile der Zelle, die Begründung darunter (Entscheidung E75).
    """
    found = VERDICT.match(answer)
    if not found:
        return "", answer
    return found.group(1), answer[found.end():]


def rendered_markdown(answer: str) -> str:
    """Antwort für die Markdown-Zelle, Urteil und Begründung durch `<br>` getrennt."""
    verdict, rest = split_verdict(answer)
    return "**%s**<br>%s" % (verdict, rest) if verdict else answer


def fill_docx(source: Path, target: Path, table_answers: dict[str, str], cell_index: int,
              notes: list[tuple[str, str]], blanks: tuple = ()) \
        -> tuple[int, int, list[str], tuple[float, float]]:
    """Antworten in Wordkopien setzen, das Original bleibt unberührt."""
    doc = Document(str(source))
    used: set[str] = set()
    filled = 0
    filled_tables: list[tuple[object, int, int]] = []
    for table in doc.tables:
        answer_chars = 0
        other_chars = 0
        for row in table.rows:
            cells = row.cells
            found = None
            for cell in cells:
                candidate = normalize(cell.text)
                if candidate in table_answers:
                    found = candidate
                    break
            if not found:
                continue
            answer = table_answers[found]
            target_cell = cells[cell_index] if len(cells) > cell_index else cells[0]
            if normalize(answer) in normalize(target_cell.text):
                used.add(found)
                continue
            paragraph = target_cell.paragraphs[0]
            if normalize(target_cell.text):
                paragraph.add_run().add_break()
            verdict, rest = split_verdict(answer)
            if verdict:
                head = paragraph.add_run(verdict)
                head.bold = True
                head.font.color.rgb = ANSWER_COLOR
                head.add_break()
                tail = paragraph.add_run(rest)
                tail.bold = False
                tail.font.color.rgb = ANSWER_COLOR
            else:
                run = paragraph.add_run(plain(answer))
                run.bold = True
                run.font.color.rgb = ANSWER_COLOR
            used.add(found)
            filled += 1
            answer_chars += len(answer)
            for position, cell in enumerate(cells):
                if position != cell_index:
                    other_chars += len(cell.text.strip())
        if answer_chars:
            filled_tables.append((table, answer_chars, other_chars))
    widths = balance_columns(filled_tables, cell_index)
    remaining = list(notes)
    inserted = 0
    for paragraph in doc.paragraphs:
        text = normalize(paragraph.text)
        for anchor, note in list(remaining):
            if normalize(anchor) in text:
                created = paragraph.insert_paragraph_before(plain(note))
                for run in created.runs:
                    run.bold = True
                    run.font.color.rgb = ANSWER_COLOR
                inserted += 1
                remaining = [entry for entry in remaining if entry[0] != anchor]
    if blanks:
        fill_docx_blanks(doc, blanks)
    keep_rows_together(doc)
    doc.save(str(target))
    return filled, inserted, [anchor for anchor, _ in remaining], widths


def keep_rows_together(document) -> int:
    """Tabellenzeilen nicht über Seiten umbrechen, Word schiebt sonst die halbe Zeile weiter.

    `w:cantSplit` steht im Schema vor `w:trHeight` und `w:tblHeader`, deshalb wird es vor dem
    ersten dieser Elemente eingefügt (Entscheidung E77).
    """
    tag = qn("w:cantSplit")
    count = 0
    for table in document.tables:
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


def main() -> int:
    jobs = [
        ("checklist", CHECKLIST_MD, CHECKLIST_DOCX,
         "CRA_Product_Maturity_Checklist_answered.docx", CHECKLIST_PART_I, CHECKLIST_PART_II,
         0, CHECKLIST_NOTES),
        ("questionnaire", QUESTIONNAIRE_MD, QUESTIONNAIRE_DOCX,
         "CRA_Supplier_Due_Diligence_Questionnaire_answered.docx", QUESTIONNAIRE, {}, 1,
         QUESTIONNAIRE_NOTES),
    ]
    for name, md_path, docx_path, docx_name, first, second, cell_index, notes in jobs:
        answers = dict(first)
        answers.update(second)
        blanks = BLANKS.get(name, ())
        source_path = SOURCE_DIR / (Path(docx_name).stem + ".md")
        filled_md, notes_md, blanks_md, missing, missing_notes = write_source(
            md_path, source_path, dict(answers), cell_index, list(notes), blanks, docx_name)
        source_answers, source_notes, source_blanks = read_source(source_path, cell_index)
        filled_docx, notes_docx, missing_docx, (answer_width, width_share) = fill_docx(
            docx_path, FILLED_DIR / docx_name, source_answers, cell_index, source_notes,
            source_blanks)
        print("%-14s Quelle: %2d Antworten, %d Hinweise, %d Leerzeilenfelder -> %s"
              % (name, filled_md, notes_md, blanks_md, source_path.name))
        print("%-14s Word:   %2d Antworten, %d Hinweise -> %s"
              % ("", filled_docx, notes_docx, docx_name))
        print("   Antwortspalte %.2f in, %.0f %% der Tabellenbreite"
              % (answer_width, width_share * 100))
        for label, items in (("Quelle", missing + missing_notes),
                             ("Word", missing_docx)):
            if items:
                print("   %s nicht zugeordnet: %d, erste: %s" % (label, len(items), items[:2]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
