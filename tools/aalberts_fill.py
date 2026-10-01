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

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/aalberts_fill.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

from docx import Document
from docx.shared import RGBColor

ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIR = ROOT / "sec" / "AalbertsChecklists"

# Markenblau der VENTREX- und Aalberts-Vorlagen, Antworten erscheinen fett in dieser Farbe.
ANSWER_COLOR = RGBColor(0x00, 0x2F, 0x87)

CHECKLIST_MD = SOURCE_DIR / "CRA_Product_Maturity_Checklist.md"
CHECKLIST_DOCX = SOURCE_DIR / "CRA product maturity checklist - Sep 2026.docx"
QUESTIONNAIRE_MD = SOURCE_DIR / "CRA_Supplier_Due_Diligence_Questionnaire.md"
QUESTIONNAIRE_DOCX = SOURCE_DIR / "CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx"
ADDENDUM_MD = SOURCE_DIR / "CRA_Compliance_Addendum_Supplier_Agreement.md"
ADDENDUM_DOCX = SOURCE_DIR / "CRA Compliance Addendum (Supplier Agreement) - template Sep 2026 .docx"

ANSWERS_ROW = ("| Answers | Filled in from the VENTREX dossier on 2026-10-01, marked in bold. "
               "Template text is unchanged. Regenerate the extract with tools/docx_extract.py "
               "and re-apply with tools/aalberts_fill.py if needed. |")

# Antwort zu jeder Sicherheitsvorgabe in Teil I der Checkliste, Schlüssel ist der Vorgabentext.
CHECKLIST_PART_I = {
    "Risk-based design: The product’s security measures match its identified cybersecurity risks (see Appendix 1 below).":
        "**N** no documented product risk assessment yet; the risks R1 to R9 from the security "
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
        "**N** settings are stored as plain JSON in LittleFS and readable (R5), measure P2.1 partly done; "
        "radio encryption open (P3.3)",
    "Integrity: Data, commands, software and settings are protected against unauthorised changes, and corruption is detected and reported.":
        "**N** state can be changed by GET without CSRF protection (R4), measure P1.2 in progress",
    "Data minimisation: The product processes only the data needed for its intended purpose.":
        "**Y** pressure, setpoint, PID parameters and valve duty only; no personal data, no cloud, "
        "no remote maintenance",
    "Availability: Essential and basic functions remain available, or can recover, after an incident, including denial-of-service attacks.":
        "**Y** limited, two clients maximum and client DoS handled in firmware 2.7.3 (measure P3.2 done); "
        "rate limiting and save debouncing still open (P2.3, risk R7)",
    "No harmful network impact: The product minimises disruption to services provided by connected devices or networks.":
        "**Y** local access point only, no outbound traffic, no cloud service",
    "Limited attack surface: External interfaces, access points and unnecessary functions are minimised.":
        "**N** file access to LittleFS is possible (R5, P2.1 partly done) and PID gains are unlimited "
        "(R6, P1.4 open); interfaces are web UI, WLAN, serial console and USB",
    "Incident impact limited: Security controls reduce the possible impact of successful attacks.":
        "**N** no hard upper limits for PID gains (R6, P1.4 open); the pressure safety shutdown limits "
        "the physical impact",
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
        "**N** vulnerabilities are documented as risks R1 to R9 in the security analysis, a component "
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
    ("Product: __________ Owner: __________ Date: __________",
     "**Product: VentCon2 valve control unit, firmware 2.7.4 (prototype). Owner: VENTREX product "
     "owner, [name to be completed]. Date: 2026-10-01.**"),
    ("Overall result:",
     "**Overall result: Gaps identified.** 20 safeguards in Part I and 10 points in Part II assessed: "
     "3 answered Yes, 22 No, 4 N/A. Key gaps: SBOM, support period, update path, disclosure policy, "
     "technical documentation, CE marking, authentication and access control. Market status: the "
     "product is a prototype in the own test rig and is not placed on the EU market, so the CRA "
     "obligations rest; the assessment shows the maturity level today."),
    ("Key gaps / evidence:",
     "**Evidence: sec/CRA_Compliance_Checklist.md (obligations 1 to 26, measures P1.1 to P5.2, risks "
     "R1 to R9), sec/analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md, 21 measure plans under "
     "sec/plan/.**"),
]

# Antwort zu jeder Frage des Fragebogens, Schlüssel ist der Fragetext.
QUESTIONNAIRE = {
    "Supplier legal name":
        "**VENTREX, part of the Aalberts group.**",
    "Supplier country of establishment":
        "**[to be completed]** entity details of the VENTREX company",
    "Component name and version + model number":
        "**VentCon2 valve control unit, firmware 2.7.4 on ESP32-S3 (Arduino Nano ESP32); prototype "
        "hardware, model number [to be completed]**",
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
        "[date to be completed].**",
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
        "**Not established yet. The single point of contact required by Art. 13(17) CRA is planned "
        "with measures P4.8 and P4.5.**",
    "Contact details for reporting a suspected vulnerability or incident":
        "**[to be completed]**; today the development contact and the serial console are the only "
        "channels",
    "Committed acknowledgement time upon receipt of a vulnerability report":
        "**Not committed yet. The CRA reporting duties of 24 h, 72 h, 14 days and one month apply "
        "from placing the product on the EU market; the internal process is measure P5.1, resting.**",
    "Committed time to a substantive response or remediation plan":
        "**Not committed yet; remediation is handled case by case. Measure P4.5 open.**",
    "Does the supplier have a published vulnerability disclosure or coordinated disclosure policy? (attach if yes)":
        "**No, measure P4.5 open.**",
    "Does the supplier conduct regular security testing of the component (e.g. static/dynamic analysis, penetration testing)? Describe.":
        "**No regular security testing. One manual security analysis of the web interface exists "
        "(2026-08-27, findings R1 to R9) and the code is reviewed during development; update tests "
        "are part of measure P4.3.**",
    "Has the component had any publicly disclosed or CVE-registered vulnerabilities in the past 24 months? List, if any.":
        "**None registered. All findings so far are internal and documented as risks R1 to R9.**",
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
        "market India). CE marking is planned for the case of EU placement; the CRA route is "
        "internal control (module A), the RED route is open in measure P1.5.**",
    "Has the component been assessed against any recognised standard (e.g. IEC 62443, ETSI EN 303 645, ISO/IEC 27001)? Attach evidence.":
        "**ETSI EN 303 645 is used as a technical benchmark in the dossier (clauses 5.1-1 for "
        "passwords and 5.4-1 for stored credentials) without certification. IEC 62443 and "
        "ISO/IEC 27001 have not been assessed. [to be completed by VENTREX management]**",
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
    ("Completed by (name, role, entity name):",
     "**Completed by: [name to be completed], [role to be completed], VENTREX (Aalberts group "
     "company).**"),
    ("Date: _______________",
     "**Date: 2026-10-01.**"),
    ("Outcome: (to be verified by legal counsel)",
     "**Outcome: Accepted with conditions / follow-up required.** Gaps identified and listed in the "
     "checklist; no material risk for the current status, because the product is not placed on the "
     "EU market. The formal outcome remains with legal counsel."),
]

ADDENDUM_NOTES = [
    ("This Addendum is entered into between",
     "**Parties as prepared by VENTREX: Company = VENTREX (Aalberts group company); Supplier = "
     "[Supplier, to be completed]. No supplier is bound by this draft.**"),
    ("Signed:",
     "**Note for VENTREX: the commitments in clauses 2 (notification within 24 hours), 3 (SBOM on "
     "request), 5 (acknowledgement within 48 hours, remediation plan within 30 days), 6 (audit "
     "right, code replacement within one year) and 7 (material breach) have to be agreed with the "
     "supplier. None of them is agreed today. The signature block stays open.**"),
]


def normalize(text: str) -> str:
    return " ".join(text.replace("<br>", " ").replace("**", "").split())


def fill_markdown(path: Path, table_answers: dict[str, str], cell_index: int,
                  notes: list[tuple[str, str]]) -> tuple[int, int, list[str]]:
    """Antworten in leere Tabellenzellen setzen und Hinweise nach Ankerzeilen einfügen.

    Eine Antwort kann mehrere Zeilen treffen, weil die Vorlage eine Zeile doppelt führt; alle
    Treffer werden gefüllt. Hinweise werden je Ankerzeile eingefügt, auch mehrere hintereinander.
    """
    lines = path.read_text(encoding="utf-8").split("\n")
    existing = "\n".join(lines)
    used: set[str] = set()
    filled = 0
    inserted = 0
    remaining = list(notes)
    output = []
    for line in lines:
        output.append(line)
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
                if answer.split(" ")[0] in cells[target_cell]:
                    used.add(found)
                    continue
                if normalize(cells[target_cell]):
                    # Vorlagenzelle enthält schon einen Hinweis, die Antwort folgt darunter.
                    cells[target_cell] = cells[target_cell].rstrip() + " <br>" + answer + " "
                else:
                    cells[target_cell] = " " + answer + " "
                output[-1] = "|".join(cells)
                used.add(found)
                filled += 1
                continue
        for anchor, note in list(remaining):
            if normalize(anchor) in normalize(line):
                if note in existing or note in "\n".join(output):
                    inserted += 1
                    remaining = [entry for entry in remaining if entry[0] != anchor]
                    continue
                output.extend(["", note])
                inserted += 1
                remaining = [entry for entry in remaining if entry[0] != anchor]
    path.write_text("\n".join(output), encoding="utf-8")
    return filled, inserted, sorted(set(table_answers) - used), [a for a, _ in remaining]


def fill_docx(source: Path, target: Path, table_answers: dict[str, str], cell_index: int,
              notes: list[tuple[str, str]]) -> tuple[int, int, list[str]]:
    """Antworten in Wordkopien setzen, das Original bleibt unberührt."""
    doc = Document(str(source))
    used: set[str] = set()
    filled = 0
    for table in doc.tables:
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
            if answer.split(" ")[0] in target_cell.text:
                used.add(found)
                continue
            paragraph = target_cell.paragraphs[0]
            if normalize(target_cell.text):
                paragraph.add_run().add_break()
            run = paragraph.add_run(answer)
            run.bold = True
            run.font.color.rgb = ANSWER_COLOR
            used.add(found)
            filled += 1
    remaining = list(notes)
    inserted = 0
    for paragraph in doc.paragraphs:
        text = normalize(paragraph.text)
        for anchor, note in list(remaining):
            if normalize(anchor) in text:
                created = paragraph.insert_paragraph_before(note)
                for run in created.runs:
                    run.bold = True
                    run.font.color.rgb = ANSWER_COLOR
                inserted += 1
                remaining = [entry for entry in remaining if entry[0] != anchor]
    doc.save(str(target))
    return filled, inserted, [anchor for anchor, _ in remaining]


def insert_head_row(path: Path) -> None:
    """Vermerk in den Kopfblock, damit der Extrakt nicht für quellentreu gehalten wird."""
    lines = path.read_text(encoding="utf-8").split("\n")
    if any(line.startswith("| Answers |") for line in lines):
        return
    for index, line in enumerate(lines):
        if line.startswith("| Note |"):
            lines.insert(index, ANSWERS_ROW)
            break
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    jobs = [
        ("checklist", CHECKLIST_MD, CHECKLIST_DOCX,
         "CRA_Product_Maturity_Checklist_answered.docx", CHECKLIST_PART_I, CHECKLIST_PART_II,
         0, CHECKLIST_NOTES),
        ("questionnaire", QUESTIONNAIRE_MD, QUESTIONNAIRE_DOCX,
         "CRA_Supplier_Due_Diligence_Questionnaire_answered.docx", QUESTIONNAIRE, {}, 1,
         QUESTIONNAIRE_NOTES),
        ("addendum", ADDENDUM_MD, ADDENDUM_DOCX,
         "CRA_Compliance_Addendum_Supplier_Agreement_answered.docx", {}, {}, 0, ADDENDUM_NOTES),
    ]
    for name, md_path, docx_path, docx_name, first, second, cell_index, notes in jobs:
        answers = dict(first)
        answers.update(second)
        insert_head_row(md_path)
        filled_md, notes_md, missing, missing_notes = fill_markdown(md_path, dict(answers),
                                                                   cell_index, list(notes))
        filled_docx, notes_docx, missing_docx = fill_docx(docx_path,
                                                          docx_path.with_name(docx_name),
                                                          dict(answers), cell_index,
                                                          list(notes))
        print("%-14s Markdown: %2d Antworten, %d Hinweise | Word: %2d Antworten, %d Hinweise -> %s"
              % (name, filled_md, notes_md, filled_docx, notes_docx, docx_name))
        for label, items in (("Markdown", missing + missing_notes), ("Word", missing_docx)):
            if items:
                print("   %s nicht zugeordnet: %d, erste: %s" % (label, len(items), items[:2]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
