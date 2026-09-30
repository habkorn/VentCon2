# -*- coding: utf-8 -*-
"""
Verifiziert die erzeugte Praesentation gegen den Inhalt des Masters.

Prueft: Folienzahl, Vollstaendigkeit der Massnahmen-IDs, Pflicht-Nummern 1 bis 26,
Dokumentnamen des Registers, Em-Dash-Freiheit und eine Schaetzung auf Zellueberlauf
in Tabellen.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" verify_cra_presentation.py
"""

import math
import sys

from pptx import Presentation
from pptx.util import Emu

PPTX = "sec/2026-09-29_CRA_Compliance_Praesentation.pptx"

EXPECTED_MEASURES = (
    ["P1.%d" % i for i in range(1, 5)]
    + ["P2.%d" % i for i in range(1, 4)]
    + ["P3.%d" % i for i in range(1, 4)]
    + ["P4.%d" % i for i in range(1, 9)]
    + ["P5.%d" % i for i in range(1, 3)]
)

EXPECTED_DOCS = [
    "CRA_Compliance_Checklist.md",
    "2026-08-27_Webinterface_Sicherheitsanalyse.md",
    "2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md",
    "2026-08-31_P1-3_Sensor_Calibration_Plan.md",
    "2026-08-31_P2-2_AP_Password_Plan.md",
    "2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md",
    "2026-09-28_P1-4_PID_Gain_Limits_Plan.md",
    "2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md",
    "2026-09-28_P3-3_Radio_Encryption_Plan.md",
    "2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md",
    "2026-09-28_P4-2_SBOM.md",
    "2026-09-28_P4-3_Signed_OTA.md",
    "2026-09-28_P4-4_Support_Period.md",
    "2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md",
    "2026-09-28_P4-6_Technical_Documentation_Annex_VII.md",
    "2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md",
    "2026-09-28_P4-8_User_Information_Annex_II.md",
    "2026-09-28_P5-1_Reporting_Process.md",
    "2026-09-28_P5-2_SRP_CSIRT_Access.md",
    "2026-08-28_Mutex_How_It_Works.md",
    "2026-08-29_Web_Auth_How_It_Works.md",
    "2026-09-28_CRA_Checkliste_als_Master_Plan.md",
    "EU_CRA_Regulation_2024_2847_EN.pdf",
]

prs = Presentation(PPTX)
problems = []
warnings = []
texts = []
cells = []           # (slide_idx, text, col_width_in, row_height_in, font_pt)
tables = 0

for si, slide in enumerate(prs.slides, start=1):
    for shape in slide.shapes:
        if shape.has_text_frame:
            for p in shape.text_frame.paragraphs:
                line = "".join(r.text for r in p.runs)
                if line.strip():
                    texts.append(line)
        if getattr(shape, "has_table", False) and shape.has_table:
            tables += 1
            tbl = shape.table
            widths = [c.width for c in tbl.columns]
            heights = [r.height for r in tbl.rows]
            for ri, row in enumerate(tbl.rows):
                for ci, cell in enumerate(row.cells):
                    t = cell.text
                    if not t.strip():
                        continue
                    texts.append(t)
                    size = 10.0
                    for p in cell.text_frame.paragraphs:
                        for r in p.runs:
                            if r.font.size:
                                size = r.font.size.pt
                                break
                    cells.append((si, t, Emu(widths[ci]).inches,
                                  Emu(heights[ri]).inches, size))

joined = "\n".join(texts)
flat = " \n ".join(texts)

# 1 Folienzahl
if len(prs.slides) != 26:
    problems.append("Folienzahl ist %d, erwartet 26" % len(prs.slides))

# 2 Massnahmen-IDs
missing = [m for m in EXPECTED_MEASURES if m not in flat]
if missing:
    problems.append("fehlende Massnahmen-IDs: " + ", ".join(missing))

# 3 Pflicht-Nummern 1 bis 26 als eigene Zellwerte
cell_values = {c[1].strip() for c in cells}
num_missing = [str(n) for n in range(1, 27) if str(n) not in cell_values]
if num_missing:
    warnings.append("Pflicht-Nummern nicht als Einzelzelle gefunden: " + ", ".join(num_missing))

# 4 Dokumentnamen
doc_missing = [d for d in EXPECTED_DOCS if d not in joined]
if doc_missing:
    problems.append("fehlende Dokumentnamen: " + ", ".join(doc_missing))

# 5 Em-Dash und typische Platzhalter
if "\u2014" in joined:
    problems.append("Em-Dash gefunden")
for token in ("TODO", "TBD", "lorem", "XXX", "None"):
    if token in joined:
        warnings.append("Platzhalterverdacht: " + token)

# 6 Zellueberlauf schaetzen
for si, t, w_in, h_in, size in cells:
    chars_per_line = max(8.0, (w_in - 0.14) * 72.0 / (0.52 * size))
    lines = 0
    for part in t.split("\n"):
        lines += max(1, math.ceil(len(part) / chars_per_line))
    needed_pt = lines * size * 1.22 + 6
    if needed_pt > h_in * 72.0 + 1:
        warnings.append(
            "Folie %d: Zelle braucht ca. %.0f pt, Zeile hat %.0f pt: %s"
            % (si, needed_pt, h_in * 72.0, t[:60]))

print("Folien:            ", len(prs.slides))
print("Tabellen:          ", tables)
print("Textabschnitte:    ", len(texts))
print("Zeichen gesamt:    ", len(joined))
print("Massnahmen-IDs:    ", len(EXPECTED_MEASURES), "geprueft")
print("Dokumentnamen:     ", len(EXPECTED_DOCS), "geprueft")

# 7 Geometrie: ausserhalb der Folie oder Kollision mit der Fusszeile
SW = Emu(prs.slide_width).inches
SH = Emu(prs.slide_height).inches
layout = []
for si, slide in enumerate(prs.slides, start=1):
    for shape in slide.shapes:
        l = Emu(shape.left).inches
        t = Emu(shape.top).inches
        w = Emu(shape.width).inches
        h = Emu(shape.height).inches
        if l < -0.01 or t < -0.01 or l + w > SW + 0.01 or t + h > SH + 0.01:
            layout.append("Folie %d: Objekt ausserhalb der Folie (%.2f, %.2f, %.2f, %.2f)"
                          % (si, l, t, w, h))
        full_bleed = w >= SW - 0.02 or h >= SH - 0.02
        if not full_bleed and t < 6.98 and t + h > 7.05:
            layout.append("Folie %d: Objekt reicht in die Fusszeile (unten %.2f in)"
                          % (si, t + h))
        if not full_bleed and t < 6.98 and t + h > 6.95:
            layout.append("Folie %d: Objekt reicht unter 6,95 in (unten %.2f in)"
                          % (si, t + h))
print("Geometrie:         ", "ok" if not layout else "%d Hinweise" % len(layout))

if problems:
    print("\nFEHLER:")
    for p in problems:
        print("  -", p)
else:
    print("\nKeine Fehler in den Vollstaendigkeitspruefungen.")

if warnings:
    print("\nHinweise (%d):" % len(warnings))
    for w in warnings:
        print("  -", w)
else:
    print("Keine Hinweise.")

if layout:
    print("\nLayout-Hinweise (%d):" % len(layout))
    for l in layout:
        print("  -", l)

sys.exit(1 if problems else 0)
