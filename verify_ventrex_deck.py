# -*- coding: utf-8 -*-
"""
Verifiziert die aus der VENTREX-Vorlage erzeugte CRA-Praesentation.

Prueft Folieninventar, Vorlagenreste, Platzhalter, Foliennummernfeld, Schrift- und
Farbkonformitaet, Inhalt, Paketintegritaet und Tabellenbindigkeit.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" verify_ventrex_deck.py
"""

import math
import re
import sys

from pptx import Presentation
from pptx.enum.shapes import PP_PLACEHOLDER
from pptx.opc.constants import RELATIONSHIP_TYPE as RT
from pptx.util import Emu

TEMPLATE = "sec/VENTREX PPT Vorlage 2026.pptx"
DECK = "sec/2026-09-30_CRA_Compliance_Praesentation.pptx"

FOOTER_NAME = "VentCon2 - EU CRA Compliance"
FOOTER_DATE = "30.09.2026"

EXPECTED_MEASURES = (
    ["P1.%d" % i for i in range(1, 6)]
    + ["P2.%d" % i for i in range(1, 4)]
    + ["P3.%d" % i for i in range(1, 4)]
    + ["P4.%d" % i for i in range(1, 9)]
    + ["P5.%d" % i for i in range(1, 3)]
)

TEMPLATE_LEFTOVERS = [
    "Hier kommt der Folientitel", "Hier k\u00f6nnte der Text stehen", "Name Pr\u00e4sentation",
    "20.05.2026", "Segnaposto", "DIAGRAMM", "STEP 1", "STEP 2", "OPTION 1", "BEREICH 1",
    "PUNKT 1", "CHALLENGE", "Titolo", "Folientitel bearbeiten", "Text bearbeiten",
    "Kapiteltitel bearbeiten", "Pr\u00e4sentationstitel bearbeiten",
]

ALLOWED_FONTS = {"Clash Display", "Satoshi"}
BANNED_FONTS = ["Segoe UI", "Arial", "Calibri", "Times New Roman"]

problems = []
warnings = []
infos = []

# --- Palette und Theme-Schriften der Vorlage als Referenz --------------------- #
tpl = Presentation(TEMPLATE)
palette = set()
theme_fonts = {}
for index, master in enumerate(tpl.slide_masters):
    theme_xml = master.part.part_related_by(RT.THEME).blob.decode("utf-8")
    scheme = re.search(r"<a:clrScheme.*?</a:clrScheme>", theme_xml, re.S)
    if scheme:
        palette |= {c.upper() for c in re.findall(r'(?:srgbClr val|lastClr)="([0-9A-Fa-f]{6})"',
                                                  scheme.group(0))}
    theme_fonts[index] = dict(re.findall(
        r'<a:(majorFont|minorFont)>\s*<a:latin typeface="([^"]*)"', theme_xml))
del tpl

# --- Deck einlesen ------------------------------------------------------------ #
prs = Presentation(DECK)
slides = list(prs.slides)

texts = []
table_cells = []
cell_geometry = []
fonts_used = set()
colors_used = set()
embeds = 0
embed_broken = 0
slidenum_ok = 0
slidenum_expected = 0
footer_ok = 0
footer_expected = 0
date_ok = 0
date_expected = 0
titles_empty = []
empty_placeholders = []

for si, slide in enumerate(slides, start=1):
    xml = slide._element.xml
    fonts_used |= set(re.findall(r'typeface="([^"]+)"', xml))
    colors_used |= {c.upper() for c in re.findall(r'srgbClr val="([0-9A-Fa-f]{6})"', xml)}

    for rid in re.findall(r'r:embed="([^"]+)"', xml):
        embeds += 1
        if rid not in slide.part.rels:
            embed_broken += 1

    layout_has_footer = any(p.placeholder_format.type == PP_PLACEHOLDER.FOOTER
                            for p in slide.slide_layout.placeholders)
    has_number = False
    for ph in slide.placeholders:
        kind = ph.placeholder_format.type
        text = ph.text_frame.text if ph.has_text_frame else ""
        if text.strip():
            texts.append(text)
        else:
            empty_placeholders.append((si, str(kind)))
        if kind == PP_PLACEHOLDER.TITLE and not text.strip():
            titles_empty.append(si)
        if kind == PP_PLACEHOLDER.FOOTER:
            footer_expected += 1
            if text.strip() == FOOTER_NAME:
                footer_ok += 1
        if kind == PP_PLACEHOLDER.DATE:
            date_expected += 1
            if text.strip() == FOOTER_DATE:
                date_ok += 1
        if kind == PP_PLACEHOLDER.SLIDE_NUMBER:
            has_number = True
            slidenum_expected += 1
            if 'type="slidenum"' in ph._element.xml:
                slidenum_ok += 1
    if layout_has_footer and not has_number:
        problems.append("Folie %d hat ein Fusszeilen-Layout, aber keine Fusszeilenplatzhalter" % si)

    for shape in slide.shapes:
        if getattr(shape, "has_table", False) and shape.has_table:
            tbl = shape.table
            widths = [Emu(c.width).inches for c in tbl.columns]
            heights = [Emu(r.height).inches for r in tbl.rows]
            for ri, row in enumerate(tbl.rows):
                for ci, cell in enumerate(row.cells):
                    table_cells.append(cell.text)
                    if not cell.text.strip():
                        continue
                    texts.append(cell.text)
                    size = 9.0
                    for para in cell.text_frame.paragraphs:
                        for run in para.runs:
                            if run.font.size:
                                size = run.font.size.pt
                                break
                    cell_geometry.append((si, cell.text, widths[ci], heights[ri], size))

joined = "\n".join(texts)
cell_values = {c.strip() for c in table_cells}

# 0 Theme-Schriften der eingesetzten Master
master_ids = {id(m): i for i, m in enumerate(prs.slide_masters)}
used_masters = sorted({master_ids[id(s.slide_layout.slide_master)] for s in slides})
for index in used_masters:
    fonts = theme_fonts.get(index, {})
    infos.append("Master %d im Einsatz: major=%s minor=%s"
                 % (index, fonts.get("majorFont"), fonts.get("minorFont")))
if theme_fonts.get(2, {}).get("minorFont") != "Satoshi":
    problems.append("Master 2 nutzt nicht Satoshi, sondern %s"
                    % theme_fonts.get(2, {}).get("minorFont"))

# 1 Folieninventar
if len(slides) != 20:
    problems.append("Folienzahl ist %d, erwartet 20" % len(slides))
counts = {}
for slide in slides:
    name = slide.slide_layout.name
    counts[name] = counts.get(name, 0) + 1
if counts.get("COVER") != 1:
    problems.append("COVER nicht genau einmal vorhanden")
if counts.get("CLOSING SLIDE") != 1:
    problems.append("CLOSING SLIDE nicht genau einmal vorhanden")
if sum(v for k, v in counts.items() if k.startswith("SUBSECTION")) != 5:
    problems.append("Divider sind nicht genau fuenf")
infos.append("Layoutverteilung: " + ", ".join("%s=%d" % kv for kv in sorted(counts.items())))

# 2 Vorlagenreste
for token in TEMPLATE_LEFTOVERS:
    if token in joined:
        problems.append("Vorlagenrest gefunden: " + token)

# 3 Platzhalter
if titles_empty:
    problems.append("Folien ohne Titel: " + ", ".join(str(i) for i in titles_empty))
if footer_expected == 0 or footer_ok != footer_expected:
    problems.append("Fusszeile: %d von %d korrekt" % (footer_ok, footer_expected))
if date_expected == 0 or date_ok != date_expected:
    problems.append("Datum: %d von %d korrekt" % (date_ok, date_expected))
infos.append("Fusszeile gesetzt: %d von %d" % (footer_ok, footer_expected))
infos.append("Datum gesetzt: %d von %d" % (date_ok, date_expected))
infos.append("Leere Platzhalter: %d" % len(empty_placeholders))
for si, kind in empty_placeholders:
    if "PICTURE" not in kind:
        infos.append("   Folie %d, leerer Platzhalter vom Typ %s" % (si, kind))

# 4 Foliennummernfeld
if slidenum_expected and slidenum_ok != slidenum_expected:
    problems.append("Foliennummernfeld fehlt auf %d von %d Folien"
                    % (slidenum_expected - slidenum_ok, slidenum_expected))
infos.append("Foliennummern: %d von %d mit Feld slidenum" % (slidenum_ok, slidenum_expected))

# 5 Schrift
theme_refs = {f for f in fonts_used if f.startswith("+")}
explicit = {f for f in fonts_used if not f.startswith("+")}
unexpected = {f for f in explicit if f not in ALLOWED_FONTS}
if unexpected:
    problems.append("Unerwartete Schriften: " + ", ".join(sorted(unexpected)))
for banned in BANNED_FONTS:
    if banned in explicit:
        problems.append("Verbotene Schrift im Deck: " + banned)
infos.append("Schriften: theme=%s, explizit=%s" % (sorted(theme_refs), sorted(explicit)))

# 6 Farben
foreign = sorted(c for c in colors_used if c not in palette)
if foreign:
    problems.append("Farben ausserhalb der Vorlagenpalette: " + ", ".join(foreign))
infos.append("Farben im Deck: " + ", ".join(sorted(colors_used)))

# 7 Inhalt
missing_measures = [m for m in EXPECTED_MEASURES if m not in joined]
if missing_measures:
    problems.append("Fehlende Massnahmen-IDs: " + ", ".join(missing_measures))
line_numbers = set(re.findall(r"(?m)^\s*(\d{1,2})\s", joined))
observed_numbers = cell_values | line_numbers
missing_numbers = [str(n) for n in range(1, 27) if str(n) not in observed_numbers]
if missing_numbers:
    problems.append("Fehlende Pflicht-Nummern: " + ", ".join(missing_numbers))
if "\u2014" in joined:
    problems.append("Em-Dash gefunden")
for token in ("TODO", "TBD", "lorem"):
    if token in joined:
        warnings.append("Platzhalterverdacht: " + token)

# 7b RED-Aussage braucht Gueltigkeitsvermerk, EN 303 645 ist technischer Massstab
for si, slide in enumerate(slides, start=1):
    parts = []
    for shape in slide.shapes:
        if getattr(shape, "has_text_frame", False) and shape.has_text_frame:
            parts.append(shape.text_frame.text)
        if getattr(shape, "has_table", False) and shape.has_table:
            for row in shape.table.rows:
                for cell in row.cells:
                    parts.append(cell.text)
    blob = "\n".join(parts)
    rest = re.sub(r"RED-\d", "", blob)
    rest = re.sub(r"RED_\w+", "", rest)
    if re.search(r"\bRED\b", rest) and "2027-12-11" not in blob and "2026/339" not in blob:
        problems.append("Folie %d nennt RED ohne Gueltigkeitsvermerk 2027-12-11" % si)
    for chunk in parts:
        if "EN 303 645" in chunk and "technischer Massstab" not in chunk:
            problems.append("Folie %d: EN 303 645 ohne Kennzeichnung als technischer Massstab" % si)
            break

# 8 Paketintegritaet
if embed_broken:
    problems.append("%d r:embed ohne gueltige Relationship" % embed_broken)
infos.append("Bilder und Embeds: %d, defekt: %d" % (embeds, embed_broken))

# 9 Tabellenbindigkeit und Ueberlaufschaetzung
for si, text, width_in, height_in, size in cell_geometry:
    chars_per_line = max(8.0, (width_in - 0.14) * 72.0 / (0.52 * size))
    lines = 0
    for part in text.split("\n"):
        lines += max(1, math.ceil(len(part) / chars_per_line))
    needed = lines * size * 1.22 + 6
    if needed > height_in * 72.0 + 1:
        warnings.append("Folie %d: Zelle braucht ca. %.0f pt, Zeile hat %.0f pt: %s"
                        % (si, needed, height_in * 72.0, text[:55]))

print("Folien:          ", len(slides))
print("Tabellen:        ", sum(1 for s in slides for sh in s.shapes
                                if getattr(sh, "has_table", False) and sh.has_table))
print("Textabschnitte:  ", len(texts))
print("Zeichen gesamt:  ", len(joined))
print("Massnahmen-IDs:  ", len(EXPECTED_MEASURES), "geprueft")
print("Pflicht-Nummern: ", 26 - len(missing_numbers), "von 26 gefunden")

print("\nInformationen:")
for item in infos:
    print("  -", item)

if problems:
    print("\nFEHLER (%d):" % len(problems))
    for item in problems:
        print("  -", item)
else:
    print("\nKeine Fehler.")

if warnings:
    print("\nHinweise (%d):" % len(warnings))
    for item in warnings:
        print("  -", item)
else:
    print("Keine Hinweise.")

sys.exit(1 if problems else 0)
