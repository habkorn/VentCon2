# -*- coding: utf-8 -*-
"""
Verifiziert die aus der VENTREX-Vorlage erzeugte CRA-Präsentation.

Prüft Folieninventar, Vorlagenreste, Platzhalter, Foliennummernfeld, Schrift- und
Farbkonformität, Inhalt, Paketintegrität und Tabellenbindigkeit.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" verify_ventrex_deck.py
"""

import math
import os
import re
import sys

from pptx import Presentation
from pptx.enum.shapes import PP_PLACEHOLDER
from pptx.opc.constants import RELATIONSHIP_TYPE as RT
from pptx.util import Emu

TEMPLATE = "sec/VENTREX PPT Vorlage 2026.pptx"
DECK = sys.argv[1] if len(sys.argv) > 1 else "sec/2026-10-01_CRA_Compliance_Praesentation.pptx"
MASTER = sys.argv[2] if len(sys.argv) > 2 else "sec/CRA_Compliance_Checklist.md"

FOOTER_NAME = "VentCon2 - EU CRA Compliance"
FOOTER_DATE = "01.10.2026"

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
row_texts = []
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
        problems.append("Folie %d hat ein Fußzeilen-Layout, aber keine Fußzeilenplatzhalter" % si)

    for shape in slide.shapes:
        if getattr(shape, "has_table", False) and shape.has_table:
            tbl = shape.table
            widths = [Emu(c.width).inches for c in tbl.columns]
            heights = [Emu(r.height).inches for r in tbl.rows]
            for ri, row in enumerate(tbl.rows):
                row_texts.append(" | ".join(cell.text for cell in row.cells))
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

# Text je Folie, für die Prüfung auf Vorwärtsverweise
slide_texts = []
for slide in slides:
    parts = []
    for shape in slide.shapes:
        if getattr(shape, "has_text_frame", False) and shape.has_text_frame:
            parts.append(shape.text_frame.text)
        if getattr(shape, "has_table", False) and shape.has_table:
            for row in shape.table.rows:
                parts.extend(cell.text for cell in row.cells)
    slide_texts.append("\n".join(parts))

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
if len(slides) != 28:
    problems.append("Folienzahl ist %d, erwartet 28" % len(slides))
counts = {}
for slide in slides:
    name = slide.slide_layout.name
    counts[name] = counts.get(name, 0) + 1
if counts.get("COVER") != 1:
    problems.append("COVER nicht genau einmal vorhanden")
if counts.get("CLOSING SLIDE") != 1:
    problems.append("CLOSING SLIDE nicht genau einmal vorhanden")
if sum(v for k, v in counts.items() if k.startswith("SUBSECTION")) != 7:
    problems.append("Divider sind nicht genau sieben")
infos.append("Layoutverteilung: " + ", ".join("%s=%d" % kv for kv in sorted(counts.items())))

# 2 Vorlagenreste
for token in TEMPLATE_LEFTOVERS:
    if token in joined:
        problems.append("Vorlagenrest gefunden: " + token)

# 3 Platzhalter
if titles_empty:
    problems.append("Folien ohne Titel: " + ", ".join(str(i) for i in titles_empty))
if footer_expected == 0 or footer_ok != footer_expected:
    problems.append("Fußzeile: %d von %d korrekt" % (footer_ok, footer_expected))
if date_expected == 0 or date_ok != date_expected:
    problems.append("Datum: %d von %d korrekt" % (date_ok, date_expected))
infos.append("Fußzeile gesetzt: %d von %d" % (footer_ok, footer_expected))
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
    problems.append("Farben außerhalb der Vorlagenpalette: " + ", ".join(foreign))
infos.append("Farben im Deck: " + ", ".join(sorted(colors_used)))

# 7 Inhalt
missing_measures = [m for m in EXPECTED_MEASURES if m not in joined]
if missing_measures:
    problems.append("Fehlende Maßnahmen-IDs: " + ", ".join(missing_measures))
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

# 7b RED-Aussage braucht Gültigkeitsvermerk, EN 303 645 ist technischer Maßstab
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
        problems.append("Folie %d nennt RED ohne Gültigkeitsvermerk 2027-12-11" % si)
    for chunk in parts:
        if "EN 303 645" in chunk and "technischer Maßstab" not in chunk:
            problems.append("Folie %d: EN 303 645 ohne Kennzeichnung als technischer Maßstab" % si)
            break

# 7c Datums-Whitelist, Statusabgleich und Quellenabgleich mit dem Master
ALLOWED_DATES = {
    "01.10.2026", "27.07.2026", "11.09.2026", "30.09.2026",
    "2024-12-10", "2025-01-30", "2025-08-01", "2026-06-11", "2026-08-29", "2026-09-04",
    "2026-09-11", "2026-09-28", "2026-09-30", "2026-10-01", "2027-12-11",
}
found_dates = set(re.findall(r"\b\d{2}\.\d{2}\.\d{4}\b", joined))
found_dates |= set(re.findall(r"\b\d{4}-\d{2}-\d{2}\b", joined))
stale = sorted(d for d in found_dates if d not in ALLOWED_DATES)
if stale:
    problems.append("Datum außerhalb der Whitelist: " + ", ".join(stale))

for required in ("zurückgestellt", "Prototypenbetrieb"):
    if required not in joined:
        problems.append("Pflichtangabe fehlt im Deck: " + required)
if "Erste Entscheidung" in joined:
    problems.append("Deck zeigt die Marktentscheidung noch als offenen Punkt")

if os.path.exists(MASTER):
    master_text = open(MASTER, encoding="utf-8").read()
    sources = master_text.split("## 11. Quellen")[-1].split("## 12.")[0]
    reference = re.compile(r"\(EU\) \d{4}/\d+|C\(2026\) 5252|M/606|\bEN \d{5}(?: \d{3})?\b")
    master_refs = set(reference.findall(sources))
    deck_refs = set(reference.findall(joined))
    only_master = sorted(master_refs - deck_refs)
    only_deck = sorted(deck_refs - master_refs)
    if only_master:
        problems.append("Quelle nur im Master, nicht im Deck: " + ", ".join(only_master))
    if only_deck:
        problems.append("Quelle nur im Deck, nicht im Master: " + ", ".join(only_deck))
    infos.append("Quellenabgleich: %d Verweise im Master, %d im Deck"
                 % (len(master_refs), len(deck_refs)))

# 7d Inhaltsverzeichnis deckt alle Folien ab
if len(slides) > 2:
    parts = []
    for shape in slides[1].shapes:
        if getattr(shape, "has_table", False) and shape.has_table:
            for row in shape.table.rows:
                for cell in row.cells:
                    parts.append(cell.text)
        elif getattr(shape, "has_text_frame", False) and shape.has_text_frame:
            parts.append(shape.text_frame.text)
    toc_text = "\n".join(parts)
    missing_entries = []
    for number, slide in enumerate(slides[2:], start=3):
        title = ""
        for shape in slide.shapes:
            if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
                title = shape.text_frame.text.strip()
        if not title:
            continue
        plain = re.sub(r"^\d+(?:\.\d+)?\s+", "", title)
        words = re.findall(r"[A-Za-z0-9]+", plain)[:2]
        if not all(word in toc_text for word in words):
            missing_entries.append("%d %s" % (number, title))
    if missing_entries:
        problems.append("Folien fehlen im Inhaltsverzeichnis: " + "; ".join(missing_entries))
    else:
        infos.append("Inhaltsverzeichnis deckt alle betitelten Folien ab")

# 7e Mehrstufige Nummerierung der Folien
EXPECTED_NUMBERS = ["1", "1.1", "1.2",
                    "2", "2.1", "2.2",
                    "3", "3.1", "3.2", "3.3",
                    "4", "4.1", "4.2", "4.3", "4.4",
                    "5", "5.1", "5.2", "5.3",
                    "6", "6.1", "6.2", "6.3",
                    "7", "7.1"]
observed_numbers = []
for slide in slides:
    for shape in slide.shapes:
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
            match = re.match(r"^(\d+(?:\.\d+)?)\s+\S", shape.text_frame.text.strip())
            if match:
                observed_numbers.append(match.group(1))
if observed_numbers != EXPECTED_NUMBERS:
    problems.append("Foliennummerierung weicht ab: erwartet %s, gefunden %s"
                    % (" ".join(EXPECTED_NUMBERS), " ".join(observed_numbers)))
else:
    infos.append("Mehrstufige Nummerierung vollständig: " + " ".join(observed_numbers))

# 7f Kapitelverweis in der Kopfzeile der Unterfolien
CHAPTER_TEXT = {
    "1": "1 Systembeschreibung",
    "2": "2 Regelwerk",
    "3": "3 Produkt und Anwendbarkeit",
    "4": "4 Risiken und Maßnahmen",
    "5": "5 Pflichten der CRA",
    "6": "6 Dokumente und Quellen",
    "7": "7 Fazit und nächste Schritte",
}
# Jede Folie mit Nummer traegt den Kapitelverweis ihres Abschnitts.
NAMED_SECTIONS = {}
kicker_problems = []
for number, slide in enumerate(slides, start=1):
    title = ""
    for shape in slide.shapes:
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
            title = shape.text_frame.text.strip()
    kickers = []
    for shape in slide.shapes:
        if shape.is_placeholder or not getattr(shape, "has_text_frame", False):
            continue
        if not shape.has_text_frame:
            continue
        text = shape.text_frame.text.strip()
        if "\n" in text:
            continue
        if re.match(r"^\d+ [A-Za-z]", text) or text in set(NAMED_SECTIONS.values()):
            kickers.append(text)
    match = re.match(r"^(\d+)\.\d+\s", title)
    expected = CHAPTER_TEXT.get(match.group(1)) if match else NAMED_SECTIONS.get(title)
    if expected and expected not in kickers:
        kicker_problems.append("Folie %d ohne Kapitelverweis %s" % (number, expected))
    if not expected and kickers:
        kicker_problems.append("Folie %d hat Kapitelverweis ohne Unterfolie" % number)
if kicker_problems:
    problems.append("Kapitelverweise: " + "; ".join(kicker_problems))
else:
    infos.append("Kapitelverweis auf allen Unterfolien, auf keiner anderen Folie")

# 7g Rechtszitate gegen die Normquelle in sec/sources prüfen
SOURCE_DIR = "sec/sources/EU_CRA_Regulation_2024_2847_EN"
if os.path.isdir(SOURCE_DIR):
    valid_articles = set()
    # Absätze je Artikel und die Punkte von Annex I, damit zitierte Absatznummern
    # und zitierte Anhangspunkte gegen die Normquelle prüfbar sind.
    article_paragraphs = {}
    annex_one_letters = set()
    annex_one_numbers = set()
    current_article = None
    annex_part = None
    for name in sorted(os.listdir(SOURCE_DIR)):
        if not name.endswith(".md"):
            continue
        annex_part = None
        for line in open(os.path.join(SOURCE_DIR, name), encoding="utf-8").read().split("\n"):
            heading = re.match(r"^## Article (\d+)\s*$", line)
            if heading:
                current_article = int(heading.group(1))
                valid_articles.add(current_article)
                article_paragraphs.setdefault(current_article, set())
                continue
            part = re.match(r"^## Part (I|II)\s*$", line)
            if part:
                annex_part = "Part " + part.group(1)
                current_article = None
                continue
            paragraph = re.match(r"^### (\d+)\.\s*$", line)
            if paragraph:
                if current_article:
                    article_paragraphs[current_article].add(int(paragraph.group(1)))
                continue
            if line.startswith("## ") or line.startswith("# "):
                current_article = None
                continue
            if not line.strip():
                continue
            item = re.match(r"^\(([a-m])\) ", line)
            if item and annex_part == "Part I":
                annex_one_letters.add(item.group(1))
            number_item = re.match(r"^\((\d+)\) ", line)
            if number_item and annex_part == "Part II":
                annex_one_numbers.add(int(number_item.group(1)))

    cited = {int(n) for n in re.findall(r"Art\.\s*(\d+)", joined)}
    unknown = sorted(n for n in cited if n not in valid_articles)
    if unknown:
        problems.append("Artikel zitiert, die es nicht gibt: "
                        + ", ".join(str(n) for n in unknown))

    # 7g1 Zitierte Absätze müssen in der Normquelle existieren
    for article, paragraph in re.findall(r"Art\.\s*(\d+)\((\d+)\)", joined):
        paragraphs = article_paragraphs.get(int(article), set())
        if paragraphs and int(paragraph) not in paragraphs:
            problems.append("Art. %s hat keinen Absatz (%s)" % (article, paragraph))
    for line_number, line in enumerate(open(MASTER, encoding="utf-8").read().split("\n"), start=1):
        for article, paragraph in re.findall(r"Art\.\s*(\d+)\((\d+)\)", line):
            paragraphs = article_paragraphs.get(int(article), set())
            if paragraphs and int(paragraph) not in paragraphs:
                problems.append("Master Zeile %d: Art. %s hat keinen Absatz (%s)"
                                % (line_number, article, paragraph))

    # 7g2 Zitierte Anhangspunkte müssen in Annex I existieren
    # Geprüft wird je Zelle oder Textabschnitt, der Annex I nennt; die Kennung kann dort
    # auch ohne die Wendung "Teil I" stehen, etwa in "nächster Punkt (2)(k)".
    annex_problems = set()
    for chunk in texts + row_texts:
        if "Annex I" not in chunk:
            continue
        for letter in re.findall(r"\(2\)\(([a-z])\)", chunk):
            if annex_one_letters and letter not in annex_one_letters:
                annex_problems.add("Annex I Teil I hat keinen Punkt (2)(%s)" % letter)
        for number in re.findall(r"Nr\.\s*(\d+)", chunk):
            if annex_one_numbers and int(number) not in annex_one_numbers:
                annex_problems.add("Annex I Teil II hat keine Nummer %s" % number)
    for number, line in enumerate(open(MASTER, encoding="utf-8").read().split("\n"), start=1):
        if "Annex I" not in line:
            continue
        for letter in re.findall(r"\(2\)\(([a-z])\)", line):
            if annex_one_letters and letter not in annex_one_letters:
                annex_problems.add("Master Zeile %d: Annex I Teil I hat keinen Punkt (2)(%s)"
                                   % (number, letter))
        for number_item in re.findall(r"Nr\.\s*(\d+)", line):
            if annex_one_numbers and int(number_item) not in annex_one_numbers:
                annex_problems.add("Master Zeile %d: Annex I Teil II hat keine Nummer %s"
                                   % (number, number_item))
    problems.extend(sorted(annex_problems))

    for chunk in texts + row_texts:
        if re.search(r"Teil I,\s*\d", chunk):
            problems.append("Zitat in erfundener Ziffernform: " + chunk[:60])
        if re.search(r"CE-Kennzeichnung[^.]{0,60}Art\.\s*28", chunk):
            problems.append("CE-Kennzeichnung mit Art. 28 belegt: " + chunk[:60])
        for number in re.findall(r"Teil II,\s*Nr\.\s*(\d+)", chunk):
            if not 1 <= int(number) <= 8:
                problems.append("Teil-II-Nummer außerhalb 1 bis 8: " + chunk[:60])

    master_lines = open(MASTER, encoding="utf-8").read().split("\n")
    for number, line in enumerate(master_lines, start=1):
        if re.search(r"Teil I,\s*\d", line):
            problems.append("Master Zeile %d: Zitat in erfundener Ziffernform" % number)
        if re.search(r"CE-Kennzeichnung[^.]{0,60}Art\.\s*28", line):
            problems.append("Master Zeile %d: CE-Kennzeichnung mit Art. 28 belegt" % number)
        for cited_number in re.findall(r"Art\.\s*(\d+)", line):
            if int(cited_number) not in valid_articles:
                problems.append("Master Zeile %d: Artikel %s existiert nicht"
                                % (number, cited_number))
    infos.append("Rechtszitate: %d Artikelnummern im Deck geprüft, %d Artikel in der Quelle"
                 % (len(cited), len(valid_articles)))

# 7g3 Meldepflichten, Pflichtenliste und der Vorbehalt nach Art. 4(4)
# Diese Regeln prüfen den ganzen Foliensatz, einschließlich Textfeldern außerhalb
# der Platzhalter, weil der Hinweis zur Pflichtenliste in einem Textfeld steht.
all_text = "\n".join(slide_texts)
if "Art. 14" in all_text:
    if "Fristen laufen ab Bereitstellung" in all_text:
        problems.append("Meldepflichten: Fristen an die Bereitstellung geknüpft, nicht an die Kenntnis")
    if "ab Kenntnis" not in all_text:
        problems.append("Meldepflichten: Anknüpfung der Fristen an die Kenntnis fehlt")
if "Pflichten 1 bis 26" in all_text and "Projektliste" not in all_text:
    problems.append("Pflichtenliste 1 bis 26 ohne Hinweis, dass es die Zählung des Dossiers ist")
if "Art. 4(3)" in all_text and "Art. 4(4)" not in all_text:
    problems.append("Art. 4(3) genannt, der Vorbehalt nach Art. 4(4) fehlt")
master_text = open(MASTER, encoding="utf-8").read()
if "Fristen laufen ab Bereitstellung" in master_text:
    problems.append("Master: Fristen an die Bereitstellung geknüpft, nicht an die Kenntnis")
if "Art. 14" in master_text and "ab Kenntnis" not in master_text:
    problems.append("Master: Anknüpfung der Fristen an die Kenntnis fehlt")
infos.append("Fristenanknüpfung, Pflichtenliste und Art. 4(4) geprüft")

# 7g4 Roter Faden: jeder Divider nach dem ersten nimmt den vorigen Abschnitt auf
# Erlaubte Übergangsmarken in der Unterzeile. Der erste Divider eröffnet das Deck und
# braucht keinen Übergang.
THREAD_MARKERS = ("aus ", "anschluss", "belege", "folgt")
thread_problems = []
divider_index = 0
for number, slide in enumerate(slides, start=1):
    if not slide.slide_layout.name.startswith("SUBSECTION"):
        continue
    divider_index += 1
    if divider_index == 1:
        continue
    parts = []
    for shape in slide.shapes:
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
            continue
        if getattr(shape, "has_text_frame", False) and shape.has_text_frame:
            text = shape.text_frame.text.strip()
            if text:
                parts.append(text)
    line = " ".join(parts).lower()
    if not any(marker in line for marker in THREAD_MARKERS):
        thread_problems.append("Divider auf Folie %d ohne Übergang zum vorigen Abschnitt" % number)
if thread_problems:
    problems.append("Roter Faden: " + "; ".join(thread_problems))
else:
    infos.append("Roter Faden: jeder Divider nach dem ersten nimmt den vorigen Abschnitt auf")

# 7g5 Auslöser der Maßnahmen: Maßnahmen nur mit dem Hinweis auf die EU-Bereitstellung
MARKET_TRIGGER = "Umzusetzen mit der Bereitstellung auf dem Unionsmarkt"
if any(measure in all_text for measure in ("P1.1", "P4.1")):
    if MARKET_TRIGGER not in all_text:
        problems.append("Maßnahmen ohne Hinweis, dass sie mit der EU-Bereitstellung umzusetzen sind")
    if "auch unentgeltlich" not in all_text:
        warnings.append("Hinweis zum Auslöser nennt die unentgeltliche Lieferung nicht")
if MARKET_TRIGGER not in master_text:
    problems.append("Master: Hinweis zum Auslöser der Maßnahmen in Abschnitt 8 fehlt")
infos.append("Auslöser der Maßnahmen im Deck und im Master geprüft")
# 7h Keine Vorwärtsverweise
# Verweise dürfen nur auf früher gezeigte Folien zeigen. Das Inhaltsverzeichnis
# (Folie 2) bündelt den Aufbau und ist ebenso ausgenommen wie Zitate aus Normen
# und Leitlinien (EN 303 645 5.1-1, Abschnitt 9.1 der Leitlinien).
ref_slide = {}
for si, slide in enumerate(slides, start=1):
    title = ""
    for shape in slide.shapes:
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
            title = shape.text_frame.text.strip()
    match = re.match(r"^(\d+(?:\.\d+)?)\s+\S", title)
    if match:
        ref_slide[match.group(1)] = si

REF_KEYWORDS = ("siehe", "vgl", "nach ", "aus ", "in ", "auf ", "an ", "abschnitt", "kapitel")
REF_EXEMPT = ("Abschnitt 9.1", "303 645 5.1-1", "303 645 5.4-1")
REF_CANDIDATE = re.compile(r"(?<![A-Za-z0-9.(])([1-7](?:\.\d)?)(?![\d.\-)(])")
ID_FIRST = (
    ("R", re.compile(r"(?<![A-Za-z0-9])R[1-9](?![0-9])"), 14),
    ("P", re.compile(r"(?<![A-Za-z0-9])P[1-3]\.[1-9](?![0-9])"), 15),
    ("P", re.compile(r"(?<![A-Za-z0-9])P[4-5]\.[1-9](?![0-9])"), 16),
)
forward_refs = []
for si, text in enumerate(slide_texts, start=1):
    if si == 2:
        continue
    for match in REF_CANDIDATE.finditer(text):
        window = text[max(0, match.start() - 20):match.end() + 3]
        if any(token in window for token in REF_EXEMPT):
            continue
        # Nummern aus Zitaten wie "Teil II Nr. 7" oder "Art. 7" sind keine Folienverweise
        if re.search(r"(?:Art\.|Nr\.|Abs\.|Annex|Teil)\s*$",
                     text[max(0, match.start() - 8):match.start()]):
            continue
        before = text[max(0, match.start() - 20):match.start()].lower()
        if not any(keyword in before for keyword in REF_KEYWORDS):
            continue
        target = match.group(1)
        target_slide = ref_slide.get(target)
        if target_slide is None:
            forward_refs.append("Folie %d: Verweis auf %s, das keine eigene Folie hat"
                                % (si, target))
        elif target_slide > si:
            forward_refs.append("Folie %d: Vorwärtsverweis auf Folie %d (%s)"
                                % (si, target_slide, target))
    for label, pattern, first in ID_FIRST:
        if si >= first:
            continue
        for match in pattern.finditer(text):
            forward_refs.append("Folie %d: Kennung %s vor der Erklärung ab Folie %d"
                                % (si, match.group(0), first))
if forward_refs:
    problems.append("Vorwärtsverweise: " + "; ".join(sorted(set(forward_refs))))
else:
    infos.append("Keine Vorwärtsverweise, alle Verweise zeigen auf frühere Folien")

# 8 Paketintegrität
if embed_broken:
    problems.append("%d r:embed ohne gültige Relationship" % embed_broken)
infos.append("Bilder und Embeds: %d, defekt: %d" % (embeds, embed_broken))

# 9 Tabellenbindigkeit und Überlaufschätzung
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
print("Maßnahmen-IDs:  ", len(EXPECTED_MEASURES), "geprüft")
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
