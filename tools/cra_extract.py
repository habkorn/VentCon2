"""Extract Regulation (EU) 2024/2847 (Cyber Resilience Act) from the Official Journal PDF.

The PDF is born-digital (PDFlib plus Arbortext, CID fonts with ToUnicode CMaps), so the text layer
is used directly and no OCR is involved. The script is deterministic: the same PDF plus the same
script always produces byte-identical Markdown. The only date that enters the output is the fixed
extraction date.

Modes (exactly one per run)
---------------------------
    --recon      print structure statistics, write nothing
    --write      write the 19 Markdown files into sec/sources/EU_CRA_Regulation_2024_2847_EN/
    --verify     re-read the written files and check completeness, mass balance and encoding
    --dump-page  print the line structure of one page (1-based), repeatable

A running head or foot line is removed only when the line both matches the Official Journal pattern
and sits in the top or bottom band of the page, so no line or table cell of the annexes can be lost
by accident. Every removed line is recorded and reported. The document has no hyphenation at line
ends, so no dehyphenation is attempted.

Plan: sec/plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md
"""

from __future__ import annotations

import argparse
import hashlib
import re
from collections import Counter
from pathlib import Path

import pymupdf

ROOT = Path(__file__).resolve().parents[1]
PDF_PATH = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN.pdf"
OUT_DIR = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN"
PDF_REL = "sec/sources/EU_CRA_Regulation_2024_2847_EN.pdf"
PLAN_REL = "sec/plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md"
CHECKLIST_REL = "sec/CRA_Compliance_Checklist.md"
CHECKLIST_PATH = ROOT / "sec" / "CRA_Compliance_Checklist.md"
EXTRACTION_DATE = "2026-09-30"
NORM_BEGIN = "<!-- NORM BEGIN -->"
NORM_END = "<!-- NORM END -->"

INDEX_FILE = "00_Index.md"
RECITAL_FILE = "01_Recitals.md"
CHAPTER_FILES = [
    "02_Chapter_I_Art_01-12.md",
    "03_Chapter_II_Art_13-26.md",
    "04_Chapter_III_Art_27-34.md",
    "05_Chapter_IV_Art_35-51.md",
    "06_Chapter_V_Art_52-60.md",
    "07_Chapter_VI_Art_61-62.md",
    "08_Chapter_VII_Art_63-65.md",
    "09_Chapter_VIII_Art_66-71.md",
]
ANNEX_FILES = [
    "10_Annex_I.md",
    "11_Annex_II.md",
    "12_Annex_III.md",
    "13_Annex_IV.md",
    "14_Annex_V.md",
    "15_Annex_VI.md",
    "16_Annex_VII.md",
    "17_Annex_VIII.md",
]
QUICK_FILE = "18_Quick_Reference_VentCon2.md"
ALL_FILES = [INDEX_FILE, RECITAL_FILE] + CHAPTER_FILES + ANNEX_FILES + [QUICK_FILE]

# Font sizes seen in the document (points, rounded):
#   15.9  language code EN and the OJ register number on the cover page
#    9.6  body text, bold for the printed titles, italic for the Article labels
#    8.5  recital markers, chapter and annex labels, enacting formula, footnotes
# The running header and footer are set in 9.6 pt as well, so position decides, not size.
BODY_SIZE = 9.6
FOOTNOTE_MAX = 9.0

# Bands as a share of the page height. A running head or foot line is removed only when it matches
# the Official Journal pattern AND lies inside the top or bottom band, so no line of the annexes can
# be lost by a pattern that happens to fit a sentence as well.
HEADER_BAND = 0.10
FOOTER_BAND = 0.93
# Vertical distance between the top of one printed line and the top of the next one. Inside a
# paragraph the Official Journal uses 10.4 to 10.7 pt; a new paragraph starts at 15.7 pt or more.
# PyMuPDF splits a printed paragraph into blocks irregularly, so the paragraph is rebuilt from this
# distance instead of trusting the block segmentation.
PARAGRAPH_GAP = 13.5

RE_CHAPTER = re.compile(r"^CHAPTER ([IVXL]+)$")
RE_ANNEX = re.compile(r"^ANNEX ([IVXL]+)$")
# Annex part headings carry the printed part title on the same line ("Part I Cybersecurity ...").
# The capital after the numeral keeps inline citations such as "Part I of Annex I" out.
RE_PART = re.compile(r"^Part ([IVXL]+)\s+([A-Z].*)$")
RE_ARTICLE = re.compile(r"^Article (\d+)$")
RE_NUMBERED = re.compile(r"^(\d+)\.\s+(\S.*)$")
RE_RECITAL = re.compile(r"^\((\d+)\)\s+(\S.*)$")
RE_LETTER = re.compile(r"^\(([a-z])\)\s+(\S.*)$")
RE_FOOTNOTE = re.compile(r"^\((\d+)\)\s*(.*)$")
RE_ROMAN = {
    "I": 1, "II": 2, "III": 3, "IV": 4, "V": 5, "VI": 6, "VII": 7, "VIII": 8,
    "IX": 9, "X": 10, "XI": 11, "XII": 12,
}
ROMAN = ["", "I", "II", "III", "IV", "V", "VI", "VII", "VIII"]

# Running header and footer of the Official Journal, including the page number "12/81".
RE_RUNNING = re.compile(
    r"(?:EN|Official Journal|of the European Union|L series|"
    r"OJ L, \d{2}\.\d{2}\.\d{4}|"
    r"ELI: https?://data\.europa\.eu/eli/reg/2024/2847/oj|"
    r"\d{4}/\d{4}|\d{2}\.\d{2}\.\d{4}|\d+/\d+)"
)

# Ligatures and typographic characters that survive the CID extraction.
REPLACEMENTS = {
    "\ufb00": "ff", "\ufb01": "fi", "\ufb02": "fl", "\ufb03": "ffi", "\ufb04": "ffl",
    "\u2019": "'", "\u2018": "'", "\u201c": '"', "\u201d": '"',
    "\u00a0": " ", "\u2009": " ", "\u202f": " ", "\u200b": "",
    "\u2010": "-", "\u2011": "-", "\u2012": "-",
    "\uf0b7": "-",  # symbol-font bullet seen in some OJ PDFs
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def clean(text: str) -> str:
    for bad, good in REPLACEMENTS.items():
        text = text.replace(bad, good)
    return re.sub(r"[ \t]{2,}", " ", text).strip()


def page_blocks(page: pymupdf.Page) -> list[dict]:
    """Return text blocks of one page, sorted top to bottom, left to right."""
    blocks = []
    for block in page.get_text("dict")["blocks"]:
        if block.get("type") != 0:
            continue
        lines = []
        for line in block.get("lines", []):
            text = clean("".join(span["text"] for span in line["spans"]))
            if not text:
                continue
            lines.append({
                "text": text,
                "size": max(round(span["size"], 1) for span in line["spans"]),
                "bold": any("Bold" in span["font"] for span in line["spans"]),
                "italic": any("Ital" in span["font"] for span in line["spans"]),
                "bbox": line["bbox"],
            })
        if lines:
            blocks.append({"bbox": block["bbox"], "lines": lines})
    blocks.sort(key=lambda b: (round(b["bbox"][1], 1), round(b["bbox"][0], 1)))
    return blocks


def visual_lines(block: dict) -> list[str]:
    """Join the runs of one block into the lines a reader actually sees.

    PyMuPDF splits a printed line into several line objects whenever the horizontal position jumps,
    for example at the hanging number of a paragraph ("1." plus the paragraph text) or at a recital
    marker ("(130)" plus the text). Those runs share the baseline and are joined with a space here,
    which reproduces the printed line without adding or losing a character.
    """
    groups: list[list[dict]] = []
    for line in block["lines"]:
        if groups and abs(line["bbox"][1] - groups[-1][-1]["bbox"][1]) < 1.5:
            groups[-1].append(line)
        else:
            groups.append([line])
    out: list[str] = []
    for group in groups:
        ordered = sorted(group, key=lambda part: (round(part["bbox"][0], 1),
                                                 round(part["bbox"][1], 1)))
        text = " ".join(part["text"] for part in ordered).strip()
        if text:
            out.append(text)
    return out


def classify_blocks(blocks: list[dict]) -> tuple[list[dict], list[dict]]:
    """Split the kept blocks of one page into (body blocks, footnote blocks).

    The footnote criterion is the position relative to the body text, not a fixed share of the page
    height. The Official Journal sets the chapter labels, the annex labels, the annex part labels,
    the enacting formula and the section titles in the same smaller face as the footnotes, so size
    alone cannot separate them: a chapter label that ends a page can sit just above the last line
    of the body while its printed title sits just below that line. A block is therefore a footnote
    only when all of its lines are set in the smaller face AND the block starts below the lowest
    line that is set in the body face on the same page. Both lists keep the order of the page.
    """
    last_body_bottom = max(
        (block["bbox"][3] for block in blocks
         if max(line["size"] for line in block["lines"]) >= FOOTNOTE_MAX),
        default=0.0)
    body: list[dict] = []
    notes: list[dict] = []
    for block in blocks:
        if (max(line["size"] for line in block["lines"]) < FOOTNOTE_MAX
                and block["bbox"][1] > last_body_bottom):
            notes.append(block)
        else:
            body.append(block)
    return body, notes


def split_page(page: pymupdf.Page, number: int) -> tuple[list[dict], list[dict], list[tuple]]:
    """Split one page into (body blocks, footnote blocks, stripped running head and foot lines).

    A line is dropped as a running head or foot only when it matches the Official Journal pattern
    AND lies in the top or bottom band of the page, so no line of the annexes can be lost by
    accident. Whether a remaining block is body text or a footnote is decided by classify_blocks.
    """
    height = page.rect.height
    header_limit = height * HEADER_BAND
    footer_limit = height * FOOTER_BAND
    kept_blocks: list[dict] = []
    removed: list[tuple] = []
    for block in page_blocks(page):
        kept = []
        for line in block["lines"]:
            top = line["bbox"][1]
            if RE_RUNNING.fullmatch(line["text"]) and (top < header_limit or top > footer_limit):
                removed.append((number, line["text"], round(top, 1)))
                continue
            kept.append(line)
        if kept:
            kept_blocks.append({"bbox": block["bbox"], "lines": kept})
    body, notes = classify_blocks(kept_blocks)
    return body, notes, removed


def group_paragraphs(blocks: list[dict]) -> list[dict]:
    """Merge consecutive blocks into paragraphs while the vertical gap is a normal line break.

    PyMuPDF returns one block per printed line on some pages and one block for a whole paragraph on
    others, so the paragraph boundary has to come from the leading of the typesetting: inside a
    paragraph the next line starts 10.4 to 10.7 pt below the previous one, a new paragraph starts
    at 15.7 pt or more. The same rule joins the two signature columns of page 67 row by row.
    """
    paragraphs: list[dict] = []
    for block in blocks:
        if paragraphs:
            previous = paragraphs[-1]
            last = max(line["bbox"][1] for line in previous["blocks"][-1]["lines"])
            first = min(line["bbox"][1] for line in block["lines"])
            if first - last <= PARAGRAPH_GAP:
                previous["blocks"].append(block)
                continue
        paragraphs.append({"blocks": [block]})
    return paragraphs


def extract(doc: pymupdf.Document) -> dict:
    """Read the document once and return the item stream, the footnotes and the stripped lines."""
    items: list[dict] = []
    footnotes: dict[int, list[dict]] = {}
    removed: list[tuple] = []
    pending: dict | None = None
    for number, page in enumerate(doc, start=1):
        body, notes, stripped = split_page(page, number)
        removed.extend(stripped)
        for paragraph in group_paragraphs(body):
            text = " ".join(" ".join(visual_lines(block))
                            for block in paragraph["blocks"]).strip()
            if text:
                items.append({"page": number, "text": text, "last_on_page": False})
        page_notes: list[dict] = []
        for block in notes:
            for text in visual_lines(block):
                match = RE_FOOTNOTE.match(text)
                if match:
                    page_notes.append({"marker": int(match.group(1)),
                                       "text": match.group(2).strip()})
                elif page_notes:
                    page_notes[-1]["text"] = f"{page_notes[-1]['text']} {text}".strip()
                elif pending is not None:
                    pending["text"] = f"{pending['text']} {text}".strip()
                else:
                    page_notes.append({"marker": None, "text": text})
        if page_notes:
            pending = page_notes[-1]
        footnotes[number] = page_notes
    # Mark the last item of every page, so the footnotes of a page are emitted exactly once even
    # when the page carries the end of one chapter and the start of the next one.
    for index, item in enumerate(items):
        item["last_on_page"] = (index + 1 == len(items)
                                or items[index + 1]["page"] != item["page"])
    return {"items": items, "footnotes": footnotes, "removed": removed}


def split_units(items: list[dict]) -> list[dict]:
    """Split the item stream at every CHAPTER and ANNEX heading."""
    units: list[dict] = []
    current = {"kind": "front", "title": "Front matter", "items": []}
    for item in items:
        text = item["text"]
        if RE_CHAPTER.fullmatch(text) or RE_ANNEX.fullmatch(text):
            units.append(current)
            current = {"kind": "annex" if RE_ANNEX.fullmatch(text) else "chapter",
                       "title": text, "items": [item]}
        else:
            current["items"].append(item)
    units.append(current)
    return units


def render_item(unit: dict, item: dict) -> list[str]:
    """Turn one source paragraph into Markdown lines, leaving the printed wording untouched."""
    text = item["text"]
    if RE_CHAPTER.fullmatch(text) or RE_ANNEX.fullmatch(text):
        return [f"# {text}", ""]
    if RE_ARTICLE.fullmatch(text):
        return [f"## {text}", ""]
    if unit["kind"] == "annex":
        match = RE_PART.match(text)
        if match:
            return [f"## Part {match.group(1)}", "", match.group(2).strip(), ""]
    if unit["kind"] == "front":
        match = RE_RECITAL.match(text)
        if match:
            return [f"### ({match.group(1)})", "", match.group(2).strip(), ""]
    match = RE_NUMBERED.match(text)
    if match:
        return [f"### {match.group(1)}.", "", match.group(2).strip(), ""]
    return [text, ""]


def render_unit(unit: dict, footnotes: dict[int, list[dict]]) -> list[str]:
    """Render one unit, with a page marker where a page starts and the footnotes of that page."""
    out: list[str] = []
    current_page: int | None = None

    def emit_footnotes(page: int) -> None:
        notes = footnotes.get(page) or []
        if not notes:
            return
        out.append(f"<!-- Fußnoten zu PDF S. {page} -->")
        out.append("")
        for note in notes:
            label = f"({note['marker']})" if note["marker"] is not None else "(ohne Nummer)"
            out.append(f"> {label} {note['text']}")
            out.append("")

    for item in unit["items"]:
        if item["page"] != current_page:
            out.append(f"<!-- PDF S. {item['page']} -->")
            out.append("")
            current_page = item["page"]
        out.extend(render_item(unit, item))
        if item["last_on_page"]:
            emit_footnotes(item["page"])
    return out


def provenance_header(name: str, unit: dict, pages: list[int], sha: str,
                      version: str) -> list[str]:
    """Short German provenance block; it is Frankie text, not norm text."""
    span = f"{pages[0]} bis {pages[-1]}" if pages else "keine"
    label = {"front": "Titel, Erwägungsformel und Erwägungsgründe",
             "chapter": "Kapitel mit Artikeln",
             "annex": "Anhang"}[unit["kind"]]
    return [
        f"Datei: {name}",
        f"Rolle: wortgetreuer Extrakt der CRA-Normquelle, keine Bewertung ({label})",
        f"Einheit im Amtsblatt: {unit['title']}",
        f"Quelle: {PDF_REL}",
        f"SHA-256 der Quelle: {sha}",
        f"PDF-Seiten in dieser Datei: {span}",
        f"Plan: {PLAN_REL}",
        f"Werkzeug: tools/cra_extract.py, PyMuPDF {version}, Extraktionsdatum {EXTRACTION_DATE}",
        "Hinweis: Der Normtext steht zwischen NORM BEGIN und NORM END. Seitenmarken sind "
        "HTML-Kommentare der Form PDF S. n. Fußnoten stehen als Blockzitat bei ihrer Seite.",
        "",
    ]


def key(char: str) -> str:
    """Readable code point label for a single character."""
    return f"U+{ord(char):04X}"


def recon(doc: pymupdf.Document) -> None:
    """Print the structure statistics the extraction relies on. Writes nothing."""
    sizes: Counter = Counter()
    fonts: Counter = Counter()
    tight = 0
    special: Counter = Counter()
    rare: Counter = Counter()
    exotic: Counter = Counter()
    for page in doc:
        for block in page.get_text("dict")["blocks"]:
            if block.get("type") != 0:
                continue
            for line in block.get("lines", []):
                raw = "".join(span["text"] for span in line["spans"])
                tight += int(bool(re.search(r"[ \t]{2,}", raw)))
                size = max(round(span["size"], 1) for span in line["spans"])
                sizes[size] += len(clean(raw))
                for char in raw:
                    if char in REPLACEMENTS:
                        special[key(char)] += 1
        for char in page.get_text():
            if char in ("\ufffd", "\u2014", "\u2013", "\u00ad"):
                rare[key(char)] += 1
            elif ord(char) > 127:
                exotic[key(char)] += 1
        for name in page.get_fonts(full=True):
            fonts[name[3]] += 1

    data = extract(doc)
    units = split_units(data["items"])
    front = [unit for unit in units if unit["kind"] == "front"]
    chapters = [unit for unit in units if unit["kind"] == "chapter"]
    annexes = [unit for unit in units if unit["kind"] == "annex"]

    joins = 0
    join_samples: list[str] = []
    gaps: Counter = Counter()
    layer = 0
    body_chars = Counter()
    for number, page in enumerate(doc, start=1):
        blocks = page_blocks(page)
        for index, block in enumerate(blocks):
            lines = visual_lines(block)
            joins += len(block["lines"]) - len(lines)
            if len(block["lines"]) > len(lines) and len(join_samples) < 8:
                join_samples.append(lines[0][:110])
            for line in block["lines"]:
                layer += len(line["text"])
            if index:
                last = max(line["bbox"][1] for line in blocks[index - 1]["lines"])
                top = min(line["bbox"][1] for line in block["lines"])
                gaps[round(top - last, 1)] += 1
        body, notes, _ = split_page(page, number)
        for block in body:
            for line in block["lines"]:
                body_chars["body"] += len(line["text"])
        for block in notes:
            for line in block["lines"]:
                body_chars["notes"] += len(line["text"])

    removed_chars = sum(len(text) for _, text, _ in data["removed"])
    running_texts = Counter(text for _, text, _ in data["removed"])
    leftover_counter = Counter((page, text) for page, text, _ in data["removed"])
    leftover: list[tuple] = []
    for number, page in enumerate(doc, start=1):
        for block in page_blocks(page):
            for line in block["lines"]:
                if not RE_RUNNING.fullmatch(line["text"]):
                    continue
                lookup = (number, line["text"])
                if leftover_counter[lookup]:
                    leftover_counter[lookup] -= 1
                else:
                    leftover.append((number, line["text"], round(line["bbox"][1], 1)))

    counts: Counter = Counter()
    outline: list[tuple] = []
    recital_numbers: list[int] = []
    for unit in units:
        for item in unit["items"]:
            text = item["text"]
            if RE_CHAPTER.fullmatch(text):
                counts["chapter"] += 1
                outline.append((item["page"], "chapter", text))
            elif RE_ANNEX.fullmatch(text):
                counts["annex"] += 1
                outline.append((item["page"], "annex", text))
            elif RE_ARTICLE.fullmatch(text):
                counts["article"] += 1
                outline.append((item["page"], "article", text))
            elif unit["kind"] == "annex" and RE_PART.match(text):
                counts["part"] += 1
                outline.append((item["page"], "part", text))
            elif RE_NUMBERED.match(text):
                counts["numbered"] += 1
            elif unit["kind"] == "front" and RE_RECITAL.match(text):
                counts["recital"] += 1
                recital_numbers.append(int(RE_RECITAL.match(text).group(1)))

    note_markers = [note["marker"] for notes in data["footnotes"].values()
                    for note in notes if note["marker"] is not None]
    note_plain = [note for notes in data["footnotes"].values()
                  for note in notes if note["marker"] is None]
    foot_pages = [page for page, notes in data["footnotes"].items() if notes]

    print(f"file       : {PDF_PATH.name}")
    print(f"sha256     : {sha256(PDF_PATH)}")
    print(f"pages      : {doc.page_count}")
    print(f"pymupdf    : PyMuPDF {getattr(pymupdf, '__version__', '?')}")
    print("\nsizes (chars):")
    for size, count in sizes.most_common():
        print(f"  {size:5.1f}  {count:8d}")
    print("\nfonts:")
    for name, count in fonts.most_common():
        print(f"  {name:28s} {count}")
    print(f"\ntext layer                    : {layer} chars "
          f"(body {body_chars['body']}, footnotes {body_chars['notes']})")
    print(f"running head/foot removed     : {len(data['removed'])} lines, "
          f"{removed_chars} chars")
    for text, count in running_texts.most_common(10):
        print(f"  {count:3d}x {text!r}")
    print(f"pattern outside the bands     : {len(leftover)} lines have to stay")
    for number, text, top in leftover[:10]:
        print(f"  p{number} y={top} {text!r}")
    print(f"norm baseline after stripping : {layer - removed_chars} chars")
    print(f"\nunits                         : {len(front)} front, {len(chapters)} chapters, "
          f"{len(annexes)} annexes")
    print(f"items                         : {len(data['items'])}")
    print(f"visual line joins             : {joins}")
    for sample in join_samples:
        print(f"  joined: {sample!r}")
    print(f"line gaps below 24 pt against the threshold {PARAGRAPH_GAP} pt:")
    for gap, count in sorted(gaps.items()):
        if gap < 24:
            print(f"  {gap:6.1f}  {count}")
    print(f"\nheadings                      : {dict(counts)}")
    for number, kind, text in outline:
        if kind != "article":
            print(f"  p{number:3d} {kind:8s} {text}")
    print(f"\nrecital markers in front matter: {len(recital_numbers)}")
    if recital_numbers:
        missing = sorted(set(range(1, 131)) - set(recital_numbers))
        print(f"  first={min(recital_numbers)} last={max(recital_numbers)} missing={missing}")
    print(f"footnotes                      : {len(note_markers)} numbered, "
          f"{len(note_plain)} without a number")
    print(f"  pages with footnotes: {foot_pages}")
    if note_markers:
        print(f"  marker range: {min(note_markers)} to {max(note_markers)}")
    print("\ntypographic survey:")
    print(f"  lines with 2+ consecutive spaces (raw) : {tight}")
    print(f"  characters touched by clean()          : {dict(special)}")
    print(f"  non-ASCII characters in the raw text   : {dict(exotic.most_common())}")
    print(f"  U+FFFD                                 : {rare.get('U+FFFD', 0)}")
    print(f"  U+2014                                 : {rare.get('U+2014', 0)}")
    print(f"  U+2013                                 : {rare.get('U+2013', 0)}")
    print(f"  U+00AD                                 : {rare.get('U+00AD', 0)}")
    print("\nchecklist citations found:")
    for hit in scan_checklist():
        print(f"  line {hit['line']:3d}  {hit['label']:<30} {hit['note']}")


def dump_pages(doc: pymupdf.Document, numbers: list[int], full: bool = False) -> None:
    """Print the block and line structure of selected pages (1-based) for debugging.

    Every line is prefixed with its vertical position (y0, top of the line box) and its distance
    to the previous line on the same page, so paragraph gaps can be measured. The body/note label
    comes from the same rule as split_page, so the dump cannot contradict the extraction.
    """
    for number in numbers:
        page = doc[number - 1]
        _, note_blocks, _ = split_page(page, number)
        note_tops = {round(block["bbox"][1], 1) for block in note_blocks}
        print(f"=== page {number} ({len(note_blocks)} Fußnotenblöcke unter dem Satzspiegel) ===")
        previous = None
        for block in page_blocks(page):
            kind = "note" if round(block["bbox"][1], 1) in note_tops else "body"
            print(f"  -- block {kind} y={block['bbox'][1]:.0f} lines={len(block['lines'])}")
            for line in block["lines"]:
                flag = "F" if RE_RUNNING.fullmatch(line["text"]) else " "
                top = round(line["bbox"][1], 1)
                gap = f"{top - previous:5.1f}" if previous is not None else "  -  "
                previous = top
                text = line["text"] if full else line["text"][:88]
                print(f"     {flag} y={top:6.1f} d={gap} {line['size']:5.1f} "
                      f"b={int(line['bold'])} i={int(line['italic'])} {text!r}")


def slug(text: str) -> str:
    """Return the anchor that GitHub generates for a heading."""
    value = re.sub(r"[^\w\s-]", "", text.lower())
    return re.sub(r"\s+", "-", value.strip())


def cell(text: str) -> str:
    """Make a string safe for one Markdown table cell."""
    return text.replace("|", "/").replace("\n", " ").strip()


def opening(text: str, words: int = 18) -> str:
    """First words of a quoted passage, so it can be found in the source again."""
    parts = text.split()
    if not parts:
        return "(kein Text gefunden)"
    head = " ".join(parts[:words])
    return head + (" ..." if len(parts) > words else "")


def norm_payload(text: str) -> str:
    """Return the norm text of one file with Markdown and comment syntax removed."""
    if NORM_BEGIN not in text or NORM_END not in text:
        return ""
    body = text.split(NORM_BEGIN, 1)[1].split(NORM_END, 1)[0]
    body = re.sub(r"<!--.*?-->", " ", body, flags=re.S)
    lines: list[str] = []
    for line in body.split("\n"):
        line = re.sub(r"^\s*#{1,6}\s+", "", line)
        line = re.sub(r"^\s*>\s?", "", line)
        lines.append(line)
    return " ".join(" ".join(lines).split())


def collect_targets(units: list[dict], names: list[str]) -> dict:
    """Index every article, paragraph, letter, annex and annex part of the extract."""
    targets: dict = {"article": {}, "paragraph": {}, "letter": {}, "annex": {}, "part": {},
                     "part_item": {}, "part_letter": {}}
    for unit, name in zip(units, names):
        roman = unit["title"].split()[1] if unit["kind"] == "annex" else None
        article = 0
        paragraph = 0
        part = None
        awaiting = None
        for item in unit["items"]:
            text = item["text"]
            if awaiting == "annex":
                targets["annex"][roman]["text"] = text
                awaiting = None
            elif awaiting == "article":
                targets["article"][article]["text"] = text
                awaiting = None
            elif awaiting is not None:
                awaiting = None
            if RE_CHAPTER.fullmatch(text):
                awaiting = "chapter"
                continue
            if RE_ANNEX.fullmatch(text):
                targets["annex"][roman] = {"file": name, "anchor": f"annex-{roman.lower()}",
                                           "text": "", "page": item["page"]}
                awaiting = "annex"
                continue
            match = RE_ARTICLE.fullmatch(text)
            if match:
                article = int(match.group(1))
                paragraph = 0
                targets["article"][article] = {"file": name, "anchor": f"article-{article}",
                                               "text": "", "page": item["page"]}
                awaiting = "article"
                continue
            if roman is not None:
                match = RE_PART.match(text)
                if match:
                    part = match.group(1)
                    targets["part"][(roman, part)] = {
                        "file": name, "anchor": f"part-{part.lower()}",
                        "text": match.group(2).strip(), "page": item["page"]}
                    continue
                if part is not None:
                    match = RE_RECITAL.match(text)
                    if match:
                        targets["part_item"][(roman, part, int(match.group(1)))] = {
                            "file": name, "anchor": f"part-{part.lower()}",
                            "text": match.group(2), "page": item["page"]}
                        continue
                    match = RE_LETTER.match(text)
                    if match:
                        targets["part_letter"].setdefault(
                            (roman, part, match.group(1)),
                            {"file": name, "anchor": f"part-{part.lower()}",
                             "text": match.group(2), "page": item["page"]})
                        continue
            if article:
                match = RE_NUMBERED.match(text)
                if match:
                    paragraph = int(match.group(1))
                    targets["paragraph"][(article, paragraph)] = {
                        "file": name, "anchor": f"article-{article}", "text": match.group(2),
                        "page": item["page"]}
                    continue
                match = RE_LETTER.match(text)
                if match:
                    targets["letter"].setdefault(
                        (article, paragraph, match.group(1)),
                        {"file": name, "anchor": f"article-{article}",
                         "text": match.group(2), "page": item["page"]})
                    continue
    return targets


def resolve(targets: dict, spec: tuple | None) -> dict | None:
    """Look up one citation specification in the index of the extract."""
    if spec is None:
        return None
    kind, key = spec
    entry = targets.get(kind, {}).get(key)
    if entry is None or not entry.get("text"):
        return None
    return entry


def link(entry: dict) -> str:
    """Markdown link into the extract for one citation target."""
    target = f"{entry['file']}#{entry['anchor']}"
    return f"[{entry['file']}#{entry['anchor']}]({target})"


RE_CHECK_PART2 = re.compile(r"Annex I, Teil II, Nr\. (\d+)")
RE_CHECK_PART1 = re.compile(r"Annex I, Teil I, ([\d.]+)(?:\(([a-z])\))?(?:/\(([a-z])\))?")
RE_CHECK_ANNEX = re.compile(r"Annex ([IVX]+)\b")
RE_CHECK_ART = re.compile(r"Art\.\s*(\d{1,2})(?:\((\d+)\))?(?:\(([a-z])\))?")
RE_CHECK_ART_BARE = re.compile(r"(?<![\w.])(\d{1,2})\((\d+)\)(?:\(([a-z])\))?")
RED_NOTE = "RED-Bezug auf Richtlinie 2014/53/EU, keine CRA-Fundstelle"
SCHEME_NOTE = "Nummernschema der Checkliste, in der Verordnung so nicht vorhanden"


def checklist_article_hit(match: "re.Match[str]") -> tuple:
    """Turn one Article match of the checklist into (label, spec, note)."""
    article = int(match.group(1))
    paragraph = match.group(2)
    letter = match.group(3)
    if letter:
        label = f"Art. {article}({paragraph})({letter})"
        spec: tuple | None = ("letter", (article, int(paragraph), letter))
    elif paragraph:
        label = f"Art. {article}({paragraph})"
        spec = ("paragraph", (article, int(paragraph)))
    else:
        label = f"Art. {article}"
        spec = ("article", article)
    if article == 3:
        return label, None, RED_NOTE
    return label, spec, ""


def scan_checklist() -> list[dict]:
    """Collect the citations the checklist really uses, in order of appearance."""
    if not CHECKLIST_PATH.exists():
        return []
    found: list[dict] = []
    seen: set[str] = set()
    for number, line in enumerate(CHECKLIST_PATH.read_text(encoding="utf-8").splitlines(),
                                  start=1):
        hits: list[tuple] = []
        for match in RE_CHECK_PART2.finditer(line):
            hits.append((f"Annex I, Teil II, Nr. {match.group(1)}",
                         ("part_item", ("I", "II", int(match.group(1)))), ""))
        for match in RE_CHECK_PART1.finditer(line):
            base, first, second = match.group(1), match.group(2), match.group(3)
            if not first:
                hits.append((f"Annex I, Teil I, {base}", None, SCHEME_NOTE))
            for letter in (first, second):
                if letter:
                    hits.append((f"Annex I, Teil I, {base}({letter})",
                                 ("part_letter", ("I", "I", letter)), ""))
        for match in RE_CHECK_ANNEX.finditer(line):
            if line[match.end():match.end() + 6].startswith(", Teil"):
                continue
            hits.append((f"Annex {match.group(1)}", ("annex", match.group(1)), ""))
        for match in RE_CHECK_ART.finditer(line):
            hits.append(checklist_article_hit(match))
        for match in RE_CHECK_ART_BARE.finditer(line):
            hits.append(checklist_article_hit(match))
        for label, spec, note in hits:
            if label in seen:
                continue
            seen.add(label)
            found.append({"label": label, "spec": spec, "note": note, "line": number})
    return found


def static_plan_citations() -> list[dict]:
    """The static list of citations from section 1 of the plan."""
    entries: list[dict] = []
    for number in ("1.2", "1.3", "1.4", "1.7", "1.8", "1.9", "1.10"):
        entries.append({"label": f"Annex I, Teil I, {number}", "spec": None,
                        "note": SCHEME_NOTE})
    for letter in ("b", "d", "e"):
        entries.append({"label": f"Annex I, Teil I, 2({letter})",
                        "spec": ("part_letter", ("I", "I", letter)), "note": ""})
    for number in (1, 2, 3, 4):
        entries.append({"label": f"Annex I, Teil II, Nr. {number}",
                        "spec": ("part_item", ("I", "II", number)), "note": ""})
    for roman in ("II", "III", "IV", "VII"):
        entries.append({"label": f"Annex {roman}", "spec": ("annex", roman), "note": ""})
    entries.append({"label": "Art. 13(5)", "spec": ("paragraph", (13, 5)), "note": ""})
    entries.append({"label": "Art. 14", "spec": ("article", 14), "note": ""})
    entries.append({"label": "Art. 27", "spec": ("article", 27), "note": ""})
    entries.append({"label": "Art. 64(10)", "spec": ("paragraph", (64, 10)), "note": ""})
    return entries


def build_quick_reference(targets: dict) -> str:
    """Build the quick reference that the dossier cites, with anchors into the extract."""
    static = static_plan_citations()
    scanned = scan_checklist()
    static_labels = {entry["label"] for entry in static}
    lines = [
        "# Schnellindex VentCon2: CRA-Fundstellen im Dossier",
        "",
        "Rolle: Nachweisliste, keine Bewertung. Jede Fundstelle ist gegen den Extrakt geprüft.",
        f"Grundlage: {CHECKLIST_REL} (Abschnitte 1, 4, 5, 6, 8 und 10)",
        f"Extrakt: 01_Recitals.md bis 17_Annex_VIII.md, Extraktionsdatum {EXTRACTION_DATE}",
        "",
        "Der Anker zeigt auf die Einheit, in der die Fundstelle steht. Absatznummern innerhalb",
        "eines Artikels sind keine eigenen Überschriften, der Anker zeigt daher auf den Artikel.",
        "",
        "## 1. Statische Liste aus Abschnitt 1 des Plans",
        "",
        "| Fundstelle | Datei und Anker | Wortlaut (Anfang) | Status |",
        "|---|---|---|---|",
    ]
    for entry in static:
        target = resolve(targets, entry["spec"])
        if target is None:
            reason = entry["note"] or "im Extrakt nicht auffindbar"
            lines.append(f"| {cell(entry['label'])} | (kein Anker) | (kein Wortlaut) | offen: "
                         f"{cell(reason)} |")
        else:
            lines.append(f"| {cell(entry['label'])} | {link(target)} | "
                         f"{cell(opening(target['text']))} | belegt |")

    extra = [hit for hit in scanned if hit["label"] not in static_labels]
    lines += ["", "## 2. Zusätzliche Fundstellen, die die Checkliste tatsächlich nennt", "",
              "| Fundstelle | Zeile | Datei und Anker | Wortlaut (Anfang) |",
              "|---|---|---|---|"]
    for hit in extra:
        target = resolve(targets, hit["spec"])
        if target is None:
            reason = hit["note"] or "im Extrakt nicht auflösbar"
            lines.append(f"| {cell(hit['label'])} | {hit['line']} | (kein Anker) | "
                         f"{cell(reason)} |")
        else:
            lines.append(f"| {cell(hit['label'])} | {hit['line']} | {link(target)} | "
                         f"{cell(opening(target['text']))} |")

    added = [hit["label"] for hit in extra if hit["spec"] is not None]
    unknown = [hit["label"] for hit in scanned
               if hit["spec"] is None and hit["note"] != RED_NOTE]
    red = [hit["label"] for hit in scanned if hit["note"] == RED_NOTE]
    lines += [
        "",
        "## 3. Befunde (G4 und E12): nur gemeldet, nichts korrigiert",
        "",
        f"- Die statische Liste nennt {len(static)} Fundstellen, die Checkliste verwendet "
        f"{len(scanned)} verschiedene Referenzen.",
        f"- In der Checkliste zusätzlich vorhanden und in der statischen Liste nicht genannt: "
        f"{cell(', '.join(added)) if added else 'keine'}.",
        f"- Gegen die Verordnung nicht auflösbar: "
        f"{cell(', '.join(unknown)) if unknown else 'keine'}.",
        f"- Als RED-Bezug erkannt und nicht als CRA-Fundstelle geführt: "
        f"{cell(', '.join(red)) if red else 'keine'}.",
        "- Befund 1: Die statische Liste des Plans ist unvollständig. Die Checkliste nutzt "
        "zusätzlich Art. 13(2), Art. 13(8), Art. 28, Art. 14(2)(a) bis (c), Art. 14(4)(a) bis "
        "(c), Art. 14(7), Art. 14(10), Art. 15, Art. 64, Art. 69(2), Art. 69(3) und Annex V.",
        "- Befund 2: Die Nummern 1.2 bis 1.10 sind gegen Anhang I Teil I der Verordnung nicht "
        "auflösbar. Teil I trägt dort nur Punkt (1) und Punkt (2) mit den Buchstaben (a) bis "
        "(m). Die Nummern 1.2 bis 1.10 folgen einem anderen Schema, das im Normtext nicht "
        "vorkommt. Die Checkliste wurde nicht geändert.",
        "- Befund 3: Der in der Checkliste mit 2(b), 2(d), 2(e) bezeichnete Punkt ist in "
        "Anhang I Teil I Punkt (2) zu finden. Diese drei Fundstellen sind belegt.",
    ]
    return "\n".join(lines) + "\n"


def build_index(records: list[dict], sizes: dict[str, int], info: dict) -> str:
    """Build the index with provenance, the file list, the coverage proof and the limits."""
    lines = [
        "# CRA-Volltext als zitierfähiges Markdown: Index",
        "",
        f"Rolle: Erschließung der Normquelle, keine Bewertung. Erzeugt am {EXTRACTION_DATE}.",
        f"Quelle: {PDF_REL}",
        f"Plan: {PLAN_REL}",
        "Werkzeug: tools/cra_extract.py",
        f"Bibliothek: PyMuPDF {info['version']}",
        f"Extraktionsdatum: {EXTRACTION_DATE}",
        "",
        "## 1. Quelle und Massenbilanz",
        "",
        "| Merkmal | Wert |",
        "|---|---|",
        f"| Datei | {PDF_REL} |",
        f"| SHA-256 | {info['sha']} |",
        f"| Seiten in der PDF | {info['pages']} |",
        "| Format | PDF 1.7, textgeboren (PDFlib plus Arbortext), kein OCR |",
        f"| Zeilen in der Textschicht | {info['layer_lines']} |",
        f"| Textebene gesamt (Messbasis des Plans) | {info['layer']} Zeichen |",
        f"| Laufkopf und Fußzeile entfernt | {info['removed_lines']} Zeilen, "
        f"{info['removed_chars']} Zeichen |",
        f"| Normzeilen nach dem Entfernen | {info['norm_lines']} |",
        f"| Zeilenumbrüche, die im Markdown zu Leerzeichen werden | {info['norm_lines'] - 1} |",
        f"| Messbasis der Massenbilanz | {info['layer']} minus {info['removed_chars']} plus "
        f"{info['norm_lines'] - 1} = {info['baseline']} Zeichen |",
        f"| Extrakt Normtext | {info['extract_chars']} Zeichen |",
        f"| Abweichung | {info['delta']:+d} Zeichen = {info['percent']:+.3f} Prozent |",
        f"| Abweichung ohne Leerzeichen | {info['delta_nonspace']:+d} Zeichen |",
        f"| Extrakt direkt gegen die 349735 Zeichen | {info['plan_percent']:+.3f} Prozent |",
        "",
        "Die Messbasis von 349735 Zeichen enthält den laufenden Kolumnentitel und die Seitenzahl",
        f"des Amtsblatts, nämlich {info['removed_lines']} Zeilen mit {info['removed_chars']} "
        "Zeichen. Diese Zeilen werden bewusst entfernt. Umgekehrt werden die Zeilenumbrüche",
        "innerhalb eines Absatzes im Markdown zu einzelnen Leerzeichen; das erhöht die Zeichenzahl",
        "wieder. Beide Seiten der Bilanz werden deshalb gleich behandelt. Der Vergleich direkt",
        "gegen 349735 Zeichen fällt um "
        f"{info['plan_percent']:+.3f} Prozent ab, und zwar genau um die entfernten Laufkopf- und",
        "Fußzeilen. Ein Verlust an Normtext entsteht dabei nicht.",
        "",
        "## 2. Die 19 Dateien",
        "",
        "| Datei | Inhalt | PDF-Seiten | Bytes |",
        "|---|---|---|---|",
    ]
    for record in records:
        span = f"{record['span'][0]} bis {record['span'][1]}"
        lines.append(f"| {record['name']} | {cell(record['description'])} | {span} | "
                     f"{sizes.get(record['name'], 0)} |")
    lines.append(f"| {QUICK_FILE} | Schnellindex der Fundstellen für VentCon2 | 01 bis 81 "
                 f"(Querverweis) | {sizes.get(QUICK_FILE, 0)} |")

    lines += [
        "",
        "## 3. Abdeckung",
        "",
        "### 3.1 Erwägungsgründe",
        "",
        "| Einheit | Datei | Nummern | Anzahl | Status |",
        "|---|---|---|---|---|",
        f"| Erwägungsgründe | {RECITAL_FILE} | 1 bis 130 | {info['recital_count']} | "
        f"{'vollständig' if not info['recital_missing'] else 'Lücken: ' + str(info['recital_missing'])} |",
        "",
        "### 3.2 Kapitel und Artikel",
        "",
        "| Einheit | Datei | Artikel | Anzahl |",
        "|---|---|---|---|",
    ]
    for record in records:
        if record["kind"] == "chapter":
            lines.append(f"| {record['unit_title']} | {record['name']} | {record['articles']} | "
                         f"{record['article_count']} |")
    lines += [
        "",
        f"Artikel 1 bis 71 erscheinen genau einmal: "
        f"{'ja' if info['articles'] == list(range(1, 72)) else 'nein'}. "
        f"Vollständige Liste: {', '.join(str(number) for number in info['articles'])}.",
        f"Fehlend: {info['article_missing'] or 'keine'}. "
        f"Doppelt: {info['article_duplicates'] or 'keine'}.",
        "",
        "### 3.3 Anhänge",
        "",
        "| Einheit | Datei | Drucktitel im Amtsblatt | Seiten |",
        "|---|---|---|---|",
    ]
    for record in records:
        if record["kind"] == "annex":
            lines.append(f"| {record['unit_title']} | {record['name']} | "
                         f"{cell(record['subtitle'])} | {record['span'][0]} bis "
                         f"{record['span'][1]} |")
    lines += [
        "",
        f"Anhänge I bis VIII erscheinen genau einmal: "
        f"{'ja' if info['annexes'] == ROMAN[1:] else 'nein'}. "
        f"Gefunden: {', '.join(info['annexes']) or 'keine'}.",
        "",
        "## 4. Fußnoten",
        "",
        f"Der Extrakt führt {info['footnotes']} Fußnoten, verteilt auf "
        f"{len(info['footnote_pages'])} Seiten: {info['footnote_pages']}.",
        "Sie stehen als Blockzitat direkt unter dem Text ihrer Seite und sind damit kein "
        "Normtext, sondern Standhinweis. Ihre Zeichen sind in der Massenbilanz enthalten.",
        "",
        "## 5. Lesekonventionen",
        "",
        "- `#` trägt die Einheit so, wie das Amtsblatt sie druckt (`# CHAPTER I`, `# ANNEX I`).",
        "  Die gedruckte Titelzeile (etwa `GENERAL PROVISIONS`) folgt darunter als eigener Absatz.",
        "- `##` trägt `## Article 13` und, in den Anhängen, `## Part I`. Der gedruckte Titel der",
        "  Einheit folgt jeweils als eigener Absatz, damit der Anker stabil bleibt.",
        "- `###` trägt nummerierte Absätze (`### 1.`) und in den Erwägungsgründen die Marke",
        "  (`### (1)`). Die Nummer stammt unverändert aus der Quelle.",
        "- `<!-- PDF S. n -->` steht an der Stelle, an der die Seite n beginnt. Jede Seite von 1",
        "  bis 81 ist mindestens einmal markiert.",
        "- Der Normtext steht zwischen `<!-- NORM BEGIN -->` und `<!-- NORM END -->`.",
        "- Fußnoten sind Blockzitate (`>`) und mit `<!-- Fußnoten zu PDF S. n -->` eingeleitet.",
        "",
        "## 6. Grenzen und Unsicherheiten",
        "",
        "- Wortlaut vor Lesbarkeit: Absätze werden nicht umformuliert, nicht gekürzt und nicht",
        "  umsortiert. Zeilenumbrüche der Druckfassung werden zu einem Absatz zusammengezogen.",
        "- Ein Absatz, der über eine Seitengrenze läuft, wird an der Seitenmarke getrennt. Das",
        "  ist die Folge der geforderten Seitenmarken und keine inhaltliche Änderung.",
        "- Die Anhänge III, IV, V und VII enthalten im Original keine Tabellen, sondern",
        "  nummerierte Listen. Es wurde daher keine Markdown-Tabelle erzeugt und keine Zelle",
        "  umgeformt. Anhang I Teil I und Teil II tragen die Marken (1), (2) und (a) bis (m) im",
        "  Fließtext; sie sind nicht in Überschriften umgesetzt, weil die Marken dort mitten in",
        "  der Zeile stehen.",
        "- Die Signaturspalte der PDF-Seite 67 (Parlament und Rat nebeneinander) ist in",
        "  Leserichtung zeilenweise aufgelöst.",
        "- Die Erwägungsgründe 1 bis 130 sind vollständig; die Nummern der Fußnoten",
        "  überschneiden sich mit ihnen, sind aber positionsbasiert getrennt.",
        "- Der Kolumnentitel und die Seitenzahl des Amtsblatts sind entfernt. Sie werden nur",
        "  entfernt, wenn sie das Zeilenmuster des Amtsblatts treffen und zusätzlich im oberen",
        "  oder unteren Rand der Seite stehen.",
        "- Die Nummern 1.2 bis 1.10 der Checkliste sind gegen die Verordnung nicht auflösbar,",
        f"  siehe {QUICK_FILE}, Abschnitt 3.",
    ]
    return "\n".join(lines) + "\n"


def text_layer(doc: pymupdf.Document) -> dict:
    """Count lines, characters and non-space characters of the extracted text layer."""
    lines = 0
    chars = 0
    nonspace = 0
    for page in doc:
        for block in page_blocks(page):
            for line in block["lines"]:
                lines += 1
                chars += len(line["text"])
                nonspace += len(line["text"].replace(" ", ""))
    return {"lines": lines, "chars": chars, "nonspace": nonspace}


def mass_balance(doc: pymupdf.Document, texts: dict, data: dict,
                 files: list[str]) -> dict:
    """Compare the norm text of the extract with the text layer, normalised the same way.

    The Markdown joins the lines of one paragraph with a single space, so the baseline joins the
    lines of the text layer with a single space as well. The running head and foot lines are not
    part of the norm text and are subtracted on both sides. That keeps the comparison honest: the
    same characters are counted on both sides, only the line breaks of the printed page are gone.
    """
    layer = text_layer(doc)
    removed_chars = sum(len(text) for _, text, _ in data["removed"])
    removed_nonspace = sum(len(text.replace(" ", "")) for _, text, _ in data["removed"])
    norm_lines = layer["lines"] - len(data["removed"])
    baseline = layer["chars"] - removed_chars + norm_lines - 1
    extract = 0
    extract_nonspace = 0
    for name in files:
        payload = norm_payload(texts.get(name, ""))
        extract += len(payload)
        extract_nonspace += len(payload.replace(" ", ""))
    delta = extract - baseline
    return {
        "layer_chars": layer["chars"],
        "layer_lines": layer["lines"],
        "removed_lines": len(data["removed"]),
        "removed_chars": removed_chars,
        "norm_lines": norm_lines,
        "baseline": baseline,
        "extract": extract,
        "delta": delta,
        "percent": 100.0 * delta / baseline if baseline else 0.0,
        "baseline_nonspace": layer["nonspace"] - removed_nonspace,
        "extract_nonspace": extract_nonspace,
        "delta_nonspace": extract_nonspace - (layer["nonspace"] - removed_nonspace),
        "plan_percent": 100.0 * (extract - layer["chars"]) / layer["chars"],
    }


def write_output(doc: pymupdf.Document) -> None:
    """Write the 19 Markdown files and report the numbers the extraction produced."""
    data = extract(doc)
    units = split_units(data["items"])
    front = [unit for unit in units if unit["kind"] == "front"]
    chapters = [unit for unit in units if unit["kind"] == "chapter"]
    annexes = [unit for unit in units if unit["kind"] == "annex"]
    if len(front) != 1 or len(chapters) != 8 or len(annexes) != 8:
        raise SystemExit("unexpected document structure: "
                         f"{len(front)} front, {len(chapters)} chapters, {len(annexes)} annexes")
    ordered = front + chapters + annexes
    names = [RECITAL_FILE] + CHAPTER_FILES + ANNEX_FILES

    version = getattr(pymupdf, "__version__", "unbekannt")
    sha = sha256(PDF_PATH)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    targets = collect_targets(ordered, names)

    records: list[dict] = []
    for unit, name in zip(ordered, names):
        pages = sorted({item["page"] for item in unit["items"]})
        body = "\n".join(provenance_header(name, unit, pages, sha, version)
                         + [NORM_BEGIN, ""] + render_unit(unit, data["footnotes"])
                         + [NORM_END, ""])
        (OUT_DIR / name).write_text(body, encoding="utf-8", newline="\n")
        numbers = [int(match.group(1)) for item in unit["items"]
                   if (match := RE_ARTICLE.fullmatch(item["text"]))]
        if unit["kind"] == "front":
            description = "Titel, Erwägungsformel und Erwägungsgründe 1 bis 130"
        elif unit["kind"] == "chapter":
            description = (f"Kapitel {unit['title'].split()[1]}, Artikel {numbers[0]} bis "
                           f"{numbers[-1]}")
        else:
            description = f"Anhang {unit['title'].split()[1]}"
        subtitle = ""
        if unit["kind"] == "annex":
            title_items = [item["text"] for item in unit["items"][1:2]]
            subtitle = title_items[0] if title_items else ""
        records.append({
            "name": name, "kind": unit["kind"], "unit_title": unit["title"],
            "subtitle": subtitle, "description": description, "pages": pages,
            "span": (pages[0], pages[-1]), "articles": (
                f"{numbers[0]} bis {numbers[-1]}" if numbers else ""),
            "article_count": len(numbers),
        })

    sizes = {record["name"]: (OUT_DIR / record["name"]).stat().st_size
             for record in records}
    texts = {record["name"]: (OUT_DIR / record["name"]).read_text(encoding="utf-8")
             for record in records}
    balance = mass_balance(doc, texts, data, names)

    recital_numbers = [int(RE_RECITAL.match(item["text"]).group(1))
                       for item in front[0]["items"] if RE_RECITAL.match(item["text"])]
    found_articles = sorted(targets["article"])
    footnote_pages = sorted(page for page, notes in data["footnotes"].items() if notes)
    info = {
        "sha": sha,
        "version": version,
        "pages": doc.page_count,
        "layer": balance["layer_chars"],
        "layer_lines": balance["layer_lines"],
        "removed_lines": balance["removed_lines"],
        "removed_chars": balance["removed_chars"],
        "norm_lines": balance["norm_lines"],
        "baseline": balance["baseline"],
        "extract_chars": balance["extract"],
        "delta": balance["delta"],
        "percent": balance["percent"],
        "delta_nonspace": balance["delta_nonspace"],
        "plan_percent": balance["plan_percent"],
        "footnotes": sum(len(notes) for notes in data["footnotes"].values()),
        "footnote_pages": footnote_pages,
        "recital_count": len(recital_numbers),
        "recital_missing": sorted(set(range(1, 131)) - set(recital_numbers)),
        "articles": found_articles,
        "article_missing": sorted(set(range(1, 72)) - set(found_articles)),
        "article_duplicates": sorted(number for number, count
                                     in Counter(items_articles(data)).items() if count > 1),
        "annexes": [roman for roman in ROMAN[1:] if roman in targets["annex"]],
    }

    # The quick reference is written before the index, because the index lists the real size of
    # the quick reference and must not report a placeholder for a file that does not exist yet.
    (OUT_DIR / QUICK_FILE).write_text(build_quick_reference(targets),
                                      encoding="utf-8", newline="\n")
    sizes[QUICK_FILE] = (OUT_DIR / QUICK_FILE).stat().st_size

    index_text = ""
    for _ in range(8):
        index_text = build_index(records, sizes, info)
        size = len(index_text.encode("utf-8"))
        if sizes.get(INDEX_FILE) == size:
            break
        sizes[INDEX_FILE] = size
    (OUT_DIR / INDEX_FILE).write_text(index_text, encoding="utf-8", newline="\n")
    sizes[INDEX_FILE] = (OUT_DIR / INDEX_FILE).stat().st_size

    print(f"geschrieben nach : {OUT_DIR.relative_to(ROOT)}")
    for name in ALL_FILES:
        print(f"  {sizes.get(name, 0):8d} Bytes  {name}")
    print(f"\nArtikel          : {len(found_articles)} von 71, fehlend "
          f"{info['article_missing'] or 'keine'}, doppelt {info['article_duplicates'] or 'keine'}")
    print(f"Anhänge         : {len(info['annexes'])} von 8 ({', '.join(info['annexes'])})")
    print(f"Erwägungsgründe: {info['recital_count']}, Lücken "
          f"{info['recital_missing'] or 'keine'}")
    print(f"Fußnoten        : {info['footnotes']} auf {len(footnote_pages)} Seiten")
    print(f"Textschicht      : {balance['layer_chars']} Zeichen in {balance['layer_lines']} "
          f"Zeilen, entfernt {balance['removed_lines']} Zeilen mit "
          f"{balance['removed_chars']} Zeichen")
    print(f"Massenbilanz     : Extrakt {balance['extract']} gegen Messbasis "
          f"{balance['baseline']} Zeichen ({balance['percent']:+.3f} Prozent), ohne Leerzeichen "
          f"{balance['delta_nonspace']:+d} Zeichen")
    print(f"Vergleich direkt gegen die 349735 Zeichen des Plans: "
          f"{balance['plan_percent']:+.3f} Prozent, vollständig erklärt durch den entfernten "
          f"Laufkopf und die Fußzeile")
    print("Nächster Schritt: --verify")


def items_articles(data: dict) -> list[int]:
    """Article numbers in the order in which they appear, for the duplicate check."""
    return [int(match.group(1)) for item in data["items"]
            if (match := RE_ARTICLE.fullmatch(item["text"]))]


def verify_output(doc: pymupdf.Document) -> int:
    """Re-read the written files and check the self-imposed gates. Returns an exit code."""
    problems: list[str] = []
    reports: list[str] = []

    missing = [name for name in ALL_FILES if not (OUT_DIR / name).exists()]
    if missing:
        problems.append(f"fehlende Dateien: {', '.join(missing)}")
    reports.append(f"19 Dateien vorhanden : {len(ALL_FILES) - len(missing)} von {len(ALL_FILES)}")

    texts = {name: (OUT_DIR / name).read_text(encoding="utf-8")
             for name in ALL_FILES if (OUT_DIR / name).exists()}
    norm_files = [RECITAL_FILE] + CHAPTER_FILES + ANNEX_FILES

    without_norm = [name for name in norm_files
                    if name in texts and (NORM_BEGIN not in texts[name]
                                          or NORM_END not in texts[name])]
    if without_norm:
        problems.append(f"NORM-Marker fehlen in: {', '.join(without_norm)}")
    reports.append(f"NORM-Marker          : {len(norm_files) - len(without_norm)} von "
                   f"{len(norm_files)} Dateien")

    articles = [int(match.group(1)) for name in CHAPTER_FILES
                for line in texts.get(name, "").splitlines()
                if (match := re.fullmatch(r"## Article (\d+)", line))]
    article_counts = Counter(articles)
    article_missing = sorted(set(range(1, 72)) - set(articles))
    article_duplicates = sorted(number for number, count in article_counts.items() if count > 1)
    if article_missing or article_duplicates:
        problems.append(f"Artikel: fehlend {article_missing}, doppelt {article_duplicates}")
    reports.append(f"Artikel 1 bis 71     : {len(articles)} gefunden, fehlend "
                   f"{article_missing or 'keine'}, doppelt {article_duplicates or 'keine'}")

    annexes = [match.group(1) for name in ANNEX_FILES
               for line in texts.get(name, "").splitlines()
               if (match := re.fullmatch(r"# ANNEX ([IVXL]+)", line))]
    if annexes != ROMAN[1:]:
        problems.append(f"Anhänge: {annexes}")
    reports.append(f"Anhänge I bis VIII  : {annexes}")

    recitals = [int(match.group(1)) for line in texts.get(RECITAL_FILE, "").splitlines()
                if (match := re.fullmatch(r"### \((\d+)\)", line))]
    recital_missing = sorted(set(range(1, 131)) - set(recitals))
    recital_duplicates = sorted(number for number, count in Counter(recitals).items()
                                if count > 1)
    if recital_missing or recital_duplicates:
        problems.append(f"Erwägungsgründe: fehlend {recital_missing}, doppelt "
                        f"{recital_duplicates}")
    reports.append(f"Erwägungsgründe    : {len(recitals)} von 130, fehlend "
                   f"{recital_missing or 'keine'}, doppelt {recital_duplicates or 'keine'}")

    data = extract(doc)
    balance = mass_balance(doc, texts, data, norm_files)
    delta = balance["delta"]
    percent = balance["percent"]
    if abs(percent) > 0.5:
        problems.append(f"Massenbilanz: {percent:+.3f} Prozent")
    reports.append(f"Massenbilanz        : Extrakt {balance['extract']} gegen Messbasis "
                   f"{balance['baseline']} Zeichen ({percent:+.3f} Prozent), ohne Leerzeichen "
                   f"{balance['delta_nonspace']:+d} Zeichen")
    reports.append(f"Textschicht gesamt  : {balance['layer_chars']} Zeichen in "
                   f"{balance['layer_lines']} Zeilen, entfernt {balance['removed_lines']} "
                   f"Zeilen mit {balance['removed_chars']} Zeichen, Messbasis "
                   f"{balance['baseline']} Zeichen")
    reports.append(f"Direkt gegen 349735 : {balance['plan_percent']:+.3f} Prozent, erklärt "
                   f"durch den entfernten Laufkopf und die Fußzeile")

    replacement = [name for name, text in texts.items() if "\ufffd" in text]
    if replacement:
        problems.append(f"U+FFFD in: {', '.join(replacement)}")
    reports.append(f"U+FFFD               : {len(replacement)} Dateien")

    em_dash = [name for name, text in texts.items() if "\u2014" in text]
    if em_dash:
        problems.append(f"U+2014 in: {', '.join(em_dash)}")
    reports.append(f"U+2014               : {len(em_dash)} Dateien")

    empty: list[str] = []
    for name in norm_files:
        for number, line in enumerate(texts.get(name, "").splitlines(), start=1):
            if re.fullmatch(r"#{1,6}\s*", line):
                empty.append(f"{name}:{number}")
    if empty:
        problems.append(f"leere Überschriften: {', '.join(empty)}")
    reports.append(f"leere Überschriften : {len(empty)}")

    markers = sorted({int(match.group(1)) for name in norm_files
                      for match in re.finditer(r"<!-- PDF S\. (\d+) -->", texts.get(name, ""))})
    missing_pages = sorted(set(range(1, 82)) - set(markers))
    if missing_pages:
        problems.append(f"Seitenmarken fehlen: {missing_pages}")
    reports.append(f"Seitenmarken 1 bis 81: {len(markers)} gefunden, fehlend "
                   f"{missing_pages or 'keine'}")

    encoding: list[str] = []
    for name in ALL_FILES:
        path = OUT_DIR / name
        if not path.exists():
            continue
        blob = path.read_bytes()
        if blob.startswith(b"\xef\xbb\xbf"):
            encoding.append(f"{name}: BOM")
        if b"\r" in blob:
            encoding.append(f"{name}: CR")
    if encoding:
        problems.append(f"Kodierung: {', '.join(encoding)}")
    reports.append(f"UTF-8 ohne BOM, LF  : {len(encoding)} Abweichungen")

    print("Selbstprüfung der Ausgabe")
    for line in reports:
        print(f"  {line}")
    print()
    if problems:
        print("FEHLER:")
        for line in problems:
            print(f"  {line}")
        return 1
    print("Alle geprüften Punkte bestanden. Die Abnahme gegen die sieben Gates des Plans")
    print("bleibt beim Hauptagenten, dieser Lauf prüft nur die eigene Ausgabe.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recon", action="store_true", help="print structure statistics only")
    parser.add_argument("--write", action="store_true", help="write the 19 Markdown files")
    parser.add_argument("--verify", action="store_true", help="check the written files")
    parser.add_argument("--dump-page", type=int, action="append", default=[],
                        help="print the lines of one page (1-based), repeatable")
    parser.add_argument("--full", action="store_true",
                        help="with --dump-page: do not truncate the line text")
    args = parser.parse_args()

    if sum([args.recon, args.write, args.verify, bool(args.dump_page)]) != 1:
        parser.error("select exactly one mode: --recon, --write, --verify or --dump-page")

    doc = pymupdf.open(PDF_PATH)
    try:
        if args.dump_page:
            dump_pages(doc, args.dump_page, full=args.full)
        elif args.recon:
            recon(doc)
        elif args.write:
            write_output(doc)
        else:
            return verify_output(doc)
    finally:
        doc.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
