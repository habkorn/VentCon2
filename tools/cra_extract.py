"""Extract Regulation (EU) 2024/2847 (Cyber Resilience Act) from the Official Journal PDF.

The PDF is born-digital (PDFlib/Arbortext, CID fonts with ToUnicode CMaps), so the text layer is
used directly and no OCR is involved. The script is deterministic: same PDF plus same version
always produces the same Markdown.

Modes
-----
    --recon   print structure statistics, write nothing
    --write   write the Markdown chapter files into sec/sources/EU_CRA_Regulation_2024_2847_EN/

Plan: sec/plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections import Counter
from pathlib import Path

import pymupdf

ROOT = Path(__file__).resolve().parents[1]
PDF_PATH = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN.pdf"
OUT_DIR = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN"

# Font sizes seen in the document (points, rounded):
#   15.9  regulation title on the cover page
#    9.6  body text, bold for headings (Article / CHAPTER / recital numbers)
#    8.5  footnotes at the bottom of the page
#    5.5  tiny OJ imprint glyphs
BODY_SIZE = 9.6
FOOTNOTE_MAX = 9.0

RE_CHAPTER = re.compile(r"^CHAPTER [IVXL]+$")
RE_SECTION = re.compile(r"^SECTION \d+$")
RE_PART = re.compile(r"^PART [IVXL]+$")
RE_ARTICLE = re.compile(r"^Article (\d+)$")
RE_ANNEX = re.compile(r"^ANNEX ([IVXL]+)$")
RE_RECITAL = re.compile(r"^\((\d+)\)$")
RE_ROMAN = {
    "I": 1, "II": 2, "III": 3, "IV": 4, "V": 5, "VI": 6, "VII": 7, "VIII": 8,
    "IX": 9, "X": 10, "XI": 11, "XII": 12,
}

# Running header and footer of the Official Journal plus the page number "12/81".
RE_FOOTER = re.compile(
    r"^(Official Journal|of the European Union|EN|L series|2024/2847|"
    r"OJ L, \d{2}\.\d{2}\.\d{4}|"
    r"\d{2}\.\d{2}\.\d{4}|ELI: https?://data\.europa\.eu/eli/reg/2024/2847/oj|"
    r"\d+/\d+)$"
)

# Footnotes and the enacting formula both use 8.5 pt. Only footnotes sit in the bottom band of the
# page, therefore the vertical position decides, not the font size.
FOOTNOTE_BAND = 0.80  # share of the page height below which a small block is a footnote

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


def page_lines(page: pymupdf.Page) -> tuple[list[dict], list[dict]]:
    """Split one page into (content lines, footnote lines).

    Small text is only a footnote when it also sits in the bottom band of the page, because the
    Official Journal sets chapter headings and the enacting formula in the same 8.5 pt face.
    """
    limit = page.rect.height * FOOTNOTE_BAND
    body: list[dict] = []
    notes: list[dict] = []
    for block in page_blocks(page):
        is_note = (
            max(line["size"] for line in block["lines"]) < FOOTNOTE_MAX
            and block["bbox"][1] > limit
        )
        for line in block["lines"]:
            (notes if is_note else body).append(line)
    return body, notes


def dehyphenate(lines: list[str], vocabulary: Counter) -> list[str]:
    """Join wrapped lines. A trailing hyphen is kept only for real compounds.

    A word broken at the line end ("resil-\\nience") must be joined without the hyphen, while a
    genuine compound broken at the line end ("cyber-\\nresilience") must keep it. The distinction
    is made with the document's own vocabulary: if the part before the hyphen also occurs as a
    standalone word elsewhere in the document, the hyphen is real.
    """
    out: list[str] = []
    for raw in lines:
        text = raw.rstrip()
        if out:
            previous = out[-1]
            if previous.endswith("-"):
                head = re.search(r"([A-Za-z]{2,})$", previous[:-1])
                tail = re.match(r"[a-z]", text)
                if head and tail:
                    if vocabulary[head.group(1).lower()] > 0:
                        out[-1] = previous + text        # cyber- + resilience
                    else:
                        out[-1] = previous[:-1] + text   # resil + ience
                    continue
            elif previous and not previous.endswith((" ", "-")) and re.match(r"^[a-zA-Z(]", text):
                out[-1] = previous + " " + text
                continue
        out.append(text)
    return out


def build_vocabulary(doc: pymupdf.Document) -> Counter:
    vocab: Counter = Counter()
    for page in doc:
        for word in re.findall(r"[A-Za-z]{2,}", page.get_text()):
            vocab[word.lower()] += 1
    return vocab


def join_paragraph(chunks: list[str]) -> str:
    """Join the dehyphenated lines of one paragraph into a single Markdown line."""
    text = " ".join(part.strip() for part in chunks if part.strip())
    return re.sub(r"\s{2,}", " ", text).strip()


def recon(doc: pymupdf.Document) -> None:
    sizes: Counter = Counter()
    fonts: Counter = Counter()
    small_samples: list[str] = []
    footer_hits: Counter = Counter()
    unknown_small: list[tuple[int, str, float]] = []
    recitals: list[int] = []
    markers: Counter = Counter()
    italics: list[tuple[int, str]] = []
    hyphen_cases: list[tuple[str, str, int]] = []
    typography: Counter = Counter()

    vocabulary = build_vocabulary(doc)

    for number, page in enumerate(doc, start=1):
        body, notes = page_lines(page)
        for line in body + notes:
            sizes[line["size"]] += len(line["text"])
            if line["size"] < 5.6:
                unknown_small.append((number, line["text"], line["size"]))
            if line["size"] < 9.0 and not RE_FOOTER.match(line["text"]):
                if len(small_samples) < 12:
                    small_samples.append(line["text"])
            if RE_FOOTER.match(line["text"]):
                footer_hits[line["text"]] += 1
            if RE_RECITAL.match(line["text"]):
                recitals.append(int(line["text"][1:-1]))
            if re.fullmatch(r"\d+\.", line["text"]):
                markers["paragraph"] += 1
            if re.fullmatch(r"\([a-z]{1,2}\)", line["text"]):
                markers["point"] += 1
            if line["italic"] and line["size"] > 9.4:
                if len(italics) < 12:
                    italics.append((number, line["text"]))
        text_lines = [line["text"] for line in body]
        for line in body + notes:
            raw = line["text"]
            typography["body_chars" if line in body else "note_chars"] += len(raw)
            typography["dash_end"] += int(raw.endswith("-"))
            typography["soft_hyphen"] += raw.count("\u00ad")
            typography["replacement"] += raw.count("\ufffd")
            typography["em_dash"] += raw.count("\u2014")
            typography["en_dash"] += raw.count("\u2013")
            typography["numbered"] += int(bool(re.fullmatch(r"\d+\.", raw)))
            typography["recital_marker"] += int(bool(re.fullmatch(r"\(\d+\)", raw)))
        for first, second in zip(text_lines, text_lines[1:]):
            if first.endswith("-"):
                head = re.search(r"([A-Za-z]{2,})$", first[:-1])
                tail = re.match(r"[a-z]", second)
                if head and tail and len(hyphen_cases) < 25:
                    hyphen_cases.append((head.group(1), second[:14],
                                         vocabulary[head.group(1).lower()]))
        for name in page.get_fonts(full=True):
            fonts[name[3]] += 1

    headings = []
    for number, page in enumerate(doc, start=1):
        body, notes = page_lines(page)
        for line in body:
            text = line["text"]
            for pattern, kind in ((RE_CHAPTER, "chapter"), (RE_SECTION, "section"),
                                  (RE_PART, "part"), (RE_ARTICLE, "article"),
                                  (RE_ANNEX, "annex")):
                if pattern.match(text):
                    headings.append((number, kind, text))

    print(f"file       : {PDF_PATH.name}")
    print(f"sha256     : {sha256(PDF_PATH)}")
    print(f"pages      : {doc.page_count}")
    print(f"pymupdf    : {pymupdf.__doc__.strip().splitlines()[0] if pymupdf.__doc__ else '?'}")
    print("\nsizes (chars):")
    for size, count in sizes.most_common():
        print(f"  {size:5.1f}  {count:8d}")
    print("\nfonts:")
    for name, count in fonts.most_common():
        print(f"  {name:28s} {count}")
    print("\nfooter lines matched:")
    for text, count in footer_hits.most_common(8):
        print(f"  {count:3d}x {text!r}")
    print(f"\nrecitals found: {len(recitals)}")
    if recitals:
        print(f"  first={min(recitals)} last={max(recitals)} "
              f"gaps={sorted(set(range(min(recitals), max(recitals) + 1)) - set(recitals))}")
    print(f"\nparagraph markers '1.' : {markers['paragraph']}")
    print(f"point markers '(a)'    : {markers['point']}")
    print("\nsmall text that is not footer (first 12):")
    for text in small_samples:
        print(f"  {text[:100]!r}")
    print("\nitalic lines in body text (first 12):")
    for number, text in italics:
        print(f"  p{number} {text[:100]!r}")
    print("\ntiny text below 5.6pt:")
    for number, text, size in unknown_small[:10]:
        print(f"  p{number} {size}pt {text[:80]!r}")
    print(f"\nheadings found: {len(headings)}")
    kinds = Counter(kind for _, kind, _ in headings)
    print(f"  by kind: {dict(kinds)}")
    for number, kind, text in headings:
        print(f"  p{number:3d} {kind:8s} {text}")
    print("\nline-end hyphen candidates (head, tail, head-as-word-count):")
    for head, tail, count in hyphen_cases:
        print(f"  {head:<22} {tail:<16} {count}")
    print("\ntypographic survey:")
    print(f"  lines ending in '-'          : {typography['dash_end']}")
    print(f"  soft hyphen U+00AD           : {typography['soft_hyphen']}")
    print(f"  replacement char U+FFFD      : {typography['replacement']}")
    print(f"  em dash U+2014               : {typography['em_dash']}")
    print(f"  en dash U+2013               : {typography['en_dash']}")
    print(f"  standalone 'N.' markers      : {typography['numbered']}")
    print(f"  standalone '(n)' markers     : {typography['recital_marker']}")
    print(f"  body chars / note chars      : {typography['body_chars']} / {typography['note_chars']}")


def dump_pages(doc: pymupdf.Document, numbers: list[int], full: bool = False) -> None:
    """Print the block and line structure of selected pages (1-based) for debugging.

    Every line is prefixed with its vertical position (y0, top of the line box) and its distance
    to the previous line on the same page, so paragraph gaps can be measured.
    """
    limit = {
        number: doc[number - 1].rect.height * FOOTNOTE_BAND for number in numbers
    }
    for number in numbers:
        page = doc[number - 1]
        print(f"=== page {number} (footnote band below y={limit[number]:.0f}) ===")
        previous = None
        for block in page_blocks(page):
            kind = "note" if block["bbox"][1] > limit[number] and \
                max(line["size"] for line in block["lines"]) < FOOTNOTE_MAX else "body"
            print(f"  -- block {kind} y={block['bbox'][1]:.0f} lines={len(block['lines'])}")
            for line in block["lines"]:
                flag = "F" if RE_FOOTER.match(line["text"]) else " "
                top = round(line["bbox"][1], 1)
                gap = f"{top - previous:5.1f}" if previous is not None else "  -  "
                previous = top
                text = line["text"] if full else line["text"][:88]
                print(f"     {flag} y={top:6.1f} d={gap} {line['size']:5.1f} "
                      f"b={int(line['bold'])} i={int(line['italic'])} {text!r}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recon", action="store_true", help="print structure statistics only")
    parser.add_argument("--write", action="store_true", help="write the Markdown files")
    parser.add_argument("--dump-page", type=int, action="append", default=[],
                        help="print the lines of one page (1-based), repeatable")
    parser.add_argument("--full", action="store_true",
                        help="with --dump-page: do not truncate the line text")
    args = parser.parse_args()

    doc = pymupdf.open(PDF_PATH)
    try:
        if args.dump_page:
            dump_pages(doc, args.dump_page, full=args.full)
        elif args.recon or not args.write:
            recon(doc)
    finally:
        doc.close()


if __name__ == "__main__":
    main()
