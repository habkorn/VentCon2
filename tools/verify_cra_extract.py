"""Unabhaengige Verifikation des CRA-Volltextextrakts gegen die Textebene der PDF.

Fuehrt die sieben Gates aus sec/plan/2026-09-30_CRA_Volltext_Extraktion_Plan.md durch.

Der Code ist bewusst unabhaengig von tools/cra_extract.py geschrieben. Alle Sollwerte
werden direkt aus der PDF gelesen, nicht aus dem Extraktor uebernommen.

Aufbau der Sollwerte:
  * Zeilen, die auf gleicher Hoehe stehen, bilden eine Lesezeile. Damit landen
    Erwaegungsgrund-Marken (8.5 pt, links neben dem Absatz) und hochgestellte
    Fussnotenverweise (8.5 pt, mitten in der Zeile) an der richtigen Stelle.
  * Kolumnentitel und Fusszeile des Amtsblatts werden als Satzspiegel-Rand erkannt und
    nicht als Normtext gezaehlt.
  * Fussnoten sind die Zeilen unterhalb der letzten Zeile des Satzspiegels. Sie sind
    ebenfalls 8.5 pt und daher nur ueber die Lage von Marken unterscheidbar.

Aufruf:
    C:\\venvs\\VentCon2\\Scripts\\python.exe tools\\verify_cra_extract.py

Rueckgabewert: 0 wenn alle Gates bestanden sind, 1 wenn ein Gate gerissen ist.
"""

from __future__ import annotations

import difflib
import re
import sys
from collections import Counter
from pathlib import Path

import pymupdf

ROOT = Path(__file__).resolve().parents[1]
PDF_PATH = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN.pdf"
OUT_DIR = ROOT / "sec" / "sources" / "EU_CRA_Regulation_2024_2847_EN"

EXPECTED_FILES = [
    "00_Index.md",
    "01_Recitals.md",
    "02_Chapter_I_Art_01-12.md",
    "03_Chapter_II_Art_13-26.md",
    "04_Chapter_III_Art_27-34.md",
    "05_Chapter_IV_Art_35-51.md",
    "06_Chapter_V_Art_52-60.md",
    "07_Chapter_VI_Art_61-62.md",
    "08_Chapter_VII_Art_63-65.md",
    "09_Chapter_VIII_Art_66-71.md",
    "10_Annex_I.md",
    "11_Annex_II.md",
    "12_Annex_III.md",
    "13_Annex_IV.md",
    "14_Annex_V.md",
    "15_Annex_VI.md",
    "16_Annex_VII.md",
    "17_Annex_VIII.md",
    "18_Quick_Reference_VentCon2.md",
]
NORM_FILES = [n for n in EXPECTED_FILES if n[:2].isdigit() and 1 <= int(n[:2]) <= 17]
INDEX_FILE = EXPECTED_FILES[0]
QUICK_FILE = EXPECTED_FILES[-1]

RE_FURNITURE = re.compile(
    r"^(EN|OJ L, \d{2}\.\d{2}\.\d{4}|\d{2}\.\d{2}\.\d{4}|\d+/\d+|"
    r"ELI: \S+|Official Journal|of the European Union|L series|\d{4}/\d{4})$"
)
FURNITURE_ANYWHERE_PAGES = {1}

RE_MD_ARTICLE = re.compile(r"^Article (\d+)$")
RE_MD_ANNEX = re.compile(r"^ANNEX ([IVXL]+)$")
RE_MD_CHAPTER = re.compile(r"^CHAPTER ([IVXL]+)$")
RE_MD_RECITAL = re.compile(r"^\((\d+)\)$")
RE_PAGE_MARK = re.compile(r"^<!-- PDF S\. (\d+) -->$")
RE_NOTE_MARK = re.compile(r"^<!-- Fussnoten zu PDF S\. (\d+) -->$")
RE_NORM_EDGE = re.compile(r"^<!-- NORM (BEGIN|END) -->$")
RE_MD_LINK = re.compile(r"\]\(([A-Za-z0-9_.\-]+\.md)#([A-Za-z0-9_\-]+)\)")
RE_FOOTNOTE_ENTRY = re.compile(r"^\((\d+)\)")

RE_MARKER = re.compile(r"^\(\s*(\d+)\s*\)$")
RE_ARTICLE_LINE = re.compile(r"^Article (\d+)$")
RE_ANNEX_LINE = re.compile(r"^ANNEX ([IVXL]+)$")
RE_CHAPTER_LINE = re.compile(r"^CHAPTER ([IVXL]+)$")

TOP_BAND = 70.0
BOTTOM_BAND = 70.0
FOOTNOTE_SIZE = 9.0
ROW_TOLERANCE = 3.0  # Zeilenhoehe, innerhalb derer Segmente eine Lesezeile bilden
PARAGRAPH_GAP = 14.0  # Zeilenabstand, ab dem die Quelle einen neuen Absatz beginnen kann
IN_PARAGRAPH_GAP = 13.5  # gemessene Obergrenze fuer den Zeilenabstand innerhalb eines Absatzes
CONTEXT = 16  # Zeichen je Seite fuer den Join-Kontext

CANON_MAP = {
    "\u00a0": " ", "\u2009": " ", "\u202f": " ", "\u200b": "",
    "\u2018": "'", "\u2019": "'", "\u201c": '"', "\u201d": '"',
    "\ufb00": "ff", "\ufb01": "fi", "\ufb02": "fl", "\ufb03": "ffi", "\ufb04": "ffl",
}


def canon(text: str) -> str:
    for bad, good in CANON_MAP.items():
        text = text.replace(bad, good)
    return re.sub(r"\s+", "", text)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def first_mismatch(left: str, right: str) -> int:
    limit = min(len(left), len(right))
    for index in range(limit):
        if left[index] != right[index]:
            return index
    return -1 if len(left) == len(right) else limit


def slug(heading: str) -> str:
    text = re.sub(r"[^\w\s\-]", "", heading.strip().lower())
    return text.replace(" ", "-")


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, bool, str]] = []
        self.notes: list[str] = []

    def add(self, gate: str, passed: bool, message: str) -> None:
        self.rows.append((gate, passed, message))

    def note(self, message: str) -> None:
        self.notes.append(message)

    @property
    def failed(self) -> list[tuple[str, bool, str]]:
        return [row for row in self.rows if not row[1]]


def pdf_pages(doc: pymupdf.Document) -> list[dict]:
    """Textebene je Seite als Lesezeilen, unabhaengig vom Extraktor aufgebaut."""
    pages: list[dict] = []
    for number, page in enumerate(doc, start=1):
        height = page.rect.height
        segments: list[dict] = []
        for block in page.get_text("dict")["blocks"]:
            if block.get("type") != 0:
                continue
            for line in block.get("lines", []):
                text = " ".join("".join(span["text"] for span in line["spans"]).split())
                if not text:
                    continue
                segments.append({
                    "text": text,
                    "size": max(round(span["size"], 1) for span in line["spans"]),
                    "x": line["bbox"][0],
                    "y": line["bbox"][1],
                    "y1": line["bbox"][3],
                })
        stripped: list[str] = []
        kept: list[dict] = []
        for segment in segments:
            furniture = RE_FURNITURE.match(segment["text"]) and (
                segment["y1"] < TOP_BAND
                or segment["y"] > height - BOTTOM_BAND
                or number in FURNITURE_ANYWHERE_PAGES
            )
            (stripped if furniture else kept).append(segment["text"] if furniture else segment)

        kept.sort(key=lambda item: (round(item["y"], 1), item["x"]))
        rows: list[dict] = []
        for segment in kept:
            if rows and abs(segment["y"] - rows[-1]["y"]) <= ROW_TOLERANCE:
                rows[-1]["segments"].append(segment)
            else:
                rows.append({"y": segment["y"], "segments": [segment]})
        for row in rows:
            row["segments"].sort(key=lambda item: item["x"])
            row["text"] = " ".join(segment["text"] for segment in row["segments"])
            row["size"] = max(segment["size"] for segment in row["segments"])
            row["body_size"] = any(segment["size"] >= FOOTNOTE_SIZE
                                   for segment in row["segments"])

        body_candidates = [row for row in rows if row["body_size"]]
        last_body_y = max((row["y"] for row in body_candidates), default=0.0)
        body = [row for row in rows if row["y"] <= last_body_y + 1.0]
        notes = [row["text"] for row in rows if row["y"] > last_body_y + 1.0]
        pages.append({"page": number, "rows": body, "notes": notes, "stripped": stripped})
    return pages


def markdown_units(path: Path) -> list[tuple[str, str]]:
    """Inhalt einer Normdatei als Folge von Marken, Ueberschriften, Absaetzen, Fussnoten."""
    units: list[tuple[str, str]] = []
    inside = False
    for raw in read_text(path).splitlines():
        line = raw.strip()
        if RE_NORM_EDGE.match(line):
            inside = line.endswith("BEGIN -->")
            continue
        if not inside or not line:
            continue
        match = RE_PAGE_MARK.match(line)
        if match:
            units.append(("page", match.group(1)))
            continue
        match = RE_NOTE_MARK.match(line)
        if match:
            units.append(("notemark", match.group(1)))
            continue
        if line.startswith("#"):
            units.append(("head", line.lstrip("#").strip()))
        elif line.startswith(">"):
            units.append(("quote", line.lstrip(">").strip()))
        else:
            units.append(("text", line))
    return units


def main() -> int:
    report = Report()
    doc = pymupdf.open(PDF_PATH)
    try:
        pages = pdf_pages(doc)
        page_count = doc.page_count
    finally:
        doc.close()

    # Je Seite erst der Satzspiegel, dann die Fussnoten dieser Seite. Genau so setzt der
    # Extrakt die Fussnoten an das Ende ihrer Seite.
    gold = "".join(
        "".join(canon(row["text"]) for row in page["rows"])
        + "".join(canon(text) for text in page["notes"])
        for page in pages
    )
    stripped_lines = [text for page in pages for text in page["stripped"]]

    # Lesezeilen, die in der Quelle denselben Absatz bilden, mit gemessenem Zeilenabstand.
    same_paragraph_contexts: dict[str, float] = {}
    for page in pages:
        rows = page["rows"]
        for previous, following in zip(rows, rows[1:]):
            gap = following["y"] - previous["y"]
            if gap <= PARAGRAPH_GAP:
                context = canon(previous["text"])[-CONTEXT:] + canon(following["text"])[:CONTEXT]
                if context not in same_paragraph_contexts or gap < same_paragraph_contexts[context]:
                    same_paragraph_contexts[context] = gap

    # ------------------------------------------------------------------ Ausgabe lesen
    present = sorted(path.name for path in OUT_DIR.glob("*.md")) if OUT_DIR.is_dir() else []
    missing = [n for n in EXPECTED_FILES if n not in present]
    extra = [n for n in present if n not in EXPECTED_FILES]
    report.add("G1 Dateien", not missing and not extra,
               f"{len(present)} Dateien, fehlend {len(missing)}, zusaetzlich {len(extra)}"
               + (f"; fehlend: {missing}" if missing else "")
               + (f"; zusaetzlich: {extra}" if extra else ""))

    md_articles: list[int] = []
    md_annexes: list[str] = []
    md_chapters: list[str] = []
    md_recitals: list[int] = []
    md_pages: list[int] = []
    md_notes: list[int] = []
    md_text: list[str] = []
    paragraph_kinds: Counter = Counter()
    anchors: dict[str, set[str]] = {}
    per_file: dict[str, list[tuple[str, str]]] = {}

    for name in NORM_FILES:
        path = OUT_DIR / name
        if not path.exists():
            continue
        units = markdown_units(path)
        per_file[name] = units
        file_anchors = anchors.setdefault(name, set())
        for kind, value in units:
            if kind == "page":
                md_pages.append(int(value))
                continue
            if kind == "notemark":
                continue
            if kind == "quote":
                match = RE_FOOTNOTE_ENTRY.match(value)
                if match:
                    md_notes.append(int(match.group(1)))
                md_text.append(value)
                continue
            md_text.append(value)
            paragraph_kinds[kind] += 1
            if kind == "head":
                file_anchors.add(slug(value))
                for pattern, bucket, cast in (
                    (RE_MD_ARTICLE, md_articles, int),
                    (RE_MD_ANNEX, md_annexes, str),
                    (RE_MD_CHAPTER, md_chapters, str),
                    (RE_MD_RECITAL, md_recitals, int),
                ):
                    match = pattern.match(value)
                    if match:
                        bucket.append(cast(match.group(1)))  # type: ignore[arg-type]
                        break

    for name in (INDEX_FILE, QUICK_FILE):
        path = OUT_DIR / name
        if path.exists():
            file_anchors = anchors.setdefault(name, set())
            for raw in read_text(path).splitlines():
                line = raw.strip()
                if line.startswith("#") and not line.startswith("<!--"):
                    file_anchors.add(slug(line.lstrip("#").strip()))

    # -------------------------------------------------------------- PDF-Sollwerte
    pdf_articles: list[int] = []
    pdf_annexes: list[str] = []
    pdf_chapters: list[str] = []
    pdf_recitals: list[int] = []
    pdf_note_markers: list[int] = []
    for page in pages:
        for row in page["rows"]:
            first = row["segments"][0]
            if first["size"] < FOOTNOTE_SIZE and RE_MARKER.match(first["text"]) and row["body_size"]:
                # Marke links neben ihrem Absatz: Erwaegungsgrund im Satzspiegel.
                pdf_recitals.append(int(RE_MARKER.match(first["text"]).group(1)))  # type: ignore[union-attr]
                continue
            text = row["text"]
            match = RE_ARTICLE_LINE.match(text)
            if match:
                pdf_articles.append(int(match.group(1)))
                continue
            match = RE_ANNEX_LINE.match(text)
            if match:
                pdf_annexes.append(match.group(1))
                continue
            match = RE_CHAPTER_LINE.match(text)
            if match:
                pdf_chapters.append(match.group(1))
        for text in page["notes"]:
            match = RE_FOOTNOTE_ENTRY.match(text)
            if match:
                pdf_note_markers.append(int(match.group(1)))

    # ----------------------------------------------------------- G1 Vollstaendigkeit
    want_articles = list(range(1, max(pdf_articles) + 1)) if pdf_articles else []
    report.add("G1 Artikel",
               sorted(md_articles) == want_articles and len(md_articles) == len(set(md_articles)),
               f"PDF {len(pdf_articles)}, Extrakt {len(md_articles)}; fehlend "
               f"{sorted(set(want_articles) - set(md_articles))}; doppelt "
               f"{[n for n, c in Counter(md_articles).items() if c > 1]}")

    report.add("G1 Anhaenge",
               sorted(md_annexes) == sorted(set(md_annexes))
               and set(md_annexes) == set(pdf_annexes),
               f"PDF {pdf_annexes}, Extrakt {md_annexes}")

    want_recitals = list(range(1, max(pdf_recitals) + 1)) if pdf_recitals else []
    report.add("G1 Erwaegungsgruende",
               sorted(md_recitals) == want_recitals and len(md_recitals) == len(set(md_recitals)),
               f"PDF {len(pdf_recitals)} (1 bis {max(pdf_recitals) if pdf_recitals else 0}), "
               f"Extrakt {len(md_recitals)}; im PDF fehlend "
               f"{sorted(set(want_recitals) - set(pdf_recitals))}; im PDF doppelt "
               f"{sorted(n for n, c in Counter(pdf_recitals).items() if c > 1)[:8]}")

    # ------------------------------------------------------------ G2 Massenbilanz
    md_canon = "".join(canon(text) for text in md_text)
    furniture_chars = sum(len(canon(text)) for text in stripped_lines)
    raw_chars = sum(len(canon(row["text"])) for page in pages for row in page["rows"]) \
        + sum(len(canon(text)) for page in pages for text in page["notes"]) + furniture_chars
    loss_adj = (len(gold) - len(md_canon)) / len(gold) * 100 if gold else 0.0
    bad_strip = [text for text in stripped_lines if not RE_FURNITURE.match(text)]
    report.add("G2 Massenbilanz", abs(loss_adj) <= 0.5,
               f"Textebene {raw_chars}, davon Amtsblatt-Kopfdaten {furniture_chars} Zeichen in "
               f"{len(stripped_lines)} Zeilen, Messbasis {len(gold)}, Extrakt {len(md_canon)}; "
               f"Abweichung {loss_adj:+.3f} Prozent")
    report.add("G2 Randzeilen-Audit", not bad_strip,
               f"{len(stripped_lines)} entfernte Zeilen, alle nach Amtsblattmuster"
               + (f"; verdaechtig: {bad_strip[:5]}" if bad_strip else ""))

    # ------------------------------------------------------------ G3 Seitengrenzen
    md_by_page: dict[int, list[str]] = {}
    current = 0
    for name in NORM_FILES:
        for kind, value in per_file.get(name, []):
            if kind == "page":
                current = int(value)
                md_by_page.setdefault(current, [])
            elif kind in ("text", "head", "quote"):
                md_by_page.setdefault(current, []).append(value)

    mismatched: list[str] = []
    for page in pages:
        number = page["page"]
        want: Counter = Counter()
        for row in page["rows"]:
            want.update(canon(row["text"]))
        for text in page["notes"]:
            want.update(canon(text))
        got: Counter = Counter()
        for text in md_by_page.get(number, []):
            got.update(canon(text))
        if want != got:
            lacking = "".join(sorted((want - got).elements()))
            surplus = "".join(sorted((got - want).elements()))
            mismatched.append(f"S.{number} fehlt {lacking[:40]!r} ueberzaehlig {surplus[:40]!r}")
    report.add("G3 Seitengrenzen", not mismatched,
               f"{page_count - len(mismatched)} von {page_count} Seiten zeichengleich"
               + ("; " + " | ".join(mismatched[:5]) if mismatched else ""))

    mismatch = first_mismatch(gold, md_canon)
    ok_sequence = mismatch < 0
    detail = ""
    if not ok_sequence:
        start = max(0, mismatch - 100)
        want_window = gold[start:mismatch + 140]
        got_window = md_canon[start:mismatch + 140]
        matcher = difflib.SequenceMatcher(None, want_window, got_window, autojunk=False)
        blocks = [op for op in matcher.get_opcodes() if op[0] != "equal"]
        detail = (f"Laenge Soll {len(gold)} gegen Ist {len(md_canon)}, erste Abweichung bei "
                  f"{mismatch}; " + "; ".join(
                      f"{tag} Soll {want_window[i1:i2][:30]!r} gegen Ist {got_window[j1:j2][:30]!r}"
                      for tag, i1, i2, j1, j2 in blocks[:4]))
    report.add("G3 Wortfolge gesamt", ok_sequence,
               "Wortlaut und Reihenfolge zeichengleich" if ok_sequence else detail)

    marks = set(md_pages)
    report.add("G3 Seitenmarken", marks == set(range(1, page_count + 1)),
               f"{len(marks)} von {page_count} Seiten markiert; fehlend "
               f"{sorted(set(range(1, page_count + 1)) - marks)}; mehrfach "
               f"{sorted(n for n, c in Counter(md_pages).items() if c > 1)}")

    # -------------------------------------------------------------- G4 Fundstellen
    links: list[tuple[str, str]] = []
    unresolved_links: list[str] = []
    quick = OUT_DIR / QUICK_FILE
    if quick.exists():
        for raw in read_text(quick).splitlines():
            for target, anchor in RE_MD_LINK.findall(raw):
                links.append((target, anchor))
                if anchor not in anchors.get(target, set()):
                    unresolved_links.append(f"{target}#{anchor}")
    report.add("G4 Anker", not unresolved_links,
               f"{len(links)} Verweise geprueft, {len(unresolved_links)} ohne Ziel"
               + (f": {sorted(set(unresolved_links))[:6]}" if unresolved_links else ""))

    unresolved_citations: list[str] = []
    red_references: list[str] = []
    if quick.exists():
        for raw in read_text(quick).splitlines():
            if "| (kein Anker)" not in raw:
                continue
            cells = raw.split("|")
            if len(cells) < 3:
                continue
            label = cells[1].strip()
            (red_references if "RED-Bezug" in raw else unresolved_citations).append(label)
    report.add("G4 Fundstellen", not unresolved_citations,
               f"{len(unresolved_citations)} CRA-Fundstellen ohne Beleg: {unresolved_citations}; "
               f"als RED-Bezug bewusst ausgenommen: {red_references}")

    # ------------------------------------------------------------------ G5 Kodierung
    problems: list[str] = []
    for name in EXPECTED_FILES:
        path = OUT_DIR / name
        if not path.exists():
            problems.append(f"{name}: fehlt")
            continue
        blob = path.read_bytes()
        if blob.startswith(b"\xef\xbb\xbf"):
            problems.append(f"{name}: BOM")
        if b"\r\n" in blob:
            problems.append(f"{name}: CRLF")
        if "\ufffd" in blob.decode("utf-8"):
            problems.append(f"{name}: U+FFFD")
    report.add("G5 Kodierung", not problems,
               "; ".join(problems) if problems else "UTF-8 ohne BOM, LF, kein U+FFFD")

    empty_heads = [f"{name}: {line!r}" for name in EXPECTED_FILES
                   if (OUT_DIR / name).exists()
                   for line in read_text(OUT_DIR / name).splitlines()
                   if line.strip().startswith("#") and not line.strip().startswith("<!--")
                   and not line.strip().lstrip("#").strip()]
    report.add("G5 Ueberschriften", not empty_heads,
               "; ".join(empty_heads) if empty_heads else "keine leere Ueberschrift")

    gold_nonascii = Counter(ch for ch in gold if ord(ch) > 127)
    md_nonascii = Counter(ch for ch in md_canon if ord(ch) > 127)
    diff_nonascii = (gold_nonascii - md_nonascii) + (md_nonascii - gold_nonascii)
    report.add("G5 Nicht-ASCII", not diff_nonascii,
               f"PDF {dict(gold_nonascii)}, Extrakt {dict(md_nonascii)}, "
               f"Abweichung {dict(diff_nonascii) if diff_nonascii else 'keine'}")

    # ------------------------------------------------------------------ G6 Wortlaut
    report.add("G6 Wortlauttreue", ok_sequence and not mismatched,
               "programmatisch ueber die vollstaendige Wortfolge; "
               "Bildsichtpruefung fuehrt der Hauptagent separat durch")

    # ----------------------------------------------------------------- G7 Hausregeln
    em = [f"{name}: {read_text(OUT_DIR / name).count(chr(0x2014))}x"
          for name in EXPECTED_FILES if (OUT_DIR / name).exists()
          and "\u2014" in read_text(OUT_DIR / name)]
    report.add("G7 Geviertstrich", not em, "; ".join(em) if em else "kein U+2014 in 19 Dateien")

    naming = [n for n in present if not re.fullmatch(r"\d\d_[A-Za-z0-9_\-]+\.md", n)]
    report.add("G7 Dateinamen", not naming,
               "; ".join(naming) if naming else "alle Namen nach Konvention")

    venvs = sorted(str(p.relative_to(ROOT)) for p in ROOT.rglob("pyvenv.cfg"))
    report.add("G7 venv in Dropbox", not venvs,
               "; ".join(venvs) if venvs else "kein venv im Projektordner")

    # --------------------------------------------------------------- Z1 Absaetze
    confirmed_count = 0
    confirmed_examples: list[str] = []
    borderline: list[str] = []
    for name in NORM_FILES:
        paragraphs = [(kind, value) for kind, value in per_file.get(name, [])
                      if kind in ("head", "text")]
        for (prev_kind, prev), (_, following) in zip(paragraphs, paragraphs[1:]):
            if prev_kind == "head":
                continue
            context = canon(prev)[-CONTEXT:] + canon(following)[:CONTEXT]
            gap = same_paragraph_contexts.get(context)
            if gap is None:
                continue
            if gap <= IN_PARAGRAPH_GAP:
                confirmed_count += 1
                if len(confirmed_examples) < 6:
                    confirmed_examples.append(f"{name}: ...{prev[-34:]!r} + {following[:34]!r}")
            else:
                borderline.append(f"{name}: Abstand {gap:.1f} pt, ...{prev[-28:]!r} + "
                                  f"{following[:28]!r}")
    report.add("Z1 Absatzbruch im Absatz", confirmed_count == 0,
               f"{confirmed_count} belegte Stellen, an denen die Quelle denselben Absatz "
               f"fortsetzt, der Extrakt aber trennt; Beispiele: {confirmed_examples}")
    report.note(f"Grenzfaelle ohne Entscheidung (Abstand zwischen {IN_PARAGRAPH_GAP} und "
                f"{PARAGRAPH_GAP} pt): {borderline if borderline else 'keine'}")
    report.note(f"Absatzeinheiten im Extrakt: {sum(paragraph_kinds.values())} "
                f"(Ueberschriften {paragraph_kinds['head']}, Absaetze {paragraph_kinds['text']})")

    # --------------------------------------------------------------- Z2 Fussnoten
    report.add("Z2 Fussnoten", sorted(pdf_note_markers) == sorted(md_notes),
               f"PDF {len(pdf_note_markers)} Marken, Extrakt {len(md_notes)} Eintraege; "
               f"nur im PDF {sorted(set(pdf_note_markers) - set(md_notes))[:12]}; "
               f"nur im Extrakt {sorted(set(md_notes) - set(pdf_note_markers))[:12]}")

    # ----------------------------------------------------------- Z3 Indexgroessen
    index = OUT_DIR / INDEX_FILE
    rows_checked = 0
    bad: list[str] = []
    if index.exists():
        for raw in read_text(index).splitlines():
            cells = [cell.strip() for cell in raw.split("|")[1:-1]]
            if len(cells) < 4 or not cells[0].strip("`").endswith(".md"):
                continue
            rows_checked += 1
            name = cells[0].strip("`")
            actual = (OUT_DIR / name).stat().st_size if (OUT_DIR / name).exists() else -1
            if not re.fullmatch(r"\d+", cells[-1]) or int(cells[-1]) != actual:
                bad.append(f"{name}: Index {cells[-1]} gegen Datei {actual}")
    report.add("Z3 Indexgroessen", rows_checked > 0 and not bad,
               f"{rows_checked} Zeilen; " + ("; ".join(bad) if bad else "alle Groessen stimmen"))

    # ---------------------------------------------------------------------- Bericht
    print("Verifikation des CRA-Volltextextrakts gegen die PDF-Textebene")
    print(f"PDF   : {PDF_PATH.name}, {page_count} Seiten")
    print(f"Ordner: {OUT_DIR.relative_to(ROOT)}")
    print()
    width = max(len(row[0]) for row in report.rows)
    for gate, passed, message in report.rows:
        print(f"  [{'ok  ' if passed else 'FEHL'}] {gate.ljust(width)}  {message}")
    print()
    for note in report.notes:
        print(f"  Hinweis: {note}")
    print()
    if report.failed:
        print(f"ERGEBNIS: {len(report.failed)} von {len(report.rows)} Pruefungen nicht bestanden.")
        return 1
    print(f"ERGEBNIS: alle {len(report.rows)} Pruefungen bestanden.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
