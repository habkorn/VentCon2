# -*- coding: utf-8 -*-
"""Extrahiert die Aalberts-Wordvorlagen in Markdown, eine Datei je Dokument.

Der Extrakt bleibt englisch, weil die Vorlagen englisch sind und rechtlich relevant bleiben
sollen. Jede Ausgabedatei beginnt mit einem Kopfblock zwischen den Marken
`<!-- extract:head-start -->` und `<!-- extract:head-end -->`, danach folgt der Inhalt in
Dokumentreihenfolge, am Ende steht der Anhang zwischen `<!-- extract:appendix-start -->` und
`<!-- extract:appendix-end -->`. Die Marken erlauben dem Verifier, Inhalt und Zugaben zu trennen.

Abgedeckt werden: Absätze mit Titel- und Überschriftenformaten, nummerierte Klauseln mit
rekonstruierten Zählern, Tabellen, Textboxen der Kopf- und Fußzeile, Kommentare und die
Bildbestandsaufnahme. Nicht übernommen werden Layout, Logo und Seitenzahlenfelder.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/docx_extract.py
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/docx_extract.py sec/AalbertsChecklists/<Datei>.docx
"""

from __future__ import annotations

import hashlib
import re
import sys
import zipfile
from datetime import date
from pathlib import Path
from xml.etree import ElementTree

from docx import Document
from docx.table import Table
from docx.text.paragraph import Paragraph

ROOT = Path(__file__).resolve().parent.parent
SOURCE_DIR = ROOT / "sec" / "AalbertsChecklists"
W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"

# Ablage neben der Quelle, Name aus dem Dokumenttitel (Entscheidung E43).
OUTPUT_NAMES = {
    "CRA Compliance Addendum (Supplier Agreement) - template Sep 2026 .docx":
        "CRA_Compliance_Addendum_Supplier_Agreement.md",
    "CRA product maturity checklist - Sep 2026.docx":
        "CRA_Product_Maturity_Checklist.md",
    "CRA- Supplier Due Diligence Questionnaire - template Sep 2026.docx":
        "CRA_Supplier_Due_Diligence_Questionnaire.md",
}

HEAD_START = "<!-- extract:head-start -->"
HEAD_END = "<!-- extract:head-end -->"
APPENDIX_START = "<!-- extract:appendix-start -->"
APPENDIX_END = "<!-- extract:appendix-end -->"


def output_name(path: Path) -> str:
    """Zieldatei zur Quelldatei, sonst Name aus dem Dokumenttitel."""
    if path.name in OUTPUT_NAMES:
        return OUTPUT_NAMES[path.name]
    stem = re.sub(r"\b(template|Sep|Okt|Oct)\b", " ", path.stem)
    stem = re.sub(r"\b20\d\d\b", " ", stem)
    words = re.findall(r"[A-Za-z0-9]+", stem)
    return "_".join(w if w.isupper() else w.capitalize() for w in words) + ".md"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def number_formats(path: Path) -> dict[int, dict[int, tuple[str, int, str]]]:
    """Nummerierungsformate je numId und Ebene aus word/numbering.xml."""
    with zipfile.ZipFile(path) as archive:
        if "word/numbering.xml" not in archive.namelist():
            return {}
        root = ElementTree.fromstring(archive.read("word/numbering.xml"))
    abstracts: dict[int, dict[int, tuple[str, int, str]]] = {}
    for abstract in root.findall(W + "abstractNum"):
        abstract_id = int(abstract.get(W + "abstractNumId", "0"))
        levels = {}
        for lvl in abstract.findall(W + "lvl"):
            ilvl = int(lvl.get(W + "ilvl", "0"))
            fmt = lvl.find(W + "numFmt")
            start = lvl.find(W + "start")
            text = lvl.find(W + "lvlText")
            levels[ilvl] = (
                fmt.get(W + "val", "decimal") if fmt is not None else "decimal",
                int(start.get(W + "val", "1")) if start is not None else 1,
                text.get(W + "val", "%1.") if text is not None else "%1.",
            )
        abstracts[abstract_id] = levels
    mapping: dict[int, dict[int, tuple[str, int, str]]] = {}
    for num in root.findall(W + "num"):
        num_id = int(num.get(W + "numId", "0"))
        ref = num.find(W + "abstractNumId")
        if ref is not None:
            mapping[num_id] = abstracts.get(int(ref.get(W + "val", "0")), {})
    return mapping


def numbering_caption(levels: dict[int, tuple[str, int, str]], ilvl: int,
                      counters: dict[int, int]) -> str:
    """Beschriftung eines nummerierten Absatzes aus Format und Zählern."""
    fmt, start, pattern = levels.get(ilvl, ("decimal", 1, "%1."))
    if fmt == "bullet":
        return "-"
    value = counters.get(ilvl, start)
    letters = "abcdefghijklmnopqrstuvwxyz"
    if fmt == "lowerLetter":
        rendered = letters[(value - 1) % 26]
    elif fmt == "upperLetter":
        rendered = letters[(value - 1) % 26].upper()
    elif fmt == "lowerRoman":
        rendered = "%d" % value
    elif fmt == "upperRoman":
        rendered = "%d" % value
    else:
        rendered = str(value)
    caption = pattern
    for level in range(0, ilvl + 1):
        token = "%" + str(level + 1)
        if token in caption:
            caption = caption.replace(token, str(counters.get(level, start)))
    if "%" in caption:
        caption = caption.replace("%1", rendered)
    caption = caption.strip()
    return caption if caption.endswith(".") or caption == "-" else caption + "."


def block_items(doc: Document):
    """Absätze und Tabellen in Dokumentreihenfolge."""
    for child in doc.element.body.iterchildren():
        if child.tag == W + "p":
            yield Paragraph(child, doc)
        elif child.tag == W + "tbl":
            yield Table(child, doc)


def paragraph_text(paragraph) -> str:
    """Absatztext mit Zeilenumbrüchen als `<br>`, damit weiche Umbrüche erhalten bleiben."""
    parts = []
    for node in paragraph._p.iter():
        if node.tag == W + "t":
            parts.append(node.text or "")
        elif node.tag in (W + "br", W + "cr"):
            parts.append("<br>")
        elif node.tag == W + "tab":
            parts.append(" ")
    chunks = [" ".join(chunk.split()) for chunk in "".join(parts).split("<br>")]
    return "<br>".join(chunk for chunk in chunks if chunk)


def cell_text(cell) -> str:
    parts = []
    for paragraph in cell.paragraphs:
        text = paragraph_text(paragraph)
        if text:
            parts.append(text)
    return "<br>".join(parts)


def render_table(table: Table) -> list[str]:
    rows = [[cell_text(cell).replace("|", "\\|") for cell in row.cells] for row in table.rows]
    if not rows:
        return []
    width = max(len(row) for row in rows)
    lines = ["| " + " | ".join(row + [""] * (width - len(row))) + " |" for row in rows]
    separator = "|" + "|".join([" --- "] * width) + "|"
    return [lines[0], separator] + lines[1:] + [""]


def part_texts(path: Path, prefix: str) -> list[str]:
    """Texte aus Kopf- oder Fußzeilenteilen, je Textbox getrennt und ohne Dopplungen."""
    texts = []
    with zipfile.ZipFile(path) as archive:
        names = [n for n in archive.namelist()
                 if n.startswith("word/" + prefix) and n.endswith(".xml")]
        for name in sorted(names):
            root = ElementTree.fromstring(archive.read(name))
            boxes = list(root.iter(W + "txbxContent"))
            if not boxes:
                boxes = [root]
            for box in boxes:
                chunks = [node.text or "" for node in box.iter(W + "t")]
                text = " ".join("".join(chunks).split())
                if text and text not in texts:
                    texts.append(text)
    return texts


def comments(path: Path) -> list[tuple[str, str, str]]:
    """Kommentare als (Autor, Datum, Text)."""
    found = []
    with zipfile.ZipFile(path) as archive:
        if "word/comments.xml" not in archive.namelist():
            return found
        root = ElementTree.fromstring(archive.read("word/comments.xml"))
    for comment in root.findall(W + "comment"):
        chunks = ["".join(node.text or "" for node in paragraph.iter(W + "t"))
                  for paragraph in comment.iter(W + "p")]
        text = " ".join(" ".join(chunks).split())
        if text:
            found.append((comment.get(W + "author", ""), comment.get(W + "date", ""), text))
    return found


def image_names(path: Path) -> list[str]:
    with zipfile.ZipFile(path) as archive:
        return [n.split("/")[-1] for n in archive.namelist() if n.startswith("word/media/")]


def heading_level(style: str, seen_title: bool) -> int:
    """Überschriftentiefe aus dem Absatzformat."""
    match = re.match(r"Heading (\d+)", style)
    if match:
        return min(3, int(match.group(1)) + 1)
    if style.strip().lower() == "title":
        return 1 if not seen_title else 2
    return 0


def extract(path: Path) -> str:
    doc = Document(str(path))
    formats = number_formats(path)
    counters: dict[tuple[int, int], int] = {}
    lines: list[str] = []
    seen_title = False
    paragraphs = 0
    tables = 0
    numbered = 0

    for block in block_items(doc):
        if isinstance(block, Table):
            tables += 1
            lines.extend(render_table(block))
            continue
        text = paragraph_text(block)
        if not text:
            continue
        paragraphs += 1
        num_pr = block._p.find(".//" + W + "numPr")
        level = heading_level(block.style.name, seen_title)
        if num_pr is not None:
            num_id_node = num_pr.find(W + "numId")
            ilvl_node = num_pr.find(W + "ilvl")
            num_id = int(num_id_node.get(W + "val", "0")) if num_id_node is not None else 0
            ilvl = int(ilvl_node.get(W + "val", "0")) if ilvl_node is not None else 0
            levels = formats.get(num_id, {})
            key = (num_id, ilvl)
            if key in counters:
                counters[key] += 1
            else:
                counters[key] = levels.get(ilvl, ("decimal", 1, "%1."))[1]
            for other in [k for k in counters if k[0] == num_id and k[1] > ilvl]:
                del counters[other]
            caption = numbering_caption(levels, ilvl, {lvl: counters[(num_id, lvl)]
                                                       for (nid, lvl) in counters if nid == num_id})
            numbered += 1
            lines.append("%s%s %s" % ("  " * ilvl, caption, text))
            lines.append("")
            continue
        if level:
            seen_title = seen_title or level == 1
            lines.append("%s %s" % ("#" * level, text.replace("<br>", " ")))
        else:
            lines.append(text)
        lines.append("")

    header = part_texts(path, "header")
    footer = part_texts(path, "footer")
    images = image_names(path)
    found = comments(path)

    props = doc.core_properties
    head = [HEAD_START, "## Extraction record", "", "| Field | Value |", "| --- | --- |",
            "| Source file | `%s` |" % rel(path),
            "| SHA256 | `%s` |" % sha256(path),
            "| Author | %s |" % (props.author or "not specified"),
            "| Created | %s |" % (props.created or "not specified"),
            "| Modified | %s |" % (props.modified or "not specified"),
            "| Extracted | %s |" % date.today().isoformat(),
            "| Tool | tools/docx_extract.py, python-docx |",
            "| Scope | paragraphs: %d, tables: %d, numbered paragraphs: %d, images: %d |"
            % (paragraphs, tables, numbered, len(images)),
            "| Formatting | headings follow the source paragraph styles (Title, Heading 1, "
            "Heading 2); clause numbers are reconstructed from the numbering definition |",
            "| Header text | %s |" % ("; ".join(header) if header else "none"),
            "| Footer text | %s |" % ("; ".join(footer) if footer else "none"),
            "| Not reproduced | company logo (%s), page layout, page number fields |"
            % (", ".join(images) if images else "none"),
            "| Note | Extract of the source document, English as in the source. "
            "The source file remains the reference. |",
            "", HEAD_END, ""]

    appendix = []
    if found:
        appendix = [APPENDIX_START, "## Reviewer comments from the source document", ""]
        for author, when, text in found:
            appendix.append("- %s (%s): %s" % (author or "unknown", when or "no date", text))
        appendix.extend(["", APPENDIX_END, ""])

    title = output_name(path)[:-3].replace("_", " ")
    return "\n".join(["# " + title, ""] + head + lines + appendix).rstrip() + "\n"


def main() -> int:
    arguments = sys.argv[1:]
    if arguments:
        targets = [Path(a) for a in arguments]
    else:
        # Befüllte Wordkopien bleiben außen vor, sonst entstehen Extrakte der Extrakte.
        targets = sorted(p for p in SOURCE_DIR.glob("*.docx")
                         if not p.stem.endswith("_answered"))
    if not targets:
        print("Keine Worddatei gefunden in", rel(SOURCE_DIR))
        return 1
    for path in targets:
        if not path.is_file():
            print("Datei fehlt:", path)
            return 1
        target = path.with_name(output_name(path))
        content = extract(path)
        target.write_text(content, encoding="utf-8")
        print("%-62s -> %s (%d Zeilen)" % (path.name, target.name,
                                           content.count("\n") + 1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
