# -*- coding: utf-8 -*-
"""Unabhängige Prüfung der Wordextrakte in `sec/AalbertsChecklists/extracts`.

Der Prüfer liest die Worddatei roh aus dem Paket (kein python-docx) und vergleicht sie mit der
erzeugten Markdown-Datei. Geprüft werden Textdeckung, Tabellenform, Absatzzahlen aus dem Kopfblock,
die Vollständigkeit der rekonstruierten Klauselnummern und Zusatztext außerhalb der erkannten
Struktur. Der Inhalt zwischen den Marken `extract:head-*` und `extract:appendix-*` gilt als Zugabe
des Extrakts und wird beim Textvergleich ausgeblendet.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/verify_docx_extract.py
"""

from __future__ import annotations

import hashlib
import re
import sys
import zipfile
from pathlib import Path
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parent.parent
BASE_DIR = ROOT / "sec" / "AalbertsChecklists"
TEMPLATE_DIR = BASE_DIR / "templates"
EXTRACT_DIR = BASE_DIR / "extracts"
W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"

HEAD_START = "<!-- extract:head-start -->"
HEAD_END = "<!-- extract:head-end -->"
APPENDIX_START = "<!-- extract:appendix-start -->"
APPENDIX_END = "<!-- extract:appendix-end -->"

problems: list[str] = []
checks = 0


def check(condition: bool, message: str) -> None:
    global checks
    checks += 1
    if not condition:
        problems.append(message)


def paragraph_text(paragraph) -> str:
    parts = []
    for node in paragraph.iter():
        if node.tag == W + "t":
            parts.append(node.text or "")
        elif node.tag in (W + "br", W + "cr"):
            parts.append("<br>")
        elif node.tag == W + "tab":
            parts.append(" ")
    chunks = [" ".join(chunk.split()) for chunk in "".join(parts).split("<br>")]
    return "<br>".join(chunk for chunk in chunks if chunk)


def raw_paragraph_texts(xml: bytes) -> list[str]:
    """Absätze direkt im Dokumentkörper, ohne Tabellenzellen und Textboxen."""
    root = ElementTree.fromstring(xml)
    body = root.find(W + "body")
    if body is None:
        return []
    texts = []
    for paragraph in body.findall(W + "p"):
        text = paragraph_text(paragraph)
        if text:
            texts.append(text)
    return texts


def part_texts(xml: bytes) -> list[str]:
    """Texte aus Kopf-, Fußzeilen- oder Kommentarteilen, je Textbox einzeln."""
    root = ElementTree.fromstring(xml)
    boxes = list(root.iter(W + "txbxContent")) or [root]
    texts = []
    for box in boxes:
        for paragraph in box.iter(W + "p"):
            text = paragraph_text(paragraph)
            if text and text not in texts:
                texts.append(text)
    return texts


def body_tables(xml: bytes) -> list[list[int]]:
    """Zeilenzahl je Tabelle im Dokumentkörper."""
    root = ElementTree.fromstring(xml)
    body = root.find(W + "body")
    shapes = []
    for table in body.findall(W + "tbl"):
        shapes.append(len(table.findall(W + "tr")))
    return shapes


def strip_additions(markdown: str) -> str:
    """Kopfblock und Anhang entfernen, damit nur der Extrakt geprüft wird."""
    markdown = re.sub(re.escape(HEAD_START) + r".*?" + re.escape(HEAD_END), "", markdown,
                      flags=re.S)
    markdown = re.sub(re.escape(APPENDIX_START) + r".*?" + re.escape(APPENDIX_END), "", markdown,
                      flags=re.S)
    return markdown


def normalize(text: str) -> str:
    return " ".join(text.replace("<br>", " ").replace("|", " ").split())


def md_table_shapes(markdown: str) -> list[int]:
    """Anzahl der Zeilen je Markdown-Tabelle, Kopfzeile und Trennzeile eingeschlossen."""
    shapes = []
    lines = 0
    for line in markdown.split("\n"):
        if line.startswith("|"):
            lines += 1
        elif lines:
            shapes.append(lines)
            lines = 0
    if lines:
        shapes.append(lines)
    return shapes


def blank_pattern(text: str):
    """Muster für eine Vorlagenzeile mit Leerstellen, die im Extrakt gefüllt sein dürfen.

    Die Antwortstrecke trägt Werte in die Leerzeilen der Vorlage ein (Entscheidung E76). Der
    Extrakt enthält dann an Stelle der Unterstriche den Wert, deshalb wird hier ein Muster
    gebildet, in dem jede Unterstrichgruppe für beliebigen Text steht.
    """
    normalized = normalize(text)
    if "_" not in normalized:
        return None
    parts = [re.escape(part) for part in re.split(r"_+", normalized)]
    return re.compile(".*?".join(parts))


def main() -> int:
    sources = sorted(p for p in TEMPLATE_DIR.glob("*.docx")
                     if not p.stem.endswith("_answered"))
    check(bool(sources), "Keine Worddatei in %s gefunden" % TEMPLATE_DIR.name)
    if not sources:
        return 1
    print("Prüfung der Wordextrakte gegen die Quelle")
    for path in sources:
        markdown_path = None
        for candidate in sorted(EXTRACT_DIR.glob("*.md")):
            if path.name in candidate.read_text(encoding="utf-8"):
                markdown_path = candidate
                break
        check(markdown_path is not None, "Kein Extrakt verweist auf %s" % path.name)
        if markdown_path is None:
            continue
        markdown = markdown_path.read_text(encoding="utf-8")
        body = strip_additions(markdown)
        normalized_md = normalize(body)

        with zipfile.ZipFile(path) as archive:
            document_xml = archive.read("word/document.xml")
            parts = [archive.read(n) for n in archive.namelist()
                     if n.startswith("word/") and (n.startswith("word/header")
                                                   or n.startswith("word/footer")
                                                   or n == "word/comments.xml")
                     and n.endswith(".xml")]
        body_texts = raw_paragraph_texts(document_xml)
        extra_texts = []
        for xml in parts:
            extra_texts.extend(part_texts(xml))

        missing = [text for text in body_texts
                   if normalize(text) not in normalized_md
                   and not (blank_pattern(text) and blank_pattern(text).search(normalized_md))]
        check(not missing, "%s: %d Absätze des Körpers fehlen im Extrakt, erste: %s"
              % (path.name, len(missing), missing[:2]))

        # Kopf-, Fuß- und Kommentartexte stehen im Kopfblock oder im Anhang des Extrakts.
        normalized_full = normalize(markdown)
        missing_extra = [text for text in extra_texts if normalize(text) not in normalized_full]
        check(not missing_extra,
              "%s: %d Texte aus Kopf-, Fuß- oder Kommentarteil fehlen, erste: %s"
              % (path.name, len(missing_extra), missing_extra[:2]))

        table_rows = body_tables(document_xml)
        md_shapes = md_table_shapes(body)
        check([rows + 1 for rows in table_rows] == md_shapes,
              "%s: Tabellenzeilen weichen ab, Quelle %s (plus Kopfzeile und Trennzeile), "
              "Extrakt %s" % (path.name, table_rows, md_shapes))

        # Umfang aus dem Kopfblock gegen die Quelle
        head = markdown.split(HEAD_END)[0]
        match = re.search(r"\| Scope \| paragraphs: (\d+), tables: (\d+)", head)
        check(match is not None, "%s: Umfang im Kopfblock fehlt" % path.name)
        if match:
            check(int(match.group(2)) == len(table_rows),
                  "%s: Tabelle im Kopfblock nennt %s, Quelle hat %d"
                  % (path.name, match.group(2), len(table_rows)))
            check(int(match.group(1)) == len(body_texts),
                  "%s: Absätze im Kopfblock nennt %s, Körper hat %d"
                  % (path.name, match.group(1), len(body_texts)))

        # Prüfsumme der Quelle
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        check(digest in markdown, "%s: SHA256 der Quelle fehlt im Kopfblock" % path.name)

        # Nummerierte Klauseln fortlaufend
        numbers = re.findall(r"^(\d+)\. ", body, flags=re.M)
        if numbers:
            expected = [str(n) for n in range(1, len(numbers) + 1)]
            check(numbers == expected,
                  "%s: Klauselnummern nicht fortlaufend: %s" % (path.name, numbers))
            print("  %-58s %3d Absätze, %2d Tabellen, Klauseln %s bis %s, SHA256 ok"
                  % (markdown_path.name, len(body_texts), len(table_rows), numbers[0],
                     numbers[-1]))
        else:
            print("  %-58s %3d Absätze, %2d Tabellen, SHA256 ok"
                  % (markdown_path.name, len(body_texts), len(table_rows)))

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
