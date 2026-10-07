# -*- coding: utf-8 -*-
"""Prüft die Umsetzung des Plans "Präsentation ohne Vorwärtsverweise" gegen die Artefakte.

Der Prüfer arbeitet unabhängig von `verify_ventrex_deck.py`: er liest die Zielbild-Tabelle
aus `sec/plan/2026-09-30_VENTREX_Praesentation_Plan.md` als Sollvorgabe und vergleicht sie
mit dem erzeugten Deck. Geprüft werden Reihenfolge, Layout und Nummer jeder Folie, die
Unterzeile auf jedem Divider, die entfernten Vorwärtsverweise, die Kennungen R1 bis R10 und
P1.1 bis P5.2 vor ihrer Erklärungsfolie und die Vollständigkeit des Inhaltsverzeichnisses.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/audit_plan_umsetzung.py
"""

import re
import sys

from pptx import Presentation
from pptx.enum.shapes import PP_PLACEHOLDER

PLAN = "sec/plan/2026-09-30_VENTREX_Praesentation_Plan.md"
DECK = "sec/2026-10-06_CRA_Compliance_Praesentation.pptx"
HEADING = "## Zielbild: 28 Folien in sieben Abschnitten"

# Kennungen erst ab der Folie, die sie erklärt (Zielreihenfolge der Umsortierung)
ID_FIRST = (
    ("R1 bis R10", re.compile(r"(?<![A-Za-z0-9])R(?:10|[1-9])(?![0-9])"), 14),
    ("P1.1 bis P3.3", re.compile(r"(?<![A-Za-z0-9])P[1-3]\.[1-9](?![0-9])"), 15),
    ("P4.1 bis P5.2", re.compile(r"(?<![A-Za-z0-9])P[4-5]\.[1-9](?![0-9])"), 16),
)

# Wendungen, die der Plan streicht (Vorwärtsverweise V1 bis V7 und zwei Folgen davon)
REMOVED = (
    "aus Abschnitt 5 weiter",
    "Befunde R1 und R4",
    "siehe 2.2 und 2.3",
    "die Folgen stehen auf 7.1",
    "Entscheidung in P1.5",
    "P1.1 bis P1.3 und P2.2 laufen weiter",
    "Befunde R1 bis R9, Maßnahmen P1.1 bis P5.2",
)

REF_KEYWORDS = ("siehe", "vgl", "nach ", "aus ", "in ", "auf ", "an ", "abschnitt", "kapitel")
REF_EXEMPT = ("Abschnitt 9.1", "303 645 5.1-1", "303 645 5.4-1")
REF_CANDIDATE = re.compile(r"(?<![A-Za-z0-9.(])([1-7](?:\.\d)?)(?![\d.\-)(])")
CITATION_BEFORE = re.compile(r"(?:Art\.|Nr\.|Abs\.|Annex|Teil)\s*$")

problems = []
checks = 0


def check(condition, message):
    global checks
    checks += 1
    if not condition:
        problems.append(message)


def plan_rows():
    """Zielbild-Tabelle als Liste (Nummer, Layout, Inhalt)."""
    lines = open(PLAN, encoding="utf-8").read().split("\n")
    start = lines.index(HEADING)
    rows = []
    for line in lines[start + 1:]:
        if not line.startswith("|"):
            if rows:
                break
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) != 3 or not cells[0].isdigit():
            continue
        rows.append((int(cells[0]), cells[1], cells[2]))
    return rows


def slide_parts(slide):
    """Titel, Unterzeile, Gesamttext einer Folie."""
    title = ""
    parts = []
    for shape in slide.shapes:
        if not getattr(shape, "has_text_frame", False) or not shape.has_text_frame:
            continue
        text = shape.text_frame.text.strip()
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.TITLE:
            title = text
            continue
        if shape.is_placeholder and shape.placeholder_format.type in (
                PP_PLACEHOLDER.FOOTER, PP_PLACEHOLDER.DATE, PP_PLACEHOLDER.SLIDE_NUMBER):
            continue
        if text:
            parts.append(text)
    for shape in slide.shapes:
        if getattr(shape, "has_table", False) and shape.has_table:
            for row in shape.table.rows:
                parts.extend(cell.text for cell in row.cells)
    return title, parts, "\n".join(parts)


def main():
    global checks
    rows = plan_rows()
    check(len(rows) == 28, "Zielbild-Tabelle hat %d Zeilen, erwartet 28" % len(rows))

    prs = Presentation(DECK)
    slides = list(prs.slides)
    check(len(slides) == 28, "Deck hat %d Folien, erwartet 28" % len(slides))

    titles = []
    subtitles = []
    texts = []
    layouts = []
    for slide in slides:
        title, parts, text = slide_parts(slide)
        titles.append(title)
        subtitles.append(parts)
        texts.append(text)
        layouts.append(slide.slide_layout.name)

    # 1 Reihenfolge, Layout und Nummer gegen die Zielbild-Tabelle
    for number, layout, content in rows:
        if number > len(slides):
            continue
        check(layouts[number - 1] == layout,
              "Folie %d: Layout %s, im Zielbild %s" % (number, layouts[number - 1], layout))
        match = re.match(r"^(\d+(?:\.\d+)?)\s", content)
        if not match:
            continue
        expected = match.group(1)
        check(titles[number - 1].startswith(expected + " "),
              "Folie %d: Titel %r beginnt nicht mit %s" % (number, titles[number - 1], expected))
        keyword = re.sub(r"[^\wÄÖÜäöüß]", " ", content[match.end():]).split()
        keyword = [w for w in keyword if len(w) > 3 and w not in ("oder", "sowie", "dazu")]
        if keyword:
            check(keyword[0] in titles[number - 1],
                  "Folie %d: %r fehlt im Titel %r" % (number, keyword[0], titles[number - 1]))

    # 2 Divider tragen eine sichtbare Unterzeile
    divider = 0
    for index, layout in enumerate(layouts, start=1):
        if not layout.startswith("SUBSECTION"):
            continue
        divider += 1
        text = " ".join(subtitles[index - 1]).strip()
        check(len(text) >= 15,
              "Divider Folie %d hat keine sichtbare Unterzeile" % index)
    check(divider == 7, "%d Divider gefunden, erwartet 7" % divider)

    # 3 Abschnitt Fristen und Sanktionen bleibt entfernt
    joined = "\n".join(texts)
    for token in ("Sanktionsrahmen", "Fristen und Sanktionen", "Zeitplan der Fristen"):
        check(token not in joined, "Abschnitt Fristen und Sanktionen wieder im Deck: " + token)

    # 4 Gestrichene Vorwärtsverweise sind weg
    for phrase in REMOVED:
        check(phrase not in joined, "Gestrichene Wendung noch im Deck: " + phrase)

    # 5 Keine Vorwärtsverweise auf Folien, die es gibt
    ref_slide = {}
    for index, title in enumerate(titles, start=1):
        match = re.match(r"^(\d+(?:\.\d+)?)\s", title)
        if match:
            ref_slide[match.group(1)] = index
    for index, text in enumerate(texts, start=1):
        if index == 2:
            continue
        for match in REF_CANDIDATE.finditer(text):
            window = text[max(0, match.start() - 20):match.end() + 3]
            before = text[max(0, match.start() - 20):match.start()]
            if any(token in window for token in REF_EXEMPT):
                continue
            if CITATION_BEFORE.search(before):
                continue
            if not any(keyword in before.lower() for keyword in REF_KEYWORDS):
                continue
            target = match.group(1)
            if target not in ref_slide:
                problems.append("Folie %d verweist auf %s, das keine eigene Folie hat"
                                % (index, target))
            elif ref_slide[target] > index:
                problems.append("Folie %d verweist auf die spätere Folie %d (%s)"
                                % (index, ref_slide[target], target))
            checks += 1

    # 6 Kennungen erst nach ihrer Erklärung
    for label, pattern, first in ID_FIRST:
        for index, text in enumerate(texts, start=1):
            if index >= first or index == 2:
                continue
            match = pattern.search(text)
            if match:
                problems.append("Folie %d nennt %s (%s) vor Folie %d"
                                % (index, label, match.group(0), first))
            checks += 1

    # 7 Inhaltsverzeichnis deckt jede betitelte Folie ab
    toc = {}
    for shape in slides[1].shapes:
        if not (getattr(shape, "has_table", False) and shape.has_table):
            continue
        for row in shape.table.rows:
            cells = [cell.text.strip() for cell in row.cells]
            if len(cells) == 4 and cells[0].isdigit():
                toc[int(cells[0])] = cells[1]
                toc[int(cells[2])] = cells[3]
    check(len(toc) == 28, "Inhaltsverzeichnis hat %d Einträge, erwartet 28" % len(toc))
    for index, title in enumerate(titles, start=1):
        if not title or index == 2:
            continue
        number = re.match(r"^(\d+(?:\.\d+)?)\s", title)
        entry = toc.get(index, "")
        check(bool(entry), "Folie %d fehlt im Inhaltsverzeichnis" % index)
        if number:
            check(entry.startswith(number.group(1) + " "),
                  "Inhaltsverzeichnis Folie %d: %r ohne Nummer %s"
                  % (index, entry, number.group(1)))
        words = re.findall(r"[A-Za-zÄÖÜäöüß]{4,}", re.sub(r"^\d+(?:\.\d+)?\s*", "", title))
        if words:
            check(words[0] in entry, "Inhaltsverzeichnis Folie %d: %r fehlt in %r"
                                     % (index, words[0], entry))

    print("Prüfung der Planumsetzung gegen %s" % PLAN)
    print("Deck: %s, %d Folien" % (DECK, len(slides)))
    print("Prüfungen: %d" % checks)
    if problems:
        print("\nABWEICHUNGEN (%d):" % len(problems))
        for item in problems:
            print("  -", item)
        return 1
    print("Alle Prüfungen bestanden.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
