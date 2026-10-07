# -*- coding: utf-8 -*-
"""Negativkontrolle für die Prüfung auf Vorwärtsverweise.

Erzeugt aus der fertigen Präsentation eine Kontrolldatei, in die absichtlich
zwei Vorwärtsverweise eingebaut werden: ein Verweis auf die spätere Folie 6.1 und
die Kennung P4.3 auf einer frühen Folie. Die Kontrolldatei muss von
verify_ventrex_deck.py mit genau diesen beiden Fehlern abgelehnt werden.

Aufruf (schreibt nur die Kontrolldatei, ändert das Deck nicht):
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/forward_ref_control.py
"""

import shutil
import sys

from pptx import Presentation

DECK = sys.argv[1] if len(sys.argv) > 1 else "sec/2026-10-06_CRA_Compliance_Praesentation.pptx"
CONTROL = "sec/_control_forward_ref.pptx"

shutil.copyfile(DECK, CONTROL)
prs = Presentation(CONTROL)
slides = list(prs.slides)

# Folie 4 (1.1 Systemübersicht): Verweis auf die spätere Folie 6.1
target = slides[3]
for shape in target.shapes:
    if not getattr(shape, "has_text_frame", False) or not shape.has_text_frame:
        continue
    frame = shape.text_frame
    if not frame.text.strip():
        continue
    paragraph = frame.paragraphs[0]
    run = paragraph.add_run()
    run.text = " Details siehe 6.1"

# Folie 5 (1.2 Bedienoberfläche): Kennung, die erst ab Folie 16 erklärt wird
target = slides[4]
for shape in target.shapes:
    if not getattr(shape, "has_text_frame", False) or not shape.has_text_frame:
        continue
    frame = shape.text_frame
    if not frame.text.strip():
        continue
    paragraph = frame.add_paragraph()
    run = paragraph.add_run()
    run.text = "Offen ist P4.3."
    break

prs.save(CONTROL)
print("Kontrolldatei geschrieben:", CONTROL)
