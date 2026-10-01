# -*- coding: utf-8 -*-
"""Gegenproben für die neuen CRA-Regeln des Verifiers.

Der Verifier prüft seit 2026-10-01 fünf zusätzliche Regeln (V1 bis V5). Dieses Werkzeug
baut nacheinander je eine Kontrolldatei mit einer absichtlichen Verletzung, lässt den
Verifier darauf laufen und prüft, ob genau die erwartete Meldung erscheint. Danach wird
die Kontrolldatei sofort gelöscht, damit nur eine Kopie gleichzeitig existiert.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/cra_rule_control.py
"""

import os
import shutil
import subprocess
import sys

from pptx import Presentation

DECK = "sec/2026-10-01_CRA_Compliance_Praesentation.pptx"
MASTER = "sec/CRA_Compliance_Checklist.md"
DECK_CONTROL = "sec/_control_rule.pptx"
MASTER_CONTROL = "sec/_control_master.md"
PYTHON = sys.executable or "python"

# (Name, Suche, Ersatz, erwartete Meldung)
DECK_VARIANTS = [
    ("V1 Fristen ab Kenntnis", "Fristen laufen ab Kenntnis", "Fristen laufen ab Bereitstellung",
     "Fristen an die Bereitstellung geknüpft"),
    ("V2 Absatz existiert", "Art. 13(8) und (19)", "Art. 13(29) und (19)",
     "Art. 13 hat keinen Absatz (29)"),
    ("V3 Anhangspunkt existiert", "nächster Punkt (2)(k)", "nächster Punkt (2)(z)",
     "Annex I Teil I hat keinen Punkt (2)(z)"),
    ("V4 Projektlisten-Hinweis", "Projektliste:", "Hinweis:",
     "ohne Hinweis, dass es die Zählung des Dossiers ist"),
    ("V5 Vorbehalt Art. 4(4)", "Art. 4(3) und Art. 4(4), Recital 37", "Art. 4(3), Recital 37",
     "Vorbehalt nach Art. 4(4) fehlt"),
    ("V6 Roter Faden", "Aus dem Regelwerk folgt die Einordnung", "Die Einordnung",
     "ohne Übergang zum vorigen Abschnitt"),
    ("V7 Auslöser der Maßnahmen", "Umzusetzen mit der Bereitstellung auf dem Unionsmarkt",
     "Umzusetzen nach der Marktentscheidung",
     "ohne Hinweis, dass sie mit der EU-Bereitstellung umzusetzen sind"),
]

# Der Master wird als zweites Argument des Verifiers übergeben.
MASTER_VARIANT = (
    "V1 Master Fristen ab Kenntnis", "Die Fristen knüpfen an die Kenntnis des Herstellers an.",
    "Die Fristen laufen ab Bereitstellung auf dem Unionsmarkt.",
    "Master: Fristen an die Bereitstellung geknüpft",
)


def replace_text(prs, old, new):
    """Ersetzt Text in allen Textrahmen und Tabellenzellen, auch über Runs hinweg."""
    hits = 0
    for slide in prs.slides:
        for shape in slide.shapes:
            frames = []
            if getattr(shape, "has_text_frame", False) and shape.has_text_frame:
                frames.append(shape.text_frame)
            if getattr(shape, "has_table", False) and shape.has_table:
                for row in shape.table.rows:
                    for cell in row.cells:
                        frames.append(cell.text_frame)
            for frame in frames:
                for paragraph in frame.paragraphs:
                    text = "".join(run.text for run in paragraph.runs)
                    if old not in text or not paragraph.runs:
                        continue
                    paragraph.runs[0].text = text.replace(old, new)
                    for run in paragraph.runs[1:]:
                        run.text = ""
                    hits += 1
    return hits


def run_verifier(deck=DECK, master=MASTER):
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    result = subprocess.run([PYTHON, "verify_ventrex_deck.py", deck, master],
                            capture_output=True, text=True, encoding="utf-8", errors="replace",
                            env=env)
    return (result.stdout or "") + (result.stderr or "")


def main():
    if not os.path.isfile(DECK):
        print("Deck fehlt:", DECK)
        return 1

    failures = 0
    for name, old, new, expected in DECK_VARIANTS:
        shutil.copyfile(DECK, DECK_CONTROL)
        prs = Presentation(DECK_CONTROL)
        hits = replace_text(prs, old, new)
        if not hits:
            print("FEHLER %s: Suchtext nicht gefunden: %s" % (name, old))
            failures += 1
            os.remove(DECK_CONTROL)
            continue
        prs.save(DECK_CONTROL)
        output = run_verifier(DECK_CONTROL)
        os.remove(DECK_CONTROL)
        ok = expected in output
        print("%s %s (%d Stelle(n), erwartet: %s)" % ("OK  " if ok else "FEHLER", name, hits,
                                                      expected))
        if not ok:
            failures += 1
            for line in output.split("\n"):
                if "FEHLER" in line or line.strip().startswith("- "):
                    print("        " + line.strip())

    # Der Master wird über das zweite Argument geprüft, dafür genügt eine Textkopie.
    name, old, new, expected = MASTER_VARIANT
    text = open(MASTER, encoding="utf-8").read()
    if old not in text:
        print("FEHLER %s: Suchtext nicht gefunden: %s" % (name, old))
        failures += 1
    else:
        open(MASTER_CONTROL, "w", encoding="utf-8").write(text.replace(old, new))
        output = run_verifier(DECK, MASTER_CONTROL)
        os.remove(MASTER_CONTROL)
        ok = expected in output
        print("%s %s (erwartet: %s)" % ("OK  " if ok else "FEHLER", name, expected))
        if not ok:
            failures += 1

    print("\nGegenproben: %d, Fehler: %d" % (len(DECK_VARIANTS) + 1, failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
