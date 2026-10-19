"""Verkleinert das gebaute Benutzerhandbuch für die Auslieferung über LittleFS.

Das Handbuch wird mit XeLaTeX in `LaTeX/manual` in Druckqualität gebaut (rund 3 MB,
überwiegend Screenshots). Der ESP32 liefert die Datei aus dem LittleFS-Image aus,
dort ist nur Platz für die verkleinerte Fassung `data/VentCon_User_Manual.pdf`.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/shrink_manual_pdf.py
    python tools/shrink_manual_pdf.py --dpi 50 --quality 65

Ergebnis: `data/VentCon_User_Manual.pdf`, Vorgabe 60 dpi und JPEG-Qualität 65.
"""
import argparse
import pathlib
import sys

import pymupdf

SOURCE = pathlib.Path("LaTeX/manual/VentCon_User_Manual.pdf")
TARGET = pathlib.Path("data/VentCon_User_Manual.pdf")
DEFAULT_DPI = 60
DEFAULT_THRESHOLD = 72
DEFAULT_QUALITY = 65


def pattern_keys(doc):
    """Sammelt alle /Pattern-Einträge in Seiten- und XObject-Ressourcen.

    `Document.rewrite_images` baut die Ressourcen-Dictionaries neu auf und lässt
    dabei /Pattern fallen. Die Inhaltsströme verweisen aber weiter auf die
    Pattern-Namen (z. B. im Schatten des Apply-Buttons), deshalb werden die
    Einträge nach dem Verkleinern wieder eingesetzt.
    """
    saved = []
    for xref in range(1, doc.xref_length()):
        for key in ("Pattern", "Resources/Pattern"):
            try:
                kind, value = doc.xref_get_key(xref, key)
            except Exception:
                continue
            if kind == "dict":
                saved.append((xref, key, value))
    return saved


def main():
    parser = argparse.ArgumentParser(description="Benutzerhandbuch für LittleFS verkleinern")
    parser.add_argument("--source", type=pathlib.Path, default=SOURCE,
                        help="gebautes Handbuch (Vorgabe: LaTeX/manual)")
    parser.add_argument("--target", type=pathlib.Path, default=TARGET,
                        help="Zieldatei im LittleFS-Verzeichnis (Vorgabe: data/)")
    parser.add_argument("--dpi", type=int, default=DEFAULT_DPI,
                        help=f"Zielauflösung der Bilder in dpi (Vorgabe: {DEFAULT_DPI})")
    parser.add_argument("--dpi-threshold", type=int, default=DEFAULT_THRESHOLD,
                        help="nur Bilder oberhalb dieser Auflösung verkleinern "
                             f"(Vorgabe: {DEFAULT_THRESHOLD})")
    parser.add_argument("--quality", type=int, default=DEFAULT_QUALITY,
                        help=f"JPEG-Qualität 0-100 (Vorgabe: {DEFAULT_QUALITY})")
    args = parser.parse_args()

    if not args.source.is_file():
        raise SystemExit(f"Quelle fehlt: {args.source}")

    doc = pymupdf.open(args.source)
    pages = doc.page_count
    patterns = pattern_keys(doc)
    doc.rewrite_images(dpi_threshold=args.dpi_threshold, dpi_target=args.dpi,
                       quality=args.quality)
    for xref, key, value in patterns:
        if doc.xref_get_key(xref, key)[0] == "null":
            doc.xref_set_key(xref, key, value)
    args.target.parent.mkdir(parents=True, exist_ok=True)
    doc.save(args.target, garbage=3, deflate=True)
    doc.close()

    before = args.source.stat().st_size
    after = args.target.stat().st_size
    print(f"Quelle: {args.source} ({pages} Seiten, {before / 1024:.0f} KB)")
    print(f"Ziel:   {args.target} ({after / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
