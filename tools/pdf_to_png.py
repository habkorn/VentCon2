"""Wandelt die Abbildungen des Benutzerhandbuchs in PNG fuer die Praesentation um.

Hintergrund: `build_ventrex_deck.py` kann keine PDF einbetten, PowerPoint braucht
Rasterbilder. Die Quelle bleibt das Handbuch unter `LaTeX/manual/pic`, die PNG liegen
als Erzeugnis unter `sec/pic`.

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/pdf_to_png.py
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" tools/pdf_to_png.py --long-edge 2000

Ergebnis: `sec/pic/<Name>.png`, lange Kante 1600 Pixel (Vorgabe), hoechstens 400 dpi.
"""
import argparse
import pathlib
import sys

import pymupdf

SOURCE = pathlib.Path("LaTeX/manual/pic")
TARGET = pathlib.Path("sec/pic")
FIGURES = ["SchematicHardware", "BrowserApp"]
MAX_DPI = 400.0


def convert(name, long_edge):
    pdf = SOURCE / (name + ".pdf")
    if not pdf.exists():
        raise SystemExit(f"Quelle fehlt: {pdf}")
    doc = pymupdf.open(pdf)
    page = doc[0]
    long_pt = max(page.rect.width, page.rect.height)
    scale = long_edge / long_pt
    if scale * 72.0 > MAX_DPI:
        scale = MAX_DPI / 72.0
    pix = page.get_pixmap(matrix=pymupdf.Matrix(scale, scale), alpha=False)
    TARGET.mkdir(parents=True, exist_ok=True)
    out = TARGET / (name + ".png")
    pix.save(out)
    doc.close()
    return out, pix.width, pix.height, scale * 72.0


def main():
    parser = argparse.ArgumentParser(description="Handbuch-Abbildungen nach PNG")
    parser.add_argument("--long-edge", type=int, default=1600,
                        help="Zielauflösung der langen Kante in Pixeln (Vorgabe 1600)")
    args = parser.parse_args()
    for name in FIGURES:
        out, width, height, dpi = convert(name, args.long_edge)
        size_kb = out.stat().st_size / 1024.0
        print(f"{name:20} {width} x {height} px, {dpi:5.0f} dpi, {size_kb:7.1f} KB -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
