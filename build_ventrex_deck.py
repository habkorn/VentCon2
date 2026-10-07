# -*- coding: utf-8 -*-
"""
Erzeugt die CRA-Präsentation aus der Firmenvorlage.

Vorlage:  sec/VENTREX PPT Vorlage 2026.pptx
Quelle:   sec/CRA_Compliance_Checklist.md (Master, Stand 2026-10-01)
Ziel:     sec/2026-10-06_CRA_Compliance_Praesentation.pptx

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" build_ventrex_deck.py
"""

import copy
import os
import re
import sys

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import PP_PLACEHOLDER
from pptx.enum.text import MSO_ANCHOR
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt

TEMPLATE = "sec/VENTREX PPT Vorlage 2026.pptx"
OUT = sys.argv[1] if len(sys.argv) > 1 else "sec/2026-10-06_CRA_Compliance_Praesentation.pptx"

FOOTER_NAME = "VentCon2 - EU CRA Compliance"
FOOTER_DATE = "06.10.2026"

# Markenschriften, damit auch Master mit Calibri-Theme markenkonform rendern
HEAD_FONT = "Clash Display"
BODY_FONT = "Satoshi"

# Markenpalette aus dem VENTREX-Theme
BRAND = RGBColor(0x00, 0x2F, 0x87)
GREEN = RGBColor(0x32, 0xC0, 0x9D)
CORAL = RGBColor(0xE9, 0x6D, 0x5A)
GRAY = RGBColor(0x53, 0x58, 0x63)
OLIVE = RGBColor(0x85, 0x7B, 0x48)
DARK = RGBColor(0x19, 0x1A, 0x1F)
ALT = RGBColor(0xEB, 0xEB, 0xEB)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

GRID_STYLE = "{5940675A-B579-460E-94D1-54222C63F5DA}"   # No Style, Table Grid
LEFT = 0.39
CONTENT_TOP = 1.45

# Kopfzeilenverweis je Abschnitt, gesetzt auf den Unterfolien
CHAPTERS = {
    "1": "1 Systembeschreibung",
    "2": "2 Regelwerk",
    "3": "3 Produkt und Anwendbarkeit",
    "4": "4 Risiken und Maßnahmen",
    "5": "5 Pflichten der CRA",
    "6": "6 Dokumente und Quellen",
    "7": "7 Fazit und nächste Schritte",
}

MASTER_BY_PREFIX = {
    "COVER": 0,
    "SUBSECTION": 1,
    "CONTENTDARK": 3,
    "CLOSING SLIDE": 5,
}


# --------------------------------------------------------------------------- #
# Aufbauhilfen
# --------------------------------------------------------------------------- #
def drop_all_slides(prs):
    """Entfernt alle Beispielfolien samt Beziehungen."""
    sld_id_lst = prs.slides._sldIdLst
    removed = 0
    for sld in list(sld_id_lst):
        rel_id = sld.get(qn("r:id"))
        prs.part.drop_rel(rel_id)
        sld_id_lst.remove(sld)
        removed += 1
    return removed


def layout_of(prs, name):
    """Liefert das Layout aus dem Master, den die Vorlage dafür vorsieht."""
    master_index = None
    for prefix, index in MASTER_BY_PREFIX.items():
        if name == prefix or name.startswith(prefix):
            master_index = index
            break
    if master_index is None:
        master_index = 2               # CONTENT A_V1 bis F
    for lay in prs.slide_masters[master_index].slide_layouts:
        if lay.name == name:
            return lay
    raise KeyError("Layout nicht gefunden: " + name)


def _next_shape_id(sp_tree):
    ids = []
    for element in sp_tree.iter(qn("p:cNvPr")):
        try:
            ids.append(int(element.get("id")))
        except (TypeError, ValueError):
            continue
    return (max(ids) + 1) if ids else 2


def clone_footer_placeholders(slide, layout):
    """Kopiert Fußzeile, Datum und Foliennummer aus dem Layout auf die Folie.

    python-pptx kopiert diese drei Platzhalter nicht mit, PowerPoint legt sie
    beim Einfügen einer Folie aber selbst an. Die Kopie bringt das Feld
    slidenum mit, deshalb bleibt die Nummerierung dynamisch.
    """
    wanted = (PP_PLACEHOLDER.FOOTER, PP_PLACEHOLDER.DATE, PP_PLACEHOLDER.SLIDE_NUMBER)
    sp_tree = slide.shapes._spTree
    cloned = 0
    for placeholder in layout.placeholders:
        if placeholder.placeholder_format.type in wanted:
            element = copy.deepcopy(placeholder._element)
            c_nv_pr = element.find(qn("p:nvSpPr")).find(qn("p:cNvPr"))
            c_nv_pr.set("id", str(_next_shape_id(sp_tree)))
            sp_tree.append(element)
            cloned += 1
    return cloned


def set_footer(slide):
    for ph in slide.placeholders:
        kind = ph.placeholder_format.type
        if kind == PP_PLACEHOLDER.FOOTER:
            ph.text_frame.text = FOOTER_NAME
        elif kind == PP_PLACEHOLDER.DATE:
            ph.text_frame.text = FOOTER_DATE
        # SLIDE_NUMBER bleibt unberührt, damit das Feld erhalten bleibt


def new_slide(prs, layout_name):
    layout = layout_of(prs, layout_name)
    slide = prs.slides.add_slide(layout)
    clone_footer_placeholders(slide, layout)
    set_footer(slide)
    return slide


def by_type(slide, ph_type):
    """Platzhalter nach Typ, sortiert nach Lage statt nach ID.

    In CONTENT A_V2 trägt ID 13 den unteren Textbereich und ID 14 den
    Untertitel, die ID-Reihenfolge ist dort also nicht brauchbar.
    """
    found = [ph for ph in slide.placeholders if ph.placeholder_format.type == ph_type]
    # Toleranz beim Top-Vergleich: die Vorlage legt Spalten auf gleicher Höhe mit
    # Abweichungen von wenigen EMU an (11 EMU in CONTENT B_V1). Ohne Rundung
    # gewinnt die linke Spalte nicht, und die Spalten wären vertauscht.
    return sorted(found, key=lambda p: (round((p.top or 0) / 45720.0), (p.left or 0)))


def set_title(slide, text, size=None, on_dark=False, chapter=None):
    holders = by_type(slide, PP_PLACEHOLDER.TITLE)
    if not holders:
        return
    tf = holders[0].text_frame
    tf.word_wrap = True
    run = tf.paragraphs[0].add_run()
    run.text = text
    run.font.name = HEAD_FONT
    if size:
        run.font.size = Pt(size)
    if on_dark:
        run.font.color.rgb = WHITE
    add_chapter(slide, text, on_dark, chapter)


def add_chapter(slide, title, on_dark=False, chapter=None):
    """Setzt den Kapitelverweis als Kopfzeile über den Titel.

    Unterfolien der Abschnitte 1 bis 7 tragen eine mehrstufige Nummer wie
    "2.2 Anwendbarkeit" und bekommen den Kapiteltitel aus CHAPTERS. Divider, Cover,
    Inhaltsverzeichnis und Abschluss bleiben ohne Kopfzeile. Der Parameter chapter
    erlaubt einen eigenen Text für Sonderfälle.
    """
    if chapter is None:
        match = re.match(r"^(\d+)\.\d+\s", title)
        if not match:
            return None
        chapter = CHAPTERS.get(match.group(1))
    if not chapter:
        return None
    box = slide.shapes.add_textbox(Inches(LEFT), Inches(0.05), Inches(12.59), Inches(0.26))
    frame = box.text_frame
    frame.word_wrap = False
    frame.margin_left = 0
    frame.margin_right = 0
    frame.margin_top = 0
    frame.margin_bottom = 0
    run = frame.paragraphs[0].add_run()
    run.text = chapter
    run.font.name = BODY_FONT
    run.font.size = Pt(10)
    run.font.bold = True
    run.font.color.rgb = WHITE if on_dark else BRAND
    return box


def fill_bullets(ph, items, size=12.5, on_dark=False, space_after=8):
    lead_color = GREEN if on_dark else BRAND
    detail_color = WHITE if on_dark else DARK
    tf = ph.text_frame
    tf.word_wrap = True
    first = True
    for item in items:
        lead, detail = item if isinstance(item, tuple) else (item, None)
        para = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        para.space_after = Pt(space_after)
        if lead:
            run = para.add_run()
            run.text = lead
            run.font.name = BODY_FONT
            run.font.bold = True
            run.font.size = Pt(size)
            run.font.color.rgb = lead_color
        if detail:
            run2 = para.add_run()
            run2.text = ("   " + detail) if lead else detail
            run2.font.name = BODY_FONT
            run2.font.size = Pt(size)
            run2.font.color.rgb = detail_color


def fill_bodies(slide, items, size=12.5, on_dark=False):
    """items: Liste von Listen mit (lead, detail) je Platzhalter."""
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if not holders:
        holders = by_type(slide, PP_PLACEHOLDER.BODY)
    for index, content in enumerate(items):
        if index < len(holders) and content:
            fill_bullets(holders[index], content, size=size, on_dark=on_dark)


def status_color(value):
    text = value.lower()
    if "erledigt" in text or "geklärt" in text:
        return GREEN
    if "akut" in text:
        return CORAL
    if "teilweise" in text:
        return OLIVE
    if "entschieden" in text:
        return GREEN
    if "läuft" in text:
        return BRAND
    return GRAY


def add_table(slide, top, headers, rows, widths, size=9.5, row_h=0.4, status_col=None,
              highlight_prefix=None):
    total = len(rows) + 1
    shape = slide.shapes.add_table(total, len(headers), Inches(LEFT), Inches(top),
                                   Inches(sum(widths)), Inches(row_h * total))
    table = shape.table
    table.first_row = False
    table.horz_banding = False

    tbl_pr = table._tbl.tblPr
    for element in tbl_pr.findall(qn("a:tableStyleId")):
        tbl_pr.remove(element)
    style = tbl_pr.makeelement(qn("a:tableStyleId"), {})
    style.text = GRID_STYLE
    tbl_pr.append(style)

    for index, width in enumerate(widths):
        table.columns[index].width = Inches(width)
    for index in range(total):
        table.rows[index].height = Inches(row_h)

    def put(cell, text, font_size, bold, color, fill):
        cell.text = text
        cell.fill.solid()
        cell.fill.fore_color.rgb = fill
        cell.vertical_anchor = MSO_ANCHOR.MIDDLE
        cell.margin_left = Inches(0.07)
        cell.margin_right = Inches(0.05)
        cell.margin_top = Inches(0.02)
        cell.margin_bottom = Inches(0.02)
        if not str(text):
            return
        run = cell.text_frame.paragraphs[0].runs[0]
        run.font.name = BODY_FONT
        run.font.size = Pt(font_size)
        run.font.bold = bold
        run.font.color.rgb = color

    for col, head in enumerate(headers):
        put(table.cell(0, col), head, size, True, WHITE, BRAND)
    highlights = set()
    if highlight_prefix:
        for row_index, row in enumerate(rows, start=1):
            for col, value in enumerate(row):
                if re.match(highlight_prefix, str(value)):
                    highlights.add((row_index, col))
                    if col > 0:
                        highlights.add((row_index, col - 1))

    for row_index, row in enumerate(rows, start=1):
        fill = WHITE if row_index % 2 else ALT
        for col, value in enumerate(row):
            bold = False
            color = DARK
            cell_fill = fill
            if status_col is not None and col == status_col:
                bold = True
                color = status_color(str(value))
            if (row_index, col) in highlights:
                bold = True
                color = WHITE
                cell_fill = BRAND
            put(table.cell(row_index, col), str(value), size, bold, color, cell_fill)
    return shape


def add_status_note(slide, shape):
    """Graue Zeile unter einer Maßnahmentafel: wann die Maßnahmen umzusetzen sind.

    Die Höhe richtet sich nach dem Tabellenrahmen, damit der Hinweis keine Zeile überdeckt.
    Der Auslöser ist die Bereitstellung auf dem Unionsmarkt. Solange nur in Drittstaaten
    vertrieben wird, entstehen aus CRA und RED-Cyber keine Pflichten, die Umsetzung ist
    dann freiwillig. Text und Master-Abschnitt 8 tragen dieselbe Aussage.
    """
    top = shape.top + shape.height + Inches(0.06)
    box = slide.shapes.add_textbox(Inches(LEFT), top, Inches(12.59), Inches(0.3))
    frame = box.text_frame
    frame.word_wrap = True
    run = frame.paragraphs[0].add_run()
    run.text = ("Umzusetzen mit der Bereitstellung auf dem Unionsmarkt, auch unentgeltlich; "
                "ohne EU-Vertrieb bleibt die Umsetzung freiwillig.")
    run.font.name = BODY_FONT
    run.font.size = Pt(9)
    run.font.color.rgb = GRAY
    return box


# --------------------------------------------------------------------------- #
# Folien
# --------------------------------------------------------------------------- #
def build_cover(prs):
    slide = new_slide(prs, "COVER")
    set_title(slide, "EU CRA Compliance")
    holders = by_type(slide, PP_PLACEHOLDER.BODY)
    if holders:
        fill_bullets(holders[0], [
            "VentCon2 - Pressure Control System",
            "Regulation (EU) 2024/2847",
            "Stand 2026-10-01   |   Grundlage: sec/CRA_Compliance_Checklist.md",
        ], size=11, space_after=2)


def build_inhaltsverzeichnis(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Inhaltsverzeichnis", size=26)
    add_table(slide, CONTENT_TOP, ["Folie", "Inhalt", "Folie", "Inhalt"], [
        ["1", "Cover: EU CRA Compliance", "15", "4.2 Maßnahmen und Status, Phase 1 bis 3"],
        ["2", "Inhaltsverzeichnis", "16", "4.3 Maßnahmen und Status, Phase 4 und 5"],
        ["3", "1 Systembeschreibung", "17", "4.4 Aktiver Strang"],
        ["4", "1.1 Systemübersicht", "18", "5 Pflichten der CRA"],
        ["5", "1.2 Bedienoberfläche im Browser", "19", "5.1 Pflichten 1 bis 10"],
        ["6", "2 Regelwerk", "20", "5.2 Pflichten 11 bis 22"],
        ["7", "2.1 RED und CRA: zwei Regime", "21", "5.3 Pflichten 23 bis 26"],
        ["8", "2.2 Produktklassifizierung und Normenlage", "22", "6 Dokumente und Quellen"],
        ["9", "3 Produkt und Anwendbarkeit", "23", "6.1 Dokumentenregister"],
        ["10", "3.1 Produkt und Vorhaben", "24", "6.2 Quellen"],
        ["11", "3.2 Anwendbarkeit", "25", "6.3 Status und Revision"],
        ["12", "3.3 Prototypen und Vorserien", "26", "7 Fazit und nächste Schritte"],
        ["13", "4 Risiken und Maßnahmen", "27", "7.1 Was folgt daraus für VENTREX"],
        ["14", "4.1 Risikobild: Befunde R1 bis R10", "28", "Abschluss"],
    ], widths=[0.7, 4.7, 0.7, 6.49], size=9.5, row_h=0.29,
        highlight_prefix=r"^\d+ [A-Za-z]")

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(5.95), Inches(12.59), Inches(1.0))
    frame = box.text_frame
    frame.word_wrap = True
    lines = [
        ("Stand 2026-10-01: zurückgestellt für den Unionsmarkt, Prototypenbetrieb, geplanter "
         "Vertrieb Indien.", 12, True, BRAND),
        ("Aktiv bleiben P1.1, P1.2, P1.3 und P2.2. P1.5 sowie P4.1 bis P4.8, P5.1 und P5.2 ruhen "
         "bis zur Marktentscheidung, die RED-Anforderungen gelten bis 2027-12-11.", 11.5, False,
         GRAY),
    ]
    for index, (text, size, bold, color) in enumerate(lines):
        paragraph = frame.paragraphs[0] if index == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = text
        run.font.name = BODY_FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color


def build_produkt(prs):
    slide = new_slide(prs, "CONTENT B_V1")
    set_title(slide, "3.1 Produkt und Vorhaben", size=24)
    # Technische Angaben aus dem Projektstand: src/Constants.h und data/default.json,
    # Versorgungsspannung und Abbildungen aus dem Benutzerhandbuch.
    fill_bodies(slide, [
        [
            ("Anschluss an 2.2", "Regime, Kategorie und Normen sind geklärt; hier geht es um "
                                 "das Produkt und seinen Weg"),
            ("VentCon2", "Ventilsteuerung für Druckregelung auf ESP32-S3, Firmware 2.7.4, "
                         "gebaut mit PlatformIO"),
            ("Aufgabe", "Regelt den Ausgangsdruck in der Kraftstoffleitung und steuert das "
                        "Absperrventil über ein Taktverhältnis"),
            ("Sensor", "Analogsensor am ADC, 0,5 bis 4,5 V entsprechend 0 bis 10 bar; die "
                       "Notabschaltung prüft den Druck"),
            ("Regelung", "PID mit 10 ms Abtastzeit und 1 kHz Regelschleife, Sollwert 6 bar, "
                         "Filterstärke einstellbar"),
            ("Stellglied", "PWM mit 2 kHz und 14 Bit, Ausgang auf 20 bis 90 Prozent begrenzt, "
                           "Hysterese 5 Prozentpunkte"),
            ("Schnittstellen", "Weboberfläche auf Port 80, Access-Point VENTCON_AP mit "
                               "192.168.4.1 und ventcon.local, serielle Konsole, LittleFS"),
            ("Werkzeuge", "PlatformIO mit Arduino-Framework, Firmwareversion aus der Datei "
                          "VERSION, Upload und Konsole über USB"),
        ],
        [
            ("Aufbau", "Prototyp im eigenen Versuchsaufbau, 12 bis 24 V Gleichspannung, am "
                       "Netz, ohne Weitergabe an Dritte"),
            ("Betrieb", "Access-Point mit höchstens zwei gleichzeitigen Clients; Änderungen an "
                        "Sollwert, PID, Filter und PWM werden gesammelt übernommen"),
            ("Dateisystem", "Einstellungen, Webseiten und Diagrammdaten liegen im "
                            "LittleFS-Flash; Dateien laufen getrennt vom Firmware-Image"),
            ("Datenverarbeitung", "Keine Cloud und keine Fernwartung: Regelung und Daten "
                                  "bleiben im Gerät und im lokalen Netz"),
            ("Vorhaben", "Serie nicht beschlossen; geplanter Markt ist Indien, ein EU-Vertrieb "
                         "ist derzeit nicht vorgesehen"),
            ("Rechtslage", "CRA und RED-Cyber greifen nur bei Bereitstellung auf dem "
                           "Unionsmarkt; die RED-Anforderungen enden am 2027-12-11"),
            ("Entwicklungsstand", "Firmware 2.7.4 im Prototypenbetrieb, Härtung läuft weiter; "
                                  "Updates über Funk sind noch nicht umgesetzt"),
            ("Rollen im Dossier", "Der Master führt den Status, die Detaildokumente die "
                                  "Umsetzung, dieses Deck fasst beides zusammen"),
        ],
    ], size=11.5)

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(6.45), Inches(12.59), Inches(0.35))
    frame = box.text_frame
    frame.word_wrap = True
    run = frame.paragraphs[0].add_run()
    run.text = ("Technische Werte aus dem Projektstand: src/Constants.h, data/default.json "
                "und Benutzerhandbuch.")
    run.font.name = BODY_FONT
    run.font.size = Pt(8.5)
    run.font.color.rgb = GRAY


def drop_picture_placeholder(slide):
    """Entfernt den leeren Bildplatzhalter, den das Layout mitbringt.

    Ein leerer Bildplatzhalter erscheint im PNG- und PDF-Export als grauer Kasten mit
    Bildsymbol. Die Divider und die beiden Bildfolien brauchen den Platzhalter nicht,
    weil das Bild als eigene Form in den Bildbereich gesetzt wird; die Titelplatzhalter
    bleiben unberührt.
    """
    for shape in list(slide.shapes):
        if shape.is_placeholder and shape.placeholder_format.type == PP_PLACEHOLDER.PICTURE:
            shape._element.getparent().remove(shape._element)


def picture_area(slide):
    """Liefert den Bildbereich des Layouts als (left, top, width, height) in Inches."""
    for ph in slide.placeholders:
        if ph.placeholder_format.type == PP_PLACEHOLDER.PICTURE:
            return (ph.left / 914400.0, ph.top / 914400.0,
                    ph.width / 914400.0, ph.height / 914400.0)
    raise RuntimeError("Layout ohne Bildbereich: " + slide.slide_layout.name)


def add_figure(slide, image, caption, size=9.0):
    """Setzt ein Bild mittig in den Bildbereich des Layouts, darunter die Quellenzeile.

    Das Bild wird auf den Bereich eingepasst (nicht beschnitten), damit die
    Abbildungen vollständig sichtbar bleiben.
    """
    left, top, width, height = picture_area(slide)
    drop_picture_placeholder(slide)
    caption_h = 0.34
    box_h = height - caption_h
    with Image.open(image) as img:
        aspect = img.width / float(img.height)
    draw_w, draw_h = width, width / aspect
    if draw_h > box_h:
        draw_h, draw_w = box_h, box_h * aspect
    pic_left = left + (width - draw_w) / 2.0
    pic_top = top + (box_h - draw_h) / 2.0
    slide.shapes.add_picture(image, Inches(pic_left), Inches(pic_top),
                             Inches(draw_w), Inches(draw_h))
    box = slide.shapes.add_textbox(Inches(left), Inches(top + box_h + 0.04),
                                   Inches(width), Inches(caption_h))
    frame = box.text_frame
    frame.word_wrap = True
    frame.margin_left = 0
    frame.margin_right = 0
    frame.margin_top = 0
    frame.margin_bottom = 0
    run = frame.paragraphs[0].add_run()
    run.text = caption
    run.font.name = BODY_FONT
    run.font.size = Pt(size)
    run.font.color.rgb = GRAY


def build_divider(prs, layout_name, title, line=None):
    """Divider mit Titel und, wenn angegeben, sichtbarer Unterzeile.

    Die Layouts SUBSECTION 1 bis 3 haben keinen Textplatzhalter. Dort wird die
    Unterzeile als Textfeld unter dem Titel eingesetzt, damit jeder Abschnitt seinen
    roten Faden sichtbar mitbringt.
    """
    slide = new_slide(prs, layout_name)
    drop_picture_placeholder(slide)
    set_title(slide, title)
    if not line:
        return
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT) or by_type(slide, PP_PLACEHOLDER.BODY)
    if holders:
        fill_bullets(holders[0], [(line, None)], size=12)
        return
    titles = by_type(slide, PP_PLACEHOLDER.TITLE)
    if not titles:
        return
    title_ph = titles[0]
    box = slide.shapes.add_textbox(title_ph.left, title_ph.top + title_ph.height - Inches(0.25),
                                   title_ph.width, Inches(1.0))
    frame = box.text_frame
    frame.word_wrap = True
    run = frame.paragraphs[0].add_run()
    run.text = line
    run.font.name = BODY_FONT
    run.font.size = Pt(12)
    run.font.color.rgb = GRAY


def build_systemuebersicht(prs):
    slide = new_slide(prs, "CONTENT E")
    set_title(slide, "1.1 Systemübersicht", size=26)
    fill_bodies(slide, [[
        ("Regelstrecke", "Kraftstofftank, Absperrventil mit elektronischem Druckregler "
                         "und Motor; geregelt wird der Ausgangsdruck in der Kraftstoffleitung"),
        ("Sensor und Stellglied", "Ein Drucksensor erfasst den Ausgangsdruck, das "
                                  "Ventil wird über ein Taktverhältnis angesteuert"),
        ("Regler", "VentCon2 läuft auf einem ESP32-S3 und arbeitet als PID-Regler auf "
                   "den Ausgangsdruck"),
        ("Bedienung", "Einstellungen erfolgen über die Weboberfläche im Netz des Geräts, "
                      "zum Beispiel auf Tablet, Laptop oder Smartphone"),
        ("Versorgung", "12 bis 24 V Gleichspannung, im Versuchsaufbau am Netz; deshalb "
                       "laufen die Härtungsmaßnahmen weiter"),
    ]], size=11.5)
    add_figure(slide, "sec/pic/SchematicHardware.png",
               "Bild: Systemübersicht des Versuchsaufbaus, Benutzerhandbuch, "
               "Kapitel System Overview")


def build_bedienung(prs):
    slide = new_slide(prs, "CONTENT E")
    set_title(slide, "1.2 Bedienoberfläche im Browser", size=26)
    fill_bodies(slide, [[
        ("Zugang", "Das Gerät öffnet einen eigenen WLAN-Zugangspunkt, die Oberfläche "
                   "liegt unter 192.168.4.1 oder http://ventcon.local"),
        ("Anzeige", "Live-Diagramm mit Ausgangsdruck (blau), Sollwert (orange) und "
                    "Ventiltaktung (grün), dazu Netzstatus und Systeminformation"),
        ("Einstellungen", "Sollwert, PID-Gains, Filterstärke, PWM-Frequenz, "
                          "PWM-Auflösung, Abtastzeit und Sensorbereich"),
        ("Übernahme", "Änderungen werden gesammelt und erst mit Apply Changes wirksam, "
                      "Reset to Default stellt die Auslieferungswerte wieder her"),
        ("Bezug zur CRA", "Dieselbe Oberfläche ist der Angriffspunkt der Befunde zur "
                          "fehlenden Authentifizierung und zur Zustandsänderung per GET; die "
                          "laufenden Maßnahmen sichern die verändernden Endpunkte"),
    ]], size=11.5)
    add_figure(slide, "sec/pic/BrowserApp.png",
               "Bild: annotierte Bedienoberfläche, Benutzerhandbuch, "
               "Kapitel Browser Interface")


def build_anwendbarkeit(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "3.2 Anwendbarkeit", size=26)
    # Spalte Entscheidung: nur bei Status geklärt, dort steht die kurze Begründung
    # der Feststellung, damit die Zeile ohne Rückgriff auf den Master lesbar ist.
    # Keine Zeilen zu Importeuren, Händlern oder Vertretern: einen Vertrieb über Dritte
    # gibt es nicht, VentCon2 betrifft allein VENTREX.
    add_table(slide, CONTENT_TOP, ["Frage", "Antwort", "Entscheidung", "Status"], [
        ["Produkt mit digitalen Elementen und Datenverbindung?",
         "Ja, ESP32-S3 Ventilsteuerung mit Webinterface, WLAN und Netzwerkverbindung; die "
         "Verbindung ist bestimmungsgemäßer Zweck",
         "CRA greift für VentCon2", "geklärt"],
        ["Ausnahme nach Art. 2(2) bis (7)?",
         "Nein; kein Medizinprodukt, keine Fahrzeug-Typgenehmigung nach Art. 2(2)(c), keine "
         "Luftfahrt-, Marine- oder Verteidigungsausrüstung, kein Ersatzteil",
         "Keine Ausnahme einschlägig", "geklärt"],
        ["Internetfähiges Funkgerät?",
         "Ja, ESP32 mit Wi-Fi; die Cyber-Anforderungen des RED gelten bis 2027-12-11",
         "RED-Cyber gilt bis 2027-12-11", "geklärt"],
        ["Wer ist Hersteller nach Art. 3(13)?",
         "Wer unter eigenem Namen bereitstellt, hier VENTREX; ein Vertrieb über Dritte ist "
         "nicht vorgesehen",
         "VENTREX ist Hersteller", "geklärt"],
        ["Gewerblich in der EU in Verkehr gebracht?",
         "Prototypenbetrieb im eigenen Aufbau, geplanter Vertrieb Indien, kein EU-Vertrieb",
         "Kein EU-Vertrieb, nur Prototypenbetrieb", "geklärt"],
        ["Vertrieb nur in Drittländern, zum Beispiel Indien?",
         "Dann greifen weder CRA noch RED-Cyber; Anknüpfungspunkt ist der Unionsmarkt, Art. 2 und 3",
         "Ohne EU-Vertrieb kein Anknüpfungspunkt", "geklärt"],
        ["Lieferung an EU-Kunden, auch unentgeltlich?",
         "Bereitstellung auf dem Unionsmarkt nach Art. 3(22), CRA und RED-Cyber greifen",
         "", "zu beachten"],
        ["Änderung nach der Bereitstellung?",
         "Änderungen, die die Konformität berühren, gelten als wesentliche Änderung und lösen "
         "die Pflichten neu aus",
         "Änderungen vorher bündeln", "beobachten"],
        ["Firmware als Open Source?",
         "Nein, keine Veröffentlichung unter freier Lizenz; damit keine Pflichten als "
         "Open-Source-Steward nach Art. 24",
         "Keine Steward-Pflichten", "geklärt"],
        ["Wiedervorlage",
         "Beim ersten EU-Kontakt laufen CE-Weg, Dokumentation, Supportzeitraum und "
         "Meldepflichten an",
         "", "beobachten"],
    ], widths=[3.5, 5.3, 2.4, 1.39], size=9.5, row_h=0.42, status_col=3)

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(6.15), Inches(12.59), Inches(0.60))
    frame = box.text_frame
    frame.word_wrap = True
    frame.margin_left = 0
    frame.margin_right = 0
    frame.margin_top = 0
    frame.margin_bottom = 0
    run = frame.paragraphs[0].add_run()
    run.text = ("Auslöser: Pflichten entstehen erst mit der Bereitstellung auf dem Unionsmarkt "
                "(Art. 2(1), Art. 3(21) und (22)); ein Vertrieb nur in Drittstaaten löst keine aus, "
                "eine unentgeltliche Lieferung in die Union dagegen schon. "
                "Bestandsschutz nach Art. 69(2) und (3): Produkte, die vor dem 2027-12-11 in "
                "Verkehr gebracht wurden, unterliegen ab dann den Pflichten nur bei wesentlicher "
                "Änderung; die Meldepflicht nach Art. 14 gilt für sie seit 2026-09-11.")
    run.font.name = BODY_FONT
    run.font.size = Pt(9)
    run.font.color.rgb = GRAY


def build_prototypen(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "3.3 Prototypen und Vorserien", size=24)
    add_table(slide, CONTENT_TOP, ["Fall", "CRA-Status", "Grundlage"], [
        ["Bleibt im Labor oder in eigener Anlage",
         "Nicht erfasst, keine Bereitstellung und keine Meldepflicht", "Art. 2, Art. 3(22)"],
        ["Messe oder Vorführung, auch nicht konform",
         "Zulässig mit sichtbarem Hinweis: nicht konform, nicht in Verkehr gebracht", "Art. 4(2)"],
        ["Alpha, Beta oder Release Candidate zum Testen",
         "Befristet zulässig, nur zum Testen, mit Hinweis und Risikobewertung; die Erleichterung "
         "gilt nicht für Sicherheitsbauteile nach anderem Unionsrecht",
         "Art. 4(3) und Art. 4(4), Recital 37"],
        ["Weitergabe an Dritte, auch kostenlos",
         "Bereitstellung auf dem Unionsmarkt, ab dann volle Herstellerpflichten",
         "Art. 3(22), Art. 13"],
        ["Nur Drittland-Vertrieb, zum Beispiel Indien", "Nicht erfasst, kein Unionsmarkt",
         "Art. 2(1)"],
        ["Eigener Prototyp am Netz, ohne Weitergabe",
         "Die Härtungsmaßnahmen laufen weiter, freiwillig und unabhängig vom Marktstatus",
         "freiwillig"],
    ], widths=[4.3, 6.0, 2.29], size=10, row_h=0.5)

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(5.15), Inches(12.59), Inches(1.2))
    frame = box.text_frame
    frame.word_wrap = True
    lines = [
        ("Vorführung in der Union: sichtbarer Hinweis erforderlich", 11.5, True, BRAND),
        ("Prototyp, nicht konform mit Verordnung (EU) 2024/2847, wird bis zur Konformität nicht "
         "in Verkehr gebracht.", 11, False, GRAY),
        ("Prototype, not in conformity with Regulation (EU) 2024/2847, not to be placed on the "
         "market until compliant.", 11, False, GRAY),
    ]
    for index, (text, size, bold, color) in enumerate(lines):
        paragraph = frame.paragraphs[0] if index == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = text
        run.font.name = BODY_FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color


def build_red_cra(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "2.1 RED und CRA: zwei Regime, ein Übergang", size=24)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if holders:
        fill_bullets(holders[0],
                     [("Der Aufbau ist beschrieben. Jetzt die Frage, welches Recht dafür gilt: "
                       "zwei Regime mit einem gemeinsamen Anknüpfungspunkt, der Bereitstellung auf "
                       "dem Unionsmarkt, die am 2027-12-11 ineinander übergehen.", None)], size=12)
    # Vergleichstabelle nach dem Muster Merkmal gegen RED-Cyber und CRA. Inhalt aus
    # Master-Abschnitt 1, damit Deck und Dossier dieselben Aussagen tragen.
    add_table(slide, 2.0,
              ["Merkmal", "RED-Cyber, gilt bis 2027-12-11",
               "CRA, Verordnung (EU) 2024/2847"], [
        ["Rechtsgrundlage",
         "Art. 3(3)(d)(e)(f) der Richtlinie 2014/53/EU, ausgestaltet durch die Delegierte "
         "VO (EU) 2022/30",
         "Verordnung (EU) 2024/2847"],
        ["Geltungsbereich",
         "Internetfähige Funkgeräte, hier der ESP32 mit Wi-Fi; Anknüpfungspunkt ist die "
         "Bereitstellung auf dem Unionsmarkt, auch unentgeltlich",
         "Produkte mit digitalen Elementen, hier die Ventilsteuerung; derselbe Anknüpfungspunkt"],
        ["Inhalt der Anforderungen",
         "Netzwerkschutz (d) und Schutz personenbezogener Daten (e), also RED-1 und RED-2; "
         "RED-3, Betrugsschutz (f), ist nicht einschlägig. Normen: EN 18031-1 und -2",
         "Annex I Teil I und II, Herstellerpflichten nach Art. 13, Meldepflichten nach Art. 14, "
         "CE-Zeichen nach Art. 29 und 30"],
        ["Pflichten im Betrieb",
         "Keine Meldepflicht; Nachweis der grundlegenden Anforderungen beim Inverkehrbringen, "
         "Selbstbewertung oder Modul B+C",
         "Sicherheitsupdates über den Supportzeitraum, Meldungen in 24 h, 72 h, 14 Tagen und "
         "einem Monat"],
        ["Zeitplan",
         "Anwendbar seit 2025-08-01; Aufhebung der Delegierten VO (EU) 2022/30 durch die "
         "Delegierte VO (EU) 2026/339 ab 2027-12-11",
         "Art. 14 seit 2026-09-11; vollständig ab 2027-12-11, danach unbefristet mit "
         "Supportzeitraum"],
    ], widths=[2.4, 4.8, 5.39], size=10, row_h=0.79)


def build_produktklassifizierung(prs):
    """2.2 zeigt die Erkenntnisse: Kategorie und Normenlage. Die Vorgabe steht in 2.1, die
    Einordnung des Produkts folgt in Abschnitt 3."""
    slide = new_slide(prs, "CONTENT B_V1")
    set_title(slide, "2.2 Produktklassifizierung und Normenlage", size=24)
    fill_bodies(slide, [
        [
            ("Anschluss an 2.1",
             "Aus den zwei Regimen folgen Kategorien und Normen. Hier steht, in welche Kategorie "
             "VentCon2 fällt und welche Normen tragen"),
            ("Standardprodukt",
             "Keine Kategorie nach Annex III und Annex IV; die Durchführungsverordnung "
             "(EU) 2025/2392 zu den wichtigen Produkten ist nicht einschlägig"),
            ("Wichtige Produkte Class I und II",
             "Nein, keine Kernfunktion einer Kategorie nach Art. 7 und Annex III; damit entfallen "
             "die Verfahren nach Art. 32(2) und (3)"),
            ("Kritische Produkte",
             "Nein, keine Kategorie nach Art. 8 und Annex IV; ein Zertifikat nach einem "
             "EU-Cybersicherheitsschema ist nicht erforderlich"),
            ("Konformitätsbewertung CRA",
             "Modul A, interne Kontrolle des Herstellers nach Art. 32(1)(a), keine notifizierte "
             "Stelle beteiligt"),
        ],
        [
            ("Vorbehalt aus dem RED",
             "Modul A trägt nur für die CRA. Unter dem RED sind die Passwortklauseln der "
             "EN 18031 von der Konformitätsvermutung ausgenommen, dort Modul B+C oder Modul H; "
             "der Nachweisweg ist noch zu entscheiden, siehe 2.1"),
            ("Normenlage CRA",
             "Keine harmonisierte Norm im Amtsblatt zitiert, Stand 2026-10-01. Die Vermutung "
             "nach Art. 27 besteht nicht; der Normungsauftrag M/606 läuft"),
            ("Normenlage RED",
             "EN 18031-1/-2/-3 durch Durchführungsbeschluss (EU) 2025/138 im Amtsblatt zitiert, "
             "mit Restriktionen bei den Passwortklauseln; gilt bis 2027-12-11"),
            ("Welche Norm trägt was",
             "EN 18031-1 für RED-1 Netzwerkschutz, EN 18031-2 für RED-2 Datenschutz; EN 18031-3 "
             "entfällt mit RED-3, siehe 2.1"),
            ("EN 303 645",
             "Die Norm ist nur technischer Maßstab ohne Rechtswirkung: keine harmonisierte Norm "
             "und keine Konformitätsvermutung, weder unter dem RED noch unter der CRA"),
        ],
    ], size=11.5)


def build_was_folgt(prs):
    """7.1 als Fazit: was aus den Erkenntnissen für VENTREX folgt."""
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "7.1 Was folgt daraus für VENTREX", size=26)
    # Die vier Maßnahmen des aktiven Strangs stehen in 4.4, ihr Einzelstand in 4.2.
    fill_bodies(slide, [[
        ("Nachweisweg festlegen",
         "Für die CRA genügt Modul A, die Selbstbewertung im eigenen Haus. Offen ist die "
         "RED-Route: P1.5 entscheidet, ob die ausgenommenen Passwortklauseln der EN 18031 "
         "Modul B+C oder Modul H verlangen; die RED-Anforderungen gelten bis 2027-12-11"),
        ("Unterlagen aufbauen",
         "Risikobewertung P4.1, technische Dokumentation nach Art. 31 in P4.6, "
         "EU-Konformitätserklärung nach Art. 28 in P4.7, CE-Zeichen nach Art. 29 und 30"),
        ("Fristen einplanen",
         "Supportzeitraum von mindestens fünf Jahren mit Enddatum in der Nutzerinformation "
         "nach Annex II; Dokumentation, Erklärung und Nutzerinformation zehn Jahre oder für den "
         "Supportzeitraum bereithalten und Sicherheitsupdates ebenso lange verfügbar halten"),
        ("Normenlage beobachten",
         "Keine harmonisierte CRA-Norm, der Normungsauftrag M/606 läuft. Fällt später eine an, "
         "wird die Konformitätsvermutung nach Art. 27 nutzbar und der Nachweis leichter"),
        ("Was heute schon läuft",
         "P1.1, P1.2, P1.3 und P2.2 sind der aktive Strang, siehe 4.4 Aktiver Strang; Stand, "
         "Detaildokument und Evidenz je Maßnahme stehen in 4.2 Maßnahmen und Status. Dieselben "
         "vier Maßnahmen sind die Vorarbeit für die Sicherheitsanforderungen der Pflichten 1 "
         "bis 26 aus 5.1"),
        ("Was der Marktstatus bedeutet",
         "Solange nichts in die Union geliefert wird, entstehen keine Pflichten; auch eine "
         "unentgeltliche Lieferung zählt. Der erste EU-Kontakt löst sie aus, siehe 3.2 "
         "Anwendbarkeit"),
        ("Drei offene Entscheidungen",
         "Erstens den RED-Nachweisweg in P1.5, zweitens den Supportzeitraum festlegen und "
         "benennen, drittens die Normenlage vor der ersten Auslieferung erneut prüfen"),
        ("Anschluss",
         "Die Pflichten 1 bis 26 stehen in 5.1 bis 5.3, der Umsetzungsstand in 4.2 bis 4.4"),
    ]], size=12)


def build_register(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "6.1 Dokumentenregister", size=26)
    add_table(slide, CONTENT_TOP, ["Dokument oder Gruppe", "Rolle", "Maßnahmen"], [
        ["CRA_Compliance_Checklist.md", "Master (Grundlage), einzige Statusquelle", "P1.1 bis P5.2"],
        ["analyse/...Sicherheitsanalyse.md", "Risiko- und Evidenzanalyse", "R1 bis R10"],
        ["plan/ 4 vorhandene Pläne", "Web_Auth, Kalibrierung, AP-Passwort, Mutex",
         "P1.1, P1.2, P1.3, P2.1, P2.2, P3.1"],
        ["plan/ 14 Skelette und Pläne", "Abschnitte 3 bis 6 der Skelette noch offen",
         "P1.4, P1.5, P2.3, P3.3, P4.1 bis P4.8, P5.1, P5.2"],
        ["grundlagen/ 2 Dokumente", "Mutex sowie Web und Authentifizierung",
         "P3.1, P1.1, P1.2, P2.1"],
        ["plan/ 3 Rahmenpläne", "Dokumentenstruktur, Abgrenzung der beiden Regime, Präsentation",
         "RED-1 bis RED-3, P1.5"],
        ["sources/...Regulation_2024_2847_EN.pdf", "Normquelle (CRA-Volltext)", "keine"],
        ["26 Dokumente insgesamt", "1 Master, 1 Analyse, 21 Pläne, 2 Grundlagen, 1 Normquelle",
         "P3.2 ohne Detaildokument, Nachweis R8"],
    ], widths=[4.4, 5.4, 2.79], size=9.5, row_h=0.5)


def build_pflichten_1_10(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "5.1 Pflichten 1 bis 10: Sicherheitsanforderungen", size=24)
    add_table(slide, CONTENT_TOP,
              ["Nr.", "Pflicht", "CRA-Referenz", "Risiko / Maßnahme", "Status"], [
        ["1", "Keine universellen Standardpasswörter",
         "Annex I, Teil I, (2)(b) und (d); EN 303 645 5.1-1 (technischer Maßstab)", "R2 / P2.2",
         "offen"],
        ["2", "Authentifizierung und Zugriffskontrolle", "Annex I, Teil I, (2)(d)", "R1 / P1.1",
         "offen"],
        ["3", "Minimale Angriffsfläche", "Annex I, Teil I, (2)(j)", "R5 / P2.1", "offen"],
        ["4", "Schutz vor unbefugtem Zugriff, Eingabevalidierung",
         "Annex I, Teil I, (2)(d); Eingabevalidierung ist Projektauslegung", "R6 / P1.4", "offen"],
        ["5", "Sichere Kommunikation", "Annex I, Teil I, (2)(e) und (f)", "Funkebene / P3.3",
         "offen"],
        ["6", "Sichere Standardkonfiguration", "Annex I, Teil I, (2)(b)", "R4 / P1.2", "offen"],
        ["7", "Sichere Update-Mechanismen, signiert",
         "Annex I, Teil I, (2)(c); signierte Verteilung nach Teil II Nr. 7", "P4.3", "offen"],
        ["8", "Verfügbarkeit und DoS-Resistenz", "Annex I, Teil I, (2)(h) und (i)",
         "R7, R8 / P2.3, P3.2", "offen"],
        ["9", "Produktsicherheit, Notabschaltung",
         "kein Punkt in Annex I, Teil I; Projektauslegung, nächster Punkt (2)(k)", "R3 / P1.3",
         "offen"],
        ["10", "Sichere Speicherung von Zugangsdaten",
         "Annex I, Teil I, (2)(e); EN 303 645 5.4-1 (technischer Maßstab)", "R2 / P2.2", "offen"],
    ], widths=[0.6, 4.6, 3.3, 2.7, 1.39], size=9, row_h=0.45, status_col=4)

    # Hinweis unter der Tabelle: die Nummern sind die Zählung des Dossiers, nicht der
    # Verordnung. Die Passwortanforderung hat keine wörtliche Entsprechung in Annex I;
    # der Hinweis steht hier, weil die Referenzspalte der Tabelle keine dritte Zeile faßt.
    box = slide.shapes.add_textbox(Inches(LEFT), Inches(6.40), Inches(12.59), Inches(0.5))
    frame = box.text_frame
    frame.word_wrap = True
    lines = [
        ("Projektliste: die Nummern 1 bis 26 sind die Zählung dieses Dossiers. Weitere "
         "Herstellerpflichten stehen in Art. 13(6) bis (23) und Art. 14(6) und (8).", 9, False,
         GRAY),
        ("Die Passwortanforderung steht nicht wörtlich in der Verordnung: sie stützt sich auf die "
         "Voreinstellung sicher ab Werk nach Annex I Teil I (2)(b). EN 303 645 5.1-1 ist "
         "insoweit nur technischer Maßstab.", 9, False, GRAY),
    ]
    for index, (text, size, bold, color) in enumerate(lines):
        paragraph = frame.paragraphs[0] if index == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = text
        run.font.name = BODY_FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color


def build_pflichten_11_22(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "5.2 Pflichten 11 bis 22: Schwachstellen, Konformität, Markt", size=24)
    add_table(slide, CONTENT_TOP, ["Nr.", "Pflicht", "Referenz", "Maßnahme", "Status"], [
        ["11", "SBOM pflegen", "Annex I, Teil II, Nr. 1", "P4.2", "offen"],
        ["12", "Schwachstellen identifizieren und dokumentieren", "Annex I, Teil II, Nr. 1",
         "P4.5", "offen"],
        ["13", "Schwachstellen ohne Verzögerung beheben", "Annex I, Teil II, Nr. 2", "P4.3",
         "offen"],
        ["14", "Disclosure-Richtlinie und Sicherheitskontakt",
         "Annex I, Teil II, Nr. 5 und 6", "P4.5", "offen"],
        ["15", "Sicherheitsupdates testen vor Auslieferung", "Annex I, Teil II, Nr. 3", "P4.3",
         "offen"],
        ["16", "Cybersicherheits-Risikobewertung dokumentieren", "Art. 13(2) bis (4)", "P4.1",
         "offen"],
        ["17", "Technische Dokumentation erstellen", "Art. 31, Annex VII", "P4.6", "offen"],
        ["18", "EU-Konformitätserklärung ausstellen", "Art. 28, Annex V", "P4.7", "offen"],
        ["19", "CE-Kennzeichnung anbringen", "Art. 29 und 30", "P4.7", "offen"],
        ["20", "Supportzeitraum und Enddatum angeben", "Art. 13(8) und (19), Annex II Nr. 7",
         "P4.4", "offen"],
        ["21", "Informationen und Anweisungen für den Nutzer", "Annex II", "P4.8", "offen"],
        ["22", "Sorgfaltspflicht bei Drittkomponenten", "Art. 13(5)", "P4.2", "offen"],
    ], widths=[0.7, 5.4, 3.0, 2.1, 1.39], size=9, row_h=0.38, status_col=4)


def build_meldepflichten(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "5.3 Pflichten 23 bis 26: Meldepflichten", size=26)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if len(holders) >= 2:
        fill_bullets(holders[0],
                     [("Art. 14 ist seit 2026-09-11 in Kraft: 24 h, 72 h, 14 Tage, 1 Monat. Die "
                       "Fristen laufen ab Kenntnis der Schwachstelle oder des Vorfalls. Die Pflicht "
                       "greift erst mit der Bereitstellung auf dem Unionsmarkt; der Prototyp ohne "
                       "Weitergabe löst keine Meldepflicht aus.", None)], size=12)
        fill_bullets(holders[1], [
            ("23  Frühwarnung, 24 h", "Art. 14(2)(a) und 14(4)(a), Maßnahme P5.1"),
            ("24  Vollständige Meldung an CSIRT und ENISA, 72 h",
             "Art. 14(2)(b) und 14(4)(b), Maßnahmen P5.1 und P5.2"),
            ("25  Abschlussbericht aktiv ausgenutzter Schwachstellen, 14 Tage nach Verfügbarkeit "
             "einer Abhilfe", "Art. 14(2)(c), Maßnahme P5.1"),
            ("26  Abschlussbericht schwerer Sicherheitsvorfälle, 1 Monat nach der 72-h-Meldung",
             "Art. 14(4)(c), Maßnahme P5.1"),
            ("Bestandsprodukte, Art. 69(3)",
             "Die Meldepflicht gilt auch für Produkte, die vor dem 2027-12-11 in Verkehr gebracht "
             "wurden. Die übrigen Pflichten greifen nur bei wesentlicher Änderung."),
            ("Operativer Stand",
             "SRP seit 2026-09-11 in Betrieb, EU Login mit MFA als Assigned Representative, "
             "kein API, kein Durchführungsrechtsakt zu Art. 14(10)."),
        ], size=11.5)


def build_status_phase_1_3(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "4.2 Maßnahmen und Status, Phase 1 bis 3", size=24)
    frame = add_table(slide, CONTENT_TOP,
                      ["Nr.", "Maßnahme", "Risiko", "Detaildokument", "Status"], [
        ["P1.1", "Authentifizierung aller mutierenden Endpunkte", "R1", "Web_Auth_Plan", "läuft"],
        ["P1.2", "POST-Umstellung und CSRF-Schutz", "R4", "Web_Auth_Plan", "läuft"],
        ["P1.3", "Sensorkalibrierung begrenzen", "R3", "Sensor_Calibration_Plan", "läuft"],
        ["P1.4", "Harte Obergrenzen für PID-Gains", "R6", "P1-4_PID_Gain_Limits", "offen"],
        ["P1.5", "RED-Konformitätsweg und CE-Route bis 2027-12-11", "RED-1, RED-2",
         "P1-5_RED_Route", "ruht"],
        ["P2.1", "Pfadbereinigung, JSON nicht ausliefern", "R5", "Web_Auth_Plan", "teilweise"],
        ["P2.2", "AP-Passwort zur Laufzeit konfigurierbar", "R2", "AP_Password_Plan", "läuft"],
        ["P2.3", "Ratenbegrenzung und Save-Debouncing", "R7", "P2-3_Rate_Limiting", "offen"],
        ["P3.1", "Mutex für den settings-Zugriff", "R9", "P3-1_Mutex", "erledigt"],
        ["P3.2", "Passive Ablehnung neuer Clients", "R8", "kein Dokument", "erledigt"],
        ["P3.3", "Verschlüsselung auf Funkebene", "Funkebene", "P3-3_Radio_Encryption", "offen"],
    ], widths=[0.7, 5.4, 2.5, 2.6, 1.39], size=9, row_h=0.42, status_col=4)

    # Auslöser der Maßnahmen sichtbar machen: ohne Bereitstellung auf dem Unionsmarkt freiwillig.
    add_status_note(slide, frame)


def build_status_phase_4_5(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "4.3 Maßnahmen und Status, Phase 4 und 5", size=24)
    frame = add_table(slide, CONTENT_TOP, ["Nr.", "Maßnahme", "Detaildokument", "Status"], [
        ["P4.1", "Cybersicherheits-Risikobewertung", "P4-1_Risk_Assessment", "offen"],
        ["P4.2", "SBOM und Sorgfaltspflicht bei Drittkomponenten", "P4-2_SBOM", "offen"],
        ["P4.3", "Signierte OTA-Updates und Update-Tests", "P4-3_Signed_OTA", "offen"],
        ["P4.4", "Supportzeitraum und Enddatum", "P4-4_Support_Period", "offen"],
        ["P4.5", "Schwachstellenmanagement und Disclosure", "P4-5_Disclosure_Policy", "offen"],
        ["P4.6", "Technische Dokumentation, Annex VII", "P4-6_Technical_Documentation", "offen"],
        ["P4.7", "EU-Konformitätserklärung und CE", "P4-7_Declaration_of_Conformity", "offen"],
        ["P4.8", "Nutzerinformationen, Annex II", "P4-8_User_Information", "offen"],
        ["P5.1", "Meldeprozess nach Art. 14", "P5-1_Reporting_Process", "ruht"],
        ["P5.2", "Zugang zu SRP und CSIRT", "P5-2_SRP_CSIRT_Access", "ruht"],
    ], widths=[0.8, 6.2, 4.2, 1.39], size=9.5, row_h=0.45, status_col=3)

    add_status_note(slide, frame)


def build_risiken(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "4.1 Risikobild: Befunde R1 bis R10", size=24)
    # Die Spalte Maßnahme nennt die Gegenmaßnahme im Klartext. Kennungen wie P1.1
    # stehen erst dort, wo die Maßnahmen erklärt werden (4.2 und 4.3), damit das
    # Deck keine Verweise auf später Gezeigtes enthält.
    add_table(slide, CONTENT_TOP, ["Risiko", "Befund", "Wirkung", "Maßnahme", "Stand"], [
        ["R1", "Keine Authentifizierung der Steueroberfläche",
         "Jeder im WLAN kann Sollwerte und PID-Parameter setzen", "Anmeldung und Sitzungen",
         "läuft"],
        ["R2", "Fest verdrahtetes AP-Passwort im Quelltext",
         "Zugang für jeden, der Code oder Handbuch kennt", "Kennwort und Erstinbetriebnahme",
         "läuft"],
        ["R3", "Kalibrierung bildet den Druck zu niedrig ab",
         "Notabschaltung greift zu spät", "Kalibriergrenzen", "läuft"],
        ["R4", "Zustandsänderung per GET, CSRF",
         "Fremde Seite kann Regelparameter verändern", "POST und CSRF-Schutz", "läuft"],
        ["R5", "Beliebiger Dateizugriff auf LittleFS",
         "Konfigurationsdateien mit Passworthash lesbar", "Angriffsfläche", "teilweise"],
        ["R6", "Unbegrenzte PID-Gains über die Weboberfläche",
         "Instabile Regelung bis zur Abschaltung", "PID-Obergrenzen", "offen"],
    ], widths=[1.0, 4.2, 4.6, 1.8, 0.99], size=9.5, row_h=0.52, status_col=4)

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(5.35), Inches(12.59), Inches(0.9))
    frame = box.text_frame
    frame.word_wrap = True
    lines = [
        ("R7 bis R9 sind bearbeitet: Flash-Verschleiß und Client-Connect-DoS in v2.7.3, "
         "Synchronisation der settings erledigt.", 11, False, GRAY),
        ("Offen: R10 Funkebene ohne eigene Authentifizierung, der Steuerverkehr läuft als "
         "Klartext über HTTP.", 11, False, GRAY),
        ("Vollständige Liste mit Evidenz: analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md",
         11, False, GRAY),
    ]
    for index, (text, size, bold, color) in enumerate(lines):
        paragraph = frame.paragraphs[0] if index == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = text
        run.font.name = BODY_FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color


def build_aktiver_strang(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "4.4 Aktiver Strang: was jetzt umgesetzt wird", size=24)
    add_table(slide, CONTENT_TOP, ["Maßnahme", "Umsetzung", "Wirkung"], [
        ["P1.3", "Kalibriergrenzen auf den Sensorbereich begrenzen, sensor_maxP kleiner gleich "
                 "SENSOR_MAX_BAR", "Die Notabschaltung rechnet mit plausiblem Druck"],
        ["P1.1", "Login-Seite, Session für alle mutierenden Endpunkte, Erstinbetriebnahme ohne "
                 "Standardpasswort", "Kein Zugriff ohne Anmeldung"],
        ["P1.2", "/set und /resetPID auf POST, CSRF-Schutz",
         "Keine Zustandsänderung per Link oder Fremdseite"],
        ["P2.2", "AP-Passwort zur Laufzeit setzen, Ablage im verschlüsselten NVS",
         "Kein Passwort mehr im Quelltext und im Handbuch"],
    ], widths=[1.5, 7.6, 3.49], size=10.5, row_h=0.62)

    box = slide.shapes.add_textbox(Inches(LEFT), Inches(4.75), Inches(12.59), Inches(0.9))
    frame = box.text_frame
    frame.word_wrap = True
    lines = [
        ("Reihenfolge: P1.3 zuerst, dann P1.1 und P1.2 gemeinsam, P2.2 danach.", 11.5, True, BRAND),
        ("Jede Maßnahme hat ein Detaildokument mit Ist-Zustand, Umsetzungsschritten und "
         "Verifikationsteil.", 11, False, GRAY),
    ]
    for index, (text, size, bold, color) in enumerate(lines):
        paragraph = frame.paragraphs[0] if index == 0 else frame.add_paragraph()
        run = paragraph.add_run()
        run.text = text
        run.font.name = BODY_FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color


def build_quellen(prs):
    slide = new_slide(prs, "CONTENT B_V1")
    set_title(slide, "6.2 Quellen", size=26)
    fill_bodies(slide, [
        [
            ("Regulation (EU) 2024/2847", "CRA-Volltext, lokal unter sources/"),
            ("Delegierte VO (EU) 2022/30", "Cyber-Anforderungen des RED"),
            ("Delegierte VO (EU) 2023/2444", "Anwendungsdatum 2025-08-01"),
            ("Delegierte VO (EU) 2026/339", "Aufhebung von 2022/30 ab 2027-12-11"),
            ("Durchführungsbeschluss (EU) 2025/138", "EN 18031-1/-2/-3 mit Restriktionen"),
            ("Durchführungsverordnung (EU) 2025/2392", "Kategorien wichtiger Produkte"),
        ],
        [
            ("Leitlinien C(2026) 5252", "vom 27.07.2026, Abschnitt 9.1 zu Art. 14"),
            ("EU-Kommission", "CRA-Politik, CRA Reporting, Standardisation M/606"),
            ("ENISA Single Reporting Platform", "portal.cra-srp.enisa.europa.eu"),
            ("ENISA CSIRT-Koordinatorenliste", "veröffentlicht am 2026-09-04"),
            ("Delegierte VO (EU) 2026/881", "Verzögerung der Weitergabe einer Meldung"),
            ("ETSI EN 303 645", "technischer Maßstab, keine Rechtswirkung"),
        ],
    ], size=11)


def build_revision(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "6.3 Status und Revision", size=26)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if len(holders) >= 2:
        fill_bullets(holders[0], [("Was jetzt läuft und was bis zur Marktentscheidung ruht", None)],
                     size=12)
        fill_bullets(holders[1], [
            ("Stand",
             "Aktiv: P1.1 bis P1.3 und P2.2. Ruhend: P1.5 sowie P4.1 bis P4.8, P5.1 und P5.2. "
             "Einzelstand in 4.2 und 4.3"),
            ("Nächster Schritt",
             "P1.3 umsetzen, danach P1.1 und P1.2 gemeinsam, P2.2 im Anschluss"),
            ("Revision des Masters",
             "Arbeitsliste vom 2026-08-29, Umbau zum Master am 2026-09-28, Trennung der Regime "
             "am 2026-09-30, Prototypenregel und Status zurückgestellt am 2026-10-01, Befund "
             "R10 aufgenommen am 2026-10-06."),
        ], size=12)


# --------------------------------------------------------------------------- #
def main():
    prs = Presentation(TEMPLATE)
    removed = drop_all_slides(prs)

    build_cover(prs)
    build_inhaltsverzeichnis(prs)
    build_divider(prs, "SUBSECTION 4", "1 Systembeschreibung",
                  "Systemübersicht und Bedienoberfläche aus dem Benutzerhandbuch")
    build_systemuebersicht(prs)
    build_bedienung(prs)
    build_divider(prs, "SUBSECTION 1", "2 Regelwerk",
                  "Aus dem Aufbau folgt die Rechtsfrage: RED-Cyber bis 2027-12-11, danach die CRA")
    build_red_cra(prs)
    build_produktklassifizierung(prs)
    build_divider(prs, "SUBSECTION 2", "3 Produkt und Anwendbarkeit",
                  "Aus dem Regelwerk folgt die Einordnung: Produkt, Anwendbarkeit und die Fälle "
                  "rund um den Prototypen")
    build_produkt(prs)
    build_anwendbarkeit(prs)
    build_prototypen(prs)
    build_divider(prs, "SUBSECTION 3", "4 Risiken und Maßnahmen",
                  "Aus der Einordnung folgen die Befunde: kritische Risiken, Gegenmaßnahmen mit "
                  "Stand je Phase, dazu der aktive Strang")
    build_risiken(prs)
    build_status_phase_1_3(prs)
    build_status_phase_4_5(prs)
    build_aktiver_strang(prs)
    build_divider(prs, "SUBSECTION 5", "5 Pflichten der CRA",
                  "Aus den Maßnahmen folgen die Pflichten: Pflichten 1 bis 26 aus Annex I, "
                  "Art. 13, Art. 14 und Art. 28")
    build_pflichten_1_10(prs)
    build_pflichten_11_22(prs)
    build_meldepflichten(prs)
    build_divider(prs, "SUBSECTION 4", "6 Dokumente und Quellen",
                  "Belege zu den Pflichten: Dokumentenregister, Rechtsakte, Leitlinien und Historie")
    build_register(prs)
    build_quellen(prs)
    build_revision(prs)
    build_divider(prs, "SUBSECTION 5", "7 Fazit und nächste Schritte",
                  "Was aus den Erkenntnissen für VENTREX folgt")
    build_was_folgt(prs)
    new_slide(prs, "CLOSING SLIDE")

    collected = []
    for slide in prs.slides:
        for shape in slide.shapes:
            if shape.has_text_frame:
                collected.append(shape.text_frame.text)
            if getattr(shape, "has_table", False) and shape.has_table:
                for row in shape.table.rows:
                    for cell in row.cells:
                        collected.append(cell.text)
    joined = "\n".join(collected)
    assert "\u2014" not in joined, "Em-Dash im erzeugten Text gefunden"
    for token in ("Hier kommt der Folientitel", "Name Pr\u00e4sentation", "20.05.2026",
                  "Segnaposto", "DIAGRAMM", "STEP 1"):
        assert token not in joined, "Vorlagenrest gefunden: " + token

    prs.save(OUT)
    size_mb = os.path.getsize(OUT) / 1048576.0
    print("Beispielfolien entfernt:", removed)
    print("Folien im Ergebnis:    ", len(prs.slides))
    print("Datei:                 ", OUT)
    print("Größe MB:            ", round(size_mb, 2))


if __name__ == "__main__":
    main()
