# -*- coding: utf-8 -*-
"""
Erzeugt eine PowerPoint-Praesentation aus dem CRA-Master-Dokument.

Quelle:  sec/CRA_Compliance_Checklist.md (Stand 2026-09-28)
Ziel:    sec/2026-09-29_CRA_Compliance_Praesentation.pptx

Aufruf:
    "C:/Program Files/Python314/python.exe" build_cra_presentation.py
"""

from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

OUT = "sec/2026-09-29_CRA_Compliance_Praesentation.pptx"

# --- Farben (angelehnt an die Web-Oberflaeche) --------------------------------
BLUE = RGBColor(0x00, 0x2F, 0x87)
BLUE_DARK = RGBColor(0x00, 0x1F, 0x5A)
GREEN = RGBColor(0x32, 0xC0, 0x9D)
DARK = RGBColor(0x1F, 0x29, 0x37)
GRAY = RGBColor(0x6B, 0x72, 0x80)
LIGHT = RGBColor(0xF3, 0xF4, 0xF6)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
AMBER = RGBColor(0xB4, 0x53, 0x09)
RED = RGBColor(0xB9, 0x1C, 0x1C)

FONT = "Segoe UI"

SW = 13.333
SH = 7.5
FOOTER_TEXT = ("VentCon2  |  EU CRA Compliance  |  Stand 2026-09-29  |  "
               "Quelle: sec/CRA_Compliance_Checklist.md")

prs = Presentation()
prs.slide_width = Inches(SW)
prs.slide_height = Inches(SH)
BLANK = prs.slide_layouts[6]

_collected_text = []


def _track(txt):
    _collected_text.append(txt)


def box(slide, l, t, w, h, fill=None, line=None, line_w=Pt(1), shape=MSO_SHAPE.RECTANGLE):
    s = slide.shapes.add_shape(shape, Inches(l), Inches(t), Inches(w), Inches(h))
    if fill is None:
        s.fill.background()
    else:
        s.fill.solid()
        s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line
        s.line.width = line_w
    s.shadow.inherit = False
    return s


def text(slide, l, t, w, h, runs, size=14, color=DARK, bold=False, align=PP_ALIGN.LEFT,
         line_spacing=1.15, space_after=6, anchor=MSO_ANCHOR.TOP):
    """runs: str oder Liste von (text, dict) Tupeln."""
    tb = slide.shapes.add_textbox(Inches(l), Inches(t), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    if isinstance(runs, str):
        runs = [(runs, {})]
    first = True
    for item in runs:
        s, opts = item if isinstance(item, tuple) else (item, {})
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.alignment = opts.get("align", align)
        p.line_spacing = opts.get("line_spacing", line_spacing)
        p.space_after = Pt(opts.get("space_after", space_after))
        if s == "":
            continue
        r = p.add_run()
        r.text = s
        f = r.font
        f.name = FONT
        f.size = Pt(opts.get("size", size))
        f.bold = opts.get("bold", bold)
        f.color.rgb = opts.get("color", color)
        _track(s)
    return tb


def new_slide(title, subtitle=None, kicker=None):
    slide = prs.slides.add_slide(BLANK)
    box(slide, 0, 0, SW, 0.10, fill=BLUE)
    if kicker:
        text(slide, 0.62, 0.26, 12.1, 0.28, kicker.upper(), size=10.5, color=GREEN, bold=True)
        ty = 0.52
    else:
        ty = 0.36
    text(slide, 0.62, ty, 12.1, 0.55, title, size=25, color=BLUE, bold=True, space_after=0)
    if subtitle:
        text(slide, 0.62, ty + 0.56, 12.1, 0.34, subtitle, size=12, color=GRAY, space_after=0)
        ly = ty + 1.02
    else:
        ly = ty + 0.62
    box(slide, 0.62, ly, 12.11, 0.022, fill=GREEN)
    return slide, ly + 0.22


def footer(slide, idx):
    text(slide, 0.62, 7.02, 10.6, 0.3, FOOTER_TEXT, size=8.5, color=GRAY, space_after=0)
    text(slide, 11.3, 7.02, 1.43, 0.3, str(idx), size=8.5, color=GRAY,
         align=PP_ALIGN.RIGHT, space_after=0)


def status_color(value):
    v = value.lower()
    if "erledigt" in v or "geklaert" in v and "zu klaeren" not in v:
        return GREEN
    if "akut" in v or "kritisch" in v:
        return RED
    if "offen" in v or "zu klaeren" in v or "teilweise" in v or "skelett" in v:
        return AMBER
    return DARK


def table(slide, top, headers, rows, widths, size=10.5, header_size=10.5, row_h=0.34,
          status_cols=(), bold_cols=()):
    n_rows = len(rows) + 1
    shape = slide.shapes.add_table(n_rows, len(headers), Inches(0.62), Inches(top),
                                   Inches(sum(widths)), Inches(row_h * n_rows))
    tbl = shape.table
    tbl.first_row = False
    tbl.horz_banding = False
    for i, w in enumerate(widths):
        tbl.columns[i].width = Inches(w)
    tbl.rows[0].height = Inches(row_h + 0.04)
    for i in range(1, n_rows):
        tbl.rows[i].height = Inches(row_h)

    for c, h in enumerate(headers):
        cell = tbl.cell(0, c)
        cell.text = h
        cell.fill.solid()
        cell.fill.fore_color.rgb = BLUE
        cell.vertical_anchor = MSO_ANCHOR.MIDDLE
        cell.margin_left = Inches(0.07)
        cell.margin_right = Inches(0.05)
        cell.margin_top = Inches(0.02)
        cell.margin_bottom = Inches(0.02)
        p = cell.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.LEFT
        r = p.runs[0]
        r.font.name = FONT
        r.font.size = Pt(header_size)
        r.font.bold = True
        r.font.color.rgb = WHITE
        _track(h)

    for ri, row in enumerate(rows, start=1):
        for ci, val in enumerate(row):
            cell = tbl.cell(ri, ci)
            cell.text = str(val)
            cell.fill.solid()
            cell.fill.fore_color.rgb = WHITE if ri % 2 else LIGHT
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.margin_left = Inches(0.07)
            cell.margin_right = Inches(0.05)
            cell.margin_top = Inches(0.02)
            cell.margin_bottom = Inches(0.02)
            p = cell.text_frame.paragraphs[0]
            r = p.runs[0]
            r.font.name = FONT
            r.font.size = Pt(size)
            r.font.bold = ci in bold_cols
            if ci in status_cols:
                r.font.color.rgb = status_color(str(val))
                r.font.bold = True
            else:
                r.font.color.rgb = DARK
            _track(str(val))
    return tbl


def rows_height(n, row_h):
    return n * row_h


def bullets(slide, top, items, size=13, width=12.11, left=0.62, gap=0.30,
            color=DARK, marker_color=GREEN):
    y = top
    for it in items:
        if isinstance(it, tuple):
            head, sub = it
        else:
            head, sub = it, None
        box(slide, left, y + 0.08, 0.055, 0.22, fill=marker_color)
        text(slide, left + 0.20, y, width - 0.20, 0.3, head, size=size, color=color,
             bold=True, space_after=0)
        y += 0.30
        if sub:
            text(slide, left + 0.20, y, width - 0.20, 0.3, sub, size=size - 1.5, color=GRAY,
                 space_after=0)
            y += 0.26 + (0.12 if len(sub) > 95 else 0)
        y += gap
    return y


def callout(slide, top, title, body, height=0.95, fill=RGBColor(0xEC, 0xF2, 0xFC),
            bar=BLUE, width=12.11, size=12):
    box(slide, 0.62, top, width, height, fill=fill)
    box(slide, 0.62, top, 0.07, height, fill=bar)
    text(slide, 0.86, top + 0.10, width - 0.40, 0.3, title, size=size, color=bar, bold=True,
         space_after=0)
    text(slide, 0.86, top + 0.40, width - 0.40, height - 0.45, body, size=size - 1,
         color=DARK, space_after=0)


# =============================================================================
# 1 Titel
# =============================================================================
s = prs.slides.add_slide(BLANK)
box(s, 0, 0, SW, SH, fill=BLUE)
box(s, 0, 4.28, SW, 0.05, fill=GREEN)
box(s, 0.9, 5.95, 3.0, 0.05, fill=GREEN)
text(s, 0.9, 2.05, 11.5, 0.5, "REGULATION (EU) 2024/2847", size=13, color=GREEN, bold=True,
     space_after=0)
text(s, 0.9, 2.55, 11.5, 1.6,
     [("EU CRA Compliance", {"size": 46, "bold": True, "color": WHITE, "space_after": 4}),
      ("VentCon2 - Pressure Control System", {"size": 20, "color": RGBColor(0xC7, 0xD7, 0xF2),
                                              "space_after": 0})])
text(s, 0.9, 4.55, 11.5, 1.2,
     [("26 Pflichten aus Annex I, Art. 13, Art. 14 und Art. 28", {"size": 15, "color": WHITE,
                                                                  "space_after": 4}),
      ("20 Massnahmen P1.1 bis P5.2 mit je einem Detaildokument",
       {"size": 15, "color": WHITE, "space_after": 4}),
      ("Grundlage: sec/CRA_Compliance_Checklist.md  |  Stand 2026-09-29",
       {"size": 15, "color": RGBColor(0xC7, 0xD7, 0xF2), "space_after": 0})])

# =============================================================================
# 2 Agenda
# =============================================================================
s, y = new_slide("Aufbau des Dokuments", "Zehn Abschnitte im Master, dieser Vortrag folgt ihnen",
                 "Agenda")
items = [
    ("1  Anwendbarkeit", "Produkt mit digitalen Elementen, RED gegen CRA, erste Entscheidung"),
    ("2  Dokumentenregister", "23 Dokumente: Master, Analyse, Plaene, Grundlagen, Normquelle"),
    ("3  Produktklassifizierung", "Standardprodukt, Modul A, Vorbehalt aus dem RED"),
    ("4  Pflichten-Checkliste", "26 Pflichten, durchgehend nummeriert, mit Bezug zu Annex I"),
    ("5  Verknuepfung", "Risiko, Pflicht, Massnahme und Detaildokument je Zeile"),
    ("6  Arbeitspakete", "Fuenf Phasen mit Status, einzige Statusquelle"),
    ("7  Zeitplan", "Von 2024-12-10 bis 2027-12-11"),
    ("8 bis 10  Strafen, Quellen, Revision", "Sanktionsrahmen, normativer Bezug, Historie"),
]
bullets(s, y, items, size=12.5, gap=0.10)
footer(s, 2)

# =============================================================================
# 3 Rolle und Status
# =============================================================================
s, y = new_slide("Rolle und Status des Dokuments",
                 "sec/CRA_Compliance_Checklist.md ist die Grundlage fuer alle Dokumente in sec/",
                 "Abschnitt 1 von 10")
callout(s, y, "Grundlage (Master)",
        "Alle Detaildokumente sind in Abschnitt 2 registriert und verweisen mit "
        "\u201eGrundlage: ../CRA_Compliance_Checklist.md\u201c auf dieses Dokument.",
        height=0.85)
y2 = y + 1.05
table(s, y2, ["Eigenschaft", "Wert"], [
    ["Status", "Arbeitsliste, offen (einzige Statusquelle)"],
    ["Vorgaenger", "Arbeitsliste vom 2026-08-29, umgebaut am 2026-09-28"],
    ["Normquelle", "sources/EU_CRA_Regulation_2024_2847_EN.pdf (CRA-Volltext)"],
    ["Umbauplan", "plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md"],
    ["Detaildokumente", "1 Analyse, 18 Plaene, 2 Grundlagenerklaerungen, 1 Normquelle"],
], widths=[3.1, 9.01], size=11.5, row_h=0.44, bold_cols=(0,))
footer(s, 3)

# =============================================================================
# 4 Anwendbarkeit
# =============================================================================
s, y = new_slide("Anwendbarkeit", "Fuenf Fragen, drei geklaert, eine offen", "Abschnitt 1")
table(s, y, ["Frage", "Antwort", "Status"], [
    ["Produkt mit digitalen Elementen?",
     "Ja, ESP32-S3 Ventilsteuerung mit Webinterface und Netzwerkverbindung", "Geklaert"],
    ["Gewerblich in der EU in Verkehr gebracht?",
     "Zu klaeren; reine Prototyp-Nutzung waere nicht betroffen", "Offen"],
    ["Funk unter RED 2014/53/EU?",
     "Ja, ESP32 mit Wi-Fi; RED-Cyber-Anforderungen gelten seit 2025-08-01", "Geklaert"],
    ["Verhaeltnis RED zu CRA",
     "Parallel bis 2027-12-11; Art. 14 gilt bereits seit 2026-09-11", "Geklaert"],
    ["Aufhebung der RED-Cyber-Anforderungen",
     "Delegierte VO (EU) 2026/339 hebt 2022/30 auf, wirksam ab 2027-12-11", "Geklaert"],
], widths=[3.5, 7.0, 1.61], size=11, row_h=0.62, status_cols=(2,))
footer(s, 4)

# =============================================================================
# 5 Erste Entscheidung
# =============================================================================
s, y = new_slide("Erste Entscheidung vor allen anderen",
                 "RED gegen CRA bestimmt Anforderungen und CE-Weg", "Abschnitt 1")
callout(s, y, "Zu klaeren: gewerbliche Inverkehrbringung in der EU",
        "Ist VentCon2 ein Produkt, das gewerblich in der EU in Verkehr gebracht wird? "
        "Reine Prototyp-Nutzung waere nicht betroffen. Davon haengt ab, ob CE-Weg, "
        "technische Dokumentation und Supportzeitraum ueberhaupt greifen.",
        height=1.15, fill=RGBColor(0xFD, 0xF3, 0xE3), bar=AMBER)
y += 1.40
bullets(s, y, [
    ("RED-Cyber gilt seit 2025-08-01",
     "Delegierte VO (EU) 2022/30 in der Fassung der Delegierten VO (EU) 2023/2444"),
    ("Die CRA-Meldepflichten gelten bereits parallel",
     "Art. 14 ist seit 2026-09-11 in Kraft, unabhaengig von der RED-Route"),
    ("Ab 2027-12-11 uebernimmt die CRA",
     "Delegierte VO (EU) 2026/339 hebt 2022/30 zum 2027-12-11 auf"),
], size=12.5, gap=0.16)
footer(s, 5)

# =============================================================================
# 6 bis 8 Dokumentenregister
# =============================================================================
s, y = new_slide("Dokumentenregister (1/3)",
                 "Master, Analyse und die bestehenden Umsetzungsplaene", "Abschnitt 2")
table(s, y, ["Datei", "Rolle", "Massnahmen", "Pruefung"], [
    ["CRA_Compliance_Checklist.md", "Master (Grundlage), einzige Statusquelle",
     "alle, P1.1 bis P5.2", "2026-09-28"],
    ["analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md", "Risiko- und Evidenzanalyse",
     "R1 bis R9", "2026-09-28"],
    ["plan/2026-08-29_P1-1_P1-2_P2-1_Web_Auth_Plan.md", "Umsetzungsplan",
     "P1.1, P1.2, P2.1", "2026-08-31"],
    ["plan/2026-08-31_P1-3_Sensor_Calibration_Plan.md", "Umsetzungsplan", "P1.3", "2026-08-31"],
    ["plan/2026-08-31_P2-2_AP_Password_Plan.md", "Umsetzungsplan", "P2.2", "2026-08-31"],
    ["plan/2026-08-27_P3-1_Mutex_Settings_Zugriff_Plan.md",
     "Umsetzungsplan (erledigt in v2.7.3)", "P3.1", "2026-08-27"],
], widths=[5.6, 3.5, 1.6, 1.41], size=10, row_h=0.5)
footer(s, 6)

s, y = new_slide("Dokumentenregister (2/3)", "Neue Skelette fuer die offenen Massnahmen",
                 "Abschnitt 2")
table(s, y, ["Datei", "Rolle", "Massnahmen", "Pruefung"], [
    ["plan/2026-09-28_P1-4_PID_Gain_Limits_Plan.md", "Umsetzungsplan (Skelett)", "P1.4", "2026-09-28"],
    ["plan/2026-09-28_P2-3_Rate_Limiting_Save_Debouncing_Plan.md", "Umsetzungsplan (Skelett)", "P2.3", "2026-09-28"],
    ["plan/2026-09-28_P3-3_Radio_Encryption_Plan.md", "Umsetzungsplan (Skelett)", "P3.3", "2026-09-28"],
    ["plan/2026-09-28_P4-1_Cybersecurity_Risk_Assessment.md", "Umsetzungsplan (Skelett)", "P4.1", "2026-09-28"],
    ["plan/2026-09-28_P4-2_SBOM.md", "Umsetzungsplan (Skelett)", "P4.2", "2026-09-28"],
    ["plan/2026-09-28_P4-3_Signed_OTA.md", "Umsetzungsplan (Skelett)", "P4.3", "2026-09-28"],
    ["plan/2026-09-28_P4-4_Support_Period.md", "Umsetzungsplan (Skelett)", "P4.4", "2026-09-28"],
], widths=[5.6, 3.5, 1.6, 1.41], size=10, row_h=0.48)
footer(s, 7)

s, y = new_slide("Dokumentenregister (3/3)",
                 "Weitere Skelette, Grundlagenerklaerungen, Strukturplan und Normquelle",
                 "Abschnitt 2")
table(s, y, ["Datei", "Rolle", "Massnahmen"], [
    ["plan/2026-09-28_P4-5_Vulnerability_Disclosure_Policy.md", "Umsetzungsplan (Skelett)", "P4.5"],
    ["plan/2026-09-28_P4-6_Technical_Documentation_Annex_VII.md", "Umsetzungsplan (Skelett)", "P4.6"],
    ["plan/2026-09-28_P4-7_EU_Declaration_of_Conformity_CE.md", "Umsetzungsplan (Skelett)", "P4.7"],
    ["plan/2026-09-28_P4-8_User_Information_Annex_II.md", "Umsetzungsplan (Skelett)", "P4.8"],
    ["plan/2026-09-28_P5-1_Reporting_Process.md", "Umsetzungsplan (Skelett)", "P5.1"],
    ["plan/2026-09-28_P5-2_SRP_CSIRT_Access.md", "Umsetzungsplan (Skelett)", "P5.2"],
    ["grundlagen/2026-08-28_Mutex_How_It_Works.md", "Grundlagenerklaerung", "P3.1"],
    ["grundlagen/2026-08-29_Web_Auth_How_It_Works.md", "Grundlagenerklaerung", "P1.1, P1.2, P2.1"],
    ["plan/2026-09-28_CRA_Checkliste_als_Master_Plan.md", "Strukturplan (kein CRA-Bezug)", "keine"],
    ["sources/EU_CRA_Regulation_2024_2847_EN.pdf", "Normquelle (CRA-Volltext)", "keine"],
], widths=[6.4, 3.6, 2.11], size=10, row_h=0.44)
text(s, 0.62, y + 4.75, 12.11, 0.3,
     "P3.2 hat bewusst kein Detaildokument und ist in Abschnitt 5 mit seinem Nachweis gefuehrt.",
     size=10.5, color=GRAY, space_after=0)
footer(s, 8)

# =============================================================================
# 9 Produktklassifizierung
# =============================================================================
s, y = new_slide("Produktklassifizierung", "Standardprodukt mit Selbstbewertung",
                 "Abschnitt 3")
table(s, y, ["Kategorie", "Einordnung"], [
    ["Standardprodukt (Annex III/IV nicht betroffen)", "Ja"],
    ["Wichtige Produkte Class I/II (Annex III)", "Nein"],
    ["Kritische Produkte (Annex IV)", "Nein"],
    ["Konformitaetsbewertung CRA", "Modul A (Selbstbewertung), keine notifizierte Stelle noetig"],
], widths=[6.0, 6.11], size=12, row_h=0.42, bold_cols=(0,))
y2 = y + 2.05
callout(s, y2, "Vorbehalt aus dem RED",
        "EN 18031-1/-2/-3 sind durch Durchfuehrungsbeschluss (EU) 2025/138 mit Restriktionen "
        "zitiert. Die Passwortklauseln 6.2.5.1 und 6.2.5.2 sind von der Konformitaetsvermutung "
        "ausgenommen. Werden ausgenommene Klauseln beruehrt, wechselt die RED-Route auf Modul B+C "
        "oder Modul H mit notifizierter Stelle. Fuer ein Geraet mit festem Standardpasswort ist "
        "das der Regelfall, siehe P2.2.",
        height=1.5, fill=RGBColor(0xFD, 0xF3, 0xE3), bar=AMBER, size=11.5)
footer(s, 9)

# =============================================================================
# 10 Normenstand
# =============================================================================
s, y = new_slide("Normenstand der CRA", "Keine harmonisierte Norm zitiert",
                 "Abschnitt 3")
callout(s, y, "Keine CRA-harmonisierte Norm im Amtsblatt (Stand 2026-09-28)",
        "Die Konformitaetsvermutung nach Art. 27 besteht derzeit nicht. Der Nachweis erfolgt "
        "ueber die Risikobewertung (P4.1) und die technische Dokumentation (P4.6). "
        "Der Normungsauftrag M/606 laeuft.",
        height=1.15)
y += 1.40
bullets(s, y, [
    ("ETSI: 17 vertikale Entwuerfe der EN 304 6xx in oeffentlicher Befragung",
     "Zitierung im Amtsblatt folgt fruehestens nach Abschluss der Entwuerfe"),
    ("Horizontal: CEN und CENELEC arbeiten an der EN 40000-Reihe", None),
    ("Konsequenz fuer VentCon2",
     "Konformitaet wird ueber Risikobewertung und Dokumentation belegt, nicht ueber Normvermutung"),
], size=12.5, gap=0.22)
footer(s, 10)

# =============================================================================
# 11 bis 15 Pflichten
# =============================================================================
s, y = new_slide("Pflichten 1 bis 5", "Abschnitt 4.1: Sicherheitsanforderungen, Annex I Teil I",
                 "Abschnitt 4 von 10")
table(s, y, ["Nr.", "Pflicht", "CRA-Referenz", "Risiko / Massnahme", "Status"], [
    ["1", "Keine universellen Standardpasswoerter",
     "Annex I, Teil I, 2(b)/(d); EN 303 645 5.1-1", "R2 / P2.2, Provisioning in P1.1", "offen"],
    ["2", "Authentifizierung und Zugriffskontrolle", "Annex I, Teil I, 1.2", "R1 / P1.1", "offen"],
    ["3", "Minimale Angriffsflaeche", "Annex I, Teil I, 1.4", "R5 / P2.1", "offen"],
    ["4", "Schutz vor unbefugtem Zugriff, Eingabevalidierung", "Annex I, Teil I, 1.7",
     "R6 / P1.4", "offen"],
    ["5", "Sichere Kommunikation", "Annex I, Teil I, 1.8", "Funkebene / P3.3", "offen"],
], widths=[0.7, 4.3, 3.2, 2.7, 1.21], size=10.5, row_h=0.72, status_cols=(4,))
footer(s, 11)

s, y = new_slide("Pflichten 6 bis 10", "Abschnitt 4.1: Sicherheitsanforderungen, Annex I Teil I",
                 "Abschnitt 4 von 10")
table(s, y, ["Nr.", "Pflicht", "CRA-Referenz", "Risiko / Massnahme", "Status"], [
    ["6", "Sichere Standardkonfiguration", "Annex I, Teil I, 1.10", "R4 / P1.2", "offen"],
    ["7", "Sichere Update-Mechanismen (signiert)", "Annex I, Teil I, 1.9", "P4.3", "offen"],
    ["8", "Verfuegbarkeit / DoS-Resistenz", "Annex I, Teil I, 1.3", "R7, R8 / P2.3, P3.2", "offen"],
    ["9", "Produktsicherheit (Notabschaltung)", "Annex I, Teil I, 1.7", "R3 / P1.3", "offen"],
    ["10", "Sichere Speicherung von Zugangsdaten", "Annex I, Teil I, 2(e); EN 303 645 5.4-1",
     "R2 / P2.2 (verschluesseltes NVS)", "offen"],
], widths=[0.7, 4.3, 3.2, 2.7, 1.21], size=10.5, row_h=0.72, status_cols=(4,))
footer(s, 12)

s, y = new_slide("Pflichten 11 bis 15",
                 "Abschnitt 4.2: Schwachstellenbehandlung, Annex I Teil II", "Abschnitt 4")
table(s, y, ["Nr.", "Pflicht", "CRA-Referenz", "Massnahme", "Status"], [
    ["11", "SBOM (Software Bill of Materials) pflegen", "Annex I, Teil II, Nr. 1", "P4.2", "offen"],
    ["12", "Schwachstellen identifizieren und dokumentieren", "Annex I, Teil II, Nr. 1", "P4.5", "offen"],
    ["13", "Schwachstellen ohne Verzoegerung beheben", "Annex I, Teil II, Nr. 2", "P4.3", "offen"],
    ["14", "Vulnerability-Disclosure-Richtlinie und Sicherheitskontakt", "Annex I, Teil II, Nr. 4",
     "P4.5", "offen"],
    ["15", "Sicherheitsupdates testen vor Auslieferung", "Annex I, Teil II, Nr. 3", "P4.3", "offen"],
], widths=[0.7, 5.7, 3.0, 1.5, 1.21], size=10.5, row_h=0.68, status_cols=(4,))
footer(s, 13)

s, y = new_slide("Pflichten 16 bis 22", "Abschnitt 4.3: Konformitaet und Markt", "Abschnitt 4")
table(s, y, ["Nr.", "Pflicht", "Referenz", "Massnahme", "Status"], [
    ["16", "Cybersicherheits-Risikobewertung erstellen und dokumentieren", "Art. 13(2)", "P4.1", "offen"],
    ["17", "Technische Dokumentation erstellen", "Annex VII", "P4.6", "offen"],
    ["18", "EU-Konformitaetserklaerung ausstellen", "Annex V", "P4.7", "offen"],
    ["19", "CE-Kennzeichnung anbringen", "Art. 28", "P4.7", "offen"],
    ["20", "Supportzeitraum und Enddatum angeben (mindestens 5 Jahre)", "Art. 13(8)", "P4.4", "offen"],
    ["21", "Informationen und Anweisungen fuer den Nutzer", "Annex II", "P4.8", "offen"],
    ["22", "Sorgfaltspflicht bei Drittkomponenten", "Art. 13(5)", "P4.2", "offen"],
], widths=[0.7, 5.7, 3.0, 1.5, 1.21], size=10.5, row_h=0.5, status_cols=(4,))
footer(s, 14)

s, y = new_slide("Pflichten 23 bis 26",
                 "Abschnitt 4.4: Meldepflichten, Art. 14, in Kraft seit 2026-09-11",
                 "Abschnitt 4, akut")
table(s, y, ["Nr.", "Pflicht", "Frist", "CRA-Referenz", "Massnahme", "Status"], [
    ["23", "Fruehwarnung aktiv ausgenutzter Schwachstellen oder schwerer Vorfaelle", "24 h",
     "Art. 14(2)(a), 14(4)(a)", "P5.1", "akut"],
    ["24", "Vollstaendige Meldung an CSIRT und ENISA (Single Reporting Platform)", "72 h",
     "Art. 14(2)(b), 14(4)(b)", "P5.1, P5.2", "akut"],
    ["25", "Abschlussbericht aktiv ausgenutzter Schwachstellen", "14 Tage", "Art. 14(2)(c)",
     "P5.1", "akut"],
    ["26", "Abschlussbericht schwerer Sicherheitsvorfaelle", "1 Monat", "Art. 14(4)(c)",
     "P5.1", "akut"],
], widths=[0.7, 5.3, 1.2, 2.5, 1.2, 1.21], size=10.5, row_h=0.78, status_cols=(5,))
footer(s, 15)

# =============================================================================
# 16 Bestandsprodukte und operativer Stand
# =============================================================================
s, y = new_slide("Bestandsprodukte und operativer Meldeweg",
                 "Art. 69(3) und der Stand der Single Reporting Platform", "Abschnitt 4.4")
callout(s, y, "Art. 69(3): Meldepflicht gilt auch fuer Bestandsprodukte",
        "Die Meldepflicht nach Art. 14 erfasst Produkte, die vor dem 2027-12-11 in Verkehr "
        "gebracht wurden, ohne dass eine wesentliche Aenderung vorliegen muss. Fuer alle "
        "uebrigen Pflichten greift Art. 69(2): nur bei einer wesentlichen Aenderung.",
        height=1.15)
y += 1.40
table(s, y, ["Frage", "Stand 2026-09-28"], [
    ["Plattform", "portal.cra-srp.enisa.europa.eu, in Betrieb seit 2026-09-11"],
    ["Zugang", "EU Login mit Multi-Faktor-Authentifizierung, Rolle Assigned Representative"],
    ["Umfang", "Kein API im ersten Release, keine freiwilligen Meldungen nach Art. 15"],
    ["CSIRTs", "Liste der Koordinatoren seit 2026-09-04, Zuordnung nach Art. 14(7)"],
    ["Format", "Kein Durchfuehrungsrechtsakt zu Art. 14(10); ENISA-Pflichtfelder als Grundlage"],
], widths=[2.6, 9.51], size=11, row_h=0.5, bold_cols=(0,))
footer(s, 16)

# =============================================================================
# 17 und 18 Verknuepfung
# =============================================================================
s, y = new_slide("Verknuepfung: Risiko, Pflicht, Massnahme (1/2)",
                 "Jede Massnahme hat genau eine Zeile, Detailpfade stehen im Master",
                 "Abschnitt 5")
table(s, y, ["Risiko", "Nr.", "Massnahme", "Detaildokument", "Stand"], [
    ["R1 Keine Authentifizierung", "2", "P1.1 Authentifizierung aller mutierenden Endpunkte",
     "Web_Auth_Plan", "offen"],
    ["R4 Zustandsaenderung per GET, CSRF", "6", "P1.2 POST-Umstellung und CSRF-Schutz",
     "Web_Auth_Plan", "offen"],
    ["R3 Kalibrierung bildet Druck zu niedrig ab", "9", "P1.3 Sensorkalibrierung begrenzen",
     "Sensor_Calibration_Plan", "offen"],
    ["R6 Unbegrenzte PID-Gains", "4", "P1.4 Harte Obergrenzen fuer PID-Gains",
     "P1-4_PID_Gain_Limits", "offen"],
    ["R5 Beliebiger Dateizugriff", "3", "P2.1 Pfadbereinigung, JSON nicht ausliefern",
     "Web_Auth_Plan", "teilweise"],
    ["R2 Fest verdrahtetes AP-Passwort", "1, 10", "P2.2 AP-Passwort zur Laufzeit konfigurierbar",
     "AP_Password_Plan", "offen"],
    ["R7 Flash-Verschleiss und DoS", "8", "P2.3 Ratenbegrenzung und Save-Debouncing",
     "P2-3_Rate_Limiting", "offen"],
    ["R9 Fehlende Synchronisation", "Robustheit", "P3.1 Mutex fuer den settings-Zugriff",
     "P3-1_Mutex", "erledigt"],
    ["R8 Client-Connect-DoS", "8", "P3.2 Passive Ablehnung neuer Clients",
     "kein Dokument", "erledigt"],
    ["Funkebene", "5", "P3.3 Verschluesselung auf Funkebene", "P3-3_Radio_Encryption", "offen"],
], widths=[3.4, 1.1, 4.5, 2.1, 1.01], size=10, row_h=0.46, status_cols=(4,))
footer(s, 17)

s, y = new_slide("Verknuepfung: Normpflichten (2/2)",
                 "Massnahmen ohne Code-Befund, ueberwiegend Dokumentationspflichten",
                 "Abschnitt 5")
table(s, y, ["Nr.", "Massnahme", "Detaildokument", "Stand"], [
    ["16", "P4.1 Cybersicherheits-Risikobewertung", "P4-1_Cybersecurity_Risk_Assessment", "offen"],
    ["11, 22", "P4.2 SBOM und Sorgfaltspflicht bei Drittkomponenten", "P4-2_SBOM", "offen"],
    ["7, 13, 15", "P4.3 Signierte OTA-Updates und Update-Tests", "P4-3_Signed_OTA", "offen"],
    ["20", "P4.4 Supportzeitraum und Enddatum", "P4-4_Support_Period", "offen"],
    ["12, 14", "P4.5 Schwachstellenmanagement und Disclosure", "P4-5_Vulnerability_Disclosure", "offen"],
    ["17", "P4.6 Technische Dokumentation (Annex VII)", "P4-6_Technical_Documentation", "offen"],
    ["18, 19", "P4.7 EU-Konformitaetserklaerung und CE", "P4-7_EU_Declaration_of_Conformity", "offen"],
    ["21", "P4.8 Nutzerinformationen (Annex II)", "P4-8_User_Information", "offen"],
    ["23, 25, 26", "P5.1 Meldeprozess nach Art. 14", "P5-1_Reporting_Process", "akut"],
    ["24", "P5.2 Zugang zu SRP und CSIRT", "P5-2_SRP_CSIRT_Access", "akut"],
], widths=[1.4, 5.4, 4.3, 1.01], size=10, row_h=0.46, status_cols=(3,))
footer(s, 18)

# =============================================================================
# 19 bis 21 Arbeitspakete
# =============================================================================
s, y = new_slide("Arbeitspakete Phase 1 und 2",
                 "Sicherheitskritisch und Haertung, alle Punkte offen", "Abschnitt 6")
table(s, y, ["ID", "Massnahme", "Detaildokument", "Status"], [
    ["P1.1", "Authentifizierung aller mutierenden Endpunkte (Login plus Provisioning)",
     "Web_Auth_Plan", "offen"],
    ["P1.2", "POST-Umstellung und CSRF-Schutz fuer /set und /resetPID", "Web_Auth_Plan", "offen"],
    ["P1.3", "Sensorkalibrierung auf den Sensorbereich begrenzen (sensor_maxP <= 10 bar)",
     "Sensor_Calibration_Plan", "offen"],
    ["P1.4", "Harte Obergrenzen fuer PID-Gains", "P1-4_PID_Gain_Limits_Plan", "offen"],
    ["P2.1", "Pfadbereinigung in handleFileRead, JSON nicht ausliefern", "Web_Auth_Plan", "offen"],
    ["P2.2", "AP-Passwort zur Laufzeit konfigurierbar", "AP_Password_Plan", "offen"],
    ["P2.3", "Ratenbegrenzung und Save-Debouncing", "P2-3_Rate_Limiting_Plan", "offen"],
], widths=[0.9, 6.4, 3.6, 1.21], size=10.5, row_h=0.5, status_cols=(3,), bold_cols=(0,))
footer(s, 19)

s, y = new_slide("Arbeitspakete Phase 3 und 4 (Beginn)",
                 "Robustheit bis zur Risikobewertung", "Abschnitt 6")
table(s, y, ["ID", "Massnahme", "Detaildokument", "Status"], [
    ["P3.1", "Mutex fuer settings-Zugriff (v2.7.3)", "P3-1_Mutex_Settings_Zugriff_Plan", "erledigt"],
    ["P3.2", "Client-Connect-DoS (v2.7.3)", "Nachweis R8 in der Analyse", "erledigt"],
    ["P3.3", "Verschluesselung oder Authentifizierung auf Funkebene", "P3-3_Radio_Encryption_Plan", "offen"],
    ["P4.1", "Cybersicherheits-Risikobewertung dokumentieren", "P4-1_Risk_Assessment", "offen"],
    ["P4.2", "SBOM erstellen (Core, Bibliotheken, chart.js)", "P4-2_SBOM", "offen"],
    ["P4.3", "Signierte OTA-Mechanik implementieren und testen", "P4-3_Signed_OTA", "offen"],
    ["P4.4", "Supportzeitraum festlegen und Enddatum dokumentieren", "P4-4_Support_Period", "offen"],
], widths=[0.9, 6.4, 3.6, 1.21], size=10.5, row_h=0.5, status_cols=(3,), bold_cols=(0,))
footer(s, 20)

s, y = new_slide("Arbeitspakete Phase 4 (Rest) und Phase 5",
                 "Phase 5 ist akut, Art. 14 ist seit 2026-09-11 in Kraft", "Abschnitt 6")
table(s, y, ["ID", "Massnahme", "Detaildokument", "Status"], [
    ["P4.5", "Vulnerability-Disclosure-Richtlinie und Sicherheitskontakt",
     "P4-5_Disclosure_Policy", "offen"],
    ["P4.6", "Technische Dokumentation (Annex VII)", "P4-6_Technical_Documentation", "offen"],
    ["P4.7", "EU-Konformitaetserklaerung (Annex V) und CE-Kennzeichnung",
     "P4-7_EU_Declaration_of_Conformity", "offen"],
    ["P4.8", "Nutzerinformationen (Annex II) aktualisieren", "P4-8_User_Information", "offen"],
    ["P5.1", "Meldeprozess fuer aktiv ausgenutzte Schwachstellen und schwere Vorfaelle",
     "P5-1_Reporting_Process", "akut"],
    ["P5.2", "Zugang zur ENISA Single Reporting Platform und zustaendiges CSIRT",
     "P5-2_SRP_CSIRT_Access", "akut"],
], widths=[0.9, 6.4, 3.6, 1.21], size=10.5, row_h=0.62, status_cols=(3,), bold_cols=(0,))
footer(s, 21)

# =============================================================================
# 22 Zeitplan
# =============================================================================
s, y = new_slide("Zeitplan der Fristen", "Vier Stichtage bestimmen den Fahrplan", "Abschnitt 7")
table(s, y, ["Datum", "Ereignis / Pflicht"], [
    ["2024-12-10", "CRA in Kraft getreten"],
    ["2025-08-01", "Delegierte VO (EU) 2022/30 i.d.F. 2023/2444 (RED Cyber) anwendbar"],
    ["2026-06-11", "Kapitel IV (Konformitaetsbewertungsstellen) anwendbar"],
    ["2026-09-11", "Meldepflichten (Art. 14) in Kraft, seit diesem Datum verbindlich"],
    ["2027-12-11", "Vollstaendige Anwendung aller uebrigen Pflichten; Aufhebung der "
                   "RED-Cyber-Delegierten-VO durch VO (EU) 2026/339"],
], widths=[2.2, 9.91], size=11.5, row_h=0.62, bold_cols=(0,))
callout(s, y + 3.35, "Handlungsdruck heute",
        "Art. 14 ist seit 18 Tagen verbindlich, Phase 5 ist nicht eingerichtet. "
        "Alle uebrigen Pflichten laufen bis 2027-12-11.",
        height=0.9, fill=RGBColor(0xFD, 0xE7, 0xE7), bar=RED, size=11.5)
footer(s, 22)

# =============================================================================
# 23 Strafen
# =============================================================================
s, y = new_slide("Sanktionsrahmen", "Drei Stufen nach Art. 64", "Abschnitt 8")
table(s, y, ["Stufe", "Verstoss", "Obergrenze"], [
    ["1", "Annex I, Art. 13, Art. 14", "15.000.000 EUR oder 2,5 % des weltweiten Jahresumsatzes"],
    ["2", "Sonstige Pflichten, unter anderem Art. 28", "10.000.000 EUR oder 2 %"],
    ["3", "Falsche, unvollstaendige oder irrefuehrende Informationen", "5.000.000 EUR oder 1 %"],
], widths=[1.2, 5.4, 5.51], size=11.5, row_h=0.6, bold_cols=(0,))
callout(s, y + 2.4, "Art. 64(10): Ausnahme fuer Kleinst- und Kleinunternehmen",
        "Von Geldstrafen ausgenommen sind sie nur fuer die 24-h-Frist des Art. 14(2)(a) und "
        "Art. 14(4)(a), nicht fuer die uebrigen Fristen der Meldepflicht und nicht fuer die "
        "uebrigen Pflichten.",
        height=1.15, fill=RGBColor(0xFD, 0xF3, 0xE3), bar=AMBER, size=11.5)
footer(s, 23)

# =============================================================================
# 24 und 25 Quellen
# =============================================================================
s, y = new_slide("Quellen (1/2)", "Rechtsakte und Kommissionsmaterial", "Abschnitt 9")
bullets(s, y, [
    ("Regulation (EU) 2024/2847 (CRA)", "Volltext lokal unter sources/EU_CRA_Regulation_2024_2847_EN.pdf"),
    ("EU-Kommission: CRA und CRA Reporting", "Politik- und Meldeinformationen"),
    ("Leitlinien C(2026) 5252 vom 2026-07-27", "Abschnitt 9.1 zu Art. 14, rechtlich nicht bindend"),
    ("CRA Standardisation, Normungsauftrag M/606", "Stand der harmonisierten Normen"),
    ("ENISA: Single Reporting Platform und CSIRT-Liste", "Betrieb seit 2026-09-11, Liste vom 2026-09-04"),
], size=12.5, gap=0.20)
footer(s, 24)

s, y = new_slide("Quellen (2/2)", "RED-Rechtsakte und Durchfuehrungsrechtsakte", "Abschnitt 9")
bullets(s, y, [
    ("Delegierte VO (EU) 2022/30 (RED Cyber)", "Cyber-Anforderungen des RED fuer Funkanlagen"),
    ("Delegierte VO (EU) 2023/2444", "Anwendungsdatum 2025-08-01"),
    ("Delegierte VO (EU) 2026/339", "Aufhebung von 2022/30 ab 2027-12-11"),
    ("Durchfuehrungsbeschluss (EU) 2025/138", "EN 18031-1/-2/-3 mit Restriktionen zitiert"),
    ("Durchfuehrungsverordnung (EU) 2025/2392", "Kategorien wichtiger und kritischer Produkte"),
    ("Delegierte VO (EU) 2026/881", "Verzoegerung der Weitergabe einer Meldung durch das CSIRT"),
], size=12.5, gap=0.16)
footer(s, 25)

# =============================================================================
# 26 Revision und naechste Schritte
# =============================================================================
s, y = new_slide("Revision und naechste Schritte",
                 "Dokumentenhistorie und der unmittelbare Handlungsbedarf", "Abschnitt 10")
table(s, y, ["Datum", "Aenderung"], [
    ["2026-08-29", "Erstanlage als Arbeitsliste, abgeleitet aus der Webinterface-Sicherheitsanalyse"],
    ["2026-08-31", "Verweise auf die Plaene zu Authentifizierung, AP-Passwort und Kalibrierung"],
    ["2026-09-28", "Umbau zum Master: Register, Nummerierung 1 bis 26, Verknuepfungstabelle, "
                   "Art. 14 als in Kraft, Rechtsstand und Quellen erweitert"],
], widths=[2.2, 9.91], size=11, row_h=0.62, bold_cols=(0,))
y2 = y + 2.4
bullets(s, y2, [
    ("Sofort: Phase 5 aufsetzen", "Prozess, zwei benannte Personen, SRP-Zugang, Testlauf"),
    ("Als Naechstes: Phase 1 umsetzen", "Authentifizierung, POST und CSRF, harte Grenzen"),
    ("Parallel: Skelette fuellen", "13 Detaildokumente, Ist-Zustand ist jeweils bereits belegt"),
], size=12.5, gap=0.16)
footer(s, 26)

# --- Em-Dash-Kontrolle -------------------------------------------------------
joined = "\n".join(_collected_text)
assert "\u2014" not in joined, "Em-Dash im erzeugten Text gefunden"

prs.save(OUT)
print("gespeichert:", OUT)
print("Folien:", len(prs.slides))
