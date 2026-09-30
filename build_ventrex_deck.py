# -*- coding: utf-8 -*-
"""
Erzeugt die CRA-Praesentation aus der Firmenvorlage.

Vorlage:  sec/VENTREX PPT Vorlage 2026.pptx
Quelle:   sec/CRA_Compliance_Checklist.md (Master, Stand 2026-09-30)
Ziel:     sec/2026-09-30_CRA_Compliance_Praesentation.pptx

Aufruf:
    "C:\\venvs\\VentCon2\\Scripts\\python.exe" build_ventrex_deck.py
"""

import copy
import os

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import PP_PLACEHOLDER
from pptx.enum.text import MSO_ANCHOR
from pptx.oxml.ns import qn
from pptx.util import Inches, Pt

TEMPLATE = "sec/VENTREX PPT Vorlage 2026.pptx"
OUT = "sec/2026-09-30_CRA_Compliance_Praesentation.pptx"

FOOTER_NAME = "VentCon2 - EU CRA Compliance"
FOOTER_DATE = "30.09.2026"

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
    """Liefert das Layout aus dem Master, den die Vorlage dafuer vorsieht."""
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
    """Kopiert Fusszeile, Datum und Foliennummer aus dem Layout auf die Folie.

    python-pptx kopiert diese drei Platzhalter nicht mit, PowerPoint legt sie
    beim Einfuegen einer Folie aber selbst an. Die Kopie bringt das Feld
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
        # SLIDE_NUMBER bleibt unberuehrt, damit das Feld erhalten bleibt


def new_slide(prs, layout_name):
    layout = layout_of(prs, layout_name)
    slide = prs.slides.add_slide(layout)
    clone_footer_placeholders(slide, layout)
    set_footer(slide)
    return slide


def by_type(slide, ph_type):
    """Platzhalter nach Typ, sortiert nach Lage statt nach ID.

    In CONTENT A_V2 traegt ID 13 den unteren Textbereich und ID 14 den
    Untertitel, die ID-Reihenfolge ist dort also nicht brauchbar.
    """
    found = [ph for ph in slide.placeholders if ph.placeholder_format.type == ph_type]
    # Toleranz beim Top-Vergleich: die Vorlage legt Spalten auf gleicher Hoehe mit
    # Abweichungen von wenigen EMU an (11 EMU in CONTENT B_V1). Ohne Rundung
    # gewinnt die linke Spalte nicht, und die Spalten waeren vertauscht.
    return sorted(found, key=lambda p: (round((p.top or 0) / 45720.0), (p.left or 0)))


def set_title(slide, text, size=None, on_dark=False):
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
    if "erledigt" in text or "geklaert" in text:
        return GREEN
    if "akut" in text:
        return CORAL
    if "teilweise" in text:
        return OLIVE
    return GRAY


def add_table(slide, top, headers, rows, widths, size=9.5, row_h=0.4, status_col=None):
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
    for row_index, row in enumerate(rows, start=1):
        fill = WHITE if row_index % 2 else ALT
        for col, value in enumerate(row):
            bold = False
            color = DARK
            if status_col is not None and col == status_col:
                bold = True
                color = status_color(str(value))
            put(table.cell(row_index, col), str(value), size, bold, color, fill)
    return table


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
            "Stand 2026-09-30   |   Grundlage: sec/CRA_Compliance_Checklist.md",
        ], size=11, space_after=2)


def build_divider(prs, layout_name, title, line=None):
    slide = new_slide(prs, layout_name)
    set_title(slide, title)
    if line:
        fill_bodies(slide, [[(line, None)]], size=12)


def build_anwendbarkeit(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Anwendbarkeit", size=26)
    add_table(slide, CONTENT_TOP, ["Frage", "Antwort", "Status"], [
        ["Produkt mit digitalen Elementen?",
         "Ja, ESP32-S3 Ventilsteuerung mit Webinterface und Netzwerkverbindung", "Geklaert"],
        ["Internetfaehiges Funkgeraet?",
         "Ja, ESP32 mit Wi-Fi; die Cyber-Anforderungen des RED gelten bis 2027-12-11",
         "Geklaert"],
        ["Gewerblich in der EU in Verkehr gebracht?",
         "Zu klaeren; reine Prototyp-Nutzung waere nicht betroffen", "Offen"],
        ["Vertrieb nur in Drittlaendern, zum Beispiel Indien?",
         "Dann greifen weder CRA noch RED-Cyber; Anknuepfungspunkt ist der Unionsmarkt, Art. 2 und 3",
         "Geklaert"],
        ["Lieferung an EU-Kunden, auch unentgeltlich?",
         "Bereitstellung auf dem Unionsmarkt nach Art. 3(22), CRA und RED-Cyber greifen",
         "Zu beachten"],
        ["Weiterverkauf durch EU-Importeur oder Haendler?",
         "Pflichten nach Art. 19 und Art. 20; mit eigenem Namen Hersteller nach Art. 21",
         "Geklaert"],
        ["Vertreter fuer einen Hersteller ausserhalb der Union?",
         "Art. 18 erlaubt einen schriftlich mandatierten Vertreter fuer die Marktaufsicht",
         "Offen"],
        ["Erste Entscheidung, vor allem anderen",
         "Inverkehrbringung feststellen: davon haengen CE-Weg, technische Dokumentation, "
         "Supportzeitraum und Meldepflichten ab", "Offen"],
    ], widths=[4.3, 6.9, 1.39], size=10.5, row_h=0.55, status_col=2)


def build_red_cra(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "RED und CRA: zwei Regime, ein Uebergang", size=24)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if len(holders) >= 2:
        fill_bullets(holders[0],
                     [("Zwei Regime, ein gemeinsamer Anknuepfungspunkt: die Bereitstellung auf dem "
                       "Unionsmarkt. Ab 2027-12-11 gehen sie ineinander ueber.", None)], size=12)
        fill_bullets(holders[1], [
            ("RED-Cyber, gilt bis 2027-12-11",
             "Art. 3(3)(d)(e)(f) der Richtlinie 2014/53/EU und Delegierte VO (EU) 2022/30; "
             "anwendbar seit 2025-08-01, Aufhebung durch Delegierte VO (EU) 2026/339"),
            ("RED-Pflichten und Nachweisweg",
             "RED-1 Netzwerkschutz und RED-2 Datenschutz einschlaegig, RED-3 Betrugsschutz nicht. "
             "Selbstbewertung; bei den ausgenommenen Passwortklauseln 6.2.5.1 und 6.2.5.2 der "
             "EN 18031 Modul B+C oder Modul H, Entscheidung in P1.5"),
            ("CRA, Verordnung (EU) 2024/2847",
             "Art. 14 in Kraft seit 2026-09-11, vollstaendig ab 2027-12-11; Pflichten 1 bis 26, "
             "Meldepflichten 24 h, 72 h, 14 Tage, 1 Monat"),
            ("CRA-Nachweisweg",
             "Modul A, Selbstbewertung; keine harmonisierte Norm zitiert, Art. 27 derzeit nicht "
             "nutzbar"),
            ("Vertrieb nur in Drittlaendern",
             "Zum Beispiel Indien: keine Pflicht nach CRA oder RED-Cyber. Sicherheitsarbeit und "
             "Updates bleiben freiwillig, aber ratsam"),
            ("EU-Importeur oder Haendler",
             "Wer die Geraete in die Union bringt, traegt die Pflichten nach Art. 19 und Art. 20; "
             "mit eigenem Namen oder wesentlicher Aenderung wird er Hersteller nach Art. 21"),
        ], size=11)


def build_produktklassifizierung(prs):
    slide = new_slide(prs, "CONTENT B_V1")
    set_title(slide, "Produktklassifizierung und Normenlage", size=24)
    fill_bodies(slide, [
        [
            ("Standardprodukt", "Annex III und Annex IV nicht betroffen"),
            ("Wichtige Produkte Class I oder II", "Nein"),
            ("Kritische Produkte", "Nein"),
            ("Konformitaetsbewertung", "Modul A, Selbstbewertung, keine notifizierte Stelle"),
        ],
        [
            ("Normenstand CRA",
             "Keine CRA-harmonisierte Norm ist im Amtsblatt zitiert, Stand 2026-09-30. Die "
             "Vermutung nach Art. 27 besteht derzeit nicht; der Normungsauftrag M/606 laeuft."),
            ("Nachweisweg", "Risikobewertung P4.1 und technische Dokumentation P4.6"),
            ("RED-Route getrennt gefuehrt",
             "Die Modul-A-Bewertung gilt fuer die CRA. Fuer die RED-Route bis 2027-12-11 gilt der "
             "Vorbehalt der EN 18031; der Konformitaetsweg wird in P1.5 geklaert."),
        ],
    ], size=11.5)


def build_register(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Dokumentenregister", size=26)
    add_table(slide, CONTENT_TOP, ["Dokument oder Gruppe", "Rolle", "Massnahmen"], [
        ["CRA_Compliance_Checklist.md", "Master (Grundlage), einzige Statusquelle", "P1.1 bis P5.2"],
        ["analyse/...Sicherheitsanalyse.md", "Risiko- und Evidenzanalyse", "R1 bis R9"],
        ["plan/ 4 vorhandene Plaene", "Web_Auth, Kalibrierung, AP-Passwort, Mutex",
         "P1.1, P1.2, P1.3, P2.1, P2.2, P3.1"],
        ["plan/ 14 Skelette und Plaene", "Abschnitte 3 bis 6 der Skelette noch offen",
         "P1.4, P1.5, P2.3, P3.3, P4.1 bis P4.8, P5.1, P5.2"],
        ["grundlagen/ 2 Dokumente", "Mutex sowie Web und Authentifizierung",
         "P3.1, P1.1, P1.2, P2.1"],
        ["plan/ 3 Rahmenplaene", "Dokumentenstruktur, Abgrenzung der beiden Regime, Praesentation",
         "RED-1 bis RED-3, P1.5"],
        ["sources/...Regulation_2024_2847_EN.pdf", "Normquelle (CRA-Volltext)", "keine"],
        ["26 Dokumente insgesamt", "1 Master, 1 Analyse, 21 Plaene, 2 Grundlagen, 1 Normquelle",
         "P3.2 ohne Detaildokument, Nachweis R8"],
    ], widths=[4.4, 5.4, 2.79], size=9.5, row_h=0.5)


def build_pflichten_1_10(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Pflichten 1 bis 10: Sicherheitsanforderungen", size=24)
    add_table(slide, CONTENT_TOP,
              ["Nr.", "Pflicht", "CRA-Referenz", "Risiko / Massnahme", "Status"], [
        ["1", "Keine universellen Standardpasswoerter",
         "Annex I, Teil I, 2(b)/(d); EN 303 645 5.1-1 (technischer Massstab)", "R2 / P2.2", "offen"],
        ["2", "Authentifizierung und Zugriffskontrolle", "Annex I, Teil I, 1.2", "R1 / P1.1",
         "offen"],
        ["3", "Minimale Angriffsflaeche", "Annex I, Teil I, 1.4", "R5 / P2.1", "offen"],
        ["4", "Schutz vor unbefugtem Zugriff, Eingabevalidierung", "Annex I, Teil I, 1.7",
         "R6 / P1.4", "offen"],
        ["5", "Sichere Kommunikation", "Annex I, Teil I, 1.8", "Funkebene / P3.3", "offen"],
        ["6", "Sichere Standardkonfiguration", "Annex I, Teil I, 1.10", "R4 / P1.2", "offen"],
        ["7", "Sichere Update-Mechanismen, signiert", "Annex I, Teil I, 1.9", "P4.3", "offen"],
        ["8", "Verfuegbarkeit und DoS-Resistenz", "Annex I, Teil I, 1.3", "R7, R8 / P2.3, P3.2",
         "offen"],
        ["9", "Produktsicherheit, Notabschaltung", "Annex I, Teil I, 1.7", "R3 / P1.3", "offen"],
        ["10", "Sichere Speicherung von Zugangsdaten",
         "Annex I, Teil I, 2(e); EN 303 645 5.4-1 (technischer Massstab)", "R2 / P2.2", "offen"],
    ], widths=[0.6, 4.6, 3.3, 2.7, 1.39], size=9, row_h=0.45, status_col=4)


def build_pflichten_11_22(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Pflichten 11 bis 22: Schwachstellen, Konformitaet, Markt", size=24)
    add_table(slide, CONTENT_TOP, ["Nr.", "Pflicht", "Referenz", "Massnahme", "Status"], [
        ["11", "SBOM pflegen", "Annex I, Teil II, Nr. 1", "P4.2", "offen"],
        ["12", "Schwachstellen identifizieren und dokumentieren", "Annex I, Teil II, Nr. 1",
         "P4.5", "offen"],
        ["13", "Schwachstellen ohne Verzoegerung beheben", "Annex I, Teil II, Nr. 2", "P4.3",
         "offen"],
        ["14", "Disclosure-Richtlinie und Sicherheitskontakt", "Annex I, Teil II, Nr. 4",
         "P4.5", "offen"],
        ["15", "Sicherheitsupdates testen vor Auslieferung", "Annex I, Teil II, Nr. 3", "P4.3",
         "offen"],
        ["16", "Cybersicherheits-Risikobewertung dokumentieren", "Art. 13(2)", "P4.1", "offen"],
        ["17", "Technische Dokumentation erstellen", "Annex VII", "P4.6", "offen"],
        ["18", "EU-Konformitaetserklaerung ausstellen", "Annex V", "P4.7", "offen"],
        ["19", "CE-Kennzeichnung anbringen", "Art. 28", "P4.7", "offen"],
        ["20", "Supportzeitraum und Enddatum angeben", "Art. 13(8)", "P4.4", "offen"],
        ["21", "Informationen und Anweisungen fuer den Nutzer", "Annex II", "P4.8", "offen"],
        ["22", "Sorgfaltspflicht bei Drittkomponenten", "Art. 13(5)", "P4.2", "offen"],
    ], widths=[0.7, 5.4, 3.0, 2.1, 1.39], size=9, row_h=0.38, status_col=4)


def build_meldepflichten(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "Pflichten 23 bis 26: Meldepflichten", size=26)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if len(holders) >= 2:
        fill_bullets(holders[0],
                     [("Art. 14 ist seit 2026-09-11 in Kraft: 24 h, 72 h, 14 Tage, 1 Monat", None)],
                     size=12)
        fill_bullets(holders[1], [
            ("23  Fruehwarnung, 24 h", "Art. 14(2)(a) und 14(4)(a), Massnahme P5.1"),
            ("24  Vollstaendige Meldung an CSIRT und ENISA, 72 h",
             "Art. 14(2)(b) und 14(4)(b), Massnahmen P5.1 und P5.2"),
            ("25  Abschlussbericht aktiv ausgenutzter Schwachstellen, 14 Tage",
             "Art. 14(2)(c), Massnahme P5.1"),
            ("26  Abschlussbericht schwerer Sicherheitsvorfaelle, 1 Monat",
             "Art. 14(4)(c), Massnahme P5.1"),
            ("Bestandsprodukte, Art. 69(3)",
             "Die Meldepflicht gilt auch fuer Produkte, die vor dem 2027-12-11 in Verkehr gebracht "
             "wurden. Die uebrigen Pflichten greifen nur bei wesentlicher Aenderung."),
            ("Operativer Stand",
             "SRP seit 2026-09-11 in Betrieb, EU Login mit MFA als Assigned Representative, "
             "kein API, kein Durchfuehrungsrechtsakt zu Art. 14(10)."),
        ], size=11.5)


def build_status_phase_1_3(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Status Phase 1 bis 3 mit Risikobezug", size=24)
    add_table(slide, CONTENT_TOP, ["Nr.", "Massnahme", "Risiko", "Detaildokument", "Status"], [
        ["P1.1", "Authentifizierung aller mutierenden Endpunkte", "R1", "Web_Auth_Plan", "offen"],
        ["P1.2", "POST-Umstellung und CSRF-Schutz", "R4", "Web_Auth_Plan", "offen"],
        ["P1.3", "Sensorkalibrierung begrenzen", "R3", "Sensor_Calibration_Plan", "offen"],
        ["P1.4", "Harte Obergrenzen fuer PID-Gains", "R6", "P1-4_PID_Gain_Limits", "offen"],
        ["P1.5", "RED-Konformitaetsweg und CE-Route bis 2027-12-11", "RED-1, RED-2",
         "P1-5_RED_Route", "offen"],
        ["P2.1", "Pfadbereinigung, JSON nicht ausliefern", "R5", "Web_Auth_Plan", "teilweise"],
        ["P2.2", "AP-Passwort zur Laufzeit konfigurierbar", "R2", "AP_Password_Plan", "offen"],
        ["P2.3", "Ratenbegrenzung und Save-Debouncing", "R7", "P2-3_Rate_Limiting", "offen"],
        ["P3.1", "Mutex fuer den settings-Zugriff", "R9", "P3-1_Mutex", "erledigt"],
        ["P3.2", "Passive Ablehnung neuer Clients", "R8", "kein Dokument", "erledigt"],
        ["P3.3", "Verschluesselung auf Funkebene", "Funkebene", "P3-3_Radio_Encryption", "offen"],
    ], widths=[0.7, 5.4, 2.5, 2.6, 1.39], size=9, row_h=0.42, status_col=4)


def build_status_phase_4_5(prs):
    slide = new_slide(prs, "CONTENT A_V1")
    set_title(slide, "Status Phase 4 und 5", size=24)
    add_table(slide, CONTENT_TOP, ["Nr.", "Massnahme", "Detaildokument", "Status"], [
        ["P4.1", "Cybersicherheits-Risikobewertung", "P4-1_Risk_Assessment", "offen"],
        ["P4.2", "SBOM und Sorgfaltspflicht bei Drittkomponenten", "P4-2_SBOM", "offen"],
        ["P4.3", "Signierte OTA-Updates und Update-Tests", "P4-3_Signed_OTA", "offen"],
        ["P4.4", "Supportzeitraum und Enddatum", "P4-4_Support_Period", "offen"],
        ["P4.5", "Schwachstellenmanagement und Disclosure", "P4-5_Disclosure_Policy", "offen"],
        ["P4.6", "Technische Dokumentation, Annex VII", "P4-6_Technical_Documentation", "offen"],
        ["P4.7", "EU-Konformitaetserklaerung und CE", "P4-7_Declaration_of_Conformity", "offen"],
        ["P4.8", "Nutzerinformationen, Annex II", "P4-8_User_Information", "offen"],
        ["P5.1", "Meldeprozess nach Art. 14", "P5-1_Reporting_Process", "akut"],
        ["P5.2", "Zugang zu SRP und CSIRT", "P5-2_SRP_CSIRT_Access", "akut"],
    ], widths=[0.8, 6.2, 4.2, 1.39], size=9.5, row_h=0.45, status_col=3)


def build_zeitplan(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "Zeitplan der Fristen", size=26)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if holders:
        fill_bullets(holders[0], [
            ("Art. 14 ist seit dem 11.09.2026 verbindlich, Phase 5 ist noch nicht eingerichtet", None),
        ], size=12)
    add_table(slide, 2.0, ["Datum", "Ereignis oder Pflicht"], [
        ["2024-12-10", "CRA in Kraft getreten"],
        ["2025-01-30", "EN 18031-1/-2/-3 im Amtsblatt zitiert, mit Restriktionen bei den "
                       "Passwortklauseln"],
        ["2025-08-01", "Delegierte VO (EU) 2022/30 in der Fassung 2023/2444 anwendbar; RED-Cyber "
                       "gilt bis 2027-12-11"],
        ["2026-06-11", "Kapitel IV, Konformitaetsbewertungsstellen, anwendbar"],
        ["2026-09-11", "Meldepflichten nach Art. 14 in Kraft"],
        ["2027-12-11", "CRA gilt vollstaendig; RED-Cyber endet, Aufhebung der Delegierten "
                       "VO (EU) 2022/30"],
    ], widths=[2.2, 10.39], size=11, row_h=0.5)


def build_sanktionen(prs):
    slide = new_slide(prs, "CONTENTDARK 1")
    set_title(slide, "Sanktionsrahmen nach Art. 64", size=26, on_dark=True)
    fill_bodies(slide, [[
        ("Stufe 1: bis 15.000.000 EUR oder 2,5 %",
         "Verstoesse gegen Annex I, Art. 13 und Art. 14"),
        ("Stufe 2: bis 10.000.000 EUR oder 2 %",
         "Sonstige Pflichten, unter anderem Art. 28 und die Konformitaetspflichten"),
        ("Stufe 3: bis 5.000.000 EUR oder 1 %",
         "Falsche, unvollstaendige oder irrefuehrende Informationen"),
        ("Getrennt davon: Verstoesse gegen die Funkrichtlinie",
         "Sanktion nach nationalem Recht der Mitgliedstaaten zur Richtlinie 2014/53/EU, "
         "nicht nach Art. 64 der CRA"),
        ("Art. 64(10): Ausnahme fuer Kleinst- und Kleinunternehmen",
         "Nur fuer die 24-h-Frist des Art. 14(2)(a) und 14(4)(a), nicht fuer die uebrigen "
         "Fristen und Pflichten"),
    ]], size=12.5, on_dark=True)


def build_quellen(prs):
    slide = new_slide(prs, "CONTENT B_V1")
    set_title(slide, "Quellen", size=26)
    fill_bodies(slide, [
        [
            ("Regulation (EU) 2024/2847", "CRA-Volltext, lokal unter sources/"),
            ("Delegierte VO (EU) 2022/30", "Cyber-Anforderungen des RED"),
            ("Delegierte VO (EU) 2023/2444", "Anwendungsdatum 2025-08-01"),
            ("Delegierte VO (EU) 2026/339", "Aufhebung von 2022/30 ab 2027-12-11"),
            ("Durchfuehrungsbeschluss (EU) 2025/138", "EN 18031-1/-2/-3 mit Restriktionen"),
            ("Durchfuehrungsverordnung (EU) 2025/2392", "Kategorien wichtiger Produkte"),
        ],
        [
            ("Leitlinien C(2026) 5252", "vom 27.07.2026, Abschnitt 9.1 zu Art. 14"),
            ("EU-Kommission", "CRA-Politik, CRA Reporting, Standardisation M/606"),
            ("ENISA Single Reporting Platform", "portal.cra-srp.enisa.europa.eu"),
            ("ENISA CSIRT-Koordinatorenliste", "veroeffentlicht am 2026-09-04"),
            ("Delegierte VO (EU) 2026/881", "Verzoegerung der Weitergabe einer Meldung"),
        ],
    ], size=11)


def build_revision(prs):
    slide = new_slide(prs, "CONTENT A_V2")
    set_title(slide, "Revision und naechste Schritte", size=26)
    holders = by_type(slide, PP_PLACEHOLDER.OBJECT)
    if len(holders) >= 2:
        fill_bullets(holders[0], [("Drei Schritte mit unmittelbarem Handlungsbedarf", None)],
                     size=12)
        fill_bullets(holders[1], [
            ("Sofort: Phase 5 aufsetzen",
             "Prozess, zwei benannte Personen, SRP-Zugang, Testlauf. Art. 14 ist seit 2026-09-11 "
             "in Kraft."),
            ("Als Naechstes: Phase 1 umsetzen",
             "Authentifizierung, POST und CSRF, harte Grenzen fuer Kalibrierung und PID-Gains."),
            ("Parallel: Skelette fuellen",
             "14 Detaildokumente, der Ist-Zustand ist in jedem Skelett bereits belegt."),
            ("Revision des Masters",
             "Arbeitsliste vom 2026-08-29, Umbau zum Master am 2026-09-28, Trennung der Regime "
             "am 2026-09-30: RED-Cyber gilt bis 2027-12-11, danach greift die CRA vollstaendig."),
        ], size=12.5)


# --------------------------------------------------------------------------- #
def main():
    prs = Presentation(TEMPLATE)
    removed = drop_all_slides(prs)

    build_cover(prs)
    build_divider(prs, "SUBSECTION 1", "Rahmen und Anwendbarkeit")
    build_anwendbarkeit(prs)
    build_red_cra(prs)
    build_produktklassifizierung(prs)
    build_register(prs)
    build_divider(prs, "SUBSECTION 2", "Pflichten der CRA")
    build_pflichten_1_10(prs)
    build_pflichten_11_22(prs)
    build_meldepflichten(prs)
    build_divider(prs, "SUBSECTION 3", "Verknuepfung und Status")
    build_status_phase_1_3(prs)
    build_status_phase_4_5(prs)
    build_divider(prs, "SUBSECTION 4", "Fristen und Sanktionen",
                  "Zeitplan bis 2027-12-11 und Sanktionsrahmen nach Art. 64")
    build_zeitplan(prs)
    build_sanktionen(prs)
    build_divider(prs, "SUBSECTION 5", "Quellen und Revision",
                  "Rechtsakte, Leitlinien und Dokumentenhistorie")
    build_quellen(prs)
    build_revision(prs)
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
    print("Groesse MB:            ", round(size_mb, 2))


if __name__ == "__main__":
    main()
