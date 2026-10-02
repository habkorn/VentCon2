# Plan: Ablage der Aalberts-Vorlagen ordnen

Datum: 2026-10-01
Gegenstand: der Ordner `sec/AalbertsChecklists`
Status: entschieden und umgesetzt am 2026-10-01

## Anlass

Der Ordner enthielt nach dem Ausfüllen der Vorlagen zwölf Dateien in vier Klassen: drei
Wordvorlagen, drei Markdown-Extrakte, drei befüllte Wordkopien und drei Dateien
`CRA_*_Answered.md`. Die letzte Klasse war ein Werkzeugfehler: `tools/docx_extract.py` hatte
zeitweise die befüllten Wordkopien als Quelle gelesen und damit Extrakte der Extrakte erzeugt.
Ohne Struktur war nicht erkennbar, welche Datei Quelle, welche Nachweis und welche Ausgabe für
Aalberts ist.

## Entscheidungen des Nutzers (2026-10-01)

| Nr. | Entscheidung |
|---|---|
| E57 | Ablage in drei Rollenordnern: `templates/` für die Originale, `extracts/` für die Markdown-Extrakte, `filled/` für die befüllten Wordkopien |
| E58 | Die drei Dubletten `CRA_*_Answered.md` werden gelöscht, mit Eintrag im `deletion_log.md`; die vollständige Liste wurde vorher als Dry-Run gezeigt |
| E59 | Die drei Wordvorlagen bleiben unversioniert und behalten ihre empfangenen Namen samt Auffälligkeiten, weil ihre Namen und Prüfsummen im Nachweis der Extrakte stehen |
| E60 | Die versionierten LaTeX-Buildreste bleiben vorerst liegen, das ist eine getrennte Aufgabe |

## Vorher und Nachher

| Datei | Vorher | Nachher |
|---|---|---|
| Wordvorlagen (Original, Aalberts) | `sec/AalbertsChecklists/` | `sec/AalbertsChecklists/templates/` |
| Markdown-Extrakte | `sec/AalbertsChecklists/` | `sec/AalbertsChecklists/extracts/` |
| Befüllte Wordkopien | `sec/AalbertsChecklists/` | `sec/AalbertsChecklists/filled/` |
| Extrakte der Wordkopien | `sec/AalbertsChecklists/` | gelöscht (E58) |

Ergebnis: neun Dateien in drei Ordnern, je Ordner ein Dokumenttyp, keine Dubletten.

## Werkzeugänderungen

| Werkzeug | Änderung |
|---|---|
| `tools/docx_extract.py` | liest aus `TEMPLATE_DIR`, schreibt nach `EXTRACT_DIR` statt neben die Quelle. Die Zeile `Source file` im Kopfblock trägt den neuen Pfad |
| `tools/verify_docx_extract.py` | sucht die Wordvorlagen in `TEMPLATE_DIR` und den zugehörigen Extrakt in `EXTRACT_DIR` |
| `tools/aalberts_fill.py` | liest Vorlagen aus `TEMPLATE_DIR`, schreibt Antworten nach `EXTRACT_DIR` und die Wordkopien nach `FILLED_DIR` |
| `tools/verify_aalberts_answers.py` | prüft Extrakte in `EXTRACT_DIR`, Kopien in `FILLED_DIR` und löst die Vorlage für den Prüfsummenvergleich über `TEMPLATE_DIR` auf |

Alle vier Werkzeuge tragen jetzt `BASE_DIR`, `TEMPLATE_DIR`, `EXTRACT_DIR` und bei Bedarf
`FILLED_DIR`, statt eines einzelnen `SOURCE_DIR`. Die Vorlagen bleiben unberührt.

## Verifikation

| Lauf | Ergebnis |
|---|---|
| Idempotenzprüfung vor dem Umbau | `tools/aalberts_fill.py` meldet 0 neue Markdown-Antworten, `git diff` zeigt keine Abweichung: die Extrakte entsprachen dem Werkzeugstand, ein Neuerzeugen war verlustfrei |
| `tools/docx_extract.py` | drei Extrakte neu erzeugt, Ablage in `extracts/` bestätigt |
| `tools/verify_docx_extract.py` | 27 Prüfungen, alle bestanden, Prüfsummen der Vorlagen unverändert |
| `tools/aalberts_fill.py` | Checkliste 31 Antworten und 3 Hinweise, Fragebogen 36 und 3, Addendum 0 und 2, Markdown und Word deckungsgleich |
| `tools/verify_aalberts_answers.py` | 164 Prüfungen, alle bestanden, alle Antworten fett und in `#002F87` |

Die sechs versionierten Dateien wurden mit `git mv` verschoben, der Index zeigt sie als
Umbenennung (`R`), die Historie bleibt erhalten. Nur die `Source file`-Zeile änderte sich in den
Extrakten, drei Zeilen in drei Dateien.

Fallstrick beim Verschieben: Windows vergleicht Pfade ohne Rücksicht auf Groß- und Kleinschreibung.
Ein Verschiebeziel mit abweichender Schreibweise hat die Vorlage `CRA product maturity
checklist - Sep 2026.docx` in `CRA Product Maturity Checklist - Sep 2026.docx` umbenannt. Der Name
wurde über zwei Umbenennungsschritte zurückgesetzt, die Prüfsummenprüfung im Antwortprüfer
bestätigt den Originalzustand.

## Löschung

| Datum | Uhrzeit | Umfang | Freigabe |
|---|---|---|---|
| 2026-10-01 | 15:31 | 3 Dateien, untracked | Dry-Run-Liste gezeigt, Freigabe "ja, löschen und protokollieren", Warnung zur Dropbox-Propagierung, Eintrag im `deletion_log.md` |

## Offen

- Die drei Wordvorlagen sind weiterhin nicht versioniert (E59, bewusst).
- Die versionierten LaTeX-Buildreste in `LaTeX/doc` und `LaTeX/manual` (11 Dateien) warten auf eine
  eigene Freigabe (E60).
- Angaben, die nur VENTREX machen kann, stehen weiter als `[to be completed]` in den Vorlagen.
