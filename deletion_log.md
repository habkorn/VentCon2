# Loeschprotokoll (Bulk-Operationen)

Format: Datum, Uhrzeit, Pfad, Umfang, Grund, Freigabe-Status.
Regel aus `.github/copilot-instructions.md`, Abschnitt "Bulk Deletion Protocol".

| Datum | Uhrzeit | Pfad | Umfang | Grund | Freigabe |
|---|---|---|---|---|---|
| 2026-09-29 | 20:10 | `C:\Dropbox\Tom\AIProjects\VentCon2\.venv` | 1 Ordner, 1638 Dateien | Der Paket-Installer hat beim Installieren von `python-pptx` ein Python-venv im Dropbox-Projektordner erzeugt. Die Projektregel "Never create Python virtual environments inside Dropbox-synced folders" verbietet das. Ersatz: `C:\venvs\VentCon2` (zentraler lokaler Cache, Muster a). | Vorab per Rueckfrage eingeholt, Dry-Run-Liste mit Pfad und Dateizahl gezeigt, Pfad war gitignored und ausschliesslich generierter Inhalt (pip, setuptools, python-pptx), keine Nutzerdaten |
| 2026-09-30 | 13:10 | `build_ventrex_deck.py`, `verify_ventrex_deck.py` | 2 Dateien | Beide Skripte wurden durch fehlerhaft angewandte Patches an ueber zehn Stellen beschaedigt (Textfragmente mitten in Zeilen, dadurch Syntaxfehler). Reparatur durch Neuanlage ersetzt das Flicken. | Vorab per Rueckfrage eingeholt (Auswahl "Ja, beide loeschen und neu schreiben"), Dry-Run mit beiden Pfaden gezeigt, beide Dateien untracked und erst am selben Tag von mir erzeugt |
| 2026-09-30 | 13:15 | `sec/2026-09-29_CRA_Compliance_Praesentation.pptx` | 1 Datei, 85.649 Bytes | Abgeloeste Praesentation im alten Design ohne Firmenvorlage. Ersetzt durch `sec/2026-09-30_CRA_Compliance_Praesentation.pptx` aus der VENTREX-Vorlage. | Durch Entscheidung E1 des Plans vom 2026-09-30 gedeckt, Dry-Run-Liste der PPTX-Dateien vor dem Loeschen gezeigt, Datei war untracked |

## Hinweise

- Dropbox-Sync war zum Zeitpunkt der Loeschung nicht pausiert. Die Loeschung propagiert daher
  auf alle synchronisierten Maschinen. Betroffen war nur generierter Inhalt, eine
  Wiederherstellung ist nicht erforderlich. Dropbox haelt zusaetzlich 30 Tage Versionshistorie vor.
- Geprueft wurde vor der Loeschung, dass `.venv/` in `.gitignore` steht, es sich also nicht um
  git-getrackten Inhalt handelt.
- Angelegt wurde der Ordner am 2026-09-29 um 19:55:58, geloescht am 2026-09-29 um 20:10.
