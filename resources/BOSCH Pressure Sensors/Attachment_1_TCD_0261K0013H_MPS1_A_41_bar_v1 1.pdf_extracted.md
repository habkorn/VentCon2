# Extracted Content: Attachment_1_TCD_0261K0013H_MPS1_A_41_bar_v1 1.pdf

**Source file:** `Attachment_1_TCD_0261K0013H_MPS1_A_41_bar_v1 1.pdf`
**Extraction method:** PyMuPDF (fitz) + pytesseract OCR fallback
**Total pages:** 68

---

## Page 1 (Method: embedded_text)

© Robert Bosch GmbH reserves all rights even in the event of industrial property rights. 
    We reserve all rights of disposal such as copying and passing on to third parties. 
    PS-SD/EPS21                                                          Confidential                                                             
MPS3evo | TCD RB Number: 0 261 KXX XXX | Edition/Version: X.X | Date: XX.XX.20XX 
 
TKU Technische Kundenunterlage 
  TCD Technical Customer Documentation 
Diese vorläufige TKU dokumentiert den gegenwärtigen Stand der abgestimmten Spezifikationen. Sie 
wird bestätigt, wenn die Produktvalidierung mit positivem Ergebnis abgeschlossen wurde. Diese TKU 
ist in zwei Sprachen verfasst. Im Zweifelsfall findet die deutsche Version Anwendung. 
 
This preliminary TCD document presents the status of the agreed specifications. It will be confirmed 
when all validation has been completed with positive results. This TCD is written in two languages. In 
case of doubt, German version must be applied. 
Produktbezeichnug / Product designation: 
Medium Pressure Sensor (MPS) 
Produktnummer / Product Number: 
 
0 261 544 02E 
Anwendung / Application:  
 
 
H2 – ICE  
Version / Edition: 
 
 
 
 
01 
Datum / Date: 
 
 
 
 
2022-09-12 
Vorläufige TKU / Preliminary TCD

---

## Page 2 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
2/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
INHALTSVERZEICHNIS/ TABLE OF CONTENTS              
1 
PRODUKTIDENTIFIKATION/ PRODUCT IDENTIFICATION ...................................................................................... 6 
2 
ALLGEMEINE PRODUKTBESCHREIBUNG ........................................................................................................... 7 
2.1 WESENTLICHE FUNKTIONEN UND EIGENSCHAFTEN DES PRODUKTS 
7 
2.1.1 
GRUNDAUFBAU DES SENSORS ..........................................................................................................7 
2.1.2 
TECHNISCHES PRINZIP ......................................................................................................................8 
2.2 BESTIMMUNGSGEMÄßER GEBRAUCH 
8 
2.3 PRODUKTSICHERHEIT 
9 
2.3.1 
FUNKTIONALE SICHERHEIT ................................................................................................................9 
2.3.2 
DATENSCHUTZ, CYBERSECURITY UND OVER-THE-AIR ASPEKTE .................................................... 10 
2.3.3 
SICHERHEITS- UND WARNHINWEISE ............................................................................................... 10 
2.4 KENNZEICHNUNG DES PRODUKTS 
12 
2.5 MAßE UND GEWICHTE 
12 
2.6 LEISTUNGSAUFNAHME/-ABGABE 
12 
2.7 ALLGEMEINER HINWEIS ZU SERVICE, REPARATUR UND WARTUNG 
12 
2.8 HINWEISE ZUR ENTSORGUNG UND ZUM RECYCLING 
12 
3 
SYSTEMBESCHREIBUNG .................................................................................................................................12 
3.1 SYSTEM OF INTEREST (SOI) 
12 
3.2 PRODUKTSCHNITTSTELLE 
14 
3.2.1 
MECHANISCHE SCHNITTSTELLE ..................................................................................................... 15 
3.2.2 
STECKER / STECKVERBINDUNG ELEKTRISCH ................................................................................... 15 
3.2.3 
STECKERBELEGUNG UND BESCHALTUNG DES SENSORS ................................................................ 15 
4 
TECHNISCHE DATEN MIT MESSGRÖßEN UND MESSBEDINGUNGEN .................................................................16 
4.1 FUNKTIONEN UND FUNKTIONSZUSTÄNDE (BETRIEBSARTEN), FUNKTIONALE KENNWERTE UND 
RANDBEDINGUNGEN 
16 
4.1.1 
ÜBERTRAGUNGSFUNKTION DRUCKSIGNAL ..................................................................................... 16 
4.1.2 
FEHLERDIAGNOSE .......................................................................................................................... 18 
4.1.3 
VERHALTEN NACH  RESET UND INITIALISIERUNG ............................................................................ 20 
4.1.4 
VERHALTEN NACH UNTER- UND ÜBERSPANNUNG........................................................................... 21 
4.1.5 
VERHALTEN BEI ÜBERTEMPERATURERKENNUNG ............................................................................ 21 
4.1.6 
VERHALTEN BEI ÜBERDRUCKERKENNUNG ..................................................................................... 21 
4.2 MECHANISCHE KENNWERTE 
21 
4.2.1 
DRÜCKE ......................................................................................................................................... 21 
4.2.2 
TOLERANZEN DER DRUCKMESSUNG ............................................................................................... 22 
4.2.3 
VIBRATIONSBELASTUNG ................................................................................................................. 22 
4.2.4 
MAXIMAL ZULÄSSIGES ANZUGSMOMENT ......................................................................................... 23 
4.3 ELEKTRISCHE KENNWERTE 
23 
4.4 KLIMATISCHE KENNWERTE 
24 
4.5 CHEMISCHE KENNWERTE 
25 
4.6 AKUSTISCHE KENNWERTE 
25 
4.7 LEBENSDAUER 
25 
4.8 TRANSPORT, MONTAGE, IN-/AUßERBETRIEBNAHME, LAGERUNG 
25 
4.8.1 
TRANSPORT ................................................................................................................................... 25 
4.8.2 
MONTAGE ...................................................................................................................................... 25 
4.8.3 
IN-/ AUßERBETRIEBNAHME ............................................................................................................. 27 
4.8.4 
LAGERUNG ..................................................................................................................................... 27 
4.8.5 
ELEKTRISCHE FEHLERDIAGNOSE ................................................................................................... 27 
4.8.6 
ELEKTRISCHE PRÜFUNG DER FUNKTION ........................................................................................ 28 
5 
SERIENBEGLEITENDE PRÜFUNGEN .................................................................................................................29 
6 
ERPROBUNG ...................................................................................................................................................29 
6.1 BOSCH-SEITIGE ERPROBUNG 
30 
6.1.1 
VIBRATIONSPRÜFUNG ..................................................................................................................... 30

---

## Page 3 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
3/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
6.1.2 
FALLPRÜFUNG ................................................................................................................................ 30 
6.1.3 
BERSTDRUCKPRÜFUNG .................................................................................................................. 30 
6.1.4 
MECHANISCH DYNAMISCHE PRÜFUNG: HOCHDRUCKPUSLVERSUCH ............................................... 31 
6.1.5 
HOCHTEMPERATURLAGERUNG ....................................................................................................... 31 
6.1.6 
TIEFTEMPERATURLAGERUNG ......................................................................................................... 31 
6.1.7 
TEMPERATURSCHOCKWECHSEL ..................................................................................................... 32 
6.1.8 
WASSERSCHUTZPRÜFUNG ............................................................................................................. 32 
6.1.9 
FEUCHTEWECHSEL/ FEUCHTEWECHSELPRÜFUNG .......................................................................... 32 
6.1.10 
CHEMISCHE BESTÄNDIGKEIT .......................................................................................................... 32 
6.1.11 
SALZNEBELPRÜFUNG ..................................................................................................................... 33 
6.1.12 
BCI-TEST ...................................................................................................................................... 33 
6.1.13 
ANTENNE ....................................................................................................................................... 34 
6.1.14 
ESD-TESTS ................................................................................................................................... 34 
6.1.15 
TEST-PULSE AUF SENSORLEITUNGEN ............................................................................................ 35 
6.2 KUNDENSEITIGE ERPROBUNG 
35 
7 
BEWERTUNG VON PRODUKTEN NACH VERWENDUNG BEI 0-KM IM FELD .........................................................35 
2 
GENERAL PRODUCT DESCRIPTION ..................................................................................................................37 
2.1 FUNCTIONS AND CHARACTERISTICS 
37 
2.1.1 
BASIC DESIGN ................................................................................................................................ 37 
2.1.2 
TECHNICAL PRINCIPLE .................................................................................................................... 38 
2.2 INTENDED USE 
39 
2.3 PRODUCT SAFETY 
39 
2.3.1 
FUNCTIONAL SAFETY ..................................................................................................................... 39 
2.3.2 
DATA PROTECTION, CYBER SECURITY AND OVER-THE-AIR ASPECTS............................................... 40 
2.3.3 
SAFETY AND WARNING NOTES ........................................................................................................ 40 
2.4 LABELING OF THE PRODUCT 
42 
2.5 DIMENSIONS AND WEIGHTS 
42 
2.6 POWER CONSUMPTION / POWER OUTPUT 
42 
2.7 GENERAL REMARKS ON SERVICE, REPAIR, AND MAINTENANCE 
42 
2.8 INFORMATION ON DISPOSAL AND RECYCLING 
42 
3 
SYSTEM DESCRIPTION ....................................................................................................................................42 
3.1 SYSTEM OF INTEREST (SOI) 
42 
3.2 PRODUCT INTERFACE 
44 
3.2.1 
MECHANICAL INTERFACE ................................................................................................................ 44 
3.2.2 
PLUG CONNECTOR / ELECTRICAL PLUG CONNECTION ..................................................................... 45 
3.2.3 
CONNECTION DIAGRAM AND WIRING OF THE SENSOR ..................................................................... 45 
4 
TECHNICAL DATA WITH MEASURED VARIABLES AND MEASURING CONDITIONS ...............................................46 
4.1 FUNCTIONS, FUNCTION STATES (MODES OF OPERATION), FUNCTIONAL CHARACTERISTICS AND BOUNDARY 
CONDITIONS 
46 
4.1.1 
TRANSFER FUNCTION OF PRESSURE SIGNAL .................................................................................. 46 
4.1.2 
ERROR DIAGNOSIS ......................................................................................................................... 48 
4.1.3 
BEHAVIOR AT STARTUP AND RESET ................................................................................................ 50 
4.1.4 
BEHAVIOR AT LOW / HIGH-VOLTAGE ............................................................................................... 50 
4.1.5 
BEHAVIOR AT OVERTEMPERATURE ................................................................................................. 51 
4.1.6 
BEHAVIOR AFTER OVER PRESSURE ................................................................................................ 51 
4.2 MECHANICAL CHARACTERISTICS 
51 
4.2.1 
PRESSURE RANGE ......................................................................................................................... 51 
4.2.2 
TOLERANCES PRESSURE SIGNAL ................................................................................................... 52 
4.2.3 
VIBRATION LOADS .......................................................................................................................... 52 
4.2.4 
PERMISSIBLE TIGHTENING TORQUE ............................................................................................... 53 
4.3 ELECTRICAL CHARACTERISTICS 
54 
4.4 CLIMATIC CHARACTERISTICS 
55 
4.5 CHEMICAL CHARACTERISTICS 
55 
4.6 ACOUSTIC CHARACTERISTICS 
55 
4.7 LIFETIME 
55 
4.8 TRANSPORT, ASSEMBLY, START AND END OF OPERATION, STORAGE 
55 
4.8.1 
TRANSPORT ................................................................................................................................... 56

---

## Page 4 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
4/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.8.2 
ASSEMBLY, START OF OPERATION ................................................................................................. 56 
4.8.3 
DISASSEMBLY AND DECOMMISSIONING ........................................................................................... 57 
4.8.4 
STORAGE ....................................................................................................................................... 57 
4.8.5 
ELECTRICAL ERROR DIAGNOSIS ..................................................................................................... 58 
4.8.6 
ELECTRICAL TESTING OF THE FUNCTION ........................................................................................ 58 
5 
SERIES-ACCOMPANYING TESTS ......................................................................................................................59 
6 
TESTING .........................................................................................................................................................59 
6.1 TESTING BY BOSCH 
60 
6.1.1 
VIBRATION TESTING ....................................................................................................................... 60 
6.1.2 
DROP TEST .................................................................................................................................... 60 
6.1.3 
BURST PRESSURE TEST ................................................................................................................. 60 
6.1.4 
MECHANICAL DYNAMIC TEST: HIGH PRESSURE PULSATION ............................................................. 61 
6.1.5 
HIGH TEMPERATURE STORAGE ....................................................................................................... 61 
6.1.6 
LOW TEMPERATURE STORAGE........................................................................................................ 61 
6.1.7 
TEMPERATURE SHOCK CYCLE TEST ................................................................................................ 62 
6.1.8 
WATER PROTECTION TEST .............................................................................................................. 62 
6.1.9 
HUMIDITY CHANGE TEST ................................................................................................................ 62 
6.1.10 
CHEMICAL DURABILITY ................................................................................................................... 62 
6.1.11 
SALT SPRAY TEST .......................................................................................................................... 63 
6.1.12 
BCI-TEST ...................................................................................................................................... 63 
6.1.13 
RADIATED INTERFERENCE (ANTENNA) ........................................................................................... 64 
6.1.14 
ESD-TEST ..................................................................................................................................... 64 
6.1.15 
TEST PULSE ON SENSOR LINES ...................................................................................................... 65 
6.2 TESTING BY CUSTOMER 
65 
7 
ASSESSMENT OF PRODUCTS RETURNED FROM 0-MILEAGE AND FROM THE FIELD ...........................................66 
8 
APPENDICES AND REFERENCES......................................................................................................................67 
8.1 REFERENCES STANDARDS 
67 
9 
HISTORIE/ HISTORY ........................................................................................................................................68 
10 KONTAKTPERSON / CONTACT PERSON ...........................................................................................................68 
 
 
 
Weitere Unterlagen / Furter Documents:

---

## Page 5 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
5/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
INFORMATIONEN ZU SICHERHEITSHINWEISEN IN DIESEM DOKUMENT 
 
 
 Dies ist das Sicherheitswarnsymbol. Es wird verwendet, um Sie auf mögliche Gefahren für 
Personenschäden aufmerksam zu machen. Befolgen Sie alle Sicherheitshinweise, die diesem Symbol folgen, 
um mögliche Verletzungen oder Todesfälle zu vermeiden. 
 
Die Risikoklassen in diesem Dokument entsprechen ANSI Z535.6-2011:  
 
 
GEFAHR 
Die GEFAHR deutet auf eine gefährliche Situation hin, die zum Tod oder zu schweren 
Verletzungen führt, wenn sie nicht vermieden wird. 
 
WARNUNG 
WARNUNG deutet auf eine gefährliche Situation hin, die zum Tod oder zu schweren Verletzungen 
führen könnte, wenn sie nicht vermieden wird. 
 
VORSICHT 
VORSICHT deutet auf eine gefährliche Situation hin, die zu geringfügigen oder moderaten  
Verletzungen führen könnte, wenn sie nicht vermieden wird. 
HINWEIS 
HINWEIS wird verwendet, um Praktiken zu adressieren, die nicht im Zusammenhang mit einer 
körperlichen Schädigung stehen. 
 
 
Die Sicherheitshinweise in Abschnitt Error! Reference source not found. SICHERHEITS- UND W
ARNHINWEISE sind wie folgt aufgebaut: 
 
 RisIkoklasse 
Art und Quelle der Gefahr! 
Folgen bei Nichteinhaltung 
• 
Maßnahmen zur Vermeidung der Gefahren 
 
 
In anderen Abschnitten sind die Sicherheitshinweise in dieser Form angegeben: 
 
 RISIKOKLASSE 
Sicherheitstext

---

## Page 6 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
6/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
1 
PRODUKTIDENTIFIKATION/ PRODUCT IDENTIFICATION 
  
 
Produktbezeichnung / Product designation: 
Medium Pressure Sensor 
Typbezeichnung / Type designation: 
MPS1-FC 
(Entwicklungs-)Teile-Nummer /  
(Development) Part number: 
0 261 544 02E-01 
Nummer der Angebotszeichnung /  
Offer drawing number: 
0 261 A00 1H7 
Name des Kunden / Name of customer: 
After Market 
Kundenspezifikation / Customer specifica-
tion: 
 
Nummer / Number: 
 
Ausgabe/Version /  
Edition/version: 
 
Datum / Date: 
 
Titel / Title: 
 
Mitgeltende, weitere Dokumente / 
Further applicable documents: 
 
 
 
 
 
 
 
 
Das Produkt wurde ausschließlich für den Einsatz in diesen Anwendungen entwickelt, validiert und 
freigegeben: Wasserstoffsystem./ 
The product has been developed, validated, and released exclusively for use in these applications: 
Hydrogen system. 
 
Bei diesem Sensor handelt es sich um ein nicht kundenspezifisches Standardprodukt. Änderungen 
des Produktes durch RB sind im Laufe der Zeit möglich (ohne Kundeninformation), sofern diese 
Änderungen, die in dieser TKU beschriebenen Eigenschaften und/oder die Funktionsweise des 
Produktes im Übrigen nicht beeinflussen. 
This sensor is a non-customer-specific standard product. RB may implement changes to the product 
over the course of time which do not influence the product’s characteristics and/or functions as de-
scribed in this TCD (without customer information).

---

## Page 7 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
7/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
2 
ALLGEMEINE PRODUKTBESCHREIBUNG 
2.1 
WESENTLICHE FUNKTIONEN UND EIGENSCHAFTEN DES PRODUKTS 
Der Mitteldrucksensor MPS1 (im Folgenden Drucksensor oder Sensor genannt) wird zur Messung des 
mittleren Flüssigkeitsdrucks in H2-ICE-Systemen verwendet. Es wird in den Druckregler für die 
Wasserstoffeinspritzung eingesetzt. 
Der Drucksensor ist vorgesehen, um Drücke im spezifizierten Druckbereich relativ zum 
Umgebungsdruck zu messen. 
2.1.1 GRUNDAUFBAU DES SENSORS 
Der Grundaufbau des Drucksensors ist in Abbildung 1 dargestellt: 
Das Sensorelement mit der Messbrücke ist auf den Träger gelötet. Auf den Druckanschluss ist 
weiterhin der Sechskant geschweißt, mit dem der Sensor in das aufnehmende Bauteil eingeschraubt 
wird. Die Leiterplatte mit der Auswerteelektronik ist auf dem Sechskant verklebt. Die elektrische 
Verbindung zwischen Sensorelement und Leiterplatte wird mittels Bonddrahtverbindungen realisiert. 
Um die elektrische Anbindung an den Gerätestecker herzustellen, werden auf der Leiterplatte 
Kontaktfedern verbaut. Der Gerätestecker, der mit einem Metalleinsatz versehen ist, wird seinerseits 
auf den Sechskant geschweißt. 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
Abbildung 1: Aufbau des Sensors (schematische Darstellung) 
 
Gerätestecker 
Kontaktfeder 
Sechskant  
ASIC 
Leiterplatte 
Gewindestück 
Sensorelement  
Träger

---

## Page 8 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
8/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.1.2 TECHNISCHES PRINZIP 
Die Wandlung des angelegten Drucks in ein elektrisches Signal erfolgt durch Messung der 
Durchbiegung einer Siliziummembran. Auf dieser Siliziummembran ist eine Widerstandsbrücke mit 
piezoresistiven Widerständen aufgebracht. Die Brücke wird mit Spannung versorgt und bei 
Druckbeaufschlagung, d.h. bei Durchbiegung der Membran, verstimmt. Dadurch entsteht ein 
Spannungssignal, das zum Druck proportional ist (siehe Abbildung 2). 
 
 
Abbildung 2: Prinzip der Druckwandlung (Widerstandsbrücke auf Sensorelement) 
Das Brückensignal wird durch eine elektronische Auswerteschaltung (ASIC) weiterverarbeitet. Diese 
verstärkt, skaliert und digitalisiert das Spannungssignal der Messbrücke. Das Drucksignal wird dann 
als analoger Wert über den Gerätestecker nach außen gegeben.  
Um Störungen durch elektromagnetische Einflüsse zu vermeiden, sind diverse EMV-Maßnahmen (EMV 
= Elektromagnetische Verträglichkeit) enthalten. Über den Kabelbaumstecker (Gegenstecker) und den 
Kabelbaum (nicht mehr Teile des hier beschriebenen Sensors) wird der Wert zur weiteren Auswertung 
zum Steuergerät übermittelt (siehe Abbildung 3). 
 
Abbildung 3: Signalkette des Drucksensors von der Messbrücke zum Steuergerät 
 
 
2.2 BESTIMMUNGSGEMÄßER GEBRAUCH 
 
Das Produkt wird auf Basis der zum Zeitpunkt der TKU-Erstellung für das Produkt direkt geltenden 
regulatorischen Anforderungen in folgenden Zielmärkten freigegeben: IN

---

## Page 9 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
9/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
Das Produkt kann in einem freigegebenen Zielmarkt und im vorstehend festgelegten Einsatzbereich 
unter Beachtung der in dieser TKU beschrieben Grenzen, Bedingungen und sonstigen Vorgaben 
verwendet werden ("Bestimmungsgemäßer Gebrauch"). 
 
Für den Fall, dass der Kunde das Produkt außerhalb des bestimmungsgemäßen Gebrauch 
verwenden möchte, hat der Kunde  
(1) in eigener Verantwortung die daraus für das Produkt resultierenden etwaigen regulatorischen 
Anforderungen zu bewerten und einzuhalten sowie die Verwendungsfähigkeit des Produktes für den 
vom Kunden beabsichtigten Einsatzbereich zu bewerten und sicherzustellen oder  
(2) eine neue, erweiternde Bosch-Freigabe einzuholen, die bei Bosch gesondert zu beauftragen ist 
(z.B. mittels Änderungsschein). 
Lieferungen und Leistungen (Vertragserfüllung) stehen unter dem Vorbehalt, dass der Erfüllung 
keine Hindernisse aufgrund nationaler oder internationaler (Re-) Exportkontrollvorschriften, 
insbesondere Embargos oder sonstiger Sanktionen, entgegenstehen. 
 
Missbräuchliche Verwendung: 
 
Die Verwendung des Sensors in anderen als der in dieser TKU beschriebenen Anwendungen ist nicht 
freigegeben. Der Sensor darf nicht für die Messung von Drücken von anderen Medien wie z.B. CNG, 
Wasser, Kühlflüssigkeit, AdBlue, Lebensmittel oder Chemikalien verwendet werden. 
 
Die Verwendung in Luft-, Wasser- oder Weltraumfahrzeugen ist nicht freigegeben. 
 
Applikationsbedingungen: 
 
Schlauchmaterialien 
oder 
Kunststoffe, 
aus 
denen 
unter 
den 
Applikationsbedingungen 
Halogenverbindungen freigesetzt werden, können die Leiterbahnstrukturen auf dem Sensorelement 
schädigen und dürfen daher nicht eingesetzt werden. Besonders kritisch sind Einbaupositionen mit 
geringer bzw. ohne Durchlüftung bei gleichzeitig geringem Volumen und großen Oberflächen (z.B. 
Zuleitungsschläuche oder geschlossene Kunststoffformteile), an denen Halogene austreten (dies ist z.B. 
der Fall bei iodid- oder bromidstabilisierten Polyamiden und FKM, deren Rezeptur iodorganische Stoffe 
enthält). 
 
 
2.3 PRODUKTSICHERHEIT 
 
2.3.1 FUNKTIONALE SICHERHEIT 
Bosch weist darauf hin, dass das System/Produkt keine ASIL-klassifizierten Anforderungen (im 
Sinne der ISO 26262:2018) umsetzt. Es ist daher von Bosch nicht für Anwendungen freigegeben, 
bei denen eine oder mehrere Anforderungen mit einer ASIL-Klassifizierung (oberhalb QM) dem 
Bosch-Lieferumfang zugeordnet sind.

---

## Page 10 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
10/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.3.2 DATENSCHUTZ, CYBERSECURITY UND OVER-THE-AIR ASPEKTE 
 
Der Sensor speichert oder verarbeitet keine digitalen Daten. Der ASIC ist gegen Umprogrammierung 
geschützt. Der Sensor kommuniziert nur unidirektional in Richtung des Motorsteuergerätes und 
besitzt keine weitere Schnittstelle nach außen. 
 
 
2.3.3 SICHERHEITS- UND WARNHINWEISE 
 
Die Produktsicherheit ist nur dann gewährleistet, wenn die zulässigen Bedingungen eingehalten werden. 
 
2.3.3.1 ALLGEMEINE SICHERHEITSHINWEISE 
 
 
• 
Beachten Sie die geltenden Vorschriften zur Unfallverhütung und zum 
Umweltschutz. 
• 
Beachten Sie die Sicherheitsbestimmungen und Bestimmungen des Landes, in dem das 
Produkt verwendet / angewendet wird. 
• 
Das 
Personal, 
das 
an 
mechanischen 
und 
elektrischen 
Installations- 
und 
Deinstallationsverfahren beteiligt ist, muss für diese Aufgaben ausreichend qualifiziert und 
geschult sein („autorisiertes Personal“). 
• 
Personen, die das Produkt bedienen oder warten, dürfen nicht unter dem Einfluss von 
Alkohol, Drogen oder Medikamenten stehen, die die Reaktionsfähigkeit beeinflussen. 
• 
Verwenden Sie das Produkt ausschließlich in technisch einwandfreiem Zustand. 
• 
Beachten Sie die technischen Daten sowie die in der TKU angegebenen Betriebs- und 
Randbedingungen. 
• 
Beachten Sie mögliche Sicherheits- und Warnhinweise des übergeordneten Systems. 
• 
Nehmen Sie das Produkt erst in Betrieb, wenn Sie sicher sein können, dass das 
Endprodukt (Motor oder Fahrzeug), in dem das Produkt installiert ist, allen geltenden 
Bestimmungen, Sicherheitsbestimmungen und Normen der Anwendung entspricht.

---

## Page 11 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
11/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.3.3.2 PRODUKT-SPEZIFISCHE SICHERHEITSHINWEISE 
 
 
 
GEFAHR 
Leckage! 
Der Sensor ist in wasserstoffführenden Systemen verbaut, die permanent unter Druck 
stehen, auch im Ruhezustand des Fahrzeuges. Austretender Wasserstoff kann sich bei 
Kontakt mit einer Zünd-Quelle entzünden bzw. zur Explosion führen.   
Bei unsachgemäßer Lagerung, Transport oder Handhabung können Schäden an der  
Einschraub-Schnittstelle entstehen, die zu Wasserstoffleckage führen. 
• 
Der Sensor ist in der vorgesehenen Verpackung zu lagern und zu transportieren. 
• 
Ein beschädigter oder ein bei der Montage heruntergefallener Sensor muss 
aussortiert werden und darf nicht mehr verbaut werden.  
• 
Vor Demontage des Sensors muss die Wasserstoffzufuhr an den Sensoranbauort 
abgestellt werden. 
• 
Ein demontierter Sensor darf nicht wiederverwendet werden. 
Es 
sind 
geeignete 
Maßnahmen 
zur 
Absicherung 
gegen 
mögliche 
Folgen 
von 
Wasserstoffleckagen durch den Kunden in eigener Verantwortung zu prüfen und zu treffen, wie 
z. B. Leckage-Erkennungsfunktionen, eine entsprechende Reaktion auf höchster Warnstufe 
(Bediener-Interface) sowie Konzepte für vorbeugende und reaktive Wartung. 
 
WARNUNG 
Hoher Druck! 
Wenn während der Montage-, Wartungs- oder Demontagearbeiten hoher Druck herrscht, 
kann dies zu schweren Verletzungen, zum Tod oder zur Beschädigung der Ausrüstung führen. 
• 
Stellen Sie vor der Arbeit mit dem Sensor sicher, dass das System abgeschaltet ist 
so dass der Druck im System auf Umgebungsdruck reduziert ist. 
• 
Der Sensor sollte langsam ausgeschraubt werden, damit der Restdruck abgebaut 
werden kann 
 
VORSICHT 
Heiße Oberfläche! 
Die Oberfläche des Sensors kann heiß genug werden, um eine Verbrennungsgefahr 
darzustellen! 
• 
Lassen Sie das System auf Raumtemperatur abkühlen, bevor Sie den Sensor 
berühren z.B. während Wartungsarbeiten. 
• 
Verwenden Sie hitzeabweisende Handschuhe. 
HINWEIS 
Gefahr der Beschädigung! 
Bei unsachgemäßem Transport oder unsachgemäßer Handhabung können der elektrische 
Stecker und Dichtkonus beschädigt werden. 
• 
Der Sensor ist in der vorgesehenen Verpackung zu lagern und zu transportieren. 
• 
Ein beschädigter oder ein bei der Montage heruntergefallener Sensor muss 
aussortiert werden und darf nicht mehr verbaut werden. 
 
Gefahr auf Fehlfunktionen! 
Wenn der Sensor während der Installation, Wartung oder Demontage unter Spannung steht, 
kann dies zu Beschädigungen des Sensors führen. 
• 
Stellen Sie vor dem Ausbauen des Sensors sicher, dass keine elektrische Spannung 
am Gerätestecker des Sensors anliegt. 
Die Verschmutzung des elektrischen Steckers und des Gewindes kann zu fehlerhaften 
Funktionen im Fuel Cell System führen. 
• 
Die Steckerschutzkappe (sofern vorhanden) ist unmittelbar vor der Entnahme aus der 
Verpackung zu entfernen. 
• 
Es ist auf Sauberkeit der Umgebung während Montage und Demontage zu achten.

---

## Page 12 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
12/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.4 KENNZEICHNUNG DES PRODUKTS 
 
Die Bestellnummer, das Herstelldatum und eine laufende Seriennummer werden auf dem 
Gerätesteckergehäuse des Sensors angegeben (siehe Angebotszeichnung). 
 
 
2.5 MAßE UND GEWICHTE 
 
  
Abbildung 4: Typische Maße und Gewicht (Beispiel).  
2.6 LEISTUNGSAUFNAHME/-ABGABE 
Der Drucksensor hat eine typische Stromaufnahme von 12 mA bei einer Speisespannung von 5 V, also 
eine typische Verlustleistung von 60 mW. 
 
 
2.7 ALLGEMEINER HINWEIS ZU SERVICE, REPARATUR UND WARTUNG 
Das Produkt sieht keine Reparatur vor. 
Ein Wiedereinbau des demontierten Sensors ist nicht möglich. 
Der Austausch des Produkts ist ausschließlich von fachlich qualifiziertem Personal durchzuführen.  
Ferner sind die Hinweise in Abschnitt 2.3 zu beachten. 
 
 
2.8 HINWEISE ZUR ENTSORGUNG UND ZUM RECYCLING 
Der Sensor muss in umweltfreundlicher Art entsorgt oder dem Recycling zugeführt werden. Hierzu sind 
die gesetzlichen Anforderungen und die Vorgehensweisen entsprechend dem Stand der Technik 
einzuhalten. Der Sensor ist entsprechend der Vorgaben der Altautorichtlinie entwickelt, die 
Stoffzusammensetzung zu diesem Erzeugnis kann im dazugehörenden Materialdatenblatt (IMDS) 
eingesehen werden. 
 
3 
SYSTEMBESCHREIBUNG 
 
3.1  SYSTEM OF INTEREST (SOI)

---

## Page 13 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
13/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Der Sensor wird über eine Schraubverbindung im Bereich des Anodenmitteldruck eingeschraubt und 
tritt so in Kontakt mit dem Wasserstoff (Prinzip s. Abbildung 5, Details zur Verschraubung siehe 
Angebotszeichnung). 
Das Drucksignal wird als analoges Ausgangssignal zur Auswertung an ein elektronisches Steuergerät 
übertragen. 
Der korrekte Verbau des Sensors im Gesamtsystem muss durch den Kunden sichergestellt werden. 
Dies bedeutet insbesondere: 
• 
Das Design der Gegenschnittstelle muss entsprechend der Vorgaben der Angebotszeichnung 
sein. 
• 
Die Einbaulage des Sensors muss entsprechend der Vorgaben der Angebotszeichnung sein. 
• 
Die Beschaltung im Steuergerät und die Versorgung des Sensors (z.B. Versorgungsspannung, 
Maximalstrom) muss entsprechend der Vorgaben in dieser TKU sein.  
• 
Die 
Kabelentlastung 
des 
Kabelbaums 
muss 
entsprechend 
der 
Vorgaben 
der 
Angebotszeichnung sein. 
• 
Die Montage- und Handlungsweise müssen entsprechend der Vorgaben dieser TKU sein. 
 
HINWEIS 
Das System ist so auszulegen, dass in allen Betriebszuständen keine Drücke/Druckspitzen, z.B. durch 
Kavitation oder dynamische Druckschläge, entstehen, die den zulässigen Überdruck des Sensors 
überschreiten.  
Der Kunde muss entsprechende Prüfungen vornehmen, um zu ermitteln, ob es bei dem konkreten 
Einsatz des Sensors in seinem System zur Vermeidung von Kavitation den Verbau einer Drossel im 
Sensor bedarf. Nur durch eine entsprechende Systemauslegung und Prüfung kann vermieden 
werden, dass es zu einer durch Kavitation oder dynamische Druckschläge verursachten Leckage in 
die Umwelt kommt.  
 
Abbildung 5: Systembild Wasserstoffsystem 
 
HINWEIS 
Durch einen geeigneten Einbau im Fahrzeug (empfohlene Einbaulage siehe Angebotszeichnung) ist 
sicherzustellen, dass sich kein Wasser an der Membran sammeln kann. 
Auswirkung: Messbrückenverstimmung oder Membranbruch im Frostfall

---

## Page 14 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
14/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Bei der Montage des Sensors sind die Hinweise unter Abschnitt 4.8.2 zu beachten. 
Beim Handling des Sensors ist darauf zu achten, dass es zu keinen ESD-Schäden kommt. Bei der 
Auslegung des Einbauortes ist darauf zu achten, dass es nicht zu elektrischen Pulsen, elektrischen 
Feldern oder induktiven elektrischen Einkopplungen auf den Drucksensor kommt, die diesen 
beschädigen könnten. Siehe auch Hinweise unter Abschnitt 4.8.2.  
Die EMV-Festigkeit des Sensors ist entsprechend den in 6.1 angegebenen Prüfungen abgesichert. 
 
Emissionen mobiler Transceiver im Nahbereich können die vorgegebenen Kundenanforderungen 
bzgl. Störfestigkeit überschreiten. Bei Abständen kleiner oder gleich 200 mm zwischen der verbauten 
Kfz-Komponente und allen möglichen Positionen mobiler Transceiver – letztere bei PKW und NKW 
eingeschränkt auf Fahrgastzelle und Kofferraum – können Funktionsstörungen der Kfz-Komponente 
nicht ausgeschlossen werden. Beabsichtigt der Kunde den Sicherheitsabstand von 200 mm zu 
unterschreiten, liegt es in seiner Verantwortung, für diese Applikation zu prüfen, ob zusätzliche 
Maßnahmen erforderlich sind und diese dann umzusetzen. 
Eine galvanische Trennung zwischen der elektrischen Masse des Motors und des Steuergeräts ist zu 
vermeiden. 
Herstellprozesse, bei denen elektrostatische Felder verwendet werden, müssen auf Einhaltung der ESD-
Festigkeit überprüft werden. 
Elektrostatisches Lackieren des Sensors ist nicht zulässig. Es besteht die Gefahr von Schäden an den 
elektronischen Bauteilen des Sensors durch elektrostatische Entladung (ESD). 
Vor Elektroschweißarbeiten am Fahrzeug muss der Kabelbaumstecker (Gegenstecker) vom Sensor 
abgezogen sein. Schädliche Ströme in den elektronischen Bauteilen werden dadurch vermieden. 
 
 
3.2  PRODUKTSCHNITTSTELLE 
Zur Identifikation werden die Schnittstellen wie folgt benannt: 
 
 
 
 
Elektrische  
Schnittstelle 
Mechanische 
Schnittstelle

---

## Page 15 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
15/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Abbildung 6: Prinzipdarstellung mechanische/elektrische Schnittstelle 
3.2.1 
MECHANISCHE SCHNITTSTELLE 
Der Drucksensor wird mit dem Gewindestück in das aufnehmende Gegenstück eingeschraubt und 
über den Dichtkonus abgedichtet. 
Bei der Montage muss das maximal zulässige Anzugsmoment gemäß Abschnitt 4.2.4 berücksichtigt 
werden. Die Gestaltung der mechanischen Schnittstelle im Gegenstück muss gemäß der 
Angebotszeichnung (geometrische Gestaltung und Materialauswahl) erfolgen. Nur für diese 
Auslegung sind die in diesem Dokument angegebenen Randbedingungen abgesichert. 
Die Leckrate des Sensors (A-Muster) beträgt max. 5·10-5 mbar·l/s (Prüfgas: 100% Helium, 30bar 
Prüfdruck).  
Die Dichtheit der Schnittstelle des Sensors zum Gegenstück ist durch den Kunden zu verantworten. 
 
3.2.2 
STECKER / STECKVERBINDUNG ELEKTRISCH 
 
Verwendung eines Robert Bosch (RB) Steckverbindungssystem mit RB Gesamtverantwortung 
RB-Sachmängelhaftung für die Funktion der Steckverbindung nur bei Verwendung der in dieser TKU 
oder 
der 
dazugehörigen 
Steckverbinder-Angebotszeichnung 
vorgeschriebenen 
Steckverbindungseinzelteile sowie bei Anwendung innerhalb der in der Steckverbinder TKU 
beschriebenen Grenzen (elektrisch / mechanisch / thermisch). 
Es sind geeignete Kabel zu verwenden (temperaturfest, nicht geschirmt), und die Vorschrift zur 
Kabelführung in der Angebotszeichnung muss eingehalten werden. 
Die Oberflächen der Flachstecker am Sensor und an den Kontakten am Kabelbaumstecker müssen aus 
dem gleichen Werkstoff sein. Eine sogenannte Mischkontaktierung (z.B. goldbeschichtete Flachstecker 
mit silberbeschichteten Kontakten) am Kabelbaumstecker ist nicht zulässig. 
3.2.3 
STECKERBELEGUNG UND BESCHALTUNG DES SENSORS 
 
Die Zuordnung der elektrischen Beschaltung zu der Position der Kontaktpins im Gerätestecker 
(Steckerbelegung) ist in der Angebotszeichnung definiert. Der Drucksensor ist durch das 
Steuergerät gemäß Error! Reference source not found. zu beschalten und mit einer S
peisespannung von 5 V zu versorgen.

---

## Page 16 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
16/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
Abbildung 7: Beschaltung Drucksensor im Steuergerät 
Der Drucksensor liefert ein analoges Ausgangssignal, das sich ratiometrisch zur 
Versorgungsspannung US verhält. 
Der elektrische Ausgang des Sensors ist so gestaltet, dass Fehlfunktionen durch Kabelbrüche oder 
Kurzschlüsse mit der in Error! Reference source not found. dargestellten Beschaltung mit einem P
ull-up-Widerstand gegen Versorgungspannung erkannt werden können. 
Zudem ist im Signalpfad ein Tiefpassfilter (R1, C1) vorzusehen. Die EMV-Spezifikation des Sensors 
ist mit dieser Referenzbeschaltung abgesichert. 
Das 
Sensorgehäuse 
ist 
elektrisch 
leitfähig 
an 
die 
Fahrzeugmasse 
anzubinden. 
Der 
Übergangswiderstand darf maximal 2,5 mΩ betragen. 
 
 
 
 
 
 
 
 
 
 
4 
TECHNISCHE DATEN MIT MESSGRÖßEN UND MESSBEDINGUNGEN 
 
4.1 FUNKTIONEN UND FUNKTIONSZUSTÄNDE (BETRIEBSARTEN), FUNKTIONALE KENNWERTE UND 
RANDBEDINGUNGEN 
 
4.1.1 
ÜBERTRAGUNGSFUNKTION DRUCKSIGNAL 
Die Signalausgangsspannung wird (bis zum Nenndruck) aus dem tatsächlichen Druck wie folgt 
berechnet:

---

## Page 17 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
17/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
UOUT = (c1prel+c0)US 
mit  
UOUT =  Signalausgangsspannung [V] 
 
US 
=  Speisespannung [V] 
prel  
=  Relativdruck [kPa] 
c0  
=  0,1 
c1 
= 0,0001951 kPa-1 
 
  
 
Abbildung 8: Signalausgangsspannung UOUT als Funktion des Drucks 
 
 
 
 
 
 
 
ZEICHEN 
WERT 
EINHEIT 
Oberer Diagnosebereich für 
Signal Range Check (SRC) 
Usrch 
[0,96 … 1] * US 
V 
Oberer ungenutzter Bereich 
 
]0,94 … 0,96[ * US 
V 
Obere Begrenzung der 
Ausgangsspannung 
Uclh (C4) 
[0,9 … 0,94] * US 
V 
Messbereichsende 
Umre (C3) 
0,9 * US 
V 
Messbereichsanfang 
Umrb (C2) 
0,1 * US 
V

---

## Page 18 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
18/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Untere Begrenzung der 
Ausgangsspannung 
Ucll (C1) 
[0,06 … 0,1] * US 
V 
Unterer ungenutzter Bereich 
 
]0,04 … 0,06[ * US 
V 
Unterer Diagnosebereich für 
Signal Range Check (SRC) 
Usrcl 
[0 … 0,04] * US 
V 
Tabelle 1: Werte der Parameter für die analoge Ausgangskennlinie 
 
 
 
4.1.2 
FEHLERDIAGNOSE 
Für die Fehlerdiagnose sind außerhalb des Arbeitsbereichs liegende Diagnosebereiche vorgesehen 
(siehe Abschnitt 4.1.1). Da die Sensorcharakteristik des oberen Arbeitsbereiches begrenzt ist, können 
Überdruckbedingungen von Fehlern unterschieden werden.  
Die Kodierung der Reaktion auf einen Fehler in der nachfolgenden Tabelle 2 ist wie folgt: 
 
-1 
→ unteres Fehlerband und kein Reset 
-2 
→ unteres Fehlerband und Reset wird ausgelöst

---

## Page 19 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
19/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Error description 
DEBOUNCE CHARACTERISTICS 
(* RESET CAN NOT BE SET EARLIER THAN 400 
MS AFTER START-UP OR AFTER THE 
PRECEDING RESET) 
ANALOG ERROR  
MODE 0 
Initialization, P and T not yet available 
 
-1 
Indicates that OTP Bit for final programming at 
Bosch of OTP Master is not set 
(lock-bit not set) 
error is set immediately,  
no Reset is triggered 
-1 
1) Power-on 
complete RAM check (Read/Write) 
beginning of continuous ROM check 
 
2) Online 
continuous ROM check 
continuous RAM check 
RAM/ROM checks DSP by Parity during each 
Access 
HW-Check of SignalProcessor (Question/An-
swer) 
signature monitoring of program counter 
1) error is set immediately,  
Reset is triggered after 400 ms 
 
2) error is set * -2immediately, Reset is trig-
gered 
-2 
1) OTP CRC check of boot loading failed 4 times 
(consecutive) 
 
2) Sum-check on trim data. Test carried out dur-
ing boot loading and continuous cycle 
1) error is set immediately,  
Reset is triggered after 400 ms 
 
2) error is set * immediately,  
Reset is triggered 
 
-2 
Test on Acquisition Chain Pressure by injection 
of test signal before ADC on power-up 
 
Thresholds are defined during EoL programming 
at Bosch for each sensor individually 
error is set immediately,  
Reset is triggered after 400 ms 
-2 
Decimation interval error (only possible in case of 
severe hardware malfunction) 
No Reset, debouncing 
-1 
Pressure Sensor element failure (Wiring Detec-
tion) 
1) Power-On 
Common Mode at Power-On 
 
2) Online 
Common Mode 
Current Modulation 
 
without monitoring (4 bond wires) pressure could 
be in normal range 
1) error is set immediately,  
Reset is triggered after 400 ms 
 
2) Reset, debouncing  * 
-2 
Signal Input ADC too high, also for Sensor Ele-
ment Error 
no Reset, debouncing 
-1 
Signal Input ADC too low, also for Sensor Ele-
ment Error 
no Reset, debouncing 
-1 
Reference temperature input too high or low 
no Reset, debouncing 
-1 
Failure of internal temperature sensor --> HW 
Defects of ADC_T or PTAT itself 
no Reset, debouncing 
-1 
Low Voltage Supply detection, programmable 
threshold 
no Reset, debouncing 
-1

---

## Page 20 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
20/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
High Voltage Supply detection, programmable 
threshold 
no Reset, debouncing 
-1 
High ASIC Temperature programmable threshold 
no Reset, debouncing 
-1 
Pressure > Maximum Pressure for error-flag 
(pn*1.25)  or << offset pressure  
no Reset, debouncing 
-1 
Overflow or saturation in Vsup path 
no Reset, debouncing 
-1 
Overflow or saturation in signal path P and T 
no Reset, debouncing 
-1 
Tabelle 2: Reaktion des Sensors im Fehlerfall 
 
 
4.1.3 
VERHALTEN NACH  RESET UND INITIALISIERUNG 
 
Bei bestimmten Fehlern (siehe Tabelle 2) wird im Drucksensor ein Reset ausgelöst. Dieser Reset wird 
dann alle 400ms generiert. Nach einem Reset und während der nachfolgenden Initialisierung des 
Sensors, wird der Ausgang auf Masse gezogen.  
Falls der Fehler noch anliegt, bleibt das Ausgangssignal im Fehlerband.  
Im Falle, dass der Fehler nicht mehr anliegt, steuert das Ausgangssignal seinen Wert in den gültigen 
Arbeitsbereich. Der Verlauf des Ausgangssignals und die dazugehörigen typischen Zeiten bei 
Raumtemperatur nach dem Reset sind in Abbildung 9 und Tabelle 3 dargestellt..  
Abbildung 9: Darstellung der Zeiten nach Reset und Initialisierung 
 
 
PARAMETER 
T1 [ms] 
T2 [ms] 
T3 [ms] 
CRC OK 
0.03 
0.9 
1.1 
CRC NOK 
0.03 
2.1 
2.3 
Tabelle 3: Typische Zeiten nach Reset und Initialisierung

---

## Page 21 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
21/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
 
4.1.4 
VERHALTEN NACH UNTER- UND ÜBERSPANNUNG 
Bei einer Unter- oder Überspannungserkennung wird die Signalausgangspannung Uout auf 
Massepotential gezogen. 
Bei diesem Drucksensor sind folgende Grenzwerte appliziert. 
PARAMETER 
MIN [V] 
TYP [V] 
MAX [V] 
Unterspannungserkennung 
4.20 
4.35 
4.50 
Überspannungserkennung 
5.50 
5.65 
5.80 
Tabelle 4: Grenzwerte für Unter-/Überspannung 
 
 
 
 
4.1.5 
VERHALTEN BEI ÜBERTEMPERATURERKENNUNG 
 
Bei einer Übertemperaturerkennung wird die Signalausgangsspannung Uout auf Massepotential 
gezogen. 
 
Bei diesem Drucksensor sind folgende Grenzwerte appliziert. 
PARAMETER 
MIN [°C] 
TYP [°C] 
MAX [°C] 
Übertemperaturerkennung 
150 
162.5 
175 
Tabelle 5: Grenzwerte für Übertemperaturerkennungc 
 
 
 
 
4.1.6 VERHALTEN BEI ÜBERDRUCKERKENNUNG 
Bei einer Überdruckerkennung wird die Signalausgangspannung Uout auf Massepotential gezogen. 
 
Bei diesem Drucksensor sind folgende Grenzwerte appliziert. 
PARAMETER 
MIN  
TYP  
MAX  
Überdruckerkennung 
-5% 
1.25 x p2 (pn) 
+5% 
Tabelle 6: Grenzwerte für Überdruckerkennung 
 
 
 
 
4.2  MECHANISCHE KENNWERTE 
 
4.2.1 
DRÜCKE 
Der Drucksensor ist für einen Systemdruck gemäß Tabelle  ausgelegt, zuzüglich der Überschwinger, die 
im Drucklastkollektiv berücksichtigt sein müssen. Die Kennwerte und die maximale Belastbarkeit sind 
unten beschrieben.

---

## Page 22 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
22/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
GRÖßE 
ZEICHEN 
WERT 
EINHEIT 
Messbereich 
(Nenndruck) 
p1 (p0) 
p2 (pn) 
0 
4100 
kPa (rel.) 
Max. Überdruck 
pmax 
6600
kPa 
Berstdruck (statisch) 
pburst 
9800
kPa 
Tabelle 7: Kennwerte Drücke 
4.2.2 
TOLERANZEN DER DRUCKMESSUNG 
Die Toleranz der Druckmessung wird in %FSS angegeben. FSS (Full Scale Span) bezieht sich auf 
die nutzbare Spanne (4V). Zu den unten genannten Toleranzen ist zusätzlich ein Ratiometriefehler 
von ±0,15 %FSS zu addieren, der sich durch Abweichungen von der nominalen 
Versorgungsspannung US (5 V) ergibt. Die Toleranz ist von der Temperatur abhängig und ist gültig 
für 0-km und Feld. Die Toleranzen werden statistisch mit 3s pro Fertigungslos und bei Beschaltung 
gemäß Error! Reference source not found. eingehalten. 100% verlesene Erzeugnisse dürfen g
eliefert werden. 
Abbildung 10: Toleranz der Druckmessung 
Beispiel: @25°C => Toleranz = ±1,5 %FSS ≙ ±60 mV ≙ ±61.5 kPa 
4.2.3 
VIBRATIONSBELASTUNG

---

## Page 23 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
23/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Die hier angegebenen zulässigen Belastungen in Error! Not a valid bookmark self-reference. sind 
für den Drucksensor durch Erprobung abgesichert und gelten für den auf der Angebotszeichnung 
angegebenen Kabelbaumstecker sowie für Matrix1,2HV-Kontakte (beim Bosch-Trapez-Stecker) bzw. 
für Bosch-Dämpfungskontakte (BDK) (beim Bosch-Kompakt-Stecker). 
 
Die Werte gelten für die Anbaustelle des Sensors gemessen am Sechskant des Sensors 
 
 
 
 
Tabelle 8: Zulässige maximale Vibrationsbelastung  
 
Die Auslegung der Schwingbelastung basiert auf Bosch-Erfahrung und bezieht sich auf die in der ISO 
16750-3 angegebenen Drehzahlkollektive und den typischen Beschleunigungsverlauf über der 
Drehzahl. Sollte die maximale Beschleunigung bereits unterhalb der Nenndrehzahl auftreten, sind die 
TKU-Werte nicht anwendbar. 
Um die Einhaltung dieser Werte zu überprüfen, ist im Fahrzeug eine Vibrationsmessung durchzuführen. 
Bei Überschreiten oben genannter Werte ist zwingend eine Vibrationsmessung durch Bosch im 
Fahrzeug durchzuführen und freizugeben. 
 
4.2.4 
MAXIMAL ZULÄSSIGES ANZUGSMOMENT 
 
Maximal zulässige Einschraubbelastung: 27 Nm 
Details s. Angebotszeichnung. 
 
 
4.3 ELEKTRISCHE KENNWERTE 
In Tabelle  sind Grenzen wichtiger Sensorgrößen dargestellt. Diese werden statistisch mit 3s pro 
Fertigungslos eingehalten. 100 % verlesene Erzeugnisse dürfen geliefert werden. 
 
1 Ruhestrom bei 5 V Versorgung 
PARAMETER 
RB TRAPEZ (PC) 
Sinusschütteln: 
Amplitude/ 
Beschleunigung 
70-147Hz:            s=0,25mm 
147-1350Hz:        a=210m/s² 
1350-2000Hz: 
a=175m/s² 
PARAMETER 
ZEICHEN 
WERTE: MIN WERTE: MAX 
EINHEIT 
Speisespannung 
US 
4,75 
5,25 
V 
Ruhestromaufnahme bei US1 
IS 
 
15 
mA 
Einschaltstrom 
Irush 
 
60 
mA 
Ausgangsimpedanz für 
0,1US < UOut < 0,94US 
Rd 
 
10 
Ω 
Max. Speisespannung (max. 1 h) 
US,max 
 
18 
V

---

## Page 24 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
24/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
4.4 KLIMATISCHE KENNWERTE 
Der Sensor ist für den Einsatz im Motorraum (direkter Motoranbau) und die entsprechenden 
Umgebungstemperaturen ausgelegt mit einer Temperaturverteilung lt. Tabelle . 
 
TEMPERATUR 
VERTEILUNG 
-40°C  
6%  
23°C  
20%  
85°C  
65%  
135°C  
8%  
140°C  
1%  
Tabelle 10: Zulässige Temperaturverteilung 
 
 
2 Autarkiezeit ist die Zeit, die der Sensor ohne Versorgungsspannung am Eingang, ohne Reset Daten überträgt ausgehend 
von Us = 4.5 V statisch 
Kurzschluss Signalleitung zu GND bzw. zu 
Speisespannung (max. 8 h) bei 
gleichzeitiger Versorgung von US mit 
US,kurz 
US,short 
0 
18 
V 
Maximal zulässiger Strom bei Verpolung 
zweier Steckerkabel (max. 5 Min.@25 °C) 
Imax 
 
260 
mA 
Lastkapazität gegen Masse 
CL 
Maximale kapazitive Last am Ausgang s 
SENT-Norm (s. Anhang [1]; “Single Edge 
Nibble Transmission forAutomotive Applica-
tions”) 
Einschwingzeit bis Ruhestromaufnahme 
tIDD 
 
2,0 
ms 
Zeit bis zu ersten SENT Datenübertragung 
(senden der Null, vorausgesetzt CRC ist 
OK) 
tup_SENT 
1,8 
2,2 
ms 
Autarkiezeit bei Unterbrechung der 
Versorgungsspannung2 
tautarky 
10 
 
µs 
Eingangsstrom bei Fehlerfall US offen und 
VAusgang = 5 V. 
IUs_offen 
 
5 
µA 
Ausgangsstrom bei Fehlerfall GND offen 
und VAusgang = 0 V. 
IGND_offen 
 
350 
µA 
Tabelle 9: Grenzwerte der Signalübertragung, Versorgungsspannung und -ströme

---

## Page 25 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
25/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Es sind die in Tabelle  angegebenen Umgebungstemperaturen zulässig. 
 
PARAMETER 
WERTE 
EINHEIT 
RANDBEDINGUNGEN 
Minimale Temperatur  
-40 
°C 
 
Maximale Temperatur  
 120 
°C 
 
Tabelle 11: Zulässige Umgebungstemperaturen im Betrieb 
 
 
4.5 CHEMISCHE KENNWERTE 
Der Sensor ist für den Einsatz in H2-ICE-Systemen vorgesehen. 
Er ist für den Einsatz mit trockenem Wasserstoff abgesichert . 
 
 
4.6 
AKUSTISCHE KENNWERTE 
Nicht zutreffend. 
 
 
4.7 LEBENSDAUER 
 
Unter der in dieser TKU beschriebenen Verwendung und den angegebenen Einsatzbedingungen ist 
das Produkt auf eine Lebensdauer von 30.000 Betriebsstunden bzw. 15 Jahren ausgelegt (je 
nachdem, was zuerst eintritt)(tbd). 
 
 
4.8 TRANSPORT, MONTAGE, IN-/AUßERBETRIEBNAHME, LAGERUNG 
Bitte unbedingt die Sicherheits- und Warnhinweise in Abschnitt 2.3 beachten! 
 
4.8.1 
TRANSPORT 
Der Transport des Drucksensors hat in dafür vorgesehenen Verpackungen zu erfolgen. Falls der 
Drucksensor bereits montiert war, ist zu beachten, dass sich noch Restmengen Prüfflüssigkeit im 
Druckstutzen befinden können. Deswegen ist die Verpackung so auszulegen, dass beim Transport keine 
Prüfflüssigkeit in den Gerätesteckerbereich gelangen kann. 
Abweichend zu den Lagerbedingungen (s. Abschnitt 4.8.4) ist während des Transportes eine Erweiterung 
des Temperaturbereiches für max. 48h zulässig (s. Tabelle 12: TransportbedingungenTabelle ). 
 
DAUER 
TEMPERATUR 
RELATIVE LUFTFEUCHTE 
Max. 48 h 
-40 °C bis +80 °C 
0 % to 80 % 
Tabelle 12: Transportbedingungen 
 
4.8.2 
MONTAGE 
Die Angaben auf der Angebotszeichnung des Sensors sind grundsätzlich zu beachten.

---

## Page 26 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
26/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Der Drucksensor ist beim Einschrauben nur am Sechskant zu handhaben. Bei der Montage ist ein Steck- 
oder Ringschlüssel zu verwenden. Dieser darf nur am Sechskant angesetzt werden und muss diesen 
sicher überdecken. Ein Verkippen oder Verkanten des Steck-/Ringschlüssels mit dem Gerätestecker 
muss ausgeschlossen werden. Gabelschlüssel sind für die Montage nicht erlaubt. 
HINWEIS 
Schlagartige Impulse wie z.B. durch Hammerschläge oder Körnerpunkte in der Nähe des Sensors 
müssen vermieden werden; diese können zur Beschädigung des Sensorelements und zu 
Undichtheiten führen. 
Bei der Montage des Sensors darf der Gerätestecker nicht gegenüber dem Druckstutzen verdreht 
werden, der Sensor wird sonst zerstört. Der Gerätesteckerbereich darf nicht mit flüssigen oder festen 
Medien verschmutzt werden. 
HINWEIS 
Es ist sicherzustellen, dass der Dichtbereich zur Einbaustelle nicht beschädigt wird. Ein Eindringen  
von Fremdstoffen in oder an den Druckstutzen während der Montage muss ausgeschlossen werden, 
die Verbindung zwischen Sensor und Anbaustelle könnte sonst undicht werden. 
Empfehlungen bezüglich der Schmierstoffe sind in der Angebotszeichnung beschrieben. Details zur 
Verschraubung siehe Angebotszeichnung. 
Nachdem der Drucksensor in der beschriebenen Weise an seine Anbaustelle eingeschraubt wurde, 
wird er am Sechskant angezogen. Das zulässige Anzugsmoment ist in der Angebotszeichnung 
festgelegt. 
Eine Wiederverschraubung des Sensors ist nicht zulässig.  
HINWEIS 
Die Kontakte im Gerätestecker dürfen bei Montagearbeiten nicht berührt werden: Gefahr von ESD-
Schäden. Elektrische Messungen am Drucksensor dürfen nur mit abgefederten sauberen Teller- oder 
Kehlkontakten (frei von flüssigen und festen Medien) erfolgen. Bei der Montage darf kein „hot 
plugging“ (= An-/Abbringen des Kabelbaumsteckers bei anliegender Spannung) erfolgen: Gefahr von 
EOS-Schäden (EOS = electrical overstress). Bei elektrischer Prüfung des Sensors ist sowohl 
Verpolung als auch Überspannung auszuschließen. Die Prüfungen sind mit einer Spannungsquelle 
mit einer Strombegrenzung von 50 mA durchzuführen. 
Werden Tauchprüfungen durchgeführt, sind die elektrische Steckverbindung und der Druckanschluss 
zum Schutz vor Eindringen von Wasser zu verschließen. 
HINWEIS 
Bei Dichtheitsprüfungen sind Unterdrücke im Bereich des Gerätesteckers zu vermeiden. 
Heruntergefallene Sensoren dürfen nicht mehr weiterverwendet werden.

---

## Page 27 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
27/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
4.8.3 
IN-/ AUßERBETRIEBNAHME 
 
 WARNUNG 
Es ist sicherzustellen, dass vor dem Ausbau des Drucksensors der Motor abgeschaltet ist, so dass 
der Druck im System auf Umgebungsdruck reduziert und der Motor abgekühlt ist (siehe auch Abschnitt 
2.3). 
Es darf beim Ausbauen keine elektrische Spannung am Gerätestecker des Sensors anliegen. 
Der Kabelbaumstecker ist abzuziehen. Die mitgeltenden Unterlagen des in der Angebotszeichnung 
angegebenen Stecksystems sind hierbei zu beachten. 
Der Bereich um den Sensor herum ist mit einem geeigneten fettlösenden Reiniger zu reinigen und 
abtrocknen zu lassen. 
Der Reiniger darf nicht in die elektrische Steckverbindung eindringen! 
HINWEIS 
Mittels Steck- oder Ringschlüssels –der nur am Sechskant angesetzt werden darf– wird der Sensor 
gegen den Uhrzeigersinn gedreht bis er sich löst und kann dann ganz herausgeschraubt werden. 
 
 
4.8.4 
LAGERUNG 
Der Drucksensor ist unter folgenden Bedingungen zu lagern: 
TEMPERATURBEREICH 
RELATIVE 
FEUCHTIGKEIT 
HÖCHSTLAGERDAUER AB AUSLIEFERUNG VOM BOSCH-
HERSTELLUNGWERK: 
-30 °C bis +60 °C 
0 % bis 80 % 
5 Jahre 
Tabelle 13: Lagerbedingungen 
Der Drucksensor ist trocken und staubfrei zu lagern. Es ist darauf zu achten, dass es zu keiner 
Verschmutzung mit flüssigen oder festen Medien im Gerätesteckerbereich bzw. am Druckstutzen 
kommt. Bei versilberten Steckkontakten sind schwefelhaltige Atmosphären zu vermeiden, da es sonst 
zu Oxidationen der Silber-Oberflächen kommen kann.  
Diese Lagerbedingungen führen zu keiner Änderung der Eigenschaften und der Funktion des Druck-
sensors. Nach Überschreiten der Höchstlagerdauer müssen die Sensoren zur kostenpflichtigen 
Überprüfung zurück an Bosch geschickt werden. 
 
 
4.8.5 ELEKTRISCHE FEHLERDIAGNOSE 
Mit folgendem Verfahren kann die Funktionsfähigkeit des Sensors festgestellt werden: 
a. Fehlerspeicher im Steuergerät auslesen, ob der Sensor als defekt angezeigt wird. 
b. Kurzdiagnose: Messung der Offsetspannung wie in Abschnitt 4.8.6 beschrieben.

---

## Page 28 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
28/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Auswertung: Bei einem funktionstüchtigen Sensor ergibt sich die Ausgangsspannung wie in Abschnitt 
4.8.6 beschrieben. 
 
 
4.8.6 ELEKTRISCHE PRÜFUNG DER FUNKTION 
Prüfbedingungen 
Umgebungstemperatur 
23 ± 5 °C 
Speisespannung 
5,000 ± 0,250 V 
Akzeptanzkriterien 
Messwert Uout bei p0 
0.5 ± 0.060 V 
Messwert Uout bei pn 
4.5 ± 0.060 V 
Tabelle 7: Elektrische Prüfung 
 
Falls die Speisespannung US vom Nennwert 5,000 V abweicht, ist die gemessene Ausgangs-
spannung Uout mit dem Faktor 5,000 V/US auf den Nennwert umzurechnen. 
Ungenauigkeiten der Messgeräte sind gesondert zu berücksichtigen. 
Die Überprüfung des Sensors kann mit nachfolgendem Prüfaufbau geschehen: 
Hinweise zur Durchführung: 
• 
Wert des Pull-up-Widerstands siehe Abschnitt 3.2.3 
• 
Steckerbelegung siehe Angebotszeichnung 
• 
Spannungsversorgung mit Strombegrenzung maximal 50 mA 
• 
p0 und pn siehe Abschnitt Error! Reference source not found.

---

## Page 29 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
29/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
Abbildung 10: Prüfaufbau elektrische Funktion 
 
 
 
5 
SERIENBEGLEITENDE PRÜFUNGEN 
Die folgenden Erprobungen werden zur Überwachung und Absicherung der Produktqualität 
serienbegleitend durchgeführt. Eine Raffung der Erprobungsdauer durch Lasterhöhung ist zulässig. 
 
• 
Sinusschüttelprüfung (DIN EN 60068-2-6: 2008-10)  
• 
Funktionsdauerlauf 
• 
Hochtemperaturlagerung (DIN EN 60068-2-2) 
• 
Temperaturwechselprüfung (DIN EN 60068-2-14) 
• 
Wasserschutzprüfung IPX6K und IPX9K (DIN EN 60068-2-18) 
Stichprobenumfang und Häufigkeit werden von der Qualitätsabteilung des Werks festgelegt. Die 
Testanforderungen können von den in Abschnitt 6.1 genannten abweichen. 
 
 
6 
ERPROBUNG 
Die nachfolgend aufgeführten Erprobungen richten sich nach den mechanischen, elektrischen, 
klimatischen und chemischen Beanspruchungen im Motorraum bei Motoranbau des Drucksensors. Sie 
decken erfahrungsgemäß die zu erwartenden Beanspruchungen über der Fahrzeuglebensdauer ab. In 
Zweifelsfällen sind die tatsächlich auftretenden Belastungen gemeinsam mit Bosch zu bewerten. Die 
Schüttelbedingungen und Temperaturen an der Sensoranbaustelle sind in jedem Fall zu messen.

---

## Page 30 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
30/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Falls nicht anders angegeben, werden die Erprobungen einmalig im Rahmen der Freigabeerprobung 
durchgeführt. Für jede Prüfung werden neue Teile und wenn zutreffend neue Gegenstecker verwendet. 
Akzeptanzkriterium, falls nicht anders angegeben: Nach der Erprobung erfüllen die Prüflinge die 
Kennlinientoleranzen gemäß Abschnitt 4.2.2. 
 
 
6.1 BOSCH-SEITIGE ERPROBUNG 
 
 
6.1.1 
VIBRATIONSPRÜFUNG 
 
 
 
6.1.2 
FALLPRÜFUNG 
 
Testbeschreibung: 
Fallprüfung 
Ziel 
Beständigkeitsprüfung gegen Stürze aus 1m Höhe 
Testprofil 
o Geführtes Fallen aus Höhe 1m auf Beton  
- 1x Fallereignis pro Achsrichtung 
Dauer: 
- 
Norm  
ISO 16750-3 (2012-12) 
Kriterium: 
Der Sensor muss anschließend entweder funktionsfähig sein  
gemäß Abschnitt 4.2.2 oder sichtbar beschädigt sein 
 
 
6.1.3 
BERSTDRUCKPRÜFUNG 
  
 
Testbeschreibung: 
Berstdruckprüfung 
Ziel: 
Beständigkeitsprüfung gegen Drücke oberhalb des Messbereichs 
Testprofil: 
Druck entspricht Berstdruck definiert in Abschnitt 4.2.1 
Dauer: 
5 Min. 
Norm  
- 
Kriterium: 
Keine Leckage / Defekt des Sensors zulässig 
Testbeschreibung: 
Sinusschüttelprüfung 
 
Ziel: 
Die Schwingbelastung stellt die Belastung einer am Kraftstoffrail eines Fahrzeugs 
montierten Komponente ab. 
Testprofil: 
Amplitude der Auslenkung: 
s = 0,35 mm 
im Bereich 70 Hz bis 147 Hz 
Amplitude der Beschleunigung: 
a = 300 m/s2 
im Bereich 147 Hz bis 1350 Hz 
 
a = 250 m/s2 
im Bereich 1350 Hz bis 2000 Hz 
Frequenzänderung: 
1 Oktave/min 
Prüflinge werden mit vorgesehenem Kabelbaumstecker und Kabel auf einem  
elektrodynamischen Schwingtisch montiert.  
Angegebene Werte gelten für die Anbaustelle des Sensors auf der Aufspannvorrichtung 
Dauer: 
100h je Raumrichtung mit demselben Prüfling 
Kriterium: 
Sensor funktionsfähig gemäß Abschnitte 4.2.2 und 4.3.2 
Steckerverschleiß in Spezifikation nach den Kriterien von CoC Connector (nur für Bosch 
Steckersysteme 
 
Norm  
DIN EN ISO 60068-2-6 (2008-10)

---

## Page 31 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
31/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
Hinweis: Der Drucksensor ist nach der Prüfung nicht mehr funktionsfähig  
 
 
6.1.4 
MECHANISCH DYNAMISCHE PRÜFUNG: HOCHDRUCKPUSLVERSUCH 
 
Testbeschreibung: 
Mechanisch dynamische Prüfung: Hochdruckpulsversuch 
Ziel 
Die Pulsations- / Druckwechselprüfung dient der Erprobung eines 
Erzeugnisses bezüglich Funktion und Stoffeigenschaften 
(Schädigungsmechanismus Ermüdung)   
Testprofil: 
• 
Druckpuls-Profil: pmin bis pmax 
• 
pmin = 0…5 bar 
• 
pmax: s. Tabelle 5 
• 
Puls-Frequenz: 5…10 Hz 
• 
Puls-Form: Sinus oder Sinusähnlich 
• 
Temperaturprofil: -40°C…+150°C; 
   zeitliche Verteilung:  
   - ¼Tieftemperatur  
   - ¼Raumtemperatur 
   - ½ Hochtemperatur 
• 
Prüfmedium: Prüföl 
• 
Voralterung: 100 TSW von -40°C…+150°C 
• 
Sensor verbaut in Aufnahme, kein elektrischer Betrieb 
Dauer: 
107 Druckwechsel 
Norm  
- 
Kriterium: 
kein Bruch oder Undichtheit am Sensor zulässig  
Kennlinientoleranz gemäß Abschnitt 4.2.2 muss erfüllt sein 
 
6.1.5 
HOCHTEMPERATURLAGERUNG 
 
Testbeschreibung: 
Lagerung bei Hochtemperatur (drucklos) 
Ziel: 
Die Lagerung bei hoher Temperatur dient der Erprobung des  
Erzeugnisses bezüglich Funktion und Stoffeigenschaften 
(Schädigungsmechanismus Alterung, insbesondere bei 
Kunststoffen).   
Testprofil: 
Temperatur: 150°C 
Dauer: 
1000h  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-2 (2008-05) 
 
6.1.6 
TIEFTEMPERATURLAGERUNG 
 
Testbeschreibung: 
Lagerung bei Tieftemperatur 
Ziel: 
Die Lagerung bei tiefer Temperatur dient der Erprobung des  
Erzeugnisses bezüglich Funktion und Stoffeigenschaften.   
Testprofil: 
Temperatur: -40°C 
Dauer: 
200h  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-1 (2008-01) 
Abgesichert durch Tieftemperatur-Anteil von 6.1.7.

---

## Page 32 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
32/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
6.1.7 
TEMPERATURSCHOCKWECHSEL 
 
Testbeschreibung: 
Temperaturschockwechselprüfung 
Ziel: 
Die unterschiedlichen in Erzeugnissen verbauten Materialien 
erfahren durch den Temperaturwechsel eine Wechselbelastung. 
Unterschiedliche Materialien dehnen sich unterschiedlich aus und 
dies kann zum Auftreten von mechanischen Spannungen in den 
Bauteilen führen. Dieses kann zu thermomechanischer  
Materialermüdung und eventuell auch zum Bruch führen. 
Testprofil: 
o Temperaturschockzyklus: -40 °C/ +150°C; 
o Verharrungszeit bei Ecktemperaturen: 0,5h; 
o Prüflinge ohne elektrischen Betrieb; 
Dauer: 
1000 Zyklen  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-14, NA (2010-04) 
 
6.1.8 
WASSERSCHUTZPRÜFUNG 
 
Testbeschreibung: 
Wasserschutzprüfung 
Ziel: 
Beständigkeit gegen Wasserstrahl 
Testprofil: 
- 
Schutzklassen: IPX6K und IPX9K 
- Prüflinge mit vorgegebenem Gegenstecker,  
ohne elektrischen Betrieb; 
Dauer: 
Gemäß Norm  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Kein Wasser im Elektronikraum oder/und Stecker zulässig 
Norm  
ISO 20653 (2013-02) 
 
6.1.9 
FEUCHTEWECHSEL/ FEUCHTEWECHSELPRÜFUNG 
 
Testbeschreibung: 
Feuchtewechselprüfung 
Ziel: 
Beständigkeit gegen relative Feuchte. 
Es soll die Eignung der Prüflinge für Betrieb und Lagerung bei 
hohen relativen Luftfeuchtigkeiten und der Widerstandsfähigkeit 
gegen Einwirkung feuchter Luft und bei Betauung getestet 
werden. 
Testprofil: 
- 
Prüfung mit 93% rel. Luftfeuchtigkeit bei 65°C und bei 25°C 
inkl. genormter Übergänge 
- 
Voralterung: 100 TSW von -40°C…+150°C 
- 
Durchführung der Prüfung mit Gegenstecker (Vermeidung  
Betauung Steckerschnittstelle) 
Dauer: 
10 Zyklen à 24 h, davon 5 Zyklen mit Frost  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-38 (2010-06) 
 
6.1.10 CHEMISCHE BESTÄNDIGKEIT

---

## Page 33 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
33/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Testbeschreibung: 
Chemische Beständigkeit 
Ziel: 
Die Medienbeständigkeit nach ISO 16750-5 beschreibt die 
Anforderungen und Prüfungen durch die Widerstandsfähigkeit der 
Systeme / Komponenten gegen die chemischen 
Beanspruchungen im Fahrzeuglebenslauf. 
Testprofil: 
- 
Gemäß Norm 
- 
Flüssigkeiten: Dieselkraftstoff, Rapsmethylester, Motoröl, 
Kaltreiniger, Motorkühlflüssigkeit und Bremsflüssigkeit; 
Dauer: 
Gemäß Norm  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
ISO 16750-5 (2010-04) 
 
6.1.11 SALZNEBELPRÜFUNG 
 
Testbeschreibung: 
Salznebelprüfung 
Ziel: 
Korrosionsprüfung für Vergleichstest von Beschichtungen 
Testprofil: 
- Salzgehalt der Lösung muss (5 +/-1) % Masseanteile betragen  
- Temperatur in der Prüfkammer muss auf (35 +/-2) °C gehalten 
werden. 
Dauer: 
240h 
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm 
DIN EN 60068-2-11 
 
 
 
 
6.1.12 BCI-TEST 
 
Open Loop Methode nach ISO 11452-4; CBCI; Modulation CW und AM. 
Akzeptanzkriterium: Druckgenauigkeit von ±1,5 %FSS, jedoch angewendet auf Signale aus 
ungestörtem Mittelwert. 
 
 
Abbildung 12: Open Loop Stromprofil

---

## Page 34 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
34/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
6.1.13 ANTENNE 
 
Nach ISO 11452-2; Frequenzbereich 200MHz-3,2GHz; Modulation CW und AM. 
Akzeptanzkriterium: Kennlinie für Drucksignal innerhalb des Toleranzbandes gemäß Abschnitt 4.2.2,  
jedoch angewendet auf Signale aus ungestörtem Mittelwert. 
 
 
 
 
6.1.14 ESD-TESTS 
 
Akzeptanzkriterium für jeden einzelnen Test: Kennlinie für Drucksignal innerhalb des Toleranzbandes 
gemäß Abschnitt 4.2.2, jedoch angewendet auf Signale aus ungestörtem Mittelwert. 
 
ESD-Handling 
Nach ISO 10605 
Netzwerk: 
150 pF/ 330  
Kontaktentladung auf Pins/ Gehäuse-Metall:  8 kV 
Luftentladung auf Gehäuse-Kunststoff: 
 15 kV 
 
ESD-Operating Direct 
Nach ISO 10605 
Netzwerk: 
330 pF/ 330  
Kontaktentladung auf Gehäuse-Metall: 
 8 kV 
Luftentladung auf Gehäuse-Kunststoff: 
 15 kV 
 
ESD-Operating Indirect 
 
Frequency 
Level 
Modulation 
[MHz] 
[V/m] 
200 – 380 
160 
CW, AM 
380 – 460 
200 
460 – 520 
520 – 800 
160 
800 – 915 
CW, AM 
915 – 1 000 
1 000 – 1 200 
CW 
1 200 – 1 400 
200 
CW 
1 400 – 1 710 
160 
CW 
1 710 – 1 980 
1 980 – 2 700 
2 700 – 3 200 
200 
CW 
 
Abbildung 13: Antenne-Profil

---

## Page 35 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
35/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Nach ISO 10605 (HCP-Setup) 
Netzwerk: 
330 pF/ 330  
Kontaktentladung auf Entladeinseln: 
 15 kV 
 
 
6.1.15 TEST-PULSE AUF SENSORLEITUNGEN 
 
Nach ISO 7637-3 
Akzeptanzkriterium: Kennlinie für Drucksignal innerhalb des Toleranzbandes gemäß Abschnitt 4.2.2,  
jedoch angewendet auf Signale aus ungestörtem Mittelwert 
 
CCC Methode 
Puls, fast 3a: 
- 200 V 
Puls, fast 3b: 
+ 200 V 
Parameter tr: 
5 ns 
Parameter t1: 
100 µs 
Prüfdauer: 
10 min 
 
 
 
ICC Methode 
Puls, slow -   (2a negativ): 
- 6 V 
Puls, slow +   (2a positiv): 
+ 6 V 
Parameter tr: 
1 μs 
Parameter t1:  
0,5 s 
Prüfdauer: 
10 min 
 
6.2 KUNDENSEITIGE ERPROBUNG 
Der Nachweis der Verwendungseignung inklusive der korrekten System-Applikation des Produkts ist 
vom Kunden eigenverantwortlich durchzuführen und abzusichern. 
 
7 BEWERTUNG VON PRODUKTEN NACH VERWENDUNG BEI 0-KM IM FELD 
Produkte gelten als mangelfrei, wenn sie die in dieser TKU aufgeführten Kenndaten/Prüfdaten für 0-km 
und Feld erreichen.

---

## Page 36 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
36/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
INFORMATION ON SAFETY INSTRUCTIONS IN THIS DOCUMENT 
 
 This is the safety alert symbol. It is used to alert you to potential personal injury hazards. Obey all safety 
messages that follow this symbol to avoid possible injury or death. 
 
Risk classes in this document are according to ANSI Z535.6-2011:  
 
DANGER 
DANGER indicates a hazardous situation which, if not avoided, will result in death or serious in-
jury. 
 
WARNING 
WARNING indicates a hazardous situation which, if not avoided, could result in death or serious 
injury. 
 
CAUTION 
CAUTION indicates a hazardous situation which, if not avoided, could result in minor or moderate 
injury. 
NOTICE 
NOTICE is used to address practices not related to physical injury. 
 
The safety instructions in special section 2.3.3 SAFETY AND WARNING NOTES are structured as follows: 
 
 RISK CLASS 
Type and source of danger! 
Consequences in case of non-compliance 
• Hazard avoidance measures 
 
In other general sections the safety instructions are shown as follows: 
 
 RISK CLASS 
Safety text.

---

## Page 37 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
37/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2 
GENERAL PRODUCT DESCRIPTION 
 
2.1 
FUNCTIONS AND CHARACTERISTICS 
The medium pressure sensor MPS1 (hereinafter called pressure sensor or sensor) is used to measure 
fluid middle pressure in H2-ICE systems. It´s placed in Hydrogen injection Pressure regulator. 
The sensor measures fluid pressure in the specified pressure range to ambient pressure. 
2.1.1 BASIC DESIGN 
The basic design of the pressure sensor is shown in Figure 1. 
The sensor chip is soldered onto a carrier which is welded to the pressure port. The hexagon, which is 
used to screw the sensor into the receiving component, is welded to pressure port. It also serves to 
incorporate the printed circuit board with spacer as well as the contact/ground springs and the connector 
housing. The electrical connection between the sensor chip and printed circuit board is made by bond 
wires. Contact springs are installed on the printed circuit board to create the connection to the connector 
housing. The device connector, which is provided with a metal insert, is in turn welded onto the hexagon. 
 
 
 
 
Figure 1: Design of the sensor (schematic representation)

---

## Page 38 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
38/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
 
2.1.2 
TECHNICAL PRINCIPLE 
The conversion of the applied fluid pressure into an electrical signal is accomplished by measuring the 
deflection of a silicon membrane integrated in a MEMS-chip. A bridge circuit (Wheatstone bridge) 
consisting of piezo resistive resistors is located on the top side of the membrane; by applying fluid 
pressure to the sensor chip, i.e. by deflecting the membrane, the Wheatstone bridge put out of tune, 
which then results in a voltage signal output at the bridge circuit. This signal is proportional to the 
applied pressure. 
 
 
Figure 2: Principle of pressure conversion 
 
The bridge signal is processed further by an electronic evaluation circuit (ASIC). This amplifies, scales, 
and linearizes the voltage signal of the measurement bridge. The pressure signal is provided as an 
analog value through the device connector. 
In order to prevent disturbances due to electro-magnetic influences, various EMC-measures (EMC= Elec-
tromagnetic Compatibility) are provided. Via the wiring harness connector (mating connector) and the 
wiring harness (no more parts of the sensor described here), the value is transmitted to the control unit 
for further evaluation (s. Figure 3). 
 
 
Figure 3: Signal chain of the pressure sensor from measurement bridge to the electronic control unit

---

## Page 39 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
39/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.2 
INTENDED USE 
The product is released based on the regulatory requirements directly applicable to the product at 
the time of TCD creation in the following target market(s): IN 
 
The product can be used in a released target market in the aforementioned applications, subject to 
the limits, conditions and other specifications described in this TCD ("Intended use"). 
 
In the case that the customer wants to use the product outside the Intended Use, the customer shall  
(1) on its own responsibility evaluate and comply with any regulatory requirements resulting there-
from for the product and evaluate and ensure the usability of the product for the area of application 
intended by the customer or  
(2) obtain a new, extending Bosch release, which shall be ordered separately from Bosch (e.g. by 
means of a change request). 
Deliveries and services (fulfilment of contract) shall be subject to the provision that there are no ob-
stacles to performance due to national or international (re-) export control regulation, in particular 
embargoes or other sanctions. 
Misuse: 
The use of the sensor in other applications then described in this TCD is not released. The sensor is 
not allowed to be used for the purpose of monitoring fluid or gas pressure levels e.g. Water, Coolant, 
AdBlue, food, chemicals.  
The sensor is not released for use in air-, water- and space mobility. 
 
Application conditions: 
Hoses materials or plastics which could release halogen compounds under the application condtions 
can cause damage of the conductor paths on the sensor element and must therefore not be used. 
Especially critical are mounting positions with reduced or without ventilation in combination with low 
volume and high surface areas (e.g. connection hoses or closed plastic parts), which give off halogens 
(which is the case e.g. for iodide or bromide stabilized polyamides and FKM, which are based on a 
composition containing iodine-organic substances).  
 
 
2.3 
PRODUCT SAFETY 
 
2.3.1 
FUNCTIONAL SAFETY 
Bosch points out that the system/product does not implement any ASIL-classified requirements (in 
the sense of ISO 26262:2018). It is therefore not approved by Bosch for applications in which one or 
more requirements with an ASIL classification (above QM) are assigned to the Bosch scope of sup-
ply.

---

## Page 40 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
40/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.3.2 
DATA PROTECTION, CYBER SECURITY AND OVER-THE-AIR ASPECTS 
The sensor does not store or process any digital data. The ASIC is protected against reprogram-
ming. The sensor only communicates unidirectionally in the direction of the engine control unit and 
has no further external interface. 
2.3.3 
SAFETY AND WARNING NOTES 
The product safety is guaranteed only if the permissible conditions have been met. 
2.3.3.1 GENERAL SAFETY INSTRUCTIONS 
•
Observe the applicable regulations on accident prevention and environmental
protection.
•
Observe the safety regulations and provisions of the country in which the product is
used/applied.
•
Personnel involved in mechanical and electrical installation and de-installation pro-
cedures must be adequately skilled and trained for these tasks (‘authorized person-
nel’).
•
Persons assembling, operating, disassembling or maintaining the product must not
be under the influence of alcohol, other drugs or medication influencing the ability
to react.
•
Exclusively use the product in technically sound condition.
•
Comply with the technical data and operating and boundary conditions specified in
the TCD.
•
Observe potential safety and warning instructions from the superordinate system.
•
Do not commission the product until you can be sure that the end product (engine
or vehicle) where the product is installed complies with all applicable provisions,
safety regulations and standards of the application.
•
The rail must be used in accordance with the requirements mentioned in this TCD.

---

## Page 41 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
41/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.3.3.2 PRODUCT SPECIFIC SAFETY INSTRUCTIONS 
  DANGER 
Leakage! 
The sensor is obstructed in hydrogen-systems which stand constantly under 
pressure; even if the vehicle is switched off. Leaking hydrogen can ignite 
or cause an explosion if it comes into contact with an ignition source. 
If the sensor is stored, transported or handled improperly, damages to high 
pressure hydraulic interface could occur and lead to leakage, causing risk to injur
the health heavily or to cause fire. 
•
The sensor must be stored and transported in intended packaging.
•
The sensor which is damaged or falls down on the ground before or
during assembly has to be sorted out. Its further use for assembly is
prohibited.
•
Before disassembly the sensor, the hydrogen supply to the sensor in-
stallation site must be switched off.
•
A dismantled sensor must not be reused.
Therefore, appropriate measures to protect against possible consequences of 
hydrogen leakage have to be checked and taken by the customer in his own 
responsibility, e.g. leakage detection function, corresponding reaction on highest 
warning level (operator interface) as well as con-cepts on preventive and reac-
tive maintenance. 
WARNING 
High pressure! 
If a high pressure is present during installation, maintenance or disassembly 
work, it may lead to serious personal injury, death or damage to equipment. 
•
Before working with the sensor, make sure that the system is switched
off so that the pressure in the system is reduced to ambient pressure
•
The sensor should be unscrewed slowly so that the residual pressure
can be released
CAUTION 
Hot surface! 
Surface of the rail can reach high temperatures and may cause severe burns. 
•
Cool down the system to ambient temperature before touching it di-
rectly, e.g. during maintenance work.
•
Use heat protective gloves when working on the injection system
NOTICE 
Danger of damage! 
In case of improper transport or improper handling, the device connector and the 
sealing cone can be damaged. 
•
The sensor must be stored and transported in the packaging provided.
•
The sensor which is damaged or falls down on the ground during as-
sembly must be sorted out and must no longer be installed.
Danger of malfunction! 
If voltage is applied on the sensor during assembly or disassembly, this can lead 
to damage of the sensor: 
•
Before disassembling of the sensor no voltage must be applied on the
connector device of the sensor.
Contaminated connector device and thread part can lead to malfunction in the 
injection system: 
•
The protective cap (if any applied) must be removed just before instal-
lation of the sensor.
Appropriate cleanliness of the environment has to be ensured during assembly 
or disassembly of the rail.

---

## Page 42 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
42/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
2.4 
LABELING OF THE PRODUCT 
The order number, manufacturing date, and a serial number are indicated on the connector device 
housing of the sensor (see offer drawing). 
 
 
2.5 
DIMENSIONS AND WEIGHTS 
 
 
Figure 4: Typical Measures and Weight (Example).  
 
 
2.6 
POWER CONSUMPTION / POWER OUTPUT 
The pressure sensor has a typical power consumption of 12 mA at a supply voltage of 5 V, i.e. a 
typical power loss of 60 mW. 
 
 
2.7 
GENERAL REMARKS ON SERVICE, REPAIR, AND MAINTENANCE 
Repair of the product is not possible. 
Reassembling of disassembled sensor is not possible. 
Replacement of the product may only be performed by technically qualified personnel.  
Furthermore, the instructions in section 2.3 are to be observed. 
 
 
2.8 
INFORMATION ON DISPOSAL AND RECYCLING 
The sensor must be disposed or recycled in an environmental friendly manner. All relevant legal reg-
ulations, general state of the art and operational standards must be complied with. The product is in 
accordance with recycling regulations (ELV, End of Life Vehicles Directive). The material composi-
tion is available in the associated material data sheet (IMDS) to this product. 
 
 
3 
SYSTEM DESCRIPTION 
 
3.1 
SYSTEM OF INTEREST (SOI) 
The sensor is screwed via a screw connection and thus comes into contact with the hydrogen (s. 
Figure 5, for details on the screw connection see offer drawing).

---

## Page 43 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
43/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
The output signal is transmitted as an analogous output signal for evaluation to an electronic control 
unit. 
The correct installation of the sensor in the overall system must be ensured by the customer. This 
means in particular:  
•
The design of the mating interface must be in accordance with the specifications of the offer
drawing.
•
The installation position of the sensor must be in accordance with the specifications of the offer
drawing.
•
The wiring in the control unit and the supply to the sensor (e.g. supply voltage, maximum current)
must be in accordance with the specifications in this TCD/TKU.
•
The cable strain relief of the wiring harness must be in accordance with the specifications of the
offer drawing.
•
The installation and handling procedure must be according to the specifications of this TCD/TKU.
NOTICE 
The system must be designed hydraulically in a way that no pressure/pressure peaks, e.g. due to 
cavitation or dynamic pressure surges, occur, which exceed the permissible overpressure of the sen-
sor.  
The customer must carry out appropriate tests to determine whether it is necessary to install a throttle 
in the sensor during the specific use of the sensor in order to avoid cavitation. Only by appropriate 
system design and testing it can be avoided that there is a leakage into the environment caused by 
cavitation or dynamic pressure surges. 
Figure 5: Schematic representation hydrogen system 
NOTICE 
A suitable installation in the vehicle (recommended installation position, see offer drawing) should b
used to ensure that water cannot collect on the membrane. 
risk: detuning of measurement bridge or membrane rupture in case of frost 
For the installation process of the sensor, the instructions in section 4.8.2 are to be followed.

---

## Page 44 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
44/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
While handling the sensor, it must be ensured that there is no ESD damage. While designing the instal-
lation location, it must be ensured that there are no electrical pulses, electrical fields, or inductive electrical 
couplings on the pressure sensor that could damage it. See also instructions under section 4.8.2.  
The EMC resistance of the sensor is ensured according to the tests specified in the section 6.1. 
 
Emissions from mobile transceivers at close range can exceed the specified customer requirements 
with respect to interference immunity. In case of distances less than or equal to 200 mm between the 
installed automobile component and all possible positions of mobile transceivers –the latter limited to 
passenger compartment and trunk for passenger cars and commercial vehicles– malfunctions of the 
automobile component cannot be ruled out. If the customer intends to keep the safety distance below 
the 200 mm clearance specified above, it is his responsibility to check whether additional measures 
are required for this application and implement them. 
A galvanic separation between the electric ground terminal of the engine and the control unit must be 
avoided. 
Manufacturing processes using electrostatic fields have to be checked for the compliance with the ESD 
resistance.  
Electrostatic painting of the sensor is not permitted. There is a risk of damage to the electronic compo-
nents of the sensor due to electrostatic discharge (ESD). The wiring harness connector (mating con-
nector) must be pulled out from the sensor, before electric welding work is carried out on the vehicle. This 
prevents harmful currents in the electronic components. 
 
3.2 
PRODUCT INTERFACE 
For identification, the interfaces are named as follows:  
 
 
Figure 6: Interface identification 
 
3.2.1 
MECHANICAL INTERFACE 
Electrical  
Interface  
Mechanical 
Interface

---

## Page 45 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
45/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
The pressure sensor is screwed in with the pressure port into the receiving component and sealed by 
a sealing cone. 
The maximum tightening torque according to section 4.2.4 is to be considered during assembly. The 
design of the mechanical interface in the receiving component shall be made according to the offer 
drawing (geometrical design and material selection). Durability and sealing of the product in described 
conditions of this document are ensured only for this design. 
The internal tightness of the sensor is 5·10-5 mbar·l/s (Test Gas: 10% Helium, 90% Nitrogen). 
The customer is responsible for the tightness of the sensor’s interface with the counterpart. 
3.2.2 
PLUG CONNECTOR / ELECTRICAL PLUG CONNECTION 
Usage of a Robert Bosch (RB) plug connection system with RB overall responsibility 
RB-liability for defects in the function of the plug connection only if the plug connection system parts 
specified in this TCD or in the corresponding plug connection offer drawing are used and if the con-
nector is used within the limits (electrical / mechanical / thermal) described in the plug connection 
specification. 
Suitable cables (temperature resistant, not shielded) should be used, the instruction for the cable rout-
ing defined in the offer drawing must be complied with. 
The blade terminals of the sensor and on the contacts of the wiring harness connector have to be 
carried out of with similar surface finishes. A so-called mixed contacting (e.g. gold-plated blade termi-
nals with silver-coated contacts) on the wiring harness connector is forbidden. 
3.2.3 
CONNECTION DIAGRAM AND WIRING OF THE SENSOR 
The assignment of the electrical wiring to the position of the contact pins in the device connector (pin 
assignment) is defined in the offer drawing.  
The pressure sensor is to be wired through the ECU according to Error! Reference source not found. a
nd to be supplied with supply voltage of 5V.

---

## Page 46 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
46/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Figure 7: Pressure sensor wiring in the ECU 
The pressure sensor provides an analog output signal that has a ratiometric relationship with the sup-
ply voltage US. 
The electrical output of the sensor is designed such that malfunctions due to cable breaks or short-
circuits with the wiring illustrated in Error! Reference source not found. can be detected with a p
ull-up resistor against supply voltage. 
In addition, a low-pass filter (R1, C1) must be provided in the signal path. The EMC specification of 
the sensor is accomplished with the proposed circuitry. 
The sensor housing is to connect electrically conductive to the vehicle ground. The contact re-
sistance must not exceed 2.5 mΩ. 
4 
TECHNICAL DATA WITH MEASURED VARIABLES AND MEASURING CONDITIONS 
4.1 
FUNCTIONS, FUNCTION STATES (MODES OF OPERATION), FUNCTIONAL CHARACTERISTICS AND 
BOUNDARY CONDITIONS 
4.1.1 
TRANSFER FUNCTION OF PRESSURE SIGNAL 
The signal output voltage (up to the nominal pressure) is calculated from the actual pressure as ex-
plained below:  
Uout= (c1prel+c0)US 
with 
Uout 
= 
Signal output voltage [V] 
US 
= 
Supply voltage [V]

---

## Page 47 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
47/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
prel 
= 
Relative Pressure [kPa] 
 
c0 
= 
0.1 
 
c1 
= 
0.0001951 kPa-1 
 
 
 Figure 8: Signal output voltage UOUT as function of the pressure 
 
 
 
SIGN 
VALUE 
UNIT 
Upper diagnosis range for Signal 
Range Check (SRC) 
Usrch 
[0.96 … 1] * US 
V 
Upper unused range 
 
]0.94 … 0.96[ * US 
V 
Upper limitation of output voltage 
range 
Uclh (C4) 
[0.9 … 0.94] * US 
V 
Upper measurement range  
Umre (C3) 
0.9 * US 
V 
Lower measurement range 
Umrb (C2) 
0.1 * US 
V 
Lower limitation of output voltage 
range 
Ucll (C1) 
[0.06 … 0.1] * US 
V 
Lower unused range 
 
]0.04 … 0.06[ * US 
V 
Lower diagnosis range for Signal 
Range Check (SRC) 
Usrcl 
[0 … 0.04] * US 
V 
 
Table 1: Values of the parameters for the analog output characteristic curve

---

## Page 48 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
48/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.1.2 
ERROR DIAGNOSIS 
Diagnostic ranges lying outside the operating range are provided for the error diagnosis (see section 
4.1.1). Since the sensor characteristic of the upper operating range is limited, overpressure conditions 
can be distinguished from errors.  
The coding of the response to an error in the Table 2 is as follows: 
-1
→lower error band and no reset
-2
→lower error band and reset is triggered
ERROR DESCRIPTION 
DEBOUNCE CHARACTERISTICS 
(* RESET CAN NOT BE SET EARLIER THAN 400 
MS AFTER START-UP OR AFTER THE 
PRECEDING RESET) 
ANALOG ERROR 
MODE 0 
Initialization, P and T not yet available 
-1
Indicates that OTP Bit for final programming at 
Bosch of OTP Master is not set 
(lock-bit not set) 
error is set immediately, 
no Reset is triggered 
-1
1) Power-on
complete RAM check (Read/Write)
beginning of continuous ROM check
2) Online
continuous ROM check
continuous RAM check
RAM/ROM checks DSP by Parity during each
Access
HW-Check of SignalProcessor (Question/An-
swer)
signature monitoring of program counter
1) error is set immediately,
Reset is triggered after 400 ms
2) error is set * -2immediately, Reset is trig-
gered
-2
1) OTP CRC check of boot loading failed 4 times
(consecutive)
2) Sum-check on trim data. Test carried out dur-
ing boot loading and continuous cycle
1) error is set immediately,
Reset is triggered after 400 ms
2) error is set * immediately,
Reset is triggered
-2
Test on Acquisition Chain Pressure by injection 
of test signal before ADC on power-up 
Thresholds are defined during EoL programming 
at Bosch for each sensor individually 
error is set immediately, 
Reset is triggered after 400 ms 
-2
Decimation interval error (only possible in case of 
severe hardware malfunction) 
No Reset, debouncing 
-1

---

## Page 49 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
49/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Pressure Sensor element failure (Wiring Detec-
tion) 
1) Power-On
Common Mode at Power-On
2) Online
Common Mode
Current Modulation
without monitoring (4 bond wires) pressure could 
be in normal range 
1) error is set immediately,
Reset is triggered after 400 ms
2) Reset, debouncing  *
-2
Signal Input ADC too high, also for Sensor Ele-
ment Error 
no Reset, debouncing 
-1
Signal Input ADC too low, also for Sensor Ele-
ment Error 
no Reset, debouncing 
-1
Reference temperature input too high or low 
no Reset, debouncing 
-1
Failure of internal temperature sensor --> HW 
Defects of ADC_T or PTAT itself 
no Reset, debouncing 
-1
Low Voltage Supply detection, programmable 
threshold 
no Reset, debouncing 
-1
High Voltage Supply detection, programmable 
threshold 
no Reset, debouncing 
-1
High ASIC Temperature programmable threshold 
no Reset, debouncing 
-1
Pressure > Maximum Pressure for error-flag 
(pn*1.25)  or << offset pressure 
no Reset, debouncing 
-1
Overflow or saturation in Vsup path 
no Reset, debouncing 
-1
Overflow or saturation in signal path P and T 
no Reset, debouncing 
-1
Table 2: Response of the sensor in case of an error

---

## Page 50 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
50/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.1.3 
BEHAVIOR AT STARTUP AND RESET 
In case of specific errors (see Table 2), a reset is triggered in the pressure sensor. These are then gen-
erated every 400ms. After a reset, and during the subsequent sensor initialization, the output is connected 
to ground.  
If the error is still present, the output signal remains in the error band. 
In case, the error is no longer present, the output signal controls its value in the applicable operating 
range. The course of the output signal and the related typical times at room temperature after the reset 
are shown in Figure 9 and Table 3. 
PARAMETER 
T1 [ms] 
T2 [ms] 
T3 [ms] 
CRC OK 
0.03 
0.9 
1.1 
CRC NOK 
0.03 
2.1 
2.3 
Table 3: Typical times after reset and initialization 
4.1.4 
BEHAVIOR AT LOW / HIGH-VOLTAGE 
In case of undervoltage or overvoltage detection, the signal output voltage Uout is drawn to ground 
potential. 
For this pressure sensor, the following limit values are applied. 
Figure 9: Representation of the times after reset and initialization

---

## Page 51 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,  
   editing, distribution, as well as in the event of applications for industrial property rights. 
51/68
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
PARAMETER 
MIN [V] 
TYPE [V] 
MAX[V] 
Undervoltage detection 
4.20 
4.35 
4.50 
Overvoltage detection 
5.50 
5.65 
5.80 
Table 4: Limit values for undervoltage/overvoltage 
4.1.5 
BEHAVIOR AT OVERTEMPERATURE 
In the case of an overtemperature detection, the signal output voltage Uout is drawn to ground potential. 
The following limits are applied to this pressure sensor. 
PARAMETER 
MIN [°C] 
TYP [°C] 
MAX [°C] 
Over-temperature detection 
150 
162.5 
175 
Table 5: Limits for over-temperature detection 
4.1.6 
BEHAVIOR AFTER OVER PRESSURE 
 In the case of overpressure detection, the signal output voltage Uout is drawn to ground potential. 
The following limits are applied to this pressure sensor. 
PARAMETER 
MIN 
TYP 
MAX 
Overpressure Detection 
-5%
1.25 x p2 (pn) 
+5%
Table 6: Limits for overpressure detection 
4.2 
MECHANICAL CHARACTERISTICS 
4.2.1 
PRESSURE RANGE 
The pressure sensor is designed for a system pressure according to Table 7, plus the overshooting, 
which must be taken into account in the pressure load collective. The characteristic values and the max-
imum load capacity are described below. 
PARAMETER 
SYMBOL 
VALUE 
UNIT 
Measurement range 
(nominal pressure) 
p1 (p0) 
p2 (pn) 
0 
4100 
kPa (rel.) 
Maximum overpressure 
(max. 15min) 
pmax 
6600
kPa 
Burst pressure (static) 
pburst 
9800
kPa 
Table 7: Characteristic values regarding pressures

---

## Page 52 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
52/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.2.2 
TOLERANCES PRESSURE SIGNAL 
The tolerance of the pressure measurement is given in %FSS. FSS (Full Scale Span) refers to the  
usable span (4V). In addition to the tolerances mentioned below, a ratiometric error of ±0.15 %FSS shall 
be added, which results from deviations from the nominal supply voltage US (5 V). The tolerance depends 
on the temperature and is valid for 0-km and field. The tolerances are statistically adhered to with ±3s per 
production lot and with wiring according to Figure 7. 100% selected products may be delivered. 
 
 
Figure 10: Tolerance of pressure measurement 
Example: @25°C => Tolerance = ±1,5 %FSS ≙ ±60 mV ≙ ±61.5 kPa 
 
 
 
4.2.3 
VIBRATION LOADS 
The permissible loads specified in Table  for the pressure sensor are validated through testing and are 
applicable for the wiring harness connector specified in the offer drawing, as well as for matrix 1.2HV 
contacts (in the RB trapeze connector) or for Bosch damping contact (BDK) (in case of RB compact 
connector). 
The values apply at the mounting position of the sensor, measured at the hexagon of the sensor. 
PARAMETER 
 
Sinusoidal vibration test: 
Amplitude/ 
Acceleration 
70-147Hz:           s=0,25mm 
147-1350Hz:       a=210m/s² 
1350-2000Hz:  
a=175m/s² 
Table 8: Maximum permissible vibration loads

---

## Page 53 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
53/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
The design of the vibration load is based on Bosch experience and refers to the speed collective specified 
in the ISO 16750-3 and the typical acceleration profiles over the speed. If the maximum acceleration is 
already below the nominal speed, the TCD values are not applicable. 
To check the compliance of these values, a vibration measurement should be carried out in the vehicle. 
If the load exceeds the values specified above, a vibration measurement in the vehicle has to be 
performed and released by Bosch. 
 
 
 
4.2.4 
PERMISSIBLE TIGHTENING TORQUE 
 
Max. permissible tightening torque load: 27 Nm 
For details s. offer drawing.

---

## Page 54 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
54/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.3 
ELECTRICAL CHARACTERISTICS 
The limits of important sensor variables are shown in Table . These are complied statistically with 
3s per production batch. 100 % sorted products should be delivered. 
 
 
 
 
 
 
 
 
 
 
3 Quiescent current at 5V supply 
4 Increase from 10 % to 90 % statical output signal. Possible influences due to the wiring harness or the control unit must 
be examined separately. 
5 Autarchy time is the time for which the sensor transmits data without supply voltage at the input, without reset assuming 
Us = 4.5 V static. 
PARAMETER 
SIGN 
VALUE MIN 
VALUE MAX 
UNIT 
Supply voltage  
US 
4.75 
5.25 
V 
Quiescent current consumption at US3 
IS 
 
16.5 
mA 
Inrush Current 
Irush 
 
60 
mA 
Output impedance for 0.1US < UOut < 0.94US 
Rd 
 
10 
Ω 
Max. Supply voltage (max. 1 h) 
US,max 
 
18 
V 
Short-circuit signal line to GND or to supply 
voltage (max. 8h) in case of simultaneous 
supply of US with US,short 
US,short 
0 
18 
V 
Maximum permissible current in case of re-
verse polarity of two connector cables (max. 
5 Min.@25 C) 
Imax 
 
260 
mA 
Load capacitance to ground 
CL 
 
13 
nF 
Response time UOut when Us  is switch-on 
tin 
 
2.0 
ms 
Jump response time to pressure changes 
UOut  4 
t10/90 
0.2 
0.8 
ms 
Autarchy time in case supply voltage is inter-
rupted5 
tautarky 
10 
 
µs 
Input current in case of error US open and 
Voutput = 5 V. 
IUs_offen 
 
5 
µA 
Output current in case of error GND open 
and Voutput = 0 V. 
IGND_offen 
 
350 
µA 
Table 9: Limit values of the signal transmission, supply voltage and supply currents

---

## Page 55 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
55/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.4 
CLIMATIC CHARACTERISTICS 
The sensor is designed for the use in the engine compartment (direct engine mounting) and the cor-
responding ambient temperatures with a typical temperature distribution as per Table 10. 
 
TEMPERATURE 
DISTRIBUTION 
-40°C  
6%  
23°C  
20%  
85°C  
65%  
135°C  
8%  
140°C  
1%  
 
Table 10: Permissible temperature distribution 
 
 
The ambient temperatures specified in Table  are permissible. 
PARAMETER 
VALUE 
UNIT 
BOUNDARY CONDTIONS 
Min. Temperature  
-40 
°C 
 
Max. Temperature  
120 
°C 
 
Table 11: Permissible ambient temperatures during operation 
 
 
4.5 
CHEMICAL CHARACTERISTICS 
 
The sensor is intended for the use in H2- ICE system. It is rated for use with dry hydrogen. 
 
 
 
4.6 
ACOUSTIC CHARACTERISTICS 
Not applicable. 
 
 
4.7 
LIFETIME 
With respect to the use and usage conditions described in this TCD, the life of the product is designed 
for 30,000 operating hours, or 15 years (whichever occurs first)(tbd).  
 
 
4.8 
TRANSPORT, ASSEMBLY, START AND END OF OPERATION, STORAGE 
Please pay special attention to the safety and warning notes in section 2.3!

---

## Page 56 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
56/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
4.8.1 
TRANSPORT 
The pressure sensor has to be transported in packaging which is designed for it. If the pressure sensor 
was already mounted, it should be noted that there may be residual quantities of test fluid in the pres-
sure ports. Therefore, the packaging must be designed in such a way that no test fluid can get into the 
device connector area during transport. 
Deviating from the storage conditions (s. section 4.8.4) an extension of the temperature range for max. 
48h is allowed (s. table 12). 
 
PERIOD 
TEMPERATURE 
RELATIVE AIR HUMIDITY 
Max. 48h 
-40 °C to +80 °C 
0 % to 80 % 
Table 12: Transport Conditions 
 
 
4.8.2 
ASSEMBLY, START OF OPERATION 
 
The information given on the offer drawing of this sensor must always be observed. 
The pressure sensor may only be handled at the hexagon while screwing. While mounting, a socket or 
ring wrench has to be used. This wrench may only be put at the hexagon and must cover it safely. Tilting 
or tipping of the socket or ring wrench with the device connector must be avoided. Open-end wrenches 
are not permitted for the assembly. 
NOTICE 
Sudden impulses such as hammer blows or center punches in close proximity to the sensor are to be 
prevented; this could lead to damage at the sensor chip and as a result of that to leakage. 
While assembling the sensor, the device connector should not be rotated against the pressure port, else 
the sensor will be damaged. The device connector area must not be contaminated with liquid or solid 
media. 
NOTICE 
It must also be ensured that the sealing area is not damaged. Ingress of foreign matter in or on the 
pressure ports during assembly must be ruled out, otherwise the connection between the sensor and 
the mounting location could become leaky. 
Recommendations regarding lubricants are described in the offer drawing. See offer drawing for de-
tails on the assembly. 
After the pressure sensor has been screwed into its installation position in the described manner, it is 
tightened at the hexagon. The permissible tightening torque is specified in the offer drawing.  
 
Reconnection of the sensor is not permitted. 
NOTICE

---

## Page 57 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
57/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
The contacts in the device connector should not be touched during the assembly work: risk of ESD-
damages. Electrical measurements on the pressure sensor should be carried out only with spring-
mounted clean disc or collar contacts (free from liquid and solid media). During the assembly, "hot 
plugging" (= plugging-in plugging-out of the cable harness connector with voltage on) is not allowed: 
Risk of EOS damages (EOS = electrical overstress). During electrical testing of the sensor, reverse 
polarity and overvoltage should be ruled out. The tests should be carried out with a voltage source 
with a current limitation of 50 mA. 
If immersion tests are carried out, the electrical plug connection and the pressure connection shall be 
sealed to prevent the ingress of water. 
NOTICE 
Vacuum in the device connector area should be avoided during leak tests. 
Dropped sensors must not be used. 
 
4.8.3 
DISASSEMBLY AND DECOMMISSIONING 
 
 WARNING 
Before a pressure sensor is demounted the engine must be switched-off and sufficiently cooled down 
and the pressure in the system must be reduced (also refer section 2.3). 
While demounting, there must not be any electrical voltage present on the device connector of the 
sensor. 
The cable harness connector should be removed. The applicable documents of the plug system 
specified in the offer drawing must be observed. 
The area around the sensor should be cleaned with a suitable grease-dissolving cleaning agent and 
kept for drying. 
The cleaning agent must not penetrate into the electrical plug connection. 
NOTICE 
Using a socket wrench or ring wrench - that must be placed only on the hexagon - the sensor is turned 
counterclockwise until it becomes loose and can then be unscrewed completely. 
 
 
4.8.4 
STORAGE 
 
The pressure sensor should be stored under the following conditions: 
TEMPERATURE RANGE 
RELATIVE 
HUMIDITY 
MAXIMUM STORAGE DURATION FROM THE TIME 
THE PRODUCT IS DELIVERED FROM THE BOSCH 
MANUFACTURING PLANT: 
-30 °C to +60 °C 
0 % to 80 % 
5 years 
Table 7: Storage conditions

---

## Page 58 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
58/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
The pressure sensor should be stored under dry and dust-free conditions. It is important to ensure that 
there is no contamination due to liquid or solid media in the device connector area or on the pressure 
ports. Atmospheres containing sulfur have to be avoided for silver-plated connector pins, otherwise silver-
plated surfaces may become black.  
These storage conditions do not lead to any change in the properties and the functioning of the pressure 
sensor. After exceeding the maximum storage duration, the sensors must be sent back to Bosch for 
checking at the owner’s expense. 
 
4.8.5 
ELECTRICAL ERROR DIAGNOSIS 
The functionality of the sensor can be determined using the following method: 
a) Read out the fault memory in the control unit, whether the sensor is indicated as defective. 
b) Short diagnosis: measuring the offset voltage as described in section Error! Reference source n
ot found. 
c) Evaluation: For a functioning sensor, the output voltage is as described in section Error! Refer-
ence source not found. 
 
 
4.8.6 
ELECTRICAL TESTING OF THE FUNCTION 
 
Test Conditions 
Ambient Temperature 
23 ± 5 °C 
Supply Voltage 
5.000 ± 0.250 V 
Pass Criteria 
Measurement Uout at p0 
0.5 ± 0.060 V 
Measurement Uout at pn 
4.5 ± 0.060 V 
Table 8: Electrical Testing 
 
If the supply voltage US deviates from the nominal value 5.000 V, the measured output voltage UA 
should be converted to the nominal value using the factor 5.000 V/US [V]. 
Inaccuracies of the measuring instruments are to be considered separately 
The sensor can be checked with the following test setup (pin assignment see offer drawing): 
Notes on the implementation: 
• 
Value for the Pull-up resistor according to section 3.2.3 
• 
For pin assignment see offer drawing 
• 
Power supply with current limitation max. 50 mA

---

## Page 59 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
59/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
• 
p0 und pn see section 4.2.1 
 
 
Figure 10: Test setup - electrical function 
 
 
 
 
5 
SERIES-ACCOMPANYING TESTS 
The following tests are carried out during series production for monitoring and ensuring the product qual-
ity. Condensing the test duration by increase in load is permissible. 
• 
Sinusoidal vibration test (DIN EN 60068-2-6: 2008-10) 
• 
Functional Endurance Testing 
• 
High temperature storage  (DIN EN 60068-2-2) 
• 
Temperature change test (DIN EN 60068-2-14) 
• 
Water protection test IPX6K and IPX9K (DIN EN 60068-2-18) 
Sample size and frequency are determined by the quality department of the plant. The test requirements 
can deviate from the requirements mentioned in section 6.1. 
 
 
6 
TESTING 
The tests listed below are based on the mechanical, electrical, climatic and chemical stresses in the 
engine compartment when mounting the pressure sensor to the engine. Experience has shown that they 
cover the expected stresses over the lifetime of the vehicle. In case of doubt, the actually occurring loads

---

## Page 60 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
60/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
are to be evaluated together with Bosch. The vibration conditions and temperatures at the sensor instal-
lation point are to be measured in each case. 
Unless otherwise specified, the tests are conducted once within the framework of release testing. New 
parts and, if needed, new mating connectors are used for each test. 
Acceptance criterion, unless otherwise stated: After the testing, the test specimens meet the character-
istic curve tolerances according to section 4.2.2. 
 
6.1 
TESTING BY BOSCH 
 
6.1.1 
VIBRATION TESTING 
 
 
 
 
6.1.2 
DROP TEST 
 
Test destignation: 
Drop test 
Goal: 
Resistance test against drop from a height of one meter 
Test profile: 
- Controlled drop from 1 m height on concrete  
- 1x drop event each axial direction 
Duration: 
- 
Standard: 
ISO 16750-3 (2012-12) 
Criteria: 
The component must then either be functional in terms of signal out-
put voltage according to section 4.2.2 or visibly damaged 
 
6.1.3 
BURST PRESSURE TEST 
 
Test description: 
Bursting pressure test 
Goal: 
Resistance test of pressures above the measuring range 
Test profile: 
Applied pressure: burst pressure according to section 4.2.1 
Duration: 
5 min. 
Standard: 
- 
Criteria: 
No break/leakage in the pressure ports allowed . 
Test description: 
Sinusoidal vibration test 
 
Goal: 
The vibration load represents the load on a component mounted on the fuel rail of a  
vehicle. 
Testprofile: 
Amplitude of deflection: 
s = 0,35 mm 
in the range 70 Hz to 147 Hz 
Amplitude of acceleration: 
a = 300 m/s2 
in the range 147 Hz to 1350 Hz 
 
a = 250 m/s2 
in the range 1350 Hz to 2000 Hz 
Frequency change: 
1 Octave/min 
The test specimens are mounted on the electro-dynamic vibration table using the dedicated 
wiring harness connector and cable 
values are applicable for the sensor installation point on the clamping fixture. 
Duration: 
100h  in each spatial direction with the same test specimen 
Criteria: 
Sensor functional according to section 4.2.2 
Connector wear in specification according to the criteria of CoC connector (only for Bosch 
connector systems). 
Standard:  
DIN EN ISO 60068-2-6 (2008-10)

---

## Page 61 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
61/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
6.1.4 
MECHANICAL DYNAMIC TEST: HIGH PRESSURE PULSATION 
 
Test description: 
Mechanical dynamic test: High Pressure Pulsation 
Goal: 
The pulsation test and pressure change test serves to test a  
product with regard to its function and material properties 
(Damage mechanism with regard to fatigue) 
Test profile: 
• Pressure Pulse Profile: pmin to pmax 
• pmin = 0…5 bar 
• pmax: s. Table 5 
• Puls-Frequency: 5…10 Hz 
• Puls Shape: sine or sine-like 
• Temperature Profile: -40°C…+150°C; 
   Temporal distribution:  
   - ¼ low temperature 
   - ¼ room temperature  
   - ½ high temperature 
• Test Medium: Test Oil 
• Pre-Aging: 100 temperature shock cycles from 40°C…+150°C 
• Sensor installed in fixture, no electrical operation 
Duration: 
107 pressure cycles 
Norm  
- 
Criteria: 
No breakage or leakage at the sensor permitted 
Tolerance of characteristics curve according to section 4.2.2 must 
be fulfilled 
 
 
6.1.5 
HIGH TEMPERATURE STORAGE 
 
Test description: 
Storage at high temperature 
Goal: 
The storage at high temperature is used to test the product in 
terms of function and material properties (damage mechanism  
aging, especially with plastics) 
Test profile: 
Temperature: 150°C 
Duration: 
1000 h  
Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
DIN EN 60068-2-2 (2008-05) 
 
6.1.6 
LOW TEMPERATURE STORAGE 
 
Test description: 
Storage at low temperature 
Goal: 
The storage at low temperature is used to test the product in 
terms of function and material properties. 
Test profile: 
Temperature: -40°C 
Duration: 
200 h  
Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
DIN EN 60068-2-1 (2008-01)

---

## Page 62 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
62/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
6.1.7 
TEMPERATURE SHOCK CYCLE TEST 
 
Test description: 
Temperature shock cycle test 
Goal: 
The different materials used in the products experience an alter-
nating load due to the temperature changes. 
Different materials expand differently and this can lead to the oc-
currence of mechanical stresses in the components. This may 
lead to thermomechanical material fatigue and possibly also lead 
to breakage. 
Test profile: 
- 
thermal shock cylces : -40 °C/ +150°C; 
- 
remain at cutoff temperatures for 0,5 h; 
- 
test specimens without electrical operation; 
Duration: 
1000 cycles  
Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
DIN EN 60068-2-14, NA (2010-04) 
 
6.1.8 
WATER PROTECTION TEST 
 
Test description: 
Water protection test 
Goal: 
Resistance to water jets 
Test profile: 
- 
protection classes: IPX6K und IPX9K 
- 
test specimens with the dedicated mating connector 
- 
without electrical operation 
Duration: 
See standard 
Criteria: 
Sensor functional according to section 4.2.2 
No water allowed in the electronics compartment and / or plug. 
Standard: 
ISO 20653 (2013-02) 
 
6.1.9 
HUMIDITY CHANGE TEST 
 
 
Test description: 
Humidity change test: resistance against relative humidity 
Goal: 
Resistance to relative humidity. 
The suitability of the test items for operation and storage at high rel-
ative humidity and the resistance to the effects of moist air and con-
densation should be tested. 
Test profile: 
- Testing with 93% relative humidity at 65°C and 25°C incl.  
standardized transitions 
- Preaging: 100 temperature shock cycles starting -40°C…+150°C 
- Test performed with mating connector (Avoidance of condensation 
on plug interface) 
Duration: 
10 cycles á 24 h each, of which 5 cycles with frost  
Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
DIN EN 60068-2-38 (2010-06) 
 
6.1.10 CHEMICAL DURABILITY 
 
Test description: 
Chemicale resistance test

---

## Page 63 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
63/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Goal: 
The media resistance according to ISO 16750-5 describes the re-
quirements and tests due to the resistance of the systems /  
components against the chemical stresses in the vehicle life cycle. 
Test profile: 
- 
Conform standard; 
- 
Resistance to diesel fuel, rapeseed methyl ester, engine oil, 
cold cleaner, engine coolant, and brake fluid. 
Duration: 
Conform standard 
Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
ISO 16750-5 (2010-04) 
 
6.1.11 SALT SPRAY TEST 
 
Test description: 
Salt spray test 
Goal: 
Corrosion test for comparative testing of coatings 
Test profile: 
- The salt content of the solution must be 5 +/- 1% parts of 
masses 
- The temperature in the test chamber must be kept at 35 +/- 2 ° 
C. 
Duration: 
240h 
Criteria: 
Sensor functional according to section 4.2.2 
Norm 
DIN EN 60068-2-11 
 
6.1.12 BCI-TEST 
 
Open Loop method according to ISO 11452-4; CBCI; Modulation CW and AM. 
Pass Criterion: Pressure Accuracy of ±1.5 %FSS but tolerance band applied to signals based on un-
disturbed mean value. 
 
Figure 11: Open Loop current profile

---

## Page 64 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
64/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
 
6.1.13 RADIATED INTERFERENCE (ANTENNA) 
According to ISO 11452-2; frequency range 200MHz-3.2GHz; Modulation CW and AM. 
Pass Criterion: Characteristic of pressure signal within tolerance band according to section 4.2.2 but 
tolerance band applied to signals based on undisturbed mean value. 
 
 
 
 
 
6.1.14 ESD-TEST 
 
Pass criterion for each individual test: Characteristic of pressure signal within tolerance band accord-
ing to section 4.2.2 but tolerance band applied to signals based on undisturbed mean value. 
 
ESD-Handling 
According to ISO 10605 
Network: 
150 pF/ 330  
Contact discharge to pins/ housing-metal: 
 8 kV 
Air discharge to housing-plastic: 
 15 kV 
 
ESD-Operating Direct 
According to ISO 10605 
Network: 
330 pF/ 330  
Contact discharge to housing-metal: 
 8 kV 
 
Frequency 
Level 
Modulation 
[MHz] 
[V/m] 
200 – 380 
160 
CW, AM 
380 – 460 
200 
460 – 520 
520 – 800 
160 
800 – 915 
CW, AM 
915 – 1 000 
1 000 – 1 200 
CW 
1 200 – 1 400 
200 
CW 
1 400 – 1 710 
160 
CW 
1 710 – 1 980 
1 980 – 2 700 
2 700 – 3 200 
200 
CW 
Figure 12: Antenna profile

---

## Page 65 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
65/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
Air discharge to housing-plastic: 
 15 kV 
 
ESD-Operating Indirect 
According to ISO 10605 (HCP-Setup) 
Network: 
330 pF/ 330  
Contact discharge to discharge islands: 
 15 kV 
 
6.1.15 TEST PULSE ON SENSOR LINES 
According to ISO 7637-3 
Pass criterion: Characteristic of pressure signal within tolerance band according to section 4.2.2 but 
tolerance band applied to signals based on undisturbed mean value. 
 
CCC Method 
Pulse, fast 3a: 
- 200 V 
Pulse, fast 3b: 
+ 200 V 
Parameter tr: 
5 ns 
Parameter t1: 
100 µs 
Test Duration: 
10 min 
 
 
 
ICC Method 
Pulse, slow -   (Negative 2a): 
- 6 V 
Pulse, slow +   (Positive 2a): 
+ 6 V 
Parameter tr: 
1 μs 
Parameter t1:  
0.5 s 
Test Duration: 
10 min 
 
 
6.2 
TESTING BY CUSTOMER 
 
Customer is responsible to verify and to prove that the product is fit for its intended purpose and is 
responsible for the proper calibration of the product.

---

## Page 66 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
66/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
 
7 
ASSESSMENT OF PRODUCTS RETURNED FROM 0-MILEAGE AND FROM THE FIELD 
Products are considered good if they fulfill the specifications/test data for 0-mileage / field listed in the 
TCD.

---

## Page 67 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
67/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
8 
APPENDICES AND REFERENCES 
8.1 
REFERENCES STANDARDS 
 
DIN 50100: 1978-02 
DIN EN 60068-2-11: 2000-02 
DIN EN 60068-2-18: 2001-10 
DIN EN 60068-2-2: 2008-05 
DIN EN 60068-2-6: 2008-10 
ISO 11452-2: 2004-11  
ISO 11452-4: 2003-01 
IEC 60068-2-6: 2007-12 
ISO 10605: 2001-12 
ISO 16750-3: 2007-08-01 
ISO 20653: 2006-08-15 
ISO 26262: 2011-11-30 
DIN 51625 (2008) 
DIN EN 60068-2-14 
DIN EN 60068-2-1 (2008-01). 
DIN EN 60068-2-14 (2000-08). 
DIN EN 60068-2-38 
DIN EN 60068-2-42 (2004-04) 
ISO 16750-5 
ISO 7637-3

---

## Page 68 (Method: embedded_text)

© Robert Bosch GmbH 2022 | All rights reserved, also regarding any disposal, exploitation, reproduction,   
   editing, distribution, as well as in the event of applications for industrial property rights. 
68/68
 
PS-MPS1 | TCD RB Number: 0 261 K00 13H| Release / Version: 01 | Date: 2022-09-12 
 
9 
HISTORIE/ HISTORY 
 
 
 
 
 
10 KONTAKTPERSON / CONTACT PERSON 
 
 
Published by:                                    MS/END52, Rumesh Kumar Krishnan 
Kontaktperson / Contact Person: 
PS-SU/ECI4, Frank Buehner 
  
Ausgabe/ 
Release 
Datum/ 
Date 
Bearbeiter / 
Edited by 
Änderungsbeschreibung/  
Release Description 
01 
13.09.2022 
Rumesh 
First Edition based

---
