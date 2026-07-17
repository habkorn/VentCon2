# Extracted Content: Attachment_2_TCD_0261K001BD_MPS_FC_A_33_bar.pdf

**Source file:** `Attachment_2_TCD_0261K001BD_MPS_FC_A_33_bar.pdf`
**Extraction method:** PyMuPDF (fitz) + pytesseract OCR fallback
**Total pages:** 68

---

## Page 1 (Method: embedded_text)

Produktbezeichnung / Product Designation: 
Medium Pressure Sensor (MPS)  
Version / Version: 
0 
Datum / Date: 
01.12.2023  
 
Diese vorläufige TKU dokumentiert den gegenwärtigen 
Stand der abgestimmten Spezifikationen. Sie wird bestätigt, 
wenn die Produktvalidierung mit positivem Ergebnis abge-
schlossen wurde.  
This preliminary TCD documents the present status of the 
agreed specifications. It will be confirmed when all valida-
tion has been completed with positive results. 
 
Diese TKU ist in zwei Sprachen verfasst. Im Zweifelsfall fin-
det die deutsche Version Anwendung. 
 
TKU Technische Kundenunterlage 
TCD Technical Customer Documentation

---

## Page 2 (Method: embedded_text)

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
DATENSCHUTZ, CYBERSECURITY UND OVER-THE-AIR ASPEKTE .....................................................10 
2.3.3 
SICHERHEITS- UND WARNHINWEISE ................................................................................................10 
2.4 KENNZEICHNUNG DES PRODUKTS 
11 
2.5 MAßE UND GEWICHTE 
11 
2.6 LEISTUNGSAUFNAHME/-ABGABE 
12 
2.7 ALLGEMEINER HINWEIS ZU SERVICE, REPARATUR UND WARTUNG 
12 
2.8 HINWEISE ZUR ENTSORGUNG UND ZUM RECYCLING 
12 
3 
SYSTEMBESCHREIBUNG ................................................................................................................................. 12 
3.1 SYSTEM OF INTEREST (SOI) 
12 
3.2 PRODUKTSCHNITTSTELLE 
14 
3.2.1 
MECHANISCHE SCHNITTSTELLE ......................................................................................................14 
3.2.2 
STECKER / STECKVERBINDUNG ELEKTRISCH ....................................................................................15 
3.2.3 
STECKERBELEGUNG UND BESCHALTUNG DES SENSORS .................................................................15 
4 
TECHNISCHE DATEN MIT MESSGRÖßEN UND MESSBEDINGUNGEN ................................................................. 16 
4.1 FUNKTIONEN UND FUNKTIONSZUSTÄNDE (BETRIEBSARTEN), FUNKTIONALE KENNWERTE UND 
RANDBEDINGUNGEN 
16 
4.1.1 
ÜBERTRAGUNGSFUNKTION DRUCKSIGNAL ......................................................................................16 
4.1.2 
FEHLERDIAGNOSE ...........................................................................................................................17 
4.1.3 
VERHALTEN NACH RESET UND INITIALISIERUNG ..............................................................................19 
Bei bestimmten Fehlern (siehe Tabelle 2) wird im Drucksensor ein Reset ausgelöst. Dieser Reset wird 
dann alle 400ms generiert. Nach einem Reset und während der nachfolgenden Initialisierung des Sensors, 
wird der Ausgang auf Masse gezogen. .........................................................................................................19 
Falls der Fehler noch anliegt, bleibt das Ausgangssignal im Fehlerband. ....................................................19 
4.1.4 
VERHALTEN NACH UNTER- UND ÜBERSPANNUNG ...........................................................................20 
4.1.5 
VERHALTEN BEI ÜBERTEMPERATURERKENNUNG ............................................................................20 
4.1.6 
VERHALTEN BEI ÜBERDRUCKERKENNUNG ......................................................................................20 
4.2 MECHANISCHE KENNWERTE 
20 
4.2.1 
DRÜCKE ..........................................................................................................................................20 
4.2.2 
TOLERANZEN DER DRUCKMESSUNG ................................................................................................21 
4.2.3 
VIBRATIONSBELASTUNG ..................................................................................................................21 
4.2.4 
MAXIMAL ZULÄSSIGES ANZUGSMOMENT ..........................................................................................22 
4.3 ELEKTRISCHE KENNWERTE 
22 
4.4 KLIMATISCHE KENNWERTE 
23 
4.5 CHEMISCHE KENNWERTE 
23 
4.6 AKUSTISCHE KENNWERTE 
23 
4.7 LEBENSDAUER 
23 
4.8 TRANSPORT, MONTAGE, IN-/AUßERBETRIEBNAHME, LAGERUNG 
24 
4.8.1 
TRANSPORT ....................................................................................................................................24 
4.8.2 
MONTAGE .......................................................................................................................................24 
4.8.3 
IN-/ AUßERBETRIEBNAHME ..............................................................................................................25 
4.8.4 
LAGERUNG ......................................................................................................................................26 
4.8.5 
ELEKTRISCHE FEHLERDIAGNOSE .....................................................................................................26 
4.8.6 
ELEKTRISCHE PRÜFUNG DER FUNKTION .........................................................................................26 
5 
SERIENBEGLEITENDE PRÜFUNGEN ................................................................................................................. 27 
6 
ERPROBUNG ................................................................................................................................................... 28 
6.1 BOSCH-SEITIGE ERPROBUNG 
28

---

## Page 3 (Method: embedded_text)

6.1.1 
VIBRATIONSPRÜFUNG .....................................................................................................................28 
6.1.2 
FALLPRÜFUNG .................................................................................................................................28 
6.1.3 
BERSTDRUCKPRÜFUNG ...................................................................................................................29 
6.1.4 
HYDROSTATISCHE FESTIGKEIT ........................................................................................................29 
6.1.5 
DICHTHEITSPRÜFUNG .....................................................................................................................29 
6.1.6 
WIDERSTAND GEGEN DREHMOMENTÜBERSCHREITUNG ..................................................................29 
6.1.7 
MECHANISCH DYNAMISCHE PRÜFUNG: HOCHDRUCKPUSLVERSUCH ................................................30 
6.1.8 
FUNKTIONSDAUERLAUF ...................................................................................................................30 
6.1.9 
HOCHTEMPERATURLAGERUNG ........................................................................................................30 
6.1.10 
TIEFTEMPERATURLAGERUNG ..........................................................................................................31 
6.1.11 
TEMPERATURSCHOCKWECHSEL ......................................................................................................31 
6.1.12 
WASSERSCHUTZPRÜFUNG ..............................................................................................................31 
6.1.13 
FEUCHTEWECHSEL/ FEUCHTEWECHSELPRÜFUNG ...........................................................................31 
6.1.14 
CHEMISCHE BESTÄNDIGKEIT ...........................................................................................................32 
6.1.15 
BESTÄNDIGKEIT GEGEN WEITERE MEDIEN .......................................................................................32 
6.1.16 
ZYKLISCHE KORROSIONSPRÜFUNG (CCT) ......................................................................................32 
6.1.17 
ISOLATIONSWIDERSTAND ................................................................................................................33 
6.1.18 
BCI-TEST .......................................................................................................................................33 
6.1.19 
ANTENNE ........................................................................................................................................33 
6.1.20 
ESD-TESTS ....................................................................................................................................34 
6.1.21 
TEST-PULSE AUF SENSORLEITUNGEN .............................................................................................35 
6.2 KUNDENSEITIGE ERPROBUNG 
35 
7 
BEWERTUNG VON PRODUKTEN NACH VERWENDUNG BEI 0-KM IM FELD ......................................................... 35 
2 
GENERAL PRODUCT DESCRIPTION .................................................................................................................. 37 
2.1 MAIN FUNCTIONS AND CHARACTERISTICS OF THE PRODUCT 
37 
2.1.1 
BASIC DESIGN .................................................................................................................................37 
2.1.2 
TECHNICAL PRINCIPLE .....................................................................................................................37 
2.2 INTENDED USE 
38 
2.3 PRODUCT SAFETY 
39 
2.3.1 
FUNCTIONAL SAFETY ......................................................................................................................39 
2.3.2 
DATA PROTECTION, CYBER SECURITY AND OVER-THE-AIR ASPECTS ................................................39 
2.3.3 
SAFETY AND WARNING NOTES .........................................................................................................40 
2.4 LABELING OF THE PRODUCT 
41 
2.5 DIMENSIONS AND WEIGHTS 
41 
2.6 POWER CONSUMPTION / POWER OUTPUT 
41 
2.7 GENERAL REMARKS ON SERVICE, REPAIR, AND MAINTENANCE 
42 
2.8 INFORMATION ON DISPOSAL AND RECYCLING 
42 
3 
SYSTEM DESCRIPTION .................................................................................................................................... 42 
3.1 SYSTEM OF INTEREST (SOI) 
42 
3.2 PRODUCT INTERFACE 
44 
3.2.1 
MECHANICAL INTERFACE .................................................................................................................44 
3.2.2 
PLUG CONNECTOR / ELECTRICAL PLUG CONNECTION.......................................................................44 
3.2.3 
CONNECTION DIAGRAM AND WIRING OF THE SENSOR ......................................................................45 
4 
TECHNICAL DATA WITH MEASURED VARIABLES AND MEASURING CONDITIONS ............................................... 45 
4.1 FUNCTIONS, FUNCTION STATES (MODES OF OPERATION), FUNCTIONAL CHARACTERISTICS AND BOUNDARY 
CONDITIONS 
45 
4.1.1 
TRANSFER FUNCTION OF PRESSURE SIGNAL ...................................................................................45 
4.1.2 
ERROR DIAGNOSIS..........................................................................................................................47 
4.1.3 
BEHAVIOR AT STARTUP AND RESET .................................................................................................48 
4.1.4 
BEHAVIOR AT LOW / HIGH-VOLTAGE ................................................................................................49 
4.1.5 
BEHAVIOR AT OVERTEMPERATURE ..................................................................................................49 
4.1.6 
BEHAVIOR AFTER OVER PRESSURE .................................................................................................50 
4.2 MECHANICAL CHARACTERISTICS 
50 
4.2.1 
PRESSURE RANGE ..........................................................................................................................50 
4.2.2 
TOLERANCES PRESSURE SIGNAL ....................................................................................................50 
4.2.3 
VIBRATION LOADS ...........................................................................................................................51 
4.2.4 
PERMISSIBLE TIGHTENING TORQUE ................................................................................................51 
4.3 ELECTRICAL CHARACTERISTICS 
52

---

## Page 4 (Method: embedded_text)

4.4 CLIMATIC CHARACTERISTICS 
53 
4.5 CHEMICAL CHARACTERISTICS 
53 
4.6 ACOUSTIC CHARACTERISTICS 
53 
4.7 LIFETIME 
53 
4.8 TRANSPORT, ASSEMBLY, START AND END OF OPERATION, STORAGE 
54 
4.8.1 
TRANSPORT ....................................................................................................................................54 
4.8.2 
ASSEMBLY ......................................................................................................................................54 
4.8.3 
START AND END OF OPERATION .......................................................................................................55 
4.8.4 
STORAGE ........................................................................................................................................55 
4.8.5 
ELECTRICAL ERROR DIAGNOSIS ......................................................................................................56 
4.8.6 
ELECTRICAL TESTING OF THE FUNCTION .........................................................................................56 
5 
SERIES-ACCOMPANYING TESTS ...................................................................................................................... 57 
6 
TESTING ......................................................................................................................................................... 57 
6.1 TESTING BY BOSCH 
58 
6.1.1 
VIBRATION TESTING ........................................................................................................................58 
6.1.2 
DROP TEST .....................................................................................................................................58 
6.1.3 
BURST PRESSURE TEST ..................................................................................................................58 
6.1.4 
HYDROSTATIC STRENGTH ...............................................................................................................58 
6.1.5 
LEAKAGE TESTING ..........................................................................................................................59 
6.1.6 
EXCESS TORQUE RESISTANCE .......................................................................................................59 
6.1.7 
MECHANICAL DYNAMIC TEST: HIGH PRESSURE PULSATION ..............................................................59 
6.1.8 
CONTINUED OPERATION TEST ........................................................................................................60 
6.1.9 
HIGH TEMPERATURE STORAGE ........................................................................................................60 
6.1.10 
LOW TEMPERATURE STORAGE ........................................................................................................60 
6.1.11 
TEMPERATURE SHOCK CYCLE TEST .................................................................................................61 
6.1.12 
WATER PROTECTION TEST ...............................................................................................................61 
6.1.13 
HUMIDITY CHANGE TEST .................................................................................................................61 
6.1.14 
CHEMICAL DURABILITY ....................................................................................................................62 
6.1.15 
RESISTANCE TO OTHER MEDIA ........................................................................................................62 
6.1.16 
CYCLIC CORROSION TESTING (CCT) ..............................................................................................62 
6.1.17 
INSULATION RESISTANCE ................................................................................................................63 
6.1.18 
BCI-TEST .......................................................................................................................................63 
6.1.19 
RADIATED INTERFERENCE (ANTENNA) ............................................................................................63 
6.1.20 
ESD-TEST ......................................................................................................................................64 
6.1.21 
TEST PULSE ON SENSOR LINES .......................................................................................................65 
6.2 TESTING BY THE CUSTOMER 
65 
7 
ASSESSMENT OF PRODUCTS RETURNED FROM 0-MILEAGE AND FROM THE FIELD ........................................... 65 
8 
APPENDICES AND REFERENCES ..................................................................................................................... 66 
8.1 REFERENCES STANDARDS 
66 
9 
HISTORIE/ HISTORY .................................................................................................................................. 67 
2 
KONTAKTPERSON / CONTACT PERSON .......................................................................................................... 67 
 
 
 
Weitere Unterlagen / Furter Documents:

---

## Page 5 (Method: embedded_text)

INFORMATIONEN ZU SICHERHEITSHINWEISEN IN DIESEM DOKUMENT 
 
 
 Dies ist das Sicherheitswarnsymbol. Es wird verwendet, um Sie auf mögliche Gefahren für Personenschä-
den aufmerksam zu machen. Befolgen Sie alle Sicherheitshinweise, die diesem Symbol folgen, um mögliche 
Verletzungen oder Todesfälle zu vermeiden. 
 
Die Risikoklassen in diesem Dokument:  
 
 
GEFAHR 
Die GEFAHR deutet auf eine gefährliche Situation hin, die zum Tod oder zu schweren Verletzun-
gen führt, wenn sie nicht vermieden wird. 
 
WARNUNG 
WARNUNG deutet auf eine gefährliche Situation hin, die zum Tod oder zu schweren Verletzungen 
führen könnte, wenn sie nicht vermieden wird. 
 
VORSICHT 
VORSICHT deutet auf eine gefährliche Situation hin, die zu geringfügigen oder moderaten  
Verletzungen führen könnte, wenn sie nicht vermieden wird. 
HINWEIS 
HINWEIS wird verwendet, um Praktiken zu adressieren, die nicht im Zusammenhang mit einer 
körperlichen Schädigung stehen. 
 
 
Die Sicherheitshinweise in Abschnitt 2.3.3 sind wie folgt aufgebaut: 
 
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

1 
PRODUKTIDENTIFIKATION/ PRODUCT IDENTIFICATION 
  
 
Produktbezeichnung / Product designation: 
Medium Pressure Sensor 
Typbezeichnung / Type designation: 
PS-MPS1-FC 
(Entwicklungs-)Teile-Nummer /  
(Development) Part number: 
0 261 544 03M -01 
RB TCD Number 
0 261 K00 1BD 
Nummer der Angebotszeichnung /  
Offer drawing number: 
0 261 A00 1WS 
Name des Kunden / Name of customer: 
Handel 
Kundenspezifikation / Customer specifica-
tion: 
N.A 
Nummer / Number: 
N.A 
Ausgabe/Version /  
Edition/version: 
N.A 
Datum / Date: 
N.A 
Titel / Title: 
N.A 
Anwendung / Application 
Pressure measurement in rail and pressure regulator of H2ICE 
Mitgeltende, weitere Dokumente / 
Further applicable documents: 
N.A 
 
Das Produkt wurde ausschließlich für den Einsatz in diesen Anwendungen entwickelt, validiert und 
freigegeben: H2ICE Wasserstoffsystem im Rail oder Druckregulierungsventil 
The product has been developed, validated, and released exclusively for use in these applications: 
H2ICE Hydrogen system mounted in Rail or Pressure regulator valve. 
Bei diesem Sensor handelt es sich um ein nicht kundenspezifisches Standardprodukt. Änderungen 
des Produktes durch RB sind im Laufe der Zeit möglich (ohne Kundeninformation), sofern diese Än-
derungen, die in dieser TKU beschriebenen Eigenschaften und/oder die Funktionsweise des Produk-
tes im Übrigen nicht beeinflussen. 
This sensor is a non-customer-specific standard product. RB may implement changes to the product 
over the course of time which do not influence the product’s characteristics and/or functions as de-
scribed in this TCD (without customer information).

---

## Page 7 (Method: embedded_text)

2 
ALLGEMEINE PRODUKTBESCHREIBUNG 
2.1 
WESENTLICHE FUNKTIONEN UND EIGENSCHAFTEN DES PRODUKTS 
Der Mitteldrucksensor MPS1 (im Folgenden Drucksensor oder Sensor genannt) dient zur Messung 
des Drucks im Rail und Druckregelventil von H2-ICE-Systemen. 
Der Drucksensor ist vorgesehen, um Drücke im spezifizierten Druckbereich relativ zum Umgebungs-
druck zu messen. 
2.1.1 GRUNDAUFBAU DES SENSORS 
Der Grundaufbau des Drucksensors ist in Abbildung 1 dargestellt: 
Das Sensorelement mit der Messbrücke ist auf den Träger gelötet. Auf den Druckanschluss ist wei-
terhin der Sechskant geschweißt, mit dem der Sensor in das aufnehmende Bauteil eingeschraubt 
wird. Die Leiterplatte mit der Auswerteelektronik ist auf dem Sechskant verklebt. Die elektrische Ver-
bindung zwischen Sensorelement und Leiterplatte wird mittels Bonddrahtverbindungen realisiert. Um 
die elektrische Anbindung an den Gerätestecker herzustellen, werden auf der Leiterplatte Kontaktfe-
dern verbaut. Der Gerätestecker, der mit einem Metalleinsatz versehen ist, wird seinerseits auf den 
Sechskant geschweißt. 
Über ein Entlüftungsloch im Steckerboden erfolgt der Druckausgleich zwischen Elektronikraum und 
Umgebung. Bei Relativdrucksensoren entspricht der Druck im Elektronikraum dem Referenzdruck. 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
Abildung 1: Aufbau des Sensors (schematische Darstellung)

---

## Page 8 (Method: embedded_text)

2.1.2 TECHNISCHES PRINZIP 
Die Wandlung des angelegten Drucks in ein elektrisches Signal erfolgt durch Messung der Durchbie-
gung einer Siliziummembran. Auf dieser Siliziummembran ist eine Widerstandsbrücke mit piezore-
sistiven Widerständen aufgebracht. Die Brücke wird mit Spannung versorgt und bei Druckbeaufschla-
gung, d.h. bei Durchbiegung der Membran, verstimmt. Dadurch entsteht ein Spannungssignal, das 
zum Druck proportional ist (siehe Abbildung 2). 
 
 
Abbildung 2: Prinzip der Druckwandlung (Widerstandsbrücke auf Sensorelement) 
Das Brückensignal wird durch eine elektronische Auswerteschaltung (ASIC) weiterverarbeitet. Diese ver-
stärkt, skaliert und digitalisiert das Spannungssignal der Messbrücke. Das Drucksignal wird dann als 
analoger Wert über den Gerätestecker nach außen gegeben.  
Um Störungen durch elektromagnetische Einflüsse zu vermeiden, sind diverse EMV-Maßnahmen (EMV 
= Elektromagnetische Verträglichkeit) enthalten. Über den Kabelbaumstecker (Gegenstecker) und den 
Kabelbaum (nicht mehr Teile des hier beschriebenen Sensors) wird der Wert zur weiteren Auswertung 
zum Steuergerät übermittelt (siehe Abbildung 3). 
 
Abbildung 3: Signalkette des Drucksensors von der Messbrücke zum Steuergerät 
 
 
 
2.2 BESTIMMUNGSGEMÄßER GEBRAUCH 
Das Produkt wird auf Basis der zum Zeitpunkt der TKU-Erstellung für das Produkt direkt geltenden 
regulatorischen Anforderungen in folgenden Zielmärkten freigegeben:

---

## Page 9 (Method: embedded_text)

AT, BE, BG, CH, CN, CY, CZ, DE, DK, EE, ES, FI, FR, GB, GR, HK, HR, HU, IE, IT, JP, KR, LT, LU, 
LV, MT, NL, PL, PT, RO, SE, SI, SG, SK, USA,TW 
Das Produkt kann in einem freigegebenen Zielmarkt und in Kapitel 1 festgelegten Einsatzbereich unter 
Beachtung der in dieser TKU beschrieben Grenzen, Bedingungen und sonstigen Vorgaben verwendet 
werden ("Bestimmungsgemäßer Gebrauch").  
Für den Fall, dass der Kunde das Produkt außerhalb des Bestimmungsgemäßen Gebrauch verwen-
den möchte, hat der Kunde 
(1) in eigener Verantwortung die daraus für das Produkt resultierenden etwaigen regulatorischen Anfor-
derungen zu bewerten und einzuhalten sowie die Verwendungsfähigkeit des Produktes für den vom 
Kunden beabsichtigten Einsatzbereich zu bewerten und sicherzustellen oder 
(2) eine neue, erweiternde Bosch-Freigabe einzuholen, die bei Bosch gesondert zu beauftragen ist (z.B. 
mittels Änderungsschein). 
Lieferungen und Leistungen (Vertragserfüllung) stehen unter dem Vorbehalt, dass der Erfüllung keine 
Hindernisse aufgrund nationaler oder internationaler (Re-)Exportkontrollvorschriften, insbesondere 
Embargos oder sonstiger Sanktionen, entgegenstehen. 
 
Missbräuchliche Verwendung: 
 
Die Verwendung des Sensors in anderen als der in dieser TKU beschriebenen Anwendungen ist nicht 
freigegeben. Der Sensor darf nicht für die Messung von Drücken von anderen Medien wie z.B. CNG, 
Wasser, Kühlflüssigkeit, AdBlue, Lebensmittel oder Chemikalien verwendet werden. 
 
Die Verwendung in Luft-, Wasser- oder Weltraumfahrzeugen ist nicht freigegeben. 
 
Applikationsbedingungen: 
 
Schlauchmaterialien oder Kunststoffe, aus denen unter den Applikationsbedingungen Halogenverbin-
dungen freigesetzt werden, können die Leiterbahnstrukturen auf dem Sensorelement schädigen und dür-
fen daher nicht eingesetzt werden. Besonders kritisch sind Einbaupositionen mit geringer bzw. ohne 
Durchlüftung bei gleichzeitig geringem Volumen und großen Oberflächen (z.B. Zuleitungsschläuche oder 
geschlossene Kunststoffformteile), an denen Halogene austreten (dies ist z.B. der Fall bei iodid- oder 
bromidstabilisierten Polyamiden und FKM, deren Rezeptur iodorganische Stoffe enthält). 
 
 
2.3 PRODUKTSICHERHEIT 
 
2.3.1 FUNKTIONALE SICHERHEIT 
Bosch weist darauf hin, dass das Sensor gemäß QM entwickelt wurde und keine ASIL-klassifizierten 
Anforderungen (im Sinne der ISO 26262) umsetzt. Es ist daher von Bosch nicht für Anwendungen 
freigegeben, bei denen eine oder mehrere Anforderungen mit einer ASIL-Klassifizierung (oberhalb 
QM) dem Bosch-Lieferumfang zugeordnet sind. 
Der Kunde muss sicherstellen, dass der Sensor für die funktionalen Sicherheitsanforderungen ge-
eignet ist, die vom Gesamtsystem an den Sensor gestellt werden.

---

## Page 10 (Method: embedded_text)

2.3.2 DATENSCHUTZ, CYBERSECURITY UND OVER-THE-AIR ASPEKTE 
 
Der Sensor speichert oder verarbeitet keine digitalen Daten. Der ASIC ist gegen Umprogrammierung 
geschützt. Der Sensor kommuniziert nur unidirektional in Richtung des Motorsteuergerätes und be-
sitzt keine weitere Schnittstelle nach außen. 
 
2.3.3 SICHERHEITS- UND WARNHINWEISE 
 
Die Produktsicherheit ist nur dann gewährleistet, wenn die zulässigen Bedingungen eingehalten werden. 
 
2.3.3.1 ALLGEMEINE SICHERHEITSHINWEISE 
 
 
• 
Beachten Sie die geltenden Vorschriften zur Unfallverhütung und zum Umwelt-
schutz. 
• 
Beachten Sie die Sicherheitsbestimmungen und Bestimmungen des Landes, in dem das 
Produkt verwendet / angewendet wird. 
• 
Das Personal, das an mechanischen und elektrischen Installations- und Deinstallations-
verfahren beteiligt ist, muss für diese Aufgaben ausreichend qualifiziert und geschult sein 
(„autorisiertes Personal“). 
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
Nehmen Sie das Produkt erst in Betrieb, wenn Sie sicher sein können, dass das Endpro-
dukt (Motor oder Fahrzeug), in dem das Produkt installiert ist, allen geltenden Bestimmun-
gen, Sicherheitsbestimmungen und Normen der Anwendung entspricht. 
 
 
2.3.3.2 PRODUKT-SPEZIFISCHE SICHERHEITSHINWEISE 
 
 
 
GEFAHR 
Leckage! 
Der Sensor ist in wasserstoffführenden Systemen verbaut, die permanent unter Druck ste-
hen, auch im Ruhezustand des Fahrzeuges. Austretender Wasserstoff kann sich bei Kon-
takt mit einer Zünd-Quelle entzünden bzw. zur Explosion führen.   
Bei unsachgemäßer Lagerung, Transport oder Handhabung können Schäden an der  
Einschraub-Schnittstelle entstehen, die zu Wasserstoffleckage führen. 
• 
Der Sensor ist in der vorgesehenen Verpackung zu lagern und zu transportieren. 
• 
Ein beschädigter oder ein bei der Montage heruntergefallener Sensor muss aussor-
tiert werden und darf nicht mehr verbaut werden.  
• 
Vor Demontage des Sensors muss die Wasserstoffzufuhr an den Sensoranbauort 
abgestellt werden. 
• 
Ein demontierter Sensor darf nicht wiederverwendet werden. 
Es sind geeignete Maßnahmen zur Absicherung gegen mögliche Folgen von Wasserstofflecka-
gen durch den Kunden in eigener Verantwortung zu prüfen und zu treffen, wie z. B. Leckage-
Erkennungsfunktionen, eine entsprechende Reaktion auf höchster Warnstufe (Bediener-Inter-
face) sowie Konzepte für vorbeugende und reaktive Wartung.

---

## Page 11 (Method: embedded_text)

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
Die Oberfläche des Sensors kann heiß genug werden, um eine Verbrennungsgefahr darzu-
stellen! 
• 
Lassen Sie das System auf Raumtemperatur abkühlen, bevor Sie den Sensor berüh-
ren z.B. während Wartungsarbeiten. 
• 
Verwenden Sie hitzeabweisende Handschuhe. 
HINWEIS 
Gefahr der Beschädigung! 
Bei unsachgemäßem Transport oder unsachgemäßer Handhabung können der elektrische 
Stecker und Dichtkonus beschädigt werden. 
• 
Der Sensor ist in der vorgesehenen Verpackung zu lagern und zu transportieren. 
• 
Ein beschädigter oder ein bei der Montage heruntergefallener Sensor muss aussor-
tiert werden und darf nicht mehr verbaut werden. 
 
Gefahr auf Fehlfunktionen! 
Wenn der Sensor während der Installation, Wartung oder Demontage unter Spannung steht, 
kann dies zu Beschädigungen des Sensors führen. 
• 
Stellen Sie vor dem Ausbauen des Sensors sicher, dass keine elektrische Spannung 
am Gerätestecker des Sensors anliegt. 
Die Verschmutzung des elektrischen Steckers und des Gewindes kann zu fehlerhaften Funk-
tionen im Fuel Cell System führen. 
• 
Die Steckerschutzkappe (sofern vorhanden) ist unmittelbar vor der Entnahme aus der 
Verpackung zu entfernen. 
• 
Es ist auf Sauberkeit der Umgebung während Montage und Demontage zu achten. 
 
 
 
 
 
2.4 KENNZEICHNUNG DES PRODUKTS 
 
Die Bestellnummer, das Herstelldatum und eine laufende Seriennummer werden auf dem Gerätestecker-
gehäuse des Sensors angegeben (siehe Angebotszeichnung). 
 
2.5 MAßE UND GEWICHTE 
 
 
Abbildung 4: Typische Maße und Gewicht (Beispiel).

---

## Page 12 (Method: embedded_text)

2.6 LEISTUNGSAUFNAHME/-ABGABE 
Der Drucksensor hat eine typische Stromaufnahme von 12 mA bei einer Speisespannung von 5 V, also 
eine typische Verlustleistung von 60 mW. 
 
2.7 ALLGEMEINER HINWEIS ZU SERVICE, REPARATUR UND WARTUNG 
Das Produkt sieht keine Reparatur vor. 
Der Austausch des Produkts ist ausschließlich von fachlich qualifiziertem Personal durchzuführen.  
Ferner sind die Hinweise in Abschnitt 2.3 zu beachten. 
 
2.8 HINWEISE ZUR ENTSORGUNG UND ZUM RECYCLING 
Der Sensor muss in umweltfreundlicher Art entsorgt oder dem Recycling zugeführt werden. Hierzu sind 
die gesetzlichen Anforderungen und die Vorgehensweisen entsprechend dem Stand der Technik einzu-
halten. Der Sensor ist entsprechend der Vorgaben der Altautorichtlinie entwickelt, die Stoffzusammen-
setzung zu diesem Erzeugnis kann im dazugehörenden Materialdatenblatt (IMDS) eingesehen werden. 
 
3 
SYSTEMBESCHREIBUNG 
 
3.1  SYSTEM OF INTEREST (SOI) 
 
Der Sensor wird über eine Schraubverbindung im Bereich des H2 Kraftstoffmitteldruck eingeschraubt 
und tritt so in Kontakt mit dem Wasserstoff (Prinzip s. 5, Details zur Verschraubung siehe Angebots-
zeichnung). 
Das Drucksignal wird als analoges Ausgangssignal zur Auswertung an ein elektronisches Steuerge-
rät übertragen.  
 
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
Die Kabelentlastung des Kabelbaums muss entsprechend der Vorgaben der Angebotszeich-
nung sein. 
• 
Die Montage- und Handlungsweise müssen entsprechend der Vorgaben dieser TKU sein. 
 
HINWEIS 
Das System ist so auszulegen, dass in allen Betriebszuständen keine Drücke/Druckspitzen, z.B. durch 
Kavitation oder dynamische Druckschläge, entstehen, die den zulässigen Überdruck des Sensors 
überschreiten.  
 
Der Kunde muss entsprechende Prüfungen vornehmen, um zu ermitteln, ob es bei dem konkreten 
Einsatz des Sensors in seinem System zur Vermeidung von Kavitation den Verbau einer Drossel im

---

## Page 13 (Method: embedded_text)

Sensor bedarf. Nur durch eine entsprechende Systemauslegung und Prüfung kann vermieden wer-
den, dass es zu einer durch Kavitation oder dynamische Druckschläge verursachten Leckage in die 
Umwelt kommt.  
 
Abbildung 5: Systembild Wasserstoffsystem 
 
 
 
 
 
 
HINWEIS 
Durch einen geeigneten Einbau im Fahrzeug (empfohlene Einbaulage siehe Angebotszeichnung) ist 
sicherzustellen, dass sich kein Wasser an der Membran sammeln kann. 
Auswirkung: Messbrückenverstimmung oder Membranbruch im Frostfall 
Bei der Montage des Sensors sind die Hinweise unter Abschnitt 4.8.2 zu beachten. 
Beim Handling des Sensors ist darauf zu achten, dass es zu keinen ESD-Schäden kommt. Bei der Aus-
legung des Einbauortes ist darauf zu achten, dass es nicht zu elektrischen Pulsen, elektrischen Feldern 
oder induktiven elektrischen Einkopplungen auf den Drucksensor kommt, die diesen beschädigen könn-
ten. Siehe auch Hinweise unter Abschnitt 4.8.2.  
Die EMV-Festigkeit des Sensors ist entsprechend den in 6.1 angegebenen Prüfungen abgesichert. 
 
Emissionen mobiler Transceiver im Nahbereich können die vorgegebenen Kundenanforderungen 
bzgl. Störfestigkeit überschreiten. Bei Abständen kleiner oder gleich 200 mm zwischen der verbauten 
Kfz-Komponente und allen möglichen Positionen mobiler Transceiver – letztere bei PKW und NKW 
eingeschränkt auf Fahrgastzelle und Kofferraum – können Funktionsstörungen der Kfz-Komponente 
nicht ausgeschlossen werden. Beabsichtigt der Kunde den Sicherheitsabstand von 200 mm zu unter-
schreiten, liegt es in seiner Verantwortung, für diese Applikation zu prüfen, ob zusätzliche Maßnahmen 
erforderlich sind und diese dann umzusetzen. 
Eine galvanische Trennung zwischen der elektrischen Masse des Motors und des Steuergeräts ist zu 
vermeiden. 
Herstellprozesse, bei denen elektrostatische Felder verwendet werden, müssen auf Einhaltung der ESD-
Festigkeit überprüft werden.

---

## Page 14 (Method: embedded_text)

Elektrostatisches Lackieren des Sensors ist nicht zulässig. Es besteht die Gefahr von Schäden an den 
elektronischen Bauteilen des Sensors durch elektrostatische Entladung (ESD). 
Vor Elektroschweißarbeiten am Fahrzeug muss der Kabelbaumstecker (Gegenstecker) vom Sensor ab-
gezogen sein. Schädliche Ströme in den elektronischen Bauteilen werden dadurch vermieden. 
 
3.2  PRODUKTSCHNITTSTELLE 
 
Zur Identifikation werden die Schnittstellen wie folgt benannt: 
 
 
 
Abbildung 6: Prinzipdarstellung mechanische/elektrische Schnittstelle 
3.2.1 
MECHANISCHE SCHNITTSTELLE 
Der Drucksensor wird mit dem Gewindestück in das aufnehmende Gegenstück eingeschraubt und 
über den Dichtkonus abgedichtet. 
Bei der Montage muss das maximal zulässige Anzugsmoment gemäß Abschnitt 4.2.5 berücksichtigt 
werden. Die Gestaltung der mechanischen Schnittstelle im Gegenstück muss gemäß der Angebots-
zeichnung (geometrische Gestaltung und Materialauswahl) erfolgen. Nur für diese Auslegung sind 
die in diesem Dokument angegebenen Randbedingungen abgesichert. 
Die Dichtheit der Schnittstelle des Sensors zum Gegenstück ist durch den Kunden zu verantworten. 
Hinsichtlich Eigenschaften bezüglich Dichtheit s. Abschnitt 6.1.5 (Dichtheitsprüfung / Messung ex-
terne Leckage). 
 
 
 
 
 
 
 
Elektrische  
Schnittstelle 
Mechanische 
Schnittstelle

---

## Page 15 (Method: embedded_text)

3.2.2 
STECKER / STECKVERBINDUNG ELEKTRISCH 
Steckverbindung nach Kundenwunsch. Kunde stellt Spezifikationskonformität und Zuverlässigkeit in 
eigener Verantwortung sicher. Für Mängel am Kabelbaumstecker und ggf. weitere sich daraus erge-
bende Folgemängel übernimmt RB keine Sachmängelhaftung und wird von Ansprüchen Dritter frei-
gestellt. Bosch stellt Schnittstellenkonformität komponentenseitig gemäß Kundenvorgabe sicher. So-
weit nicht anders vereinbart, ist folgende Verantwortungsmatrix (RASIC) gültig: 
Umfang / Gegenstand 
Verantwortlich: Bosch 
Verantwortlich: Kunde 
Kabelbaumstecker 
Nein 
Ja 
Einhaltung Spezifikation Steckverbindung 
Nein 
Ja 
Ausführung Steckerschnittstelle, komponentenseitig 
Ja 
Nein 
 
Es sind geeignete Kabel zu verwenden (temperaturfest, nicht geschirmt), und die Vorschrift zur Kabel-
führung in der Angebotszeichnung muss eingehalten werden. 
Die Oberflächen der Flachstecker am Sensor und an den Kontakten am Kabelbaumstecker müssen aus 
dem gleichen Werkstoff sein. Eine sogenannte Mischkontaktierung (z.B. goldbeschichtete Flachstecker 
mit silberbeschichteten Kontakten) am Kabelbaumstecker ist nicht zulässig. 
3.2.3 
STECKERBELEGUNG UND BESCHALTUNG DES SENSORS 
Die Zuordnung der elektrischen Beschaltung zu der Position der Kontaktpins im Gerätestecker  
(Steckerbelegung) ist in der Angebotszeichnung definiert. Der Drucksensor ist durch das Steuergerät 
gemäß Abbildung 7 zu beschalten und mit einer Speisespannung von 5 V zu versorgen. 
 
Abbildung 7: Beschaltung Drucksensor im Steuergerät 
Der Drucksensor liefert ein analoges Ausgangssignal, das sich ratiometrisch zur Versorgungsspan-
nung US verhält. 
Der elektrische Ausgang des Sensors ist so gestaltet, dass Fehlfunktionen durch Kabelbrüche oder 
Kurzschlüsse mit der in Abbildung 7 dargestellten Beschaltung mit einem Pull-up-Widerstand gegen 
Versorgungspannung erkannt werden können.

---

## Page 16 (Method: embedded_text)

Zudem ist im Signalpfad ein Tiefpassfilter (R1, C1) vorzusehen. Die EMV-Spezifikation des Sensors 
ist mit dieser Referenzbeschaltung abgesichert. 
Das Sensorgehäuse ist elektrisch leitfähig an die Fahrzeugmasse anzubinden. Der Übergangswider-
stand darf maximal 2,5 mΩ betragen. 
 
4 
TECHNISCHE DATEN MIT MESSGRÖßEN UND MESSBEDINGUNGEN 
 
4.1 FUNKTIONEN UND FUNKTIONSZUSTÄNDE (BETRIEBSARTEN), FUNKTIONALE KENNWERTE UND 
RANDBEDINGUNGEN 
 
4.1.1 
ÜBERTRAGUNGSFUNKTION DRUCKSIGNAL 
Die Signalausgangsspannung wird (bis zum Nenndruck) aus dem tatsächlichen Druck wie folgt berech-
net:  
 
UOUT = (c1⋅prel+c0)⋅US 
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
 
  
 
Abbildung 8: Ausgangssignal als Funktion des Drucks

---

## Page 17 (Method: embedded_text)

ZEICHEN 
WERT 
EINHEIT 
Oberer Diagnosebereich für Sig-
nal Range Check (SRC) 
Usrch 
[0,96 … 1] * US 
V 
Oberer ungenutzter Bereich 
 
]0,94 … 0,96[ * US 
V 
Obere Begrenzung der Aus-
gangsspannung 
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
Untere Begrenzung der Aus-
gangsspannung 
Ucll (C1) 
[0,06 … 0,1] * US 
V 
Unterer ungenutzter Bereich 
 
]0,04 … 0,06[ * US 
V 
Unterer Diagnosebereich für Sig-
nal Range Check (SRC) 
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
 unteres Fehlerband und kein Reset 
-2 
 unteres Fehlerband und Reset wird ausgelöst

---

## Page 18 (Method: embedded_text)

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
High Voltage Supply detection, programmable 
threshold 
no Reset, debouncing 
-1

---

## Page 19 (Method: embedded_text)

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
 
4.1.3 VERHALTEN NACH RESET UND INITIALISIERUNG 
Bei bestimmten Fehlern (siehe Tabelle 2) wird im Drucksensor ein Reset ausgelöst. Dieser Reset wird 
dann alle 400ms generiert. Nach einem Reset und während der nachfolgenden Initialisierung des 
Sensors, wird der Ausgang auf Masse gezogen.  
Falls der Fehler noch anliegt, bleibt das Ausgangssignal im Fehlerband.  
Im Falle, dass der Fehler nicht mehr anliegt, steuert das Ausgangssignal seinen Wert in den gültigen 
Arbeitsbereich. Der Verlauf des Ausgangssignals und die dazugehörigen typischen Zeiten bei Raumtem-
peratur nach dem Reset sind in Abbildung 9 und Tabelle 3 dargestellt..  
 
 
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
 
 
Tabelle 2: Reaktion des Sensors im Fehlerfall 
Abbildung 9: Darstellung der Zeiten nach Reset und Initialisierung

---

## Page 20 (Method: embedded_text)

4.1.4 VERHALTEN NACH UNTER- UND ÜBERSPANNUNG 
Bei einer Unter- oder Überspannungserkennung wird die Signalausgangspannung Uout auf Massepo-
tential gezogen. 
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
4.1.5 VERHALTEN BEI ÜBERTEMPERATURERKENNUNG 
Bei einer Übertemperaturerkennung wird die Signalausgangsspannung Uout auf Massepotential ge-
zogen. 
 
Bei diesem Drucksensor sind folgende Grenzwerte appliziert. 
PARAMETER 
MIN [°C] 
TYP [°C] 
MAX [°C] 
Übertemperaturerkennung 
150 
162.5 
175 
Tabelle 4: Grenzwerte für Übertemperaturerkennung 
 
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
 
Tabelle 5: Grenzwerte für Überdruckerkennung 
 
4.2  MECHANISCHE KENNWERTE 
 
4.2.1 
DRÜCKE 
Der Drucksensor ist für einen Systemdruck gemäß Tabelle 3 ausgelegt, zuzüglich der Überschwinger, 
die im Drucklastkollektiv berücksichtigt sein müssen. Die Kennwerte und die maximale Belastbarkeit sind 
unten beschrieben. 
GRÖßE 
ZEICHEN 
WERT 
EINHEIT 
Messbereich 
(Nenndruck) 
p1 (p0) 
p2 (pn) 
0 
3300 
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

---

## Page 21 (Method: embedded_text)

4.2.2 
TOLERANZEN DER DRUCKMESSUNG 
 
Die Toleranz der Druckmessung wird in %FSS angegeben. FSS (Full Scale Span) bezieht sich auf 
die nutzbare Spanne (4V). Zu den unten genannten Toleranzen ist zusätzlich ein Ratiometriefehler 
von ±0,15 %FSS zu addieren, der sich durch Abweichungen von der nominalen Versorgungsspan-
nung US (5 V) ergibt. Die Toleranz ist von der Temperatur abhängig und ist gültig für 0-km und Feld. 
Die Toleranzen werden statistisch mit ±3s pro Fertigungslos und bei Beschaltung gemäß Abbildung 
7 eingehalten. 100% verlesene Erzeugnisse dürfen geliefert werden.  
 
Abbildung 10: Toleranz der Druckmessung 
Beispiel: @25°C => Toleranz = ±1,5 %FSS ≙ ±60 mV ≙ ±49.5 kPa 
 
4.2.3 
VIBRATIONSBELASTUNG 
 
Die hier angegebenen zulässigen Belastungen in Tabelle 8 sind für den Drucksensor durch Erprobung 
abgesichert und gelten für den Sensor. Die Steckverbindung wird nicht beurteilt. 
Die Werte gelten für die Anbaustelle des Sensors gemessen am Sechskant des Sensors 
 
PARAMETER 
RB KOMPAKT / 
FREMDSTECKSYSTEM  
Sinusschütteln: 
Amplitude/ 
Beschleunigung 
70-147Hz:            s=0,25mm 
147-1350Hz:        a=210m/s² 
1350-2000Hz: 
a=175m/s² 
Tabelle 8: Zulässige maximale Vibrationsbelastung  
Die zulässige Schwingbeschleunigung im Betrieb richtet sich nach der Prüfschärfe und –dauer und gilt 
für Sensoren mit Stecksystem in Bosch-Verantwortung. Bei Fremdstecksystemen sind zusätzlich die 
hierfür zulässigen Grenzen einzuhalten. 
Die Absicherung erfolgt mit der Schwingprüfung nach Abschnitt 6.1.1.

---

## Page 22 (Method: embedded_text)

4.2.4 
MAXIMAL ZULÄSSIGES ANZUGSMOMENT 
Maximal zulässige Einschraubbelastung: 27 Nm 
Details s. Angebotszeichnung. 
 
 
4.3 ELEKTRISCHE KENNWERTE 
In Tabelle 6 sind Grenzen wichtiger Sensorgrößen dargestellt. Diese werden statistisch mit ±3s pro Fer-
tigungslos eingehalten. 100 % verlesene Erzeugnisse dürfen geliefert werden. 
 
 
1 Ruhestrom bei 5 V Versorgung 
2 Anstieg von 10 % auf 90 % statistisches Ausgangssignal. Eventuelle Einflüsse durch den Kabelbaum oder das Steuerge-
rät müssen gesondert untersucht werden 
3 Autarkiezeit ist die Zeit, die der Sensor ohne Versorgungsspannung am Eingang, ohne Reset Daten überträgt ausgehend 
von Us = 4.5 V statisch. 
PARAMETER 
ZEICHEN 
WERTE: MIN WERTE: MAX 
EINHEIT 
Speisespannung 
US 
4,75 
5,25 
V 
Ruhestromaufnahme bei US 1 
IS 
 
16,5 
mA 
Einschaltstrom 
Irush 
 
60 
mA 
Ausgangsimpedanz für 
0,1⋅US < UOut < 0,94⋅US 
Rd 
 
10 
Ω 
Max. Speisespannung (max. 1 h) 
US,max 
 
18 
V 
Kurzschluss Signalleitung zu GND bzw. 
zu Speisespannung (max. 8 h) bei 
gleichzeitiger Versorgung von US mit 
US,kurz 
US,short 
0 
18 
V 
Maximal zulässiger Strom bei Verpolung 
zweier Steckerkabel (max. 5 Min.@25 C) 
Imax 
 
260 
mA 
Lastkapazität gegen Masse  
CL 
 
13 
nF 
Ansprechzeit UOut beim Einschalten von 
Us 
tini 
 
2,0 
ms 
Sprungantwortzeit auf Druckänderungen 
UOut 2 
t10/90 
0,2 
0,8 
ms 
Autarkiezeit bei Unterbrechung der Ver-
sorgungsspannung 3 
tautarky 
10 
 
µs 
Eingangsstrom bei Fehlerfall US offen 
und VAusgang = 5 V. 
IUs_offen 
 
5 
µA 
Ausgangsstrom bei Fehlerfall GND offen 
und VAusgang = 0 V. 
IGND_offen 
 
350 
µA 
Tabelle 6: Grenzwerte der Signalübertragung, Versorgungsspannung und -ströme

---

## Page 23 (Method: embedded_text)

4.4 KLIMATISCHE KENNWERTE 
Der Sensor ist für den Einsatz im Motorraum (direkter Motoranbau) und die entsprechenden Umge-
bungstemperaturen ausgelegt mit einer Temperaturverteilung lt. Tabelle 7. 
 
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
Tabelle 7: Zulässige Temperaturverteilung 
 
Es sind die in Tabelle 8 angegebenen Temperaturen zulässig. 
PARAMETER 
WERTE 
EINHEIT 
RANDBEDINGUNGEN 
Minimale Temperatur (Umgebung) 
-40 
°C 
 
Maximale Temperatur (Umgebung) 
 120 
°C 
 
Minimale Temperatur (Medium) 
-40 
°C 
 
Maximale Temperatur (Medium) 
 120 
°C 
 
Tabelle 8: Zulässige Temperaturen im Betrieb 
 
 
4.5 CHEMISCHE KENNWERTE 
Der Sensor ist für den Einsatz in H2-ICE-Systemen vorgesehen.  
Er ist für den Einsatz mit trockenem Wasserstoff abgesichert . 
 
 
4.6 
AKUSTISCHE KENNWERTE 
Nicht zutreffend. 
 
 
4.7 LEBENSDAUER 
 
Unter der in dieser TKU beschriebenen Verwendung und den angegebenen Einsatzbedingungen (s. 
Kapitel 4.4) ist das Produkt auf eine Lebensdauer von 30.000 Betriebsstunden bzw. 15 Jahren aus-
gelegt (je nachdem, was zuerst eintritt). 
 
Diese Lebensdauerangabe muss von der anwendbaren Gewährleistungsfrist unterschieden werden, 
die unabhängig von dieser Lebensdauerangabe abläuft.

---

## Page 24 (Method: embedded_text)

Bosch geht davon aus, dass der OEM prüft, ob die in dieser TKU angegebenen Lebensdaueranga-
ben des Bosch-Produkts mit den für das OEM-System im jeweiligen Zielland geltenden gesetzlichen 
Anforderungen an die Lebensdauer übereinstimmen und ob bestimmte Maßnahmen getroffen wer-
den müssen, um die relevanten gesetzlichen Lebensdauervorgaben einzuhalten (z.B. ob das Bosch-
Produkt nach der Lebensdauerangabe ausgetauscht werden muss). 
 
 
4.8 TRANSPORT, MONTAGE, IN-/AUßERBETRIEBNAHME, LAGERUNG 
Bitte unbedingt die Sicherheits- und Warnhinweise in Abschnitt 2.3 beachten! 
 
4.8.1 
TRANSPORT 
Der Transport des Drucksensors hat in dafür vorgesehenen Verpackungen zu erfolgen. Falls der Druck-
sensor bereits montiert war, ist zu beachten, dass sich noch Restmengen Prüfflüssigkeit im Druckstutzen 
befinden können. Deswegen ist die Verpackung so auszulegen, dass beim Transport keine Prüfflüssig-
keit in den Gerätesteckerbereich gelangen kann. 
Abweichend zu den Lagerbedingungen (s. Abschnitt 4.8.4) ist während des Transportes eine Erweiterung 
des Temperaturbereiches für max. 48h zulässig (s. Tabelle 9: TransportbedingungenTabelle 9). 
 
DAUER 
TEMPERATUR 
RELATIVE LUFTFEUCHTE 
Max. 48 h 
-40 °C bis +80 °C 
0 % to 80 % 
Tabelle 9: Transportbedingungen 
 
4.8.2 
MONTAGE 
Die Angaben auf der Angebotszeichnung des Sensors sind grundsätzlich zu beachten. 
Der Drucksensor ist beim Einschrauben nur am Sechskant zu handhaben. Bei der Montage ist ein Steck- 
oder Ringschlüssel zu verwenden. Dieser darf nur am Sechskant angesetzt werden und muss diesen 
sicher überdecken. Ein Verkippen oder Verkanten des Steck-/Ringschlüssels mit dem Gerätestecker 
muss ausgeschlossen werden. Gabelschlüssel sind für die Montage nicht erlaubt. 
HINWEIS 
Schlagartige Impulse wie z.B. durch Hammerschläge oder Körnerpunkte in der Nähe des Sensors 
müssen vermieden werden; diese können zur Beschädigung des Sensorelements und zu Undichthei-
ten führen. 
Bei der Montage des Sensors darf der Gerätestecker nicht gegenüber dem Druckstutzen verdreht 
werden, der Sensor wird sonst zerstört. Der Gerätesteckerbereich darf nicht mit flüssigen oder festen 
Medien verschmutzt werden. 
HINWEIS 
Es ist sicherzustellen, dass der Dichtbereich zur Einbaustelle nicht beschädigt wird. Ein Eindringen  
von Fremdstoffen in oder an den Druckstutzen während der Montage muss ausgeschlossen werden, 
die Verbindung zwischen Sensor und Anbaustelle könnte sonst undicht werden. 
Empfehlungen bezüglich der Schmierstoffe sind in der Angebotszeichnung beschrieben. Details zur 
Verschraubung siehe Angebotszeichnung.

---

## Page 25 (Method: embedded_text)

Nachdem der Drucksensor in der beschriebenen Weise an seine Anbaustelle eingeschraubt wurde, 
wird er am Sechskant angezogen. Das zulässige Anzugsmoment ist in der Angebotszeichnung fest-
gelegt. 
Eine Wiederverschraubung des Sensors ist nicht zulässig.  
HINWEIS 
Die Kontakte im Gerätestecker dürfen bei Montagearbeiten nicht berührt werden: Gefahr von ESD-
Schäden. Elektrische Messungen am Drucksensor dürfen nur mit abgefederten sauberen Teller- oder 
Kehlkontakten (frei von flüssigen und festen Medien) erfolgen. Bei der Montage darf kein „hot plug-
ging“ (= An-/Abbringen des Kabelbaumsteckers bei anliegender Spannung) erfolgen: Gefahr von EOS-
Schäden (EOS = electrical overstress). Bei elektrischer Prüfung des Sensors ist sowohl Verpolung als 
auch Überspannung auszuschließen. Die Prüfungen sind mit einer Spannungsquelle mit einer Strom-
begrenzung von 50 mA durchzuführen. 
Werden Tauchprüfungen durchgeführt, sind die elektrische Steckverbindung und der Druckanschluss 
zum Schutz vor Eindringen von Wasser zu verschließen. 
HINWEIS 
Bei Dichtheitsprüfungen sind Unterdrücke im Bereich des Gerätesteckers zu vermeiden. 
Heruntergefallene Sensoren dürfen nicht mehr weiterverwendet werden. 
 
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

---

## Page 26 (Method: embedded_text)

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
Tabelle 10: Lagerbedingungen 
Der Drucksensor ist trocken und staubfrei zu lagern. Es ist darauf zu achten, dass es zu keiner Ver-
schmutzung mit flüssigen oder festen Medien im Gerätesteckerbereich bzw. am Druckstutzen kommt. 
Bei versilberten Steckkontakten sind schwefelhaltige Atmosphären zu vermeiden, da es sonst zu Oxi-
dationen der Silber-Oberflächen kommen kann.  
Ebenso darf der Sensor nicht in der Nähe von Materialien gelagert werden, die Halogenverbindungen 
freisetzen könnten (vgl. Kapitel 2.2). 
 
Diese Lagerbedingungen führen zu keiner Änderung der Eigenschaften und der Funktion des Druck-
sensors. Nach Überschreiten der Höchstlagerdauer müssen die Sensoren zur kostenpflichtigen Über-
prüfung zurück an Bosch geschickt werden. 
 
4.8.5 
ELEKTRISCHE FEHLERDIAGNOSE 
 
Mit folgendem Verfahren kann die Funktionsfähigkeit des Sensors festgestellt werden:  
 
a. Fehlerspeicher im Steuergerät auslesen, ob der Sensor als defekt angezeigt wird.  
b. Kurzdiagnose: Messung der Offsetspannung wie in Abschnitt 4.8.6 beschrieben.  
 
Auswertung: Bei einem funktionstüchtigen Sensor ergibt sich die Ausgangsspannung wie in Ab-
schnitt 4.8.6 beschrieben. 
 
4.8.6 
ELEKTRISCHE PRÜFUNG DER FUNKTION 
 
Prüfbedingungen 
Umgebungstemperatur 
23 ± 5 °C 
Speisespannung 
5,000 ± 0,250 V 
Akzeptanzkriterien 
Messwert Uout bei p0 
0,5 ± 0,060 V 
Messwert Uout bei pn 
4,5 ± 0,060 V 
 
Tabelle 10: Elektrische Prüfung 
Falls die Speisespannung US vom Nennwert 5,000 V abweicht, ist die gemessene Ausgangs-span-
nung Uout mit dem Faktor 5,000 V/US auf den Nennwert umzurechnen.

---

## Page 27 (Method: embedded_text)

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
p0 und pn siehe Abschnitt 4.2.1 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
Abbildung 10: Prüfaufbau elektrische Funktion 
 
5 
SERIENBEGLEITENDE PRÜFUNGEN 
Die folgenden Erprobungen werden zur Überwachung und Absicherung der Produktqualität serienbeglei-
tend durchgeführt. Eine Raffung der Erprobungsdauer durch Lasterhöhung ist zulässig. 
 
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

---

## Page 28 (Method: embedded_text)

Stichprobenumfang und Häufigkeit werden von der Qualitätsabteilung des Werks festgelegt. Die Test-
anforderungen können von den in Abschnitt 6.1 genannten abweichen. 
 
6 
ERPROBUNG 
Die nachfolgend aufgeführten Erprobungen richten sich nach den mechanischen, elektrischen, klimati-
schen und chemischen Beanspruchungen im Motorraum bei Motoranbau des Drucksensors. Sie decken 
erfahrungsgemäß die zu erwartenden Beanspruchungen über der Fahrzeuglebensdauer ab. In Zweifels-
fällen sind die tatsächlich auftretenden Belastungen gemeinsam mit Bosch zu bewerten. Die Schüttelbe-
dingungen und Temperaturen an der Sensoranbaustelle sind in jedem Fall zu messen. 
Falls nicht anders angegeben, werden die Erprobungen einmalig im Rahmen der Freigabeerprobung 
durchgeführt. Für jede Prüfung werden neue Teile und wenn zutreffend neue Gegenstecker verwendet. 
Akzeptanzkriterium, falls nicht anders angegeben: Nach der Erprobung erfüllen die Prüflinge die Kennli-
nientoleranzen gemäß Abschnitt 4.2.2. 
 
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
 
 
 
Testbeschreibung: Sinusschüttelprüfung 
 
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

## Page 29 (Method: embedded_text)

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
Hinweis: Der Drucksensor ist nach der Prüfung nicht mehr funktionsfähig  
 
 
6.1.4 
HYDROSTATISCHE FESTIGKEIT 
 
Testbeschreibung: 
Prüfung der Hydrostatischen Festigkeit 
Ziel: 
Beständigkeitsprüfung gegen Drücke oberhalb des Messbereichs 
Testprofil: 
Prüfablauf 
 
1) Prüfung der Hydrostatischen Festigkeit 
        Prüfdruck: 62,5 bar_rel 
        Druckanstiegsrate: ca. 3 bar/s 
        Prüfdauer: 3 min 
        Prüfmedium: Luft 
        Prüftemperatur: Raumtemperatur 
 
2) Berstdruckprüfung 
  Druckanstiegsrate: ≤ 10 bar/s 
  Prüfmedium: Öl 
 
Norm: 
ISO 12619-8:2017(E) 
Kriterium: 
… für Schritt 1) des Prüfablaufs: keine sichtbaren Anzeichen eines Bruchs 
… für Schritt 2) des Prüfablaufs: 
      - Berstdruck ≥ 80 % der Berstdruckfestigkeit eines Neuteil-Sensors 
         oder 
      - Berstdruck > 100 bar_rel 
 
Der Sensor muss nach der Prüfung nicht mehr funktionsfähig sein. 
 
 
6.1.5 
DICHTHEITSPRÜFUNG 
 
Testbeschreibung: 
Dichtheitsprüfung 
Ziel: 
Messung der Externen Leckage 
Testprofil: 
Prüfbedingungen 
25 bar (relativ) @ -40 °C 
37,5 bar (relativ) @ 95 °C 
 
Norm: 
Durchführung gemäß ISO 12619-2:2014(E), Test 6.2 
Kriterium: 
Externe Leckage < 2,78*10-3 mbar * l / s 
 
6.1.6 
WIDERSTAND GEGEN DREHMOMENTÜBERSCHREITUNG 
 
Testbeschreibung: 
Widerstand gegen Drehmomentüberschreitung 
Ziel: 
Robustheit gegenüber Überschreitung des Anzugmoments 
Testprofil: 
Prüfdrehmoment: 30 Nm 
Prüfdauer: 15 min 
 
Norm: 
Durchführung gemäß ISO 12619-2:2014(E), Test 7 
Kriterium: 
Kein(e) Verformung, Bruch oder Leckage des Sensors

---

## Page 30 (Method: embedded_text)

6.1.7 
MECHANISCH DYNAMISCHE PRÜFUNG: HOCHDRUCKPUSLVERSUCH 
 
Testbeschreibung: 
Mechanisch dynamische Prüfung: Hochdruckpulsversuch 
Ziel 
Die Pulsations- / Druckwechselprüfung dient der Erprobung eines 
Erzeugnisses bezüglich Funktion und Stoffeigenschaften (Schädi-
gungsmechanismus Ermüdung)   
Testprofil: 
• 
Druckpuls-Profil: pmin bis pmax 
• 
pmin = 0…5 bar 
• 
pmax = 66 bar 
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
 
6.1.8 
FUNKTIONSDAUERLAUF 
 
Testbeschreibung: 
Funktionsdauerlauf 
Ziel: 
Prüfung gegenüber Ermüdung 
Testprofil: 
Anzahl Druckwechselzyklen 
18000 Zyklen @ Raumtemperatur 
1000 Zyklen @ 90 °C 
1000 Zyklen @ -40 °C 
 
Druckzyklus: Druckwechsel von 12,5 nach 25 bar_rel 
 
Testmedium: Wasserstoff 5.0 (trocken) 
Norm: 
Durchführung gemäß ISO 12619-8:2017(E), Test 6.4 
Kriterium: 
Kennlinie, Externe Leckage und Hydrostatische Festigkeit gemäß 
Spezifikation 
 
6.1.9 
HOCHTEMPERATURLAGERUNG 
 
Testbeschreibung: 
Lagerung bei Hochtemperatur (drucklos) 
Ziel: 
Die Lagerung bei hoher Temperatur dient der Erprobung des  
Erzeugnisses bezüglich Funktion und Stoffeigenschaften (Schädi-
gungsmechanismus Alterung, insbesondere bei Kunststoffen).   
Testprofil: 
Temperatur: 150°C 
Dauer: 
1000h  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-2 (2008-05)

---

## Page 31 (Method: embedded_text)

6.1.10 TIEFTEMPERATURLAGERUNG 
 
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
 
6.1.11 TEMPERATURSCHOCKWECHSEL 
 
Testbeschreibung: 
Temperaturschockwechselprüfung 
Ziel: 
Die unterschiedlichen in Erzeugnissen verbauten Materialien er-
fahren durch den Temperaturwechsel eine Wechselbelastung. 
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
 
6.1.12 WASSERSCHUTZPRÜFUNG 
 
Testbeschreibung: 
Wasserschutzprüfung 
Ziel: 
Beständigkeit gegen Wasserstrahl 
Testprofil: 
- 
Schutzklassen: IPX6K und IPX9K 
- Prüflinge mit abgedichtetem Stecker,  
ohne elektrischen Betrieb; 
Dauer: 
Gemäß Norm  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Kein Wasser im Elektronikraum Stecker zulässig 
Norm  
ISO 20653 (2013-02) 
 
6.1.13 FEUCHTEWECHSEL/ FEUCHTEWECHSELPRÜFUNG 
 
Testbeschreibung: 
Feuchtewechselprüfung 
Ziel: 
Beständigkeit gegen relative Feuchte. 
Es soll die Eignung der Prüflinge für Betrieb und Lagerung bei ho-
hen relativen Luftfeuchtigkeiten und der Widerstandsfähigkeit ge-
gen Einwirkung feuchter Luft und bei Betauung getestet werden. 
Testprofil: 
- 
Prüfung mit 93% rel. Luftfeuchtigkeit bei 65°C und bei 25°C 
inkl. genormter Übergänge 
- 
Voralterung: 100 TSW von -40°C…+150°C 
- 
Durchführung der Prüfung mit Gegenstecker (Vermeidung

---

## Page 32 (Method: embedded_text)

Betauung Steckerschnittstelle) 
Dauer: 
10 Zyklen à 24 h, davon 5 Zyklen mit Frost  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
DIN EN 60068-2-38 (2010-06) 
 
6.1.14 CHEMISCHE BESTÄNDIGKEIT 
 
Testbeschreibung: 
Chemische Beständigkeit 
Ziel: 
Die Medienbeständigkeit nach ISO 16750-5 beschreibt die Anfor-
derungen und Prüfungen durch die Widerstandsfähigkeit der Sys-
teme / Komponenten gegen die chemischen Beanspruchungen im 
Fahrzeuglebenslauf. 
Testprofil: 
- 
Gemäß Norm 
- 
Flüssigkeiten: Dieselkraftstoff, Rapsmethylester, Motoröl, Kalt-
reiniger, Motorkühlflüssigkeit und Bremsflüssigkeit; 
Dauer: 
Gemäß Norm  
Kriterium: 
Sensor funktionsfähig gemäß Abschnitt 4.2.2 
Norm  
ISO 16750-5 (2010-04) 
 
6.1.15 BESTÄNDIGKEIT GEGEN WEITERE MEDIEN 
 
Testbeschreibung: 
Chemische Beständigkeit gegen weitere Medien 
Ziel: 
Die Medienbeständigkeit nach ISO 16750-5 beschreibt die Anforde-
rungen und Prüfungen durch die Widerstandsfähigkeit der Systeme / 
Komponenten gegen die chemischen Beanspruchungen im Fahr-
zeuglebenslauf. 
Testprofil: 
Einbauort: Motorraum  
 
Medien 
• 
Alkalisches Waschmittel Bonderite C-SO HK (25 °C, 22 h)  
• 
Getriebeöl TITAN Gear Hyp 90, Fuchs (85 °C, 22 h)  
• 
Servolenkungsflüssigkeit Titan ZH 4300 B (85 °C, 22 h)  
• 
HVAC-Öl PAG 100 (85 °C, 22 h)  
• 
Kühlmittel Glysantin G30 (85 °C, 22 h) 
• 
Mischung Glysantin G30 / DI-Wasser (48 % / 52 %; 85 °C, 22 h) 
• 
Düngemittelnitrate (85 °C, 22 h) 
• 
25-Gew.-% Natriumhydroxid in Wasser (85 °C, 22 h) 
 
Norm: 
ISO 16750-5 (2010-04) 
Kriterium: 
Externe Leckage und Hydrostatische Festigkeit gemäß Spezifikation 
 
 
6.1.16 ZYKLISCHE KORROSIONSPRÜFUNG (CCT) 
 
Testbeschreibung: 
Zyklische Korrosionsprüfung 
Ziel: 
Prüfung des Langszeitkorrosionsverhaltens unter extrem korrosi-
ven Bedingungen 
Testprofil: 
Anzahl Testzyklen: 200 
 
Beschreibung eines Testzyklus: 
1. Salzsprühnebelphase: 4h, 35 °C, 5 % NaCl 
2. Trockenphase: 2 h, 60 °C, RH < 50 % 
3. Feuchtphase: 2 h, 50 °C, RH = 90 … 95 %

---

## Page 33 (Method: embedded_text)

Norm 
RB-Norm N42AP 226 
Kriterium: 
Kennlinie und Externe Leckage gemäß Spezifikation; Sensor ist 
demontierbar 
 
 
6.1.17 ISOLATIONSWIDERSTAND 
 
Testbeschreibung: 
Isolationswiderstand  
Ziel: 
Prüfung auf mögliche Ausfälle der Isolierung zwischen Kontaktstif-
ten und Bauteilgehäuse 
Testprofil: 
Prüfspannung: 1000 V 
Prüfdauer: 60 s 
Prüfpartner: Kontaktpins gegen Kunststoff-Steckergehäuse 
Kriterium: 
Isolationswiderstand > 10 MΩ 
Norm 
Durchführung deckt ISO 12619-8:2017 und  
ANSI HGV 3.1:22 (Kapitel 13.3.4 - Insulation Resistance) 
 
 
6.1.18 BCI-TEST 
 
Open Loop Methode nach ISO 11452-4; CBCI; Modulation CW und AM. 
Akzeptanzkriterium: Druckgenauigkeit von ±1,5 %FSS, jedoch angewendet auf Signale aus ungestör-
tem Mittelwert. 
 
 
6.1.19 ANTENNE 
 
Nach ISO 11452-2; Frequenzbereich 200MHz-3,2GHz; Modulation CW und AM. 
Abbildung 11: Open Loop Stromprofil

---

## Page 34 (Method: embedded_text)

Akzeptanzkriterium: Kennlinie für Drucksignal innerhalb des Toleranzbandes gemäß Abschnitt 4.2.2,  
jedoch angewendet auf Signale aus ungestörtem Mittelwert. 
 
 
6.1.20 ESD-TESTS 
Akzeptanzkriterium für jeden einzelnen Test: Kennlinie für Drucksignal innerhalb des Toleranzbandes 
gemäß Abschnitt 4.2.2, jedoch angewendet auf Signale aus ungestörtem Mittelwert. 
 
ESD-Handling 
Nach ISO 10605 
Netzwerk: 
150 pF/ 330 Ω 
Kontaktentladung auf Pins/ Gehäuse-Metall: ± 8 kV 
Luftentladung auf Gehäuse-Kunststoff: 
± 15 kV 
 
ESD-Operating Direct 
Nach ISO 10605 
Netzwerk: 
330 pF/ 330 Ω 
Kontaktentladung auf Gehäuse-Metall: 
± 8 kV 
Luftentladung auf Gehäuse-Kunststoff: 
± 15 kV 
 
ESD-Operating Indirect 
Nach ISO 10605 (HCP-Setup) 
Netzwerk: 
330 pF/ 330 Ω 
Kontaktentladung auf Entladeinseln: 
± 15 kV 
 
 
 
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
 
Abbildung 12: Antenne-Profil

---

## Page 35 (Method: embedded_text)

6.1.21 TEST-PULSE AUF SENSORLEITUNGEN 
 
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
Der Kunde ist dafür verantwortlich, zu überprüfen und nachzuweisen, dass das Produkt für den vorgese-
henen Zweck geeignet ist, und ist für die ordnungsgemäße Kalibrierung des Produkts verantwortlich. 
 
7 
BEWERTUNG VON PRODUKTEN NACH VERWENDUNG BEI 0-KM IM FELD 
Produkte gelten als mangelfrei, wenn sie die in dieser TKU aufgeführten Kenndaten/Prüfdaten für 0-km 
und Feld erreichen.

---

## Page 36 (Method: embedded_text)

INFORMATION ON SAFETY INSTRUCTIONS IN THIS DOCUMENT 
 
 This is the safety alert symbol. It is used to alert you to potential personal injury hazards. Obey all safety 
messages that follow this symbol to avoid possible injury or death. 
 
Risk classes in this document are:  
 
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
 
The safety instructions in special section 2.3.3 are structured as follows: 
 
 RISK CLASS 
Type and source of danger! 
Consequences in case of non-compliance 
• Hazard avoidance measures 
 
In other general sections the safety instructions are shown as follows: 
 
 RISK CLASS 
Safety text.

---

## Page 37 (Method: embedded_text)

2 
GENERAL PRODUCT DESCRIPTION 
 
2.1 
MAIN FUNCTIONS AND CHARACTERISTICS OF THE PRODUCT 
The medium pressure sensor MPS1 (hereinafter called pressure sensor or sensor) is used to measure 
fluid middle pressure in H2-ICE systems. It´s placed in Hydrogen injection system. 
The sensor measures fluid pressure in the specified pressure range to ambient pressure. 
2.1.1 BASIC DESIGN 
The basic design of the pressure sensor is shown in Figure 1. 
The sensor chip is soldered onto a carrier which is welded to the pressure port. The hexagon, which is 
used to screw the sensor into the receiving component, is welded to pressure port. It also serves to 
incorporate the printed circuit board with spacer as well as the contact/ground springs and the connector 
housing. The electrical connection between the sensor chip and printed circuit board is made by bond 
wires. Contact springs are installed on the printed circuit board to create the connection to the connector 
housing. The device connector, which is provided with a metal insert, is in turn welded onto the hexagon. 
Pressure balancing between electronic compartment and surrounding is done via a ventilation hole in 
the ground of the connector. In case of a relative pressure sensor the pressure in the electronic com-
partment (connector housing) represents the reference pressure. 
 
 
Figure 1: Design of the sensor (schematic representation) 
 
2.1.2 
TECHNICAL PRINCIPLE 
The conversion of the applied fluid pressure into an electrical signal is accomplished by measuring the 
deflection of a silicon membrane integrated in a MEMS-chip. A bridge circuit (Wheatstone bridge) 
consisting of piezo resistive resistors is located on the top side of the membrane; by applying fluid 
pressure to the sensor chip, i.e. by deflecting the membrane, the Wheatstone bridge put out of tune,

---

## Page 38 (Method: embedded_text)

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
 
 
2.2 
INTENDED USE 
The product is released based on the regulatory requirements directly applicable to the product at the 
time of TCD creation in the following target market(s): 
AT, BE, BG, CH, CN, CY, CZ, DE, DK, EE, ES, FI, FR, GB, GR, HK, HR, HU, IE, IT, JP, KR, LT, LU, 
LV, MT, NL, PL, PT, RO, SE, SI, SG, SK, USA,TW 
 
The product can be used in a released target market in the applications set out in chapter 1, subject 
to the limits, conditions and other specifications described in this TCD ("Intended use").

---

## Page 39 (Method: embedded_text)

In the case that the customer wants to use the product outside the Intended Use, the customer shall 
(1) on its own responsibility evaluate and comply with any regulatory requirements resulting there from 
for the product and evaluate and ensure the usability of the product for the area of application in-
tended by the customer or  
(2) obtain a new, extending Bosch release, which shall be ordered separately from Bosch (e.g. by means 
of a change request). 
Deliveries and services (fulfilment of contract) shall be subject to the provision that there are no ob-
stacles to performance due to national or international (re-)export control regulation, in particular em-
bargoes or other sanctions. 
 
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
 
Bosch points out that the sensor has been developed according to QM and does not implement any 
ASIL-classified requirements (in the sense of ISO 26262). It is therefore not approved by Bosch for 
applications in which one or more requirements with an ASIL classification (above QM) are assigned 
to the Bosch scope of supply. 
 
The customer has to ensure that the sensor is suitable to be used within the overall system regard-
ing any functional safety requirements assigned to the sensor in the overall system. 
 
2.3.2 
DATA PROTECTION, CYBER SECURITY AND OVER-THE-AIR ASPECTS 
 
The sensor does not store or process any digital data. The ASIC is protected against reprogram-
ming. The sensor only communicates unidirectionally in the direction of the engine control unit and 
has no further external interface.

---

## Page 40 (Method: embedded_text)

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
 
 
2.3.3.2 PRODUCT SPECIFIC SAFETY INSTRUCTIONS 
 
   DANGER 
Leakage! 
The sensor is obstructed in hydrogen-systems which stand constantly under  
pressure; even if the vehicle is switched off. Leaking hydrogen can ignite  
or cause an explosion if it comes into contact with an ignition source. 
If the sensor is stored, transported or handled improperly, damages to high  
pressure hydraulic interface could occur and lead to leakage, causing risk to inju
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

---

## Page 41 (Method: embedded_text)

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

---

## Page 42 (Method: embedded_text)

2.7 
GENERAL REMARKS ON SERVICE, REPAIR, AND MAINTENANCE 
Repair of the product is not possible. 
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

---

## Page 43 (Method: embedded_text)

Figure 5: Schematic representation hydrogen system 
 
NOTICE 
A suitable installation in the vehicle (recommended installation position, see offer drawing) should b
used to ensure that water cannot collect on the membrane. 
risk: detuning of measurement bridge or membrane rupture in case of frost 
For the installation process of the sensor, the instructions in section 4.8.2 are to be followed. 
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

---

## Page 44 (Method: embedded_text)

3.2 
PRODUCT INTERFACE 
For identification, the interfaces are named as follows:  
 
 
Figure 6: Interface identification 
 
3.2.1 
MECHANICAL INTERFACE 
The pressure sensor is screwed in with the pressure port into the receiving component and sealed by 
a sealing cone. 
The maximum tightening torque according to section 4.2.5 is to be considered during assembly. The 
design of the mechanical interface in the receiving component shall be made according to the offer 
drawing (geometrical design and material selection). Durability and sealing of the product in described 
conditions of this document are ensured only for this design. 
The customer is responsible for the tightness of the sensor’s interface with the counterpart. 
Concerning performance with regards to tightness see chapter 6.1.5 (Leakage Testing / Leak Rate 
Measurement). 
 
3.2.2 
PLUG CONNECTOR / ELECTRICAL PLUG CONNECTION 
Plug connection system according to customer request. Customer is responsible to assure specifica-
tion conformity and reliability. RB is not liable for defects on the wiring harness plug and any possible 
consequential defects resulting thereof and is exempt from third party claims. Bosch assures compo-
nent-related interface conformity in accordance with customer specification. Unless otherwise speci-
fied, the following responsibility matrix (RASIC) is valid: 
 
Scope / subject 
Responsible: Bosch 
Responsible: customer 
Wiring harness plug 
no 
yes 
Observance of specification plug connection 
no 
yes 
Type plug interface, component related 
yes 
no 
Suitable cables (temperature resistant, not shielded) should be used, the instruction for the cable rout-
ing defined in the offer drawing must be complied with. 
Electrical  
Interface 
Mechanical 
Interface

---

## Page 45 (Method: embedded_text)

The blade terminals of the sensor and on the contacts of the wiring harness connector have to be 
carried out of with similar surface finishes. A so-called mixed contacting (e.g. gold-plated blade termi-
nals with silver-coated contacts) on the wiring harness connector is forbidden. 
 
3.2.3 
CONNECTION DIAGRAM AND WIRING OF THE SENSOR 
The assignment of the electrical wiring to the position of the contact pins in the device connector (pin 
assignment) is defined in the offer drawing. The pressure sensor is to be wired through the ECU accord-
ing to Figure 7 and to be supplied with supply voltage of 5V. 
 
 
Figure 7: Pressure sensor wiring in the ECU 
 
The pressure sensor provides an analog output signal that has a ratiometric relationship with the sup-
ply voltage US. 
The electrical output of the sensor is designed such that malfunctions due to cable breaks or short-
circuits with the wiring illustrated in Figure 7. can be detected with a pull-up resistor against supply 
voltage. 
In addition, a low-pass filter (R1, C1) must be provided in the signal path. The EMC specification of 
the sensor is accomplished with the proposed circuitry. 
The sensor housing is to connect electrically conductive to the vehicle ground. The contact resistance 
must not exceed 2.5 mΩ. 
 
4 
TECHNICAL DATA WITH MEASURED VARIABLES AND MEASURING CONDITIONS  
 
4.1 
FUNCTIONS, FUNCTION STATES (MODES OF OPERATION), FUNCTIONAL CHARACTERISTICS AND 
BOUNDARY CONDITIONS 
 
4.1.1 
TRANSFER FUNCTION OF PRESSURE SIGNAL 
 
The signal output voltage (up to the nominal pressure) is calculated from the actual pressure as ex-
plained below:

---

## Page 46 (Method: embedded_text)

Uout= (c1⋅prel+c0)⋅US 
 
 
with  
Uout 
= 
Signal output voltage [V] 
 
US 
= 
Supply voltage [V] 
 
prel 
= 
Relative Pressure [kPa] 
 
c0 
= 
0.02 
 
c1 
= 
0.0002424 kPa-1 
  
 
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
Figure 8: Signal output voltage UOUT as function of the pressure

---

## Page 47 (Method: embedded_text)

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
 
 
4.1.2 
ERROR DIAGNOSIS  
 
Diagnostic ranges lying outside the operating range are provided for the error diagnosis (see section 
4.1.1). Since the sensor characteristic of the upper operating range is limited, overpressure conditions 
can be distinguished from errors.  
The coding of the response to an error in the Table 2 is as follows: 
 
-1 
 lower error band and no reset 
-2 
 lower error band and reset is triggered 
 
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

## Page 48 (Method: embedded_text)

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
 
 
4.1.3 
BEHAVIOR AT STARTUP AND RESET 
In case of specific errors (see Table 2), a reset is triggered in the pressure sensor. These are then 
generated every 400ms. After a reset, and during the subsequent sensor initialization, the output is 
connected to ground.  
If the error is still present, the output signal remains in the error band.  
In case, the error is no longer present, the output signal controls its value in the applicable operating 
range. The course of the output signal and the related typical times at room temperature after the 
reset are shown in Figure 9 and Table 3.

---

## Page 49 (Method: embedded_text)

Figure 9: Representation of the times after reset and initialization 
 
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

---

## Page 50 (Method: embedded_text)

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
3300 
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
 
 
4.2.2 TOLERANCES PRESSURE SIGNAL 
The tolerance of the pressure measurement is given in %FSS. FSS (Full Scale Span) refers to the  
usable span (4V). In addition to the tolerances mentioned below, a ratiometric error of ±0.15 %FSS 
shall be added, which results from deviations from the nominal supply voltage US (5 V). The tolerance 
depends on the temperature and is valid for 0-km and field. The tolerances are statistically adhered to 
with ±3s per production lot and with wiring according to Figure 7. 100% selected products may be 
delivered..

---

## Page 51 (Method: embedded_text)

Figure 10: Tolerance of pressure measurement 
Example: @25°C => Tolerance = ±1,5 %FSS ≙ ±60 mV ≙ ±49.5 kPa 
 
 
4.2.3 
VIBRATION LOADS 
The permissible loads specified in Table 5 for the pressure sensor are validated through testing and are 
applicable for the sensor. The plug connection is not evaluated. 
The values apply at the mounting position of the sensor, measured at the hexagon of the sensor. 
PARAMETER 
THIRD PARTY 
CONNECTORS  
Sinusoidal vibration 
test: 
Amplitude/ 
Acceleration 
70-147Hz:           s=0,25mm 
147-1350Hz:       a=210m/s² 
1350-2000Hz:  
a=175m/s² 
Table 8: Maximum permissible vibration loads 
 
The permissible vibration acceleration during operation depends on the test severity and duration and 
applies to sensors with a plug-in system under Bosch's responsibility. In the case of third-party con-
nector systems, the permissible limits must also be adhered to. 
The protection is carried out with the vibration test according to Section 6.1.1. 
 
4.2.4 
PERMISSIBLE TIGHTENING TORQUE 
Max. permissible tightening torque load: 27 Nm 
For details s. offer drawing.

---

## Page 52 (Method: embedded_text)

4.3 
ELECTRICAL CHARACTERISTICS 
The limits of important sensor variables are shown in Table 6. These are complied statistically with 
±3s per production batch. 100 % sorted products should be delivered. 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
4 Quiescent current at 5V supply 
5 Increase from 10 % to 90 % statical output signal. Possible influences due to the wiring harness or the control unit must 
be examined separately. 
6 Autarchy time is the time for which the sensor transmits data without supply voltage at the input, without reset assuming 
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
Quiescent current consumption at US4 
IS 
 
16.5 
mA 
Inrush Current 
Irush 
 
60 
mA 
Output impedance for 0.1⋅US < UOut < 0.94⋅US 
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
UOut  5 
t10/90 
0.2 
0.8 
ms 
Autarchy time in case supply voltage is inter-
rupted6 
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
Table 7: Limit values of the signal transmission, supply voltage and supply currents

---

## Page 53 (Method: embedded_text)

4.4 
CLIMATIC CHARACTERISTICS 
The sensor is designed for the use in the engine compartment (direct engine mounting) and the cor-
responding ambient temperatures with a typical temperature distribution as per Table 7. 
 
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
Table 8: Permissible temperature distribution 
 
The temperatures specified in Table 8 are permissible. 
PARAMETER 
VALUE 
UNIT 
BOUNDARY CONDTIONS 
Min. Temperature (ambient) 
-40 
°C 
 
Max. Temperature (ambient) 
120 
°C 
 
Min. Temperature (Medium) 
-40 
°C 
 
Max. Temperature (Medium) 
120 
°C 
 
Table 9: Permissible temperatures during operation 
 
 
4.5 
CHEMICAL CHARACTERISTICS 
 
The sensor is intended for the use in H2-ICE system. It is rated for use with dry hydrogen. 
 
 
 
4.6 
ACOUSTIC CHARACTERISTICS 
Not applicable. 
 
 
4.7 
LIFETIME 
With respect to the use and usage conditions described in this TCD (see chapter 4.4), the life of the 
product is designed for 30,000 operating hours, or 15 years (whichever occurs first).  
This lifetime statement must be differentiated from the applicable warranty period which expires inde-
pendently from the duration of this lifetime statement. 
 
Bosch trusts that the OEM will assess if the Bosch-product’s lifetime specified in this TCD is in line 
with the relevant legal lifetime requirements applicable to the OEM’s system in the specific target 
country and whether any particular measures need to be taken in order to comply with relevant legal

---

## Page 54 (Method: embedded_text)

lifetime requirements (e.g. if the Bosch-product needs to be replaced after the Bosch-product’s life-
time period). 
 
4.8 
TRANSPORT, ASSEMBLY, START AND END OF OPERATION, STORAGE 
Please pay special attention to the safety and warning notes in section 2.3! 
 
 
4.8.1 
TRANSPORT 
The pressure sensor has to be transported in packaging which is designed for it. If the pressure sensor 
was already mounted, it should be noted that there may be residual quantities of test fluid in the pres-
sure ports. Therefore, the packaging must be designed in such a way that no test fluid can get into the 
device connector area during transport. 
Deviating from the storage conditions (s. section 4.8.4) an extension of the temperature range for max. 
48h is allowed (s. table 9). 
 
PERIOD 
TEMPERATURE 
RELATIVE AIR HUMIDITY 
Max. 48h 
-40 °C to +80 °C 
0 % to 80 % 
Table 10: Transport Conditions 
 
 
4.8.2 
ASSEMBLY 
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

---

## Page 55 (Method: embedded_text)

It is not permitted to re-screw the sensor. 
NOTICE 
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
START AND END OF OPERATION 
 
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
Table 11: Storage conditions

---

## Page 56 (Method: embedded_text)

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
b) Short diagnosis: measuring the offset voltage as described in section 4.8.6 
c) Evaluation: For a functioning sensor, the output voltage is as described in section 4.8.6 
 
 
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
Table 12: Electrical Testing 
 
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
• 
p0 und pn see section 4.2.1

---

## Page 57 (Method: embedded_text)

Figure 9: Test setup - electrical 
function 
 
 
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
are to be evaluated together with Bosch. The vibration conditions and temperatures at the sensor instal-
lation point are to be measured in each case. 
Unless otherwise specified, the tests are conducted once within the framework of release testing. New 
parts and, if needed, new mating connectors are used for each test.

---

## Page 58 (Method: embedded_text)

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
 
6.1.4 
HYDROSTATIC STRENGTH 
 
Test Description: 
Test of Hydrostatic Strength 
Goal: 
Resistance test of pressures above the measuring range 
Test Profile: 
Test Sequence 
 
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

## Page 59 (Method: embedded_text)

1) Hydrostatic Strength Test 
        Test Pressure: 62.5 bar_g 
        Pressure Ramp-up: ca. 3 bar/s 
        Test Duration: 3 min 
        Test Medium: Air 
        Test Temperature: Room Temperature 
 
2) Burst Pressure Test 
        Pressure Ramp-up: ≤ 10 bar/s 
        Test Medium: Oil 
 
Standard: 
ISO 12619-8:2017(E)  
Criteria: 
… for step 1) of test sequence: no visible evidence of rupture 
… for step 2) of test sequence:  
       - burst pressure ≥ 80% of the burst pressure strength of a new part sensor  
          or  
       - burst pressure > 100 bar_g  
 
The sensor must no longer be functional after the test. 
 
 
6.1.5 
LEAKAGE TESTING 
 
Test Description: 
Leakage Testing 
Goal: 
Leak Rate Measurement 
Test Profile: 
Test Conditions 
25 bar (relative) @ -40 °C 
37.5 bar (relative) @ 95 °C 
 
Standard: 
Testing according to ISO 12619-2:2014(E), Test 6.2 
Criterion: 
External Leakage < 2,78*10-3 mbar * l / s 
 
6.1.6 
EXCESS TORQUE RESISTANCE 
 
Test Description: 
Excess Torque Resistance 
Goal: 
Robustness against exceeding tightening torque  
Test Profile: 
Test Torque: 30 Nm 
Test Duration: 15 min 
 
Standard: 
Testing according to ISO 12619-2:2014(E), Test 7 
Criterion: 
No deformation, breakage, or leakage of DUT 
 
6.1.7 
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

---

## Page 60 (Method: embedded_text)

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
 
 
6.1.8 
CONTINUED OPERATION TEST 
 
Test Description: 
Continued Operation Test 
Goal: 
Test against fatigue 
Test Profile: 
Number of Pressure Cycles 
18000 cycles @ Room Temperature 
1000 cycles @ 90 °C 
1000 cycles @ -40 °C 
 
Pressure Cycle: Cycle from 12.5 to 25 bar_g 
 
Test Medium: Hydrogen 5.0 (dry) 
 
Standard: 
Test according to ISO 12619-8:2017(E), Test 6.4 
Criterion: 
Characteristic curve, external leakage, and hydrostatic strength 
according to spec 
 
 
6.1.9 
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
 
6.1.10 LOW TEMPERATURE STORAGE 
 
Test description: 
Storage at low temperature 
Goal: 
The storage at low temperature is used to test the product in 
terms of function and material properties. 
Test profile: 
Temperature: -40°C 
Duration: 
200 h

---

## Page 61 (Method: embedded_text)

Criteria: 
Sensor functional according to section 4.2.2 
Standard: 
DIN EN 60068-2-1 (2008-01) 
 
6.1.11 TEMPERATURE SHOCK CYCLE TEST 
 
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
 
6.1.12 
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
 
6.1.13 HUMIDITY CHANGE TEST 
 
 
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

---

## Page 62 (Method: embedded_text)

6.1.14 CHEMICAL DURABILITY 
 
Test description: 
Chemicale resistance test 
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
 
6.1.15 RESISTANCE TO OTHER MEDIA 
 
Test Description: 
Chemicale resistance test 
Goal: 
The media resistance according to ISO 16750-5 describes the re-
quirements and tests due to the resistance of the systems /  
components against the chemical stresses in the vehicle life cycle. 
Test Profile 
Mounting Location: Engine Compartment 
 
Media 
• 
Alkaline Detergent Bonderite C-SO HK (25 °C, 22 h)  
• 
Gear Oil TITAN Gear Hyp 90, Fuchs (85 °C, 22 h)  
• 
Power Steering Fluid Titan ZH 4300 B (85 °C, 22 h)  
• 
HVAC Oil PAG 100 (85 °C, 22 h)  
• 
Coolant Glysantin G30 (85 °C, 22 h) 
• 
Mixture Glysantin G30 / DI Water (48 % / 52 %; 85 °C, 22 h) 
• 
Fertilizer nitrates (85 °C, 22 h) 
• 
25 wt. % Sodium hydroxide in water (85 °C, 22 h) 
 
Standard: 
ISO 16750-5 (2010-04) 
Criterion: 
External leakage and hydrostatic strength according to spec. 
 
6.1.16 CYCLIC CORROSION TESTING (CCT) 
 
Test Description: 
Cyclic Corrosion Testing 
Goal: 
Test of long-term corrosion behavior under extremely corrosive 
conditions 
Test Profile: 
Number of Test cycles: 200 
 
Description of one test cycle: 
4. Salt spray fog phase: 4h, 35 °C, 5 % NaCl 
5. Dry phase: 2 h, 60 °C, RH < 50 % 
6. Humid phase: 2 h, 50 °C, RH = 90 … 95 %  
 
Standard. 
RB Standard N42AP 226 
Criterion: 
Characteristic curve and external leakage according to spec; sen-
sor is demountable

---

## Page 63 (Method: embedded_text)

6.1.17 INSULATION RESISTANCE 
 
Test Description: 
Insulation Resistance 
Goal: 
Check for potential failures of the insulation between contact pins 
and component casing 
Test Profile: 
Test Voltage: 1000 V 
Test Duration: 60 s 
Test Partner: Contact pins against plastic connector housing 
Criterion: 
Insolation Resistance > 10 MΩ 
Standard: 
Test execution covers ISO 12619-8:2017 and  
ANSI HGV 3.1:22 (chapter 13.3.4 - Insulation Resistance) 
 
6.1.18 BCI-TEST 
 
Open Loop method according to ISO 11452-4; CBCI; Modulation CW and AM. 
Pass Criterion: Pressure Accuracy of ±1.5 %FSS but tolerance band applied to signals based on un-
disturbed mean value. 
 
 
 
6.1.19 RADIATED INTERFERENCE (ANTENNA) 
According to ISO 11452-2; frequency range 200MHz-3.2GHz; Modulation CW and AM. 
Pass Criterion: Characteristic of pressure signal within tolerance band according to section 4.2.2 but 
tolerance band applied to signals based on undisturbed mean value. 
Figure 10: Open Loop current profile

---

## Page 64 (Method: embedded_text)

6.1.20 ESD-TEST 
Pass criterion for each individual test: Characteristic of pressure signal within tolerance band accord-
ing to section 4.2.2 but tolerance band applied to signals based on undisturbed mean value. 
ESD-Handling 
According to ISO 10605 
Network: 
150 pF/ 330 Ω 
Contact discharge to pins/ housing-metal: 
± 8 kV 
Air discharge to housing-plastic: 
± 15 kV 
 
ESD-Operating Direct 
According to ISO 10605 
Network: 
330 pF/ 330 Ω 
Contact discharge to housing-metal: 
± 8 kV 
Air discharge to housing-plastic: 
± 15 kV 
 
ESD-Operating Indirect 
According to ISO 10605 (HCP-Setup) 
Network: 
330 pF/ 330 Ω 
Contact discharge to discharge islands: 
± 15 kV 
 
 
Frequency 
Level 
Modula-
tion 
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
Figure 11: Antenna profile

---

## Page 65 (Method: embedded_text)

6.1.21 TEST PULSE ON SENSOR LINES 
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
TESTING BY THE CUSTOMER 
The customer is responsible for verifying and demonstrating that the product is fit for its intended use, 
and is responsible for the correct calibration of the product. 
 
7 
ASSESSMENT OF PRODUCTS RETURNED FROM 0-MILEAGE AND FROM THE FIELD 
Products are considered good if they fulfill the specifications/test data for 0-mileage / field listed in the 
TCD.

---

## Page 66 (Method: embedded_text)

8 
APPENDICES AND REFERENCES 
8.1 
REFERENCES STANDARDS 
 
 
List of mentioned Regulations and Standards 
ANSI HGV 3.1:22 
DIN 50100: 1978-02 
DIN EN 60068-2-11: 2000-02 
DIN EN 60068-2-18: 2001-10 
DIN EN 60068-2-2: 2008-05 
DIN EN 60068-2-6: 2008-10 
ISO 11452-2: 2004-11  
ISO 11452-4: 2003-01 
IEC 60068-2-6: 2007-12 
ISO 10605: 2001-12 
ISO 12619-2:2014(E) 
ISO 12619-8:2017(E) 
ISO 16750-2:2012(E) 
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
RB Standard N42AP 226

---

## Page 67 (Method: embedded_text)

9 
HISTORIE/ HISTORY  
 
 
 
2 
KONTAKTPERSON / CONTACT PERSON 
 
Kontaktperson / Contact Person: 
PS-SU/ECI4, Frank Bühner 
 
 
Ausgabe/ 
Release 
Datum/ 
Date 
Bearbeiter / 
Edited by 
Änderungsbeschreibung/  
Release Description 
01 
07.12.2023 
Bühner 
Draft Edition

---

## Page 68 (Method: OCR (pytesseract))

Robert Bosch GmbH
PS-SU

---
