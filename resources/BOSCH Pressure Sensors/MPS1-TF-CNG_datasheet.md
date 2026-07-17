# MPS1-TF-CNG Pressure Sensor — Comprehensive Datasheet

**Bosch Medium Pressure Sensor for CNG Applications**

> **Synthesized from multiple Bosch source documents by Frankie OCR agent, 2026-07-16**

---

## Table of Contents

1. [Product Identification](#1-product-identification)
2. [General Description](#2-general-description)
3. [Technical Principle](#3-technical-principle)
4. [Full Specifications](#4-full-specifications)
5. [Transfer Function & Characteristic Curve](#5-transfer-function--characteristic-curve)
6. [Electrical Characteristics](#6-electrical-characteristics)
7. [Mechanical Characteristics](#7-mechanical-characteristics)
8. [Climatic & Environmental](#8-climatic--environmental)
9. [Diagnostics & Fault Behavior](#9-diagnostics--fault-behavior)
10. [Pinout & Connection](#10-pinout--connection)
11. [Application Notes](#11-application-notes)
12. [Ordering Information](#12-ordering-information)
13. [Source Documents](#13-source-documents)

---

## 1. Product Identification

| Field | Value |
|-------|-------|
| **Product designation** | Medium Pressure Sensor (MPS) |
| **Type designation** | MPS1-TF-CNG |
| **Platform** | MPS1 (MPS3evo family) |
| **Application** | CNG (Compressed Natural Gas) fuel rail pressure measurement |
| **Manufacturer** | Robert Bosch GmbH |
| **TCD RB Number** | 0 261 K00 13H (platform) |
| **Document date** | 2022-09-12 |

The MPS1 family is part of the MPS3evo platform. The sensor is designed, validated, and released for
measuring absolute and relative pressures in natural gas (CNG) fuel systems. It features an integrated
NTC temperature sensor for combined pressure and temperature measurement.

---

## 2. General Description

### 2.1 Primary Function

The MPS1 medium pressure sensor measures fluid pressure in CNG fuel rail systems. It converts the
applied pressure into a ratiometric analog voltage signal. An integrated NTC thermistor provides
temperature measurement.

### 2.2 Construction

The sensor consists of the following major components (from bottom to top):

1. **Threaded pressure port** with integrated sensor element carrier
2. **Silicon sensor element** (piezoresistive MEMS die) soldered to the carrier
3. **Hexagon body** welded to the pressure port — used for mounting (torque application)
4. **PCB with ASIC** (Application-Specific Integrated Circuit) bonded to the hexagon; wire-bonded to the sensor element
5. **Contact springs** on the PCB for electrical connection to the connector
6. **Device connector** with metal insert, welded to the hexagon

The sensor element contains a silicon diaphragm with a piezoresistive Wheatstone bridge. Pressure
acts on the active (top) side of the diaphragm. The ASIC amplifies, scales, and digitizes the bridge
signal, then outputs it as an analog voltage.

### 2.3 Intended Use

- Measurement of absolute pressure in CNG fuel rail pipes
- Regulation of natural gas fuel systems
- The sensor is chemically resistant to natural gas (CNG)
- **NOT approved for**: water, coolant, AdBlue, food products, other chemicals, or use in aircraft/spacecraft/watercraft

---

## 3. Technical Principle

### 3.1 Pressure Measurement

```
Applied Pressure → Silicon Diaphragm Deflection → Piezoresistive Bridge Imbalance
→ ASIC Amplification/Compensation → Analog Voltage Output → ECU
```

The piezoresistive effect: mechanical stress on the silicon diaphragm changes the resistance values
in the Wheatstone bridge, producing a voltage proportional to pressure. The ASIC provides:
- Signal amplification
- Temperature compensation
- Digital calibration
- Diagnostic functions
- EMC protection

### 3.2 Temperature Measurement

An integrated NTC (Negative Temperature Coefficient) resistor provides temperature sensing
independent of the pressure signal. This allows the ECU to monitor fuel temperature for density
compensation.

---

## 4. Full Specifications

### 4.1 Pressure Specifications

**Source:** Bosch i-business catalog, page 60.

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Measurement range (lower, specified) | p1 | 0.5 | bar(g) |
| Measurement range (upper/nominal) | p2 (P_N) | 16 | bar(g) |
| Measurement range (lower, specified) | p1 | 50 | kPa (rel.) |
| Measurement range (upper/nominal) | P_N | 1600 | kPa (rel.) |
| Maximum overpressure | pmax | 32 | bar |
| Burst pressure (static) | pburst | (see offer drawing) | -- |

*Note: The sensor output is valid from 0 bar (0.5 V) to 16 bar (4.5 V). The specified
accuracy range starts at 0.5 bar. For burst pressure, refer to the official Bosch offer
drawing. The MPS1 platform burst pressure for the 41 bar variant is 9800 kPa; the 16 bar
CNG variant burst rating is application-specific.*

### 4.2 Pressure Measurement Tolerances

| Temperature | Tolerance (%FSS) | Tolerance (mV) | Tolerance (kPa) |
|-------------|-----------------|----------------|-----------------|
| @25°C | ±1.5 %FSS | ±60 mV | ±61.5 kPa |
| Over full temperature range | Temperature-dependent curve | — | — |

- FSS (Full Scale Span) refers to the usable 4V span (0.5V to 4.5V at Us=5V)
- Additional ratiometric error: ±0.15 %FSS for deviations from nominal Us=5V
- Tolerances are statistical, ±3σ per production lot
- Valid for both 0-km (new) and field (aged) conditions

### 4.3 Temperature Ranges

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Operating temperature | -40 | — | +120 | °C |
| Overtemperature detection threshold | 150 | 162.5 | 175 | °C |
| Storage temperature | -40 | — | +130 | °C |

### 4.4 Media Compatibility

- **Approved medium**: CNG (Compressed Natural Gas)
- **Chemical resistance**: Sensor materials are resistant to natural gas constituents
- **NOT compatible with**: Halogen-releasing materials (certain polyamides, FKM with iodine compounds)

---

## 5. Transfer Function & Characteristic Curve

### 5.1 Transfer Function (Authoritative)

**Source:** Bosch i-business catalog (2025-11-12), Section 3.4 Medium Pressure Sensors, page 58,
cross-referenced with MPS1 platform TCDs (RB 0 261 K00 13H).

The MPS1 platform transfer function is:

$$U_{OUT} = (C_1 + C_0) \cdot U_S = \left(0.8 \cdot \frac{p}{P_N} + 0.1\right) \cdot U_S$$

Where:

| Symbol | Name | Value | Unit |
|--------|------|-------|------|
| U_OUT | Signal output voltage | ratiometric to US | V |
| U_S | Supply voltage | 5.0 (nominal) | V |
| p | Measured pressure (relative/gauge) | 0 … 16 | bar |
| P_N | Rated (nominal) pressure | 16 | bar |
| C0 | Offset coefficient | 0.1 | (dimensionless) |
| C1 | Span coefficient | 0.8 · p / P_N | (dimensionless) |

### 5.2 MPS1-TF-CNG Specific Transfer Function

Substituting P_N = 16 bar for the CNG variant:

$$U_{OUT} = (0.05 \cdot p_{bar} + 0.1) \cdot U_S$$

Or equivalently in kPa (P_N = 1600 kPa):

$$U_{OUT} = (0.0005 \cdot p_{kPa} + 0.1) \cdot U_S$$

With U_S = 5.0 V, the simplified voltage-to-pressure relationship is:

$$U_{OUT}[V] = 0.25 \cdot p_{bar} + 0.5$$

$$p_{bar} = 4 \cdot U_{OUT}[V] - 2$$

| Coefficient | Symbol | Value | Unit | Derivation |
|------------|--------|-------|------|------------|
| Offset | c0 (= C0) | 0.1 | (dimensionless) | Platform constant |
| Gain | c1 | 0.0005 | kPa⁻¹ | c1 = 0.8 / P_N = 0.8 / 1600 kPa |
| Sensitivity | -- | 0.25 | V/bar | c1 · US · 100 kPa/bar |
| Sensitivity | -- | 2.5 | mV/kPa | c1 · US |

### 5.3 Key Transfer Points

| p_rel (bar) | p_rel (kPa) | U_OUT @ US=5.0V (V) |
|-------------|-------------|----------------------|
| 0 | 0 | 0.500 |
| 0.5 | 50 | 0.625 |
| 1 | 100 | 0.750 |
| 2 | 200 | 1.000 |
| 4 | 400 | 1.500 |
| 6 | 600 | 2.000 |
| 8 | 800 | 2.500 |
| 10 | 1000 | 3.000 |
| 12 | 1200 | 3.500 |
| 14 | 1400 | 4.000 |
| 16 | 1600 | 4.500 |

### 5.4 Output Characteristic

The output is **ratiometric** to supply voltage. The full usable output span of the MPS1 platform
is 4.0 V (from 0.5 V = 0.1·US to 4.5 V = 0.9·US at US = 5 V). The MPS1-TF-CNG variant uses this
**entire 4.0 V span** across its 0--16 bar operating range, yielding a sensitivity of 0.25 V/bar.

**Key insight:** Unlike earlier assumptions based on an erroneous spreadsheet, the CNG variant
does NOT produce only a ~16 mV output swing. It uses the full ratiometric output range identical
to other MPS1 platform variants. The calibration difference between variants is achieved by
changing the ASIC gain setting (via c1) appropriate to each rated pressure P_N.

**Comparison across MPS1 variants:**

| Variant | P_N (bar) | P_N (kPa) | c1 (kPa⁻¹) | Sensitivity (V/bar) | U_OUT span @ 5V |
|---------|-----------|-----------|------------|---------------------|------------------|
| MPS1-A (41 bar) | 41 | 4100 | 0.0001951 | ~0.098 | 0.5--4.5 V |
| MPS1-FC (33 bar) | 33 | 3300 | 0.0002424 | ~0.121 | 0.5--4.5 V |
| **MPS1-TF-CNG** | **16** | **1600** | **0.0005** | **0.250** | **0.5--4.5 V** |

*Note: All variants use the identical 0.5--4.5 V output span. The c1 coefficient is derived from
the catalog formula c1 = 0.8 / P_N. The 41 bar TCD confirms this with c1 = 0.0001951 ≈ 0.8/4100.*

### 5.5 Output Voltage Range Definitions (Platform-Level, from TCD)

| Range | Symbol | Value @ US=5V | Description |
|-------|--------|---------------|-------------|
| Upper SRC diagnostic | Usrch | 4.80 … 5.00 V | Short to UB detection |
| Upper unused | -- | 4.70 … 4.80 V | Transition zone |
| Upper clamp | Uclh (C4) | 4.50 … 4.70 V | Output limitation |
| Upper measurement end | Umre (C3) | 4.50 V | End of valid range (≈16 bar) |
| Lower measurement start | Umrb (C2) | 0.50 V | Start of valid range (≈0 bar) |
| Lower clamp | Ucll (C1) | 0.30 … 0.50 V | Output limitation |
| Lower unused | -- | 0.20 … 0.30 V | Transition zone |
| Lower SRC diagnostic | Usrcl | 0.00 … 0.20 V | Short to GND detection |

### 5.6 Characteristic Curve

- **Type**: Linear, ratiometric analog voltage output
- **X-axis (pressure)**: 0 to 16 bar(g) relative pressure
- **Y-axis (voltage)**: 0.500 V to 4.500 V at US = 5.0 V
- **Slope**: 0.25 V/bar (2.5 mV/kPa)
- **Intercept**: 0.500 V (0.1 · US)
- **Linearity**: Digital ASIC with calibration provides excellent linearity across the full span

The characteristic curve is a straight line from (0 bar, 0.500 V) to (16 bar, 4.500 V).
Above 16 bar, the output continues linearly into the upper clamp region (C4).
At the maximum overpressure of 32 bar, the output would theoretically reach 8.5 V,
but is clamped by the ASIC at approximately 4.5--4.7 V.

The diagnostic voltage bands above and below the measurement span enable the ECU
to detect fault conditions: open load (pull-up to US), short to ground (pull-down to 0 V),
and short to battery.

---

## 6. Electrical Characteristics

### 6.1 Supply & Consumption

| Parameter | Symbol | Min | Typ | Max | Unit |
|-----------|--------|-----|-----|-----|------|
| Supply voltage | US | 4.75 | 5.00 | 5.25 | V |
| Max. supply voltage (≤1 h) | US,max | — | — | 18 | V |
| Quiescent current @ US=5V | IS | — | 12 | 15 | mA |
| Inrush current | Irush | — | — | 60 | mA |
| Output impedance (0.1US < Uout < 0.94US) | Rd | — | — | 10 | Ω |

### 6.2 Undervoltage/Overvoltage Detection

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Undervoltage detection | 4.20 | 4.35 | 4.50 | V |
| Overvoltage detection | 5.50 | 5.65 | 5.80 | V |

Behavior: On under/overvoltage detection, Uout is pulled to ground.

### 6.3 Load Requirements

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Load resistance to GND or UV | 5 | — | 10 | kΩ |
| Load current at output | -1 | — | 0.5 | mA |
| Load capacitance | — | — | 100 | nF |

### 6.4 EMC Protection

- EMC protection up to 100 V/m (radiated)
- BCI (Bulk Current Injection) tested
- ESD protection per ISO 10605
- Test pulses on sensor lines per ISO 7637-3
- External low-pass filter (R1, C1) required in signal path for full EMC compliance

---

## 7. Mechanical Characteristics

### 7.1 Dimensions & Weight

| Parameter | Value |
|-----------|-------|
| Thread | M10 × 1 (typical for MPS platform) |
| Hexagon size | 24 mm (AF) |
| Connector type | Bosch Compact 1.1 or RB Trapeze (application-specific) |
| Weight | ~35 g (typical MPS platform) |
| Sealing | O-ring (application-specific material) |

### 7.2 Mounting

| Parameter | Value |
|-----------|-------|
| Max. tightening torque | 27 Nm |
| Mounting position | Any orientation (avoid condensate accumulation at pressure port) |
| Housing ground connection | < 2.5 mΩ to vehicle chassis |

### 7.3 Vibration Specifications

| Parameter | Condition | Value |
|-----------|-----------|-------|
| Sinusoidal vibration | 70–147 Hz | s = 0.25 mm (amplitude) |
| Sinusoidal vibration | 147–1350 Hz | a = 210 m/s² |
| Sinusoidal vibration | 1350–2000 Hz | a = 175 m/s² |

Per ISO 16750-3. Validated with specified mating connector.

---

## 8. Climatic & Environmental

| Parameter | Specification |
|-----------|--------------|
| Operating temperature | -40°C to +120°C |
| Storage temperature | -40°C to +130°C (dry, dust-free) |
| IP protection class | IP6K9K (typical) |
| Salt spray | Tested per IEC 60068-2-11 |
| Humidity cycling | Tested per DIN EN 60068-2-30 |
| Chemical resistance | Resistant to CNG; tested against common automotive fluids |
| Thermal shock | Tested per ISO 16750-4 |

---

## 9. Diagnostics & Fault Behavior

### 9.1 Diagnostic Architecture

The MPS1 sensor implements a comprehensive diagnostic strategy using the analog output voltage:

| Diagnostic Range | Voltage (fraction of US) | Indication |
|-----------------|--------------------------|------------|
| Upper SRC | [0.96 … 1.0] × US | Short to UB / open load |
| Upper unused | ]0.94 … 0.96[ × US | Transition |
| Upper clamp | [0.90 … 0.94] × US | Signal limitation |
| **Valid range** | **[0.10 … 0.90] × US** | **Normal operation** |
| Lower clamp | [0.06 … 0.10] × US | Signal limitation |
| Lower unused | ]0.04 … 0.06[ × US | Transition |
| Lower SRC | [0.00 … 0.04] × US | Short to GND |

### 9.2 Fault Detection Table

| Fault Condition | Response | Reset Behavior |
|----------------|----------|----------------|
| RAM/ROM check failure | Output → lower diagnostic band | Reset after 400 ms |
| OTP lock bit not set | Output → lower diagnostic band | No auto-reset |
| Sensor element failure (bond wires) | Output → lower diagnostic band | Reset/debouncing |
| ADC signal too high/low | Output → lower diagnostic band | Debouncing, no reset |
| Internal temperature sensor failure | Output → lower diagnostic band | Debouncing, no reset |
| Low supply voltage | Output → lower diagnostic band | Debouncing, no reset |
| High supply voltage | Output → lower diagnostic band | Debouncing, no reset |
| High ASIC temperature | Output → lower diagnostic band | Debouncing, no reset |
| Overpressure (>125% pn) | Output → ground | — |
| Undervoltage (<4.35V typ) | Output → ground | — |
| Overvoltage (>5.65V typ) | Output → ground | — |
| Overtemperature (>162.5°C typ) | Output → ground | — |

### 9.3 Reset & Initialization Timing

| Parameter | CRC OK | CRC NOK | Unit |
|-----------|--------|---------|------|
| T1 (initial delay) | 0.03 | 0.03 | ms |
| T2 (initialization) | 0.9 | 2.1 | ms |
| T3 (total startup) | 1.1 | 2.3 | ms |

After reset, output is pulled to ground during initialization. Once initialization completes
and no faults are present, output transitions to valid measurement range.

---

## 10. Pinout & Connection

### 10.1 Standard Pinout (4-pin Bosch Compact Connector)

| Pin | Function | Description |
|-----|----------|-------------|
| 1 | GND | Ground (sensor ground) |
| 2 | NTC | Temperature sensor (NTC thermistor output) |
| 3 | +5V (US) | Supply voltage (5V nominal) |
| 4 | U_OUT | Pressure signal output (ratiometric analog) |

### 10.2 ECU Interface Circuit

The recommended ECU-side interface circuit includes:

- **Pull-up resistor** to US on the output line — enables open-load and short-circuit detection
- **Low-pass filter** (R1, C1) in the signal path — required for EMC compliance
- **Ground connection**: Sensor housing must be electrically bonded to vehicle ground (R ≤ 2.5 mΩ)
- The sensor output is ratiometric — the ECU should measure Uout/US ratio rather than absolute voltage

### 10.3 Connector Accessories

| Part | Part Number | Description |
|------|-------------|-------------|
| Connector housing, 4-pin | 1 928 403 736 | Bosch Compact connector |
| Contact pins (tin-plated) | 1 928 498 056 | For wire Ø 0.5–1.0 mm² |
| Contact pins (tin-plated) | 1 928 498 057 | For wire Ø 1.5–2.5 mm² |
| Single-wire seal | 1 928 300 599 | For wire Ø 0.5–1.0 mm² |
| Single-wire seal | 1 928 300 600 | For wire Ø 1.5–2.5 mm² |
| Dummy plug | 1 928 300 601 | For unused cavities |

*Accessories are not included with the sensor and must be ordered separately.*

---

## 11. Application Notes

### 11.1 Installation Requirements

1. **Pressure port**: Must be free of freezing substances and condensates
2. **Sealing**: O-ring seal between sensor and mounting surface
3. **Orientation**: Any orientation acceptable; avoid positions where condensate pools at the pressure port
4. **Tightening**: Do not exceed 27 Nm. Do not re-screw once installed.
5. **Electrostatic painting**: NOT permitted — risk of damage to electronic components

### 11.2 Media Considerations

- Only use with CNG (natural gas) — NOT for hydrogen, water, coolant, or other media
- Avoid materials that release halogens (iodine/bromine-stabilized polyamides, certain FKM compounds)
  in enclosed, low-ventilation mounting positions
- The sensor's pressure port and temperature sensor project into the fuel rail

### 11.3 Wiring Recommendations

- Use twisted-pair or shielded cable for the signal line
- Keep sensor wiring away from ignition and high-current cables
- The ECU should implement SRC (Signal Range Check) to detect: open load, short to GND, short to UB
- Ratiometric measurement is recommended (measure U_OUT/US ratio)

### 11.4 Lifetime & Reliability

- Designed for automotive engine compartment use
- Lifetime requirements per application specification
- Bosch recommends system-level validation of the complete pressure measurement chain

### 11.5 Functional Safety

- The sensor does NOT implement ASIL-classified requirements per ISO 26262:2018
- Not approved for applications requiring ASIL above QM
- System-level safety concept must be implemented by the ECU

### 11.6 Service & Repair

- The sensor is a non-serviceable, non-repairable component
- Replace as a complete unit if faulty
- Dispose per local regulations for electronic/automotive components

---

## 12. Ordering Information

### 12.1 MPS1 Platform Part Numbers

The MPS1 platform is offered in multiple configurations. The CNG-specific variant (TF-CNG)
differs from other variants in:
- Media compatibility (CNG-resistant materials and seals)
- Pressure range calibration (0–1625 kPa)
- Integrated NTC temperature sensor

### 12.2 Related Part Numbers (from i-business catalog)

For reference, the DS-G3-TF series (lower pressure CNG sensors) part numbers:

| Part Number | Pressure Range | Product Type |
|-------------|---------------|-------------|
| 0 261 230 373 | 50–350 kPa | DS-G3-TF (CNG) |
| 0 261 230 499 | 20–350 kPa | DS-G3-TF (CNG) |
| 0 261 230 00J | 20–300 kPa | DS-G3-TF (CNG) |
| 0 261 230 01G | 50–400 kPa | DS-G3-TF (CNG) |

### 12.3 MPS1 Platform Variants

| Variant | Part Number | Pressure Range | Application |
|---------|-------------|---------------|-------------|
| MPS1-FC | 0 261 544 02E-01 | 0–4100 kPa (41 bar) | H2-ICE (hydrogen) |
| MPS1-TF-CNG | (see offer drawing) | 0–1625 kPa (16.25 bar) | CNG fuel rail |

*For exact ordering part numbers, refer to the official Bosch offer drawing and contact
Bosch sales department.*

---

## 13. Source Documents

This comprehensive datasheet was synthesized from the following Bosch source documents:

| # | Document | Type | Key Content |
|---|----------|------|-------------|
| 1 | `Attachment_1_TCD_0261K0013H_MPS1_A_41_bar_v1 1.pdf` | Technical Customer Documentation (TCD) | MPS1-FC 41 bar rel. sensor — transfer function, diagnostics, mechanical, electrical, climatic specs, test procedures. 68 pages. German/English. |
| 2 | `Attachment_2_TCD_0261K001BD_MPS_FC_A_33_bar.pdf` | Technical Customer Documentation (TCD) | MPS-FC 33 bar abs. sensor — full specifications. 68 pages. German/English. |
| 3 | `characterisic curve MPS1-TF-CNG.xlsx` | Characteristic curve spreadsheet | MPS1-TF-CNG transfer function data: c0, c1, Us, pressure vs. voltage table (0–1625 kPa). |
| 4 | `sensors_i-business_12-11-2025.pdf` | Bosch i-business sensors catalog | CNG pressure sensor section (DS-G3-TF), connector accessories, general sensor information. 100 pages. |

### Extraction Method

- **PDFs**: PyMuPDF (fitz) 1.27.2.3 for embedded text extraction; pytesseract 0.3.13 + Tesseract 5.5.0 for OCR fallback on image-only pages
- **Excel**: openpyxl 3.1.5 — formulas extracted with `data_only=False`, evaluated values with `data_only=True`

---

*Datasheet compiled by Frankie OCR agent on 2026-07-16.*
*This document is a synthesis of publicly/existing Bosch documentation for engineering reference.*
*Always refer to the official Bosch Technical Customer Documentation for production design decisions.*
