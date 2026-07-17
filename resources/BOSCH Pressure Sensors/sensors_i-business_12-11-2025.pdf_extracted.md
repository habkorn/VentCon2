# Bosch Sensors i-Business Catalog

*Comprehensive catalog of automotive sensors: air-mass, lambda, pressure, rotational-speed, structure-borne sound, and temperature measurement.*

**Source:** `sensors_i-business_12-11-2025.pdf` (100 pages) | **Extraction:** PyMuPDF embedded text | **Formatted:** 2026-07-16

---

## Table of Contents

- [0. General Information](#0-general-information)
- [1. Air-Mass Sensors](#1-air-mass-sensors)
  - [1.1 HFM with Analog Interface](#11-hfm-with-analog-interface)
  - [1.2 HFM with Digital Interface](#12-hfm-with-digital-interface)
  - [1.3 PFM Pressure-Based Flow Meter](#13-pfm-pressure-based-flow-meter)
- [2. Lambda Sensors](#2-lambda-sensors)
  - [2.1 LSF-4.2 (Switching Type)](#21-lsf-42-switching-type)
  - [2.2 LSU-4.9 (Wideband)](#22-lsu-49-wideband)
  - [2.3 LSU-5.2 (Wideband)](#23-lsu-52-wideband)
  - [2.4 NOx Sensors Gen 3](#24-nox-sensors-gen-3)
- [3. Pressure Sensors](#3-pressure-sensors)
  - [3.1 Differential Pressure Sensors](#31-differential-pressure-sensors)
  - [3.2 Absolute Pressure Sensors](#32-absolute-pressure-sensors)
  - [3.3 CNG Pressure Sensors](#33-cng-pressure-sensors)
  - [3.4 Medium Pressure Sensors](#34-medium-pressure-sensors)
  - [3.5 High Pressure Sensors](#35-high-pressure-sensors)
- [4. Rotational-Speed Sensors](#4-rotational-speed-sensors)
- [5. Structure-Borne Sound Sensors](#5-structure-borne-sound-sensors)
- [6. Temperature Sensors](#6-temperature-sensors)
- [7. Part Number Index](#7-part-number-index)

---

## 0. General Information

### IP Protection Codes (DIN 40 050 Part 9)

Valid for electrical equipment of road vehicles.

| 1st Digit | Solid Foreign Object Protection | Person Protection | 2nd Digit | Water Ingress Protection |
|-----------|-------------------------------|-------------------|-----------|-------------------------|
| 0 | Non-protected | Non-protected | 0 | Non-protected |
| 1 | Foreign bodies dia. >= 50 mm | Back of hand | 1 | Vertically dripping water |
| 2 | Foreign bodies dia. >= 12.5 mm | Finger | 2 | Dripping water (15 deg incline) |
| 3 | Foreign bodies dia. >= 2.5 mm | Tool | 3 | Splash water |
| 4 | Foreign bodies dia. >= 1.0 mm | Wire | 4 | Spray water |
| 5K | Dust-protected | Wire | 4K | High-pressure spray water |
| 6K | Dust-proof | Wire | 5 | Jets of water |
| -- | -- | -- | 6 | Powerful jets of water |
| -- | -- | -- | 6K | High-pressure jets of water |
| -- | -- | -- | 7 | Temporary immersion |
| -- | -- | -- | 9 | Continuous immersion |
| -- | -- | -- | 9K | High-pressure/steam-jet cleaners |

**Additional letters (optional):** A = Back of hand, B = Finger, C = Tool, D = Wire, M = Parts in motion during water test, S = Parts stationary during water test, K = Road vehicle equipment

### CE Compliance

Under the EU Directive, all electrically-powered machines, devices and systems manufactured, imported and sold within the EU must carry a CE label. Relevant directives:

1. **Machine Directive:** Valid for self-contained operational machines or interlinked machine systems. Not valid for components (e.g., sensors) with no independent function.
2. **EMC Directive:** Valid for all electrical/electronic devices. Bosch sensors shipped as supplied or replacement parts are not subject to mandatory CE labeling per EMC Act Section 5, Paragraph 5.

### Liability Disclaimer

All products are designed for automotive vehicles in their intended use. If used within specification but outside intended use, the customer is responsible for establishing suitability through testing. Contact: www.bosch-ibusiness.com/contact/

[Back to TOC](#table-of-contents)

---

## 1. Air-Mass Sensors

### 1.1 HFM with Analog Interface

Nominal air-flow up to 1,050 kg/h. Analog interface. Compact, lightweight, fast response, low power, pulsation flow detection.

**Design:** Plug-in sensor in cylinder housing. Electronic module with sensor element in plug-in, extending into bypass metering duct. NTC temperature sensor on backside of connector housing. Thermal flowmeter -- temperature difference across heating zone proportional to mass air flow, evaluated in bridge circuit.

**Application:** Measures intake air mass and temperature in diesel and gasoline engines.

---

#### HFM7-R5 | Part No. 0 280 218 120

*(Successor of 0 280 218 037/116)*

| Feature | Value |
|---------|-------|
| Intake air temperature sensor | Yes |
| Interface | Analog |

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Nominal airflow (m_N) | -- | 480 | -- | kg/h |
| Measuring range (m) | -30 | -- | +640 | kg/h |
| Rated supply voltage (U_N) | -- | 14 | -- | V |
| Supply-voltage range (U_V) | 8 | -- | 17 | V |
| Relative accuracy (delta m/m) [1] | -- | -- | +/-3 | % |
| Temperature range [2] | -40 | -- | +130 | deg C |
| Pressure drop at m_N (delta p) | -- | -- | <15 | hPa |
| Current input (I_V) | -- | -- | <0.1 | A |
| Time constant tau_63 [3] | -- | -- | <=10 | ms |
| Time constant tau_delta [4] | -- | -- | <=30 | ms |

*[1] For 0.04 <= delta m/m_N <= 1.3 | [2] Short-time (<=3 min.) to 130 deg C | [3] Time for 63% response to abrupt 10->310 kg/h change | [4] Delay until relative deviation |delta m/m| <= 5%*

| Accessory | Specification | Part Number |
|-----------|--------------|-------------|
| Compact connector | 5-pin | 1 928 403 738 |
| Contact pins | Wire dia. 0.5--1.0 mm2 | 1 928 498 056 |
| Contact pins | Wire dia. 1.5--2.5 mm2 | 1 928 498 057 |
| Single-wire seals | Wire dia. 0.5--1.0 mm2 | 1 928 300 599 |
| Single-wire seals | Wire dia. 1.5--2.5 mm2 | 1 928 300 600 |
| Dummy plug | -- | 1 928 300 601 |

---

#### HFM7-R5 | Part No. 0 280 218 446

| Feature | Value |
|---------|-------|
| Intake air temperature sensor | No |
| Interface | Analog |

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Nominal airflow (m_N) | -- | 850 | -- | kg/h |
| Measuring range (m) | -50 | -- | +1100 | kg/h |
| Rated supply voltage (U_N) | -- | 14 | -- | V |
| Supply-voltage range (U_V) | 8 | -- | 17 | V |
| Relative accuracy [1] | -- | -- | +/-3 | % |
| Temperature range [2] | -40 | -- | +120 | deg C |
| Pressure drop at m_N | -- | -- | <15 | hPa |
| Current input (I_V) | -- | -- | <0.1 | A |
| Time constant tau_63 [3] | -- | -- | <=15 | ms |
| Time constant tau_delta [4] | -- | -- | <=30 | ms |

| Accessory | Specification | Part Number |
|-----------|--------------|-------------|
| Compact connector | 5-pin | 1 928 403 836 |
| Contact pins | Wire dia. 0.5--1.0 mm2 | 1 928 498 056 |
| Contact pins | Wire dia. 1.5--2.5 mm2 | 1 928 498 057 |
| Single-wire seals | Wire dia. 0.5--1.0 mm2 | 1 928 300 599 |
| Single-wire seals | Wire dia. 1.5--2.5 mm2 | 1 928 300 600 |
| Dummy plug | -- | 1 928 300 601 |

---

#### HFM-7 | Part No. 0 280 218 218

| Feature | Value |
|---------|-------|
| Intake air temperature sensor | Yes |
| Interface | Analog |

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Nominal airflow (m_N) | -- | 1050 | -- | kg/h |
| Measuring range (m) | -90 | -- | +1150 | kg/h |
| Rated supply voltage (U_N) | -- | 14 | -- | V |
| Supply-voltage range (U_V) | 6 | -- | 17 | V |
| Relative accuracy [1] | -- | -- | +/-2 | % |
| Temperature range [2] | -40 | -- | +120 | deg C |
| Pressure drop at m_N | -- | -- | <10 | hPa |
| Current input (I_V) | -- | -- | <0.1 | A |
| Time constant tau_63 [3] | -- | -- | <=10 | ms |
| Time constant tau_delta [4] | -- | -- | <=30 | ms |

| Accessory | Specification | Part Number |
|-----------|--------------|-------------|
| Connector housing | 5-pin | 1 928 405 159 |
| Contact pins | Wire dia. 0.35--0.5 mm2 | 1 928 498 143 |
| Contact pins | Wire dia. 0.75--1.0 mm2 | 1 928 498 144 |
| Single-wire seal | Wire dia. 1.2--1.6 mm2 | 1 928 300 934 |
| Single-wire seal | Wire dia. 1.7--2.1 mm2 | 1 928 300 936 |
| Dummy plug | -- | 1 928 300 935 |

[Back to TOC](#table-of-contents)

---

### 1.2 HFM with Digital Interface

Nominal air-flow up to 2,300 kg/h. Digital interface (frequency/SENT). SENT is a one-way asynchronous voltage interface (3 wires: supply, signal, ground) per SAE J2716 APR 2016. HFM8: Fast Channel 1 = air mass (14-bit), Fast Channel 2 = pressure (10-bit), Slow Channel = temperature, humidity, status (12-bit).

---

| Part Number | Product Type | Interface | Nom. Airflow | Features | Connector |
|-------------|-------------|-----------|-------------|----------|-----------|
| 0 280 218 416 | HFM-7 | FAS | 850 kg/h | +temp sensor | 4-pin 1 928 404 745 |
| 0 281 002 956 | HFM-7 | FAS | 640 kg/h | No temp sensor | -- |
| 0 280 218 429 | HFM-7 SF | FAS | 640 kg/h | +temp sensor | 5-pin 1 928 405 138 |
| 0 280 218 03X | HFM-8-PTH SF | SENT | 640 kg/h | +P+H+temp | Hirschmann 4-pin |
| 0 280 218 04B | HFM-8-T | FAS | 480 kg/h | +temp sensor | Hirschmann 4-pin |
| 0 280 218 07T | HFM-8-PTH | SENT | 480 kg/h | +P+H+temp | Hirschmann 4-pin |
| 0 280 218 07U | HFM-8-PTH | SENT | 640 kg/h | +P+H+temp | Hirschmann 4-pin |
| 0 281 006 597 | HFM-8-T SF | SENT | 640 kg/h | +temp sensor | Hirschmann 4-pin |
| 0 280 218 07K | HFM-8-T SF | FAS | 640 kg/h | +temp sensor | Hirschmann 4-pin |
| 0 281 006 812 | HFM-8-TH SF | SENT | 640 kg/h | +H+temp | Hirschmann 4-pin |
| 0 281 007 740 | HFM-8-T | SENT | 640 kg/h | +temp sensor | Hirschmann 4-pin |

*P = integrated pressure sensor, H = integrated humidity sensor. All HFM-8 use 5V supply, 4.85--5.15V range. All share: accuracy +/-1.5 to 2%, temp range -40 to +130/+140 deg C, current <0.03A, tau_63 <=10 ms, tau_delta <=30 ms.*

[Back to TOC](#table-of-contents)

---

### 1.3 PFM Pressure-Based Flow Meter

**Key Features:** Pressure range up to 4.5 bar, SENT 2-channel digital interface, compact and lightweight, fast response, robust dynamic mass flow measurement.

**Design:** Pitot-static tube concept with two pressure sensors and one temperature sensor. Mounted in measuring tube (nozzle + measurement section + diffusor). Differential pressure sensor in lower housing (taps exposed to flow), absolute pressure sensor in upper housing, temperature sensor directly immersed.

**Application:** Fresh air mass measurement (without EGR) in CV-engine ducts, downstream of charge air intercooler. Outputs: differential pressure (SENT 1), absolute pressure + temperature (SENT 2).

---

#### PFM VarA | Part No. 0 280 218 902

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Differential pressure range | -16 | -- | 2 | kPa |
| Absolute pressure range | 50 | -- | 450 | kPa |
| Supply voltage | 4.85 | 5.0 | 5.15 | V |
| Accuracy (delta m/m) | -- | ~2--4 | -- | % |
| Temperature range | -40 | -- | +130 | deg C |
| Current input (I_DD) | 18 | -- | 45 | mA |
| Time constant, supply settle | -- | -- | <=7 | ms |
| Time constant, 1st valid SENT | -- | -- | <=5 | ms |

| Pin | Function |
|-----|----------|
| 1 | VDD (5V) |
| 2 | SENT 1 -- Differential pressure |
| 3 | GND |
| 4 | SENT 2 -- Absolute pressure + air temperature |

Connector: 4-pin Tyco HDSCS Code A.

---

#### PFM VarB | Part No. 0 280 218 900

Same technical data as VarA. Connector: 4-pin Tyco HDSCS Code B. Same pinout.

[Back to TOC](#table-of-contents)

---

## 2. Lambda Sensors

### 2.1 LSF-4.2 (Switching Type)

Planar ZrO2 galvanic oxygen concentration cell with solid electrolyte. Wafer-shaped sensor element with integrated heater. Compares residual oxygen in exhaust gas with reference atmosphere (air). Sudden voltage jump at lambda = 1; two-state control evaluates jump in 450--500 mV area.

**Typical voltages:** Rich (lambda < 1): 800--1000 mV | Lean (lambda > 1): ~100 mV | Operating temp: ~350 deg C (ceramic) | Heat-up: lambda control within 10 sec of engine start.

**TSP (Thermal Shock Protection)** available on some variants.

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Lambda range | 0.97 | -- | 1.10 | -- |
| Voltage at lambda = 0.97 | -- | 800 +/-55 | -- | mV |
| Voltage at lambda = 1.10 | -- | 50 +/-30 | -- | mV |
| Internal resistance | -- | -- | <=0.5 | kohm |
| Response 600->300 mV | -- | -- | <125 | ms |
| Response 300->600 mV | -- | -- | <60 | ms |
| Heater current | -- | 0.48 +/-0.1 | -- | A |
| Heater power (13V) | -- | 7 | -- | W |
| Heater nominal voltage | -- | 12 | -- | V |
| Exhaust gas temperature | 350 | -- | 930 | deg C |

| Part Number | TSP | Voltage | Cable Length |
|-------------|-----|---------|-------------|
| 0 258 006 956 | No | 12 V | 460 mm |
| 0 258 986 784 | Yes | 12 V | 1135 mm |

[Back to TOC](#table-of-contents)

---

### 2.2 LSU-4.9 (Wideband)

Planar ZrO2 dual-cell limit current sensor (Nernst concentration cell + amperometric pump cell). Constant characteristic curve lambda = 0.65 to air. Requires special operating electronics (ETAS LA4 or IC CJ125). Individual air calibration recommended at each maintenance (20.9% O2). **CNG compatible.**

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Lambda range | 0.65 | -- | infinity | -- |
| Heater power (7.5V) | -- | 7.5 | -- | W |
| Heater nominal voltage | -- | 7.5 | -- | V |
| Exhaust gas temperature | -- | -- | <=930 | deg C |

| Part Number | TSP | Voltage | Cable Length | Notes |
|-------------|-----|---------|-------------|-------|
| 0 281 004 805 | Yes | 12/24 V | 1000 mm | CNG compatible |
| 0 258 017 594 | Yes | 12 V | 450 mm | PWM heater, CNG compatible |
| 0 258 017 025 | No | 12 V | 1000 mm | -- |

| Accessory | Part Number |
|-----------|-------------|
| Mating connector set (housing, contacts, grommet) | 1 987 280 016 |

[Back to TOC](#table-of-contents)

---

### 2.3 LSU-5.2 (Wideband)

Improved LSU-4.9 with 2nd Gen TSP. Higher exhaust gas temp tolerance (980 deg C). Output signal scatters +/-12% at air.

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Lambda range | 0.65 | -- | infinity | -- |
| Heater power (6.8V) | -- | 8.1 | -- | W |
| Heater nominal voltage | -- | 6.8 | -- | V |
| Exhaust gas temperature | -- | -- | <=980 | deg C |

| Part Number | Cable Length |
|-------------|-------------|
| 0 258 037 056 | 1000 mm |

| Accessory | Part Number |
|-----------|-------------|
| Mating connector set | 1 986 280 016 |

[Back to TOC](#table-of-contents)

---

### 2.4 NOx Sensors Gen 3 (12/24 Volt)

**Application:** Diesel, CHP, H2, CNG, marine. SCR + DPF aftertreatment. Long service life up to 250,000 km. J1939 communication interface.

| Parameter | Value |
|-----------|-------|
| Part Number | 0 281 050 074 |
| Voltage | 12/24V |
| Cable length | 1040 mm |
| Light off time | <60 s |
| Response time | <1000 ms at 12 m/s |

**NOx Accuracy:**

| Concentration | Accuracy |
|-------------|----------|
| 0 ppm | +/-5 ppm |
| 10 ppm | +/-7 ppm |
| 100 ppm | +/-7% |
| 500 ppm | +/-8% |
| 1500 ppm | +/-10% |

[Back to TOC](#table-of-contents)

---

## 3. Pressure Sensors

### 3.1 Differential Pressure Sensors

**Application:** DPF load condition measurement or fuel tank vapor pressure. Piezo-resistive element with gel film protection.

| Part Number | Product | Pressure Range (kPa) | Supply (V) | Load Current (mA) | Response (ms) | Op. Temp (deg C) |
|-------------|--------|---------------------|------------|-------------------|---------------|-----------------|
| 0 261 230 161 | DS-T3 | -3.75 to +1.25 | 4.75--5.25 | -1.0 to 0.5 | <=5.0 | 0 to +80 (limit -50 to +90) |
| 0 261 232 0HD | LPS4-2DUO | -3.75 to +1.25 | 4.75--5.25 | 15 to 30 | 1/2 to 4.7/3.4 | -40 to +140 (limit +150) |
| 0 281 006 300 | PS-4-DPF | 0 to +100 | 4.75--5.25 | 10 to 20 | <=2.4 | -40 to +140 (limit +150) |
| 0 261 232 051 | PS-4-GPF | -20 to +80 | 4.5--5.5 | -1.0 to 0.5 | <=2.4 | -40 to +150 |

[Back to TOC](#table-of-contents)

---

### 3.2 Absolute Pressure Sensors

**Application:** Intake-manifold/boost pressure measurement. Versions with integrated NTC temperature sensor. EMC protection >100 V/m. Ratiometric output.

| Pin | Function |
|-----|----------|
| 1 | Ground |
| 2 | NTC (Temperature sensor) |
| 3 | +5 V (Supply voltage) |
| 4 | Output signal (Pressure) |

**Common Specs (all DS-S3-TF):**

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Load current (I_L) | -1 | -- | 0.5 | mA |
| Operating temperature | -40 | -- | +130 | deg C |
| Lower output limit (U_S=5V) | 0.25 | 0.30 | 0.35 | V |
| Upper output limit (U_S=5V) | 4.65 | 4.70 | 4.75 | V |
| Response time (tau_10/90) | -- | -- | 1.0 | ms |

**NTC (where applicable):** Measurement current 1 mA (with 1 kohm series R), Rated R at 20 deg C: 2.5 kohm +/-5%, tau_63 <=10 s.

| Part Number | Pressure Range (kPa) | Connector Housing | Contact Plating |
|-------------|---------------------|-------------------|-----------------|
| 0 261 230 217 | 10--115 | 1 928 403 736 | -- |
| 0 261 230 245 | 10--115 | 1 928 404 745 | Tin |
| 0 261 230 247 | 20--250 | 1 928 404 745 | Gold |
| 0 261 230 280 | 20--300 | 1 928 404 745 | Tin |
| 0 261 230 283 | 20--250 | 1 928 403 736 | Tin |
| 0 261 230 302 | 20--250 | 1 928 404 745 | Gold |
| 0 261 230 310 | 10--115 | 1 928 404 745 | Gold |
| 0 261 230 416 | 10--300 | -- | -- |
| 0 281 006 028 | 20--300 | 1 928 403 736 | Tin |
| 0 281 006 076 | 20--300 | 1 928 403 736 | Tin |
| 0 281 006 102 | 50--400 | 1 928 403 736 | Tin |

**Connector Accessories:**

| Accessory | Specification | Part Number (403 736) | Part Number (404 745) |
|-----------|--------------|----------------------|----------------------|
| Connector housing | 4-pin | 1 928 403 736 | 1 928 404 745 |
| Contact pins (tin) | 0.5--1.0 mm2 | 1 928 498 056 | 1 928 498 056 |
| Contact pins (tin) | 1.5--2.5 mm2 | 1 928 498 057 | 1 928 498 057 |
| Contact pins (gold) | 0.5--1.0 mm2 | -- | 1 928 498 054 |
| Contact pins (gold) | 1.5--2.5 mm2 | -- | 1 928 498 055 |
| Single-wire seal | 0.35--1.0 mm2 | 1 928 300 599 | 1 928 300 599 |
| Single-wire seal | 1.5--2.5 mm2 | 1 928 300 600 | 1 928 300 600 |
| Dummy plug | -- | 1 928 300 601 | 1 928 300 601 |

[Back to TOC](#table-of-contents)

---

### 3.3 CNG Pressure Sensors

**Application:** Absolute pressure and temperature measurement/regulation in CNG fuel rail pipes. All sensors and sensor cells resistant against natural gas (CNG). Ratiometric output signal. EMC protection up to 100 V/m.

**Design:** Piezo-resistive element on silicon chip with ASIC amplification and temperature compensation. NTC temperature sensor. Sealed by O-ring at flat surface mounting. Pressure port and temp sensor project into fuel line.

**Installation:** No freezing substances or condensates at pressure port. Medium purity side only.

---

#### DS-G3-TF Series

**All DS-G3-TF share:**

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Current input (I_V at U_V=5V) | 6 | 9 | 12.5 | mA |
| Load current (I_L at output) | -1 | -- | 0.5 | mA |
| Load resistance to GND or U_V | 5 | -- | 10 | kohm |
| Lower output limit (U_V=5V) | 0.25 | 0.30 | 0.35 | V |
| Upper output limit (U_V=5V) | 4.65 | 4.70 | 4.75 | V |
| Output resistance to GND (U_V open) | 1 | 1.6 | 2 | kohm |
| Output resistance to U_V (GND open) | 1 | 1.6 | 2 | kohm |
| Response time (tau_10/90) | -- | -- | 1 | ms |
| Operating temperature | -40 | -- | +120 | deg C |

| Feature | Value |
|---------|-------|
| Integrated temperature sensor | Yes (NTC) |
| Application/medium | Approved for CNG |
| Connector | 4-pin, housing 1 928 403 736 |

| Pin | Function |
|-----|----------|
| 1 | Ground |
| 2 | NTC (Temperature sensor) |
| 3 | +5 V (Supply voltage) |
| 4 | Output signal (Pressure) |

| Part Number | Pressure Range (kPa) |
|-------------|---------------------|
| 0 261 230 373 | 50--350 |
| 0 261 230 499 | 20--350 |
| 0 261 230 00J | 20--300 |
| 0 261 230 01G | 50--400 |

**Connector Accessories (all DS-G3-TF):**

| Accessory | Specification | Part Number |
|-----------|--------------|-------------|
| Connector housing | 4-pin | 1 928 403 736 |
| Contact pins (tin-plated) | Wire dia. 0.5--1.0 mm2 | 1 928 498 056 |
| Contact pins (tin-plated) | Wire dia. 1.5--2.5 mm2 | 1 928 498 057 |
| Single-wire seal | Wire dia. 0.5--1.0 mm2 | 1 928 300 599 |
| Single-wire seal | Wire dia. 1.5--2.5 mm2 | 1 928 300 600 |
| Dummy plug | -- | 1 928 300 601 |

[Back to TOC](#table-of-contents)

---

### 3.4 Medium Pressure Sensors

**Application:** Rapid, highly precise fuel and oil pressure measurement in all combustion engines (gasoline, diesel, CNG, LPG) and transmissions. Controls pump delivery quantity.

**Design:** Silicon membrane resistance bridge evaluated by ASIC. Optional integrated NTC temperature sensor. Pressure and temperature transmitted as analog or SENT signals. Various hydraulic interfaces and connectors available.

**Characteristic function:** U_A output voltage, U_V/U_S supply voltage, p pressure [MPa], C0 = 0.1, C1 = 0.8 * p / P_N, P_N rated pressure [MPa].

**Installation:** Handle ONLY at hexagon. Socket or ring wrench only -- no open-end wrenches. Do not rotate connector against pressure port. Tighten at hexagon after positioning. Prevent water accumulation on membrane.

---

#### MPS1-TF | Part No. 0 261 544 01F

| Feature | Value |
|---------|-------|
| Integrated temperature sensor | Yes |
| Application | Diesel and gasoline |
| Thread | M10 x 1 |

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Pressure range (p1--p2) | 0 | -- | 10 | bar |
| Max. overpressure | -- | -- | 20 | bar |
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Supply current (I_S) | -- | -- | 16.5 | mA |
| Max. supply voltage (U_S) | -- | -- | 18 | V |
| Load capacitance to ground | -- | -- | 13 | nF |
| Response time (tau_10/90) | -- | -- | 2.0 | ms |
| Operating temperature | -40 | -- | +150 | deg C |

---

#### MPS1-TF-CNG | Part No. 0 261 544 00K

> **KEY SENSOR for VentCon2 project.** See also: `MPS1-TF-CNG_datasheet.md` for comprehensive specifications, transfer function, characteristic curve data, and diagnostic ranges.

| Feature | Value |
|---------|-------|
| Application/medium | CNG (Compressed Natural Gas) |
| Thread | M10 x 1 |

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Pressure range (P_N) | 0.5 | -- | 16 | bar |
| Max. overpressure (p_max) | -- | -- | 32 | bar |
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Max. feed voltage (U_S) | -- | -- | 18 | V |
| Load capacitance to ground | -- | -- | 13 | nF |
| Response time (tau_ini) | -- | -- | 2.0 | ms |
| Temperature range | -40 | -- | +150 | deg C |

**Transfer Function:** U_OUT = (0.8 * p / P_N + 0.1) * U_S  (per catalog Section 3.4)
- C0 = 0.1 (offset coefficient, platform constant)
- C1 = 0.8 * p / P_N (span coefficient, where P_N = 16 bar)
- c1 = 0.8 / 1600 kPa = 0.0005 kPa^-1 (derived gain coefficient)
- U_S = 5.0V (nominal supply)
- Output span: 0.500 V (0 bar) to 4.500 V (16 bar) -- **4.0 V full-scale span, 0.25 V/bar sensitivity**

| Pin | Function |
|-----|----------|
| 1 | GND (Ground) |
| 2 | NTC (Temperature sensor) |
| 3 | +5V (Supply voltage) |
| 4 | U_OUT (Pressure signal, ratiometric) |

| Accessory | Specification | Part Number |
|-----------|--------------|-------------|
| Connector housing | -- | 1 928 405 159 |
| Contact pins (silver-plated) | Wire dia. 0.35--0.5 mm2 | 1 928 498 143 |
| Single-wire seal | -- | 1 928 300 934 |
| Dummy plug | -- | 1 928 300 935 |

---

#### Other MPS1 Variants

| Part Number | Product | Pressure Range (bar) | Max Overpressure (bar) | Application | Temperature Range (deg C) |
|-------------|---------|---------------------|------------------------|-------------|--------------------------|
| 0 261 544 018 | MPS1-D | 0.48--11 | 22 | Diesel, gasoline | -40 to +80 |
| 0 261 544 02G | MPS1-TF | 10 | 20 | Oil, diesel, gasoline | -40 to +150 |
| 0 261 232 02X | MPS1-A | 10 | 20 | Diesel, gasoline | -40 to +80 |

All share: M10 x 1 thread, 5V supply (4.75--5.25V), max feed 18V, 13 nF load capacitance, 2.0 ms response time.

[Back to TOC](#table-of-contents)

---

### 3.5 High Pressure Sensors

**Application:** Fuel rail pressure in direct-injection gasoline and common-rail diesel engines, plus hydraulic applications.

**Design:** Polysilicon metal thin-film strain gauge elements in Wheatstone bridge. Evaluation IC with offset/sensitivity correction and temperature compensation. Self-diagnosis for: bond wire break, signal wire break, supply/ground wire break.

**Key features:** Pressure range 5--340 MPa, ratiometric output, stainless steel media resistance, reverse polarity/overvoltage/short circuit protection.

**Characteristic function:** C0 = 0.1, C1 = 0.8 * p / P_N

---

#### Hydraulic Sensor Program Overview

| Nominal Pressure | Part Number | Interface | Mechanical Interface |
|-----------------|-------------|-----------|---------------------|
| 50 bar | 0 261 546 003 | Analog | G1/4 per ISO 1179-1 w/ form gasket |
| 260 bar | 0 261 545 108 | Analog | M10 x 1 per ISO 14405-1 |
| 300 bar | 0 261 545 115 | Analog | M10 x 1 per ISO 14405-1 |
| 400 bar | 0 261 546 00A | Analog | M14 x 1.5 per ISO 6149-1 w/ O-Ring |
| 500 bar | 0 261 547 012 | Digital (SENT) | ISO 16750.3 |
| 1800 bar | 0 281 007 302 | Analog | M18 x 1.5 per ISO 14405-1 |
| 2000 bar | 0 281 006 372 | Analog | ISO 26262 |

*Program regularly adjusted based on market requirements.*

---

#### HPS4 | Part No. 0 261 545 188 *(CNG, 260 bar)*

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Pressure range (P_N) | -- | 260 (26) | -- | bar (MPa) |
| Max. overpressure | -- | -- | 400 | bar |
| Rupture pressure | -- | -- | 2500 | bar |
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Max. feed voltage (U_S) | -- | -- | 18 | V |
| Supply current (I_S) | -- | -- | 15 | mA |
| Load capacitance to ground | -- | -- | 13 | nF |
| Response time (tau_10/90) | -- | -- | 1.0 | ms |
| Temperature range | -40 | -- | +140 | deg C |
| Thread | -- | M10 x 1 | -- | -- |
| Connector | -- | Compact 1.1 | -- | -- |

---

#### HPS4 | Part No. 0 261 546 00A *(Hydraulic, 400 bar)*

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Pressure range (P_N) | -- | 400 (40) | -- | bar (MPa) |
| Max. overpressure | -- | -- | 840 | bar |
| Rupture pressure | -- | -- | 4500 | bar |
| Supply voltage (U_V) | 4.75 | 5.0 | 5.25 | V |
| Supply current (I_S) | -- | -- | 15 | mA |
| Response time (tau_10/90) | 0.2 | -- | 0.8 | ms |
| Temperature range | -40 | -- | +130 | deg C |
| Thread | -- | M14 x 1.5 | -- | -- |

*Also available in 600 bar. Contact Bosch.*

---

#### RPS4.2 Diesel High Pressure Sensors

| Part Number | Pressure Range | Max Overpressure | Rupture | Max Feed | Temperature | Thread |
|-------------|---------------|-----------------|---------|----------|-------------|--------|
| 0 281 007 302 | 1800 bar (180 MPa) | -- | -- | 15 V | -40 to +130 deg C | M18 x 1.5 |
| 0 281 002 930 | 2000 bar (200 MPa) | 2300 bar | 4000 bar | 16 V | -40 to +130 deg C | M18 x 1.5 |

*0 281 002 930 compatible with biodiesel (RME). Also available in 1500 bar (contact Bosch).*

**3-pin Connector Accessories (Compact 1.1):**

| Accessory | Specification | Part Number (gold) | Part Number (tin) |
|-----------|--------------|--------------------|--------------------|
| Connector housing | 3-pin | 1 928 403 966 | 1 928 403 966 |
| Contact pins | Wire dia. 0.5--1.0 mm2 | 1 928 498 054 | 1 928 498 056 |
| Contact pins | Wire dia. 1.5--2.5 mm2 | 1 928 498 055 | 1 928 498 057 |
| Single-wire seal | Wire dia. 0.35--1.0 mm2 | 1 928 300 599 | 1 928 300 599 |
| Single-wire seal | Wire dia. 1.5--2.5 mm2 | 1 928 300 600 | 1 928 300 600 |
| Dummy plug | -- | 1 928 300 601 | 1 928 300 601 |

[Back to TOC](#table-of-contents)

---

## 4. Rotational-Speed Sensors

### 4.1 Camshaft/Crankshaft Sensors

Non-contact, wear-free Hall-effect measurement. Permanent magnet + Hall IC. Open collector output (3-wire). Rotation direction detection on crankshaft sensors for Start-Stop.

| Part Number | Product | Type | Supply | Max RPM | Air Gap | Key Features |
|-------------|---------|------|--------|---------|---------|-------------|
| 0 261 210 389 | RSC-D5 | Crankshaft, Diff. Hall | 5V | 10,000 | 0.2--1.8 mm | Direction detect, 60-2 wheel |
| 0 232 103 10K | PG-3-8 | Camshaft, Single Hall | 5V | 4,500 | 0.2--1.8 mm | TIM, TPO |
| 0 232 103 10W | PG-3-8 | Camshaft, Single Hall | 5V | 4,500 | 0.2--1.8 mm | TIM, TPO, PPS |
| 0 232 103 502 | CPS-4 | Camshaft, Single Hall | 5/12V | 4,500 | 0.2--1.8 mm | TIM, TPO, 70TL05-86 |
| 0 232 103 506 | CPS-4 | Camshaft, Single Hall | 5/12V | 4,500 | 0.2--1.8 mm | Newest platform, Code 1 |
| 0 232 103 510 | CPS-4 | Camshaft, Single Hall | 5/12V | 4,500 | 0.2--1.8 mm | Newest platform, Code 2 |
| 0 232 103 50C | CPS-4 HA | Camshaft, Diff. Hall | 5V | 5,000 | 0.2--1.8 mm | High Accuracy, 96-SO |
| 0 232 103 50F | CPS-4 HA | Camshaft, Diff. Hall | 5V | 5,000 | 0.2--1.8 mm | High Accuracy, 96-SO |

*TIM = Twist Insensitive Mounting, TPO = True Power On, PPS = Polyphenylene Sulfide*

**Common specs (all camshaft/crankshaft):** Supply current <9--15 mA, Pull-up 1--10 kohm, Output LOW <=0.5V, Output HIGH >=0.9*U_S_OUT, Fall time <=1--4 us, Rise time <=10 us, Temp range -40 to +150/+160 deg C.

### 4.2 Transmission Sensors (TRD)

Two-wire current interface (7 mA/14 mA). Wire break and short detection. Target wheel vibration suppression.

| Part Number | Supply | Max RPM | Air Gap | Output LOW | Output HIGH | Temp Range |
|-------------|--------|---------|---------|------------|-------------|------------|
| 0 261 210 32Y | 5V (4.5--5.5V) | 12,000 (60 teeth) | 0.2--2.0 mm | 7 mA | 14 mA | -40 to +140 deg C |

Connector: 3-pin Hirschmann 872-975-...00, Coding B, Silver plated.

### 4.3 Inductive Speed Sensors

Passive inductive (EMI principle). 4300 turns, 860 ohm +/-10% at 20 deg C, 370 +/-60 mH at 1 kHz. Speed range 20--7,000 min^-1. Air gap 0.3--1.8 mm. IPx9K.

| Part Number | Product | Connections |
|-------------|---------|-------------|
| 0 281 002 214 | DG-6-K | A, B, Shield |
| 0 281 002 629 | DG-6-K | A, B, Shield (275 mm) |
| 0 281 002 315 | DG-6-S | A, B |

[Back to TOC](#table-of-contents)

---

## 5. Structure-Borne Sound Sensors

Piezoelectric vibration sensors. Mass exerts compressive force on annular piezo-ceramic element in vibration rhythm. Self-impedance >1 Mohm. Operating temp -40 to +130 deg C (head up to +150 deg C).

| Part Number | Product | Cable | Frequency Range | Permissible Vibration | Pin Coating |
|-------------|---------|-------|-----------------|----------------------|-------------|
| 0 261 231 196 | KS-4-K | With cable | 0--24 kHz | <=80 g | Gold-plated |
| 0 261 231 218 | KS-4-K | With cable (530 mm) | 0--24 kHz | <=80 g | Gold-plated |
| 0 261 231 173 | KS-4-S | Without cable | 3--22 kHz | <=80 g | Gold-plated |
| 0 261 231 176 | KS-4-S | Without cable | 3--22 kHz | <=50 g | Tin-plated |
| 0 261 231 208 | KS-4-S | Without cable | 0--24 kHz | <=80 g | Tin-plated |
| 0 261 231 300 | KS-4-S | Without cable | 0--24 kHz | <=80 g | Gold-plated |

**2-pin Connector Accessories:** Housing 1 928 403 874 or 1 928 403 137. Contact pins 1 928 498 054/056. Seals 1 928 300 599/600. Dummy plug 1 928 300 601.

[Back to TOC](#table-of-contents)

---

## 6. Temperature Sensors

NTC thermistors (negative temperature coefficient). Semi-conducting heavy metal oxides. For air, coolant, fuel, and oil temperature measurement. Rated resistance: 2.5 kohm at 20 deg C (typical).

---

### TF-L (Air Temperature)

| Part Number | Housing | Thread | Temp Range | tau_63 | Tightening Torque |
|-------------|---------|--------|------------|--------|-------------------|
| 0 280 130 039 | Steel | M12 x 1.5 | -40 to +130 deg C | <=45 s | 15 +/-2 Nm |

Resistance: 8.727--10.067 kohm at -10 deg C, 2.375--2.625 kohm at +20 deg C, 0.296--0.349 kohm at +80 deg C. IP6K9K. Max measurement current 1 mA.

---

### TF-W Series (Oil/Water/Coolant/Fuel)

| Part Number | Housing | Application | Thread | Temp Range | tau_63 | Torque |
|-------------|---------|-------------|--------|------------|--------|--------|
| 0 280 130 026 | Brass | Oil/water | M12 x 1.5 | -40 to +130 deg C | <=15 s | 20 +/-5 Nm |
| 0 280 130 093 | Brass | Coolants, fuel, oil | M12 x 1.5 | -40 to +130 deg C | <=15 s | 20 +/-5 Nm |
| 0 281 002 170 | Brass | Oil/water | M12 x 1.5 | -40 to +150 deg C | <=15 s | 20 +/-5 Nm |
| 0 281 002 209 | Brass | Oil/water | M12 x 1.5 | -40 to +130 deg C | <=15 s | 20 +/-5 Nm |
| 0 281 002 412 | Brass | Oil/water | M14 x 1.5 | -40 to +130 deg C | <=15 s | 20 +/-5 Nm |
| 0 280 130 01F | Aluminum | Oil/water/fuel | M12 x 1.5 | -40 to +140 deg C | <=15 s | 20 +/-5 Nm |

**Common resistance values:** -10 deg C: 8.640--10.149 kohm, +20 deg C: 2.351--2.648 kohm, +80 deg C: 0.313--0.332 kohm. Rated voltage 5 +/-0.15V. IP6K9K. Corrosion tested per DIN EN 60068-2-11.

[Back to TOC](#table-of-contents)

---

## 7. Part Number Index

| Part Number | Section | Product Type |
|-------------|---------|-------------|
| 0 232 103 10K | 4.1 | PG-3-8 Camshaft Sensor |
| 0 232 103 10W | 4.1 | PG-3-8 Camshaft Sensor |
| 0 232 103 502 | 4.1 | CPS-4 Camshaft Sensor |
| 0 232 103 506 | 4.1 | CPS-4 Camshaft Sensor |
| 0 232 103 50C | 4.1 | CPS-4 HA Camshaft Sensor (High Accuracy) |
| 0 232 103 50F | 4.1 | CPS-4 HA Camshaft Sensor (High Accuracy) |
| 0 232 103 510 | 4.1 | CPS-4 Camshaft Sensor |
| 0 258 006 956 | 2.1 | LSF-4.2 Lambda Sensor |
| 0 258 017 025 | 2.2 | LSU-4.9 Wideband Lambda Sensor |
| 0 258 017 594 | 2.2 | LSU-4.9 Wideband Lambda Sensor (TSP, CNG) |
| 0 258 037 056 | 2.3 | LSU-5.2 Wideband Lambda Sensor |
| 0 258 986 784 | 2.1 | LSF-4.2 Lambda Sensor (TSP) |
| 0 261 210 32Y | 4.2 | TRD Transmission Sensor |
| 0 261 210 389 | 4.1 | RSC-D5 Crankshaft Sensor |
| 0 261 230 00J | 3.3 | DS-G3-TF CNG Pressure Sensor (20--300 kPa) |
| 0 261 230 01G | 3.3 | DS-G3-TF CNG Pressure Sensor (50--400 kPa) |
| 0 261 230 161 | 3.1 | DS-T3 Differential Pressure Sensor |
| 0 261 230 217 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 245 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 247 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 280 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 283 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 302 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 310 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 373 | 3.3 | DS-G3-TF CNG Pressure Sensor (50--350 kPa) |
| 0 261 230 416 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 261 230 499 | 3.3 | DS-G3-TF CNG Pressure Sensor (20--350 kPa) |
| 0 261 231 173 | 5 | KS-4-S Vibration Sensor |
| 0 261 231 176 | 5 | KS-4-S Vibration Sensor |
| 0 261 231 196 | 5 | KS-4-K Vibration Sensor |
| 0 261 231 208 | 5 | KS-4-S Vibration Sensor |
| 0 261 231 218 | 5 | KS-4-K Vibration Sensor |
| 0 261 231 300 | 5 | KS-4-S Vibration Sensor |
| 0 261 232 02X | 3.4 | MPS1-A Medium Pressure Sensor |
| 0 261 232 051 | 3.1 | PS-4-GPF Differential Pressure Sensor |
| 0 261 232 0HD | 3.1 | LPS4-2DUO Differential Pressure Sensor |
| **0 261 544 00K** | **3.4** | **MPS1-TF-CNG Medium Pressure Sensor (CNG)** |
| 0 261 544 018 | 3.4 | MPS1-D Medium Pressure Sensor |
| 0 261 544 01F | 3.4 | MPS1-TF Medium Pressure Sensor |
| 0 261 544 02G | 3.4 | MPS1-TF Medium Pressure Sensor |
| 0 261 545 108 | 3.5 | HPS4 High Pressure Sensor (260 bar) |
| 0 261 545 115 | 3.5 | HPS4 High Pressure Sensor (300 bar) |
| 0 261 545 188 | 3.5 | HPS4 High Pressure Sensor (CNG, 260 bar) |
| 0 261 546 003 | 3.5 | High Pressure Sensor (50 bar, hydraulic) |
| 0 261 546 00A | 3.5 | HPS4 High Pressure Sensor (400 bar, hydraulic) |
| 0 261 547 012 | 3.5 | High Pressure Sensor (500 bar, SENT) |
| 0 280 130 01F | 6 | TF-W Alu Temperature Sensor |
| 0 280 130 026 | 6 | TF-W Temperature Sensor |
| 0 280 130 039 | 6 | TF-L Temperature Sensor (Air) |
| 0 280 130 093 | 6 | TF-W Temperature Sensor |
| 0 280 218 03X | 1.2 | HFM-8-PTH SF Air-Mass Sensor |
| 0 280 218 04B | 1.2 | HFM-8-T Air-Mass Sensor |
| 0 280 218 07K | 1.2 | HFM-8-T SF Air-Mass Sensor |
| 0 280 218 07T | 1.2 | HFM-8-PTH Air-Mass Sensor |
| 0 280 218 07U | 1.2 | HFM-8-PTH Air-Mass Sensor |
| 0 280 218 120 | 1.1 | HFM7-R5 Air-Mass Sensor |
| 0 280 218 218 | 1.1 | HFM-7 Air-Mass Sensor |
| 0 280 218 416 | 1.2 | HFM-7 Air-Mass Sensor (Digital) |
| 0 280 218 429 | 1.2 | HFM-7 SF Air-Mass Sensor |
| 0 280 218 446 | 1.1 | HFM7-R5 Air-Mass Sensor |
| 0 280 218 900 | 1.3 | PFM VarB Flow Meter |
| 0 280 218 902 | 1.3 | PFM VarA Flow Meter |
| 0 281 002 170 | 6 | TF-W Temperature Sensor |
| 0 281 002 209 | 6 | TF-W Temperature Sensor |
| 0 281 002 214 | 4.3 | DG-6-K Inductive Speed Sensor |
| 0 281 002 315 | 4.3 | DG-6-S Inductive Speed Sensor |
| 0 281 002 412 | 6 | TF-W Temperature Sensor |
| 0 281 002 629 | 4.3 | DG-6-K Inductive Speed Sensor |
| 0 281 002 930 | 3.5 | RPS4.2 High Pressure Sensor (Diesel, 2000 bar) |
| 0 281 002 956 | 1.2 | HFM-7 Air-Mass Sensor (Digital) |
| 0 281 004 805 | 2.2 | LSU-4.9 Wideband Lambda Sensor (TSP, CNG) |
| 0 281 006 028 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 281 006 076 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 281 006 102 | 3.2 | DS-S3-TF Absolute Pressure Sensor |
| 0 281 006 300 | 3.1 | PS-4-DPF Differential Pressure Sensor |
| 0 281 006 372 | 3.5 | High Pressure Sensor (2000 bar) |
| 0 281 006 597 | 1.2 | HFM-8-T SF Air-Mass Sensor |
| 0 281 006 812 | 1.2 | HFM-8-TH SF Air-Mass Sensor |
| 0 281 007 302 | 3.5 | RPS4.2 High Pressure Sensor (Diesel, 1800 bar) |
| 0 281 007 740 | 1.2 | HFM-8-T Air-Mass Sensor |
| 0 281 050 074 | 2.4 | EGS-NX3 NOx Sensor Gen 3 |

---

*Catalog data extracted from Bosch i-business sensors catalog (2025-11-12 edition, 100 pages). All specifications subject to change. Contact www.bosch-ibusiness.com/contact/ for current information and individual offers. Accessories not included with sensors unless stated otherwise.*
