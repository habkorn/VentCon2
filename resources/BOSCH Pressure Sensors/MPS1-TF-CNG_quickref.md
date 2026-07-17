# MPS1-TF-CNG Quick Reference Card

**Bosch Medium Pressure Sensor for CNG | P/N 0 261 544 00K**

---

## Pinout (4-pin Connector, Housing 1 928 405 159)

```
    +-------+
    | 4 3 2 |  ← wire side view
    |   1   |
    +-------+
```

| Pin | Function | Wire Color (suggested) | Connect To |
|:---:|----------|----------------------|------------|
| 1 | GND | Black | ESP32 GND |
| 2 | NTC | Yellow | *(optional)* ESP32 ADC or leave floating |
| 3 | +5V | Red | ESP32 5V rail |
| 4 | U_OUT | Green | ADS1015 AIN0 (or ESP32 ADC) |

---

## Transfer Function

```
p_bar = 4.0 * U_V - 2.0          (use this)
p_kPa = 400 * U_V - 200
```

| U_OUT (V) | Pressure (bar) | Pressure (kPa) |
|:---:|:---:|:---:|
| 0.50 | 0.0 | 0 |
| 0.75 | 1.0 | 100 |
| 1.00 | 2.0 | 200 |
| 1.50 | 4.0 | 400 |
| 2.00 | 6.0 | 600 |
| 2.50 | 8.0 | 800 |
| 3.00 | 10.0 | 1000 |
| 3.50 | 12.0 | 1200 |
| 4.00 | 14.0 | 1400 |
| 4.50 | 16.0 | 1600 |

**Ratiometric:** For best accuracy, measure Uout/Us ratio and scale by Us_actual/5.0.

---

## VentCon2 Calibration

```
Sensor min voltage:   0.5 V
Sensor max voltage:   4.5 V
Sensor min pressure:  0.0 bar
Sensor max pressure:  16.0 bar
Display max pressure: 16.0 bar
```

*Copy these values into VentCon2 web UI sensor calibration fields.*

---

## Specifications at a Glance

| Parameter | Value |
|-----------|-------|
| Pressure range | 0.5 -- 16 bar(g) |
| Max overpressure | 32 bar |
| Supply voltage | 5.0 V +/- 0.25 V |
| Output span | 0.5 -- 4.5 V (ratiometric) |
| Sensitivity | 0.25 V/bar |
| Response time | <= 2.0 ms |
| Operating temp | -40 to +150 deg C |
| Thread | M10 x 1 |
| Tightening torque | max 27 Nm |
| Media | CNG only |

---

## Connector Parts

| Accessory | Part Number |
|-----------|-------------|
| Sensor | **0 261 544 00K** |
| Connector housing | 1 928 405 159 |
| Contact pins (silver-plated) | 1 928 498 143 |
| Single-wire seal | 1 928 300 934 |
| Dummy plug (unused cavities) | 1 928 300 935 |

*Wire size: 0.35--0.5 mm2. Accessories not included with sensor.*

---

## Fault Detection (Signal Range Check)

| Condition | Output Voltage |
|-----------|:---:|
| Normal operation | 0.50 -- 4.50 V |
| Short to GND | 0.00 -- 0.20 V |
| Short to UB / open load | 4.80 -- 5.00 V |
| Overpressure (>32 bar) | pulled to GND |
| Overtemperature (>162.5 deg C) | pulled to GND |

---

*Full documentation: MPS1-TF-CNG_datasheet.md | Characteristic curve derivation: MPS1-TF-CNG_characteristic_curve.md*
