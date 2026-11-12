# VentCon 3: PCBWay order package

Generated 2026-11-11 from
`HardwareSourcing/orderingDocs/VentCon3_Manufacturing_Output.zip` and
`HardwareSourcing/orderingDocs/Ventcon 3_BOM.csv`.

Board: VentCon 3.0 (Ventrex Automotive GmbH, 2026), 4 layers, 55.88 x 90.15 mm.

## Files

| File | Use |
|---|---|
| `VentCon3_PCBWay_Gerber.zip` | PCB fabrication upload (11 Gerber + 2 drill files + job file) |
| `VentCon3_BOM_PCBWay.csv` | Assembly BOM, PCBWay turnkey format, 22 lines, 33 parts |
| `VentCon3_Centroid_BothSides.csv` | Placement data both sides in one file, 28 SMT parts, for the single centroid slot |
| `VentCon3_Centroid_Top.csv` | Placement data top side, 13 SMT parts |
| `VentCon3_Centroid_Bottom.csv` | Placement data bottom side, 15 SMT parts |
| `VentCon3_OrderNotes.txt` | Text to paste into the PCBWay order remarks |
| `gerber/` | Unzipped source of `VentCon3_PCBWay_Gerber.zip` |

## Order settings, PCB

| Item | Value |
|---|---|
| Layers | 4 |
| Size | 55.88 x 90.15 mm |
| Material | FR-4, TG150 |
| Thickness | **2.8 mm** |
| Copper weight | 1 oz |
| Surface finish | HASL lead-free (ENIG optional) |
| Solder mask | green |
| Silkscreen | white |
| Min drill | 0.35 mm (14 vias) |
| Min trace | 0.254 mm |
| Min annular ring | 0.203 mm |
| Quantity | 15 |

Notes:
- 0.35 mm vias in a 2.8 mm board give an 8:1 aspect ratio. Confirm with PCBWay
  in the order remarks that this is inside their standard process.
- Thickness tolerance is +/-10 % at 2.8 mm, so expect 2.52 to 3.08 mm.
- Before ordering, check the pin length of the Phoenix SDC connector
  `1864095` (J1, hand-soldered): its pins must protrude through 2.8 mm.

## Order settings, assembly

| Item | Value |
|---|---|
| Service | Turnkey, both sides |
| Quantity | 15 |
| SMT | 28 parts, 66 pads/joints (top 13 parts / 36 pads, bottom 15 parts / 30 pads) |
| THT | 5 parts, 13 joints: C4, C10 (radial), IC1, IC2, Q1 (TO-220) |
| Stencils | provided by PCBWay with the assembly service |
| Files | Gerber zip + BOM + centroid top + centroid bottom |

The BOM marks the 5 THT parts with `Package Type = THT`.
Per 15 boards: 420 SMT placements (990 SMT joints) and 195 THT joints.
If you want SMT only, delete BOM lines 4, 11, 12 and 13 before uploading and
hand-solder those five parts yourself. The centroid files already contain SMT
parts only.

## Verified design data

| Item | Value |
|---|---|
| Outline | 1 closed contour, 55.88 x 90.15 mm, chamfered corners, open notch at the top edge |
| Layers | L1 signal, L2 plane (114 polygons), L3 signal, L4 signal |
| Drill hits | 100 total: 86 plated + 14 non-plated |
| NPTH | 2 x 3.30, 4 x 2.05, 4 x 1.55, 4 x 1.651 mm |
| SMT pads | 36 top + 30 bottom |
| Paste layers | complete for all 28 SMT parts |
| Copper outside outline | none |
| Artwork inside top notch | none |
| Solder mask expansion | 0.1 mm |

## Changes against the original Fusion/EAGLE export

1. `profile.gbr` contained the board outline **plus 14 circular contours** with
   exactly the NPTH drill diameters. They were removed to
   `VentCon3-Edge_Cuts.gbr`; the holes remain in the NPTH drill file.
2. `drill_1_64.xln` was split into `VentCon3-PTH.drl` (86 holes) and
   `VentCon3-NPTH.drl` (14 holes, marked `;TYPE=NON_PLATED`).
3. The centroid files lost the 5 DNP parts (J1, U1, S1, S2, S3), the two
   stand-off pads H1/H2, and the 5 THT parts. They gained a header row
   `Ref,Val,Package,PosX,PosY,Rot,Side` (mm) and BOM-consistent values.
4. The BOM lost the DNP lines, gained `Package Type` (SMT/THT), `Side` and
   `Note` columns, and is comma separated with quoted fields.
   All non-ASCII characters were transliterated (`+/-`, `u`, `dia`, `deg`, `Ohm`).
5. Layer files were renamed to the usual `VentCon3-<layer>.gbr` scheme.
6. `VentCon3-job.gbrjob` now says 2.8 mm instead of 2.6908 mm, so the
   PCBWay quick quote prefills the ordered thickness.

## Still open

1. Four parts have no MPN: C2 (47 nF), C12 (22 uF), F1 (4 A), F2 (100 mA).
   The BOM says `TBD` and carries the full specification; substitution is
   allowed. Add your preferred MPN if you have one.
2. Decide whether PCBWay hand-solders the 5 THT parts.
3. U2 (VSSOP-10, 0.5 mm pitch) has mask openings of 0.5032 mm on a 0.5 mm pitch,
   so the mask dam is zero. Ask PCBWay to reduce the mask expansion and keep a
   dam of at least 0.10 mm.
4. The board has no fiducials. Ask PCBWay to add panel fiducials if their
   assembly line requires them.
5. Silkscreen lines are 0.076 to 0.254 mm wide, most of them 0.127 mm, below
   the 0.15 mm standard. Small text may come out thinner or partly clipped;
   check the PCBWay preview after uploading.

## DNP parts to buy yourself

15 sets each, since the order is 15 boards.

| Ref | Part | Order code |
|---|---|---|
| J1 | Phoenix Contact SDC 8-pin, 1864095 | Mouser 651-1864095 |
| U1 | Arduino Nano ESP32, ABX00083 | Arduino ABX00083 |
| S1, S2, S3 | Tactile switch 6 mm THT, SPST | SparkFun COM-00097 |
