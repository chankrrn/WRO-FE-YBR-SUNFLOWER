# Electrical Architecture & Sensing — YBR-SUNFLOWER

> **Engineering record · WRO 2026 Future Engineers · Vehicle development version 2.0 (draft)**  
> **Scope:** all electrical hardware, energy distribution, sensing, control interfaces, wiring, verification, and supporting experiments.  
> **Status:** design documentation in progress. The wiring illustration is preliminary; the completed chassis, final harness, runtime measurements, and validation evidence are not yet available.  
> **Maintained by:** YBR-SUNFLOWER · **Last reviewed:** [TODO: actual review date] · **Hardware revision:** [TODO] · **Code revision / Git SHA:** [TODO]

[← Project overview](../README.md)

**What this record is for.** This document is the electrical engineering journal *and* detailed electrical reference for YBR-SUNFLOWER. It is intended to let another team understand the reasons behind our electrical design, identify every connection, repeat our tests, and reproduce the electrical subsystem. It is not a collection of component advertisements or merely a wiring picture.

### Navigate this record

| Section | Topic |
|---|---|
| [A](#a-electrical-design-in-one-page) | Electrical goals, known facts and rubric evidence |
| [B](#b-hardware-register--exact-devices-and-roles) | Identified hardware and component inventory |
| [C](#c-system-level-electrical-architecture) | Power, interfaces and physical integration |
| [D](#d-battery-and-electrical-energy-source) | LiHV pack, cutoff and 12 V input limitation |
| [E](#e-regulators-and-power-rails) | 5 V, 12 V, servo voltage, branch validation |
| [F](#f-power-budget-efficiency-and-endurance) | Measured current, losses and runtime |
| [G](#g-grounds-protection-cabling-and-thermal-effects) | Returns, safety, wire ratings and thermal tests |
| [H](#h-electrical-diagrams--from-illustration-to-verified-schematic) | EasyEDA/KiCad, wiring, review checklist |
| [I](#i-control-interfaces-and-exact-pin-assignments) | I²C, GPIO, motor, servo and button |
| [J](#j-sensor-roles-and-selection-rationale) | Selection logic and WRO field constraints |
| [K](#k-rplidar-s2l--installation-and-characterization) | LiDAR hookup, placement and experiments |
| [L](#l-ov5647-camera--optical-and-electrical-characterization) | Lens, mounting, calibration and illumination |
| [M](#m-motor-encoder-and-steering--electrical-testing) | Actuation and encoder calibration |
| [N](#n-noise-interference-latency-and-subsystem-interaction) | Coupled electrical/sensing effects |
| [O](#o-reliability-protection-and-electrical-risks) | Failure modes and acceptance testing |
| [P](#p-engineering-decisions-and-revisions) | YBR decision records and iteration log |
| [Q](#q-matlab--simulink-evidence-program--planned-not-yet-produced) | Proposed measurements, graphs and models |
| [R](#r-reproduction-and-inspection-checklist) | Electrical sign-off criteria |
| [S](#s-open-questions-and-evidence-collection-queue) | Prioritized missing information |
| [T](#t-sources-and-engineering-references) | Rules, datasheets and methodological references |

**Evidence convention used throughout**

| Tag | Meaning | Example of acceptable source |
|---|---|---|
| **Confirmed by team** | A model, component, or fact directly identified by our team | Component photo or inventory |
| **Manufacturer specification** | A value from a manufacturer datasheet, under specified conditions | Manufacturer link and section |
| **Design intention** | What the new robot is intended to use or do | Drawing / design review |
| **Calculated / simulated** | An estimate from explicit assumptions or a model, **not a measured result** | Equations, MATLAB script/model, input revision |
| **Measured** | A real measurement with equipment, setup, conditions, and raw data | Test ID, CSV, instrument details |
| **[TODO]** | Missing evidence, an unresolved design choice, or an artifact not yet created | Owner, target date, acceptance criterion |

Never change a **design intention** into a **measured fact** without a test record. Every completed figure should point to its raw data and, where relevant, MATLAB/Simulink code and parameter values. Figures planned below have **not** been generated.

---

## A. Electrical design in one page

### A.1 Design objective

Our electronics must supply a Raspberry Pi 5, motor-control hardware, steering, camera, LiDAR, and a single-button start interface from a lightweight removable battery. The design uses commercially available modules and connectors instead of a custom-made PCB. The engineering work is therefore concentrated on choosing and integrating modules correctly: supply rails, signal levels, connector polarity, power stability, sensor placement, and failure prevention.

**[TODO] Explain the team's original requirements** in our own words: target operating time, mass/size limit, acceptable downtime, budget, access to replacement parts, and why a modular architecture was preferable to designing a custom PCB.

### A.2 What is known now

- **Confirmed by team:** Raspberry Pi 5 (8 GB); DFR0592 motor-driver HAT; DFR0604 I/O-expansion HAT; DFR0785 white LED button; DFR1023 RPLIDAR S2L; OV5647 5 MP fisheye camera with lens advertised as **222° field of view** (not yet measured); CHP-20GP-180 geared motor and matching encoder; SURPASS Hobby S0017M steering servo; DFR1202 5 V converter; DFR1208 12 V converter; GNB 4S 15.2 V 1300 mAh 120C LiHV battery; switch, voltmeter, and listed connectors.
- **Confirmed by team:** the DFR0592/DFR0604 I²C conflict has been **resolved on the team's setup**. The final addresses and method are **[TODO]**, so reproducibility is not yet established.
- **Design intention:** one battery feeding nominal 5 V and 12 V branches. A preliminary illustrated wiring layout exists, but it is **not** an electrically verified schematic.
- **Development status:** new 3D-printed chassis and Raspberry Pi parking implementation are under development; no new-car performance or reliability result is claimed here.

### A.3 How this record maps to the WRO 2026 criterion

The WRO 2026 *Appendix C.3.2, Criterion 2 — Power and Sensor Architecture* describes an advanced submission as one that includes **power-budget reasoning; justified sensor selection and placement relative to field geometry; calibration; failure-point consideration; and evidence of iteration improving reliability**. A wiring diagram and a parts list alone cover only part of that requirement.

| Rubric evidence sought | Where YBR will place the evidence | Present state |
|---|---|---|
| Electrical and power architecture | Sections C–E, architecture and supply diagrams | [TODO] Draft topology needs verified nets |
| Current draw and regulator sizing | Sections F–G, measured power budget | [TODO] |
| Wiring schematic and reproducible pin map | Sections H–I, `schemes/` | [TODO] |
| Sensor choice and physical placement | Sections J–L | [TODO] Placement pending chassis |
| Camera, LiDAR, encoder, servo calibration | Sections K–M | [TODO] |
| Noise, interference, shadow / blind-zone tests | Section N | [TODO] |
| Failure modes and mitigation | Section O | Draft risk hypotheses, not passed tests |
| Iteration and engineering decisions | Section P | [TODO] Add actual decisions and evidence |
| Source data, MATLAB figures, Simulink models | Section Q | All planned, none produced |
| Independent reproducibility | Section R | [TODO] |

---

## B. Hardware register — exact devices and roles

This is the **component master record**, not a shopping-list substitute. Quantities are assumed to be one per pictured item unless explicitly confirmed otherwise. **[TODO]** Verify every quantity, board revision, connector variant, serial/part marking and installed location.

| ID | Component / model | Proposed function | Known interface / supply | Source-of-truth still needed |
|---|---|---|---|---|
| E01 | Raspberry Pi 5, 8 GB | Central computation, sensor processing, navigation and actuator commands | 5 V USB-C; CSI; GPIO/I²C/USB | OS/boot image, actual power negotiation and current |
| E02 | DFRobot **DFR0592**, DC Motor Driver HAT V1.0 | Motor power stage and encoder acquisition | I²C logic; external motor input 7–12 V according to DFRobot | Actual motor port, I²C address/bus, supply tests |
| E03 | DFRobot **DFR0604**, I/O Expansion HAT | PWM output and digital peripherals | I²C interface; 5 V logic; external servo supply option | Which PWM port, VP jumper/supply, bus/address |
| E04 | DFRobot **DFR0785-W**, LED Button (white) | One program-start button and visible indication | Digital output; manufacturer operating range 3.3–5 V | Exact Pi/DFR0604 signal pin and boot/start logic |
| E05 | SURPASS Hobby **S0017M**, metal gear | Front steering actuation | Servo power nominal 4.8–6 V per product information | Actual feed, signal pulse settings, peak current |
| E06 | **CHP-20GP-180** DC gearmotor | Rear drivetrain propulsion | DC motor via DFR0592 | Motor rated voltage, stall current, actual ratio |
| E07 | **CHP-20GP-180 encoder** | Rotational feedback | Quadrature A/B [TODO verify] | Voltage, counts/revolution definition, polarity |
| E08 | **OV5647 5 MP fisheye camera**, stated 222° lens | Colour/pillar perception | Raspberry Pi CSI interface [TODO verify carrier/cable] | Lens datasheet, FOV definition, cable orientation |
| E09 | DFRobot **DFR1023 / RPLIDAR S2L** | 2D distance scanning / field geometry | LiDAR native UART; supplied USB adapter option | Actual connection: native 5-pin cable or adapter |
| E10 | DFRobot **DFR1202**, 6–24 V → 5 V, max 16 A | Nominal 5 V rail | VIN/GND/VOUT | Actual regulation, efficiency, wire drop, cooling |
| E11 | DFRobot **DFR1208**, 15–24 V → 12 V, max 12 A | Nominal 12 V rail | VIN/GND/VOUT | Battery low-voltage behavior **critical** |
| E12 | GNB LiHV **4S 15.2 V, 1300 mAh, 120C**, XT60 | Energy storage | Main DC battery input | Cell chemistry limits, per-cell test record |
| E13 | Black rocker switch | Main electrical power switch | Mechanical power circuit | Current/DC rating, pole arrangement, switch position |
| E14 | D1-2 connector, 1 input → 3 outputs | Power distribution junction | As wired | Exact part rating, number of poles, topology |
| E15 | PCT-214, 4-in connector | Power / ground junction | As wired | Exact role and current rating |
| E16 | OLED DC voltmeter, 0–30 V, 0.5-inch | Operator-visible voltage check | Two-wire or three-wire [TODO] | Measurement point, accuracy, calibration |
| E17 | USB-C to 2-pin power lead | 5 V regulator → Pi power port | USB-C VBUS/GND [TODO wiring] | Cable resistance, polarity, 5 A recognition behavior |
| E18 | XH2.54 5-pin male connector | LiDAR cable interface | Pinout [TODO] | Mating connector, pin order and signal voltage |
| E19 | Heatsink / fan assembly visible in diagram | Raspberry Pi thermal management | Power source [TODO] | Fan current, GPIO/cooling method |
| E20 | Camera ribbon / LiDAR USB bridge / intermediate connectors | Data cabling | To be confirmed | Exact length, orientation, wire map and count |

**[TODO] Hardware photo board:** one legible photo per electronic item showing model, labels and connectors; annotated orientation; manufacturer URL; quantity, mass, cost, procurement source, and revision. Avoid copying manufacturer specifications into the “measured” column.

**[TODO] Bill of materials:** `part ID | vendor/model | quantity | purchased cost/currency | mass | rated supply | maximum current | physical location | source URL | replacement part | verification date`.

**Primary manufacturer references for the listed modules:** see Section T. Some source pages may contain inconsistent wording about ratings; record the exact page/version consulted and validate critical limits using the board/datasheet.

---

## C. System-level electrical architecture

### C.1 Power and signal domains

The architecture is partitioned conceptually into:

1. **Energy input:** LiHV battery, connector, power switch and distribution nodes.
2. **5 V regulated domain:** Raspberry Pi 5 and any peripherals actually powered from this branch.
3. **12 V regulated domain:** motor driver external-power input and any consumers actually connected to it.
4. **Servo power domain:** **must be positively identified**; do not assume a PWM signal header's supply pin has the same voltage as the 5 V Pi supply.
5. **Data/control domain:** camera CSI, LiDAR UART/USB, I²C HATs, quadrature encoder, servo PWM, LED button and voltmeter sense.
6. **Ground reference:** shared reference return paths, with high-current branches and sensitive signals mapped separately.

**[TODO: Figure E-01 — Electrical system block diagram]** Show all domains and directional arrows labelled `power`, `control`, `sensor data` or `ground`. This should be readable in 15 seconds, unlike a dense point-to-point wiring picture. Proposed asset: `schemes/electrical-architecture.png`; source: `[TODO: editable diagram file]`.

**[TODO: Figure E-02 — Power-distribution tree]** Battery → switch/protection → both converters → individually named loads. Include rated input ranges and measured branch currents when available. The diagram must show the **actual** servo supply and return path, not an assumed one.

**[TODO: Figure E-03 — Signal/interface map]** Pi ↔ HATs; camera; LiDAR; encoder; motor; steering; start button. Include `interface type`, bus/device address, direction and voltage level. No generic unnamed lines.

### C.2 Physical arrangement and wire routing

**[TODO]** Annotate the final chassis with electronic placement, vertical stacking, battery position, cable lengths, wire fixation, LiDAR line of sight, camera sightlines, cooling airflow, and access to XT60 and power switch. Explain why the locations were selected.

**[TODO: Figure E-04 — 3D mounting/clearance overlay]** Create only once the new chassis CAD is ready. Show component dimensions, cabling bends and mounting clearances. Cross-reference `journal/mech.md` rather than duplicate the full chassis chapter.

### C.3 Electrical constraints from the competition

- International vehicle dimensions may not exceed **300 × 200 × 300 mm** (2026 rules §9.17). The new sensor assembly and cabling count toward the finished envelope.
- Start procedure requires an **off state → power on → waiting state → single start-button press** (§§9.6–9.14). A rocker power switch and one LED start button are conceptually compatible, but behavior must be validated on the real system.
- Physical adjustments must not serve as hidden data entry or sensor calibration at the round start (§9.9). Distinguish internal, autonomous sensor initialization from prohibited manual parameter entry; verify the latest official Q&A.
- WRO explicitly expects independent work and technically reproducible documentation. Using off-the-shelf HATs is an engineering choice; designing a custom PCB is **not** a documented prerequisite for the 2026 Electrical rubric.

**[TODO] Track any new international surprise rule or Q&A clarification affecting sensors, startup or electronics.**

---

## D. Battery and electrical energy source

### D.1 Battery specification and reason for selection

**Team-confirmed label:** GAONENG GNB, LiHV 4S, nominal **15.2 V**, capacity **1300 mAh**, marked **120C**, XT60. This is a **label specification**, not an experimentally measured capacity, safe current, or runtime.

Discuss the engineering criteria: battery mass, dimensions, supplier availability, voltage variation, expected current, capacity, mounting clearance, cable strain, and need for sufficient reserve over an entire attempt.

**[TODO]** Document pack mass, model label photograph, age/cycle count, charger settings and datasheet limits. Do not use the 120C printed label as proof of measured safe discharge current.

### D.2 Voltage envelope and the key DFR1208 constraint

A 4S **LiHV** pack can have a manufacturer-permitted full-charge voltage **higher than a standard 4S LiPo pack**; confirm the exact charger/cell rating rather than assume a universal full-charge voltage. Regardless, **nominal 15.2 V is not the lowest loaded pack voltage**.

**Important design-review item:** DFR1208 is specified by DFRobot for **15–24 V input**, but the 4S pack voltage can fall below 15 V as it discharges or sags under load. Therefore, a fixed 12 V output cannot currently be assumed over the whole discharge window.

**[TODO — BLOCKING ELECTRICAL VALIDATION]** Characterize `Vin`/`Vout`/load current and regulator behavior from full charge through the team's chosen safe cutoff, including acceleration load steps. If regulation fails, **re-evaluate the regulator/power topology**; do not claim software can fix an unsupported input-voltage range.

**[TODO: Figure E-05 — Battery voltage versus elapsed time and delivered capacity]** MATLAB plot from logged terminal/pack voltages, labelled state, current and cutoff. Data required: `time_s, battery_v, current_a, test_state, cell_min_v, pack_id`.

**[TODO: Figure E-06 — 12 V output versus battery input]** Scatter/line with safe/regulating region annotated using manufacturer range, including loaded transients; do not use invented points.

### D.3 Safe operation, distribution and measurement

Record:

- Battery connection, main switch location, wiring order and polarity labels.
- Fusing or other overcurrent protection actually installed, if any. **[TODO]** Select after verified steady/peak current and harness ratings.
- Whether switch cuts off **every** branch and peripheral. **[TODO]** Verify with a meter.
- Low-battery indication, voltage measurement point and intended stop/shutdown response. **[TODO]** Specify safe operating threshold from battery maker and load testing, not guesswork.
- Safe mechanical restraint, insulation, protection against LiHV short circuits, and charging/storage procedure.

**[TODO: Figure E-07 — Battery discharge and estimated remaining runtime]** MATLAB model calibrated from measured current and actual discharge data. Distinguish estimated state of charge from directly observed voltage.

---

## E. Regulators and power rails

### E.1 DFR1202 — nominal 5 V supply

**Manufacturer specification:** input **6–24 V**, nominal output **5 V**, advertised maximum **16 A**. These figures describe converter capability under stated conditions; they **do not** establish the current the Pi receives, the stability at the Pi connector, or the actual system demand.

**Design intention:** feed Raspberry Pi 5 via a USB-C power cable. Confirm whether other 5 V loads are fed from the same branch and how the fan, LiDAR and electronics receive power.

**[TODO]** Record converter VIN and VOUT with idle Pi, sensor initialization, CPU load, LiDAR motor on, steering movement, and drive-motor acceleration. Measure at both **converter output** and **Pi USB-C input** where safely accessible.

**Pi power warning:** Raspberry Pi 5 is generally recommended a **5 V/5 A-capable** supply; a two-wire USB-C power adapter may not advertise that capability the way the Pi expects. Validate startup, any USB peripheral-current restrictions, undervoltage flags, wiring resistance and connector heating. An “80 W” regulator rating is **not** proof of a compliant Pi supply.

**[TODO: Figure E-08 — 5 V rail: loaded voltage and ripple over time]** MATLAB plot of Pi-side and regulator-side voltage against time, with CPU/sensor/steering/drive events overlaid. Capture fast transients with equipment of sufficient sampling bandwidth.

### E.2 DFR1208 — nominal 12 V motor branch

**Manufacturer specification:** input **15–24 V**, nominal output **12 V**, maximum **12 A**. Proposed load is the **DFR0592 external motor-power terminal**; confirm no other device is connected.

**[TODO]** Determine which CHP-20GP-180 motor winding voltage is actually rated and the maximum safe drive voltage, then verify compatibility with 12 V motor-driver input. A 12 V converter does not by itself prove the motor should be driven at 12 V.

**[TODO]** Verify regulator regulation and temperature across the LiHV discharge envelope. Section D.2 is a blocking check.

**[TODO: Figure E-09 — Motor rail current and voltage during duty/speed steps]** MATLAB plot showing load steps, supply sag and speed response. Data must come from instrumented tests.

### E.3 Servo supply is a separate question

**SURPASS S0017M:** product references identify a **4.8–6 V** operating range. DFR0604 supports an external `VP` servo-power input in the **6–12 V** range according to its documentation, and its PWM-header positive pin can therefore differ from 5 V depending on how `VP` is used.

**Critical check: do not feed the S0017M with the 12 V motor rail.** Before applying power, identify the voltage physically present on the servo header `+`, the DFR0604 `VP` jumper/input configuration, and the actual wiring shown in the new drawing. Choose and validate an appropriate servo supply, with enough transient-current capacity.

**[TODO — BLOCKING ELECTRICAL VALIDATION]** Provide voltmeter photo of the servo power pin, connection diagram, manufacturer-supported servo voltage, and observed voltage during rapid steering under load.

**[TODO: Figure E-10 — Servo command, current transient and rail voltage]** MATLAB plot measuring repeated movements at controlled angle steps; specify whether the servo is on a dedicated branch or the common 5 V rail.

### E.4 Rail summary (to be filled only with real values)

| Rail | Source | Consumers | Nominal V | Measured idle V | Measured worst transient V | Peak A | Status |
|---|---|---|---:|---:|---:|---:|---|
| PACK | GNB 4S LiHV | Converters / voltmeter [TODO] | 15.2 (label) | [TODO] | [TODO] | [TODO] | Not tested |
| 5V_MAIN | DFR1202 | Pi; other loads [TODO] | 5 | [TODO] | [TODO] | [TODO] | Not tested |
| 12V_MOTOR | DFR1208 | DFR0592 external VIN [TODO] | 12 | [TODO] | [TODO] | [TODO] | Input range unresolved |
| SERVO_V+ | [TODO] | S0017M | [TODO: within rating] | [TODO] | [TODO] | [TODO] | Supply unverified |
| 3V3_LOGIC | Controller/HAT regulated logic | Digital signals only [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | Signal-level map pending |

**[TODO: Figure E-11 — Full power-flow Sankey or stacked-bar view]** Show measured power delivered to each branch, converted loss estimates and battery input. No graph until currents are measured.

---

## F. Power budget, efficiency and endurance

### F.1 Budget methodology

Separate **manufacturer nominal**, **measured idle**, **measured operating**, **measured transient peak**, and **worst-case design assumptions**. Measure at a defined point: battery input, converter input, converter output, or component input. **Do not add currents from different voltage rails directly** as if they were all battery current. Compare powers, with conversion losses accounted for.

Use clearly defined relationships:

- Component or rail power: `P (W) = V (V) × I (A)`.
- Battery input power from independent branches: approximately `sum(P_out / η)` plus quiescent and other battery-side loads, with actual efficiency measured/estimated per converter.
- Energy estimate: `E_nominal (Wh) = V_nominal × Ah`; usable energy depends on load, age, temperature and safe cutoff.
- Predicted endurance: `t (h) ≈ E_usable (Wh) / P_average (W)` as an estimate; compare against a timed test.

### F.2 Component and branch current budget

| Device / state | Supply node | Datasheet typ. A | Datasheet maximum A | Measured idle A | Measured operating A | Measured peak A | Measurement ID |
|---|---|---:|---:|---:|---:|---:|---|
| Raspberry Pi 5 + cooling | 5V_MAIN | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| OV5647 camera | Pi/CSI | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| RPLIDAR S2L | [TODO: 5 V routing] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| DFR0592 logic | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| CHP-20GP-180 drive | 12V_MOTOR | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Encoder | DFR0592 / [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| DFR0604 logic | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Steering S0017M | SERVO_V+ | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| LED button | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| Voltmeter | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| **Total at battery** | PACK | — | — | [TODO] | [TODO] | [TODO] | [TODO] |

**[TODO]** Explicitly note that DFR0592 published current figures are inconsistent across some DFRobot descriptions (chip versus board continuous rating). Check the board revision, heatsinking, datasheet, wiring and observed temperature rather than relying on the largest advertised current.

### F.3 Test conditions to compare

| Case | Pi | Camera | LiDAR | Steering | Drive motor | Expected use |
|---|---|---|---|---|---|---|
| PWR-01: boot | Startup | initializing | [TODO] | stationary | disabled | Detect inrush/brownout |
| PWR-02: ready | idle/wait | ready | scan | center | off | Start state |
| PWR-03: perception | processing | capture | scan | center | off | Sensor power |
| PWR-04: steering | processing | capture | scan | repeated movement | off | Servo transients |
| PWR-05: motion | processing | capture | scan | moving | varying | Realistic load |
| PWR-06: peak | high CPU load | capture | scan | rapid angle changes | acceleration | Worst realistic simultaneous load |
| PWR-07: endurance | normal | normal | normal | normal | normal | Runtime test |

**[TODO: Figure E-12 — Operating-state power comparison]** MATLAB grouped bars with data and uncertainty for average/peak battery power in each named state; include sample count and instrument accuracy.

**[TODO: Figure E-13 — Energy/running-time model versus measured run]** MATLAB prediction + recorded discharge, sensitivity to efficiency and mean drive load; legend distinguishes simulation and observation.

---

## G. Grounds, protection, cabling and thermal effects

### G.1 Common reference and current-return paths

**[TODO]** Trace every ground return from battery negative through splitters/regulators to Pi, HATs, motor, encoder, sensors, servo and voltmeter. Show what shares a physical return conductor and what is merely logically common ground. Highlight potential coupling of motor/servo return current into sensor references.

**[TODO: Figure E-14 — Current-return/ground-routing schematic]** Current direction and net names (`GND_BATT`, `GND_5V`, `GND_MOTOR`, etc.) are documentary labels, **not a claim of isolation**. If grounds are connected, depict those junctions exactly.

### G.2 Wire and connector selection

**[TODO]** Wire-gauge/length table for every power branch (battery, converter inputs and outputs, Pi USB-C, HAT motor VIN, steering, sensor harness), with expected current, conductor temperature/voltage-drop reasoning, connector pin ratings, retention method and polarity inspection.

**[TODO]** Confirm PCT-214 and D1-2 connector actual current/voltage suitability and whether terminals are correctly rated for flexible wire type, ferrules and vibration. Add a disconnect sequence.

**[TODO]** Check power switch DC voltage/current rating, placement and protection. Verify no branch bypasses the OFF switch unintentionally.

### G.3 Thermal management

**[TODO]** Record Raspberry Pi 5 cooling method, fan power source, throttling flags, motor driver/regulator temperatures and operating ambient temperature. Measure after a representative mission and after a longer bench test. Thermal capability must be assessed at the actual chassis airflow and mounting conditions.

**[TODO: Figure E-15 — Temperature against time and load state]** MATLAB comparison for Pi CPU, power converters and driver temperatures. Only include temperatures actually measured or available from a validated sensor.

---

## H. Electrical diagrams — from illustration to verified schematic

A wiring *illustration* shows physical components and approximate coloured wires. A true *electrical schematic* documents named nets, device pins, supply symbols, protection, and connector pin numbers so someone can check the circuit and build it without guessing.

**Current input artifact:** user-supplied pictorial wiring image labelled `Wiring Diagram(1).png` (not yet committed into `schemes/`). The current drawing is **a draft, not signed off**. The new schematic and connector map must be verified against the **physical new vehicle**, not inferred from the older national-final schematic or firmware.

**[TODO] Place the current picture in** `schemes/wiring-draft-v2.png` (or the team's chosen filename); annotate **DRAFT / NOT VERIFIED** until pin-by-pin signed off.

### H.1 Planned figure set — no custom PCB required

| Drawing ID | Deliverable | Suggested editable format | What it must answer | State |
|---|---|---|---|---|
| SCH-01 | System block diagram | diagrams.net / Inkscape / EasyEDA | What are the power and signal domains? | [TODO] |
| SCH-02 | High-level power tree | EasyEDA / KiCad | Which source powers each device, at what voltage? | [TODO] |
| SCH-03 | **Electrical schematic** | **EasyEDA or KiCad** | What is every actual pin, net, protection and connector? | [TODO] |
| SCH-04 | Harness / wiring diagram | diagrams.net + annotated photo | How do people physically connect modules? | [TODO] |
| SCH-05 | Connector pinout sheets | Markdown + EasyEDA | Looking into connector, what is pin 1 and polarity? | [TODO] |
| SCH-06 | Raspberry Pi / DFR0592 / DFR0604 interface | EasyEDA | Which bus/address/GPIO/PWM/encoder connections exist? | [TODO] |
| SCH-07 | Ground return / noise paths | EasyEDA / diagrams.net | Where do high currents return? | [TODO] |
| SCH-08 | Sensor sightline plan + side elevation | CAD screenshot | Can sensors see the necessary field geometry? | [TODO] |
| SCH-09 | Mounted electrical layout and labels | Annotated chassis photo | Can someone reproduce the build reliably? | [TODO] |
| SCH-10 | Power and protection truth table | Markdown / schematic notes | What is live in each switch or fault state? | [TODO] |

**No self-designed PCB is necessary for these drawings.** Use the standard representations of purchased modules, ideally with a symbol describing only their *external pins*, rather than copying proprietary internals. Keep the original `.json` (EasyEDA) or `.kicad_sch` source and export a readable PDF/PNG. A custom PCB should only be attempted if justified by real needs such as size, connector security or power distribution—not because other teams designed one.

### H.2 Drawing discipline / review checklist

For every electrical drawing:

- [ ] Title, author, revision, date and **vehicle version** included.
- [ ] `VIN`, `VOUT`, `GND`, connector orientation and pin 1 marked for each module.
- [ ] Supply and signal lines visually distinguished; every rail carries its **measured/nominal** voltage label.
- [ ] Every net has a name; crossings vs junctions are unambiguous.
- [ ] No output is accidentally tied to another converter output; no unintended backfeeding.
- [ ] Servo header **positive supply voltage** verified independently of PWM signal.
- [ ] Battery power switch, low-voltage monitor and actual installed protection represented.
- [ ] Raspberry Pi USB-C VBUS/GND connector polarity physically continuity-tested.
- [ ] Motor and encoder channels match the code and the terminal blocks exactly.
- [ ] HAT I²C addresses/buses and the resolution method are documented.
- [ ] All connector pin numbering matches a real photographed plug from the specified viewing side.
- [ ] Reviewer performs connection-by-connection continuity checks with the battery **disconnected**.
- [ ] A second team member signs off the as-built map before marking **VERIFIED**.

**[TODO: Table E-01 — Schematic versus physical harness versus software audit]**

| Net / signal | Electrical schematic pin | Actual connector / wire | Actual Pi/driver interface | Code/config reference | Pass? |
|---|---|---|---|---|---|
| PACK+ | [TODO] | [TODO] | N/A | N/A | [TODO] |
| PACK_GND | [TODO] | [TODO] | N/A | N/A | [TODO] |
| 5V_MAIN | [TODO] | [TODO] | USB-C / [TODO] | N/A | [TODO] |
| 12V_MOTOR | [TODO] | [TODO] | DFR0592 VIN | N/A | [TODO] |
| MOTOR_M1+/M1− | [TODO] | [TODO] | DFR0592 M1 / M2 [TODO] | [TODO] | [TODO] |
| ENC_A/ENC_B | [TODO] | [TODO] | DFR0592 E1/E2 [TODO] | [TODO] | [TODO] |
| SERVO_PWM | [TODO] | [TODO] | DFR0604 P0–P3 [TODO] | [TODO] | [TODO] |
| SERVO_V+ | [TODO] | [TODO] | DFR0604 VP/PWM+ [TODO] | N/A | [TODO] |
| BUTTON_D | [TODO] | [TODO] | GPIO/PWM [TODO] | [TODO] | [TODO] |
| LIDAR_TX/RX or USB | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |
| CAMERA_CSI | [TODO] | [TODO] | Pi CSI [TODO] | [TODO] | [TODO] |
| I2C_SDA/SCL | [TODO] | [TODO] | bus [TODO] | [TODO] | [TODO] |

---

## I. Control interfaces and exact pin assignments

### I.1 Raspberry Pi 5 as electrical interface host

**[TODO]** Specify 40-pin GPIO mapping, board orientation and HAT stacking/extension method. A 40-pin header physically accommodating two expansion boards does **not** automatically establish that both board connectors, power rails and buses are compatible. Verify each shared and exclusive pin.

**[TODO]** OS release; enabled I²C/UART/CSI interfaces; actual `/dev/*` devices; supply status; device-tree overlays; third-party libraries (and exact versions); startup dependency order. The software architecture itself belongs in `journal/software.md`, but interface contracts belong here.

### I.2 DFR0592 motor HAT

From DFRobot's reference: motor driver controlled via I²C, standard address **0x10** before modification; motor external input **7–12 V**; on-board encoder terminals and motor channel terminals. These are manufacturer details, **not** the team's verified final settings.

**[TODO]** State motor channel (M1/M2), external VIN+, GND, encoder E1/E2 channel, encoder voltage, direction polarity, command scale, PWM frequency, encoder-speed calibration, and safe motor disable behavior on lost Raspberry Pi commands.

### I.3 DFR0604 expansion HAT and I²C conflict resolution

Manufacturer default address is also **0x10**. **Team reports conflict resolved.** We need the actual bus number/address pair(s) or other tested approach and the exact steps reproducing the result.

| Device | Factory address | Installed bus | Installed address | Resolution action | Verification artifact |
|---|---|---|---|---|---|
| DFR0592 | `0x10` | [TODO] | [TODO] | [TODO] | [TODO: command output] |
| DFR0604 | `0x10` | [TODO] | [TODO] | [TODO] | [TODO: command output] |

**[TODO]** Capture both board responses while concurrently connected; explain persistence across reboot, compatible library constructor arguments, and whether direct physical header stacking is used.

**[TODO: Figure E-16 — I²C bus map and device-detection evidence]** Screenshot or reproducible text log + diagram of actual bus topology and addresses. No address is assumed merely because the conflict is said to be fixed.

### I.4 Steering servo command, power and stop behavior

**[TODO]** Identify selected DFR0604 PWM channel; commanded PWM period, pulse bounds, mechanical steering angle mapping, center value and safety limits. Record voltage/current behavior separately from angle calibration.

### I.5 One start button and power switch

The **rocker switch** is intended to power the vehicle; the **DFR0785** is intended as the single software start interface. Verify actual wiring and state machine behavior against WRO §§9.6–9.14.

**[TODO]** Digital signal polarity, GPIO/port, pull-up/down, debounce, boot waiting-state indicator, accidental re-trigger protection, and whether the LED circuit has an independent control input or turns on with a press.

### I.6 OLED voltage display

**[TODO]** Determine whether the display reads PACK voltage, the 12 V rail, or the 5 V rail. Document whether it is powered from the measured line, its polarity, quiescent current, calibration offset, readability and wiring. The present picture displays the **back** of the module, so no model-specific assumption is made about internal circuit topology.

---

## J. Sensor roles and selection rationale

### J.1 Sensor roles and signal chain

| Sensor / feedback source | Engineering purpose | Raw output | Electrical interface | Calibration needed |
|---|---|---|---|---|
| RPLIDAR S2L | Wall geometry / distance scans | Scan angle + range | UART or adapter, [TODO] | Mount pose, angle zero, range repeatability |
| OV5647 fisheye | Red/green pillar classification and image geometry | Camera frames | CSI | Lens calibration, actual field of view, colour thresholds |
| CHP encoder | Wheel/motor speed and distance estimation | Quadrature counts / speed | DFR0592 encoder inputs | Counts definition, polarity, gear ratio, rolling distance |
| S0017M steering servo | Steering actuation, not a position sensor unless external feedback exists | PWM command | DFR0604 | Pulse/angle/vehicle response mapping |
| OLED voltmeter | Human-visible voltage observation | Displayed value | Analog power measurement | Meter comparison under several voltages |

**[TODO]** Add a decision matrix for whether extra sensors (IMU, ToF, ultrasonic or other) were considered. Do not assert that a rejected part was tested unless the team actually tested it. Compare response delay, range, FOV, update rate, ambient sensitivity, interface complexity, cost, mass, and ability to work with the final chassis/algorithm.

**[TODO: Figure E-17 — Sensor-selection comparison]** MATLAB radar/parallel-coordinate chart or an evidence-based table. If subjective weights are used, publish weighting rationale and sensitivity analysis; avoid pretending weighted scores are measured facts.

### J.2 Placement justified by WRO field geometry

Key external constraints from the rule illustrations and text should be translated to actual sensor installation:

- Field has straight and corner sections, variable layouts, walls, red/green pillars and a parking lot.
- Obstacle Challenge corridor separation is **1000 ± 10 mm** for the International Final (§8, p. 13); Open Challenge has wider/narrower sections (p. 11).
- Parking lot width is **200 mm**; length is **1.5 × robot length** (§5, p. 8).
- The positioning of sensors must also respect the total vehicle size limit.

**[TODO]** Calculate coverage using **actual chassis height, sensor positions and pitch/roll alignment**. Document blind zones directly around the vehicle, LiDAR self-occlusion, camera pillar visibility, wall/floor separation and occlusion from mounts/cables.

**[TODO: Figure E-18 — Top/side sensor coverage on official field geometry]** Use scale dimensions from the rules, distinct sensor origins, and measured or manufacturer-valid angular ranges. Mark sensing assumptions clearly.

---

## K. RPLIDAR S2L — installation and characterization

### K.1 Identification, power and data

**Identified:** DFR1023 RPLIDAR S2L. DFRobot lists **5 V** input, **TTL UART** as the scanner interface, 360° scanning and a supplied USB adapter. The manufacturer listing states approximately 10 Hz scan frequency; **measure the actual setup** rather than assuming a constant 10 Hz under all conditions.

**[TODO]** Clarify the new wiring diagram's **5-pin XH2.54 connector**: exactly which pins are supply, GND, TX, RX and auxiliary/control, based on the actual S2L model harness and adapter. Do not assume wire colours establish the pin order. Document whether a USB serial adapter is installed or Pi UART is used directly, and the applicable logic level and port settings.

**[TODO]** Measure LiDAR current, inrush, scan startup delay, available scans/second, time gaps, rejected/invalid points, and CPU/USB impact, noting whether values come from ROS/SDK or direct serial frames.

### K.2 Physical alignment and scan reliability

**[TODO]** Record height above floor, X/Y position relative to rear axle, angular zero, mounting tilt, body-occluded azimuths, and why this geometry is suitable for the WRO 100 mm wall height and pillar geometry. Include the new chassis design when available.

**[TODO: Figure E-19 — LiDAR range bias and repeatability by distance]** MATLAB line/error-bar visualization using fixed-distance wall measurements. Metadata: reference range, material, ambient illumination, number of scans, orientation and temperature.

**[TODO: Figure E-20 — Per-angle scan validity / blind-zone polar profile]** MATLAB polar graphic from actual scans with missing-return counts and mounted body geometry; do not draw a perfectly unoccluded circle unless observed.

### K.3 Failure cases

**[TODO]** Investigate USB/UART disconnection, warm start, LiDAR startup delay, reflective/sunlit/dark surfaces, blind arcs due to mounts, vibration and blocked scan paths. Document detection, fallback behavior and measured frequency of occurrence; link software handling to `journal/software.md`.

---

## L. OV5647 camera — optical and electrical characterization

### L.1 Camera module and stated lens FOV

**Identified by team:** Raspberry Pi OV5647, 5 MP, fisheye lens **advertised as 222°**. That number is *not yet an observed field of view*, and its dimension (diagonal/horizontal/vertical) is not yet established. Avoid calling a pinhole-camera projection valid for the fisheye without calibration.

**[TODO]** Manufacturer/seller lens sheet, sensor board variant, ribbon cable/adapter type for Pi 5, CSI port used, resolution/FPS chosen, actual frame timestamp and processing delay, exposure/white-balance configuration, and sensor stability during full CPU load.

### L.2 Lens and coordinate calibration

**[TODO]** Record checkerboard/AprilTag calibration procedure appropriate for the actual fisheye model; calibration image set, lighting, image resolution, distortion model and fitted parameter uncertainty. Retain images and parameters with a unique calibration identifier.

**[TODO: Figure E-21 — Undistortion and reprojection-error comparison]** Display input calibration examples, measured image-point error and error-versus-field-angle plot; raw images and MATLAB code should reproduce the result.

**[TODO: Figure E-22 — Camera coverage and pillar visibility]** Measured angular/ground-plane coverage, physically mounted camera height/tilt, field landmarks and sensor blind regions. If LiDAR–camera overlay is used, document extrinsic transform and alignment error separately.

### L.3 Colour and illumination

**[TODO]** Explain red/green traffic-sign detection conditions: exposure, gain, white balance, floor glare, ambient illumination change and when thresholds require retuning. The electrical journal owns optical calibration and sensor characterization; algorithm details and control logic belong in `journal/software.md`.

**[TODO: Figure E-23 — Pillar detection reliability under different lighting]** MATLAB grouped chart from labelled image dataset. Separate red and green, distance bins, false negatives and false positives. Never invent accuracy percentages.

---

## M. Motor, encoder and steering — electrical testing

### M.1 CHP-20GP-180 drive motor

**[TODO]** Photograph complete motor label and confirm gear ratio, rated motor voltage, terminal polarity, measured no-load current, driving current, maximum realistic load and motor temperature. If the pictured gearbox is marked **1:19**, separately verify whether that is the entire effective reduction to wheel speed.

**[TODO: Figure E-24 — Motor PWM / voltage / speed / current sweep]** MATLAB series with explicitly chosen independent variable and evidence for all measured channels. For a PWM driver, distinguish DC supply current from actual motor winding current.

### M.2 Encoder interpretation

**[TODO]** Identify encoder output technology, supply voltage, A/B direction, counts per motor shaft revolution, quadrature counting convention (`×1`, `×2`, `×4`), gearbox ratio and how the DFR0592 library reports speed. Cross-check with a manual known number of wheel rotations.

**[TODO: Figure E-25 — Encoder count linearity and velocity error]** Compare counted revolutions and externally measured revolutions at multiple speeds; plot measured error and repeatability.

### M.3 Steering servo electrical response

**[TODO]** Measured power rail, standby/hold/turning current, signal timing and no-load vs loaded response; mechanical linkage/steering geometry is covered by `journal/mech.md`.

**[TODO: Figure E-26 — Steering command-to-angle / latency]** MATLAB curves based on actual angle measurement with repeat trials, backlash/hysteresis and supply under load.

---

## N. Noise, interference, latency and subsystem interaction

### N.1 Electrical disturbances

A motor, motor driver, DC-DC converters and steering servo can introduce transients or conducted/radiated interference. The question for the journal is **whether our actual sensors/compute show an effect**, not whether it is theoretically possible.

**[TODO]** Define A/B operating conditions: motor OFF/ON, PWM duty sweep, steering OFF/moving, LiDAR OFF/ON, fan OFF/ON, full perception load. Keep camera/LiDAR geometry and scene fixed where possible.

**[TODO: Figure E-27 — Sensor noise under actuator loads]** MATLAB plots comparing LiDAR range jitter and camera timing/frame drop statistics under identical environmental conditions, with actuator state and sample count annotated.

**[TODO: Figure E-28 — 5 V rail transient versus compute faults]** Overlay undervoltage event timestamps with measured 5 V dips, USB/device resets and process logs. Correlation alone does not prove causation; repeat controlled tests.

### N.2 Timing chain

**[TODO]** Measure sensor sampling, timestamping, Pi I²C transaction delay, command-to-driver response and motor/steering reaction. This chapter owns the **physical communication and sensor-latency evidence**. `journal/software.md` owns how algorithms use the data.

**[TODO: Figure E-29 — Sensor-to-actuator timing budget]** MATLAB stacked latency diagram of measured stages, with a protocol-aware definition of start/end and percentiles rather than only means.

---

## O. Reliability, protection and electrical risks

### O.1 Failure register (draft hypotheses, not test outcomes)

| Risk ID | Potential failure / condition | Why it matters | Evidence needed | Planned mitigation / decision |
|---|---|---|---|---|
| R-E01 | **DFR1208 drops out below 15 V input** | 12 V motor rail may cease regulating during LiHV discharge | Loaded voltage sweep | Re-evaluate converter/topology if required |
| R-E02 | **S0017M powered above rated voltage** | Servo damage/overheating | Meter servo `V+` with actual VP setting | Supply only within verified servo rating |
| R-E03 | Pi receives inadequate 5 V at USB-C | Brownout, USB restrictions or resets | Pi-side rail test, power status | Validate cable and supply presentation |
| R-E04 | Undocumented I²C resolution | HAT commands fail on clean rebuild | Address/bus scan + reboot test | Record exact change and libraries |
| R-E05 | DFR0592 thermal/current overload | Unreliable motion or device failure | Running/peak currents and driver temperature | Compare against actual board and chip limits |
| R-E06 | Reversed XT60, USB-C or sensor connector | Electrical damage | Continuity map, connector keying | Polarity labels, keyed plugs, peer review |
| R-E07 | Motor ground transient affects sensing | Scan jitter, I²C errors, resets | Paired noise/power test | Routing/decoupling/protection as validated |
| R-E08 | LiDAR UART pinout/level mismatch | Lost scan data or damage | Native pin datasheet, actual harness | Verified wiring and adapter choice |
| R-E09 | Camera lens spec differs from real FOV | Poor mapping of pillars to field | Calibrated FOV and reprojection data | Use measured calibration |
| R-E10 | Missing software stop on communication loss | Unexpected continued motor output | Controlled disconnect test | Defined hardware/software fail-safe |
| R-E11 | Loose splitter/connector from vibration | Intermittent power | Pull/strain and motion tests | Retention, ferrules and physical support |
| R-E12 | Inadequate battery cutoff or monitoring | Excess discharge or unexpected shutdown | Endurance trace + battery requirements | Appropriate warning/cutoff process |
| R-E13 | Cooling insufficient in printed chassis | Pi throttling, regulator/driver overheating | Thermal tests with chassis | Airflow and mounting iteration |

**[TODO]** After testing, score each risk using an explained likelihood/severity scale; record owner, discovery date, remedy, retest ID and whether the risk is still open. Do not put “solved” without measured or observed evidence.

### O.2 Bench bring-up and safe verification order

1. **Unpowered:** verify wiring with the battery disconnected; inspect polarity and continuity; independently check servo `+` to ground and that it cannot receive 12 V.
2. **Regulator-only:** use a suitable controlled source/current-limited bench supply if available; verify unloaded output and each regulator's permitted input range.
3. **Power distribution:** add one branch/load at a time; record current and voltage near the load.
4. **Pi only:** test boot, power status and temperature.
5. **Interfaces:** test I²C, camera, LiDAR and button separately; record enumerated addresses/devices.
6. **Actuators isolated:** wheels clear of the table; test steering and motor with safety limits and current monitoring.
7. **Simultaneous operation:** collect transient power/sensor logs.
8. **Chassis tests:** inspect under realistic vibration/temperature and retest after each wiring change.

**[TODO]** For each step, add operator, date, measuring instruments, photo/log, pass criterion, failure notes and sign-off. Do not perform high-current stall/short tests without proper equipment and protective procedures.

---

## P. Engineering decisions and revisions

### P.1 Record important choices as evidence-backed decisions

For each design choice, use the same **YBR Decision Record** template:

> **Decision ID:** E-DEC-[TODO]  
> **Problem / constraint:** [TODO]  
> **Options we considered:** [TODO]  
> **Predicted trade-off:** [TODO]  
> **Evidence / calculation / experiment:** [TODO: test IDs and raw data]  
> **What we chose and why:** [TODO]  
> **New cost, limitation or risk introduced:** [TODO]  
> **Verification after implementation:** [TODO]  
> **Current status:** draft / accepted / revised / rejected.

High-value candidates **only if they reflect actual team reasoning**:

- E-DEC-01: Pi 5 plus purchased HATs versus a microcontroller or custom board.
- E-DEC-02: Why the GNB 4S LiHV and two converters were selected; voltage-range assessment.
- E-DEC-03: Why S2L LiDAR and a 222°-advertised camera were chosen and how placement was validated.
- E-DEC-04: Why DFR0592 encoder interface and CHP-20GP-180 were chosen.
- E-DEC-05: How the I²C conflict was resolved in the current build.
- E-DEC-06: How the steering servo power was allocated safely.
- E-DEC-07: Any changes made after noise, transient, wiring or optical calibration experiments.

### P.2 Revision table

| Date | Hardware / harness revision | Problem or hypothesis | Change made | Evidence before | Evidence after | Outcome |
|---|---|---|---|---|---|---|
| [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] | [TODO] |

**[TODO]** Link meaningful Git commits and record test-data association. Version 1.0 can be mentioned only where it provides a meaningful engineering comparison; the root README remains intentionally concise.

---

## Q. MATLAB & Simulink evidence program — planned, not yet produced

Every proposed plot is a **planned deliverable**. No graph, MATLAB script, Simulink block model, raw dataset or validation result has been created in this first draft.

### Q.1 Principles: real data versus models

- **Measurement plot:** graph measured samples, uncertainty, instrument metadata and raw-source path.
- **Calculated estimate:** show the equation and all assumptions; call it *calculated*, not measured.
- **Simulink simulation:** explicitly identify modeled parameters and initial conditions; validate against measured output before making reliability claims.
- **Interactive/control parameters:** MATLAB scripts should expose one editable parameter structure (for example, sample window, battery initial voltage, converter efficiency curve, operating state, motor duty, servo duty cycle, sensor threshold). The user can change these **locally in MATLAB/Simulink** and regenerate output. Do not promise GitHub Markdown itself provides live MATLAB controls.
- **Reproducibility:** commit raw data (or a referenced publicly accessible dataset), `.m` and `.slx` sources if made, generated PNG/SVG figures, MATLAB release/toolboxes used, command to regenerate, and a named revision for each figure.

### Q.2 Proposed work packages

| Work package | Visualization / model to create | Data/model source | Controllable variables | Corresponding figures |
|---|---|---|---|---|
| MW-01 | LiHV discharge and usable-energy estimation | Measured battery voltage/current | cutoff, initial charge, load profile | E-05, E-07, E-13 |
| MW-02 | **DFR1208 low-input verification** | Bench voltage sweep | pack voltage, output load | E-06 |
| MW-03 | 5 V rail voltage sag, buck transient | Oscilloscope/logger, optional Simulink equivalent | load step, cable resistance, filter | E-08, E-28 |
| MW-04 | Motor supply/current/speed response | Current, PWM, encoder time series | duty, acceleration ramp, load | E-09, E-24 |
| MW-05 | Servo load pulses and steering latency | Servo rail current, angle observations | pulse command, angle sequence | E-10, E-26 |
| MW-06 | Battery-to-load power balance | Measured rail current/voltage and efficiency | CPU load, motor duty, converter efficiency | E-11, E-12 |
| MW-07 | Pi/regulator/driver temperature vs load | Temperature log, ambient | cooling condition, run length | E-15 |
| MW-08 | Sensor comparison decision support | Published + tested sensor properties | weighting, test assumptions | E-17 |
| MW-09 | Camera geometric calibration | Calibration photo set | distortion model, calibration resolution | E-21, E-22 |
| MW-10 | LiDAR scan repeatability and occlusion | Recorded scans and known targets | azimuth mask, confidence threshold | E-19, E-20 |
| MW-11 | Colour detection reliability | Labelled images under light conditions | exposure, thresholds, distance bin | E-23 |
| MW-12 | Encoder and motor feedback characterization | Encoder counts + external measurement | sample time, CPR and reduction | E-25 |
| MW-13 | Sensor disturbance from motor/servo PWM | Paired synchronized logs | drive duty, rail state, vibration | E-27 |
| MW-14 | Sensor-to-actuator timing distribution | Time-stamped logs | event boundaries, percentile | E-29 |

**[TODO]** Select the highest-value first **after safe electrical bring-up**: MW-02, MW-03 and MW-06 address the most consequential unresolved power questions. Add the other studies as real equipment and data become available; the rubric does not require dozens of decorative graphs.

### Q.3 Raw data and artifact naming (within the existing folder conventions)

Proposed under the team's existing `other/` folder; **no new top-level `docs/` directory**:

```text
other/
  elec-data/
    power/          # CSV and test logs
    calibration/    # camera and lidar measurements
    interfaces/     # scan, timing and I2C evidence
  matlab/
    elec/           # .m scripts, optional .mlx and .slx projects
  elec-figures/     # generated images, each with test ID
schemes/
  ...               # editable circuits, wiring drawings, PDF/PNG exports
journal/
  elec.md           # this single comprehensive chapter
```

**[TODO]** Adopt actual directory names together with the team; this block is a suggested future arrangement, **not a claim those files already exist**.

### Q.4 Minimum dataset and figure metadata

| Field | What to record |
|---|---|
| `test_id` | Immutable ID such as `E-PWR-004` |
| `date_utc`, time zone | When collected |
| `hardware_rev`, `git_sha`, `config_id` | Exactly which car and software |
| `test_state`, test environment | Motor, steering, CPU/sensors, ambient, stationary/driving |
| `instrument`, `accuracy`, `sample_rate_hz` | Measurement equipment, uncertainty and time resolution |
| `time_s`, `value`, `unit`, `channel` | Raw measurement; one value per channel and time |
| `matlab_version`, toolbox requirements | Reproducible figure generation |
| `script_path`, `model_path`, `parameter_set` | Editable MATLAB/Simulink sources |
| `figure_path`, `interpretation`, `limitations` | Export and what it actually supports |

**[TODO]** Define a standard import table/CSV schema with one example after the team's first real measurement. Do not populate it with fake data to make the journal look complete.

---

## R. Reproduction and inspection checklist

Full assembly instructions belong in `journal/reproducibility.md`; this section contains the **electrical acceptance criteria** that a reproducer must be able to follow.

### R.1 Before connecting battery

- [ ] Every device has a component ID and exact pin/connector assignment.
- [ ] Purchased module revisions and manufacturer limits are recorded.
- [ ] Final drawing matches the **as-built vehicle**, not the earlier photograph alone.
- [ ] Polarity and grounds are verified; no unexpected continuity or short between supply rails.
- [ ] Servo positive rail is measured and lies in approved range before the servo is plugged in.
- [ ] DFR1208 operating limits are compared with intended pack discharge/cutoff range.
- [ ] Pi USB-C power lead is correctly wired and physically retained.
- [ ] Main switch and any installed fuse/protection are identifiable.

### R.2 Powered functional inspection

- [ ] Voltages verified under load at each consumer.
- [ ] Pi 5 starts repeatedly with all peripherals, no unexplained power warnings.
- [ ] DFR0592 and DFR0604 are both detected after cold boot, with documented bus/address configuration.
- [ ] Single-button waiting/start behavior matches WRO requirements.
- [ ] Camera initialized at documented resolution; LiDAR scans at recorded rate; encoder and motor port directions verified.
- [ ] Motor and steering are disabled by the documented stop/shutdown action.
- [ ] Current peaks and temperatures remain within justified module/wire/connector limits.
- [ ] Peer verifies all schematic-versus-harness-versus-code entries and logs sign-off.

**[TODO]** Attach a fresh-assembly inspection record from another team member and link exact Git revision / hardware revision. If a test is incomplete, say so explicitly.

---

## S. Open questions and evidence collection queue

This is a prioritized **draft backlog**, not a list of completed tests.

| Priority | Question / missing information | Requested evidence | Owner / date |
|---|---|---|---|
| **P0** | How is the **S0017M servo actually powered**, and is its header below 6 V? | Photographed wiring + measured header V+ | [TODO] |
| **P0** | How does **DFR1208** behave as pack falls below 15 V input? | Load test, Vin–Vout graph, design decision | [TODO] |
| **P0** | What I²C change resolved `0x10` collision? | Both final addresses/buses, method and power-cycle test | [TODO] |
| **P0** | Is Raspberry Pi 5 USB-C input valid under load? | Pi-side 5 V and supply-recognition/undervoltage report | [TODO] |
| **P1** | Exact power switching, splitters, ground topology, protection | Signed electrical schematic + physical check | [TODO] |
| **P1** | Exact LiDAR native 5-pin / USB adapter connection | Manufacturer pin map + actual harness test | [TODO] |
| **P1** | Which DFR0592 motor/encoder ports and pin mapping? | Wire labels, code reference, encoder test | [TODO] |
| **P1** | Camera actual lens FOV and CSI connection | Module photos, calibration images/results | [TODO] |
| **P1** | Real battery/power budget and cable losses | Instruments and named power-state dataset | [TODO] |
| **P1** | Sensor mounting heights, offsets, occlusion | Chassis CAD + photographs + measured geometry | [TODO] |
| **P2** | Sensor selection alternatives and documented trade-offs | Decision records, actual tests where available | [TODO] |
| **P2** | Interference, temperature and sustained operation | Logs, graphs and after-change retests | [TODO] |
| **P2** | Editable EasyEDA/KiCad schematic and consistent PNG/PDF exports | Native project, net names and review record | [TODO] |
| **P2** | MATLAB/Simulink version, parameter control and data reproducibility | Scripts/models, CSV, regeneration instructions | [TODO] |

---

## T. Sources and engineering references

### Competition requirements

- [WRO International Association — 2026 season](https://wro-association.org/competition/2026-season/)
- [WRO Official Questions & Answers](https://wro-association.org/competition/questions-answers/)
- *WRO 2026 Future Engineers Self-Driving Cars General Rules*, supplied by team; especially **§7**, **§9**, **Appendix C.1–C.4**. [TODO: store or link a properly credited and current official public PDF without duplicating it unnecessarily.]

### Manufacturer / product documentation (verify revision before completing tables)

- [DFRobot DFR0592 DC Motor Driver HAT](https://wiki.dfrobot.com/dfr0592)
- [DFRobot DFR0604 I/O Expansion HAT](https://wiki.dfrobot.com/dfr0604/)
- [DFRobot DFR0604 servo/PWM power description](https://wiki.dfrobot.com/dfr0604/docs/19728)
- [DFRobot DFR1202 5 V / 16 A converter](https://www.dfrobot.com/product-2907.html)
- [DFRobot DFR1208 12 V / 12 A converter](https://www.dfrobot.com/product-2908.html)
- [DFRobot DFR1023 RPLIDAR S2L](https://www.dfrobot.com/product-2617.html)
- [DFRobot DFR0785 white LED pushbutton](https://wiki.dfrobot.com/dfr0785-w/)
- [Raspberry Pi power-supply guidance](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- [TODO: exact CHP-20GP-180 motor/encoder datasheet]
- [TODO: exact SURPASS S0017M datasheet or manufacturer document]
- [TODO: exact OV5647 + 222° lens module datasheet, including focal optics and ribbon specification]
- [TODO: verified GNB LiHV pack/charger specifications and safe cutoff]
- [TODO: actual rocker switch, PCT-214, D1-2 and OLED product references]

### Documentation-style references — *methodology only*

- [KMW Battlepillars — WRO 2026](https://github.com/Battlepillars/Wro26): rationale, technical comparisons, power budget and engineering iteration.
- [MäcLEGO — WRO FE 2026](https://github.com/Mac-Lego-RMS/MaecLEGO-WRO-FE-2026/blob/main/docs/journal/03-power-sensors.md): traceable measurements, explicit data provenance, calibration and failure analysis.

These repositories are **examples of documentation quality, not hardware designs to copy**. YBR-SUNFLOWER will publish only its own circuit, configuration, experiments and engineering explanations.

---

*This page is a living engineering record. An unchecked [TODO] is not a completed design claim. The next revision should begin with verified supply rails, I²C configuration and a pin-by-pin electrical schematic before adding elaborate plots.*
