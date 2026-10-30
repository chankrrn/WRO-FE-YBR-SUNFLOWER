# Overview

> **Scope:** This journal introduces YBR-SUNFLOWER's system architecture, engineering challenges, design decisions, and verification approach. It also guides readers to the detailed technical journals and supporting evidence.

[← Project README](../README.md) · [Mobility and Mechanical Design →](1-mech.md)

---

## Contents

1. [Purpose and Project Scope](#1-purpose-and-project-scope)
2. [Engineering Journal Map](#2-engineering-journal-map)
3. [Robot System at a Glance](#3-robot-system-at-a-glance)
4. [Operating Challenges and Design Constraints](#4-operating-challenges-and-design-constraints)
5. [Modeling, Visualization, and Verification](#5-modeling-visualization-and-verification)
6. [Engineering Decisions and Progress](#6-engineering-decisions-and-progress)
7. [Evidence and Reproducibility](#7-evidence-and-reproducibility)
8. [References](#8-references)

---

## 1. Purpose and Project Scope

YBR-SUNFLOWER develops an autonomous, car-like robot for the **World Robot Olympiad (WRO) 2026 Future Engineers** challenge. This overview explains how the mechanical, electrical, sensing, and software subsystems fit together and points readers to the team's detailed technical records.

The robot has been completed, including its 3D-printed chassis, system integration, and Raspberry Pi parking software. The repository contains the component lists, illustrations, models, and algorithms documenting the robot's design and implementation.

**Engineering approach.** The team documented design choices, alternative solutions, observations, and revisions throughout the project. Calculations, MATLAB/Simulink models, physical measurements, and on-track tests are distinguished explicitly to clarify the type and strength of evidence supporting each engineering decision.

> [TODO — Project status] Add the vehicle's working version identifier, last verified hardware revision, software commit, latest test date, and development status.
>
> [TODO — Overview image] Add one photograph or labeled system illustration of the *completed* current vehicle when available. Do not use the old vehicle's image without a version label.
---

## 2. Engineering Journal Map

The following **technical journals** combine design explanation and supporting evidence within the same file. Each engineering discipline has its own journal, integrating technical analysis and supporting evidence in one place.

| Record | What a reader will find |
|---|---|
| **[Mobility and Mechanical Design](1-mech.md)** | Chassis, drivetrain, steering, physical packaging, fabrication, dimensional tolerances and design trade-offs |
| **[Power and Sensor Architecture](2-elec.md)** | Supply rails, regulators, components, wiring, camera, LiDAR, encoder, control interfaces and calibration |
| **[Software Architecture and Obstacle Strategy](3-software.md)** | Perception, localization, trajectory generation, obstacle strategy, speed control and parking states |
| **[Build and Reproduction Guide](4-build.md)** | Assembly, dependencies, wiring checks, configuration, startup, testing and reproduction |

System thinking is integrated into each relevant technical journal. The Mechanical Design, Electrical Engineering, Software Architecture, and Build and Reproduction chapters each include their own system-level analysis, covering subsystem interactions, design decisions, constraints, failure analysis, integration testing, and revisions relevant to that topic.

> [TODO — Review] Confirm every journal link is valid after the folder names are finalized.
---

## 3. Robot System at a Glance

### 3.1 Functional Architecture

The robot integrates one mechanical driving and steering platform with an on-board computing system, environment sensors, electrical power distribution, and actuator interfaces. The Raspberry Pi is intended to coordinate sensing, planning, and control.

| Subsystem | Hardware or Implementation | Current evidence status |
|---|---|---|
| Central computing | Raspberry Pi 5 (8 GB) | Identified; installed/configured state [TODO] |
| Drive | CHP-20GP-180 DC gearmotor, CHP-20GP-180 encoder; DFR0592 motor-driver HAT | Identified; mapping and performance [TODO] |
| Steering | SURPASS Hobby S0017M metal-gear servo; DFR0604 I/O expansion HAT | Identified; servo supply, PWM and calibration [TODO] |
| Range sensing | DFR1023 RPLiDAR S2L | Identified; final mounting and calibration [TODO] |
| Image sensing | OV5647 5 MP fisheye Raspberry Pi camera, stated 222° lens field of view | Lens specification unmeasured; camera calibration [TODO] |
| Power storage | GAONENG GNB 4S 15.2 V nominal LiHV, 1300 mAh, 120C, XT60 | Specification supplied; battery-load tests [TODO] |
| Regulation | DFR1202 5 V (6–24 V input, advertised 16 A output); DFR1208 12 V (15–24 V input, advertised 12 A output) | Ratings supplied; input-range and loaded output verification [TODO] |
| User control/indication | DFR0785 white LED button, rocker power switch, 0–30 V mini OLED voltmeter | Identified; exact circuit functions [TODO] |
| Electrical distribution | D1-3 1-to-3 connector, PCT-214 four-port connector, USB-C power lead, LiDAR XH2.54-5P connector | Identified; connector pinouts/polarity [TODO] |

[TODO] The team reports that the DFR0592/DFR0604 I²C conflict has been **resolved**, but its final bus configuration, addresses, and method have not yet been documented. Details belong in [Electrical and Sensor Engineering](2-elec.md).

> [TODO — Architecture figure] Create a labeled, readable block diagram showing **power rails separately from data/control signals**. Indicate Raspberry Pi, drivers, servo, motor/encoder, LiDAR, camera, switches, converters and grounds. Link the verified schematic in `../schemes/`.
>
> [TODO — Verification] Confirm servo supply voltage, Pi power-input behavior, converter operation as the battery discharges, motor-driver ratings, wiring protection and common-ground arrangement.

### 3.2 Key Integrations [TODO: CHECK]

- **Mechanics ↔ Control:** The wheelbase, steering limits, tire geometry and drivetrain transmission affect robot motion and achievable paths.
- **Power ↔ Computing:** Motor and servo transients may interact with the regulated computing supply; robustness requires measurement.
- **Sensors ↔ Geometry:** Camera/LiDAR position and orientation affect observable walls and pillars, uncertainty and collision clearance.
- **Perception ↔ Planning:** Estimated boundaries and pillar locations influence the selected driving path and control outputs.
- **Software ↔ Reproduction:** The documented command, configuration and active hardware interfaces must describe the same release.

> [TODO — Cross-reference] Add exact subsection links after all five journals have stable headings.
---

## 4. Operating Challenges and Design Constraints

The WRO 2026 Future Engineers rules specify two autonomous tasks:

- **Open Challenge:** Three laps with variable track-wall arrangements.
- **Obstacle Challenge:** Three laps with red/green traffic pillars, followed by parallel parking. Red pillars must be passed on their right; green pillars on their left.

The challenge arrangement, direction, and starting configuration can be randomized. Therefore, the design must prioritize reliable perception, control, safety clearance and recovery from imperfect observations.

| Constraint or challenge | Engineering consequence | Documentation location |
|---|---|---|
| Limited robot envelope | CAD/measured dimensions and component packaging | [Mechanical](1-mech.md) |
| Varying road geometry | Boundary detection, feasible steering paths, validation scenarios | [Software](3-software.md) |
| Random pillar layout and side rules | Detection, classification, clearance and strategy evaluation | [Software](3-software.md) |
| Parking in constrained space | Geometric feasibility, steering range, parking states and tests | [Software](3-software.md) |
| Sensor field of view and occlusion | Mounting position, camera calibration and sensor limitations | [Electrical](2-elec.md) |
| Battery sag and transient loads | Converter selection, protection, startup and load measurements | [Electrical](2-elec.md) |
| Reproducible assembly and startup | Schematics, exact configs, software version and checklists | [Build and Verification](5-reproducibility.md) |

> [TODO — Requirements] Add a short requirements-to-evidence matrix referencing actual test IDs. Avoid declaring a rule requirement satisfied before checking it on the assembled robot.
---

## 5. Modeling, Visualization, and Verification

**MathWorks MATLAB and Simulink are planned engineering tools** that can be used to investigate model assumptions, compare candidate solutions, reproduce diagnostic results, and communicate observations.

### 5.1 Evidence Labeling

Every published technical figure should specify:

1. **Evidence class:** *simulation*, *calculation*, *measured bench data*, or *physical driving test*.
2. **Purpose:** the design question it evaluates.
3. **Inputs:** geometry, model assumptions, key parameters and units.
4. **Method and tool:** MATLAB/Simulink version, script/model filename where relevant.
5. **Interpretation:** what the figure supports and what remains uncertain.
6. **Version:** which hardware/software configuration was studied.

### 5.2 Optional Visualization Catalogue

The team has explored visual examples including an Open Challenge field animation, an Obstacle Challenge layout/parking animation, pillar-clearance heatmaps, steering-response traces, trajectory/clearance dashboards, and camera-like pillar renderings. These are candidate evidence types, **not mandatory content** for this overview.

| Candidate figure | Question it could answer | Publication status |
|---|---|---|
| Open Challenge field and simulated route | Does the modeled test environment capture the intended track layout and robot footprint? | [TODO — verify model geometry and source] |
| Obstacle layout / start and parking positions | Are modeled pillar and parking arrangements consistent with the rules? | [TODO — verify rules and dimensions] |
| Pillar clearance versus simulation time | Does the model predict low-clearance or overlap events for chosen routes? | [TODO — document collision model, sign convention, and parameter values] |
| Target versus simulated steering response | What lag, saturation, overshoot or tracking limitations arise under stated assumptions? | [TODO — identify response model; compare with measured servo if possible] |
| Speed profile and route-tracking error | How do speed requests influence path-tracking in simulation? | [TODO — define metrics and validation] |
| Electrical rail voltage and current traces | Are computing/actuator rails stable under actual load and transients? | [TODO — bench measurements and optional MATLAB analysis] |
| Sensor-error versus distance or angle | How do camera/LiDAR calibration uncertainties affect obstacle coordinates? | [TODO — calibration measurements and analysis] |

> [TODO — Curate] Select only **2–4 defensible, well-explained figures** for this overview when sufficient evidence exists; place the detailed analyses in the relevant journal. More plots do not automatically improve the documentation.
>
> [TODO — MATLAB assets] For any published simulation, retain an editable `.m` script or `.slx` model in a suitable existing folder (e.g. `../other/`) if available. If the figure is only illustrative, say so. **A CSV directory is not required.**
>
> [TODO — Image references] Insert Markdown image links only after final filenames are chosen. Avoid dead or temporary screenshot paths.

### 5.3 Simulation-to-Reality checks

The team will compare simulated models with physical observations where possible. Candidate comparisons include steering-command versus measured angle, modeled versus observed turning radius, voltage response under acceleration, camera/LiDAR calibration error, and predicted versus observed pillar clearance.

> [TODO — Define test matrix] Add test date, model revision, track setup, run count, metric, result and link to its technical journal. Do not report improvement percentages without the underlying observations.
---

## 6. Engineering Decisions and Progress

Each technical journal also explains **why** each choice was made, not just which components were selected. A brief decision register is provided here, while detailed analyses are documented in the relevant technical journals.

| Decision under review | Alternatives / constraints to document | Result status |
|---|---|---|
| Camera and LiDAR integration | Visibility, geometry, processing load, complementary limitations | [TODO] |
| Raspberry Pi computing and I/O HATs | Interface compatibility, latency, wiring and extensibility | [TODO] |
| Drive motor, gearing and encoder | Torque/speed trade-off, control resolution, package size | [TODO] |
| Servo steering mechanism | Range, responsiveness, supply, backlash and geometry | [TODO] |
| 5 V / 12 V distribution | Source-voltage limits, load transients, efficiency and thermal margin | [TODO] |
| Path planning and parking development | Feasibility, clearance, tuning and observed failure cases | [TODO] |

> [TODO — Decision evidence] For each significant choice record: **problem → candidate options → criteria → experiment/analysis → observation → choice → remaining risk**.
>
> [TODO — Revision status] Describe the present vehicle's development status without presenting unfinished functions as competition-tested.
---

## 7. Evidence and Reproducibility

The repository contains the relevant source code, configuration files, robot photographs, engineering drawings, and build instructions, providing a complete record of the project's design, implementation, and reproduction process.

| Resource | Where to find it |
|---|---|
| Raspberry Pi and other controller source code | [`../src/`](../src/) |
| Formal electrical schematics and wiring drawings | [`../schemes/`](../schemes/) |
| MATLAB-related figures and visual material | [TODO — choose and create directory; recommended `../visualizations/`] |
| Team and roobt images | [`../t-photos/`](../t-photos/) and [`../v-photos/`](../v-photos/) |
| Version history / earlier robot | [`../history/`](../history/) |
| Assembly, configuration and end-to-end checks | [Build and Verification](5-reproducibility.md) |

> [TODO — Release traceability] Add release identifier/commit, hardware configuration, firmware, Python dependencies, calibration files, schematic revision, CAD revision and tested startup command.
>
> [TODO — Acceptance] Ask another team member to follow the reproduction journal from a clean checkout and record ambiguities.
---

## 8. References

- [WRO International Association — Future Engineers competition and 2026 resources](https://wro-association.org/competition/2026-season/)
- [WRO official Questions & Answers](https://wro-association.org/competition/questions-answers/)
- [World Robot Olympiad Association](https://wro-association.org/)
- [MathWorks MATLAB](https://www.mathworks.com/products/matlab.html)
- [MathWorks Simulink](https://www.mathworks.com/products/simulink.html)


## Remaining work[TODO: REMOVE THIS]

- [ ] Confirm hardware configuration and upload verified vehicle photographs.
- [ ] Finalize safety-reviewed power/wiring drawings and exact HAT/pin mapping.
- [ ] Complete the 3D-printed mechanical system and record measured geometry.
- [ ] Complete and verify Raspberry Pi software including the parking implementation.
- [ ] Validate sensor mounting, camera/LiDAR calibration, encoder and steering calibration.
- [ ] Test electrical stability under realistic drive/steering loads.
- [ ] Validate MATLAB models against physical measurements where appropriate.
- [ ] Publish only clear, correctly labeled figures with reproducible methods.
- [ ] Link every major engineering claim to its detailed journal and evidence.
- [ ] Verify all relative Markdown links and release references before submission.