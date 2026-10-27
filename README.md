# YBR-SUNFLOWER 🌻

<div align="center">

### WRO 2026 · Future Engineers · Self-Driving Cars

**Thailand · Yothinburana School · Science–Mathematics English Program**

</div>

---

## Project at a glance

YBR-SUNFLOWER is developing a self-driving robotic car for the **World Robot Olympiad (WRO) 2026 Future Engineers** category. The robot is designed to navigate an unfamiliar track autonomously, interpret its surroundings, steer around traffic signs, and perform the required maneuvers using onboard computation and sensors. Our engineering work brings together mechanical design, electrical integration, perception, planning, control, and practical testing.

This repository shares our robot design and source code alongside the reasoning and evidence behind engineering choices. The detailed technical material is organized into **five integrated engineering records** under [`engineering_journal/`](engineering_journal/). Each record combines design explanations, calculations, development decisions, experiments, results, and reproduction information for its subject.

| Project detail | Overview |
|:---|:---|
| Competition | WRO 2026, Future Engineers |
| Team | YBR-SUNFLOWER, Yothinburana School, Thailand |
| robot | Four-wheeled autonomous car with controlled steering and driven wheels |
| Main controller | Raspberry Pi 5 (8 GB) |
| Perception | LiDAR and fisheye camera |
| Current stage | Revised robot in development; final assembly and validation pending |
| Main materials | Engineering records, source code, diagrams, photographs, and demonstrations |

---

## Explore our engineering work

Each link below leads to **one complete record**. The records are organized by engineering responsibility, with testing evidence and design decisions included where they belong.

| Engineering record | What it covers |
|:---|:---|
| **[Mobility and Mechanical Design](journal/1-mech.md)** | Chassis, drivetrain, steering geometry, mechanical iterations, CAD, fabrication, assembly, and measured mechanical performance |
| **[Power and Sensor Architecture](journal/2-elec.md)** | Power distribution, circuits, wiring, components, electronic interfaces, sensors, calibration, and electrical tests |
| **[Software Architecture and Obstacle Strategy](journal/3-software.md)** | Software architecture, perception, localization, motion planning, obstacle handling, control, parking, and test results |
| **[Build and Reproduction Guide](journal/4-build.md)** | Parts and fabrication files, step-by-step physical assembly, electrical checks, dependencies, setup, configuration, launch procedures, and verification |

These five records cover the documentation areas in the WRO 2026 evaluation rubric. **[TO DO]** Complete and review the linked records as the new robot is built and tested. Draft descriptions and planned tests must remain visibly distinguished from verified results.

---

## Our Team

![YBR-SUNFLOWER team photograph](t-photos/t_pic.jpg)

All photos are in [`t-photos/`](t-photos/)

| Member | Primary responsibility | Supporting responsibilities |
|---|---|---|
| **Peradon Nimsongprasert (Tun)** | Robot construction and software development | Testing, debugging, and optimization |
| **Chanakarn Yimsakul (Indy)** | Engineering documentation and technical writing | Development records and software support |
| **Thanphisit Sakulvitulthai (Peach)** | Hardware development and electrical systems | Assembly, tests, and documentation |

**Mentor:** Mr. Punnapon Tanasnitikul

All team members contribute to the engineering process through collaborative design reviews, integration, testing, and documentation. The repository is intended to communicate how the robot is constructed and how technical decisions are evaluated, not simply to display the final result.

---

## Our Robot

**[TO DO]** Add representative version 2.0 photographs once the physical car is complete

All sides are in [`v-photos/`](v-photos/)

### Standard views

| Front | Left | Top |
|---|---|---|
| **[TODO]** Front-view photo | **[TODO]** Right-side photo | **[TODO]** Rear-view photo |

| Rear | Right | Bottom |
|---|---|---|
| **[TODO]** Left-side photo | **[TODO]** Top-view photo | **[TODO]** Bottom-view photo |

### Photoshoot

| Right-front | Right-rear | Left-front | Left-rear |
|---|---|---|---|
| **[TODO]** Photo | **[TODO]** Photo | **[TODO]** Photo | **[TODO]** Photo |

Our robot combines a Raspberry Pi computer, range and vision sensors, a geared drive motor with encoder feedback, and servo-controlled steering. The computer processes observations and coordinates the car's motion through the onboard software and electrical interfaces. Full component specifications, circuit diagrams, pin assignments, mechanical dimensions, tuning parameters, and validation results belong in the engineering records above.

| Version | Short description | Status |
|:---|:---|:---|
| **[1.0](history/README.md)** | Original completed robot design | Completed |
| **2.0** | Redesigned robot with updated chassis, electronics, sensing, and parking development | In progress |

*robot shown: version 1.0. A version 2.0 photograph will be added after assembly.*

**[TO DO]** Add representative version 2.0 photographs once the physical car is complete, including front, rear, left, right, top, and bottom views. The current wiring layout is a design draft; verified connections and test evidence will be recorded in [`engineering_journal/elec.md`](engineering_journal/elec.md).

---

## Project files

The technical records explain **why** decisions were made; the source and supporting materials show **what** was built and implemented. This distinction helps readers follow a design from documentation to an actual component, configuration, or test.

| Resource | Location |
|:---|:---|
| Engineering records | [`engineering_journal/`](engineering_journal/) |
| Program source | [`src/`](src/) |
| Circuit and wiring diagrams | [`schemes/`](schemes/) |
| robot photographs | [`robot-photos/`](v-photos/) |
| Team photographs | [`team-photos/`](t-photos/) |
| Demonstration videos | [`video/`](video/) |
| Supporting resources | [`other/`](other/) |
| License | [`LICENSE.md`](LICENSE.md) |

**[TO DO]** Confirm any changed folder names before publishing the revised README. **[TO DO]** Add working links to the required Open Challenge and Obstacle Challenge demonstration videos. The robot's final software configuration, wiring, and CAD must refer to the same build revision; the verification procedure will be described in the [Build & Reproduction Guide](journal/reproducibility.md).

---

## Official references

- [WRO International Association — 2026 season rules and resources](https://wro-association.org/competition/2026-season/) — Future Engineers general rules, competition material, and documentation rubric.
- [WRO official Questions & Answers](https://wro-association.org/competition/questions-answers/) — official clarifications and additions that may affect the international competition.
- [WRO International Association — main website](https://wro-association.org/) — organizational and competition information.
