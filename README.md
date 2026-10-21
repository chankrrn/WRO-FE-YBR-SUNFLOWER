# YBR-SUNFLOWER 🌻

<div align="center">

### WRO 2026 · Future Engineers · Self-Driving Cars

**Yothinburana School · Science–Mathematics English Program · Thailand**

*Designing, testing, and documenting an autonomous vehicle as a team.*

</div>

## Project at a glance

YBR-SUNFLOWER is developing a self-driving robotic car for the **World Robot Olympiad (WRO) 2026 Future Engineers** category. The vehicle is designed to navigate an unfamiliar track autonomously, interpret its surroundings, steer around traffic signs, and perform the required maneuvers using onboard computation and sensors. Our engineering work brings together mechanical design, electrical integration, perception, planning, control, and practical testing.

This repository shares our vehicle design and source code alongside the reasoning and evidence behind engineering choices. The detailed technical material is organized into **five integrated engineering records** under [`journal/`](journal/). Each record combines design explanations, calculations, development decisions, experiments, results, and reproduction information for its subject. The front page is intentionally an overview rather than a second copy of the technical reports.

| Project detail | Overview |
|:---|:---|
| Competition | WRO 2026, Future Engineers |
| Team | YBR-SUNFLOWER, Yothinburana School, Thailand |
| Vehicle | Four-wheeled autonomous car with controlled steering and driven wheels |
| Main controller | Raspberry Pi 5 (8 GB) |
| Perception | LiDAR and fisheye camera |
| Current stage | Revised vehicle in development; final assembly and validation pending |
| Main materials | Engineering records, source code, diagrams, photographs, and demonstrations |

## Explore our engineering work

Each link below leads to **one complete record**, rather than separate overview and journal documents. The records are organized by engineering responsibility, with testing evidence and design decisions included where they belong.

| Engineering record | What it covers |
|:---|:---|
| **[Mechanical Design & Construction](journal/mech.md)** | Chassis, drivetrain, steering geometry, mechanical iterations, CAD, fabrication, assembly, and measured mechanical performance |
| **[Electronics & Perception Hardware](journal/elec.md)** | Power distribution, circuits, wiring, components, electronic interfaces, sensors, calibration, and electrical tests |
| **[Autonomous Software & Navigation](journal/software.md)** | Software architecture, perception, localization, motion planning, obstacle handling, control, parking, and test results |
| **[Integrated Engineering & Design Choices](journal/system.md)** | Cross-system trade-offs, engineering decisions, design constraints, failures, alternatives, and evidence-based improvements |
| **[Build & Reproduction Guide](journal/reproducibility.md)** | Parts and fabrication files, step-by-step physical assembly, electrical checks, dependencies, setup, configuration, launch procedures, and verification |

These five records cover the documentation areas in the WRO 2026 evaluation rubric. **[TO DO]** Complete and review the linked records as the new vehicle is built and tested. Draft descriptions and planned tests must remain visibly distinguished from verified results.

## Meet the team

![YBR-SUNFLOWER team photograph](team-photos/t_pic.jpg)

All photos are in [`t-photos/`](t-photos/)

| Member | Main focus |
|:---|:---|
| **Peradon Nimsongprasert** | Robot construction and software development |
| **Chanakarn Yimsakul** | Engineering documentation and technical writing |
| **Thanphisit Sakulvitulthai** | Hardware integration and electrical systems |

**Mentor:** Mr. Punnapon Tanasnitikul

All team members contribute to the engineering process through collaborative design reviews, integration, testing, and documentation. The repository is intended to communicate how the vehicle is constructed and how technical decisions are evaluated, not simply to display the final result.

## The vehicle

**[TO DO]** Add representative version 2.0 photographs once the physical car is complete

All sides are in [`v-photos/`](v-photos/)

Our vehicle combines a Raspberry Pi computer, range and vision sensors, a geared drive motor with encoder feedback, and servo-controlled steering. The computer processes observations and coordinates the car's motion through the onboard software and electrical interfaces. Full component specifications, circuit diagrams, pin assignments, mechanical dimensions, tuning parameters, and validation results belong in the engineering records above.

| Version | Short description | Status |
|:---|:---|:---|
| **[1.0]** | Original completed vehicle design | Completed |
| **2.0** | Redesigned vehicle with updated chassis, electronics, sensing, and parking development | In progress |

*Vehicle shown: version 1.0. A version 2.0 photograph will be added after assembly.*

**[TO DO]** Add representative version 2.0 photographs once the physical car is complete, including front, rear, left, right, top, and bottom views. The current wiring layout is a design draft; verified connections and test evidence will be recorded in [`journal/elec.md`](journal/elec.md).

## Project files

The technical records explain **why** decisions were made; the source and supporting materials show **what** was built and implemented. This distinction helps readers follow a design from documentation to an actual component, configuration, or test.

| Resource | Location |
|:---|:---|
| Engineering records | [`journal/`](journal/) |
| Program source | [`src/`](src/) |
| Circuit and wiring diagrams | [`schemes/`](schemes/) |
| Vehicle photographs | [`robot-photos/`](robot-photos/) |
| Team photographs | [`team-photos/`](team-photos/) |
| Demonstration videos | [`video/`](video/) |
| Supporting resources | [`other/`](other/) |
| License | [`LICENSE.md`](LICENSE.md) |

**[TO DO]** Confirm any changed folder names before publishing the revised README. **[TO DO]** Add working links to the required Open Challenge and Obstacle Challenge demonstration videos. The vehicle's final software configuration, wiring, and CAD must refer to the same build revision; the verification procedure will be described in the [Build & Reproduction Guide](journal/reproducibility.md).

## Official references

- [WRO International Association — 2026 season rules and resources](https://wro-association.org/competition/2026-season/) — Future Engineers general rules, competition material, and documentation rubric.
- [WRO official Questions & Answers](https://wro-association.org/competition/questions-answers/) — official clarifications and additions that may affect the international competition.
- [WRO International Association — main website](https://wro-association.org/) — organizational and competition information.

The applicable official rules and published Q&A take precedence over explanatory statements in this repository. **[TO DO]** Verify all relevant 2026 Q&A updates and the organizer's submission instructions before the final documentation release.
