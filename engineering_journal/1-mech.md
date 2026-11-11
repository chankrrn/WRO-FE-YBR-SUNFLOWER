# Mobility and Mechanical Design

> **YBR-SUNFLOWER · World Robot Olympiad 2026 · Future Engineers · Engineering Journal 01**  
> **Scope:** Mechanical architecture, chassis, drivetrain, steering, sensor mounting, CAD models, fabrication, engineering calculations, design decisions, and mechanical verification.  
> **Current generation:** V2.1–V2.4 overhaul; V1.1–V1.3 are historical first-generation revisions.  
> **Implementation:** Custom 3D-printed structures, a geared DC motor, an external gear pair and LEGO differential, and servo-controlled front steering.

[← Project README](../README.md)

[← Engineering Overview](0-overview.md) · [Power and Sensor Architecture →](2-elec.md)

---

## Contents

1. [Mechanical design objectives and evidence policy](#1-mechanical-design-objectives-and-evidence-policy)
2. [Mechanical architecture and development](#2-mechanical-architecture-and-development)
3. [Mechanical components and vehicle geometry](#3-mechanical-components-and-vehicle-geometry)
4. [Chassis and structural layout](#4-chassis-and-structural-layout)
5. [Drivetrain and external gearing](#5-drivetrain-and-external-gearing)
6. [Differential, axles and bearings](#6-differential-axles-and-bearings)
7. [Steering mechanism and turning geometry](#7-steering-mechanism-and-turning-geometry)
8. [Sensor mounting and mechanical integration](#8-sensor-mounting-and-mechanical-integration)
9. [CAD models and part previews](#9-cad-models-and-part-previews)
10. [Materials and fabrication](#10-materials-and-fabrication)
11. [Assembly and mechanical interfaces](#11-assembly-and-mechanical-interfaces)
12. [Engineering decisions and trade-offs](#12-engineering-decisions-and-trade-offs)
13. [Mechanical verification and calibration](#13-mechanical-verification-and-calibration)
14. [Reproducibility and evidence map](#14-reproducibility-and-evidence-map)
15. [Technical references](#15-technical-references)

## 1. Mechanical design objectives and evidence policy

The mechanical system must translate motor and steering commands into repeatable vehicle motion while supporting the electronics and preserving useful sensor views. Our design priorities are a compact structure, accessible modules, freely moving steering and drivetrain assemblies, and a LiDAR scan plane clear of surrounding components.

This chapter follows the evidence conventions used in the electrical journal.

| Evidence type | Meaning in this chapter |
|---|---|
| Repository-documented | A component, assembly method or design choice described in the current project README or build guide |
| CAD asset | An STL or corresponding PNG present in the repository; its presence alone does not prove installation, fit or tested performance |
| Calculated | An equation-derived result with stated inputs and assumptions |
| Measured | A physical observation with its method and test conditions recorded |
| Pending verification | Information needed to connect the documented design to the final assembled car |

No measured turning radius, vehicle mass, wheel diameter, speed, friction loss or steering accuracy is claimed here. The calculations below describe the documented mechanism and identify the quantities required for calibration. Proposed verification procedures are not completed experiments.

## 2. Mechanical architecture and development

### 2.1 Version naming and development sequence

The project consists of two generations of robots, each with multiple design iterations. V1.1, V1.2, and V1.3 belong to the first generation, representing the initial development of our team's robot. V2.1, V2.2, V2.3, and V2.4 belong to the second generation, which represents a major redesign based on our experience with the first generation. V0 refers to the previous-season robot designed by our senior team, which served as a reference for our initial design but is not part of our team's prototype sequence.

| Revised label | Meaning |
|---|---|
| V0 | Previous-generation reference vehicle |
| V1.1 | First team mechanical prototype |
| V1.2 | Functional first-generation prototype |
| V1.3 | Final first-generation competition robot |
| V2.1–V2.4 | Second-generation robot development |

References to “Version 2.0” in the documentation describe the second-generation platform as a family. This chapter uses V2.1–V2.4 for its development stages.

### 2.2 First generation — V1.1 to V1.3

The team's older mechanical account describes an initial 28:28 drivetrain, followed by 21:28 and then 16:28. It reports difficulty obtaining the desired low-speed behavior from the first configuration and qualitative improvement after increasing reduction. These are team-reported observations, without numerical repeatability or speed results attached here.

| Revision | Driver : driven teeth | External reduction | Development recorded in the older account |
|---|---|---|---|
| V1.1 | 28:28 | 1.000:1 | First printed chassis, drivetrain and steering concept |
| V1.2 | 21:28 | 1.333:1 | Functional prototype; Ackermann-style steering and rear bearing support introduced |
| V1.3 | 16:28 | 1.750:1 | Rebuilt first-generation competition platform with reinforced rear supports and revised electronics/sensor structures |

The V1 architecture included an Arduino UNO R4-based control layer, separate computing/electronics plates, a GEEKSERVO steering actuator and older converter mounts. These belong to the historical robot. They are not the current V2 fabrication specification.

The [project README](../README.md#version-1--national-final-robot-design) explains that the original robot was designed before LiDAR integration. An RPLIDAR C1 was added later, but structures obstructed its rear field of view, limiting rear-side perception and parking. This packaging limitation helped motivate the overhaul.

### 2.3 Second generation — V2.1 to V2.4

The second generation is a substantial mechanical and electrical redesign. The team upgraded to **RPLIDAR S2L**, removed the separate Arduino controller, reorganized the chassis for **Raspberry Pi 5 with DFR0592 and DFR0604 HATs**, and changed the external drivetrain to **18:28**.

| Revision | Development stage | Documentation boundary |
|---|---|---|
| V2.1 | Initial overhaul prototype | Starting point of the new robot generation |
| V2.2 | Upgrades to V2.1 | [**TODO**] Add exact modified parts and observations when the revision record is available |
| V2.3 | Functional overhaul prototype | [**TODO**] Add its configuration and verification record |
| V2.4 | Final second-generation competition-design stage | [**TODO**] Current configuration described below; final release and validation evidence remain to be linked |

The stage names follow the team's supplied development sequence. The supplied records do not identify which V2 revision first received each new component. Accordingly, the S2L, controller and gearing changes are documented as generation-level changes rather than assigned invented introduction dates.

### 2.4 What the overhaul changed

| Subsystem | First-generation record | Current second-generation configuration | Mechanical consequence |
|---|---|---|---|
| Drive gearing | V1.3: 16T driver / 28T differential | 18T driver / 28T differential | Recheck gear-center placement, mesh, drivetrain loading and encoder distance scale |
| LiDAR | C1, added to the earlier chassis | S2L, integrated into a LiDAR-centered layout | Accommodate the new sensor's mounting and cable routing; preserve a clear scan plane |
| Main control | Raspberry Pi plus separate Arduino control layer | Raspberry Pi 5 host with motor-driver and I/O HATs | Remove the dedicated Arduino layer from the current assembly and provide stack/connector clearance |
| Steering actuator | GEEKSERVO in the historical account | SURPASS Hobby S0017M | Use current servo base and linkage; recalibrate command-to-wheel angle |
| Power mounting | Historical LM2596 / XL4015 mounting arrangement | DFR1202 5 V and DFR1208 12 V converter assembly | Electrical plate, converter mounts and spacer organize the revised supply modules |
| Distribution | Historical distribution arrangement | D1-3 positive splitter and PCT-214 common ground | Provide accessible fixed mounting and strain relief |
| Structure | Older separate Arduino and Pi plates | Main base, electrical plate and top plate | Reorganize component positions and service access around the sensor view |

### 2.5 LiDAR-centered mechanical architecture

The overhaul places surrounding components below the LiDAR scan plane where practical. Its objective is a clear surrounding view, especially toward the rear. The physical system has four interacting assemblies:

| Assembly | Main elements | Mechanical responsibility |
|---|---|---|
| Structure | Main base, electrical base, top plate and spacers | Locate components and maintain alignment |
| Rear propulsion | CHP motor, 18:28 external pair, differential, axles and bearings | Transmit motor torque to the driven wheels |
| Front steering | S0017M, servo base, linkage, rack, knuckles, hubs and wheel connectors | Change road-wheel direction |
| Sensor mounting | Top-plate LiDAR installation, camera holder/plate and IMU plate | Maintain sensor position and orientation relative to the chassis |

The [build guide](4-build.md#8-mechanical-assembly) documents assembly photographs and inspection steps. Its checklists describe required inspections; a blank checklist does not establish a passed test.

## 3. Mechanical components and vehicle geometry

### 3.1 Documented components

| Component | Documented information | Source |
|---|---|---|
| Drive motor | CHP-20GP-180 with encoder; internal reduction 19:1 | [Electrical journal, Section 15](2-elec.md#15-motor-driver-geared-motor-and-encoder) |
| External gear pair | 18T driver and 28T differential gear | [Build guide, Section 8.4](4-build.md#84-assemble-drivetrain) |
| Differential | LEGO Technic 28T, quantity 1 | [Mechanical BOM](4-build.md#32-mechanical-components) |
| Steering servo | SURPASS Hobby S0017M | [Electrical journal, Section 16](2-elec.md#16-steering-servo-interface) |
| Front bearings | 8 × 14 × 4 mm; listed quantity 2 | [Mechanical BOM](4-build.md#32-mechanical-components) |
| Rear bearings | 8 × 16 × 5 mm; listed quantity 3 | [Mechanical BOM](4-build.md#32-mechanical-components) |
| Wheels and tires | Four LEGO wheel/tire assemblies | [Mechanical BOM](4-build.md#32-mechanical-components) |
| Printed structures | 26 STL files with matching PNG previews | [CAD catalog below](#9-cad-models-and-part-previews) |

[**TODO**] The BOM's tire entry says “30.4 mm D. × 20 mm.” Confirm whether this identifies a rim or a complete tire assembly before using it as rolling diameter. Effective rolling diameter must describe the installed tire under load.

### 3.2 Geometry to confirm on the final car [**TODO**] Check required record

| Quantity | Symbol | Required record |
|---|---|---|
| Overall length, width and height |  $l_{\text{body}}, w_{\text{body}}, h_{\text{body}}$ | Final assembled dimensions, including protruding parts |
| Wheelbase | (L) | Distance between front and rear axle centerlines |
| Front/rear track | $t_f,t_r$ | Left-to-right wheel-center spacing at each axle |
| Loaded rolling radius | $r_w$ | Effective radius or distance traveled per wheel revolution |
| Ground clearance | $h_g$ | Lowest fixed structure above the running surface |
| Front/rear overhang | $O_f,O_r$ | Axle centerline to furthest body point |
| Vehicle mass | (m) | Ready-to-run mass with battery |
| Road-wheel steering limits | $\delta_L,\delta_R$ | Measured angles at the wheels, independently of servo command |

[**TODO**] **Pending:** Add final measurements with date and robot revision. Use the same wheelbase, rolling radius and steering convention in the software configuration. Earlier geometry values must not silently become final dimensions after a chassis revision.

## 4. Chassis and structural layout

![Figure M-01 — Main base CAD preview](../models/pic/Base_p.PNG)

**Figure M-01.** Main base preview from the published CAD assets. The build guide places the Raspberry Pi on the main base, the converters on the electrical plate, and the LiDAR and user controls on the top plate.

The layered construction provides separate mounting surfaces for propulsion, computing, power conversion and sensing. This makes the interfaces between assemblies easier to inspect and allows a module to be removed without redesigning the entire vehicle.

The corresponding trade-offs are additional fasteners, stack height and possible plate movement. Joint stiffness and component placement influence sensor alignment and cable clearances. A low center of gravity is desirable, but no center-of-gravity location or tipping margin has been measured in this record. [**TODO**] Last sentence of this para.

## 5. Drivetrain and external gearing

### 5.1 Power transmission

The documented transmission path is:

**Geared motor output → 18T driver → 28T differential input → rear axles → rear wheels.**

The motor's 19:1 reduction is inside the purchased motor gearbox. The 18T-to-28T stage is a separate external reduction.

### 5.2 Speed and torque relationships

For driver tooth count (z_d=18) and driven tooth count (z_o=28):

```math
i_{ext}=\frac{z_o}{z_d}=\frac{28}{18}\approx1.556,
\qquad
n_c=n_g\frac{18}{28}\approx0.643n_g.
```

Here (n_g) is gearbox-output speed and (n_c) is differential-carrier speed. For an ideal symmetric differential, carrier speed is the average rear-wheel speed:

```math
n_c=\frac{n_L+n_R}{2}.
```

With equal wheel speeds, both rear wheels turn at (n_c). The total nominal reduction from the motor shaft to the differential carrier is:

```math
i_{total}=19\times\frac{28}{18}\approx29.56.
```

For external transmission efficiency (eta_{ext}), carrier torque is approximately:

```math
\tau_c=\eta_{ext}\,i_{ext}\,\tau_g.
```

This is total torque delivered to the differential carrier, not torque available independently at each wheel. Losses and wheel-ground traction determine the useful propulsion.

The electrical journal reports supplier gearbox-output speeds of 780 rpm no-load and 680 rpm rated. Applying only the external ratio gives approximately **501 rpm** and **437 rpm** at the carrier, respectively. These are calculated conversions of supplier operating points, not measured wheel speeds or predicted race speeds.

For straight rolling without slip:

```math
v=\frac{2\pi r_w n_c}{60}.
```

A numerical vehicle speed requires the installed rolling radius and actual loaded motor speed. The external reduction trades speed for torque; this record does not claim that 18:28 is experimentally optimal. 

[**TODO**] LAst sentence of this last para. Keep or not. Its meaning = ตัวเลขเหล่านี้เป็นการคำนวณแปลงจากจุดการทำงานของผู้ผลิต ไม่ใช่ความเร็วล้อที่วัดได้จริง หรือความเร็วในการแข่งขันที่คาดการณ์ไว้

### 5.3 Why 18:28 must be distinguished from the V1.3 ratio

The historical progression was **28:28 → 21:28 → 16:28**. The overhaul then changed to **18:28**. Increasing the driver from 16T to 18T decreases external reduction, so the final change reverses part of the earlier torque-for-speed trade-off.

| Quantity | V1.3: 16:28 | Current V2: 18:28 |
|---|---:|---:|
| External reduction | 1.750:1 | 1.556:1 |
| Gearbox-output-to-carrier speed factor | 0.5714 | 0.6429 |
| Ideal carrier torque multiplier | 1.750 | 1.556 |
| Total nominal reduction including 19:1 gearbox | 33.25:1 | 29.56:1 |

At the same gearbox-output speed, the new ratio gives **18/16 = 1.125**, or **12.5% higher carrier speed**. At equal gearbox-output torque and equal external efficiency, it gives **16/18 ≈ 0.889**, or **11.1% lower carrier torque**. These are kinematic comparisons, not measured improvements in lap time or acceleration.
[**TODO**] Last sentence = ทั้งนี้ ข้อมูลดังกล่าวเป็นการเปรียบเทียบเชิงจลนศาสตร์ (kinematic comparisons) เท่านั้น ไม่ใช่การวัดผลลัพธ์ที่ปรับปรุงขึ้นจริงในแง่ของเวลาต่อรอบ (lap time) หรืออัตราเร่ง

A ratio change also affects encoder calibration. With the same encoder counting convention, nominal counts per wheel revolution scale with external reduction. Changing 16:28 to 18:28 therefore reduces counts per wheel revolution by 11.1%, while distance per count increases by 12.5%, assuming the same tire and straight rolling. Revalidate the software scale rather than retaining a calibration from V1.3.

The team confirms the new selection; a measured test explaining the final performance balance should accompany the release. No claim is made that each step in the full history increased torque.

## 6. Differential, axles and bearings

The differential allows the rear wheels to rotate at different speeds during a turn. The outside wheel follows a longer path than the inside wheel, so forcing equal rear-wheel speeds would require tire slip or structural compliance.

The build guide shows the differential, LEGO axle, bearing mounts and axle sleeves as the rear assembly. Bearings support rotation, while the mounts locate them relative to the base. Sleeves help establish axial positioning; their exact positions and required clearances should follow the assembly photographs and final fit check.

Gear mesh must allow smooth rotation without tooth skipping or excessive preload. A rigid motor bracket maintains gear-center alignment. Binding at a bearing seat or axle sleeve can increase motor load, linking mechanical assembly quality directly to the motor-driver current constraints discussed in the electrical journal.

## 7. Steering mechanism and turning geometry

### 7.1 Published steering assembly

The build guide documents the servo base, kingpin, left/right knuckles, steering rack, wheel connectors and front bearings. It also describes a 1.75 mm filament pin retained with an M2.5 screw at a steering joint.

The CAD catalog includes separate top/bottom knuckles, steering hubs, support and arm. Each STL is a part design, not a complete assembly or quantity declaration. Left/right instances and any mirrored printing must be confirmed from the final build.

### 7.2 Kinematic model

For low-speed planar motion, negligible tire slip and an equivalent single front steering angle $\delta$, the bicycle model gives rear-axle midpoint turning radius:

```math
R=\frac{L}{\tan\delta}.
```

For ideal Ackermann geometry with front track $t_f$:

```math
\tan\delta_{in}=\frac{L}{R-t_f/2},
\qquad
\tan\delta_{out}=\frac{L}{R+t_f/2}.
```

These equations provide a design reference. They do not establish that the printed linkage achieves ideal Ackermann steering. Its actual inner/outer wheel-angle relationship requires measurement.

The rear-axle midpoint radius is also different from the swept outer-body radius. Wheelbase, track, overhang and body width all affect obstacle clearance.

### 7.3 Servo-to-wheel calibration

A servo command is not automatically the road-wheel angle. The servo horn, linkage geometry, backlash and mechanical limits determine the mapping.

Record the command and both wheel angles at center and several left/right positions. Approach each position from both directions to reveal backlash. Establish usable limits before the linkage binds or the servo pushes against a mechanical stop. Apply the measured map in software and verify turning radius on the track.

## 8. Sensor mounting and mechanical integration

### 8.1 RPLIDAR S2L upgrade and placement

The current sensor is **RPLIDAR S2L**, replacing the first generation's C1. The electrical journal identifies its regulated 5 V supply and TTL UART interface through the I/O HAT. The scanner mounts on the top plate in the build guide, so its physical mounting, rotating clearance and cable exit must be checked for the S2L installation.

The LiDAR-centered redesign aims to keep the scanner's surroundings clear. Mount rigidity, scan-plane height and pitch/roll affect which track surfaces the scanner sees. A scanner with a nominal 360° field does not guarantee unobstructed installed coverage.

Inspect the complete robot, including cables and controls, at scan-plane height. Record any blocked sectors and compare stationary scans with known nearby objects. No measured occlusion improvement is claimed here.

### 8.2 Camera and IMU

The repository supplies a camera holder, camera plate and IMU plate. Their final mounting must keep orientation repeatable after removal or maintenance.

The camera's installed view depends on pitch, yaw, lens projection and chassis obstruction. The advertised lens field of view is not a measured usable view. The IMU mounting frame must agree with the software orientation convention.

Route the camera ribbon and sensor wires outside moving steering parts and gears. Sensor mounting changes require updating the relevant calibration or frame transform in the software journal.

### 8.3 Raspberry Pi-only host architecture and HAT packaging

The Arduino has been removed from the current robot. The Raspberry Pi 5 is the main programmable host; the DFR0592 motor-driver HAT and DFR0604 I/O Expansion HAT retain their onboard interface controllers. “Pi-only” therefore describes removal of the separate Arduino, not direct drive of the motor or servo from Pi GPIO.

| Current interface | Assignment from electrical/build records | Mechanical integration requirement |
|---|---|---|
| Pi 5 and HAT stack | Pi on main base; DFR0592 and DFR0604 on the header stack | Correct mounting orientation, board clearance and access to cooler/cables |
| Propulsion and encoder | DFR0592 M1 / E1 | Motor leads remain outside gear and axle paths; harness does not load terminals |
| Steering | S0017M on DFR0604 PWM3 | Servo base maintains linkage alignment; lead clears full steering travel |
| LiDAR | S2L, regulated 5 V and UART | Secure top mounting; strain relief outside the rotating and scanning region |
| Camera | Pi CSI ribbon | Protect ribbon from sharp bends, gears and linkage |
| IMU | SEN0253 via I²C | Fix plate orientation and record chassis-to-sensor axes |
| Start button | DFR0604 Digital 16 | Accessible mounting and secured lead |

Removing the Arduino eliminates its separate board layer and communication harness from the new assembly. The replacement HAT stack still needs vertical space, cooling clearance and service access. No measured mass reduction or software timing improvement is claimed from this change alone.

The electrical journal uses BNO055 IMUPLUS mode with magnetometer-based heading disabled. Describe IMU mounting in terms of frame alignment and rigidity; do not claim an unmeasured magnetic-heading improvement from relocating the plate.

## 9. CAD models and part previews

The following catalog links every published STL to its corresponding PNG. **Preview images show individual models; they are not measured drawings or proof of final assembly.** Roles below follow the build guide where available; ambiguous accessory placement is marked for confirmation.

### 9.1 Structure and mounting

| Part / STL | Model preview | Role / evidence boundary |
|---|---|---|
| [`Base.stl`](../models/CAD/Base.stl) | <img src="../models/pic/Base_p.PNG" width="180" alt="Base CAD preview"> | Main chassis mounting structure |
| [`bottom_buck.stl`](../models/CAD/bottom_buck.stl) | <img src="../models/pic/bottom_buck_p.PNG" width="180" alt="bottom buck CAD preview"> | Buck-converter mounting component; confirm exact placement |
| [`buck_spacer.stl`](../models/CAD/buck_spacer.stl) | <img src="../models/pic/buck_spacer_p.PNG" width="180" alt="buck spacer CAD preview"> | Converter-stack spacer |
| [`elec_base.stl`](../models/CAD/elec_base.stl) | <img src="../models/pic/elec_base_p.PNG" width="180" alt="elec base CAD preview"> | Electrical plate mounting the Buck and wires |
| [`top_buck.stl`](../models/CAD/top_buck.stl) | <img src="../models/pic/top_buck_p.PNG" width="180" alt="top buck CAD preview"> | Buck-converter mounting component |
| [`top_plate.stl`](../models/CAD/top_plate.stl) | <img src="../models/pic/top_plate_p.PNG" width="180" alt="top plate CAD preview"> | Upper plate; LiDAR and control mounting |
| [`IMU_plate.stl`](../models/CAD/IMU_plate.stl) | <img src="../models/pic/IMU_plate_p.PNG" width="180" alt="IMU plate CAD preview"> | IMU mounting plate |
| [`cam_holder.stl`](../models/CAD/cam_holder.stl) | <img src="../models/pic/cam_holder_p.PNG" width="180" alt="cam holder CAD preview"> | Camera support |
| [`cam_plate.stl`](../models/CAD/cam_plate.stl) | <img src="../models/pic/cam_plate_p.PNG" width="180" alt="cam plate CAD preview"> | Camera mounting plate |

### 9.2 Drivetrain

| Part / STL | Model preview | Role / evidence boundary |
|---|---|---|
| [`Mid_axle_sleeve.stl`](../models/CAD/Mid_axle_sleeve.stl) | <img src="../models/pic/Mid_axle_sleeve_p.PNG" width="180" alt="Mid axle sleeve CAD preview"> | Intermediate axle positioning component |
| [`axle_sleeve.stl`](../models/CAD/axle_sleeve.stl) | <img src="../models/pic/axle_sleeve_p.PNG" width="180" alt="axle sleeve CAD preview"> | Axle positioning componen |
| [`bearing_mount.stl`](../models/CAD/bearing_mount.stl) | <img src="../models/pic/bearing_mount_p.PNG" width="180" alt="bearing mount CAD preview"> | Rear bearing support |
| [`gear.stl`](../models/CAD/gear.stl) | <img src="../models/pic/gear_p.PNG" width="180" alt="gear CAD preview"> | Driver gear; build guide identifies the installed driver as 18T |
| [`motor_bracket.stl`](../models/CAD/motor_bracket.stl) | <img src="../models/pic/motor_bracket_p.PNG" width="180" alt="motor bracket CAD preview"> | Motor mounting and gear alignment |

### 9.3 Steering components

| Part / STL | Model preview | Role / evidence boundary |
|---|---|---|
| [`servo_base.stl`](../models/CAD/servo_base.stl) | <img src="../models/pic/servo_base_p.PNG" width="180" alt="servo base CAD preview"> | Servo and steering assembly mounting |
| [`bottom_knuckle.stl`](../models/CAD/steering/bottom_knuckle.stl) | <img src="../models/pic/steering_p/bottom_knuckle_p.PNG" width="180" alt="bottom knuckle CAD preview"> | Lower steering-knuckle component |
| [`steering_arm.stl`](../models/CAD/steering/steering_arm.stl) | <img src="../models/pic/steering_p/steering_arm_p.PNG" width="180" alt="steering arm CAD preview"> | Steering-linkage component. |
| [`steering_hub.stl`](../models/CAD/steering/steering_hub.stl) | <img src="../models/pic/steering_p/steering_hub_p.PNG" width="180" alt="steering hub CAD preview"> | Steering hub |
| [`steering_rack.stl`](../models/CAD/steering/steering_rack.stl) | <img src="../models/pic/steering_p/steering_rack_p.PNG" width="180" alt="steering rack CAD preview"> | Steering linkage/rack component |
| [`steering_support.stl`](../models/CAD/steering/steering_support.stl) | <img src="../models/pic/steering_p/steering_support_p.PNG" width="180" alt="steering support CAD preview"> | Support attaching steering assembly to the base |
| [`top_knuckle.stl`](../models/CAD/steering/top_knuckle.stl) | <img src="../models/pic/steering_p/top_knuckle_p.PNG" width="180" alt="top knuckle CAD preview"> | Upper steering-knuckle component |
| [`wheel_connector.stl`](../models/CAD/steering/wheel_connector.stl) | <img src="../models/pic/steering_p/wheel_connector_p.PNG" width="180" alt="wheel connector CAD preview"> | Front wheel connection; used with the bearing assembly |

### 9.4 Essential Parts

| Part / STL | Model preview | Role / evidence boundary |
|---|---|---|
| [`cap.stl`](../models/CAD/cap.stl) | <img src="../models/pic/cap_p.PNG" width="180" alt="cap CAD preview"> | Cover and protect the main components |
| [`bumper.stl`](../models/CAD/bumper.stl) | <img src="../models/pic/bumper_p.PNG" width="180" alt="bumper CAD preview"> | Protective body component; confirm installed position |
| [`wing.stl`](../models/CAD/wing.stl) | <img src="../models/pic/wing_p.PNG" width="180" alt="wing CAD preview"> | A front that also acts as a bumper(isn't use in the final robot) |
| [`magnet_holder.stl`](../models/CAD/magnet_holder.stl) | <img src="../models/pic/magnet_holder_p.PNG" width="180" alt="magnet holder CAD preview"> | Hold the magnets to connect to the Cap |


### 9.5 Current part groups and historical parts

The current CAD catalog above replaces the old Arduino-base, Pi-plate, step-down-tray and GEEKSERVO-bracket list as the reproduction reference for the overhaul.

| Current part group | Function in the overhauled robot | Installation evidence / remaining detail |
|---|---|---|
| Base, electrical base and top plate | Support the revised computing, converter and sensor layout | Build Sections 8.2, 8.3 and 8.5 |
| Bottom/top buck components and buck spacer | Support and separate the revised converter assembly | Converter stack shown in Build Section 8.2; exact STL placement to confirm |
| Motor bracket and 18T gear | Maintain drive alignment with the 28T differential | Build Section 8.4 |
| Bearing mounts and axle sleeves | Locate the rear axle and differential supports | Build Section 8.4; confirm printed quantities and sleeve positions |
| Servo base, support, rack, arm, knuckles, hubs and wheel connectors | Carry S0017M actuation to the front wheels | Build Section 8.1; verify left/right instances and wheel-angle mapping |
| Camera holder and plate | Maintain camera position and view | Published models; installed pose to record |
| IMU plate | Maintain IMU position and coordinate orientation | Published model; installed pose to record |
| Bumper, cap, wing and magnet holder | Additional published parts | Exact fitted location and function require confirmation; presence in CAD alone is insufficient |

Historical camera arms and the old shared LiDAR/IMU mount should not be assumed interchangeable with the new files. The old wing's airfoil description is not automatically applicable to the current wing model. No aerodynamic or sensing purpose is assigned to an accessory solely from its filename.

## 10. Materials and fabrication

### 10.1 Documented process

The build guide identifies a **Bambu Lab H2D**, **Polymaker ABS Pro** and **Bambu Lab ABS-GF**. ABS Pro is listed for general structural and impact-tolerant parts; ABS-GF is listed for parts requiring greater stiffness and dimensional stability.

These are documented material-selection purposes. No comparative printed-part stiffness, impact strength or dimensional test is supplied here. The final part-by-part material assignment remains to be recorded.

### 10.2 Published slicer settings

| Setting | ABS | ABS-GF |
|---|---|---|
| Wall loops | 4 | 4 |
| Infill | 15% | 15% |
| Supports | Automatic, enabled | Automatic, enabled |
| Nozzle diameter | 0.4 mm | 0.4 mm |
| Build plate | Textured PEI | Textured PEI |

**Source:** [Build guide, Section 5.3](4-build.md#53-final-printing-settings).

These settings do not fully reproduce a print. Add layer height, temperatures, print orientation, infill pattern, support interfaces and any dimensional compensation. Confirm filament-compatible nozzle requirements against the selected material and printer configuration.

Inspect bearing seats, gear teeth and steering joints after removing supports. Test-fit mating parts before final assembly. Do not treat the nominal STL geometry as a guaranteed printed tolerance.

## 11. Assembly and mechanical interfaces

The [photographic assembly guide](4-build.md#8-mechanical-assembly) is the primary reproduction reference.

| Stage | Main work | Acceptance check |
|---|---|---|
| Steering | Assemble servo base, pins, knuckles, rack, hubs and wheel connectors | Wheels rotate; linkage moves through its intended range without binding |
| Electrical plate | Mount converters and distribution hardware | Fasteners secure; board undersides and terminals remain clear |
| Top plate | Install LiDAR, voltmeter and switch | Scanner rotation and scan plane remain clear |
| Drivetrain | Mount motor, gear, bearings, sleeves, axle and differential | Smooth rotation and consistent gear engagement |
| Integration | Join structural layers and front assembly; mount computing hardware | No cable or fastener contacts a moving part or exposed circuit |

**Documentation discrepancy:** The mechanical BOM lists seven M3 × 30 pillars, whereas assembly Section 8.5 describes five M3 × 30 lower spacers and four M2.5 × 25 upper spacers. Confirm final quantities and locations before treating the BOM as a complete fabrication package.

## 12. Engineering decisions and trade-offs

| Problem | Documented choice | Engineering effect and trade-off | Current evidence |
|---|---|---|---|
| Rear LiDAR view obstructed in Version 1 | Arrange Version 2 around the scanner | Clearer intended scan plane; constrains component height and placement | Team explanation in project README; quantitative comparison pending |
| Propulsion requires both speed and torque | 19:1 gearbox plus 18:28 external stage | External stage reduces speed to 64.3% and increases ideal carrier torque by 1.556× | Build guide plus calculation; loaded performance pending |
| Rear wheels follow different paths in a turn | Differential | Permits different wheel speeds; adds gear and axle interfaces | BOM and assembly photographs; scrub/loss comparison pending |
| New-generation packaging and control | Remove Arduino; use Pi 5 with DFR0592/DFR0604 | Removes a separate controller layer; requires accessible HAT stack, cooling and connector clearance | Team confirmation, electrical architecture and build guide |
| Sensor upgrade and rear visibility | C1 replaced with S2L in LiDAR-centered layout | Requires new installed coverage and mounting checks | Team confirmation and electrical/build records; scan comparison pending |
| Electronics require organized mounting | Separate base, electrical and top plates | Modular access; additional joints and stack height | Published models and build photographs |
| Structural parts need different properties | ABS Pro and ABS-GF | Material choice can target toughness or stiffness; process and fit remain part-dependent | Build material list; per-part assignments and comparison pending |

A complete decision record should attach the actual problem, considered options, modification, test conditions, result and final decision. The table records supported choices without inventing rejected alternatives or successful tests.

## 13. Mechanical verification and calibration

### 13.1 Planned physical checks

The procedures below are **proposed verification**, not reported results.

| Test | Method | Record and practical use |
|---|---|---|
| Geometry | Measure assembled axle centers, track, envelope, clearance and mass | Date, revision, tools and uncertainty; update software geometry |
| Steering mapping | Measure both wheel angles across commands, approaching from both directions | Center, limits, asymmetry and backlash; derive command map |
| Turning envelope | Run slow constant-steering circles in both directions | Rear-axle radius and outer-body clearance; check planning assumptions |
| Straight-line rolling | Compare travel distance with encoder observations over repeated runs | Effective distance/count, surface, tire and load state; calibrate odometry |
| Gear and bearing fit | Inspect free rotation and gear engagement before powered testing | Binding, play and adjustment; identify assembly-induced load |
| Loaded propulsion | Compare commanded motion, measured speed and electrical load | Battery state, surface, load and driver conditions; evaluate gearing |
| Sensor clearance | Inspect installed scan plane and known targets around the robot | Occluded sectors and mounting orientation; validate placement |

### 13.2 Historical measurements and carry-over limits

The supplied older account reports a first-generation envelope of **255 × 150 × 230 mm**, mass **855 g with battery**, wheelbase **140 mm**, front track **120 mm** and rear track **115 mm**. Its wording mixes measurement requests and final-value entries, and supplies no method/date. Retain these as historical reported values requiring verification; **do not use them as V2.4 dimensions**.

The same account reports approximately **21% more turning than an assumed full-lock model**. Without its test definition, configuration and linked result, this remains a historical qualitative calibration finding. It is not a V2.4 correction factor, especially after changing the servo, linkage or wheelbase.

V2.4 requires its own geometry, steering, encoder and sensor-placement records. The historical GEEKSERVO specification table and converter mounts likewise do not define current S0017M or HAT integration.

### 13.3 Mechanical risks after the overhaul

| Failure mode | Cross-system effect | Verification |
|---|---|---|
| 18T gear placed at unsuitable center distance | Poor mesh, friction or tooth skipping; higher motor load | Rotate manually, inspect engagement and evaluate under controlled load |
| Reused 16:28 encoder scale | Incorrect distance/speed estimate | Repeat measured rolling-distance calibration |
| Reused old servo map | Incorrect curvature and obstacle clearance | Measure S0017M-to-wheel mapping and turning envelope |
| S2L scan obstruction or tilted top plate | Missing rear observations or inconsistent wall geometry | Inspect installed scan plane and known targets |
| HAT or cooler clearance conflict | Mechanical stress, poor access or restricted airflow | Inspect complete board stack with all cables fitted |
| Converter mounts or cable strain | Module movement or intermittent connections | Inspect fastening, insulation and harness restraint |

### 13.4 Reporting results

For each completed test, record **date, Git SHA, robot revision, configuration, method, surface/environment, repetitions and evidence link**. Report measured data separately from calculated predictions. If a design changes, repeat the same relevant test and state what improved or worsened.

No mechanical MATLAB or Simulink study is linked because the inspected `other/matlab/` directory contains electrical studies only. Future mechanical simulations should state equations, assumptions, adjustable inputs, instructions, outputs and how their findings inform this chapter. Simulation output must remain labeled as simulation.

## 14. Reproducibility and evidence map

| Engineering topic | Primary repository evidence | Remaining confirmation |
|---|---|---|
| Mechanical architecture and iteration | [Project README](../README.md#versions) | Measured comparison of revisions |
| Geometry | Section 3.2 of this chapter | Final installed measurements and software agreement |
| Drivetrain | [Build Section 8.4](4-build.md#84-assemble-drivetrain), CAD catalog | Installed gear verification, rolling scale and loaded tests |
| Steering | [Build Section 8.1](4-build.md#81-assemble-steerng-system), steering CAD | Wheel-angle map, turning radius and backlash |
| Fabrication | [Build Section 5](4-build.md#5-3d-printing-and-mechanical-preparation) | Per-part material, quantities, orientation and complete slicer profile |
| Assembly | [Build Section 8](4-build.md#8-mechanical-assembly) | Spacer-count reconciliation and completed inspections |
| Electrical coupling | [Power and sensor architecture](2-elec.md) | Mechanical load/current correlation |
| Autonomous integration | [Software journal](3-software.md) | Final geometry and actuator calibration in configuration |
| Physical appearance | [Robot photographs](../v-photos/) | Photos tied to the tested final revision |

The published assets currently provide **26 STL files and 26 matching previews**. They do not provide native parametric CAD, an assembly file, part tolerances or a complete printed-instance BOM in the inspected models directory. Preserve the exact file names and case when linking them on GitHub.

## 15. Technical references

- **Team-supplied development account and version correction** — historical V1.1–V1.3 sequence, V2.1–V2.4 stage names, and confirmation of the overhaul, 18:28 gearing, S2L upgrade and Arduino removal. Per-revision change dates and test evidence remain to be added.
- [Project README](../README.md) — robot versions and LiDAR-centered redesign rationale.
- [Build and reproduction guide](4-build.md) — component list, fabrication settings and assembly photographs.
- [Power and sensor architecture](2-elec.md) — motor/servo specification attribution, encoder scaling and actuator electrical constraints.
- [CAD files](../models/CAD/) and [model previews](../models/pic/) — published mechanical assets.
- [Software architecture and obstacle strategy](3-software.md) — integration point for measured geometry and steering calibration.

The gear-ratio, differential and low-speed steering equations in this chapter are analytical models with their assumptions stated alongside them; they are not experimental evidence.

