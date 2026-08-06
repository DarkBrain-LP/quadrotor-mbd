# quadrotor-mbd

Building a quadrotor **from scratch**, using **Model-Based Design (MBD)**: physical modeling in Simulink, controller design and validation in simulation (MIL → SIL → PIL), before any implementation on real hardware.

Personal project built end-to-end to demonstrate a full skill chain: flight mechanics, control theory, Simulink modeling, and embedded C firmware on STM32.

## Project goal

Build a quadrotor capable, eventually, of autonomous navigation — broken down into progressive workpackages, each validated before moving to the next:

| Workpackage | Goal | Status |
|---|---|---|
| **WP1** | Attitude control (roll/pitch/yaw stabilization, hover) | 🟡 In progress |
| WP2 | Altitude / position control | ⏳ Upcoming |
| WP3 | Autonomous navigation | ⏳ Upcoming |

Control architecture: cascaded rate + angle loops per axis (6 PID controllers for WP1), designed from the start to remain extensible to later workpackages without a redesign.

## Methodology

Five-stage validation pipeline, from fully simulated to fully real:

```
MIL → SIL → PIL → HIL / Bench test → Real flight
```

- **MIL** (Model-in-the-Loop) — control algorithm as Simulink blocks, simulated physical plant
- **SIL** (Software-in-the-Loop) — real C code, compiled for the host machine, simulated physical plant
- **PIL** (Processor-in-the-Loop) — same C code, cross-compiled for the STM32 target, simulated physical plant
- **HIL / Bench test** — real firmware, real actuators, controlled/safe environment
- **Real flight** — final validation under real conditions

## Tech stack

- **Modeling & control**: MATLAB / Simulink (rigid body dynamics, vectorized plant model, PID tuning)
- **Embedded target**: STM32F411 (Nucleo-F411RE), ARM Cortex-M4
- **Firmware**: C, FreeRTOS (upcoming — Phase 5)
- **Attitude sensor**: ICM-20948 9-axis IMU (I2C)

## Repo structure

<!-- ```
quadrotor-mbd/
├── docs/                     # Design documents (theory, decisions, build log)
│   ├── phase1_aerodynamics/
│   ├── phase2_components/
│   └── phase3_plant_model/
├── matlab/                   # MATLAB scripts (dynamics, numerical validation, PID tuning)
│   └── plant_rotational.m
├── simulink/                 # Simulink models (.slx)
│   └── drone_plant_wp1.slx
├── firmware/                 # Embedded C / FreeRTOS code (upcoming, Phase 5)
│   ├── src/
│   ├── inc/
│   └── drivers/
└── README.md -->
```

## Design documentation

The `docs/` folder contains documents detailing each phase — theory, design decisions, verified derivations, and validation results:

1. **Phase 1** — Aerodynamics and flight dynamics (reference frames, rigid body, propeller aerodynamics)
2. **Phase 2** — Component selection (frame, motors, IMU, MCU, with justifications)
3. **Phase 3** — Simulink modeling (vectorized rotational plant, cross-validated against MATLAB)
4. **Phase 4** — PID controller design (in progress)

## Progress (WP1)

- [x] Aerodynamics and flight dynamics study
- [x] Component selection (5" frame, 2207/2306 motors, ICM-20948 IMU, Nucleo-F411RE)
- [x] Rotational plant model (3 axes) — derived, simulated in MATLAB (ode45), translated and validated in Simulink (vectorized)
- [ ] Tuning the 6 PID controllers (rate + angle × 3 axes) — rate roll loop in progress
- [ ] Translation block (deferred to WP2)
- [ ] C / FreeRTOS firmware
- [ ] SIL / PIL validation
- [ ] Physical assembly and HIL testing
- [ ] Real flight

## License

Personal educational project — license to be defined.
