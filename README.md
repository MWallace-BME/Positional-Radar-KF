[README_1.md](https://github.com/user-attachments/files/32136636/README_1.md)
# Positional-Radar-KF
24GHz FMCW positional radar (RF Beam K-LC7 + STM32F446) with an embedded Extended Kalman Filter for real-time indoor tracking — joint UNR EE782/EE426-626 project.
# Positional Radar with Kalman Filter

A 24 GHz FMCW radar that tracks a person's position in a room in real time, using an embedded Extended Kalman Filter (EKF) to declutter noisy returns into a smooth, continuous track.

This project satisfies two UNR courses simultaneously and doubles as a thesis project:

- **EE 782 — Random Signal Analysis and Estimation Theory** (Dr. Sami Fadali): Kalman filter theory, design, and validation (discrete KF, EKF, UKF comparison, RTS smoother, particle-filter reference).
- **EE426/626 — Microprocessor Applications** (Anthony Turvey): a complete, working embedded product built on the STM32F446, integrating the RF front-end, DSP pipeline, and the EKF into a standalone tracking device.

Built on top of prior UNR Microwaves Lab work (N. Woo, *"The Development of an Independent Frequency Modulated Continuous Wave Radar for Perimeter Security,"* 2021), which validated the RF Beam K-LC7 transceiver for human detection.

## How it works

The RF Beam K-LC7 (24–24.25 GHz FMCW transceiver, integrated 1-Tx/2-Rx patch antenna array) is driven by an STM32F446 dev board, which generates the chirp ramp via its onboard DAC and samples both receive channels simultaneously via its onboard ADCs — enabling phase-based angle-of-arrival estimation with no external DAC/ADC hardware. Range and Doppler FFTs and peak detection run on the STM32; range/angle measurements feed an EKF that tracks position and velocity (x, y, vx, vy) in Cartesian coordinates and outputs a smoothed track plus an uncertainty estimate.

Output is split two ways: a PC-side viewer (serial telemetry → matplotlib/PyQt) renders a top-down room view with raw detections overlaid against the filtered track, trail, and covariance ellipse — visually demonstrating what the filter is doing — and a small onboard I2C OLED reports standalone tracking status without requiring a tethered laptop.

## Repository structure

```
radar-project/
├── firmware/              # 626 team — STM32CubeIDE project (RF front-end, DSP, state machine, display)
│   ├── Core/
│   ├── Drivers/
│   └── ...
├── kalman/                # 782 team — portable C EKF module + host-side test harness
│   ├── src/                # kf.c / ekf.c / kf.h — CMSIS-DSP based, no HAL/board dependencies
│   ├── test/                # host-side unit tests, validated against MATLAB reference
│   └── matlab_reference/    # KF vs EKF vs UKF comparison, RTS smoother, particle-filter reference
├── shared/                # RadarMeasurement.h / TrackState.h — the struct contract between kalman/ and firmware/
├── docs/                  # proposal, BOM, state machine architecture, Gantt chart, meeting minutes
└── README.md
```

The `kalman/` module is written in portable, embedded-friendly C from the start (float32, static allocation, no recursion, no board-specific includes) using CMSIS-DSP matrix functions, so it builds and runs identically on a laptop test harness and on the STM32 target — no separate "port to C" step. `shared/` is the interface seam between the two teams and should only change by cross-team agreement.

## Hardware

- RF Beam K-LC7-RFB-00H (24 GHz FMCW transceiver)
- STM32 Nucleo-F446RE (Cortex-M4, 180 MHz)
- Mikroe Microwave Click (IF gain stage)
- SSD1306 128x64 I2C OLED (onboard status display)

Full bill of materials with dual-sourced pricing: see `docs/`.

## Getting started

**Firmware (`firmware/`):** open in STM32CubeIDE, build and flash to the Nucleo-F446RE.

**Kalman module (`kalman/`):** builds standalone on any machine with CMSIS-DSP available.
```
cd kalman
# build/run host test harness — see kalman/test/README for exact steps once added
```

## Team

| Role | Course |
|---|---|
| Project Manager & Administrative Lead — Matthew Wallace | 626 |
| Technical Lead / Hardware Dev | 426 | — Julian Estorga
| Software Dev 1 (application + firmware) | 426 | — Ayla Velasquez
| Software Dev 2 (firmware) | 426 | — Hayden Walsh
| Kalman filter design & validation | 782 | — Cade Ball, Matthew Wallace   

## Branching / workflow

`main` is protected and always demo-able. Work happens on short-lived `feature/*` branches merged via pull request. Tag commits by subsystem where useful (`[fw]`, `[kalman]`, `[docs]`). Regenerate STM32CubeIDE `.ioc` peripheral config on one branch at a time to avoid merge conflicts in generated code.

## Acknowledgments

Radar design and validation approach based on N. Woo's 2021 UNR thesis (UNR Microwaves Lab, advisor: Dr. Yoon).
