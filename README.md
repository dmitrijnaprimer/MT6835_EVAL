# MT6835_EVAL

Evaluation system for the MagnTek MT6835 21-bit magnetic encoder IC.

Measures MT6835 nonlinearity against a LIR-DA237T 23-bit optical reference encoder, computes NLC (Non-Linearity Compensation) correction tables, visualizes results.
Built as a platform for evaluation and a proof-of-concept, pieces of this work can be used for real in-system calibration fixtures.

## Used hardware

- **MCU:** STM32H503CBT6 (not recommended — any dual-SPI MCU will do)
- **Evaluated encoder:** MagnTek MT6835 (SPI, 21-bit)
- **Reference encoder:** LIR-DA237T (BiSS-C, 23-bit, ±0.0083°)
- **Motor driver:** TMC2225 (STEP/DIR, 32 microsteps)
- **Motor:** 400 steps/rev (0.9°), 12 800 microsteps/rev
- **Fixture:** ASA 3D-printed horisontal mechanical bench, 8mm main shaft, ball bearing holder.

Single-shaft fixture — both encoders and the stepper on the same shaft. Magnet is fixed on the shaft end via replaceable adapter. Adapters available for 4×2 mm round, 6×2.5 mm round, 6×6×6 mm cube magnets. Optimal air gap: 0.5–2.0 mm.

## Key findings

| Parameter | Value |
|-----------|-------|
| NLC LSB | 360°/2¹⁸ = 0.001373° (8 counts of 21-bit angle) |
| NLC packing | MSB-first, 6-bit two's complement |
| NLC grid | indexed by raw magnetic angle (before ZERO_POS) |
| DC removal | chip subtracts the mean automatically |
| NLC range | ±0.044° per point (±32 LSB) |

## Quick Start — Python GUI

```bash
cd software
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

## Quick Start — Firmware

### Prerequisites

- [ARM GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-gnu-toolchain-15.2.rel1-x86_64-arm-none-eabi`)
- CMake ≥ 3.22
- Ninja (or another CMake generator)

### Build

```bash
cd firmware/MT6835_EVAL_STM32H503
cmake --preset=Debug
cmake --build --preset=Debug
```

Output: `build/Debug/MT6835_EVAL_01.elf` (`.bin`, `.hex`).

### Flash

Via ST-Link (OpenOCD, STM32CubeProgrammer, or `st-flash`):

```bash
st-flash write build/Debug/MT6835_EVAL_01.bin 0x08000000
```

Or load the `.elf` directly from IDE (STM32CubeIDE, VS Code + Cortex-Debug), or with CubeProgrammer.

## Evaluation workflow

1. **User Auto-Cal** — built-in MT6835 calibration at 50–100 RPM (gets INL to ~±0.07°)
2. **Home** — move shaft to LIR zero position
3. **Set Zero Pos** — set MT6835 ZERO_POS (must be done before NLC, never after)
4. **Collect Data** — stepped measurement at 256 positions across 360°
5. **Generate NLC** — computes correction table from collected error profile
6. **Upload NLC** — writes table to MT6835 registers + EEPROM
7. **Collect Data** again to verify improvement

## Project structure

```
software/         — Python GUI (PyQt5) + calibration logic
firmware/         — STM32 firmware (CMake, C)
data/             — collected CSV data (gitignored)
  nlc/            — generated NLC hex files (gitignored)
construction/     — mechanical STEP files (see note below)
docs/             — technical report, datasheets
```

### Construction files

The `construction/` folder contains individual STEP files for each 3D printed part — **no assembly file is provided**. These are reference geometry only.

## License

MIT
