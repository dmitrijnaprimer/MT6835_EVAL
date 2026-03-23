# MT6835_EVAL

Evaluation fixture for the MagnTek MT6835 21-bit magnetic encoder IC.

Measures MT6835 nonlinearity against a LIR-DA237T 21/22/23-bit optical reference encoder, computes NLC (Non-Linearity Compensation) correction tables.

## Hardware

- **MCU:** STM32H503CBT6 (Not recommended under ANY circumstances!)
- **Evaluated encoder:** MT6835
- **Reference encoder:** LIR-DA237T (BiSS-C, 21/22/23-bit)
- **Motor driver:** TMC2225 (STEP/DIR, 32 microsteps)
- **Motor:** 400 steps/rev (0.9°), 12800 microsteps/rev

Single-shaft fixture — both encoders read the same shaft, motor rotates it in controlled steps for data collection. Magnet is fixed on the shaft end, tension-fitted to special adapter.
Available are adapters for 4x2mm round, 6x2.5mm round, 6x6x6 cube neodymium magnets.

## Quick Start

```
pip install -r requirements.txt
python main.py
```

## Workflow

1. Run **User Auto-Cal** at 50 RPM (built-in MT6835 calibration, gets INL to ~±0.07°)
2. **Collect Data** — stepped measurement at 256-512 positions around 360°
3. **Generate NLC** — computes a 256-entry correction table from the collected error
4. **Upload NLC** — writes the table to MT6835 registers and programs EEPROM
5. Collect again to verify improvement

## License

MIT
