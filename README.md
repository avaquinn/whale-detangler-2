# Whale Detangler: data logger firmware

Arduino Pro Mini 3.3 V / 8 MHz (ATmega328P). Arduino IDE board: **Arduino Pro or Pro Mini,
ATmega328P (3.3V, 8 MHz)**. The sketch folder must be named `whale-detangler-2` to match
`whale-detangler-2.ino`.

## Layout

| File | Role |
|---|---|
| `whale-detangler-2.ino` | Setup, main loop, state machine (SAFE_IDLE → MONITORING → CHARGING → FIRED / FAULT) |
| `config.h` | **All** pins, feature flags, and tunable thresholds |
| `types.h` | Shared enums/structs and the FRAM log payload formats |
| `detector.*` | Deployment-phase tracking (surface / descent / soak / ascent) and anomaly → charge / abort / fire |
| `cycle.*` | Per-cycle summary (bottom depth, soak range, descent/ascent accel profiles) |
| `pressure.*` | Bridge front end (Rev 1.0 ADXL aux ADC or Rev 1.1 NAU7802) + depth calibration |
| `logger.*`, `FRAM.*` | FRAM layout: A/B header, 12-year journal ring, recent-sample ring, CRC per record, dump |
| `ADXL.*`, `battery.*`, `pyro.*`, `status.*` | Accelerometer, MAX17048 fuel gauge, pyro outputs, LEDs |
| `spi_bus.*`, `i2c_bus.*` | Bus helpers (chip-select discipline, I2C timeout) |
| `console.*` | Serial command console |
| `bringup/` | Standalone hardware tests: `gauge_i2c`, `fram_spi`, `strain_bridge` |
| `test/detector_sim/` | Host simulation of the real detector against normal and entanglement profiles |
| `tools/serial_capture.py` | Record console output (log dumps, pressure streams) to a file |
| `docs/strain_gauge.md` | Depth-sensing analysis, Rev 1.1 proposal, and the test plan |

## Safety flags (`config.h`)

- `BENCH_NO_PYRO 1` (default): the pyro outputs are never driven. Keep it until the pyro board works
  with a simulated load.
- `ENABLE_WATCHDOG 0` (default): burn Optiboot first (e.g. MiniCore over ISP). The stock Pro Mini
  bootloader may boot-loop after a watchdog reset.
- The device **never arms without a depth calibration**. A charge timeout always aborts; only a
  confirmed, deep-enough detector decision fires. The fired latch is kept in FRAM (`rearm YES`
  clears it on the bench).

## Console (serial monitor, 115200, newline)

`help`, `info`, `dump`, `serial <id>`, `stream <s>`, `cal zero`, `cal span <depth_cm>`,
`cal set <zero> <cm/count>`, `cal clear`, `afecal`, `erase YES`, `rearm YES`.
Only `help` and `info` work while the device is armed underwater.

## First bring-up on a board

1. `bringup/gauge_i2c`: the fuel gauge answers at 0x36 and VCELL is plausible (if not, check the CELL pin).
2. `bringup/fram_spi`: FRAM ID `04 7F 48 03`, write/readback passes.
3. `bringup/strain_bridge`: the front end isn't clipped; note the warm-up and noise results.
4. Flash the main sketch, `serial YY.MM.1000X`, then calibrate (see `docs/strain_gauge.md`).

## Detector simulation

`test/detector_sim/run.sh` builds the real `detector.cpp` for the PC and replays synthetic pot
profiles. Every normal-cycle scenario must end in NO FIRE. Rerun it after changing any `Detector::`
constant, and add pier/tow traces as new scenarios.

## Realtime 3D ADXL plotter

```bash
python3 realtime_3d_plot.py --port /dev/cu.usbserial-XXXXX --baud 115200
```

## Known hardware issues (Rev 1.0 → fix on Rev 1.1)

- MAX17048 CELL (pin 2) appears unconnected: tie it to VBAT, or VCELL/SOC are meaningless.
- Pro Mini RAW tied to VCC; its regulator and power LED waste milliamps. Remove them, or use a bare ATmega328P.
- R14/R15 (INA333 REF divider) run from always-on VCC: 27 µA continuous.
- No pull-up on FRAM_CS; no reverse-polarity protection on the user-replaceable battery; no 0.1 µF on ADXL363 VDDI/O.
- Depth front end: see `docs/strain_gauge.md` (NAU7802 + temperature-compensated bridge).
