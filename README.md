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
| `bringup/` | Standalone hardware tests: `gauge_i2c`, `fram_spi`, `accel_spi`, `strain_bridge` |
| `test/detector_sim/` | Host simulation of the real detector against normal and entanglement profiles |
| `tools/viewer.py` | Live 3D view + graphs of the logger, recording, and playback of recordings or FRAM dumps |
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

## Console (serial monitor, 38400, newline)

`help`, `info`, `dump`, `live on|off`, `logsurface on|off`, `serial <id>`, `stream <s>`,
`cal zero`, `cal span <depth_cm>`, `cal set <zero> <cm/count>`, `cal clear`, `afecal`,
`erase YES`, `rearm YES`, `rec <hz> YES`, `rec stop`, `rec clear YES`, `bridge on|off`.
Only `help`, `info` and `live` work while the device is armed underwater.

Every sample is also printed as a machine-readable line (and `live on` adds 25 Hz lines):
`S,t_ms,state,phase,ax_mg,ay_mg,az_mg,motion_mg,temp_raw,p_raw,depth_cm,mv,soc_x100,flags`.

## First bring-up on a board

1. `bringup/gauge_i2c`: the fuel gauge answers at 0x36 and VCELL is plausible (if not, check the CELL pin).
2. `bringup/fram_spi`: FRAM ID `04 7F 48 03`, write/readback passes.
3. `bringup/accel_spi`: 200/200 ID reads, writes stick, |a| ~1000 mg at rest, taps raise INT1 and INT2.
4. `bringup/strain_bridge`: the front end isn't clipped; note the warm-up and noise results.
5. Flash the main sketch, `serial YY.MM.1000X`, then calibrate (see `docs/strain_gauge.md`).

## Detector simulation

`test/detector_sim/run.sh` builds the real `detector.cpp` for the PC and replays synthetic pot
profiles. Every normal-cycle scenario must end in NO FIRE. Rerun it after changing any `Detector::`
constant, and add pier/tow traces as new scenarios.

## Standalone recording (no laptop attached)

Records acceleration at a chosen rate into the board's FRAM, running on battery, then plays it back.

1. Main firmware on the board, battery connected, Serial Monitor at 38400 (or the viewer's command box).
2. `rec 25 YES`. This **erases the logs** and starts recording at 25 Hz (1-50 Hz). The reply says how
   many minutes fit. The green LED flashes every 2 s while recording.
3. Unplug the laptop and do the test. A reset or power cut does not stop the recording; when the
   memory is full it stops by itself (LEDs alternate green/yellow) and keeps everything.
4. Plug back in: `python tools/viewer.py fetch` downloads it to `fram_<date>_<time>.csv` and opens
   the playback. `rec stop` ends a recording early.
5. `rec clear YES` erases it and returns to normal logging (normal logging is paused while a
   recording is held).

| Rate | Fits in FRAM |
|---|---|
| 50 Hz | ~8 min |
| 25 Hz | ~16 min |
| 10 Hz | ~33 min |
| 5 Hz | ~50 min |
| 1 Hz | ~1.5 h |

Depth, battery and state are also saved once per second (~39 bytes/s), which is most of the
space at low rates. Every acceleration sample keeps its
own timestamp; 25 Hz measured 25.05 Hz with no gaps, 50 Hz measured 49 Hz.

## Viewer: live 3D plot, recording, playback

Close the Arduino Serial Monitor first (only one program can use the port).

```bash
pip install pyserial numpy PyQt6 pyqtgraph PyOpenGL

python tools/viewer.py live --port COM5                    # live view (works with accel_spi too)
python tools/viewer.py live --port COM5 --record bench.csv # ...and record everything
python tools/viewer.py play bench.csv                      # play a recording back

# play back the FRAM log: capture a dump, then open it
python tools/serial_capture.py --port COM5 --send dump --until-end -o dump.csv
python tools/viewer.py play dump.csv
```

The 3D view tilts a box with the measured gravity (+X red, +Y green, +Z blue faces; yaw cannot be
measured) and draws the acceleration vector with a trail coloured by device state. The graphs show
acceleration, depth (phases shaded), motion and battery; the panel shows every field of the
current sample. Playback has play/pause (space), speed up to 1000x, and a scrub slider. In live
mode there is a Record button and a box for console commands. FRAM only holds samples taken
underwater unless `logsurface on` was set; for bench sessions, a live recording holds more.

## Known hardware issues (Rev 1.0 → fix on Rev 1.1)

- MAX17048 CELL (pin 2) appears unconnected: tie it to VBAT, or VCELL/SOC are meaningless.
- **On battery alone the 3.3 V rail collapses when the bridge is powered** (bench, Oct 2026: 45 of 49
  resets were power-on + brown-out at the first pressure reading, ~5 mA extra load; fine with the FTDI
  cable supplying 3.3 V). The TPS63900 is configured for 3.3 V / 100 mA input limit on paper, so check
  R3/R4/R5 (CFG1 36.5 kΩ, CFG2 511 Ω, CFG3 16.2 kΩ, ±1%) and the battery path. Diagnose with
  `bridge off` (resets should stop) and the reset breadcrumb printed at boot.
- Pro Mini RAW tied to VCC; its regulator and power LED waste milliamps. Remove them, or use a bare ATmega328P.
- R14/R15 (INA333 REF divider) run from always-on VCC: 27 µA continuous.
- No pull-up on FRAM_CS; no reverse-polarity protection on the user-replaceable battery; no 0.1 µF on ADXL363 VDDI/O.
- Depth front end: see `docs/strain_gauge.md` (NAU7802 + temperature-compensated bridge).
