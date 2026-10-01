# Strain gauge / depth sensing: analysis, fix, and test plan

## 1. Can the Rev 1.0 bridge work?

**Short answer:** it can show that depth is changing. As built it will probably clip at rest, and
it cannot meet the ≤ 0.5 ft depth-resolution requirement across the operating temperature range.

Rev 1.0 chain: BF350 quarter bridge (3 × 350 Ω completion resistors), excited by the P-FET-switched
3.3 V rail, feeding an INA333 with G = 1 + 100 k/249 Ω ≈ 403 and REF ≈ 0.60 V (R14/R15). The INA333
output goes to the ADXL363 aux ADC, which is 12-bit signed over 10–90 % of VS.

| Quantity | Value | Notes |
|---|---|---|
| Bridge sensitivity | 1.65 µV/µε | V<sub>ex</sub>·GF/4 with V<sub>ex</sub> = 3.3 V, GF = 2 |
| INA333 output | 0.66 mV/µε | × 403 |
| ADC step | 0.645 mV/code | 0.8 × 3.3 V / 4096 |
| **Resolution** | **≈ 1 µε/code** | fine on its own |
| Output window | 0.33 – 2.97 V | ADC input range |
| Allowed bridge offset | **−0.66 mV … +5.9 mV** | REF sits only 0.27 V above the bottom of the ADC range |
| Offset from one 1 % completion resistor | up to ±8 mV | V<sub>ex</sub>/4 × 1 % |

### Problem 1: clipping at rest
Three 1 % resistors plus the gauge's own tolerance give a typical untrimmed offset of several mV.
That is outside the window above, especially in the negative direction. Gluing the gauge onto the
diaphragm adds more offset. The `strain_bridge` sketch checks for this directly (test 1).

### Problem 2: temperature
In a quarter bridge, the gauge and the completion resistor next to it sit in different places, on
different materials, with different temperature coefficients. Over the operating range of
35–80 °F (25 K):
- A 100 ppm/°C thick-film completion resistor shifts the bridge like ~1,250 µε of strain.
- A 25 ppm/°C resistor still shifts it like ~310 µε.
- A gauge compensated for steel but bonded to a different material drifts by a similar order.

The whole depth range is unlikely to be more than about 1,000 µε, so temperature drift can be as big
as the signal. 0.5 ft out of 300 ft is 1/600 of full scale.

### Problem 3: sealed-housing gas pressure
The diaphragm measures outside pressure minus the internal N₂ pressure. Internal pressure changes
with temperature by P·ΔT/T ≈ 14.7 psi × 25 K / 287 K ≈ **1.3 psi ≈ 2.9 ft** over the operating
range. This affects any front end, so it needs temperature compensation (the ADXL363 temperature is
logged in every sample) or a zero taken at the surface on every deployment.

### Problem 4: non-ratiometric
The ADXL363 ADC reference is not the bridge excitation, so any change in the 3.3 V rail shows up as
a gain error.

## 2. The fix

### Firmware (done, works on Rev 1.0 today)
- **Bridge power polarity:** Q1 is a P-FET, so the bridge is on when PFET_EN is LOW. The old code had
  it backwards: the bridge stayed on all the time (~9.4 mA), and readings were taken with it off.
- **Signed ADC:** the aux-ADC output is signed. The old `& 0x0FFF` mask turned every reading below
  mid-scale into a large positive number, which is why the unpowered bench reading looked "deep"
  and armed the device.
- **Settling and averaging:** wait 30 ms (3 ODR periods) after power-on, then average 4 fresh
  conversions.
- **Clip detection:** a clipped reading is marked invalid and is never used for arming or detection.
- **Calibrated depth only:** the device only arms on calibrated depth in cm (`cal zero` / `cal span`,
  stored in FRAM). An uncalibrated unit stays SAFE_IDLE and blinks red.
- **NAU7802 driver:** switch with `PRESSURE_FRONTEND PRESSURE_FE_NAU7802` in `config.h`.

### Hardware (Rev 1.1 proposal)
1. Replace INA333 → ADXL363 ADC with a **NAU7802** (24-bit, PGA ×128, I2C at 0x2A, which doesn't
   clash with the fuel gauge at 0x36). Follow the datasheet reference schematic:
   - REFP tied to AVDD and REFN to AVSS, so the measurement is ratiometric and excitation drift cancels.
   - Bridge excited from the AVDD pin (internal LDO set to 3.0 V; the datasheet specifies at least 10 mA).
   - VIN1P/VIN1N from the bridge.
   - The PGA bypass capacitor across VIN2P/VIN2N (the firmware enables PGA_CAP_EN).
2. Delete Q1/R7/R24 (NAU7802 power-down also switches off the bridge), plus INA333, R8–R10, R14–R16,
   and C8. This also removes the 27 µA always-on R14/R15 divider.
3. Bridge topology, in order of preference:
   - **Full bridge, 4 active gauges** on the diaphragm: tangential gauges near the centre see tension,
     radial gauges near the edge see compression. This gives 4× the signal and inherent temperature
     compensation.
   - **Half bridge, 2 active gauges** (one centre-tangential, one edge-radial): 2× the signal, and
     temperature compensated.
   - **Minimum: active + dummy gauge.** The dummy is from the same lot, bonded to an unstrained coupon
     of the housing material, at the same temperature. The other half of the bridge is a matched
     thin-film resistor pair in one package (ratio tempco ≤ 5 ppm/°C). Those resistors can be 10 kΩ,
     because only their ratio matters.
4. The resolution budget changes from ~1 µε/code to ~0.001 µε/count, so noise and drift are set by
   the gauges and the mechanics, not the ADC.

**Prototype before the respin:** wire a SparkFun Qwiic Scale (NAU7802 breakout) to the Pro Mini
(A4 = SDA, A5 = SCL, 3.3 V, GND). Its I2C pull-ups end up in parallel with R1/R2, giving ~2.3 kΩ,
which is fine. Wire the gauge bridge to its E+/E−/A+/A− terminals, set `PRESSURE_FE_NAU7802`, and
run the same tests. `bringup/strain_bridge` auto-detects the NAU7802.

## 3. Test plan

Run these in order. Each test tells you whether the next one is worth running. Keep
`BENCH_NO_PYRO 1` throughout. Capture data with
`python tools/serial_capture.py --port COMx --send "stream 600" --until-end -o file.csv`;
type reference readings as you go and they're saved as MARK lines.

| # | Test | How | Pass criteria |
|---|---|---|---|
| T0 | **DMM static checks** | Flash the main firmware and run `stream 60`. Measure: bridge top node (≈3.3 V while streaming, 0 V otherwise); both midpoints (≈1.65 V); the difference between the midpoints (mV); INA333 REF (≈0.60 V) and OUT. Compare the logged ADC code with (V<sub>OUT</sub> − 1.65 V)/0.645 mV. | Midpoint difference × 403 + 0.60 V stays inside 0.33–2.97 V. The code matches the DMM within a few %, which confirms the ADC transfer assumption. |
| T1 | **Front-end characterisation** | `bringup/strain_bridge` | Test 1 not clipped. Test 2: pick the time where the warm-up curve flattens and set `Pressure::ADXL_SETTLE_MS` / `NAU_SETTLE_MS` to it. Test 3: noise ≪ the 0.5 ft signal step. |
| T2 | **Shunt calibration** (checks the whole gain chain without any pressure) | Put a known resistor R<sub>s</sub> across the gauge. Simulated strain = R/(GF·(R+R<sub>s</sub>)); for example 174 kΩ ≈ 1,000 µε. Read the MEAN lines before and after. | Measured change within ±5 % of the predicted change (≈1,030 codes on Rev 1.0). On Rev 1.0, shunting the gauge drives the output negative, where there is only ~400 µε of headroom. If it clips, shunt R13 instead (opposite sign) or use 1 MΩ (≈175 µε). |
| T3 | **Thermal zero drift** | Housing at 0 psi. Fridge (~35 °F) → room (~80 °F), ≥ 2 h at each, `stream` running (it logs temp_raw). Do it **twice**: housing open (electronics + gauge only), then sealed (adds the gas-law effect). | After fitting counts vs temp_raw, the leftover drift over 35–80 °F is < 0.5 ft. Without compensation, record the slope; it sets the compensation you need. |
| T4 | **Pressure calibration** | Pressure chamber or pressure pot rated > 150 psi with a reference gauge (≤ 0.25 % FS). Step 0 → 25 → 50 → … → 150 → … → 0 psi, 2 min per step, MARK each reference reading, 3 cycles. Then `cal zero` at 0 psi and `cal span <cm>` at a high step (1 psi = 68.6 cm of seawater). | Linearity residual, up/down hysteresis, and cycle-to-cycle repeatability each < 0.22 psi (0.5 ft). |
| T5 | **Creep during soak** | Hold 150 psi for 6–24 h (normal soaks are 6–96 h). | Drift < 0.22 psi, or a characterised curve the detector can tolerate (`SOAK_EXCURSION_CM`). |
| T6 | **Power** | Measure average current with a µCurrent or Nordic PPK2 while soaking (1 Hz) and in transit (5 Hz). | Record it for the battery budget. The bridge should only draw current during readings. |
| T7 | **System (pier)** | Lower the unit on a marked line at the Cal Poly Pier through a descent / hold / haul cycle, then `dump`. | Logged depth matches the line marks. PHASE_CHANGE events show DESCENT → SOAK → ASCENT → SURFACE, a CYC record is written, and there's no CHARGE_REQUESTED. Feed the traces into `test/detector_sim` as new scenarios. |

T4 needs a pressure fixture. The cheapest option is a pipe cap with a pressure fitting that clamps
over the diaphragm, so only the sensing face is pressurised. A hydrostatic column or a pool only
reaches a few psi, which is fine for checking the low end (arm depth).
