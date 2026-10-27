# How the CPAP algorithms work — plain-English guide

This document explains, in ordinary language, what each of the important
algorithms in this firmware actually does and why it is built that way. It is
aimed at someone who needs to work on the code, tune it on the bench, or
understand a strange trace, without first reading 3000 lines of C.

It is a companion to [`pressure_control_block_diagram.md`](pressure_control_block_diagram.md),
which has the signal-flow diagrams and the exact variable names. This document
is the "why" and the "in words"; that one is the "what connects to what".

Written against the working tree on **2026-09-21**. Default values quoted here
are the compiled-in defaults; almost all of them are `volatile` globals you can
change live from the debugger or over USB, so a running unit may differ.

---

## 1. The big picture

The device has one job: **keep the air pressure at the patient's mask at the
pressure the clinician set**, while the patient breathes in and out of the same
hose.

That is harder than it sounds, for three reasons:

1. **There is no sensor at the mask.** The pressure sensor is inside the device.
   Air flowing down the hose loses pressure on the way, so what the device
   measures is not what the patient gets.
2. **The patient is a disturbance.** Every breath in sucks air out of the
   circuit and pressure sags; every breath out pushes air back and pressure
   rises. The controller has to fight this several times a minute, forever.
3. **The blower is slow and non-linear.** You cannot change the pressure
   directly. You can only change the fan speed, and the fan takes time to
   change speed, and the relationship between speed and pressure is a curve,
   not a straight line.

The control chain, end to end:

```
therapy setting (e.g. 8 cmH2O)
   -> therapy state machine (ramp, EPR, APAP, mask-off)
   -> pressure setpoint
   -> pressure PID + feed-forward        [CM7, 200 times a second]
   -> target motor RPM
   -> shared memory mailbox
   -> motor field-oriented control       [CM4]
   -> blower
   -> hose -> mask -> patient
   -> pressure and flow sensors -> back to the top
```

### Two processors

This is an STM32H747, which has two CPU cores in one chip, and the work is
split between them.

- **CM7** (the big core) runs everything you would call "the product": the
  screen, the sensors, the pressure control, therapy logic, safety, logging.
- **CM4** (the small core) runs only the motor. It does field-oriented control
  (FOC) of the blower's brushless motor — a fast, dedicated job that must never
  be interrupted by a screen redraw.

They talk through a small block of shared memory (a "mailbox") at address
`0x38000000`, defined in `Common/Inc/blower_ipc.h`. CM7 writes commands into it
(start, stop, set speed); CM4 writes status back (actual RPM, faults, bus
voltage). Each side bumps a sequence counter so the other can tell a new
message from a repeated one.

The important consequence: **CM7 never controls the motor directly.** It asks
for an RPM, and CM4 delivers it. If the mailbox stops being serviced, CM7
notices through a heartbeat check and faults.

---

## 2. Who runs what, and how often

Everything on CM7 runs under FreeRTOS. The tasks, in priority order:

| Task | Rate | Priority | Job |
|---|---|---|---|
| SafetyTask | 200 Hz | Realtime | Watches pressure and the CM4 heartbeat. Can stop therapy. |
| SensorTask | 200 Hz | Above normal | Reads sensors, runs leak, therapy and the pressure PID. |
| GuiTask | ~60 Hz | Normal | TouchGFX screen drawing, rotary knob. |
| NfcTask | 100 Hz | Normal | NFC tag reading. |
| SystemTask | — | Below normal | USB logging, bench commands. |
| ClimateTask | 4 Hz | Below normal | Temperature/humidity sensors, humidifier loop. |
| SelfTestTask | — | Low | SD card and memory tests. |

SafetyTask deliberately sits **above** the control loop. If the control loop
ever hangs or misbehaves, the thing that can shut it down still runs.

Inside SensorTask, one tick (5 ms) does this, in this exact order:

1. Read the pressure sensors over SPI.
2. Convert the flow sensor's differential pressure into a flow number.
3. Update the leak estimate and patient flow.
4. `Therapy_Update()` — state machine, EPR, APAP, mask-off. This sets the
   pressure setpoint.
5. `BlowerCtrl_Step()` — the pressure PID. This sets the motor RPM.
6. Push telemetry to USB and to the ESP32.

The order matters. Therapy runs **before** the PID so that the setpoint the PID
sees is the one decided this tick, not last tick's.

---

## 3. What the sensors actually measure

Three AMS5935 pressure sensors sit on the SPI bus:

- **PS — patient/mask pressure.** An absolute-ish reading of the pressure in
  the circuit, measured inside the device. This is the feedback for the
  pressure loop. Reported in millibar, converted to cmH₂O
  (1 cmH₂O = 0.980665 mbar).
- **FS — flow.** This does *not* measure flow. It measures the tiny pressure
  difference across a restriction in the air path (±5 mbar full scale). Flow is
  worked out from that difference through a calibration table.
- **AS — ambient.** Room pressure, used for compensation.

Two practical facts that shape the code:

- **SPI is shared**, so PS is not refreshed on every single 5 ms tick. The code
  publishes a `g_ps_sample_fresh` flag, and anything that differentiates
  pressure (the D term, the dip/rise detectors) only steps on fresh samples and
  divides by the real interval the sample covers. Getting this wrong produced a
  fake 10 Hz wobble that used to trip the dip detector at random.
- **A bad read is replaced with the last good one** rather than a zero, so a
  single dropped sample can never inject a garbage spike into the PID.

---

## 4. Flow: from a tiny pressure difference to a usable number

`flow_sensor.c`

The raw differential pressure is genuinely noisy. A centrifugal blower makes
1–30 Hz pressure pulses, the restriction itself makes broadband turbulence
hiss, and the housing vibrates into the sensor ports. So the raw signal is
cleaned up in stages:

1. **Low-pass filter**, 10 Hz by default. Fast enough to keep the shape of a
   breath, slow enough to kill the hiss.
2. **Zero offset removal.** The sensor has a small offset that drifts. The code
   watches for genuinely quiet periods and re-learns the zero; you can also
   force it from USB or at therapy stop.
3. **Lookup table** converts the cleaned pressure difference into litres per
   minute (slm).
4. **Spike watch.** Sudden jumps are latched with a full snapshot of what else
   moved at the same moment, so you can tell afterwards whether a needle came
   from the air path or from the arithmetic.

A step-rejection rule freezes the filter on sudden jumps, but with an important
exception: a real disconnect also produces a sudden jump, so the filter is
allowed to catch up at about 600 slm/s. A rejection rule that hid mask-off
would be worse than the noise it removed.

There are two flow numbers and they are **not** interchangeable:

- **`Flow_GetSlm()` — total flow.** Everything leaving the blower. Raw, no
  leak model involved. This is what mask-off detection uses, deliberately: it
  cannot be corrupted by a leak estimate that is still settling.
- **`Flow_GetPatientSlm()` — patient flow.** Total flow minus the vent flow
  minus the estimated leak. This is what the breath detector, EPR and the
  screen use.

If you are reading a plot and wondering why flow "decayed to zero" after a
disconnect: that will be **patient** flow, and what you are watching is the
leak estimator learning that the extra flow is leak, not breathing. Total flow
did not change.

---

## 5. Tube compensation: the pressure we control is an estimate

`tube_comp.c`

The sensor is in the device; the patient is at the far end of a two-metre hose.
When air flows, the hose eats some pressure:

```
hose loss  =  R_lam × |Q|  +  R_turb × Q × |Q|
mask P     =  sensor P  −  sign(Q) × hose loss
```

In words: a bit of loss proportional to flow (laminar), plus more loss
proportional to flow squared (turbulent). When air is flowing towards the
patient the mask sees *less* than the sensor; when the patient breathes out, the
mask sees *more*.

The PID closes the loop on this **estimated mask pressure**, not on the raw
sensor. That means the thing being held constant is what the patient
experiences, which is the point of the whole device. It also means the
controller is trusting a model. If `R_lam`/`R_turb` are wrong for your hose, the
loop will hold the wrong pressure, confidently. Typical values for a 15 mm
slimline hose are around `R_lam = 0.002`, `R_turb = 0.00018`; the built-in pair
(0.00353 / 0.000118) was fitted for the bench hose, but only from 16 to 54 L/min.

Turning tube compensation off makes the estimate equal the raw sensor reading,
and the loop then regulates device-side pressure.

### 5.1 Learn circuit: measuring the attached tube (checklist #85)

`circuit_learn.c` — **MORE > Learn Tube**, or `circuit learn` over USB.

The user takes the mask off the tube, so its end is open to the room. The
pressure there is then atmospheric, and the device sensor reads the tube's drop
directly — no external reference, and (unlike a vent) nothing ties flow to
pressure. The run:

1. blower at rest for 1 s: sensor zero and flow zero;
2. open-loop speed steps 12000, 15500, 19000, 22500, 26000 rpm, each held 0.8 s
   at speed and then averaged for 1 s;
3. a controlled ramp-down to 12000 rpm, then stop;
4. a least-squares fit of `drop = R_lam × Q + R_turb × Q²` (both ≥ 0).

**Is the end open?** After the first step, the pressure must be under 60 % of
the fan's shut-off pressure at that speed *and* under 0.0015 × Q² (a map-free
test, needed where the fan map isn't the blower's), with at least 20 L/min. On
the bench at 12 krpm an open tube gave P/Q² = 0.00022 and the connected circuit
(vent + lung) 0.0033. A refused run stops at about 3 cmH2O and changes nothing.

**Is the result believable?** At least 3 steps, the top one over 50 L/min, fit
rms under max(0.10, 5 % of the largest drop), and at most 3 cmH2O at 60 L/min
(over that is a kinked or blocked tube, or an accessory the compensation should
not quietly absorb). The climb stops early at 10 cmH2O, 140 L/min, a saturated
flow sensor, or 0.8 A of supply current. A good result replaces
`R_lam`/`R_turb`; *new tube* is flagged when it moves the drop at 60 L/min by
more than max(0.15 cmH2O, 30 %).

**Persistence.** Every run is a row in `0:/CIRCUIT.CSV`; at boot, and when a
card goes in while therapy is idle, the last applied row is loaded back.
`circuit reset` returns to the built-in pair (and logs that, so it sticks).

Bench, 2026-10-27 (tube opened just after the vent): R_lam 0, R_turb 0.000209,
0.75 cmH2O at 60 L/min (built-in 0.64), rms 0.034 over 60–145 L/min. Breathing
at 12 cmH2O against the external sensor at the lung, the estimate's error in the
25–45 L/min inspiratory band went from +0.24 (built-in) to +0.11 cmH2O, the same
as at low flow — the remaining +0.1 is the known sensor-to-sensor offset.
Opened at the lung instead, with the external sensor's body still on the end,
it measured 3.4 cmH2O at 60 L/min and refused (*resistance too high*).

Compliance is not measured: the tube's ~1 mL/cmH2O is under 10 mL over a
10 cmH2O step, about what the flow element's zero error integrates to in a
second.

---

## 6. Leak estimation

`leak_estimator.c`, `leak_estimator_vivo.c`

Every mask has holes in it on purpose — the vent — so the patient does not
rebreathe their own CO₂. That intentional flow is not a fault. What matters
clinically is *unintentional* leak: a bad seal, an open mouth, a hose off.

The book equation is:

```
Q_total = Q_patient + Q_vent + Q_leak
```

- `Q_patient` averages to roughly zero over several breaths (what goes in comes
  out), so a slow filter removes it.
- `Q_vent` follows an orifice law, `K × sqrt(mask pressure)`, with `K` around
  8.7 slm per √cmH₂O by default for an adult mask.
- Whatever is left over is the unintentional leak.

Two backends are implemented and selectable at runtime:

- **Simple**: exactly the equation above, with a 10 second filter.
- **Vivo-style** (the default): a bench-measured lookup table of expected leak
  against pressure, multiplied by an adaptive `MeanRatio` that is corrected once
  per breath from the imbalance between inhaled and exhaled volume. It settles
  faster and copes better with masks that do not match the orifice law.

Above about 24 slm of unintentional leak the device calls it "high leak", which
matches the usual industry convention. High leak also **blocks APAP pressure
increases**, because the algorithms that decide to raise pressure cannot trust
their inputs when the circuit is leaking badly.

One subtlety worth knowing: during a disconnect, the leak estimator would
happily learn that 60 slm of escaping air is "normal leak". Mask-off therefore
freezes leak adaptation while it is active (`Leak_SetDisconnectFreeze`).

---

## 7. Pressure control — the core loop

`blower_ctrl.c`, `BlowerCtrl_Step()`, 200 times a second.

This is the heart of the device, so this section is the long one.

### 7.1 The job

Given a target pressure, choose a motor RPM. Do it fast enough that a breath
does not visibly move the pressure, and smoothly enough that the patient does
not feel the controller working.

The output is a single number — RPM — posted to CM4 every tick.

### 7.2 Step one: slew the setpoint

The user-facing setpoint can jump instantly (ramp, EPR, a GUI change). The
setpoint that actually feeds the loop is rate-limited to 18 cmH₂O per second in
both directions.

Why: a step demand asks the loop to do something physically impossible, and the
integrator winds up trying. Slewing the target means the error stays realistic
and the loop stays in its linear region.

### 7.3 Step two: pick the gains (gain scheduling)

A blower is not a linear device. Near 4 cmH₂O, a small RPM change moves the
pressure a lot. Near 20 cmH₂O, the same RPM change barely moves it. One set of
gains cannot work across the whole range: tuned for the top it is sluggish at
the bottom, tuned for the bottom it oscillates at the top.

So there are three sets, chosen by the **setpoint**:

| Zone | Range (cmH₂O) | Kp | Ki | Kd |
|---|---|---|---|---|
| Low | 4 – 8 | 1800 | 4500 | 2.0 |
| Mid | 8 – 15 | 1200 | 4200 | 1.5 |
| High | 15 – 25 | 1000 | 3900 | 0.8 |

These came from an open-loop step-response fit on the bench (a first-order plus
dead-time model: gain 0.000934 cmH₂O/RPM, time constant 68 ms, dead time 9 ms)
run through the SIMC tuning rules, then adjusted by hand.

Scheduling is on the *setpoint*, not the measurement, and there is a 0.5 cmH₂O
hysteresis band. If it scheduled on the measurement, a pressure sitting right
on a zone boundary would flap between two gain sets several times a second.

### 7.4 Step three: the measurement

The feedback value is the tube-compensated mask estimate, lightly smoothed
(roughly half old value, half new, each tick).

There is one refinement: during an inhale, if the pressure is sagging below
target by more than a threshold, the loop blends towards the *unfiltered*
measurement in proportion to how big the sag is. Filtering costs time, and time
is exactly what you do not have when a breath is pulling the pressure down. The
blend is only active during inhalation sags, so it does not add noise anywhere
else.

### 7.5 Step four: P, I and D

**P — proportional.** `Kp × error`. The workhorse. It has an asymmetric twist:

- If the patient is inhaling and pressure is *below* target by more than
  0.08 cmH₂O, P is multiplied by 3.5.
- If pressure is *above* target by more than 0.12 cmH₂O, P is multiplied by 6.0.

Both directions are hurry-up terms. Overshoot is punished harder than
undershoot because an over-pressure lingers — the air has to bleed out through
the vent — whereas an under-pressure is corrected by the blower itself.

**I — integral.** Removes steady-state error: the slow, patient accumulation
that finds the exact RPM needed to sit on target. It has three protections,
covered in 7.7.

It also has a **deadband** (1.2 cmH₂O by default): outside that band the
integrator is frozen. The integrator's job is fine trim near the target, not
powering through a large transient — that is P's and feed-forward's job.

And it integrates a **filtered** error rather than the raw one, with a filter
that starts fast and slows down over the first minute of therapy. A slow filter
from the very first tick would carry stale error from the previous session into
this one.

**D — derivative.** `−Kd × rate of change of the measurement`. Note: of the
*measurement*, not of the error. If it used the error, every setpoint change
would produce a violent kick. Using the measurement, a setpoint change is
invisible to D. The derivative is filtered (alpha 0.25) because raw
differentiation of a 200 Hz signal is mostly noise.

### 7.6 Step five: feed-forward — the part that does the real work

Here is the key idea. A PID is **reactive**: it can only respond to an error
that has already happened. By the time the pressure has sagged enough for P to
notice, the patient has already felt it.

Feed-forward is **predictive**: it works out the RPM the blower *should* need,
from physics, before any error exists. The PID is then left with only the small
leftovers. This is why the gains above look modest — they are not carrying the
load.

There are several feed-forward terms. Which are active depends on the profile.

**The steady-state model (always the backbone).** The relationship between fan
speed and pressure is close to a square law, so inverting it gives:

```
RPM = 6141 × sqrt(pressure) + 2000
```

The 6141 came from a bench sweep of RPM against pressure — the scatter across
the whole 4–25 cmH₂O range was under 0.5%, which is unusually good for a fan.

The `+2000` offset matters and is worth understanding. The sweep was done with
the outlet open to atmosphere, where the blower moves almost no air. In a real
circuit the blower is always pushing the vent leak, so it needs more RPM than
the open-air curve predicts — on the bench, about 2000 RPM more at 8 cmH₂O.
That gap must be closed with an **additive offset**, not by raising the
coefficient, because the coefficient is multiplicative and would also stretch
every breath excursion and every RPM floor derived from it. An earlier attempt
to raise 6141 to 6820 made the expiratory pressure hump noticeably worse.

**Physics feed-forward (default ON).** The steady-state model above assumes no
flow. Extend it with the two things that change when air actually moves:

```
equivalent pressure = setpoint + hose loss(Q) + c_fan × Q²
RPM = 6141 × sqrt(equivalent pressure) + 2000
```

- *hose loss* reuses the same hose coefficients as tube compensation.
- *c_fan × Q²* is fan-curve droop: a fan produces less pressure when it is
  moving more air. Default 0.003, clamped at 0.005.

`c_fan` is the single biggest lever on the inspiratory dip. At the old value of
0.00012 the feed-forward anticipated only about 570 RPM of the ~2500 RPM a
breath needs, so the P term supplied the rest — reactively, after the pressure
had already dropped. To measure it properly: run the blower open-loop at a
fixed RPM, sweep the flow, and plot pressure against flow squared. The slope is
`c_fan`.

The flow that feeds this model is itself filtered, with different cutoffs for
inhale (12 Hz, needs to be quick) and exhale (10 Hz).

**Onset lead.** Waiting for the flow to be large means arriving late. This term
watches the *rate of change* of flow and adds an extra boost the moment a breath
starts, before the flow peaks — up to 2500 RPM. The rate signal has to be
filtered (30 ms) or its own noise produces a permanent phantom boost of a
couple of hundred RPM.

**Dip feed-forward (ON).** If the sensor pressure is falling faster than
2 cmH₂O/s, add RPM in proportion, up to 1500. This is a backstop that works
directly from pressure, so it still fires when the flow-based terms are confused
about which way the patient is breathing.

The threshold has to sit just above the noise floor, *not* above the breath.
The original 10 cmH₂O/s never fired once, because a 25 slm bench breath only
produces about 1.6 cmH₂O/s. If the expiratory baseline looks hairy, this
threshold has fallen into the noise — raise it to about 4.

**Rise feed-forward (ON).** The mirror image: when pressure is above the
setpoint *and* still climbing faster than the threshold, subtract RPM. Nothing
used to brake a rising pressure, and the physics model has nothing useful to say
about it, because the real disturbance is the patient emptying their lungs into
the circuit's compliance, which can only bleed out through the vent. So this
term brakes on the measured pressure rate instead of on any model. It is
deliberately not gated on breath phase, because the worst of the rise happens
exactly at flow reversal, where the phase detector is least certain.

**Expiratory flow feed-forward (ON).** Fitted from real captures: the leftover
over-pressure during exhalation is proportional to expiratory flow, at about
23 RPM per slm. But cross-correlation showed the pressure excess happens
200 ms *before* the flow — the flow is a correlate, not the cause. So this term
uses a phase lead (`Q + 0.2 × dQ/dt`) rather than instantaneous flow, or it
would arrive 200 ms late. It cannot remove the residual completely; flow only
explains about half the variance.

**Profiles.** Two presets rearrange which terms run:

- *CPAP Plateau*: square-law feed-forward only, everything else off, softer
  gains, heavier filtering. Gives the flattest pressure platform.
- *CPAP Hybrid*: physics feed-forward during inhalation, plateau-style hold
  during exhalation.

They are mutually exclusive, and `BlowerCtrl_EnforceFfMutualExclusion()` stops
incompatible combinations from being set by accident.

### 7.7 Step six: anti-windup, in three layers

Integral windup is the classic way a controller like this misbehaves: the output
is already at maximum, the error persists, so the integrator keeps growing, and
when the error finally clears the loop overshoots enormously because the
integrator has to unwind first.

There are three independent defences:

1. **Conditional integration.** Compute what the output *would* be. If it is
   saturated and the integrator step would push it further into saturation,
   skip the step.
2. **Headroom clamp.** Limit the integrator to the RPM range actually left over
   after the feed-forward, P and D terms have taken their share.
3. **Hard magnitude clamp.** An absolute ceiling on the integrator, scaled by
   the setpoint.

The integrator is also frozen while the slewed setpoint is still catching up to
the user setpoint, since the error during that climb is expected, not a fault of
the loop.

### 7.8 Step seven: clamp, filter, post

The terms are summed, clamped to the allowed RPM range (0 to 30000, where the
ceiling is the safety limit), and passed through an output filter — though that
filter is **disabled by default** (alpha 1.0), because the measurement filter
and the term slew limits already do the smoothing.

Then the number goes to CM4 via `BlowerIpc_CM7_SetSpeedRpmSilent()`. "Silent"
means no debug print; at 200 Hz the logging would flood the USB link. The ramp
time is deliberately zero: a high-rate control loop wants each new target
applied immediately, and any smoothing should happen inside the control law
where it can be reasoned about, not inside the motor controller.

### 7.9 The whole loop in one line

```
RPM = feed-forward(setpoint, flow)        <- predicts most of the answer
    + Kp × error × (hurry-up factor)      <- reacts to what is left
    + integral(filtered error)            <- removes the last steady bias
    − Kd × d(measurement)/dt              <- damps
    + dip/rise/expiratory-flow terms      <- catches the breath transients
  then clamped and posted to CM4
```

### 7.10 WM6850 reference mode

Setting `g_blower_ctrl_wm6850_mode = 1` switches the loop to imitate a
reference implementation: 100 Hz, no feed-forward, no setpoint slew, no gain
zones, no tube compensation, integrator starting at zero. It exists purely for
A/B comparison against known behaviour, and it bypasses most of this document.

---

## 8. EPR — expiratory pressure relief

`epr_ctrl.c`

Breathing out against pressure is the main comfort complaint with CPAP. EPR
drops the pressure by a fixed amount (1, 2 or 3 cmH₂O) while the patient
exhales.

The convention here follows AirSense: the baseline pressure is
`CPAP − relief`, and **full CPAP is delivered only during confirmed
inspiration**. That is worth repeating because it surprises people on the
bench: with EPR on and nobody breathing, the device sits at `CPAP − relief`,
not at CPAP. That is correct behaviour, not a bug.

Breath phase comes from a small state machine on patient flow:

```
IDLE -> (sustained inspiratory flow) -> INSPIRATION
     -> (sustained expiratory flow)  -> EXPIRATION
```

Both transitions need the flow to persist, and inspiration must reach a minimum
peak, so a cough or a leak wobble does not flip the pressure. Full CPAP is only
applied in the confirmed INSPIRATION state — the "pending" state proved far too
twitchy and caused visible pressure flapping.

Two types: **full time** (EPR always active) and **ramp only** (EPR during the
ramp, then plain CPAP).

At therapy start there is a *pressurisation hold*: EPR is pinned at the
expiratory pressure until the circuit has actually reached it and stayed there
for a second, plus a 5 second startup grace where breath detection is ignored.
This stops spin-up flow transients from being mistaken for breaths. Note that
this hold is deliberately **not** applied when resuming after a mask
reconnect — doing so held the resumed pressure a relief level low for five
seconds and made the recovery look stalled.

---

## 9. The therapy state machine

`therapy.c`

```
IDLE -> RAMP -> RUNNING -> STOPPING -> IDLE
            \      |
             \     v
              -> FAULT (needs stop + start to clear)
```

- **RAMP** (optional, 0–45 minutes): start at 4 cmH₂O and climb linearly to the
  session target, so the patient can fall asleep at a low pressure. The climb
  is so slow (~0.01 cmH₂O/s) that the PID's own slew limiter never notices it.
  The start pressure is a setting (#5, GUI "Ramp Start", `therapy set
  ramp_start`): 4 cmH₂O by default, up to the CPAP pressure or the AutoSet
  minimum.
  **AutoRamp** (#38, `therapy set ramp auto`, GUI ramp "Auto") instead holds
  the start pressure until sleep onset and then climbs at 1 cmH₂O/min
  (`apap set autoramp_rate`). Onset is ResMed's three signs: 30 stable breaths
  in a row (period within ±20 % and peak within ±25 % of their running
  averages), 3 obstructive apneas or hypopneas within 2 min, or 5 snore breaths
  in a row. Whatever happens, the climb starts early enough to reach the target
  30 min after start (`autoramp_max_min`). `therapy ramp status` shows the
  phase, the cause and the stable-breath count.
  During any ramp the APAP detectors score (events in the ramp count, as on the
  AirSense) but titration is frozen: the APAP target is put back every tick, so
  AutoSet starts from its minimum when the ramp ends.
- **RUNNING**: hold the target. In CPAP that is a fixed number; in Autoset the
  APAP algorithm moves it.
- **Pre-therapy self-test** (#83, `precheck.c`): every start runs it first,
  from values the sensor loop already keeps. A hard failure refuses the start
  (FAULT "Self-test: <what>", FAULTS.CSV code 6): the mask-pressure, flow or
  ambient sensor frozen (its raw word unchanged for 500 ms / 500 ms / 3 s -
  a dead link, a stopped conversion and a stuck sensor all look the same),
  mask pressure outside -10..+50 cmH₂O or more than 3 cmH₂O off zero with the
  blower stopped, a NaN or saturated flow reading, ambient outside
  500..1150 mbar, the CM4 status counter not advancing for 500 ms, a drive
  fault active, or the motor supply outside 20..28 V. Warnings (start
  allowed): the analog pressure sensor more than 2 cmH₂O from the mask sensor,
  no SD card, clock not set, humidity / tube-temperature sensor not reading.
  Then a **spin check** (state PRECHECK, ~2-3 s): the blower runs open-loop at
  12000 rpm (about 3 cmH₂O); once it has held that speed for 0.5 s, device
  pressure, flow and speed are sampled for 1 s. The speed must be within
  1500 rpm, the pressure must sit on the fan map for that speed and flow
  (P = a N² + b N Q + c Q|Q| + d Q, within 0.8 cmH₂O + 15 %) - the pressure
  and flow sensors and the rotor speed have to agree, whatever the mask or leak
  load - and neither sensor may be flat or wildly noisy. A pass hands the
  spinning motor to the pressure loop without a stop; a failure latches the
  self-test fault. `therapy precheck spin 0` / `map 0` switch the spin check /
  its map comparison off (the map is the WM6850's).
  `therapy precheck` prints the check, its readings and the last spin;
  `therapy precheck test <what>` forces a failure for testing.
- **STOPPING**: a two-second spin-down.
- **FAULT**: blower disabled and stopped. There is no automatic recovery.

The session target is snapshotted at start so that GUI changes cannot yank a
running ramp around, but the running target *is* re-read each tick so a
deliberate change during therapy takes effect.

The state machine also collects session statistics — a pressure histogram
producing the median, 90th and 95th percentiles, run time, mask-off and
reconnect counts.

---

## 10. Mask-off detection

`therapy.c`. This is under active development; the behaviour below is current
as of this document.

**The problem.** If the patient takes the mask off mid-session, the circuit is
open. The pressure collapses, so the controller does the only thing it knows
and drives the blower to maximum — loud, pointless, and it makes reconnecting
unpleasant. So the device should notice, park the blower at a low speed, and
resume properly when the mask goes back on.

**Parking.** On detection: disable the PID, ramp the motor down to 6000 RPM
(soft, to avoid the deceleration pumping the DC bus into over-voltage), freeze
leak adaptation, and hold. 6000 RPM is chosen so that there is still enough
airflow to *sense* a reconnection.

**Detecting the disconnect.** Three independent paths, any of which can fire:

1. **Flow path.** Total flow above 45 slm for 600 ms (300 ms if it is very
   high), together with collapsed pressure. Fast and obvious for a hose
   disconnect.
2. **Leak-ratio path.** The Vivo estimator's 10-second mean ratio above a
   disconnect limit for 8 seconds. Slower, catches gradual seal failures.
3. **Pressure-only path.** Commanded RPM at or above 12000 while both the
   sensor and the mask estimate sit below `min(0.55 × setpoint, 4.0)` cmH₂O,
   for 2 seconds. This reads no flow at all, so it still works when the flow
   signal is unusable or when the event is too small for the 45 slm threshold.

All three require the pressure to have actually collapsed, so a strong
inhalation cannot trigger a mask-off.

**Why the third path exists.** A mask lifted off the face is a much smaller
event than a hose disconnect: the vent orifice limits the flow. Measured on this
bench at a setpoint of 8, the circuit still holds about 2.5 cmH₂O and passes
only about 20 slm with the blower pinned at 30000 RPM. That is below the 45 slm
flow threshold and above a fixed low-pressure floor, so neither of the first two
paths fires. Hence a threshold that scales with the pressure being asked for.

**Detecting the reconnect.** At 6000 RPM, an open hose reads near zero pressure
and a sealed mask tips up to roughly 0.7 cmH₂O. So the device learns an
open-circuit reference once the deceleration has settled, then watches for the
pressure to rise above it. There are two fallbacks:

- *Sustained flow collapse*, for the case where the mask went back on before
  the deceleration finished — the reference is then the sealed pressure itself,
  and no rise can ever appear above it.
- *A 60 second watchdog* that re-probes at therapy pressure, so a mask-off can
  never latch for the rest of the session no matter what the sensors do.

**Resuming.** Re-seed the controller at the steady-state RPM for the target,
floor the RPM for 5 seconds so it cannot collapse, suppress patient-flow
feed-forward for 3 seconds (the leak estimator is re-seeding and briefly
mistakes hose-filling flow for the patient), reset the breath state, and give
the safety supervisor a 12 second grace while the pressure climbs back.

**Known limitation.** Detection is reliable from about 5 cmH₂O upwards and gets
better as the setpoint rises. At the very bottom of the range it degrades,
because the open-circuit pressure is set by the blower's speed limit and the
circuit — not by the setpoint — so at a setpoint of 4 the difference between
"mask on" and "mask off" is only about 1.5 cmH₂O. A pressure-based test cannot
reliably separate those. The robust fix is to compare commanded RPM against the
RPM the square-law model says *should* deliver this pressure, which is
independent of the pressure scale; that is not implemented yet.

---

## 11. APAP / Autoset

`apap_ctrl.c`

In Autoset mode the device chooses its own pressure between a minimum and a
maximum. This is a functional prototype, not a clone of anyone's algorithm, and
the thresholds are heuristics awaiting clinical validation.

The loop:

1. Start at the minimum pressure.
2. **Monitor** while breathing continues. Look for flow limitation (a flattened
   or peak-dropped inspiratory shape), hypopnea, and snore. Each raises the
   pressure by a small or moderate step depending on severity.
3. **No breath for about 4 seconds** → start the forced-oscillation test
   (below). Score the event only if it lasts around 10 seconds, so that a small
   obstructed gasp does not abort the measurement.
4. Classify: if the pressure oscillation gets through but the flow oscillation
   does not, the airway is closed — an obstructive apnoea, so raise the
   pressure in duration-based steps. If flow *is* transmitted, the airway is
   open and the patient simply is not breathing — a central apnoea, and raising
   pressure would not help, so it does not.
5. **High leak blocks every pressure rise.** If the circuit is leaking, the
   event detectors cannot be trusted. Once the leak has been HIGH for 3 s it
   also stops scoring (#19): no events are counted or annotated, FL / snore
   time and the PLD indices stop, the CSR detector closes its span, and the
   time is left out of the AHI denominator. Scoring resumes 10 s after the
   leak drops (the flow estimate needs a few breaths). The excluded time and
   the number of periods go to SUMMARY.CSV (`leak_excl_s`, `leak_excl_n`), so
   the exclusion is visible rather than silent.
6. After a quiet, stable period, drift the pressure slowly back down.

A "Soft" response setting makes all the rises gentler.

### Forced oscillation (FOT)

`blower_osc.c`

To tell an open airway from a blocked one when the patient is not breathing, the
device adds a small sine wave (about 4 Hz, around 1 cmH₂O peak) to the pressure
setpoint. The PID reproduces it, so the pressure in the circuit gently
oscillates.

If the airway is open, that oscillation produces a matching oscillation in the
flow. If it is blocked, the pressure wiggles but the flow does not. The ratio of
the two is the airway impedance.

Implementation details that matter: the phase is kept as an accumulator in
[0, 1) so it cannot drift after hours of running; the sine always starts at a
zero crossing so switching it on does not step the pressure; amplitude and
frequency are clamped so a typo in a watch window cannot harm a patient.

The FOT signal must be scored from the **raw** flow, not the published patient
flow — the breath-detection filters on the patient-flow path remove the 4 Hz
carrier entirely.

---

## 12. The safety supervisor

`safety_supervisor.c`, its own task, highest priority.

It watches five things and latches a fault on any of them. A fault disables the
controller, stops the motor, and requires an explicit stop-then-start.

| Fault | Condition | Meaning |
|---|---|---|
| Low pressure at high RPM | Sensor below 1.5 cmH₂O while commanding over 12000 RPM, for 4 s | Hose off, sensor dead, or huge leak |
| Over pressure | More than 5 cmH₂O above target for 3 s | Sensor stuck high or blockage |
| Sensor out of range | Outside −10…+50 cmH₂O for 2 s | Electrical fault |
| RPM saturated | Pinned at the 30000 RPM limit without reaching target, for 6 s | Controller cannot deliver — leak too big or blower undersized |
| CM4 heartbeat | Mailbox status counter stopped for 100 ms | Motor core dead |

Three exemptions are built in, and they exist for real reasons:

- **A start grace** (10 s) and a **reconnect grace** (12 s), because the
  pressure is legitimately low while the circuit fills.
- **While mask-off is active**, because parking at 6000 RPM with near-zero
  pressure is intended behaviour, not a fault.
- **While a mask-off detector is arming**, because the fault timers can start
  counting earlier than the detectors (a large leak pins the RPM well before a
  full disconnect drops the pressure to the floor), and without this the fault
  would stop the motor while the detector was still confirming.

That last exemption is narrow on purpose. A genuine "cannot reach target" leak
never puts the pressure low enough to arm mask-off, so it still faults exactly
as intended.

---

## 13. The motor side

`CM4/Core/Src/blower_ipc_cm4.c` plus the ST motor control library.

CM4 runs field-oriented control, which is the standard way to drive a brushless
motor smoothly: it measures the phase currents, works out where the rotor is,
and computes the voltages that produce the torque wanted with the least ripple.
The details live in ST's generated library; this firmware mostly configures it
and feeds it a speed target.

Two pieces of local logic matter:

**Auto-arming.** CM7 often posts a speed command without a preceding start, so a
`SET_SPEED` arriving while the motor is idle or in a cleared fault will
acknowledge the fault and start the motor itself. Without this, a lone speed
command could leave the rotor coasting.

**The regenerative brake.** Slowing a spinning blower turns the motor into a
generator and pushes energy back into the DC bus. The over-voltage trip is at
24.9 V, which is not far above the normal rail, so a fast deceleration can trip
it and kill the PWM — leaving the rotor coasting exactly when control is wanted.
A brake resistor on a PWM output dumps that energy: it engages above 22 V,
scales with how far over the threshold the bus is, and releases at 21 V. Large
planned decelerations (like the mask-off park) pre-engage it.

---

## 14. Humidifier

`humidifier.c`, on the slow ClimateTask.

A 25 W coil heats about 100 mL of water; an SHT4x measures humidity and air
temperature above the bowl. Full power reaches roughly 50 °C in 10–15 minutes,
so the thermal time constant is minutes with significant dead time.

The loop is therefore deliberately slow and unambitious: a PI controller at
1 Hz, conditional-integration anti-windup, a slew limit on the PWM output, hard
duty clamps, an over-temperature trip, and a "heater stuck on" watchdog that
faults if the PWM sits near maximum for a long time without the humidity rising.

Disabled at boot.

---

## 15. Tuning and observability

Nearly every constant mentioned here is a `volatile` global, which means:

- You can watch and change it live in the Keil watch window or STM Studio.
- You can change many of them over USB with `therapy set ...`,
  `blower ...` and similar commands (see `telemetry_stream.c` for the list).
- The names follow a pattern: `g_<module>_<thing>`, for example
  `g_blower_ctrl_kp_low`, `g_therapy_mask_off_enter_slm`.

**Telemetry.** A 50 Hz stream over USB CDC carries sensor pressure, mask
pressure, patient flow, commanded RPM and actual RPM. `pc_tools/cpap_telemetry_plot.py`
plots it live with a status panel. A 100 Hz stream goes to an ESP32 over UART
for MQTT. Sessions can also be recorded to SD card in EDF format
(see [`SESSION_EDF_LOGGING.md`](SESSION_EDF_LOGGING.md)).

**Suggested PID bring-up order**, if you ever need to retune from scratch:

1. Set Ki and Kd to zero. Raise Kp until you see mild oscillation, then back off
   about 30%.
2. Raise Ki until steady-state error clears in 200–500 ms.
3. Add a small Kd (5–30) only if there is overshoot you cannot remove by
   lowering Kp.

Do it with the feed-forward configured the way you intend to ship it. The gains
assume feed-forward is carrying the steady-state load; with it off you will need
two to three times more Kp and Ki.

---

## 16. Gotchas worth knowing before you debug something

- **The two filter alphas mean opposite things.**
  `g_blower_ctrl_meas_filter_alpha` weights the *old* value (0.45 = light
  smoothing). `g_blower_ctrl_out_filter_alpha` weights the *new* value
  (1.0 = no filtering at all). This has caught people out.

- **`BlowerCtrl_GetOutputRpm()` freezes when the PID is disabled.** During
  mask-off or a fault, `BlowerCtrl_Step()` returns before it updates the output
  registers, so that value keeps whatever it last commanded. Mask-off now
  publishes the parked RPM explicitly to keep it honest, but anything else that
  disables the loop will leave a stale number on the screen and in the logs.

- **Patient flow is a derived number.** If it does something surprising, check
  total flow first. Most "flow did something impossible" moments are the leak
  estimator adapting.

- **The mask pressure is an estimate, not a measurement.** Nothing in the device
  actually measures pressure at the mask.

- **Total flow, not excess flow, drives mask-off.** This is deliberate. Excess
  flow depends on the leak and vent estimates, which are exactly the things that
  go wrong during a disconnect.

- **Setpoint zones schedule on the setpoint, not the measurement**, so during a
  big transient the gains in use are the ones for where you are *going*, not
  where you *are*.

- **A dropped sensor sample repeats the previous value.** Anything that
  differentiates must check `g_ps_sample_fresh`, or it will see a zero
  derivative that is not real.

---

## 17. Where to look in the code

| Topic | File |
|---|---|
| Pressure PID and feed-forward | `CM7/Core/Src/blower_ctrl.c` |
| Mask pressure estimate | `CM7/Core/Src/tube_comp.c` |
| Learn circuit (tube resistance) | `CM7/Core/Src/circuit_learn.c` |
| Flow conditioning | `CM7/Core/Src/flow_sensor.c` |
| Leak estimation | `CM7/Core/Src/leak_estimator.c`, `leak_estimator_vivo.c` |
| Therapy states, mask-off | `CM7/Core/Src/therapy.c` |
| EPR | `CM7/Core/Src/epr_ctrl.c` |
| APAP / Autoset | `CM7/Core/Src/apap_ctrl.c` |
| Forced oscillation | `CM7/Core/Src/blower_osc.c` |
| Safety | `CM7/Core/Src/safety_supervisor.c` |
| Cross-core mailbox | `Common/Inc/blower_ipc.h`, `CM7/.../blower_ipc_cm7.c`, `CM4/.../blower_ipc_cm4.c` |
| Humidifier | `CM7/Core/Src/humidifier.c` |
| USB telemetry and commands | `CM7/Core/Src/telemetry_stream.c` |

The header files carry the design rationale — in this codebase they are worth
reading before the implementation.
