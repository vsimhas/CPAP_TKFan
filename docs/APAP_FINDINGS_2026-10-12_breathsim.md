# APAP findings from the BreathSimulator — 12 October 2026

Bench results from four AutoSet trials, explained against `CM7/Core/Src/apap_ctrl.c`.
Written from the BreathSimulator session.

This is **evidence and analysis, not authorization to change firmware, flash, or commit.**
Every code reference is to `apap_ctrl.c` at repo HEAD `0589158`. The build actually
running on the CPAP during the trials was not independently verified against that HEAD.

> **Status after re-testing commit `5ff3544`** (same day, see
> [Re-test of the fixes in `5ff3544`](#re-test-of-the-fixes-in-5ff3544)):
> findings 1 (apnea timing) and 5 (AHI) are **fixed**.
> Finding 2 (hypopnea duration) works but now **counts a long hypopnea twice**.
> Findings 3 and 4 are **not fixed**, and the 55 % hypopnea got **worse**. All of
> those trace back to one cause: the baseline is zeroed on every NO_BREATH entry.
> Two code changes are proposed below, with line numbers against `5ff3544`.

> **Status after re-testing commit `4ab7a0c`** (19 October, see
> [Re-test of R1 and R2 in `4ab7a0c`](#re-test-of-r1-and-r2-in-4ab7a0c-19-october-2026)):
> **R1 and R2 are fixed.** 50 % and 55 % hypopneas each score one Hyp, and there's no
> false FL after the arousal breaths.
> Two problems remain:
> - **R4:** the 60 % hypopnea now scores **three central apneas**, one per FOT burst.
> - **R5:** apnea detection is late, or the apnea is missed altogether, because a
>   breath that didn't happen interrupts NO_BREATH or FOT. Not in the `4ab7a0c`
>   diff. A 50 Hz capture found two causes:
>   - brief 6–11 slm, ~0.15 s flow spikes that pass the EPR breath test;
>   - the VIVO leak estimate drifting during FOT, so FOT's own flow counts as a
>     breath.
>
> Proposed changes: **R4** (one score per low-flow event), **R5a** (an episode ends
> only on a breath of plausible length and volume) and **R5b** (freeze leak
> adaptation during NO_BREATH/FOT). Line numbers are against `4ab7a0c`.
>
> **Update, 19 October: the spike source is found (R6).** The spikes are not air. They
> are torn AMS5935 flow-sensor reads at the sensor/MCU clock beat:
> - strictly periodic, every **17.75 s**;
> - seen by the CPAP only, not by the external or simulator sensors;
> - confirmed by the CPAP's own `flow spike` gap of 17 740 ms.
>
> Each needle also makes the pressure loop push a small real puff of air. Fix it at
> the sensor read (R6).
>
> **Update, trial 7b (R4/R5 build):**
> - **R5a and R5b work.** Every 10 s and 20 s apnea was scored CA, FOT started ~6–7 s
>   in and ran to the end of each apnea even through a needle, and the leak estimate
>   stayed flat.
> - **R4 is untested.** The simulator couldn't produce the deep hypopneas in that
>   mode.
> - Trial 7 itself was void, from a simulator start-up deadlock (now fixed in the
>   BreathSimulator firmware).
>
> **Update, trial 7c (pressure FF, fixed simulator): the full test.**
> - **R5 and R4 work.** All 7 apneas were scored CA. The 55 % hypopnea's CA was
>   withdrawn in favour of one Hyp. The 60 % hypopnea is one event, not three.
> - **Two new problems (R7):**
>   - **R7a:** R5b froze an unseeded start-up leak estimate (~23 vs ~12.7 slm), so
>     FOT ran ~56 s while the patient breathed normally.
>   - **R7b:** under FOT the 4 Hz ripple cuts breaths short; R5a rejects the
>     fragments, and the 60 % hypopnea became a CA instead of a Hyp.
> - Fix R7b by notching 4 Hz out of the flow the breath FSM sees while FOT is active.
>
> **Update, trial 7d (R6 + R7 build):**
> - **R6 and R7a fixed.** Needles went from 9 to 1, there is no long start-up FOT,
>   the apneas are all correct, and the 50 % and 55 % hypopneas score one Hyp each.
> - **Still wrong:** the 55 % hypopnea gets an extra CA at its tail (Hyp then CA),
>   and the 60 % hypopnea is still a CA.
> - **Cause (R8):** the CPAP sees the weak breaths at about half size. The leak
>   estimate reads ~2 slm high during hypopneas, so a 226 mL breath shows as about
>   100 mL outside FOT and 40–70 mL in FOT. That straddles R5a's 80 mL limit.
> - **Proposed:** lower or relativise R5a's volume limit, and add the mirror of R4's
>   reclassification (Hyp then CA in one event).
>
> **Update, trial 7e (R8 build):**
> - The 55 % hypopnea is now one Hyp, and OSA-likely is down from 39 to 2.
> - **Only the 60 % hypopnea is still wrong** (FOT → CA). Its first breath reaches the
>   CPAP at 2.8 slm, below the 4 slm breath threshold, because the CPAP's patient
>   flow drops ~3 slm as soon as breaths shrink.
> - **R9:** find whether that offset is the CPAP's leak estimator or the simulator
>   box's leak, with a three-sensor bench capture, before changing code again.
>
> **R9 result:** mostly the CPAP.
> - The real box leak drops only ~0.7 slm in a hypopnea.
> - The CPAP's patient-flow mean drops 2.4 slm (60 %) and 1.3 slm (50 %). It does so
>   **instantly on the first weak breath** and recovers on the first normal one, so it
>   is not the leak estimate adapting.
> - The CPAP sees a 60 % hypopnea breath at 52 % of its peak; the external sensor sees
>   it at 93 %.
> - Fix: make the APAP's breath detection relative to the tracked flow baseline, and
>   find the in-breath non-linearity (next capture: log the CPAP's total flow).
>
> **Update, trial 7f (R9 build):**
> - Needles are fully gone (0) and the apneas are still all correct.
> - **The hypopneas got worse:** the 55 % one now gets a 28 s FOT and a tail CA, the
>   60 % one is still a CA, and OSA-likely is back to 40 flags.
> - Cause: the EPR flow baseline climbs onto weak breaths (+1…+2 slm), because their
>   flow sits inside its 5 slm deadband. The relative test then shrinks them further.
> - **R10:** turn `epr_rel_baseline` off again (runtime A/B), and fix the CPAP's ~4 slm
>   low inspiratory reading at its source.
>
> **R10 result (total-flow capture):** the error is in the subtracted model, not the
> sensor.
> - The CPAP's total flow is right: vent-as-measured is flat to 0.3 slm, and a 60 %
>   hypopnea breath peaks at 7.6 slm vs 7.7 on the external sensor.
> - The vent + leak it subtracts rises 1.5 slm in weak breathing while the real leak
>   falls 0.7, so the model is ~2.2 slm off, per breath.
> - Look at VIVO's per-breath level (`calcLeakLevelFromRestVolume`, phase split from
>   `updateBreathFsm`).
> - Or detect APAP breaths on total-flow excursions, which carry the hypopneas
>   faithfully.
>
> **Correction (20 October):** the stage was the patient-flow pause-DC zeroing
> (found by the CPAP thread), not VIVO's per-breath level.
>
> **Status after trial 7g (R10 build, 20 October): all findings resolved.**
> - Patient flow is level in weak breathing.
> - All 7 apneas are scored CA with FOT ~7 s in, the 5 s apnea is not scored, and
>   there are no early exits.
> - The 55 % and 60 % hypopneas score **one Hyp each with no FOT**, the first time
>   for the 60 % one.
> - No needles, OSA-likely 2, no false CAs.
> - The 50 % hypopnea now reads at 54 % of baseline, above the 0.50 Hyp threshold, so
>   it scores FL only. That is correct by the CPAP's rule; the definition is the
>   team's choice.
>
> **Update, trial 8 (20 October): a new blocker (R11).**
> - With the harness fixed to start therapy on an already-breathing patient, the
>   start-up CA is gone.
> - But after the first apnea, **FOT never ended on normal 500 mL breathing**: 14 min
>   of oscillation, nothing scored, > 0.8 A in 1 010 samples.
> - The CPAP's patient flow was normal, so the block is inside the firmware's
>   breath-stamp / recovery path. Proposed: an `apap diag` command and a reproduction.
>
> **Update, trial 8b (R11 build): fixed.**
> - Therapy starting on a breathing patient is clean: no start-up CA, and FOT always
>   ends on breathing (longest 12 s).
> - All 7 apneas are CA, 55 % → one Hyp, 50 % → FL (by the rule), OSA-likely 0.
> - **One remaining fault (R12):** the 60 % hypopnea, at 45–49 % of baseline, was
>   counted twice. When the 20 s hold expired, a one-breath wobble over 0.50 had
>   already re-armed it. Fix: hysteresis on the Hyp re-arm.
>
> **Status after the R12 build (20 October): all scoring correct.**
> - Clean start on a breathing patient; FOT always ends (max 13 s).
> - All 7 apneas scored CA; the 5 s apnea not scored.
> - 55 % and 60 % hypopneas score **one Hyp each**; 50 % scores FL only (by the
>   rule).
> - Totals CA 7 + Hyp 2, exactly as expected.
> - Remaining: the Hyp-definition choice, residual harmless needles, and committing
>   the CPAP work.
>
> **Status after the R13 build (23 October, see
> [R13 result](#r13-result-whole-inspiration-shape-23-october)): flow limitation is
> judged on the whole inspiration.**
> - Plateau 90 %, M 90 % and plateau 60 % each score **one FL episode**, with the flag
>   on 92 / 71 / 92 % of the block.
> - Normal breathing reads flatness ≤ 0.66 against the new 0.72 line (it sat on the
>   old 0.78 line before), and never turns FL on.
> - Still open from 20 October: the raise rate (~0.5 cmH₂O/min on FL), the 120 s wait
>   before lowering, and the FL amplitude arm on central hypopneas.
>
> **Status after the raise / lowering build (23 October, see
> [Raise rate and lowering wait](#raise-rate-and-lowering-wait-23-october)): titration
> now follows the AirSense 11 on the flattened-breath script.**
> - On FL the target rises at ~2–3 cmH₂O/min (was ~0.5) and reaches the 12 cmH₂O max
>   inside the plateau 90 % block, as the AirSense does.
> - Lowering starts ~20 s after FL clears (was 120 s), at 0.3 cmH₂O/min (was 0.48).
>   There is no hunting in the 2-minute gaps.
> - Still open: the FL amplitude arm on central hypopneas. Ours also raises a little on
>   plateau 60 %, where the AirSense drifts down.

## Summary

The APAP does what it was built to do on the clear-cut cases. It detected flow
limitation, scored apneas of 30 s and longer as central every time, raised pressure for
flow limitation and hypopnea, and did not raise it for central apneas. Breath-phase
polarity was correct throughout.

Five things need attention. Three of them come straight from the code:

1. **Apneas shorter than about 22 s cannot be scored.** The 12 s "recent breath"
   window and the 10 s minimum apnea time add up, because the episode clock only
   starts once the 12 s window has expired. The clinical minimum is 10 s.
2. **The hypopnea duration counts inspiratory time only.** The 10 s threshold therefore
   needs at least about 20 s of actual hypopnea at I:E 1:1. That likely explains why a
   50 % hypopnea scores only at its very end, and a deeper 55 % one never scores.
3. **The breath baseline only updates on breaths that aren't limited.** Large arousal
   breaths push it up, and normal breaths that then read as "limited" can't pull it back
   down. The result is a prolonged false flow-limitation call.
4. **The reported AHI leaves out central apneas.** It is (OA + Hyp) / h, so it read
   **0.00** in a run with three scored central apneas.
5. **FOT oscillation drives the CPAP supply current to about 1.05–1.12 A** in brief peaks.
   Breathing alone never exceeded 0.78 A. This matters for the 2026-10-05 fuse limits.

## Rig and method

```
CPAP (COM13) -> 22 mm hose -> vent -> external sensor -> collapsible chamber
            -> BreathSimulator (COM15, its blower is the patient's muscle) -> 2 L lung
```

- **CPAP:** AutoSet, range 8–12 cmH₂O, EPR off, ramp 0. The range was held near
  10 cmH₂O because the bench current guard (0.8 A sustained) stayed in place.
- **Chamber:** the squeeze blower was off. At CPAP pressure the tube stays open, so
  every simulated apnea is **central-looking**, with an open airway.
- **Simulator:** closed-loop flow control with pressure-compensated feed-forward,
  12 bpm, I:E 1:1 (Ti 2.5 s), 500 mL normal breaths with a peak of about 20.3 slm.
- **Ground truth:** the simulator marks every event on the breath boundary where it
  actually took effect (`E,` lines). It also reports per breath the delivered volume,
  measured peak flow and measured flattening index (inspiratory mean/peak, the same
  quantity the APAP thresholds at 0.78) (`V,` lines).
- **CPAP side:** `therapy apap events` was polled about every 4 s for counters and live
  flags, `therapy status` likewise for state, target, oscillation and OSA call, and the
  `B` line (`blower set telem_motor 1`) at about 25 Hz for supply current, using
  P = 1.5·(vq·iq + vd·id).

CPAP lines arrive about 2 s late and are sampled every ~4 s, so timings below are good
to about ±4 s.

## What was delivered, and what the APAP scored

| stimulus | delivered (simulator sensor) | APAP response | runs |
|---|---|---|---|
| normal breathing | 500 mL, peak 20.3 slm, flat 0.59 | nothing | all |
| flow limitation (plateau, sev 1.0) | 498 mL, peak 14.2, **flat 0.84** | FL flag ~25 s in, FL counted, target 8.0 → 8.6 | 1 |
| hypopnea 30 % | 350 mL, peak 72 % of normal | nothing | 1 |
| hypopnea 45 % | 274 mL, peak 57 % | FL only | 1 |
| hypopnea 50 % | 250 mL, peak 51 % | FL, then **Hyp scored at the event's end** | 2/2 |
| hypopnea 55 % | 224 mL, peak 43–44 % | **FL only — no Hyp** | **3/3** |
| hypopnea 60 % | 204 mL, peak 39 % | oscillation → **central apnea** | 2/2 |
| apnea 20 s | ~0 | oscillation from ~14 s in; CA scored | **3/4** |
| apnea 30 s / 40 s | ~0 | oscillation from 14.3 / 16.8 s in; CA scored | 2/2 |
| normal breaths after two 1.5× arousal breaths | 500 mL | **false FL**, flag on ~90 s | 1 (0/1 with arousals off) |

No OSA calls were made in any run, as expected with an open airway.

## Findings

### 1. The effective minimum apnea is about 22 s, not 10 s

**Code:**

- In MONITOR, the state only moves to NO_BREATH when `recentBreath()` (line 235) has
  seen no breath for `g_apap_breath_window_s` = **12 s** (line 26). Entering NO_BREATH
  resets `s_episode_ms` to 0 (line 1028).
- NO_BREATH waits `g_apap_no_breath_timeout_s` = **4 s** (lines 25, 1052) before starting
  FOT, so **FOT begins about 16 s after the last breath.**
- In OSCILLATE, an event is scored only on an evaluation tick (every 2 s of FOT), and
  only once `s_episode_ms >= apnea_min_ms` (10 s) (line 1087).
- `s_episode_ms` started at the 12 s mark, so **scoring needs about 12 + 10 = 22 s
  without a breath.**

**Bench:** FOT began 14.2, 14.3 and 16.8 s after apnea onset. The previous inspiration
ended about 2.5 s before the onset marker, which puts FOT at about 16–19 s after the last
breath. A 20 s apnea gives the APAP roughly 23.5–24.5 s with no breath, only 1.5–2.5 s
over the threshold, and was scored in 3 of 4 runs.

**Consequence:** apneas of about 10–20 s are invisible to the APAP: no FOT, no
classification, no score. Most of the 4 s figure quoted for "no breath → FOT" in earlier
notes is really the 12 s window.

**Suggested direction:** measure the apnea episode from the last detected breath rather
than from NO_BREATH entry, or subtract the window from `apnea_min`. It's also worth
checking whether a 12 s window is needed at all: it doubles as the slow-breathing guard.

### 2. Hypopnea duration accumulates inspiratory time only

**Code:**

- `finalizeInspBreath()` is called with `s_insp_ms`, the detected *inspiration* length
  (line 587), and adds it to `s_hyp_limited_ms` (line 535).
- The hypopnea is scored when `s_hyp_limited_ms >= 10 s` and the hypopnea streak is at
  least 2 breaths.
- At I:E 1:1, 10 s of inspiration is at least about 20 s of hypopnea, and more if the
  breath detector finds shorter inspirations on weak breaths.
- The fallback of `3000U` when no time is available suggests the author intended a
  per-breath period.

**Bench:** this fits both results.

- The 50 % hypopnea (8 breaths over 40 s) scored only **at the end of the event**: by
  the last breath, the inspiratory time had only just reached 10 s.
- The deeper 55 % hypopnea **never scored, in 3 of 3 runs.**

The second point rests on an assumption I could not measure: that the weaker 55 % breaths
register shorter detected inspirations. The CPAP doesn't report its detected inspiration
times.

**Suggested direction:** accumulate the breath period (time between breath starts), or
the wall-clock time since the first limited breath.

### 3. The hypopnea/apnea boundary sits right where hypopneas are

- **Code:** NO_BREATH exits only on a `recoveryBreath()` (line 244). That is a breath with
  a peak of at least `g_apap_resume_min_peak_slm` = **10 slm** (line 27), a fixed value,
  not relative to the baseline.
- **Bench:** with normal peaks around 20 slm, the 60 % hypopnea (peaks around 8 slm) was
  routed through NO_BREATH and FOT and **scored as a central apnea**, in 2 of 2 runs.
- **Inference:** that it got there because the breath detector (`Epr_HasRecentBreath`)
  doesn't count breaths that weak. I didn't examine its threshold.
- **Consequence:** whether a deep hypopnea counts as a breath depends on a fixed flow
  threshold, so the outcome changes with patient size. Combined with finding 2, only a
  narrow band of depths (around 50 % here) is ever scored as a hypopnea. Shallower ones
  score as flow limitation; deeper ones as central apnea.
- **Suggested direction:** make the recovery-breath and breath-detection thresholds
  relative to the running peak baseline.

### 4. The baseline can't come down once it has been inflated

**Code:**

- `s_fl_baseline_slm` is updated, with alpha 0.25, only in the **non-limited** branch
  (line 521).
- A large breath that isn't limited, such as an arousal breath, pulls it up.
- Normal breaths that then fall below 70 % of the inflated value are classed as limited,
  so they can't update it, and the baseline stays high.

**Bench:**

- With two 1.5× recovery breaths after an apnea, the normal breaths that followed were
  scored as flow-limited, and the FL flag stayed on for about 90 s.
- The same sequence with arousal breaths off gave no FL call.

That is one run each way: supportive, not conclusive. Real arousal breaths after a real
apnea would trigger this.

**Suggested direction:** reject breaths well above the baseline (more than about 1.4×) from
the update, and/or let the baseline drift slowly downward during a long limited streak.

### 5. The reported AHI leaves out central apneas

- **Code:** `Apap_GetSessionAhi()` returns `(OA + Hyp) / h` (line 956);
  `Apap_GetSessionRdi()` returns `(OA + Hyp + FL) / h` (line 963).
- **Bench:** **AHI 0.00** in a run with **CA 3**. The earlier values, 8.30 and 5.68,
  match one hypopnea per session length.
- **Consequence:** clinical AHI counts all apneas, central included. If an obstructive-only
  index is intended, it should be labelled as one; otherwise CA belongs in the sum.

### 6. FOT oscillation is the CPAP's current peak

- **Bench:** supply current was logged at about 25 Hz across two runs. Peaks above 1 A
  (1.05–1.12 A) happened **only while oscillating**. Ordinary breathing reached at most
  0.78 A, with no sample above 0.8 A in about 27 000 breathing samples.
- The FOT setpoint swing is ±1 cmH₂O at 4 Hz (`g_apap_osc_amplitude_cmh2o`, line 29),
  with a clamp at 1.75.
- The peaks are brief (11 samples above 0.8 A in about 65 s of FOT), and a 0.8 A guard
  sustained for 0.7 s never tripped, even through 29 s of FOT.
- **Relevance:** not a fault. But FOT is the CPAP's most power-hungry state, and the
  2026-10-05 fuse event makes this worth recording against the supply sizing.

## What the simulator can and can't test, as of today

**Can:**

- Flow limitation, with a measured flattening index. Plateau at severity 1.0 gives about
  0.83 at CPAP 10, normal breaths about 0.59. The M shape at 0.9 gives about 0.73, a
  morphology that is flow-limited but below the 0.78 threshold.
- Central apneas, with a true zero-flow apnea.
- Hypopneas at a chosen depth.
- Arousal breaths up to 1.5× (fully delivered).
- Per-breath ground truth for automated scoring.

**Can't:**

- **Obstructive apneas.** The chamber can't close the airway. The earlier chamber tests
  also found that the vent's own oscillating flow (about 2 slm p-p) exceeds the 2.0 slm
  OSA threshold, so even a closed airway might not be called OSA. That is still untested.
- **Snore.** Commanded, but the simulator can't confirm it above about 45 Hz: both flow
  sensors sample at about 100 Hz.

**Test-design notes:**

- Use apneas of at least 30 s if they must score; that's finding 1.
- Expect only around 50 % hypopneas to be scored as hypopneas; that's findings 2–3.
- Turn arousal breaths off unless the test is about them; that's finding 4.

## Suggested next tests

1. **Apnea length 10–25 s in 3 s steps,** to pin down the scoring edge. The prediction
   from the code is about 22 s from the last breath.
2. **Hypopnea depth 50/55 % at I:E 1:2** (shorter inspirations). If finding 2 is right,
   the 50 % hypopnea should stop scoring too.
3. **Arousal gain 1.2× vs 1.5×,** to find where the baseline inflation starts producing
   false FL calls.
4. **Any change to `apap_ctrl.c`** can be checked against the same scripted sequences.
   The simulator runs them unattended and scores them per breath.

## Re-test of the fixes in `5ff3544`

Trial 5 used the same rig, CPAP settings (AutoSet 8–12 cmH₂O) and simulator settings
as before. Before therapy started, the device answered
`apap set fl_baseline_reject 1.4` with an ack. Only the `5ff3544` build has that
setting, and 1.4 is its default, so the probe changes nothing.

**Script:** recovery breaths were OFF for everything except the last apnea.

```
normal 90 | apnea 5 | normal 45 | apnea 10 | normal 60 | apnea 20 | normal 60 |
hyp 50% 40 | normal 60 | hyp 55% 40 | normal 60 | hyp 60% 40 | normal 60 |
[recovery 2 x 1.5 on] apnea 20 | normal 90
```

The hypopneas came out as before: 50 % → peak 10.6 slm (52 %); 55 % → 8.9 slm (44 %);
60 % → 7.9 slm (39 %).

### Scorecard

| finding | before `5ff3544` | after `5ff3544` | verdict |
|---|---|---|---|
| 1. apnea timing | 10–20 s apneas never probed; FOT 14–17 s in | 5 s not scored; **10 s scored** (FOT 7.9 s in); 20 s scored (FOT 10.0 s in) | **fixed** |
| 2. hypopnea duration (50 %) | Hyp only at the event's end | **Hyp 8.1 s in**, but **scored twice** (8 s and 32 s) in one 40 s event | **fixed, but double-counts** |
| 2. hypopnea duration (55 %) | FL only (3/3) | **nothing at all** | **worse** |
| 3. recovery threshold (60 %) | scored as central apnea | no CA, **no Hyp either**: the event is invisible | **partly** |
| 4. arousal baseline | false FL, flag on ~90 s | **false FL still**, 30 s after the apnea, flag on 12 s | **not fixed** |
| 5. AHI includes CA | AHI 0.00 with CA 3 | AHI 23.49 = (OA 0 + CA 3 + Hyp 2) / 0.213 h | **fixed** |

The APAP state log, sampled every ~4 s, shows why 55 % and 60 % failed and 50 % didn't:

| hypopnea | APAP state during the event |
|---|---|
| 50 % | MONITOR throughout |
| 55 % | **NO_BREATH at 6 s in**, then MONITOR |
| 60 % | **OSCILLATE at 7 s in** (one brief FOT burst), then MONITOR |

### R1. The baseline is zeroed on every NO_BREATH entry and re-seeded from the next breath

**This is the single cause behind findings 3 and 4 failing, and behind the 55 % regression.**

**Code (`5ff3544`):**

- On MONITOR → NO_BREATH, the pre-episode baseline is saved to `s_resume_ref_slm`
  (line 1098). Then `resetBreathDetectors()` is called (line 1099), which sets
  `s_fl_baseline_slm = 0` (line 440).
- `exitProbeToMonitor()` (line 420) returns to MONITOR **without restoring it**.
- The next breath therefore re-seeds the baseline directly, with no limit or
  rejection test (line 541: `if (!(s_fl_baseline_slm >= min_peak)) { s_fl_baseline_slm = peak_slm; ... }`).
- `s_resume_ref_slm` is currently used only for the recovery-breath test.

**Consequences on the bench:**

- **After an apnea with arousal breaths:** the first breath is the ~31 slm arousal
  breath, so the baseline restarts at ~31. Normal ~20 slm breaths are then below
  0.7 × 31 and count as limited, which gives the **false FL**.
  - The 1.4× arousal rejection (line 566) can't help: it compares against the
    already inflated baseline.
  - This was predicted from the code before the run.
- **During a weak hypopnea:**
  - The breath detector apparently misses a breath or two as flow drops. That is
    inferred; the CPAP doesn't report its detected breaths.
  - With the window shortened to **8 s** (line 42, finding 1's fix), the gap now
    reaches NO_BREATH. With the old 12 s window it didn't.
  - The detectors reset, and the baseline is re-seeded from the hypopnea's own
    8–9 slm breaths.
  - The remaining hypopnea breaths then look normal against that baseline, so
    **nothing is scored**. Before `5ff3544`, the same 55 % hypopnea scored FL.

**Proposed change:**

- In `exitProbeToMonitor()`, restore the baseline instead of leaving it at zero:

  ```c
  /* Restore the pre-episode baseline instead of letting the first breath
   * after the episode re-seed it. That first breath is the arousal breath
   * after an apnea, or a hypopnea's own weak breath after a brief NO_BREATH
   * visit - both made the baseline wrong (bench re-test of 5ff3544). */
  if (s_resume_ref_slm >= clampf(g_apap_fl_min_peak_slm, 1.0f, 30.0f))
  {
      s_fl_baseline_slm = s_resume_ref_slm;
      g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
  }
  ```

- **Expected effect:**
  - After the apnea, the baseline stays at ~20. The 1.4× rejection then keeps the
    31 slm arousal breaths out, and normal breaths stay above 0.7 × 20, so there is
    no false FL.
  - The 55 % / 60 % hypopnea breaths stay below 0.5 × 20 after the brief NO_BREATH
    visit, so a Hyp is scored about 10 s after MONITOR resumes.
- **Things to decide when implementing:**
  - Whether to cap how old the saved baseline may be. Episodes here were 5–40 s; a
    multi-minute episode might warrant re-learning.
  - Whether a NO_BREATH visit that **never reached FOT** should keep the hypopnea
    progress (`s_hyp_streak`, `s_hyp_limited_ms`) as well as the baseline. Keeping it
    would let a hypopnea that briefly tripped NO_BREATH score without starting over.

### R2. One long hypopnea is counted more than once

**Code (`5ff3544`):**

- A hypopnea is counted at line 591 when the streak and duration are met and
  `!s_hyp_active`. `s_hyp_active` is then held for `g_apap_hyp_hold_s` = 20 s.
- When the hold expires, the hold-expiry block in `updateFlHypDetector()`
  (lines 663–669) clears `s_hyp_active` **and** resets `s_hyp_streak` and
  `s_hyp_limited_ms`. It does this even if the breaths are still limited.
- The still-limited breaths then build up again (about 2 breaths / 10 s now that the
  duration uses the breath period) and **count a second event**.

**Bench:** a single 40 s, 50 % hypopnea scored Hyp at 8.1 s and again at 32.2 s. The
AHI of 23.49 above includes that duplicate. Any hypopnea longer than about
hold + 10 s ≈ 30 s is counted at least twice, which inflates AHI and RDI.

**Proposed change:** don't re-arm a hypopnea until breathing has recovered.

- Keep `s_hyp_active` latched past the hold while breaths remain hypopnea-limited, or
  add a `s_hyp_need_recovery` flag: set it when an event is counted, clear it on a
  non-limited breath (the `else` branch next to line 586), and require it to be
  clear before counting again.
- The hold timer can still end the raise window; it just shouldn't reset the
  event state on its own.

### R3. Minor: a spurious OSA-likely during the 60 % hypopnea

The brief FOT burst during the 60 % hypopnea raised `OSA likely` once: FOT flow p-p was
1.08 slm, under the 2.0 slm threshold. FOT ran while the patient was still breathing
weakly, so the probe's flow reading doesn't mean what it means during an apnea. No OA
was scored. This is a side effect of R1's NO_BREATH trip. *(Correction, 19 October:
an earlier version said it should go away with R1. It didn't; see "R3, updated" below.)*

### Also observed

- **CPAP supply current:** FOT peaked at 1.00 A. One breathing sample reached 0.85 A,
  at this run's higher pressure (the target rose to 9.6 cmH₂O). Both were brief; the
  0.8 A sustained guard never tripped.

### How to verify the next change

Re-run the same trial-5 sequence (simulator script `apap_trial5.py`). After R1 and R2:

| check | expected |
|---|---|
| hypopnea 55 % and 60 % | **one** Hyp each, about 10–15 s in, no CA |
| hypopnea 50 % | **one** Hyp, not two |
| after the apnea with arousal breaths | no FL call |
| apneas and AHI | unchanged (they're already right) |

## Re-test of R1 and R2 in `4ab7a0c` (19 October 2026)

Trial 6 repeated trial 5 exactly: same rig, AutoSet 8–12 cmH₂O, same simulator
settings and script. Before therapy, the device acked
`apap set baseline_restore_max_s 120`. Only `4ab7a0c` has that setting, and 120 is its
default, so the probe changes nothing. Code references in this section are to
`apap_ctrl.c` at `4ab7a0c`.

The simulator delivered the same events as in trial 5:

| event | peak flow |
|---|---|
| 50 % hypopnea | 10.6 slm (52 %) |
| 55 % hypopnea | 8.9 slm (44 %) |
| 60 % hypopnea | 8.1 slm (40 %) |
| apneas | 0.2 slm peak (true zero) |

### Scorecard

| test | trial 5 (`5ff3544`) | trial 6 (`4ab7a0c`) | verdict |
|---|---|---|---|
| hypopnea 50 % | Hyp at 8.1 s **and** 32.2 s | **one** Hyp, 8.1 s in | **R2 fixed** |
| hypopnea 55 % | nothing | **one** Hyp, 13.2 s in (FL at 17.2 s) | **R1 fixed** |
| after the apnea with arousal breaths | false FL 29.8 s after | **no FL call** | **R1 fixed** |
| hypopnea 60 % | nothing (one ~4 s FOT burst) | **3 CA**, no Hyp, OSA-likely once | **worse**, see R4 |
| apnea 5 s | not scored | not scored | correct |
| apnea 10 s | CA, FOT 7.9 s in | **not scored**, no FOT | **regressed**, see R5 |
| apnea 20 s | CA, FOT 10.0 s in | CA, FOT **18.1 s** in | **later**, see R5 |
| apnea 20 s + arousal | CA, FOT 11.2 s in | CA, FOT **15.3 s** in | **later**, see R5 |
| AHI | 23.49 (CA 3 + Hyp 2, one duplicate) | 32.88 = (OA 0 + CA 5 + Hyp 2) / 0.213 h | wrong mix, see below |

The script holds three apneas long enough to score (10, 20 and 20 s) and three
hypopneas, so the right totals are **CA 3 + Hyp 3**, an AHI of about 28. Trial 6 got
CA 5 + Hyp 2:

- three of the CAs are the 60 % hypopnea;
- the 10 s apnea is missing.

**Predictions written before the run:**

- **Right:** one Hyp for 50 %; one Hyp for 55 %; no FL after the arousal breaths;
  OSA-likely might persist.
- **Wrong:** "60 %: one Hyp after FOT ends, no CA".
- **Not predicted:** the slower apnea detection.

### R4. Repeated FOT bursts in one low-flow event each score a central apnea

**Bench:**

- The 60 % hypopnea ran from 494.8 to 534.8 s.
- During it, the APAP went NO_BREATH → OSCILLATE → MONITOR three times, with FOT at
  about 501.6–509.9 s, 517.9–525.8 s and 533.8–538.1 s.
- It scored a CA in each burst.
- These times come from status lines, which are sampled every ~4 s and arrive ~2 s
  late.

The bursts mean some of the ~8 slm breaths were not qualified by the EPR breath
detector. That is inferred: the CPAP doesn't report its detected breaths.

**Code (`4ab7a0c`). Every burst scores, because:**

1. **Each NO_BREATH entry starts the episode clock at the full 8 s window**
   (line 1175). FOT starts 2 s later, so the episode is already at `apnea_min_s` =
   10 s (line 63) when FOT begins. The first 2 s evaluation (lines 1243–1250)
   therefore scores a CA unless it sees OSA.
2. **CA scoring is re-armed on every burst.** `s_ca_event_armed` is set to true by
   both `enterFotProbe()` (line 439) and `exitProbeToMonitor()` (line 485).
   Nothing ties consecutive bursts to the same event.
3. **The hypopnea detector can never catch up between bursts.** By design, R1 does
   not restore FL/Hyp progress after an episode that reached FOT (line 455), so the
   Hyp streak starts from zero after each burst.

**Why trial 5 differed:** trial 5 had one burst of ~4 s and no CA. Whether a burst
scores depends on whether a breath ends it before the first 2 s evaluation, which is
down to timing. The underlying mechanism predates `4ab7a0c`.

**Proposed change, same idea as R2:** one low-flow event should produce one score.

- **Stop the extra CA/OA scores.** Add an `s_apnea_need_recovery` flag:
  - set it when a CA or OA is scored;
  - clear it on a breath that is **not** hypopnea-limited, i.e. the non-limited
    branch of `finalizeInspBreath()` next to line 657;
  - require it to be clear before scoring again;
  - alternatively, only re-arm in `exitProbeToMonitor()` when the breath that ended
    the episode is near the restored baseline.
- **Decide what a burst ended by a still-limited breath should become.** Two options:
  - Treat it like a non-FOT visit: restore the Hyp progress, so the event is scored
    as a hypopnea.
  - Keep scoring it as a central apnea, but only once.

  A 40 % flow reduction is a hypopnea by any scoring rule. The first option gives the
  clinically right answer.

### R5. A true apnea is interrupted by a breath that didn't happen (not caused by `4ab7a0c`)

**Bench (20 s apnea, onset 214.8 s), from status lines:**

| time | APAP state | leak estimate | note |
|---|---|---|---|
| 216.7 s | MONITOR | 11.6 slm | |
| 220.7 s | NO_BREATH | 11.7 slm | on time, ~6 s after onset |
| 224.7 s | **MONITOR** | **9.84 slm** | left NO_BREATH without FOT |
| 228.7 s | MONITOR | 10.0 slm | |
| 232.9 s | OSCILLATE | 9.48 slm | |

The CA was scored at 238.7 s. In trial 5, at the same point, the APAP went straight
from NO_BREATH to FOT at 224.8 s.

The other two apneas behaved the same way:

- **10 s apnea:** no NO_BREATH was seen at all.
- **Last 20 s apnea:** NO_BREATH was first seen ~11 s after onset.

**Code:**

- The only way from NO_BREATH back to MONITOR is `recoveryBreath()` (line 305,
  called at lines 1141 and 1191).
- That needs a breath stamped by the EPR detector within the window
  (`epr_ctrl.c:688`, peak ≥ 4 slm) **and** a peak of at least 0.3 ×
  `s_resume_ref_slm`, about 6 slm here.
- The simulator's own sensor shows 0.2 slm peak through the apnea. So the CPAP stamped
  a ≥ 6 slm breath that did not happen, about 8 s into the apnea.
- MONITOR then waits for a fresh 8 s window, which delays FOT by about 8 s. A 10 s
  apnea ends before that.

The diff `5ff3544..4ab7a0c` touches neither the EPR breath detector nor the NO_BREATH
entry or exit. The cause was found with a dedicated capture (next section).

#### R5 capture (19 October): two causes, both on the CPAP's own patient-flow signal

**Method:**

- Ran six apneas: 3 × 10 s, then 3 × 20 s, with normal breathing between them.
  Recovery breaths were off. Settings as in trial 6 (AutoSet 8–12, `4ab7a0c`).
- Logged the CPAP's `T,` line at 50 Hz. Its flow field is `Flow_GetPatientSlm()`,
  which is exactly the signal `Epr_Update()` gets (`therapy.c:2584`). Also logged
  `PH,` lines, and status lines every ~1 s instead of every 4 s.

**Results** (times from the simulator's apnea onset; status times arrive ~1–2 s late):

| # | apnea | NO_BREATH | FOT | scored | what happened |
|---|---|---|---|---|---|
| 1 | 10 s | +6.0 s | +7.3 s | CA | normal |
| 2 | 10 s | +4.5 s | ~+6.5 s, **ended at ~+7.7 s** | **missed** | a spike 1.2 s into FOT (cause a) ended it before the first 2 s evaluation |
| 3 | 10 s | +5.0 s | +7.3 s | CA | normal |
| 4 | 20 s | **+12.4 s** | +14.6 s | CA | a spike at +3.9 s (cause a) was stamped as a breath |
| 5 | 20 s | +4.0 s | +7.3 s | CA | normal |
| 6 | 20 s | +4.6 s | +7.9 s, **ended at ~+16 s** | CA | FOT ended early (cause b); the CA was already scored |

Apnea 2 shows trial 6's missed 10 s apnea, and apnea 4 its late FOT. In this run the
problem showed in 2 of 6 apneas (3 of 3 in trial 6).

**Cause a – brief flow transients that the CPAP takes for breaths:**

- **What they look like:** patient flow jumps from ~0 to 6–11 slm in one 40 ms
  sample, stays above 4 slm for ~0.15 s, and decays over ~0.3 s.
  - Each carries 10–40 mL. A real breath here is 500 mL over 2.5 s.
  - There were six in 8 minutes, at no consistent point in the simulator's breath
    cycle.
- **Apnea 4 example (onset 259.8 s):** at 263.61 s, flow went 0.0 → 2.6 → 7.5 → 9.4
  slm in three samples, then 7.8, 6.3, 4.5, 2.9, 1.9 … back to 0 by 263.97 s.
  P_sensor dipped to 8.12 cmH₂O and then overshot to 8.66 as the pressure loop
  reacted.
- **Where they come from: the CPAP's own flow-sensor read, not air.** See R6 below.
  They are torn AMS5935 reads at the sensor-clock beat, every 17.75 s.
- **Why the CPAP accepts them:** the EPR breath detector is tuned for fast relief, and
  a spike passes every test:
  - `insp_confirm_ms` = 50 ms;
  - `insp_min_hold_ms` = 120 ms;
  - peak ≥ `min_insp_peak_slm` = 4 slm;
  - fall from the peak ≥ `exp_min_peak_fall_slm` = 6 slm (`epr_ctrl.c:43–51`).

  `noteQualifiedBreath()` (`epr_ctrl.c:678`) then stamps `s_last_breath_ms`. The
  APAP is the only user of that stamp (`Epr_HasRecentBreath()`, via `recentBreath()`
  and `recoveryBreath()`).
- **Effects:**
  - Before NO_BREATH, a spike restarts the 8 s window, which delays FOT by up to 8 s
    (apnea 4).
  - During NO_BREATH or FOT, a spike above 6 slm passes `recoveryBreath()` and ends
    the episode. If that happens before the first 2 s evaluation, nothing is scored
    (apnea 2).
- **This needs fixing even if the spikes turn out to be a bench artefact.** A real
  patient circuit makes similar transients: coughs, swallows, a mask being bumped.

**Cause b – the leak estimate drops during FOT, and FOT's own flow then counts as a breath:**

- In apneas 5 and 6, about 13–15 s after onset (i.e. during FOT):
  - the leak estimate fell from ~12.4 to 9–10 slm;
  - the 1 s mean patient flow moved from about −0.6…−1.0 to +1…+2.4 slm.
- The 4 Hz FOT flow swing (about ±3 slm) on top of that offset peaked at 4.5–6.1 slm,
  with peaks ~0.22 s apart.
- In apnea 6 one of those peaks was accepted as a breath at ~+15.5 s, and FOT ended
  at ~+16 s with the apnea still running.
- The default leak algorithm is VIVO (`leak_estimator.c:16`). Nothing freezes its
  adaptation during an APAP episode.
- **Candidate trigger, not verified:** `cpapBackupIfNeeded()`
  (`leak_estimator_vivo.c:504`) re-seeds the leak ratio from the last 10 s mean once
  the leak module has seen no inspiration for `LEAK_VIVO_BACKUP_MS` (15 s). That
  10 s mean includes FOT. The exact timing wasn't confirmed.

**Ruled out:**

- **Slow leak drift before NO_BREATH:** flow stayed within ±0.1 slm during the first
  seconds of every apnea.
- **The periodic physiology FOT probe:** `PH` lines with `fot_src` 1 appeared in
  apneas 1 and 4, but neither exit lines up with one.
- **The 1.2 s `insp_max_hold` forced expiration:** the spikes pass the normal end-of-
  inspiration path.

#### R5 proposed changes

**R5a. An apnea episode should only be ended (or its window restarted) by a breath of
plausible size and length, not just a peak.**

- In `noteQualifiedBreath()`, also require:
  - an **inspiratory duration** of at least ~0.4 s, measured from the start of
    `BREATH_INSP_PENDING`;
  - an **inspiratory volume** of at least ~80 mL.
- Accumulate the volume alongside the peak in `trackInspFlowPeak()`, as
  patient-directed flow × dt.
- Expose both values the way `Epr_GetLastInspPeakSlm()` is exposed.
- Make both thresholds runtime-settable, like the other EPR/APAP parameters.

**Margins:**

| | inspiration above 4 slm | volume |
|---|---|---|
| spikes | ≤ 0.15 s | ≤ 42 mL |
| FOT half-cycle | ~0.125 s | — |
| normal breath on this rig | 2.5 s | 500 mL |
| 60 % hypopnea (weakest real breaths tested) | 2.5 s | ~200 mL |

**Things to decide when implementing:**

- **Which detector to change.** The stamp is used only by the APAP, so tightening
  `noteQualifiedBreath()` doesn't touch EPR pressure relief: that runs on the FSM
  phase, not on the stamp. If EPR ever comes to depend on the stamp, give the APAP a
  separate one.
- **Also consider requiring the recovery breath** in `recoveryBreath()` (line 305) to
  have a volume of at least a fraction of the pre-episode breath size. Today it uses
  0.3 × peak only.

**R5b. Freeze leak-estimator adaptation while the APAP is in NO_BREATH or OSCILLATE.**

- With no breathing, leak can't be separated from patient flow, and FOT modulates
  the pressure the vent model depends on.
- The VIVO freeze already exists: `LeakVivo_SetAdaptationFreeze()`
  (`leak_estimator_vivo.c:646`). It blocks the seed, the rest-volume level and the
  15 s backup.
- But the only wrapper, `Leak_SetDisconnectFreeze()`, is owned by the disconnect
  logic (`therapy.c:1468`, `1537`, `2542`). Calling it from the APAP would let the
  two clear each other.
- **Proposal:** a second reason flag, e.g. `Leak_SetApneaFreeze(bool)`, so that
  `freeze = disconnect || apnea`. The APAP sets it on MONITOR → NO_BREATH and clears
  it in `exitProbeToMonitor()`.
- The SIMPLE path's `Leak_SetHoldAdaptation(Flow_IsSampleStale())` (`main.c:2525`)
  doesn't act on VIVO, so it is not the place for this.
- **Verify on the bench:** during FOT, the status line's leak estimate and the `T,`
  patient-flow mean should stay flat.

**Order:** implement R4, R5a and R5b together, since they interact.

- R5a removes the spike-driven exits.
- R5b removes the drift-driven exits.
- R4 still has to make one weak hypopnea score once.

Note that R5a makes breath qualification **stricter**. The 60 % hypopnea breaths
(~200 mL, Ti 2.5 s) clear it comfortably, but check that this doesn't add NO_BREATH
trips during hypopneas: it is the opposite failure to R4's.

### R6. The spikes are torn flow-sensor reads at the clock beat, every 17.75 s (19 October)

**Method:**

- Logged three flow sensors at once, in air-path order:
  1. the CPAP's `T,` patient flow (50 Hz);
  2. the external sensor just downstream of the vent (COM4, ~100 Hz);
  3. the simulator's lung-side SFM3300 (`F,` line, ~38 Hz).
- CPAP fixed at 8 cmH₂O, so no FOT. Script: 1 min breathing, a 5 min apnea, 3 min
  breathing.
- The three clocks were aligned by cross-correlating breathing flow:
  r = 0.994 for CPAP vs external, 0.993 for simulator vs external.
- The CPAP was running the R4/R5 build (it acked `apap set breath_min_insp_ms 400`).

**Results:**

| | |
|---|---|
| CPAP-side transients (≥ 4 slm rise in 40 ms, back down within 0.4 s) | **16**: 8 in the apnea, 8 while breathing |
| …also seen by the external sensor | **0** |
| …also seen by the simulator sensor | **0** |
| period | **17.753 s**, rms timing error 28 ms over 9 minutes |
| beats that produced a transient | 16 of 33; the rest fell under the 4 slm threshold |
| R5 capture's 9 transients | on the same grid: gaps of 1, 5, 6 and 7 periods |

The CPAP's own `flow spike` snapshot agrees: **`gap_ms=17740`** between its last two
needles. At that needle the raw ΔP sample looked normal (−0.047 mbar, against a zero
offset of −0.045), but the filtered ΔP had jumped to **+0.474 mbar**. So the bad
samples had already gone into the filter.

**Mechanism.** It is already described in `flow_sensor.c:110–121`:

- The AMS5935 is polled every 5 ms with no EOC line (`hamsFS.use_eoc = 0`,
  `main.c:2306`). The sensor's oscillator drifts against our tick.
- Once per beat a read lands on the register update and returns a torn 24-bit value.
  Near zero the LUT is about 100 slm/mbar, so a small tear is a large flow error.
- A 17.75 s beat means the two clocks differ by about 280 ppm. That figure will vary
  with temperature and from unit to unit.
- The existing guards don't stop it:
  - the median-of-5 (`flow_sensor.c:1160`) only rejects one or two consecutive tears;
  - the impulse cap (`flow_sensor.c:1180`, 15 slm per sample since 2026-10-02) still
    lets a capped step through;
  - the 2 Hz patient LPF then turns that step into the 6–11 slm, ~0.15 s needle seen
    in R5.
- **Inference, not verified:** at a slow beat the read phase moves only ~1.4 µs per
  sample, so a run of several consecutive torn reads is likely. That would explain
  why the median fails.

**Effects beyond the APAP:**

- **The needle drives the pressure loop.** About 0.2 s after each needle, the external
  and simulator sensors both see a small **real** puff of 1.5–2.5 slm. The plots show
  the CPAP needle first and ~4× larger, then the downstream puff.
- In the R5 capture the PID command jumped ~2,000 rpm (19.4k → 21.2k → 17.4k), and
  P_sensor went −0.14 then +0.40 cmH₂O.
- So every ~18 s the patient gets a small, needless pressure and flow kick.
- Every consumer of `Flow_GetPatientSlm()` sees the needle: the EPR breath FSM, the
  APAP, the leak estimator and the physiology metrics.

**Proposed change (R6):** fix it at the read, not downstream.

1. **Don't accept a torn read.** Options, in order of preference:
   - wire and use EOC (`use_eoc = 1`);
   - check the sensor's status/busy bit and hold the last good sample if the
     conversion is in progress;
   - read twice back to back and accept only if both agree within a small tolerance,
     otherwise treat the tick as stale (the stale path already exists).
2. **If a guard has to stay downstream, make the impulse cap reject rather than
   clamp.**
   - Treat a capped sample as stale (hold the last good value) instead of stepping
     15 slm toward it.
   - Accept a real fast change once a few consecutive samples agree.
   - Real fast transients (mask-off, end of inspiration at high flow) persist; a tear
     does not.

**Verify on the bench:** re-run this three-sensor capture (`spike_source.py`). Expect
no CPAP-only transients, `flow spike` showing no needles in the apnea, and no
downstream puffs.

**Relation to R5a:**

- R5a hardens the APAP against these needles and against real coughs and bumps. Keep
  it.
- R6 removes the needles at the source, which also stops the blower kicks R5a can't
  touch.
- Whether the R4/R5 build already rejects them in the APAP is the next test (trial 7).

### R3, updated

The spurious OSA-likely during the 60 % hypopnea **persisted** (one sample). R1
restores the detector state after the NO_BREATH trip but doesn't prevent the trip or
the FOT burst. The earlier statement that it "should go away with R1" was wrong. It
belongs with R4: no FOT burst while the patient is still breathing weakly, no
misleading probe reading.

### Also observed (trial 6)

- **CPAP supply current:** FOT peaked at 1.11 A, with 19 samples over 0.8 A, all
  while oscillating. Breathing reached 0.81 A once. The 0.8 A sustained (0.7 s) guard
  never tripped.

### How to verify the next change

After R4, R5a and R5b, re-run the trial-6 sequence (`apap_trial6.py`) and the R5
apnea capture (`apnea_capture.py`):

| check | expected |
|---|---|
| hypopnea 50 % / 55 % | one Hyp each (as now) |
| hypopnea 60 % | **one** event: a Hyp, or at most one CA |
| apnea 10 s / 20 s / 20 s | CA each, FOT within ~8–11 s of onset |
| after the arousal breaths | no FL call (as now) |
| totals | CA 3 + Hyp 3, AHI ≈ 28 |
| R5 capture, 6 apneas | all scored; no NO_BREATH/FOT exit before the apnea ends, even when a spike occurs |
| leak estimate during FOT | flat (no 2–3 slm drop) |

## Re-test of the R4/R5 build: trials 7 and 7b (19 October 2026)

The CPAP ran the uncommitted working tree on top of `4ab7a0c` with R4, R5a and R5b
(probe `apap set breath_min_insp_ms 400` acked). R6 is not fixed, so the needles are
still present.

**Script:** the trial-6 sequence with the R5 apnea block inserted:

```
normal 90 | apnea 5 | normal 45 | 3 × (apnea 10 | normal 45) |
3 × (apnea 20 | normal 60) | hyp 50/55/60 % 40 each, normal 60 between |
[recovery on] apnea 20 | normal 90        (955 s)
```

### Trial 7 (simulator in pressure feed-forward): void, a start-up deadlock

- The APAP entered NO_BREATH/FOT during start-up, before the simulator's first breath.
- **The simulator then never breathed.** Its pressure feed-forward holds base speed
  until it has an inlet-pressure estimate. That estimate needs a still lung: every
  sample of the last 0.3 s of expiration under 3 slm. The CPAP's 4 Hz FOT never
  allows that, and there was no fallback. This is a BreathSimulator bug, now fixed
  in its firmware (mean-flow test, plus a static fallback after 2 breaths), pending
  flash.
- The "breaths" were 70 mL at a 5.9 slm peak: only the FOT moving the lung.
- The CPAP saw no breath, so FOT never ended and nothing was scored in 16 minutes.
- Its leak estimate was frozen (R5b) at ~21 slm, against ~12–13 normally, so patient
  flow sat at about −16 slm.
- Earlier builds escaped this only by accident: needles counted as breaths.

### Trial 7b (simulator in static feed-forward): a valid test of the apnea side

| check | result | verdict |
|---|---|---|
| start-up, before the simulator breathed | NO_BREATH/FOT, one CA; **recovered as soon as breathing began**, normal leak estimate (12.6–13 slm) | **R5b does not trap the CPAP**; the CA is a harness artefact (the CPAP starts before the simulator breathes) |
| apnea 5 s | not scored | correct |
| apneas 10 s ×3 | **CA each** | **fixed** (trial 6 and the R5 capture each missed one) |
| apneas 20 s ×3, plus 20 s + arousal | CA each; FOT ran to the end of every apnea (16–17 s) | **no early exits (R5a)** |
| FOT onset | ~6–7.5 s after the true onset | good |
| an R6 needle mid-FOT (apnea 6: 7.0 slm, 0.15 s) | FOT carried on | **R5a rejects it** |
| leak estimate during FOT | changed ≤ 0.9 slm (R5 capture: −2 to −3 slm) | **R5b works** |
| after the arousal breaths | no FL call | as before |
| hypopneas 50/55/60 % | nothing scored | **not a valid test** (below) |
| totals | CA 8 (7 apneas + start-up), Hyp 0, FL 0, AHI 28.2 | — |

**R4 is still untested.**

- Static feed-forward can't make these hypopneas. The flow loop overshoots, so the
  delivered peaks were 71–76 % of normal instead of 40–52 %, all ~250 mL.
- None of them crosses the Hyp rule (< 0.5 × baseline) or the FL rule (< 0.7 ×), so
  scoring nothing was correct.
- R4 needs the pressure-FF simulator with its fix: re-run trial 7.

**CPAP current:**

- FOT peaked at 1.11 A.
- Breathing peaked at **1.00 A**, with 66 samples over 0.8 A. That is higher than
  before, because static mode's breaths peak higher (27 slm vs 20 slm).
- The 0.8 A sustained (0.7 s) guard never tripped.

**Harness notes:**

- The simulator dropped 4 of its event-start lines in this run. Onsets are therefore
  taken from its per-breath labels, which is how the onset times above were
  corrected.
- During trial 7, a lost CPAP status reply made the old teardown skip the stop, and
  the CPAP was left running. It was stopped by hand the safe way, and the harness now
  stops whenever the status is unknown.

### Trial 7c (pressure feed-forward, fixed simulator): the full test

The simulator now has the inlet-estimate fix: a mean-flow test, plus a static
fallback after 2 breaths. It breathed normally (505 mL, 22 slm peak) from the first
breath, even though the CPAP was already in FOT. The hypopneas were delivered as in
trial 6: Vt 250 / 229 / 207 mL.

| check | result | verdict |
|---|---|---|
| apnea 5 s | not scored | correct |
| apneas 10 s ×3, 20 s ×3, 20 s + arousal | **CA each**; FOT 6.8–8.7 s in; FOT ran to the end of every 20 s apnea | **R5 fixed** |
| hypopnea 50 % | one Hyp, 9.4 s in; FL at 13.5 s | correct (as in trial 6) |
| hypopnea 55 % | FOT 11 s mid-event → CA scored (count 7→8) → Hyp at 34.7 s → **CA withdrawn** (8→7) | **R4 reclassification works**: one event, a Hyp |
| hypopnea 60 % | FOT for ~21 s of the 40 s event → **one CA**, no Hyp | R4 met ("at most one CA", trial 6 had 3), but it should be a Hyp; see R7b |
| after the hypopnea | another 5 s FOT burst (788–794 s) during normal breathing | see R7b |
| after the arousal breaths | no FL | correct |
| start-up | **FOT for ~56 s while the simulator breathed 505 mL breaths**, then one CA | **wrong**; see R7a |
| totals | CA 9 (7 apneas + 60 % hyp + start-up), Hyp 2, FL 2, AHI 38.8 | — |
| CPAP current | FOT peaks 1.15 A (42 samples > 0.8 A); breathing ≤ 0.90 A | the 0.8 A / 0.7 s guard never tripped; the long FOT periods are the highest-current time |

Both new problems were checked with `epr_replay.py`, which replays the EPR breath FSM
on the logged `T,` patient flow:

| | candidates | inspiration (median) | volume (median / p10) | R5a accepts |
|---|---|---|---|---|
| outside FOT | 156 | 1260 ms (the max-hold stamp) | 280 / 127 mL | 97 % |
| **during FOT** | 15 | **720 ms** | **55 / 18 mL** | **13 %** |

### R7a. R5b freezes a leak estimate that breathing never seeded

**What happened:**

- At start-up the APAP entered NO_BREATH/FOT before the simulator's first breath, and
  R5b froze the leak estimate at **23–24 slm**. With breathing it settles at ~12.7.
- Patient flow therefore sat **~12.7 slm low**: −12.7 slm mean over 20–30 s, while the
  simulator delivered 505 mL breaths.
- Each real breath reached only 6–10 slm in the CPAP's view and was stamped early as
  an 18–57 mL fragment, which R5a rejected. FOT ran ~56 s with a normally breathing
  patient.
- In trial 7 the same frozen value (~21 slm) plus the simulator bug made it permanent.

**Proposed:**

- Freeze only an estimate that breathing has seeded: skip R5b until the VIVO
  estimator has completed its breathing seed (`g_leak_vivo_seed_done`, or the
  equivalent) after therapy start.
- And/or cap the freeze, e.g. 60 s, so a bad freeze can't hold indefinitely.
- A real patient who falls asleep within the first minute, or whose mask settles late,
  meets exactly this.

### R7b. During FOT the 4 Hz ripple cuts each breath short, and R5a then rejects the fragment

**What happened:**

- During FOT the ±3–6 slm ripple satisfies the peak-fall end-of-inspiration test
  (fall ≥ 6 slm from a ripple crest) early, often ~0.4–1.2 s in. The stamp is taken
  then.
- So a real 200–230 mL hypopnea breath is judged on 55–85 mL and rejected by the 80 mL
  test.
- Effects:
  - the 60 % hypopnea held FOT for ~21 s and became a CA;
  - the 55 % one was rescued only by R4's reclassification;
  - a burst also ran into normal breathing after the 60 % event.
- Before R5a, these fragments counted, because the peak alone was enough.

**Proposed**, best first:

1. **Notch the FOT frequency (4 Hz) out of the flow fed to the EPR breath FSM while
   FOT is active**, or low-pass it at ~1.5–2 Hz. Breaths are under 1 Hz, and the FOT
   frequency is known, so the ripple stops chopping inspirations. The APAP's FOT
   metrics keep using the band-passed flow they already use.
2. Or, while FOT is active, skip the peak-fall path and judge the breath at the
   max-hold point (1.2 s) or the real zero crossing.
3. As a stopgap: while FOT is active, apply only R5a's duration test. 400 ms still
   rejects the 0.15 s needles and the 0.125 s ripple half-cycles.

**Verify:** re-run trial 7c and check:

- the start-up FOT ends within a breath or two of breathing starting;
- the 60 % hypopnea scores one Hyp;
- no FOT bursts during normal breathing;
- R5's apnea results unchanged.

### Trial 7d (R6 + R7 build): same script as 7c

The probe `apap set breath_notch 1` was acked. Simulator delivery was as in 7c:
hypopneas of Vt 252 / 226 / 209 mL.

| check | 7c | 7d | verdict |
|---|---|---|---|
| start-up | FOT ~56 s while breathing | no long FOT, one start-up CA | **R7a fixed** |
| needles (T flow, outside FOT) | 9 in 21.6 min | **1 in 18.2 min** | **R6 fixed** (one left) |
| apnea 5 s / 10 s ×3 / 20 s ×4 | not scored / CA / CA | same; FOT 6.7–8.6 s; no early exits | unchanged, good |
| hypopnea 50 % | one Hyp, 9.4 s | one Hyp, 9.4 s | good |
| hypopnea 55 % | CA, then Hyp (CA withdrawn) | **one Hyp at 14.2 s, no FOT during the event**, then a FOT burst at the event's last breaths → **CA** | **two events for one**; see R8b |
| hypopnea 60 % | FOT ~21 s → CA | FOT for the **whole event** (~35 s), ending at the first normal breath → **CA** | still a CA; see R8a |
| normal breathing | one FOT burst | none: FOT always ended at the first normal breath | good |
| OSA-likely | 1 sample | **39 samples**, all in the long FOT over the 60 % event | spurious (FOT read during breathing); no OA scored |
| totals | CA 9, Hyp 2 | CA 10 (7 apneas + start-up + 55 % tail + 60 %), Hyp 2 | expected: CA 7 (+1) + Hyp 3 |
| CPAP current | FOT 1.15 A | FOT 1.16 A, breathing 1.02 A once | the guard never tripped |

### R8. Weak breaths shrink further at the CPAP, and R5a's 80 mL limit sits right on them

**R8a. The hypopnea breaths reach the APAP at about half size.**

| | simulator delivered | CPAP saw, outside FOT | CPAP saw, in FOT |
|---|---|---|---|
| 55 % / 60 % hypopnea breaths | 207–227 mL, peak 8.7–11.6 slm | **96–107 mL, peak 5.7–6.4** | 29–73 mL, peak 4.3–6.7 |
| normal breaths | 500 mL, peak 20 | ~280 mL at the 1.2 s stamp, peak 17–19 | — |

*(volumes are the R5a volumes, up to the stamp)*

- The R5a volume is counted only up to the stamp (max-hold 1.2 s, or peak-fall).
- The CPAP's mean patient flow during the hypopneas is **−2.1 to −2.6 slm** (7c and
  7d), against ≈ 0 in normal breathing. The leak estimate reads ~2 slm high once
  breaths get small, and every weak breath loses that offset on top.
- Normal breaths lose a similar absolute amount (20 → 17 slm peak) but have the margin
  to absorb it.
- The 4 Hz notch (R7b) can't help: the problem is volume, not ripple.
- Result: weak but real breaths sit right on the 80 mL test. Some pass, some fail, so
  NO_BREATH trips, and once FOT runs almost none pass.

**Proposed:**

1. **Lower R5a's volume limit**, e.g. to ~40 mL, or make it relative (e.g.
   ≥ 0.25 × the median breath volume, with a ~30 mL floor). Keep the 400 ms
   duration test.
   - With R6 removing the needles at the source (1 left in 18 min), and the duration
     test rejecting 0.15 s needles and 0.125 s FOT half-cycles, the volume test no
     longer carries the weight it did when R5a was written.
2. **Look at the leak estimator's bias at small tidal volumes** (VIVO): a ~2 slm
   over-read during hypopneas. Lower priority, but it shrinks exactly the breaths the
   APAP most needs to see.
   - A future run with the external sensor logging would show the true patient flow
     next to the CPAP's.

**R8b. A Hyp followed by a CA in the same event still counts twice.**

- R4's reclassification withdraws a CA when a Hyp follows within
  `ca_reclass_window_s`.
- In 7d's 55 % hypopnea the order was the reverse: Hyp at 14.2 s, then a FOT burst on
  the event's last weak breaths, then a CA.
- **Proposed:** don't score a CA (or OA) for an episode that starts within the Hyp
  hold, or within ~30 s of a Hyp, while breathing is still hypopnea-limited. That is
  the mirror of R4's reclassification.

**Verify:** re-run the same script (`apap_trial7d.py`). Expect:

- the 55 % and 60 % hypopneas each score one Hyp and no CA;
- no OSA-likely run during the hypopneas;
- totals CA 7 (+1 start-up) + Hyp 3.

### Trial 7e (R8 build): same script

The probe `apap set hyp_ca_guard_s 30` was acked.

| check | 7d | 7e | verdict |
|---|---|---|---|
| start-up, apneas, 50 % hypopnea | good | same (7 × CA, FOT 6.8–8.7 s; one Hyp) | good |
| hypopnea 55 % | Hyp + tail CA | **one Hyp at 10.2 s, no CA** | **R8b fixed** |
| OSA-likely | 39 samples | **2** | fixed |
| hypopnea 60 % | FOT whole event → CA | **same: FOT 758–794 s → CA** | **not fixed** (below) |
| needles | 1 | 1 in 21 min | R6 holds |
| totals | CA 10, Hyp 2 | CA 9 (7 apneas + start-up + 60 %), Hyp 2 | one event wrong |
| CPAP current | FOT 1.16 A | FOT 1.08 A, breathing 0.85 A | fine |

**Why the 60 % hypopnea still fails.** Replayed on the 50 Hz flow:

- **The first 60 % breath was not seen as a breath at all.** The simulator delivered
  183 mL at a 7.4 slm peak. The CPAP's patient flow peaked at **+2.8 slm**, under the
  4 slm needed to start an inspiration.
- **The CPAP's mean patient flow stepped from −0.7 to −2.7 slm** at that breath and
  stayed at **−3.4 to −3.6 slm** through the event. Normal breathing is ≈ 0.
- So NO_BREATH tripped 8 s after the last normal breath (754.8 s), before the
  hypopnea detector could score anything. R8b doesn't apply: there was no Hyp to
  follow.
- In FOT the remaining breaths (208–213 mL delivered) reached the CPAP at a
  **5.6–6.6 slm** peak. That straddles the recovery test (0.3 × baseline peak ≈ 6 slm),
  so FOT ran to the end of the event.
- **R8a's lower volume limit is no longer the gate. The patient-flow offset is.**

### R9. Find where the −3 slm patient-flow offset during weak breathing comes from

This should be settled before the CPAP code changes again. There are two candidates,
and they need opposite fixes:

1. **The CPAP's leak estimate** (VIVO per-breath leak level) **over-reads when breaths
   shrink.**
   - It fits that the offset appears **at the first weak breath**, and does **not**
     appear in apneas: apneas sit at ≈ 0 for their first seconds, with no breath to
     update from.
   - If so, it is a CPAP bug a real patient would hit, and the fix belongs in the
     leak estimator.
2. **Real air: the simulator box's ~13 L/min leak changes with how hard the
   simulator's blower works.**
   - Less blower effort in a hypopnea would mean less box leak and less total flow
     through the CPAP.
   - If so, it is a rig artefact, and the CPAP's reading is right.

**Test (bench, no firmware change):** the three-sensor capture (`spike_source.py`
method: CPAP T + external sensor + simulator F), with CPAP fixed at 8 cmH₂O and a
script of normal | 60 % hypopnea | normal | 50 % hypopnea | normal. Then compare:

- **external minus simulator flow** = box leak, normal vs hypopnea;
- **the CPAP's patient flow vs the simulator's lung flow**, breath by breath.

If the box leak is flat and only the CPAP's patient flow drops, it is cause 1.

#### R9 result (19 October): mostly the CPAP, and instantaneous, not adaptive

**Method:**

- Three-sensor capture (`leak_source.py` + `leak_analysis.py`); CPAP fixed at
  8 cmH₂O (no APAP) on the R8 build.
- Clocks aligned on breathing flow: r = 0.994 for CPAP vs external, 0.993 for
  simulator vs external.
- 59 breaths, split at the simulator's inspiration starts.

| segment | sim Vt | sim peak | external peak (− mean) | **real box leak** (ext − sim) | **CPAP patient-flow mean** | CPAP peak | CPAP / sim peak |
|---|---|---|---|---|---|---|---|
| normal | 500 mL | 21.2 slm | 19.0 | 10.13 | +0.19 | 17.9 | 0.84 |
| **hypopnea 60 %** | 200 | 8.3 | 7.7 | **9.47** | **−2.24** | **4.3** | **0.52** |
| normal | 501 | 21.3 | 19.0 | 10.18 | +0.16 | 17.8 | 0.83 |
| **hypopnea 50 %** | 250 | 11.4 | 10.4 | **9.33** | **−1.17** | 7.9 | 0.69 |
| normal | 500 | 21.1 | 19.0 | 10.16 | +0.14 | 17.7 | 0.84 |

**Split of the CPAP's offset into real air and the CPAP's own part:**

| | change in the CPAP's mean | real air (box-leak drop, a rig effect) | **made inside the CPAP** |
|---|---|---|---|
| 60 % hypopnea | −2.4 slm | −0.7 | **−1.7** |
| 50 % hypopnea | −1.3 slm | −0.8 | **−0.5** |

The CPAP's part grows sharply as breaths shrink.

**It is not the leak estimate drifting:**

- The offset appears **in full on the very first hypopnea breath** (−2.79 slm) and is
  gone **on the very first normal breath** after it (+0.14). There is no settling over
  breaths.
- The status line's unintentional-leak estimate actually went **down** ~0.7 slm in
  the hypopneas, tracking the real box leak.
- So the extra −0.5 to −1.7 slm is formed **within each breath**, in how patient flow
  is computed or measured.
- **Candidates for the CPAP thread:**
  - a non-linearity in the AMS5935 ΔP→flow LUT, or the total-flow path near low or
    reversed flow;
  - the tube-compensation / vent-model terms that depend on instantaneous flow or
    pressure;
  - the VIVO patient-flow formation.
- The external sensor sees the hypopnea breaths at the same proportion as normal
  ones (7.7/8.3 vs 19.0/21.2). The shrinkage is the CPAP's.

**Follow-up that would split sensor from model:** re-run with the CPAP's **total**
flow logged (B-line field 10, `Flow_GetSlmCtrl()`, 25 Hz).

- CPAP total minus external = the CPAP's own view of vent flow.
- If that stays flat between normal and hypopnea breathing, the total-flow
  measurement is right and the error is in the vent/leak/patient model. If not, it is
  the sensor path.

**Fix options for the APAP, whatever the cause:**

- **Make the breath detector offset-robust.**
  - `patientInspiratoryFlow()` compares the absolute patient flow with 4 slm.
  - The EPR FSM already tracks a flow baseline (`g_epr_flow_baseline_slm`).
  - Detecting inspiration on `q − baseline` (as `dir` already is) would make a
    −2…−3 slm offset harmless.
- **Make the recovery-breath test relative** to the same baseline, for the same
  reason.

### Trial 7f (R9 build: `epr_rel_baseline 1`, `rel_baseline 1`): worse in the hypopneas

The probe `apap set epr_rel_baseline 1` was acked. Same script as 7c–7e.

| check | 7e | 7f | verdict |
|---|---|---|---|
| start-up, apneas (7 × CA, FOT 6.5–7.7 s, no early exits), 50 % hypopnea | good | same | good |
| needles | 1 | **0** in 19.6 min | R6 complete |
| hypopnea 55 % | one Hyp, no FOT | Hyp at 10.3 s, **then FOT for 28 s → tail CA** | **regressed** |
| hypopnea 60 % | FOT the whole event → CA | FOT from ~17 s in for 12 s → CA | still a CA |
| OSA-likely | 2 | **40** | regressed (FOT during breathing) |
| totals | CA 9, Hyp 2 | CA 10, Hyp 2 | worse |

**Why: the EPR flow baseline climbs onto weak breaths.**

`g_epr_flow_baseline_slm` (τ 2.5 s) only updates while the flow is within the 5 slm
deadband of it. That excludes a normal breath's swings, but **a hypopnea breath's
inspiratory flow sits inside the deadband**, so the baseline rides up on it.

I emulated the tracker on the logged patient flow. For each breath: peak patient
flow, the baseline at that peak, and the peak minus the baseline (what the 4 slm test
now compares):

| breaths | raw patient-flow peak | baseline at the peak | peak − baseline | absolute test (> 4.08) | relative test |
|---|---|---|---|---|---|
| normal | 17–18 slm | −0.3 | 17–18 | pass | pass |
| 55 % hypopnea | 5.1–5.3 (early), 7–8 (later) | **+1.0 … +2.2** | **3.0–3.6** (early), 6–6.5 | pass | **fail** (early ones) |
| 60 % hypopnea | 2.5–7.6 | +1.2 … +1.6 | 1.9–6.3 | 5 of 9 pass | 4 of 9 pass |

So a baseline taken from mid-breath flow shrinks small breaths further instead of
correcting them. Also, the R9 offset is **not a DC shift** between breaths: the CPAP
reads the inspiratory peaks ~3.5–4 slm low for both normal (21 → 17.8) and
hypopnea (8.3 → 4.3) breaths. No DC baseline can undo that.

**Proposed (R10):**

1. **Turn the relative breath-start test off again** (`apap set epr_rel_baseline 0`;
   runtime, no reflash). It is not offset-robust against this error.
   - An A/B on the same build would confirm that it alone caused the 55 %
     regression; whether `rel_baseline` (the FL/Hyp measurement) helps or hurts is
     the second half of that A/B.
2. **If a reference is still wanted,** take it from end-expiration: the flow level in
   the last ~0.3 s before inspiration starts, as the simulator does for its inlet
   estimate. A running mid-flow tracker is not suitable.
3. **The real fix is upstream:** find why the CPAP reads inspiratory flow ~4 slm low
   (R9: sensor/LUT path vs vent/leak/patient model). Next step is the capture that
   logs the CPAP's total flow (B-line field 10) next to the external and simulator
   sensors.

#### R10 result (19 October): the flow sensor is right, the subtracted leak/vent model is wrong

**Method:**

- `total_source.py` + `total_analysis.py`: the R9 three-sensor capture plus the
  CPAP's **total** flow (B line field 10, `Flow_GetSlmCtrl()`, 25 Hz). It shares the
  CPAP's tick with the `T,` patient flow.
- CPAP fixed at 8 cmH₂O; script normal | 60 % hypopnea | normal | 50 % hypopnea |
  normal; 59 breaths.
- Alignment: r = 0.991 for CPAP vs external, 0.990 for simulator vs external.

**Air path:** CPAP sensor → hose → **vent** → external → chamber → box (leak) →
simulator sensor → lung. Per breath:

- CPAP total − external = vent flow as the CPAP's sensor sees it;
- CPAP total − CPAP patient = the vent + leak the CPAP's model subtracts.

| segment | real box leak (ext − sim) | **vent as measured** (total − ext) | **model subtracted** (total − patient) | patient mean | ext peak (− mean) | CPAP total peak (− mean) | CPAP patient peak |
|---|---|---|---|---|---|---|---|
| normal | 10.14 | 19.60 | 30.42 | +0.18 | 18.9 | 18.1 | 17.5 |
| **hypopnea 60 %** | 9.53 | **19.97** | **32.04** | −2.06 | 7.7 | **7.6** | 4.6 |
| normal | 10.20 | 19.68 | 30.46 | +0.24 | 19.1 | 18.8 | 17.8 |
| **hypopnea 50 %** | 9.33 | **19.91** | **31.59** | −1.57 | 10.5 | 10.1 | 7.3 |
| normal | 10.16 | 19.66 | 30.17 | +0.46 | 19.2 | 18.6 | 18.0 |

**Hypopnea minus normal:**

| | change |
|---|---|
| real box leak | −0.74 slm |
| vent as measured | **+0.29 slm** |
| model subtracted | **+1.47 slm** |
| patient mean | −2.11 slm |

**Verdict:**

- **The CPAP measures total flow correctly.**
  - The vent flow its sensor sees is flat to within 0.3 slm.
  - Its total-flow breath matches the external sensor even in a 60 % hypopnea (7.6 vs
    7.7 slm peak).
- **The model it subtracts is wrong.** It rises 1.5 slm in weak breathing while the
  real leak falls 0.7 slm, a ~2.2 slm error that appears per breath.
- The breath shape mostly survives: the hypopnea patient peak above its own mean is
  6.7 slm vs 7.6 total. The error is mainly a per-breath **level** shift in the
  subtracted vent + leak.
- So R8a's "leak estimate over-reads at small Vt" was the right place, but it is not
  the slow unintentional-leak estimate (that went *down* ~0.7, correctly). It is the
  per-breath VIVO level.

**Where to look (VIVO, `leak_estimator_vivo.c`):**

- `calcLeakLevelFromRestVolume()` sets the per-breath leak level from the breath's
  net (rest) volume. It splits it between inspiration and expiration using
  `s_last_insp_dur_ms` / `s_last_exp_dur_ms` and the table flow at
  `s_last_insp_press_idx` / `s_last_exp_press_idx`.
- Those come from VIVO's own breath FSM (`updateBreathFsm`): total-flow excursions
  about a ~0.35 Hz baseline, entering inspiration at ≥ 2.5 slm and expiration at
  ≤ −2.0 slm, after 100 / 80 ms confirms.
- With hypopnea-sized excursions the detected phase durations and pressures shift,
  and with them the level.
- **Check first:** VIVO's insp/exp durations and the rest volume per breath, normal
  vs hypopnea.

**Robust option for the APAP, independent of the VIVO fix:**

- Detect breaths on **total-flow excursions** (total minus a slow baseline, as VIVO's
  own FSM does), not on the model-corrected patient flow. This capture shows total
  flow carries the hypopnea breaths faithfully.
- The R6 needles are already handled at the flow-sensor level.
- And keep `epr_rel_baseline` off (R10 item 1).

**Correction (20 October): it was not the VIVO per-breath level.**

- The CPAP thread traced the subtracted ~2 slm to the patient-flow **pause-DC
  zeroing** stage (`g_leak_patient_pause_zero_enable`, `leak_estimator.c`).
- Its "quiet" envelope (≤ 4 slm, only left at |Q| ≥ 7 slm) never releases during a
  weak inspiration. The pause DC then slews onto the breath and is subtracted from it.
- Replaying that filter on this capture reproduces the CPAP's patient flow. The
  capture's verdict (model, not sensor) stands; my pointer at
  `calcLeakLevelFromRestVolume` was wrong.

### Trial 7g (R10 build: pause-DC zeroing off; `epr_rel_baseline` 0, `rel_baseline` 0): 20 October

The probe `leak set patient_pause_zero 0` was acked. Same script as 7c–7f.

| check | 7e / 7f | 7g | verdict |
|---|---|---|---|
| CPAP patient-flow mean, hypopnea vs normal | −2 … −3.5 slm | **−0.05 … +0.03** in every segment | **R9/R10 fixed** |
| start-up | one start-up CA | same | harness artefact (the CPAP starts first) |
| apnea 5 s | not scored | not scored | correct |
| apneas 10 s ×3, 20 s ×3, 20 s + arousal | CA each | CA each; FOT 6.7–8.6 s, no early exits | correct |
| hypopnea 55 % | one Hyp (7e) / Hyp + tail CA (7f) | **one Hyp at 10.2 s, no FOT** | correct |
| hypopnea 60 % | CA (every run so far) | **one Hyp at 7.0 s, no FOT** | **fixed** |
| hypopnea 50 % | one Hyp | **FL only, no Hyp** | correct by the CPAP's rule; see below |
| FOT during any hypopnea | 12–35 s | **none** | fixed |
| OSA-likely | 2 / 40 | 2 | fine |
| needles | 1 / 0 | 0 | R6 holds |
| after the arousal breaths | no FL | no FL | correct |
| totals | CA 9–10, Hyp 2 | **CA 8 (7 apneas + start-up), Hyp 2, FL 3** | matches the rules |
| CPAP current | FOT ≤ 1.16 A | FOT 1.15 A, breathing ≤ 0.76 A | fine |

**The 50 % hypopnea is now measured correctly, and it sits just above the Hyp
threshold.**

| | CPAP-seen breath peak | % of the CPAP's normal-breath peak (17.6 slm) |
|---|---|---|
| "50 %" hypopnea | 9.0–9.9 slm | **54 %** |
| "55 %" hypopnea | 8.8 slm | 50 % |
| "60 %" hypopnea | 7.8 slm | 45 % |

- The Hyp rule is **peak < 0.50 × baseline** (`g_apap_hyp_ratio`), so 54 % is
  correctly not a Hyp, while the FL rule (< 0.70 ×) did score it.
- In 7e the same event read 38 %, only because the pause-zeroing stage shrank it.

**Whether a 46 % peak reduction should be a hypopnea is a definition choice for the
CPAP team, not a bug.**

- AASM scores a hypopnea from a ≥ 30 % drop, with desaturation or arousal, neither of
  which a CPAP sees.
- Many devices use ~50 % of ventilation (volume) rather than of peak.
- To test the threshold itself, the simulator levels to use are 45 % and 55 %, not
  50 %, which lands within a few percent of it.

**Open items, none blocking:**

1. **Start-up CA.** The harness starts the CPAP before the simulator breathes. Fix it
   in the harness, by starting the simulator first, rather than in the device.
2. **Hyp definition.** Decide peak vs volume and the ratio (above).
3. **Commit the CPAP working tree.** R4–R10 are uncommitted on top of `4ab7a0c`.
   On the BreathSimulator side, the simulator's inlet-estimate fix is uncommitted.

### Trial 8 (20 October): therapy started on a breathing patient → FOT never ends (R11)

**Change from 7g: the start order only.**

- The harness (`apap_trial8.py`) now has the simulator breathing first, on static
  feed-forward with the CPAP off.
- It then starts AutoSet, switches the simulator to pressure feed-forward, settles
  30 s and starts the same script.
- Same R10 CPAP build (probe `leak set patient_pause_zero 0` acked).

**Start-up is fixed:** the APAP was in MONITOR at script start with every counter at
0, so there is no start-up CA.

**Then the APAP got stuck in FOT:**

- The first 5 s apnea (90–95 s) went to NO_BREATH at ~89.8 s (status time) and FOT
  at ~94 s.
- **FOT then never ended on breathing:** 93–594 s, and again 634–978 s.
- Nothing was scored properly: 0 of 7 apneas, 0 Hyp, CA 2 and FL 0 in total.
- The **CPAP drew > 0.8 A in 1 010 samples** (peak 1.18 A) over ~14 min of
  oscillation. The sustained guard (> 0.8 A for 0.7 s) never tripped, because FOT
  current is spiky.

**The CPAP's own patient flow was normal the whole time.** The breaths should have
ended FOT:

| window | patient-flow mean | breaths |
|---|---|---|
| 40–85 s (before) | +0.02 slm | normal |
| 100–140 s (in FOT) | +0.25 slm | simulator 505–508 mL; CPAP peak 19–20 slm |
| 200–240 s (in FOT) | +0.40 slm | normal |

- Replaying the pre-R5a EPR breath FSM on that flow, every breath after the apnea
  becomes a stamp at the 1.2 s max-hold: **267–284 mL, peak 19–20 slm**.
- That is far above R5a (400 ms, max(30 mL, 0.15 × median)) and the recovery test
  (0.3 × baseline ≈ 5–6 slm).
- So the block is **inside the firmware's breath-stamp / recovery path during FOT**.
  The replay does not model the R7b notch, the R8a median or the R9/R10 changes.
- In 7g, started the other way round, every apnea's FOT ended at the first breath.
  So the trigger looks like **therapy starting on a patient who is already
  breathing**. That is the normal real-world case.

**Candidates** (the CPAP thread can check these quickly; none is observable from
outside):

- R8a's running **median breath volume**: inflated by the qualified "breaths" during
  the pressure ramp-up? Patient flow there swung −40 … +25 slm.
- The R7b **notch band-pass state**: the start-up transient; or the notch output when
  FOT is enabled.
- **`s_resume_ref_slm` / the FL baseline** at NO_BREATH entry.
- **R7a's `s_breaths_seen`** gate interacting with the start-up.

**Proposed:**

1. **Add a read-only diagnostic command**, e.g. `apap diag`, that prints:
   - the last EPR candidate (ms, mL, peak) and whether it was stamped;
   - `g_epr_apap_breath_rejects` and the R8a volume median;
   - `recentBreath()`, `s_resume_ref_slm`, the FL baseline, and the notch state.

   Without it these bench runs can only infer the internals.
2. **Reproduce:** start AutoSet on a breathing patient, then do one 5–10 s apnea.
3. **Bench-side protection, harness only:** a FOT watchdog that ramps down and stops if
   the APAP oscillates continuously for more than ~60 s. The current guard can't see
   spiky FOT current.

**Root cause, found by the CPAP thread:**

- At therapy start on a breathing patient, the EPR flow baseline was seeded from a
  mid-inspiration sample (+11 slm). Its deadband tracker then only saw flow near
  that value, so it stayed there.
- Every inspiration then began below the baseline and ended at the 120 ms minimum
  hold, as a ~23 mL fragment that R5a rejected.
- **Fix (R11):** seed the baseline at 0, and add a slow (`epr_baseline_slow_tau_s`
  30 s) pull toward the mean flow. The build also adds `apap diag`.
- The harness now has a 60 s FOT watchdog.

### Trial 8b (R11 build, simulator-first start, FOT watchdog): 20 October

The probe `apap set epr_baseline_slow_tau_s 30` was acked. `apap diag` was polled
about every 5 s (194 samples, `apap_trial8b_diag.csv`).

| check | trial 8 | 8b | verdict |
|---|---|---|---|
| start-up | clean | **clean**: MONITOR, all counters 0 at script start | **start-up CA gone** |
| FOT ends on breathing | never (~14 min stuck) | **always**; longest continuous FOT 12 s; watchdog not tripped | **R11 fixed** |
| EPR flow baseline (`apap diag` `base=`) | stuck mid-breath | −2.0 … +1.0 slm over the whole run | fixed |
| apnea 5 s | — | not scored | correct |
| apneas 10 s ×3, 20 s ×3, 20 s + arousal | — | **CA each**; FOT 6.6–9.3 s in | correct |
| hypopnea 50 % | — | FL only (51 % delivered; above the 0.50 Hyp rule, as in 7g) | correct by the rule |
| hypopnea 55 % | — | one Hyp at 8.3 s | correct |
| hypopnea 60 % | — | Hyp at 9.0 s, **and again at 34.2 s** (same event) | **double count**, R12 |
| OSA-likely | 7 | **0** | — |
| totals | CA 2, Hyp 0 | **CA 7, Hyp 3** (one a duplicate), FL 3 | one event wrong |
| needles | 0 | **3**, at 5.8–6.0 slm, exactly on the 17.75 s grid (rms 0.08 s) | R6 mostly holds; harmless here (`rej` stayed 0, no exits) |
| CPAP current | > 0.8 A in 1 010 samples | FOT 1.11 A; breathing 0.96 A (3 samples > 0.8 A) | fine |

### R12. A hypopnea right at the 0.50 line is re-counted when its hold expires

**What happened in the 60 % hypopnea** (weak breaths from 749.8 to 789.7 s):

- Hyp at 758.8 s.
- The 20 s hold (`g_apap_hyp_hold_s`) ended at 779.0 s.
- A **second Hyp at 784.0 s**, with the weak breaths still running.

**Why:**

- The CPAP sees these breaths at ~45–49 % of its baseline peak (the diag `flbase`
  ≈ 18.4 slm), right against the 0.50 Hyp ratio.
- R2's re-arm (`s_hyp_need_recovery`) clears on **one** breath that is not
  hypopnea-limited. A single breath fluctuating just over 0.50 is enough, so when
  the hold expired the event was counted again.
- In 7g the same event happened not to cross.

**Proposed: add hysteresis to the re-arm.** Either:

- recovery needs a breath clearly back to normal, e.g. peak ≥ 0.70 × baseline (the
  FL line); or
- it needs ~2 consecutive non-limited breaths.

A one-breath wobble across 0.50 then can't split one event into two.

**Also noted:**

- **The needles:** 3 small ones (~6 slm) remain on the clock-beat grid. The APAP
  ignored them, but the R6 reject/agree path is not catching every torn read.
  Worth a look, low priority.
- **`apap diag` works well.** It made this run readable at a glance (baseline,
  rejects, median volume, exit reference). Keep it.

### R12 result: trial 8b script on the R12 build (20 October)

- **Build:** the Hyp re-arm needs `hyp_rearm_breaths` (2) consecutive breaths at
  ≥ `hyp_rearm_ratio` (0.70) × baseline. The probe `apap set hyp_rearm_ratio 0.7`
  was acked.
- **Run:** same harness and script as 8b (`apap_trial8c.py`, logs
  `apap_trial8c_*`).

| check | expected | result |
|---|---|---|
| start-up | no events | MONITOR, all counters 0 at script start |
| apnea 5 s | not scored | not scored |
| apneas 10 s ×3, 20 s ×3, 20 s + arousal | CA each | **CA each**; FOT 6.5–9.3 s in, always ends on breathing |
| hypopnea 50 % | FL only (54 % of baseline > 0.50) | FL only |
| hypopnea 55 % | one Hyp | **one Hyp** at 13.1 s |
| hypopnea 60 % | one Hyp | **one Hyp** at 9.0 s; the hold ended at 779.0 s with **no re-count** (8b: second Hyp at 784 s) |
| after the arousal breaths | no FL | no FL |
| totals | CA 7 + Hyp 2 | **CA 7, Hyp 2, FL 3**, AHI 31.9 |
| OSA-likely | ~0 | 1 sample |
| FOT | always ends | longest continuous 13 s; watchdog quiet |
| EPR baseline (`apap diag`) | near 0 | −2.0 … +1.3 slm |
| needles | — | 1 in 18.4 min |
| CPAP current | — | FOT 1.09 A; breathing 0.85 A (1 sample > 0.8 A) |

**R12 fixed. Every scored event in this script now matches the CPAP's own rules,
with therapy started on a breathing patient.**

**Open, none blocking:**

1. **Hyp definition.** Peak < 0.50 × baseline leaves a 46 % reduction unscored. That
   is a choice for the team; test it at 45 % / 55 %.
2. **The R6 needles.** 0–3 per run remain on the clock-beat grid, harmless to the
   APAP now.
3. ~~Commit the CPAP working tree~~: done, `9b67c19`.

### Hypopnea threshold test, 45 % and 55 % (20 October, `9b67c19`)

**Set-up:**

- Same harness as 8c: simulator first, FOT watchdog, `apap diag`
  (`apap_hyp4555.py`, logs `apap_hyp4555_*`).
- Script: normal 90 | 3 × (45 % hypopnea 40 s | normal 60) |
  3 × (55 % hypopnea 40 s | normal 60).
- Per event, `hyp_ratio.py` gives each weak breath's **CPAP-seen** peak (T-line
  patient flow) over the median of the normal breaths in the 30 s before it. That is
  the quantity the Hyp rule (< 0.50) and the FL rule (< 0.70) compare.

| # | simulator level | Vt | CPAP-seen peak / normal: median (range) | breaths < 50 % | scored |
|---|---|---|---|---|---|
| 1 | 45 % | 276 mL | 58 % (54–60) | 0 of 8 | FL |
| 2 | 45 % | 276 | 58 % (55–59) | 0 of 8 | FL |
| 3 | 45 % | 275 | 57 % (52–60) | 0 of 8 | FL |
| 4 | 55 % | 226 | **47 %** (46–50) | 7 of 8 | **Hyp** (21.5 s in) + FL |
| 5 | 55 % | 226 | **47 %** (44–49) | 8 of 8 | **Hyp** (7.1 s in) + FL |
| 6 | 55 % | 225 | **50 %** (49–54) | 4 of 8 | FL only |

- No CA, no FOT at all (watchdog: 0 s), OSA-likely 0, clean start.
- Totals: Hyp 2, FL 6, AHI 9.6.
- CPAP current peaked at 0.82 A.
- Earlier runs on the same build fit the same line: 50 % → 54 %, FL only (8c);
  55 % → 49 %, Hyp (8c); 60 % → 42 %, Hyp (8c).

**What this shows:**

- **The Hyp rule works exactly as written, and the boundary is sharp:**
  - CPAP-seen median ≤ ~47–49 % → Hyp every time (4 of 4 across runs);
  - ≥ ~54 % → never (4 of 4);
  - ~50 % → a coin flip (event 6).
- A 2 % change decides it. Event 6 came after the APAP had raised pressure from 8 to
  10.6 cmH₂O over the run, and the simulator delivered it ~2 % shallower than events
  4–5.
- **By AASM's ≥ 30 % reduction, all six events are hypopneas.** The CPAP scores the
  45 % ones (a ~42 % peak reduction) as flow limitation only.
  - That is the decision for the team: keep peak < 0.50, move toward ~0.70 (with
    FL and Hyp then overlapping), or use a volume/ventilation measure instead of
    peak.
  - A device without SpO₂/arousal input usually leans conservative, so the current
    rule is defensible. It is just stricter than the clinical definition.
- **For future tests:** the simulator's 55 % level lands on the line, so use 50 %
  (clearly not Hyp at this rule) and 60 % (clearly Hyp) for pass/fail runs, and keep
  55 % for probing the boundary.

## AirSense 11 reference run (20 October)

**Set-up:**

- Same rig with a ResMed AirSense 11 in place of our CPAP (its own hose): vent →
  external sensor → chamber (squeeze blower off) → simulator box → 2 L lung.
- AirSense settings: AutoSet 8–12 cmH₂O, EPR off, ramp/AutoRamp off, response
  standard.
- Same script and start order as our trial 8c (simulator breathing first).
- Harness `airsense_run.py 8c`; SD-card files compared with `airsense_compare.py`
  (dependency-free EDF+ reader `airsense_edf.py`).
- The AirSense clock was **+2 h 32 min 38 s** ahead of the PC. It was aligned on its
  25 Hz flow (BRP) against the external sensor, r = 0.995.
- The simulator delivered the same as in 8c: normal 500 mL; apnea breaths ~20 mL;
  hypopneas 252 / 226 / 201 mL.

| event | AirSense 11 | our CPAP (8c, same script) |
|---|---|---|
| apnea 5 s | not scored | not scored |
| apneas 10 s ×3 | **CA** each, annotated at the apnea's end, duration 12 s | **CA** each |
| apneas 20 s ×3 | **CA** each, 22 s | **CA** each |
| apnea 20 s + arousal | **CA**, 21 s | **CA** |
| hypopnea 50 % | **nothing** | FL |
| hypopnea 55 % | **nothing** | **Hyp** + FL |
| hypopnea 60 % | **nothing** | **Hyp** + FL |
| therapy pressure | **8.0 cmH₂O throughout**: never raised | raised to ~10 cmH₂O |
| flow-limitation index (`FlowLim.2s`) | 0.00 throughout, including the hypopneas | FL scored in all three hypopneas |
| unintentional leak | 0 L/min | ~12 L/min |
| totals | **CA 7** | CA 7, Hyp 2, FL 3 |

Both devices also oscillate during apneas: the AirSense's probe is visible on the
external sensor in every apnea.

### What it says about our APAP

1. **Central apnea detection matches the reference exactly:** the same 7 events, and
   the same choice not to score 5 s.
2. **Hypopnea: we score more than the AirSense does, because it measures
   ventilation, not peak flow.** Its own 2 s records over each window:

   | | true Vt | AirSense TidVol | AirSense minute ventilation | AirSense flow peak |
   |---|---|---|---|---|
   | normal | 500 mL | 460 mL | 5.62 L/min | 18.2 L/min |
   | hypopnea 50 % | 250 | 320 (70 %) | 3.75 (**67 %**) | 9.6 (53 %) |
   | hypopnea 55 % | 226 | 300 (65 %) | 3.50 (**62 %**) | 8.7 (48 %) |
   | hypopnea 60 % | 201 | 280 (61 %) | 3.31 (**59 %**) | 7.9 (43 %) |

   - By peak flow it saw the same depths our CPAP sees (43–53 %).
   - By ventilation it saw only a **33–41 % reduction**: its tidal-volume estimate
     shrinks less than the real breaths do.
   - That is consistent with a ventilation-based hypopnea rule at ~50 %, which none
     of these events reached. ResMed's exact rule is not public in detail, so this is
     an inference.
   - Our peak-based rule scores the 55 % and 60 % events. So **ours is the more
     sensitive of the two on this rig, not the stricter**, which turns the earlier
     worry around.
   - To match the AirSense, the Hyp criterion would move to ventilation (or
     breath volume) with a ~50 % reduction.
3. **Pressure titration is the biggest difference.**
   - The AirSense never left 8.0 cmH₂O. Its flow-limitation index stayed at 0, because
     these breaths are round (unflattened) half-sines, and it does not raise pressure
     for central apneas.
   - Our CPAP raised pressure to ~10 cmH₂O. The cause is its FL rule's
     **amplitude arm** (peak < 0.70 × baseline), which fires on any reduced-effort
     breath, plus Hyp raises.
   - These simulated events are **central** hypopneas: open airway, lower effort, no
     flattening. Raising pressure for them is what the reference device avoids, and
     clinically it can do harm.
   - **Recommended:**
     - only titrate for FL on a **shape** (flattening) signal, not on amplitude alone;
     - and/or don't raise for a hypopnea whose breaths are not flow-limited in shape.

**Next reference runs:**

- **The threshold script (45 % / 55 %).** Expect no AirSense hypopneas at all.
- **FL shapes at full amplitude** (plateau, M): where the AirSense's FL index should
  respond and drive pressure up, the case our shape arm should match.
- **Chamber-squeezed apneas:** does the AirSense call them obstructive on this rig?
  That is the validity check for any obstructive test of ours.

### AirSense 11: flattened breaths at full volume (20 October, `flshape` script)

**Script:** normal 90 s | **plateau 90 %** 120 s | normal 120 s | **M 90 %** 120 s |
normal 120 s | **plateau 60 %** 120 s | normal 120 s. All breaths stay at 500 mL.

**Run:** same rig and settings as the 8c reference run. Harness
`airsense_run.py flshape`, which switches the simulator's `fl_shape` in the normal
gaps. Clock +2 h 32 min 38 s, flow r = 0.995. The AirSense session is
`20261020_2034*`.

| block | delivered flatness (mean/peak) | peak vs normal | AirSense FlowLim index, median / max | AirSense pressure: onset → end → end + 60 s |
|---|---|---|---|---|
| normal breathing | 0.58 | 100 % | 0.00 | 8.0 |
| plateau 90 % | **0.82** | 71 % | 0.28 / 0.42 | **8.0 → 12.0** → 11.9 (at its 12 cmH₂O max) |
| M 90 % | **0.73** | 80 % | **0.45 / 0.60** | 11.7 → 12.0 → 11.9 |
| plateau 60 % | 0.75 | 77 % | 0.04 / 0.09 | 11.7 → 11.5 → 11.3 |

- No events are annotated in EVE: the AirSense reports flow limitation only through
  the PLD `FlowLim.2s` index and through pressure.

**AirSense titration, as observed:**

- **Up:** FlowLim rises within 1–2 breaths of the shape changing. Pressure climbs at
  **~3 cmH₂O/min** (8 → 12 in ~75 s) and pins at the set maximum.
- **Down:** about **−0.3 cmH₂O/min** between blocks. FlowLim clears ~20 s after
  breathing returns to normal.
- With the 8c run: **it raises pressure for flattened breaths, never for rounded
  low-amplitude ones** (hypopneas, central apneas).

**What our APAP would do with the same breaths** (prediction from its rules, not yet
run):

| block | flat > 0.78? | peak < 0.70 × base? | predicted | AirSense |
|---|---|---|---|---|
| plateau 90 % | yes (0.82) | borderline (0.71) | FL → raise | strong FL, raised to max |
| M 90 % | **no (0.73)** | no (0.80) | **missed: no FL, no raise** | **strongest FL**, held at max |
| plateau 60 % | no (0.75) | no (0.77) | missed | barely (≤ 0.09), drifting down |

**Conclusions:**

1. **The M shape is our biggest flow-limitation gap.**
   - The mean/peak flatness index puts it at 0.73, under the 0.78 line. It was a
     known blind spot in the simulator notes.
   - The AirSense's index rates it as the **most** flow-limited of the three.
   - A shape measure that sees a mid-inspiratory dip is needed, for example:
     - the inspiratory peak-to-valley ratio,
     - curvature,
     - or comparison against a template of the breath's own rounded shape.
2. **The plateau threshold looks close to right.** The AirSense barely reacted to
   flatness 0.75 and reacted strongly to 0.82, which brackets our 0.78.
3. **Titration rates to compare** once our CPAP runs this script: up at about
   3 cmH₂O/min, down at about 0.3 cmH₂O/min.
4. With the 8c finding: the AirSense titrates on **shape**, not amplitude. That
   supports dropping the amplitude-only arm of our FL rule for pressure raises.

**Next:**

- Run this same script on our CPAP with the trial harness, to replace the
  predictions above with measurements.
- Then the chamber-squeezed obstructive test on the AirSense.

### Our CPAP on the same flattened-breath script (20 October, `9b67c19`)

- **Run:** `apap_flshape.py`, the trial 8c harness with the `flshape` script.
  Simulator first; FOT watchdog; current peak 0.80 A.
- **Report:** `cpap_fl_report.py`.
- The predictions above were **wrong**: our CPAP scored FL in all three blocks.

| block | delivered flatness (full breath) | our CPAP | AirSense 11 |
|---|---|---|---|
| plateau 90 % | 0.82 | **FL** (flag on 91 % of the block); target 8.0 → 8.7 → 8.8 | FL 0.28 / 0.42; **8.0 → 12.0** in ~75 s |
| M 90 % | 0.73 | **FL** (88 %); 8.8 → 10.0 | FL 0.45 / 0.60 (strongest); held at 12 |
| plateau 60 % | 0.75 | **FL** (77 %); 10.1 → 10.6 | barely (≤ 0.09); easing down |
| no CA / Hyp / FOT | — | correct | correct |
| fastest pressure rise | — | **0.8 cmH₂O/min** | ~3 cmH₂O/min |
| pressure between blocks | — | **held, never lowered** within the 2 min gaps | fell ~0.3 cmH₂O/min |

### R13. The CPAP's flatness index only sees the first ~1.2 s of each inspiration

**Code (`apap_ctrl.c`, `updateFlHypDetector`):**

- The mean/peak flatness is accumulated only while `Epr_GetPhase()` is
  INSPIRATION.
- That phase ends at the EPR breath stamp: the 1.2 s `insp_max_hold` (+ 50 ms
  confirm), or earlier on a peak-fall.
- With this rig's 2.5 s inspirations it ends at max-hold every time. The replay found
  all breaths in all blocks ending there, at ~1260 ms.

**Replay** (`epr_replay.py` on the logged patient flow, 50 Hz, so approximate): mean
÷ peak over the window the detector actually uses.

| breaths | full-breath flatness (simulator) | flatness in the CPAP's window |
|---|---|---|
| **normal** | 0.59 | **0.78–0.80** |
| plateau 90 % | 0.82 | 0.85 |
| M 90 % | 0.73 | **0.91** |
| plateau 60 % | 0.75 | 0.86 |

**Consequences:**

1. **Normal breathing sits on the 0.78 FL line.** There is essentially no margin
   against false FL; only the 3-breath persistence keeps it quiet here.
2. **The M shape is caught by accident.** The first half of an M breath looks like a
   plateau, and its defining mid-inspiratory dip is never seen. The same applies to
   any shape whose flattening is in the second half (e.g. "late"), which would be
   missed.
3. **FL sensitivity depends on inspiratory time and breathing rate:**
   - with Ti ≤ ~1.2 s the window covers the whole breath and the index drops
     (normal ~0.59);
   - with long Ti it judges only the rising half.
   - The same patient can flip between "flow-limited" and not by breathing slower.
4. Plateau 60 % is scored strongly here; the AirSense barely reacted to it.

**Proposed:**

- **Compute flatness over the whole inspiration**, from flow onset to the actual end
  of inspiration (flow crossing back to zero), not over the EPR relief phase.
- Then re-tune against the full-breath values above:
  - normal ~0.59–0.65;
  - plateau 90 % 0.82, plateau 60 % 0.75;
  - M 0.73, which a mean/peak index alone then misses.
- So also add a mid-inspiratory **dip** feature for M-type limitation: e.g. the
  valley-to-peak ratio between the two halves, or a shape template.

**Pressure response**, compared with the AirSense:

- **Ours rises about 4× slower:** 0.8 vs ~3 cmH₂O/min.
- **Ours did not come down at all** in 2 min of normal breathing between blocks. The
  AirSense came down at ~0.3 cmH₂O/min, and 8c also showed ours never lowering.
- Worth checking the lowering path (`g_apap_lowering`, the stable-time requirement)
  in a longer normal-breathing run.

### Pressure lowering on our CPAP (20 October, `9b67c19`)

**Run:** `apap_lowering.py` in BreathSimulator `pc_tools/apap_bench`. Script:
normal 60 s | plateau 90 % 180 s, to raise the pressure | normal 720 s. Breaths are
500 mL throughout; simulator first; current peak 0.79 A.

**Firmware path:** lowering only starts after `g_apap_stable_before_lower_s`
(120 s) with no FL, Hyp or snore active. It then runs at
`g_apap_lower_rate_cmh2o_per_s` 0.008, i.e. ~0.5 cmH₂O/min.

| | our CPAP | prediction (before the run) | AirSense 11 (flattened-breath run) |
|---|---|---|---|
| rise during the 3 min plateau block | 8.0 → **9.2 cmH₂O** (0.4–0.6 cmH₂O/min) | ~9.5–10 | 8 → **12** (its max) in ~75 s, ~3 cmH₂O/min |
| FL flag clears after the block | 15 s | 20–40 s | index clears ~20 s |
| lowering ("Monitor+Lower") starts | **136 s after the block** (15 s + the 120 s stable wait) | 2–2.5 min | immediately |
| rate down | **0.48 cmH₂O/min** | ~0.5 | ~0.3 cmH₂O/min |
| back at the minimum | ~4.7 min after the block | — | still at ~11 when its run ended |

No CA, Hyp or FOT.

**Conclusions:**

1. **Lowering works as coded, but needs 2 min of clean breathing first.**
   - That is why trial 8c and the flattened-breath run never lowered: their gaps
     between events were ≤ 120 s, and the FL flag ate the first ~15–40 s of each.
   - In a real night of frequent events, the target could only ratchet up.
2. **The rise is the bigger gap.** On clear flow limitation, ours raised 1.2 cmH₂O
   in 3 min; the AirSense raised ~4 cmH₂O in 75 s. That is **~5–7× slower**, so a
   patient with flow limitation would stay under-treated for several minutes.
3. **The two devices take opposite strategies:**
   - ours: slow up, hold, then a moderate fall;
   - the AirSense: fast up, then an immediate slow fall.
   - The raise rate (`applyRaise` with the FL severity scaling) and the hold/fall
     pair are tuning decisions for the team. The AirSense numbers above are a
     reference point, not a target.

### R13 result: whole-inspiration shape (23 October)

**Change (`apap_ctrl.c`):**

- **One segment per inspiration.** A breath is now the stretch of patient flow above
  `fl_seg_floor` (0 slm) that contains an EPR inspiration onset. It is judged when the
  flow returns to the floor, so peak, flatness and the new dip all see the whole
  inspiration.
- **Two safeguards:**
  - A breath already under way when the detectors are reset is skipped, e.g. the
    first breath after FOT, which still carries the 4 Hz ripple.
  - If an inspiration never returns to the floor (a flow offset), it is closed at the
    next onset and judged on its peak only, as before R13.
- **Flatness threshold re-tuned:** `fl_flat_ratio` 0.78 → **0.72** (For Her
  0.74 → 0.70).
- **New dip arm for M-type limitation.** The inspiration is kept in 40 ms bins. The dip
  is the lowest bin between the peaks of the two halves of the breath, divided by the
  smaller of those peaks. Below `fl_dip_ratio` (**0.93**; 0 = off) the breath is
  flow-limited.
- **Telemetry:** `therapy apap events` ends with `flat=` and `dip=` for the last
  breath. New settings `apap set fl_dip_ratio` / `fl_seg_floor`.

**Tuning:**

- The thresholds come from replaying five logs from 12–20 October through a
  line-for-line Python port of the new code.
- On the bench, the firmware's per-breath flat and dip matched that port to
  **±0.003**.

| breaths (CPAP patient flow) | whole-inspiration flatness | dip |
|---|---|---|
| normal | 0.61–0.68 | ≈1.0 |
| hypopneas 45–60 % | ≤ 0.68 | ≈1.0 |
| plateau 90 % | 0.74–0.82 | ≈0.97 |
| plateau 60 % | 0.73–0.77 | ≈0.97 |
| M 90 % | 0.67–0.75 | median 0.86–0.90 |

**Bench:** the `flshape` script on our CPAP (`apap_flshape.py`, which now also probes
`apap set fl_dip_ratio`). AutoSet 8–12, EPR off, simulator first.

| FL flag on, % of block | before R13 (20 Oct) | R13, dip 0.90 | **R13, dip 0.93** |
|---|---|---|---|
| normal breathing | none | none | **none** |
| plateau 90 % | 91 % | 92 % | **92 %** |
| M 90 % | 88 % (by accident) | 25 % (3 short episodes) | **71 %**: one episode, from 36 s in |
| plateau 60 % | 77 % | 92 % | **92 %** |
| FL episodes | 3 | 4 | **3** (one per block) |

- **Dip 0.90 was too tight.** That day the M breaths' dip had a median of 0.90, so
  only about half of them crossed the line. At 0.93, 22 of 25 M breaths are limited
  (median 0.86).
- **In the replay, 0.93 is the highest safe setting.** No normal or hypopnea block
  turns FL on; at most 3/36 hypopnea and 1/745 normal breaths cross it, never 3 in a
  row. At 0.95, false limited breaths begin to rise.
- **Normal breathing on the 0.93 run:** flatness ≤ 0.66, dip ≥ 0.92. One breath
  crossed 0.93, alone. The dip margin is thin: watch it on other breath shapes.
- **The M block is now caught by its shape.** It reads 71 % rather than the 88 % it
  got by accident before, because the first FL call came after the 3-breath
  persistence.
- **No other events:** no OA, CA, Hyp or FOT. Peak CPAP current 0.75 A.
- **Pressure:** 8.0 → 9.9 cmH₂O over the run. The raise rate is unchanged
  (~0.5 cmH₂O/min).

**Limits:**

- One rig: 2 L lung, the simulator's synthetic shapes. Not clinically validated.
- **A +2 slm patient-flow offset** would make the shape arms miss limitation: the
  segment then includes the expiratory pause. That fails safe, with no false raises.
  Raising `fl_seg_floor` restores detection but makes round hypopneas read flat, so
  keep it at 0; the protection is the R10 fix at the source.
- The simulator dropped the plateau 60 % END event on the 0.93 run; that block is
  labelled by the script clock.

**Still open, unchanged by R13:**

1. ~~**The raise rate:** ~0.5 cmH₂O/min on FL, against ~3 for the AirSense.~~ Fixed,
   see the next section.
2. ~~**Lowering** waits 120 s with no events.~~ Fixed, see the next section.
3. **The FL amplitude arm** (peak < 0.70 × baseline) still raises pressure on
   central, unflattened hypopneas.

### Raise rate and lowering wait (23 October)

**Why it was slow (`apap_ctrl.c`, `applyMonitorTitration`):**

- **Raise.** The FL raise was `raiseRate()` (standard 0.05 cmH₂O/s, ~3/min) ×
  `fl_raise_scale` 0.50 × a shape severity.
  - The severity was measured over the whole span from the threshold up to 1.0.
  - A plateau at flatness 0.80 against the 0.72 line therefore scored only 0.29, and
    the M dip scored about the same. Both sit at the 0.25 floor.
  - The net rise was ~0.4–0.5 cmH₂O/min.
- **Lowering.** It began only after `stable_before_lower_s` 120 s with no FL, Hyp or
  snore, then ran at 0.008 cmH₂O/s (0.48/min).
  - Every gap in an eventful script was ≤ 120 s, so the target could only ratchet up.

**Change:**

| setting | before | now |
|---|---|---|
| `fl_raise_scale` (For Her as well) | 0.50 (For Her 0.55) | **1.00**; For Her stays gentler via its soft response and her scale |
| FL shape severity | (shape − threshold) / (span to 1.0) | (shape − threshold) / **0.10** (`FL_SEV_SPAN`), for flatness and dip alike |
| `stable_before_lower_s` | 120 s (clamp 30–600) | **20 s** (clamp 5–600) |
| `lower_rate` | 0.008 cmH₂O/s (0.48/min) | **0.005** (0.30/min) |

- Expected raise: plateau 90 % (flat ~0.80) ~2.4 cmH₂O/min, M 90 % (dip ~0.86) ~2.1,
  plateau 60 % (flat ~0.75) ~0.9.
- Hyp and snore raises are unchanged. GUI defaults updated.

**Bench 1: pressure lowering** (`apap_lowering.py`: normal 60 | plateau 90 % 180 |
normal 720; AutoSet 8–12):

| | before (20 Oct) | **now** | AirSense 11 |
|---|---|---|---|
| rise during the plateau block | 8.0 → 9.2 (0.4–0.6 cmH₂O/min) | **8.0 → 12.0 (max) in 2.0 min from the FL call, ~2.0/min** | 8 → 12 in ~75 s, ~3/min |
| FL flag clears after the block | 15 s | 10 s | ~20 s |
| lowering starts | 136 s after the block | **32 s after** | immediately |
| fall rate | 0.48 cmH₂O/min | **0.30**, a smooth ramp (residual sd 0.003) | ~0.3 |
| end of the 12 min normal block | back at the 8.0 minimum after ~4.7 min | 8.6 and still falling, never re-raised | still ~11 when its run ended |

- FL came on 14 s into the block.
- No OA, CA or Hyp; no FOT.
- Peak CPAP supply current 0.85 A (3 samples > 0.8 A, at 12 cmH₂O).

**Bench 2: flattened breaths** (`apap_flshape.py`, the same script as R13). Pressure
target onset → block end → end + 60 s:

| block | R13 build (before this change) | **raise / lowering build** | AirSense 11 (20 Oct) |
|---|---|---|---|
| plateau 90 % | 8.0 → 8.8 → 8.8 | **8.0 → 12.0 → 11.8** | 8.0 → 12.0 → 11.9 |
| M 90 % | 8.8 → 9.2 → 9.2 | **11.6 → 12.0 → 11.8** | 11.7 → 12.0 → 11.9 |
| plateau 60 % | 9.2 → 9.8 → 9.9 | **11.5 → 12.0 → 11.8** | 11.7 → 11.5 → 11.3 |
| fastest rise / fall | 0.5 / −0.5 cmH₂O/min | **2.9 / −0.4** | ~3 / −0.3 |
| FL flag on, % of block | 92 / 71 / 92 % | **91 / 88 / 91 %** | — |

- **The rise and fall now track the AirSense on plateau 90 % and M 90 %.**
- **No hunting.** In each 2-minute normal gap, FL cleared, the target held ~20 s and
  then fell at 0.3 cmH₂O/min. FL never re-fired between blocks.
- **FL episodes:** 3, one per block; no OA, CA, Hyp or FOT.
- **Plateau 60 %:** ours raises a little (11.5 → 12.0), where the AirSense drifts
  down. Both are near the 12 max here. If milder flattening should be ignored, widen
  `FL_SEV_SPAN` or raise the 0.72 flatness line.
- **Current:** peak 0.88 A, one sample > 0.8 A, at 12 cmH₂O. The APAP now reaches
  high pressure sooner, so watch the 2026-10-05 fuse limits with wider AutoSet ranges
  (e.g. 4–20).

**Limits:** one rig, synthetic breaths; not clinically validated.

**Still open:** the FL amplitude arm (peak < 0.70 × baseline) still raises pressure on
central, unflattened hypopneas. The AirSense does not.

## Evidence

- **Logs:** BreathSimulator session scratchpad,
  `apap_trial{,2,3,4,5,6}_{cpap,status,sim}.csv` and
  `apap_trial{3,4,5,6}_current.csv`. R5 capture: `apnea_cap_raw.pkl` (the raw
  `T,`, `PH,` and status lines), `apnea_cap_T.csv`, `apnea_cap_apnea{1..6}.csv`, and
  the harness `apnea_capture.py`. R6 spike source: `spike_raw.pkl`,
  `spike_events.csv`, `spike_source.png`, and the harness `spike_source.py` with
  `spike_analysis.py`. Trials 7, 7b–7g and 8: `apap_trial7{,b,c,d,e,f,g}_raw.pkl`, `apap_trial8{,b,c}_raw.pkl` (+ `apap_trial8{b,c}_diag.csv`), the threshold test `apap_hyp4555_raw.pkl` + `hyp_ratio.py`, the AirSense runs `airsense_8c_1637_raw.pkl` + `airsense_8c_1637_compare.png` and `airsense_flshape_1803_raw.pkl` + `airsense_flshape_1803_compare.png`; our CPAP on the same script `apap_flshape_raw.pkl` + `cpap_fl_report.py`, the pressure-lowering run `apap_lowering_raw.pkl` + `apap_lowering.log` (SD-card files in BreathSimulator `airsense_data/20261020`), the R13 runs in
  BreathSimulator `pc_tools/apap_bench/runs/` (`pre_r13/` = 20 Oct baseline,
  `r13_dip090/` = first R13 flash, `r13_dip093/` = dip 0.93), the raise / lowering
  runs (`apap_lowering_*` and `apap_flshape_*` in the same folder), and
  `apap_trial7{,b,c,d,e,f,g}.log`, plus `needle_count.py`, `leak_source.py` + `leak_analysis.py` (R9: `leak_raw.pkl`, `leak_breaths.csv`, `leak_source.png`), `total_source.py` + `total_analysis.py` (R10: `total_raw.pkl`, `total_breaths.csv`, `total_source.png`) and `epr_replay.py`, which replays the EPR breath FSM on
  the logged `T,` data. These are temporary and may be cleared.
- **Summaries:** BreathSimulator project memory, `breathsim-apap-waveforms.md`.
- **Simulator firmware:** BreathSimulator `4492ae8` (the flow-limit event fix and the
  refit feed-forward defaults, both used in trials 5 and 6).
