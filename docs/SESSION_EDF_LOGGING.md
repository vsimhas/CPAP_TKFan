# Therapy session EDF logging (SD card)

## What this is

When therapy starts, CM7 writes one session to the FAT-formatted SD card in the
ResMed card **layout**, which OSCAR's own ResMed loader recognises (only the
layout is borrowed: `Identification.json` carries our identity, not a ResMed
serial):

```
0:/Identification.json, 0:/STR.edf                       (once per card)
0:/DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_BRP.edf   25 Hz waveforms
0:/DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_PLD.edf    1 Hz summary channels
0:/DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_EVE.edf    scored events (EDF+ annotations)
```

The date and time come from the RTC (see Setup). Each therapy start makes a new
set; a second session in the same second bumps the seconds field.

| File | Records | Signals |
|------|---------|---------|
| BRP | 1 s @ 25 Hz | `Flow.40ms` (L/s), `Press.40ms` (cmH2O) |
| PLD | 1 s @ 1 Hz | `MaskPress.2s`, `Press.2s` (target), `Leak.2s` (L/s), `Snore.2s`, `FlowLim.2s` |
| EVE | EDF+D, 64-byte records | `EDF Annotations` + `Crc16` |

**EVE** (checklist #59) follows the AirSense 11's own file byte for byte: the
first record is `Recording starts`, then one record per scored event,
`Obstructive Apnea`, `Central Apnea` or `Hypopnea`. As with ResMed, an event is
stamped at its **end** (seconds from the file start) and carries its duration:

- an apnea is written when its episode ends, with the episode's length;
- a central apnea that may still be re-scored as a hypopnea (R4, 30 s window) is
  held until that window closes, and dropped if re-scored;
- a hypopnea is written when breathing recovers.

Flow limitation and snore have no EVE entry in OSCAR's loader. As on the AirSense,
they are continuous indices in PLD (#60 / #61), which OSCAR plots as its Snore and
Flow Limitation graphs. Both use the AirSense's ranges, and both read 0 when the
session is not scored (`apap set cpap_scoring 0`):

- **`FlowLim.2s`** (0 to 1): the last breath's flow-limitation index. It takes the
  larger of two terms, computed from the same whole-inspiration shape that the FL
  detector uses (R13):
  - `(flatness - 0.66) / 0.20`;
  - `(0.96 - dip) / 0.10`.

  Normal breathing reads 0. The detection lines (flatness 0.72, dip 0.93) read 0.30.
  A plateau or an M notch at 0.86 reads 1. The index goes back to 0 after 10 s with
  no breath. This is **our own scale** on ResMed's range, not ResMed's: on the same
  simulator breaths, the AirSense read plateau 90 % at 0.28 / 0.42 (median / max)
  and M 90 % at 0.45 / 0.60.
- **`Snore.2s`** (0 to 5): while the snore detector is on, the inspiratory
  high-frequency flow RMS divided by its threshold (`snore_rms_thresh`, 0.35 slm);
  otherwise 0. So 1 means "at threshold".
  The detector input is a 2nd-order Butterworth high-pass (`snore_hp_hz`, 20 Hz,
  limited to 5-40 Hz) on raw patient flow, sampled at ~100 Hz, so snore content
  above ~50 Hz is not seen. It counts over the whole inspiration (the R13 segment,
  not just the ~1.2 s EPR phase) and needs `snore_on_s` (1.5 s) of it to turn on;
  expiration neither builds nor clears it.
  Bench 2026-10-27: clean simulated breathing reads ~0.20 slm (rig noise floor),
  so the 0.35 threshold has only ~1.7x margin and wants a check against a real
  snore recording. The simulator's 35 Hz blower modulation does not reach the
  flow, so no positive snore test is possible on the rig; the logging path was
  proven by lowering `snore_rms_thresh` to 0.05.

`STR.edf` is still a header-only stub; the session summary is in `SUMMARY.CSV`
(below).

Inspect a file without a viewer:

```
python pc_tools/edf_inspect.py <file>.edf
```

## Session summary: `0:/SUMMARY.CSV` (checklist #57)

When therapy stops, one line per session is appended to `SUMMARY.CSV` at the card
root (created with a `# ... schema N` comment and a column header). Column 1 is the
schema version: columns are only ever added at the end, with the version bumped.

| Columns | Meaning |
|---------|---------|
| `start`, `end`, `rtc_valid` | RTC time at therapy start / stop; `rtc_valid` 0 = clock not set since the last power loss |
| `run_s`, `mask_on_s`, `mask_off_s`, `mask_off_n` | therapy run time, of which with the mask on / off; mask-off episodes |
| `mode`, `p_cpap`, `apap_min`, `apap_max`, `epr_type`, `epr_level`, `ramp_min` | settings (`p_cpap`: set pressure in CPAP, last target in AutoSet) |
| `p_avg` ... `p95` | mask pressure avg / min / max / P50 / P90 / P95 (mask-off excluded) |
| `leak_avg`, `leak_p50`, `leak_p95`, `leak_max`, `leak_high_s`, `leak_low_s` | unintentional leak (L/min) and seconds above the high / below the low threshold |
| `events_scored`, `oa`, `ca`, `hyp`, `fl`, `snore` | scored events; scored in AutoSet and in CPAP mode (#33: detectors on, no titration); `events_scored` 0 only with `apap set cpap_scoring 0` |
| `ahi`, `rdi`, `si` | per hour of **mask-on** time |
| `edf` | the session's DATALOG name, e.g. `20261026_140818` |
| `fl_idx_avg`, `fl_idx_p95` | flow-limitation index (`FlowLim.2s` scale), mean and P95 over the session's breaths (schema 2) |
| `fl_s`, `snore_s` | seconds with the FL flag / snore detector on (schema 2) |

Schema 2 (#60 / #61) added the last four columns. When a card already holds a schema-1
file, a `# schema 2` comment and the new header line are appended before the next
record. Readers (`adherence.c`, `adherence_check.py`) re-map the columns at every
header line.

The record is snapshotted in RAM at stop and written to the card. The card is mounted
whenever one is in and has passed its self-test, **whether or not EDF logging is
enabled** - `therapy edf enable` only switches the BRP/PLD/EVE waveform files. Pulling
the card unmounts it (open files are dropped without touching the missing card); a
failed mount is retried on the next insertion or `therapy edf enable 1`. `therapy summary` prints the last record over USB (`# SUMR`) with
its column header (`# SUMH`), and `pending/written/err` for the card write.

## Adherence (checklist #65)

Computed on the device from `SUMMARY.CSV`: the file is read once each time the card is
mounted (columns found by header name), and every new session record is added as it is
written. `therapy adherence` prints, for example:

```
# ADH built=1 clock=1 night=2026-10-26 days=30 used=25 ge4h=20 pct4h=67 avg_min=236
  avg_used_min=284 first=2026-07-03 since=116 sessions=113 undated=1 total_h=485 best30=23 met=1
```

- **Night:** a session counts toward the night it **started** in, split at noon like
  ResMed (a start before 12:00 belongs to the previous date). Usage is `mask_on_s`.
- **Window:** the last 30 nights ending tonight (tonight included), never before first
  use: `used` (any usage), `ge4h` (>= 4 h), `pct4h`, `avg_min` over all nights and
  `avg_used_min` over nights used.
- **Reimbursement test:** `best30` = most nights >= 4 h in any 30 consecutive nights
  within the first 90 from first use; `met=1` at >= 21 (70 %). `/partial` while fewer
  than 30 nights exist.
- **Clock:** `clock=0` means the RTC is not set (e.g. after a power cycle without a VBAT
  battery); tonight is then the last night that has a session. Records without a valid
  clock count as `undated` and are left out. Local time as set; no DST / time zone.

Cross-check a card on the PC with the same definitions:

```
python pc_tools/adherence_check.py SUMMARY.CSV --now "2026-10-26 16:06:11"
```

## Fault log: `0:/FAULTS.CSV` (checklist #86)

Every fault is recorded with the state around it, not just the code, and appended to
`FAULTS.CSV` at the card root (schema comment + header, like `SUMMARY.CSV`):

- **therapy** faults: every safety-supervisor latch (`Sensor/Hose Lost`, `Over Pressure`,
  `Sensor Out Of Range`, `Cannot Reach Target`, `CM4 Heartbeat Lost`), captured at the
  moment of the latch, before the blower is stopped;
- **drive** faults: new bits in the CM4 occurred-fault word (`OVER_VOLT`, `OVER_CURR`,
  `DP_FAULT`, ... from MCSDK `mc_type.h`);
- **system** faults: an abnormal last reset (`watchdog`, `cm4-watchdog`, ...), recorded at
  boot with the reason left before the reset (stalled task, or fault handler and PC) - see
  [WATCHDOG.md](WATCHDOG.md). `code` is the reset-flag word `RCC_C1_RSR >> 16`.

Columns: `seq, time (RTC), rtc_valid, uptime_s, source, code, name, detail,
therapy_state, mode, run_s, target, setpoint, meas, flow, leak, rpm_cmd, rpm_mech, iq_ma,
id_ma, vbus_mv, mc_state, cur_faults, occ_faults`.

The last 16 records stay in RAM; `faults` prints the count, how many are still waiting
for the card (`pending`) and the 8 most recent (`# FLT ...`, newest first). Records made
while no card is in are written at the next mount.

## OSCAR

Copy the card (or its `DATALOG`, `STR.edf`, `Identification.json`) and import it in
OSCAR as a ResMed card. BRP gives the flow / pressure graphs, PLD the leak and
target pressure, and EVE the event flags (OA / CA / H) on the flow graph.

## Setup

1. Format the SD card as **FAT32** on a PC (not exFAT if possible).
2. Insert card; wait for SD self-test OK.
3. Set the clock once (GUI: "Set RTC to PC clock"). The CM7 RTC keeps it
   across resets, and across power cycles with a battery on VBAT; each session
   is stamped from it:
   ```
   rtc set 26 08 19 22 30 00
   rtc status
   ```
   `rtc status` prints `src=` LSE (32.768 kHz crystal) or LSI (internal,
   about +-5 %: reset the clock now and then), `valid=1` once the time has
   been set since the backup domain was last lost (until then the clock
   counts from 2026-01-01), and `now=`.
4. Enable logging (off by default so USB telemetry is never blocked by FatFs):
   ```
   therapy edf enable 1
   ```
5. `therapy start` … `therapy stop`
6. Check status:
   ```
   therapy edf status
   ```
7. Remove card and open the `.EDF` shown in `path=` in EDFbrowser.

## USB commands

| Command | Effect |
|---------|--------|
| `therapy edf status` | Enable/state/path/records/error |
| `therapy edf enable 0\|1` | Disable/enable auto-log on therapy start |
| `therapy edf datetime YY MM DD HH MI SS` | Sets the RTC (same as `rtc set`) |
| `rtc set YY MM DD HH MI SS` | Sets the RTC; sessions are stamped from it |
| `rtc status` | Clock source, running, valid, current date/time |

## Status fields

`therapy edf status` prints `rec` (records written), `fsize` (last known file size) and
`err`. A healthy session shows `fsize=1280` immediately after start (header only), then
`fsize = 1280 + 200 * rec`.

| `err` | Meaning |
|-------|---------|
| `0x0` | No error |
| `0x1000` \| `0x1001` \| `0x1002` | No card / SD not ready / FatFs settling |
| `0x2S xx` | `Edf_Open` failed at stage `S`, FatFs code `xx` |
| `0x3000` | Data-record write failed (retried up to 8 service ticks) |
| `0x31xx` | `Edf_Close` failed with FatFs code `xx` |

`Edf_Open` stages: `1` create, `2` header write, `3` header sync, `4` size verify.
So `0x2301` means the header sync (directory-entry commit) hit `FR_DISK_ERR`.

`dio_err` is the raw SDMMC error of the last failed transfer (`0x4` = command response
timeout, `0x80000000` = timeout). It is cleared on the next successful write.

## SD write strategy

The SDMMC path on this board is intolerant of frequent FAT metadata updates, so the
writer is deliberately conservative:

- SDMMC **hardware flow control is enabled**. Without it, RTOS and motor-control
  interrupts preempting the polling loop starved the write FIFO and produced
  `SDMMC_ERROR_TX_UNDERRUN` (`sd_hal=0x10`), which failed both the SD self-test
  write step and EDF logging.
- Single-block (`CMD24`) transfers only; multi-block polling writes produced
  `SDMMC_ERROR_CMD_RSP_TIMEOUT`.
- Up to 8 one-second records are queued in RAM and written in one `f_write`.
- The header's `number of data records` field is patched **only on close** — seeking
  back to the header mid-session was aborting recordings.
- Data is `f_sync`'d every 5 records so the directory size keeps up even if the
  session ends badly.
- Writes retry 4× (FatFs level) and 6× (block level) before a session is failed.

## Notes

- Logging soft-fails if the card is missing or not FAT-mountable; therapy still runs.
- Raw “dummy sector” SD test is skipped while EDF logging is enabled (avoids FAT corruption).
- Always **Stop Therapy** before pulling the card, otherwise the record count stays `-1`
  and EDFbrowser will refuse the file.
