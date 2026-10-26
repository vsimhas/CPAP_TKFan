# CM7 watchdog (checklist #80)

`CM7/Core/Src/watchdog.c`, `CM7/Core/Inc/watchdog.h`.

## What it does

IWDG1 (independent watchdog, LSI clock) with a ~1 s timeout (0.94 to 1.07 s across the
LSI tolerance). Once started it cannot be stopped. A timeout resets the whole system,
both cores. CM4 restarts too and the blower drive stops.

**SafetyTask** (realtime priority, 5 ms) is the only task that refreshes IWDG1. It
refreshes only while every monitored task has checked in (`Watchdog_Kick`) within that
task's own deadline. If SafetyTask alone kept refreshing, a stuck task would go
unnoticed:

| Task | Deadline | Loop | Why this deadline |
|------|----------|------|-------------------|
| sensors | 500 ms | 5 ms | 4 SPI transfers per tick x 100 ms HAL timeout |
| system | 1 s | 10 ms | USB command / log pump |
| gui | 2 s | ~16 ms | one TouchGFX frame |
| climate | 3 s | 250 ms | blocking Sensirion I2C reads; humidifier heater loop |
| nfc | 5 s | 10 ms | RFAL worker |
| selftest | 60 s | 10 ms | SD / FatFs (HAL SD timeouts are 30 s), QSPI/SDRAM tests |

- **Startup grace.** A task that has not checked in yet gets 20 s from start, which
  covers its one-time bring-up.
- **A missed deadline latches.** Refreshing stops for good, so IWDG1 resets within about
  1 s.
- **CPU wedged.** If the CPU itself is stuck (a fault handler loop, interrupts off, or the
  scheduler dead), SafetyTask cannot run and IWDG1 fires on its own. Before this change,
  a HardFault or a call to `Error_Handler()` stopped the device until someone power-cycled
  it.

IWDG1 is frozen while the debugger halts CM7, so breakpoints still work. It starts from
SafetyTask, so boot (CM4 sync, SDRAM, TouchGFX init) runs without it.

## Why the last reset happened

The reason is written to RTC backup registers BKP15R..BKP17R just before the reset. The
backup domain survives a system reset.

| BKP17R | Written by | BKP16R | BKP15R |
|--------|------------|--------|--------|
| `0xA501_00tt` | SafetyTask, when task `tt` misses its deadline | age, ms | - |
| `0xA502_00xx` | HardFault / MemManage / BusFault / UsageFault (`xx` = 'H' 'M' 'B' 'U') | PC (H, U) | CFSR |
| `0xA503_00ss` | `Error_Handler()` (`ss` = boot_stage) | caller address | - |

Ownership of the backup registers:

- BKP0R: RTC
- BKP15R..BKP17R: watchdog
- BKP18R..BKP19R: boot trace

At boot, `Watchdog_CaptureResetCause()` (in main, before `RtcClock_Init()`) latches
`RCC_C1_RSR` and the note, then clears both. If the reset was abnormal (IWDG1, IWDG2,
WWDG or low-power), SafetyTask writes a fault-log record with source `system` once the RTC
is running (so it is timestamped; undated after 10 s without one)
to `FAULTS.CSV`:

- `code`: `RCC_C1_RSR >> 16`
- `name`: `watchdog`, `cm4-watchdog`, `window-watchdog` or `low-power`
- `detail`: one of
  - `system task stalled 1005 ms`
  - `UsageFault pc=0x08012345 cfsr=0x00010000`
  - `Error_Handler from 0x...`
  - `no note - CPU or interrupt lock-up`

## USB commands

| Command | Effect |
|---------|--------|
| `wdg` | `# WDG run= timeout_ms= tripped= reset= rsr= note=`, then one `# WDGT name deadline_ms= max_gap_ms= age_ms=` line per task |
| `wdg clear` | Reset the per-task worst-gap statistics |
| `wdg test stall` | SystemTask spins, which exercises the task-deadline path. Expect reset `watchdog`, note `system task stalled ~1000 ms` |
| `wdg test hang` | Interrupts off and spin, which exercises the hardware path. Expect note `no note - CPU or interrupt lock-up` |
| `wdg test fault` | Undefined instruction, then the UsageFault loop. Expect note `UsageFault pc=...` |

The `wdg test` commands are refused unless therapy is idle and the blower is stopped:
a reset cuts the drive without a ramp-down. Each test starts 300 ms after its ack so
the ack gets through first. USB re-enumerates after the reset.

`max_gap_ms` is the longest gap seen between check-ins since boot (or since
`wdg clear`). Use it to confirm the deadlines leave margin over real worst cases: SD
mount, adherence rebuild, a full therapy session with EDF logging.

## Not covered

- **CM4.** CM4 has no IWDG2 of its own. CM7 still detects a stopped CM4 through the IPC
  heartbeat (`CM4 Heartbeat Lost` therapy fault).
- **Therapy does not resume after a watchdog reset.** The device boots to idle.
  Auto-resume would be a separate feature.
