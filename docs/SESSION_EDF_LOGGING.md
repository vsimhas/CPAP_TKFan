# Therapy session EDF logging (SD card)

## What this is

When therapy starts, CM7 writes a classic **EDF** (European Data Format) file to the
root of a FAT-formatted SD card:

```
0:/YYmmddNN.EDF
```

`NN` is chosen by probing for the first unused name, so **each therapy start creates a
new file** and never overwrites a previous session. Subdirectories are avoided because
`f_mkdir` writes were timing out on this SDMMC bus.

Signals (1 s records, 25 Hz):

| Label    | Meaning              | Unit  |
|----------|----------------------|-------|
| Flow     | Patient flow         | L/min |
| Pressure | Mask pressure        | cmH2O |
| Leak     | Unintentional leak   | L/min |
| TargetP  | Therapy target       | cmH2O |

View with **EDFbrowser**, MATLAB, Python (`pyedflib` / MNE), etc.

To check a file without a viewer (header fields, size consistency and decoded
physical values):

```
python pc_tools/edf_inspect.py 26082100.EDF
python pc_tools/edf_inspect.py 26082100.EDF --dump TargetP --seconds 5
```

In EDFbrowser, load all four traces with **Select All** then **Add signal(s)**.
The `Add ->` arrow builds a *derivation*, which requires every signal to share one
physical dimension — these do not (`L/min` and `cmH2O`), so it reports
"Cannot make derivation".

## OSCAR — important limitation

**OSCAR will not import these files as-is.**

OSCAR expects a **vendor SD-card layout** (e.g. ResMed `DATALOG/` + `SETTINGS/` +
`Identification.*` + proprietary companion files). Cloning that layout would mean
reproducing another manufacturer’s proprietary card format, which this project
does not do.

Practical paths:

1. Use **EDFbrowser** (or similar) for waveform review today.
2. Later: a **SomnaLink-specific OSCAR machine loader** (open-source plugin) that
   reads these `.EDF` files — that is the correct long-term OSCAR integration.

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
