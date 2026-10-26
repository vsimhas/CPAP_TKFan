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
| PLD | 1 s @ 1 Hz | `MaskPress.2s`, `Press.2s` (target), `Leak.2s` (L/s) |
| EVE | EDF+D, 64-byte records | `EDF Annotations` + `Crc16` |

**EVE** (checklist #59) follows the AirSense 11's own file byte for byte: the
first record is `Recording starts`, then one record per scored event,
`Obstructive Apnea`, `Central Apnea` or `Hypopnea`. As with ResMed, an event is
stamped at its **end** (seconds from the file start) and carries its duration:

- an apnea is written when its episode ends, with the episode's length;
- a central apnea that may still be re-scored as a hypopnea (R4, 30 s window) is
  held until that window closes, and dropped if re-scored;
- a hypopnea is written when breathing recovers.

Flow limitation and snore have no EVE entry in OSCAR's loader; they belong in PLD as
continuous indices (#60/#61). `STR.edf` is still a header-only stub (#57).

Inspect a file without a viewer:

```
python pc_tools/edf_inspect.py <file>.edf
```

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
