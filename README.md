# sbus_mon

A minimal SBUS protocol monitor for BeagleBone (and any Linux system with a
hardware UART). Decodes all 16 analogue channels plus the digital/flag bits
from a live RC receiver and displays them in a live-updating ncurses terminal.

---

## Hardware requirements

| Item | Detail |
|---|---|
| UART | `/dev/ttyO1` (default) – any hardware UART works |
| Signal level | SBUS is **inverted** 3.3 V logic. Use a signal inverter (single NPN transistor + 10kΩ pull-up, or dedicated IC) between your receiver and the UART RX pin |
| Baud rate | 100 000 baud, 8E2 – set automatically by the driver |

> **BeagleBone pin note:** `ttyO1` maps to UART1 on the P9 header
> (P9.24 TX / P9.26 RX). Enable it in your device-tree overlay if it is
> not already active.

---

## Build

```bash
gcc -Wall -Wextra -o sbus_mon sbus_mon.c -lncurses
```

Requires libncurses:

```bash
sudo apt install libncurses5-dev
```

---

## Usage

```bash
# Default port (/dev/ttyO1)
sudo ./sbus_mon

# Specify a different port
sudo ./sbus_mon /dev/ttyS1
```

`sudo` is required unless your user is in the `dialout` group:

```bash
sudo usermod -aG dialout $USER   # log out and back in to take effect
```

Press **q** to quit.

---

## Display

The ncurses display updates in place — one row per channel, no scrolling.

```
SBUS Monitor  port:/dev/ttyO1          q=quit

Channel               <---------  -1.0  ----------|----------  +1.0  --------->  raw
CH1  Ail   Turn       --------------------|-########------------------            1124
CH2  Ele   Drive      --------------------|-----------#############--             1543
CH5  SA    Arm        [LOW ] [MID ] [HIGH]                                         992
CH6  SB    Kill       [LOW ] [MID ] [HIGH]                                        1811
CH12 SF    AuxF       [OFF] [ ON]                                                  172
...
Flags:  FAILSAFE:no   FRAME_LOST:no   D17:0  D18:0  bad:0
```

| Element | Meaning |
|---|---|
| Bipolar bar (`A`) | Analogue channels — negative fills left of centre, positive fills right |
| 3-pos switch (`S`) | Active position highlighted: `[LOW]` `[MID]` `[HIGH]` |
| 2-pos switch (`T`) | Active position highlighted: `[OFF]` `[ON]` |
| `raw` | Raw 11-bit channel value (172 min · 992 centre · 1811 max) |
| `FAILSAFE` | Receiver entered failsafe — highlighted red when active |
| `FRAME_LOST` | Signal gap detected — highlighted red when active |
| `D17` / `D18` | Digital channels 17 and 18 |
| `bad` | Count of frames discarded due to bad end byte |

---

## Channel map (Jumper T16 / OpenTX AETR)

| CH | Switch | Function |
|----|--------|----------|
| CH1 Ail | Right stick X | Turn / yaw |
| CH2 Ele | Right stick Y | Drive / pitch |
| CH3 Thr | Left stick Y | Throttle (unused) |
| CH4 Rud | Left stick X | (unused) |
| CH5 SA | 3-pos | Arm / Disarm |
| CH6 SB | 3-pos | Kill switch |
| CH7 S1 | Pot/slider | Aux analogue 1 |
| CH8 S2 | Pot/slider | Aux analogue 2 |
| CH9 SC | 3-pos | Aux C |
| CH10 SD | 3-pos | Speed mode |
| CH11 SE | 3-pos | Aux E |
| CH12 SF | 2-pos | Aux F |

---

## SBUS frame format

```
Byte  0     : 0x0F  (start)
Bytes 1–22  : 16 × 11-bit channels, packed LSB-first
Byte  23    : Flags  [b0=CH17  b1=CH18  b2=FrameLost  b3=Failsafe]
Byte  24    : 0x00 or 0x04  (end)
```

Frames arrive every ~14 ms at 100 000 baud.

---

## File structure

```
sbus_mon.h   – constants, types, and function prototypes
sbus_mon.c   – UART init, frame reader, decoder, ncurses display, main()
README.md    – this file
```

---

## Error handling

| Condition | Behaviour |
|---|---|
| Cannot open port | Prints error with `errno` string, exits non-zero |
| `tcgetattr` / `tcsetattr` fail | Prints error, exits non-zero |
| `ioctl` TIOCSSERIAL fails | Prints error, exits non-zero |
| `read()` returns `EINTR` | Retried automatically |
| Bad end byte | Frame discarded, `bad` counter incremented in display |
| Frame-lost flag set | `FRAME_LOST` highlighted red in display |
| Failsafe flag set | `FAILSAFE` highlighted red in display |
| Unrecoverable `read` error | Restores terminal via `endwin()`, exits non-zero |
| `q` / `Q` keypress | Clean exit, terminal restored |

---

## Integrating into a larger project

The public API in `sbus_mon.h` is designed to be dropped into any C project:

```c
#include "sbus_mon.h"

int fd = sbus_open("/dev/ttyO1");
if (fd < 0) { /* handle error */ }

sbus_frame_t frame;
while (1) {
    if (sbus_read_frame(fd, &frame) == 0) {
        // frame.ch[0] … frame.ch[15]  – channel values (172–1811)
        // frame.failsafe              – act on lost link
        // frame.frame_lost            – signal gap
    }
}
```

Note: `sbus_print_frame()` declared in the header is the legacy single-line
printer. The ncurses display (`draw_frame()`) is internal to `sbus_mon.c` and
not exposed in the public API.

---

## Licence

MIT – do whatever you like, no warranty expressed or implied.