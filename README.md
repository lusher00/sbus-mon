# sbus_mon

A minimal SBUS protocol monitor for BeagleBone (and any Linux system with a
hardware UART). Decodes all 16 analogue channels plus the digital/flag bits
from a live RC receiver and prints them on a continuously-updating single line.

---

## Hardware requirements

| Item | Detail |
|---|---|
| UART | `/dev/ttyO1` (default) – any hardware UART works |
| Signal level | SBUS is **inverted** 3.3 V logic. Use a signal inverter (single transistor or dedicated IC) between your receiver and the UART RX pin |
| Baud rate | 100 000 baud, 8E2 (set automatically by the driver) |

> **BeagleBone pin note:** `ttyO1` maps to UART1 on the P9 header
> (P9.24 TX / P9.26 RX). Enable it in your device-tree overlay if it is
> not already active.

---

## Build

```bash
gcc -Wall -Wextra -o sbus_mon sbus_mon.c
```

No external libraries required.

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

Example output (single line, updated in place):

```
CH1: 992  CH2: 992  CH3: 172  CH4: 992  ... CH16: 992  | FL:0 FS:0 D17:0 D18:0
```

| Column | Meaning |
|---|---|
| `CH1`–`CH16` | Analogue channel value (172 min · 992 centre · 1811 max) |
| `FL` | Frame lost – signal gap detected |
| `FS` | Failsafe active |
| `D17` / `D18` | Digital channels 17 and 18 |

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
sbus_mon.c   – UART init, frame reader, decoder, display, main()
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
| Bad end byte | Frame discarded; warning printed every 100 bad frames |
| Frame-lost flag set | Warning printed to `stderr`, decoding continues |
| Failsafe flag set | Warning printed to `stderr`, decoding continues |
| Unrecoverable `read` error | Prints error, closes fd, exits non-zero |

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
        // frame.ch[0] … frame.ch[15]  – channel values
        // frame.failsafe              – act on lost link
    }
}
```

---

## Licence

MIT – do whatever you like, no warranty expressed or implied.
