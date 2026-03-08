/**
 * @file sbus_mon.c
 * @brief SBUS protocol monitor – BeagleBone / Linux implementation
 *
 * Build:
 *   gcc -Wall -Wextra -o sbus_mon sbus_mon.c
 *
 * Run:
 *   sudo ./sbus_mon [/dev/ttyO1]
 *
 * The program reads SBUS frames from the configured UART, decodes all
 * 16 analogue channels plus the flag bits, and prints them on a single
 * updating line until Ctrl-C is pressed.
 */

#include "sbus_mon.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/* ── UART setup ─────────────────────────────────────────────────── */

int sbus_open(const char *port)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "sbus_open: cannot open %s: %s\n",
                port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "sbus_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /*
     * SBUS: 100000 baud, 8E2, non-canonical, no flow control.
     * Linux lacks a B100000 constant so we set the closest standard
     * rate (B38400) then override it with a custom divisor via
     * TIOCSSERIAL.
     */
    tty.c_cflag  = CS8 | PARENB | CSTOPB | CLOCAL | CREAD;
    tty.c_iflag  = 0;   /* no input processing                        */
    tty.c_oflag  = 0;   /* no output processing                       */
    tty.c_lflag  = 0;   /* raw mode – no signals, echo, or canonical  */
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "sbus_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    /* Override baud rate to 100000 using the custom divisor mechanism */
    struct serial_struct ser;
    memset(&ser, 0, sizeof(ser));

    if (ioctl(fd, TIOCGSERIAL, &ser) < 0) {
        fprintf(stderr, "sbus_open: TIOCGSERIAL failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    if (ser.baud_base == 0) {
        fprintf(stderr, "sbus_open: baud_base is 0, cannot set custom divisor\n");
        close(fd);
        return -1;
    }

    ser.flags        = (ser.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / 100000;

    if (ser.custom_divisor == 0) {
        fprintf(stderr, "sbus_open: baud_base %d too low for 100000 baud\n",
                ser.baud_base);
        close(fd);
        return -1;
    }

    if (ioctl(fd, TIOCSSERIAL, &ser) < 0) {
        fprintf(stderr, "sbus_open: TIOCSSERIAL failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

/* ── Frame decoding ─────────────────────────────────────────────── */

void sbus_decode(const uint8_t *b, sbus_frame_t *frame)
{
    /* 16 channels, each 11 bits, packed LSB-first across bytes 1-22 */
    frame->ch[0]  = ((b[1]       | (b[2]  << 8))  & 0x07FF);
    frame->ch[1]  = ((b[2]  >> 3 | (b[3]  << 5))  & 0x07FF);
    frame->ch[2]  = ((b[3]  >> 6 | (b[4]  << 2)  | (b[5]  << 10)) & 0x07FF);
    frame->ch[3]  = ((b[5]  >> 1 | (b[6]  << 7))  & 0x07FF);
    frame->ch[4]  = ((b[6]  >> 4 | (b[7]  << 4))  & 0x07FF);
    frame->ch[5]  = ((b[7]  >> 7 | (b[8]  << 1)  | (b[9]  <<  9)) & 0x07FF);
    frame->ch[6]  = ((b[9]  >> 2 | (b[10] << 6))  & 0x07FF);
    frame->ch[7]  = ((b[10] >> 5 | (b[11] << 3))  & 0x07FF);
    frame->ch[8]  = ((b[12]      | (b[13] << 8))  & 0x07FF);
    frame->ch[9]  = ((b[13] >> 3 | (b[14] << 5))  & 0x07FF);
    frame->ch[10] = ((b[14] >> 6 | (b[15] << 2)  | (b[16] << 10)) & 0x07FF);
    frame->ch[11] = ((b[16] >> 1 | (b[17] << 7))  & 0x07FF);
    frame->ch[12] = ((b[17] >> 4 | (b[18] << 4))  & 0x07FF);
    frame->ch[13] = ((b[18] >> 7 | (b[19] << 1)  | (b[20] <<  9)) & 0x07FF);
    frame->ch[14] = ((b[20] >> 2 | (b[21] << 6))  & 0x07FF);
    frame->ch[15] = ((b[21] >> 5 | (b[22] << 3))  & 0x07FF);

    /* Flags byte */
    frame->flags      = b[23];
    frame->ch17       = (b[23] & SBUS_FLAG_CH17)       ? 1 : 0;
    frame->ch18       = (b[23] & SBUS_FLAG_CH18)       ? 1 : 0;
    frame->frame_lost = (b[23] & SBUS_FLAG_FRAME_LOST) ? 1 : 0;
    frame->failsafe   = (b[23] & SBUS_FLAG_FAILSAFE)   ? 1 : 0;
}

/* ── Frame reader ───────────────────────────────────────────────── */

int sbus_read_frame(int fd, sbus_frame_t *frame)
{
    uint8_t  buf[SBUS_FRAME_SIZE];
    uint8_t  byte;

    /* Scan for start byte */
    while (1) {
        ssize_t n = read(fd, &byte, 1);
        if (n < 0) {
            if (errno == EINTR)
                continue;           /* interrupted – retry */
            fprintf(stderr, "sbus_read_frame: read error: %s\n",
                    strerror(errno));
            return -1;
        }
        if (n == 0)
            continue;               /* no data yet          */
        if (byte == SBUS_START_BYTE)
            break;
    }

    buf[0] = SBUS_START_BYTE;

    /* Read remaining 24 bytes */
    int got = 1;
    while (got < SBUS_FRAME_SIZE) {
        ssize_t n = read(fd, buf + got, (size_t)(SBUS_FRAME_SIZE - got));
        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "sbus_read_frame: read error at byte %d: %s\n",
                    got, strerror(errno));
            return -1;
        }
        got += (int)n;
    }

    /* Validate end byte */
    if (buf[24] != SBUS_END_BYTE_A && buf[24] != SBUS_END_BYTE_B) {
        /* Discard and let the caller loop for the next frame */
        return 1;   /* non-fatal – bad frame */
    }

    sbus_decode(buf, frame);
    return 0;
}

/* ── Display ────────────────────────────────────────────────────── */

void sbus_print_frame(const sbus_frame_t *frame)
{
    printf("\r");
    for (int i = 0; i < SBUS_NUM_CH; i++)
        printf("CH%-2d:%4d  ", i + 1, frame->ch[i]);

    printf("| FL:%d FS:%d D17:%d D18:%d   ",
           frame->frame_lost, frame->failsafe,
           frame->ch17, frame->ch18);

    fflush(stdout);
}

/* ── Entry point ────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    const char *port = (argc > 1) ? argv[1] : SBUS_PORT;

    printf("SBUS Monitor  |  port: %s  |  Ctrl-C to exit\n\n", port);

    int fd = sbus_open(port);
    if (fd < 0)
        return EXIT_FAILURE;

    sbus_frame_t frame;
    int bad_frames = 0;

    while (1) {
        int rc = sbus_read_frame(fd, &frame);

        if (rc < 0) {
            /* Unrecoverable I/O error */
            fprintf(stderr, "\nFatal read error – exiting.\n");
            close(fd);
            return EXIT_FAILURE;
        }

        if (rc == 1) {
            /* Bad end byte – count and continue */
            if (++bad_frames % 100 == 0)
                fprintf(stderr, "\n[warn] %d bad frames discarded\n",
                        bad_frames);
            continue;
        }

        /* Warn about link quality without aborting */
        if (frame.frame_lost)
            fprintf(stderr, "\n[warn] frame lost – signal gap detected\n");
        if (frame.failsafe)
            fprintf(stderr, "\n[warn] failsafe active\n");

        sbus_print_frame(&frame);
    }

    /* Unreachable in normal operation; handles compiler warnings */
    close(fd);
    return EXIT_SUCCESS;
}
