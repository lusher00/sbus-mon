/**
 * @file sbus_mon.c
 * @brief SBUS protocol monitor – BeagleBone / Linux implementation
 *
 * Build:
 *   gcc -Wall -Wextra -o sbus_mon sbus_mon.c -lncurses
 *
 * Run:
 *   sudo ./sbus_mon [/dev/ttyO1]
 */

#include "sbus_mon.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <ncurses.h>

/* ── UART setup ─────────────────────────────────────────────────── */

int sbus_open(const char *port)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "sbus_open: cannot open %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "sbus_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd); return -1;
    }

    tty.c_cflag  = CS8 | PARENB | CSTOPB | CLOCAL | CREAD;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;
    cfsetispeed(&tty, B38400);
    cfsetospeed(&tty, B38400);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "sbus_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd); return -1;
    }

    struct serial_struct ser;
    memset(&ser, 0, sizeof(ser));
    if (ioctl(fd, TIOCGSERIAL, &ser) < 0) {
        fprintf(stderr, "sbus_open: TIOCGSERIAL failed: %s\n", strerror(errno));
        close(fd); return -1;
    }
    if (ser.baud_base == 0) {
        fprintf(stderr, "sbus_open: baud_base is 0\n");
        close(fd); return -1;
    }
    ser.flags = (ser.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / 100000;
    if (ser.custom_divisor == 0) {
        fprintf(stderr, "sbus_open: baud_base %d too low\n", ser.baud_base);
        close(fd); return -1;
    }
    if (ioctl(fd, TIOCSSERIAL, &ser) < 0) {
        fprintf(stderr, "sbus_open: TIOCSSERIAL failed: %s\n", strerror(errno));
        close(fd); return -1;
    }

    return fd;
}

/* ── Frame decoding ─────────────────────────────────────────────── */

void sbus_decode(const uint8_t *b, sbus_frame_t *frame)
{
    frame->ch[0]  = ((b[1]       | (b[2]  << 8))                    & 0x07FF);
    frame->ch[1]  = ((b[2]  >> 3 | (b[3]  << 5))                    & 0x07FF);
    frame->ch[2]  = ((b[3]  >> 6 | (b[4]  << 2)  | (b[5]  << 10))  & 0x07FF);
    frame->ch[3]  = ((b[5]  >> 1 | (b[6]  << 7))                    & 0x07FF);
    frame->ch[4]  = ((b[6]  >> 4 | (b[7]  << 4))                    & 0x07FF);
    frame->ch[5]  = ((b[7]  >> 7 | (b[8]  << 1)  | (b[9]  <<  9))  & 0x07FF);
    frame->ch[6]  = ((b[9]  >> 2 | (b[10] << 6))                    & 0x07FF);
    frame->ch[7]  = ((b[10] >> 5 | (b[11] << 3))                    & 0x07FF);
    frame->ch[8]  = ((b[12]      | (b[13] << 8))                    & 0x07FF);
    frame->ch[9]  = ((b[13] >> 3 | (b[14] << 5))                    & 0x07FF);
    frame->ch[10] = ((b[14] >> 6 | (b[15] << 2)  | (b[16] << 10))  & 0x07FF);
    frame->ch[11] = ((b[16] >> 1 | (b[17] << 7))                    & 0x07FF);
    frame->ch[12] = ((b[17] >> 4 | (b[18] << 4))                    & 0x07FF);
    frame->ch[13] = ((b[18] >> 7 | (b[19] << 1)  | (b[20] <<  9))  & 0x07FF);
    frame->ch[14] = ((b[20] >> 2 | (b[21] << 6))                    & 0x07FF);
    frame->ch[15] = ((b[21] >> 5 | (b[22] << 3))                    & 0x07FF);

    frame->flags      = b[23];
    frame->ch17       = (b[23] & SBUS_FLAG_CH17)       ? 1 : 0;
    frame->ch18       = (b[23] & SBUS_FLAG_CH18)       ? 1 : 0;
    frame->frame_lost = (b[23] & SBUS_FLAG_FRAME_LOST) ? 1 : 0;
    frame->failsafe   = (b[23] & SBUS_FLAG_FAILSAFE)   ? 1 : 0;
}

/* ── Frame reader ───────────────────────────────────────────────── */

int sbus_read_frame(int fd, sbus_frame_t *frame)
{
    uint8_t buf[SBUS_FRAME_SIZE];
    uint8_t byte;

    while (1) {
        ssize_t n = read(fd, &byte, 1);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        if (n == 0) continue;
        if (byte == SBUS_START_BYTE) break;
    }

    buf[0] = SBUS_START_BYTE;
    int got = 1;
    while (got < SBUS_FRAME_SIZE) {
        ssize_t n = read(fd, buf + got, (size_t)(SBUS_FRAME_SIZE - got));
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        got += (int)n;
    }

    if (buf[24] != SBUS_END_BYTE_A && buf[24] != SBUS_END_BYTE_B)
        return 1;

    sbus_decode(buf, frame);
    return 0;
}

/* ── ncurses display ────────────────────────────────────────────── */

/*
 * Channel table — label and display type.
 *   'A' = bipolar analogue bar  (-1.0 … 0 … +1.0)
 *   'S' = 3-pos switch          [LOW] [MID] [HIGH]
 *   'T' = 2-pos switch          [OFF] [ON]
 */
static const struct { const char *label; char type; } CH_INFO[SBUS_NUM_CH] = {
    { "CH1  Ail   Turn",    'A' },
    { "CH2  Ele   Drive",   'A' },
    { "CH3  Thr   (unused)",'A' },
    { "CH4  Rud   (unused)",'A' },
    { "CH5  SA    Arm",     'S' },
    { "CH6  SB    Kill",    'S' },
    { "CH7  S1    Aux1",    'A' },
    { "CH8  S2    Aux2",    'A' },
    { "CH9  SC    AuxC",    'S' },
    { "CH10 SD    Speed",   'S' },
    { "CH11 SE    AuxE",    'S' },
    { "CH12 SF    AuxF",    'T' },
    { "CH13 (stub)",        'A' },
    { "CH14 (stub)",        'A' },
    { "CH15 (stub)",        'A' },
    { "CH16 (stub)",        'A' },
};

#define BAR_W   40      /* chars across the full bar (centre at BAR_W/2) */
#define C_LABEL  2      /* column: channel label starts here */
#define C_BAR   24      /* column: bar starts here */
#define C_RAW   (C_BAR + BAR_W + 2)   /* column: raw value */

/* colour pairs */
enum { CP_TITLE=1, CP_LABEL, CP_POS, CP_NEG, CP_MID,
       CP_SW_ON, CP_SW_OFF, CP_WARN, CP_OK };

static void init_colours(void)
{
    start_color();
    use_default_colors();
    init_pair(CP_TITLE,  COLOR_CYAN,  -1);
    init_pair(CP_LABEL,  COLOR_WHITE, -1);
    init_pair(CP_POS,    COLOR_GREEN, -1);
    init_pair(CP_NEG,    COLOR_BLUE,  -1);
    init_pair(CP_MID,    COLOR_WHITE, -1);
    init_pair(CP_SW_ON,  COLOR_BLACK, COLOR_GREEN);
    init_pair(CP_SW_OFF, COLOR_BLACK, COLOR_WHITE);
    init_pair(CP_WARN,   COLOR_RED,   -1);
    init_pair(CP_OK,     COLOR_GREEN, -1);
}

/* Bipolar bar: negative fills left of centre, positive fills right */
static void draw_bar(int row, int raw)
{
    int half = BAR_W / 2;
    float norm = (float)(raw - SBUS_CH_MID) /
                 (float)(SBUS_CH_MAX - SBUS_CH_MID);
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    int filled = (int)(norm * half);

    for (int i = 0; i < BAR_W; i++) {
        int  col_pos = i - half;   /* -half … +half-1 */
        char ch;
        int  cp;

        if (i == half) {
            ch = '|'; cp = CP_MID;
        } else if (filled > 0 && col_pos > 0 && col_pos <= filled) {
            ch = '#'; cp = CP_POS;
        } else if (filled < 0 && col_pos < 0 && col_pos >= filled) {
            ch = '#'; cp = CP_NEG;
        } else {
            ch = '-'; cp = CP_MID;
        }

        attron(COLOR_PAIR(cp));
        mvaddch(row, C_BAR + i, ch);
        attroff(COLOR_PAIR(cp));
    }
    mvprintw(row, C_RAW, "%4d", raw);
}

/* 3-pos switch: highlight the active position */
static void draw_3pos(int row, int raw)
{
    int pos = (raw < 400) ? 0 : (raw > 1600) ? 2 : 1;
    const char *lbl[3] = { "LOW ", "MID ", "HIGH" };
    int x = C_BAR;
    for (int i = 0; i < 3; i++) {
        attron(COLOR_PAIR(i == pos ? CP_SW_ON : CP_SW_OFF));
        mvprintw(row, x, "[%s]", lbl[i]);
        attroff(COLOR_PAIR(CP_SW_ON));
        attroff(COLOR_PAIR(CP_SW_OFF));
        x += 7;
    }
    mvprintw(row, C_RAW, "%4d", raw);
}

/* 2-pos switch */
static void draw_2pos(int row, int raw)
{
    int on = (raw > 1600);
    attron(COLOR_PAIR(on ? CP_SW_OFF : CP_SW_ON));
    mvprintw(row, C_BAR,     "[OFF]");
    attroff(COLOR_PAIR(CP_SW_ON)); attroff(COLOR_PAIR(CP_SW_OFF));
    attron(COLOR_PAIR(on ? CP_SW_ON : CP_SW_OFF));
    mvprintw(row, C_BAR + 6, "[ ON]");
    attroff(COLOR_PAIR(CP_SW_ON)); attroff(COLOR_PAIR(CP_SW_OFF));
    mvprintw(row, C_RAW, "%4d", raw);
}

static void draw_frame(const sbus_frame_t *frame, int bad, const char *port)
{
    /* title */
    attron(COLOR_PAIR(CP_TITLE) | A_BOLD);
    mvprintw(0, C_LABEL, "SBUS Monitor  port:%-14s  q=quit", port);
    attroff(COLOR_PAIR(CP_TITLE) | A_BOLD);

    /* header */
    attron(A_DIM);
    mvprintw(1, C_LABEL, "%-21s", "Channel");
    mvprintw(1, C_BAR,   "<---------  -1.0  ----------|----------  +1.0  --------->");
    mvprintw(1, C_RAW,   " raw");
    attroff(A_DIM);

    /* channel rows */
    for (int i = 0; i < SBUS_NUM_CH; i++) {
        int row = i + 2;
        attron(COLOR_PAIR(CP_LABEL));
        mvprintw(row, C_LABEL, "%-21s", CH_INFO[i].label);
        attroff(COLOR_PAIR(CP_LABEL));

        switch (CH_INFO[i].type) {
            case 'A': draw_bar(row,  frame->ch[i]); break;
            case 'S': draw_3pos(row, frame->ch[i]); break;
            case 'T': draw_2pos(row, frame->ch[i]); break;
        }
    }

    /* flags */
    int frow = SBUS_NUM_CH + 3;
    move(frow, C_LABEL);
    printw("Flags:  ");

    attron(COLOR_PAIR(frame->failsafe   ? CP_WARN : CP_OK));
    printw("FAILSAFE:%-3s  ", frame->failsafe   ? "YES" : "no");
    attroff(COLOR_PAIR(CP_WARN)); attroff(COLOR_PAIR(CP_OK));

    attron(COLOR_PAIR(frame->frame_lost ? CP_WARN : CP_OK));
    printw("FRAME_LOST:%-3s  ", frame->frame_lost ? "YES" : "no");
    attroff(COLOR_PAIR(CP_WARN)); attroff(COLOR_PAIR(CP_OK));

    printw("D17:%d  D18:%d  bad:%d", frame->ch17, frame->ch18, bad);
    clrtoeol();
    refresh();
}

/* ── signal handling ────────────────────────────────────────────── */

static volatile int g_running = 1;
static void on_sigint(int s) { (void)s; g_running = 0; }

/* ── main ───────────────────────────────────────────────────────── */

int main(int argc, char *argv[])
{
    const char *port = (argc > 1) ? argv[1] : SBUS_PORT;

    int fd = sbus_open(port);
    if (fd < 0)
        return EXIT_FAILURE;

    signal(SIGINT, on_sigint);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    nodelay(stdscr, TRUE);   /* non-blocking so we don't stall on getch */
    if (has_colors())
        init_colours();

    sbus_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    int bad_frames = 0;

    while (g_running) {
        int key = getch();
        if (key == 'q' || key == 'Q') break;

        int rc = sbus_read_frame(fd, &frame);
        if (rc < 0) {
            endwin();
            fprintf(stderr, "Fatal read error – exiting.\n");
            close(fd);
            return EXIT_FAILURE;
        }
        if (rc == 1) { bad_frames++; continue; }

        draw_frame(&frame, bad_frames, port);
    }

    endwin();
    close(fd);
    printf("\nsbus_mon exited cleanly.\n");
    return EXIT_SUCCESS;
}
