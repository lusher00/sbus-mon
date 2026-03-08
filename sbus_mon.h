/**
 * @file sbus_mon.h
 * @brief SBUS protocol monitor for BeagleBone / Linux UART
 *
 * Decodes Futaba SBUS frames received over a UART configured at 100000 baud,
 * 8E2 (8 data bits, even parity, 2 stop bits), inverted signal.
 *
 * SBUS frame layout (25 bytes):
 *   Byte  0    : Start byte (0x0F)
 *   Bytes 1-22 : 16 channels × 11 bits, packed LSB-first
 *   Byte  23   : Flags (digital ch17/18, frame lost, failsafe)
 *   Byte  24   : End byte (0x00 or 0x04)
 *
 * Channel values range from 172 (min) to 1811 (max), centre ~992.
 */

#ifndef SBUS_MON_H
#define SBUS_MON_H

#include <stdint.h>

/* ── Constants ──────────────────────────────────────────────────── */

#define SBUS_PORT        "/dev/ttyO1"   /**< Default UART device          */
#define SBUS_FRAME_SIZE  25             /**< Bytes per SBUS frame          */
#define SBUS_NUM_CH      16             /**< Analogue channel count        */
#define SBUS_START_BYTE  0x0F           /**< Frame start marker            */
#define SBUS_END_BYTE_A  0x00           /**< Valid end byte (normal)       */
#define SBUS_END_BYTE_B  0x04           /**< Valid end byte (some RX)      */

#define SBUS_CH_MIN      172            /**< Minimum channel value         */
#define SBUS_CH_MID      992            /**< Centre / neutral value        */
#define SBUS_CH_MAX      1811           /**< Maximum channel value         */

/** Bit masks for the flags byte (buf[23]) */
#define SBUS_FLAG_CH17       (1 << 0)   /**< Digital channel 17            */
#define SBUS_FLAG_CH18       (1 << 1)   /**< Digital channel 18            */
#define SBUS_FLAG_FRAME_LOST (1 << 2)   /**< Signal lost (>1 frame gap)    */
#define SBUS_FLAG_FAILSAFE   (1 << 3)   /**< Receiver entered failsafe     */

/* ── Types ──────────────────────────────────────────────────────── */

/** Decoded SBUS frame */
typedef struct {
    int      ch[SBUS_NUM_CH];   /**< 16 analogue channels (172-1811)   */
    uint8_t  flags;             /**< Raw flags byte                    */
    int      ch17;              /**< Digital channel 17 (0 or 1)       */
    int      ch18;              /**< Digital channel 18 (0 or 1)       */
    int      frame_lost;        /**< 1 = signal lost                   */
    int      failsafe;          /**< 1 = failsafe active               */
} sbus_frame_t;

/* ── Function prototypes ────────────────────────────────────────── */

/**
 * @brief  Open and configure the UART for SBUS reception.
 * @param  port  Device path (e.g. "/dev/ttyO1").
 * @return File descriptor on success, -1 on error (errno set).
 */
int sbus_open(const char *port);

/**
 * @brief  Block until a valid SBUS frame is received.
 *
 * Scans the byte stream for a start byte, reads the remaining 24 bytes,
 * and validates the end byte.  Silently discards malformed frames.
 *
 * @param  fd     File descriptor returned by sbus_open().
 * @param  frame  Pointer to caller-allocated sbus_frame_t to fill.
 * @return 0 on success, -1 on unrecoverable read error.
 */
int sbus_read_frame(int fd, sbus_frame_t *frame);

/**
 * @brief  Decode a raw 25-byte SBUS buffer into an sbus_frame_t.
 * @param  buf    Raw frame buffer (must be SBUS_FRAME_SIZE bytes).
 * @param  frame  Output structure.
 */
void sbus_decode(const uint8_t *buf, sbus_frame_t *frame);

/**
 * @brief  Print all 16 channels and flag bits on a single overwriting line.
 * @param  frame  Pointer to decoded frame.
 */
void sbus_print_frame(const sbus_frame_t *frame);

#endif /* SBUS_MON_H */
