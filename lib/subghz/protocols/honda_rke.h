#pragma once
/**
 * honda_rke.h
 * Honda RKE protocol — Pandora DXL 5000 → Flipper Zero port
 * Protocol ID: 0x17 | 433.92 MHz / 315.00 MHz | OOK PWM | 32-bit frame
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Timing constants (microseconds)
 * ------------------------------------------------------------------------- */
#define HONDA_PULSE_HIGH_US    300u
#define HONDA_BIT1_LOW_US      900u
#define HONDA_BIT0_LOW_US      300u
#define HONDA_SYNC_HIGH_US     300u
#define HONDA_SYNC_LOW_US     4500u
#define HONDA_REPEAT_GAP_US  10000u
#define HONDA_TOLERANCE_PCT     15u

/* Frequency options */
#define HONDA_FREQ_EU_HZ    433920000ul   /* Europe / Japan */
#define HONDA_FREQ_US_HZ    315000000ul   /* North America  */

/* Button codes — nibble [7:4] of payload */
#define HONDA_BTN_LOCK       0x1u
#define HONDA_BTN_UNLOCK     0x2u
#define HONDA_BTN_TRUNK      0x4u
#define HONDA_BTN_PANIC      0x8u

/* -------------------------------------------------------------------------
 * Data types
 * ------------------------------------------------------------------------- */

/** Decoded / ready-to-encode Honda RKE frame */
typedef struct {
    uint16_t fixed_id;   /**< 16-bit fixed fob ID                          */
    uint8_t  counter;    /**< 8-bit rolling counter (wraps at 256)         */
    uint8_t  button;     /**< Button code: HONDA_BTN_*                     */
    uint8_t  checksum;   /**< 4-bit XOR checksum (filled by encode, checked by decode) */
    bool     valid;      /**< true after decode if checksum matched        */
} HondaFrame;

/** Raw pulse buffer (positive = HIGH µs, negative = LOW µs) */
typedef struct {
    int32_t  pulses[256];
    uint32_t count;
} HondaRawBuf;

/* -------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/**
 * Encode a HondaFrame into a SubGHz RAW pulse buffer.
 * Emits 3 repetitions. Fills frame->checksum automatically.
 *
 * @param frame  Input frame (fixed_id, counter, button must be set).
 * @param buf    Output pulse buffer.
 */
void honda_encode(const HondaFrame *frame, HondaRawBuf *buf);

/**
 * Decode a raw pulse buffer into a HondaFrame.
 *
 * @param buf    Input pulse buffer from SubGHz capture.
 * @param frame  Output decoded frame.
 * @return       true if a valid (checksum-passing) frame was found.
 */
bool honda_decode(const HondaRawBuf *buf, HondaFrame *frame);

/**
 * Validate a received rolling counter against the last stored value.
 * Honda receivers accept counters in window [stored+1, stored+16].
 *
 * @param stored    Last accepted counter value.
 * @param received  Counter from received frame.
 * @return          true if within acceptance window.
 */
bool honda_counter_valid(uint8_t stored, uint8_t received);

#ifdef __cplusplus
}
#endif
