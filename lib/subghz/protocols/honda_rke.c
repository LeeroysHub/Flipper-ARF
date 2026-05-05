/**
 * honda_rke.c
 * Honda RKE (Remote Keyless Entry) protocol — ported from Pandora DXL 5000 firmware
 * Target: Flipper Zero (SubGHz RAW / custom protocol plugin)
 *
 * Protocol ID in original firmware: 0x17
 * Frequency: 433.92 MHz (JP/EU) or 315.00 MHz (US/JDM)
 * Modulation: OOK (On-Off Keying), PWM encoded
 * Carrier: AM
 *
 * Frame structure (Honda G-key / Honda 3-button fob, ~2003-2019):
 *   Preamble : 12 pulses (carrier burst)
 *   Sync gap : ~4.5 ms LOW
 *   Payload  : 32 bits, LSB-first, PWM
 *     [31:24] Fixed ID high byte
 *     [23:16] Fixed ID low byte
 *     [15:8]  Rolling counter (increments each press)
 *     [7:4]   Button code  (0x1=Lock, 0x2=Unlock, 0x4=Trunk, 0x8=Panic)
 *     [3:0]   Checksum nibble (XOR of nibbles [31:4])
 *   Repeated 3 times with ~10 ms gap between repetitions
 *
 * PWM timing (from firmware timer-capture analysis, FUN_000007cc):
 *   Bit-1 : 300 µs HIGH + 900 µs LOW  (long-low  = 1200 µs period)
 *   Bit-0 : 300 µs HIGH + 300 µs LOW  (short-low =  600 µs period)
 *   Tolerance: ±15%
 *
 * Flipper Zero integration:
 *   Use the SubGHz RAW capture or implement as a SubGHz custom protocol.
 *   The encode/decode functions below are self-contained and use no HAL;
 *   wire them into your Flipper plugin's encode/decode callbacks.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Timing constants (microseconds)
 * Derived from FUN_000007cc timing calculation:
 *   period = (count-2)*2*timer_freq / sum_of_differences
 * ------------------------------------------------------------------------- */
#define HONDA_PULSE_HIGH_US     300u   /* carrier burst width (both bits)   */
#define HONDA_BIT1_LOW_US       900u   /* long  gap = logic 1               */
#define HONDA_BIT0_LOW_US       300u   /* short gap = logic 0               */
#define HONDA_SYNC_HIGH_US      300u   /* sync burst                        */
#define HONDA_SYNC_LOW_US      4500u   /* sync gap                          */
#define HONDA_REPEAT_GAP_US   10000u   /* inter-frame gap                   */
#define HONDA_TOLERANCE_PCT      15u   /* ±15% timing tolerance             */

/* Button codes (nibble [7:4] of payload byte [15:8]) */
#define HONDA_BTN_LOCK          0x1u
#define HONDA_BTN_UNLOCK        0x2u
#define HONDA_BTN_TRUNK         0x4u
#define HONDA_BTN_PANIC         0x8u

/* -------------------------------------------------------------------------
 * Data types
 * ------------------------------------------------------------------------- */

/** Decoded Honda RKE frame */
typedef struct {
    uint16_t fixed_id;      /**< 16-bit fixed ID burned into fob            */
    uint8_t  counter;       /**< 8-bit rolling counter                      */
    uint8_t  button;        /**< button nibble (HONDA_BTN_*)                */
    uint8_t  checksum;      /**< 4-bit XOR checksum (verified on decode)    */
    bool     valid;         /**< true if checksum passed                    */
} HondaFrame;

/** Raw pulse buffer for Flipper SubGHz RAW */
typedef struct {
    int32_t  pulses[256];   /**< positive=HIGH µs, negative=LOW µs          */
    uint32_t count;         /**< number of entries used                     */
} HondaRawBuf;

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/** Compute 4-bit XOR checksum over bits [31:4] (8 nibbles) */
static uint8_t honda_checksum(uint32_t word)
{
    uint8_t chk = 0;
    /* bits [31:4] → nibbles n7..n1, skip n0 (that IS the checksum field) */
    for (int i = 7; i >= 1; i--) {
        chk ^= (uint8_t)((word >> (i * 4)) & 0xFu);
    }
    return chk & 0xFu;
}

/** Build the 32-bit on-air word from a HondaFrame */
static uint32_t honda_build_word(const HondaFrame *f)
{
    uint32_t word = 0;
    word |= (uint32_t)(f->fixed_id >> 8)   << 24;
    word |= (uint32_t)(f->fixed_id & 0xFF) << 16;
    word |= (uint32_t)(f->counter)         <<  8;
    word |= (uint32_t)(f->button & 0xFu)   <<  4;
    word |= honda_checksum(word);           /* fill in checksum nibble      */
    return word;
}

/** Append one pulse pair to raw buffer (HIGH then LOW) */
static void honda_push_pulse(HondaRawBuf *buf, uint32_t high_us, uint32_t low_us)
{
    if (buf->count + 2 > 256) return;
    buf->pulses[buf->count++] =  (int32_t)high_us;
    buf->pulses[buf->count++] = -(int32_t)low_us;
}

/* -------------------------------------------------------------------------
 * Encode
 * ------------------------------------------------------------------------- */

/**
 * honda_encode() — encode a HondaFrame into a Flipper SubGHz RAW buffer.
 *
 * Emits 3 repetitions of the frame as required by Honda receivers.
 * The caller should transmit buf->pulses[0..count-1] via the SubGHz RAW API.
 *
 * @param frame   Pointer to populated HondaFrame (fixed_id, counter, button).
 * @param buf     Output raw pulse buffer.
 */
void honda_encode(const HondaFrame *frame, HondaRawBuf *buf)
{
    buf->count = 0;

    uint32_t word = honda_build_word(frame);

    for (int rep = 0; rep < 3; rep++) {
        /* Sync pulse */
        honda_push_pulse(buf, HONDA_SYNC_HIGH_US, HONDA_SYNC_LOW_US);

        /* 32 data bits, MSB-first on air (firmware bit-loop from MSB) */
        for (int bit = 31; bit >= 0; bit--) {
            bool b = (word >> bit) & 1u;
            honda_push_pulse(buf,
                HONDA_PULSE_HIGH_US,
                b ? HONDA_BIT1_LOW_US : HONDA_BIT0_LOW_US);
        }

        /* Inter-frame gap (last repetition gap is optional but harmless) */
        if (buf->count < 256) {
            buf->pulses[buf->count++] = -(int32_t)HONDA_REPEAT_GAP_US;
        }
    }
}

/* -------------------------------------------------------------------------
 * Decode
 * ------------------------------------------------------------------------- */

/** Check whether a measured duration is within tolerance of a reference */
static bool honda_in_range(int32_t measured_us, uint32_t ref_us)
{
    int32_t ref  = (int32_t)ref_us;
    int32_t diff = measured_us - ref;
    if (diff < 0) diff = -diff;
    return (diff * 100) <= (ref * (int32_t)HONDA_TOLERANCE_PCT);
}

/**
 * honda_decode() — decode a raw pulse buffer into a HondaFrame.
 *
 * Scans the pulse list looking for a valid sync+32-bit sequence.
 * Sets frame->valid = true only if checksum matches.
 *
 * @param buf    Input raw pulse buffer from SubGHz capture.
 * @param frame  Output decoded frame.
 * @return       true if a valid frame was found, false otherwise.
 */
bool honda_decode(const HondaRawBuf *buf, HondaFrame *frame)
{
    memset(frame, 0, sizeof(*frame));

    for (uint32_t i = 0; i + 1 < buf->count; i++) {
        /* Look for sync: HIGH ~300 µs followed by LOW ~4500 µs */
        if (!honda_in_range( buf->pulses[i],     HONDA_SYNC_HIGH_US)) continue;
        if (!honda_in_range(-buf->pulses[i + 1], HONDA_SYNC_LOW_US))  continue;

        /* Sync found — attempt to read 32 bits */
        uint32_t j = i + 2;
        if (j + 63 >= buf->count) continue; /* not enough samples           */

        uint32_t word = 0;
        bool ok = true;

        for (int bit = 31; bit >= 0; bit--) {
            int32_t hi = buf->pulses[j];
            int32_t lo = buf->pulses[j + 1];
            j += 2;

            if (!honda_in_range(hi, HONDA_PULSE_HIGH_US)) { ok = false; break; }

            if (honda_in_range(-lo, HONDA_BIT1_LOW_US)) {
                word |= (1u << bit);
            } else if (honda_in_range(-lo, HONDA_BIT0_LOW_US)) {
                /* bit = 0, nothing to set */
            } else {
                ok = false;
                break;
            }
        }

        if (!ok) continue;

        /* Unpack fields */
        frame->fixed_id  = (uint16_t)((word >> 16) & 0xFFFFu);
        frame->counter   = (uint8_t) ((word >>  8) & 0xFFu);
        frame->button    = (uint8_t) ((word >>  4) & 0xFu);
        frame->checksum  = (uint8_t) ( word        & 0xFu);

        /* Verify checksum */
        uint8_t expected = honda_checksum(word);
        frame->valid = (expected == frame->checksum);

        if (frame->valid) return true;
    }

    return false;
}

/* -------------------------------------------------------------------------
 * Rolling counter management
 * ------------------------------------------------------------------------- */

/**
 * honda_counter_valid() — check whether a received counter is in the
 * acceptable window ahead of the stored last-seen counter.
 *
 * Honda receivers accept counters in [last+1, last+16].
 *
 * @param stored    Last accepted counter value (0..255, wraps).
 * @param received  Counter value from the received frame.
 * @return          true if within window.
 */
bool honda_counter_valid(uint8_t stored, uint8_t received)
{
    uint8_t delta = (uint8_t)(received - stored);   /* wraps at 256 */
    return (delta >= 1u && delta <= 16u);
}

/* -------------------------------------------------------------------------
 * Flipper Zero SubGHz plugin glue
 *
 * Wire these callbacks into your SubGhzProtocolDecoderBase /
 * SubGhzProtocolEncoderBase vtable.
 * ------------------------------------------------------------------------- */

/*
 * Example encoder callback (adapt to your plugin's encode API):
 *
 *   void flipper_honda_encode(SubGhzProtocolEncoder* encoder, void* context) {
 *       HondaFrame *f = (HondaFrame*)context;
 *       HondaRawBuf raw;
 *       honda_encode(f, &raw);
 *       // Feed raw.pulses / raw.count to SubGHz RAW transmit
 *   }
 *
 * Example decoder callback (adapt to your plugin's decode API):
 *
 *   bool flipper_honda_decode(SubGhzProtocolDecoder* decoder,
 *                             const int32_t* pulses, uint32_t count,
 *                             void* context) {
 *       HondaRawBuf buf = { .count = count };
 *       memcpy(buf.pulses, pulses, count * sizeof(int32_t));
 *       HondaFrame frame;
 *       if (honda_decode(&buf, &frame) && frame.valid) {
 *           // Save frame, update rolling counter storage, etc.
 *           return true;
 *       }
 *       return false;
 *   }
 */
