#include "kibisis_encoder.hpp"

#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "config.hpp"
#include "kibisis_pwm_converter.hpp"
#include "quadrature_encoder.pio.h"

// ─────────────────────────────────────────────────────────────────────────────
// kibisis_encoder.cpp
//
// Two PIO state machines on pio0 maintain signed 32-bit encoder counts
// entirely in hardware. encoder_update() snapshots the latest count from
// each FIFO and forwards it to the PWM converter for I2C readback.
//
// The PIO program handles all quadrature decoding and count arithmetic –
// there is no software lookup table here.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

struct EncoderChannel {
    PIO  pio;
    uint sm;
};

EncoderChannel s_left;
EncoderChannel s_right;

// Drain the FIFO and return the latest count pushed by the PIO program.
// If the FIFO is empty (no edges since last call) returns the provided
// fallback value unchanged.
int32_t read_latest_count(const EncoderChannel& enc, int32_t fallback)
{
    int32_t count = fallback;
    while (!pio_sm_is_rx_fifo_empty(enc.pio, enc.sm))
        count = static_cast<int32_t>(pio_sm_get(enc.pio, enc.sm));
    return count;
}

} // anonymous namespace

namespace kibisis {

// Cached counts so encoder_get_left/right() always return a valid value
// even between FIFO updates.
static int32_t s_count_left  = 0;
static int32_t s_count_right = 0;

void encoder_init()
{
    // Load the PIO program once – both state machines share the same program
    const uint offset = pio_add_program(pio0, &quadrature_encoder_program);

    s_left  = {pio0, 0};
    s_right = {pio0, 1};

    quadrature_encoder_program_init(s_left.pio,  s_left.sm,  offset, ENCODER_LEFT_PIN_A);
    quadrature_encoder_program_init(s_right.pio, s_right.sm, offset, ENCODER_RIGHT_PIN_A);
}

void encoder_update()
{
    s_count_left  = read_latest_count(s_left,  s_count_left);
    s_count_right = read_latest_count(s_right, s_count_right);

    // Apply reversal correction
    const int32_t left  = ENCODER_LEFT_REVERSED  ? -s_count_left  : s_count_left;
    const int32_t right = ENCODER_RIGHT_REVERSED ? -s_count_right : s_count_right;

    // Forward to PWM converter for I2C readback to Pi 5.
    // Note: truncated to uint16 – to be extended to int32 in a future update.
    pwm_converter_set_encoder(
        static_cast<uint16_t>(left),
        static_cast<uint16_t>(right));
}

int32_t encoder_get_left()
{
    return ENCODER_LEFT_REVERSED ? -s_count_left : s_count_left;
}

int32_t encoder_get_right()
{
    return ENCODER_RIGHT_REVERSED ? -s_count_right : s_count_right;
}

} // namespace kibisis
