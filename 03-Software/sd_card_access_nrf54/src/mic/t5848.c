/** @file t5848_aad_mode.c
 *
 * @brief A description of the module's purpose.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2023 Irnas. All rights reserved.
 */

#include "t5848.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(t5848, CONFIG_T5848_LOG_LEVEL);

#define T5848_START_PILOT_CLKS 	10      // Max value 10 -> overflow
#define T5848_ZERO	      		1 * T5848_START_PILOT_CLKS
#define T5848_ONE	      		3 * T5848_START_PILOT_CLKS
#define T5848_STOP	      		130 * T5848_START_PILOT_CLKS
#define T5848_SPACE	      		1 * T5848_START_PILOT_CLKS

#define T5848_POST_WRITE_CYCLES 60 /* >50clk cycles */
#define T5848_PRE_WRITE_CYCLES  60 /* >50clk cycles */
#define T5848_DEVICE_ADDRESS    0x53

#define T5848_CLK_PERIOD_US 10 /* approx 100kHz */

/* >2ms of clock is required before entering sleep with AAD  */
#define T5848_ENTER_SLEEP_MODE_CLOCKING_TIME_US 2500
#define T5848_ENTER_SLEEP_MODE_CLK_PERIOD_US	10 /* approx 100kHz */

#define T5848_RESET_TIME_MS 10 /* Time required for device to reset when power is disabled*/

#define SPI_BUF_SIZE 512
static uint8_t tx_buffer[SPI_BUF_SIZE];
static uint32_t bit_cursor = 0;

int t5848_generate_aad_a_pair(const struct t5848_aad_a_conf *conf, struct t5848_address_data_pair *reg_data_pairs) 
{
    uint8_t i = 0;
    /* AAD UNLOCK WRITE SEQUENCE */
    *reg_data_pairs++ = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    i++;
    *reg_data_pairs++ = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    i++;
    *reg_data_pairs++ = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    i++;
    *reg_data_pairs++ = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    i++;
    *reg_data_pairs++ = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};
    i++;

    *reg_data_pairs++ = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};
    i++;

    /* AAD A LPF VALUES */
    *reg_data_pairs++ = (struct t5848_address_data_pair){T5848_REG_AAD_A_LPF, (uint8_t) conf->aad_a_lpf};
    i++;
     /* AAD A TH VALUES */
    *reg_data_pairs++ = (struct t5848_address_data_pair){T5848_REG_AAD_A_THR, (uint8_t) conf->aad_a_thr};
    i++;
    /* AAD A ENABLED */
    *reg_data_pairs++ = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, (uint8_t) conf->aad_select};
    i++;

    printk("AAD A %d pairs generated.", i);
    return i;
}

int t5848_generate_aad_d_pair(const struct t5848_aad_d_conf *conf, struct t5848_address_data_pair *reg_data_pairs) 
{
    uint8_t i = 0;

    /* AAD UNLOCK WRITE SEQUENCE */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};

    /* FLOOR VALUES */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_HI, (uint8_t)(conf->aad_d_floor >> 8)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_LO, (uint8_t)(conf->aad_d_floor & 0xFF)};

    // Reserved Register (0x2C)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_RESERVED, 0x32};

    // Algo Select (0x2D)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_AADD_ALGO_SEL, (uint8_t)((conf->aad_d_algo_sel << 6) & 0xC0)};

    /* Pulse Minimums */
    uint8_t rel_msb = (conf->aad_d_rel_pulse_min >> 8) & 0x0F;
    uint8_t abs_msb = (conf->aad_d_abs_pulse_min >> 8) & 0x0F;
    uint8_t shared = (abs_msb << 4) | rel_msb;

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_PULSE_MIN_LO, (uint8_t)(conf->aad_d_rel_pulse_min & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED, shared};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_PULSE_MIN_LO, (uint8_t)(conf->aad_d_abs_pulse_min & 0xFF)};

    /* Absolute Threshold */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_LO, (uint8_t)(conf->aad_d_abs_thr & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_HI, (uint8_t)((conf->aad_d_abs_thr >> 8) | 0x40)}; 

    // Relative Threshold (0x33)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_THR, conf->aad_d_rel_thr};

    /* Enable Register - Written LAST */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, conf->aad_select};

    // Log sequence
    // for(int j = 0; j < i; j++){
    //     // Access array normally, pointer is still at start
    //     LOG_INF("%d. %d, %d", j, reg_data_pairs[j].address, reg_data_pairs[j].data);
    // }

    // LOG_INF("AAD D %d pairs generated.", i);

    return i;
}

// Flushes buffer to hardware
int flush_buffer(struct spi_dt_spec *spec) {
    if (bit_cursor == 0) return;
    uint32_t bytes_to_send = (bit_cursor + 7) / 8;

    struct spi_buf tx_buf = {.buf = tx_buffer, .len = bytes_to_send};
    struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

    int err = spi_write_dt(spec, &tx_bufs);
    if (err < 0) {
        LOG_ERR("spi_write_dt() failed, err: %d", err);
        return err;
    }
    LOG_INF("Bytes send to mic : %d", bytes_to_send);
    
    // Reset for next chunk
    memset(tx_buffer, 0, SPI_BUF_SIZE);
    bit_cursor = 0;
    return 0;
}

/**
 * @brief Appends 'count' bits of value 'val' (0 or 1) to the buffer
 */
void append_raw_sequence(uint8_t val, uint32_t count) {
    for (int i = 0; i < count; i++) {
        uint32_t byte_index = bit_cursor / 8;
        uint32_t bit_pos    = 7 - (bit_cursor % 8);

        if (byte_index >= SPI_BUF_SIZE) {
            LOG_ERR("Buffer overflow! Increase SPI_BUF_SIZE");
            return; 
        }

        if (val) {
            tx_buffer[byte_index] |= (1 << bit_pos);
        } else {
            tx_buffer[byte_index] &= ~(1 << bit_pos);
        }
        bit_cursor++;
    }
}

/**
 * @brief Encodes a logical byte into the PWM waveform
 */
void encode_logical_byte(uint8_t val) {
    // Iterate MSB to LSB (7 down to 0)
    for (int i = 7; i >= 0; i--) {
        uint8_t current_bit = (val << (7 - i)) & 128;

        if (current_bit) {
            // Logic 1: Long Pulse + Space
            append_raw_sequence(1, T5848_ONE);
            append_raw_sequence(0, T5848_SPACE);
        } else {
            // Logic 0: Short Pulse + Space
            append_raw_sequence(1, T5848_ZERO);
            append_raw_sequence(0, T5848_SPACE);
        }
    }
}

void t5848_generate_bit_pattern(struct t5848_address_data_pair *reg_data_pairs, size_t count, struct spi_dt_spec *spec) {
    for (int i = 0; i < count; i++) {
        // 1. Start Sequence: Space, Start(High), Space
        append_raw_sequence(0, T5848_SPACE);
        append_raw_sequence(1, T5848_START_PILOT_CLKS);
        append_raw_sequence(0, T5848_SPACE);

        // 2. Build Write Sequence: [DeviceWrite, Addr, Data]
        encode_logical_byte((uint8_t) T5848_DEVICE_WRITE_PATTERN);
        encode_logical_byte((uint8_t) reg_data_pairs[i].address);
        encode_logical_byte((uint8_t) reg_data_pairs[i].data);

        // 3. Stop Sequence
        append_raw_sequence(1, T5848_STOP);

        flush_buffer(spec);
    }
}

