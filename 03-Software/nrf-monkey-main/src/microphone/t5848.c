/*
 * Copyright (c) 2025, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/drivers/gpio.h>

#include "t5848.h"

#include "../define.h"

LOG_MODULE_REGISTER(t5848, CONFIG_T5848_LOG_LEVEL);

// One wire symbol duration
#define T5848_START_PILOT_CLKS  10    
#define T5848_ZERO	      		1 * T5848_START_PILOT_CLKS
#define T5848_ONE	      		3 * T5848_START_PILOT_CLKS
#define T5848_STOP	      		13 * T5848_START_PILOT_CLKS
#define T5848_SPACE	      		1 * T5848_START_PILOT_CLKS

#define T5848_DEVICE_ADDRESS    0x53

#define T5848_POST_WRITE_CYCLES 60
#define T5848_PRE_WRITE_CYCLES  60
#define T5848_CLK_PERIOD_US     10


/**
 * @brief Generate AAD A configuration register pairs
 *
 * Generates a sequence of register address/data pairs for configuring the AAD 
 * module on the T5848 audio processor.
 *
 * @param[in] config Pointer to the AAD A configuration structure
 * @param[out] reg_data_pairs Array to store the generated register address/data pairs
 *
 * @return Number of register pairs generated and populated in reg_data_pairs
 *
 */
int t5848_generate_aad_a_pair(const struct t5848_aad_a_conf *config, struct t5848_address_data_pair *reg_data_pairs) 
{
    uint8_t i = 0;

    // AAD unlock write sequence
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};

    // AAD A LPF values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_LPF, (uint8_t) config->aad_a_lpf};
    // AAD A TH values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_THR, (uint8_t) config->aad_a_thr};
    // AAD A enabled
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, (uint8_t) config->aad_select};

    return i;
}

/**
 * @brief Generate AAD D configuration register pairs
 *
 * Generates a sequence of register address/data pairs for configuring the AAD 
 * module on the T5848 audio processor.
 *
 * @param[in] config Pointer to the AAD D configuration structure
 * @param[out] reg_data_pairs Array to store the generated register address/data pairs
 *
 * @return Number of register pairs generated and populated in reg_data_pairs
 *
 */
int t5848_generate_aad_d_pair(const struct t5848_aad_d_conf *config, struct t5848_address_data_pair *reg_data_pairs) 
{
    uint8_t i = 0;

    // AAD unlock write sequence
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};

    // Floor values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_HI, (uint8_t)((config->aad_d_floor >> 8) & 0x1F)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_LO, (uint8_t)(config->aad_d_floor & 0xFF)};

    // Reserved register (0x2C)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_RESERVED, 0x32};
 
    // Algo select (0x2D)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_AADD_ALGO_SEL, (uint8_t)((config->aad_d_algo_sel << 6) & 0xC0)}; //ERROR

    // Pulse minimums 
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_PULSE_MIN_LO, (uint8_t)(config->aad_d_rel_pulse_min & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED, (uint8_t)((config->aad_d_abs_pulse_min >> 4) / 0xF0) | ((config->aad_d_rel_pulse_min >> 8) & 0x0F)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_PULSE_MIN_LO, (uint8_t)(config->aad_d_abs_pulse_min & 0xFF)};

    // Absolute threshold
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_LO, (uint8_t)(config->aad_d_abs_thr & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_HI, (uint8_t)(((config->aad_d_abs_thr >> 8) & 0x1F) | 0x40)}; 

    // Relative threshold (0x33)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_THR, config->aad_d_rel_thr}; 

    // AAD D enabled
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, config->aad_select};

    return i;
}

/**
 * @brief Generate AAD A and D configuration register pairs
 *
 * Generates a sequence of register address/data pairs for configuring both the AAD 
 * A and AAD D modules on the T5848 audio processor.
 *
 * @param[in] config_a Pointer to the AAD A configuration structure
 * @param[in] config_d Pointer to the AAD D configuration structure
 * @param[out] reg_data_pairs Array to store the generated register address/data pairs
 *
 */
int t5848_generate_aad_a_and_d_pair(const struct t5848_aad_a_conf *config_a, const struct t5848_aad_d_conf *config_d, struct t5848_address_data_pair *reg_data_pairs){
    uint8_t i = 0;

    // AAD unlock write sequence
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};

    // ------------------------------- AAD A CONFIG -------------------------------
    // AAD A LPF values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_LPF, (uint8_t) config_a->aad_a_lpf};
    // AAD A TH values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_THR, (uint8_t) config_a->aad_a_thr};

    // ------------------------------- AAD D CONFIG -------------------------------
    // Floor values
    // Floor values
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_HI, (uint8_t)((config_d->aad_d_floor >> 8) & 0x1F)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_LO, (uint8_t)(config_d->aad_d_floor & 0xFF)};

    // Reserved register (0x2C)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_RESERVED, 0x32};
 
    // Algo select (0x2D)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_AADD_ALGO_SEL, (uint8_t)((config_d->aad_d_algo_sel << 6) & 0xC0)};

    // Pulse minimums 
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_PULSE_MIN_LO, (uint8_t)(config_d->aad_d_rel_pulse_min & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED, (uint8_t)((config_d->aad_d_abs_pulse_min >> 4) & 0xF0) | ((config_d->aad_d_rel_pulse_min >> 8) & 0x0F)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_PULSE_MIN_LO, (uint8_t)(config_d->aad_d_abs_pulse_min & 0xFF)};

    // Absolute threshold
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_LO, (uint8_t)(config_d->aad_d_abs_thr & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_HI, (uint8_t)(((config_d->aad_d_abs_thr >> 8) & 0x1F) | 0x40)}; 

    // Relative threshold (0x33)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_THR, config_d->aad_d_rel_thr}; 

    // AAD enabled
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, (config_a->aad_select + config_d->aad_select)};

    return i;
}

/**
 * @brief Toggle GPIO clock pin for a specified number of cycles
 *
 * Generates a clock signal by toggling the clock GPIO pin at a specified period.
 * Each cycle consists of a high pulse followed by a low pulse.
 *
 * @param[in] cycles Number of clock cycles to generate
 * @param[in] period Clock period in microseconds
 * @param[in] clk_gpio Pointer to the clock GPIO pin specification
 *
 * @return void
 *
 */
void t5848_clock_bitbang(uint16_t cycles, uint16_t period, struct gpio_dt_spec *clk_gpio)
{
    for (int i = 0; i < cycles; i++) {
        gpio_pin_set_dt(clk_gpio, 1);
        k_busy_wait(period / 2);
        gpio_pin_set_dt(clk_gpio, 0);
        k_busy_wait(period / 2);
    }
}

/**
 * @brief Write a single register to the T5848 device via one-wire protocol
 *
 * Transmits a register address and data value to the T5848 using GPIO bit-banging
 * with one-wire protocol timing. Includes pre/post write cycles and proper signal encoding.
 *
 * @param[in] reg Register address to write
 * @param[in] data Data value to write
 * @param[in] clk_gpio Pointer to the clock GPIO pin specification
 * @param[in] thsel_gpio Pointer to the THSEL GPIO pin specification
 *
 * @return 0 on success, negative error code on failure
 *
 */
int t5848_reg_write(uint8_t reg, uint8_t data, struct gpio_dt_spec *clk_gpio, struct gpio_dt_spec *thsel_gpio)
{
    uint8_t wr_buf[] = {T5848_DEVICE_ADDRESS << 1, reg, data};
    uint8_t cyc_buf = 0;

    LOG_DBG("t5848_reg_write :, reg: 0x%x, data: 0x%x", reg, data);
    
    // Send PRE write
    gpio_pin_set_dt(thsel_gpio, 0);
    t5848_clock_bitbang(T5848_PRE_WRITE_CYCLES, T5848_CLK_PERIOD_US, clk_gpio);
    // Send START
    gpio_pin_set_dt(thsel_gpio, 1);
    t5848_clock_bitbang(T5848_START_PILOT_CLKS, T5848_CLK_PERIOD_US, clk_gpio);
    // Send SPACE
    gpio_pin_set_dt(thsel_gpio, 0);
    t5848_clock_bitbang(T5848_SPACE, T5848_CLK_PERIOD_US, clk_gpio);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 8; j++) {

            // Set correct symbol duration
            if (wr_buf[i] & BIT(7 - j)) {
                cyc_buf = T5848_ONE;
            } else {
                cyc_buf = T5848_ZERO;
            }

            // Send ONE for cyc_buf period
            gpio_pin_set_dt(thsel_gpio, 1);
            t5848_clock_bitbang(cyc_buf, T5848_CLK_PERIOD_US, clk_gpio);

            // Send SPACE
            gpio_pin_set_dt(thsel_gpio, 0);
            t5848_clock_bitbang(T5848_SPACE, T5848_CLK_PERIOD_US, clk_gpio);
        }
    }

    // Send STOP
    gpio_pin_set_dt(thsel_gpio, 1);
    t5848_clock_bitbang(T5848_STOP, T5848_CLK_PERIOD_US, clk_gpio);
    //gpio_pin_set_dt(&thsel_gpio, 0);

    // Send POST write
    t5848_clock_bitbang(T5848_POST_WRITE_CYCLES, T5848_CLK_PERIOD_US, clk_gpio);

    return 0;
}

/**
 * @brief Write complete configuration to the T5848 device
 *
 * Generates and writes a sequence of register address/data pairs to configure
 * the T5848 audio processor based on the provided configuration type (AAD A or AAD D).
 *
 * @param[in] config_a Pointer to the AAD A configuration structure (NULL if not used)
 * @param[in] config_d Pointer to the AAD D configuration structure (NULL if not used)
 * @param[in] clk_gpio Pointer to the clock GPIO pin specification
 * @param[in] thsel_gpio Pointer to the THSEL GPIO pin specification
 *
 * @return 0 on success, -EINVAL if config is NULL or type is invalid, 
 *         or negative error code from register write operations
 *
 */
int t5848_write_config(const struct t5848_aad_a_conf *config_a, const struct t5848_aad_d_conf *config_d, struct gpio_dt_spec *clk_gpio, struct gpio_dt_spec *thsel_gpio)
{
    uint8_t count = 0;
    struct t5848_address_data_pair reg_data_pairs[T5848_MAX_CONFIG_PAIRS];

    // AAD A configuration
    if (config_a != NULL && config_d == NULL) {
        count = t5848_generate_aad_a_pair(config_a, reg_data_pairs);

    // AAD D configuration
    } else if (config_a == NULL && config_d != NULL) {
        count = t5848_generate_aad_d_pair(config_d, reg_data_pairs);

    // AAD A a nd AAD D configuration
    } else if (config_a != NULL && config_d != NULL) {
        count = t5848_generate_aad_a_and_d_pair(config_a, config_d, reg_data_pairs);

    // Invalid configuration
    } else {
        LOG_ERR("Invalid argument: %d", -EINVAL);
        return -EINVAL;
    }

    // Write all the data pairs
    for (int i = 0; i < count; i++) {
        int err = t5848_reg_write(reg_data_pairs[i].address, reg_data_pairs[i].data, clk_gpio, thsel_gpio);
        if (err < 0) {
            LOG_ERR("Error during register writing : %d, addr: 0x%x, data: 0x%x", err,
                reg_data_pairs[i].address, reg_data_pairs[i].data);
            return err;
        }
    }

    LOG_DBG("MIC config sent.");
    return 0;
}
