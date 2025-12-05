/*
 * Copyright (c) 2023, HES-SO Valais-Wallis, HEI, Sion
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "t5848.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#include "../define.h"

LOG_MODULE_REGISTER(t5848, CONFIG_T5848_LOG_LEVEL);

#define T5848_START_PILOT_CLKS  10    
#define T5848_ZERO	      		1 * T5848_START_PILOT_CLKS
#define T5848_ONE	      		3 * T5848_START_PILOT_CLKS
#define T5848_STOP	      		13 * T5848_START_PILOT_CLKS
#define T5848_SPACE	      		1 * T5848_START_PILOT_CLKS

#define T5848_DEVICE_ADDRESS    0x53

#define T5848_POST_WRITE_CYCLES 60
#define T5848_PRE_WRITE_CYCLES  60
#define T5848_CLK_PERIOD_US     10

int t5848_generate_aad_a_pair(const struct t5848_aad_a_conf *config, struct t5848_address_data_pair *reg_data_pairs) 
{

    uint8_t i = 0;
    /* AAD UNLOCK WRITE SEQUENCE */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x5C, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3E, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x6F, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x3B, (uint8_t) 0x00};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){0x4C, (uint8_t) 0x00};

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, 0x00};

    /* AAD A LPF VALUES */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_LPF, (uint8_t) config->aad_a_lpf};
    /* AAD A TH VALUES */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_THR, (uint8_t) config->aad_a_thr};
    /* AAD A ENABLED */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, (uint8_t) config->aad_select};

    return i;
}

int t5848_generate_aad_d_pair(const struct t5848_aad_d_conf *config, struct t5848_address_data_pair *reg_data_pairs) 
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
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_HI, (uint8_t)(config->aad_d_floor >> 8)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_FLOOR_LO, (uint8_t)(config->aad_d_floor & 0xFF)};

    // Reserved Register (0x2C)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_RESERVED, 0x32};

    // Algo Select (0x2D)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_AADD_ALGO_SEL, (uint8_t)((config->aad_d_algo_sel << 6) & 0xC0)};

    /* Pulse Minimums */
    uint8_t rel_msb = (config->aad_d_rel_pulse_min >> 8) & 0x0F;
    uint8_t abs_msb = (config->aad_d_abs_pulse_min >> 8) & 0x0F;
    uint8_t shared = (abs_msb << 4) | rel_msb;

    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_PULSE_MIN_LO, (uint8_t)(config->aad_d_rel_pulse_min & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED, shared};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_PULSE_MIN_LO, (uint8_t)(config->aad_d_abs_pulse_min & 0xFF)};

    /* Absolute Threshold */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_LO, (uint8_t)(config->aad_d_abs_thr & 0xFF)};
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_ABS_THR_HI, (uint8_t)((config->aad_d_abs_thr >> 8) | 0x40)}; 

    // Relative Threshold (0x33)
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_D_REL_THR, config->aad_d_rel_thr};

    /* Enable Register - Written LAST */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, config->aad_select};

    // Log sequence
    // for(int j = 0; j < i; j++){
    //     // Access array normally, pointer is still at start
    //     LOG_INF("%d. %d, %d", j, reg_data_pairs[j].address, reg_data_pairs[j].data);
    // }

    // LOG_INF("AAD D %d pairs generated.", i);

    return i;
}

void t5848_clock_bitbang(uint16_t cycles, uint16_t period, struct gpio_dt_spec *clk_gpio)
{
	for (int i = 0; i < cycles; i++) {
		gpio_pin_set_dt(clk_gpio, 1);
		k_busy_wait(period / 2);
		gpio_pin_set_dt(clk_gpio, 0);
		k_busy_wait(period / 2);
	}
}

int t5848_reg_write(uint8_t reg, uint8_t data, struct gpio_dt_spec *clk_gpio, struct gpio_dt_spec *thsel_gpio)
{
	uint8_t wr_buf[] = {T5848_DEVICE_ADDRESS << 1, reg, data};
	uint8_t cyc_buf = 0;

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
			if (wr_buf[i] & BIT(7 - j)) {
                // Send ONE
				cyc_buf = T5848_ONE;
			} else {
                // Send ZERO
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

int t5848_write_config(const struct t5848_config_container *config, struct gpio_dt_spec *clk_gpio, struct gpio_dt_spec *thsel_gpio)
{
    /* 
    * -EINVAL means that the argument is invalid.
    */
    if (config == NULL) {
        LOG_ERR("Invalid argument: %d", -EINVAL);
        return -EINVAL;
    }

    struct t5848_address_data_pair reg_data_pairs[T5848_MAX_CONFIG_PAIRS]; 
    uint8_t count = 0;

    switch (config->type) {
        case T5848_CONF_AAD_A:
            struct t5848_aad_a_conf *conf_a = (struct t5848_aad_a_conf *)&config->config.a;
            count = t5848_generate_aad_a_pair(conf_a, reg_data_pairs);
            break;

        case T5848_CONF_AAD_D:
            struct t5848_aad_d_conf *conf_d = (struct t5848_aad_d_conf *)&config->config.d;
            count = t5848_generate_aad_d_pair(conf_d, reg_data_pairs);
            break;

        default:
            /* 
            * -EINVAL means that the type is invalid.
            */
            return -EINVAL;
    }

	for (int i = 0; i < count; i++) {
		int err = t5848_reg_write(reg_data_pairs[i].address, reg_data_pairs[i].data, clk_gpio, thsel_gpio);
		if (err < 0) {
			LOG_ERR("Error during register writing : %d, addr: 0x%x, data: 0x%x", err,
				reg_data_pairs[i].address, reg_data_pairs[i].data);
			return err;
		}
    }

    //LOG_INF("MIC config sent.");
	return 0;
}
