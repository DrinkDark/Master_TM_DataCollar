/** @file t5848_aad_mode.c
 *
 * @brief A description of the module's purpose.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2023 Irnas. All rights reserved.
 */

#include "t5848.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#include "../define.h"

LOG_MODULE_REGISTER(t5848, CONFIG_T5848_LOG_LEVEL);

K_SEM_DEFINE(thread_t5848_config, 1, 1);

#define T5848_START_PILOT_CLKS  10    
#define T5848_ZERO	      		1 * T5848_START_PILOT_CLKS
#define T5848_ONE	      		3 * T5848_START_PILOT_CLKS
#define T5848_STOP	      		13 * T5848_START_PILOT_CLKS
#define T5848_SPACE	      		1 * T5848_START_PILOT_CLKS

#define T5848_DEVICE_ADDRESS    0x53

#define T5848_POST_WRITE_CYCLES 60
#define T5848_PRE_WRITE_CYCLES  60
#define T5848_CLK_PERIOD_US     10

#define BUF_SIZE                512
static uint8_t tx_buffer[BUF_SIZE];
static uint32_t bit_cursor = 0;

struct gpio_dt_spec mic_clk_gpio    = GPIO_DT_SPEC_GET(MIC_CLK_NODE, gpios);
struct gpio_dt_spec mic_thsel_gpio  = GPIO_DT_SPEC_GET(MIC_THSEL_NODE, gpios);
struct gpio_dt_spec mic_wake_gpio   = GPIO_DT_SPEC_GET(MIC_WAKE_NODE, gpios);

static struct gpio_callback mic_wake_cb_data;

bool mic_init_done;
uint32_t pulse_start_cycle;

K_SEM_DEFINE(sem_mic_ready, 0, 1);

bool mic_initialize(void) {
    if (!gpio_is_ready_dt(&mic_wake_gpio)) {
        LOG_ERR("%s is not ready", mic_wake_gpio.port->name);
        return false;
    }

    int ret = gpio_pin_configure_dt(&mic_wake_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure %s pin %d: %d", mic_thsel_gpio.port->name, mic_thsel_gpio.pin, ret);
        return false;
    }

    ret = gpio_pin_interrupt_configure_dt(&mic_wake_gpio, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on %s pin %d: %d", mic_wake_gpio.port->name, mic_wake_gpio.pin, ret);
        return false;
    }

    gpio_init_callback(&mic_wake_cb_data, mic_wake_callback, BIT(mic_wake_gpio.pin));
    gpio_add_callback(mic_wake_gpio.port, &mic_wake_cb_data);

    ret = mic_config_pin_initialize();
    if (ret < 0) {
        return false;
    }

    LOG_INF("T5848 microphone initialized !");
}

bool mic_config_pin_initialize(void) {
    if (!gpio_is_ready_dt(&mic_clk_gpio)) {
        LOG_ERR("%s is not ready", mic_clk_gpio.port->name);
        return false;
    }

    if (!gpio_is_ready_dt(&mic_thsel_gpio)) {
        LOG_ERR("%s is not ready", mic_thsel_gpio.port->name);
        return false;
    }

    int ret = gpio_pin_configure_dt(&mic_clk_gpio, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure %s pin %d: %d", mic_clk_gpio.port->name, mic_clk_gpio.pin, ret);
        return false;
    }

    ret = gpio_pin_configure_dt(&mic_thsel_gpio, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure %s pin %d: %d", mic_thsel_gpio.port->name, mic_thsel_gpio.pin, ret);
        return false;
    }
}

bool mic_config_pin_release(void) {
	int ret = gpio_pin_configure_dt(&mic_clk_gpio, GPIO_DISCONNECTED);
    if (ret < 0) {
        LOG_ERR("Failed to configure %s pin %d: %d", mic_clk_gpio.port->name, mic_clk_gpio.pin, ret);
        return false;
    }

    ret = gpio_pin_configure_dt(&mic_thsel_gpio, GPIO_DISCONNECTED);
    if (ret < 0) {
        LOG_ERR("Failed to configure %s pin %d: %d", mic_thsel_gpio.port->name, mic_thsel_gpio.pin, ret);
        return false;
    }

    LOG_INF("T5848 pins disconnected (High-Z)");
}

int mic_generate_aad_a_pair(const struct t5848_aad_a_conf *conf, struct t5848_address_data_pair *reg_data_pairs) 
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
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_LPF, (uint8_t) conf->aad_a_lpf};
    /* AAD A TH VALUES */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_A_THR, (uint8_t) conf->aad_a_thr};
    /* AAD A ENABLED */
    reg_data_pairs[i++] = (struct t5848_address_data_pair){T5848_REG_AAD_MODE, (uint8_t) conf->aad_select};

    return i;
}

int mic_generate_aad_d_pair(const struct t5848_aad_d_conf *conf, struct t5848_address_data_pair *reg_data_pairs) 
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

void mic_clock_bitbang(uint16_t cycles, uint16_t period)
{
	for (int i = 0; i < cycles; i++) {
		gpio_pin_set_dt(&mic_clk_gpio, 1);
		k_busy_wait(period / 2);
		gpio_pin_set_dt(&mic_clk_gpio, 0);
		k_busy_wait(period / 2);
	}
}

int mic_reg_write(uint8_t reg, uint8_t data)
{
	LOG_INF("reg_write, reg: 0x%x, data: 0x%x", reg, data);

    /**prepare data */
	uint8_t wr_buf[] = {T5848_DEVICE_ADDRESS << 1, reg, data};
	/* put into wr_buf since it gets written first */
	uint8_t cyc_buf = 0;

    /** start with thsel low */
	gpio_pin_set_dt(&mic_thsel_gpio, 0);
	/** Clock device before writing to prepare device for communication */
	mic_clock_bitbang(T5848_PRE_WRITE_CYCLES, T5848_CLK_PERIOD_US);
	/** write start condition*/
	gpio_pin_set_dt(&mic_thsel_gpio, 1);
	mic_clock_bitbang(T5848_START_PILOT_CLKS, T5848_CLK_PERIOD_US);
	/** write first space before writing data */
	gpio_pin_set_dt(&mic_thsel_gpio, 0);
	mic_clock_bitbang(T5848_SPACE, T5848_CLK_PERIOD_US);
	/** write data */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 8; j++) {
			if (wr_buf[i] & BIT(7 - j)) {
				/* sending one */
				cyc_buf = T5848_ONE;
			} else {
				/* sending 0 */
				cyc_buf = T5848_ZERO;
			}
			/** Send data bit */
			gpio_pin_set_dt(&mic_thsel_gpio, 1);
			mic_clock_bitbang(cyc_buf, T5848_CLK_PERIOD_US);
			/** Send space */
			gpio_pin_set_dt(&mic_thsel_gpio, 0);
			mic_clock_bitbang(T5848_SPACE, T5848_CLK_PERIOD_US);
		}
	}
	/**write stop condition */
	gpio_pin_set_dt(&mic_thsel_gpio, 1);
	mic_clock_bitbang(T5848_STOP, T5848_CLK_PERIOD_US);
	//gpio_pin_set_dt(&mic_thsel_gpio, 0);

	/**keep clock to apply */
	mic_clock_bitbang(T5848_POST_WRITE_CYCLES, T5848_CLK_PERIOD_US);
	return 0;
}

int mic_write_config(struct t5848_address_data_pair *data, uint8_t num)
{
    mic_init_done = false;

	int err;
	for (int i = 0; i < num; i++) {
		err = mic_reg_write(data[i].address, data[i].data);
		if (err < 0) {
			LOG_ERR("multi_reg_write err: %d, addr: 0x%x, data: 0x%x", err,
				data[i].address, data[i].data);
			return err;
		}
    }

    mic_config_pin_release();
	return 0;
}

bool mic_wait_for_ack_pulse(void) {
    uint32_t start_cycles, end_cycles, duration_cycles;
    uint32_t duration_us;
    
    int32_t timeout_cycles = 6400000; 
    uint32_t entry_time = k_cycle_get_32();

    while (gpio_pin_get_dt(&mic_wake_gpio) == 0) {
        if ((k_cycle_get_32() - entry_time) > timeout_cycles) {
            LOG_ERR("MIC config pulse entry timeout.");
            return false;
        }
    }
    
    start_cycles = k_cycle_get_32();

    while (gpio_pin_get_dt(&mic_wake_gpio) == 1) {
        if ((k_cycle_get_32() - start_cycles) > timeout_cycles) {
            LOG_ERR("MIC config pulse exit timeout.");
            return false;
        }
    }

    end_cycles = k_cycle_get_32();

    if (end_cycles >= start_cycles) {
        duration_cycles = end_cycles - start_cycles;
    } else {
        duration_cycles = (UINT32_MAX - start_cycles) + end_cycles;
    }

    duration_us = k_cyc_to_us_floor32(duration_cycles);

    LOG_INF("Detected Pulse: %d us", duration_us);


    if (duration_us >= 10 && duration_us <= 14) {
        LOG_INF("Microphone has received config !");
        mic_init_done = true;
        k_sem_give(&sem_mic_ready);
        return true;
    } else {
        LOG_ERR("Pulse invalid! Expected ~12us, got %d us", duration_us);
        return false;
    }
}

void mic_wake_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t now = k_cycle_get_32();

    if (!mic_init_done) {
        if (gpio_pin_get_dt(&mic_wake_gpio) == 1) {
            pulse_start_cycle = now;
        } else {
            uint32_t pulse_stop_cycle = now;

            if (pulse_start_cycle == 0) {
                return; 
            }

            uint32_t duration_cycles = now - pulse_start_cycle;
            uint32_t duration_us = k_cyc_to_us_floor32(duration_cycles);
            pulse_start_cycle = 0; 

            if (duration_us >= 8 && duration_us <= 16) {
                LOG_INF("Microphone has received config !");
                mic_init_done = true;
                k_sem_give(&sem_mic_ready);
            } else {
                LOG_WRN("Pulse invalid! Expected ~12us, got %d us", duration_us);
            }
        }
    }
}