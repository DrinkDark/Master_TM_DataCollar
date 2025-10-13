/*
 * Example: I2S with T5848 digital microphone (nRF54L15 DK as I2S master)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>

#if DT_NODE_EXISTS(DT_NODELABEL(i2s_rxtx))
#define I2S_RX_NODE  DT_NODELABEL(i2s_rxtx)
#define I2S_TX_NODE  I2S_RX_NODE
#else
#define I2S_RX_NODE  DT_NODELABEL(i2s_rx)
#define I2S_TX_NODE  DT_NODELABEL(i2s_tx)
#endif

#define I2S_MODE_HQM
//#define I2S_MODE_HQM

/* === Audio configuration === */
#define I2S_SAMPLE_FREQUENCY_SLEEP  2048     /* SCK = 2048 * 32 * 2 = 131 kHz : Sleep */

#if defined(I2S_MODE_LPM)
    #define SAMPLE_FREQUENCY    16000    /* SCK = 16000 * 24 * 2 = 768 kHz : Low Power Mode*/
    #define SAMPLE_BIT_WIDTH    16        /* 24 bits on the I2S bus */ 
#endif

#if defined(I2S_MODE_HQM)
    #define I2S_SAMPLE_FREQUENCY    39000    /* SCK = 39062 * 32 * 2 = 2.4 MHz : High Quality Mode*/
    #define I2S_SAMPLE_BIT_WIDTH    32       /* 24 bits on a 32 bits word on the I2S bus */ 
#endif

#if !defined(I2S_MODE_LPM) && !defined(I2S_MODE_HQM)
    printk("Please define either I2S_MODE_LPM or I2S_MODE_HQM");
#endif

#define I2S_BYTES_PER_SAMPLE        sizeof(int32_t)
#define I2S_SAMPLES_PER_BLOCK       ((I2S_SAMPLE_FREQUENCY / 10) * 2)
#define I2S_NUMBER_OF_CHANNELS      2       /* Always 2 in I2S mode */
#define I2S_TIMEOUT                 SYS_FOREVER_MS

#define I2S_BLOCK_SIZE  (I2S_BYTES_PER_SAMPLE * I2S_SAMPLES_PER_BLOCK)

K_MEM_SLAB_DEFINE_STATIC(i2s_mem_slab, I2S_BLOCK_SIZE, 4, 4);

/* Buffer for simple processing */
static int16_t process_buf[I2S_SAMPLES_PER_BLOCK];

static int16_t echo_block[I2S_SAMPLES_PER_BLOCK];
static volatile bool echo_enabled = false;
static K_SEM_DEFINE(toggle_transfer, 1, 1);

static void process_block_data(void *mem_block, uint32_t number_of_samples)
{
	static bool clear_echo_block;

	if (echo_enabled) {
		for (int i = 0; i < number_of_samples; ++i) {
			int16_t *sample = &((int16_t *)mem_block)[i];
			*sample += echo_block[i];
			echo_block[i] = (*sample) / 2;
		}

		clear_echo_block = true;
	} else if (clear_echo_block) {
		clear_echo_block = false;
		memset(echo_block, 0, sizeof(echo_block));
	}
}


static bool configure_streams(const struct device *i2s_dev_rx,
			      const struct device *i2s_dev_tx,
			      const struct i2s_config *config)
{
	int ret;

	if (i2s_dev_rx == i2s_dev_tx) {
		ret = i2s_configure(i2s_dev_rx, I2S_DIR_BOTH, config);
		if (ret == 0) {
			return true;
		}
		/* -ENOSYS means that the RX and TX streams need to be
		 * configured separately.
		 */
		if (ret != -ENOSYS) {
			printk("Failed to configure streams: %d\n", ret);
			return false;
		}
	}

	ret = i2s_configure(i2s_dev_rx, I2S_DIR_RX, config);
	if (ret < 0) {
		printk("Failed to configure RX stream: %d\n", ret);
		return false;
	}

	ret = i2s_configure(i2s_dev_tx, I2S_DIR_TX, config);
	if (ret < 0) {
		printk("Failed to configure TX stream: %d\n", ret);
		return false;
	}

	return true;
}


static bool trigger_command(const struct device *i2s_dev_rx,
			    const struct device *i2s_dev_tx,
			    enum i2s_trigger_cmd cmd)
{
	int ret;

	if (i2s_dev_rx == i2s_dev_tx) {
		ret = i2s_trigger(i2s_dev_rx, I2S_DIR_BOTH, cmd);
		if (ret == 0) {
			return true;
		}
		/* -ENOSYS means that commands for the RX and TX streams need
		 * to be triggered separately.
		 */
		if (ret != -ENOSYS) {
			printk("Failed to trigger command %d: %d\n", cmd, ret);
			return false;
		}
	}

	ret = i2s_trigger(i2s_dev_rx, I2S_DIR_RX, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d on RX: %d\n", cmd, ret);
		return false;
	}

	ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, cmd);
	if (ret < 0) {
		printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
		return false;
	}

	return true;
}


static bool prepare_transfer(const struct device *i2s_dev_rx,
			     const struct device *i2s_dev_tx)
{
	int ret;

	for (int i = 0; i < 2; ++i) {
		void *mem_block;

		ret = k_mem_slab_alloc(&i2s_mem_slab, &mem_block, K_NO_WAIT);
		if (ret < 0) {
			printk("Failed to allocate TX block %d: %d\n", i, ret);
			return false;
		}

		memset(mem_block, 0, I2S_BLOCK_SIZE);

		ret = i2s_write(i2s_dev_tx, mem_block, I2S_BLOCK_SIZE);
		if (ret < 0) {
			printk("Failed to write block %d: %d\n", i, ret);
			return false;
		}
	}

	return true;
}


int main(void)
{
    const struct device *const i2s_dev_rx = DEVICE_DT_GET(I2S_RX_NODE);
    const struct device *const i2s_dev_tx = DEVICE_DT_GET(I2S_TX_NODE);

    struct i2s_config config;
    int ret;

    printk("=== I2S RX ===\n");

    /* Check I2S device */
    if (!device_is_ready(i2s_dev_rx)) {
        printk("I2S device %s not ready\n", i2s_dev_rx->name);
        return 0;
    }

    if (!device_is_ready(i2s_dev_tx)) {
        printk("I2S TX device %s not ready\n", i2s_dev_tx->name);
        return 0;
    }

    /* Configure I2S RX only */
    config.word_size      = I2S_SAMPLE_BIT_WIDTH;
    config.channels       = I2S_NUMBER_OF_CHANNELS;  // Ignored in I2S mode, always 2
    config.format         = I2S_FMT_DATA_FORMAT_I2S;
    config.options        = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    config.frame_clk_freq = I2S_SAMPLE_FREQUENCY;
    config.mem_slab       = &i2s_mem_slab;
    config.block_size     = I2S_BLOCK_SIZE;
    config.timeout        = I2S_TIMEOUT;

    /*ret = i2s_configure(i2s_dev_rx, I2S_DIR_RX, &config);
    if (ret < 0) {
        printk("Failed to configure I2S RX: %d\n", ret);
        return 0;
    }*/

    if (!configure_streams(i2s_dev_rx, i2s_dev_tx, &config)) {
        return 0;
    }

    /*ret = i2s_trigger(i2s_dev_rx, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start I2S RX: %d\n", ret);
        return 0;
    }*/
    
    printk("Audio capture started...\n");

    /*void *mem_block;
    uint32_t block_size;

    ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
    if (ret < 0) {
        printk("I2S read error: %d\n", ret);
        if (mem_block != NULL) {
            k_mem_slab_free(&i2s_mem_slab, &mem_block);
        }
        return 0;
    }
    printk("Read success: %d bytes\n", (int)block_size);

    process_block_data(mem_block, block_size);
    
    k_mem_slab_free(&i2s_mem_slab, mem_block);
    k_sleep(K_MSEC(10));

    return 0;*/

    for (;;) {
		k_sem_take(&toggle_transfer, K_FOREVER);

		if (!prepare_transfer(i2s_dev_rx, i2s_dev_tx)) {
			return 0;
		}

		if (!trigger_command(i2s_dev_rx, i2s_dev_tx,
				     I2S_TRIGGER_START)) {
			return 0;
		}

		printk("Streams started\n");

		while (k_sem_take(&toggle_transfer, K_NO_WAIT) != 0) {
			void *mem_block;
			uint32_t block_size;
			int ret;

			ret = i2s_read(i2s_dev_rx, &mem_block, &block_size);
			if (ret < 0) {
				printk("Failed to read data: %d\n", ret);
				break;
			}

			process_block_data(mem_block, I2S_SAMPLES_PER_BLOCK);

            ret = i2s_write(i2s_dev_tx, mem_block, block_size);
            if (ret < 0) {
                printk("Failed to write data: %d\n", ret);
                break;
            }
		}

		if (!trigger_command(i2s_dev_rx, i2s_dev_tx,
				     I2S_TRIGGER_DROP)) {
			return 0;
		}

		printk("Streams stopped\n");
	}
}
