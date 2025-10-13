/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>

#define MODE_HQM
//#define MODE_HQM

/* === Audio configuration === */
#define SAMPLE_FREQUENCY_SLEEP  2048     /* SCK = 2048 * 32 * 2 = 131 kHz : Sleep */

#if defined(MODE_LPM)
    #define SAMPLE_FREQUENCY    16000    /* SCK = 16000 * 24 * 2 = 768 kHz : Low Power Mode*/
    #define SAMPLE_BIT_WIDTH    24        /* 24 bits on the I2S bus */ 
#endif

#if defined(MODE_HQM)
    #define SAMPLE_FREQUENCY    39000    /* SCK = 39062 * 32 * 2 = 2.4 MHz : High Quality Mode*/
    #define SAMPLE_BIT_WIDTH    32        /* 32 bits on the I2S bus */
#endif

#if !defined(MODE_LPM) && !defined(MODE_HQM)
    printk("Please define either MODE_LPM or MODE_HQM");
#endif

#define BYTES_PER_SAMPLE    	sizeof(int32_t) /* Always 32 bits on bus, lower 16 bits zero in LPM */
#define NUMBER_OF_CHANNELS      1
#define SAMPLES_PER_BLOCK       ((SAMPLE_FREQUENCY / 10) * 2)
#define TIMEOUT                 1000

#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)

#define NUM_BLOCKS 4

/* Fill buffer with sine wave on left channel, and sine wave shifted by
 * 90 degrees on right channel. "att" represents a power of two to attenuate
 * the samples by
 */
static void fill_buf(int16_t *tx_block, int att)
{
	int r_idx;

	for (int i = 0; i < NUM_BLOCKS; i++) {
		/* Left channel is sine wave */
		//tx_block[2 * i] = data[i] / (1 << att);
		tx_block[2 * i] = INT32_MAX;
		
		/* Right channel is same sine wave, shifted by 90 degrees */
		// r_idx = (i + (ARRAY_SIZE(data) / 4)) % ARRAY_SIZE(data);
		// tx_block[2 * i + 1] = INT16_MIN; // Right channel: min value
		tx_block[2 * i + 1] = INT32_MIN; // Right channel: min value
	}
}

#ifdef CONFIG_NOCACHE_MEMORY
	#define MEM_SLAB_CACHE_ATTR __nocache
#else
	#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
				WB_UP(BLOCK_SIZE), NUM_BLOCKS);

int main(void)
{
	void *tx_block[NUM_BLOCKS];

	struct i2s_config i2s_cfg;
	int ret;
	uint32_t tx_idx;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));

	if (!device_is_ready(dev_i2s)) {
		printk("I2S device not ready\n");
		return -ENODEV;
	}
	/* Configure I2S stream */
    i2s_cfg.word_size      = SAMPLE_BIT_WIDTH;
    i2s_cfg.channels       = NUMBER_OF_CHANNELS;
    i2s_cfg.format         = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options        = I2S_OPT_BIT_CLK_SLAVE | I2S_OPT_FRAME_CLK_SLAVE;
    i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY_SLEEP;
    i2s_cfg.block_size     = BLOCK_SIZE;
    i2s_cfg.timeout        = TIMEOUT;
	i2s_cfg.mem_slab 	   = &tx_0_mem_slab;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printk("Failed to configure I2S stream\n");
		return ret;
	}

	/* Prepare all TX blocks */
	for (tx_idx = 0; tx_idx < NUM_BLOCKS; tx_idx++) {
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block[tx_idx],
				       K_FOREVER);
		if (ret < 0) {
			printk("Failed to allocate TX block\n");
			return ret;
		}
		tx_block[tx_idx] = memset(tx_block[tx_idx], 0, BLOCK_SIZE);
		
		fill_buf((uint16_t *)tx_block[tx_idx], 0);
	}
	
	tx_idx = 0;
	/* Send first block */
	ret = i2s_write(dev_i2s, tx_block[tx_idx++], BLOCK_SIZE);
	if (ret < 0) {
		printk("Could not write TX buffer %d\n", tx_idx);
		return ret;
	}

	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("Could not trigger I2S tx\n");
		return ret;
	}

	for (; tx_idx < NUM_BLOCKS;) {
		ret = i2s_write(dev_i2s, tx_block[tx_idx++], BLOCK_SIZE);
		if (ret < 0) {
			printk("Could not write TX buffer %d\n", tx_idx);
			return ret;
		}
	}
	/* Drain TX queue */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		printk("Could not trigger I2S tx\n");
		return ret;
	}
	printk("All I2S blocks written\n");


	return 0;
}
