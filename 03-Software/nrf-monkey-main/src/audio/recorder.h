#ifndef _RECORDER_H_
#define _RECORDER_H_

#include <zephyr/drivers/i2s.h>

// extern bool recorder_isEnable;
extern struct k_sem recorder_toggle_transfer_sem;
extern struct k_sem thread_i2s_busy_sem;
extern struct k_sem thread_recorder_store_busy_sem;

void recorder_enable_record(void);
void recorder_disable_record(void);
bool recorder_is_busy(void);

void recorder_enable_record_saving(void);
void recorder_disable_record_saving(void);

void recorder_i2s_initialize(struct i2s_config *config);

void recorder_get_dc_offset(void* samples, uint32_t samples_size);
int32_t recorder_normalize_sample(int32_t sample, int gain, int divider, uint8_t rshift);
bool recorder_calibration(const struct device *const i2s_dev_rx, const struct device *const i2s_dev_tx);

bool recorder_configure_streams(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx, const struct i2s_config *config);
bool recorder_prepare_transfer(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx);
bool recorder_trigger_command(const struct device *i2s_dev_rx, const struct device *i2s_dev_tx, enum i2s_trigger_cmd cmd);

// Around Thread
void recorder_thread_i2s(void);
void recorder_thread_store_to_file(void);
#endif // _RECORDER_H_
