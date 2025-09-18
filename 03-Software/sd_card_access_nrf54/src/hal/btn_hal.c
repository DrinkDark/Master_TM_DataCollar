/*
 * Copyright (c) 2025, HES-SO Valais-Wallis, HEI, Sion
 */

#include "btn_hal.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "../define.h"
#include "../fatfs/sdcard.h"


LOG_MODULE_REGISTER(btn_hal, CONFIG_BTN_HAL_LOG_LEVEL);
static bool btn0_is_pressed = false;
static bool btn1_is_pressed = false;
static bool btn2_is_pressed = false;
static bool btn3_is_pressed = false;

// Handling Buttons on board
#if !DT_NODE_HAS_STATUS_OKAY(BTN0_NODE)
	#warning "Button0 is NOT okay"
    bool btn0_has_been_pressed(void) { return false; }
#else
	static struct gpio_dt_spec  btn0 = GPIO_DT_SPEC_GET(BTN0_NODE, gpios);
    static struct gpio_callback btn0_cb_data;
    static void btn0_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
    {
        LOG_DBG("Button_0 pressed");
        btn0_is_pressed = true;
        k_sem_give(&sdcard_activity_sem);
    }
    
    bool btn0_has_been_pressed(void) {
        if (btn0_is_pressed) {
            btn0_is_pressed = false;
            return true;
        }

        return false;
    }
#endif

#if !DT_NODE_HAS_STATUS_OKAY(BTN1_NODE)
	#warning "Button1 is NOT okay"
    bool btn1_has_been_pressed(void) { return false; }
#else
	static struct gpio_dt_spec  btn1 = GPIO_DT_SPEC_GET(BTN1_NODE, gpios);
    static struct gpio_callback btn1_cb_data;
    static void btn1_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
    {
        LOG_DBG("Button_1 pressed");
        btn1_is_pressed = true;
        k_sem_give(&sdcard_activity_sem);
    }

    bool btn1_has_been_pressed(void) {
        if (btn1_is_pressed) {
            btn1_is_pressed = false;
            return true;
        }

        return false;
    }
#endif

#if !DT_NODE_HAS_STATUS_OKAY(BTN2_NODE)
	#warning "Button2 is NOT okay"
    bool btn2_has_been_pressed(void) { return false; }
#else
	static struct gpio_dt_spec  btn2 = GPIO_DT_SPEC_GET(BTN2_NODE, gpios);
    static struct gpio_callback btn2_cb_data;
    static void btn2_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
    {
        LOG_DBG("Button_2 pressed");
        btn2_is_pressed = true;
        k_sem_give(&sdcard_activity_sem);
    }

    bool btn2_has_been_pressed(void){
        if (btn2_is_pressed) {
            btn2_is_pressed = false;
            return true;
        }

        return false;
    }
#endif

#if !DT_NODE_HAS_STATUS_OKAY(BTN3_NODE)
	#warning "Button3 is NOT okay"
    bool btn3_has_been_pressed(void) { return false; }
#else
	static struct gpio_dt_spec  btn3 = GPIO_DT_SPEC_GET(BTN3_NODE, gpios);
    static struct gpio_callback btn3_cb_data;
    static void btn3_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
    {
        LOG_DBG("Button_3 pressed");
        btn3_is_pressed = true;
        k_sem_give(&sdcard_activity_sem);
    }

    bool btn3_has_been_pressed(void) {
        if (btn3_is_pressed) {
            btn3_is_pressed = false;
            return true;
        }

        return false;
    }
#endif

bool btn_hal_buttons_init(void)
{
    int ret = 0;
    #if DT_NODE_HAS_STATUS_OKAY(BTN0_NODE)
    {
        if (!gpio_is_ready_dt(&btn0)) {
            LOG_ERR("Button0 is NOT ready!");
            return false;
        }

        ret = gpio_pin_configure_dt(&btn0, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure %s pin %d", ret, btn0.port->name, btn0.pin);
            return false;
        }

        ret = gpio_pin_interrupt_configure_dt(&btn0, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, btn0.port->name, btn0.pin);
        	return false;
        }

        gpio_init_callback(&btn0_cb_data, btn0_pressed, BIT(btn0.pin));
        gpio_add_callback(btn0.port, &btn0_cb_data);
        LOG_INF("Set up button at %s pin %d", btn0.port->name, btn0.pin);
    }
    #endif // #if DT_NODE_HAS_STATUS_OKAY(BTN0_NODE)

    #if DT_NODE_HAS_STATUS_OKAY(BTN1_NODE)
    {
        if (!gpio_is_ready_dt(&btn1)) {
            LOG_ERR("Button1 is NOT ready!");
            return false;
        }

        ret = gpio_pin_configure_dt(&btn1, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure %s pin %d", ret, btn1.port->name, btn1.pin);
            return false;
        }

        ret = gpio_pin_interrupt_configure_dt(&btn1, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, btn1.port->name, btn1.pin);
        	return false;
        }

        gpio_init_callback(&btn1_cb_data, btn1_pressed, BIT(btn1.pin));
        gpio_add_callback(btn1.port, &btn1_cb_data);
        LOG_INF("Set up button at %s pin %d", btn1.port->name, btn1.pin);
    }
    #endif // #if DT_NODE_HAS_STATUS_OKAY(BTN1_NODE)

    #if DT_NODE_HAS_STATUS_OKAY(BTN2_NODE)
    {
        if (!gpio_is_ready_dt(&btn2)) {
            LOG_ERR("Button2 is NOT ready!");
            return false;
        }

        ret = gpio_pin_configure_dt(&btn2, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure %s pin %d", ret, btn2.port->name, btn2.pin);
            return false;
        }

        ret = gpio_pin_interrupt_configure_dt(&btn2, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, btn2.port->name, btn2.pin);
        	return false;
        }

        gpio_init_callback(&btn2_cb_data, btn2_pressed, BIT(btn2.pin));
        gpio_add_callback(btn2.port, &btn2_cb_data);
        LOG_INF("Set up button at %s pin %d", btn2.port->name, btn2.pin);
    }
    #endif // #if DT_NODE_HAS_STATUS_OKAY(BTN2_NODE)

    #if DT_NODE_HAS_STATUS_OKAY(BTN3_NODE)
    {
        if (!gpio_is_ready_dt(&btn3)) {
            LOG_ERR("Button3 is NOT ready!");
            return false;
        }

        ret = gpio_pin_configure_dt(&btn3, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure %s pin %d", ret, btn3.port->name, btn3.pin);
            return false;
        }

        ret = gpio_pin_interrupt_configure_dt(&btn3, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Error %d: failed to configure interrupt on %s pin %d", ret, btn3.port->name, btn3.pin);
        	return false;
        }

        gpio_init_callback(&btn3_cb_data, btn3_pressed, BIT(btn3.pin));
        gpio_add_callback(btn3.port, &btn3_cb_data);
        LOG_INF("Set up button at %s pin %d", btn3.port->name, btn3.pin);
    }
    #endif // #if DT_NODE_HAS_STATUS_OKAY(BTN3_NODE)
    return true;
}
