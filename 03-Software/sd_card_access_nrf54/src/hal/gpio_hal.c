/*
 * Copyright (c) 2025, HES-SO Valais-Wallis, HEI, Sion
 */

#include "gpio_hal.h"

#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>


#include "../define.h"


LOG_MODULE_REGISTER(gpio_hal, CONFIG_GPIO_HAL_LOG_LEVEL);

// Gpios & Hardware handling
#if !DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
	#warning "GPIO0 Node is NOT okay"
#else
	static const struct device* gpio0_dev = DEVICE_DT_GET(GPIO0_NODE);
#endif

#if !DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
	#warning "GPIO1 Node is NOT okay"
#else
	static const struct device* gpio1_dev = DEVICE_DT_GET(GPIO1_NODE);
#endif

#if !DT_NODE_HAS_STATUS(GPIO2_NODE, okay)
	#warning "GPIO2 Node is NOT okay"
#else
	static const struct device* gpio2_dev = DEVICE_DT_GET(GPIO2_NODE);
#endif


void gpio_hal_connect_i2s_gpio(void)
{
	// /*
	// 	----- pinctrl for PCA10095 board (nRF5340 DK) -----
	// 	psels = <NRF_PSEL(I2S_SCK_M,  1, 2)>,
	// 			<NRF_PSEL(I2S_LRCK_M, 0, 6)>,
	// 			<NRF_PSEL(I2S_SDIN,   0, 5)>,
	// 			<NRF_PSEL(I2S_SDOUT,  0, 25)>;

	// 	----- pinctrl for Monkey PCB -----
	// 	psels = <NRF_PSEL(I2S_SCK_M,  1, 2)>,
	// 			<NRF_PSEL(I2S_LRCK_M, 0, 2)>,
	// 			<NRF_PSEL(I2S_SDIN,   0, 3)>;
	// 			// <NRF_PSEL(I2S_SDOUT,  0, 25)>;
	// 	};
	// */
    // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
    // {
    //     if (gpio0_dev)
    //     {
    //         LOG_DBG("Configure I2S GPIOs for gpio0");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio0_dev,  2, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //             gpio_pin_configure(gpio0_dev,  3, GPIO_INPUT);
    //         }
    //         #else
    //         {
    //             gpio_pin_configure(gpio0_dev,  5, GPIO_INPUT);
    //             gpio_pin_configure(gpio0_dev,  6, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //             gpio_pin_configure(gpio0_dev, 25, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //         }
    //         #endif // #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Configure I2S GPIOs for gpio1");
    // 		gpio_pin_configure(gpio1_dev, 2, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
	//     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}

void gpio_hal_connect_spi_gpio(void)
{
	// /*
	// 	----- pinctrl for PCA10095 board (nRF5340 DK) -----
	// 	psels = <NRF_PSEL(SPIM_SCK,  1, 12)>,
	// 			<NRF_PSEL(SPIM_MISO, 1, 13)>,
	// 			<NRF_PSEL(SPIM_MOSI, 1, 11)>;
	// 	cs-gpios = <&gpio1 10 GPIO_ACTIVE_LOW>;

	// 	----- pinctrl for Monkey PCB -----
	// 	psels = <NRF_PSEL(SPIM_SCK,  1, 06)>,
	// 			<NRF_PSEL(SPIM_MISO, 0, 26)>,
	// 			<NRF_PSEL(SPIM_MOSI, 1, 04)>;
	// 	cs-gpios = <&gpio0 19 GPIO_ACTIVE_LOW>;
	// */
    // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
    // {
    //     if (gpio0_dev)
    //     {
    //         LOG_DBG("Configure SPI GPIOs for gpio0");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio0_dev, 26, GPIO_INPUT);
    //             gpio_pin_configure(gpio0_dev, 19, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //         }
    //         #endif // #ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Configure SPI GPIOs for gpio1");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio1_dev, 6, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    //             gpio_pin_configure(gpio1_dev, 4, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    //         }
    //         #else
    //         {
    //             gpio_pin_configure(gpio1_dev, 10, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //             gpio_pin_configure(gpio1_dev, 11, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //             gpio_pin_configure(gpio1_dev, 12, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_LOW);
    //             gpio_pin_configure(gpio1_dev, 13, GPIO_INPUT);
    //         }
    //         #endif // #ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
	//     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}


// This function sets all unused GPIO to non pulled, non buffered inputs
void gpio_hal_disconnect_all_unused(void)
{
    // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
    // {
    //     if (gpio0_dev)
    //     {
    //         LOG_DBG("Disconnect all GPIOs for gpio0 (Except pin 0 & 1)");
    //         for (uint8_t i = 2; i < 32; i++) {
    //             gpio_pin_configure(gpio0_dev, i, GPIO_DISCONNECTED);
    //         }
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Disconnect all GPIOs for gpio1");
    //         for (uint8_t i = 0; i < 16; i++) {
    //             gpio_pin_configure(gpio1_dev, i, GPIO_DISCONNECTED);
    //         }
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO2_NODE, okay)
    // {
    //     if (gpio2_dev)
    //     {
    //         LOG_DBG("Disconnect all GPIOs for gpio2");
    //         for (uint8_t i = 0; i < 16; i++) {
    //             gpio_pin_configure(gpio2_dev, i, GPIO_DISCONNECTED);
    //         }
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}

void gpio_hal_disconnect_i2s_gpio(void)
{
	// /*
	// 	----- pinctrl for PCA10095 board (nRF5340 DK) -----
	// 	psels = <NRF_PSEL(I2S_SCK_M,  1, 2)>,
	// 			<NRF_PSEL(I2S_LRCK_M, 0, 6)>,
	// 			<NRF_PSEL(I2S_SDIN,   0, 5)>,
	// 			<NRF_PSEL(I2S_SDOUT,  0, 25)>;

	// 	----- pinctrl for Monkey PCB -----
	// 	psels = <NRF_PSEL(I2S_SCK_M,  1, 2)>,
	// 			<NRF_PSEL(I2S_LRCK_M, 0, 2)>,
	// 			<NRF_PSEL(I2S_SDIN,   0, 3)>;
	// 			// <NRF_PSEL(I2S_SDOUT,  0, 25)>;
	// 	};
	// */
    // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
    // {
    //     if (gpio0_dev)
    //     {
    //         LOG_DBG("Disconnect I2S GPIOs for gpio0");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio0_dev,  2, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio0_dev,  3, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio0_dev, 25, GPIO_DISCONNECTED);
    //         }
    //         #else
    //         {
    //             gpio_pin_configure(gpio0_dev,  5, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio0_dev,  6, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio0_dev, 25, GPIO_DISCONNECTED);
    //         }
    //         #endif // #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Disconnect I2S GPIOs for gpio1");
    // 		gpio_pin_configure(gpio1_dev, 2, GPIO_DISCONNECTED);
	//     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}

void gpio_hal_disconnect_low_batt_gpio(void)
{
	// // ----- pinctrl for PCA10095 board (nRF5340 DK) -----
	// // lb: low_batt_0 {
	// // 	gpios = <&gpio1  9 (GPIO_ACTIVE_LOW)>;
	// // 	label = "Low Batt status";
	// // };

	// // ----- pinctrl for Monkey PCB -----
	// // lb: low_batt_0 {
	// // 	gpios = <&gpio1 13 GPIO_ACTIVE_HIGH>;
	// // 	label = "Low Batt status";
	// // };

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Disconnect Low Batt Detection GPIOs for gpio1");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
	// 	    	gpio_pin_configure(gpio1_dev, 13, GPIO_DISCONNECTED);
    //         }
    //         #else
    //         {
    //             gpio_pin_configure(gpio1_dev,  9, GPIO_DISCONNECTED);
    //         }
    //         #endif // #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
	// }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}

void gpio_hal_disconnect_spi_gpio(void)
{
	// /*
	// 	----- pinctrl for PCA10095 board (nRF5340 DK) -----
	// 	psels = <NRF_PSEL(SPIM_SCK,  1, 12)>,
	// 			<NRF_PSEL(SPIM_MISO, 1, 13)>,
	// 			<NRF_PSEL(SPIM_MOSI, 1, 11)>;
	// 	cs-gpios = <&gpio1 10 GPIO_ACTIVE_LOW>;

	// 	----- pinctrl for Monkey PCB -----
	// 	psels = <NRF_PSEL(SPIM_SCK,  1, 06)>,
	// 			<NRF_PSEL(SPIM_MISO, 0, 26)>,
	// 			<NRF_PSEL(SPIM_MOSI, 1, 04)>;
	// 	cs-gpios = <&gpio0 19 GPIO_ACTIVE_LOW>;
	// */
    // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)
    // {
    //     if (gpio0_dev)
    //     {
    //         LOG_DBG("Disconnect SPI GPIOs for gpio0");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio0_dev, 26, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio0_dev, 19, GPIO_DISCONNECTED);
    //         }
    //         #endif // #ifndef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO0_NODE, okay)

    // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
    // {
    //     if (gpio1_dev)
    //     {
    //         LOG_DBG("Disconnect SPI GPIOs for gpio1");
    //         #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //         {
    //             gpio_pin_configure(gpio1_dev, 6, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio1_dev, 4, GPIO_DISCONNECTED);
    //         }
    //         #else
    //         {
    //             gpio_pin_configure(gpio1_dev, 10, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio1_dev, 11, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio1_dev, 12, GPIO_DISCONNECTED);
    //             gpio_pin_configure(gpio1_dev, 13, GPIO_DISCONNECTED);
    //         }
    //         #endif // #ifdef CONFIG_COMPILE_FOR_MONKEY_PCB
    //     }
    // }
    // #endif // #if DT_NODE_HAS_STATUS(GPIO1_NODE, okay)
}
