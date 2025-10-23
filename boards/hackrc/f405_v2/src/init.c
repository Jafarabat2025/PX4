/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file init.c
 *
 * omnibusf4sd-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/*
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	/* configure the GPIO pins to outputs and keep them low */
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	/* On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 */

	if (status >= 0) {
		up_mdelay(400);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	/*───────────────────────────────
	 * Reset all PWM outputs to LOW
	 *───────────────────────────────*/
	board_on_reset(-1);

	/*───────────────────────────────
	 * LEDs
	 *───────────────────────────────*/
	board_autoled_initialize();

	/*───────────────────────────────
	 * ADC inputs
	 *───────────────────────────────*/
	stm32_configgpio(GPIO_ADC1_IN11);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN12);	/* BATT_CURRENT_SENS */

	/*───────────────────────────────
	 * Power rails
	 *───────────────────────────────*/
	// включаем 3.3В для сенсоров, если линия определена
// #ifdef GPIO_VDD_3V3_SENSORS_EN
// 	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
// 	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, true);
// 	up_mdelay(10);
// #endif

	/*───────────────────────────────
	 * SPI1 — IMU (ICM42688P)
	 *───────────────────────────────*/
	stm32_configgpio(GPIO_SPI1_SCK);    // PA5
	stm32_configgpio(GPIO_SPI1_MISO);   // PA6
	stm32_configgpio(GPIO_SPI1_MOSI);   // PA7
	stm32_configgpio(GPIO_SPI1_NSS);    // PA4
	stm32_gpiowrite(GPIO_SPI1_NSS, true);

	/*───────────────────────────────
	 * SPI2 — Flash (W25Qxx)
	 *───────────────────────────────*/
	stm32_configgpio(GPIO_SPI2_SCK);    // PB13
	stm32_configgpio(GPIO_SPI2_MISO);   // PB14
	stm32_configgpio(GPIO_SPI2_MOSI);   // PC3
	stm32_configgpio(GPIO_SPI2_NSS);    // PB12
	stm32_gpiowrite(GPIO_SPI2_NSS, true);

	/*───────────────────────────────
	 * SPI3 — OSD (AT7456E) / DPS310 (если SPI)
	 *───────────────────────────────*/
	stm32_configgpio(GPIO_SPI3_SCK);    // PC10
	stm32_configgpio(GPIO_SPI3_MISO);   // PC11
	stm32_configgpio(GPIO_SPI3_MOSI);   // PB5
	stm32_configgpio(GPIO_SPI3_NSS);    // PA15
	stm32_gpiowrite(GPIO_SPI3_NSS, true);

	/*───────────────────────────────
	 * I2C1 — Barometer (DPS310)
	 *───────────────────────────────*/
	stm32_configgpio(GPIO_I2C1_SCL);    // PB8
	stm32_configgpio(GPIO_I2C1_SDA);    // PB9

	/*───────────────────────────────
	 * Initialize SPI peripheral drivers
	 *───────────────────────────────*/
	stm32_spiinitialize();
}


/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/
#define CONFIG_SPI_CHARDEV
extern int spi_register(int bus, FAR struct spi_dev_s *dev);
static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;

__EXPORT int board_app_initialize(uintptr_t arg)
{
    syslog(LOG_INFO, "[boot] board_app_initialize() starting...\n");

    px4_platform_init();

    /* DMA allocator */
    if (board_dma_alloc_init() < 0) {
        syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
    }

#if defined(SERIAL_HAVE_RXDMA)
    static struct hrt_call serial_dma_call;
    hrt_call_every(&serial_dma_call, 1000, 1000,
                   (hrt_callout)stm32_serial_dma_poll, NULL);
#endif

    /* LED setup */
    drv_led_start();
    led_off(LED_BLUE);

    if (board_hardfault_init(2, true) != 0) {
        led_on(LED_BLUE);
    }

    /* Enable 3V3 sensors power if defined */
#ifdef GPIO_VDD_3V3_SENSORS_EN
    stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
    stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, true);
    up_mdelay(10);
#endif

    /*─────────────────────────────────────────────
     * SPI1 — IMU (ICM42688)
     *────────────────────────────────────────────*/
    spi1 = stm32_spibus_initialize(1);
    if (!spi1) {
        syslog(LOG_ERR, "[boot] FAILED to initialize SPI1 (IMU)\n");
    } else {
        SPI_SETFREQUENCY(spi1, 8 * 1000 * 1000);
        SPI_SETBITS(spi1, 8);
        SPI_SETMODE(spi1, SPIDEV_MODE3);
#ifdef CONFIG_SPI_CHARDEV
        spi_register(1, spi1);
#endif
        syslog(LOG_INFO, "[boot] SPI1 initialized\n");
    }

    /*─────────────────────────────────────────────
     * SPI2 — External Flash (W25Q32 / GD25Q64)
     *────────────────────────────────────────────*/
    spi2 = stm32_spibus_initialize(2);
    if (!spi2) {
        syslog(LOG_ERR, "[boot] FAILED to initialize SPI2 (Flash)\n");
    } else {
        SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
        SPI_SETBITS(spi2, 8);
        SPI_SETMODE(spi2, SPIDEV_MODE0);
        SPI_SELECT(spi2, SPIDEV_FLASH(0), false);
#ifdef CONFIG_SPI_CHARDEV
        spi_register(2, spi2);
#endif
        syslog(LOG_INFO, "[boot] SPI2 initialized\n");
    }

    /*─────────────────────────────────────────────
     * SPI3 — OSD (MAX7456) / Barometer
     *────────────────────────────────────────────*/
    spi3 = stm32_spibus_initialize(3);
    if (!spi3) {
        syslog(LOG_ERR, "[boot] FAILED to initialize SPI3 (OSD/Baro)\n");
    } else {
        SPI_SETFREQUENCY(spi3, 10 * 1000 * 1000);
        SPI_SETBITS(spi3, 8);
        SPI_SETMODE(spi3, SPIDEV_MODE0);
#ifdef CONFIG_SPI_CHARDEV
        spi_register(3, spi3);
#endif
        syslog(LOG_INFO, "[boot] SPI3 initialized\n");
    }

#if defined(FLASH_BASED_PARAMS)
    static sector_descriptor_t params_sector_map[] = {
        {1, 16 * 1024, 0x08008000},
        {0, 0, 0},
    };

    int result = parameter_flashfs_init(params_sector_map, NULL, 0);
    if (result != OK) {
        syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
        led_on(LED_AMBER);
    }
#endif

    /* Configure the rest of the platform (UARTs, I2C, etc.) */
    px4_platform_configure();

    syslog(LOG_INFO, "[boot] board_app_initialize() done\n");
    return OK;
}

