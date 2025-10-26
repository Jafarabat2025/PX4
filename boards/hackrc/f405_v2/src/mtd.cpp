/****************************************************************************
 * boards/hackrc/f405_v2/src/mtd.c
 *
 *  MTD configuration for HACKRC F405 V2 (SPI‑Flash W25Q32/W25Q64).
 *
 *  Создаёт единственный раздел во флеш‑памяти, доступный как /fs/mtd_params.
 *  Этот раздел используется для хранения параметров PX4 (FLASH_BASED_PARAMS).
 *
 *  Для работы этого файла нужно, чтобы драйвер флеш‑памяти (например m25p)
 *  был включён в конфигурации Nuttx (CONFIG_MTD_M25P) и чтобы spi/shash были
 *  корректно инициализированы в функции px4_platform_configure().
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>      // для PX4_INFO / PX4_ERR макросов
#include <drivers/device/spi.h> 

/* Определяем SPI‑устройство: шина SPI и device id для SPI‑флеш */
static const px4_mft_device_t spi_flash_dev = {
    .bus_type = px4_mft_device_t::SPI,  /* Мы используем обычный SPI (не QSPI) */
    .devid    = SPIDEV_FLASH(0)        /* CS=0 соответствует пину PB12 */
};

/* Определяем таблицу разделов во флеш‑памяти */
static const px4_mtd_entry_t qspi_flash_entry = {
    .device  = &spi_flash_dev,
    .npart   = 1,
    .partd   = {
        {
            .type    = MTD_PARAMETERS,     /* Этот раздел предназначен для хранения параметров */
            .path    = "/fs/mtd_params",   /* Точка монтирования */
            .nblocks = 32                  /* Количество блоков (для W25Q32JV блок = 4 КБ) */
        }
    },
};

/* MTD‑манифест – массив конфигураций */
static const px4_mtd_manifest_t board_mtd_config = {
    .nconfigs = 1,
    .entries  = {
        &qspi_flash_entry
    }
};

/* Описываем запись манифеста: тип MTD + указатель на наш конфиг */
static const px4_mft_entry_s mtd_mft = {
    .type = MTD,
    .pmft = (void *)&board_mtd_config
};

/* Общий объект манифеста. nmft = 1 означает, что есть одна запись (MTD). */
static const px4_mft_s mft = {
    .nmft = 1,
    .mfts = {
        &mtd_mft
    }
};

/* Функция, которую вызывает PX4 для получения манифеста */
const px4_mft_s *board_get_manifest(void)
{
    return &mft;
}
