/****************************************************************************
 * Board Manifest for HAKRC F405 V2
 *
 * Определяет дополнительные устройства платы:
 *  - SPI-Flash (W25Q32/W25Q64) → /fs/mtd_params
 *  - IMU ICM42688P (SPI1)
 *  - OSD/MAX7456 и Baro DPS310 (SPI3)
 *
 * PX4 автоматически создаст шины и MTD-устройство при старте.
 ****************************************************************************/

#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

/*─────────────────────────────────────────────
 * SPI-Flash (MTD Parameters)
 *────────────────────────────────────────────*/
static const px4_mft_device_t spi_flash = {
	.bus_type = BUS_TYPE_SPI,       //  корректный C-enum
	.devid    = SPIDEV_FLASH(0)
};

static const px4_mtd_entry_t mtd_flash = {
	.device = &spi_flash,
	.npart  = 1,
	.partd  = {{
		.type    = MTD_PARAMETERS,       // PX4 хранит параметры тут
		.path    = "/fs/mtd_params",      // стандартный путь PX4
		.nblocks = 32                     // размер (примерно 32 KiB)
	}},
};

static const px4_mtd_manifest_t mtd_manifest = {
	.nconfigs = 1,
	.entries  = { &mtd_flash },
};

/*─────────────────────────────────────────────
 * Общая таблица устройств (manifest)
 *────────────────────────────────────────────*/
static const px4_mft_entry_s mtd_mft = {
	.type = MTD,
	.pmft = (void *)&mtd_manifest,
};

static const px4_mft_s board_manifest = {
	.nmft = 1,
	.mfts = { &mtd_mft },
};

/*─────────────────────────────────────────────
 * Экспорт функции, которую вызывает PX4
 *────────────────────────────────────────────*/
const px4_mft_s *board_get_manifest(void)
{
	return &board_manifest;
}
