#include <nuttx/spi/spi.h>
#include <px4_platform_common/px4_manifest.h>

static const px4_mft_device_t qspi_flash_dev = {
    .bus_type = px4_mft_device_t::SPI,
    .devid    = SPIDEV_FLASH(0)  // CS0 на SPI2
};

static const px4_mtd_entry_t qspi_flash_entry = {
    .device = &qspi_flash_dev,
    .npart = 1,
    .partd = {
        {
            .type     = MTD_PARAMETERS,
            .path     = "/fs/mtd_params",
            .nblocks  = 32  // или другое количество блоков
        }
    },
};
static const px4_mtd_manifest_t board_mtd_config = {
    .nconfigs   = 1,
    .entries    = { &qspi_flash_entry }
};

static const px4_mft_entry_s mtd_mft = {
    .type = MTD,
    .pmft = (void *)&board_mtd_config,
};

static const px4_mft_s mft = {
    .nmft = 1,
    .mfts = { &mtd_mft }
};

// экспортируем манифест
extern "C" const px4_mft_s *board_get_manifest(void)
{
    return &mft;
}
