/****************************************************************************
 * boards/hackrc/f405_v2/src/manifest.c
 *
 *  PX4 board manifest for HAKRC F405 V2
 *
 ****************************************************************************/

#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/log.h>
#include <nuttx/spi/spi.h>

/* ────────────────────────────────────────────────
 * Define MTD entry for QSPI flash (W25Q32JV/W25Q64JV)
 * ──────────────────────────────────────────────── */

static const px4_mft_device_t spi2 = {
    .bus_type = px4_mft_device_t::SPI,
    .devid    = SPIDEV_FLASH(0)
};

static const px4_mtd_entry_t qspi_flash_entry = {
    .device = &spi2,
    .npart = 1,
    .partd = {
        {
            .type     = MTD_PARAMETERS,
            .path     = "/fs/mtd_params",
            .nblocks  = 32   // параметры занимают немного
        }
    },
};

/* ────────────────────────────────────────────────
 * Build manifest structure
 * ──────────────────────────────────────────────── */

static const px4_mtd_manifest_t board_mtd_config = {
    .nconfigs = 1,
    .entries  = {
        &qspi_flash_entry
    }
};

static const px4_mft_entry_s mtd_mft = {
    .type = MTD,
    .pmft = (void *)&board_mtd_config,
};

static const px4_mft_s mft = {
    .nmft = 1,
    .mfts = {
        &mtd_mft
    }
};

const px4_mft_s *board_get_manifest(void)
{
    //PX4_INFO("Returning board manifest (HAKRC F405 V2)");
    return &mft;
}
