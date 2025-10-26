#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/log.h>

static const px4_mft_s mft = {
    .nmft = 0,
    .mfts = { NULL }
};

const px4_mft_s *board_get_manifest(void)
{
    //PX4_INFO("[boot] board_get_manifest() HACKRCF405V2 minimal");
    return &mft;
}
