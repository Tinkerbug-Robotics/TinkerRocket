#pragma once

#include <stdint.h>

#include <RocketComputerTypes.h>   // SensorHealthState
#include <SdrAid.h>                // EpochAid: the board's per-epoch aiding block, kWhy*
#include <SdrRtcm.h>               // TrAlign: the board's mounting report (4001 sub-type 5)

// Whether the gnss-sdr-dev board's own IMU aids its tracking loops, as a
// sensor-health state for the apps' health dots: SH_NA with no board on J1,
// SH_OK while the IMU aids fully (its heading known: a whole mounting into
// this flight computer, which the apps' six-face calibration sets, or one the
// board found itself), SH_DEGRADED while it does not fully aid (the vertical
// only, its heading not known yet; or off in the board's settings), SH_BAD
// when the board refused or dropped its IMU (its at-rest check failed, its
// samples stopped, its attitude lapsed). The flight computer sends it in FC_STATUS_MSG's flags
// (FCS_GNSS_IMU_*); the out computer passes it to the apps ("gi").
//
// From the board's own frames as the flight loop drains them: each epoch's
// aiding block (4001 sub-type 1, 100 a second: what aids the loops now, and
// why the IMU does not), and its mounting report (4001 sub-type 5, once a
// second from firmware that has it: whether its IMU runs). The epochs decide
// while they come; the report alone (before the board's first fix it sends
// no epochs) says full aiding only while it says the IMU aids with a whole
// mounting in use. Either one goes stale after kStaleUs. A board from before
// the aiding block's version 2 never says why, so it shows SH_OK or
// SH_DEGRADED only (and an IMU aiding at all as SH_OK).
//
// Advisory: the state is shown, never part of go/no-go.
//
// Pure: main.cpp's board-frame drain hands it the frames' decodes, its status
// sender asks for the state. Host-tested in tests_cpp/test_sdr_imu_status.cpp.

namespace sdr_imu_status
{

class Tracker
{
public:
    static constexpr int64_t kStaleUs = 2000000;   // 2 s without a frame: the board says nothing now

    void noteAid(const sdr_aid::EpochAid& a, int64_t now_us)
    {
        if (!a.present)
            return;
        aid_ = a;
        aid_us_ = now_us;
        have_aid_ = true;
    }

    void noteAlign(const sdr_rtcm::TrAlign& al, int64_t now_us)
    {
        align_imu_ = al.imu;
        align_whole_ = al.have == sdr_rtcm::TrAlign::kHaveWhole;
        align_us_ = now_us;
        have_align_ = true;
    }

    // board_on_j1: the flight computer found the gnss-sdr-dev board on J1.
    SensorHealthState state(bool board_on_j1, int64_t now_us) const
    {
        if (!board_on_j1)
            return SH_NA;
        if (have_aid_ && now_us - aid_us_ < kStaleUs)
        {
            if (aid_.src == 2)                       // aids: fully, or the vertical only
                return (aid_.ver >= 2 && aid_.why == sdr_aid::kWhyVertical) ? SH_DEGRADED : SH_OK;
            if (aid_.check == 2)
                return SH_BAD;   // the board's at-rest check refused its IMU
            if (aid_.ver >= 2 && aid_.why >= sdr_aid::kWhyStale && aid_.why <= sdr_aid::kWhyPadCheck)
                return SH_BAD;   // given to the loops, then dropped (or refused)
            return SH_DEGRADED;  // off, let go, or (an older board) no mounting yet
        }
        if (have_align_ && now_us - align_us_ < kStaleUs && align_imu_ == 2)
            return align_whole_ ? SH_OK : SH_DEGRADED;   // a whole mounting: its heading known
        return SH_DEGRADED;      // the board is there, and says nothing of its IMU aiding
    }

private:
    sdr_aid::EpochAid aid_{};
    int64_t aid_us_ = 0;
    bool    have_aid_ = false;
    uint8_t align_imu_ = 0;
    bool    align_whole_ = false;
    int64_t align_us_ = 0;
    bool    have_align_ = false;
};

}  // namespace sdr_imu_status
