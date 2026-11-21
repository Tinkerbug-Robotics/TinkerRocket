#pragma once
// raw_nav_auto.h — which navigation filter the one image flies this boot
//
// The image that flies the raw-GNSS filter on every J1 receiver giving
// measurements (config.h TR_RAW_NAV_AUTO, -DTR_GNSS_AUTO_RAW=1;
// docs/plans/gnss-one-image-plan.md) compiles in each raw receiver's paths of
// main.cpp -- TR_RAW_NAV_MOSAIC, TR_RAW_NAV_SDR and TR_RAW_NAV_PX1105R, and
// TR_RAW_NAV_P4 for what they share -- which in their own images are the
// image's alone. Here each runs only when its receiver is the one this boot
// flies on: RAW_NAV_WHEN(cond) { ... } is `if (cond)` in this image and a
// plain block in every other, so those images compile the code they always
// did. With the u-blox, or no receiver and none remembered that gives
// measurements, none of them runs and the image is the auto image: GpsInsEKF
// on the receiver's fix.
//
// The choice (rawNavChoice, pure) is made once in setup_fc, after the
// sensors' bring-up found the receiver and before either filter starts, and
// holds for the boot. `ekf` holds both filters (NavFilterAuto) and hands each
// call to the one chosen.
#include <cstdint>

#include <RocketComputerTypes.h>   // GNSS_TYPE_*

// And RAW_NAV_AND(cond) x, for a condition: `(cond) && x` here, `x` in every
// other image; kRawNavAuto the same switch as a value.
#if TR_RAW_NAV_AUTO
#define RAW_NAV_WHEN(cond) if (cond)
#define RAW_NAV_AND(cond) (cond) &&
constexpr bool kRawNavAuto = true;
#else
#define RAW_NAV_WHEN(cond)
#define RAW_NAV_AND(cond)
constexpr bool kRawNavAuto = false;
#endif

// The receiver the raw-GNSS filter flies on, from what the bring-up found and
// what the board had last boot (TR_GNSSAuto's NVS hint): the receiver found
// when it gives measurements; with none found, the one remembered when it
// does -- the board sends nothing until its first fix and the PX1105R's and
// the mosaic's drivers go on looking, so a boot that missed them still flies
// their filter, degraded until they answer, as their own images do;
// GNSS_TYPE_NONE otherwise (the u-blox, or nothing found and nothing of
// theirs remembered): GpsInsEKF.
constexpr bool gnssTypeGivesRaw(uint8_t type)
{
    return type == GNSS_TYPE_MOSAIC || type == GNSS_TYPE_SDR || type == GNSS_TYPE_PX1105R;
}

constexpr uint8_t rawNavChoice(uint8_t found, uint8_t remembered)
{
    return gnssTypeGivesRaw(found)                                     ? found
         : (found == GNSS_TYPE_NONE && gnssTypeGivesRaw(remembered))   ? remembered
                                                                       : GNSS_TYPE_NONE;
}

#if TR_RAW_NAV_AUTO
#include <utility>
#include <TR_GpsInsEKF.h>
#include "raw_nav_adapter.h"

namespace raw_nav_auto
{
// This boot's choice: set once by setup_fc (rawNavChoice), read everywhere.
inline uint8_t rx = GNSS_TYPE_NONE;
}  // namespace raw_nav_auto

#define RAW_ON        (raw_nav_auto::rx != GNSS_TYPE_NONE)
#define RAW_IS_MOSAIC (raw_nav_auto::rx == GNSS_TYPE_MOSAIC)
#define RAW_IS_SDR    (raw_nav_auto::rx == GNSS_TYPE_SDR)
#define RAW_IS_PX     (raw_nav_auto::rx == GNSS_TYPE_PX1105R)

// `ekf` in the one image: both filters, each call handed to the one this
// boot flies (useRaw, once, before init). RawNavAdapter is GpsInsEKF's face
// on the raw-GNSS filter's state, so the calls they share have the same
// shape; the raw filter's own calls (attach, refresh, the restore's) reach
// it alone, and main.cpp makes them only where it flies.
class NavFilterAuto
{
public:
    void useRaw(bool on) { raw_on_ = on; }
    bool usesRaw() const { return raw_on_; }

#define TR_NAV_BOTH(m)                                                                                   \
    template <typename... A>                                                                             \
    auto m(A&&... a)->decltype(std::declval<GpsInsEKF&>().m(std::forward<A>(a)...))                      \
    {                                                                                                    \
        if (raw_on_) return raw_.m(std::forward<A>(a)...);                                              \
        return gins_.m(std::forward<A>(a)...);                                                           \
    }                                                                                                    \
    template <typename... A>                                                                             \
    auto m(A&&... a) const->decltype(std::declval<const GpsInsEKF&>().m(std::forward<A>(a)...))          \
    {                                                                                                    \
        if (raw_on_) return raw_.m(std::forward<A>(a)...);                                              \
        return gins_.m(std::forward<A>(a)...);                                                           \
    }
#define TR_NAV_RAW(m)                                                                                    \
    template <typename... A>                                                                             \
    auto m(A&&... a)->decltype(std::declval<RawNavAdapter&>().m(std::forward<A>(a)...))                  \
    {                                                                                                    \
        return raw_.m(std::forward<A>(a)...);                                                            \
    }                                                                                                    \
    template <typename... A>                                                                             \
    auto m(A&&... a) const->decltype(std::declval<const RawNavAdapter&>().m(std::forward<A>(a)...))      \
    {                                                                                                    \
        return raw_.m(std::forward<A>(a)...);                                                            \
    }

    TR_NAV_BOTH(init)
    TR_NAV_BOTH(update)
    TR_NAV_BOTH(baroMeasUpdate)
    TR_NAV_BOTH(landedZeroVelocityUpdate)
    TR_NAV_BOTH(isHealthy)
    TR_NAV_BOTH(getState)
    TR_NAV_BOTH(setState)
    TR_NAV_BOTH(getPosEst)
    TR_NAV_BOTH(getVelEst)
    TR_NAV_BOTH(getOrientEst)
    TR_NAV_BOTH(getQuaternion)
    TR_NAV_BOTH(setQuaternion)
    TR_NAV_BOTH(getRotRateBias)
    TR_NAV_BOTH(getCovDiag)
    TR_NAV_BOTH(getCovOrient)
    TR_NAV_BOTH(getCovVel)
    TR_NAV_BOTH(setCovFromDiag)
    TR_NAV_BOTH(inflateCovDiag)
    TR_NAV_BOTH(setAttitudeCovariance)
    TR_NAV_BOTH(getMagCalSuspect)
    TR_NAV_BOTH(getMagReferenceTotal_uT)
    TR_NAV_BOTH(getMagReferenceValid)
    TR_NAV_BOTH(setMagReference)
    TR_NAV_BOTH(setDeclination)
    TR_NAV_BOTH(gyroBiasClipCount)
    TR_NAV_BOTH(gyroBiasGateTrips)
    TR_NAV_BOTH(gyroBiasHealthy)
    TR_NAV_BOTH(gyroBiasSigmaMaxDps)
    TR_NAV_BOTH(setGnssVerticalHeldOut)
    TR_NAV_BOTH(setGpsNoiseScale)
    TR_NAV_BOTH(setNoseFirstFlight)
    TR_NAV_BOTH(setShockGateSettle)
    TR_NAV_BOTH(shockGateHeld)
    TR_NAV_BOTH(shockGateHoldTicks)
    TR_NAV_BOTH(shockGateTrips)
    TR_NAV_BOTH(underCanopy)

    TR_NAV_RAW(attach)
    TR_NAV_RAW(refresh)
    TR_NAV_RAW(running)
    TR_NAV_RAW(hasRun)
    TR_NAV_RAW(gnssAided)
    TR_NAV_RAW(noteImu)
    TR_NAV_RAW(forwardShort)
    TR_NAV_RAW(geoidSeparation)
    // by value: main.cpp hands it a packed snapshot field, which a forwarding
    // reference cannot bind
    void setGeoidSeparation(float sep_m) { raw_.setGeoidSeparation(sep_m); }
    TR_NAV_RAW(rebaseRestoreHeight)
    TR_NAV_RAW(restartFromLastGood)
    TR_NAV_RAW(restorePending)
    TR_NAV_RAW(setHoldLastGood)
    TR_NAV_RAW(setMagCalibration)
    TR_NAV_RAW(startSeed)
    TR_NAV_RAW(stateTimeUs)
    TR_NAV_RAW(stoppedInFlight)
    TR_NAV_RAW(takeMaxAgeUs)
    TR_NAV_RAW(takeMaxForwardUs)
#undef TR_NAV_BOTH
#undef TR_NAV_RAW

private:
    bool          raw_on_ = false;
    RawNavAdapter raw_;
    GpsInsEKF     gins_;
};
#endif
