// Navigation frames from the PX1105R's 20-minute static capture outdoors,
// 2026-09-26 (bench-backups/px1105r_outdoor_20hz_1200s_2026-09-26.log.gz, not
// in the repository), with the bench prototype's decode of each as the
// expected value (gnss/sim/tinkerrocket_gnss/gnss_raw.py EphStore, the decoder
// the receiver rig has used on real data since 2026-09).
//
// For each satellite below: exactly the bodies (from the message ID on, as
// SkyTraq frames them, checksum stripped) that the prototype held when it
// completed that ephemeris, in the order they arrived; the last completes it.
// GPS: 0xE0 subframes 1-3; Galileo: 0xE6 word types 1-5; BeiDou: 0xE2
// subframes 1-3. And one GPS subframe 4 page 18 (the ionosphere).
//
// The prototype's ephemeris dict -> gps_eph_t (made by
// a one-off script from its dump, kept outside the repository):
//   sqrt_a     sqrt(A)          (it keeps A = sqrtA^2; py_A below is A itself)
//   m0, delta_n, omega0, omega, omega_dot, i0, idot
//              M0, deln, OMG0, omg, OMGd, i0, idot (already rad, rad/s:
//              semicircles * 3.1415926535898, as GPS_PI_ICD)
//   e, cuc, cus, crc, crs, cic, cis   the same names
//   af0, af1, af2                     f0, f1, f2
//   tgd        tgd_use          (GPS T_GD, Galileo BGD(E1,E5b), BeiDou TGD1)
//   toe, toc   toe, toc for GPS and Galileo; for BeiDou the prototype's
//              value LESS 14 s: it moves BDT to GPS time (+14 s), gps_eph_t
//              keeps BeiDou's own BDT
//   iode       iode             (GPS IODE, Galileo IODnav, BeiDou AODE)
//   iodc       GPS iodc (10-bit IODC); Galileo = iode (IODnav), as
//              RtcmEphemeris; BeiDou AODC, which the prototype does not
//              decode: read by the script from subframe 1 bits 43-47
//   health     svh              (GPS 6-bit SV health; Galileo (E1-B HS << 1) |
//              E1-B DVS; BeiDou SatH1)
//   ura        sva              (GPS URA index, Galileo SISA, BeiDou URAI)
//   week       not in the prototype (it works in seconds of week): the
//              script reads it from the frames: GPS the 10-bit week made
//              whole near 2437 (gnss_raw::fullGpsWeek), Galileo word 5's GST
//              week + 1024, BeiDou subframe 1's BDT week + 1356; no toe on
//              09-26 is near a week's turn, so each is the transmit week
#pragma once

#include "gnss/eph.h"
#include "gnss/types.h"

namespace skytraq_nav_fixture {

// The receiver's GPS week on 09-26 (0xE5 [3..4]).
constexpr int kWeek = 2437;

struct Golden
{
    const char* label;
    int         n_frames;
    const char* frames[5];   // hex bodies, in arrival order
    double      py_A;        // the prototype's A (m), = sqrt_a^2
    gps_eph_t (*expect)();
};

inline gps_eph_t G32_iod120()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GPS;
    e.prn = 32;
    e.week = 2437;
    e.iodc = 120;
    e.iode = 120;
    e.health = 0;
    e.ura = 0;
    e.toe = 576000.0;
    e.toc = 576000.0;
    e.sqrt_a = 5153.595571517944;
    e.e = 0.009590421570464969;
    e.i0 = 0.9698162628886741;
    e.omega0 = 1.511485120610807;
    e.omega = -1.9744255541094913;
    e.m0 = 3.0938391000283305;
    e.delta_n = 4.8887750656760076e-09;
    e.idot = 9.964700784070764e-11;
    e.omega_dot = -8.27927343639944e-09;
    e.cuc = 9.387731552124023e-07;
    e.cus = 2.987682819366455e-06;
    e.crc = 324.875;
    e.crs = 17.78125;
    e.cic = 1.2852251529693604e-07;
    e.cis = -1.862645149230957e-07;
    e.af0 = 0.0002865782007575035;
    e.af1 = 1.807620719773695e-11;
    e.af2 = 0.0;
    e.tgd = 9.313225746154785e-10;
    return e;
}

inline gps_eph_t G23_iod144()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GPS;
    e.prn = 23;
    e.week = 2437;
    e.iodc = 400;
    e.iode = 144;
    e.health = 0;
    e.ura = 1;
    e.toe = 576000.0;
    e.toc = 576000.0;
    e.sqrt_a = 5153.599382400513;
    e.e = 0.00674423249438405;
    e.i0 = 0.9903101666905995;
    e.omega0 = 0.456837554740716;
    e.omega = -2.7169210964291377;
    e.m0 = -0.8239968107289;
    e.delta_n = 3.7072972809553594e-09;
    e.idot = 4.803771524937339e-10;
    e.omega_dot = -7.63067499109935e-09;
    e.cuc = 2.332031726837158e-06;
    e.cus = 1.1596828699111938e-05;
    e.crc = 170.875;
    e.crs = 43.28125;
    e.cic = 1.1362135410308838e-07;
    e.cis = 2.60770320892334e-08;
    e.af0 = 0.0006926604546606541;
    e.af1 = 3.524291969370097e-12;
    e.af2 = 0.0;
    e.tgd = -8.381903171539307e-09;
    return e;
}

inline gps_eph_t G10_iod45()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GPS;
    e.prn = 10;
    e.week = 2437;
    e.iodc = 45;
    e.iode = 45;
    e.health = 0;
    e.ura = 0;
    e.toe = 576000.0;
    e.toc = 576000.0;
    e.sqrt_a = 5153.650625228882;
    e.e = 0.011450332938693464;
    e.i0 = 0.9953987253466353;
    e.omega0 = 0.48991186791575986;
    e.omega = -2.2621957788830076;
    e.m0 = -1.7889846847015092;
    e.delta_n = 3.600864276164926e-09;
    e.idot = 5.810956335370298e-10;
    e.omega_dot = -7.411022984568758e-09;
    e.cuc = 2.9224902391433716e-06;
    e.cus = 1.1377036571502686e-05;
    e.crc = 174.25;
    e.crs = 58.75;
    e.cic = 1.6763806343078613e-07;
    e.cis = -7.450580596923828e-08;
    e.af0 = -0.0005246270447969437;
    e.af1 = 8.185452315956354e-12;
    e.af2 = 0.0;
    e.tgd = 2.3283064365386963e-09;
    return e;
}

inline gps_eph_t E11_iod58()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GAL;
    e.prn = 11;
    e.week = 2437;
    e.iodc = 58;
    e.iode = 58;
    e.health = 0;
    e.ura = 107;
    e.toe = 568200.0;
    e.toc = 568200.0;
    e.sqrt_a = 5440.614072799683;
    e.e = 0.00015618803445249796;
    e.i0 = 0.9942386254581042;
    e.omega0 = -0.34551844781880237;
    e.omega = 1.3678824750299252;
    e.m0 = 1.2945785144714468;
    e.delta_n = 2.328311269224276e-09;
    e.idot = -3.0179828539569156e-10;
    e.omega_dot = -5.218074496604797e-09;
    e.cuc = -4.803761839866638e-06;
    e.cus = 1.2366101145744324e-05;
    e.crc = 92.09375;
    e.crs = -104.5;
    e.cic = -7.450580596923828e-09;
    e.cis = 3.3527612686157227e-08;
    e.af0 = 0.0058060974115505815;
    e.af1 = 3.2599700716673397e-11;
    e.af2 = 0.0;
    e.tgd = -1.7229467630386353e-08;
    return e;
}

inline gps_eph_t E04_iod54()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GAL;
    e.prn = 4;
    e.week = 2437;
    e.iodc = 54;
    e.iode = 54;
    e.health = 0;
    e.ura = 107;
    e.toe = 565800.0;
    e.toc = 565800.0;
    e.sqrt_a = 5440.60431098938;
    e.e = 0.00018794811330735683;
    e.i0 = 0.9692586716627614;
    e.omega0 = 1.7358222036650703;
    e.omega = 0.6556976078429908;
    e.m0 = 0.44936131189662115;
    e.delta_n = 3.76372820295834e-09;
    e.idot = -1.446488823494143e-10;
    e.omega_dot = -6.173114277845128e-09;
    e.cuc = -1.2423843145370483e-06;
    e.cus = 1.3243407011032104e-06;
    e.crc = 315.15625;
    e.crs = -30.0;
    e.cic = 1.1548399925231934e-07;
    e.cis = 5.4016709327697754e-08;
    e.af0 = 0.00018565950449556112;
    e.af1 = 7.126743639673805e-11;
    e.af2 = 3.469446951953614e-18;
    e.tgd = -6.05359673500061e-09;
    return e;
}

inline gps_eph_t E10_iod60()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_GAL;
    e.prn = 10;
    e.week = 2437;
    e.iodc = 60;
    e.iode = 60;
    e.health = 0;
    e.ura = 107;
    e.toe = 569400.0;
    e.toc = 569400.0;
    e.sqrt_a = 5440.605415344238;
    e.e = 0.00025696761440485716;
    e.i0 = 0.9971535965240638;
    e.omega0 = -0.3468547181541112;
    e.omega = -2.980076887729843;
    e.m0 = -0.060428046122102175;
    e.delta_n = 2.33616873937659e-09;
    e.idot = -2.9858386578792683e-10;
    e.omega_dot = -5.209502710984091e-09;
    e.cuc = -5.146488547325134e-06;
    e.cus = 1.2634322047233582e-05;
    e.crc = 89.21875;
    e.crs = -110.71875;
    e.cic = -2.421438694000244e-08;
    e.cis = 2.9802322387695312e-08;
    e.af0 = -0.000822921865619719;
    e.af1 = -2.7284841053187847e-12;
    e.af2 = 0.0;
    e.tgd = -1.6298145055770874e-09;
    return e;
}

inline gps_eph_t C41_iod1()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_BDS;
    e.prn = 41;
    e.week = 2437;
    e.iodc = 1;
    e.iode = 1;
    e.health = 0;
    e.ura = 0;
    e.toe = 568800.0;   // the prototype's 568814.0 less 14 s
    e.toc = 568800.0;
    e.sqrt_a = 5282.627731323242;
    e.e = 0.0013826106442138553;
    e.i0 = 0.9884479144003167;
    e.omega0 = 1.0566059655252975;
    e.omega = -1.1337828437034831;
    e.m0 = 3.136423846562526;
    e.delta_n = 3.7365842151594384e-09;
    e.idot = 5.725238479163238e-10;
    e.omega_dot = -6.849213868678317e-09;
    e.cuc = 4.135537892580032e-06;
    e.cus = 5.919951945543289e-06;
    e.crc = 255.125;
    e.crs = 81.578125;
    e.cic = -2.0023435354232788e-08;
    e.cis = 9.452924132347107e-08;
    e.af0 = -0.0009654770838096738;
    e.af1 = -3.540279180924699e-12;
    e.af2 = 0.0;
    e.tgd = -2.4e-08;
    return e;
}

inline gps_eph_t C23_iod1()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_BDS;
    e.prn = 23;
    e.week = 2437;
    e.iodc = 1;
    e.iode = 1;
    e.health = 0;
    e.ura = 0;
    e.toe = 568800.0;   // the prototype's 568814.0 less 14 s
    e.toc = 568800.0;
    e.sqrt_a = 5282.6603717803955;
    e.e = 0.0002712511923164129;
    e.i0 = 0.9445029495618543;
    e.omega0 = 3.1411770941543673;
    e.omega = -0.05125028764876594;
    e.m0 = -0.0485508029529643;
    e.delta_n = 3.8355169075317535e-09;
    e.idot = -2.0215127755498393e-10;
    e.omega_dot = -6.830284508765924e-09;
    e.cuc = -6.146728992462158e-08;
    e.cus = 1.319916918873787e-05;
    e.crc = 85.5;
    e.crs = 1.828125;
    e.cic = -5.9138983488082886e-08;
    e.cis = -4.6100467443466187e-08;
    e.af0 = -0.0004601740511134267;
    e.af1 = -2.077449323678593e-12;
    e.af2 = 0.0;
    e.tgd = 2.2000000000000002e-08;
    return e;
}

inline gps_eph_t C29_iod1()
{
    gps_eph_t e{};
    e.valid = 1;
    e.sys = GNSS_SYS_BDS;
    e.prn = 29;
    e.week = 2437;
    e.iodc = 1;
    e.iode = 1;
    e.health = 0;
    e.ura = 0;
    e.toe = 568800.0;   // the prototype's 568814.0 less 14 s
    e.toc = 568800.0;
    e.sqrt_a = 5282.651082992554;
    e.e = 0.00010872853454202414;
    e.i0 = 0.943841644758712;
    e.omega0 = -1.0903226211593402;
    e.omega = 2.580900431948202;
    e.m0 = -0.8338196268371902;
    e.delta_n = 4.51447376023851e-09;
    e.idot = -3.821587755898106e-10;
    e.omega_dot = -7.468882537508523e-09;
    e.cuc = -4.866160452365875e-06;
    e.cus = 3.289896994829178e-06;
    e.crc = 283.296875;
    e.crs = -95.96875;
    e.cic = 6.984919309616089e-09;
    e.cis = 3.67872416973114e-08;
    e.af0 = 2.6070047169923782e-06;
    e.af1 = 5.148770299001626e-12;
    e.af2 = 0.0;
    e.tgd = 2e-10;
    return e;
}

const Golden kGoldens[] = {
    {"G32 IOD 120", 3,   // capture lines 597, 878, 1159
     {
         "e020018b0614b9532561500065d2f9e8a2e5a72dd441a902788ca000009f258ff9",
         "e020028b0614b953a878023935787e0de99101f804e909260644a10cc3bb8ca07e",
         "e020038b0614b9542f00453d955c97ff9c27838cf1289caf8dfe94ffa57378045e",
     },
     26559547.31476937, G32_iod120},
    {"G23 IOD 144", 3,   // capture lines 609, 890, 1170
     {
         "e017018b0614b953256151016dd2f9e8a2e5a72dd441a9ee908ca000001f5ac9d5",
         "e017028b0614b953a8900569288cde6d66c904e40373fae41852a10ccb898ca07f",
         "e017038b0614b9542f003d129cfd11000e28594f4a155c914d7c2affac8b901504",
     },
     26559586.594278947, G23_iod144},
    {"G10 IOD 45", 3,   // capture lines 2006, 2288, 2570
     {
         "e00a018b0614b955a461500065d2f9e8a2e5a72dd441a9052d8ca0000048bb3c73",
         "e00a028b0614b9562b2d07582762b71c367f062105dcd16b17dca10d347b8ca05b",
         "e00a038b0614b956ad005a13f5f755ffd8288e62a415c8a3d4712fffaef22d196c",
     },
     26560114.766922045, G10_iod45},
    {"E11 IOD 58", 5,   // capture lines 459, 555, 1389, 1490, 1581
     {
         "e601030b08020ebc7b0728ca20923e0deee2084000bcb340000000002aaaaa7c3f803f4000",
         "e601030b08040e8bfffc001293f82f9044d0020000bd8040000000002aaaaa4891743f4000",
         "e601030b08010ea4fe34bef197001478cdaa04c000a99fc0000000002aaaaa7402e33f4000",
         "e601030b08030ebff1bb865dfd7b467bc2e0fc8000bc1ac0000000002aaaaa6eb3f1ff4000",
         "e601030b08053cc04c0300779db00b0b15feea8000aaaa80000000002aaaaa64ea4a3f4000",
     },
     29600281.48914595, E11_iod58},
    {"E04 IOD 54", 5,   // capture lines 463, 547, 1393, 1494, 1584
     {
         "e601030408020d91ae5211c9df6f04c6adcb250000be6b40000000002aaaaa63f6b8410000",
         "e601030408040d84003e001d935801855b3004c000a5c240000000002aaaaa4a1c16cac000",
         "e601030408010da4d6124f022d0018a27eaa04c00095a1c0000000002aaaaa4505b88ac000",
         "e601030408030dbfef1f0a4abf5940b1c9d97f0000901ac0000000002aaaaa52772e0bc000",
         "e601030408053cc04c03007d5f300b0b15feea8000aaaa80000000002aaaaa4149c8810000",
     },
     29600175.268756226, E04_iod54},
    {"E10 IOD 60", 5,   // capture lines 35310, 35401, 36227, 36326, 36412
     {
         "e601030a08020f3c778b23ca282c1d61a52aea4000bcbc64a31dedcbeaaaaa5fe0fdc10000",
         "e601030a08040f0afff30010944bf9423537ffc000900040d9367844aaaaaa7189c98ac000",
         "e601030a08010f2512fd89b66e0021ae67aa04c00097e4e607c3c09f2aaaaa5acda0cac000",
         "e601030a08030f3ff1c186637d4d469fc2c9fc80008a5acc72e5468b2aaaaa78024c0bc000",
         "e601030a08053cc04c03007f1fc80b0b165caa8000aaaa808644bb866aaaaa5adc6c010000",
     },
     29600187.285473052, E10_iod60},
    {"C41 IOD 1", 3,   // capture lines 1262, 1545, 1827
     {
         "e2f101e24062bf781021cc56f310c409fefd8551f88af7c0040b9fbffe0dc1",
         "e2f102e240a2bf90a37822b17fca165f00b538b70c6a43fc80519694541662",
         "e2f103e240e2bfa8ade1422f15ffffabff6a2e00658c8656199999a39c6e94",
     },
     27906155.747745346, C41_iod1},
    {"C23 IOD 1", 3,   // capture lines 1276, 1558, 1840
     {
         "e2df01e24062bf781021cc56f0dc3709fefd8551f88af7c0061d78fffedba1",
         "e2df02e240a2bf90a7cfff7cfe0598c500238dae1bae41560001d6945521c6",
         "e2df03e240e2bfa8ade133dc2f5fff03ff6a99ffcefb94fff754c5fbd2e156",
     },
     27906500.603578985, C23_iod1},
    {"C29 IOD 1", 3,   // capture lines 44435, 44715, 44993
     {
         "e2e501e24062ce001021cc56f0020089fefd8551f88af7c00002bbd002d4a1",
         "e2e502e240a2ce18c583d72ede06f221000e405306e6446d3fa00a9454d5ae",
         "e2e503e240e2ce30ade133a500f0001fff5ca00027f7a5a7270b8ad24f8936",
     },
     27906402.4646424, C29_iod1},
};

// GPS subframe 4 page 18 (SV ID 56), G32, capture line 22369: the first the
// prototype took (EphStore.ion keeps the first). Its scaled parameters, and the
// broadcast integers they are (alpha0..3 at 2^-30, 2^-27, 2^-24, 2^-24 s/semicircle^n,
// beta0..3 at 2^11, 2^14, 2^16, 2^16 s/semicircle^n). Its HOW TOW count * 6 = 569784 s,
// GPS day 2437 * 7 + 569784 / 86400 = 17065.
const char* const kIonoFrame = "e020048b0614b97a33780e02fffe3204fdfc000006000000052486128907120000";
constexpr double kPyIon[8] = {1.30385160446167e-08, 1.4901161193847656e-08, -5.960464477539063e-08, -1.1920928955078125e-07, 102400.0, 65536.0, -196608.0, -262144.0};
constexpr int kIonoRaw[8] = {14, 2, -1, -2, 50, 4, -3, -4};
constexpr int kIonoPrn = 32;
constexpr unsigned kIonoTowS = 569784;
constexpr unsigned kIonoGpsDay = 17065;

// The prototype over the whole capture (EphStore, read_capture's frame selection):
// nav messages, ephemerides decoded (a repeat of an issue included), the distinct
// issues it kept, and Galileo CRC failures.
constexpr int kCaptureGpsFrames = 1054, kCaptureBdsFrames = 1081, kCaptureGalFrames = 3121;
constexpr int kCaptureGpsDecodes = 207, kCaptureGalDecodes = 200, kCaptureBdsDecodes = 174;
constexpr int kCaptureGpsIssues = 6, kCaptureGalIssues = 14, kCaptureBdsIssues = 8;
constexpr int kCaptureGalCrcFails = 0;

// Every issue it kept: system, PRN, IODE, toe in the system's own time (BeiDou BDT).
struct Issue
{
    int sys, prn, iode;
    double toe;
};
const Issue kCaptureIssues[] = {
    {GNSS_SYS_GPS, 10, 45, 576000.0},
    {GNSS_SYS_GPS, 15, 125, 576000.0},
    {GNSS_SYS_GPS, 18, 194, 576000.0},
    {GNSS_SYS_GPS, 23, 144, 576000.0},
    {GNSS_SYS_GPS, 24, 58, 576000.0},
    {GNSS_SYS_GPS, 32, 120, 576000.0},
    {GNSS_SYS_GAL, 4, 54, 565800.0},
    {GNSS_SYS_GAL, 4, 60, 569400.0},
    {GNSS_SYS_GAL, 6, 58, 568200.0},
    {GNSS_SYS_GAL, 6, 59, 568800.0},
    {GNSS_SYS_GAL, 6, 60, 569400.0},
    {GNSS_SYS_GAL, 10, 53, 565200.0},
    {GNSS_SYS_GAL, 10, 59, 568800.0},
    {GNSS_SYS_GAL, 10, 60, 569400.0},
    {GNSS_SYS_GAL, 11, 58, 568200.0},
    {GNSS_SYS_GAL, 28, 58, 568200.0},
    {GNSS_SYS_GAL, 28, 59, 568800.0},
    {GNSS_SYS_GAL, 36, 53, 565200.0},
    {GNSS_SYS_GAL, 36, 59, 568800.0},
    {GNSS_SYS_GAL, 36, 60, 569400.0},
    {GNSS_SYS_BDS, 11, 1, 568800.0},
    {GNSS_SYS_BDS, 23, 1, 568800.0},
    {GNSS_SYS_BDS, 25, 1, 568800.0},
    {GNSS_SYS_BDS, 27, 1, 568800.0},
    {GNSS_SYS_BDS, 29, 1, 568800.0},
    {GNSS_SYS_BDS, 30, 1, 568800.0},
    {GNSS_SYS_BDS, 32, 1, 568800.0},
    {GNSS_SYS_BDS, 41, 1, 568800.0},
};

}  // namespace skytraq_nav_fixture
