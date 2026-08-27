#ifndef SENSORCOLLECTOR_H
#define SENSORCOLLECTOR_H

#include <cstdint>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <TR_ISM6HG256.h>
#include <TR_BMP585.h>
#include <TR_MMC5983MA.h>
#include <TR_IIS2MDC.h>
// GNSS driver is a compile-time seam: the rocket-computer boards carry a
// u-blox receiver, the mini carries a Quectel LC86G. Both drivers expose the
// identical begin/pollNewPVT/getGNSSData surface and produce the same
// GNSSData, so the selection is the type alone. Projects opt in with
// add_compile_definitions(TR_GNSS_DRIVER_LC86=1) at project level (same
// pattern as TR_GUIDANCE_AVAILABLE); default is u-blox, unchanged.
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
#include <TR_GNSSReceiverLC86_Serial.h>
#else
#include <TR_GNSSReceiverUBlox_Serial.h>
#endif
#include <RocketComputerTypes.h>

typedef struct
{
    uint32_t isr_hits;
    uint32_t notify_wakes;
    uint32_t notify_timeouts;
    uint32_t drdy_lg_hits;
    uint32_t drdy_hg_hits;
    uint32_t drdy_g_hits;
    uint32_t drdy_triplet_hits;
    uint32_t drdy_histogram[8];
} ISM6HG256DebugSnapshot;

// Polling-task timing diagnostics for identifying data-gap root causes.
typedef struct
{
    uint32_t poll_iter_max_us;      // worst-case full poll-loop iteration
    uint32_t gnss_max_us;           // worst-case GNSS serial parse time
    uint32_t bmp_max_us;            // worst-case BMP585 read time
    uint32_t mmc_max_us;            // worst-case MMC5983MA read time
    uint32_t ism6_read_max_us;      // worst-case ISM6 triplet read time
    uint32_t gap_count;             // ISM6 timestamp gaps > GAP_THRESHOLD_US
    uint32_t gap_worst_us;          // largest ISM6 timestamp gap observed
    uint32_t gnss_calls;            // total GNSS parse calls
    uint32_t gnss_over_1ms;         // GNSS parses > 1 ms
    uint32_t gnss_over_5ms;         // GNSS parses > 5 ms
    uint32_t gnss_over_10ms;        // GNSS parses > 10 ms
    uint32_t ism6_queue_drops;      // IMU samples evicted from a full handoff queue
} PollTimingSnapshot;

typedef struct
{
    uint32_t isr_hits;
    uint32_t loop_checks;
    uint32_t irq_path_hits;
    uint32_t status_done_hits;
    uint32_t status_not_ready_hits;
    uint32_t status_read_failures;
    uint32_t process_hits;
    uint32_t clear_ok;
    uint32_t clear_fail;
    uint32_t read_ok;
    uint32_t read_fail;
    uint32_t rearm_ok;
    uint32_t rearm_fail;
    uint32_t cmm_lost_hits;
    uint32_t recovery_attempts;
    uint32_t recovery_success;
    uint8_t last_status;
    uint8_t last_ctrl0;
    uint8_t last_ctrl1;
    uint8_t last_ctrl2;
} MMC5983MADebugSnapshot;

class SensorCollector
{
public:

    /** GNSS high-perf-clock OTP state at boot; a gnss_otp::* constant.
     *  Board-agnostic: the LC86 driver reports NOT_M10 (#837 item 6). */
    uint8_t gnssOtpState() const { return gnss_receiver.otpState(); }
    SensorCollector(uint8_t ISM6HG256_CS,
                    uint8_t ISM6HG256_INT,
                    uint16_t ISM6HG256_UPDATE_RATE,
                    uint8_t BMP585_CS,
                    uint8_t BMP585_INT,
                    uint16_t BMP585_UPDATE_RATE,
                    uint8_t MMC5983MA_CS,
                    uint8_t MMC5983MA_INT,
                    uint16_t MMC5983MA_UPDATE_RATE,
                    uint8_t IIS2MDC_SDA,
                    uint8_t IIS2MDC_SCL,
                    uint8_t IIS2MDC_INT,
                    uint32_t IIS2MDC_I2C_FREQ_HZ,
                    uint8_t IIS2MDC_I2C_ADDR,
                    uint16_t GNSS_UPDATE_RATE,
                    uint8_t GNSS_RX,
                    uint8_t GNSS_TX,
                    int8_t GNSS_RESET_N,
                    int8_t GNSS_SAFEBOOT_N,
                    bool use_bmp585,
                    bool use_mmc5983ma,
                    bool use_iis2mdc,
                    bool use_gnss,
                    bool use_ism6hg256,
                    uint32_t spi_speed,
                    // #386: ISM6 full-scale settings. Were hardcoded 16/256/4000
                    // literals in begin() while the converter scale + snapshots
                    // read config::ISM6_*_FS — editing config.h silently
                    // desynced chip FS from conversion scale (2x-8x data
                    // error). Defaults preserve the flown values; the FC
                    // passes its config constants so there is one source of
                    // truth.
                    uint8_t  ism6_low_g_fs_g   = 16,
                    uint16_t ism6_high_g_fs_g  = 256,
                    uint16_t ism6_gyro_fs_dps  = 4000,
                    // Mini seam: the mini's magnetometer shares one physical
                    // I2C bus with the INA230, so the app creates the bus and
                    // hands the same handle to both. nullptr (the default and
                    // every existing caller) keeps the collector creating and
                    // owning its own bus on I2C_NUM_1.
                    i2c_master_bus_handle_t external_i2c_bus = nullptr);

    void begin(uint8_t imu_execution_core);

    // #386: chip full-scale settings (see ctor comment).
    const uint8_t  ism6_low_g_fs_g_;
    const uint16_t ism6_high_g_fs_g_;
    const uint16_t ism6_gyro_fs_dps_;

    volatile bool bmp585_data_ready;
    volatile bool mmc5983ma_data_ready;
    volatile bool iis2mdc_data_ready;
    volatile bool gnss_data_ready;

    bool getISM6HG256Data(ISM6HG256Data &data_out);
    bool getBMP585Data(BMP585Data &data_out);
    bool getMMC5983MAData(MMC5983MAData &data_out);
    bool getIIS2MDCData(IIS2MDCData &data_out);
    bool getGNSSData(GNSSData &data_out);
    void getISM6HG256DebugSnapshot(ISM6HG256DebugSnapshot &snapshot_out) const;
    void getMMC5983MADebugSnapshot(MMC5983MADebugSnapshot &snapshot_out) const;
    void getPollTimingSnapshot(PollTimingSnapshot &snapshot_out) const;
    void resetPollTimingSnapshot();

    // Pad calibration: gyro zero-rate + high-g accel cross-cal against low-g
    void calibrateGyro(float rotation_z_deg = 0.0f);
    int16_t gyro_cal_x;
    int16_t gyro_cal_y;
    int16_t gyro_cal_z;

    // True iff IIS2MDC was detected at boot and is the active mag path.
    // (Issue #96 — needed by the cal flow to choose IIS2MDC OFFSET regs
    // vs. MMC software offset.)
    bool isIIS2MDCActive() const { return iis2mdc_active; }

    // #557: true iff GNSS is enabled in the build AND the module came up at
    // boot.  False after a dead/deaf-UART bring-up failure — the flight
    // computer reads this to enter GNSS-absent mode (baro+IMU EKF init,
    // guidance forced off).  Always false when use_gnss is compiled out.
    bool isGnssOnline() const { return use_gnss && gnss_online_; }

    // Program IIS2MDC OFFSET_X/Y/Z hard-iron registers.  Returns false if
    // the IIS2MDC isn't active or the I2C write failed.  Issue #96.
    bool setIIS2MDCHardIronOffset(int16_t cx, int16_t cy, int16_t cz);

    // Calibration results (populated by calibrateGyro)
    float hg_bias_x = 0.0f, hg_bias_y = 0.0f, hg_bias_z = 0.0f;  // m/s², body frame
    float cal_gravity_mag = 0.0f;  // measured gravity magnitude from low-g (m/s²)

private:
    static SensorCollector *ism6hg256_instance;
    static SensorCollector *bmp585_instance;
    static SensorCollector *mmc5983ma_instance;

    uint32_t start_time;

    uint32_t spi_speed;

    uint8_t ISM6HG256_CS;
    uint8_t ISM6HG256_INT;
    uint16_t ISM6HG256_UPDATE_RATE;
    bool ism6_rate_applied_ = false;  // Begin() programmed the chip ODR
    // Live rate change staged by setIsm6Rate(); pollIMUdata applies it
    // between transactions. The ISM6 uses spi_device_polling_transmit, so
    // only the poll task may touch the chip once it is running (#475).
    volatile uint16_t ism6_rate_pending_ = 0;
    uint8_t BMP585_CS;
    uint8_t BMP585_INT;
    uint16_t BMP585_UPDATE_RATE;
    uint8_t MMC5983MA_CS;
    uint8_t MMC5983MA_INT;
    uint16_t MMC5983MA_UPDATE_RATE;
    uint8_t IIS2MDC_SDA;
    uint8_t IIS2MDC_SCL;
    uint8_t IIS2MDC_INT;
    uint32_t IIS2MDC_I2C_FREQ_HZ;
    uint8_t IIS2MDC_I2C_ADDR;
    uint16_t GNSS_UPDATE_RATE;
    uint8_t GNSS_RX;
    uint8_t GNSS_TX;
    int8_t GNSS_RESET_N;
    int8_t GNSS_SAFEBOOT_N;
    bool use_bmp585;
    bool use_mmc5983ma;
    bool use_iis2mdc;
    bool use_gnss;
    bool use_ism6hg256;

    // #557: runtime GNSS liveness.  Starts true; cleared in begin() if the
    // module fails bring-up (dead/deaf UART).  Gates the GNSS poll task and is
    // exposed via isGnssOnline() so the FC can enter GNSS-absent mode.
    bool gnss_online_ = true;

    // Set true after IIS2MDC is detected and configured at boot. When true,
    // the legacy MMC5983MA SPI path is skipped.
    bool iis2mdc_active = false;
    i2c_master_bus_handle_t iis2mdc_bus = nullptr;
    // True when iis2mdc_bus was supplied by the app (shared with other
    // devices) — the collector then must not create or delete it.
    bool iis2mdc_bus_external = false;

    TR_ISM6HG256 ism6hg256;
    TR_BMP585 bmp585;
    TR_MMC5983MA mmc5983ma;
    TR_IIS2MDC iis2mdc;
#if defined(TR_GNSS_DRIVER_LC86) && TR_GNSS_DRIVER_LC86
    TR_GNSSReceiverLC86Serial gnss_receiver;
#else
    TR_GNSSReceiverUBloxSerial gnss_receiver;
#endif
    ISM6HG256Data ism6hg256_data;   // scratch for the poll task (queue is the handoff)
    BMP585Data bmp585_data;
    MMC5983MAData mmc5983ma_data;
    IIS2MDCData iis2mdc_data;
    GNSSData gnss_data;

    // IMU handoff queue (poll task -> consumer).  The previous single-slot
    // latest-sample handoff silently dropped any sample the consumer didn't
    // collect before the next DRDY — capping the LOGGED rate at the flight
    // loop rate (~980/s) no matter the chip ODR.  A short queue lets the loop
    // drain every sample (it logs each one; control uses the freshest), so
    // the logged rate follows ISM6HG256_UPDATE_RATE.  Depth 32 ≈ 17 ms at
    // 1920 Hz: loop jitter is ~1.4 ms, but blocking loop work (NVS commits on
    // config saves) measured ~1-3 dropped samples per commit at depth 8 on
    // the 2026-07-09 bench — 32 gives margin over the worst observed stall so
    // in-session drops genuinely mean something is wrong.  Overflow drops the
    // OLDEST sample and counts it (imu_q_drops in [GAP DIAG], zeroed by the
    // consumer when it starts draining — the ~4 s of pre-loop boot produce
    // thousands of meaningless pre-consumer drops otherwise).
    QueueHandle_t ism6Queue = nullptr;
    // 256 samples = ~67 ms of buffer at 3840 Hz (~133 ms at 1920) — sized to
    // ABSORB the worst-case ground-state consumer stall, not merely tolerate a
    // fraction of it.  loop_fc() services the I2C command poll inline, and its
    // masterRead carries a 50 ms timeout (#399: aborting mid-read desyncs the OC
    // slave TX ring, so it can't be shortened) plus up to ~15 ms of config-read
    // retries — a ~65 ms stall.  The earlier depth-64 (33 ms @1920) let that
    // stall silently drop IMU samples at the logging handoff (#474: a 34.6 ms
    // all-stream hole at activateLogging with zero recorded counters); the
    // 47.7 ms I2C-recovery stall overflowed even 64.  256 covers the full stall
    // at every user-selectable rate, so the SPI-fed queue buffers straight
    // through it and loop_fc drains the burst on the next pass (the 512-deep
    // I2S TX queue absorbs it).  Witnesses: the [GAP DIAG] imu_q_drops serial
    // gauge plus the logged NSF2_FC_IMU_DROP flag if any drop still occurs.
    static constexpr UBaseType_t ISM6_QUEUE_DEPTH = 256;
    volatile uint32_t ism6_queue_drops = 0;

public:
    // Zero the drop counter (and nothing else).  Called once by the consumer
    // when it begins draining, so the gauge counts only consumer-era drops.
    void resetIsm6QueueDrops() { ism6_queue_drops = 0; }

    // Consumer-era IMU-queue drop count since the last resetIsm6QueueDrops().
    // Nonzero means loop_fc() stalled long enough to overflow the handoff queue
    // and lose IMU samples — the FC-side witness for the #474 silent hole. Read
    // by the NonSensor builder to set the logged NSF2_FC_IMU_DROP flag.
    uint32_t getIsm6QueueDrops() const { return ism6_queue_drops; }

    // Runtime IMU logging-rate change (user setting, BLE cmd 67): reprograms
    // the ISM6HG256 ODR on all three channels and the derived period. Safe
    // while the DRDY-driven poll task runs — the task services whatever
    // cadence the chip produces; one inter-sample gap changes length at the
    // switch and downstream consumers use per-sample timestamps. Call before
    // Begin() to only stage the rate for init.
    bool setIsm6Rate(uint16_t rate_hz);
    uint16_t ism6Rate() const { return ISM6HG256_UPDATE_RATE; }

private:

    SemaphoreHandle_t bmp585DataSemaphore;
    SemaphoreHandle_t mmc5983maDataSemaphore;
    SemaphoreHandle_t iis2mdcDataSemaphore;
    SemaphoreHandle_t gnssDataSemaphore;

    // Time-gated polling state for IIS2MDC (no DRDY pin yet).
    uint32_t iis2mdc_last_sample_us = 0;
    static constexpr uint32_t IIS2MDC_PERIOD_US = 10000;  // 100 Hz, matches default ODR
    TaskHandle_t pollIMUTaskHandle;
    TaskHandle_t pollGNSSTaskHandle = nullptr;

    uint32_t ism6hg256_update_period;
    uint32_t mmc_last_sample_time_us;
    uint32_t mmc_last_recover_time_us;
    uint32_t mmc_stall_threshold_us;
    uint32_t mmc_recover_cooldown_us;

    volatile bool ism6_isr_fired;        // Set by ISR, cleared by poll loop
    volatile uint32_t ism6_isr_hits;
    volatile uint32_t ism6_notify_wakes;
    volatile uint32_t ism6_notify_timeouts;
    volatile uint32_t ism6_drdy_lg_hits;
    volatile uint32_t ism6_drdy_hg_hits;
    volatile uint32_t ism6_drdy_g_hits;
    volatile uint32_t ism6_drdy_triplet_hits;
    volatile uint32_t ism6_drdy_histogram[8];
    volatile uint32_t bmp585_isr_hits;
    volatile uint32_t bmp585_irq_pending_count;
    volatile uint32_t mmc5983ma_isr_hits;
    volatile uint32_t mmc5983ma_irq_pending_count;
    volatile uint32_t mmc5983ma_loop_checks;
    volatile uint32_t mmc5983ma_irq_path_hits;
    volatile uint32_t mmc5983ma_status_done_hits;
    volatile uint32_t mmc5983ma_status_not_ready_hits;
    volatile uint32_t mmc5983ma_status_read_failures;
    volatile uint32_t mmc5983ma_process_hits;
    volatile uint32_t mmc5983ma_clear_ok;
    volatile uint32_t mmc5983ma_clear_fail;
    volatile uint32_t mmc5983ma_read_ok;
    volatile uint32_t mmc5983ma_read_fail;
    volatile uint32_t mmc5983ma_rearm_ok;
    volatile uint32_t mmc5983ma_rearm_fail;
    volatile uint32_t mmc5983ma_cmm_lost_hits;
    volatile uint32_t mmc5983ma_recovery_attempts;
    volatile uint32_t mmc5983ma_recovery_success;
    volatile uint8_t mmc5983ma_last_status;
    volatile uint8_t mmc5983ma_last_ctrl0;
    volatile uint8_t mmc5983ma_last_ctrl1;
    volatile uint8_t mmc5983ma_last_ctrl2;

    // Polling-task gap diagnostics
    static constexpr uint32_t GAP_THRESHOLD_US = 10000;  // 10 ms
    volatile uint32_t pt_iter_max_us    = 0;
    volatile uint32_t pt_gnss_max_us    = 0;
    volatile uint32_t pt_bmp_max_us     = 0;
    volatile uint32_t pt_mmc_max_us     = 0;
    volatile uint32_t pt_ism6_read_max_us = 0;
    volatile uint32_t pt_gap_count      = 0;
    volatile uint32_t pt_gap_worst_us   = 0;
    volatile uint32_t pt_last_ism6_time_us = 0;
    volatile uint32_t pt_gnss_calls     = 0;
    volatile uint32_t pt_gnss_over_1ms  = 0;
    volatile uint32_t pt_gnss_over_5ms  = 0;
    volatile uint32_t pt_gnss_over_10ms = 0;

    void startPollingTask(uint8_t imu_execution_core);
    static void pollIMUdata(void *parameter);
    static void pollGNSSdata(void *parameter);
    // IDF gpio_isr_handler signature is void(*)(void*).  The arg is
    // unused — each trampoline dispatches via its sensor-specific
    // static *_instance pointer set in begin().
    static void onISM6HG256Int1Trampoline(void* arg);
    void onISM6HG256Int1();
    static void onBMP585IntTrampoline(void* arg);
    void onBMP585Int();
    static void onMMC5983MAIntTrampoline(void* arg);
    void onMMC5983MAInt();
};

#endif
