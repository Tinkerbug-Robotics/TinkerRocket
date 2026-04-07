#ifndef SENSORCOLLECTOR_H
#define SENSORCOLLECTOR_H

#include <compat.h>
#include <TR_ISM6HG256.h>
#include <TR_BMP585.h>
#include <TR_MMC5983MA.h>
#include <TR_GNSSReceiverUBlox_Serial.h>
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
    SensorCollector(uint8_t ISM6HG256_CS,
                    uint8_t ISM6HG256_INT,
                    uint16_t ISM6HG256_UPDATE_RATE,
                    uint8_t BMP585_CS,
                    uint8_t BMP585_INT,
                    uint16_t BMP585_UPDATE_RATE,
                    uint8_t MMC5983MA_CS,
                    uint8_t MMC5983MA_INT,
                    uint16_t MMC5983MA_UPDATE_RATE,
                    uint16_t GNSS_UPDATE_RATE,
                    uint8_t GNSS_RX,
                    uint8_t GNSS_TX,
                    int8_t GNSS_RESET_N,
                    int8_t GNSS_SAFEBOOT_N,
                    bool use_bmp585,
                    bool use_mmc5983ma,
                    bool use_gnss,
                    bool use_ism6hg256,
                    SPIClass &spi,
                    uint32_t spi_speed);

    void begin(uint8_t imu_execution_core);

    volatile bool ism6hg256_data_ready;
    volatile bool bmp585_data_ready;
    volatile bool mmc5983ma_data_ready;
    volatile bool gnss_data_ready;

    bool getISM6HG256Data(ISM6HG256Data &data_out);
    bool getBMP585Data(BMP585Data &data_out);
    bool getMMC5983MAData(MMC5983MAData &data_out);
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

    // Calibration results (populated by calibrateGyro)
    float hg_bias_x = 0.0f, hg_bias_y = 0.0f, hg_bias_z = 0.0f;  // m/s², body frame
    float cal_gravity_mag = 0.0f;  // measured gravity magnitude from low-g (m/s²)

private:
    static SensorCollector *ism6hg256_instance;
    static SensorCollector *bmp585_instance;
    static SensorCollector *mmc5983ma_instance;

    uint32_t start_time;

    SPIClass &spi;
    uint32_t spi_speed;

    uint8_t ISM6HG256_CS;
    uint8_t ISM6HG256_INT;
    uint16_t ISM6HG256_UPDATE_RATE;
    uint8_t BMP585_CS;
    uint8_t BMP585_INT;
    uint16_t BMP585_UPDATE_RATE;
    uint8_t MMC5983MA_CS;
    uint8_t MMC5983MA_INT;
    uint16_t MMC5983MA_UPDATE_RATE;
    uint16_t GNSS_UPDATE_RATE;
    uint8_t GNSS_RX;
    uint8_t GNSS_TX;
    int8_t GNSS_RESET_N;
    int8_t GNSS_SAFEBOOT_N;
    bool use_bmp585;
    bool use_mmc5983ma;
    bool use_gnss;
    bool use_ism6hg256;

    TR_ISM6HG256 ism6hg256;
    TR_BMP585 bmp585;
    TR_MMC5983MA mmc5983ma;
    TR_GNSSReceiverUBloxSerial gnss_receiver;
    ISM6HG256Data ism6hg256_data;
    BMP585Data bmp585_data;
    MMC5983MAData mmc5983ma_data;
    GNSSData gnss_data;

    SemaphoreHandle_t ism6hg256DataSemaphore;
    SemaphoreHandle_t bmp585DataSemaphore;
    SemaphoreHandle_t mmc5983maDataSemaphore;
    SemaphoreHandle_t gnssDataSemaphore;
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
    static void onISM6HG256Int1Trampoline();
    void onISM6HG256Int1();
    static void onBMP585IntTrampoline();
    void onBMP585Int();
    static void onMMC5983MAIntTrampoline();
    void onMMC5983MAInt();
};

#endif
