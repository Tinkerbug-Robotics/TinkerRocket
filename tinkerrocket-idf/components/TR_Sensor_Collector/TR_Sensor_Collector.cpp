// Suppress C++20 volatile++ deprecation — these are single-writer ISR counters
#pragma GCC diagnostic ignored "-Wvolatile"

#include <TR_Sensor_Collector.h>
#include <esp_log.h>
static const char* SC_TAG = "SENSORS";

SensorCollector* SensorCollector::ism6hg256_instance = nullptr;
SensorCollector* SensorCollector::bmp585_instance = nullptr;
SensorCollector* SensorCollector::mmc5983ma_instance = nullptr;

SensorCollector::SensorCollector(
                           uint8_t ISM6HG256_CS,
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
                           SPIClass &spi,
                           uint32_t spi_speed)
    : ISM6HG256_CS(ISM6HG256_CS),
      ISM6HG256_INT(ISM6HG256_INT),
      ISM6HG256_UPDATE_RATE(ISM6HG256_UPDATE_RATE),
      BMP585_CS(BMP585_CS),
      BMP585_INT(BMP585_INT),
      BMP585_UPDATE_RATE(BMP585_UPDATE_RATE),
      MMC5983MA_CS(MMC5983MA_CS),
      MMC5983MA_INT(MMC5983MA_INT),
      MMC5983MA_UPDATE_RATE(MMC5983MA_UPDATE_RATE),
      IIS2MDC_SDA(IIS2MDC_SDA),
      IIS2MDC_SCL(IIS2MDC_SCL),
      IIS2MDC_INT(IIS2MDC_INT),
      IIS2MDC_I2C_FREQ_HZ(IIS2MDC_I2C_FREQ_HZ),
      IIS2MDC_I2C_ADDR(IIS2MDC_I2C_ADDR),
      GNSS_UPDATE_RATE(GNSS_UPDATE_RATE),
      GNSS_RX(GNSS_RX),
      GNSS_TX(GNSS_TX),
      GNSS_RESET_N(GNSS_RESET_N),
      GNSS_SAFEBOOT_N(GNSS_SAFEBOOT_N),
      use_bmp585(use_bmp585),
      use_mmc5983ma(use_mmc5983ma),
      use_iis2mdc(use_iis2mdc),
      use_gnss(use_gnss),
      use_ism6hg256(use_ism6hg256),
      spi(spi),
      spi_speed(spi_speed),
      bmp585(this->spi, BMP585_CS, SPISettings(spi_speed, MSBFIRST, SPI_MODE0)),
      mmc5983ma(this->spi, MMC5983MA_CS, SPISettings(2000000, MSBFIRST, SPI_MODE0)),  // MMC5983MA supports Mode 0 and Mode 3
      iis2mdc(IIS2MDC_I2C_ADDR),
      ism6hg256(&this->spi, ISM6HG256_CS, spi_speed),
      gyro_cal_x(0), gyro_cal_y(0), gyro_cal_z(0) {}

void SensorCollector::begin(uint8_t imu_execution_core) 
{
    uint8_t i = 0;

    // Data ready flag init
    ism6hg256_data_ready = false;
    bmp585_data_ready = false;
    mmc5983ma_data_ready = false;
    iis2mdc_data_ready = false;
    gnss_data_ready = false;
    iis2mdc_last_sample_us = 0;
    bmp585_irq_pending_count = 0;
    mmc5983ma_irq_pending_count = 0;
    ism6_isr_fired = false;
    ism6_isr_hits = 0;
    ism6_notify_wakes = 0;
    ism6_notify_timeouts = 0;
    ism6_drdy_lg_hits = 0;
    ism6_drdy_hg_hits = 0;
    ism6_drdy_g_hits = 0;
    ism6_drdy_triplet_hits = 0;
    bmp585_isr_hits = 0;
    mmc5983ma_isr_hits = 0;
    mmc5983ma_loop_checks = 0;
    mmc5983ma_irq_path_hits = 0;
    mmc5983ma_status_done_hits = 0;
    mmc5983ma_status_not_ready_hits = 0;
    mmc5983ma_status_read_failures = 0;
    mmc5983ma_process_hits = 0;
    mmc5983ma_clear_ok = 0;
    mmc5983ma_clear_fail = 0;
    mmc5983ma_read_ok = 0;
    mmc5983ma_read_fail = 0;
    mmc5983ma_rearm_ok = 0;
    mmc5983ma_rearm_fail = 0;
    mmc5983ma_cmm_lost_hits = 0;
    mmc5983ma_recovery_attempts = 0;
    mmc5983ma_recovery_success = 0;
    mmc5983ma_last_status = 0;
    mmc5983ma_last_ctrl0 = 0;
    mmc5983ma_last_ctrl1 = 0;
    mmc5983ma_last_ctrl2 = 0;
    for (i = 0; i < 8; i++)
    {
        ism6_drdy_histogram[i] = 0;
    }
    
    // Initialize and turn off SPI for all sensors
    pinMode(ISM6HG256_CS, OUTPUT);
    digitalWrite(ISM6HG256_CS, HIGH);
    pinMode(BMP585_CS, OUTPUT);
    digitalWrite(BMP585_CS, HIGH);
    pinMode(MMC5983MA_CS, OUTPUT);
    digitalWrite(MMC5983MA_CS, HIGH);
    delay(10);
    
    // Update periods calculated from frequencies
    ism6hg256_update_period = (uint32_t)(1000000/ISM6HG256_UPDATE_RATE);
    mmc_last_sample_time_us = micros();
    mmc_last_recover_time_us = 0;
    // Stall = 5 nominal periods; cooldown = 15 nominal periods. Scales with configured rate.
    mmc_stall_threshold_us  = 5U  * (1000000UL / (uint32_t)MMC5983MA_UPDATE_RATE);
    mmc_recover_cooldown_us = 15U * (1000000UL / (uint32_t)MMC5983MA_UPDATE_RATE);

    ism6hg256DataSemaphore = xSemaphoreCreateBinary();
    if (ism6hg256DataSemaphore == NULL)
    {
        ESP_LOGE(SC_TAG, "Failed to create ISM6HG256 data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(ism6hg256DataSemaphore);

    bmp585DataSemaphore = xSemaphoreCreateBinary();
    if (bmp585DataSemaphore == NULL)
    {
        ESP_LOGE(SC_TAG, "Failed to create BMP585 data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(bmp585DataSemaphore);

    mmc5983maDataSemaphore = xSemaphoreCreateBinary();
    if (mmc5983maDataSemaphore == NULL)
    {
        ESP_LOGE(SC_TAG, "Failed to create MMC5983MA data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(mmc5983maDataSemaphore);

    iis2mdcDataSemaphore = xSemaphoreCreateBinary();
    if (iis2mdcDataSemaphore == NULL)
    {
        ESP_LOGE(SC_TAG, "Failed to create IIS2MDC data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(iis2mdcDataSemaphore);

    gnssDataSemaphore = xSemaphoreCreateBinary();
    if (gnssDataSemaphore == NULL)
    {
        ESP_LOGE(SC_TAG, "Failed to create GNSS data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(gnssDataSemaphore);

    // ### Initialize Sensors ###
    ESP_LOGI(SC_TAG, "Initializing BMP585...");
    if (use_bmp585)
    {
        uint32_t bmp_retry_count = 0;
        while (!bmp585.begin())
        {
            ESP_LOGW(SC_TAG, "BMP585 initialization failed, chip ID 0x%02X", bmp585.readChipIdCached());
            bmp_retry_count++;

            // Warm-reset recovery path: after MCU flash, sensor may still be
            // in active mode and not respond cleanly until a soft reset.
            const bool reset_ok = bmp585.forceSoftResetRaw();
            ESP_LOGI(SC_TAG, "BMP585 soft-reset recovery %s", reset_ok ? "OK" : "pending");

            // Re-drive CS high between retries to avoid bus ambiguity.
            pinMode(BMP585_CS, OUTPUT);
            digitalWrite(BMP585_CS, HIGH);
            delay(10);

            if ((bmp_retry_count % 5U) == 0U)
            {
                ESP_LOGW(SC_TAG, "BMP585 retry count: %lu", (unsigned long)bmp_retry_count);
            }
            delay(500);
        }

        ESP_LOGI(SC_TAG, "BMP585 OK. chip_id = 0x%02X", bmp585.readChipId());

        bool bmp_ok = true;
        // Continuous mode ignores the ODR register (datasheet §4.3.6).
        // Throughput is set solely by OSR: x1/x1 ≈ 498 Hz theoretical.
        // Continuous mode (§4.3.6) ignores the ODR register — throughput is
        // determined solely by OSR settings.  OSR x1/x1 yields ~460 Hz.
        bmp_ok = bmp_ok && bmp585.setTemperatureOversampling(TR_BMP585::Oversampling::x1);
        bmp_ok = bmp_ok && bmp585.setPressureOversampling(TR_BMP585::Oversampling::x1);
        bmp_ok = bmp_ok && bmp585.setIirFilter(TR_BMP585::IirCoeff::Bypass,
                                               TR_BMP585::IirCoeff::Bypass);
        bmp_ok = bmp_ok && bmp585.enableDataReadyInterrupt(true, false, true, false);
        bmp_ok = bmp_ok && bmp585.setPowerMode(TR_BMP585::PowerMode::Continuous);

        if (!bmp_ok)
        {
            ESP_LOGE(SC_TAG, "BMP585 configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        ESP_LOGI(SC_TAG, "BMP585 continuous mode active; throughput set by OSR/filter (ODR ignored in this mode).");

        bmp585_instance = this;
        pinMode(BMP585_INT, INPUT);
        attachInterrupt(BMP585_INT, onBMP585IntTrampoline, RISING);
    }

    // ### Magnetometer auto-detection ###
    // New PCB rev replaces the MMC5983MA (SPI) with an IIS2MDC (I2C).
    // Pin 13 is shared between MMC5983MA_CS and IIS2MDC_SDA, so we probe
    // I2C first; only if WHO_AM_I returns 0x40 do we keep the I2C bus
    // alive. On miss we tear the bus down (freeing pin 13 for SPI CS use)
    // and fall through to the legacy MMC5983MA path.
    if (use_iis2mdc && use_mmc5983ma)
    {
        ESP_LOGI(SC_TAG, "Probing for IIS2MDC on I2C SDA=%d SCL=%d addr=0x%02X...",
                 (int)IIS2MDC_SDA, (int)IIS2MDC_SCL, (unsigned)IIS2MDC_I2C_ADDR);

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_1;  // FC->OC bus already owns I2C_NUM_0
        bus_cfg.sda_io_num = (gpio_num_t)IIS2MDC_SDA;
        bus_cfg.scl_io_num = (gpio_num_t)IIS2MDC_SCL;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;

        esp_err_t bus_err = i2c_new_master_bus(&bus_cfg, &iis2mdc_bus);
        if (bus_err != ESP_OK)
        {
            ESP_LOGW(SC_TAG, "IIS2MDC I2C bus init failed (%s) — falling back to MMC5983MA",
                     esp_err_to_name(bus_err));
            iis2mdc_bus = nullptr;
        }
        else if (iis2mdc.begin(iis2mdc_bus, IIS2MDC_I2C_FREQ_HZ) == TR_IIS2MDC_OK)
        {
            // Configure: 100 Hz continuous, high-resolution, offset cancel on,
            // BDU on, DRDY pin off (we'll add interrupt routing in a follow-up).
            if (iis2mdc.configure() != TR_IIS2MDC_OK)
            {
                ESP_LOGE(SC_TAG, "IIS2MDC configuration failed, stopping.");
                while (1) { delay(1000); }
            }

            iis2mdc_active = true;
            ESP_LOGI(SC_TAG, "IIS2MDC found and initialized (100 Hz continuous, BDU on)");

            // Dump OFFSET_X/Y/Z hard-iron correction registers — should be all
            // zero after softReset(); non-zero values would be subtracted from
            // every reading and could explain a fixed offset.
            uint8_t off[6] = {0};
            for (int i = 0; i < 6; i++)
            {
                (void)iis2mdc.readRegister(IIS2MDC_Reg::OFFSET_X_REG_L + i, &off[i]);
            }
            ESP_LOGI(SC_TAG,
                "IIS2MDC OFFSET regs: X=%02X%02X Y=%02X%02X Z=%02X%02X",
                off[1], off[0], off[3], off[2], off[5], off[4]);

            // Sanity print — read a few samples to confirm the part is alive.
            // Continuous mode at 100 Hz means a fresh sample is ready every 10 ms.
            delay(20);  // allow the first conversion to complete
            for (int s = 0; s < 5; s++)
            {
                float x_uT = 0.0f, y_uT = 0.0f, z_uT = 0.0f;
                if (iis2mdc.readFieldsXYZ_uT(&x_uT, &y_uT, &z_uT) == TR_IIS2MDC_OK)
                {
                    const float mag = sqrtf(x_uT * x_uT + y_uT * y_uT + z_uT * z_uT);
                    ESP_LOGI(SC_TAG, "IIS2MDC sample %d: x=%.2f y=%.2f z=%.2f uT  |B|=%.2f uT",
                             s, (double)x_uT, (double)y_uT, (double)z_uT, (double)mag);
                }
                else
                {
                    ESP_LOGW(SC_TAG, "IIS2MDC sample %d read failed", s);
                }
                delay(15);
            }
        }
        else
        {
            ESP_LOGI(SC_TAG, "IIS2MDC not detected — falling back to MMC5983MA");
            // Tear down the bus so pin 13 is free for SPI CS.
            (void)i2c_del_master_bus(iis2mdc_bus);
            iis2mdc_bus = nullptr;
        }
    }

    ESP_LOGI(SC_TAG, "Initializing MMC5983MA...");
    if (use_mmc5983ma && !iis2mdc_active)
    {
        // Pin 13 was potentially driven by the I2C probe above; restore CS high.
        pinMode(MMC5983MA_CS, OUTPUT);
        digitalWrite(MMC5983MA_CS, HIGH);
        delay(2);

        while (!mmc5983ma.begin())
        {
            ESP_LOGW(SC_TAG, "MMC5983MA did not respond. Retrying...");
            delay(500);
            (void)mmc5983ma.softReset();
            delay(500);
        }

        (void)mmc5983ma.softReset();
        (void)mmc5983ma.performSetOperation();
        delay(10);
        (void)mmc5983ma.performResetOperation();
        delay(10);
        (void)mmc5983ma.enableAutomaticSetReset();  // Eliminates hysteresis drift

        bool mmc_ok = true;
        mmc_ok = mmc_ok && mmc5983ma.setFilterBandwidth((MMC5983MA_UPDATE_RATE >= 1000U) ? 800U : 400U);
        mmc_ok = mmc_ok && mmc5983ma.setContinuousModeFrequency((MMC5983MA_UPDATE_RATE >= 1000U) ? 1000U
                                                     : (MMC5983MA_UPDATE_RATE >= 200U) ? 200U
                                                     : (MMC5983MA_UPDATE_RATE >= 100U) ? 100U
                                                     : (MMC5983MA_UPDATE_RATE >= 50U) ? 50U
                                                     : (MMC5983MA_UPDATE_RATE >= 20U) ? 20U
                                                     : (MMC5983MA_UPDATE_RATE >= 10U) ? 10U
                                                     : (MMC5983MA_UPDATE_RATE >= 1U) ? 1U : 0U);
        mmc_ok = mmc_ok && mmc5983ma.enableContinuousMode();
        mmc_ok = mmc_ok && mmc5983ma.enableInterrupt();

        if (!mmc_ok)
        {
            ESP_LOGE(SC_TAG, "MMC5983MA configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        ESP_LOGI(SC_TAG, "MMC5983MA found and initialized");
        if (MMC5983MA_UPDATE_RATE >= 1000U)
        {
            ESP_LOGI(SC_TAG, "MMC5983MA max-rate mode: BW=800Hz, CM_FREQ=1000Hz, CMM_EN=1.");
        }

        uint8_t ctrl0 = 0;
        uint8_t ctrl1 = 0;
        uint8_t ctrl2 = 0;
        if (mmc5983ma.readControl0(&ctrl0))
        {
            mmc5983ma_last_ctrl0 = ctrl0;
        }
        if (mmc5983ma.readControl1(&ctrl1))
        {
            mmc5983ma_last_ctrl1 = ctrl1;
        }
        if (mmc5983ma.readControl2(&ctrl2))
        {
            mmc5983ma_last_ctrl2 = ctrl2;
        }

        mmc5983ma_instance = this;
        pinMode(MMC5983MA_INT, INPUT);
        attachInterrupt(MMC5983MA_INT, onMMC5983MAIntTrampoline, RISING);
    }

    ESP_LOGI(SC_TAG, "Initializing ISM6HG256...");
    if(use_ism6hg256)
    {
        uint8_t whoami = 0;

        if (ism6hg256.ReadWhoAmI(&whoami) != TR_ISM6HG256_OK)
        {
            ESP_LOGE(SC_TAG, "ISM6HG256 WHOAMI read failed, stopping.");
            while (1) { delay(1000); }
        }
        if (whoami != ISM6HG256X_ID)
        {
            ESP_LOGE(SC_TAG, "ISM6HG256 WHOAMI mismatch, got 0x%02X expected 0x%02X", whoami, ISM6HG256X_ID);
            while (1) { delay(1000); }
        }

        // Initialize and configure ISM6HG256 (low-g + high-g + gyro)
        TR_ISM6HG256Status status = TR_ISM6HG256_OK;
        status = (TR_ISM6HG256Status)(status | ism6hg256.begin());
        status = (TR_ISM6HG256Status)(status | ism6hg256.Enable_X());
        status = (TR_ISM6HG256Status)(status | ism6hg256.Enable_HG_X());
        status = (TR_ISM6HG256Status)(status | ism6hg256.Enable_G());
        status = (TR_ISM6HG256Status)(status | ism6hg256.Route_DRDY_To_INT1());

        // Apply configured update rate.
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_X_OutputDataRate((float)ISM6HG256_UPDATE_RATE));
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_HG_X_OutputDataRate((float)ISM6HG256_UPDATE_RATE));
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_G_OutputDataRate((float)ISM6HG256_UPDATE_RATE));

        // Full-scale settings: low-g=16g, high-g=256g, gyro=4000 dps.
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_X_FullScale(16));
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_HG_X_FullScale(256));
        status = (TR_ISM6HG256Status)(status | ism6hg256.Set_G_FullScale(4000));

        if (status != TR_ISM6HG256_OK)
        {
            ESP_LOGE(SC_TAG, "ISM6HG256 initialization/configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        ESP_LOGI(SC_TAG, "ISM6HG256 found and initialized");

        // Route DRDY to MCU via INT1 and wake polling task from ISR.
        ism6hg256_instance = this;
        pinMode(ISM6HG256_INT, INPUT);

        // Flush any pending DRDY by reading the data registers.
        // Without this, DRDY may already be HIGH before attachInterrupt
        // is called, meaning the first rising edge is missed and the
        // pin stays HIGH permanently (no more edges → no interrupts).
        {
            TR_ISM6HG256_AxesRaw_t dummy_axes = {};
            ism6hg256.Get_X_AxesRaw(&dummy_axes);
            ism6hg256.Get_HG_X_AxesRaw(&dummy_axes);
            ism6hg256.Get_G_AxesRaw(&dummy_axes);
            ESP_LOGI(SC_TAG, "ISM6 DRDY flushed (pin=%d)", digitalRead(ISM6HG256_INT));
        }

        attachInterrupt(ISM6HG256_INT, onISM6HG256Int1Trampoline, RISING);
    }

    ESP_LOGI(SC_TAG, "Initializing GNSS (RX=%d TX=%d)...", GNSS_RX, GNSS_TX);
    if (use_gnss)
    {
        if (!gnss_receiver.begin((uint8_t)GNSS_UPDATE_RATE,
                                 GNSS_RX,
                                 GNSS_TX,
                                 GNSS_RESET_N,
                                 GNSS_SAFEBOOT_N))
        {
            ESP_LOGE(SC_TAG, "GNSS initialization failed, stopping.");
            while (1) { delay(1000); }
        }
        ESP_LOGI(SC_TAG, "GNSS found and initialized");
    }

    // TODO Calibrate low and high g accel
    
    startPollingTask(imu_execution_core);
}

void SensorCollector::startPollingTask(uint8_t imu_execution_core)
{
    xTaskCreatePinnedToCore(
        pollIMUdata,         // Task function
        "Poll IMU Data",     // Task name
        8096,                // Stack size
        this,                // Task parameter (pass 'this' pointer)
        4,                   // Task priority (above I2C sender at 2)
        &pollIMUTaskHandle,  // Task handle
        imu_execution_core); // Core to run on

    xTaskCreatePinnedToCore(
        pollGNSSdata,        // Task function
        "Poll GNSS Data",    // Task name
        4096,                // Stack size (GNSS serial parsing only)
        this,                // Task parameter
        3,                   // Priority (above I2C sender at 2, below IMU at 4)
        &pollGNSSTaskHandle, // Task handle
        imu_execution_core); // Same core — keeps Serial1 single-threaded
}

void SensorCollector::pollIMUdata(void* parameter) 
{
    SensorCollector* self = static_cast<SensorCollector*>(parameter);
    ESP_LOGI(SC_TAG, "Starting pollIMUdata");

    while (true)
    {
        // ### Read Sensor Data ###
        const uint32_t iter_start_us = micros();

        // Block for sensor interrupt to avoid busy-spinning and WDT starvation.
        const uint32_t notify_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        if (notify_count > 0)
        {
            self->ism6_notify_wakes += notify_count;
        }
        else
        {
            self->ism6_notify_timeouts++;
        }

        // === ISM6HG256 FIRST — highest rate sensor, latency-critical ===
        // Use the ISR flag instead of an SPI DRDY register read.  The ISR
        // fires when ANY routed DRDY goes high.  We read all three channels
        // unconditionally — at the same ODR they're virtually synchronous,
        // and BDU prevents partial-word tearing if one channel lags by a
        // sample.  This eliminates the previous drdy_bits==0x07 gate that
        // caused 50 % of samples to be skipped when hg lagged by 1 cycle.
        // Fallback: if the ISR didn't fire but the DRDY pin is HIGH,
        // treat it as if the ISR fired.  This handles the case where the
        // initial rising edge was missed before attachInterrupt was called.
        if (self->use_ism6hg256 && !self->ism6_isr_fired &&
            digitalRead(self->ISM6HG256_INT))
        {
            self->ism6_isr_fired = true;
        }

        if (self->use_ism6hg256 && self->ism6_isr_fired)
        {
            self->ism6_isr_fired = false;

            const uint32_t ism6_t0 = micros();
            self->ism6_drdy_triplet_hits++;

            TR_ISM6HG256_AxesRaw_t lg_raw = {};
            TR_ISM6HG256_AxesRaw_t hg_raw = {};
            TR_ISM6HG256_AxesRaw_t g_raw = {};

            // Bulk read: gyro + lg accel + hg accel in one 24-byte SPI transaction
            (void)self->ism6hg256.Get_AllAxesRaw(&g_raw, &lg_raw, &hg_raw);

            if (xSemaphoreTake(self->ism6hg256DataSemaphore, 0) == pdTRUE)
            {
                self->start_time = micros();
                self->ism6hg256_data.time_us = self->start_time;

                self->ism6hg256_data.acc_low_raw.x = lg_raw.x;
                self->ism6hg256_data.acc_low_raw.y = lg_raw.y;
                self->ism6hg256_data.acc_low_raw.z = lg_raw.z;

                self->ism6hg256_data.acc_high_raw.x = hg_raw.x;
                self->ism6hg256_data.acc_high_raw.y = hg_raw.y;
                self->ism6hg256_data.acc_high_raw.z = hg_raw.z;

                self->ism6hg256_data.gyro_raw.x = g_raw.x - self->gyro_cal_x;
                self->ism6hg256_data.gyro_raw.y = g_raw.y - self->gyro_cal_y;
                self->ism6hg256_data.gyro_raw.z = g_raw.z - self->gyro_cal_z;

                // Track ISM6 timestamp gaps
                const uint32_t this_time = self->start_time;
                const uint32_t prev_time = self->pt_last_ism6_time_us;
                if (prev_time != 0)
                {
                    const uint32_t gap = this_time - prev_time;
                    if (gap > GAP_THRESHOLD_US)
                    {
                        self->pt_gap_count++;
                        if (gap > self->pt_gap_worst_us)
                        {
                            self->pt_gap_worst_us = gap;
                        }
                    }
                }
                self->pt_last_ism6_time_us = this_time;

                self->ism6hg256_data_ready = true;
                xSemaphoreGive(self->ism6hg256DataSemaphore);
            }

            const uint32_t ism6_elapsed = micros() - ism6_t0;
            if (ism6_elapsed > self->pt_ism6_read_max_us)
            {
                self->pt_ism6_read_max_us = ism6_elapsed;
            }
        }

        // === BMP585 — only process when interrupt pending ===
        if (self->use_bmp585)
        {
            uint32_t bmp_pending = 0;
            noInterrupts();
            bmp_pending = self->bmp585_irq_pending_count;
            if (bmp_pending > 0) self->bmp585_irq_pending_count = 0;
            interrupts();

            if (bmp_pending > 0)
            {
                const uint32_t bmp_t0 = micros();

                while (bmp_pending > 0)
                {
                    TR_BMP585::BmpCompFrame f = {};
                    if (self->bmp585.readCompFrame(f))
                    {
                        if (xSemaphoreTake(self->bmp585DataSemaphore, 0) == pdTRUE)
                        {
                            self->bmp585_data.time_us = f.t_us;
                            self->bmp585_data.temp_q16 = f.temp_q16;
                            self->bmp585_data.press_q6 = f.press_q6;
                            self->bmp585_data_ready = true;
                            xSemaphoreGive(self->bmp585DataSemaphore);
                        }
                    }
                    bmp_pending--;
                }

                const uint32_t bmp_elapsed = micros() - bmp_t0;
                if (bmp_elapsed > self->pt_bmp_max_us)
                {
                    self->pt_bmp_max_us = bmp_elapsed;
                }
            }
        }

        // === MMC5983MA — only do SPI work when interrupt pending or stall recovery needed ===
        // Skip entirely on boards where the IIS2MDC was detected at boot — the
        // MMC5983MA isn't populated, so polling and stall-recovery just burn
        // SPI cycles trying to talk to a chip that isn't there.
        if (self->use_mmc5983ma && !self->iis2mdc_active)
        {
            uint32_t mmc_pending = 0;
            noInterrupts();
            mmc_pending = self->mmc5983ma_irq_pending_count;
            if (mmc_pending > 0) self->mmc5983ma_irq_pending_count = 0;
            interrupts();

            self->mmc5983ma_loop_checks++;

            if (mmc_pending > 0)
            {
                const uint32_t mmc_t0 = micros();
                self->mmc5983ma_irq_path_hits += mmc_pending;
                self->mmc5983ma_process_hits++;

                uint32_t mx = 0;
                uint32_t my = 0;
                uint32_t mz = 0;

                // Read data FIRST (datasheet: check status → read → clear interrupt)
                if (self->mmc5983ma.readFieldsXYZ(&mx, &my, &mz))
                {
                    self->mmc5983ma_read_ok++;
                    if (xSemaphoreTake(self->mmc5983maDataSemaphore, 0) == pdTRUE)
                    {
                        self->mmc5983ma_data.time_us = micros();
                        self->mmc5983ma_data.mag_x = mx;
                        self->mmc5983ma_data.mag_y = my;
                        self->mmc5983ma_data.mag_z = mz;
                        self->mmc5983ma_data_ready = true;
                        xSemaphoreGive(self->mmc5983maDataSemaphore);
                    }
                    self->mmc_last_sample_time_us = micros();
                }
                else
                {
                    self->mmc5983ma_read_fail++;
                }

                // Clear interrupt AFTER reading data (datasheet order)
                if (self->mmc5983ma.clearMeasDoneInterrupt(0x03))
                {
                    self->mmc5983ma_clear_ok++;
                }
                else
                {
                    self->mmc5983ma_clear_fail++;
                }

                const uint32_t mmc_elapsed = micros() - mmc_t0;
                if (mmc_elapsed > self->pt_mmc_max_us)
                {
                    self->pt_mmc_max_us = mmc_elapsed;
                }
            }

            // Stall recovery — infrequent, only when no data for several periods
            const uint32_t now_us = micros();
            const bool mmc_stalled = ((int32_t)(now_us - self->mmc_last_sample_time_us) > (int32_t)self->mmc_stall_threshold_us);
            const bool recover_due = ((int32_t)(now_us - self->mmc_last_recover_time_us) > (int32_t)self->mmc_recover_cooldown_us);
            if (mmc_stalled && recover_due)
            {
                self->mmc5983ma_cmm_lost_hits++;
                self->mmc5983ma_recovery_attempts++;
                self->mmc_last_recover_time_us = now_us;

                const bool want_max_rate = (self->MMC5983MA_UPDATE_RATE >= 1000U);
                const uint16_t freq = want_max_rate ? 1000U
                                    : (self->MMC5983MA_UPDATE_RATE >= 200U) ? 200U
                                    : (self->MMC5983MA_UPDATE_RATE >= 100U) ? 100U
                                    : (self->MMC5983MA_UPDATE_RATE >= 50U) ? 50U
                                    : (self->MMC5983MA_UPDATE_RATE >= 20U) ? 20U
                                    : (self->MMC5983MA_UPDATE_RATE >= 10U) ? 10U
                                    : (self->MMC5983MA_UPDATE_RATE >= 1U) ? 1U : 0U;

                const uint16_t bw = want_max_rate ? 800U
                                  : (self->MMC5983MA_UPDATE_RATE >= 200U) ? 400U
                                  : 100U;
                bool recovered = true;
                recovered = recovered && self->mmc5983ma.setFilterBandwidth(bw);
                recovered = recovered && self->mmc5983ma.setContinuousModeFrequency(freq);
                recovered = recovered && self->mmc5983ma.enableContinuousMode();
                recovered = recovered && self->mmc5983ma.enableInterrupt();
                recovered = recovered && self->mmc5983ma.clearMeasDoneInterrupt(0x03);
                if (recovered)
                {
                    self->mmc5983ma_recovery_success++;
                }
            }
        }

        // === IIS2MDC — time-gated I2C poll at ODR (default 100 Hz) ===
        // No DRDY pin yet; we throttle reads to IIS2MDC_PERIOD_US so we don't
        // spam the I2C bus from the ~1 kHz polling task. BDU is on, so a
        // partial read across an internal sample boundary returns the prior
        // complete sample instead of tearing.
        if (self->iis2mdc_active)
        {
            const uint32_t iis2_now_us = micros();
            if ((int32_t)(iis2_now_us - self->iis2mdc_last_sample_us) >=
                (int32_t)IIS2MDC_PERIOD_US)
            {
                IIS2MDC_RawData iis2_raw = {};
                if (self->iis2mdc.readRawXYZ(&iis2_raw) == TR_IIS2MDC_OK)
                {
                    if (xSemaphoreTake(self->iis2mdcDataSemaphore, 0) == pdTRUE)
                    {
                        self->iis2mdc_data.time_us = iis2_now_us;
                        self->iis2mdc_data.mag_x = iis2_raw.x;
                        self->iis2mdc_data.mag_y = iis2_raw.y;
                        self->iis2mdc_data.mag_z = iis2_raw.z;
                        self->iis2mdc_data_ready = true;
                        xSemaphoreGive(self->iis2mdcDataSemaphore);
                    }
                    self->iis2mdc_last_sample_us = iis2_now_us;
                }
            }
        }

        // Track total iteration time (excluding the notification wait)
        const uint32_t iter_elapsed = micros() - iter_start_us;
        if (iter_elapsed > self->pt_iter_max_us)
        {
            self->pt_iter_max_us = iter_elapsed;
        }

        taskYIELD();
    }
}

void SensorCollector::pollGNSSdata(void* parameter)
{
    SensorCollector* self = static_cast<SensorCollector*>(parameter);
    ESP_LOGI(SC_TAG, "Starting pollGNSSdata (data-driven)");

    // Poll interval: check for new PVT at ~2× the navigation rate so we
    // never miss a fix.  E.g. 10 Hz nav → check every 50 ms.
    const TickType_t xPollInterval = pdMS_TO_TICKS(
        (1000U / self->GNSS_UPDATE_RATE) / 2U);

    while (true)
    {
        vTaskDelay(xPollInterval);

        if (!self->use_gnss) continue;

        const uint32_t gnss_t0 = micros();

        // Non-blocking: parse serial bytes, return true only if a
        // brand-new NAV-PVT message was fully received.
        GNSSData gnss_sample = {};
        const bool have_new = self->gnss_receiver.pollNewPVT(gnss_sample);

        const uint32_t gnss_elapsed = micros() - gnss_t0;
        self->pt_gnss_calls++;
        if (gnss_elapsed > self->pt_gnss_max_us)
        {
            self->pt_gnss_max_us = gnss_elapsed;
        }
        if (gnss_elapsed > 1000)  self->pt_gnss_over_1ms++;
        if (gnss_elapsed > 5000)  self->pt_gnss_over_5ms++;
        if (gnss_elapsed > 10000) self->pt_gnss_over_10ms++;

        if (have_new)
        {
            if (xSemaphoreTake(self->gnssDataSemaphore, 0) == pdTRUE)
            {
                self->gnss_data = gnss_sample;
                self->gnss_data_ready = true;
                xSemaphoreGive(self->gnssDataSemaphore);
            }
        }
    }
}

// Returns the latest sensor data if available, and marks the data as read so that new data can be collected
bool SensorCollector::getISM6HG256Data(ISM6HG256Data& ism6hg256_out)
{
    if (ism6hg256_data_ready && xSemaphoreTake(ism6hg256DataSemaphore, 0) == pdTRUE) 
    {
        ism6hg256_out = ism6hg256_data;
        xSemaphoreGive(ism6hg256DataSemaphore);
        ism6hg256_data_ready = false;
        return true;
    }
    return false;
}

bool SensorCollector::getBMP585Data(BMP585Data& bmp585_out)
{
    if (bmp585_data_ready && xSemaphoreTake(bmp585DataSemaphore, 0) == pdTRUE)
    {
        bmp585_out = bmp585_data;
        xSemaphoreGive(bmp585DataSemaphore);
        bmp585_data_ready = false;
        return true;
    }
    return false;
}

bool SensorCollector::getMMC5983MAData(MMC5983MAData& mmc5983ma_out)
{
    if (mmc5983ma_data_ready && xSemaphoreTake(mmc5983maDataSemaphore, 0) == pdTRUE)
    {
        mmc5983ma_out = mmc5983ma_data;
        xSemaphoreGive(mmc5983maDataSemaphore);
        mmc5983ma_data_ready = false;
        return true;
    }
    return false;
}

bool SensorCollector::getIIS2MDCData(IIS2MDCData& iis2mdc_out)
{
    if (iis2mdc_data_ready && xSemaphoreTake(iis2mdcDataSemaphore, 0) == pdTRUE)
    {
        iis2mdc_out = iis2mdc_data;
        xSemaphoreGive(iis2mdcDataSemaphore);
        iis2mdc_data_ready = false;
        return true;
    }
    return false;
}

bool SensorCollector::getGNSSData(GNSSData& gnss_out)
{
    if (gnss_data_ready && xSemaphoreTake(gnssDataSemaphore, 0) == pdTRUE)
    {
        gnss_out = gnss_data;
        xSemaphoreGive(gnssDataSemaphore);
        gnss_data_ready = false;
        return true;
    }
    return false;
}

void SensorCollector::getISM6HG256DebugSnapshot(ISM6HG256DebugSnapshot &snapshot_out) const
{
    uint8_t i = 0;
    snapshot_out.isr_hits = ism6_isr_hits;
    snapshot_out.notify_wakes = ism6_notify_wakes;
    snapshot_out.notify_timeouts = ism6_notify_timeouts;
    snapshot_out.drdy_lg_hits = ism6_drdy_lg_hits;
    snapshot_out.drdy_hg_hits = ism6_drdy_hg_hits;
    snapshot_out.drdy_g_hits = ism6_drdy_g_hits;
    snapshot_out.drdy_triplet_hits = ism6_drdy_triplet_hits;
    for (i = 0; i < 8; i++)
    {
        snapshot_out.drdy_histogram[i] = ism6_drdy_histogram[i];
    }
}

void SensorCollector::getMMC5983MADebugSnapshot(MMC5983MADebugSnapshot &snapshot_out) const
{
    snapshot_out.isr_hits = mmc5983ma_isr_hits;
    snapshot_out.loop_checks = mmc5983ma_loop_checks;
    snapshot_out.irq_path_hits = mmc5983ma_irq_path_hits;
    snapshot_out.status_done_hits = mmc5983ma_status_done_hits;
    snapshot_out.status_not_ready_hits = mmc5983ma_status_not_ready_hits;
    snapshot_out.status_read_failures = mmc5983ma_status_read_failures;
    snapshot_out.process_hits = mmc5983ma_process_hits;
    snapshot_out.clear_ok = mmc5983ma_clear_ok;
    snapshot_out.clear_fail = mmc5983ma_clear_fail;
    snapshot_out.read_ok = mmc5983ma_read_ok;
    snapshot_out.read_fail = mmc5983ma_read_fail;
    snapshot_out.rearm_ok = mmc5983ma_rearm_ok;
    snapshot_out.rearm_fail = mmc5983ma_rearm_fail;
    snapshot_out.cmm_lost_hits = mmc5983ma_cmm_lost_hits;
    snapshot_out.recovery_attempts = mmc5983ma_recovery_attempts;
    snapshot_out.recovery_success = mmc5983ma_recovery_success;
    snapshot_out.last_status = mmc5983ma_last_status;
    snapshot_out.last_ctrl0 = mmc5983ma_last_ctrl0;
    snapshot_out.last_ctrl1 = mmc5983ma_last_ctrl1;
    snapshot_out.last_ctrl2 = mmc5983ma_last_ctrl2;
}

void SensorCollector::getPollTimingSnapshot(PollTimingSnapshot &snapshot_out) const
{
    snapshot_out.poll_iter_max_us = pt_iter_max_us;
    snapshot_out.gnss_max_us     = pt_gnss_max_us;
    snapshot_out.bmp_max_us      = pt_bmp_max_us;
    snapshot_out.mmc_max_us      = pt_mmc_max_us;
    snapshot_out.ism6_read_max_us = pt_ism6_read_max_us;
    snapshot_out.gap_count       = pt_gap_count;
    snapshot_out.gap_worst_us    = pt_gap_worst_us;
    snapshot_out.gnss_calls      = pt_gnss_calls;
    snapshot_out.gnss_over_1ms   = pt_gnss_over_1ms;
    snapshot_out.gnss_over_5ms   = pt_gnss_over_5ms;
    snapshot_out.gnss_over_10ms  = pt_gnss_over_10ms;
}

void SensorCollector::resetPollTimingSnapshot()
{
    pt_iter_max_us    = 0;
    pt_gnss_max_us    = 0;
    pt_bmp_max_us     = 0;
    pt_mmc_max_us     = 0;
    pt_ism6_read_max_us = 0;
    pt_gap_count      = 0;
    pt_gap_worst_us   = 0;
    pt_gnss_calls     = 0;
    pt_gnss_over_1ms  = 0;
    pt_gnss_over_5ms  = 0;
    pt_gnss_over_10ms = 0;
}

// --- Pad calibration ---
// Reads 1000 raw samples over ~10 seconds.
//  • Gyro: averages to compute zero-rate offset (subtracted in polling task).
//  • Accel: cross-calibrates high-g against low-g to compute body-frame bias
//    (subtracted by SensorConverter during conversion).
//  • Computes gravity magnitude from low-g for sanity check.
// Must be called when the rocket is stationary (e.g. on the launch pad).
void SensorCollector::calibrateGyro(float rotation_z_deg)
{
    if (!use_ism6hg256)
    {
        ESP_LOGW(SC_TAG, "No ISM6HG256 — skipping calibration");
        return;
    }

    ESP_LOGI(SC_TAG, "Calibrating sensors (1000 samples)...");

    // Suspend the IMU polling task to prevent SPI bus contention during calibration
    if (pollIMUTaskHandle != nullptr) {
        vTaskSuspend(pollIMUTaskHandle);
    }

    // Gyro accumulators
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    // Accel accumulators (raw LSBs, int32 is sufficient for 1000 * int16)
    int32_t sum_lg_x = 0, sum_lg_y = 0, sum_lg_z = 0;
    int32_t sum_hg_x = 0, sum_hg_y = 0, sum_hg_z = 0;

    const int num_samples = 1000;
    int gyro_count = 0;
    int accel_count = 0;

    for (int i = 0; i < num_samples; i++)
    {
        // Bulk read gyro + low-g accel + high-g accel
        TR_ISM6HG256_AxesRaw_t g_raw, lg_raw, hg_raw;
        if (ism6hg256.Get_AllAxesRaw(&g_raw, &lg_raw, &hg_raw) == TR_ISM6HG256_OK)
        {
            sum_gx += g_raw.x;
            sum_gy += g_raw.y;
            sum_gz += g_raw.z;
            gyro_count++;

            sum_lg_x += lg_raw.x;
            sum_lg_y += lg_raw.y;
            sum_lg_z += lg_raw.z;
            sum_hg_x += hg_raw.x;
            sum_hg_y += hg_raw.y;
            sum_hg_z += hg_raw.z;
            accel_count++;
        }

        delay(10);
    }

    // --- Gyro offsets ---
    if (gyro_count > 0)
    {
        gyro_cal_x = (int16_t)(sum_gx / gyro_count);
        gyro_cal_y = (int16_t)(sum_gy / gyro_count);
        gyro_cal_z = (int16_t)(sum_gz / gyro_count);
        ESP_LOGI(SC_TAG, "Gyro offsets: %d, %d, %d  (%d samples)",
                      gyro_cal_x, gyro_cal_y, gyro_cal_z, gyro_count);
    }
    else
    {
        ESP_LOGW(SC_TAG, "No gyro samples collected");
    }

    // --- Accel cross-calibration ---
    if (accel_count > 0)
    {
        // ISM6HG256 sensitivity (must match SensorConverter config):
        //   low-g  ±16g  → 16*1000/32768 mg/LSB → * 1e-3 * 9.80665 m/s²/LSB
        //   high-g ±256g → 256*1000/32768 mg/LSB → * 1e-3 * 9.80665 m/s²/LSB
        static constexpr float g_ms2 = 9.80665f;
        static constexpr float lg_ms2_per_lsb = (16.0f * 1000.0f / 32768.0f) * 1e-3f * g_ms2;
        static constexpr float hg_ms2_per_lsb = (256.0f * 1000.0f / 32768.0f) * 1e-3f * g_ms2;

        // Average raw → SI (sensor frame)
        const float avg_lg_x = ((float)sum_lg_x / accel_count) * lg_ms2_per_lsb;
        const float avg_lg_y = ((float)sum_lg_y / accel_count) * lg_ms2_per_lsb;
        const float avg_lg_z = ((float)sum_lg_z / accel_count) * lg_ms2_per_lsb;

        const float avg_hg_x = ((float)sum_hg_x / accel_count) * hg_ms2_per_lsb;
        const float avg_hg_y = ((float)sum_hg_y / accel_count) * hg_ms2_per_lsb;
        const float avg_hg_z = ((float)sum_hg_z / accel_count) * hg_ms2_per_lsb;

        // Rotate both to body frame (same rotation for all ISM6HG256 channels)
        const float rot_rad = rotation_z_deg * (PI / 180.0f);
        const float c = cosf(rot_rad);
        const float s = sinf(rot_rad);

        const float lg_bx = avg_lg_x * c - avg_lg_y * s;
        const float lg_by = avg_lg_x * s + avg_lg_y * c;
        const float lg_bz = avg_lg_z;

        const float hg_bx = avg_hg_x * c - avg_hg_y * s;
        const float hg_by = avg_hg_x * s + avg_hg_y * c;
        const float hg_bz = avg_hg_z;

        // Bias = high_g - low_g in body frame
        hg_bias_x = hg_bx - lg_bx;
        hg_bias_y = hg_by - lg_by;
        hg_bias_z = hg_bz - lg_bz;

        // Gravity magnitude from low-g (body frame magnitude = sensor frame magnitude)
        cal_gravity_mag = sqrtf(avg_lg_x * avg_lg_x +
                                avg_lg_y * avg_lg_y +
                                avg_lg_z * avg_lg_z);

        ESP_LOGI(SC_TAG, "Low-g  avg (body): %.3f, %.3f, %.3f m/s²",
                      (double)lg_bx, (double)lg_by, (double)lg_bz);
        ESP_LOGI(SC_TAG, "High-g avg (body): %.3f, %.3f, %.3f m/s²",
                      (double)hg_bx, (double)hg_by, (double)hg_bz);
        ESP_LOGI(SC_TAG, "HG bias:           %.3f, %.3f, %.3f m/s²",
                      (double)hg_bias_x, (double)hg_bias_y, (double)hg_bias_z);
        ESP_LOGI(SC_TAG, "Gravity magnitude: %.3f m/s²  (%d samples)",
                      (double)cal_gravity_mag, accel_count);
    }
    else
    {
        ESP_LOGW(SC_TAG, "No accel samples collected");
    }

    // Resume the IMU polling task
    if (pollIMUTaskHandle != nullptr) {
        vTaskResume(pollIMUTaskHandle);
    }
}

void IRAM_ATTR SensorCollector::onISM6HG256Int1Trampoline()
{
    if (ism6hg256_instance)
    {
        ism6hg256_instance->onISM6HG256Int1();
    }
}

void IRAM_ATTR SensorCollector::onISM6HG256Int1()
{
    ism6_isr_fired = true;
    ism6_isr_hits++;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pollIMUTaskHandle != nullptr)
    {
        vTaskNotifyGiveFromISR(pollIMUTaskHandle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR SensorCollector::onBMP585IntTrampoline()
{
    if (bmp585_instance)
    {
        bmp585_instance->onBMP585Int();
    }
}

void IRAM_ATTR SensorCollector::onBMP585Int()
{
    bmp585_isr_hits++;
    bmp585_irq_pending_count++;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pollIMUTaskHandle != nullptr)
    {
        vTaskNotifyGiveFromISR(pollIMUTaskHandle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR SensorCollector::onMMC5983MAIntTrampoline()
{
    if (mmc5983ma_instance)
    {
        mmc5983ma_instance->onMMC5983MAInt();
    }
}

void IRAM_ATTR SensorCollector::onMMC5983MAInt()
{
    mmc5983ma_isr_hits++;
    mmc5983ma_irq_pending_count++;

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (pollIMUTaskHandle != nullptr)
    {
        vTaskNotifyGiveFromISR(pollIMUTaskHandle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}
