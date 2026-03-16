
#include <TR_Sensor_Collector.h>

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
      GNSS_UPDATE_RATE(GNSS_UPDATE_RATE),
      GNSS_RX(GNSS_RX),
      GNSS_TX(GNSS_TX),
      GNSS_RESET_N(GNSS_RESET_N),
      GNSS_SAFEBOOT_N(GNSS_SAFEBOOT_N),
      use_bmp585(use_bmp585),
      use_mmc5983ma(use_mmc5983ma),
      use_gnss(use_gnss),
      use_ism6hg256(use_ism6hg256),
      spi(spi),
      spi_speed(spi_speed),
      bmp585(this->spi, BMP585_CS, SPISettings(1000000, MSBFIRST, SPI_MODE0)),
      mmc5983ma(this->spi, MMC5983MA_CS, SPISettings(10000000, MSBFIRST, SPI_MODE0)),
      ism6hg256(&this->spi, ISM6HG256_CS, spi_speed),
      gyro_cal_x(0), gyro_cal_y(0), gyro_cal_z(0) {}

void SensorCollector::begin(uint8_t imu_execution_core) 
{
    uint8_t i = 0;

    // Data ready flag init
    ism6hg256_data_ready = false;
    bmp585_data_ready = false;
    mmc5983ma_data_ready = false;
    gnss_data_ready = false;
    bmp585_irq_pending_count = 0;
    mmc5983ma_irq_pending_count = 0;
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
        Serial.println("Failed to create ISM6HG256 data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(ism6hg256DataSemaphore);

    bmp585DataSemaphore = xSemaphoreCreateBinary();
    if (bmp585DataSemaphore == NULL)
    {
        Serial.println("Failed to create BMP585 data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(bmp585DataSemaphore);

    mmc5983maDataSemaphore = xSemaphoreCreateBinary();
    if (mmc5983maDataSemaphore == NULL)
    {
        Serial.println("Failed to create MMC5983MA data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(mmc5983maDataSemaphore);

    gnssDataSemaphore = xSemaphoreCreateBinary();
    if (gnssDataSemaphore == NULL)
    {
        Serial.println("Failed to create GNSS data semaphore!");
        while (1) delay(1000);
    }
    xSemaphoreGive(gnssDataSemaphore);

    // ### Initialize Sensors ###
    if (use_bmp585)
    {
        uint32_t bmp_retry_count = 0;
        while (!bmp585.begin())
        {
            Serial.print("BMP585 initialization failed, chip ID 0x");
            Serial.println(bmp585.readChipIdCached(), HEX);
            bmp_retry_count++;

            // Warm-reset recovery path: after MCU flash, sensor may still be
            // in active mode and not respond cleanly until a soft reset.
            const bool reset_ok = bmp585.forceSoftResetRaw();
            Serial.print("BMP585 soft-reset recovery ");
            Serial.println(reset_ok ? "OK" : "pending");

            // Re-drive CS high between retries to avoid bus ambiguity.
            pinMode(BMP585_CS, OUTPUT);
            digitalWrite(BMP585_CS, HIGH);
            delay(10);

            if ((bmp_retry_count % 5U) == 0U)
            {
                Serial.print("BMP585 retry count: ");
                Serial.println(bmp_retry_count);
            }
            delay(500);
        }

        Serial.print("BMP585 OK. chip_id = 0x");
        Serial.println(bmp585.readChipId(), HEX);

        bool bmp_ok = true;
        bmp_ok = bmp_ok && bmp585.setPowerMode(TR_BMP585::PowerMode::Continuous);
        bmp_ok = bmp_ok && bmp585.setTemperatureOversampling(TR_BMP585::Oversampling::x1);
        bmp_ok = bmp_ok && bmp585.setPressureOversampling(TR_BMP585::Oversampling::x1);
        bmp_ok = bmp_ok && bmp585.setIirFilter(TR_BMP585::IirCoeff::Bypass,
                                               TR_BMP585::IirCoeff::Bypass);
        bmp_ok = bmp_ok && bmp585.enableDataReadyInterrupt(true, false, true, false);

        if (!bmp_ok)
        {
            Serial.println("BMP585 configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        Serial.println("BMP585 continuous mode active; throughput set by OSR/filter (ODR ignored in this mode).");

        bmp585_instance = this;
        pinMode(BMP585_INT, INPUT);
        attachInterrupt(BMP585_INT, onBMP585IntTrampoline, RISING);
    }

    if (use_mmc5983ma)
    {
        while (!mmc5983ma.begin())
        {
            Serial.println("MMC5983MA did not respond. Retrying...");
            delay(500);
            (void)mmc5983ma.softReset();
            delay(500);
        }

        (void)mmc5983ma.softReset();
        (void)mmc5983ma.performSetOperation();
        delay(10);
        (void)mmc5983ma.performResetOperation();
        delay(10);

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
            Serial.println("MMC5983MA configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        Serial.println("MMC5983MA found and initialized");
        if (MMC5983MA_UPDATE_RATE >= 1000U)
        {
            Serial.println("MMC5983MA max-rate mode: BW=800Hz, CM_FREQ=1000Hz, CMM_EN=1.");
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

    if(use_ism6hg256)
    {
        uint8_t whoami = 0;

        if (ism6hg256.ReadWhoAmI(&whoami) != TR_ISM6HG256_OK)
        {
            Serial.println("ISM6HG256 WHOAMI read failed, stopping.");
            while (1) { delay(1000); }
        }
        if (whoami != ISM6HG256X_ID)
        {
            Serial.print("ISM6HG256 WHOAMI mismatch, got 0x");
            Serial.print(whoami, HEX);
            Serial.print(" expected 0x");
            Serial.println(ISM6HG256X_ID, HEX);
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
            Serial.println("ISM6HG256 initialization/configuration failed, stopping.");
            while (1) { delay(1000); }
        }

        Serial.println("ISM6HG256 found and initialized");

        // Route DRDY to MCU via INT1 and wake polling task from ISR.
        ism6hg256_instance = this;
        pinMode(ISM6HG256_INT, INPUT);
        attachInterrupt(ISM6HG256_INT, onISM6HG256Int1Trampoline, RISING);
    }

    if (use_gnss)
    {
        if (!gnss_receiver.begin((uint8_t)GNSS_UPDATE_RATE,
                                 GNSS_RX,
                                 GNSS_TX,
                                 GNSS_RESET_N,
                                 GNSS_SAFEBOOT_N))
        {
            Serial.println("GNSS initialization failed, stopping.");
            while (1) { delay(1000); }
        }
        Serial.println("GNSS found and initialized");
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
        3,                   // Task priority
        &pollIMUTaskHandle,  // Task handle
        imu_execution_core); // Core to run on

    xTaskCreatePinnedToCore(
        pollGNSSdata,        // Task function
        "Poll GNSS Data",    // Task name
        4096,                // Stack size (GNSS serial parsing only)
        this,                // Task parameter
        2,                   // Priority (below IMU at 3)
        &pollGNSSTaskHandle, // Task handle
        imu_execution_core); // Same core — keeps Serial1 single-threaded
}

void SensorCollector::pollIMUdata(void* parameter) 
{
    SensorCollector* self = static_cast<SensorCollector*>(parameter);
    Serial.println("Starting pollIMUdata");

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

        if (self->use_bmp585)
        {
            const uint32_t bmp_t0 = micros();

            uint32_t bmp_pending = 0;
            noInterrupts();
            bmp_pending = self->bmp585_irq_pending_count;
            self->bmp585_irq_pending_count = 0;
            interrupts();

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

        if (self->use_mmc5983ma)
        {
            const uint32_t mmc_t0 = micros();
            self->mmc5983ma_loop_checks++;

            uint32_t mmc_pending = 0;
            noInterrupts();
            mmc_pending = self->mmc5983ma_irq_pending_count;
            self->mmc5983ma_irq_pending_count = 0;
            interrupts();

            uint8_t mmc_status = 0;
            const bool status_ok = self->mmc5983ma.readStatus(&mmc_status);
            const bool status_done = status_ok && ((mmc_status & 0x01U) != 0U);

            if (status_ok)
            {
                self->mmc5983ma_last_status = mmc_status;
                if (status_done)
                {
                    self->mmc5983ma_status_done_hits++;
                }
                else
                {
                    self->mmc5983ma_status_not_ready_hits++;
                }
            }
            else
            {
                self->mmc5983ma_status_read_failures++;
            }

            if (mmc_pending > 0)
            {
                self->mmc5983ma_irq_path_hits += mmc_pending;
            }

            if ((mmc_pending > 0) || status_done)
            {
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

                // NOTE: enableInterrupt() removed - INT_MEAS_DONE_EN stays armed in CMM mode.
                // Re-arming per sample was unnecessary and may have disrupted CMM stability.
                // Counters (rearm_ok/rearm_fail) kept for compatibility but no longer incremented.
            }

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

            const uint32_t mmc_elapsed = micros() - mmc_t0;
            if (mmc_elapsed > self->pt_mmc_max_us)
            {
                self->pt_mmc_max_us = mmc_elapsed;
            }
        }

        // ISM6HG256 low-g, high-g, and gyro are read together when all are ready
        if (self->use_ism6hg256)
        {
            ism6hg256x_data_ready_t drdy = {};
            if (self->ism6hg256.Get_DRDY_Status(&drdy) != TR_ISM6HG256_OK)
            {
                taskYIELD();
                continue;
            }

            const uint8_t drdy_bits = (drdy.drdy_xl ? 0x01 : 0x00)
                                    | (drdy.drdy_hgxl ? 0x02 : 0x00)
                                    | (drdy.drdy_gy ? 0x04 : 0x00);

            if (drdy.drdy_xl) self->ism6_drdy_lg_hits++;
            if (drdy.drdy_hgxl) self->ism6_drdy_hg_hits++;
            if (drdy.drdy_gy) self->ism6_drdy_g_hits++;
            self->ism6_drdy_histogram[drdy_bits & 0x07]++;

            // Read when all three are ready
            if (drdy_bits == 0x07)
            {
                const uint32_t ism6_t0 = micros();
                self->ism6_drdy_triplet_hits++;

                TR_ISM6HG256_AxesRaw_t lg_raw = {};
                TR_ISM6HG256_AxesRaw_t hg_raw = {};
                TR_ISM6HG256_AxesRaw_t g_raw = {};

                (void)self->ism6hg256.Get_X_AxesRaw(&lg_raw);
                (void)self->ism6hg256.Get_HG_X_AxesRaw(&hg_raw);
                (void)self->ism6hg256.Get_G_AxesRaw(&g_raw);

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
    Serial.println("Starting pollGNSSdata");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / self->GNSS_UPDATE_RATE);

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (!self->use_gnss) continue;

        const uint32_t gnss_t0 = micros();

        GNSSData gnss_sample = {};
        self->gnss_receiver.getGNSSData(gnss_sample);

        const uint32_t gnss_elapsed = micros() - gnss_t0;
        self->pt_gnss_calls++;
        if (gnss_elapsed > self->pt_gnss_max_us)
        {
            self->pt_gnss_max_us = gnss_elapsed;
        }
        if (gnss_elapsed > 1000)  self->pt_gnss_over_1ms++;
        if (gnss_elapsed > 5000)  self->pt_gnss_over_5ms++;
        if (gnss_elapsed > 10000) self->pt_gnss_over_10ms++;

        if (xSemaphoreTake(self->gnssDataSemaphore, 0) == pdTRUE)
        {
            self->gnss_data = gnss_sample;
            self->gnss_data_ready = true;
            xSemaphoreGive(self->gnssDataSemaphore);
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
        Serial.println("[CAL] No ISM6HG256 — skipping calibration");
        return;
    }

    Serial.println("[CAL] Calibrating sensors (1000 samples)...");

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
        // Read gyro
        TR_ISM6HG256_AxesRaw_t g_raw;
        if (ism6hg256.Get_G_AxesRaw(&g_raw) == TR_ISM6HG256_OK)
        {
            sum_gx += g_raw.x;
            sum_gy += g_raw.y;
            sum_gz += g_raw.z;
            gyro_count++;
        }

        // Read low-g and high-g accel
        TR_ISM6HG256_AxesRaw_t lg_raw, hg_raw;
        bool lg_ok = (ism6hg256.Get_X_AxesRaw(&lg_raw) == TR_ISM6HG256_OK);
        bool hg_ok = (ism6hg256.Get_HG_X_AxesRaw(&hg_raw) == TR_ISM6HG256_OK);
        if (lg_ok && hg_ok)
        {
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
        Serial.printf("[CAL] Gyro offsets: %d, %d, %d  (%d samples)\n",
                      gyro_cal_x, gyro_cal_y, gyro_cal_z, gyro_count);
    }
    else
    {
        Serial.println("[CAL] WARNING: no gyro samples collected");
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

        Serial.printf("[CAL] Low-g  avg (body): %.3f, %.3f, %.3f m/s²\n",
                      (double)lg_bx, (double)lg_by, (double)lg_bz);
        Serial.printf("[CAL] High-g avg (body): %.3f, %.3f, %.3f m/s²\n",
                      (double)hg_bx, (double)hg_by, (double)hg_bz);
        Serial.printf("[CAL] HG bias:           %.3f, %.3f, %.3f m/s²\n",
                      (double)hg_bias_x, (double)hg_bias_y, (double)hg_bias_z);
        Serial.printf("[CAL] Gravity magnitude: %.3f m/s²  (%d samples)\n",
                      (double)cal_gravity_mag, accel_count);
    }
    else
    {
        Serial.println("[CAL] WARNING: no accel samples collected");
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
