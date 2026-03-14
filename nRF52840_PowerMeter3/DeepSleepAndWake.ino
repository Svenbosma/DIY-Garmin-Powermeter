void setupWakeUpInterrupt()
{
  // Reinitialize the IMU in a low-power accelerometer-only wake-on-motion mode.
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.accelEnabled = 0;
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.fifoEnd();
  delay(1000);
  myIMU.begin();

  // Follow the application-note sequence to avoid spurious wake interrupts.

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00); // No duration
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x08); // Set wake-up threshold
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);    // Enable interrupts and apply slope filter; latch mode disabled
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x70);    // Turn on the accelerometer
  delay(4);                                                // Delay time per application note
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x10);    // ODR_XL = 26 Hz, FS_XL = ±2 g
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);     // High-performance operating mode disabled for accelerometer
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);     // Wake-up interrupt driven to INT1 pin

  // Clear any pending wake source before arming GPIO sense.
  uint8_t dummy;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);

  // System-off wake uses GPIO sense only; no ISR is required here.
	pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);

  return;
}

void goToSystemOff() {
  Bluefruit.Advertising.stop();
  Bluefruit.autoConnLed(false);
  sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE); // Disable DC-DC converter

  digitalWrite(HX_SCK, HIGH);
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_RED, OUTPUT);  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);digitalWrite(LED_GREEN, HIGH);

  uint8_t int1 = 0;
  myIMU.readRegister(&int1, LSM6DS3_ACC_GYRO_INT1_CTRL);
  int1 &= ~0x08;   // Clear FIFO watermark routing on INT1.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, int1);
  detachInterrupt(digitalPinToInterrupt(MOTION_INT_PIN));
  setupWakeUpInterrupt();

  // Clear wake source again just before enabling level-sensitive GPIO wake.
  uint8_t dummy;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
  delay(10);

  pinMode(MOTION_INT_PIN, INPUT_PULLDOWN);
  delay(50);

  nrf_gpio_cfg_sense_input(
    g_ADigitalPinMap[MOTION_INT_PIN],
    NRF_GPIO_PIN_PULLDOWN,
    NRF_GPIO_PIN_SENSE_HIGH
  );
  delay(50);

  logPrintln("Entering SYSTEMOFF...");
  uint8_t sd_en = 0;
  (void) sd_softdevice_is_enabled(&sd_en);
  if (sd_en) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
  }
}
