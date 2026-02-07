void setupWakeOnMotion() {
  // accel: 26Hz, ±2g, low power friendly
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);//

  // gyro OFF
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);

  // enable embedded functions
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL9_XL, 0x07);

  // wake-up interrupt enable
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);//

  // threshold + duration tuning
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x08); //
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x01); //

  // route wake-up interrupt to INT1
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);

  // clear latched source
  uint8_t dummy;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
}

void setupWakeUpInterrupt()
{
  // Start with LSM6DS3 in disabled to save power
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.accelEnabled = 0;
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.fifoEnd();
  delay(1000);
  myIMU.begin();

  // Set up the accelerometer for Wake-up interrupt.
  // Per the application note, use a two step set up to avoid spurious interrupts
  // Set up values are from the application note, and then adjusted for minimum power

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00); // No duration
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x02); // Set wake-up threshold
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);    // Enable interrupts and apply slope filter; latch mode disabled
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x70);    // Turn on the accelerometer
                                                           // ODR_XL = 833 Hz, FS_XL = ±2 g
  delay(4);                                                // Delay time per application note
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0xB0);    // ODR_XL = 1.6 Hz
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);     // High-performance operating mode disabled for accelerometer
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);     // Wake-up interrupt driven to INT1 pin

  // Set up the sense mechanism to generate the DETECT signal to wake from system_off
  // No need to attach a handler, if just waking with the GPIO input.
	pinMode(PIN_LSM6DS3TR_C_INT1, INPUT_PULLDOWN_SENSE);

  return;
}

void goToSystemOff() {
  Bluefruit.Advertising.stop();
  Bluefruit.autoConnLed(false);

  digitalWrite(HX_SCK, HIGH);
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_RED, OUTPUT);  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);digitalWrite(LED_GREEN, HIGH);

  uint8_t int1 = 0;
  myIMU.readRegister(&int1, LSM6DS3_ACC_GYRO_INT1_CTRL);
  int1 &= ~0x08;   // clear bit 3
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, int1);
  detachInterrupt(digitalPinToInterrupt(MOTION_INT_PIN));
  setupWakeUpInterrupt();


  pinMode(MOTION_INT_PIN, INPUT_PULLDOWN);
  delay(50);

  nrf_gpio_cfg_sense_input(
    g_ADigitalPinMap[MOTION_INT_PIN],
    NRF_GPIO_PIN_PULLDOWN,
    NRF_GPIO_PIN_SENSE_HIGH
  );
  delay(50);

  logPrintln("Entering SYSTEMOFF...");
  NRF_POWER->SYSTEMOFF = 1;
}
