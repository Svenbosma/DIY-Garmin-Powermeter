void setupWakeOnMotion() {
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL9_XL, 0x07);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x80);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x08);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x01);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
}

void goToSystemOff() {
  digitalWrite(HX_SCK, HIGH);
  pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, HIGH);
  pinMode(LED_RED, OUTPUT);  digitalWrite(LED_RED, HIGH);
  pinMode(LED_GREEN, OUTPUT);digitalWrite(LED_GREEN, HIGH);

  setupWakeOnMotion();

  uint8_t dummy;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);

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
