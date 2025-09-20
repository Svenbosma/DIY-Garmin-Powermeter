// BLE
BLEDis bledis;
BLEService cyclingPowerService = BLEService(0x1818);
BLECharacteristic cpMeasurementChar = BLECharacteristic(0x2A63);
BLECharacteristic cpFeatureChar = BLECharacteristic(0x2A65);
BLEService batteryService = BLEService(0x180F);
BLECharacteristic batteryLevelChar = BLECharacteristic(0x2A19);
BLEUart bleUart;

void setupBLE() {
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DIY-Powermeter");

  bledis.setManufacturer("DIY");
  bledis.setModel("Powermeter");
  bledis.begin();

  cyclingPowerService.begin();
  cpMeasurementChar.setProperties(CHR_PROPS_NOTIFY);
  cpMeasurementChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpMeasurementChar.setFixedLen(8);
  cpMeasurementChar.begin();

  cpFeatureChar.setProperties(CHR_PROPS_READ);
  cpFeatureChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpFeatureChar.setFixedLen(4);
  cpFeatureChar.begin();
  uint32_t features = 0;
  cpFeatureChar.write(&features, sizeof(features));

  batteryService.begin();
  batteryLevelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryLevelChar.setFixedLen(1);
  batteryLevelChar.begin();
  uint8_t initialBatt = 100;
  batteryLevelChar.write8(initialBatt);

  bleUart.begin();

  Bluefruit.Advertising.addService(cyclingPowerService);
  Bluefruit.Advertising.addService(batteryService);
  Bluefruit.Advertising.addService(bleUart);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start();

  logPrintln("BLE advertising started (Cycling Power + UART + Battery ready)");
}

void sendCyclingPowerMeasurement(int16_t powerWatts) {
  uint8_t buf[8] = {0};
  buf[0] = 0x20; buf[1] = 0x00;
  buf[2] = powerWatts & 0xFF;
  buf[3] = (powerWatts >> 8) & 0xFF;
  buf[4] = cumulativeCrankRevs & 0xFF;
  buf[5] = (cumulativeCrankRevs >> 8) & 0xFF;
  buf[6] = lastCrankEventTime & 0xFF;
  buf[7] = (lastCrankEventTime >> 8) & 0xFF;
  cpMeasurementChar.notify(buf, sizeof(buf));
}
