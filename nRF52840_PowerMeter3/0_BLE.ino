// BLE services exposed by the powermeter.
BLEService cyclingPowerService = BLEService(0x1818);

BLECharacteristic cpMeasurementChar = BLECharacteristic(0x2A63);
BLECharacteristic cpFeatureChar     = BLECharacteristic(0x2A65);
BLECharacteristic cpControlPointChar = BLECharacteristic(0x2A66);


BLEService batteryService = BLEService(0x180F);
BLECharacteristic batteryLevelChar = BLECharacteristic(0x2A19);

// Nordic UART is used for calibration, tare and debug messages.
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic uartTXChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic uartRXChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

void cpControlPointWriteCallback(uint16_t conn_hdl,
                                 BLECharacteristic* chr,
                                 uint8_t* data,
                                 uint16_t len);


void setupBLE() {
  Bluefruit.begin(1, 0);  // Peripheral only
  Bluefruit.autoConnLed(true);
  Bluefruit.setTxPower(0);
  Bluefruit.setName("DIY-Powermeter");

  Bluefruit.Periph.setConnInterval(320, 640); // 400-800 ms
  Bluefruit.Periph.setConnSupervisionTimeout(4000);
  Bluefruit.Periph.setConnSlaveLatency(4);

  // Cycling Power Service
  cyclingPowerService.begin();

  cpMeasurementChar.setProperties(CHR_PROPS_NOTIFY);
  cpMeasurementChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpMeasurementChar.setFixedLen(8);
  cpMeasurementChar.begin();

  cpFeatureChar.setProperties(CHR_PROPS_READ);
  cpFeatureChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cpFeatureChar.setFixedLen(4);
  cpFeatureChar.begin();

  uint32_t features =
  (1 << 5) |  // Crank Revolution Data Supported
  (1 << 0);   // Pedal Power Balance Supported (dummy, ok)

  cpFeatureChar.write((uint8_t*)&features, 4);

  // Control Point is used by head units for zero-offset requests.
  cpControlPointChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
  cpControlPointChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cpControlPointChar.setMaxLen(20);
  cpControlPointChar.setWriteCallback(cpControlPointWriteCallback);
  cpControlPointChar.begin();


  // Battery Service
  batteryService.begin();

  batteryLevelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  batteryLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  batteryLevelChar.setFixedLen(1);
  batteryLevelChar.begin();

  uint8_t batt = 100;
  batteryLevelChar.write(&batt, 1);

  // UART Service
  uartService.begin();

  uartTXChar.setProperties(CHR_PROPS_NOTIFY);
  uartTXChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  uartTXChar.setMaxLen(20);
  uartTXChar.begin();

  uartRXChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  uartRXChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  uartRXChar.setMaxLen(20);
  uartRXChar.setWriteCallback(uartRXWriteCallback);
  uartRXChar.begin();


  // Advertising payload
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.addService(cyclingPowerService);
  Bluefruit.Advertising.addService(batteryService);

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

}

void sendCyclingPowerMeasurement(int16_t powerWatts,
                                 uint16_t cumulativeCrankRevs,
                                 uint16_t lastCrankEventTime) {
  uint8_t buf[8] = {0};

  // Flags: crank revolution data present (bit 5).
  buf[0] = 0x20;
  buf[1] = 0x00;

  buf[2] = powerWatts & 0xFF;
  buf[3] = (powerWatts >> 8) & 0xFF;

  buf[4] = cumulativeCrankRevs & 0xFF;
  buf[5] = (cumulativeCrankRevs >> 8) & 0xFF;

  buf[6] = lastCrankEventTime & 0xFF;
  buf[7] = (lastCrankEventTime >> 8) & 0xFF;

  if (cpMeasurementChar.notifyEnabled()) {
    cpMeasurementChar.notify(buf, sizeof(buf));
  }
}

void cpControlPointWriteCallback(uint16_t conn_hdl,
                                 BLECharacteristic* chr,
                                 uint8_t* data,
                                 uint16_t len)
{
  if (len < 1) return;

  uint8_t opcode = data[0];

  if (opcode == 0x0C) {  // Start Offset Compensation
    doTare();

    uint8_t response[3] = {
      0x20, // Response Code
      0x0C, // Request opcode
      0x01  // Success
    };

    if (cpControlPointChar.indicateEnabled(conn_hdl)) {
      cpControlPointChar.indicate(conn_hdl, response, sizeof(response));
    }
  }
  else {
    uint8_t response[3] = {
      0x20,
      opcode,
      0x02  // Op Code Not Supported
    };

    if (cpControlPointChar.indicateEnabled(conn_hdl)) {
      cpControlPointChar.indicate(conn_hdl, response, sizeof(response));
    }

  }
}
