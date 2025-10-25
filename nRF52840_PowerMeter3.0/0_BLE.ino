// --- BLE Services & Characteristics ---
BLEService cyclingPowerService("1818"); // Cycling Power Service
BLECharacteristic cpMeasurementChar("2A63", BLERead | BLENotify, 8);  // Cycling Power Measurement
BLECharacteristic cpFeatureChar("2A65", BLERead, 4);                  // Cycling Power Feature

BLEService batteryService("180F"); // Battery Service
BLECharacteristic batteryLevelChar("2A19", BLERead | BLENotify, 1);   // Battery Level

// (Optional replacement for Bluefruit UART)
BLEService uartService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic uartTXChar("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, 20);
BLECharacteristic uartRXChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 20);

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setDeviceName("DIY-Powermeter");
  BLE.setLocalName("DIY-Powermeter");
  BLE.setAdvertisedService(cyclingPowerService);

  // --- Cycling Power Service ---
  cyclingPowerService.addCharacteristic(cpMeasurementChar);
  cyclingPowerService.addCharacteristic(cpFeatureChar);
  BLE.addService(cyclingPowerService);

  uint32_t features = 0;
  cpFeatureChar.writeValue((uint8_t*)&features, sizeof(features));

  // --- Battery Service ---
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);

  uint8_t initialBatt = 100;
  batteryLevelChar.writeValue(initialBatt);

  // --- UART Service (optional) ---
  uartService.addCharacteristic(uartTXChar);
  uartService.addCharacteristic(uartRXChar);
  BLE.addService(uartService);

  // Start advertising
  BLE.advertise();

  Serial.println("BLE advertising started (Cycling Power + UART + Battery ready)");
}

// --- Send Cycling Power Measurement ---
void sendCyclingPowerMeasurement(int16_t powerWatts, uint16_t cumulativeCrankRevs, uint16_t lastCrankEventTime) {
  uint8_t buf[8] = {0};
  buf[0] = 0x20; buf[1] = 0x00;
  buf[2] = powerWatts & 0xFF;
  buf[3] = (powerWatts >> 8) & 0xFF;
  buf[4] = cumulativeCrankRevs & 0xFF;
  buf[5] = (cumulativeCrankRevs >> 8) & 0xFF;
  buf[6] = lastCrankEventTime & 0xFF;
  buf[7] = (lastCrankEventTime >> 8) & 0xFF;

  cpMeasurementChar.writeValue(buf, sizeof(buf)); // automatically notifies if subscribed
}
