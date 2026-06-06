// BLE services exposed by the powermeter.
BLEService cyclingPowerService = BLEService(0x1818);

static const uint8_t CPS_OPCODE_SET_CRANK_LENGTH = 0x04;
static const uint8_t CPS_OPCODE_REQUEST_SUPPORTED_SENSOR_LOCATIONS = 0x03;
static const uint8_t CPS_OPCODE_REQUEST_CRANK_LENGTH = 0x05;
static const uint8_t CPS_OPCODE_START_OFFSET_COMPENSATION = 0x0C;
static const uint8_t CPS_OPCODE_RESPONSE_CODE = 0x20;

static const uint8_t CPS_RESPONSE_SUCCESS = 0x01;
static const uint8_t CPS_RESPONSE_OPCODE_NOT_SUPPORTED = 0x02;
static const uint8_t CPS_RESPONSE_INVALID_PARAMETER = 0x03;
static const uint8_t CPS_RESPONSE_OPERATION_FAILED = 0x04;

BLECharacteristic cpMeasurementChar = BLECharacteristic(0x2A63);
BLECharacteristic cpFeatureChar     = BLECharacteristic(0x2A65);
BLECharacteristic cpControlPointChar = BLECharacteristic(0x2A66);

static volatile bool cpControlPointPending = false;
static uint16_t cpPendingConnHdl = BLE_CONN_HANDLE_INVALID;
static uint8_t cpPendingData[20] = {0};
static uint16_t cpPendingLen = 0;


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
void serviceCyclingPowerControlPoint();

static void indicateControlPointResponse(uint16_t conn_hdl,
                                         const uint8_t* response,
                                         uint16_t len) {
  if (cpControlPointChar.indicateEnabled(conn_hdl)) {
    cpControlPointChar.indicate(conn_hdl, response, len);
  }
}


void setupBLE() {
  Bluefruit.begin(1, 0);  // Peripheral only
  Bluefruit.autoConnLed(true);
  Bluefruit.setTxPower(0);
  Bluefruit.setName("DIY-Powermeter");

  Bluefruit.Periph.setConnInterval(6, 12);   // 7.5–15 ms
  Bluefruit.Periph.setConnSlaveLatency(0);
  Bluefruit.Periph.setConnSupervisionTimeout(400);

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
  (1UL << 3) |   // Crank Revolution Data Supported
  (1UL << 9) |   // Offset Compensation Supported
  (1UL << 12);   // Crank Length Adjustment Supported

  cpFeatureChar.write((uint8_t*)&features, 4);

  // Control Point is used by head units for zero-offset requests.
  cpControlPointChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_INDICATE);
  cpControlPointChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cpControlPointChar.setMaxLen(20);
  cpControlPointChar.setWriteCallback(cpControlPointWriteCallback);
  cpControlPointChar.begin();

  // Sensor Location
  BLECharacteristic sensorLocationChar = BLECharacteristic(0x2A5D);
  sensorLocationChar.setProperties(CHR_PROPS_READ);
  sensorLocationChar.setFixedLen(1);
  sensorLocationChar.begin();

  uint8_t location = 0x05; // Left Crank (see https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml)
  sensorLocationChar.write(&location, 1);

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

  uint16_t copyLen = (len > sizeof(cpPendingData)) ? sizeof(cpPendingData) : len;
  memcpy(cpPendingData, data, copyLen);
  cpPendingLen = copyLen;
  cpPendingConnHdl = conn_hdl;
  cpControlPointPending = true;
}

void serviceCyclingPowerControlPoint()
{
  if (!cpControlPointPending) return;

  cpControlPointPending = false;

  uint16_t conn_hdl = cpPendingConnHdl;
  uint16_t len = cpPendingLen;
  uint8_t opcode = cpPendingData[0];

  if (opcode == CPS_OPCODE_SET_CRANK_LENGTH) {
    if (len < 3) {
      uint8_t response[3] = {
        CPS_OPCODE_RESPONSE_CODE,
        opcode,
        CPS_RESPONSE_INVALID_PARAMETER
      };
      indicateControlPointResponse(conn_hdl, response, sizeof(response));
      return;
    }

    uint16_t requestedHalfMm = (uint16_t)cpPendingData[1] | ((uint16_t)cpPendingData[2] << 8);
    uint8_t response[3] = {
      CPS_OPCODE_RESPONSE_CODE,
      opcode,
      setCrankLengthHalfMm(requestedHalfMm, true) ? CPS_RESPONSE_SUCCESS
                                                  : CPS_RESPONSE_INVALID_PARAMETER
    };
    indicateControlPointResponse(conn_hdl, response, sizeof(response));
  }
  else if (opcode == CPS_OPCODE_REQUEST_SUPPORTED_SENSOR_LOCATIONS) {
    uint8_t response[4] = {
      CPS_OPCODE_RESPONSE_CODE,
      opcode,
      CPS_RESPONSE_SUCCESS,
      0x05  // Left Crank
    };
    indicateControlPointResponse(conn_hdl, response, sizeof(response));
  }
  else if (opcode == CPS_OPCODE_REQUEST_CRANK_LENGTH) {
    uint8_t response[5] = {
      CPS_OPCODE_RESPONSE_CODE,
      opcode,
      CPS_RESPONSE_SUCCESS,
      (uint8_t)(crankLengthHalfMm & 0xFF),
      (uint8_t)((crankLengthHalfMm >> 8) & 0xFF)
    };
    indicateControlPointResponse(conn_hdl, response, sizeof(response));
  }
  else if (opcode == CPS_OPCODE_START_OFFSET_COMPENSATION) {
    int16_t garminOffset = doGarminOffsetCompensation();

    uint8_t response[5] = {
      CPS_OPCODE_RESPONSE_CODE,
      opcode,
      CPS_RESPONSE_SUCCESS,
      (uint8_t)(garminOffset & 0xFF),
      (uint8_t)((garminOffset >> 8) & 0xFF)
    };
    indicateControlPointResponse(conn_hdl, response, sizeof(response));
  }
  else {
    uint8_t response[3] = {
      CPS_OPCODE_RESPONSE_CODE,
      opcode,
      CPS_RESPONSE_OPCODE_NOT_SUPPORTED
    };
    indicateControlPointResponse(conn_hdl, response, sizeof(response));
  }
}
