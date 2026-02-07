// Timestamp state persists across FIFO drain cycles.
static bool gHasPrevTimestamp = false;
static uint32_t gPrevTimestampTicks = 0;

// ------------------------------------------------------------
// IMU settings
// ------------------------------------------------------------
void configureIMUSettings()
{
  //Over-ride default settings if desired
  myIMU.settings.gyroEnabled = 1;  //Can be 0 or 1
  myIMU.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  myIMU.settings.gyroSampleRate = 208;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  myIMU.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  myIMU.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  myIMU.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  myIMU.settings.accelEnabled = 0;
  myIMU.settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
  myIMU.settings.tempEnabled = 0;
  
  //Non-basic mode settings
  myIMU.settings.commMode = 1;

  myIMU.settings.timestampEnabled=1;    // 1: enable timestamp ; 0: disable timestamp
  myIMU.settings.timestampFifoEnabled=1;// 1: enable write timestamp into fifo ; 0: disable
  myIMU.settings.timestampResolution=1; // 1: Set timestamp resolution ; 0: 6.4ms  1: 25us  

  //FIFO control settings
  myIMU.settings.fifoThreshold = FIFO_WATERMARK;  //Can be 0 to 4096 (16 bit bytes)
  myIMU.settings.fifoSampleRate = 50;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
  myIMU.settings.fifoModeWord = 6;  //FIFO mode.

}

// ------------------------------------------------------------
// FIFO setup
// ------------------------------------------------------------
void setupIMUFIFO()
{
  myIMU.fifoBegin();
  myIMU.fifoClear();

  // Route FIFO watermark to INT1
  uint8_t int1 = 0;
  myIMU.readRegister(&int1, LSM6DS3_ACC_GYRO_INT1_CTRL);
  int1 |= 0x08;
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, int1);

  pinMode(MOTION_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MOTION_INT_PIN), imuISR, RISING);
}


// ------------------------------------------------------------
// Angle + revolution detection
// ------------------------------------------------------------
bool detectRevolution(float integratedDegZ)
{
  static float angle = 0.0f;
  const float deadband = 0.01f;

  // Integrate angular rate into crank angle.
  if (integratedDegZ >= deadband or integratedDegZ <= -deadband)
  {
    angle += integratedDegZ;
  }

  // Detect wrap-around and count revolutions
  if (angle >= 360.0f)
  {
    angle -= 360.0f;
    return true;
  }
  if (angle <= -360.0f)
  {
    angle += 360.0f;
    return true;
  }

  return false;
}

// ------------------------------------------------------------
// Drain FIFO completely
// Detect revolutions per sample while draining
// ------------------------------------------------------------
void drainFifoAvgGx()
{
  static unsigned long reportWindowStartMs = millis();
  static uint32_t samplesThisSecond = 0;
  static float integratedDegZ = 0.0f;
  static float accumulatedDtSeconds = 0.0f;
  uint8_t nested = 0;
  
  // Drain FIFO until empty
  while ( ( myIMU.fifoGetStatus() & 0x1000 ) == 0 )
  {
    float gx = myIMU.calcGyro(myIMU.fifoRead());
    float gy = myIMU.calcGyro(myIMU.fifoRead());
    float gz = myIMU.calcGyro(myIMU.fifoRead());

    // Read and discard external sensor dataset before timestamp.
    (void)myIMU.fifoTimestamp();
    uint32_t fifoTimestamp = myIMU.fifoTimestamp();

    // Keep per-revolution average gyro input for power estimation.
    sumGyroZ += gz;
    imuSampleCount++;

    unsigned long eventMs = millis();

    if (gHasPrevTimestamp) {
      float dtSeconds = (fifoTimestamp - gPrevTimestampTicks) * 0.000025f;
      if ((dtSeconds > 0) and (dtSeconds < 0.5)){
        accumulatedDtSeconds += dtSeconds;
        integratedDegZ = gz * dtSeconds;
        if (detectRevolution(integratedDegZ)) {
          CadencePowerCalc(eventMs);
        }
      }
    } else {
      gHasPrevTimestamp = true;
    }
    gPrevTimestampTicks = fifoTimestamp;
    samplesThisSecond++;
  }

  unsigned long nowMs = millis();
  if (nowMs - reportWindowStartMs >= 1000UL) {
    Serial.print("samples_per_second: ");
    Serial.print(samplesThisSecond);
    Serial.print(", accumulated_dt_s: ");
    Serial.print(accumulatedDtSeconds, 6);
    Serial.print(", integrated_deg_z: ");
    Serial.println(integratedDegZ, 6);

    samplesThisSecond = 0;
    integratedDegZ = 0.0f;
    accumulatedDtSeconds = 0.0f;
    reportWindowStartMs = nowMs;
  }
}
