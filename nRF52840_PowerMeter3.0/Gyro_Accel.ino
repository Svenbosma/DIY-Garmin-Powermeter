void accumulateIMU(float gx) {
  sumGyroX += gx;
  imuSampleCount++;
}

bool detectRevolution(float gx) {
  static float angle = 0.0;
  static unsigned long lastTime = 0;
  static bool revolutionReported = false;

  unsigned long now = millis();
  if (lastTime == 0) {
    lastTime = now;
    return false;
  }
  float dt = (now - lastTime) / 1000.0; // seconds
  lastTime = now;

  // integrate gyro
  angle += gx * dt;

  // normalize
  if (angle >= 360.0) {
    angle -= 360.0;
    revolutionReported = false;
  }

  // detect revolution
  if (!revolutionReported && angle < 10.0) { // small threshold near zero
    revolutionReported = true;
    return true;
  }

  return false;
}

float ReadGxFIFO() {
  float sum = 0;        // accumulator for samples
  int count = 0;       // number of samples
  // Loop until FIFO is empty
  while ((myIMU.fifoGetStatus() & 0x1000) == 0) {
    // Read X, Y, Z gyro in order
    int16_t gxRaw = myIMU.fifoRead();
    int16_t gyRaw = myIMU.fifoRead(); // discard
    int16_t gzRaw = myIMU.fifoRead(); // discard

    uint16_t thirdData = myIMU.fifoTimestamp();
    uint32_t fifoTimestamp = myIMU.fifoTimestamp();

    // Convert raw X gyro to deg/s and add to sum
    sum += myIMU.calcGyro(gxRaw);
    count++;
  }
  uint16_t tempUnsigned = myIMU.fifoGetStatus();

  if (count == 0) return 0.01; // no samples available
  return (float)sum / (float)count; // average
}
