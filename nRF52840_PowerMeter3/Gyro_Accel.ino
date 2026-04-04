void configureActiveIMU()
{
  // Active riding only needs Z gyro data for cadence detection.
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroFifoEnabled = 0;
  myIMU.settings.gyroFifoDecimation = 0;

  myIMU.settings.accelEnabled = 0;
  myIMU.settings.accelFifoEnabled = 0;
  myIMU.settings.accelFifoDecimation = 0;

  myIMU.settings.tempEnabled = 0;
  myIMU.settings.fifoModeWord = 0;
  myIMU.settings.fifoSampleRate = 0;
}

// Integrate filtered gyro magnitude and emit one event per full crank rotation.
bool detectRevolution(uint32_t sampleMicros, float degZ)
{
  static float angle = 0.0f;
  static float gzFiltered = 0.0f;
  static uint32_t lastMicros = 0;
  static float deltaAngle = 0;

  const float deadband = 0.01f;
  const float alpha = 0.25f;

  // Time delta between consecutive HX711-paced samples.
  uint32_t dtUs = sampleMicros - lastMicros;
  lastMicros = sampleMicros;

  // Smooth the gyro signal before integrating.
  //gzFiltered += alpha * (degZ - gzFiltered);
  gzFiltered = degZ - gyroBiasZ_dps;

  // Convert deg/s into degrees moved since the previous sample.
  float integratedDegZ = gzFiltered * (dtUs / 1000000.0f);

  if (fabs(integratedDegZ) >= deadband) {
    deltaAngle = integratedDegZ;
    angle += deltaAngle;
  }

  // Interpolate within the current sample so the BLE event time lands close to
  // the actual crossing instead of the end of the sample period.
  if (angle >= 360.0f) {
    float overshoot = angle - 360.0f;
    float fraction = (deltaAngle - overshoot) / deltaAngle;
    float revolutionDtUs = dtUs * fraction;

    revolutionTimestampUs = sampleMicros - (dtUs - revolutionDtUs);
    angle -= 360.0f;
    return true;
  }

  if (angle <= -360.0f) {
    float overshoot = angle + 360.0f;
    float fraction = (deltaAngle - overshoot) / deltaAngle;
    float revolutionDtUs = dtUs * fraction;

    revolutionTimestampUs = sampleMicros - (dtUs - revolutionDtUs);
    angle += 360.0f;
    return true;
  }

  return false;
}
