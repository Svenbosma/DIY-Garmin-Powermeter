// Collapse the pedal-plane gyro components into one angular-rate magnitude.
float integrateYZ(float gyroY, float gyroZ)
{
  return sqrt(gyroY * gyroY + gyroZ * gyroZ);
}

// Integrate filtered gyro magnitude and emit one event per full crank rotation.
bool detectRevolution(unsigned long sampleMillis, float degZ)
{
  static float angle = 0.0f;
  static float gzFiltered = 0.0f;
  static unsigned long lastMillis = 0;
  static float deltaAngle = 0;

  const float deadband = 0.01f;
  const float alpha = 0.25f;

  // Time delta between consecutive HX711-paced samples.
  unsigned long dt = (sampleMillis - lastMillis);
  lastMillis = sampleMillis;

  // Smooth the gyro signal before integrating.
  gzFiltered += alpha * (degZ - gzFiltered);

  // Convert deg/s into degrees moved since the previous sample.
  float integratedDegZ = gzFiltered * (dt / 1000.0f);

  if (fabs(integratedDegZ) >= deadband) {
    deltaAngle = integratedDegZ;
    angle += deltaAngle;
  }

  // Interpolate within the current sample so the BLE event time lands close to
  // the actual crossing instead of the end of the sample period.
  if (angle >= 360.0f) {
    float overshoot = angle - 360.0f;
    float fraction = (deltaAngle - overshoot) / deltaAngle;
    float revolutionDt = dt * fraction;

    revolutionTimestamp = sampleMillis - (dt - revolutionDt);
    angle -= 360.0f;
    return true;
  }

  if (angle <= -360.0f) {
    float overshoot = angle + 360.0f;
    float fraction = (deltaAngle - overshoot) / deltaAngle;
    float revolutionDt = dt * fraction;

    revolutionTimestamp = sampleMillis - (dt - revolutionDt);
    angle += 360.0f;
    return true;
  }

  return false;
}
