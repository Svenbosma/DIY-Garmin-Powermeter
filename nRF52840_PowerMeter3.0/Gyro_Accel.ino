void accumulateIMU(float gx, float ax) {
  sumGyroX += gx;
  sumAccelX += ax;
  imuSampleCount++;
}

bool detectRevolution(float gx, float ax) {
  bool accelPositive = (ax > ACCEL_THRESHOLD_G);
  bool forward = (gx > GYRO_FORWARD_THRESHOLD);

  bool revolution = false;
  if (!prevAccelPositive && accelPositive && forward) {
    revolution = true;
  }

  prevAccelPositive = accelPositive;
  return revolution;
}

float calcPower(float torqueNm, float angVelRad) {
  return torqueNm * angVelRad;
}
