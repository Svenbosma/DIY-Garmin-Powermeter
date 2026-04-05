bool shouldAccumulateTorqueSample(float torqueNm, float correctedGyroZ_dps) {
  float angularVelocityRadPerSec = correctedGyroZ_dps * (PI / 180.0f);
  float samplePowerW = (torqueNm * angularVelocityRadPerSec) * 2.0f;

  return isfinite(samplePowerW) && fabs(samplePowerW) <= MAX_VALID_POWER_W;
}

void CadencePowerCalc(uint32_t revolutionUs) {
  lastRevUs = revolutionUs;

  // Average all torque samples collected since the previous revolution.
  float avgTorque = (torqueSampleCount > 0) ? 
                    (sumTorqueNm / torqueSampleCount) : 0.0f;

  logPrint("torqueSampleCount= ");
  logPrint(torqueSampleCount);
  logPrint(" avg torque=");
  logPrint(avgTorque, 1);
  logPrint(" gyroZ=");
  logPrint(lastGyroZ_dps, 2);

  // Cadence and power come from the same revolution-to-revolution period that
  // Garmin will infer from the CPS crank event timestamps.
  uint32_t nowUs = revolutionUs;
  float dt = 0.0f;
  float cadence_rpm = 0.0f;
  float angVelRad = 0.0f;
  float powerW = 0.0f;

  if (previousCadenceRevUs != 0) {
    dt = (nowUs - previousCadenceRevUs) / 1000000.0f;   // seconds

    // Clamp to a plausible upper cadence limit.
    if (dt < 0.2f) dt = 0.2f;   // max 300 rpm

    cadence_rpm = 60.0f / dt;

    // Convert one revolution per dt into angular speed in rad/s.
    float angVelRadLocal = (2.0f * PI) / dt;
    angVelRad = angVelRadLocal;

    if (cadence_rpm >= MIN_POWER_CADENCE_RPM) {
      // This meter reads one crank arm, so report estimated total power multiplied by two.
      powerW = (avgTorque * angVelRadLocal) * 2.0f;
    } else {
      powerW = 0.0f;
    }

    if (powerW > MAX_VALID_POWER_W) {
      powerW = 0.0f;
    }
  }
  previousCadenceRevUs = nowUs;

  cumulativeCrankRevs++;

  // BLE Cycling Power uses 1/1024 second event timestamps.
  lastCrankEventTime = (uint16_t)(((uint64_t)nowUs * 1024ULL) / 1000000ULL);

  sendCyclingPowerMeasurement(
      (int16_t)powerW,
      cumulativeCrankRevs,
      lastCrankEventTime
  );

  // Start a fresh averaging window for the next crank revolution.
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0;

  logPrint(" Cadence=");
  logPrint(cadence_rpm, 1);
  logPrint(" rpm  Power=");
  logPrintln(powerW, 1);
}
