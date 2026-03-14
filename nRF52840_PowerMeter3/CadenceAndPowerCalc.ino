void CadencePowerCalc(unsigned long revolutionMs) {
  // Average all torque samples collected since the previous revolution.
  float avgTorque = (torqueSampleCount > 0) ? 
                    (sumTorqueNm / torqueSampleCount) : 0.0f;

  logPrint("torqueSampleCount= ");
  logPrint(torqueSampleCount);
  logPrint(" avg torque=");
  logPrint(avgTorque, 1);

  // Cadence comes from the revolution-to-revolution period.
  unsigned long now = revolutionMs;
  float dt = (now - lastRevMs) / 1000.0f;   // seconds
  lastRevMs = now;

  // Clamp to a plausible upper cadence limit.
  if (dt < 0.2f) dt = 0.2f;   // max 300 rpm

  float cadence_rpm = 60.0f / dt;

  // Convert one revolution per dt into angular speed in rad/s.
  float angVelRad = (2.0f * PI) / dt;

  // This meter reads one crank arm, so report estimated total power multiplied by two.
  float powerW = (avgTorque * angVelRad) * 2.0f;

  cumulativeCrankRevs++;

  // BLE Cycling Power uses 1/1024 second event timestamps.
  lastCrankEventTime = (uint16_t)(((uint32_t)now * 1024UL) / 1000UL);

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
