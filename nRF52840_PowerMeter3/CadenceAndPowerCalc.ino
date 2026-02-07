void CadencePowerCalc(unsigned long revolutionMs) {

  // --- Average sensor values ---
  float avgTorque = (torqueSampleCount > 0) ? 
                    (sumTorqueNm / torqueSampleCount) : 0.0f;

  float avgGyro = (imuSampleCount > 0) ? 
                  (sumGyroZ / imuSampleCount) : 0.0f;

  // --- Time since last revolution ---
  unsigned long now = revolutionMs;
  float dt = (now - lastRevMs) / 1000.0f;
  lastRevMs = now;

  // Clamp unrealistically fast cadence
  if (dt < 0.2f) dt = 0.2f;  // 300 rpm max

  float cadence_rpm = 60.0f / dt;

  // --- 1: Power calculation ---
  float angVelRad = avgGyro * DEG_TO_RAD;  // only valid if gyro is deg/s
  float powerW = avgTorque * angVelRad;

  // --- 2: Cumulative crank revolutions ---
  cumulativeCrankRevs++;

  // --- 3: Crank time difference ---
  lastCrankEventTime = (uint16_t)(((uint32_t)now * 1024UL) / 1000UL);

  // --- Send BLE measurement ---
  sendCyclingPowerMeasurement(
      (int16_t)powerW,
      cumulativeCrankRevs,
      lastCrankEventTime
  );

  // --- Reset accumulators ---
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0;

  sumGyroZ = 0.0f;
  imuSampleCount = 0;

  // --- Debug ---
  logPrint("Cadence=");
  logPrint(cadence_rpm, 1);
  logPrint(" rpm  Power=");
  logPrintln(powerW, 1);
}
