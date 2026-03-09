void CadencePowerCalc(unsigned long revolutionMs) {

  // --- Average torque ---
  float avgTorque = (torqueSampleCount > 0) ? 
                    (sumTorqueNm / torqueSampleCount) : 0.0f;

  logPrint("torqueSampleCount= ");
  logPrint(torqueSampleCount);
  logPrint(" avg torque=");
  logPrint(avgTorque, 1);

  // --- Time since last revolution ---
  unsigned long now = revolutionMs;
  float dt = (now - lastRevMs) / 1000.0f;   // seconds
  lastRevMs = now;

  // Prevent division by zero / unrealistic cadence
  if (dt < 0.2f) dt = 0.2f;   // max 300 rpm

  float cadence_rpm = 60.0f / dt;

  // --- Angular velocity from revolution timing ---
  float angVelRad = (2.0f * PI) / dt;

  // --- Power calculation ---
  float powerW = avgTorque * angVelRad;

  // --- Cumulative crank revolutions ---
  cumulativeCrankRevs++;

  // --- BLE crank timestamp (1/1024 s units) ---
  lastCrankEventTime = (uint16_t)(((uint32_t)now * 1024UL) / 1000UL);

  // --- Send BLE measurement ---
  sendCyclingPowerMeasurement(
      (int16_t)powerW,
      cumulativeCrankRevs,
      lastCrankEventTime
  );

  // --- Reset torque accumulator ---
  sumTorqueNm = 0.0f;
  torqueSampleCount = 0;

  // --- Debug ---
  logPrint(" Cadence=");
  logPrint(cadence_rpm, 1);
  logPrint(" rpm  Power=");
  logPrintln(powerW, 1);
}