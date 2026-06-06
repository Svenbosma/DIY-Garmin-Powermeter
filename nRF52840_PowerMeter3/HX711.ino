void hx711ISR() { hx711ReadyFlag = true; }

static float crankLengthHalfMmToMeters(uint16_t halfMm) {
  return ((float)halfMm * 0.5f) / 1000.0f;
}

long averageCounts(int n, unsigned long timeoutMs) {
  long sum = 0; int k=0;
  unsigned long t0 = millis();
  while (k<n && (millis()-t0)<timeoutMs) {
    if (scale.is_ready()) { sum+=scale.read(); k++; }
  }
  return (k>0) ? (sum/k) : 0L;
}

void accumulateTorque(float torqueNm) {
  sumTorqueNm += torqueNm;
  torqueSampleCount++;
}

bool setCrankLengthHalfMm(uint16_t halfMm, bool persist)
{
  if (halfMm == 0) {
    return false;
  }

  crankLengthHalfMm = halfMm;
  crankLengthM = crankLengthHalfMmToMeters(halfMm);

  logPrint("crankLengthM = ");
  logPrintln(String(crankLengthM, 4));

  if (persist) {
    saveCrankLength();
  }

  return true;
}

void ensureGarminOffsetReference()
{
  if (garminOffsetReferenceValid) {
    return;
  }

  garminOffsetReferenceCounts = zeroOffsetCounts;
  garminOffsetReferenceValid = true;

  logPrint("Initialized Garmin offset reference = ");
  logPrintln(String(garminOffsetReferenceCounts));
  saveGarminOffsetReference();
}

int16_t getGarminDisplayedOffset()
{
  ensureGarminOffsetReference();

  // Keep Garmin's displayed offset anchored to a saved baseline so users see
  // a small drift value (for example +10) instead of the full HX711 raw count.
  long deltaCounts = zeroOffsetCounts - garminOffsetReferenceCounts;
  if (deltaCounts > INT16_MAX) deltaCounts = INT16_MAX;
  if (deltaCounts < INT16_MIN) deltaCounts = INT16_MIN;

  return (int16_t)deltaCounts;
}


void doTare(bool updateGarminReference)
{
  logPrintln("Tare: remove load from pedal");
  delay(1000);

  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);

  logPrint("zeroOffsetCounts = ");
  logPrintln(String(zeroOffsetCounts));

  saveTare();

  if (updateGarminReference) {
    garminOffsetReferenceCounts = zeroOffsetCounts;
    garminOffsetReferenceValid = true;

    logPrint("Saved UART tare reference = ");
    logPrintln(String(garminOffsetReferenceCounts));
    saveGarminOffsetReference();
  }
}

void doTareGyro()
{
  logPrintln("Gyro tare: keep crank still");
  delay(1000);

  float sumGyroY = 0.0f;
  float sumGyroZ = 0.0f;

  for (int i = 0; i < TARE_SAMPLES; i++) {
    sumGyroY += myIMU.readFloatGyroY();
    sumGyroZ += myIMU.readFloatGyroZ();
    delay(5);
  }

  gyroBiasY_dps = sumGyroY / TARE_SAMPLES;
  gyroBiasZ_dps = sumGyroZ / TARE_SAMPLES;

  logPrint("gyroBiasY_dps = ");
  logPrintln(String(gyroBiasY_dps, 6));
  logPrint("gyroBiasZ_dps = ");
  logPrintln(String(gyroBiasZ_dps, 6));

  saveGyroTare();
}

void runCalibration(float knownMassKg)
{
  logPrintln("=== Calibration Start ===");

  long loadedAvg = averageCounts(CAL_SAMPLES, 5000);
  long deltaCounts = loadedAvg - zeroOffsetCounts;

  const float g = 9.80665f;
  float forceN = knownMassKg * g;

  if (forceN <= 0.0f || deltaCounts == 0)
  {
    logPrintln("Calibration failed (invalid delta)");
    calibrationActive = false;
    return;
  }

  scaleFactor_counts_per_N = (float)deltaCounts / forceN;

  logPrint("loadedAvg = "); logPrintln(String(loadedAvg));
  logPrint("zeroOffsetCounts = "); logPrintln(String(zeroOffsetCounts));
  logPrint("deltaCounts = "); logPrintln(String(deltaCounts));
  logPrint("forceN = "); logPrintln(String(forceN,6));
  logPrint("scaleFactor_counts_per_N = "); logPrintln(String(scaleFactor_counts_per_N,6));
  logPrintln("=== Calibration Done ===");

  saveCalibration();
}

int16_t doGarminOffsetCompensation()
{
  calibrationActive = true;

  doTare(false);
  doTareGyro();

  int16_t garminOffset = getGarminDisplayedOffset();

  logPrint("Garmin offset value = ");
  logPrintln(String(garminOffset));

  calibrationActive = false;
  return garminOffset;
}
