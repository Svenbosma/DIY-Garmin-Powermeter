void hx711ISR() { hx711ReadyFlag = true; }

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

void doTare() {
  logPrintln("Tare: remove load from pedal");
  delay(1000);
  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);
  logPrint("zeroOffsetCounts = "); logPrintln(String(zeroOffsetCounts));
}

void runCalibration(float knownMassKg) {
  logPrintln("=== Calibration Start ===");
  doTare();
  logPrintln("Step 1: Tare complete.");
  logPrint("Step 2: Hang "); logPrint(String(knownMassKg)); logPrintln(" kg and press any key...");

  String dummy;
  if (!waitForInput(dummy, 120000)) {
    logPrintln("No input received, aborting...");
    return;
  }

  long loadedAvg = averageCounts(CAL_SAMPLES, 5000);
  long deltaCounts = loadedAvg - zeroOffsetCounts;
  const float g = 9.80665f;
  float forceN = knownMassKg * g;

  if (forceN <= 0.0f || deltaCounts <= 0) {
    logPrintln("Calibration failed");
    return;
  }

  scaleFactor_counts_per_N = (float)deltaCounts / forceN;

  logPrint("deltaCounts = "); logPrintln(String(deltaCounts));
  logPrint("forceN = "); logPrintln(String(forceN,6));
  logPrint("scaleFactor_counts_per_N = "); logPrintln(String(scaleFactor_counts_per_N,6));
  logPrintln("=== Calibration Done ===");
}
