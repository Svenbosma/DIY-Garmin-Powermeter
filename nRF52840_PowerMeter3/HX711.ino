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

void loadFlashValues()
{
  InternalFS.begin();

  // Load calibration first so raw counts can be converted to force.
  calFile.open(CAL_FILE, FILE_O_READ);

  if (calFile)
  {
    char buffer[32] = {0};
    uint32_t len = calFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    scaleFactor_counts_per_N = atof(buffer);

    logPrint("Loaded calibration: ");
    logPrintln(String(scaleFactor_counts_per_N, 6));

    calFile.close();
  }
  else
  {
    scaleFactor_counts_per_N = 0.0f;
    logPrintln("No calibration file found, using 0.0");
  }

  // Load tare so each HX711 reading can be zero-referenced.
  tareFile.open(TARE_FILE, FILE_O_READ);

  if (tareFile)
  {
    char buffer[32] = {0};
    uint32_t len = tareFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    zeroOffsetCounts = atol(buffer);

    logPrint("Loaded tare: ");
    logPrintln(String(zeroOffsetCounts));

    tareFile.close();
  }
  else
  {
    zeroOffsetCounts = 0;
    logPrintln("No tare file found, using 0");
  }

  gyroTareFile.open(GYRO_TARE_FILE, FILE_O_READ);

  if (gyroTareFile)
  {
    char buffer[64] = {0};
    uint32_t len = gyroTareFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    int parsed = sscanf(buffer, "%f %f", &gyroBiasY_dps, &gyroBiasZ_dps);
    if (parsed == 2) {
      logPrint("Loaded gyro tare Y/Z = ");
      logPrint(String(gyroBiasY_dps, 6));
      logPrint(", ");
      logPrintln(String(gyroBiasZ_dps, 6));
    } else {
      gyroBiasY_dps = 0.0f;
      gyroBiasZ_dps = 0.0f;
      logPrintln("Invalid gyro tare file, using 0.0");
    }

    gyroTareFile.close();
  }
  else
  {
    gyroBiasY_dps = 0.0f;
    gyroBiasZ_dps = 0.0f;
    logPrintln("No gyro tare file found, using 0.0");
  }
}

void saveTare()
{
  InternalFS.remove(TARE_FILE);

  if (tareFile.open(TARE_FILE, FILE_O_WRITE))
  {
    String val = String(zeroOffsetCounts);
    tareFile.write(val.c_str(), val.length());
    tareFile.close();

    logPrintln("Tare saved to flash");
  }
  else
  {
    logPrintln("Failed to save tare");
  }
}

void saveCalibration()
{
  InternalFS.remove(CAL_FILE);

  if (calFile.open(CAL_FILE, FILE_O_WRITE))
  {
    String val = String(scaleFactor_counts_per_N, 6);
    calFile.write(val.c_str(), val.length());
    calFile.close();

    logPrintln("Calibration saved to flash");
  }
  else
  {
    logPrintln("Failed to save calibration");
  }
}

void saveGyroTare()
{
  InternalFS.remove(GYRO_TARE_FILE);

  if (gyroTareFile.open(GYRO_TARE_FILE, FILE_O_WRITE))
  {
    String val = String(gyroBiasY_dps, 6) + " " + String(gyroBiasZ_dps, 6);
    gyroTareFile.write(val.c_str(), val.length());
    gyroTareFile.close();

    logPrintln("Gyro tare saved to flash");
  }
  else
  {
    logPrintln("Failed to save gyro tare");
  }
}


void doTare()
{
  logPrintln("Tare: remove load from pedal");
  delay(1000);

  zeroOffsetCounts = averageCounts(TARE_SAMPLES, 4000);

  logPrint("zeroOffsetCounts = ");
  logPrintln(String(zeroOffsetCounts));

  saveTare();
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
  return;
}
