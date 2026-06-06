void loadFlashValues()
{
  InternalFS.begin();
  bool loadedTare = false;

  File loggingFile(InternalFS);
  loggingFile.open(LOGGING_FILE, FILE_O_READ);

  if (loggingFile)
  {
    char buffer[8] = {0};
    uint32_t len = loggingFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    loggingEnabled = (atoi(buffer) != 0);

    loggingFile.close();
  }
  else
  {
    loggingEnabled = true;
  }

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
    loadedTare = true;

    logPrint("Loaded tare: ");
    logPrintln(String(zeroOffsetCounts));

    tareFile.close();
  }
  else
  {
    zeroOffsetCounts = 0;
    logPrintln("No tare file found, using 0");
  }

  File crankLengthFile(InternalFS);
  crankLengthFile.open(CRANK_LENGTH_FILE, FILE_O_READ);

  if (crankLengthFile)
  {
    char buffer[32] = {0};
    uint32_t len = crankLengthFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    uint16_t loadedHalfMm = (uint16_t)atoi(buffer);
    if (!setCrankLengthHalfMm(loadedHalfMm, false)) {
      logPrintln("Invalid crank length file, using default 172.5 mm");
      crankLengthHalfMm = DEFAULT_CRANK_LENGTH_HALF_MM;
      crankLengthM = DEFAULT_CRANK_LENGTH_M;
    } else {
      logPrint("Loaded crank length = ");
      logPrint((float)crankLengthHalfMm * 0.5f, 1);
      logPrintln(" mm");
    }

    crankLengthFile.close();
  }
  else
  {
    crankLengthHalfMm = DEFAULT_CRANK_LENGTH_HALF_MM;
    crankLengthM = DEFAULT_CRANK_LENGTH_M;
    logPrintln("No crank length file found, using default 172.5 mm");
  }

  File garminOffsetRefFile(InternalFS);
  garminOffsetRefFile.open(GARMIN_OFFSET_REF_FILE, FILE_O_READ);

  if (garminOffsetRefFile)
  {
    char buffer[32] = {0};
    uint32_t len = garminOffsetRefFile.read(buffer, sizeof(buffer)-1);
    buffer[len] = 0;

    garminOffsetReferenceCounts = atol(buffer);
    garminOffsetReferenceValid = true;

    logPrint("Loaded Garmin offset reference = ");
    logPrintln(String(garminOffsetReferenceCounts));

    garminOffsetRefFile.close();
  }
  else
  {
    garminOffsetReferenceCounts = 0;
    garminOffsetReferenceValid = false;
    logPrintln("No Garmin offset reference file found");

    if (loadedTare) {
      ensureGarminOffsetReference();
    }
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

void saveCrankLength()
{
  File crankLengthFile(InternalFS);
  InternalFS.remove(CRANK_LENGTH_FILE);

  if (crankLengthFile.open(CRANK_LENGTH_FILE, FILE_O_WRITE))
  {
    String val = String(crankLengthHalfMm);
    crankLengthFile.write(val.c_str(), val.length());
    crankLengthFile.close();

    logPrintln("Crank length saved to flash");
  }
  else
  {
    logPrintln("Failed to save crank length");
  }
}

void saveGarminOffsetReference()
{
  File garminOffsetRefFile(InternalFS);
  InternalFS.remove(GARMIN_OFFSET_REF_FILE);

  if (garminOffsetRefFile.open(GARMIN_OFFSET_REF_FILE, FILE_O_WRITE))
  {
    String val = String(garminOffsetReferenceCounts);
    garminOffsetRefFile.write(val.c_str(), val.length());
    garminOffsetRefFile.close();

    logPrintln("Garmin offset reference saved to flash");
  }
  else
  {
    logPrintln("Failed to save Garmin offset reference");
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

bool saveLoggingState()
{
  File loggingFile(InternalFS);
  InternalFS.remove(LOGGING_FILE);

  if (loggingFile.open(LOGGING_FILE, FILE_O_WRITE))
  {
    const char *val = loggingEnabled ? "1" : "0";
    loggingFile.write(val, 1);
    loggingFile.close();

    return true;
  }

  return false;
}
