uint8_t readBatteryPercent() {
  float vbatSum = 0.0f;
  for (int i = 0; i < NUM_BATT_SAMPLES; i++) {
    int raw = analogRead(PIN_VBAT);
    float vbat = raw * (3.3f / 4095.0f) * 2.0f;
    vbatSum += vbat;
  }
  float vbatAvg = vbatSum / NUM_BATT_SAMPLES;

  int percent = (int)((vbatAvg - 3.0f) * 100.0f / (4.2f - 3.0f));
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  return percent;
}

void batteryCheckAndLED() {
  unsigned long now = millis();

  if (now - lastBattMs >= BATTERY_UPDATE_INTERVAL) {
    lastBattMs = now;
    battPercent = readBatteryPercent();
    batteryLevelChar.write8(battPercent);
    batteryLevelChar.notify8(battPercent);
    logPrint("Battery: "); logPrintln(battPercent);

    blinkCount = 0;
    ledState = false;
    digitalWrite(LED_RED, HIGH);
    lastBlinkMs = now;
  }

  if (battPercent <= LOW_BATT_THRESHOLD && blinkCount < NUM_BLINKS) {
    if (now - lastBlinkMs >= BLINK_INTERVAL) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_RED, ledState ? LOW : HIGH);
      if (!ledState) blinkCount++;
    }
  } else if (battPercent > LOW_BATT_THRESHOLD) {
    digitalWrite(LED_RED, HIGH);
  }
}
