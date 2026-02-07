uint8_t batteryPercentFromVoltage(float v)
{
  if (v >= 4.20) return 100;
  if (v >= 4.15) return 95;
  if (v >= 4.10) return 90;
  if (v >= 4.05) return 85;
  if (v >= 4.00) return 80;
  if (v >= 3.95) return 70;
  if (v >= 3.90) return 60;
  if (v >= 3.85) return 50;
  if (v >= 3.80) return 40;
  if (v >= 3.75) return 30;
  if (v >= 3.70) return 20;
  if (v >= 3.65) return 15;
  if (v >= 3.60) return 10;
  if (v >= 3.55) return 7;
  if (v >= 3.50) return 5;
  if (v >= 3.45) return 3;
  if (v >= 3.40) return 2;
  if (v >= 3.30) return 1;
  return 0;
}

uint8_t readBatteryPercent() {
  float vbatSum = 0.0f;

  for (int i = 0; i < NUM_BATT_SAMPLES; i++) {
    int Vadc = analogRead(PIN_VBAT);
    float Vbatt = 3 * 2.448 * Vadc / 4096;
    vbatSum += Vbatt;
  }

  float vbatAvg = vbatSum / NUM_BATT_SAMPLES;

  logPrint("Battery voltage = ");
  logPrintln(String(vbatAvg, 2));

  vbatAvg = constrain(vbatAvg, 3.30f, 4.25f);
  int percent = batteryPercentFromVoltage(vbatAvg);
  return (uint8_t)percent;
}



void batteryCheckAndLED() {
  unsigned long now = millis();

  // Periodic battery update
  if (now - lastBattMs >= BATTERY_UPDATE_INTERVAL) {
    lastBattMs = now;
    battPercent = readBatteryPercent();
    logPrint("Battery percentage = ");
    logPrintln(String(battPercent));
    // Update BLE battery characteristic (auto-notifies if subscribed)
    if (batteryLevelChar.notifyEnabled()) {
      batteryLevelChar.notify(&battPercent, 1);
    } else {
      batteryLevelChar.write(&battPercent, 1);
    }

    // Reset blink pattern
    blinkCount = 0;
    ledState = false;
    digitalWrite(LED_RED, HIGH);
    lastBlinkMs = now;
  }

  // Blink LED if battery is low
  if (battPercent <= LOW_BATT_THRESHOLD && blinkCount < NUM_BLINKS) {
    if (now - lastBlinkMs >= BLINK_INTERVAL) {
      lastBlinkMs = now;
      ledState = !ledState;
      digitalWrite(LED_RED, ledState ? LOW : HIGH);
      if (!ledState) blinkCount++;
    }
  } 
  else if (battPercent > LOW_BATT_THRESHOLD) {
    digitalWrite(LED_RED, HIGH);
  }
}

