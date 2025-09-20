void handleUART() {
  while (bleUart.available()) {
    String cmd = bleUart.readString();
    processUARTCommand(cmd);
  }
}

void processUARTCommand(String cmd) {
  cmd.trim();
  logPrint("UART command received: "); logPrintln(cmd);

  if (cmd.equalsIgnoreCase("c")) runCalibration(10.0f);
  else if (cmd.equalsIgnoreCase("t")) doTare();
  else if (cmd.startsWith("m")) {
    float kg = cmd.substring(1).toFloat();
    if (kg > 0.0f) runCalibration(kg);
    else logPrintln("Usage: m <mass_kg>");
  } else {
    logPrint("Unknown UART command: "); logPrintln(cmd);
  }
}

bool waitForInput(String &cmd, unsigned long timeoutMs = 30000) {
  unsigned long start = millis();
  cmd = "";
  while ((millis() - start) < timeoutMs) {
    if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();
      return true;
    }
    if (bleUart.available()) {
      cmd = bleUart.readStringUntil('\n');
      cmd.trim();
      return true;
    }
    delay(10);
  }
  return false;
}
