// Buffer to collect incoming UART data
String uartBuffer = "";

void uartRXWriteCallback(uint16_t conn_hdl,
                         BLECharacteristic* chr,
                         uint8_t* data,
                         uint16_t len)
{
  if (len == 0) return;

  // Append incoming bytes to buffer
  for (uint16_t i = 0; i < len; i++) {
    uartBuffer += (char)data[i];
  }

  // Process complete commands (newline terminated)
  int newlineIndex = uartBuffer.indexOf('\n');
  while (newlineIndex >= 0) {
    String cmd = uartBuffer.substring(0, newlineIndex);
    uartBuffer.remove(0, newlineIndex + 1);
    processUARTCommand(cmd);
    newlineIndex = uartBuffer.indexOf('\n');
  }
}

// Command processing logic (unchanged except for compatibility)
void processUARTCommand(String cmd) {
  cmd.trim();
  logPrint("UART command received: "); 
  logPrintln(cmd);

  if (cmd.equalsIgnoreCase("c")) {
    runCalibration(10.0f);
  } 
  else if (cmd.equalsIgnoreCase("t")) {
    doTare();
  } 
  else if (cmd.startsWith("m")) {
    float kg = cmd.substring(1).toFloat();
    if (kg > 0.0f) {
      runCalibration(kg);
    } else {
      logPrintln("Usage: m <mass_kg>");
    }
  } 
  else {
    logPrint("Unknown UART command: "); 
    logPrintln(cmd);
  }
}

bool waitForInput(String &cmd, unsigned long timeoutMs = 30000) {
  unsigned long start = millis();
  cmd = "";

  while ((millis() - start) < timeoutMs) {

    // Serial input
    if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();
      return true;
    }

    // BLE UART input (from RX callback buffer)
    int newlineIndex = uartBuffer.indexOf('\n');
    if (newlineIndex >= 0) {
      cmd = uartBuffer.substring(0, newlineIndex);
      uartBuffer.remove(0, newlineIndex + 1);
      cmd.trim();
      return true;
    }

    delay(10);  // BLE stack runs in background
  }

  return false;
}
