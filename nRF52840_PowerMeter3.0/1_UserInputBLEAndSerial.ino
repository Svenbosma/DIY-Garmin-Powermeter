// Buffer to collect incoming UART data
String uartBuffer = "";

// Call this regularly from loop()
void handleUART() {
  // Check if RX characteristic received new data
  if (uartRXChar.written()) {
    int len = uartRXChar.valueLength();
    if (len > 0) {
      // Read incoming data
      char data[len + 1];
      uartRXChar.readValue((unsigned char*)data, len);
      data[len] = '\0';
      
      uartBuffer += String(data);

      // Commands end with newline '\n'
      int newlineIndex = uartBuffer.indexOf('\n');
      while (newlineIndex >= 0) {
        String cmd = uartBuffer.substring(0, newlineIndex);
        uartBuffer.remove(0, newlineIndex + 1);
        processUARTCommand(cmd);
        newlineIndex = uartBuffer.indexOf('\n');
      }
    }
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

// Wait for input from Serial or BLE UART
bool waitForInput(String &cmd, unsigned long timeoutMs = 30000) {
  unsigned long start = millis();
  cmd = "";

  while ((millis() - start) < timeoutMs) {
    // Check Serial input
    if (Serial.available()) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();
      return true;
    }

    // Check BLE input
    if (uartRXChar.written()) {
      int len = uartRXChar.valueLength();
      if (len > 0) {
        char data[len + 1];
        uartRXChar.readValue((unsigned char*)data, len);
        data[len] = '\0';
        cmd = String(data);
        cmd.trim();
        return true;
      }
    }

    BLE.poll(); // keep BLE stack alive
    delay(10);
  }
  return false;
}
