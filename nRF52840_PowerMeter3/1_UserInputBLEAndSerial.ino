// Collect newline-terminated BLE UART commands.
String uartBuffer = "";
volatile bool uartCommandPending = false;

void uartRXWriteCallback(uint16_t conn_hdl,
                         BLECharacteristic* chr,
                         uint8_t* data,
                         uint16_t len)
{
  if (len == 0) return;

  // Append the latest BLE write to the command buffer.
  for (uint16_t i = 0; i < len; i++) {
    uartBuffer += (char)data[i];
  }

  // Defer command execution to loop() so BLE callbacks stay short.
  uartCommandPending = true;
}

void serviceUARTCommands() {
  if (!uartCommandPending) return;

  calibrationActive = true;

  int newlineIndex = uartBuffer.indexOf('\n');
  while (newlineIndex >= 0) {
    String cmd = uartBuffer.substring(0, newlineIndex);
    uartBuffer.remove(0, newlineIndex + 1);
    processUARTCommand(cmd);
    newlineIndex = uartBuffer.indexOf('\n');
  }

  calibrationActive = false;
  uartCommandPending = (uartBuffer.indexOf('\n') >= 0);
}

// Handle calibration, tare and maintenance commands.
void processUARTCommand(String cmd) {
  cmd.trim();
  logPrint("UART command received: "); 
  logPrintln(cmd);

  if (cmd.equalsIgnoreCase("c")) {
    runCalibration(10.0f);
  } 
  else if (cmd.equalsIgnoreCase("t")) {
    doTare();
    doTareGyro();
  } 
  else if (cmd.startsWith("m")) {
    String massStr = cmd.substring(1);
    massStr.trim();
    float kg = massStr.toFloat();
    if (kg > 0.0f) {
      runCalibration(kg);
    } else {
      logPrintln("Usage: m <mass_kg>");
    }
  } 
  else if (cmd.equalsIgnoreCase("dfu")) { 
    NRF_POWER->GPREGRET = 0xA8; 
    NVIC_SystemReset(); 
    } 
  else {
    logPrint("Unknown UART command: "); 
    logPrintln(cmd);
  }
}
