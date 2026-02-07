// Print helpers (Serial + BLE UART)
void logPrint(const String &msg) {
  Serial.print(msg);

  // Send to BLE UART if a device is connected
  if (Bluefruit.connected()){
    uartTXChar.notify((const uint8_t*)msg.c_str(), msg.length());
  }
}

void logPrintln(const String &msg) {
  Serial.println(msg);

  // Send to BLE UART if a device is connected
  if (Bluefruit.connected()){
    String fullMsg = msg + "\r\n";
    uartTXChar.notify((const uint8_t*)fullMsg.c_str(), fullMsg.length());
  }
}


