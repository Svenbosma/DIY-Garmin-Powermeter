// Mirror log output to Serial and BLE UART when connected.
void logPrint(const String &msg) {
  Serial.print(msg);

  // BLE UART gives you the same diagnostics wirelessly.
  if (Bluefruit.connected()){
    uartTXChar.notify((const uint8_t*)msg.c_str(), msg.length());
  }
}

void logPrintln(const String &msg) {
  Serial.println(msg);

  // Add the newline when mirroring over BLE.
  if (Bluefruit.connected()){
    String fullMsg = msg + "\r\n";
    uartTXChar.notify((const uint8_t*)fullMsg.c_str(), fullMsg.length());
  }
}


