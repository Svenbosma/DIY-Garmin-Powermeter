// Print helpers (Serial + BLE UART)
void logPrint(const String &msg) {
  Serial.print(msg);

  // Send to BLE UART if a device is connected
  if (BLE.connected()) {
    uartTXChar.writeValue((const unsigned char*)msg.c_str(), msg.length());
  }
}

void logPrintln(const String &msg) {
  Serial.println(msg);

  // Send to BLE UART if a device is connected
  if (BLE.connected()) {
    String fullMsg = msg + "\r\n";
    uartTXChar.writeValue((const unsigned char*)fullMsg.c_str(), fullMsg.length());
  }
}


