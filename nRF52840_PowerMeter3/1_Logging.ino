// Mirror log output to BLE UART when connected.
void logPrint(const String &msg) {
  if (Bluefruit.connected()){
    uartTXChar.notify((const uint8_t*)msg.c_str(), msg.length());
  }
}

void logPrintln(const String &msg) {
  if (Bluefruit.connected()){
    String fullMsg = msg + "\r\n";
    uartTXChar.notify((const uint8_t*)fullMsg.c_str(), fullMsg.length());
  }
}


