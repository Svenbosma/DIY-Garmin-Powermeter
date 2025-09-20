// Print helpers (Serial + BLE UART)
void logPrint(const String &msg) {
  Serial.print(msg);
  if (Bluefruit.connected()) bleUart.print(msg);
}
void logPrintln(const String &msg) {
  Serial.println(msg);
  if (Bluefruit.connected()) bleUart.println(msg);
}

