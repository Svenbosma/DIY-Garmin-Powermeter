// Mirror log output to BLE UART when connected.
static const uint16_t UART_NOTIFY_MAX_BYTES = 20;
static const uint8_t UART_NOTIFY_RETRY_COUNT = 8;
static const uint8_t UART_NOTIFY_RETRY_DELAY_MS = 2;

void uartNotifyChunked(const String &msg) {
  if (!Bluefruit.connected() || !uartTXChar.notifyEnabled()) {
    return;
  }

  const char *text = msg.c_str();
  uint16_t len = msg.length();
  uint16_t offset = 0;

  while (offset < len) {
    uint16_t chunkLen = len - offset;
    if (chunkLen > UART_NOTIFY_MAX_BYTES) {
      chunkLen = UART_NOTIFY_MAX_BYTES;
    }

    bool sent = false;
    for (uint8_t attempt = 0; attempt < UART_NOTIFY_RETRY_COUNT && !sent; attempt++) {
      sent = uartTXChar.notify((const uint8_t*)(text + offset), chunkLen);
      if (!sent) {
        delay(UART_NOTIFY_RETRY_DELAY_MS);
      }
    }

    offset += chunkLen;
    delay(UART_NOTIFY_RETRY_DELAY_MS);
  }
}

void logPrint(const String &msg) {
  if (!loggingEnabled) {
    return;
  }

  uartNotifyChunked(msg);
}

void logPrintln(const String &msg) {
  if (!loggingEnabled) {
    return;
  }

  uartNotifyChunked(msg + "\r\n");
}


