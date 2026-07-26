#include <Arduino.h>
#include "BluetoothSerial.h"

// AmbatuDrone transparent Bluetooth Classic (SPP) <-> UART bridge.
//
// The ESP32 does not parse commands or telemetry. It forwards the exact bytes:
//   Windows Bluetooth COM -> SerialBT -> Serial2 -> Arduino Mega Serial1
//   Arduino Mega Serial1  -> Serial2  -> SerialBT -> Windows Bluetooth COM

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled for the selected ESP32 board.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is unavailable. Select an original ESP32 board, not an ESP32-C3/S3.
#endif

BluetoothSerial SerialBT;

constexpr char DEVICE_NAME[] = "AmbatuDrone";
constexpr uint32_t USB_DEBUG_BAUD = 115200;
constexpr uint32_t MEGA_UART_BAUD = 115200;

// ESP32 UART2 pins. These names describe the ESP32 side of each connection.
constexpr int ESP_RX_FROM_MEGA = 16;
constexpr int ESP_TX_TO_MEGA = 17;

void setup() {
  Serial.begin(USB_DEBUG_BAUD);
  Serial2.begin(
    MEGA_UART_BAUD,
    SERIAL_8N1,
    ESP_RX_FROM_MEGA,
    ESP_TX_TO_MEGA
  );

  if (!SerialBT.begin(DEVICE_NAME)) {
    Serial.println("ERROR: Bluetooth SPP failed to start.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("AmbatuDrone Bluetooth bridge ready.");
  Serial.println("Pair Windows with Bluetooth device: AmbatuDrone");
  Serial.println("UART2: RX=GPIO16, TX=GPIO17, 115200 baud");
}

void loop() {
  // Computer/Python -> Bluetooth -> ESP32 -> Mega.
  while (SerialBT.available() > 0) {
    const int incoming = SerialBT.read();
    if (incoming >= 0) {
      Serial2.write((uint8_t)incoming);
    }
  }

  // Mega -> ESP32 -> Bluetooth -> computer/Python.
  while (Serial2.available() > 0) {
    const int incoming = Serial2.read();
    if (incoming >= 0) {
      SerialBT.write((uint8_t)incoming);
    }
  }

  // Yield briefly to the ESP32 Bluetooth stack without slowing the 25 Hz link.
  delay(1);
}
