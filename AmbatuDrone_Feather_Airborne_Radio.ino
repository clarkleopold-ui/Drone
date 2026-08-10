#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

/*
  AmbatuDrone airborne radio
  Target: Adafruit Feather M0 with integrated RFM95 900 MHz radio

  Ground Feather <-> 915 MHz radio <-> this Feather <-> Serial1 <-> Teensy 4.1

  UART wiring:
    Feather TX -> Teensy pin 0 / RX1
    Feather RX <- Teensy pin 1 / TX1
    Feather GND -- Teensy GND

  Never connect the Feather or Teensy directly to the 6S battery. Use the
  regulated power arrangement described in WIRING_AND_POWER.txt.
*/

constexpr uint8_t RFM95_CS = 8;
constexpr uint8_t RFM95_RST = 4;
constexpr uint8_t RFM95_INT = 3;
constexpr float RFM95_FREQUENCY_MHZ = 915.0f;
constexpr int8_t RFM95_TX_POWER_DBM = 20;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t TEENSY_UART_BAUD = 115200;
constexpr size_t MAX_TELEMETRY_LINE = RH_RF95_MAX_MESSAGE_LEN - 1u;

RH_RF95 radio(RFM95_CS, RFM95_INT);
char telemetryLine[MAX_TELEMETRY_LINE + 1u] = "";
char latestTelemetry[MAX_TELEMETRY_LINE + 1u] = "";
size_t telemetryIndex = 0;
bool telemetryOverflow = false;
bool haveTelemetry = false;

void resetRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

bool initializeRadio() {
  resetRadio();
  if (!radio.init()) {
    return false;
  }
  if (!radio.setFrequency(RFM95_FREQUENCY_MHZ)) {
    return false;
  }
  radio.setTxPower(RFM95_TX_POWER_DBM, false);
  radio.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  radio.setPreambleLength(8);
  return true;
}

void pollTeensyTelemetry() {
  while (Serial1.available() > 0) {
    const char incoming = static_cast<char>(Serial1.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (!telemetryOverflow && telemetryIndex > 4u) {
        telemetryLine[telemetryIndex] = '\0';
        if (strncmp(telemetryLine, "ACK ", 4) == 0) {
          memcpy(latestTelemetry, telemetryLine, telemetryIndex + 1u);
          haveTelemetry = true;
        }
      }
      telemetryIndex = 0;
      telemetryOverflow = false;
      continue;
    }

    if (!telemetryOverflow) {
      if (telemetryIndex < MAX_TELEMETRY_LINE) {
        telemetryLine[telemetryIndex++] = incoming;
      } else {
        telemetryIndex = 0;
        telemetryOverflow = true;
      }
    }
  }
}

void sendLatestTelemetry() {
  if (!haveTelemetry) {
    return;
  }
  const size_t telemetryLength = strlen(latestTelemetry);
  if (telemetryLength + 1u > RH_RF95_MAX_MESSAGE_LEN) {
    // Dropping an oversized ACK is safer than truncating its safety flags.
    return;
  }
  uint8_t packet[RH_RF95_MAX_MESSAGE_LEN];
  packet[0] = 'T';
  memcpy(packet + 1, latestTelemetry, telemetryLength);
  radio.send(packet, static_cast<uint8_t>(telemetryLength + 1u));
  radio.waitPacketSent();
}

void handleRadioPacket() {
  if (!radio.available()) {
    return;
  }
  uint8_t packet[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t packetLength = sizeof(packet);
  if (!radio.recv(packet, &packetLength) || packetLength < 1u) {
    return;
  }

  if (packet[0] == 'C' && packetLength >= 3u && packet[1] == '{' &&
      packet[packetLength - 1u] == '}') {
    Serial1.write(packet + 1, packetLength - 1u);
    Serial1.write('\n');
  } else if (packet[0] != 'P') {
    return;
  }

  // Give the Teensy a short opportunity to produce a fresh 25 Hz ACK.
  const uint32_t waitStarted = millis();
  while (millis() - waitStarted < 4u) {
    pollTeensyTelemetry();
  }
  sendLatestTelemetry();
}

void setup() {
  Serial.begin(USB_BAUD);
  Serial1.begin(TEENSY_UART_BAUD);
  if (!initializeRadio()) {
    Serial.println("ERROR RFM95 initialization failed.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("AmbatuDrone airborne Feather ready.");
}

void loop() {
  pollTeensyTelemetry();
  handleRadioPacket();
}
