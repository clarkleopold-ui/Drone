#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

/*
  AmbatuDrone ground radio
  Target: Adafruit Feather M0 with integrated RFM95 900 MHz radio

  Computer AmbatuDrone app <-> USB Serial <-> this Feather <-> 915 MHz radio

  The app opens this Feather's USB serial port at 115200 baud. The sketch
  forwards newline-terminated JSON controller packets to the airborne Feather
  and prints returned ACK telemetry back to the app.

  Install the RadioHead library before compiling. Both Feathers must use the
  same frequency and modem configuration, and both must have antennas fitted.
*/

constexpr uint8_t RFM95_CS = 8;
constexpr uint8_t RFM95_RST = 4;
constexpr uint8_t RFM95_INT = 3;
constexpr float RFM95_FREQUENCY_MHZ = 915.0f;
constexpr int8_t RFM95_TX_POWER_DBM = 20;
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t TELEMETRY_POLL_MS = 40;
constexpr uint16_t RADIO_REPLY_TIMEOUT_MS = 28;
constexpr size_t MAX_SERIAL_LINE = 180;

RH_RF95 radio(RFM95_CS, RFM95_INT);
char serialLine[MAX_SERIAL_LINE] = "";
size_t serialIndex = 0;
bool serialOverflow = false;
uint32_t lastTransactionMs = 0;

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

bool transact(const uint8_t* outgoing, uint8_t outgoingLength) {
  if (!radio.send(outgoing, outgoingLength)) {
    return false;
  }
  radio.waitPacketSent();

  if (!radio.waitAvailableTimeout(RADIO_REPLY_TIMEOUT_MS)) {
    return false;
  }

  uint8_t incoming[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t incomingLength = sizeof(incoming);
  if (!radio.recv(incoming, &incomingLength) || incomingLength < 2 ||
      incoming[0] != 'T') {
    return false;
  }

  Serial.write(incoming + 1, incomingLength - 1);
  Serial.write('\n');
  return true;
}

void sendControllerLine(const char* line) {
  const size_t lineLength = strlen(line);
  if (lineLength + 1u > RH_RF95_MAX_MESSAGE_LEN) {
    Serial.println("ERROR Controller packet is too long for the radio.");
    return;
  }

  uint8_t packet[RH_RF95_MAX_MESSAGE_LEN];
  packet[0] = 'C';
  memcpy(packet + 1, line, lineLength);
  transact(packet, static_cast<uint8_t>(lineLength + 1u));
  lastTransactionMs = millis();
}

void pollTelemetry() {
  const uint8_t pollPacket[1] = {'P'};
  transact(pollPacket, sizeof(pollPacket));
  lastTransactionMs = millis();
}

void pollComputerSerial() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (!serialOverflow && serialIndex > 0) {
        serialLine[serialIndex] = '\0';
        sendControllerLine(serialLine);
      }
      serialIndex = 0;
      serialOverflow = false;
      continue;
    }

    if (!serialOverflow) {
      if (serialIndex < MAX_SERIAL_LINE - 1u) {
        serialLine[serialIndex++] = incoming;
      } else {
        serialIndex = 0;
        serialOverflow = true;
      }
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t waitStarted = millis();
  while (!Serial && millis() - waitStarted < 3000u) {
  }

  if (!initializeRadio()) {
    Serial.println("ERROR RFM95 initialization failed.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("AmbatuDrone ground Feather ready.");
  lastTransactionMs = millis() - TELEMETRY_POLL_MS;
}

void loop() {
  pollComputerSerial();
  if (millis() - lastTransactionMs >= TELEMETRY_POLL_MS) {
    pollTelemetry();
  }
}
