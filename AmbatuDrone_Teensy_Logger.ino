#include <Arduino.h>
#include <ADC.h>
#include <IntervalTimer.h>
#include <SD.h>
#include <SPI.h>

/*
  AmbatuDrone synchronized motor-health logger
  Target: Teensy 4.1 with a microSD card in the built-in slot

  WAV channel 1: microphone
  WAV channel 2: ACS758LCB-050B current sensor

  The logger is intentionally separate from the Arduino Mega flight controller.
  It samples both analog channels at the same instant and stores them as one
  stereo, 16-bit, 48 kHz WAV file. A matching JSON file records calibration and
  run information.

  Serial commands at 115200 baud:
    help
    status
    zero    (motors must be off)
    start
    stop

  Optional record button:
    Teensy pin 2 to GND, using the internal pull-up.
*/

// -------------------- User configuration --------------------
constexpr uint8_t MICROPHONE_PIN = A2;       // ADC0 channel in synchronized read
constexpr uint8_t CURRENT_SENSOR_PIN = A3;   // ADC1 channel in synchronized read
constexpr uint8_t RECORD_BUTTON_PIN = 2;     // Momentary button to GND
constexpr uint8_t RECORD_LED_PIN = LED_BUILTIN;

constexpr uint32_t SAMPLE_RATE_HZ = 48000;
constexpr float SAMPLE_PERIOD_US = 1000000.0f / SAMPLE_RATE_HZ;
constexpr uint8_t ADC_BITS = 12;
constexpr uint16_t ADC_MAX_VALUE = (1u << ADC_BITS) - 1u;
constexpr uint16_t ADC_MIDPOINT = 1u << (ADC_BITS - 1u);

// The ACS758LCB-050B is 40 mV/A at 5 V and is ratiometric.
// When the sensor supply and ADC reference scale together, this is about
// 32.76 ADC counts per amp at 12-bit resolution.
constexpr float ACS758_MV_PER_AMP_AT_5V = 40.0f;
constexpr float CURRENT_COUNTS_PER_AMP =
    ADC_MAX_VALUE * (ACS758_MV_PER_AMP_AT_5V / 1000.0f) / 5.0f;

// Designed for short motor-health trials. A run automatically stops at 120 s.
constexpr uint32_t MAX_RUN_SECONDS = 120;
constexpr uint64_t MAX_DATA_BYTES =
    static_cast<uint64_t>(SAMPLE_RATE_HZ) * 2u * sizeof(int16_t) *
    MAX_RUN_SECONDS;
constexpr uint64_t PREALLOCATE_BYTES = 44u + MAX_DATA_BYTES;

// 16,384 frames = about 341 ms of buffering at 48 kHz.
constexpr uint32_t RING_CAPACITY = 16384;
constexpr uint32_t RING_MASK = RING_CAPACITY - 1u;
constexpr uint32_t WRITE_BLOCK_FRAMES = 1024;
static_assert((RING_CAPACITY & (RING_CAPACITY - 1u)) == 0,
              "RING_CAPACITY must be a power of two");

constexpr uint16_t CLIP_LOW_RAW = 8;
constexpr uint16_t CLIP_HIGH_RAW = ADC_MAX_VALUE - 8;
constexpr uint16_t CURRENT_ZERO_MIN_RAW = 1200;
constexpr uint16_t CURRENT_ZERO_MAX_RAW = 2900;
constexpr uint32_t ZERO_CALIBRATION_SAMPLES = 4096;

// -------------------- Types and global state --------------------
struct __attribute__((packed)) StereoFrame {
  int16_t microphone;
  int16_t current;
};
static_assert(sizeof(StereoFrame) == 4, "Stereo WAV frames must be 4 bytes");

ADC adc;
IntervalTimer sampleTimer;
FsFile wavFile;

StereoFrame ringBuffer[RING_CAPACITY];
volatile uint32_t ringHead = 0;
volatile uint32_t ringTail = 0;

volatile bool recording = false;
volatile uint32_t capturedFrames = 0;
volatile uint32_t droppedFrames = 0;
volatile uint32_t adcErrorCount = 0;
volatile uint32_t microphoneClipCount = 0;
volatile uint32_t currentClipCount = 0;
volatile uint16_t microphoneMinRaw = ADC_MAX_VALUE;
volatile uint16_t microphoneMaxRaw = 0;
volatile uint16_t currentMinRaw = ADC_MAX_VALUE;
volatile uint16_t currentMaxRaw = 0;

uint64_t writtenFrames = 0;
uint16_t currentZeroRaw = ADC_MIDPOINT;
bool currentZeroValid = false;
bool sdReady = false;
bool writeFailed = false;
bool preallocationSucceeded = false;

char wavFilename[20] = "";
char metadataFilename[20] = "";

char commandBuffer[32] = "";
uint8_t commandIndex = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
uint32_t buttonChangedMs = 0;
uint32_t lastStatusMs = 0;

// -------------------- Low-level helpers --------------------
int16_t rawAdcToPcm(int32_t raw) {
  int32_t centered = raw - static_cast<int32_t>(ADC_MIDPOINT);
  centered <<= (16 - ADC_BITS);
  if (centered > INT16_MAX) {
    centered = INT16_MAX;
  } else if (centered < INT16_MIN) {
    centered = INT16_MIN;
  }
  return static_cast<int16_t>(centered);
}

void writeLittleEndian16(FsFile& file, uint16_t value) {
  uint8_t bytes[2] = {
      static_cast<uint8_t>(value & 0xFFu),
      static_cast<uint8_t>((value >> 8u) & 0xFFu),
  };
  file.write(bytes, sizeof(bytes));
}

void writeLittleEndian32(FsFile& file, uint32_t value) {
  uint8_t bytes[4] = {
      static_cast<uint8_t>(value & 0xFFu),
      static_cast<uint8_t>((value >> 8u) & 0xFFu),
      static_cast<uint8_t>((value >> 16u) & 0xFFu),
      static_cast<uint8_t>((value >> 24u) & 0xFFu),
  };
  file.write(bytes, sizeof(bytes));
}

bool writeWavHeader(FsFile& file, uint32_t dataBytes) {
  constexpr uint16_t channelCount = 2;
  constexpr uint16_t bitsPerSample = 16;
  constexpr uint16_t blockAlign =
      channelCount * static_cast<uint16_t>(bitsPerSample / 8u);
  constexpr uint32_t byteRate = SAMPLE_RATE_HZ * blockAlign;
  const uint32_t riffSize = 36u + dataBytes;

  if (!file.seekSet(0)) {
    return false;
  }

  file.write("RIFF", 4);
  writeLittleEndian32(file, riffSize);
  file.write("WAVE", 4);
  file.write("fmt ", 4);
  writeLittleEndian32(file, 16);            // PCM format chunk size
  writeLittleEndian16(file, 1);             // PCM format
  writeLittleEndian16(file, channelCount);
  writeLittleEndian32(file, SAMPLE_RATE_HZ);
  writeLittleEndian32(file, byteRate);
  writeLittleEndian16(file, blockAlign);
  writeLittleEndian16(file, bitsPerSample);
  file.write("data", 4);
  writeLittleEndian32(file, dataBytes);

  return file.getWriteError() == 0;
}

bool chooseRunFilenames() {
  for (uint16_t run = 1; run <= 9999; ++run) {
    snprintf(wavFilename, sizeof(wavFilename), "RUN%04u.WAV", run);
    if (!SD.exists(wavFilename)) {
      snprintf(metadataFilename, sizeof(metadataFilename), "RUN%04u.JSON", run);
      return true;
    }
  }
  return false;
}

void resetRunState() {
  noInterrupts();
  ringHead = 0;
  ringTail = 0;
  capturedFrames = 0;
  droppedFrames = 0;
  adcErrorCount = 0;
  microphoneClipCount = 0;
  currentClipCount = 0;
  microphoneMinRaw = ADC_MAX_VALUE;
  microphoneMaxRaw = 0;
  currentMinRaw = ADC_MAX_VALUE;
  currentMaxRaw = 0;
  interrupts();

  writtenFrames = 0;
  writeFailed = false;
  preallocationSucceeded = false;
}

// -------------------- Sampling --------------------
void sampleIsr() {
  if (!recording) {
    return;
  }

  const ADC::Sync_result result =
      adc.analogSynchronizedRead(MICROPHONE_PIN, CURRENT_SENSOR_PIN);

  if (result.result_adc0 < 0 || result.result_adc0 > ADC_MAX_VALUE ||
      result.result_adc1 < 0 || result.result_adc1 > ADC_MAX_VALUE) {
    ++adcErrorCount;
    return;
  }

  const uint16_t microphoneRaw =
      static_cast<uint16_t>(result.result_adc0);
  const uint16_t currentRaw =
      static_cast<uint16_t>(result.result_adc1);

  if (microphoneRaw < microphoneMinRaw) {
    microphoneMinRaw = microphoneRaw;
  }
  if (microphoneRaw > microphoneMaxRaw) {
    microphoneMaxRaw = microphoneRaw;
  }
  if (currentRaw < currentMinRaw) {
    currentMinRaw = currentRaw;
  }
  if (currentRaw > currentMaxRaw) {
    currentMaxRaw = currentRaw;
  }

  if (microphoneRaw <= CLIP_LOW_RAW || microphoneRaw >= CLIP_HIGH_RAW) {
    ++microphoneClipCount;
  }
  if (currentRaw <= CLIP_LOW_RAW || currentRaw >= CLIP_HIGH_RAW) {
    ++currentClipCount;
  }

  const uint32_t nextHead = (ringHead + 1u) & RING_MASK;
  if (nextHead == ringTail) {
    ++droppedFrames;
    return;
  }

  ringBuffer[ringHead].microphone = rawAdcToPcm(microphoneRaw);
  ringBuffer[ringHead].current = rawAdcToPcm(currentRaw);
  ringHead = nextHead;
  ++capturedFrames;
}

bool drainRingBuffer() {
  if (!wavFile || writeFailed) {
    return false;
  }

  uint32_t tailSnapshot;
  uint32_t headSnapshot;

  noInterrupts();
  tailSnapshot = ringTail;
  headSnapshot = ringHead;
  interrupts();

  if (tailSnapshot == headSnapshot) {
    return true;
  }

  uint32_t availableFrames;
  if (headSnapshot > tailSnapshot) {
    availableFrames = headSnapshot - tailSnapshot;
  } else {
    availableFrames = RING_CAPACITY - tailSnapshot;
  }

  const uint32_t framesToWrite =
      min(availableFrames, WRITE_BLOCK_FRAMES);
  const size_t bytesToWrite =
      static_cast<size_t>(framesToWrite) * sizeof(StereoFrame);

  const size_t bytesWritten = wavFile.write(
      reinterpret_cast<const uint8_t*>(&ringBuffer[tailSnapshot]),
      bytesToWrite);

  if (bytesWritten != bytesToWrite || wavFile.getWriteError() != 0) {
    writeFailed = true;
    return false;
  }

  noInterrupts();
  ringTail = (tailSnapshot + framesToWrite) & RING_MASK;
  interrupts();

  writtenFrames += framesToWrite;
  return true;
}

// -------------------- Calibration and metadata --------------------
bool calibrateCurrentZero() {
  if (recording) {
    Serial.println("ERROR Stop recording before zero calibration.");
    return false;
  }

  Serial.println(
      "ZERO Keep all motors off and do not move the wiring...");
  delay(250);

  uint64_t sum = 0;
  uint32_t validSamples = 0;

  for (uint32_t i = 0; i < ZERO_CALIBRATION_SAMPLES; ++i) {
    const ADC::Sync_result result =
        adc.analogSynchronizedRead(MICROPHONE_PIN, CURRENT_SENSOR_PIN);
    if (result.result_adc1 >= 0 &&
        result.result_adc1 <= ADC_MAX_VALUE) {
      sum += static_cast<uint16_t>(result.result_adc1);
      ++validSamples;
    }
    delayMicroseconds(20);
  }

  if (validSamples < ZERO_CALIBRATION_SAMPLES / 2u) {
    currentZeroValid = false;
    Serial.println("ERROR Current ADC did not return enough valid samples.");
    return false;
  }

  currentZeroRaw =
      static_cast<uint16_t>((sum + validSamples / 2u) / validSamples);
  currentZeroValid =
      currentZeroRaw >= CURRENT_ZERO_MIN_RAW &&
      currentZeroRaw <= CURRENT_ZERO_MAX_RAW;

  Serial.print("ZERO current_zero_adc=");
  Serial.print(currentZeroRaw);
  Serial.print(" estimated_zero_amps=");
  Serial.println(
      (static_cast<float>(currentZeroRaw) - ADC_MIDPOINT) /
          CURRENT_COUNTS_PER_AMP,
      3);

  if (!currentZeroValid) {
    Serial.println(
        "ERROR Zero value is outside the expected ACS758 range. "
        "Check 3.3 V, GND, OUT, and the A3 connection.");
  }
  return currentZeroValid;
}

bool writeMetadataFile(uint32_t dataBytes) {
  FsFile metadata =
      SD.sdfs.open(metadataFilename, O_WRITE | O_CREAT | O_TRUNC);
  if (!metadata) {
    return false;
  }

  uint32_t finalCaptured;
  uint32_t finalDropped;
  uint32_t finalAdcErrors;
  uint32_t finalMicClips;
  uint32_t finalCurrentClips;
  uint16_t finalMicMin;
  uint16_t finalMicMax;
  uint16_t finalCurrentMin;
  uint16_t finalCurrentMax;

  noInterrupts();
  finalCaptured = capturedFrames;
  finalDropped = droppedFrames;
  finalAdcErrors = adcErrorCount;
  finalMicClips = microphoneClipCount;
  finalCurrentClips = currentClipCount;
  finalMicMin = microphoneMinRaw;
  finalMicMax = microphoneMaxRaw;
  finalCurrentMin = currentMinRaw;
  finalCurrentMax = currentMaxRaw;
  interrupts();

  metadata.println("{");
  metadata.print("  \"wav_file\": \"");
  metadata.print(wavFilename);
  metadata.println("\",");
  metadata.println("  \"format\": \"stereo_pcm_s16le\",");
  metadata.print("  \"sample_rate_hz\": ");
  metadata.print(SAMPLE_RATE_HZ);
  metadata.println(",");
  metadata.println("  \"channel_1\": \"microphone\",");
  metadata.println("  \"channel_2\": \"current_sensor\",");
  metadata.print("  \"adc_bits\": ");
  metadata.print(ADC_BITS);
  metadata.println(",");
  metadata.print("  \"adc_midpoint\": ");
  metadata.print(ADC_MIDPOINT);
  metadata.println(",");
  metadata.print("  \"current_sensor\": \"ACS758LCB-050B\",");
  metadata.println();
  metadata.print("  \"current_zero_adc\": ");
  metadata.print(currentZeroRaw);
  metadata.println(",");
  metadata.print("  \"current_counts_per_amp\": ");
  metadata.print(CURRENT_COUNTS_PER_AMP, 6);
  metadata.println(",");
  metadata.print("  \"microphone_distance_mm\": 7.5,");
  metadata.println();
  metadata.println(
      "  \"current_path\": \"battery_positive_to_esc\",");
  metadata.print("  \"captured_frames\": ");
  metadata.print(finalCaptured);
  metadata.println(",");
  metadata.print("  \"written_frames\": ");
  metadata.print(static_cast<uint32_t>(writtenFrames));
  metadata.println(",");
  metadata.print("  \"data_bytes\": ");
  metadata.print(dataBytes);
  metadata.println(",");
  metadata.print("  \"duration_seconds\": ");
  metadata.print(
      static_cast<double>(writtenFrames) / SAMPLE_RATE_HZ, 6);
  metadata.println(",");
  metadata.print("  \"dropped_frames\": ");
  metadata.print(finalDropped);
  metadata.println(",");
  metadata.print("  \"adc_error_count\": ");
  metadata.print(finalAdcErrors);
  metadata.println(",");
  metadata.print("  \"microphone_clip_count\": ");
  metadata.print(finalMicClips);
  metadata.println(",");
  metadata.print("  \"current_clip_count\": ");
  metadata.print(finalCurrentClips);
  metadata.println(",");
  metadata.print("  \"microphone_min_adc\": ");
  metadata.print(finalMicMin);
  metadata.println(",");
  metadata.print("  \"microphone_max_adc\": ");
  metadata.print(finalMicMax);
  metadata.println(",");
  metadata.print("  \"current_min_adc\": ");
  metadata.print(finalCurrentMin);
  metadata.println(",");
  metadata.print("  \"current_max_adc\": ");
  metadata.print(finalCurrentMax);
  metadata.println(",");
  metadata.print("  \"preallocation_succeeded\": ");
  metadata.print(preallocationSucceeded ? "true" : "false");
  metadata.println(",");
  metadata.print("  \"write_failed\": ");
  metadata.println(writeFailed ? "true" : "false");
  metadata.println("}");

  const bool ok = metadata.getWriteError() == 0;
  metadata.close();
  return ok;
}

// -------------------- Recording control --------------------
bool startRecording() {
  if (recording) {
    Serial.println("INFO Recording is already active.");
    return true;
  }
  if (!sdReady) {
    Serial.println("ERROR microSD is not ready.");
    return false;
  }
  if (!currentZeroValid) {
    Serial.println(
        "ERROR Current sensor is not zeroed. Turn all motors off and send: zero");
    return false;
  }
  if (!chooseRunFilenames()) {
    Serial.println("ERROR No unused RUN0001-RUN9999 filename is available.");
    return false;
  }

  resetRunState();
  wavFile = SD.sdfs.open(
      wavFilename, O_RDWR | O_CREAT | O_TRUNC);
  if (!wavFile) {
    Serial.print("ERROR Could not create ");
    Serial.println(wavFilename);
    return false;
  }

  preallocationSucceeded = wavFile.preAllocate(PREALLOCATE_BYTES);
  if (!preallocationSucceeded) {
    Serial.println(
        "WARNING File preallocation failed; recording will continue "
        "using the RAM ring buffer.");
  }

  if (!writeWavHeader(wavFile, 0) || !wavFile.seekSet(44)) {
    Serial.println("ERROR Could not write the WAV header.");
    wavFile.close();
    return false;
  }

  digitalWriteFast(RECORD_LED_PIN, HIGH);
  recording = true;
  sampleTimer.priority(64);
  if (!sampleTimer.begin(sampleIsr, SAMPLE_PERIOD_US)) {
    recording = false;
    digitalWriteFast(RECORD_LED_PIN, LOW);
    wavFile.close();
    Serial.println("ERROR No IntervalTimer resource is available.");
    return false;
  }

  Serial.print("RECORDING file=");
  Serial.print(wavFilename);
  Serial.print(" sample_rate=");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.println(" channels=2");
  return true;
}

void stopRecording(const char* reason) {
  if (!recording && !wavFile) {
    Serial.println("INFO Recording is not active.");
    return;
  }

  recording = false;
  sampleTimer.end();
  digitalWriteFast(RECORD_LED_PIN, LOW);

  while (ringTail != ringHead && !writeFailed) {
    drainRingBuffer();
  }

  const uint64_t dataBytes64 =
      writtenFrames * static_cast<uint64_t>(sizeof(StereoFrame));
  const uint32_t dataBytes =
      dataBytes64 > UINT32_MAX ? UINT32_MAX
                              : static_cast<uint32_t>(dataBytes64);
  const uint64_t finalFileBytes = 44u + dataBytes64;

  bool finalized = !writeFailed;
  if (wavFile) {
    if (!wavFile.truncate(finalFileBytes)) {
      finalized = false;
    }
    if (!writeWavHeader(wavFile, dataBytes)) {
      finalized = false;
    }
    if (!wavFile.sync()) {
      finalized = false;
    }
    wavFile.close();
  }

  const bool metadataOk = writeMetadataFile(dataBytes);

  Serial.print("STOPPED reason=");
  Serial.print(reason);
  Serial.print(" file=");
  Serial.print(wavFilename);
  Serial.print(" seconds=");
  Serial.print(static_cast<double>(writtenFrames) / SAMPLE_RATE_HZ, 3);
  Serial.print(" dropped=");
  Serial.print(droppedFrames);
  Serial.print(" adc_errors=");
  Serial.print(adcErrorCount);
  Serial.print(" mic_clips=");
  Serial.print(microphoneClipCount);
  Serial.print(" current_clips=");
  Serial.println(currentClipCount);

  if (!finalized || !metadataOk) {
    Serial.println(
        "ERROR The recording stopped, but file finalization was not clean.");
  } else {
    Serial.print("SAVED ");
    Serial.print(wavFilename);
    Serial.print(" and ");
    Serial.println(metadataFilename);
  }
}

void printStatus() {
  uint32_t headSnapshot;
  uint32_t tailSnapshot;
  uint32_t capturedSnapshot;
  uint32_t droppedSnapshot;
  uint32_t adcErrorsSnapshot;

  noInterrupts();
  headSnapshot = ringHead;
  tailSnapshot = ringTail;
  capturedSnapshot = capturedFrames;
  droppedSnapshot = droppedFrames;
  adcErrorsSnapshot = adcErrorCount;
  interrupts();

  const uint32_t queuedFrames =
      (headSnapshot - tailSnapshot) & RING_MASK;

  Serial.print("STATUS recording=");
  Serial.print(recording ? 1 : 0);
  Serial.print(" sd_ready=");
  Serial.print(sdReady ? 1 : 0);
  Serial.print(" current_zero_valid=");
  Serial.print(currentZeroValid ? 1 : 0);
  Serial.print(" current_zero_adc=");
  Serial.print(currentZeroRaw);
  Serial.print(" captured=");
  Serial.print(capturedSnapshot);
  Serial.print(" written=");
  Serial.print(static_cast<uint32_t>(writtenFrames));
  Serial.print(" queued=");
  Serial.print(queuedFrames);
  Serial.print(" dropped=");
  Serial.print(droppedSnapshot);
  Serial.print(" adc_errors=");
  Serial.print(adcErrorsSnapshot);
  if (recording) {
    Serial.print(" file=");
    Serial.print(wavFilename);
  }
  Serial.println();
}

// -------------------- User input --------------------
void printHelp() {
  Serial.println("AmbatuDrone Teensy logger commands:");
  Serial.println("  status - show logger state");
  Serial.println("  zero   - calibrate ACS758 with every motor off");
  Serial.println("  start  - begin synchronized microphone/current recording");
  Serial.println("  stop   - finalize the WAV and JSON files");
  Serial.println("  help   - show this list");
  Serial.println("Pin 2 to GND also toggles start/stop.");
}

void handleCommand(const char* command) {
  if (strcmp(command, "start") == 0) {
    startRecording();
  } else if (strcmp(command, "stop") == 0) {
    stopRecording("serial");
  } else if (strcmp(command, "zero") == 0) {
    calibrateCurrentZero();
  } else if (strcmp(command, "status") == 0) {
    printStatus();
  } else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  } else if (command[0] != '\0') {
    Serial.print("ERROR Unknown command: ");
    Serial.println(command);
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      commandBuffer[commandIndex] = '\0';
      handleCommand(commandBuffer);
      commandIndex = 0;
      commandBuffer[0] = '\0';
      continue;
    }

    if (commandIndex < sizeof(commandBuffer) - 1u) {
      if (incoming >= 'A' && incoming <= 'Z') {
        commandBuffer[commandIndex++] = incoming - 'A' + 'a';
      } else {
        commandBuffer[commandIndex++] = incoming;
      }
    } else {
      commandIndex = 0;
      commandBuffer[0] = '\0';
      Serial.println("ERROR Command was too long and was discarded.");
    }
  }
}

void pollRecordButton() {
  const bool reading = digitalReadFast(RECORD_BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    buttonChangedMs = millis();
  }

  if (millis() - buttonChangedMs >= 30u &&
      reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) {
      if (recording) {
        stopRecording("button");
      } else {
        startRecording();
      }
    }
  }
}

// -------------------- Arduino entry points --------------------
void setup() {
  pinMode(RECORD_LED_PIN, OUTPUT);
  digitalWriteFast(RECORD_LED_PIN, LOW);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  const uint32_t serialWaitStarted = millis();
  while (!Serial && millis() - serialWaitStarted < 3000u) {
    // Continue after three seconds even when no computer is connected.
  }

  Serial.println();
  Serial.println("AmbatuDrone Teensy 4.1 logger starting...");

  adc.adc0->setAveraging(0);
  adc.adc0->setResolution(ADC_BITS);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  adc.adc1->setAveraging(0);
  adc.adc1->setResolution(ADC_BITS);
  adc.adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  sdReady = SD.sdfs.begin(SdioConfig(FIFO_SDIO));
  if (!sdReady) {
    Serial.println(
        "ERROR microSD initialization failed. Insert a FAT32 card and reboot.");
  } else {
    Serial.println("SD microSD ready using Teensy 4.1 built-in SDIO.");
  }

  calibrateCurrentZero();
  printHelp();
  printStatus();
}

void loop() {
  pollSerialCommands();
  pollRecordButton();

  if (recording) {
    if (!drainRingBuffer()) {
      stopRecording("sd_write_error");
    } else if (capturedFrames >=
               SAMPLE_RATE_HZ * MAX_RUN_SECONDS) {
      stopRecording("maximum_duration");
    }

    if (millis() - lastStatusMs >= 1000u) {
      lastStatusMs = millis();
      printStatus();
    }
  }
}
