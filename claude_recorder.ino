#include <WiFi.h>
const char* ssid = "FLEMING";
const char* password = "aMi4mTsg";
String lastRecordedFile = "";
#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"
#include <math.h>

const uint32_t SAMPLERATE = 16000;
const int BUFFER_SIZE = 256; // Reduced buffer size for better responsiveness
const int SILENCE_BLOCKS = 5; // Reduced silence blocks
const byte btnPin = D7;
const byte ledPin = BUILTIN_LED;
bool recording = false;
const int16_t GAIN = 3; // sample multiplier
const int LOGLEVEL = 2; // 0 = instructions, 1 = plotter data, 2 = debug messages, 3 = data and debug, 4 = trace
const bool INSTRUCT = true; // Print instructions to serial

int debugCounter = 0;

I2SClass i2s;

// WAV header helper
void writeWavHeader(File &file, uint32_t sampleRate, uint32_t numSamples) {
  uint32_t dataSize = numSamples * 2; // 16-bit mono
  uint32_t chunkSize = 36 + dataSize;
  
  file.seek(0);
  file.write((const uint8_t*)"RIFF", 4);
  file.write((uint8_t*)&chunkSize, 4);
  file.write((const uint8_t*)"WAVE", 4);
  file.write((const uint8_t*)"fmt ", 4);
  
  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1; // Mono
  uint32_t byteRate = sampleRate * 2;
  uint16_t blockAlign = 2;
  uint16_t bitsPerSample = 16;
  
  file.write((uint8_t*)&subchunk1Size, 4);
  file.write((uint8_t*)&audioFormat, 2);
  file.write((uint8_t*)&numChannels, 2);
  file.write((uint8_t*)&sampleRate, 4);
  file.write((uint8_t*)&byteRate, 4);
  file.write((uint8_t*)&blockAlign, 2);
  file.write((uint8_t*)&bitsPerSample, 2);
  file.write((const uint8_t*)"data", 4);
  file.write((uint8_t*)&dataSize, 4);
}

void log(int logLevel, char *message) {
  if (logLevel == 0 && INSTRUCT) {
    Serial.print(message);
  } else if ((logLevel == 1 && LOGLEVEL == 1) || LOGLEVEL == 3) {
    Serial.print("Data:\t"  + String(message) + "\n");
  } else if ((logLevel == 2 && LOGLEVEL == 2) || LOGLEVEL == 3) {
    Serial.print(message);
  } else if (LOGLEVEL >= 4) {
    Serial.print(message);
  }
}

void record() {
  static int fileCount = 0;
  char filename[32];
  int16_t buffer[BUFFER_SIZE];
  File wavFile;
  uint32_t samplesWritten = 0;
  int silenceBlocks = 0;
  int speechBlocks = 0;  // Add confirmation blocks for speech
  const int MIN_SPEECH_BLOCKS = 3;  // Require 3 consecutive detections
  char logMessage[128]; 

  auto startRecording = [&](const char* reason) {
    if (recording) {
      log(0, "Already recording. Stopping previous recording.\n");
      recording = false;
      return;
    }

    sprintf(filename, "/audio%d.wav", fileCount++);
    sprintf(logMessage, "\n%s. Creating %d\n", reason, filename);
    log(0, logMessage);
    wavFile = SD.open(filename, FILE_WRITE);
    if (!wavFile) {
      log(0, "Failed to open file for writing!\n");
      return;
    }
    byte header[44] = {0};
    wavFile.write(header, 44);
    samplesWritten = 0;
    recording = true;
    digitalWrite(ledPin, LOW);
    lastRecordedFile = String(filename);
  };

  auto stopRecording = [&](const char* reason) {
    writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
    wavFile.close();
    sprintf(logMessage, "\n%s. Saved %d samples to %s\n", reason, samplesWritten, filename);
    log(0, logMessage);
    
    uploadWavFile(String(filename));
    silenceBlocks = 0;
    speechBlocks = 0;
    recording = false;
    digitalWrite(ledPin, HIGH);
  };

  startRecording("Starting recording");
  log(0, "- Press button to stop recording\n");
  log(0, "- Send 's' via serial to stop\n");

  while (recording) {
    int validSamples = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      if (i2s.available()) {
        int32_t sample = i2s.read();
        int16_t sample16 = (int16_t)(sample & 0xFFFF) * GAIN;
        buffer[i] = sample16;
        validSamples++;

      } else {
        log(3, "No data available\n");
        break;
      }
    }
    if (validSamples == 0) {
      delay(1);
      log(3, "No valid samples\n");
      continue;
    }

    debugCounter++;
    digitalWrite(ledPin, LOW);

    wavFile.write((uint8_t*)buffer, validSamples * 2);
    samplesWritten += validSamples;
    if (debugCounter % 100 == 0) {
      log(0, "o");
    }
    
    delay(1);
    
    // Button or serial stop (same as before)
    if (!digitalRead(btnPin)) {
      stopRecording("Recording stopped by button press");
      log(1, "Recording stopped by button press");
    }

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      if (cmd == "r") {
        startRecording("Recording stopped by command");
      }
      if (cmd == "s") {
        stopRecording("Recording stopped by command");
      }
    }
  }

  log(0, "Loop Exited. Recording complete.\n");
}

// Function to upload a WAV file to the server
void uploadWavFile(const String& filePath) {
  char logMessage[128]; 
  sprintf(logMessage, "Uploading file: %s", filePath.c_str());
  log(0, logMessage);

  File file = SD.open(filePath.c_str(), FILE_READ);
  if (!file) {
    log(0, "Failed to open file for upload!");
    return;
  }
  WiFiClient client;
  const char* host = "omen";
  const int httpPort = 3000;
  if (!client.connect(host, httpPort)) {
    sprintf(logMessage,"Connection to server failed");
    log(0, logMessage);
    file.close();
    return;
  }
  String url = "/upload";
  String contentType = "audio/wav";
  size_t contentLength = file.size();
  // Send HTTP POST header
  client.print(String("POST ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Content-Type: " + contentType + "\r\n" +
               "Content-Length: " + contentLength + "\r\n" +
               "Connection: close\r\n\r\n");
  // Send file data
  uint8_t buf[512];
  size_t sent = 0;
  while (file.available()) {
    size_t n = file.read(buf, sizeof(buf));
    client.write(buf, n);
    sent += n;
  }
  file.close();
  sprintf(logMessage,"Uploaded %d bytes to server.\n", sent);
  log(0, logMessage);
  // Wait for server response
  unsigned long timeout = millis();
  while (client.connected() && !client.available()) {
    if (millis() - timeout > 5000) {
      log(0, "Server timeout");
      client.stop();
      return;
    }
    delay(10);
  }
  String response;
  while (client.available()) {
    response += client.readStringUntil('\n');
  }
  sprintf(logMessage,"Server response: %s...", response.substring(0, 20).c_str());
  log(0, logMessage);
  client.stop();
}

void setup() {
  char logMessage[128];
  Serial.begin(115200);
  while (!Serial) delay(100);

  // WiFi setup
  sprintf(logMessage,"Connecting to WiFi...");
  log(0, logMessage);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    log(0, ".");
  }
  sprintf(logMessage, "Connected to WiFi: %s  IP address: %s", ssid, WiFi.localIP().toString().c_str());
  log(0, logMessage);
    
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // LED off initially
  
  sprintf(logMessage,"Initializing I2S...");
  log(0, logMessage);

  // Set I2S pins for PDM microphone
  i2s.setPinsPdmRx(42, 41); // CLK=42, DATA=41 - adjust for your board
  
  // Initialize I2S
  if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLERATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    sprintf(logMessage,"Failed to initialize I2S!");
    log(0, logMessage);
    while (1) {
      digitalWrite(ledPin, LOW);
      delay(200);
      digitalWrite(ledPin, HIGH);
      delay(200);
    }
  }
  
  log(0, "Initializing SD card...");
  if (!SD.begin(21)) { // Adjust CS pin if needed
    log(0, "Failed to mount SD Card!");
    while (1) {
      digitalWrite(ledPin, LOW);
      delay(500);
      digitalWrite(ledPin, HIGH);
      delay(500);
    }
  }

  log(0, "Setup complete!\n");
  log(0, "Commands:\n");
  log(0, "- Press button to start recording\n");
  log(0, "- Send 'r' via serial to start recording\n");
}

void loop() {
  char logMessage[128]; 
  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "r") {
      sprintf(logMessage,"Start Recording (command):");
      log(0, logMessage);
      record();
      digitalWrite(ledPin, HIGH); // LED off
    } else if (cmd == "s") {
      recording = false;
    } else {
      sprintf(logMessage,"Unknown command: %s\n", cmd);
      log(0, logMessage);
    }
  }
  
  // Check for button press
  if (!digitalRead(btnPin)) {
    sprintf(logMessage,"Start Recording (button):");
    log(0, logMessage);
    delay(200); // Debounce
    record();
    delay(500); // Prevent immediate restart
  }
  
  delay(100);
}
