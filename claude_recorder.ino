#include <WiFi.h>
const char* ssid = "FLEMING";
const char* password = "aMi4mTsg";
String lastRecordedFile = "";
#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"
#include <math.h>

// Speech detection parameters
const int FFT_SIZE = 256;
const float SPEECH_FREQ_MIN = 300.0;   // Hz - minimum speech frequency
const float SPEECH_FREQ_MAX = 3400.0;  // Hz - maximum speech frequency
const float SPEECH_ENERGY_THRESHOLD = 3.00; // Ratio of speech band energy to total energy

// Adapative threshold
float backgroundNoise = 1000.0;  // Running average of background noise
float adaptationRate = 0.01;     // How quickly to adapt to new noise levels
const float SPEECH_MULTIPLIER = 1.1;  // Speech should be X times louder than background

// Temporal analysis variables
const int PATTERN_HISTORY = 20;  // Number of recent measurements to consider
uint32_t recentLevels[PATTERN_HISTORY];
int historyIndex = 0;
bool historyFull = false;

const uint32_t SAMPLERATE = 16000;
const int BUFFER_SIZE = 512; // Reduced buffer size for better responsiveness
const int SILENCE_BLOCKS = 5; // Reduced silence blocks
const byte btnPin = D7;
const byte ledPin = BUILTIN_LED;
bool stop = false;
const int16_t GAIN = 3; // sample multiplier
const int LOGLEVEL = 2; // 0 = instructions, 1 = plotter data, 2 = debug messages, 3 = data and debug, 4 = trace
const bool INSTRUCT = true; // Print instructions to serial

int debugCounter = 0;
int16_t VOLUME_THRESHOLD = 1350; // initial Lower threshold for speech detection

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

// Returns ratio of energy in speech band to total energy
float speechBandEnergyRatio(int16_t* buffer, int bufferSize, uint32_t sampleRate) {
  float totalEnergy = 0;
  float speechEnergy = 0;
  int N = bufferSize;
  for (int k = 0; k < N / 2; k++) {
    float freq = (float)k * sampleRate / N;
    float re = 0, im = 0;
    for (int n = 0; n < N; n++) {
      float angle = 2 * M_PI * k * n / N;
      re += buffer[n] * cos(angle);
      im -= buffer[n] * sin(angle);
    }
    float mag = sqrt(re * re + im * im);
    totalEnergy += mag;
    if (freq >= 300.0 && freq <= 3400.0) {
      speechEnergy += mag;
    }
  }
  if (totalEnergy == 0) return 0;
  return speechEnergy / totalEnergy * 100;
}

// Usage in your detection:
bool detectSpeechSpectral(int16_t* buffer, int bufferSize, uint32_t sampleRate) {
  float ratio = speechBandEnergyRatio(buffer, bufferSize, sampleRate);

  bool isSpeech = ratio > SPEECH_ENERGY_THRESHOLD;
  // Debug output  
  char logMessage[128];
  if (debugCounter % 50 == 0) {
    sprintf(logMessage, "\nSpectral ratio: %.2f, Speech: %s\n", ratio, isSpeech ? "YES" : "NO");
    log(2, logMessage);
  }

  return isSpeech;
}

// Adaptive threshold calculation
void updateBackgroundNoise(uint32_t currentLevel, bool isSpeech) {
  if (!isSpeech) {
    // Only update background noise during non-speech periods
    backgroundNoise = backgroundNoise * (1.0 - adaptationRate) + currentLevel * adaptationRate;
  }
}

bool detectSpeechAdaptive(int16_t* buffer, int bufferSize) {
  // Calculate current audio level
  uint32_t sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += abs(buffer[i]);
  }
  uint32_t currentLevel = sum / bufferSize;
  
  // Dynamic threshold based on background noise
  float dynamicThreshold = backgroundNoise * SPEECH_MULTIPLIER;
  
  bool isSpeech = currentLevel > dynamicThreshold;
  
  // Update background noise estimate
  updateBackgroundNoise(currentLevel, isSpeech);
  
  // Debug output
  if (debugCounter % 50 == 0) {
    char logMessage[128]; 
    sprintf(logMessage, "\nLevel: %d, Background: %.1f, Threshold: %.1f, Speech: %s\n", 
                  currentLevel, backgroundNoise, dynamicThreshold, isSpeech ? "YES" : "NO");
    log(2, logMessage);
  }
  
  return isSpeech;
}

bool detectSpeechTemporal(int16_t* buffer, int bufferSize) {
  // Calculate current level
  uint32_t sum = 0;
  for (int i = 0; i < bufferSize; i++) {
    sum += abs(buffer[i]);
  }
  uint32_t currentLevel = sum / bufferSize;
  
  // Store in circular buffer
  recentLevels[historyIndex] = currentLevel;
  historyIndex = (historyIndex + 1) % PATTERN_HISTORY;
  if (historyIndex == 0) historyFull = true;
  
  if (!historyFull) return currentLevel > VOLUME_THRESHOLD; // Fallback to simple method
  
  // Calculate statistics over recent history
  uint32_t minLevel = UINT32_MAX, maxLevel = 0, avgLevel = 0;
  for (int i = 0; i < PATTERN_HISTORY; i++) {
    avgLevel += recentLevels[i];
    if (recentLevels[i] < minLevel) minLevel = recentLevels[i];
    if (recentLevels[i] > maxLevel) maxLevel = recentLevels[i];
  }
  avgLevel /= PATTERN_HISTORY;
  
  // Speech characteristics:
  // 1. Dynamic range - speech varies more than steady noise
  float dynamicRange = (float)(maxLevel - minLevel) / (avgLevel + 1);
  
  // 2. Current level should be significantly above minimum
  float currentRatio = (float)currentLevel / (minLevel + 1);
  
  // 3. Moderate variability (not too steady, not too chaotic)
  uint32_t variance = 0;
  for (int i = 0; i < PATTERN_HISTORY; i++) {
    int32_t diff = recentLevels[i] - avgLevel;
    variance += diff * diff;
  }
  variance /= PATTERN_HISTORY;
  float stdDev = sqrt(variance);
  float coefficientOfVariation = stdDev / (avgLevel + 1);
  
  // Speech detection criteria
  bool hasDynamicRange = dynamicRange > 0.02;  // At least 30% variation
  bool aboveBackground = currentRatio > 1.0;   // 50% above recent minimum
  bool moderateVariability = coefficientOfVariation > 0.005 && coefficientOfVariation < 2.0;
  bool aboveBasicThreshold = currentLevel > VOLUME_THRESHOLD;
  
  bool isSpeech = hasDynamicRange && aboveBackground && moderateVariability && aboveBasicThreshold;
  
  char logMessage[128]; 
  if (debugCounter % 50 == 0) {
    sprintf(logMessage, "\nAvg: %d, Range: %.2f, Ratio: %.2f, CoV: %.2f, Speech: %s\n", 
                  avgLevel, dynamicRange, currentRatio, coefficientOfVariation, 
                  isSpeech ? "YES" : "NO");
    log(2, logMessage);
  }
  
  return isSpeech;
}

void recordWithSpeechDetectionEnhanced() {
  static int fileCount = 0;
  char filename[32];
  int16_t buffer[BUFFER_SIZE];
  bool recording = false;
  File wavFile;
  uint32_t samplesWritten = 0;
  int silenceBlocks = 0;
  int speechBlocks = 0;  // Add confirmation blocks for speech
  const int MIN_SPEECH_BLOCKS = 3;  // Require 3 consecutive detections
  char logMessage[128]; 

  log(0, "Listening for speech (enhanced detection)...\n");

  auto startRecording = [&]() {
    sprintf(filename, "/audio%d.wav", fileCount++);
    wavFile = SD.open(filename, FILE_WRITE);
    if (!wavFile) {
      log(0, "Failed to open file for writing!\n");
      return false;
    }
    byte header[44] = {0};
    wavFile.write(header, 44);
    samplesWritten = 0;
    recording = true;
    digitalWrite(ledPin, LOW);
    sprintf(logMessage, "\nSpeech detected! Recording to %s\n", filename);
    log(0, logMessage);
    lastRecordedFile = String(filename);
    return true;
  };

  auto stopRecording = [&](const char* reason) {
    writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
    wavFile.close();
    sprintf(logMessage, "\n%s. Saved %d samples to %s\n", reason, samplesWritten, filename);
    log(0, logMessage);
    
    uploadWavFile(String(filename));
    recording = false;
    silenceBlocks = 0;
    speechBlocks = 0;
  };

  while (true) {
    int validSamples = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      if (i2s.available()) {
        int32_t sample = i2s.read();
        int16_t sample16 = (int16_t)(sample & 0xFFFF) * GAIN;
        buffer[i] = sample16;
        validSamples++;

        sprintf(logMessage, "%d", sample16);
        log(1, logMessage);
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

    // Use multiple detection methods
    debugCounter++;
    bool method1 = detectSpeechSpectral(buffer, validSamples, SAMPLERATE); // Spectral:
    bool method2 = detectSpeechAdaptive(buffer, validSamples);             // Level:
    bool method3 = detectSpeechTemporal(buffer, validSamples);             // Avg:
    
    // Voting system - require majority agreement
    int votes = (method1 ? 1 : 0) + (method2 ? 1 : 0) + (method3 ? 1 : 0);
    bool speechDetected = votes >= 2;  // Require at least 2 out of 3 methods to agree

    digitalWrite(ledPin, speechDetected ? LOW : HIGH);

    sprintf(logMessage, "Speech detected: %s\n", speechDetected ? "YES" : "NO");
    log(3, logMessage);
    
    if (speechDetected) {
      speechBlocks++;
      silenceBlocks = 0;
      
      // Start recording only after consistent speech detection
      if (!recording && speechBlocks >= MIN_SPEECH_BLOCKS) {
        if (!startRecording()) return;
      }
      
      if (recording) {
        wavFile.write((uint8_t*)buffer, validSamples * 2);
        samplesWritten += validSamples;
        sprintf(logMessage, "\n%d speech blocks. %d samples writen. Filename: %s\n", speechBlocks, samplesWritten, lastRecordedFile);
        log(3, logMessage);
        log(2, "o");
      } else {
        log(2, "+");
      }
    } else {
      speechBlocks = 0;  // Reset speech counter
      log(2, ".");

      if (recording) {
        wavFile.write((uint8_t*)buffer, validSamples * 2);
        samplesWritten += validSamples;
        sprintf(logMessage, "\n$d silence blocks.  %d silent samples writen. Filename: %s\n", silenceBlocks, samplesWritten, lastRecordedFile);
        log(2, logMessage);
        silenceBlocks++;
        if (silenceBlocks > SILENCE_BLOCKS) {
          stopRecording("Silence detected. Recording stopped");
        }
      }
    }

    delay(1);
    
    // Button or serial stop (same as before)
    if (!digitalRead(btnPin)) {
      if (recording) {
        stopRecording("Recording stopped by button press");
      }
      log(0, "\nExiting record loop. Button pressed.");
      break;
    }

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      if (cmd == "s") {
        if (recording) {
          stopRecording("Recording stopped by command");
        }
        log(0, "\nExiting record loop. Command received.");
        stop = true;
        break;
      }
    }

    if (stop) {
      log(0, "Stopping. stop == true.");
      break;
    }
  }
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

void setThreshold() {
  char logMessage[128]; 
  log(0, "Setting threshold for speech detection. Stay silent for 3 seconds...");
  unsigned long startTime = millis();
  int32_t sampleSum = 0;
  int32_t sampleCount = 0;
  
  while (millis() - startTime < 3000) {
    if (i2s.available()) {
      int32_t sample = i2s.read();
      int16_t sample16 = (int16_t)(sample & 0xFFFF) * 2; // Convert to 16-bit

      sampleCount++;
      sampleSum += sample16;
    }

    delay(1);
  }

  int32_t avgSample = sampleSum / sampleCount;
  VOLUME_THRESHOLD = avgSample;
  backgroundNoise = avgSample;
  
  sprintf(logMessage,"Threshold set to: %d\n", VOLUME_THRESHOLD);
  log(0, logMessage);
}

void testMicrophone() {
  char logMessage[128];
  log(0, "Testing microphone for 5 seconds...");
  unsigned long startTime = millis();
  int sampleCount = 0;
  int32_t sampleSum = 0;
  int32_t maxSample = 0;
  int32_t minSample = VOLUME_THRESHOLD;
  int32_t avg = 0;

  while (millis() - startTime < 5000) {
    if (i2s.available()) {
      int32_t sample = i2s.read();
      int16_t sample16 = (int16_t)(sample & 0xFFFF) * 2; // Convert to 16-bit

      if (sampleCount % 200 == 0) {
        sprintf(logMessage, "Sample: %d, Max: %d, Min: %d, Avg: %d", sampleCount, maxSample, minSample, avg);
        log(2, logMessage);
      }
      sampleCount++;
      sampleSum += sample16;

      if (sample16 > maxSample) maxSample = sample16;
      if (sample16 < minSample) minSample = sample16;

      if (sampleCount % 10 == 0) {
        avg = sampleSum / sampleCount;        
        if (avg > VOLUME_THRESHOLD) {
          log(2, "o");
        } else {
          log(2, ".");
        }
      }

      if (sampleCount % 1000 == 0) {
        sprintf(logMessage,"\nSamples: %d, Range: %d to %d  Average: %d  Threshold: %d\n", 
                     sampleCount, minSample, maxSample, avg, VOLUME_THRESHOLD);
        log(2, logMessage);
        sampleCount = 0;
        sampleSum = 0;
        maxSample = 0;
        minSample = VOLUME_THRESHOLD;
      }
    } else {
      log(2, "-");
    }
    delay(1);
  } 
}

void setup() {
  char logMessage[128];
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
  
  Serial.begin(115200);
  while (!Serial) delay(100);
  
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

  log(0, "Setup complete!");
  log(0, "Commands:");
  log(0, "- Press button to start/stop recording");
  log(0, "- Send 't' via serial to test microphone");
  log(0, "- Send 'q' via serial to set threshold (quash)");
  log(0, "- Send 'r' via serial to start recording");
  log(0, "- Send 's' via serial to stop");
  log(0, "- Send 'u' via serial to toggle upload mode");
  log(0, "- Send number (+/-) to adjust threshold");

  testMicrophone();
  setThreshold();
  recordWithSpeechDetectionEnhanced();
}

void loop() {
  char logMessage[128]; 
  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "t") {
      testMicrophone();
    } else if (cmd == "r") {
      stop = false;
      sprintf(logMessage,"Start Recording (command):");
      log(0, logMessage);
      recordWithSpeechDetectionEnhanced();
      digitalWrite(ledPin, HIGH); // LED off
    } else if (cmd == "s") {
      stop = true;
    } else if (cmd == "q") {
      setThreshold();
      testMicrophone(); 
    } else if (isdigit(cmd.charAt(0)) || (cmd.charAt(0) == '+' || cmd.charAt(0) == '-')) {
      // Adjust threshold based on input
      sprintf(logMessage,"Input received: %s", cmd);
      log(0, logMessage);
      int adjustment = cmd.toInt();
      VOLUME_THRESHOLD += adjustment;
      sprintf(logMessage,"Threshold adjusted to: %d\n", VOLUME_THRESHOLD);
      log(0, logMessage);
      testMicrophone();
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
    recordWithSpeechDetectionEnhanced();
    delay(500); // Prevent immediate restart
  }
  
  delay(100);
}