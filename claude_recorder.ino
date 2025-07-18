#include <WiFi.h>
const char* ssid = "FLEMING";
const char* password = "aMi4mTsg";
String lastRecordedFile = "";
bool uploadMode = true;
#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"

const uint32_t SAMPLERATE = 16000;
const int BUFFER_SIZE = 512; // Reduced buffer size for better responsiveness
const int SILENCE_BLOCKS = 20; // Reduced silence blocks
const byte btnPin = D7;
const byte ledPin = BUILTIN_LED;
bool stop = false;
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

void recordWithSpeechDetection() {
  static int fileCount = 0;
  char filename[32];
  int16_t buffer[BUFFER_SIZE];
  bool recording = false;
  File wavFile;
  uint32_t samplesWritten = 0;
  int silenceBlocks = 0;
  
  Serial.println("Listening for speech...");
  while (1) {
    int validSamples = 0;
    
    // Fill buffer with available samples
    for (int i = 0; i < BUFFER_SIZE; i++) {
      if (i2s.available()) {
        int32_t sample = i2s.read();
        
        // Convert 32-bit sample to 16-bit and apply gain
        buffer[i] = (int16_t)((sample & 0xFFFF) * 2); // Use lower 16 bits
        // Or apply gain: buffer[i] = (int16_t)((sample >> 16) * 2); // 2x gain
        
        validSamples++;
      } else {
        // No more data available
        break;
      }
    }
    
    if (validSamples == 0) {
      delay(1);
      continue;
    }
    
    // Calculate RMS for volume detection
    uint32_t sum = 0;
    for (int i = 0; i < validSamples; i++) {
      sum += abs(buffer[i]);
    }
    uint32_t avg = sum / validSamples / 2; // Average volume, divided by 2 for sensitivity
    
    // Debug output every 100 blocks
    static int debugCounter = 0;
    if (debugCounter++ % 100 == 0) {
      Serial.printf("\nAverage: %d, Recording: %s, Threshold: %d\n", 
                   avg, recording ? "YES" : "NO", VOLUME_THRESHOLD);
    }
    
    if (avg > VOLUME_THRESHOLD) {
      digitalWrite(ledPin, LOW);
      if (debugCounter % 100 == 1) {
        Serial.printf("Avg: %d > Threshold: %d\n", avg, VOLUME_THRESHOLD);
      }

      silenceBlocks = 0;
      
      if (!recording) {
        sprintf(filename, "/audio%d.wav", fileCount++);
        wavFile = SD.open(filename, FILE_WRITE);
        if (!wavFile) {
          Serial.println("Failed to open file for writing!");
          return;
        }
        // Write placeholder header (will be updated later)
        byte header[44] = {0};
        wavFile.write(header, 44);
        samplesWritten = 0;
        recording = true;
        digitalWrite(ledPin, LOW); // LED on during recording
        Serial.printf("\nSpeech detected! Recording to %s\n", filename);
        lastRecordedFile = String(filename);
      }
      
      // Write audio data
      wavFile.write((uint8_t*)buffer, validSamples * 2);
      samplesWritten += validSamples;
      Serial.printf("o");      
    } else {
      Serial.printf(".");
      if (recording) {
        // Still write the buffer even during silence to maintain continuity
        wavFile.write((uint8_t*)buffer, validSamples * 2);
        samplesWritten += validSamples;
        
        silenceBlocks++;
        if (silenceBlocks > SILENCE_BLOCKS) {
          // Update header with correct sample count
          writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
          wavFile.close();
          Serial.printf("\nSilence detected. Recording stopped. Saved %d samples to %s\n", samplesWritten, filename);
          if (uploadMode) {
            uploadWavFile(String(filename));
          }
          recording = false;
          silenceBlocks = 0;
        }
      } else {
        Serial.printf(".");
      }
    }
    
    // Check for button press to exit
    if (!digitalRead(btnPin)) {
      if (recording) {
        writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
        wavFile.close();
        Serial.println("\nRecording stopped by button press.");
      }
      Serial.println("Exiting record loop.");
      Serial.println("Stopping.  Button pressed.");
      break;
    }
    
    digitalWrite(ledPin, HIGH); // LED off
    delay(1); // Small delay to prevent overwhelming the system

    // check for 's' stop command
    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 's') {
        Serial.println("Stopping.  S command received.");
        stop = true;        
        break;
      }
    }

    if (stop) {
      Serial.println("Stopping.  stop == true.");      
      break;
    }
  }
}

// Function to upload a WAV file to the server
void uploadWavFile(const String& filePath) {
  Serial.println("Uploading file: " + filePath);

  File file = SD.open(filePath.c_str(), FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for upload!");
    return;
  }
  WiFiClient client;
  const char* host = "omen";
  const int httpPort = 3000;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection to server failed");
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
  Serial.printf("Uploaded %d bytes to server.\n", sent);
  // Wait for server response
  unsigned long timeout = millis();
  while (client.connected() && !client.available()) {
    if (millis() - timeout > 5000) {
      Serial.println("Server timeout");
      client.stop();
      return;
    }
    delay(10);
  }
  String response;
  while (client.available()) {
    response += client.readStringUntil('\n');
  }
  Serial.println("Server response: " + response.substring(0, 20) + "...");
  client.stop();
}

void setThreshold() {
  Serial.println("Setting threshold for speech detection. Stay silent for 3 seconds...");
  unsigned long startTime = millis();
  int32_t sampleSum = 0;
  int32_t sampleCount = 0;
  
  while (millis() - startTime < 3000) {
    if (i2s.available()) {
      int32_t sample = i2s.read();
      sampleCount++;
      sampleSum += sample;
    }

    delay(1);
  }

  int32_t avgSample = sampleSum / sampleCount;
  VOLUME_THRESHOLD = avgSample;
  Serial.printf("Threshold set to: %d\n", VOLUME_THRESHOLD);
}

void testMicrophone() {
  Serial.println("Testing microphone for 5 seconds...");
  unsigned long startTime = millis();
  int sampleCount = 0;
  int32_t sampleSum = 0;
  int32_t maxSample = 0;
  int32_t minSample = 0;
  int32_t avg = 0;
  
  while (millis() - startTime < 5000) {
    if (i2s.available()) {
      int32_t sample = i2s.read();
      sampleCount++;
      sampleSum += sample;
      
      if (sample > maxSample) maxSample = sample;
      if (sample < minSample) minSample = sample;
      
      if (sampleCount % 10 == 0) {
        avg = (sampleSum / sampleCount) * 2;
        if (avg > VOLUME_THRESHOLD) {
          Serial.printf("o");
        } else {
          Serial.printf(".");
        }
      }

      if (sampleCount % 1000 == 0) {
        Serial.printf("\nSamples: %d, Range: %d to %d  Average: %d  Threshold: %d\n", 
                     sampleCount, minSample, maxSample, avg, VOLUME_THRESHOLD);
        sampleSum = 0;
        maxSample = 0;
        minSample = 0;                     
      }
    }
    delay(1);
  }
  
  Serial.printf("\nTest complete. Total samples: %d\n", sampleCount);
  Serial.printf("Sample range: %d to %d\n", minSample, maxSample);
}

void setup() {
  // WiFi setup
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  Serial.begin(115200);
  while (!Serial) delay(100);
  
  pinMode(btnPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // LED off initially
  
  Serial.println("Initializing I2S...");
  
  // Set I2S pins for PDM microphone
  i2s.setPinsPdmRx(42, 41); // CLK=42, DATA=41 - adjust for your board
  
  // Initialize I2S
  if (!i2s.begin(I2S_MODE_PDM_RX, SAMPLERATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
    Serial.println("Failed to initialize I2S!");
    while (1) {
      digitalWrite(ledPin, LOW);
      delay(200);
      digitalWrite(ledPin, HIGH);
      delay(200);
    }
  }
  
  Serial.println("Initializing SD card...");
  if (!SD.begin(21)) { // Adjust CS pin if needed
    Serial.println("Failed to mount SD Card!");
    while (1) {
      digitalWrite(ledPin, LOW);
      delay(500);
      digitalWrite(ledPin, HIGH);
      delay(500);
    }
  }
  
  Serial.println("Setup complete!");
  Serial.println("Commands:");
  Serial.println("- Press button to start/stop recording");
  Serial.println("- Send 't' via serial to test microphone");
  Serial.println("- Send 'q' via serial to set threshold (quash)");  
  Serial.println("- Send 'r' via serial to start recording");
  Serial.println("- Send 's' via serial to stop");
  Serial.println("- Send 'u' via serial to toggle upload mode");
  Serial.println("- Send number (+/-) to adjust threshold");
  
  testMicrophone();
  setThreshold();
  recordWithSpeechDetection();
  if (uploadMode && lastRecordedFile.length() > 0) {
    Serial.println("Uploading last recorded file...");
    uploadWavFile(lastRecordedFile);
  }
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "t") {
      testMicrophone();
    } else if (cmd == "r") {
      stop = false;
      Serial.println("Start Recording (command):");
      recordWithSpeechDetection();
      digitalWrite(ledPin, HIGH); // LED off
      if (uploadMode && lastRecordedFile.length() > 0) {
        Serial.println("Uploading last recorded file...");
        uploadWavFile(lastRecordedFile);
      }
    } else if (cmd == "s") {
      stop = true;
    } else if (cmd == "q") {
      setThreshold();
      testMicrophone(); 
    } else if (cmd == "u") {
      uploadMode = !uploadMode;
      Serial.printf("Upload mode %s. Will upload after next recording.\n", uploadMode ? "enabled" : "disabled");
    } else if (isdigit(cmd.charAt(0)) || (cmd.charAt(0) == '+' || cmd.charAt(0) == '-')) {
      // Adjust threshold based on input
      Serial.println("Input received: " + cmd);
      int adjustment = cmd.toInt();
      VOLUME_THRESHOLD += adjustment;
      Serial.printf("Threshold adjusted to: %d\n", VOLUME_THRESHOLD);
      testMicrophone();
    } else {
      Serial.printf("Unknown command: %s\n", cmd);
    }
  }
  
  // Check for button press
  if (!digitalRead(btnPin)) {
    Serial.println("Start Recording (button):");
    delay(200); // Debounce
    recordWithSpeechDetection();
    delay(500); // Prevent immediate restart
  }
  
  delay(100);
}