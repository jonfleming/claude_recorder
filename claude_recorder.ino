#include "ESP_I2S.h"
#include "FS.h"
#include "SD.h"

const uint32_t SAMPLERATE = 16000;
const int BUFFER_SIZE = 512; // Reduced buffer size for better responsiveness
const int16_t VOLUME_THRESHOLD = 1000; // Lower threshold for better speech detection
const int SILENCE_BLOCKS = 20; // Reduced silence blocks
const byte btnPin = D7;
const byte ledPin = BUILTIN_LED;
bool stop = false;

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
        // The sample might need scaling depending on your microphone
        //buffer[i] = (int16_t)(sample >> 16); // Use upper 16 bits
        // Alternative: buffer[i] = (int16_t)(sample & 0xFFFF); // Use lower 16 bits
        buffer[i] = (int16_t)(sample & 0xFFFF); // Use lower 16 bits
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
    uint16_t avg = sum / validSamples;
    
    // Debug output every 100 blocks
    static int debugCounter = 0;
    if (debugCounter++ % 100 == 0) {
      Serial.printf("Volume: %d, Recording: %s, Samples: %d\n", 
                   avg, recording ? "YES" : "NO", validSamples);
    }
    
    if (avg > VOLUME_THRESHOLD) {
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
        Serial.printf("Speech detected! Recording to %s\n", filename);
      }
      
      // Write audio data
      wavFile.write((uint8_t*)buffer, validSamples * 2);
      samplesWritten += validSamples;
      
    } else {
      if (recording) {
        Serial.println("Silence detected. Recording: true");
        // Still write the buffer even during silence to maintain continuity
        wavFile.write((uint8_t*)buffer, validSamples * 2);
        samplesWritten += validSamples;
        
        silenceBlocks++;
        if (silenceBlocks > SILENCE_BLOCKS) {
          // Update header with correct sample count
          writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
          wavFile.close();
          Serial.printf("Recording stopped. Saved %d samples to %s\n", 
                       samplesWritten, filename);
          recording = false;
          silenceBlocks = 0;
        }
      } else {
        Serial.println("Silence detected. Recording: false");
      }
    }
    
    // Check for button press to exit
    if (!digitalRead(btnPin)) {
      if (recording) {
        writeWavHeader(wavFile, SAMPLERATE, samplesWritten);
        wavFile.close();
        Serial.println("Recording stopped by button press.");
      }
      Serial.println("Exiting record loop.");
      Serial.println("Stopping.  Button pressed.");
      break;
    }
    
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

void testMicrophone() {
  Serial.println("Testing microphone for 10 seconds...");
  unsigned long startTime = millis();
  int sampleCount = 0;
  int32_t maxSample = 0;
  int32_t minSample = 0;
  
  while (millis() - startTime < 10000) {
    if (i2s.available()) {
      int32_t sample = i2s.read();
      sampleCount++;
      
      if (sample > maxSample) maxSample = sample;
      if (sample < minSample) minSample = sample;
      
      if (sampleCount % 1000 == 0) {
        Serial.printf("Samples: %d, Range: %d to %d\n", 
                     sampleCount, minSample, maxSample);
      }
    }
    delay(1);
  }
  
  Serial.printf("Test complete. Total samples: %d\n", sampleCount);
  Serial.printf("Sample range: %d to %d\n", minSample, maxSample);
}

void setup() {
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
  Serial.println("- Send 'r' via serial to start recording");
  Serial.println("- Send 's' via serial to stop");
  
  // Run initial microphone test
  testMicrophone();
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't') {
      stop = false;
      testMicrophone();
    } else if (cmd == 'r') {
      stop = false;
      Serial.println("Start Recording (command):");
      digitalWrite(ledPin, LOW); // LED on during recording
      recordWithSpeechDetection();
      digitalWrite(ledPin, HIGH); // LED off
    } else if (cmd == 's') {
      stop = true;
    }
  }
  
  // Check for button press
  if (!digitalRead(btnPin)) {
    Serial.println("Start Recording (button):");
    delay(200); // Debounce
    digitalWrite(ledPin, LOW); // LED on during recording
    recordWithSpeechDetection();
    digitalWrite(ledPin, HIGH); // LED off
    delay(500); // Prevent immediate restart
  }
  
  delay(100);
}