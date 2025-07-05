#include <arduinoFFT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <time.h>
#include <base64.h>
#include <base64.h>
#include <ArduinoJson.h>

// Audio buffer configuration
const uint16_t SAMPLE_SIZE = 256;         // FFT size (keep this for frequency analysis)
const double SAMPLING_FREQUENCY = 5000;  // Reduced to 5kHz for lower quality but manageable size
const float VOLUME_THRESHOLD = 0.45;

// Circular buffer for 2 seconds of audio
const int BUFFER_DURATION_SECONDS = 1;
const int AUDIO_BUFFER_SIZE = SAMPLING_FREQUENCY * BUFFER_DURATION_SECONDS;  // 10,000 samples
uint16_t audioBuffer[AUDIO_BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFull = false;

// FFT arrays (separate from audio buffer)
double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLE_SIZE, SAMPLING_FREQUENCY);

const int micPin = 36;
unsigned long lastBarkTime = 0;
const unsigned long COOLDOWN_MS = 3000;

static StaticJsonDocument<1024> doc;

void setup() {
  Serial.begin(115200);

  WiFiManager wm;
  if (!wm.autoConnect("BarkSensor-Setup")) {
    Serial.println("❌ WiFi connection failed.");
    ESP.restart();
  }
  Serial.println("✅ WiFi connected!");

  // Test for Internet access
  if (testInternetConnection()) {
    Serial.println("✅ Internet access confirmed.");
  } else {
    Serial.println("⚠️ WiFi connected, but no internet access.");
  }

  // Sync time with NTP
  configTime(0, 0, "pool.ntp.org");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.println("⏳ Waiting for NTP time...");
    delay(500);
  }
  Serial.println("✅ Time synced.");
}

void loop() {
  // === 1. Continuously capture and store audio samples ===
  uint16_t currentSample = analogRead(micPin);
  
  // Store in circular buffer
  audioBuffer[bufferIndex] = currentSample;
  bufferIndex = (bufferIndex + 1) % AUDIO_BUFFER_SIZE;
  if (bufferIndex == 0) bufferFull = true;  // Buffer has wrapped around
  
  // === 2. Every 256 samples, do FFT analysis ===
  static int analysisCounter = 0;
  analysisCounter++;
  
  if (analysisCounter >= 256) {  // Analyze every 256 samples (~51ms at 5kHz)
    analysisCounter = 0;
    
    // Copy recent samples for FFT analysis
    int startIdx = (bufferIndex - SAMPLE_SIZE + AUDIO_BUFFER_SIZE) % AUDIO_BUFFER_SIZE;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      int idx = (startIdx + i) % AUDIO_BUFFER_SIZE;
      vReal[i] = audioBuffer[idx];
      vImag[i] = 0;
    }
    
    // === 3. Compute volume (RMS) ===
    double rms = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) {
      rms += sq(vReal[i]);
    }
    rms = sqrt(rms / SAMPLE_SIZE);
    double volume = rms / 4095.0;  // Normalize (0–1)
    
    // === 4. Apply FFT for frequency analysis ===
    double avg = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++) avg += vReal[i];
    avg /= SAMPLE_SIZE;
    for (int i = 0; i < SAMPLE_SIZE; i++) vReal[i] -= avg;
    
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    
    // === 5. Find dominant frequency ===
    double maxMag = 0;
    int maxIndex = 0;
    for (int i = 1; i < SAMPLE_SIZE / 2; i++) {
      if (vReal[i] > maxMag) {
        maxMag = vReal[i];
        maxIndex = i;
      }
    }
    double frequency = (maxIndex * SAMPLING_FREQUENCY) / SAMPLE_SIZE;
    
    // === 6. Trigger bark event if loud enough and cooldown passed ===
    if (volume > VOLUME_THRESHOLD) {
      unsigned long now = millis();
      if (now - lastBarkTime >= COOLDOWN_MS) {
        lastBarkTime = now;
        Serial.printf("⚠️  Possible bark: Vol=%.2f  Freq=%.1f Hz\n", volume, frequency);
        
        // Only send if we have enough data
        if (bufferFull || bufferIndex > AUDIO_BUFFER_SIZE / 2) {
          sendBarkWithAudio(volume, frequency);
        }
      } else {
        Serial.println("🕒 Cooldown active — skipping bark");
      }
    }
  }
  
  delayMicroseconds(200);  // 5kHz sampling rate
}

void sendBarkWithAudio(float volume, float frequency) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin("https://bark-server-881599834430.us-central1.run.app/bark");
  http.addHeader("Content-Type", "application/json");

  time_t timestamp = time(nullptr);

  // === Pack 2 seconds of audio data ===
  int samplesToSend = bufferFull ? AUDIO_BUFFER_SIZE : bufferIndex;
  uint8_t* audioBytes = (uint8_t*)malloc(samplesToSend * 2);  // 2 bytes per sample
  
  if (audioBytes == nullptr) {
    Serial.println("❌ Failed to allocate memory for audio data");
    return;
  }
  
  // Copy circular buffer data in correct order
  int srcIdx = bufferFull ? bufferIndex : 0;  // Start from oldest sample if buffer is full
  for (int i = 0; i < samplesToSend; i++) {
    uint16_t sample = audioBuffer[(srcIdx + i) % AUDIO_BUFFER_SIZE];
    audioBytes[i * 2] = sample & 0xFF;             // low byte
    audioBytes[i * 2 + 1] = (sample >> 8) & 0xFF;  // high byte
  }

  // === Encode as base64 ===
  String encoded = base64::encode(audioBytes, samplesToSend * 2);
  free(audioBytes);  // Free allocated memory
  
  doc.clear();
  doc["device_id"] = "esp32-yard";
  doc["timestamp"] = timestamp;
  doc["volume"] = volume;
  doc["frequency"] = frequency;
  doc["event"] = "possible_bark";
  doc["audio_base64"] = encoded;
  doc["sample_rate"] = (int)SAMPLING_FREQUENCY;
  doc["duration_seconds"] = BUFFER_DURATION_SECONDS;
  doc["num_samples"] = samplesToSend;

  String payload;
  serializeJson(doc, payload);
  int code = http.POST(payload);
  Serial.printf("📤 Sent bark with %d samples → HTTP %d\n", samplesToSend, code);
  http.end();
}

bool testInternetConnection() {
  WiFiClient client;
  const char* host = "www.google.com";
  const int port = 80;

  Serial.print("🌐 Pinging ");
  Serial.print(host);

  if (client.connect(host, port)) {
    Serial.println(" ... success");
    client.stop();
    return true;
  } else {
    Serial.println(" ... failed");
    return false;
  }
}
