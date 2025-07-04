#include <arduinoFFT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <time.h>
#include <base64.h>

const uint16_t SAMPLE_SIZE = 256;          // Must be a power of 2
const double SAMPLING_FREQUENCY = 10000;   // Hz
const float VOLUME_THRESHOLD = 0.45;

double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];

ArduinoFFT FFT = ArduinoFFT(vReal, vImag, SAMPLE_SIZE, SAMPLING_FREQUENCY);

const int micPin = 36;
unsigned long lastBarkTime = 0;
const unsigned long COOLDOWN_MS = 3000;

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
  // === 1. Capture mic samples ===
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    vReal[i] = analogRead(micPin);
    vImag[i] = 0;
    delayMicroseconds(100);  // 10kHz sampling
  }

  // === 2. Compute volume (RMS) ===
  double rms = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    rms += sq(vReal[i]);
  }
  rms = sqrt(rms / SAMPLE_SIZE);
  double volume = rms / 4095.0;  // Normalize (0–1)

  // === 3. Apply FFT (after DC bias removal) ===
  double avg = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) avg += vReal[i];
  avg /= SAMPLE_SIZE;
  for (int i = 0; i < SAMPLE_SIZE; i++) vReal[i] -= avg;

  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // === 4. Find dominant frequency ===
  double maxMag = 0;
  int maxIndex = 0;
  for (int i = 1; i < SAMPLE_SIZE / 2; i++) {
    if (vReal[i] > maxMag) {
      maxMag = vReal[i];
      maxIndex = i;
    }
  }
  double frequency = (maxIndex * SAMPLING_FREQUENCY) / SAMPLE_SIZE;

  // === 5. Trigger bark event if loud enough and cooldown passed ===
  if (volume > VOLUME_THRESHOLD) {
    unsigned long now = millis();
    if (now - lastBarkTime >= COOLDOWN_MS) {
      lastBarkTime = now;
      Serial.printf("⚠️  Possible bark: Vol=%.2f  Freq=%.1f Hz\n", volume, frequency);
      sendSingleBark(volume, frequency);
    } else {
      Serial.println("🕒 Cooldown active — skipping bark");
    }
  }

  delay(100);  // Short loop delay
}

void sendSingleBark(float volume, float frequency) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin("https://bark-server-881599834430.us-central1.run.app/bark");
  http.addHeader("Content-Type", "application/json");

  time_t timestamp = time(nullptr);

  // === Serialize audio data ===
  String rawData = "";
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    rawData += (uint16_t)vReal[i]; // cast to 2-byte integer
    rawData += ",";
  }
  rawData.remove(rawData.length() - 1); // remove trailing comma

  String encoded = base64::encode(rawData);

  String json = "{";
  json += "\"device_id\":\"esp32-yard\",";
  json += "\"timestamp\":" + String(timestamp) + ",";
  json += "\"volume\":" + String(volume, 3) + ",";
  json += "\"frequency\":" + String(frequency, 1) + ",";
  json += "\"audio_base64\":\"" + encoded + "\",";
  json += "\"event\":\"possible_bark\"";
  json += "}";

  int code = http.POST(json);
  Serial.printf("📤 Sent bark → HTTP %d\n", code);
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
