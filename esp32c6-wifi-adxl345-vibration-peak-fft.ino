// Include necessary libraries
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <arduinoFFT.h>
#include <time.h>  // Added for time functions
#include "arduino_secrets.h" //disable if entering credentials below

// WiFi and MQTT credentials - modify these with your own network/server details
char ssid[] = SECRET_SSID;   // your network SSID (name) 
char password[] = SECRET_PASSWORD;   // your network password

const char* mqtt_server = "192.168.1.253";  // e.g., "192.168.1.100" or domain name
const int mqtt_port = 1883;  // default MQTT port

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Create an ADXL345 sensor instance with a unique sensor ID
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool calibrateOnStartup = true; // Set to false to disable calibration at startup
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

// FFT configuration
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1000.0
unsigned long lastFFTTime = 0;

void calibrateSensor() {
  Serial.println("Calibrating sensor... Please keep the sensor still.");
  const int samples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    delay(50); // delay 50ms between samples
  }
  offsetX = sumX / samples;
  offsetY = sumY / samples;
  offsetZ = sumZ / samples;
  Serial.println("Calibration complete.");
  Serial.print("Offset X: "); Serial.println(offsetX);
  Serial.print("Offset Y: "); Serial.println(offsetY);
  Serial.print("Offset Z: "); Serial.println(offsetZ);
}

// Function to perform FFT on x-axis data
void performFFT() {
  double vReal[SAMPLES];
  double vImag[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    double ax = event.acceleration.x - offsetX;
    vReal[i] = ax;
    vImag[i] = 0;
    delay(1); // reduced delay for faster sampling (~1000Hz target)
  }
  ArduinoFFT<double> FFT(vReal, vImag, (uint_fast16_t)SAMPLES, SAMPLING_FREQUENCY);
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double peak = 0;
  int index = 0;
  for (int i = 1; i < SAMPLES/2; i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      index = i;
    }
  }
  double peakFrequency = (index * SAMPLING_FREQUENCY) / SAMPLES;

  // Get current time in milliseconds since epoch
  time_t now = time(NULL);
  unsigned long long timeMs = (unsigned long long)now * 1000ULL;

  char fftMsg[250];
  snprintf(fftMsg, sizeof(fftMsg), "{\"Time\": %llu, \"peak_frequency\": %.2f, \"peak_magnitude\": %.2f}", timeMs, peakFrequency, peak);
  client.publish("esp32/fft", fftMsg);
}

// Function to connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Function to connect/reconnect to the MQTT server
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println(" connected");
      // Publish an initial message
      client.publish("esp32/status", "ESP32C6 connected");
      // Optionally subscribe to topics here if needed
      // client.subscribe("esp32/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(19,20);
  Wire.setClock(400000);  // Set I2C speed to 400 kHz for faster sensor reads
  // Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }
  // Optionally set the range (e.g., +/- 16g). Options: ADXL345_RANGE_2_G, _4_G, _8_G, _16_G
  accel.setRange(ADXL345_RANGE_16_G);
  if (calibrateOnStartup) {
    calibrateSensor();
  }

  // Initialize WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  // Initialize NTP for time synchronization
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("Time synchronized.");
  } else {
    Serial.println("Time synchronization failed.");
  }
}

// New loop: continuously perform FFT without additional delays, streaming FFT only
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Continuously perform FFT and stream results
  performFFT();
} 