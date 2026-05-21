// AC Current Monitoring System Using SCT-013 and MQTT
// Team: ATOLE

#include <Wire.h>               // I2C communication library
#include <Adafruit_ADS1X15.h>   // ADS1115 libary
#include <WiFi.h>               // WiFi libary
#include <PubSubClient.h>

// WIFI AND MQTT CONFIGURATION
const char* ssid =  "Tec-IoT";                        // WiFi network name
const char* password = "spotless.magnetic.bridge";    // WiFi network password
const char* mqtt_server = "10.25.110.236";            // MQTT broker IP address

WiFiClient espClient;                       // WiFi client used by ESP32 for network communication
PubSubClient client(espClient);             // MQTT client that uses the WiFi connection

// ADC CONFIGURATION
Adafruit_ADS1115 ads;                 // Create a ADS1115 object
constexpr float ADS_VREF = 4.096f;    // Maximum ADC reference voltage with GAIN ONE
constexpr float ADS_MAX = 32768.0f;   // Maximum ADC resolution (16 bit ADC)

// SENSOR CONFIGURATION
constexpr float OFFSET_VOLTAGE = 1.64f;     // DC offset voltage for AC waveform centering
constexpr float BURDEN_RESISTOR = 150.0f;   // Burden resistor value
constexpr float SCT013_RATIO = 2000.0f;     // SCT013 current transformer ratio
constexpr float ADC_CALIBRATION = 1.0052f;  // ADC calibration correction factor

// SAMPLING CONFIGURATION
constexpr int SAMPLE_COUNT = 500;     // Number of samples to capture
constexpr int SAMPLE_DELAY_US = 200;  // Delay between samples (us)

// SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();

  // WiFi + MQTT
  client.setServer(mqtt_server,1883);

  // ADS115
  if (!ads.begin()) {
    Serial.println("ADS1115 not found");
    while (1)
      ;
  }

  ads.setGain(GAIN_ONE);                  // Configure ADC gain to ±4.096V
  ads.setDataRate(RATE_ADS1115_860SPS);   // Set ADC sampling rate to 860 samples/sec
  delay(1000);                            // Stabilization
  Serial.println("Reading voltage...");
}

// WIFI SETUP
void setup_wifi() {
  delay(10);
  
  // Connecting to a WiFi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  // Wait until the ESP32 successfully connects to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// MQTT CONNECTION
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("ESP32_ATOLE")){
      Serial.println("Connected");

    }
    else {
      Serial.print("Failed,rc=");
      Serial.print(client.state());
      Serial.println("Trying again");
      delay(1000);
    }
  }
}

// LOOP
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  int16_t raw;    // Raw ADC reading
  float voltage;  // Calculated voltage

  // Variables to track waveform peaks
  float maxPeak = OFFSET_VOLTAGE;
  float minPeak = OFFSET_VOLTAGE;

  // Collect multiple samples 
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    raw = ads.readADC_SingleEnded(0);       // Read single-ended ADC channel 0
    voltage = (raw * ADS_VREF) / ADS_MAX;   // Convert ADC value to voltage
    voltage *= ADC_CALIBRATION;             // Apply calibration factor

    // Update maximum and minimum peak values
    if (voltage > maxPeak) {
      maxPeak = voltage;
    }
    if (voltage < minPeak) {
      minPeak = voltage;
    }
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  // Current calculation
  float vpp = maxPeak - minPeak;            // Calculate peak to peak voltage
  float vrms = 0.3536f * vpp;               // RMS voltage formula
  float irms = vrms / BURDEN_RESISTOR;      // Converting from RMS voltage to RMS current
  float loadCurrent = irms * SCT013_RATIO;  // Scaling current based on the SCT013's ratio
  Serial.print("Current: ");
  Serial.print(loadCurrent);
  Serial.println(" A");

  // Publish with MQTT
  char currentString[16];                     // Convert float to string
  dtostrf(loadCurrent, 6, 2, currentString);
  
  if(client.publish("sct", currentString)) {
    Serial.println("MQTT publish OK");
    }
    else {
      Serial.println("MQTT publish FAILED");
      }
  delay(500);
}
