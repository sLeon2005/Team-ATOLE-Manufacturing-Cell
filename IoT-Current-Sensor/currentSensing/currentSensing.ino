// AC Current Monitoring System Using SCT-013 and MQTT
// Team: ATOLE

#include <Wire.h>               // I2C communication library
#include <Adafruit_ADS1X15.h>   // ADS1115 libary
#include <WiFi.h>               // WiFi libary
#include <PubSubClient.h>
#include <time.h>


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

// POWER & ENERGY METRICS
constexpr float lineVoltage = 220.0f;      // Voltage between phases
constexpr float powerFactor = 0.85f;       // Estimated power factor
constexpr float electricityRate = 2.5f;    // MXN per kWh
float energykWh = 0.0f;
unsigned long lastTime = 0;

// MACHINE STATES VARIABLES
enum MachineState {
  OFF,
  IDLE,
  CUTTING,
  FAULT
};

MachineState state = OFF;

constexpr float off_threshold = 0.40f;
constexpr float cutting_threshold = 7.3f;
constexpr float max_safe_current = 15.0f;

unsigned long lastStateUpdate = 0;
float off_time_s = 0.0f;
float idle_time_s = 0.0f;
float cutting_time_s = 0.0f;
float fault_time_s = 0.0f;


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

// MACHINE STATE LOGIC
void updateMachineState(float loadCurrent) {
  if (loadCurrent > max_safe_current) {
      state = FAULT;
  }
  else if (loadCurrent < off_threshold) {
      state = OFF;
  }
  else if (loadCurrent >= cutting_threshold) {
      state = CUTTING;
  }
  else {
      state = IDLE;
  }
}

// STATE TIME TRACKING
void updateStateTime() {
  unsigned long now = millis();
  if (lastStateUpdate == 0) {
    lastStateUpdate = now;
    return;
  }
  float delta_s = (now - lastStateUpdate) / 1000.0f;
  lastStateUpdate = now;

  switch (state) {
    case OFF:
      off_time_s += delta_s;
      break;
    case IDLE:
      idle_time_s += delta_s;
      break;
    case CUTTING:
      cutting_time_s += delta_s;
      break;
    case FAULT:
      fault_time_s += delta_s;
      break;
  }
}


// SETUP
void setup() {
  Serial.begin(115200);
  Wire.begin();
  setup_wifi();

  lastTime = millis();

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

  // Power calculation
  float power = 1.732f * lineVoltage * loadCurrent * powerFactor;

  // Energy calculation
  unsigned long currentTime = millis();
  float deltaHours = (currentTime-lastTime) / 3600000.0f;
  lastTime = currentTime;
  energykWh += (power/1000.f) * deltaHours;

  // Cost calculation
  float costMXN = energykWh * electricityRate;

  // States functions
  updateMachineState(loadCurrent);
  updateStateTime();

  //Results
  Serial.print("Current: ");
  Serial.print(loadCurrent,4);
  Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(power/1000.0f,4);
  Serial.println(" kW");

  Serial.print("Energy: ");
  Serial.print(energykWh,4);
  Serial.println(" kWh");

  Serial.print("Cost: ");
  Serial.print(costMXN,4);
  Serial.println(" MXN");


  // Publish with MQTT
  char buf[32];

  // Current
  dtostrf(loadCurrent, 6, 2, buf);
  client.publish("current/nr", buf);

  // Power
  dtostrf(power/1000.0f, 6, 2, buf);
  client.publish("power/nr", buf);

  // Energy
  dtostrf(energykWh, 6, 2, buf);
  client.publish("energy/nr", buf);

  // Cost
  dtostrf(costMXN, 8, 2, buf);
  client.publish("cost/nr", buf);

  // State
  client.publish("state/nr", String(state).c_str());

  // Machining states times
  dtostrf(off_time_s, 8, 2, buf);
  client.publish("time/off/nr", buf);

  dtostrf(idle_time_s, 8, 2, buf);
  client.publish("time/idle/nr", buf);

  dtostrf(cutting_time_s, 8, 2, buf);
  client.publish("time/cutting/nr", buf);

  dtostrf(fault_time_s, 8, 2, buf);
  client.publish("time/fault/nr", buf);

  delay(500);
}
