// AC Current Measurement System using SCT-013 and ADS115
// Team: ATOLE

#include <Wire.h>               // I2C communication library
#include <Adafruit_ADS1X15.h>   // ADS1115 libary

Adafruit_ADS1115 ads;           // Create a ADS1115 object

// ADC CONFIGURATION
constexpr float ADS_VREF = 4.096f;    // Maximum ADC reference voltage with GAIN ONE
constexpr float ADS_MAX = 32768.0f;   // Maximum ADC resolution (16 bit ADC)

// SENSOR CONFIGURATION
constexpr float OFFSET_VOLTAGE = 1.64f;     // DC offset voltage for AC waveform centering
constexpr float BURDEN_RESISTOR = 150.0f;   // Burden resistor value
constexpr float SCT013_RATIO = 2000.0f;     // SCT013 current transformer ratio
constexpr float ADC_CALIBRATION = 1.0052f;  // ADC calibration correction factor

// SAMPLING CONFIGURATION
constexpr int SAMPLE_COUNT = 500;     // Number of samples to capture
constexpr int SAMPLE_DELAY_US = 200;  // Delay between samples (ms)

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Check if ADS115 is connected
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

void loop() {
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

  float vpp = maxPeak - minPeak;            // Calculate peak to peak voltage
  float vrms = 0.3536f * vpp;               // RMS voltage formula
  float irms = vrms / BURDEN_RESISTOR;      // Converting from RMS voltage to RMS current
  float loadCurrent = irms * SCT013_RATIO;  // Scaling current based on the SCT013's ratio
  Serial.println(loadCurrent);
  delay(500);
}

























