#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// =========================
// ADC CONFIGURATION
// =========================

constexpr float ADS_VREF = 4.096f;
constexpr float ADS_MAX = 32768.0f;

// =========================
// SENSOR CONFIGURATION
// =========================

constexpr float OFFSET_VOLTAGE = 1.64f;

constexpr float BURDEN_RESISTOR = 150.0f;

constexpr float SCT013_RATIO = 2000.0f;

constexpr float ADC_CALIBRATION = 1.0052f;

// =========================
// SAMPLING CONFIGURATION
// =========================

constexpr int SAMPLE_COUNT = 500;

constexpr int SAMPLE_DELAY_US = 200;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  if (!ads.begin()) {

    Serial.println("ADS1115 not found");

    while (1)
      ;
  }

  // ±4.096V
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  delay(1000);

  Serial.println("Reading voltage...");
}

void loop() {

  int16_t raw;

  float voltage;

  float maxPeak = OFFSET_VOLTAGE;
  float minPeak = OFFSET_VOLTAGE;

  // get numerous readings from the adc to get min and max values
  for (int i = 0; i < SAMPLE_COUNT; i++) {

    raw = ads.readADC_SingleEnded(0);

    voltage = (raw * ADS_VREF) / ADS_MAX;

    voltage *= ADC_CALIBRATION;

    if (voltage > maxPeak) {
      maxPeak = voltage;
    }

    if (voltage < minPeak) {
      minPeak = voltage;
    }

    delayMicroseconds(SAMPLE_DELAY_US);
  }

  // get peak to peak voltage based on the highest and lowest voltages
  float vpp = maxPeak - minPeak;

  // rms voltage formula
  float vrms = 0.3536f * vpp;

  // converting from rms voltage to rms current
  float irms = vrms / BURDEN_RESISTOR;

  // scaling current based on the SCT013's ratio
  float loadCurrent = irms * SCT013_RATIO;

  Serial.println(loadCurrent);

  delay(500);
}
