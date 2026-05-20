#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

// ADS1115
const float ADS_VREF = 4.096;      // GAIN_ONE
const float ADS_MAX = 32768.0;

// Offset virtual
const float OFFSET = 1.65;

// Burden resistor
const float RESISTOR = 1000;

// Muestreo
const int NUM_SAMPLES = 2000;

// Número de picos a promediar
const int NUM_PEAKS = 800;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  if (!ads.begin()) {

    Serial.println("ADS1115 no encontrado");

    while (1);
  }

  // ±4.096V
  ads.setGain(GAIN_ONE);

  delay(1000);

  Serial.println("Leyendo corriente RMS...");
}

void loop() {

  float peaks[NUM_PEAKS];

  // Inicializar arreglo
  for (int i = 0; i < NUM_PEAKS; i++) {
    peaks[i] = 0;
  }

  // Muestreo
  for (int i = 0; i < NUM_SAMPLES; i++) {

    // Leer ADS1115 canal A0
    int16_t raw = ads.readADC_SingleEnded(0);

    // ADC -> volts
    float voltage = (raw * ADS_VREF) / ADS_MAX;

    // Quitar offset
    float acVoltage = voltage - OFFSET;

    // Valor absoluto
    float absVoltage = abs(acVoltage);

    // Buscar el menor pico guardado
    int minIndex = 0;

    for (int j = 1; j < NUM_PEAKS; j++) {

      if (peaks[j] < peaks[minIndex]) {
        minIndex = j;
      }
    }

    // Reemplazar si el valor actual es mayor
    if (absVoltage > peaks[minIndex]) {
      peaks[minIndex] = absVoltage;
    }

    delayMicroseconds(1000);
  }

  // Promediar picos máximos
  float vPeak = 0;

  for (int i = 0; i < NUM_PEAKS; i++) {
    vPeak += peaks[i];
  }

  vPeak /= NUM_PEAKS;

  // Vrms para senoide
  float vrms = vPeak / sqrt(2.0);

  // Corriente RMS secundaria
  float irms = vrms / RESISTOR;

  // Escalado SCT013-000
  float imeasure = irms * 2000.0;

  Serial.print("Vpeak: ");
  Serial.print(vPeak, 4);
  Serial.println(" V");

  Serial.print("Vrms: ");
  Serial.print(vrms, 4);
  Serial.println(" V");

  Serial.print("Measured current: ");
  Serial.print(imeasure, 3);
  Serial.println(" A");

  Serial.println();

  delay(500);
}