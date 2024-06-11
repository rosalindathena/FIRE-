#include <Arduino_BMI270_BMM150.h>

const int sampleSize = 500; // Number of samples to collect for calibration

float x_data[sampleSize];
float y_data[sampleSize];
float z_data[sampleSize];

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  collectMagnetometerData();
  calculateOffsets();
}

void loop() {
  // The main loop does nothing; everything is handled in setup.
}

void collectMagnetometerData() {
  int sampleCount = 0;
  Serial.println("Collecting data. Move the sensor around to capture a full range of values.");

  while (sampleCount < sampleSize) {
    float x, y, z;

    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(x, y, z);
      x_data[sampleCount] = x;
      y_data[sampleCount] = y;
      z_data[sampleCount] = z;
      sampleCount++;


      Serial.print(" Sample ");
      Serial.println(sampleCount);

    }

    // Small delay to avoid overwhelming the serial output
  }

  Serial.println("Data collection complete.");
}

void calculateOffsets() {
  float min_x = x_data[0];
  float max_x = x_data[0];
  float min_y = y_data[0];
  float max_y = y_data[0];
  float min_z = z_data[0];
  float max_z = z_data[0];

  for (int i = 1; i < sampleSize; i++) {
    if (x_data[i] < min_x) min_x = x_data[i];
    if (x_data[i] > max_x) max_x = x_data[i];
    if (y_data[i] < min_y) min_y = y_data[i];
    if (y_data[i] > max_y) max_y = y_data[i];
    if (z_data[i] < min_z) min_z = z_data[i];
    if (z_data[i] > max_z) max_z = z_data[i];
  }

  float mag_bias_x = (max_x + min_x) / 2;
  float mag_bias_y = (max_y + min_y) / 2;
  float mag_bias_z = (max_z + min_z) / 2;

  Serial.println("Calibration complete.");

  Serial.print("Hard iron offset: X= ");
  Serial.print(mag_bias_x);
  Serial.print(", Y= ");
  Serial.print(mag_bias_y);
  Serial.print(", Z= ");
  Serial.println(mag_bias_z);
}
