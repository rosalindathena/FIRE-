#include <Arduino_BMI270_BMM150.h>

// Hard iron offsets (youll have to calibrate it every time u plug in ur arduino it sucks)
float hardIronOffsetX = -8.5; // X offset
float hardIronOffsetY = -2.5; // Y offset
float hardIronOffsetZ = -0.5; // Z offset

// Complementary filter constant (so like here the gyro integration method has a weighting of 0.98 and is corrected by the 0.02 accel + magno)
const float alpha = 0.98;
const float correctionThreshold = 4.0; // Maximum allowed disagreement in degrees (so if the gyro more than 4 degrees, it assumes its completely wrong and takes the accel + magno value)
const int correctionWindow = 4; // Number of readings to average

float gyroPitch = 0, gyroYaw = 0, gyroRoll = 0;
unsigned long lastTime = 0;
bool initialized = false;

// Circular buffers for accel/mag averages
float pitchAccMagBuffer[correctionWindow];
float yawAccMagBuffer[correctionWindow];
float rollAccMagBuffer[correctionWindow];
int bufferIndex = 0;

// Buffer to track average disagreement counts
int disagreementCount = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Yaw,Pitch,Roll,UpAcceleration,NorthAcceleration,EastAcceleration"); // Header for the Serial Plotter

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  lastTime = millis();
}

void loop() {
  float magX, magY, magZ;
  float accX, accY, accZ;
  float gyrX, gyrY, gyrZ;

  if (IMU.magneticFieldAvailable() && IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    // Read magnetometer data
    IMU.readMagneticField(magX, magY, magZ);
    // Apply hard iron offset correction 
    magX -= hardIronOffsetX;
    magY -= hardIronOffsetY;
    magZ -= hardIronOffsetZ;

    // Read accelerometer data
    IMU.readAcceleration(accX, accY, accZ);

    // Normalize accelerometer values to get the down vector
    float accNorm = sqrt(accX * accX + accY * accY + accZ * accZ);
    float downX = accX / accNorm;
    float downY = accY / accNorm;
    float downZ = accZ / accNorm;

    // Calculate east vector as the cross product of down and magnetic field vectors
    float eastX = downY * magZ - downZ * magY;
    float eastY = downZ * magX - downX * magZ;
    float eastZ = downX * magY - downY * magX;

    // Normalize east vector
    float eastNorm = sqrt(eastX * eastX + eastY * eastY + eastZ * eastZ);
    eastX /= eastNorm;
    eastY /= eastNorm;
    eastZ /= eastNorm;

    // Calculate north vector as the cross product of east and down vectors
    float northX = eastY * downZ - eastZ * downY;
    float northY = eastZ * downX - eastX * downZ;
    float northZ = eastX * downY - eastY * downX;

    // Normalize north vector
    float northNorm = sqrt(northX * northX + northY * northY + northZ * northZ);
    northX /= northNorm;
    northY /= northNorm;
    northZ /= northNorm;

    // Construct the Direction Cosine Matrix (DCM) (so like to convert from arduino xyz to real xyz)
    float DCM[3][3] = {
      {northX, northY, northZ},
      {eastX, eastY, eastZ},
      {downX, downY, downZ}
    };

    // Calculate Euler angles from the DCM (this is all yap from a research paper i dont understand but it seems to work)
    float pitchAccMag = atan2(-DCM[2][0], sqrt(DCM[2][1] * DCM[2][1] + DCM[2][2] * DCM[2][2])) * 180.0 / PI;
    float yawAccMag = atan2(DCM[1][0], DCM[0][0]) * 180.0 / PI;
    float rollAccMag = atan2(DCM[2][1], DCM[2][2]) * 180.0 / PI;

    if (!initialized) {
      gyroPitch = pitchAccMag;
      gyroYaw = yawAccMag;
      gyroRoll = rollAccMag;
      initialized = true;
    }

    // Read gyroscope data
    IMU.readGyroscope(gyrX, gyrY, gyrZ);

    // Gyroscope integration
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    gyroPitch += gyrX * dt;
    gyroYaw += gyrY * dt;
    gyroRoll += gyrZ * dt;

    // Complementary filter to fuse accelerometer/magnetometer and gyroscope data (so that 0.98 weighting thing from the top to correct the gyro essentially)
    float pitch = alpha * (gyroPitch) + (1 - alpha) * pitchAccMag;
    float yaw = alpha * (gyroYaw) + (1 - alpha) * yawAccMag;
    float roll = alpha * (gyroRoll) + (1 - alpha) * rollAccMag;

    // Update gyroscope angles with the fused data to prevent drift
    gyroPitch = pitch;
    gyroYaw = yaw;
    gyroRoll = roll;

    // Store accel/mag readings in the circular buffer
    pitchAccMagBuffer[bufferIndex] = pitchAccMag;
    yawAccMagBuffer[bufferIndex] = yawAccMag;
    rollAccMagBuffer[bufferIndex] = rollAccMag;
    bufferIndex = (bufferIndex + 1) % correctionWindow;

    // Calculate the average accel/mag readings (reduces noise cause these tend to be jumpy af)
    float avgPitchAccMag = 0, avgYawAccMag = 0, avgRollAccMag = 0;
    for (int i = 0; i < correctionWindow; i++) {
      avgPitchAccMag += pitchAccMagBuffer[i];
      avgYawAccMag += yawAccMagBuffer[i];
      avgRollAccMag += rollAccMagBuffer[i];
    }
    avgPitchAccMag /= correctionWindow;
    avgYawAccMag /= correctionWindow;
    avgRollAccMag /= correctionWindow;

    // Check if the gyro and accel/mag readings disagree significantly (cause gyro is ass when rotated fast istg)
    if (abs(pitch - avgPitchAccMag) > correctionThreshold ||
        abs(yaw - avgYawAccMag) > correctionThreshold ||
        abs(roll - avgRollAccMag) > correctionThreshold) {
      disagreementCount++;
    } else {
      disagreementCount = 0; // Reset the count if there's no significant disagreement
    }

    // Correct the gyro readings if the disagreement persists over the last 5 averages
    if (disagreementCount >= correctionWindow) {
      gyroPitch = avgPitchAccMag;
      gyroYaw = avgYawAccMag;
      gyroRoll = avgRollAccMag;
      disagreementCount = 0; // Reset the count after correction
    }

    // Calculate the acceleration in the up, north, and east directions (this fails idek why when the rest seems to be fine)
    float upAcc = downX * accX + downY * accY + downZ * accZ;
    float northAcc = northX * accX + northY * accY + northZ * accZ;
    float eastAcc = eastX * accX + eastY * accY + eastZ * accZ;

    // Print combined results for the Serial Plotter
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",  ");
    Serial.print(upAcc);
    Serial.print(",");
    Serial.print(northAcc);
    Serial.print(",");
    Serial.println(eastAcc);
  }
}
