#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Filters.h> // Library Butterworth Filter
#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

// MPU6050 sensor initialization
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO
#define BUTTERWORTH_FILTER

#define LED_PIN 13
bool blinkState = false;

// Sampling frequency
const double f_s = 100; // Hz
// Cut-off frequency (-3 dB)
const double f_c = 10; // Hz
// Normalized cut-off frequency
const double f_n = 2 * f_c / f_s;

// Sample timer
Timer<micros> timer = std::round(1e6 / f_s);

// Define Butterworth filters for each axis
auto butterworthAccX = butter<9>(f_n);
auto butterworthAccY = butter<9>(f_n);
auto butterworthAccZ = butter<9>(f_n);
auto butterworthGyroX = butter<9>(f_n);
auto butterworthGyroY = butter<9>(f_n);
auto butterworthGyroZ = butter<9>(f_n);

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(115200);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setXGyroOffset(192);
  accelgyro.setYGyroOffset(-7);
  accelgyro.setZGyroOffset(-11);

  accelgyro.setXAccelOffset(-3967);
  accelgyro.setYAccelOffset(-933);
  accelgyro.setZAccelOffset(825);

  // configure Arduino LED pin for output
  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  if (timer)
  {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float Gx, Gy, Gz, Ax, Ay, Az;

    Ax = float(ax) / 16384.00;
    Ay = float(ay) / 16384.00;
    Az = float(az) / 16384.00;

    Gx = float(gx) / 131;
    Gy = float(gy) / 131;
    Gz = float(gz) / 131;

#ifdef BUTTERWORTH_FILTER
    // Apply Butterworth filters
    Ax = butterworthAccX(Ax);
    Ay = butterworthAccY(Ay);
    Az = butterworthAccZ(Az);

    Gx = butterworthGyroX(Gx);
    Gy = butterworthGyroY(Gy);
    Gz = butterworthGyroZ(Gz);
#endif

    Serial.print(Gx);
    Serial.print(",");
    Serial.print(Gy);
    Serial.print(",");
    Serial.print(Gz);
    Serial.print(",");
    Serial.print(Ax);
    Serial.print(",");
    Serial.print(Ay);
    Serial.print(",");
    Serial.println(Az);

    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
