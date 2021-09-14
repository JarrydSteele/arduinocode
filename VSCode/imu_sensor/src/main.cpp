#include <Arduino.h>
#include "GY521.h"

GY521 sensor(0x68);

uint32_t counter = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);

  Wire.begin();

  delay(100);
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521");
    delay(1000);
  }
  sensor.setAccelSensitivity(2);  // 8g
  sensor.setGyroSensitivity(1);   // 500 degrees/s

  sensor.setThrottle();
  Serial.println("start...");
  
  // set callibration values from calibration sketch.
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
  sensor.gxe = 0;
  sensor.gye = 0;
  sensor.gze = 0;
}

void loop()
{
  sensor.read();
  float pitch = sensor.getPitch();
  float roll  = sensor.getRoll();
  float yaw   = sensor.getYaw();

  Serial.print("Pitch: ");
  Serial.print(pitch, 3);
  Serial.print(", Roll: ");
  Serial.print(roll, 3);
  Serial.print(", Yaw: ");
  Serial.print(yaw, 3);
  Serial.println();

  counter++;
}

// -- END OF FILE --