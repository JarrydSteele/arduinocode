#include <SimpleKalmanFilter.h>

/*
 This sample code demonstrates how the SimpleKalmanFilter object can be used with a
 pressure sensor to smooth a set of altitude measurements. 
 This example needs a BMP180 barometric sensor as a data source.
 https://www.sparkfun.com/products/11824

 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

#include <Wire.h>

#include "SparkFunBME280.h"
BME280 pressure; //Uses I2C address 0x76 (jumper closed)

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  pressure.setI2CAddress(0x76); //Connect to a second sensor
  if (pressure.beginI2C() == false) Serial.println("Sensor B connect failed");

  pressure.setReferencePressure(100200); //Adjust the sea level pressure used for altitude calculations
 
}

void loop() {
  
  float p = pressure.readFloatPressure();
  float altitude_m = pressure.readFloatAltitudeMeters();
  float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude_m);

  if (millis() > refresh_time) {
    Serial.print(altitude_m,6);
    Serial.print(",");
    Serial.print(estimated_altitude,6);
    Serial.println();
    refresh_time=millis()+SERIAL_REFRESH_TIME;
  }
  
}
