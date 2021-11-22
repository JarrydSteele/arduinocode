#include "Arduino.h"
#include <Wire.h>
#include "SparkFunBME280.h"

BME280 PressureSensor; //Uses I2C address 0x76 (jumper closed)

// the setup() method runs once, when the sketch starts

void setup() {
  // initialize the digital pin as an output.

  Serial.begin(115200);
  Serial.println("Example showing alternate I2C addresses");

  

  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  PressureSensor.setI2CAddress(0x76); //Connect to a second sensor

  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!
  
  if (PressureSensor.beginI2C() == false) Serial.println("Sensor B connect failed");

  PressureSensor.setReferencePressure(102650); //Adjust the sea level pressure used for altitude calculations 100200
}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  Serial.print(" Pressure: ");
  Serial.print(PressureSensor.readFloatPressure(), 0);

  Serial.print(" Temp: ");
  Serial.print(PressureSensor.readTempC(), 2);

  Serial.print(" Locally Adjusted Altitude: ");
  Serial.print(PressureSensor.readFloatAltitudeMeters(), 2);

  Serial.println();

  delay(50);
}