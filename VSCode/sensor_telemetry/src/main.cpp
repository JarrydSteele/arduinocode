#include <Arduino.h>





// Start ADC Init
#include "Adafruit_ADS1X15.h"
#include "Wire.h"
Adafruit_ADS1X15 ADC1;         //Address (0x48)
Adafruit_ADS1X15 ADC2;         //Address (0x49)

int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7;
// End ACD Init

// Start IMU Init
#include "GY521.h"
GY521 sensor(0x68);

uint32_t counter = 0;

float pitch, roll, yaw;
// End IMU Init

// Start Pressure Init
#include "SparkFunBME280.h"
BME280 PressureSensor; //Uses I2C address 0x76 (jumper closed)
float pressure, temp_c, altitude;
// End Pressure Init

const long SERIAL_REFRESH_TIME = 100;
unsigned long refresh_time;



void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

// Start ADC Setup
  Serial.println("Getting single-ended readings from AIN0..7");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  ADC1.begin(); //0x48
  ADC2.begin(0x49);
// End ADC Setup

// Start IMU Setup
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!  //**************************************

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
  
  // set callibration values from calibration sketch.
  sensor.axe = 0;
  sensor.aye = 0;
  sensor.aze = 0;
  sensor.gxe = 0;
  sensor.gye = 0;
  sensor.gze = 0;
// End IMU Setup

// Start Pressure Setup
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.

  PressureSensor.setI2CAddress(0x76); //Connect to a second sensor
  if (PressureSensor.beginI2C() == false) Serial.println("Sensor B connect failed");

  PressureSensor.setReferencePressure(100200); //Adjust the sea level pressure used for altitude calculations
// End Pressure Setup
  
}



void printtelem(void)
{
  // Start Print ADC Readings
  Serial.print("AIN0: ");                        Serial.print(adc0);
  Serial.print(", AIN1: ");                      Serial.print(adc1);
  Serial.print(", AIN2: ");                      Serial.print(adc2);
  Serial.print(", AIN3: ");                      Serial.print(adc3);
  Serial.print(", AIN4: ");                      Serial.print(adc4);
  Serial.print(", AIN5: ");                      Serial.print(adc5);
  Serial.print(", AIN6: ");                      Serial.print(adc6);
  Serial.print(", AIN7: ");                      Serial.print(adc7);
  //Serial.println("");
  // End Print ADC Readings

  // Start Print IMU Readings
  Serial.print(", Pitch: ");                     Serial.print(pitch, 3);
  Serial.print(", Roll: ");                      Serial.print(roll, 3);
  Serial.print(", Yaw: ");                       Serial.print(yaw, 3);
  //Serial.println();
  // End Print IMU Readings

  // Start Print Pressure Readings
  Serial.print(", Pressure: ");                  Serial.print(pressure, 0);
  Serial.print(", Temp: ");                      Serial.print(temp_c, 2);
  Serial.print(", Locally Adjusted Altitude: "); Serial.print(altitude, 2);

  Serial.println();
  // End Print Pressure Readings
}

void printrawtelem(void)
{
  // Start Print ADC Readings
  Serial.print(adc0);                Serial.print("\t");
  Serial.print(adc1);                Serial.print("\t");
  Serial.print(adc2);                Serial.print("\t");
  Serial.print(adc3);                Serial.print("\t");
  Serial.print(adc4);                Serial.print("\t");
  Serial.print(adc5);                Serial.print("\t");
  Serial.print(adc6);                Serial.print("\t");
  Serial.print(adc7);                Serial.print("\t");
  // End Print ADC Readings

  // Start Print IMU Readings
  Serial.print(pitch, 3);            Serial.print("\t");
  Serial.print(roll, 3);             Serial.print("\t");
  Serial.print(yaw, 3);              Serial.print("\t");
  // End Print IMU Readings

  // Start Print Pressure Readings
  Serial.print(pressure, 0);         Serial.print("\t");
  Serial.print(temp_c, 2);           Serial.print("\t");
  Serial.print(altitude, 2);

  Serial.println();
  // End Print Pressure Readings
}



void loop(void)
{ 
  // Start ADC Readings
  adc0 = ADC1.readADC_SingleEnded(0);
  adc1 = ADC1.readADC_SingleEnded(1);
  adc2 = ADC1.readADC_SingleEnded(2);
  adc3 = ADC1.readADC_SingleEnded(3);
  adc4 = ADC2.readADC_SingleEnded(0);
  adc5 = ADC2.readADC_SingleEnded(1);
  adc6 = ADC2.readADC_SingleEnded(2);
  adc7 = ADC2.readADC_SingleEnded(3);
  // End ADC Readings

  // Start IMU Readings
  sensor.read();
  pitch = sensor.getPitch();
  roll  = sensor.getRoll();
  yaw   = sensor.getYaw();
  counter++;
  // End IMU Readings

  // Start Pressure Readings
  pressure = PressureSensor.readFloatPressure();          //Pressure
  temp_c = PressureSensor.readTempC();                    //Temp (C)
  altitude = PressureSensor.readFloatAltitudeMeters();  //Locally Adjusted Altitude (m)
  // End Pressure Readings

  if (millis() > refresh_time)
  {
    printrawtelem();
    refresh_time=millis()+SERIAL_REFRESH_TIME;
  }
  
  
  delay(100);
}

