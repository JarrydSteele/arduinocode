#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include "Wire.h"
#include "MPU9250.h"
#include "EEPROM.h"
#include "SparkFunBME280.h"

const long SERIAL_REFRESH_TIME = 25;
unsigned long refresh_time;

//FIXME: make work

// Start ADC Init

Adafruit_ADS1X15 ADC1;         //Address (0x48)
Adafruit_ADS1X15 ADC2;         //Address (0x49)

int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7;
// End ACD Init

// Start IMU Init


MPU9250 mpu;

bool calibrate = false;

float AccBiasX = -38.42, AccBiasY = 11.31, AccBiasZ = 197.08;
float GyroBiasX = 0.77, GyroBiasY = 1.00, GyroBiasZ = -0.26;
float MagBiasX = -286.83, MagBiasY = 510.20, MagBiasZ = -287.20;
float MagScaleX = 1.26, MagScaleY = 0.71, MagScaleZ = 1.26;

float calibrated = 0;

float pitch, roll, yaw;

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
// End IMU Init

// Start Pressure Init

BME280 PressureSensor; //Uses I2C address 0x76 (jumper closed)
float pressure, temp_c, altitude;
// End Pressure Init

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
  //Wire.setClock(400000); //Increase to fast I2C speed!  //**************************************
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  // calibrate anytime you want to
  if (calibrate)
  {
    // calibration sequence
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    // get values from calibration
    AccBiasX = mpu.getAccBiasX();     AccBiasY = mpu.getAccBiasY();     AccBiasZ = mpu.getAccBiasZ();
    GyroBiasX = mpu.getGyroBiasX();   GyroBiasY = mpu.getGyroBiasY();   GyroBiasZ = mpu.getGyroBiasZ();
    MagBiasX = mpu.getMagBiasX();     MagBiasY = mpu.getMagBiasY();     MagBiasZ = mpu.getMagBiasZ();
    MagScaleX = mpu.getMagScaleX();   MagScaleY = mpu.getMagScaleY();   MagScaleZ = mpu.getMagScaleZ();
    
    // write calibration values to EEPROM
    EEPROM.put(0,AccBiasX);   EEPROM.put(4,AccBiasY);     EEPROM.put(8,AccBiasZ);
    EEPROM.put(12,GyroBiasX);  EEPROM.put(16,GyroBiasY);    EEPROM.put(20,GyroBiasZ);
    EEPROM.put(24,MagBiasX);   EEPROM.put(28,MagBiasY);     EEPROM.put(32,MagBiasZ);
    EEPROM.put(36,MagScaleX);  EEPROM.put(40,MagScaleY);   EEPROM.put(44,MagScaleZ);
    
    Serial.print("New Calibration Parameters written to EEPROM.");
    delay(1000);

    print_calibration();
    mpu.verbose(false);
    
    calibrated = 1;
    EEPROM.put(48,calibrated);

  } 
  else 
  {
    EEPROM.get(48,calibrated);
    
    if (calibrated) 
    {
      Serial.print("Fetching Calibration Parameters from EEPROM.");
      EEPROM.get(0,AccBiasX);   EEPROM.get(4, AccBiasY);    EEPROM.get(8, AccBiasZ);
      EEPROM.get(12,GyroBiasX);  EEPROM.get(16, GyroBiasY);   EEPROM.get(20, GyroBiasZ);
      EEPROM.get(24,MagBiasX);   EEPROM.get(28, MagBiasY);    EEPROM.get(32, MagBiasZ);
      EEPROM.get(36,MagScaleX);  EEPROM.get(40, MagScaleY);  EEPROM.get(44, MagScaleZ);
    }
    else
    {
      Serial.print("Using Default Calibration Parameters.");
    }
  
    mpu.setAccBias(AccBiasX, AccBiasY, AccBiasZ);
    mpu.setGyroBias(GyroBiasX, GyroBiasY, GyroBiasZ);
    mpu.setMagBias(MagBiasX, MagBiasY, MagBiasZ);
    mpu.setMagScale(MagScaleX, MagScaleY, MagScaleZ);
    delay(1000);
  }


    
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
  mpu.update();
  yaw = mpu.getYaw();
  pitch = mpu.getPitch();
  roll = mpu.getRoll();
  print_roll_pitch_yaw();
  

  // End IMU Readings

  // Start Pressure Readings
  pressure = PressureSensor.readFloatPressure();          //Pressure
  temp_c = PressureSensor.readTempC();                    //Temp (C)
  altitude = PressureSensor.readFloatAltitudeMeters();    //Locally Adjusted Altitude (m)
  // End Pressure Readings

  if (millis() > refresh_time)
  {
    printtelem();
    refresh_time=millis()+SERIAL_REFRESH_TIME;
  }
  
  
  delay(100);
}

