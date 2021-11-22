#include <Arduino.h>
#include "MPU9250.h"
#include "EEPROM.h"

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

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
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
    delay(5000);

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
    delay(5000);
  }
}

void loop() {
    if (mpu.update()) 
    {
        yaw = mpu.getYaw();
        pitch = mpu.getPitch();
        roll = mpu.getRoll();
        
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 250) 
        {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}