#include <Arduino.h>

#include <Adafruit_ADS1X15.h>

#include <Wire.h>

#include <SPI.h>
 
Adafruit_ADS1115 ADC1;         //Address (0x48)
Adafruit_ADS1115 ADC2;         //Address (0x49)

 
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Hello!");
  
  Serial.println("Getting single-ended readings from AIN0..7");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");
  ADC1.begin(); //0x48
  ADC2.begin(0x49);
}
 
void loop(void)
{
  int16_t adc0, adc1, adc2, adc3, adc4, adc5, adc6, adc7;
 
  adc0 = ADC1.readADC_SingleEnded(0);
  adc1 = ADC1.readADC_SingleEnded(1);
  adc2 = ADC1.readADC_SingleEnded(2);
  adc3 = ADC1.readADC_SingleEnded(3);
  adc4 = ADC2.readADC_SingleEnded(0);
  adc5 = ADC2.readADC_SingleEnded(1);
  adc6 = ADC2.readADC_SingleEnded(2);
  adc7 = ADC2.readADC_SingleEnded(3);
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  Serial.print("AIN4: "); Serial.println(adc4);
  Serial.print("AIN5: "); Serial.println(adc5);
  Serial.print("AIN6: "); Serial.println(adc6);
  Serial.print("AIN7: "); Serial.println(adc7);
  Serial.println();
  
  delay(250);
}