#include <Arduino.h>
#include <TinyGPS.h>

const long SERIAL_REFRESH_PERIOD = 1000;
unsigned long serial_refresh_time;

const long GPS_REFRESH_PERIOD = 3000;
unsigned long gps_refresh_time;

// Start GPS Init

TinyGPS gps;

long gps_lat, gps_lon;
float gps_flat, gps_flon;
unsigned long gps_age, gps_date, gps_time, gps_chars;
int gps_year;
byte gps_month, gps_day, gps_hour, gps_minute, gps_second, gps_hundredths;
unsigned short gps_sentences, gps_failed;

/* On Teensy, the UART (real serial port) is always best to use. */

HardwareSerial GPS = Serial2;

void print_gps(TinyGPS &gps);
void gpsPrintFloat(double f, int digits = 2);
void get_gps_data();

bool gps_fix = false;

// End GPS Init

void setup(void)
{
  Serial.begin(115200);
// Start GPS Setup
  GPS.begin(9600);

  gps_refresh_time = millis() + GPS_REFRESH_PERIOD;
// End GPS Setup
  
}

void printtelem(void)
{
  // Start GPS Readings
  if (gps_fix) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    print_gps(gps);
    Serial.println("-------------");
    Serial.println();
  } else {
    Serial.println(" --------------------------------------------------------- No GPS fix.");
  }
  // End GPS Readings
}

void loop(void)
{ 
// Start GPS Readings
  if (GPS.available()) {
    char c = GPS.read();
    //Serial.print(c);  // uncomment to see raw GPS data
    if (gps.encode(c)) {
      gps_fix = true;
    }
  }
  
  if (millis() > gps_refresh_time){
    if (gps_fix)
      get_gps_data();
    gps_refresh_time=millis()+GPS_REFRESH_PERIOD;
  }

  // End GPS Readings

  if (millis() > serial_refresh_time)
  {
    printtelem();
    serial_refresh_time=millis()+SERIAL_REFRESH_PERIOD;
  }

}

void get_gps_data()
{
  gps.get_position(&gps_lat, &gps_lon, &gps_age);
  gps.f_get_position(&gps_flat, &gps_flon, &gps_age);
  gps.get_datetime(&gps_date, &gps_time, &gps_age);
  gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second, &gps_hundredths, &gps_age);
  gps.stats(&gps_chars, &gps_sentences, &gps_failed);
}

void print_gps(TinyGPS &gps)
{
  Serial.print("Lat/Long(float): "); gpsPrintFloat(gps_flat, 5); Serial.print(", "); gpsPrintFloat(gps_flon, 5);
  Serial.print(" Fix age: "); Serial.print(gps_age); Serial.println("ms.");
  Serial.print("Date: "); Serial.print(static_cast<int>(gps_month)); Serial.print("/"); 
  Serial.print(static_cast<int>(gps_day)); Serial.print("/"); Serial.print(gps_year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(gps_hour)); Serial.print(":"); 
  Serial.print(static_cast<int>(gps_minute)); Serial.print(":"); Serial.print(static_cast<int>(gps_second));
  Serial.print("."); Serial.print(static_cast<int>(gps_hundredths));
  Serial.print("  Fix age: ");  Serial.print(gps_age); Serial.println("ms.");
  Serial.print("Alt(float): "); gpsPrintFloat(gps.f_altitude()); Serial.print(" Course(float): "); gpsPrintFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(mps): "); gpsPrintFloat(gps.f_speed_mps()); Serial.print(" (kmph): "); gpsPrintFloat(gps.f_speed_kmph()); Serial.println();
}

void gpsPrintFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}