#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "ArduinoLowPower.h"
#include <Adafruit_MPL3115A2.h>
#include <SPIMemory.h>

const int flashCS = 5; // Chip select for Flash

SPIFlash flash(flashCS, &SPI); // Our Flash is on a different SPI bus

Adafruit_MPL3115A2 mpl;
RTC_DS3231 rtc;

void initFlash(){
  SPI.begin();
  flash.begin();
  // uint8_t b1, b2, b3;
  uint32_t JEDEC = flash.getJEDECID();
  // //uint16_t ManID = flash.getManID();
  // b1 = (JEDEC >> 16);
  // b2 = (JEDEC >> 8);
  // b3 = (JEDEC >> 0);
  Serial.println("Initializing SPI Flash");
  // Serial.printf("Manufacturer ID: %02xh\nMemory Type: %02xh\n", b1, b2);
  Serial.printf("JEDEC_ID: %04x\n",JEDEC);
  uint32_t cap = flash.getCapacity(); 
  Serial.printf("Flash Capacity: %ld bytes\n",cap);
  flash.eraseChip();
  Serial.println("Erased Chip!");
}

void setup() {
  // initialize digital pin 13 as an output.
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }



  pinMode(13, OUTPUT);



  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }


  if (!mpl.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  // set mode before starting a conversion
  Serial.println("Setting mode to altimeter.");
  mpl.setMode(MPL3115A2_ALTIMETER);
  initFlash();



}

DateTime checkRTC(){

    DateTime now = rtc.now();

    // Serial.print(now.year(), DEC);
    // Serial.print('-');
    // Serial.printf("%02d",now.month(), DEC);
    // Serial.print('-');
    // Serial.printf("%02d",now.day(), DEC);
    // Serial.print("T");
    // Serial.printf("%02d",now.hour(), DEC);
    // Serial.print(':');
    // Serial.printf("%02d",now.minute(), DEC);
    // Serial.print(':');
    // Serial.printf("%02d",now.second(), DEC);
    // Serial.println();

    // Serial.print(" since midnight 1/1/1970 = ");
    // Serial.print(now.unixtime());

    // Serial.print(" Temperature: ");
    // Serial.print(rtc.getTemperature());
    // Serial.println(" C");
  return now;

}

float checkAltimeter(){
  float pressure = mpl.getPressure();
  float altitude = mpl.getAltitude();
  float temperature = mpl.getTemperature();

  // Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
  // Serial.print("altitude = "); Serial.print(altitude); Serial.println(" m");
  // Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");
  return altitude;

}

// the loop function runs over and over again forever
void loop() {
  // digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(1000);              // wait for a second
  // digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  // delay(1000);              // wait for a second
  analogReadResolution(12);

  uint16_t lightSensorValue = analogRead(A0);
  DateTime dt = checkRTC();
  float altitude = checkAltimeter();
  Serial.printf("Unix Time: %ld | Altitude: %0.2f | Analog Value: %ld\n",dt.unixtime(), altitude, lightSensorValue);
  delay(1000);
}
