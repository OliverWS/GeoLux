#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "ArduinoLowPower.h"
#include <Adafruit_MPL3115A2.h>
#include <SPIMemory.h>

// This part just for the prototype
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//----------------------------------------------------------------

SPIFlash flash(A2, &SPI); // Our Flash is on a different SPI bus

Adafruit_MPL3115A2 mpl;
RTC_DS3231 rtc;

uint32_t N_SAMPLES_ADDR = 0;
uint32_t nSamples;



struct DataPacket {
  uint32_t timestamp;
  uint16_t luminance;
  float altitude;
};

void initFlash(){
  Serial.println("Initializing SPI Flash");
  SPI.begin();
  flash.setClock(1000000);
  flash.powerUp();
  uint8_t err = flash.begin(MB(128));
  Serial.printf("Initialized Flash with return code: %02x\n",err);

  flash.powerUp();
  // uint8_t b1, b2, b3;
  uint32_t JEDEC = flash.getJEDECID();
  Serial.printf("JEDEC_ID: %04x\n",JEDEC);
  Serial.print("Capacity: ");
  Serial.println(flash.getCapacity());
  Serial.print("Unique ID: ");
  Serial.println(flash.getUniqueID());
  Serial.print("Man ID: ");
  Serial.println(flash.getManID());

}


void setup() {
  pinMode(13, OUTPUT); // initialize digital pin 13 (LED) as an output.
  Serial.begin(9600);
  for(int i=0; i < 5; i ++){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);              // wait for a 1/3 second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(300);              // wait for a 1/3 second
  }


  //Only for Prototyping
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println("SSD1306 allocation failed");
  }
  else {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("P123 Geolocator v0.1");
      display.display();

  }

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
  mpl.setSeaPressure(1020.5);
  initFlash();



}

void displayStatus( char *statusStr){
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.println("P123 Geolocator v0.1");
  display.setCursor(0, 20);
  // Display static text

  display.println(statusStr);
  display.display(); 

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
  // float pressure = mpl.getPressure();
  float altitude = mpl.getAltitude();
  // float temperature = mpl.getTemperature();

  // Serial.print("pressure = "); Serial.print(pressure); Serial.println(" hPa");
  // Serial.print("altitude = "); Serial.print(altitude); Serial.println(" m");
  // Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");
  return altitude;

}

void saveDataPoint(DataPacket packet){
  //flash.powerUp();
  nSamples = flash.readULong(N_SAMPLES_ADDR);
  nSamples = nSamples + 1;
  uint32_t _packetAddress = 10 + sizeof(DataPacket) * nSamples;
  uint8_t code;
  flash.eraseSection(_packetAddress, sizeof(packet));
  code = flash.writeAnything(_packetAddress,packet);
  Serial.printf("Wrote packet to flash, got code %d.\n",code);
  code = flash.writeULong(N_SAMPLES_ADDR,nSamples);
  Serial.printf("Wrote updated nSamples to flash, got code %d\n",code);

  Serial.printf("Have stored %ld samples so far...\n",nSamples);
  //flash.powerDown();
}

void exportData(){
  for(uint32_t n=0; n < nSamples; n++){
      uint32_t _packetAddress = 10 + sizeof(DataPacket) * n;
      DataPacket dp;
      flash.readAnything(_packetAddress,dp);
      Serial.printf("Timestamp: %ld | Altitude: %0.2f meters | Light Sensor: %ld\n",dp.timestamp, dp.altitude, dp.luminance);

  }
}


void takeMeasurement(){
  
  analogReadResolution(12);

  DataPacket dp;
  uint16_t lightSensorValue = analogRead(A0);
  DateTime dt = checkRTC();
  float altitude = checkAltimeter();

  dp.luminance = lightSensorValue;
  dp.timestamp = dt.unixtime();
  dp.altitude = altitude;
  char timeStr[40];
  sprintf(timeStr, "%d-%02d-%02dT%02d:%02d:%02d", dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second());
  Serial.printf("Timestamp: %s | Altitude: %0.2f meters | Light Sensor: %ld",timeStr, altitude, lightSensorValue);
  Serial.println();

  char dispStr[80];
  sprintf(dispStr,"Date:%d-%02d-%02d\nTime: %02d:%02d:%02d\nAltitude: %0.2fm\nLight Sensor: %ld", dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second(), altitude, lightSensorValue );
  displayStatus(dispStr);

  saveDataPoint(dp);

}

// the loop function runs over and over again forever
void loop() {
  takeMeasurement();
  delay(60*1000*2);

}
