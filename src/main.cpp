#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "ArduinoLowPower.h"
#include <Adafruit_BMP085.h>
#include <SPIMemory.h>

struct DataPacket {
  uint32_t timestamp;
  uint16_t luminance;
  float altitude;
};

struct CONFIG {
  uint32_t SAMPLE_PERIOD;
  float SEA_LEVEL_PRESSURE;
};

uint32_t nSamples = 0;

#define N_SAMPLES_ADDRESS 0
#define CONFIG_DATA_ADDRESS (N_SAMPLES_ADDRESS + sizeof(nSamples) + 1)
#define CONFIG_MODE_TIMEOUT 120000

#define DEFAULT_SAMPLE_PERIOD 10000

#define DEBUG_DISPLAY true
#ifdef DEBUG_DISPLAY

// This part just for the prototype
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

enum PROGRAM_STATE {
  STATE_RECORDING,
  STATE_CONFIG_MODE
};

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//----------------------------------------------------------------
#endif

SPIFlash flash(A2, &SPI); // Our Flash is on a different SPI bus
Adafruit_BMP085 bmp;
RTC_DS3231 rtc;
bool RESET_TIME = 0;
volatile long lastDebounceTime = 0;
long debounceDelay = 50;
uint32_t SAMPLE_PERIOD = 10000;

long configModeTimeoutCounter = 0;
CONFIG config;

volatile PROGRAM_STATE STATE = STATE_RECORDING;

#define DEBUG true
#define DEBUG_SERIAL if(DEBUG)Serial



void initFlash(){
  DEBUG_SERIAL.println("Initializing SPI Flash");
  SPI.begin();
  flash.setClock(1000000);
  flash.powerUp();
  uint8_t err = flash.begin(MB(128));
  DEBUG_SERIAL.printf("Initialized Flash with return code: %02x\n",err);
  uint32_t JEDEC = flash.getJEDECID();
  DEBUG_SERIAL.printf("JEDEC_ID: %04x\n",JEDEC);
  DEBUG_SERIAL.print("Capacity: ");
  DEBUG_SERIAL.println(flash.getCapacity());
  DEBUG_SERIAL.print("Unique ID: ");
  DEBUG_SERIAL.println(flash.getUniqueID());
  DEBUG_SERIAL.print("Man ID: ");
  DEBUG_SERIAL.println(flash.getManID());

}


void button_pressed(){
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    if((millis() - lastDebounceTime) < 1000){
      Serial.println("Button double clicked: Switching to config mode");
      STATE = STATE_CONFIG_MODE;
      configModeTimeoutCounter = millis();
    }
    else {
      Serial.println("Button pressed!");
    }
    
  }
  lastDebounceTime = millis();
}

void initRTC(){

  if (! rtc.begin()) {
    DEBUG_SERIAL.println("Couldn't find RTC");
    DEBUG_SERIAL.flush();
    while (1) delay(10);
  }

  rtc.disable32K();
  rtc.writeSqwPinMode(DS3231_OFF);
}

void initAltimeter(){

  if (!bmp.begin()) {
    DEBUG_SERIAL.println("Could not find sensor. Check wiring.");
    while(1);
  }

  DEBUG_SERIAL.println("Setting mode to altimeter.");
  

}
#ifdef DEBUG_DISPLAY
void initDisplay(){

  //Only for Prototyping
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    DEBUG_SERIAL.println("SSD1306 allocation failed");
  }
  else {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0, 0);
      display.println("P123 Geolocator v0.2");
      display.display();

  }
}
#endif

void loadConfig(){
  flash.powerUp();
  flash.readAnything(CONFIG_DATA_ADDRESS,config);
  if(config.SAMPLE_PERIOD == NAN || config.SAMPLE_PERIOD < 100){
    config.SAMPLE_PERIOD = DEFAULT_SAMPLE_PERIOD;
  }

  if(config.SEA_LEVEL_PRESSURE == NAN){
    config.SEA_LEVEL_PRESSURE = 1020.20;
  }
  flash.powerDown();
}

void saveConfig(CONFIG config){
    flash.powerUp();
    nSamples = flash.readULong(N_SAMPLES_ADDRESS);
    flash.eraseSection(0,sizeof(nSamples) + sizeof(CONFIG));
    if(flash.writeULong(N_SAMPLES_ADDRESS,nSamples) && flash.writeAnything(CONFIG_DATA_ADDRESS,config)){
      DEBUG_SERIAL.println("Configuration saved to flash succesfully");
    }
    else {
      DEBUG_SERIAL.println("ERROR: Saving configuration to flash failed!");
    }
    flash.powerDown();
}


void setup() {
  pinMode(13, OUTPUT); // initialize digital pin 13 (LED) as an output.
  #if DEBUG == true
  Serial.begin(9600);
  #endif 
  for(int i=0; i < 5; i ++){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);              // wait for a 1/3 second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(300);              // wait for a 1/3 second
  }
  #ifdef DEBUG_DISPLAY
  initDisplay(); // Only for Debugging Purposes
  #endif
  initRTC();
  initAltimeter();
  initFlash();

  pinMode(6, INPUT_PULLUP);
  loadConfig();

  attachInterrupt(digitalPinToInterrupt(6), button_pressed, FALLING);
  LowPower.attachInterruptWakeup(6, button_pressed, FALLING);

}
#ifdef DEBUG_DISPLAY
void displayStatus( char *statusStr){
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  display.println("P123 Geolocator v0.2");
  display.setCursor(0, 20);
  // Display static text

  display.println(statusStr);
  display.display(); 

}
#endif
DateTime checkRTC(){
  return rtc.now();
}


float checkAltimeter(){
  float altitude = bmp.readAltitude(config.SEA_LEVEL_PRESSURE*100);
  return altitude;
}


void saveDataPoint(DataPacket packet){
  flash.powerUp();
  nSamples = flash.readULong(N_SAMPLES_ADDRESS) + 1;

  uint32_t _packetAddress = 4096 + sizeof(DataPacket) * nSamples;
  while(!flash.writeAnything(_packetAddress,packet)){ 
    DEBUG_SERIAL.printf("Erasing sector: %ld\n",_packetAddress);
    flash.eraseSection(_packetAddress,sizeof(DataPacket));
  }
  DEBUG_SERIAL.printf("Have stored %ld samples so far...\n",nSamples);
  flash.eraseSection(0,sizeof(nSamples) + sizeof(CONFIG));
  if(flash.writeULong(N_SAMPLES_ADDRESS,nSamples) && flash.writeAnything(CONFIG_DATA_ADDRESS,config)){
    DEBUG_SERIAL.printf("Wrote updated nSamples %ld to Flash\n",nSamples);
  }
  flash.powerDown();

}

void exportData(){
  flash.powerUp();
  nSamples = flash.readULong(N_SAMPLES_ADDRESS);  

  Serial.printf("Timestamp,Altitude(m),Light Sensor\r\n");
  for(uint32_t n=0; n < nSamples; n++){
      uint32_t _packetAddress = 4096 + sizeof(DataPacket) * n;
      DataPacket dp;
      flash.readAnything(_packetAddress,dp);
      Serial.printf("%s,%0.2f,%ld\r\n",DateTime(dp.timestamp).timestamp().c_str(), dp.altitude, dp.luminance);
  }
  flash.powerDown();
}


void takeMeasurement(){
  uint32_t sampleStartTime = millis();
  analogReadResolution(12);

  DataPacket dp;
  uint16_t lightSensorValue = analogRead(A0);
  DateTime dt = checkRTC();
  float altitude = checkAltimeter();

  dp.luminance = lightSensorValue;
  dp.timestamp = dt.unixtime();
  dp.altitude = altitude;

  //Send to Serial Port for Debugging
  char timeStr[40];
  sprintf(timeStr, "%d-%02d-%02dT%02d:%02d:%02d", dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second());
  DEBUG_SERIAL.printf("Timestamp: %s | Altitude: %0.2f meters | Light Sensor: %ld",timeStr, altitude, lightSensorValue);
  DEBUG_SERIAL.println();

  #ifdef DEBUG_DISPLAY
  //Send to display
  char dispStr[80];
  sprintf(dispStr,"Date:%d-%02d-%02d\nTime: %02d:%02d:%02d\nAltitude: %0.2fm\nLight Sensor: %ld", dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second(), altitude, lightSensorValue );
  displayStatus(dispStr);
  #endif
  
  saveDataPoint(dp);
  DEBUG_SERIAL.printf("Collection and saving of sample took %dms\n",(millis()-sampleStartTime));
}


void setConfigTime(){
  Serial.println("Enter current time as an ISO8601 string (e.g. 2023-04-30T13:21:00):");
  String isoTimeStr = Serial.readStringUntil('\n');
  const char *isoTime = isoTimeStr.c_str();
  rtc.adjust(DateTime(isoTime));
  delay(500);
  Serial.printf("\nCurrent Time is now: %s\n", rtc.now().timestamp().c_str());
}

void setSampleInterval(){
    Serial.println("Enter new sample period in seconds:");
    String sampleRateStr = Serial.readStringUntil('\n');
    uint32_t newSampleRate = atoi(sampleRateStr.c_str());
    if(newSampleRate > 0){
      config.SAMPLE_PERIOD = newSampleRate*1000;
      Serial.printf("Sample period has been set to: %d\n",config.SAMPLE_PERIOD);
    }
    else {
      Serial.printf("Invalid sample period: %d seconds\n",newSampleRate);
    }
    saveConfig(config);

}

void setSeaLevelPressure(){
    Serial.println("Enter new Sea Level Pressure in hPa (e.g. 1020.20):");
    String seaLevelPressure = Serial.readStringUntil('\n');
    float updatedPressure = atof(seaLevelPressure.c_str());
    if(updatedPressure > 800.0){
      config.SEA_LEVEL_PRESSURE = updatedPressure;
      Serial.printf("Sea Level Pressure has been set to: %0.2f; Current Alititude: %0.2f m\n",config.SEA_LEVEL_PRESSURE,checkAltimeter());
    }
    else {
      Serial.printf("Invalid Sea Level Pressure: %f hPa\n",updatedPressure);
    }
    saveConfig(config);

}


void printConfig(){
  Serial.printf("Sample Period: %d seconds\nSea Level Pressure: %0.2f\nCurrent Time: %s\n",config.SAMPLE_PERIOD/1000,config.SEA_LEVEL_PRESSURE,rtc.now().timestamp());
}

void printConfigHelp(){
  Serial.println("Available Commands:");
  for(int i=0; i < 80; i++) Serial.print("-");
  Serial.println("SET_TIME: Set time on the real time clock");
  Serial.println("SET_SAMPLE_INTERVAL: Set the sample interval in seconds");
  Serial.println("SET_SEA_LEVEL_PRESSURE: Set the sea level pressure in hPA");
  Serial.println("EXPORT_DATA: Print saved data to Serial console in CSV format");
  Serial.println("PRINT_CONFIG: Display current configuration");
  Serial.println("HELP: Display this message");
  Serial.println("EXIT: Resume Sampling");
  Serial.println();
}

// the loop function runs over and over again forever
void loop() {
  if(STATE == STATE_CONFIG_MODE){
    Serial.setTimeout(CONFIG_MODE_TIMEOUT);
    String cmdStr = Serial.readStringUntil('\n');
    if(cmdStr.length() > 0){
      configModeTimeoutCounter = millis();
    }
    if(cmdStr == "SET_TIME"){
      setConfigTime();
    }
    else if(cmdStr == "SET_SAMPLE_INTERVAL"){
      setSampleInterval();
    }
    else if (cmdStr == "SET_SEA_LEVEL_PRESSURE"){
      setSeaLevelPressure();
    }
    else if(cmdStr == "EXPORT_DATA"){
      exportData();
    }
    else if(cmdStr == "PRINT_CONFIG"){
      printConfig();
    }
    else if(cmdStr == "EXIT"){
      STATE = STATE_RECORDING;
    }
    else {
      printConfigHelp();
    }


    if((millis() - configModeTimeoutCounter) > CONFIG_MODE_TIMEOUT){
      STATE = STATE_RECORDING;
    }
  }
  else {
    takeMeasurement();
    LowPower.deepSleep(config.SAMPLE_PERIOD);
  }

}
