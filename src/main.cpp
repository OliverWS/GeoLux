#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "ArduinoLowPower.h"
#include <Adafruit_BMP3XX.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <flash_devices.h>
#include <ArduinoJson.h>

bool was_mounted = false;
void tud_umount_cb(void) {
    was_mounted  = true;
}

static const SPIFlash_Device_t possible_devices[] = {W25Q128JV_PM};
Adafruit_FlashTransport_SPI flashTransport(5, &SPI);
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;
Adafruit_USBD_MSC usb_msc;
bool fs_changed = false;
uint32_t timeSinceLastUpdate = 0;

// Configuration for the datalogging file:
#define FILE_NAME      "data.csv"
#define CONFIG_FILE_NAME      "config.json"
#define TIME_FILE_NAME "current_time.txt"


struct DataPacket {
  uint32_t timestamp;
  uint16_t luminance;
  float altitude;
};

struct CONFIG {
  uint16_t SAMPLE_PERIOD;
  float SEA_LEVEL_PRESSURE;
};

enum PROGRAM_STATE {
  STATE_RECORDING,
  STATE_CONFIG_MODE
};


uint32_t nSamples = 0;

#define N_SAMPLES_ADDRESS 0
#define CONFIG_DATA_ADDRESS (N_SAMPLES_ADDRESS + sizeof(nSamples) + 1)
#define CONFIG_MODE_TIMEOUT 120000

#define DEFAULT_SAMPLE_PERIOD 10000

// #define DEBUG_DISPLAY true
#ifdef DEBUG_DISPLAY

// This part just for the prototype
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//----------------------------------------------------------------
#endif

Adafruit_BMP3XX bmp;
RTC_DS3231 rtc;
RTCZero rtc_internal;
bool RESET_TIME = 0;
volatile long lastDebounceTime = 0;
long debounceDelay = 50;
uint32_t SAMPLE_PERIOD = 10000;
uint32_t last_time_update = 0;

long configModeTimeoutCounter = 0;
CONFIG config;

volatile PROGRAM_STATE STATE = STATE_RECORDING;

#define DEBUG true
#define DEBUG_SERIAL if(DEBUG)Serial

void powerUp(){


}

void powerDown(){

}




// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size) 
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{
  digitalWrite(LED_BUILTIN, HIGH);

  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(rtc_internal.getYear(), rtc_internal.getMonth(), rtc_internal.getDay());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(rtc_internal.getHours(), rtc_internal.getMinutes(), rtc_internal.getSeconds());
}

DateTime convert_fatfs_time(uint16_t fdate, uint16_t ftime) {
  uint16_t year;
  uint8_t month, day, hours, minutes, seconds;
  // The year is stored as an offset from 1980.
  year = (fdate >> 9) + 1980;

  // The month is stored as a value from 1 to 12.
  month = (fdate >> 5) & 0x0f;

  // The day is stored as a value from 1 to 31.
  day = fdate & 0x1f;

  // The hours are stored as a value from 0 to 23.
  hours = (ftime >> 11) & 0x1f;

  // The minutes are stored as a value from 0 to 59.
  minutes = (ftime >> 5) & 0x3f;

  // The seconds are stored as a value from 0 to 59.
  seconds = ftime & 0x1f;

  return DateTime(year,month,day,hours,minutes,seconds);
}



void loadConfig(){
  if(fatfs.exists(CONFIG_FILE_NAME)){
    DEBUG_SERIAL.println("Found existing configuration file!");
    StaticJsonDocument<JSON_OBJECT_SIZE(10)> doc;
    File32 configFile = fatfs.open(CONFIG_FILE_NAME);
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, configFile);
    if (error){
      Serial.println(F("Failed to read file, using default configuration"));
      Serial.printf("Error: %s\n",error.c_str());
    }
    else {
      Serial.println(F("Decoded config file: "));
      serializeJsonPretty(doc, Serial);
      Serial.println();

    }
    
    // Copy values from the JsonDocument to the Config
    config.SAMPLE_PERIOD = doc["SAMPLE_PERIOD"].as<uint16_t>();
    Serial.printf("Setting Sample Period: %d\n",doc["SAMPLE_PERIOD"].as<uint16_t>());
    config.SEA_LEVEL_PRESSURE = doc["SEA_LEVEL_PRESSURE"].as<float>();
    Serial.printf("Setting Pressure: %f\n",doc["SEA_LEVEL_PRESSURE"].as<float>());

    // Close the file (Curiously, File's destructor doesn't close the file)
    configFile.close();
    
  }
  else {
    DEBUG_SERIAL.println("Config file missing, creating!");

    config.SAMPLE_PERIOD = 10;
    config.SEA_LEVEL_PRESSURE = 1020.20;
    File32 configFile = fatfs.open(CONFIG_FILE_NAME,FILE_WRITE);
    const size_t CAPACITY = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<CAPACITY> doc;
    doc["SAMPLE_PERIOD"] = config.SAMPLE_PERIOD;
    doc["SEA_LEVEL_PRESSURE"] = config.SEA_LEVEL_PRESSURE;
    serializeJson(doc, configFile);
    configFile.close();
  }
//hard code for now
config.SAMPLE_PERIOD = 120;
config.SEA_LEVEL_PRESSURE = 1020.20;
}

void updateTimeFromFile(void){
  timeSinceLastUpdate = millis();
  if(fatfs.exists(TIME_FILE_NAME)){
    File32 conf_file = fatfs.open(TIME_FILE_NAME);
    uint16_t cdate; 
    uint16_t ctime;
    conf_file.getCreateDateTime(&cdate,&ctime);
    conf_file.close();
    DateTime now = convert_fatfs_time(cdate,ctime);
    DEBUG_SERIAL.printf("Updating time to: %s\n",now.timestamp().c_str());
    rtc_internal.setDate(now.day(),now.month(),(uint8_t) (now.year()-2000));
    rtc_internal.setTime(now.hour(),now.minute(),now.second());
    //rtc.adjust(now);
    delay(1000);
    fatfs.remove(TIME_FILE_NAME);
  }else {
    loadConfig();
  }
}
// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fs_changed = true;
  if((millis() - timeSinceLastUpdate) > 10000){
    updateTimeFromFile();
  }

  digitalWrite(LED_BUILTIN, LOW);
}

void initMassStorage(){
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "External Flash", "1.0");

  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  
  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);

  usb_msc.begin();

}


void initFlash(){

  if (!flash.begin(possible_devices)) {
    DEBUG_SERIAL.println("Error, failed to initialize flash chip!");
    while(1) delay(100);
  }
  DEBUG_SERIAL.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  while (!fatfs.begin(&flash)) {
    DEBUG_SERIAL.println("Error, failed to mount newly formatted filesystem!");
    DEBUG_SERIAL.println("Was the flash chip formatted with the fatfs_format example?");
    delay(1000);
  }
  DEBUG_SERIAL.println("Mounted filesystem!");
  initMassStorage();
  
}


void button_pressed(){
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if((millis() - lastDebounceTime) < 1000){
      if( STATE == STATE_RECORDING){
        Serial.println("Button double clicked: Switching to config mode");
        STATE = STATE_CONFIG_MODE;
        configModeTimeoutCounter = millis();
      }
      else {
        Serial.println("Button double clicked: Switching to RECORDING mode");
        STATE = STATE_RECORDING;
      }
    }
    else {
      Serial.println("Button pressed!");
    }
    
  }
  lastDebounceTime = millis();
}

void initRTC(){

  if (! rtc.begin()) {
    DEBUG_SERIAL.println("Couldn't find RTC, using internal");
    rtc_internal.begin();
  }
  else {
    rtc.disable32K();
    rtc.writeSqwPinMode(DS3231_OFF);
  }

}

void initAltimeter(){

  if (!bmp.begin_I2C()) {
    DEBUG_SERIAL.println("Could not find sensor. Check wiring.");
    while(1);
  }
  // bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  // bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_0_05_HZ);  

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

void printConfig(){
  Serial.printf("Sample Period: %d seconds\nSea Level Pressure: %0.2f\nCurrent Time: %s\n",config.SAMPLE_PERIOD,config.SEA_LEVEL_PRESSURE,rtc.now().timestamp().c_str());
}

void saveConfig(CONFIG config){
  //Do nothing
}


void setup() {
  pinMode(13, OUTPUT); // initialize digital pin 13 (LED) as an output.
  initFlash();
  #if DEBUG == true
  Serial.begin(9600);
  delay(3000);
  #endif 
  for(int i=0; i < 10; i ++){
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
  

  pinMode(6, INPUT_PULLUP);
  loadConfig();

  attachInterrupt(digitalPinToInterrupt(6), button_pressed, FALLING);
  LowPower.attachInterruptWakeup(6, button_pressed, FALLING);

  // setup timer counter
  for (size_t i = 0; i < 5; i++)
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(150);              // wait for a 1/3 second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(150);              // wait for a 1/3 second
  }
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)


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
  if(rtc.begin()){
    return rtc.now();
  }
  else {
    return DateTime(rtc_internal.getEpoch());
  }
}


float checkAltimeter(){
  float altitude = bmp.readAltitude(config.SEA_LEVEL_PRESSURE);
  return altitude;
}


void saveDataPoint(DataPacket dp){
  bool initializeFile = !fatfs.exists(FILE_NAME);
  FsDateTime::setCallback(&dateTime);
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
    if(initializeFile){
      dataFile.print("Timestamp,Altitude(m),Light Sensor\r\n");
    }
    dataFile.printf("%s,%0.2f,%ld\r\n",DateTime(dp.timestamp).timestamp().c_str(), dp.altitude, dp.luminance);

    dataFile.close();
    Serial.println("Wrote new measurement to data file!");
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }


}

void exportData(){
  // flash.powerUp();
  // nSamples = flash.readULong(N_SAMPLES_ADDRESS);  

  // Serial.printf("Timestamp,Altitude(m),Light Sensor\r\n");
  // for(uint32_t n=0; n < nSamples; n++){
  //     uint32_t _packetAddress = 4096 + sizeof(DataPacket) * n;
  //     DataPacket dp;
  //     flash.readAnything(_packetAddress,dp);
  //     Serial.printf("%s,%0.2f,%ld\r\n",DateTime(dp.timestamp).timestamp().c_str(), dp.altitude, dp.luminance);
  // }
  // flash.powerDown();
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
  DateTime now = DateTime(isoTime);
  rtc.adjust(DateTime(isoTime));
  rtc_internal.setDate(now.day(),now.month(),now.year());
  rtc_internal.setTime(now.hour(),now.minute(),now.second());

  delay(500);
  Serial.printf("\nCurrent Time (external) is now: %s\n", rtc.now().timestamp().c_str());
  Serial.printf("\nCurrent Time (internal) is now: %d-%02d-%02dT%02d:%02d:%02d\n", rtc_internal.getYear(),rtc_internal.getMonth(),rtc_internal.getDay(),rtc_internal.getHours(),rtc_internal.getMinutes(),rtc_internal.getSeconds());

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

void eraseData(){
  // flash.powerUp();
  // nSamples = 0;
  // flash.eraseSection(0,sizeof(nSamples) + sizeof(CONFIG));
  // if(flash.writeULong(N_SAMPLES_ADDRESS,nSamples) && flash.writeAnything(CONFIG_DATA_ADDRESS,config)){
  //   DEBUG_SERIAL.printf("Wrote updated nSamples %ld to Flash\n",nSamples);
  // }
  // flash.powerDown();

}

// the loop function runs over and over again forever
void loop() {

  if (tud_ready()) { 
    digitalWrite(13, LOW);    
  } else {
      digitalWrite(13, LOW);    
      takeMeasurement();
      digitalWrite(13, HIGH);    
      LowPower.deepSleep(config.SAMPLE_PERIOD*1000);
  }



}

