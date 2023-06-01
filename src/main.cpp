#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h"
#include "ArduinoLowPower.h"
#include <Adafruit_BMP3XX.h>
#include <SparkFun_RV8803.h>
#include "Adafruit_VEML7700.h"
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <flash_devices.h>
#include <ArduinoJson.h>

enum PROGRAM_STATE {
  STATE_RECORDING,
  STATE_CONFIG_MODE
};

long configTime = 0;

volatile PROGRAM_STATE STATE = STATE_RECORDING;

void tud_mount_cb(void) {
    STATE = STATE_CONFIG_MODE;
    configTime = millis();
}

void tud_umount_cb(void) {
    STATE = STATE_RECORDING;
}
void tud_suspend_cb(void){
    STATE = STATE_RECORDING;
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
  float luminance;
  float altitude;
  float temperature;
};

struct CONFIG {
  uint16_t SAMPLE_PERIOD;
  float SEA_LEVEL_PRESSURE;
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
Adafruit_VEML7700 veml = Adafruit_VEML7700();
RV8803 rtc;
RTCZero rtc_internal;
bool RESET_TIME = 0;
volatile long lastDebounceTime = 0;
long debounceDelay = 50;
uint32_t SAMPLE_PERIOD = 10000;
uint32_t last_time_update = 0;

long configModeTimeoutCounter = 0;
CONFIG config;


#define DEBUG true
#define DEBUG_SERIAL if(DEBUG)Serial

void powerUp(){


}

void powerDown(){

}


void powerOnSelfTest(){
  
  Serial.println("Checking VEML7700....");
  Serial.printf("Current LUX: %0.2f\n",veml.readLux(VEML_LUX_AUTO));
  Serial.printf("Current Altitude is %0.2f meters\n",bmp.readAltitude(1020.20));
  Serial.printf("Current Temperature is %0.2fC\n",bmp.readTemperature());

  Serial.println("Checking RTC...");
  if(rtc.updateTime()){
    Serial.print("Current RTC Time is: ");
    Serial.println(rtc.stringTime8601());
  }
  else {
    Serial.println("RTC connection FAILED");
  }
  

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
  digitalWrite(LED_BUILTIN, LOW);

  // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
}

void dateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  rtc.updateTime();
  *date = FAT_DATE(rtc.getYear(), rtc.getMonth(), rtc.getDate());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
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
    rtc.setEpoch(now.unixtime(),false);
    delay(1000);
    fatfs.remove(TIME_FILE_NAME);
    fatfs.cacheClear();
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
  pinMode(5, OUTPUT); // initialize CS pin;

  if (!flash.begin(possible_devices)) {
    DEBUG_SERIAL.println("Error, failed to initialize flash chip!");
    while(1) delay(100);
  }
  DEBUG_SERIAL.print("Flash chip JEDEC ID: 0x"); 
  DEBUG_SERIAL.println(flash.getJEDECID(), HEX);

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


void initRTC(){
  Serial.println("Preparing to initialize RTC!");
  delay(1000);
  if (! rtc.begin()) {
    DEBUG_SERIAL.println("Couldn't find RTC!");
    // rtc_internal.begin();
  }
  else {
    DEBUG_SERIAL.println("Connected to RTC");
    rtc.updateTime();
    DEBUG_SERIAL.printf("RTC Time is %s",rtc.stringTime8601TZ());
  }
}

void initAltimeter(){
  DEBUG_SERIAL.println("Connecting to Altimeter...");
  if (!bmp.begin_I2C()) {
    DEBUG_SERIAL.println("Could not find sensor. Check wiring.");
  }
  else {
    // bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    // bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_0_05_HZ);  
  }

}

void initVEML(){
  DEBUG_SERIAL.println("Connecting to VEML7700...");
  if(veml.begin()){
    DEBUG_SERIAL.printf("Current LUX: %0.1f\n", veml.readLux(VEML_LUX_AUTO));
  }


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
  rtc.updateTime();
  Serial.printf("Sample Period: %d seconds\nSea Level Pressure: %0.2f\nCurrent Time: %s\n",config.SAMPLE_PERIOD,config.SEA_LEVEL_PRESSURE,rtc.stringTime8601TZ());
}


void setup() {

  pinMode(13, OUTPUT); // initialize digital pin 13 (LED) as an output.
  Serial.begin(9600);
  initFlash();

  for(int i=0; i < 10; i ++){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(300);              // wait for a 1/3 second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(300);              // wait for a 1/3 second
  }
  digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW


  Wire.begin();
  Serial.println("Starting up");


  #ifdef DEBUG_DISPLAY
  initDisplay(); // Only for Debugging Purposes
  #endif

  initRTC();
  initAltimeter();
  initVEML();
  Serial.println("Conducting Self-Test");

  powerOnSelfTest();
  Serial.println("Preparing to load configuration");
  loadConfig();
  // setup timer counter
  digitalWrite(13, HIGH);   // turn the LED OFF (HIGH is the voltage level)
  Serial.println("Setup complete!");


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
  if(rtc.updateTime()){
    return DateTime(rtc.getEpoch());
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
      dataFile.print("Timestamp,Altitude(m),Temperature,Light Sensor\r\n");
    }
    dataFile.printf("%s,%0.2f,%0.1f,%0.1f\r\n",DateTime(dp.timestamp).timestamp().c_str(), dp.altitude,dp.temperature, dp.luminance);

    dataFile.close();
    Serial.println("Wrote new measurement to data file!");
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }


}


void takeMeasurement(){

  uint32_t sampleStartTime = millis();

  DataPacket dp;
  DateTime dt = checkRTC();
  float altitude = checkAltimeter();
  float temperature = bmp.readTemperature();

  dp.luminance = veml.readLux(VEML_LUX_AUTO);
  dp.timestamp = dt.unixtime();
  dp.temperature = temperature;
  dp.altitude = altitude;

  //Send to Serial Port for Debugging
  char timeStr[40];
  sprintf(timeStr, "%d-%02d-%02dT%02d:%02d:%02d", dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second());
  DEBUG_SERIAL.printf("Timestamp: %s | Altitude: %0.2f meters | Light Sensor: %ld",timeStr, dp.altitude, dp.luminance);
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

// the loop function runs over and over again forever
void loop() {

  if (STATE == STATE_CONFIG_MODE) { 
    digitalWrite(13, LOW);    
    delay(300);
    digitalWrite(13, HIGH);  
    delay(300);  
    if((millis() - configTime) > CONFIG_MODE_TIMEOUT){
      STATE = STATE_RECORDING;
    }

  } else {
      digitalWrite(13, LOW);    
      takeMeasurement();
      digitalWrite(13, HIGH);    
      LowPower.deepSleep(config.SAMPLE_PERIOD*1000);
  }
  


}

