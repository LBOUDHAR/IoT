/**************************************************
  COPYRIGHT (c) Lies BOUDHAR 1982-2022
                           
  Project name: IoT in Agriculture
  Date: 20/01/2022
  Version: 1.0
  Circuit: 
  CODE NAME: LiesB_IoT012022_00A
  Coder: Lies BOUDHAR
  programmer: Lies BOUDHAR
  maintainer: Lies BOUDHAR
  Revision: 12/07/2022 à 17:27
  Microcontroller: ESP32 dev module OR ESP8266 generic module
  instagram: soko.009
  youtube: lies boudhar
**************************************************/

/* For SH1106_128X64, BME280, TSL2561
   Connect Vin to 3-5VDC
   Connect GND to ground
   Connect SCL to I2C clock pin (GPIO22 on ESP32. GPIO05(D1) on ESP8266)
   Connect SDA to I2C data pin  (GPIO21 on ESP32. GPIO04(D2) on ESP8266) */

/*
 * Project Description:
 * this project use ESP32 or ESP8266, SH1106_128X64, TinyRTC (DS1307), nRF24l01,
 * sensors(18B20, BMP280, capacitve soil moisture, NPK, soil PH, TSL2561, 
 * 
 * data are sent to the Cloud (ThingSpeak), to be stored and retrieved remotely 
 * by any suitable application. then the data can be processed to predict things, 
 * or for direct control of the system. 
 * 
 */

#include <SoftwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TSL2561.h>
#include <ThingSpeak.h>
#include <Ticker.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #define ONE_WIRE_BUS 13      // Data wire is plugged into digital GPIO13 (D7)
#elif defined(ESP32)
  #include <WiFi.h>
  #define ONE_WIRE_BUS 17    // Data wire is plugged into digital pin27 GPIO17
#else
#error "It's not an ESP8266 or ESP32"
#endif

String apiWritekey_1 = "UVWCEHWBHG868AG0";    // ThingSpeak  API key 1 for write Channel ID: 1745413
String apiWritekey_2 = "HD84DB1KX269FBVE";    // ThingSpeak  API key 2 for write Channel ID: 1810281
String apiReadkey_1 = "WG1I7OU3GXMZXQN0";    // ThingSpeak  API key 1 for read 
String apiReadkey_2 = "RKUL1VDHY951311F";    // ThingSpeak  API key 1 for read 
//const char* ssid = "POCO X3 Pro";             // wifi SSID name
//const char* password = "lieslies1408" ;    // wifi password
//const char* ssid = "Crysis 3_EXT";             // wifi SSID name
//const char* password = "celiawassimsyrine" ;    // wifi password
const char* ssid = "ENSA2022";             // wifi SSID name
const char* password = "lsb2022ensa" ;    // wifi password

const char* server = "api.thingspeak.com";
WiFiClient client;

/* 
 * Channel ID: 1745413
 * Field1: Air Temp, Field2: Air Humidity, Field3: Pressure, Field4: Soil Moisture, Field5: Soil Temperature, Field6: PH
 * Field7: Conductivity, Field8: Luminosity.
 * Channel ID: 1810281
 * Field1: Nitrogen, Field2: Phosphorus, Field3: Potassium
 */

// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
TSL2561 tsl(TSL2561_ADDR_FLOAT); 

Adafruit_BME280 bme;

OneWire oneWire(ONE_WIRE_BUS);  // Create a oneWire instance to communicate with any OneWire device
DallasTemperature sensors(&oneWire);  // Pass oneWire reference to DallasTemperature library
DeviceAddress DS18B20[4];               // Create an array for DS18B20 sensor addresses

int NumOfDevices = 0;               // Number of dallas temperature devices found
float T_18B20[4];

String mlxStatus = "MLX90614 faound";
String bmeStatus = "BME280 faound";
String tslStatus = "TSL2561 faound";
float datasBME[4];      // Temperature, Humidity, Pressure, Altitude;
uint16_t datasTSL[4];   //lum, ir, full, lux;
float dataSOIL[7];      // N, P, K, Moisture, T, PH, EC;

int count = 0;
Ticker periodicTicker;

#define RE 2              // GPIO2 (D4)
#define DE 0              // GPIO0 (D3)

const byte rxPin = 14;    // GPIO14 (D5)
const byte txPin = 12;    // GPIO12 (D6)

const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

const byte moist[]  = {0x01,0x03,0x00,0x12,0x00,0x01,0x24,0x0f};
const byte cond[] = {0x01,0x03, 0x00, 0x15, 0x00, 0x01, 0x95, 0xce};
const byte ph[] = {0x01,0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0b};
 
byte values[11];
SoftwareSerial mod(rxPin,txPin);



U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_R0 ou U8G2_R2: mode paysage, U8G2_R1 ou U8G2_R3: mode portrait

/* definition of a bitmap image of the logo of the name Lies BOUDHAR */
static const unsigned char logoBitmap[] PROGMEM = {
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFC,0X03,0XC0,0X7F,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XFF,0X1F,0XF8,0XFF,0X03,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0XF8,0X3F,0XF0,0XFF,0XFF,0XFF,0XFF,0X1F,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFD,0X03,0XFC,0X7F,0X80,0X3F,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0XE0,0XFF,0XFF,0X7F,0X00,0XE0,0X07,0X00,0XFC,0X00,0X00,
0X00,0X00,0X00,0XF8,0X7F,0XF0,0X07,0XE0,0X1F,0X00,0XE0,0X07,0X00,0XF0,0X01,0X00,
0X00,0X00,0X80,0XFF,0XFF,0XFF,0X01,0X00,0X1F,0X00,0XE0,0X07,0X00,0XE0,0X01,0X00,
0X00,0X00,0XE0,0XFF,0XFF,0X3F,0X00,0X80,0X1F,0X00,0X00,0X00,0X00,0XC0,0X0F,0X00,
0X00,0X00,0XF8,0X0F,0XC0,0X7F,0X00,0X00,0X01,0X00,0X00,0X00,0X00,0X80,0X3F,0X00,
0X00,0X00,0XFE,0X00,0X00,0X7E,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0X00,
0X00,0X00,0X1F,0X00,0X00,0X60,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XFB,0X03,
0X00,0X80,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0X07,
0X00,0XC0,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0X0F,
0X00,0XC0,0XE1,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0E,
0X00,0XE0,0XC1,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X1E,
0X00,0XE0,0XC0,0X01,0X00,0X06,0X00,0X00,0X00,0X00,0XC0,0X3F,0X00,0X00,0X00,0X1C,
0X00,0XE0,0XC0,0X00,0X00,0X03,0X00,0X00,0X00,0X00,0XF8,0XFF,0X00,0X00,0X00,0X1C,
0X00,0XE0,0XE0,0X00,0XE0,0X01,0X00,0X00,0X00,0X00,0X7C,0XF0,0X00,0X00,0X00,0X1C,
0X00,0XFF,0XE0,0X00,0XE0,0X00,0X00,0X00,0X0E,0X00,0X1E,0XE0,0X00,0X00,0X00,0X1C,
0XC0,0XFF,0XE0,0X00,0X00,0X00,0XF8,0X01,0X1F,0X00,0XE7,0XE0,0X00,0X00,0X00,0X1E,
0XF0,0XFF,0XE0,0X00,0X00,0X00,0X9E,0X83,0X0F,0X00,0XE3,0X70,0X00,0X00,0X00,0X1F,
0XFC,0XC1,0X60,0X00,0X70,0X00,0X87,0X83,0X07,0X0C,0X60,0X38,0X00,0X00,0X80,0X3F,
0X3E,0X00,0X70,0X00,0X70,0X80,0X83,0XE3,0X03,0X06,0X70,0X1F,0X00,0X00,0XE0,0X3F,
0X1E,0X00,0X70,0X00,0X70,0XC0,0XC3,0X61,0X07,0X03,0XF0,0X7F,0X00,0X00,0XE0,0X79,
0X0F,0X00,0X70,0X00,0X70,0XE0,0XE1,0X70,0X8F,0X01,0X30,0XF0,0X00,0X00,0XE0,0X70,
0X07,0X00,0X30,0X00,0X70,0XF0,0X7D,0X38,0XEE,0X00,0X38,0XC0,0X01,0X00,0X00,0XE0,
0X07,0X00,0X30,0X00,0X38,0XF8,0X07,0X1C,0X7E,0X00,0X38,0XC0,0X01,0X00,0X00,0XE0,
0X07,0X00,0X38,0X00,0X38,0XFC,0X01,0X0E,0X3E,0X00,0X38,0XC0,0X01,0X00,0X00,0XE0,
0X0E,0X00,0X38,0XC0,0X38,0XDE,0X81,0X07,0X3F,0X00,0X18,0XE0,0X79,0X00,0X00,0XE0,
0X1E,0X00,0X78,0X70,0X38,0XCF,0XE3,0XC3,0X39,0X1E,0X1C,0XF0,0XF8,0X01,0X00,0X70,
0XFC,0X00,0XF8,0X7F,0XF8,0X87,0XFF,0XE0,0X38,0X1E,0X1C,0X78,0XF0,0X07,0X00,0X70,
0XF8,0X0F,0XF0,0X0F,0XF0,0X01,0X3F,0XE0,0X1C,0X1E,0XFC,0X1F,0XC0,0X0F,0X00,0X78,
0XF0,0X7F,0X00,0X00,0X00,0X00,0X00,0XE0,0X1F,0X00,0XF8,0X03,0X00,0X1F,0X00,0X3C,
0XE0,0XFF,0X00,0X00,0X00,0X00,0X00,0XC0,0X07,0X00,0X00,0X00,0X00,0X3C,0X00,0X1F,
0XE0,0X71,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X78,0XC0,0X0F,
0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X70,0XFC,0X07,
0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0XFF,0X01,
0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0X3F,0X00,
0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0X03,0X00,
0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X70,0X00,0X00,
0XE0,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X78,0X00,0X00,
0XE0,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X38,0X00,0X00,
0XC0,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X60,0X00,0X00,0X3C,0X00,0X00,
0X00,0XFF,0XFF,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X70,0X00,0X00,0X1F,0X00,0X00,
0X00,0XFE,0XFF,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X70,0X00,0XE0,0X0F,0X00,0X00,
0X00,0XF0,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XF0,0X01,0XFE,0X03,0X00,0X00,
0X00,0X00,0X1F,0X00,0X00,0X80,0X03,0X00,0X00,0X00,0XF8,0XFF,0XFF,0X00,0X00,0X00,
0X00,0X00,0XFE,0X00,0X00,0X80,0X03,0X00,0X00,0X00,0XFC,0XFF,0X1F,0X00,0X00,0X00,
0X00,0X00,0XF8,0X07,0X00,0XF0,0X07,0X00,0X00,0X00,0X3E,0XFC,0X00,0X00,0X00,0X00,
0X00,0X00,0XE0,0XFF,0XFF,0XFF,0X1F,0X00,0X00,0X00,0X1F,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XF0,0XFF,0XFF,0XFF,0X7E,0X00,0X00,0XC0,0X0F,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XF8,0XFF,0XFF,0X1F,0XF8,0X07,0X00,0XF8,0X03,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X7C,0X00,0X1F,0X00,0XE0,0XFF,0XFF,0XFF,0X01,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X1C,0X00,0X1C,0X00,0XC0,0XFF,0XFF,0X3F,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X1C,0X00,0X1C,0X00,0X00,0XFC,0XFF,0X07,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X7E,0X00,0X1F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X80,0XFF,0XFF,0X0F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XC0,0XFF,0XFF,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XF8,0XC1,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XFE,0XFF,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XFE,0XFF,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XFE,0X7F,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XFE,0X07,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XF8,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};

void periodicUpload2TS(){
  count++;
}

void setup() {
  mod.begin(9600);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  // put RS-485 into receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
 
  u8g2.begin();
  //u8g2.enableUTF8Print(); 
  
  u8g2.clearBuffer(); 
  u8g2.drawXBMP(0, 0, 128, 64, logoBitmap); 
  u8g2.sendBuffer();  
  delay(3000); 
  
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_7x13B_tf); 
  u8g2.setCursor(2, 10); 
  u8g2.print("IoT in Agriculture"); 
  u8g2.setCursor(2, 22);
  u8g2.print("ENSA 2022");
  u8g2.setCursor(2, 34);
  u8g2.print("ALGIERS 09/2022");
  u8g2.setCursor(2, 46);
  u8g2.print("ESP8266 ThingSpeak");  
  u8g2.setCursor(2, 58);
  u8g2.print("Version 2.0");    
  u8g2.sendBuffer();  
  delay(3000); 

  Wire.begin();
  
  Init_DS18B20(9);      // Initialize DS18B20 temperature sensors with precision set to 9

  if(!bme.begin(0x76)){
    bmeStatus = "BME280 not faound"; 
  }

  if (!tsl.begin()) {
    tslStatus = "TSL not faound";
  }
    
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)

  u8g2.clearBuffer();  
  u8g2.setCursor(2, 10); 
  u8g2.print(bmeStatus);
  u8g2.setCursor(2, 22); 
  u8g2.print(NumOfDevices);
  u8g2.print(" DS18B20 faound");
  u8g2.setCursor(2, 34); 
  u8g2.print(tslStatus);
  u8g2.setCursor(2, 46); 
  u8g2.print(mlxStatus);  
  u8g2.sendBuffer();  
  delay(2000);  
  u8g2.clearBuffer();
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("conecting");
  u8g2.setCursor(0,22);
  u8g2.sendBuffer();
    
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    u8g2.print(".");
    u8g2.sendBuffer();
  }  

  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("WiFi connected to:");
  u8g2.setCursor(0,22);
  u8g2.print(ssid);
  u8g2.setCursor(0,36);
  u8g2.print("IP: "); 
  u8g2.setCursor(20,48);
  u8g2.print(WiFi.localIP()); 
  u8g2.sendBuffer();
  delay(3000);
  
  periodicTicker.attach(60, periodicUpload2TS);
}

void loop() {
  dataSOIL[0] = read_N();
  delay(250);
  dataSOIL[1] = read_P();
  delay(250);
  dataSOIL[2] = read_K();
  delay(250);  
  dataSOIL[5] = read_PH();
  delay(250);
  dataSOIL[3] = read_M();
  delay(250);
  dataSOIL[6] = read_C();
/*  delay(250);
  dataSOIL[4] = read_T();
  delay(250);*/

  read_bme_data();
  read_dallas_data();
  read_TSL2561_data();
  
  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("== Air ==");  
  u8g2.setCursor(2, 22);
  u8g2.print("Temp: ");
  u8g2.print(datasBME[0]);
  u8g2.print(" C"); 
  u8g2.setCursor(2, 34);
  u8g2.print("Humidity: ");
  u8g2.print(datasBME[1]);
  u8g2.print(" %");
  u8g2.setCursor(2, 46);
  u8g2.print("Pressure: ");
  u8g2.print(datasBME[2]);
  u8g2.print(" bar");
  u8g2.sendBuffer();
  delay(2000);

  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("== Light ==");  
  u8g2.setCursor(2, 22);
  u8g2.print("Visible: ");
  u8g2.print(datasTSL[0]); 
  u8g2.setCursor(2, 34);
  u8g2.print("InfraRed: ");
  u8g2.print(datasTSL[2]);
  u8g2.setCursor(2, 46);
  u8g2.print("FullSpectrum: ");
  u8g2.print(datasTSL[1]); 
  u8g2.setCursor(2, 58);
  u8g2.print("lux: ");
  u8g2.print(datasTSL[3]);      
  u8g2.sendBuffer();
  delay(2000); 

  u8g2.clearBuffer();
  u8g2.setCursor(2, 10);
  u8g2.print("= Soil == ");   
  u8g2.setCursor(2, 22);
  u8g2.print("N: ");
  u8g2.print(dataSOIL[0]);
  u8g2.print(" mg/kg");
  u8g2.setCursor(2, 34);
  u8g2.print("P: ");
  u8g2.print(dataSOIL[1]);
  u8g2.print(" mg/kg");  
  u8g2.setCursor(2, 46);
  u8g2.print("K: ");
  u8g2.print(dataSOIL[2]);
  u8g2.print(" mg/kg");
  u8g2.sendBuffer(); 
  delay(2000);  

  u8g2.clearBuffer(); 
  u8g2.setCursor(2, 10);
  u8g2.print("= Soil == ");  
  u8g2.setCursor(2, 22);
  u8g2.print("PH: ");
  u8g2.print(dataSOIL[5]);
  u8g2.setCursor(2, 34);
  u8g2.print("Moist: ");
  u8g2.print(dataSOIL[3]);
  u8g2.print(" %");  
  u8g2.setCursor(2, 46);
  u8g2.print("EC: ");
  u8g2.print(dataSOIL[6]);
  u8g2.print(" us/cm");
  u8g2.setCursor(2, 58);
  u8g2.print("Temp: ");
  u8g2.print(dataSOIL[4]);
  u8g2.print(" C");
  u8g2.sendBuffer(); 
  delay(2000);

  if(count > 1){
    u8g2.clearBuffer();
    u8g2.setCursor(2, 10);
    u8g2.print("Upload to"); 
    u8g2.setCursor(2, 22);
    u8g2.print("Thingspeak");   
    u8g2.sendBuffer();
    upload2TS_key1(apiWritekey_1, datasBME, dataSOIL, datasTSL);
    upload2TS_key2(apiWritekey_2, dataSOIL);
    count = 0;
  } 
}

void read_bme_data(){
  datasBME[0] = bme.readTemperature();
  datasBME[1] = bme.readHumidity();
  datasBME[2] = bme.readPressure() / 100.0F;
  //datasBME[3] = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void read_TSL2561_data(){
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 13-402 milliseconds! Uncomment whichever of the following you want to read
  datasTSL[0] = tsl.getLuminosity(TSL2561_VISIBLE);         //lum
  datasTSL[1] = tsl.getLuminosity(TSL2561_FULLSPECTRUM);    //full
  datasTSL[2] = tsl.getLuminosity(TSL2561_INFRARED);        //ir

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  //lum = tsl.getFullLuminosity();
  //ir = lum >> 16;
  //full = lum & 0xFFFF;
  datasTSL[3] = tsl.calculateLux(datasTSL[1], datasTSL[2]);                 //lux
}

void Init_DS18B20(int precision){
  sensors.begin();
  NumOfDevices = sensors.getDeviceCount();
  for(int x = 0; x!= NumOfDevices; x++){
    if(sensors.getAddress(DS18B20[x], x)){
      sensors.setResolution(DS18B20[x], precision);
    }
  }
}

void read_dallas_data(){
    sensors.requestTemperatures();
    T_18B20[0] = sensors.getTempC(DS18B20[0]);

    sensors.requestTemperatures();
    T_18B20[1] = sensors.getTempC(DS18B20[1]);

    //sensors.requestTemperatures();
    T_18B20[2] = sensors.getTempC(DS18B20[2]);

    //sensors.requestTemperatures();
    T_18B20[3] = sensors.getTempC(DS18B20[3]);  

    dataSOIL[4]= T_18B20[1];  
}


float read_N(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(nitro,sizeof(nitro))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
   }
  return values[4];
}

float read_P(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(phos,sizeof(phos))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
  }
  return values[4];
}
 
float read_K(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(pota,sizeof(pota))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
  }
  return values[4];
}

float read_PH(){
  float PHvalue;
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(ph,sizeof(ph))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
    PHvalue = (((values[3] * 256.0) + values[4])/10); // converting hexadecimal to decimal
   }
  return PHvalue;
}

float read_M(){
  float Mvalue;
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(moist,sizeof(moist))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
    Mvalue = (((values[3] * 256.0) + values[4])/10); // converting hexadecimal to decimal
   }
  return Mvalue;
}

float read_C(){
  float Cvalue;
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(cond,sizeof(cond))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
    }
    Cvalue = (((values[3] * 256.0) + values[4])); // converting hexadecimal to decimal
   }
  return Cvalue;
}

void upload2TS_key1(String apiWritekey, float datas0[], float datas1[], uint16_t datas2[]){
  if (client.connect(server,80)){  
    String tsData1 = apiWritekey;
           tsData1 +="&field1=";
           tsData1 += String(datas0[0]);
           tsData1 +="&field2=";
           tsData1 += String(datas0[1]);
           tsData1 +="&field3=";
           tsData1 += String(datas0[2]);      
           tsData1 +="&field4=";
           tsData1 += String(datas1[3]);
           tsData1 +="&field5=";
           tsData1 += String(datas1[4]);
           tsData1 +="&field6=";
           tsData1 += String(datas1[5]);
           tsData1 +="&field7=";
           tsData1 += String(datas1[6]);  
           tsData1 +="&field8=";
           tsData1 += String(datas2[3]);                                          
           tsData1 += "\r\n\r\n";
        
     client.print("POST /update HTTP/1.1\n");
     client.print("Host: api.thingspeak.com\n");
     client.print("Connection: close\n");
     client.print("X-THINGSPEAKAPIKEY: "+apiWritekey+"\n");
     client.print("Content-Type: application/x-www-form-urlencoded\n");
     client.print("Content-Length: ");
     client.print(tsData1.length());
     client.print("\n\n");  // the 2 carriage returns indicate closing of Header fields & starting of data
     client.print(tsData1);
  }
  client.stop();
}

void upload2TS_key2(String apiWritekey, float datas0[]){
  if (client.connect(server,80)){  
    String tsData1 = apiWritekey;
           tsData1 +="&field1=";
           tsData1 += String(datas0[0]);
           tsData1 +="&field2=";
           tsData1 += String(datas0[1]);
           tsData1 +="&field3=";
           tsData1 += String(datas0[2]);                                              
           tsData1 += "\r\n\r\n";
        
     client.print("POST /update HTTP/1.1\n");
     client.print("Host: api.thingspeak.com\n");
     client.print("Connection: close\n");
     client.print("X-THINGSPEAKAPIKEY: "+apiWritekey+"\n");
     client.print("Content-Type: application/x-www-form-urlencoded\n");
     client.print("Content-Length: ");
     client.print(tsData1.length());
     client.print("\n\n");  // the 2 carriage returns indicate closing of Header fields & starting of data
     client.print(tsData1);
  }
  client.stop();
}
