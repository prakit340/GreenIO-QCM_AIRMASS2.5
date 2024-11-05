#define WDTPin 33
#define boot5VPIN 4
unsigned long ms;

#include "HardwareSerial_NB_BC95.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <TaskScheduler.h>
#include <PubSubClient.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "time.h"
#include <ArduinoOTA.h>

#include "Logo.h"
#include "lv1.h"
#include "lv2.h"
#include "lv3.h"
#include "lv4.h"
#include "lv5.h"
#include "lv6.h"
#include "NBIOT.h"
#include "Free_Fonts.h"
#include "EEPROM.h"

#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32

#define title1 "PM2.5" // Text that will be printed on screen in any font
#define title2 "PM1"
#define title3 "PM10"
#define title4 "CO2"
#define title5 "VOC"
#define title6 "Update"
#define title7 "ug/m3"
#define title8 "HUMI : "
#define title9 "TEMP : "

#define FILLCOLOR1 0xFFFF
#define TFT_BURGUNDY 0xF1EE

// # OTA Name and Password
#define HOSTNAME "CRA_PM25_CO2"
#define PASSWORD "123456789"

// PMS7003 PIN
#define SERIAL1_RXPIN 16
#define SERIAL1_TXPIN 17

// Instantiate eeprom objects with parameter/argument names and sizes
EEPROMClass TVOCBASELINE("eeprom1");
EEPROMClass eCO2BASELINE("eeprom2");

Scheduler runner;

String pro = "";

int xpos = 0;
int ypos = 0;

boolean ready2display = false;

int wtd = 0;
int maxwtd = 10;
int tftMax = 240; // Width / 2

int error;

Signal meta;
String json = "";
String attr = "";

String deviceToken = "";
String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956";        // Your Server Port;

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);
HardwareSerial hwSerial(2);
Adafruit_BME280 bme; // I2C
HardwareSerial_NB_BC95 AISnb;
TFT_eSPI tft = TFT_eSPI();

float temp(NAN), hum(NAN), pres(NAN);
float TempOffset = 0;
float HumOffset = 0;

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600 * 7;

int nbErrorTime = 0;
bool connectWifi = false;
JsonDocument doc;
bool validEpoc = false;
unsigned long time_s = 0;
unsigned long _epoch = 0;
struct tm timeinfo;
WiFiManager wifiManager;
Adafruit_SGP30 sgp;

String imsi = "";
String NCCID = "";
boolean readPMS = false;

TFT_eSprite stringPM25 = TFT_eSprite(&tft);
TFT_eSprite stringPM1 = TFT_eSprite(&tft);
TFT_eSprite stringPM10 = TFT_eSprite(&tft);
TFT_eSprite stringUpdate = TFT_eSprite(&tft);
TFT_eSprite stringCO2 = TFT_eSprite(&tft);
TFT_eSprite stringVOC = TFT_eSprite(&tft);

TFT_eSprite topNumber = TFT_eSprite(&tft);
TFT_eSprite ind = TFT_eSprite(&tft);
TFT_eSprite H = TFT_eSprite(&tft);
TFT_eSprite T = TFT_eSprite(&tft);

int status = WL_IDLE_STATUS;
String downlink = "";
char *bString;
int PORT = 8883;

unsigned BMEstatus;
unsigned SGPstatus;

// Callback methods prototypes

void drawT(float num, int x, int y);
void drawH(float num, int x, int y);
void drawCO2(int num, int x, int y);
void drawPM10(int num, int x, int y);
void drawPM1(int num, int x, int y);
void drawPM2_5(int num, int x, int y);

void tCallback();
void t1CallGetProbe();
void t2CallShowEnv();
void t3CallSendData();
void t4CallPrintPMS7003();
void t5CallSendAttribute();
void t6CheckTime();
void t7showTime();

// Tasks
Task t1(2000, TASK_FOREVER, &t1CallGetProbe); // adding task to the chain on creation
Task t2(2000, TASK_FOREVER, &t2CallShowEnv);
Task t3(60000, TASK_FOREVER, &t3CallSendData);
Task t4(2000, TASK_FOREVER, &t4CallPrintPMS7003);      // adding task to the chain on creation
Task t5(10400000, TASK_FOREVER, &t5CallSendAttribute); // adding task to the chain on creation
Task t6(60000, TASK_FOREVER, &t6CheckTime);
Task t7(500, TASK_FOREVER, &t7showTime);

struct pms7003data
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms7003data data;

char char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

void drawUpdate(int num, int x, int y)
{
  stringUpdate.createSprite(50, 20);
  stringUpdate.fillScreen(TFT_BLACK);
  stringUpdate.setFreeFont(FSB9);
  stringUpdate.setTextColor(TFT_ORANGE);
  stringUpdate.setTextSize(1);
  stringUpdate.drawNumber(num, 0, 3);
  stringUpdate.drawString("%", 25, 3, GFXFF);
  stringUpdate.pushSprite(x, y);
}

void setupOTA()
{

  ArduinoOTA.setHostname(HOSTNAME);
  // No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  ArduinoOTA.onStart([]()
                     {
    Serial.println("Start Updating....");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem"); });

  ArduinoOTA.onEnd([]()
                   {
    Serial.println("Update Complete!");
    ESP.restart(); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {
                          pro = String(progress / (total / 100)) + "%";
                          int progressbar = (progress / (total / 100));
                          // int progressbar = (progress / 5) % 100;
                          // int pro = progress / (total / 100);

                          drawUpdate(progressbar, 170, 13);

                          Serial.printf("."); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }

    
    Serial.println(info);
    ESP.restart(); });

  ArduinoOTA.begin();
}

void setupWIFI()
{
  WiFi.setHostname(HOSTNAME);

  // 等待5000ms，如果没有连接上，就继续往下
  // 不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count++;
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}

void writeString(char add, String data)
{
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); // Add termination null character for String Data
  EEPROM.commit();
}

void _writeEEPROM(String data)
{
  writeString(10, data); // Address 10 and String type data
  delay(10);
}

String read_String(char add)
{
  int i;
  char data[100]; // Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) // Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';

  return String(data);
}

void getIP(String IP, String Port, String Data)
{
  json = "";
  do
  {
    //    if (AISnb.pingIP(serverIP).status == false) {
    //      ESP.restart();
    //    }
    UDPSend udp = AISnb.sendUDPmsgStr(IP, Port, Data);

    // String nccid = AISnb.getNCCID();
    // Serial.print("nccid:");
    // Serial.println(nccid);

    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    if (udp.status == false)
    {
      connectWifi = true;
      break;
    }
    else
    {
      for (int x = 0; x < resp.data.length(); x += 2)
      {
        char c = char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

        json += c;
      }
      // Serial.println(json);
      DeserializationError error = deserializeJson(doc, json);

      // Test if parsing succeeds.
      if (error)
      {
        // Serial.print(F("deserializeJson() failed: "));
        // Serial.println(error.f_str());
        validEpoc = true;
        delay(1000);
      }
      else
      {
        validEpoc = false;
        time_s = millis();
        _epoch = doc["epoch"];
        String ip = doc["ip"];
        if (ip != "null")
        {
          serverIP = ip;
          _writeEEPROM(serverIP);
          if (EEPROM.commit())
          {
            Serial.println("EEPROM successfully committed");
          }
          Serial.print("Server IP : ");
          Serial.println(serverIP);
        }
        // SerialBT.println(json);
        Serial.println(json);
        // Serial.print("epoch:");
        // Serial.println(_epoch);
      }
    }
    //
  } while (validEpoc);
}

void t6CheckTime()
{
  // Serial.println("Check Time");
  if (connectWifi == false)
  {
    if (_epoch != 0 && (millis() - time_s) > 300000 && hour(_epoch + ((millis() - time_s) / 1000) + (7 * 3600)) == 0)
    {
      Serial.println("Restart Check Time");
      ESP.restart();
    }
  }
  else
  {
    if (!getLocalTime(&timeinfo))
    {
      // Serial.println("Failed to obtain time");
      return;
    }
    Serial.print("timeinfo.tm_hour:");
    Serial.println(timeinfo.tm_hour);
    Serial.print("timeinfo.tm_min:");
    Serial.println(timeinfo.tm_min);
    if ((timeinfo.tm_hour == 0) && (timeinfo.tm_min < 3))
    {
      Serial.println("Restart @ midnight2");
      ESP.restart();
    }
  }
}

void tCallback()
{
  //  Scheduler &s = Scheduler::currentScheduler();
  //  Task &t = s.currentTask();

  //  Serial.print("Task: "); Serial.print(t.getId()); Serial.print(":\t");
  //  Serial.print(millis()); Serial.print("\tStart delay = "); Serial.println(t.getStartDelay());
  //  delay(10);

  if (t1.isFirstIteration())
  {
    runner.addTask(t2);
    t2.enable();
    //    Serial.println("t1CallgetProbe: enabled t2CallshowEnv and added to the chain");
  }
}

void calibrate()
{
  uint16_t readTvoc = 0;
  uint16_t readCo2 = 0;
  Serial.println("Done Calibrate");
  TVOCBASELINE.get(0, readTvoc);
  eCO2BASELINE.get(0, readCo2);

  //  Serial.println("Calibrate");
  Serial.print("****Baseline values: eCO2: 0x");
  Serial.print(readCo2, HEX);
  Serial.print(" & TVOC: 0x");
  Serial.println(readTvoc, HEX);
  sgp.setIAQBaseline(readCo2, readTvoc);
}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void splash()
{
  int xpos = 0;
  int ypos = 0;

  tft.init();
  // Swap the colour byte order when rendering
  tft.setSwapBytes(true);
  tft.setRotation(3); // landscape

  tft.fillScreen(TFT_BLACK);

  // Draw the icons
  tft.pushImage(tft.width() / 2 - logoWidth / 2, 80, logoWidth, logoHeight, Logo);
  tft.setTextColor(tft.color24to16(0x0ade46));
  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  tft.setFreeFont(FSB9);
  xpos = tft.width() / 2; // Half the screen width
  ypos = 200;
  tft.drawString("AIRMASS2.5 Inspector", xpos, ypos, GFXFF); // Draw the text string in the selected GFX free font

  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(tft.color24to16(0xFFFFFF));
  tft.setFreeFont(FF1);
  tft.drawString("V.2.0", 480, 320, GFXFF); // Draw the text string in the selected GFX free font
  // AISnb.debug = true;
  // AISnb.setupDevice(serverPort);
  // //

  // imsi = AISnb.getIMSI();
  // NCCID = AISnb.getNCCID();
  // imsi.trim();
  // NCCID.trim();
  // String nccidStr = "";
  // nccidStr.concat("NCCID:");
  // nccidStr.concat(NCCID);
  // String imsiStr = "";
  // imsiStr.concat("IMSI:");
  // imsiStr.concat(imsi);
  // delay(4000);
  tft.setTextColor(TFT_WHITE);
  tft.setFreeFont(FSB9);
  // tft.drawString(nccidStr, xpos, ypos + 40, GFXFF);
  // tft.drawString(imsiStr, xpos, ypos + 60, GFXFF);
  delay(5000);

  tft.setTextFont(GLCD);
  tft.fillScreen(TFT_DARKCYAN);
  // Select the font
  ypos += tft.fontHeight(GFXFF); // Get the font height and move ypos down
  tft.setFreeFont(FSB9);

  delay(1200);
  tft.setTextPadding(180);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  for (int i = 0; i < 170; i++)
  {
    tft.drawString("Waiting for NB-IoT", xpos, 100, GFXFF);
    tft.drawString(".", 1 + 2 * i, 210, GFXFF);
    delay(10);
    Serial.println(i);
  }
  Serial.println("end");
}

void _initLCD()
{
  tft.fillScreen(TFT_BLACK);
  // TFT
  splash();
}

void _initBME280()
{
  BMEstatus = bme.begin(0x76);
  if (!BMEstatus)
  {
    Serial.println("Sensor BME280 not found :(");
  }
  Serial.print("Found BME280 :)");
}

void errorTimeDisplay(int i)
{
  tft.fillScreen(TFT_DARKCYAN);
  int xpos = tft.width() / 2; // Half the screen width
  int ypos = tft.height() / 2;
  tft.drawString("Connect NB failed " + String(i + 1) + " times", xpos, ypos, GFXFF);
}

void _initSGP30()
{
  SGPstatus = sgp.begin();
  if (!SGPstatus)
  {
    Serial.println("Sensor SGP30 not found :(");
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  calibrate();
}

void setup()
{

  Serial.begin(115200);

  EEPROM.begin(512);

  pinMode(boot5VPIN, OUTPUT);
  digitalWrite(boot5VPIN, HIGH);

  _initLCD();

  //  pinMode(4, OUTPUT);
  //  digitalWrite(4, HIGH);
  // delay(500);

  // deviceToken = AISnb.getNCCID();
  // Serial.println(AISnb.cgatt(1));
  // if (EEPROM.read(10) == 255)
  // {
  //   _writeEEPROM("147.50.151.130");
  // }
  // serverIP = read_String(10);
  // while (nbErrorTime < 10)
  // {
  //   meta = AISnb.getSignal();
  //   Serial.print("meta.rssi:");
  //   Serial.println(meta.rssi);
  //   if (!meta.rssi.equals("N/A"))
  //   {
  //     if (meta.rssi.toInt() > -110)
  //     {
  //       break;
  //     }
  //     else
  //     {
  //       errorTimeDisplay(nbErrorTime);
  //       nbErrorTime++;
  //       delay(1000);
  //     }
  //   }
  //   else
  //   {
  //     errorTimeDisplay(nbErrorTime);
  //     nbErrorTime++;
  //     delay(1000);
  //   }
  // }

  tft.fillScreen(TFT_DARKCYAN);
  tft.drawString("Wait for WiFi Setting (Timeout 120 Sec)", tft.width() / 2, tft.height() / 2, GFXFF);
  wifiManager.setTimeout(120);

  wifiManager.setAPCallback(configModeCallback);
  String wifiName = "@AIRMASS2.5";
  wifiName.concat(String((uint32_t)ESP.getEfuseMac(), HEX));
  if (!wifiManager.autoConnect(wifiName.c_str()))
  {
    // Serial.println("failed to connect and hit timeout");
    // reset and try again, or maybe put it to deep sleep
    //     ESP.reset();
    // delay(1000);
  }
  //  setupWIFI();

  setupOTA();

  // if (nbErrorTime == 10)
  // {
  //   connectWifi = true;
  // }
  // if (connectWifi == false)
  // {
  //   json = "{\"_type\":\"retrattr\",\"Tn\":\"";
  //   json.concat(deviceToken);
  //   json.concat("\",\"keys\":[\"epoch\",\"ip\"]}");
  //   getIP(serverIP, serverPort, json);
  // }

  connectWifi = true;

  if (connectWifi == true)
  {
    configTime(gmtOffset_sec, 0, ntpServer);
    client.setServer("mqtt.thingcontrol.io", PORT);
  }

  hwSerial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);

  _initBME280();
  _initSGP30();
  runner.init();

  runner.addTask(t1);
  runner.addTask(t2);
  runner.addTask(t3);
  runner.addTask(t4);
  runner.addTask(t6);
  runner.addTask(t7);
  delay(2000);

  t1.enable();
  t2.enable();
  t3.enable();
  t4.enable();
  t6.enable();
  t7.enable();

  tft.fillScreen(TFT_BLACK); // Clear screen

  tft.fillRect(5, 240, tft.width() - 15, 5, tft.color24to16(0x82c442));      // Print the test text in the custom font
  tft.fillRect(80, 240, tft.width() - 15, 5, tft.color24to16(0xfaf05d));   // Print the test text in the custom font
  tft.fillRect(155, 240, tft.width() - 15, 5, tft.color24to16(0xed872d));   // Print the test text in the custom font
  tft.fillRect(235, 240, tft.width() - 15, 5, tft.color24to16(0xeb3220));      // Print the test text in the custom font
  tft.fillRect(315, 240, tft.width() - 15, 5, tft.color24to16(0x874596));   // Print the test text in the custom font
  tft.fillRect(395, 240, tft.width() - 15, 5, tft.color24to16(0x74091e)); // Print the test text in the custom font
  tft.fillRect(475, 240, tft.width() - 15, 5, TFT_BLACK); // Print the test text in the custom font
}

void printBME280Data()
{
  _initBME280();
  temp = bme.readTemperature();
  hum = bme.readHumidity();
  pres = bme.readPressure() / 100.0F;

  // Temp Humi Compensate
  temp = temp + TempOffset;
  hum = hum + HumOffset;
}

void composeJson()
{
  // meta = AISnb.getSignal();
  //  SerialBT.println(deviceToken);
  json = "";
  if (connectWifi == false)
  {
    json.concat(" {\"Tn\":\"");
    json.concat(deviceToken);
    json.concat("\",\"temp\":");
  }
  else
  {
    json.concat(" {\"temp\":");
  }
  json.concat(temp);
  json.concat(",\"hum\":");
  json.concat(hum);
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"pm1\":");
  json.concat(data.pm01_env);
  json.concat(",\"pm2.5\":");
  json.concat(data.pm25_env);
  json.concat(",\"pm10\":");
  json.concat(data.pm100_env);

  json.concat(",\"pn03\":");
  json.concat(data.particles_03um);
  json.concat(",\"pn05\":");
  json.concat(data.particles_05um);
  json.concat(",\"pn10\":");
  json.concat(data.particles_10um);
  json.concat(",\"pn25\":");
  json.concat(data.particles_25um);
  json.concat(",\"pn50\":");
  json.concat(data.particles_50um);
  json.concat(",\"pn100\":");
  json.concat(data.particles_100um);
  json.concat(",\"co2\":");
  json.concat(sgp.eCO2);
  json.concat(",\"voc\":");
  json.concat(sgp.TVOC);
  json.concat(",\"project\":\"CRA\"");
  json.concat(",\"rssi\":");
  if (connectWifi == false)
  {
    json.concat(meta.rssi);
  }
  else
  {
    json.concat(WiFi.RSSI());
  }
  json.concat("}");
  Serial.println(json);
  // SerialBT.println(json);
  // SerialBT.println(json);
  if (data.pm25_env > 1000)
  {
    //ESP.restart();
    digitalWrite(boot5VPIN, LOW);
    delay(1000);
    digitalWrite(boot5VPIN, HIGH);
  }
}

void t4CallPrintPMS7003()
{

  // reading data was successful!
  //  Serial.println();
  //  Serial.println("---------------------------------------");
  //    Serial.println("Concentration Units (standard)");
  //    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
  //    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
  //    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
  //    Serial.println("---------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM1.0:");
  Serial.print(data.pm01_env);
  Serial.print("\tPM2.5:");
  Serial.print(data.pm25_env);
  Serial.print("\tPM10:");
  Serial.println(data.pm100_env);
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:");
  Serial.println(data.particles_03um);
  Serial.print("Particles > 0.5um / 0.1L air:");
  Serial.println(data.particles_05um);
  Serial.print("Particles > 1.0um / 0.1L air:");
  Serial.println(data.particles_10um);
  Serial.print("Particles > 2.5um / 0.1L air:");
  Serial.println(data.particles_25um);
  Serial.print("Particles > 5.0um / 0.1L air:");
  Serial.println(data.particles_50um);
  Serial.print("Particles > 10.0 um / 0.1L air:");
  Serial.println(data.particles_100um);
  Serial.println("---------------------------------------");
}

void drawVOC(int num, int x, int y)
{
  stringVOC.createSprite(60, 20);
  //  stringVOC.fillSprite(TFT_GREEN);
  stringVOC.setFreeFont(FSB9);
  stringVOC.setTextColor(TFT_WHITE);
  stringVOC.setTextSize(1);
  stringVOC.drawNumber(num, 0, 3);
  stringVOC.pushSprite(x, y);
  stringVOC.deleteSprite();
}

void t2CallShowEnv()
{
  if (true)//(ready2display)
  {
    tft.setTextDatum(MC_DATUM);
    xpos = tft.width() / 2; // Half the screen width

    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setFreeFont(CF_OL24);
    int mid = (tftMax / 2) - 90;
    tft.setTextPadding(100);
    tft.drawString(title7, xpos - 90, 170, GFXFF); // Print the test text in the custom font

    tft.setFreeFont(CF_OL32);
    tft.drawString(title1, xpos - 90, 200, GFXFF); // Print the test text in the custom font
    
    drawPM2_5(data.pm25_env, mid, 70);

    tft.setTextSize(1);
    tft.setFreeFont(CF_OL32); // Select the font

    tft.setTextDatum(BR_DATUM);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.setFreeFont(FSB9); // Select Free Serif 9 point font, could use:

    tft.drawString(title2, 45, 280, GFXFF); // Print the test text in the custom font
    drawPM1(data.pm01_env, 11, 290);

    tft.drawString(title3, 115, 280, GFXFF); // Print the test text in the custom font
    drawPM10(data.pm100_env, 70, 290);

    tft.drawString(title4, 175, 280, GFXFF); // Print the test text in the custom font
    drawCO2(sgp.eCO2, 140, 290);

    tft.drawString(title5, 245, 280, GFXFF); // Print the test text in the custom font
    drawVOC(sgp.TVOC, 205, 290);

    tft.drawString(title9, 365, 280, GFXFF); // Print the test text in the custom font
    drawT(temp, 375, 260);
    tft.drawString("C", 435, 280, GFXFF);

    tft.drawString(title8, 365, 310, GFXFF); // Print the test text in the custom font
    drawH(hum, 375, 290);
    tft.drawString("%", 435, 310, GFXFF);

    // Clear Stage

    ind.createSprite(480, 10);
    ind.fillSprite(TFT_BLACK);

    if ((data.pm25_env >= 0) && (data.pm25_env <= 12))
    {
      tft.setWindow(0, 25, 55, 55);
      tft.pushImage(tft.width() - lv1Width - 60, 70, lv1Width, lv1Height, lv1);
      ind.fillTriangle(7, 0, 12, 5, 17, 0, FILLCOLOR1);
    }
    else if ((data.pm25_env > 12) && (data.pm25_env <= 35))
    {
      tft.pushImage(tft.width() - lv2Width - 60, 70, lv2Width, lv2Height, lv2);
      ind.fillTriangle(82, 0, 87, 5, 92, 0, FILLCOLOR1);
    }
    else if ((data.pm25_env > 35) && (data.pm25_env <= 55))
    {
      tft.pushImage(tft.width() - lv3Width - 60, 70, lv3Width, lv3Height, lv3);
      ind.fillTriangle(160, 0, 165, 5, 170, 0, FILLCOLOR1);
    }
    else if ((data.pm25_env > 55) && (data.pm25_env <= 150))
    {
      tft.pushImage(tft.width() - lv4Width - 60, 70, lv4Width, lv4Height, lv4);
      ind.fillTriangle(240, 0, 245, 5, 250, 0, FILLCOLOR1);
    }
    else if ((data.pm25_env > 150) && (data.pm25_env <= 250))
    {
      tft.pushImage(tft.width() - lv5Width - 60, 70, lv5Width, lv5Height, lv5);
      ind.fillTriangle(320, 0, 325, 5, 330, 0, FILLCOLOR1);
    }
    else if ((data.pm25_env > 250))
    {
      tft.pushImage(tft.width() - lv6Width - 60, 70, lv6Width, lv6Height, lv6);
      ind.fillTriangle(400, 0, 405, 5, 410, 0, FILLCOLOR1);
    }
    ind.pushSprite(29, 230);
    ind.deleteSprite();
  }
}

String a0(int n)
{
  return (n < 10) ? "0" + String(n) : String(n);
}

void t7showTime()
{

  topNumber.createSprite(200, 40);
  //  stringPM1.fillSprite(TFT_GREEN);
  topNumber.setFreeFont(FS9);
  topNumber.setTextColor(TFT_WHITE);
  topNumber.setTextSize(1); // Font size scaling is x1

  // topNumber.drawString(">1.0um", 0, 21, GFXFF); // Print the test text in the custom font
  // topNumber.drawNumber(data.particles_10um, 10, 0);   //tft.drawString("0.1L air", 155, 5, GFXFF);
  // topNumber.drawString(">2.5um", 95, 21, GFXFF); // Print the test text in the custom font
  // topNumber.drawNumber(data.particles_25um, 105, 0);   //tft.drawString("0.1L air", 155, 5, GFXFF);
  // topNumber.drawString(">5.0um", 180, 21, GFXFF); // Print the test text in the custom font
  // topNumber.drawNumber(data.particles_50um, 192, 0);   //tft.drawString("0.1L air", 155, 5, GFXFF);
  unsigned long NowTime = _epoch + ((millis() - time_s) / 1000) + (7 * 3600);
  String timeS = "";
  if (connectWifi == false)
  {
    timeS = a0(day(NowTime)) + "/" + a0(month(NowTime)) + "/" + String(year(NowTime)) + "  " + a0(hour(NowTime)) + ":" + a0(minute(NowTime)) + "";
  }
  else
  {
    if (!getLocalTime(&timeinfo))
    {
      // Serial.println("Failed to obtain time");
      return;
    }
    timeS = a0(timeinfo.tm_mday) + "/" + a0(timeinfo.tm_mon + 1) + "/" + String(timeinfo.tm_year + 1900) + "  " + a0(timeinfo.tm_hour) + ":" + a0(timeinfo.tm_min) + "";
  }
  topNumber.drawString(timeS, 5, 5, GFXFF);
  // Serial.println(timeS);
  topNumber.pushSprite(5, 5);
  topNumber.deleteSprite();
}

boolean readPMSdata(Stream *s)
{
  //  Serial.println("readPMSdata");
  if (!s->available())
  {
    Serial.println("readPMSdata.false");
    // SerialBT.println("readPMSdata.false");

    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  memcpy((void *)&data, (void *)buffer_u16, 30);
  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }
  if (sum != data.checksum)
  {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}

void t5CallSendAttribute()
{
  attr = "";
  attr.concat("{\"Tn\":\"");
  attr.concat(deviceToken);
  attr.concat("\",\"IMSI\":");
  attr.concat("\"");
  attr.concat(imsi);
  attr.concat("\"}");
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, attr);
}

void getDataSGP30()
{
  // put your main code here, to run repeatedly:
  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  float temperature = temp; // [°C]
  float humidity = hum;     // [%RH]
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (!sgp.IAQmeasure())
  {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC ");
  Serial.print(sgp.TVOC);
  Serial.print(" ppb\t");
  Serial.print("eCO2 ");
  Serial.print(sgp.eCO2);
  Serial.println(" ppm");

  if (!sgp.IAQmeasureRaw())
  {
    Serial.println("Raw Measurement failed");
    return;
  }
  Serial.print("Raw H2 ");
  Serial.print(sgp.rawH2);
  Serial.print(" \t");
  Serial.print("Raw Ethanol ");
  Serial.print(sgp.rawEthanol);
  Serial.println("");

  uint16_t TVOC_base, eCO2_base;

  //  Serial.print("eCo2: ");   Serial.println(readCo2);
  //  Serial.print("voc: ");  Serial.println(readTvoc);

  //      sgp.setIAQBaseline(eCO2_base, TVOC_base);
  if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
  {
    Serial.println("Failed to get baseline readings");
    return;
  }

  Serial.print("****Get Baseline values: eCO2: 0x");
  Serial.print(eCO2_base, HEX);
  Serial.print(" & TVOC: 0x");
  Serial.println(TVOC_base, HEX);
}
void t1CallGetProbe()
{
  tCallback();
  boolean pmsReady = readPMSdata(&hwSerial);

  if (pmsReady)
  {
    ready2display = true;
    wtd = 0;
  }
  else
  {
    ready2display = false;
  }

  if (wtd > maxwtd)
    ESP.restart();

  printBME280Data();
  getDataSGP30();
}

void drawPM2_5(int num, int x, int y)
{
  // Create a sprite 80 pixels wide, 50 high (8kbytes of RAM needed)
  stringPM25.createSprite(230, 75);
  //  stringPM25.fillSprite(TFT_YELLOW);
  stringPM25.setTextSize(3);       // Font size scaling is x1
  stringPM25.setFreeFont(CF_OL24); // Select free font

  stringPM25.setTextColor(TFT_WHITE);

  stringPM25.setTextSize(3);

  int mid = (tftMax / 2) - 1;

  stringPM25.setTextColor(TFT_WHITE); // White text, no background colour
  // Set text coordinate datum to middle centre
  stringPM25.setTextDatum(MC_DATUM);
  // Draw the number in middle of 80 x 50 sprite
  stringPM25.drawNumber(num, mid, 25);
  // Push sprite to TFT screen CGRAM at coordinate x,y (top left corner)
  stringPM25.pushSprite(x, y);
  // Delete sprite to free up the RAM
  stringPM25.deleteSprite();
}

void drawT(float num, int x, int y)
{
  T.createSprite(40, 20);
  T.fillSprite(TFT_BLACK);
  T.setFreeFont(FSB9);
  T.setTextColor(TFT_WHITE);
  T.setTextSize(1);
  String myString = ""; // empty string
  myString.concat(num);
  T.drawString(myString, 0, 3);
  T.pushSprite(x, y);
  //  T.deleteSprite();
}

void drawH(float num, int x, int y)
{
  H.createSprite(40, 20);
  //  stringPM1.fillSprite(TFT_GREEN);
  H.setFreeFont(FSB9);
  H.setTextColor(TFT_WHITE);
  H.setTextSize(1);
  String myString = ""; // empty string
  myString.concat(num);
  H.drawString(myString, 0, 3);
  H.pushSprite(x, y);
  H.deleteSprite();
}

void drawPM1(int num, int x, int y)
{
  stringPM1.createSprite(60, 20);
  stringPM1.fillSprite(TFT_BLACK);
  stringPM1.setFreeFont(FSB9);
  stringPM1.setTextColor(TFT_WHITE);
  stringPM1.setTextSize(1);
  stringPM1.drawNumber(num, 0, 3);
  stringPM1.pushSprite(x, y);
  //  stringPM1.deleteSprite();
}
//
void drawCO2(int num, int x, int y)
{
  stringCO2.createSprite(60, 20);
  stringCO2.fillSprite(TFT_BLACK);
  stringCO2.setFreeFont(FSB9);
  stringCO2.setTextColor(TFT_WHITE);
  stringCO2.setTextSize(1);
  stringCO2.drawNumber(num, 0, 3);
  stringCO2.pushSprite(x, y);
  //  stringCO2.deleteSprite();
}
//

void drawPM10(int num, int x, int y)
{
  stringPM10.createSprite(60, 20);
  //  stringPM1.fillSprite(TFT_GREEN);
  stringPM10.setFreeFont(FSB9);
  stringPM10.setTextColor(TFT_WHITE);
  stringPM10.setTextSize(1);
  stringPM10.drawNumber(num, 0, 3);
  stringPM10.pushSprite(x, y);
  stringPM10.deleteSprite();
}

void t3CallSendData()
{
  digitalWrite(12, HIGH);
  delay(2000);
  digitalWrite(12, LOW);
  composeJson();

  tft.setTextColor(0xFFFF);
  int mapX = 475;
  int mapY = 25;
  Serial.println(WL_CONNECTED);
  Serial.print("(WiFi.status():");
  Serial.println(WiFi.status());
  if (connectWifi == false)
  {
    // if (AISnb.pingIP(serverIP).status == false) {
    //  ESP.restart();
    // }
    int rssi = map(meta.rssi.toInt(), -110, -50, 25, 99);
    if (rssi > 99)
      rssi = 99;
    if (rssi < 0)
      rssi = 0;
    tft.fillRect(435, 5, 45, 35, 0x0000);
    tft.drawString(String(rssi) + " %", mapX, mapY, GFXFF);
    tft.pushImage(400, 0, nbiotWidth, nbiotHeight, nbiot);
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  }
  else if (WiFi.status() == WL_CONNECTED)
  {
    int rssi = map(WiFi.RSSI(), -90, -50, 25, 99);
    if (rssi > 99)
      rssi = 99;
    if (rssi < 0)
      rssi = 0;
    tft.fillRect(435, 5, 45, 35, 0x0000);
    tft.drawString(String(rssi) + " %", mapX, mapY, GFXFF);
    tft.fillCircle(406, 16, 16, 0x00a7ff);
    tft.setTextColor(0x0000);
    tft.setFreeFont(FSSB9);
    tft.drawString("W", 415, 27);
    // client.setInsecure();
    Serial.print(" deviceToken.c_str()");
    Serial.println(deviceToken.c_str());
    if (client.connect("AIRMASS", deviceToken.c_str(), NULL))
    {
      Serial.println("******************************************************8Connected!");
      Serial.println(json.c_str());
      client.publish("v1/devices/me/telemetry", json.c_str());
    }
  }
}

void heartBeat()
{
  // Sink current to drain charge from watchdog circuit
  pinMode(WDTPin, OUTPUT);
  digitalWrite(WDTPin, LOW);

  // Return to high-Z
  pinMode(WDTPin, INPUT);

  Serial.println("Heartbeat");
  // SerialBT.println("Heartbeat");
}

void loop()
{
  ArduinoOTA.handle();
  runner.execute();
  ms = millis();

  if (ms % 300000 == 0)
  {
    heartBeat();
  }

  if (ms % 600000 == 0)
  {

    Serial.println("Attach WiFi for，OTA ");
    Serial.println(WiFi.RSSI());
    // SerialBT.println("Attach WiFi for OTA"); SerialBT.println(WiFi.RSSI() );
    setupWIFI();
    setupOTA();
  }
}
