#define TINY_GSM_MODEM_BG96 // EC25

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_BME280.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <ESPUI.h>
#include <ESPmDNS.h>
#include <TaskScheduler.h>
#include <Adafruit_SGP30.h>
#include <time.h>
#include <Update.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SSLClient.h>
#include "cert.h"
#include <PMserial.h>

#include <http_ota.h>
#include "QcomLoGo.h"
#include "lv1.h"
#include "lv2.h"
#include "lv3.h"
#include "lv4.h"
#include "lv5.h"
#include "lv6.h"
#include "NBIOT.h"
#include "wifilogo.h"
#include "Logo4g.h"
#include "Free_Fonts.h"

EEPROMClass TVOCBASELINE("eeprom1");
EEPROMClass eCO2BASELINE("eeprom2");

struct tm tmstruct;
struct tm timeinfo;

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_PRIORITY
#define _TASK_WDT_IDS
#define _TASK_TIMECRITICAL

#define ProjectName "AIRMASS 2.5 Inspector"
#define FirmwareVersion "0.0.6"
String current_version = "0.0.6";

const char *passAP = "greenio7650";

#define SEALEVELPRESSURE_HPA (1013.25)

#define TINY_GSM_RX_BUFFER 1030
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_TCP true
#define SerialMon Serial
#define GSM_PIN ""
#define CURRENT_VERSION_ADDR 2
#define UART_BAUD 115200

#define MODEM_TX 27
#define MODEM_RX 14
#define GSM_RESET 25

// Your GPRS credentials
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

void readPMSdata();
void composeJson();
void enterDetailsCallback(Control *sender, int type);
void sendAttribute();
void processAtt(char jsonAtt[]);
void reconnectMqtt();
void heartBeat();

void t1CallGetProbe();
void t2CallShowEnv();
void t3CallSendData();
void t4CallPrintPMS7003();
void t6OTA();
void t7showTime();
void getDataSGP30();
void PowerOFF_PMS7003();
void PowerON_PMS7003();

int periodSendTelemetry = 60; // the value is a number of seconds

/*
Task t1(60000, TASK_FOREVER, &t1CallGetProbe);  //adding task to the chain on creation
Task t2(60000, TASK_FOREVER, &t2CallShowEnv);
Task t3(300000, TASK_FOREVER, &t3CallSendData);
Task t4(300000, TASK_FOREVER, &t4CallPrintPMS7003);  //adding task to the chain on creation
Task t5(120000, TASK_FOREVER, &heartBeat);
Task t6(600000, TASK_FOREVER, &OTA_git_CALL);
Task t7(500, TASK_FOREVER, &t7showTime);

const unsigned long time2send = periodSendTelemetry * 1000;
Task t8(time2send, TASK_FOREVER, &composeJson);
*/

#define TFT_BLACK 0x0000       /*   0,   0,   0 */
#define TFT_NAVY 0x000F        /*   0,   0, 128 */
#define TFT_DARKGREEN 0x03E0   /*   0, 128,   0 */
#define TFT_DARKCYAN 0x03EF    /*   0, 128, 128 */
#define TFT_MAROON 0x7800      /* 128,   0,   0 */
#define TFT_PURPLE 0x780F      /* 128,   0, 128 */
#define TFT_OLIVE 0x7BE0       /* 128, 128,   0 */
#define TFT_LIGHTGREY 0xD69A   /* 211, 211, 211 */
#define TFT_DARKGREY 0x7BEF    /* 128, 128, 128 */
#define TFT_BLUE 0x001F        /*   0,   0, 255 */
#define TFT_GREEN 0x07E0       /*   0, 255,   0 */
#define TFT_CYAN 0x07FF        /*   0, 255, 255 */
#define TFT_RED 0xF800         /* 255,   0,   0 */
#define TFT_MAGENTA 0xF81F     /* 255,   0, 255 */
#define TFT_WHITE 0xFFFF       /* 255, 255, 255 */
#define TFT_GREENYELLOW 0xB7E0 /* 180, 255,   0 */
#define TFT_PINK 0xFE19        /* 255, 192, 203 */
#define TFT_BROWN 0x9A60       /* 150,  75,   0 */
#define TFT_GOLD 0xFEA0        /* 255, 215,   0 */
#define TFT_SILVER 0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE 0x867D     /* 135, 206, 235 */
#define TFT_VIOLET 0x915C      /* 180,  46, 226 */
#define TFT_BURGUNDY 0xF1EE

#define WDTPin 33   // Watch Dog pin for Trig
#define boot5VPIN 4 // PMS7003 Power pin. It is step up 3.3Vv to 5v

#define SERIAL1_RXPIN 16 // PMS7003 UART RX to TX
#define SERIAL1_TXPIN 17 // PMS7003 UART TX to RX

String deviceToken = "";
const char *thingsboardServer = "tb.thingcontrol.io";
const int PORT = 1883;

const char serverOTA[] = "raw.githubusercontent.com";
const int port = 443;

String new_version;
const char version_url[] = "/prakit340/GreenIO-OTA/main/ota/product/qualcomm/airmass25/version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path

String firmware_url;

HardwareSerial SerialPMS(2);
HardwareSerial SerialAT(1);
Adafruit_SGP30 sgp;
Adafruit_BME280 bme; // I2C
WiFiClient wifiClient;
PubSubClient client(wifiClient);
WiFiManager wifiManager;
Scheduler runner;
SerialPM pms(PMSx003, SerialPMS); // PMSx003, UART

// GSM Object
TinyGsm modem(SerialAT);

// HTTPS Transport MQTT
TinyGsmClient gsm_mqtt_client(modem, 0);
PubSubClient GSMmqtt(gsm_mqtt_client);

// HTTPS Transport OTA
TinyGsmClient base_client(modem, 1);
SSLClient secure_layer(&base_client);
HttpClient GSMclient = HttpClient(secure_layer, serverOTA, port);

String host = "";
#define FORCE_USE_HOTSPOT 0

#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32

#define title1 "PM2.5" // Text that will be printed on screen in any font
#define title2 "PM1"
#define title3 "PM10"
#define title4 "CO2"
#define title5 "VOC"
#define title6 "Update"
#define title7 "ug/m3"
#define title8 "HUMI: "
#define title9 "TEMP: "
#define FILLCOLOR1 0xFFFF

unsigned BMEstatus;
unsigned SGPstatus;

float temp(NAN), humi(NAN), pres(NAN), altitude(NAN);
int TempOffset = 0;
int HumiOffset = 0;
int pm01Offset = 0;
int pm25Offset = 0;
int pm10Offset = 0;
int pn03Offset = 0;
int pn05Offset = 0;
int pn10Offset = 0;
int pn25Offset = 0;
int pn50Offset = 0;
int pn100Offset = 0;
int eCO2Offset = 0;
int TVOCOffset = 0;
String bmeStatus = "";
String mqttStatus = "";

boolean GSMnetwork = false;
boolean GSMgprs = false;

int xpos = 0;
int ypos = 0;

int testNum = 0;

int tftMax = 240;

bool connectWifi = false;

String json = "";

/// UI handles
uint16_t nameLabel, idLabel, cuationlabel, firmwarelabel, mainSwitcher, mainSlider, mainText, settingZNumber, resultButton, mainTime, downloadButton, selectDownload, logStatus;
uint16_t tempText, humiText, saveConfigButton, interval, emailText;
uint16_t pm01Text, pm25Text, pm10Text, pn03Text, pn05Text, pn10Text, pn25Text, pn50Text, pn100Text, lineText, eCO2Text, TVOCText;
uint16_t bmeLog, wifiLog, teleLog;

String email = "";
String lineID = "";

String imsi = "";
String NCCID = "";
boolean readPMS = false;

TFT_eSPI tft = TFT_eSPI();

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

TFT_eSprite otaStat = TFT_eSprite(&tft);

int status = WL_IDLE_STATUS;

uint32_t lastReconnectAttempt = 0;

unsigned long previous_t1 = 0;
unsigned long previous_t2 = 0;
unsigned long previous_t3 = 0;
unsigned long previous_t4 = 0;
unsigned long previous_t5 = 0;

uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

struct pms7003data
{
  uint16_t pm01_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
};

struct pms7003data data;

void splash()
{
  int xpos = 0;
  int ypos = 0;

  tft.init();
  // Swap the colour byte order when rendering
  tft.setSwapBytes(true);
  tft.setRotation(3); // landscape

  tft.fillScreen(TFT_WHITE);

  // Draw the icons
  tft.pushImage(tft.width() / 2 - logoWidth / 2, 40, logoWidth, logoHeight, Logo);
  tft.setTextColor(tft.color24to16(0xa907f5));
  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  tft.setFreeFont(FSB9);
  xpos = tft.width() / 2; // Half the screen width
  ypos = 190;
  String namePro = Project;
  tft.drawString(namePro, xpos, ypos, GFXFF); // Draw the text string in the selected GFX free font
  //  tft.drawString("", xpos, ypos + 20, GFXFF); // Draw the text string in the selected GFX free font
  //  AISnb.debug = true;
  //  AISnb.setupDevice(serverPort);

  tft.setTextDatum(BR_DATUM);
  tft.setFreeFont(FF1);
  tft.setTextColor(tft.color24to16(0x000000));
  String verPro = " v." + FirmwareVer;
  tft.drawString(verPro, 475, 320, GFXFF);

  NCCID = deviceToken.c_str();
  NCCID.trim();
  NCCID.toUpperCase();
  String nccidStr = "";
  nccidStr.concat("Device ID:");
  nccidStr.concat(NCCID);

  Serial.println(" ");
  Serial.println(nccidStr);
  Serial.println(" ");
  // delay(4000);

  tft.setTextDatum(TC_DATUM);
  tft.setFreeFont(FSB9);
  tft.setTextColor(tft.color24to16(0xec0606));
  tft.drawString(nccidStr, xpos, ypos + 30, GFXFF);

  delay(15000);

  tft.setTextFont(GLCD);
  tft.fillScreen(TFT_DARKCYAN);
  // Select the font
  ypos += tft.fontHeight(GFXFF); // Get the font height and move ypos down
  tft.setFreeFont(FSB9);

  delay(1200);
  tft.setTextPadding(180);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  Serial.println(F("Start..."));
  for (int i = 0; i < 30; i++)
  {
    tft.drawString("Waiting for Initializing GSM modem ...", xpos, 100, GFXFF);
    tft.drawString(".", 1 + 2 * i, 260, GFXFF);
    delay(10);
    //    Serial.println(i);
  }
  Serial.println("end");
}

void _initLCD()
{
  tft.fillScreen(TFT_BLACK);
  // TFT
  splash();
}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void _initBME280()
{
  BMEstatus = bme.begin(0x76);

  if (!BMEstatus)
  {
    bmeStatus = "Could not find BME280 sensor!";
    Serial.println(bmeStatus);
    ESPUI.updateLabel(bmeLog, String(bmeStatus));
    delay(1000);
  }

  bmeStatus = "Initialize BME sensor";
  ESPUI.updateLabel(bmeLog, String(bmeStatus));
}

void calibrateSGP30()
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

  calibrateSGP30();
}

void printBME280Data()
{
  Serial.println("----- Read BME280 ------");
  temp = bme.readTemperature() + (TempOffset); // compensate
  humi = bme.readHumidity() + (HumiOffset);
  pres = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println(" °c");
  Serial.print("Humi: ");
  Serial.print(humi);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(pres);
  Serial.println(" hPa");
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.println(" ");
}

void setUpUI()
{
  // Turn off verbose  ging
  ESPUI.setVerbosity(Verbosity::Quiet);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  /*
     Tab: Home
    -----------------------------------------------------------------------------------------------------------*/
  auto maintab = ESPUI.addControl(Tab, "", "Home");
  nameLabel = ESPUI.addControl(Label, "Device Name", ProjectName, Emerald, maintab);
  idLabel = ESPUI.addControl(Label, "Device ID", String(deviceToken), Emerald, maintab);
  firmwarelabel = ESPUI.addControl(Label, "Firmware", String(FirmwareVer), Emerald, maintab);

   /*
     Tab: Setting
    -----------------------------------------------------------------------------------------------------------*/
  auto settingTab = ESPUI.addControl(Tab, "", "Setting");
  cuationlabel = ESPUI.addControl(Label, "Cuation", "Offset will be divided by 100 after saving.", Emerald, settingTab);
  ESPUI.addControl(Separator, "Offset Configuration", "", None, settingTab);
  tempText = ESPUI.addControl(Number, "Temperature", String(TempOffset), Emerald, settingTab, enterDetailsCallback);
  humiText = ESPUI.addControl(Number, "Humidity", String(HumiOffset), Emerald, settingTab, enterDetailsCallback);
  pm01Text = ESPUI.addControl(Number, "PM1.0", String(pm01Offset), Emerald, settingTab, enterDetailsCallback);
  pm25Text = ESPUI.addControl(Number, "PM2.5", String(pm25Offset), Emerald, settingTab, enterDetailsCallback);
  pm10Text = ESPUI.addControl(Number, "PM10", String(pm10Offset), Emerald, settingTab, enterDetailsCallback);
  ESPUI.addControl(Separator, "Offset Number of Particles Configuration", "", None, settingTab);
  pn03Text = ESPUI.addControl(Number, "0.3 micrometer", String(pn03Offset), Emerald, settingTab, enterDetailsCallback);
  pn05Text = ESPUI.addControl(Number, "0.5 micrometer", String(pn05Offset), Emerald, settingTab, enterDetailsCallback);
  pn10Text = ESPUI.addControl(Number, "1.0 micrometer", String(pn10Offset), Emerald, settingTab, enterDetailsCallback);
  pn25Text = ESPUI.addControl(Number, "2.5 micrometer", String(pn25Offset), Emerald, settingTab, enterDetailsCallback);
  pn50Text = ESPUI.addControl(Number, "5.0 micrometer", String(pn50Offset), Emerald, settingTab, enterDetailsCallback);
  pn100Text = ESPUI.addControl(Number, "10 micrometer", String(pn100Offset), Emerald, settingTab, enterDetailsCallback);
  eCO2Text = ESPUI.addControl(Number, "Carbon dioxide (eCO2)", String(eCO2Offset), Emerald, settingTab, enterDetailsCallback);
  TVOCText = ESPUI.addControl(Number, "Volatile organic Compounds", String(TVOCOffset), Emerald, settingTab, enterDetailsCallback);
  ESPUI.addControl(Separator, "Interval Configuration", "", None, settingTab);
  interval = ESPUI.addControl(Number, "Interval (second)", String(periodSendTelemetry), Emerald, settingTab, enterDetailsCallback);

  ESPUI.addControl(Separator, "Email Registry", "", None, settingTab);
  emailText = ESPUI.addControl(Text, "Email User", "", Emerald, settingTab, enterDetailsCallback);
  lineText = ESPUI.addControl(Text, "Line ID", "", Emerald, settingTab, enterDetailsCallback);

  ESPUI.addControl(Separator, "", "", None, settingTab);
  ESPUI.addControl(Button, "Save", "SAVE", Peterriver, settingTab, enterDetailsCallback);

  auto eventTab = ESPUI.addControl(Tab, "", "Event Log");
  ESPUI.addControl(Separator, "Error Log", "", None, eventTab);
  teleLog = ESPUI.addControl(Label, "Server Connection Status", String(mqttStatus), Alizarin, eventTab, enterDetailsCallback);
  bmeLog = ESPUI.addControl(Label, "Sensor Connection Status", String(bmeStatus), Alizarin, eventTab, enterDetailsCallback);

  host = "QCM-" + deviceToken;
  ESPUI.begin(host.c_str());
}

void enterDetailsCallback(Control *sender, int type)
{
  Serial.println(sender->value);
  ESPUI.updateControl(sender);

  if (type == B_UP)
  {
    Serial.println("Saving Offset to EEPROM...");

    // Fetch controls
    Control *TempOffset_ = ESPUI.getControl(tempText);
    Control *HumiOffset_ = ESPUI.getControl(humiText);
    Control *pm01Offset_ = ESPUI.getControl(pm01Text);
    Control *pm25Offset_ = ESPUI.getControl(pm25Text);
    Control *pm10Offset_ = ESPUI.getControl(pm10Text);
    Control *pn03Offset_ = ESPUI.getControl(pn03Text);
    Control *pn05Offset_ = ESPUI.getControl(pn05Text);
    Control *pn10Offset_ = ESPUI.getControl(pn10Text);
    Control *pn25Offset_ = ESPUI.getControl(pn25Text);
    Control *pn50Offset_ = ESPUI.getControl(pn50Text);
    Control *pn100Offset_ = ESPUI.getControl(pn100Text);
    Control *eCO2Offset_ = ESPUI.getControl(eCO2Text);
    Control *TVOCOffset_ = ESPUI.getControl(TVOCText);
    Control *periodSendTelemetry_ = ESPUI.getControl(interval);
    Control *email_ = ESPUI.getControl(emailText);
    Control *lineID_ = ESPUI.getControl(lineText);

    // Store control values
    TempOffset = TempOffset_->value.toInt();
    HumiOffset = HumiOffset_->value.toInt();
    pm01Offset = pm01Offset_->value.toInt();
    pm25Offset = pm25Offset_->value.toInt();
    pm10Offset = pm10Offset_->value.toInt();
    pn03Offset = pn03Offset_->value.toInt();
    pn05Offset = pn05Offset_->value.toInt();
    pn10Offset = pn10Offset_->value.toInt();
    pn25Offset = pn25Offset_->value.toInt();
    pn50Offset = pn50Offset_->value.toInt();
    pn100Offset = pn100Offset_->value.toInt();
    eCO2Offset = eCO2Offset_->value.toInt();
    TVOCOffset = TVOCOffset_->value.toInt();
    periodSendTelemetry = periodSendTelemetry_->value.toInt();
    email = email_->value;
    lineID = lineID_->value;
    char data1[40];
    char data2[40];
    char data3[40];
    email.toCharArray(data1, 40); // Convert String to char array
    lineID.toCharArray(data2, 40);
    deviceToken.toCharArray(data3, 40);

    // Print to Serial
    Serial.println("----- Submit by ESPUI -----");
    Serial.println("put TempOffset: " + String(TempOffset));
    Serial.println("put HumiOffset: " + String(HumiOffset));
    Serial.println("put periodSendTelemetry: " + String(periodSendTelemetry));
    Serial.println("put email: " + String(email));
    Serial.println(" ");

    // Write to EEPROM
    EEPROM.begin(512); // Ensure enough size for data
    int addr = 0;

    EEPROM.put(addr, TempOffset);
    addr += sizeof(TempOffset);
    EEPROM.put(addr, HumiOffset);
    addr += sizeof(HumiOffset);
    EEPROM.put(addr, pm01Offset);
    addr += sizeof(pm01Offset);
    EEPROM.put(addr, pm25Offset);
    addr += sizeof(pm25Offset);
    EEPROM.put(addr, pm10Offset);
    addr += sizeof(pm10Offset);
    EEPROM.put(addr, pn03Offset);
    addr += sizeof(pn03Offset);
    EEPROM.put(addr, pn05Offset);
    addr += sizeof(pn05Offset);
    EEPROM.put(addr, pn10Offset);
    addr += sizeof(pn10Offset);
    EEPROM.put(addr, pn25Offset);
    addr += sizeof(pn25Offset);
    EEPROM.put(addr, pn50Offset);
    addr += sizeof(pn50Offset);
    EEPROM.put(addr, pn100Offset);
    addr += sizeof(pn100Offset);
    EEPROM.put(addr, eCO2Offset);
    addr += sizeof(eCO2Offset);
    EEPROM.put(addr, TVOCOffset);
    addr += sizeof(TVOCOffset);
    EEPROM.put(addr, periodSendTelemetry);
    //  addr += sizeof(periodSendTelemetry);
    addr = 70;
    for (int len = 0; len < email.length(); len++)
    {
      EEPROM.write(addr + len, data1[len]); // Write each character
    }
    EEPROM.write(addr + email.length(), '\0'); // Add null terminator at the end
    addr = 110;
    for (int len = 0; len < lineID.length(); len++)
    {
      EEPROM.write(addr + len, data2[len]); // Write each character
    }
    EEPROM.write(addr + lineID.length(), '\0'); // Add null terminator at the end

    EEPROM.commit();

    EEPROM.end();
    sendAttribute(); // Assuming this function is required to send attributes
  }
}

void readEEPROM()
{
  Serial.println("Reading credentials from EEPROM...");
  EEPROM.begin(512); // Ensure enough size for data

  int addr = 0;
  EEPROM.get(addr, TempOffset);
  addr += sizeof(TempOffset);
  EEPROM.get(addr, HumiOffset);
  addr += sizeof(HumiOffset);
  EEPROM.get(addr, pm01Offset);
  addr += sizeof(pm01Offset);
  EEPROM.get(addr, pm25Offset);
  addr += sizeof(pm25Offset);
  EEPROM.get(addr, pm10Offset);
  addr += sizeof(pm10Offset);
  EEPROM.get(addr, pn03Offset);
  addr += sizeof(pn03Offset);
  EEPROM.get(addr, pn05Offset);
  addr += sizeof(pn05Offset);
  EEPROM.get(addr, pn10Offset);
  addr += sizeof(pn10Offset);
  EEPROM.get(addr, pn25Offset);
  addr += sizeof(pn25Offset);
  EEPROM.get(addr, pn50Offset);
  addr += sizeof(pn50Offset);
  EEPROM.get(addr, pn100Offset);
  addr += sizeof(pn100Offset);
  EEPROM.get(addr, eCO2Offset);
  addr += sizeof(eCO2Offset);
  EEPROM.get(addr, TVOCOffset);
  addr += sizeof(TVOCOffset);
  EEPROM.get(addr, periodSendTelemetry);
  //  addr += sizeof(periodSendTelemetry);
  addr = 70;
  for (int len = 0; len < 50; len++)
  {
    char data1 = EEPROM.read(addr + len);
    if (data1 == '\0' || data1 == 255)
      break;
    email += data1;
  }
  //  addr += sizeof(email);
  addr = 110;
  for (int len = 0; len < 50; len++)
  {
    char data2 = EEPROM.read(addr + len);
    if (data2 == '\0' || data2 == 255)
      break;
    lineID += data2;
  }

  EEPROM.end();

  // Print to Serial
  Serial.println("----- Read EEPROM Storage value by ESPUI -----");
  Serial.println("get TempOffset: " + String(TempOffset));
  Serial.println("get HumOffset1: " + String(HumiOffset));
  Serial.println("get periodSendTelemetry: " + String(periodSendTelemetry));
  Serial.println("get outputEmail1: " + String(email));
  Serial.println(" ");

  pm01Offset = 0;
  pm25Offset = 0;
  pm10Offset = 0;
  pn03Offset = 0;
  pn05Offset = 0;
  pn10Offset = 0;
  pn25Offset = 0;
  pn50Offset = 0;
  pn100Offset = 0;

  ESPUI.updateNumber(tempText, TempOffset);
  ESPUI.updateNumber(humiText, HumiOffset);
  ESPUI.updateNumber(interval, periodSendTelemetry);
  ESPUI.updateNumber(pm01Text, pm01Offset);
  ESPUI.updateNumber(pm25Text, pm25Offset);
  ESPUI.updateNumber(pm10Text, pm10Offset);
  ESPUI.updateNumber(pn03Text, pn03Offset);
  ESPUI.updateNumber(pn05Text, pn05Offset);
  ESPUI.updateNumber(pn10Text, pn10Offset);
  ESPUI.updateNumber(pn25Text, pn25Offset);
  ESPUI.updateNumber(pn50Text, pn50Offset);
  ESPUI.updateNumber(pn100Text, pn100Offset);
  ESPUI.updateNumber(eCO2Text, eCO2Offset);
  ESPUI.updateNumber(TVOCText, TVOCOffset);
  ESPUI.updateText(emailText, String(email));
  ESPUI.updateText(lineText, String(lineID));
}

void processAtt(char jsonAtt[])
{
  char *aString = jsonAtt;
  Serial.println("OK");
  Serial.print(F("+:topic v1/devices/me/attributes , "));
  Serial.println(aString);
  client.publish("v1/devices/me/attributes", aString);
}

boolean reconnectWiFiMqtt()
{

  SerialMon.print("Connecting to ");
  SerialMon.print(thingsboardServer);

  boolean status = client.connect(deviceToken.c_str(), deviceToken.c_str(), NULL);

  if (status == false)
  {
    SerialMon.println(" fail");
    mqttStatus = "Failed to Connect Server with WiFi!";
    ESPUI.updateLabel(teleLog, String(mqttStatus));
    return false;
  }
  SerialMon.println(" success");
  mqttStatus = "Succeed to Connect Server with WiFi!";
  ESPUI.updateLabel(teleLog, String(mqttStatus));
  Serial.println(F("Connect MQTT Success."));
  client.subscribe("v1/devices/me/rpc/request/+");
  return client.connected();
}

boolean reconnectGSMMqtt()
{
  SerialMon.print("Connecting to ");
  SerialMon.print(thingsboardServer);

  boolean status = GSMmqtt.connect(deviceToken.c_str(), deviceToken.c_str(), NULL);

  if (status == false)
  {
    SerialMon.println(" fail");
    mqttStatus = "Failed to Connect Server with GSM!";
    ESPUI.updateLabel(teleLog, String(mqttStatus));
    return false;
  }
  SerialMon.println(" success");
  mqttStatus = "Succeed to Connect Server with GSM!";
  ESPUI.updateLabel(teleLog, String(mqttStatus));
  Serial.println(F("Connect MQTT Success."));
  GSMmqtt.subscribe("v1/devices/me/rpc/request/+");
  return GSMmqtt.connected();
}

void sendAttribute()
{
  String json = "";
  json.concat("{\"email\":\"");
  json.concat(String(email));
  json.concat("\"}");
  Serial.println(json);

  int str_len = json.length() + 1;
  char char_array[str_len];

  json.toCharArray(char_array, str_len);
  processAtt(char_array);
}

void t1CallGetProbe()
{
  Serial.println("----- t1 Call Get Sensor -----");
  readPMSdata();
  t4CallPrintPMS7003();
  printBME280Data();
  getDataSGP30();
  Serial.println("----- END t1 Call Get Sensor -----");
  Serial.println(" ");
}

void getDataSGP30()
{
  Serial.println("----- Read SGP30 Sensor -----");
  float temperature = temp; // [°C]
  float humidity = humi;     // [%RH]
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
  Serial.println(" ");
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

void t3CallSendData()
{
  Serial.println("----- Send DATA -----");

  tft.setTextColor(0xFFFF);
  int mapX = 475;
  int mapY = 25;
  Serial.println(WL_CONNECTED);
  Serial.print("(WiFi.status():");
  Serial.println(WiFi.status());
  if (connectWifi == false)
  {
    Serial.println("connectwifi boolean: " + String(connectWifi));

    int csq = modem.getSignalQuality();
    int rssi = map(csq, 0, 31, 25, 100);
    tft.fillRect(425, 5, 55, 35, 0x0000);
    tft.drawString(String(rssi) + "%", mapX, mapY, GFXFF);
    // tft.pushImage(400, 0, logo4gWidth, logo4gHeight, Logo4g);
    tft.fillCircle(406, 16, 16, tft.color24to16(0x06ec3e));
    tft.setTextColor(TFT_WHITE);
    tft.setFreeFont(FSSB9);
    tft.drawString("4G", 415, 27);
  }
  else if (WiFi.status() == WL_CONNECTED)
  {
    int rssi = map(WiFi.RSSI(), -90, -50, 25, 100);
    if (rssi > 100)
      rssi = 100;
    if (rssi < 0)
      rssi = 0;
    tft.fillRect(425, 5, 55, 35, 0x0000);
    tft.drawString(String(rssi) + "%", mapX, mapY, GFXFF);
    tft.fillCircle(406, 16, 16, tft.color24to16(0x00a7ff));
    tft.setTextColor(TFT_YELLOW);
    tft.setFreeFont(FSSB9);
    tft.drawString("W", 415, 27);
    // client.setInsecure();
    Serial.print(" deviceToken.c_str()");
    Serial.println(deviceToken.c_str());
  }
  Serial.println(" ");
}

void composeJson()
{
  json = "";
  Serial.println("----- JSON Compose -----");
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  json.concat("\",\"temp\":");
  json.concat(temp + (TempOffset / 100));
  json.concat(",\"hum\":");
  json.concat(humi + (HumiOffset / 100));
  json.concat(",\"pres\":");
  json.concat(pres);
  json.concat(",\"altitude\":");
  json.concat(altitude);
  json.concat(",\"pm1\":");
  json.concat(data.pm01_env + (pm01Offset / 100));
  json.concat(",\"pm2.5\":");
  json.concat(data.pm25_env + (pm25Offset / 100));
  json.concat(",\"pm10\":");
  json.concat(data.pm100_env + (pm10Offset / 100));
  json.concat(",\"pn03\":");
  json.concat(data.particles_03um + (pn03Offset / 100));
  json.concat(",\"pn05\":");
  json.concat(data.particles_05um + (pn05Offset / 100));
  json.concat(",\"pn10\":");
  json.concat(data.particles_10um + (pn10Offset / 100));
  json.concat(",\"pn25\":");
  json.concat(data.particles_25um + (pn25Offset / 100));
  json.concat(",\"pn50\":");
  json.concat(data.particles_50um + (pn50Offset / 100));
  json.concat(",\"pn100\":");
  json.concat(data.particles_100um + (pn100Offset / 100));
  json.concat(",\"co2\":");
  json.concat(sgp.eCO2);
  json.concat(",\"voc\":");
  json.concat(sgp.TVOC);
  json.concat(",\"project\":\"AIRMASS2.5\"");
  json.concat(",\"v\":\"");
  json.concat(FirmwareVer);
  json.concat("\"}");
  Serial.println(json);

  // Length (with one extra character for the null terminator)
  int str_len = json.length() + 1;
  // Prepare the character array (the buffer)
  char char_array[str_len];
  // Copy it over
  json.toCharArray(char_array, str_len);
  if (connectWifi)
  {
    client.publish("v1/devices/me/telemetry", char_array);
  }
  else
  {
    GSMmqtt.publish("v1/devices/me/telemetry", char_array);
  }
  Serial.println(" ");
}

void t4CallPrintPMS7003()
{
  Serial.println(" ");
  Serial.println("---- t4 Call Print PMS7003 ----");

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

  Serial.println(" ");
}

void heartBeat()
{
  //   Sink current to drain charge from watchdog circuit
  pinMode(WDTPin, OUTPUT);
  digitalWrite(WDTPin, LOW);

  delay(100);

  // Return to high-Z
  pinMode(WDTPin, INPUT);

  Serial.println(" ");
  Serial.println("Trig - Heartbeat (!!)");
  Serial.println(" ");
}

void t2CallShowEnv()
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

  // ################################################################ for testing
  //        data.pm25_env = testNum;    //for testing
  //        testNum++;
  // ################################################################ end test

  drawPM2_5(data.pm25_env + (pm25Offset / 100), mid, 70);

  tft.setTextSize(1);
  tft.setFreeFont(CF_OL32); // Select the font

  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(FSB9); // Select Free Serif 9 point font, could use:

  tft.drawString(title2, 45, 280, GFXFF); // Print the test text in the custom font
  drawPM1(data.pm01_env + (pm01Offset / 100), 11, 290);

  tft.drawString(title3, 115, 280, GFXFF); // Print the test text in the custom font
  drawPM10(data.pm100_env + (pm10Offset / 100), 70, 290);

  tft.drawString(title4, 175, 280, GFXFF); // Print the test text in the custom font
  drawCO2(sgp.eCO2, 140, 290);

  tft.drawString(title5, 245, 280, GFXFF); // Print the test text in the custom font
  drawVOC(sgp.TVOC, 205, 290);

  tft.drawString(title9, 365, 280, GFXFF); // Print the test text in the custom font
  drawT(temp + (TempOffset / 100), 375, 260);
  tft.drawString("°C", 435, 280, GFXFF);

  tft.drawString(title8, 365, 310, GFXFF); // Print the test text in the custom font
  drawH(humi + (HumiOffset / 100), 375, 290);
  tft.drawString("%", 435, 310, GFXFF);

  // Clear Stage

  ind.createSprite(480, 10);
  ind.fillSprite(TFT_BLACK);

  if ((data.pm25_env + (pm25Offset / 100) >= 0) && (data.pm25_env + (pm25Offset / 100) <= 15.4))
  {
    tft.setWindow(0, 25, 55, 55);
    tft.pushImage(tft.width() - lv1Width - 60, 70, lv1Width, lv1Height, lv1);
    ind.fillTriangle(7, 0, 12, 5, 17, 0, FILLCOLOR1);
  }
  else if ((data.pm25_env + (pm25Offset / 100) >= 15.5) && (data.pm25_env + (pm25Offset / 100) <= 40.4))
  {
    tft.pushImage(tft.width() - lv2Width - 60, 70, lv2Width, lv2Height, lv2);
    ind.fillTriangle(82, 0, 87, 5, 92, 0, FILLCOLOR1);
  }
  else if ((data.pm25_env + (pm25Offset / 100) >= 40.5) && (data.pm25_env + (pm25Offset / 100) <= 65.4))
  {
    tft.pushImage(tft.width() - lv3Width - 60, 70, lv3Width, lv3Height, lv3);
    ind.fillTriangle(160, 0, 165, 5, 170, 0, FILLCOLOR1);
  }
  else if ((data.pm25_env + (pm25Offset / 100) >= 65.5) && (data.pm25_env + (pm25Offset / 100) <= 150.4))
  {
    tft.pushImage(tft.width() - lv4Width - 60, 70, lv4Width, lv4Height, lv4);
    ind.fillTriangle(240, 0, 245, 5, 250, 0, FILLCOLOR1);
  }
  else if ((data.pm25_env + (pm25Offset / 100) >= 150.5) && (data.pm25_env + (pm25Offset / 100) <= 250.4))
  {
    tft.pushImage(tft.width() - lv5Width - 60, 70, lv5Width, lv5Height, lv5);
    ind.fillTriangle(320, 0, 325, 5, 330, 0, FILLCOLOR1);
  }
  else
  {
    tft.pushImage(tft.width() - lv6Width - 60, 70, lv6Width, lv6Height, lv6);
    ind.fillTriangle(400, 0, 405, 5, 410, 0, FILLCOLOR1);
  }
  ind.pushSprite(29, 230);
  ind.deleteSprite();
}

String a0(int n)
{
  return (n < 10) ? "0" + String(n) : String(n);
}

String dateTimeStr = "";
long timezone = 7;
byte daysavetime = 0;

void t7showTime()
{
  Serial.println("---- Show time ----");
  topNumber.createSprite(200, 40);
  //  stringPM1.fillSprite(TFT_GREEN);
  topNumber.setFreeFont(FS9);
  topNumber.setTextColor(TFT_WHITE);
  topNumber.setTextSize(1); // Font size scaling is x1
  String yearStr = "";
  String monthStr = "";
  String dayStr = "";
  String hourStr = "";
  String minStr = "";
  tmstruct.tm_year = 0;
  configTime(3600 * timezone, daysavetime * 3600, "pool.ntp.org");
  getLocalTime(&tmstruct, 5000);
  yearStr = String(tmstruct.tm_year + 1900, DEC);
  monthStr = String(tmstruct.tm_mon + 1, DEC);
  dayStr = String(tmstruct.tm_mday, DEC);
  hourStr = String(a0(tmstruct.tm_hour));
  minStr = String(a0(tmstruct.tm_min));

  int year3 = 0;
  int month3 = 0;
  int day3 = 0;
  int hour3 = 0;
  int min3 = 0;
  int sec3 = 0;
  float timezone = 0;

  //  unsigned long NowTime = _epoch + ((millis() - time_s) / 1000) + (7 * 3600);
  String timeS = "";

  if (connectWifi == false)
  {
    Serial.println("---- Asking modem to sync with NTP ----");
    modem.NTPServerSync("132.163.96.5", 20);

    for (int8_t i = 5; i; i--)
    {
      Serial.println("Requesting current network time.. ");
      if (modem.getNetworkTime(&year3, &month3, &day3, &hour3, &min3, &sec3,
                               &timezone))
      {
        Serial.println("OK");
        // Serial.print("Year:");
        // Serial.print(year3);
        // Serial.print("\tMonth:");
        // Serial.print(month3);
        // Serial.print("\tDay:");
        // Serial.println(day3);
        // Serial.print("Hour:");
        // Serial.print(hour3);
        // Serial.print("\tMinute:");
        // Serial.print(min3);
        // Serial.print("\tSecond:");
        // Serial.println(sec3);
        // Serial.print("Timezone:");
        // Serial.println(timezone);
        break;
      }
      else
      {
        Serial.println("Couldn't get network time, retrying in next loop");
        return;
      }
    }
    timeS = String(a0(day3)) + "/" + String(a0(month3)) + "/" + String(year3) + "  " + String(a0(hour3)) + ":" + String(a0(min3)) + "";
  }
  else
  {
    if (!getLocalTime(&timeinfo))
    {
      Serial.println("Failed to obtain time");
      //      ESP.restart();
      return;
    }
  }

  topNumber.drawString(timeS, 5, 5, GFXFF);
  topNumber.pushSprite(5, 5);
  topNumber.deleteSprite();
  Serial.println(" ");
}

void readPMSdata()
{
  Serial.println("Read PMS data");

  pms.read();
  if (pms)
  { // successfull read
    Serial.println("Have PMS data");

    // // print formatted results
    // Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
    //               pms.pm01, pms.pm25, pms.pm10);

    // if (pms.has_number_concentration())
    //   Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
    //                 pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);

    data.pm01_env = pms.pm01;
    data.pm25_env = pms.pm25;
    data.pm100_env = pms.pm10;

    data.particles_03um = pms.n0p3;
    data.particles_05um = pms.n0p5;
    data.particles_10um = pms.n1p0;
    data.particles_25um = pms.n2p5;
    data.particles_50um = pms.n5p0;
    data.particles_100um = pms.n10p0;
  }
  else
  { // something went wrong
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }

    PowerOFF_PMS7003();
    delay(3000);
    PowerON_PMS7003();
    pms.init();
  }
}

void getMac()
{
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.println("OK");
  Serial.print("+deviceToken: ");
  Serial.println(WiFi.macAddress());
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] < 0x10)
    {
      deviceToken += "0"; // Add leading zero if needed
    }
    deviceToken += String(mac[i], HEX); // Convert byte to hex
  }
  deviceToken.toUpperCase();
  ESPUI.updateLabel(idLabel, String(deviceToken));
}

void PowerON_PMS7003()
{
  digitalWrite(boot5VPIN, HIGH);
}

void PowerOFF_PMS7003()
{
  digitalWrite(boot5VPIN, LOW);
}

bool checkForUpdate(String &firmware_url)
{
  heartBeat();

  Serial.println("Making GET request securely...");
  GSMclient.get(version_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  String response_body = GSMclient.responseBody();
  delay(1000);

  Serial.print("Status code: ");
  Serial.println(status_code);
  Serial.print("Response: ");
  Serial.println(response_body);

  response_body.trim();
  response_body.replace("\r", ""); // Remove carriage returns
  response_body.replace("\n", ""); // Remove newlines

  // Extract the version number from the response
  new_version = response_body;

  Serial.println("Current version: " + current_version);
  Serial.println("Available version: " + new_version);
  GSMclient.stop();

  if (new_version != current_version)
  {
    Serial.println("New version available. Updating...");
    firmware_url = String("/prakit340/GreenIO-OTA/main/ota/product/qualcomm/airmass25/firmware_v") + new_version + ".bin";
    Serial.println("Firmware URL: " + firmware_url);
    return true;
  }
  else
  {
    Serial.println("Already on the latest version");
  }

  return false;
}

// Update the latest firmware which has uploaded to Github
void performOTA(const char *firmware_url)
{
  heartBeat();

  // Initialize HTTP
  Serial.println("Making GET request securely...");
  GSMclient.get(firmware_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  long contentlength = GSMclient.contentLength();
  delay(1000);

  Serial.print("Contentlength: ");
  Serial.println(contentlength);

  if (status_code == 200)
  {

    if (contentlength <= 0)
    {
      SerialMon.println("Failed to get content length");
      GSMclient.stop();
      return;
    }

    // Begin OTA update
    bool canBegin = Update.begin(contentlength);
    size_t written;
    long totalBytesWritten = 0;
    uint8_t buffer[1024];
    int bytesRead;
    long contentlength_real = contentlength;

    if (canBegin)
    {
      heartBeat();

      while (contentlength > 0)
      {
        heartBeat();

        bytesRead = GSMclient.readBytes(buffer, sizeof(buffer));
        if (bytesRead > 0)
        {
          written = Update.write(buffer, bytesRead);
          if (written != bytesRead)
          {
            Serial.println("Error: written bytes do not match read bytes");
            Update.abort();
            return;
          }
          totalBytesWritten += written; // Track total bytes written

          Serial.printf("Write %.02f%% (%ld/%ld)\n", (float)totalBytesWritten / (float)contentlength_real * 100.0, totalBytesWritten, contentlength_real);

          String OtaStat = "OTA Updating: " + String((float)totalBytesWritten / (float)contentlength_real * 100.0) + " % ";
          otaStat.createSprite(220, 40);
          otaStat.fillSprite(TFT_BLACK);
          otaStat.setFreeFont(FS9);
          otaStat.setTextColor(TFT_YELLOW);
          otaStat.setTextSize(1);
          otaStat.drawString(OtaStat, 5, 5);
          otaStat.pushSprite(160, 5);

          contentlength -= bytesRead; // Reduce remaining content length
        }
        else
        {
          Serial.println("Error: Timeout or no data received");
          break;
        }
      }

      if (totalBytesWritten == contentlength_real)
      {
        Serial.println("Written : " + String(totalBytesWritten) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentlength_real) + ". Retry?");
        otaStat.deleteSprite();
      }

      if (Update.end())
      {
        SerialMon.println("OTA done!");
        if (Update.isFinished())
        {
          SerialMon.println("Update successfully completed. Rebooting.");
          delay(300);
          ESP.restart();
        }
        else
        {
          SerialMon.println("Update not finished? Something went wrong!");
        }
      }
      else
      {
        SerialMon.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      Serial.println("Not enough space to begin OTA");
    }
  }
  else
  {
    Serial.println("Cannot download firmware. HTTP code: " + String(status_code));
  }

  GSMclient.stop();
}

void GSM_OTA()
{
  Serial.println("---- GSM OTA Check version before update ----");

  if (checkForUpdate(firmware_url))
  {
    performOTA(firmware_url.c_str());
  }
}

void setup()
{
  pinMode(boot5VPIN, OUTPUT);

  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH); // RS-485

  PowerON_PMS7003();

  Serial.begin(115200);
  SerialPMS.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  Project = ProjectName;
  FirmwareVer = FirmwareVersion;
  getMac();

  _initLCD();
  _initBME280();
  _initSGP30();
  pms.init();
  readEEPROM();

  Serial.println(F("---- Starting... QCM - AIRMASS 2.5 ----"));

  boolean GSMgprs = true;

  delay(2000);
  // pinMode(GSM_RESET, OUTPUT);
  // digitalWrite(GSM_RESET, HIGH); // RS-485
  delay(10);
  SerialMon.println("Wait...");

  secure_layer.setCACert(root_ca);

  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  for (int i = 30; i < 100; i++)
  {
    tft.drawString(".", 1 + 2 * i, 260, GFXFF);
    delay(10);
  }
  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);

  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  for (int i = 100; i < 170; i++)
  {
    tft.drawString(".", 1 + 2 * i, 260, GFXFF);
    delay(10);
  }

  tft.setTextPadding(180);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Waiting for network ... ", 100, 170, GFXFF);
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    GSMnetwork = false;
    tft.drawString(" Fail", 320, 170, GFXFF);
    Serial.println(" fail");
    delay(10000);
  }
  else
  {
    GSMnetwork = true;
    tft.drawString(" OK", 320, 170, GFXFF);
    Serial.println(" OK");
  }
  delay(2000);

  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  for (int i = 170; i < 205; i++)
  {
    tft.drawString(".", 1 + 2 * i, 260, GFXFF);
    delay(10);
  }

  String showText = "Connecting to ";
  showText += apn;
  showText += " ...";
  tft.setTextPadding(180);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(showText, 100, 200, GFXFF);
  Serial.print("Connecting to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass))
  {
    GSMgprs = false;
    tft.drawString(" Fail", 320, 200, GFXFF);
    Serial.println(" fail");
    delay(10000);
  }
  else
  {
    GSMgprs = true;
    tft.drawString(" OK", 320, 200, GFXFF);
    Serial.println(" OK");
  }
  delay(3000);

  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  for (int i = 205; i < 240; i++)
  {
    tft.drawString(".", 1 + 2 * i, 260, GFXFF);
    delay(10);
  }

  Serial.println(" ");
  Serial.println("===================");
  Serial.print("GSM Network: ");
  Serial.print(GSMnetwork);
  Serial.print("\tGSM GPRS: ");
  Serial.println(GSMgprs);
  Serial.println("===================");
  Serial.println(" ");

  if ((GSMnetwork == true) && (GSMgprs == true))
  {
    connectWifi = false;
  }

  if ((GSMnetwork == false) || (GSMgprs == false))
  {
    connectWifi = true;
  }

  if (connectWifi)
  {
    Serial.print("Start Config WiFi because GSM lose ..");
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_GREEN);
    tft.fillScreen(TFT_DARKCYAN);
    tft.drawString("Wait for WiFi Setting (Timeout 60 Sec)", tft.width() / 2, tft.height() / 2, GFXFF);

    // wifiManager.resetSettings();
    String host1 = "QCM-" + deviceToken;
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setConfigPortalTimeout(60); // auto close configportal after n seconds
    wifiManager.setAPClientCheck(true);     // avoid timeout if client connected to softap
    wifiManager.setBreakAfterConfig(true);  // always exit configportal even if wifi save fails
    if (!wifiManager.autoConnect(host1.c_str()))
    {
      Serial.println("failed to connect and hit timeout");
      delay(1000);
    }

    configTime(3600 * timezone, daysavetime * 3600, "0.pool.ntp.org", "1.pool.ntp.org", "time.nist.gov");
  }

  if (connectWifi)
  {
    tft.setTextDatum(BR_DATUM);
    tft.setFreeFont(FF1);
    tft.setTextColor(TFT_RED);
    String onlineBy = "Online by WiFi";
    tft.drawString(onlineBy, 475, 320, GFXFF);

    client.setServer(thingsboardServer, PORT);
  }
  else
  {
    tft.setTextDatum(BR_DATUM);
    tft.setFreeFont(FF1);
    tft.setTextColor(TFT_RED);
    String onlineBy = "Online by GSM";
    tft.drawString(onlineBy, 475, 320, GFXFF);

    GSMmqtt.setServer(thingsboardServer, PORT);
  }
  delay(2000);

  host = "QCM-" + deviceToken;
  MDNS.begin(host.c_str());
  WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(host.c_str(), passAP);

  setUpUI(); // Start the GUI
  delay(200);

  tft.fillScreen(TFT_BLACK); // Clear screen

  tft.fillRect(5, 240, tft.width() - 15, 5, tft.color24to16(0x82c442));   // Print the test text in the custom font
  tft.fillRect(80, 240, tft.width() - 15, 5, tft.color24to16(0xfaf05d));  // Print the test text in the custom font
  tft.fillRect(155, 240, tft.width() - 15, 5, tft.color24to16(0xed872d)); // Print the test text in the custom font
  tft.fillRect(235, 240, tft.width() - 15, 5, tft.color24to16(0xeb3220)); // Print the test text in the custom font
  tft.fillRect(315, 240, tft.width() - 15, 5, tft.color24to16(0x874596)); // Print the test text in the custom font
  tft.fillRect(395, 240, tft.width() - 15, 5, tft.color24to16(0x74091e)); // Print the test text in the custom font
  tft.fillRect(475, 240, tft.width() - 15, 5, TFT_BLACK);                 // Print the test text in the custom font

  t1CallGetProbe();
  t2CallShowEnv();
  t3CallSendData();
  t7showTime();

  previous_t1 = millis() / 1000;
  previous_t2 = millis() / 1000;
  previous_t3 = millis() / 1000;
  previous_t4 = millis() / 1000;
  previous_t5 = millis() / 1000;
}

void loop()
{
  // runner.execute();

  if (connectWifi)
  {
    if (!client.connected())
    {
      Serial.println("=== WiFi MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L)
      {
        lastReconnectAttempt = t;
        if (reconnectWiFiMqtt())
        {
          lastReconnectAttempt = 0;
        }
      }
      delay(100);
      return;
    }

    client.loop();
  }
  else
  {
    if (!modem.isNetworkConnected())
    {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true))
      {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isNetworkConnected())
      {
        SerialMon.println("Network re-connected");
      }

      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected())
      {
        SerialMon.println("GPRS disconnected!");
        SerialMon.print(F("Connecting to "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn, user, pass))
        {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        if (modem.isGprsConnected())
        {
          SerialMon.println("GPRS reconnected");
        }
      }
    }

    if (!GSMmqtt.connected())
    {
      SerialMon.println("=== GSM MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis();
      if (t - lastReconnectAttempt > 10000L)
      {
        lastReconnectAttempt = t;
        if (reconnectGSMMqtt())
        {
          lastReconnectAttempt = 0;
        }
      }
      delay(100);
      return;
    }

    GSMmqtt.loop();
  }

  const unsigned long currentMillis = millis() / 1000;
  const unsigned long time2send = periodSendTelemetry;

  if (currentMillis % time2send == 0)
  {
    composeJson();
    delayMicroseconds(200000);
  }

  if ((currentMillis - previous_t1) >= 60)
  {
    previous_t1 = millis() / 1000;
    composeJson();
  }

  if ((currentMillis - previous_t2) >= 5)
  {
    previous_t2 = millis() / 1000;
    t3CallSendData();
    t1CallGetProbe();
    t2CallShowEnv();
  }

  if ((currentMillis - previous_t3) >= 30)
  {
    previous_t3 = millis() / 1000;
    heartBeat();
  }

  if ((currentMillis - previous_t4) >= 15)
  {
    previous_t4 = millis() / 1000;
    t7showTime();
  }

  if ((currentMillis - previous_t5) >= 300)
  {
    previous_t5 = millis() / 1000;
    // OTA_git_CALL();
    GSM_OTA();
  }
}