#include <Arduino.h>
#include <EEPROM.h>

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
int periodSendTelemetry = 60; // the value is a number of seconds
String email = "";
String lineID = "";

void writeEEPROM()
{
  Serial.println("Writing value offset to EEPROM...");

  char data1[40];
  char data2[40];
  email.toCharArray(data1, 40); // Convert String to char array
  lineID.toCharArray(data2, 40);

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

  Serial.println("Finish writed");
  Serial.println("--------------------------------");
  Serial.println(" ");
  Serial.println(" ");
  Serial.println(" ");
}

void readEEPROM()
{
  Serial.println("Reading value offset from EEPROM...");
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
  Serial.println("----- Read EEPROM Storage value by Default -----");
  Serial.println("get TempOffset: " + String(TempOffset));
  Serial.println("get HumiOffset: " + String(HumiOffset));
  Serial.println("get pm01Offset: " + String(pm01Offset));
  Serial.println("get pm25Offset: " + String(pm25Offset));
  Serial.println("get pm10Offset: " + String(pm10Offset));
  Serial.println("get pn03Offset: " + String(pn03Offset));
  Serial.println("get pn05Offset: " + String(pn05Offset));
  Serial.println("get pn10Offset: " + String(pn10Offset));
  Serial.println("get pn25Offset: " + String(pn25Offset));
  Serial.println("get pn50Offset: " + String(pn50Offset));
  Serial.println("get pn100Offset: " + String(pn100Offset));
  Serial.println("get eCO2Offset: " + String(eCO2Offset));
  Serial.println("get TVOCOffset: " + String(TVOCOffset));
  Serial.println("get periodSendTelemetry: " + String(periodSendTelemetry));
  Serial.println("get outputEmail: " + String(email));
  Serial.println("get outputLineID: " + String(lineID));
  Serial.println(" ");
}

void setup()
{
  Serial.begin(115200);
  writeEEPROM();
  delay(1000);
  readEEPROM();
}

void loop()
{
  // put your main code here, to run repeatedly:
}
