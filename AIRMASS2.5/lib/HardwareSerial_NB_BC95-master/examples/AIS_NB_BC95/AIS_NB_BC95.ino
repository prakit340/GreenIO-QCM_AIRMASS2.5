#include "HardwareSerial_NB_BC95.h"

 
String deviceToken = "jGmVGPqcZKlXt8ppagBT";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";
String udpData = "{\"Tn\":\"jGmVGPqcZKlXt8ppagBT\",\"temp\":34.3}";

HardwareSerial_NB_BC95 AISnb;

const long interval = 20000;  //millisecond
unsigned long previousMillis = 0;

long cnt = 0;
void setup()
{ 
  AISnb.debug = true;

  Serial.begin(115200);

 
  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();  
  delay(1000);
  
  pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();

}
void loop()
{ 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
    {
      cnt++;     
           
      // Send data in String 
      UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, udpData);
      previousMillis = currentMillis;
	  String nccid = AISnb.getNCCID();
	  Serial.print("nccid:");
	  Serial.println(nccid);
  
    }
  UDPReceive resp = AISnb.waitResponse();
     
}
