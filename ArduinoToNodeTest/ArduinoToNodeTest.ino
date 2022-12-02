//Test of Arduino to NodeMCU  - Node Perspective
#include <ArduinoWiFiServer.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>
//#include <WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>

#define rxPin 12 //D6
#define txPin 13 //D7

String incomingData = "";
String curData = "";
int packetArray[5];
SoftwareSerial nodeMCU(rxPin, txPin);
bool wait = false;
bool ready = false;
//Uno NodeMCU Broadcast MAC Address that we are sending, UDP
uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xCF, 0x83};
bool received = false;
struct sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  bool manual = false; //False = Automatic, True = Manual Control
  bool ack = false; //False = Was not Received any response, True = Received Packet
};
struct sensor_data packet;
struct sensor_data incomingPacket;
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingMotor = incomingReadings.motor;
  incomingAcknowledgement = incomingReadings.acknowledgement;
}


void printIncomingReadings()
{
  Serial.println("INCOMING READINGS");
  Serial.print("Acknowledgement:  ");
  Serial.println();
  Serial.print("Motor:  ");
  Serial.println(incomingMotor);
}
void setup()
{
  Serial.begin(115200);
  nodeMCU.begin(115200);
  //dataTransfer.begin(nodeMCU);
}

void loop()
{
  //Check if we are receiving anything from Arduino Central Command
  if(nodeMCU.available() != 0)
  {
    readIncomingData();
    //Serial.println(wait);
  }

  if(ready)
  {
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    ready = false;
  }
}


void readIncomingData()
{
  //Serial.println("We are reading something");
  char temp = nodeMCU.read();
  incomingData += String(temp);
  if(temp == '>')
  {
    Serial.println(incomingData);
    String data = incomingData;
    curData = incomingData;
    incomingData  = "";
    convertDataIntoPacket(data);
    ready = true;
  }
}

void convertDataIntoPacket(String data)
{
  String temp = "";
  int count = 0;
  
  for(int i = 0; i < data.length(); i++)
  {
    char ch = data.charAt(i);
    if(ch == '<' || ch == '>')
    {
      continue;
    }
    else if(ch == ',')
    {
      packetArray[count] = temp.toInt();
      count++;
      temp = "";
    }
    else
    {
      temp += ch;
    }
  }
  displayData();
  populatePacketData();
}

void displayData()
{
  Serial.print("Packet Microhpone = ");
  Serial.print(packetArray[0]);
  Serial.println();
  Serial.print("Packet Ultrasonic = ");
  Serial.print(packetArray[1]);
  Serial.println();
  Serial.print("Packet Start = ");
  Serial.print(packetArray[2]);
  Serial.println();
  Serial.print("Packet Manual = ");
  Serial.print(packetArray[3]);
  Serial.println();
  Serial.print("Packet Ack = ");
  Serial.print(packetArray[4]);
  Serial.println();
  
}
void populatePocketData()
{
  packet.microphone_direction = packArray[0];
  packet.ultrasonic_distance = packArray[1];
  packet.start = packArray[2];
  packet.manual = packArray[3];
  packet.ack = packArray[4];
}