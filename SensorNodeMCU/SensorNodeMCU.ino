//Test of Arduino to NodeMCU  - Node Perspective, Slave - Sensor = 50:02:91:DC:C0:34
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
bool received = false;
//From Mega Sensor Side
uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xCF, 0x83};
//uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xC0, 0x34};
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
    ready = true; //Re-Attempt
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingPacket, incomingData, sizeof(incomingPacket));
  Serial.print("Bytes received: ");
  Serial.println(len);
  packet.microphone_direction = incomingPacket.microphone_direction;
  packet.ultrasonic_distance = incomingPacket.ultrasonic_distance;
  packet.start = incomingPacket.start;
  packet.manual = incomingPacket.manual;
  packet.ack = incomingPacket.ack;
  received = true;
}


void printIncomingReadings()
{
  Serial.println("INCOMING READINGS");
  Serial.print("Microhpone Direction:  ");
  Serial.println(incomingPacket.microphone_direction);
  Serial.print("Ultrasonic Distance:  ");
  Serial.println(incomingPacket.ultrasonic_distance);
  Serial.print("Start Set to:  ");
  Serial.println(incomingPacket.start);
  Serial.print("Manual Set to:  ");
  Serial.println(incomingPacket.manual);
  Serial.print("Ack:  ");
  Serial.println(incomingPacket.ack);
}
void setup()
{
  Serial.begin(115200);
  nodeMCU.begin(115200);
  //dataTransfer.begin(nodeMCU);
  Serial.print("ESP Uno MAC Board Address is:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0)
  {
    Serial.println("Error in Initializing ESP-NOW");
  }
  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
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
    Serial.println("Sending from Sensor");
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    ready = false;
  }

  if(received)
  {
    printIncomingReadings();
    received = false;
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
  //displayData();
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
void populatePacketData()
{
  packet.microphone_direction = packetArray[0];
  packet.ultrasonic_distance = packetArray[1];
  packet.start = packetArray[2];
  packet.manual = packetArray[3];
  packet.ack = packetArray[4];
}