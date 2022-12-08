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

#define rxPin D1 //D1
#define txPin D2 //D2

String incomingData = "";
String curData = "";
int packetArray[8];
SoftwareSerial NodeMcu_SoftSerial(rxPin, txPin);
bool wait = false;
bool ready = false;
bool received = false;
bool newData = false;
bool sendData = false;
String dataIn = "";
char c;
long interval = 1000; //Adjust this for nodeMCU as Sensor will have its own timer
long prevMillis = 0;
//From Mega Sensor Side
uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xCF, 0x83};
//uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xC0, 0x34};
struct sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  bool leftMic = 0;  // 0 = Never asked or request, anything else is from manual
  bool rightMic = 0; // 0 = Never asked or request, anything else is from manual
  int16_t heading = 0; // 0 = Never asked or request, anything else is from manual
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  int8_t manual = 0; //0: auto , 1: motor, 2: distance, 3: L mic, 4: R mic, 5: compass
  bool ack = false; //False = Was not Received any response, True = Received Packet
};
struct sensor_data packet;
struct sensor_data incomingPacket;
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
    sendData = false;
  }
  else{
    Serial.println("Delivery fail");
    //ready = true; //Re-Attempt
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  if(!received)
  {
    memcpy(&incomingPacket, incomingData, sizeof(incomingPacket));
    Serial.print("Bytes received: ");
    Serial.println(len);
    received = true;
  }
}

void transferIncomingIntoPacket()
{
  packet.microphone_direction = incomingPacket.microphone_direction;
  packet.ultrasonic_distance = incomingPacket.ultrasonic_distance;
  packet.leftMic = incomingPacket.leftMic;
  packet.rightMic = incomingPacket.rightMic;
  packet.heading = incomingPacket.heading;
  packet.start = incomingPacket.start;
  packet.manual = incomingPacket.manual;
  packet.ack = incomingPacket.ack;
}

void printIncomingReadings()
{
  Serial.println("INCOMING READINGS");
  Serial.print("Microhpone Direction:  ");
  Serial.println(incomingPacket.microphone_direction);
  Serial.print("Ultrasonic Direction:  ");
  Serial.println(incomingPacket.ultrasonic_distance);
  Serial.print("Left Mircophone:  ");
  Serial.println(incomingPacket.leftMic);
  Serial.print("Right Microphone:  ");
  Serial.println(incomingPacket.rightMic);
  Serial.print("Compass Heading:  ");
  Serial.println(incomingPacket.heading);
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
  NodeMcu_SoftSerial.begin(57600);
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
  // attachInterrupt(D5, receiveData, FALLING);
  // attachInterrupt(D6, masterCommand, RISING);
}

void loop()
{
  //Check if we are receiving anything from Arduino Central Command
  ReadIncoming();
  if(newData)
  {
    //Serial.println("receiving from Uno");
    //Serial.println(dataIn);
    convertDataIntoPacket(dataIn);
    populatePacketData();
    //displayData();
    //int bytes = esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    ready = true;
    newData = false;
  }
  if((millis() - prevMillis > interval))  //We may not need this as Sensor will have its own
  {
    prevMillis = millis();
    sendData = true;      //may not be needed
  }
  if(ready && !newData && sendData)   //SendData may not be needed
  {
    //Serial.println("We Sent the data to Master");  
    if(!received)  
    {
      esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
      //Serial.println(dataIn);
    }
    //String sendMessage = convertPacketToString();
    //Serial.println(sendMessage);
    //NodeMcu_SoftSerial.println(sendMessage);    
    ready = false;
    sendData = false;
    dataIn = "";
    c = 0;
  }


  if(received && !newData)
  {
    //printIncomingReadings();
   
    transferIncomingIntoPacket();
    //NodeMcu_SoftSerial.println("<3,5,0,1,3,2,0,3>");
    String sendMessage = convertPacketToString();
    //Serial.println(sendMessage);
    NodeMcu_SoftSerial.println(sendMessage);
    received = false;
    sendData = false;
  }
}

void ReadIncoming()
{
  while(NodeMcu_SoftSerial.available() > 0  && newData == false)
  {
    c = NodeMcu_SoftSerial.read();
    if(c=='>')
    {
      dataIn += c;
      newData = true;
      //Serial.print("We are here \n");
      break;
    }
    else
    {
      dataIn += c;
    }
  }
}

String convertPacketToString()
{
  String converter = "";
  converter += "<";
  converter += String(packet.microphone_direction);
  converter += ",";
  converter += String(packet.ultrasonic_distance);
  converter += ",";
  converter += String(packet.leftMic);
  converter += ",";
  converter += String(packet.rightMic);
  converter += ",";
  converter += String(packet.heading);
  converter += ",";
  converter += String(packet.start);
  converter += ",";
  converter += String(packet.manual);
  converter += ",";
  converter += String(packet.ack);
  converter += ">";
  return converter;
}

// void readIncomingData() {
//   bool recvInProgress = false;
//   //Serial.println("We are reading something");
//   while(nodeMCU.available() > 0 && newData == false)
//   {
//     char temp = nodeMCU.read();
//     if(recvInProgress == true)
//     {
//       if(temp != '>')
//       {
//         incomingData += String(temp);
//       }
//       else if(temp == '>')
//       {
//         incomingData += String(temp);
//         //Serial.println(incomingData);
//         newData = true;
//         recvInProgress = false;
//       }
//     }
//     else if(temp == '<')
//     {
//       recvInProgress = true;
//       incomingData += String(temp);
//     }

//   }
// }


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
      if(count >= sizeof(packetArray))
      {
        Serial.println("Error in packet size or conversion");
        break;
      }
      temp = "";
    }
    else
    {
      temp += ch;
    }
  }
}

void displayData()
{
  Serial.print("Packet Microhpone = ");
  Serial.print(packetArray[0]);
  Serial.println();
  Serial.print("Packet Ultrasonic = ");
  Serial.print(packetArray[1]);
  Serial.println();
  Serial.print("Packet Left Microphone Reading = ");
  Serial.print(packetArray[2]);
  Serial.println();
  Serial.print("Packet Right Microphone Reading = ");
  Serial.print(packetArray[3]);
  Serial.println();
  Serial.print("Packet Compass Heading = ");
  Serial.print(packetArray[4]);
  Serial.println();
  Serial.print("Packet Start = ");
  Serial.print(packetArray[5]);
  Serial.println();
  Serial.print("Packet Manual = ");
  Serial.print(packetArray[6]);
  Serial.println();
  Serial.print("Packet Ack = ");
  Serial.print(packetArray[7]);
  Serial.println();
}
void populatePacketData()
{
  packet.microphone_direction = packetArray[0];
  packet.ultrasonic_distance = packetArray[1];
  packet.leftMic = packetArray[2];
  packet.rightMic = packetArray[3];
  packet.heading = packetArray[4];
  packet.start = packetArray[5];
  packet.manual = packetArray[6];
  packet.ack = packetArray[7];
}