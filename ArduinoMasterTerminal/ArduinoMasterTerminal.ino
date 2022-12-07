//Test of Arduino to NodeMCU  - Arduino Perspective
#include <I2CTransfer.h>
#include <Packet.h>
#include <PacketCRC.h>
#include <SPITransfer.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

SoftwareSerial Arduino_SoftSerial(10,11);
int packetArray[8];

char c;
String dataIn;
String sender = "";
String incomingData = "";
String curData = "";
bool newData = false;
bool ready = false;
volatile bool sending = false;
volatile bool receiving = false;
bool sensorReady = false;
int compassX = -1;
int compassY = -1;
int side = -1;
struct __attribute__((packed)) sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = ultrasonic_distance Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  bool leftMic = 0;  // 0 = Never asked or request, anything else is from manual
  bool rightMic = 0; // 0 = Never asked or request, anything else is from manual
  int16_t heading = 0; // 0 = Never asked or request, anything else is from manual
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  int8_t manual = 0; //0: auto , 1: motor, 2: distance, 3: L mic, 4: R mic, 5: compass
  bool ack = false; //False = Was not Received any response, True = Received Packet
};

struct sensor_data packet;
struct sensor_data incomingPacket;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Arduino_SoftSerial.begin(57600);
  // attachInterrupt(6, masterCommand, RISING);
  // attachInterrupt(5, receiveData, FALLING);
  //dataTransfer.begin(nodeMCUSerial);
  //packet.microphone_direction = random(0, 3);
  //packet.ultrasonic_distance = random(0,3);
  //delay(2000);
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

void readCommand(char command)
{
  switch(command){
    case '0':
      ready = true;
      packet.start = true;
      packet.manual = 0;
      break;
    case '1':
      ready = true;
      packet.start = true;
      packet.manual = 1;
      break;
    case '2':
      ready = true;
      packet.start = true;
      packet.manual = 2;
      break;
    case '3':
      ready = true;
      packet.start = true;
      packet.manual = 3;
      break;
    case '4':
      ready = true;
      packet.start = true;
      packet.manual = 4;
      break;
    case '5':
      ready = true;
      packet.start = true;
      packet.manual = 5;
      break;
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

void populateIncomingPacketData()
{
  incomingPacket.microphone_direction = packetArray[0];
  incomingPacket.ultrasonic_distance = packetArray[1];
  incomingPacket.leftMic = packetArray[2];
  incomingPacket.rightMic = packetArray[3];
  incomingPacket.heading = packetArray[4];
  incomingPacket.start = packetArray[5];
  incomingPacket.manual = packetArray[6];
  incomingPacket.ack = packetArray[7];
}

void loop() {
  // put your main code here, to run repeatedly:
  checkCommand();
  // if(ready && !newData)
  // {
  //   sender = convertPacketToString();
  //   //Serial.println(packet.manual);
  //   Arduino_SoftSerial.println(sender);
  //   //sentSize = send_data_to_nodemcu();
  //   ready = false;
  // }
  ReadIncoming();
  if(newData == true)
  {
    convertDataIntoPacket(dataIn);
    displayData();
    newData = false;
    //nodeMCUSerial.write(incomingData.c_str());
    c = 0;
    dataIn = "";
  }
  
  //When ready to be sent, place packet here
}

void checkCommand()
{
  while(Serial.available() > 0)
  {
    char temp = Serial.read();
    readCommand(temp);
    sender = convertPacketToString();
    break;
  }
  if(ready)
  {
    Arduino_SoftSerial.print(sender);
    ready = false;
  }
}
void ReadIncoming()
{
  while(Arduino_SoftSerial.available() > 0  && newData == false)
  {
    c = Arduino_SoftSerial.read();
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

void convertDataIntoPacket(String data)
{
  String temp = "";
  int count = 0;
  
  for(int i = 0; i < data.length(); i++)
  {
    char ch = data.charAt(i);
    if(ch == '<')
    {
      continue;
    }
    else if(ch == '>')
    {
      //Serial.println(temp);
      packetArray[count] = temp.toInt();
      count++;
      temp = "";
    }
    else if(ch == ',')
    {
      //Serial.println(temp);
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
