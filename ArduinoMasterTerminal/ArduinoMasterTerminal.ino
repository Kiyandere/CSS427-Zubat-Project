//Test of Arduino to NodeMCU  - Arduino Perspective
#include <I2CTransfer.h>
#include <Packet.h>
#include <PacketCRC.h>
#include <SPITransfer.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

SerialTransfer dataTransfer;
SoftwareSerial nodeMCUSerial(2,3);
int packetArray[8];
String sender = "";
bool ready = false;
bool sensorReady = false;
int compassX = -1;
int compassY = -1;
int side = -1;
struct __attribute__((packed)) sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = ultrasonic_distance Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  int16_t leftMic = 0;  // 0 = Never asked or request, anything else is from manual
  int16_t rightMic = 0; // 0 = Never asked or request, anything else is from manual
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
  nodeMCUSerial.begin(115200);
  //dataTransfer.begin(nodeMCUSerial);
  //packet.microphone_direction = random(0, 3);
  //packet.ultrasonic_distance = random(0,3);
  //delay(2000);
}


void loop() {
  // put your main code here, to run repeatedly:
  //uint16_t sentSize = 0;
  // if(Serial.available() > 0)
  // {
  //   int input = Serial.parseInt();
  //   Serial.println("Sending....");
  //   if(input == 1)
  //   {
  //     ready = true;
  //   }
  // }
  if(nodeMCUSerial.available())
  {
    //Store Data into SD Card and print out incoming data from Node Side
  }
  if(Serial.available())
  {
    char temp = Serial.read();
    readCommand(temp);
  }
  if(ready)
  {
    packet.microphone_direction = random(0,10);
    packet.ultrasonic_distance = random(0,3);
    sender = convertPacketToString();
    Serial.println(sender);
    nodeMCUSerial.println(sender);
    //sentSize = send_data_to_nodemcu();
    ready = false;
    // Serial.print("Bytes Sent to NodeMCU:  ");
    // Serial.print(sentSize);
    // Serial.println();
    // Serial.println(packet.microphone_direction);
  }
  //When ready to be sent, place packet here
}


String convertPacketToString()
{
  String converter = "";
  converter += "<";
  converter += String(packet.microphone_direction);
  converter += ",";
  converter += String(packet.ultrasonic_distance);
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
    case '1':
      ready = true;
      packet.start = true;
      packet.manual = 1;
    case '2':
      ready = true;
      packet.start = true;
      packet.manual = 2;
    case '3':
      ready = true;
      packet.start = true;
      packet.manual = 3;
    case '4':
      ready = true;
      packet.start = true;
      packet.manual = 4;
    case '5':
      ready = true;
      packet.start = true;
      packet.manual = 5;
  }
}

void readIncomingData()
{
  while()
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
  incomingPacket.microphone_direction = packetArray[0];
  incomingPacket.ultrasonic_distance = packetArray[1];
  incomingPacket.microphone_direction = packetArray[2];
  incomingPacket.ultrasonic_distance = packetArray[3];
  incomingPacket.microphone_direction = packetArray[4];
  incomingPacket.start = packetArray[5];
  incomingPacket.manual = packetArray[6];
  incomingPacket.ack = packetArray[7];
}


