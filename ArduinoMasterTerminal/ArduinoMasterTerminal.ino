//Test of Arduino to NodeMCU  - Arduino Perspective
#include <I2CTransfer.h>
#include <Packet.h>
#include <PacketCRC.h>
#include <SPITransfer.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

SoftwareSerial Arduino_SoftSerial(10,11);
int packetArray[8];
int ackCount[7];
int commandCount[7];
int TCount = 0;
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
struct sensor_data basePacket;
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

String convertBasePacketToString()
{
  String converter = "";
  converter += "<";
  converter += String(basePacket.microphone_direction);
  converter += ",";
  converter += String(basePacket.ultrasonic_distance);
  converter += ",";
  converter += String(basePacket.leftMic);
  converter += ",";
  converter += String(basePacket.rightMic);
  converter += ",";
  converter += String(basePacket.heading);
  converter += ",";
  converter += String(basePacket.start);
  converter += ",";
  converter += String(basePacket.manual);
  converter += ",";
  converter += String(basePacket.ack);
  converter += ">";
  return converter;
}

void readCommand(char command)
{
  switch(command){
    case '0':
      ready = false;
      basePacket.start = true;
      basePacket.manual = 0;
      TCount = 0;
      commandCount[0] = 0;
      commandCount[1] = 0;
      commandCount[2] = 0;
      commandCount[3] = 0;
      commandCount[4] = 0;
      commandCount[5] = 0;
      ackCount[0] = 0;
      ackCount[1] = 0;
      ackCount[2] = 0;
      ackCount[3] = 0;
      ackCount[4] = 0;
      ackCount[5] = 0;
      break;
    case '1':
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 1;
      break;
    case '2':
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 2;
      break;
    case '3':
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 3;
      break;
    case '4':
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 4;
      break;
    case '5':
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 5;
      break;
    default:
      ready = true;
      //basePacket.start = true;
      basePacket.manual = 0;
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
  packet.microphone_direction = packetArray[0];
  packet.ultrasonic_distance = packetArray[1];
  packet.leftMic = packetArray[2];
  packet.rightMic = packetArray[3];
  packet.heading = packetArray[4];
  packet.start = packetArray[5];
  packet.manual = packetArray[6];
  packet.ack = packetArray[7];
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
    //displayData();
    Serial.print("--------------");
    Serial.println(dataIn);
    populateIncomingPacketData();
    ackCount[packet.manual] += 1;
    displayDataToHuman();
    newData = false;
    Serial.println("--------------");
    //Serial.println("Incoming Data:");
    
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
    if(temp == '\n')
    {
      break;      
    }
    readCommand(temp);
    //Serial.print(basePacket.manual);
    sender = convertBasePacketToString();
    break;
  }
  if(ready)
  {
    TCount++;
    commandCount[basePacket.manual]++;
    Serial.print("T Command Sent = ");
    Serial.println(TCount);
    Serial.print("Command Sent = ");
    Serial.println(commandCount[basePacket.manual]);
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
void displayDataToHuman()
{
  int manual = packet.manual;

  switch(manual)
  {
    case 1:
      Serial.print("MD = ");
      Serial.println(packet.microphone_direction);
      Serial.print("Ack Count = ");
      Serial.println(ackCount[manual]);
      break;
    case 2:
      Serial.print("LMic = ");
      Serial.println(packet.leftMic);
      Serial.print("Ack");
      Serial.println(ackCount[manual]);
      break;
    case 3:
      Serial.print("RMic = ");
      Serial.println(packet.rightMic);
      Serial.print("Ack Count = ");
      Serial.println(ackCount[manual]);
      break;
    case 4:
      Serial.print("Heading = ");
      Serial.println(packet.heading);
      Serial.print("Ack Count = ");
      Serial.println(ackCount[manual]);
      break;
    case 5:
      Serial.print("Ultrasonic = ");
      Serial.println(packet.ultrasonic_distance);
      Serial.print("Ack Count = ");
      Serial.println(ackCount[manual]);
      break;
    default:
      Serial.print("MD = ");
      Serial.println(packet.microphone_direction);
      Serial.print("LMic = ");
      Serial.println(packet.leftMic);
      Serial.print("RMic = ");
      Serial.println(packet.rightMic);
      Serial.print("Heading = ");
      Serial.println(packet.heading);
      break;
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
