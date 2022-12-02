//Test of Arduino to NodeMCU  - Arduino Perspective
#include <I2CTransfer.h>
#include <Packet.h>
#include <PacketCRC.h>
#include <SPITransfer.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>

SerialTransfer dataTransfer;
SoftwareSerial nodeMCUSerial(2,3);
int converterArray[5];
String sender = "";
bool ready = false;
bool sensorReady = false;
int compassX = -1;
int compassY = -1;
int side = -1;
struct __attribute__((packed)) sensor_data {
  int8_t microphone_direction = 3; //0 = middle, 1 = left, 2 = right, 3 = error
  uint16_t ultrasonic_distance = 0; // 0 = Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  bool manual = false; //False = Automatic, True = Manual Control
  bool ack = false; //False = Was not Received any response, True = Received Packet
};
struct sensor_data packet;
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
  if(Serial.available())
  {
    char temp = Serial.read();
    readCommand(temp);
  }
  if(!ready)
  {
    packet.microphone_direction = random(0,10);
    packet.ultrasonic_distance = random(0,3);
    sender = convertPacketToString();
    Serial.println(sender);
    nodeMCUSerial.println(sender);
    //sentSize = send_data_to_nodemcu();
    ready = true;
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
      ready = false;
      packet.start = true;
      packet.manual = false;
    case '1':
      packet.start = true;
      packet.manual = true;
  }
}


