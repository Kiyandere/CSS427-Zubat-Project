//------------------------------- Imports -------------------------------
/*Will need to inport Adafruit stuff*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <SPI.h>
#include <SD.h>
#include <Stepper.h>

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
#define rxPin 10 //Pin to connect to D6
#define txPin 11 //Pin to connect to D7
//------------------------------- Wifi stuff -------------------------------
//struct for packet
struct sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = ultrasonic_distance Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  bool leftMic = false;
  bool rightMic = false;
  int16_t heading = 0;
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  int8_t manual = 0; //0: auto , 1: motor, 2: distance, 3: L mic, 4: R mic, 5: compass
  bool ack = false; //False = Was not Received any response, True = Received Packet
};
struct sensor_data packet;
struct sensor_data incomingPacket;

//packet array for wifi
int packetArray[8];

//wifi global variable
String incomingData = "";
String curData = "";


//wifi checkpoint
bool wait = false;
bool ready = false;
bool received = false;

//Setup softwareSerial
SoftwareSerial nodeMCU(rxPin, txPin);

//MAC address: 
//uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xCF, 0x83};
uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xC0, 0x34};

//------------------------------- Global Variable -------------------------------
//stepper
const int stepsPerRevolution = 512;  // change this to fit the number of steps per revolution

//local data
int micLocal = -1;
int ultraLocal = 0;
bool leftMicLocal = false;
bool rightMicLocal = false;
int headingLocal = 0;
bool start = false;
int manual = 0;
bool ack = false;

//------------------------------- Pins Define -------------------------------
//Ultrasonic pins
#define echoPin 5
#define trigPin 6

//Microphone pins
#define micLPin 13
#define micRPin 12

//Stepper pins
#define in1 3
#define in2 2
#define in3 1
#define in4 0

//------------------------------- Setup -------------------------------
//Setup Accel Unquique ID
Adafruit_LSM303_Mag_Unified accel = Adafruit_LSM303_Mag_Unified(54321);

//Set up the Stepper motor variable
Stepper stepper(stepsPerRevolution, in1, in2, in3, in4);
void setup() {
  //setup serial first
  Serial.begin(115200);

  //Accel for heading
  if (!accel.begin())
  {
    Serial.println("Error in Accel");
    while(1)
      ;
  }
  accel.enableAutoRange(true);

  //Ultrasonic for distance
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);

  //Mic setup
  pinMode(micLPin, INPUT);
  pinMode(micRPin, INPUT);

  //setting stepper speed
  stepper.setSpeed(20);

  //wifi setup
  Serial.begin(115200);
  nodeMCU.begin(115200);
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

//------------------------------- Active Func -------------------------------

/*Heading Reading
 * y and z is used since placement
 */
void headingReading()
{
  sensors_event_t event;
  accel.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,z
  int accHeading = (atan2(event.magnetic.y, event.magnetic.z) * 180) / Pi;

  // Normalize to 0-360
  if (accHeading < 0) {
    accHeading = 360 + accHeading;
  }

  //save into packet
  headingLocal = accHeading;
}

/*Ultrasonic reader
 * This calulate the distance from the object once the rotation have been made
 */
void ultraReading()
{
  //clear trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  int ultDuration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  int ultDistance = ultDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  //Save into packet
  ultraLocal = ultDistance;
}

/*Stepper
 * Turn left or right by 45 degree
 * Also have a reset function
 */
void stepperRightTurn()
{
  int rightTurn = (stepsPerRevolution);
  stepper.step(rightTurn);
  delay(500);
}
void stepperLeftTurn()
{
  int leftTurn = (-stepsPerRevolution);
  stepper.step(leftTurn);
  delay(500);
}
void stepperReset()
{
  if (leftMicLocal)  //turn Right
  {
    stepperRightTurn();
  }
  if (rightMicLocal) //turn left
  {
    stepperLeftTurn();
  }
}

/*Microphone reader
 * Detect left or right
 */
void micReading()
{
  //Local variable
  boolean rightVal = 0;
  boolean leftVal = 0;

  //read it
  rightVal = digitalRead(micRPin);
  leftVal = digitalRead(micLPin);

  if (rightVal == HIGH && leftVal == LOW) //right
  {
    leftMicLocal = false;
    rightMicLocal = true;
    micLocal = 2;
    stepperRightTurn(); //turn right by 45 degree
  }
  else if (rightVal == LOW && leftVal == HIGH) //left
  {
    leftMicLocal = true;
    rightMicLocal = false;
    micLocal = 1;
    stepperLeftTurn(); //turn left by 45 degree
  }
  else //same
  {
    leftMicLocal = true;
    rightMicLocal = true;
    micLocal = 0;
  }
}



//------------------------------- Wifi stuff -------------------------------
/*Wifi
 * Check if it's ready or received then run accordingly
 */
void wifiRun()
{
  if(nodeMCU.available() != 0)
  {
    readIncomingData();
    //Serial.println(wait);
  }

  //sending to master?
  if(ready)
  {
    Serial.println("Sending from master");
    esp_now_send(broadcastAddress, (uint8_t *) &packet, sizeof(packet));
    ready = false;
  }

  //when getting stuff
  if(received)
  {
    printIncomingReadings();
    cli();
    transferIncomingIntoPacket();
    sei();
    String sendMessage = convertPacketToString();
    nodeMCU.println(sendMessage);
    received = false;
  }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) 
{
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
    ready = true; //Re-Attempt
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  memcpy(&incomingPacket, incomingData, sizeof(incomingPacket));
  Serial.print("Bytes received: ");
  Serial.println(len);
  received = true;
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

void printIncomingReadings()
{
  Serial.println("INCOMING READINGS");
  Serial.print("Microhpone Direction:  ");
  Serial.println(incomingPacket.microphone_direction);
  Serial.print("Ultrasonic Distance:  ");
  Serial.println(incomingPacket.ultrasonic_distance);
  Serial.print("Left Mic Reading:  ");
  Serial.println(incomingPacket.leftMic);
  Serial.print("Right Mic Reading:  ");
  Serial.println(incomingPacket.rightMic);
  Serial.print("Heading:  ");
  Serial.println(incomingPacket.heading);
  Serial.print("Start Set to:  ");
  Serial.println(incomingPacket.start);
  Serial.print("Manual Set to:  ");
  Serial.println(incomingPacket.manual);
  Serial.print("Ack:  ");
  Serial.println(incomingPacket.ack);
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
  converter += ";";
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
  //displayData();
  populatePacketData();
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


void loop() {
  /*
  * Start buy running the wifi stuff
  * Start to run all the sensor reading
  * Reset the stepper when done
  */
  wifiRun();
  headingReading();
  ultraReading();
  micReading();
  stepperReset();

  //we can cheat on the manual command
  //have the slave send all data into master
  //but if manual is on, dont print that data unless
  //the user asked for it
}
