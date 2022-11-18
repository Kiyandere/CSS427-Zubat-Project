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
#define rxPin 5
#define txPin 6 

//Mega NodeMCU Broadcast MAC Address that we are sending, UDP
uint8_t broadcastAddress[] = {0x50, 0x02, 0x91, 0xDC, 0xC0, 0x34};
//Set a delay timer (Current Delay Time is 10)
const long interval = 10000; 
unsigned long previousMillis = 0;    // will store last time Sensor Reading was updated 

//This is what we are sending to the Sensor Platform
String motorVal = "Moved Platform";
String ack_message = "ACKNOWLEDGED";

//What we are receiving on the Motor Platform
float incomingX;
float incomingY;
String incomingMessage;

//This is a struct message that will be received for Mega
typedef struct message {
  String acknowledgement;
  String motor;
} message;

typedef struct struct_message {
  float x;
  float y;
  String slave_message;
} struct_message;

message motorReadings;
struct_message incomingReadings;

String converter(uint8_t *str){
    return String((char *)str);
}
const char* convertStringToChar(String target)
{
  return target.c_str();
}
uint8_t* convertStringToBuffer(String target)
{
  return (uint8_t*)target.c_str();
}
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
  incomingX = incomingReadings.x;
  incomingY = incomingReadings.y;
  incomingMessage = incomingReadings.slave_message;
}


void printIncomingReadings()
{
  Serial.println("INCOMING READINGS");
  Serial.print("Position X: ");
  Serial.print(incomingX);
  Serial.println(" º");
  Serial.print("Position Y: ");
  Serial.print(incomingY);
  Serial.println(" º");
  Serial.print("Slave Message: ");
  Serial.println(incomingMessage);
}
void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println();
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

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    //Get Motor Movement readings
    //getReadings();

    //Set values to send
    motorReadings.acknowledgement = ack_message;
    motorReadings.motor = motorVal;
    
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &motorReadings, sizeof(motorReadings));

    // Print incoming readings
    printIncomingReadings();
  }
}
