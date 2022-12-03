//------------ Imports -----------
/*Will need to inport Adafruit stuff*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <SPI.h>
#include <SD.h>
#include <Stepper.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//----------- Global Variable ----------
//Accel heading
int accHeading;

//Ultrasonic
long ultDuration;
int ultDistance;

//Microphone
int leftMicRead[20];
int rightMicRead[20];
int lMic;
int rMic;
int soundLocation = -1; //-1 error, 0 same, 1 left, 2 right

//stepper
const int stepsPerRevolution = 512;  // change this to fit the number of steps per revolution

//Package array
int packageArray[8];

//struct
struct sensor_data {
  int8_t microphone_direction = -1; //0 = middle, 1 = left, 2 = right, -1 = error
  int8_t ultrasonic_distance = 0; // 0 = ultrasonic_distance Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
  int16_t leftMic = 0;
  int16_t rightMic = 0;
  int16_t heading = 0;
  bool start = false; //False = Not Sent/Awaiting, True = Begin Transmission/Sent
  int8_t manual = 0; //0: auto , 1: motor, 2: distance, 3: L mic, 4: R mic, 5: compass
  bool ack = false; //False = Was not Received any response, True = Received Packet
}
struct sensor_data packet;
struct sensor_data incomingPacket;

//----------------------------- Setup Func -------------------------------
//WIFI module pins
#define rxPin 10
#define txPin 11
SoftwareSerial unoSerial = SoftwareSerial(rxPin, txPin);

//Ultrasonic pins
#define echoPin 8
#define trigPin 9

//Microphone pins
#define micLPin A1
#define micRPin A0


//Setup Accel Unquique ID
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

//stepper setup
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);
int startLocation = 1;
int rightTurn = 0;
int leftTurn = 0;

// Setup func for Accel
bool accelSetup()
{
  //Check of the accel is on
  if (!accel.begin())
  {
    Serial.println("Error in Accel");
    return false;
  }
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);
  return true;
}

//Setip func for Ultrasonic
bool ultraSetup()
{
  //Setup pins
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);
}

// Master Setup
void setup() 
{
  //setup serial first
  Serial.begin(115200);

  //check if something is wrong
  if (!accelSetup)
  {
    Serial.println("E: Accel");

  }
  if (!ultraSetup())
  {
    Serial.println("E: Ultra");
  }

  //setting stepper speed
  stepper.setSpeed(10);
}


//----------------------------- Active Func -------------------------------

// /*Accel + Mag Reading
//  * Z is not needed
//  */
// void magReading()
// {
//   sensors_event_t event;
//   accel.getEvent(&event);
//   magX = event.magnetic.x;
//   magY = event.magnetic.y;
// }
// void accReading()
// {
//   sensors_event_t event;
//   accel.getEvent(&event);
//   accX = event.acceleration.x;
//   accY = event.acceleration.y;
// }

/*Microphone reader
 * This will also calculate the distance from the microphone and how far left/right the sound is
 * 
 */
void micReading()
{
  //local variable
  int rRead = 0;
  int lRead = 0;
  int ranalread, lanalread;

  //add 100 sample then find the average
  for(int i = 0; i < 20; i++)
  {
    //even localer variable
     ranalread = analogRead(micRPin);
     lanalread = analogRead(micLPin);

    //right side
    rRead += ranalread;
    rightMicRead[i] = ranalread;

    //left side
    lRead += lanalread;
    leftMicRead[i] = lanalread;
  }
  rRead /= 20;
  lRead /= 20;

  //save


  //check which side have the biggest reading
  if ((rRead < lRead) && (lRead - rRead) > 10) //left side
  {
    soundLocation = 1;
    delay(500);
  }
  else if ((rRead > lRead) && (rRead - lRead) > 10) //right side
  {
    soundLocation = 2;
    delay(500);
  }
  else if ((rRead > lRead) && (rRead - lRead) > 10) //same
  {
    soundLocation = 0;
    delay(500);
  }
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
  ultDuration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  ultDistance = ultDuration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}


/* Stepper
 *  Turn left or right by 15 degree
 */
 void stepperRightTurn()
 {
  rightTurn = (stepsPerRevolution);
  stepper.step(rightTurn);
  delay(500);
 }
 
 void stepperLeftTurn()
 {
  leftTurn = (-stepsPerRevolution);
  stepper.step(leftTurn);
  delay(500);
 }


/*Reset
 * This reset every value at the moment
 */
void reset()
{
  //Ultrasonic
  ultDuration = 0;
  ultDistance = 0;
  
  //Microphone
  soundLocation = -1;
  for (int i = 0; i < 20; i++)
  {
    leftMicRead[i] = 0;
    rightMicRead[i] = 0;
  }

  //stepper
  rightTurn = 0;
  leftTurn = 0;
}

/*Packaging
 * This package all the necessary data to send the goods over
 */
void packaging(String data)
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

  //Adding data into array
  packArray[0] = soundLocation;
  packArray[1] = ultDistance;
  packArray[2] = lMic;
  packArray[3] = rMic;
  packArray[4] = accHeading;
  packArray[5] =  false; //start
  packArray[6] =  0; //manual
  packArray[7] =  true; //ack

  displayPackageData();

  //populate Pocket Data
  packet.microphone_direction = packArray[0];
  packet.ultrasonic_distance = packArray[1];
  packet.start = packArray[2];
  packet.manual = packArray[3];
  packet.ack = packArray[4];

}

//----------------------------- Debug Func -------------------------------

void displayPackageData()
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

//----------------------------- Running Func -------------------------------

void loop() 
{
  
}
