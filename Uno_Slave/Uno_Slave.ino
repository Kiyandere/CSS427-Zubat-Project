//----------- Global Variable ----------
//Accel + Magnet
int magX, magY;
int accX, accY;

//Ultrasonic
long ultDuration;
int ultDistance;

//Microphone
int leftMicRead[20];
int rightMicRead[20];
int soundLocation = -1; //-1 error, 0 same, 1 left, 2 right

//stepper
const int stepsPerRevolution = 512;  // change this to fit the number of steps per revolution

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

//SD card pins TODO: might be moved to Master instead
#define CS 7
#define SCK 6
#define MOSI 5
#define Miso 4

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

/*Accel + Mag Reading
 * Z is not needed
 */
void magReading()
{
  sensors_event_t event;
  accel.getEvent(&event);
  magX = event.magnetic.x;
  magY = event.magnetic.y;
}
void accReading()
{
  sensors_event_t event;
  accel.getEvent(&event);
  accX = event.acceleration.x;
  accY = event.acceleration.y;
}

/*Microphone reader
 * This will also calculate the distance from the microphone and how far left/right the sound is
 * 
 * Will need to be adapt for outlier +/-
 * TODO: need research on how to calculate distance from sound
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

/* Store soundwave ADC into SD card
 *  will be send as .TXT files
 */
void sdSaver()
{
  
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
String packaging()
{
  
}

void loop() 
{
  
}
