//----------- Global Variable ----------
//Accel + Magnet
int magX, magY;
int accX, accY;

//Ultrasonic
long ultDuration;
int ultDistance;


//------------ Imports -----------
/*Will need to inport Adafruit stuff*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>



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


// Setup func for Accel
bool accelSetup()
{
  //Check of the accel is on
  if (!accel.begin())
  {
    Serial.println("Error in Accel");
    return false
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
  Serial.begin(9600);

  //check if something is wrong
  if (!accelSetup)
  {
    Serial.println("E: Accel");
    System.exit(1);
  }
  if (!ultraSetup())
  {
    Serial.println("E: Ultra");
    System.exit(1);
  }
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
  int rRead;
  int lRead;

  //add 100 sample then find the average
  for(int i = 0; i < 100; i++)
  {
    rRead += analogRead[micRPin];
    lRead += analogRead[micLPin];
  }
  rRead /= 100;
  lRead /= 100;

  //check which side have the biggest reading
  if (rRead < lRead) //left side
  {
    
  }
  else if (rRead > lRead) //right side
  {
    
  }
  else //same
  {
    
  }
}

/*Ultrasonic reader
 * This calulate the distance
 */

void loop() 
{
  
}
