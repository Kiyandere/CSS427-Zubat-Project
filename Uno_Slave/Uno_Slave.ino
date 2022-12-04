#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>


//------------ Imports -----------
/*Will need to inport Adafruit stuff*/
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>


#include <SPI.h>
#include <SD.h>
#include <Stepper.h>

//----------- Global Variable ----------
//Accel heading
int accHeading;

//Ultrasonic
long ultDuration;
int ultDistance;

//Microphone
int leftMicRead[10];
int rightMicRead[10];
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
};
struct sensor_data packet;
struct sensor_data incomingPacket;

//----------------------------- Setup Func -------------------------------
//WIFI module pins
#define rxPin 10
#define txPin 11
SoftwareSerial unoSerial = SoftwareSerial(rxPin, txPin);

//Ultrasonic pins
#define echoPin 5
#define trigPin 6

//Microphone pins
#define micLPin A0
#define micRPin A1


//Setup Accel Unquique ID
Adafruit_LSM303_Mag_Unified accel = Adafruit_LSM303_Mag_Unified(54321);

//stepper setup
Stepper stepper(stepsPerRevolution, 8, 10, 9, 11);
int startLocation = 1;
int rightTurn = 0;
int leftTurn = 0;

// Setup func for Accel
void accelSetup()
{
  //Check of the accel is on
  if (!accel.begin())
  {
    Serial.println("Error in Accel");
    while(1)
      ;
  }
  accel.enableAutoRange(true);

}

//Setip func for Ultrasonic
void ultraSetup()
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
  accelSetup();
  ultraSetup();

  //setting stepper speed
  stepper.setSpeed(10);
}


//----------------------------- Active Func -------------------------------

/*Accel + Mag Reading
 * Z is not needed
 */
void heading()
{
  sensors_event_t event;
  accel.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  accHeading = (atan2(event.magnetic.y, event.magnetic.z) * 180) / Pi;

  // Normalize to 0-360
  if (accHeading < 0) {
    accHeading = 360 + accHeading;
  }
}

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
  for(int i = 0; i < 10; i++)
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
  rRead /= 10;
  lRead /= 10;

  //save
  lMic = lRead;
  rMic = rRead;

  //check which side have the biggest reading
  if ((rRead < lRead) && (lRead - rRead) > 2) //left side
  {
    soundLocation = 1;

  }
  else if ((rRead > lRead) && (rRead - lRead) > 2) //right side
  {
    soundLocation = 2;

  }
  else if ((lRead - rRead) <= 2 || (rRead - lRead) <= 2) //same
  {
    soundLocation = 0;

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
      packageArray[count] = temp.toInt();
      count++;
      temp = "";
    }
    else
    {
      temp += ch;
    }
  }

  //Adding data into array
  packageArray[0] = soundLocation;
  packageArray[1] = ultDistance;
  packageArray[2] = lMic;
  packageArray[3] = rMic;
  packageArray[4] = accHeading;
  packageArray[5] =  false; //start
  packageArray[6] =  0; //manual
  packageArray[7] =  true; //ack

  displayPackageData();

  //populate Pocket Data
  packet.microphone_direction = packageArray[0];
  packet.ultrasonic_distance = packageArray[1];
  packet.start = packageArray[2];
  packet.manual = packageArray[3];
  packet.ack = packageArray[4];

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

//----------------------------- Debug Func -------------------------------

void displayPackageData()
{
  Serial.print("Packet Microhpone = ");
  Serial.print(packageArray[0]);
  Serial.println();
  Serial.print("Packet Ultrasonic = ");
  Serial.print(packageArray[1]);
  Serial.println();
  Serial.print("Packet Start = ");
  Serial.print(packageArray[2]);
  Serial.println();
  Serial.print("Packet Manual = ");
  Serial.print(packageArray[3]);
  Serial.println();
  Serial.print("Packet Ack = ");
  Serial.print(packageArray[4]);
  Serial.println();
}

//----------------------------- Running Func -------------------------------

void loop() 
{
  
}
