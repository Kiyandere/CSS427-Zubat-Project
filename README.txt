1. Goal of Project Zubat:
  This fun project is to create a bat-like model that is "blind" and based on audio and ultrasonic sensor. There are two platforms that are labelled as Sensor and Motor
  Platforms. Sensor platform contains the following sensors: left/right mircophones, accelrometer/compass, SD Card Adapter(Recording), UltraSonic. The Motor Platform
  will contain a step motor to move sensor platform and LCD display for debugging. Both platforms will communicate wirelessly with one another using ESP8266 NodeMCU.
 
 
 Sensor (A) ----> ESP8266 NodeMCU 12-E (B) ----> ESP8266 NodeMCU 12-E (C) ----> Motor (D)
 
 Sensor (A) <---- ESP8266 NodeMCU 12-E (B) <---- ESP8266 NodeMCU 12-E (C) <---- Motor (D)
 
2. Sensor Plaform:
    Peripheral Device that is connected to the sensors.
  
3. Microphone Sensors:
   There are two sensors that handle the left and right side of the bat model. There are two global or local stored variables for left and right mircophones for 
   additional calculations in a step motor rotation. We will have an initial microphone integer that has been set to -1. We will set the integer to the following value: 
      0 = directly in front
      1 = Sound is on the left
      2 = Sound is on the right
     -1 = ERROR in sensors (Debug Mode/Initialization)
 
3. Compass Sensor:
   This sensor would be mounted in the center of the bat-like model. Based on the two local microphone variables of left and right, it would calculate the degrees needed
   to turn (NO LONGER WORKED ON). 
   
   [NEW] Once the Microphone Integer says that the sound source is in front of the sensor, record the XY coordinates and store onto the SD card.
   
4. Ultrasonic Sensor:
   There is a boolean value to indicate device state (0 = Off, 1 = ON). There is an integer value that indicates the general distance of object (to be recorded
   with the following value:
      0 = Device Disabled/Error
      1 = Close to Bat(within ultrasonic's detection)
      2 = Far from Bat(Outside ultrasonic's detection)
 
 5. Motor Platform:
    Central Device that is connected directly to the PC terminal. It has boolean variable called "Start" to begin the entire communication between the two platforms, 
    which means starting everything.
    
 6. Step Motor:
    Receives data of moving left and right from sensor platform. Rotates the machine in fixed degrees.
    
 7. LCD Display:
    Will only display if the Sensor says it is front, left, or right.
 
 8. Sensor Packet Data (Applied to all devices' programs):
    The packet will contain the following variables:
      A. Float x [DISCONTINUED]
      B. Float y [DISCONTINUED]
      C. int microphone_direction -----> 0 = middle, 1 = left, 2 = right, -1 = error
      D. int ultrasonic_distance -----> 0 = Device Disabled/Error, 1 = Close to Sensor, 2 = Far from Sensor
      E. bool start  -----> False = Not Sent/Awaiting, True = Begin Transmission/Sent
      F. bool manual -----> False = Automatic, True = Manual/Remote Control
      G. bool ACK -----> False = Was not Received any response, True = Received Packet
   
