/*Android OI Interface

This is an interface to make it a *little* for a more complex system
to communicate with the iRobot Create. Think of it like subconcious 
motor control

*/

#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <stdin.h>

SoftwareSerial oiSerial(/*pins here*/) // Recieve pin, transmit pin

void setup()
{
  Serial.begin(57600); //change to good rate for RPi
  
  oiSerial.begin(4800); //Change to robot rate
}

void loop()
{
  
  
}
