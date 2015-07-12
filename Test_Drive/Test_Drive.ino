#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const int forwardPin = 12;
const int reversePin = 11;
const int rightPin = 10;
const int leftPin = 9;

const float directionTolerance = 10.0;
unsigned long timer;


void setup()
{
  
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.begin(9600);
  
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  Serial.println("I totally work!");
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  goForward();
  turnLeft();
  delay(3000);
  resetTimer();
}

void loop()
{
//  Serial.println("Starting loop");
  float degreesFromNorth = getHeading();
  if (timeElapsed() > 10000 ) {
    Serial.println("stopping due to time");
    stopCar();
  }
  
  if (abs(degreesFromNorth) < 10) {
    Serial.println("stopping due to north");
    stopCar();
  }
  delay(250);
}

//Car commands

void goForward()
{
  digitalWrite (reversePin, LOW);
  digitalWrite (forwardPin, HIGH);

}

void goReverse()
{
  digitalWrite (forwardPin, LOW);
  digitalWrite (reversePin, HIGH);

}

void turnRight()
{
  digitalWrite (leftPin, LOW);
  digitalWrite (rightPin, HIGH);

}

void turnLeft()
{
  digitalWrite (rightPin, LOW);
  digitalWrite (leftPin, HIGH);

}

void stopCar()
{
  digitalWrite (reversePin, LOW);
  digitalWrite (forwardPin, LOW);
  digitalWrite (rightPin, LOW);
  digitalWrite (leftPin, LOW);

}

//Compass

float getHeading ()
{
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  return heading * 180/M_PI; 
}

// Timer 
void resetTimer(){
  timer=millis();
}

long timeElapsed(){
  return millis()-timer;
}
