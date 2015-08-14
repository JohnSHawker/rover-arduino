#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

const int forwardPin = 12;
const int reversePin = 11;
const int rightPin = 10;
const int leftPin = 9;

float targetLat = 40.315070;
float targetLng = -111.659797;
bool arrivedAtTarget = false;
float targetDistanceTolerance = 0.00005;

// Magnetometer
// sda to A4
// scl to A5
const float directionTolerance = 10.0;

//GPS 
// 5v power
// tx to 3
// rx to 2
#define GPSECHO  false
const int gpsTxPin = 3;
const int gpsRxPin = 2; 
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);
Adafruit_GPS GPS(&gpsSerial);
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
bool usingInterrupt = false;

unsigned long myTimer;


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
  
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  delay(3000);
  
  //GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  
  useInterrupt(true);
  delay(1000);
  gpsSerial.println(PMTK_Q_RELEASE); // Ask for firmware version
  // END GPS

  resetTimer();
}

void loop()
{
  if (arrivedAtTarget) {
    Serial.println("already there");
    stopCar();
    return;
  }
  
  if (GPS.newNMEAreceived()) {
    Serial.println("new NMEA");
    if (!GPS.parse(GPS.lastNMEA())) {  // get latest data
      return;  // return if parsing fails.
    }
  }  
  
  if (!GPS.fix) {
    Serial.println("no fix");
    stopCar();
    return; // return if we don't have a fix yet.
  }
  
  Serial.println("going forward");
  goForward();  // If pass the above, we can go forward.
  
  float currentLat = GPS.latitudeDegrees;
  float currentLng = GPS.longitudeDegrees;
  float myHeading = getHeading();
  float targetBearing = relativeBearing(myHeading, magneticBearingTarget(currentLat, currentLng, targetLat, targetLng));

  if (abs(targetBearing) < directionTolerance) {
    Serial.println("go straight");
    turnNone();
  } else if (targetBearing < 0) {
    Serial.println("go right");
    turnRight();
  } else {
    Serial.println("go left");
    turnLeft();
  }
  
  if (nearTarget(currentLat, currentLng, targetLat, targetLng)) {
    Serial.println("near target");
    stopCar();
    arrivedAtTarget = true;
  }
  
  delay(20);
  
}

void onArrival() {
  stopCar();
  arrivedAtTarget = true;
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

void turnNone() 
{
  digitalWrite (rightPin, LOW);
  digitalWrite (leftPin, LOW);
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
  return directionDegrees(event.magnetic.y, event.magnetic.x);
}



// Timer 
void resetTimer(){
  myTimer=millis();
}

long timeElapsed(){
  return millis()-myTimer;
}

// Target

bool nearTarget(float myLat, float myLng, float targetLat, float targetLng){
  return distance(myLat, myLng, targetLat, targetLng) < targetDistanceTolerance;
}

// MATH

float magneticBearingTarget(float myLat, float myLng, float targetLat, float targetLng) {
  return directionRadians(targetLng - myLng, targetLat - myLat);
}

float relativeBearing(float myHeading, float targetBearing) {
  return targetBearing - myHeading;
}

float radiansToDegrees(float r) {
  return r * (180/M_PI);
}

float directionRadians(float x, float y) {
  return atan2(y, x);
}

float directionDegrees(float x, float y) {
  return radiansToDegrees(directionRadians(x, y));
}

float distance(float x1, float y1, float x2, float y2) {
  return pythagorean(x2 - x1, y2 - y1);
}

float pythagorean(float x, float y) {
  return sqrtf((x * x) + (y * y));
}  



// GPS Stuff

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
