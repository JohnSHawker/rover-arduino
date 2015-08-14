const int forwardPin = 12;
const int reversePin = 11;
const int rightPin = 10;
const int leftPin = 9;

void setup()
{
  
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  
  Serial.begin(9600);
  stopCar();
}

void loop()
{
goForward();
delay(500);
stopCar();
delay(500);
goReverse();
delay(500);
stopCar();
delay(500);
turnRight();
delay(500);
stopCar();
delay(500);
turnLeft();
delay(500);
stopCar();
delay(500);
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

