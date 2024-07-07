const int triggerPin = A1;
const int echoPin = A2;
//这里triggerPin与GreenLED冲突了，可以考虑更换LED引脚

class Ultrasonic
{
public:
  Ultrasonic(int trigger, int echo)
  {
    this->trigger = trigger;
    this->echo = echo;

    pinMode(this->trigger, OUTPUT);
    pinMode(this->echo, INPUT);
  }
  ~Ultrasonic()
  {

  }

private:
  int trigger;
  int echo;

  double timeToDistance(long microseconds)
  {
    return microseconds / 58.2;
  }

public:
  double getDistance()
  {
    digitalWrite(this->trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trigger, LOW);

    long duration = pulseIn(this->echo, HIGH);
    double distance = timeToDistance(duration);

    return distance;
  }
};

Ultrasonic ultrasonic(triggerPin, echoPin);

void setup() 
{
  Serial.begin(9600);
}

void test(double distance)
{
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  delay(100);
}

void loop() 
{
  double distance = ultrasonic.getDistance();

  // if(distance < 50.0)
  // {
    
  // }
  // else
  // {
    
  // }

  test(distance);
}

