const int GreenSignal = A3;
const int BlueSignal = A0;
const int greenLED = A1;
const int yellowLED = A2;

class LED
{
public:
  LED(int pin)
  {
    this->pin = pin;
    pinMode(this->pin,OUTPUT);
    digitalWrite(this->pin,LOW);
  }

  LED::~LED()
  {

  }

private:
  int pin;

public:
  void lighten()
  {
    digitalWrite(this->pin,HIGH);
  }

  void extinguish()
  {
    digitalWrite(this->pin,LOW);
  }

  void flicker()//这里使用了delay(),尽量不要使用
  {
    digitalWrite(this->pin,HIGH);
    delay(100);
    digitalWrite(this->pin,LOW);
    delay(100);
    digitalWrite(this->pin,HIGH);
    delay(100);
    digitalWrite(this->pin,LOW);
    delay(100);
  }
};

LED GreenLED(greenLED);
LED YellowLED(yellowLED);

void setup() 
{
  pinMode(GreenSignal, INPUT);
  pinMode(BlueSignal, INPUT);

  GreenLED.flicker();
  YellowLED.flicker();
}

void loop() 
{
  if(digitalRead(GreenSignal) == 1 && digitalRead(BlueSignal) == 0)
  {
    GreenLED.lighten();
    YellowLED.extinguish();
  }
  else if(digitalRead(GreenSignal) == 0 && digitalRead(BlueSignal) == 1)
  {
    GreenLED.extinguish();
    YellowLED.lighten();
  }
}
