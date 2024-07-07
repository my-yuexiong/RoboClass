const int rightSignal1 = A4;
const int leftSignal1 = A5;
const int rightSignal2 = A6;
const int leftSignal2 = A7;

//A1 -- green
//2 -- yellow
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

//4 -- Beep
class Beep
{
public:
  Beep(int pin)
  {
    this->pin = pin;
    pinMode(this->pin,OUTPUT);
    digitalWrite(this->pin,LOW);
  }
  ~Beep()
  {

  }

private:
  int pin;

public:
  void open()
  {
    digitalWrite(this->pin,HIGH);
  }

  void close()
  {
    digitalWrite(this->pin,LOW);
  }

  void flicker()
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

class Infrared
{
public:
  Infrared(int rightPin1, int leftPin1, int rightPin2, int leftPin2)
  {
    this->rightPin1 = rightPin1;
    this->leftPin1 = leftPin1;
    this->rightPin2 = rightPin2;
    this->leftPin2 = leftPin2;

    pinMode(this->rightPin1, INPUT_PULLUP);
    pinMode(this->leftPin1, INPUT_PULLUP);
    pinMode(this->rightPin2, INPUT_PULLUP);
    pinMode(this->leftPin2, INPUT_PULLUP);
  }
  ~Infrared()
  {

  }

private:
  int rightPin1;
  int leftPin1;
  int rightPin2;
  int leftPin2;

public:
  enum Mode
  {
    WHITE = 0,
    BLACK = 1
  };

  enum Result
  {
    STOP = 0,
    FORWARD = 1,
    RIGHT = 3,
    LEFT = 4
  };

  Infrared::Result getResult(Infrared::Mode mode)
  {
    Infrared::Result result;
    int right = digitalRead(this->rightPin1);
    int left = digitalRead(this->leftPin1);

    if(mode == WHITE)
    {
      if(right == 1 && left == 1)
      {
        result = FORWARD;
      }
      else if(right == 1 && left ==0)
      {
        result = RIGHT;
      }
      else if(right == 0 && left == 1)
      {
        result = LEFT;
      }
      else if(right == 0 && left == 0)
      {
        result = FORWARD;
      }
    }
    else if(mode == BLACK)
    {
      if(right == 1 && left == 1)
      {
        result = FORWARD;
      }
      else if(right == 1 && left ==0)
      {
        result = LEFT;
      }
      else if(right == 0 && left == 1)
      {
        result = RIGHT;
      }
      else if(right == 0 && left == 0)
      {
        result = FORWARD;
      }
    }

    return result;
  }
};

LED GreenLED(A1);
LED YellowLED(2);
Beep beep(4);
Infrared infrared(rightSignal1, leftSignal1, rightSignal2, leftSignal2);

void setup() 
{
  
}

void loop() 
{
  Infrared::Result result = infrared.getResult(Infrared::Mode::BLACK);
  if(result == Infrared::Result::FORWARD)
  {
    // beep.open();
    GreenLED.extinguish();
    YellowLED.extinguish();
  }
  else if(result == Infrared::Result::RIGHT)
  {
    beep.close();
    GreenLED.lighten();
    YellowLED.extinguish();
  }
  else if(result == Infrared::Result::LEFT)
  {
    beep.close();
    GreenLED.extinguish();
    YellowLED.lighten();
  }
}
