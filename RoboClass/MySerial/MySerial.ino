#include <SoftwareSerial.h>

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

class MySerial
{
public:
  MySerial()
  {

  }
  MySerial(int baud)
  {
    this->baud = baud;
    Serial.begin(this->baud);
  }
  ~MySerial()
  {

  }

private:
  int baud;

public:
  void sendMsg()
  {
    
  }

  String readMsg()
  {
    if(Serial.available() > 0)
    {
      String date = Serial.readStringUntil('\n');
      return date;
    }
    else
    {
      return "";
    }
  }

  int readDate(const String& date)
  {
    const char* cstr = date.c_str();
    int num = atoi(cstr);
    return num;
  }
};

LED GreenLED(A1);
LED YellowLED(2);
Beep beep(4);
MySerial serial;

void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  String date = serial.readMsg();
  if (date != "") {
    int num = serial.readDate(date);
    if(num > 300)
    {
      GreenLED.lighten();
      YellowLED.extinguish();
    }
    else
    {
      YellowLED.lighten();
      GreenLED.extinguish();
    }
    Serial.print("Received number: ");
    Serial.println(num);
  } else {
    Serial.println("Fail to receive message!");
  }
  delay(100);
}
