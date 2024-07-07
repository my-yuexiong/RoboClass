char cmd_return_tmp[64];
// int greenLED = A1;
// int yellowLED = 2;
const int rightSignal = A4;
const int leftSignal = A5;

void setup(){
  Serial.begin(115200);
  // pinMode(greenLED,OUTPUT);
  // pinMode(yellowLED,OUTPUT);
  pinMode(rightSignal,INPUT_PULLUP);
  pinMode(leftSignal,INPUT_PULLUP);
  delay(400);
}

class LED
{
public:
  LED(int pin)
  {
    this->pin = pin;
    pinMode(this->pin,OUTPUT);
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

class Motor
{
public:
  Motor() {}
  Motor(int id) : id(id) 
  {
    this->id = id;
  }
  
  Motor::~Motor() 
  {

  }

  int id;//6~9

private:
  int pwm;//500~1500 正转 1500~2500 反转
  int time;//ms

public:
  void reversal(int pwm)//0~1000
  {
    if(pwm < 0)
    {
      pwm = 0;
    }
    if(pwm > 1000)
    {
      pwm = 1000;
    }

    if(this->id == 6 || this->id == 8)
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 - pwm,0);
      Serial.println(cmd_return_tmp);
    }
    else
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 + pwm,0);
      Serial.println(cmd_return_tmp);
    }
  }

  void foreward(int pwm)//0~1000
  {
    if(pwm < 0)
    {
      pwm = 0;
    }
    if(pwm > 1000)
    {
      pwm = 1000;
    }
    if(this->id == 6 || this->id == 8)
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 + pwm,0);
      Serial.println(cmd_return_tmp);
    }
    else
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 - pwm,0);
      Serial.println(cmd_return_tmp);
    }
  }

  void start(int pwm)
  {
    if(pwm > 0)
    {
      this->foreward(pwm);
    }
    else
    {
      pwm = 0 - pwm;
      this->reversal(pwm);
    }
  }

  void stop()
  {
    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500,0);
    Serial.println(cmd_return_tmp);
  }
};

class Motion
{
public:
  Motion(int id1, int id2, int id3, int id4)
  {
    this->motor1.id = id1;
    this->motor2.id = id2;
    this->motor3.id = id3;
    this->motor4.id = id4;
  }

  Motion::~Motion() 
  {

  }

private:
  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;

  int speed;//这里等同于Motor的pwm

public:
  void moveForeward(int speed)
  {
    if(speed < 0)
    {
      speed = 0;
    }
    if(speed > 1000)
    {
      speed = 1000;
    }
    motor1.foreward(speed);
    motor2.foreward(speed);
    motor3.foreward(speed);
    motor4.foreward(speed);
  }

  void moveBack(int speed)
  {
    if(speed < 0)
    {
      speed = 0;
    }
    if(speed > 1000)
    {
      speed = 1000;
    }
    motor1.reversal(speed);
    motor2.reversal(speed);
    motor3.reversal(speed);
    motor4.reversal(speed);
  }

  void moveStop()
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }

  void moveRight(int rspeed,int lspeed)
  {
    if(rspeed < 0)
    {
      rspeed = 0;
    }
    if(rspeed > 1000)
    {
      rspeed = 1000;
    }
    if(lspeed < 0)
    {
      lspeed = 1000;
    }
    if(lspeed > 1000)
    {
      lspeed = 1000;
    }

    motor1.foreward(lspeed);
    motor3.foreward(lspeed);

    motor2.reversal(rspeed);
    motor4.reversal(rspeed);
  }

  void moveLeft(int rspeed,int lspeed)
  {
    if(rspeed < 0)
    {
      rspeed = 0;
    }
    if(rspeed > 1000)
    {
      rspeed = 1000;
    }
    if(lspeed < 0)
    {
      lspeed = 1000;
    }
    if(lspeed > 1000)
    {
      lspeed = 1000;
    }

    motor1.reversal(lspeed);
    motor3.reversal(lspeed);

    motor2.foreward(rspeed);
    motor4.foreward(rspeed);
  }

};

void trackControl()
{

}


LED green(A1);
LED yellow(2);

void loop(){
  Motion motion(6,7,8,9);
  
  motion.moveForeward(800);

}
