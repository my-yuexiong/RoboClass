char cmd_return_tmp[64];

void setup(){
  Serial.begin(115200);
  delay(400);
}

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
  void reversal(int pwm)//电机反转, pwm: 0~1000
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
      delay(10);
    }
    else
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 + pwm,0);
      Serial.println(cmd_return_tmp);
      delay(10);
    }
  }

  void foreward(int pwm)//电机正转, pwm: 0~1000
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
      delay(10);
    }
    else
    {
      sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500 - pwm,0);
      Serial.println(cmd_return_tmp);
      delay(10);
    }
  }

  void start(int pwm)//电机启动, pwm:-1000~1000, 正传: 0~1000, 反转: -1000~0
  {
    if(pwm <= 0)
    {
      if(pwm < -1000)
      {
        pwm = -1000;
      }
      pwm = 0 - pwm;

      this->reversal(pwm);
    }
    else if(pwm >= 0)
    {
      if(pwm > 1000)
      {
        pwm = 1000;
      }

      this->foreward(pwm);
    }
  }

  void stop()//电机停止
  {
    sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",this->id,1500,0);
    Serial.println(cmd_return_tmp);
    delay(10);
  }
};


/*********************************************************/

/**************************超声波**************************/
/*  -----------               | y轴           ----------- */  
/*  |motor1(6)|               |               |motor2(7)| */
/*  -----------               |               ----------- */
/*                            |                           */
/*                            |                           */
/* -----------------------------------------------------> */
/*                            |                     x轴   */
/*                            |                           */
/*  -----------               |               ----------- */  
/*  |motor3(8)|               |               |motor4(9)| */
/*  -----------               |               ----------- */
/**********************************************************/


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
  void moveForeward(int speed)//底盘前进
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

  void moveBack(int speed)//底盘后退
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

  void moveStop()//底盘停止
  {
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
  }

  void setDifferential(int rspeed, int lspeed)//设置左右差速
  {
    motor1.start(rspeed);
    motor3.start(rspeed);

    motor2.start(lspeed);
    motor4.start(lspeed);
  }

  void moveRight(int speed)//底盘向右平移
  {
    motor1.start(speed);
    motor4.start(speed);

    motor2.start(0 - speed);
    motor3.start(0 - speed);
  }

  void moveLeft(int speed)//底盘向左平移
  {
    motor1.start(0 - speed);
    motor4.start(0 - speed);

    motor2.start(speed);
    motor3.start(speed);
  }
};

void loop(){
  Motion motion(6,7,8,9);
  motion.moveRight(500);
  delay(500);
  motion.moveLeft(500);
  delay(500);
}
