#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>

Servo servo_7;
Servo servo_3;
Servo servo_5;
Servo servo_6;
Servo servo_9;
Servo servo_8;

const double pi = 3.1415926;

const int rightSignal = A4;
const int leftSignal = A5;
const int beepPin = 4;
// const int greenLED = A1;
// const int yellowLED = 2;
const int BlackSignal = 2;
const int triggerPin = A1;
const int echoPin = A2;
// const int RXPin = A3;
// const int TXPin = A0;
const int GreenSignal = A3;
const int BlueSignal = A0;
// SoftwareSerial mySerial(RXPin, TXPin);

char cmd_return_tmp[64];

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

// LED GreenLED(greenLED);
// LED YellowLED(yellowLED);
Beep beep(beepPin);

class Infrared
{
public:
  Infrared(int rightPin, int leftPin)
  {
    this->rightPin = rightPin;
    this->leftPin = leftPin;

    pinMode(this->rightPin, INPUT_PULLUP);
    pinMode(this->leftPin, INPUT_PULLUP);
  }
  ~Infrared()
  {

  }

private:
  int rightPin;
  int leftPin;

public:
  enum Mode
  {
    WHITE = 0,
    BLACK = 1
  };

  enum Result
  {
    BACK = 0,
    FORWARD = 1,
    RIGHT = 3,
    LEFT = 4
  };

  Infrared::Result getResult(Infrared::Mode mode)
  {
    Infrared::Result result;
    int right = digitalRead(this->rightPin);
    int left = digitalRead(this->leftPin);

    if(mode == WHITE)
    {
      if(right == 1 && left == 1)
      {
        result = BACK;
      }
      else if(right == 1 && left == 0)
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
        result = BACK;
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

/**********************************************************/

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
/*                            |                           */
/**********************************************************/

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

class PID
{
public:
  PID(double kp, double ki, double kd)
  {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  ~PID()
  {

  }

private:
  double kp = 0;
  double ki = 0;
  double kd = 0;
  int error;
  int last_error = 0;

public:
  double getKp()
  {
    return this->kp;
  }

  void setKp(double kp)
  {
    this->kp = kp;
  }

  double getKi()
  {
    return this->ki;
  }

  void setKi(double ki)
  {
    this->ki = ki;
  }

  double getKd()
  {
    return this->kd;
  }

  void setKd(double kd)
  {
    this->kd = kd;
  }

  int output(int error)
  {
    this->error = error;
    int error_sum = this->error + error_sum;
    int output = this->kp * error + this->ki * error_sum + this->kd * (this->error - this->last_error);
    this->last_error = error;

    if(output > 1000)
    {
      output = 1000;
    }
    if(output < -1000)
    {
      output = -1000;
    }

    return output;
  }
};

class MyServo
{
public:
  MyServo(int id, int zeroPosition, double maxAngle, double minAngle, double realError, double step, int stepTime)
  {
    this->id = id;
    this->zeroPosition = zeroPosition;
    this->maxAngle = maxAngle;
    this->minAngle = minAngle;
    this->realError = realError;
    this->step = step;
    this->stepTime = stepTime;
  }
  ~MyServo()
  {
    
  }

private:
  int id;
  int zeroPosition;
  double maxAngle;
  double minAngle;
  double realError;
  double step;
  int stepTime;
  double nowAngle;

  bool isAngleValid(double angle)
  {
    if(angle <= this->maxAngle && angle >= this->minAngle)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

public:
  void set_id(int id)
  {
    this->id = id;
  }

  int get_id()
  {
    return this->id;
  }

  void set_zeroPosition(int zero)
  {
    this->zeroPosition = zero;
  }

  int get_zeroPosition()
  {
    return this->zeroPosition;
  }

  void set_angleRange(double minAngle, double maxAngle)
  {
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
  }

  void writeAngle(double angle)
  {
    if(this->isAngleValid(angle) == true)
    {
      double compensate = this->realError / 90 * angle;

      if(this->id == 0)
      {
        double targetAngle = this->zeroPosition - (angle * 2 / 3 + compensate);
        servo_7.write(targetAngle);
      }
      else if(this->id == 1)
      {
        double targetAngle = this->zeroPosition + (angle * 2 / 3 + compensate);
        servo_3.write(targetAngle);
      }
      else if(this->id == 2)
      {
        double targetAngle = this->zeroPosition + (angle * 2 / 3 + compensate);
        servo_5.write(targetAngle);
      }
      else if(this->id == 3)
      {
        double targetAngle = this->zeroPosition + (angle * 2 / 3 + compensate);
        servo_6.write(targetAngle);
      }
      else if(this->id == 4)
      {
        double targetAngle = this->zeroPosition + (angle * 2 / 3 + compensate);
        servo_9.write(targetAngle);
      }
      else if(this->id == 5)
      {
        double targetAngle = angle * 2 / 3;
        servo_8.write(targetAngle);
      }

      this->nowAngle = angle;
    }
    else
    {
      // YellowLED.flicker();
      beep.flicker();
    }
  }

  double readAngle()
  {
    return this->nowAngle;
  }

  void slowlyToAngle(double angle)
  {
    int moveTimes = (angle - this->nowAngle) / this->step;
    int residueAngle = angle - this->nowAngle - moveTimes * this->step;

    for(int i = 0; i < moveTimes; i++)
    {
      double targetAngle = this->nowAngle + this->step;
      this->writeAngle(targetAngle);
      delay(this->stepTime);
    }

    this->writeAngle(this->nowAngle + residueAngle);
  }
};

MyServo *HolderServo = new MyServo(0, 87, 90, -90, 6, 5, 100);
MyServo *L1Servo = new MyServo(1, 120, 0, -180, 5, 5, 100);
MyServo *L2Servo = new MyServo(2, 90, 135, -135, 2, 5, 100);
MyServo *L3Servo = new MyServo(3, 90, 90, -90, 5, 5, 100);
MyServo *L4Servo = new MyServo(4, 90, 90, -90, 6, 5, 100);
MyServo *ClawServo = new MyServo(5, 0, 99, 0, 0, 5, 100);

struct ServoAngles
{
  double holderAngle;
  double L1Angle;
  double L2Angle;
  double L3Angle;
};

struct Position
{
  double x;
  double y;
  double z;
};

class RoboArm
{
public:
  RoboArm(MyServo *servo1, MyServo *servo2, MyServo *servo3, MyServo *servo4, MyServo *servo5, MyServo *servo6,
          double L1, double L2, double L3, double L4)
  {
    this->holder = servo1;
    this->L1joint = servo2;
    this->L2joint = servo3;
    this->L3joint = servo4;
    this->L4jiont = servo5;
    this->Claw = servo6;

    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->L4 = L4;
  }
  ~RoboArm()
  {

  }

private:
  double L1;
  double L2;
  double L3;
  double L4;
  MyServo *holder = NULL;
  MyServo *L1joint = NULL;
  MyServo *L2joint = NULL;
  MyServo *L3joint = NULL;
  MyServo *L4jiont = NULL;
  MyServo *Claw = NULL;

  double nowPitch;

  ServoAngles getAnglesByPosition(double x, double y, double z, double pitch)
  {
    ServoAngles targetAngles;
    double horizontalAngle = pitch * pi / 180;

    double seta0 = atan(x / y);

    double d = sqrt(x * x + y * y);

    double n = z - this->L1 + this->L4 * sin(horizontalAngle);
    double m = d - this->L4 * cos(horizontalAngle);

    double seta2 = acos((pow(m, 2) + pow(n, 2) - pow(this->L2, 2) - pow(this->L3, 2)) / (2 * this->L2 * this->L3));

    double A = this->L3 * sin(seta2);
    double B = this->L2 + this->L3 * cos(seta2);

    double phy = asin(B / sqrt(pow(A, 2) + pow(B, 2)));
    double seta1 = pi - asin(m / sqrt(pow(A, 2) + pow(B, 2))) - phy;
    double seta3 = horizontalAngle + seta1 - seta2;

    targetAngles.holderAngle = seta0 / pi * 180;
    targetAngles.L1Angle = 0 - seta1 / pi * 180;
    targetAngles.L2Angle = seta2 / pi * 180;
    targetAngles.L3Angle = seta3 / pi * 180;

    return targetAngles;
  }

  Position getPositionByAngles(double angle0, double angle1, double angle2, double angle3)
  {
    Position targetPosition;

    double seta0 = angle0 * pi / 180;
    double seta1 = angle1 * pi / 180;
    double seta2 = angle2 * pi / 180;
    double seta3 = angle3 * pi / 180;

    double d1 = this->L2 * cos(seta1);
    double d2 = this->L3 * cos(seta1 - seta2);
    double d3 = this->L4 * cos(seta2 + seta3 - seta1);
    double d = d1 + d2 + d3;

    double z1 = this->L2 * sin(seta1);
    double z2 = this->L3 * sin(seta1 - seta2);
    double z3 = this->L4 * sin(seta2 + seta3 -seta1);
    double z = this->L1 + z1 + z2 - z3;

    double x = d * sin(seta0);
    double y = d * cos(seta0);

    targetPosition.x = x;
    targetPosition.y = y;
    targetPosition.z = z;

    return targetPosition;
  }

  Position* interPosition(const Position& origin, const Position& destination, int num)
  {
    Position* positions = new Position[num + 2];
    positions[0] = origin;

    double x_step = (destination.x - origin.x) / (num + 1);
    double y_step = (destination.y - origin.y) / (num + 1);
    double z_step = (destination.z - origin.z) / (num + 1);

    for(int i = 1; i < num + 1; i++)
    {
      Position interPolation;
      interPolation.x = origin.x + x_step * i;
      interPolation.y = origin.y + y_step * i;
      interPolation.z = origin.z + z_step * i;

      positions[i] = interPolation;
    }

    positions[num + 1] = destination;

    return positions;
  }

public:
  Position clawPosition;
  ServoAngles jointAngles;

  void clawOpen()
  {
    this->Claw->writeAngle(0);
  }

  void clawClose()
  {
    this->Claw->writeAngle(99);
  }

  void armMoveByAngle(double angle1, double angle2, double angle3, double angle4)
  {
    this->jointAngles.holderAngle = angle1;
    this->jointAngles.L1Angle = angle2;
    this->jointAngles.L2Angle = angle3;
    this->jointAngles.L3Angle = angle4;

    this->holder->writeAngle(angle1);
    this->L1joint->writeAngle(angle2);
    this->L2joint->writeAngle(angle3);
    this->L3joint->writeAngle(angle4);
  }

  ServoAngles armMoveToPosition(double x, double y, double z, double pitch)
  {
    ServoAngles nowAngles = this->getAnglesByPosition(x, y, z, pitch);
    this->armMoveByAngle(nowAngles.holderAngle, nowAngles.L1Angle, nowAngles.L2Angle, nowAngles.L3Angle);
    delay(150);
    this->nowPitch = pitch;
    return nowAngles;
  }

  void pointToPoint(const Position& origin, const Position& destination, int num)
  {
    Position* positions = interPosition(origin, destination, num);

    for(int i = 0; i < num + 2; i++)
    {
      this->armMoveToPosition(positions[i].x, positions[i].y, positions[i].z, this->nowPitch);
    }
  }
};

// class MySerial
// {
// public:
//   MySerial()
//   {

//   }
//   MySerial(int baud)
//   {
//     this->baud = baud;
//     mySerial.begin(this->baud);
//   }
//   ~MySerial()
//   {

//   }

// private:
//   int baud;

// public:
//   void sendMsg()
//   {
    
//   }

//   String readMsg()
//   {
//     if(mySerial.available() > 0)
//     {
//       String date = mySerial.readStringUntil('\n');
//       return date;
//     }
//     else
//     {
//       return "";
//     }
//   }

//   int readDate(const String& date)
//   {
//     const char* cstr = date.c_str();
//     int num = atoi(cstr);
//     return num;
//   }
// };

void servo_init()
{
  HolderServo->writeAngle(0);
  // L1Servo->writeAngle(-90);
  // L2Servo->writeAngle(80);
  // L3Servo->writeAngle(30);
  L1Servo->writeAngle(-110);
  L2Servo->writeAngle(80);
  L3Servo->writeAngle(40);
  L4Servo->writeAngle(0);
  ClawServo->writeAngle(99);
}

Infrared infrared(rightSignal, leftSignal);
Motion motion(6,7,8,9);
Ultrasonic ultrasonic(triggerPin, echoPin);
RoboArm Arm(HolderServo, L1Servo, L2Servo, L3Servo, L4Servo, ClawServo, 150.0, 105.0, 98.0, 175.0);
// MySerial serial;

void cleanup()
{
  delete HolderServo;
  delete L1Servo;
  delete L2Servo;
  delete L3Servo;
  delete L4Servo;
  delete ClawServo;
}

void writeOne()
{
  Arm.armMoveToPosition(0, 244.3, 324.3, 0);
  delay(1000);

  Position point1;
  point1.x = 0;
  point1.y = 244.3;
  point1.z = 324.3;

  Position point2;
  point2.x = 0;
  point2.y = 246.3;
  point2.z = 270.3;

  Arm.pointToPoint(point1, point2, 4);
  delay(200);
}

void writeTwo()
{
  Arm.armMoveToPosition(-25, 244.3, 324.3, 0);
  delay(1000);

  Position point1;
  point1.x = -25.0;
  point1.y = 244.3;
  point1.z = 324.3;

  Position point2;
  point2.x = 25.0;
  point2.y = 245.3;
  point2.z = 324.3;

  Position point3;
  point3.x = 25.0;
  point3.y = 246.3;
  point3.z = 304.3;

  Position point4;
  point4.x = -25.0;
  point4.y = 247.3;
  point4.z = 304.3;

  Position point5;
  point5.x = -25.0;
  point5.y = 248.3;
  point5.z = 280.3;

  Position point6;
  point6.x = 25.0;
  point6.y = 249.3;
  point6.z = 280.3;

  Arm.pointToPoint(point1, point2, 4);
  delay(200);
  Arm.pointToPoint(point2, point3, 4);
  delay(200);
  Arm.pointToPoint(point3, point4, 4);
  delay(200);
  Arm.pointToPoint(point4, point5, 4);
  delay(200);
  Arm.pointToPoint(point5, point6, 4);
  delay(200);
}

void setup() 
{
  Serial.begin(115200);
  // mySerial.begin(9600);
  
  servo_7.attach(7);//云台
  servo_3.attach(3);//舵机1
  servo_5.attach(5);//舵机2
  servo_6.attach(6);//舵机3
  servo_9.attach(9);//舵机4
  servo_8.attach(8);//舵机5

  pinMode(GreenSignal, INPUT_PULLUP);
  pinMode(BlueSignal, INPUT_PULLUP);
  pinMode(BlackSignal, OUTPUT);
  digitalWrite(BlackSignal, LOW);

  servo_init();
  delay(1000);

  // motion.moveForeward(500);
  // delay(1000);
  // motion.setDifferential(-500, 500);
  // delay(500);

}

int flag = 0;
int TCross = 0;
int num = 0;

void loop() 
{
  int distance = ultrasonic.getDistance();
  // String date = serial.readMsg();
  // if (date != "") {
  //   int error = serial.readDate(date);
  //   if(error > 50)
  //   {
  //     motion.setDifferential(200,-200);
  //   }
  //   else if(error < -50)
  //   {
  //     motion.setDifferential(-200,200);
  //   }
  //   else
  //   {
  //     motion.moveForeward(200);
  //   }   
  //   Serial.print("Received number: ");
  //   Serial.println(error);
  // } else {
  //   Serial.println("Fail to receive message!");
  // }
  // delay(100);

  if(distance < 25)
  {
    motion.moveStop();
    // YellowLED.lighten();
    // delay(4000);
    if(num == 1)
    {
      motion.moveStop();

      delay(500);

      if(digitalRead(GreenSignal) == 1 && digitalRead(BlueSignal) == 0)
      {
        Arm.armMoveByAngle(0, -90, 45, 45);
        delay(10);
        while(1)
        {
          int nowDistance = ultrasonic.getDistance();
          if(nowDistance <= 10)
          {
            motion.moveStop();
            break;
          }
          else 
          {
            motion.moveForeward(200);
          }
        }
        writeTwo();
      }
      else if(digitalRead(GreenSignal) == 0 && digitalRead(BlueSignal) == 1)
      {
        Arm.armMoveByAngle(0, -90, 45, 45);
        delay(10);
        while(1)
        {
          int nowDistance = ultrasonic.getDistance();
          if(nowDistance <= 10)
          {
            motion.moveStop();
            break;
          }
          else 
          {
            motion.moveForeward(200);
          }
        }
        writeOne();
      }

    }
    else if(num == 0)
    {
      digitalWrite(BlackSignal, HIGH);
      motion.moveStop();

      delay(500);
      // digitalWrite(BlackSignal, LOW);
      if(digitalRead(GreenSignal) == 1 && digitalRead(BlueSignal) == 0)//绿色
      {
        // motion.moveForeward(200);
        // delay(50);
        motion.setDifferential(500,-500);
        delay(500);
        motion.moveForeward(200);
        delay(150);

        TCross = 1;
      }
      else if(digitalRead(GreenSignal) == 0 && digitalRead(BlueSignal) == 1)//蓝色
      {
        // motion.moveForeward(200);
        // delay(50);
        motion.setDifferential(-500,500);
        delay(400);
        motion.moveForeward(200);
        delay(150);

        TCross = 2;
      }
      else
      {
        motion.setDifferential(500,-500);
        delay(500);
        motion.moveForeward(200);
        delay(150);

        TCross = 1;
      }

      num = 1;
    }
    
  }
  else
  {
    // if(digitalRead(BlackSignal) == 1)
    if(digitalRead(GreenSignal) == 1 && digitalRead(BlueSignal) == 1)
    {
      if(TCross == 1)//绿色
      {
        motion.moveForeward(200);
        delay(100);
        motion.setDifferential(500,-500);
        delay(500);
        motion.moveForeward(200);
        delay(50);

        TCross = 3;
      }
      else if(TCross == 2)//蓝色
      {
        motion.setDifferential(-500,500);
        delay(500);
        motion.moveForeward(200);
        delay(50);

        TCross = 4;
      }
    }
    // YellowLED.extinguish();
    if(TCross == 1)
    {
      Arm.armMoveByAngle(90, -90, 90, 60);
      delay(10);
    }
    else if(TCross == 2)
    {
      Arm.armMoveByAngle(-90, -90, 90, 60);
      delay(10);
    }
    else if(TCross == 3)
    {
      Arm.armMoveByAngle(0, -90, 90, 0);
      delay(10);
    }
    else if(TCross == 4)
    {
      Arm.armMoveByAngle(0, -90, 90, 0);
      delay(10);
    }
    Infrared::Result result = infrared.getResult(Infrared::Mode::BLACK);
    if(result == Infrared::Result::FORWARD)
    {
      motion.moveForeward(300);
    }
    else if(result == Infrared::Result::RIGHT)
    {
      // motion.setDifferential(300, 250);
      motion.setDifferential(350, 300);
      flag = 1;
    }
    else if(result == Infrared::Result::LEFT)
    {
      motion.setDifferential(300, 350);
      flag = 2;
    }
    else if(result == Infrared::Result::BACK)
    {
      if(flag == 2)
      {
        // motion.setDifferential(-300, 300);
        motion.setDifferential(-250, 250);
        flag = 0;
        delay(5);
      }
      else if(flag == 1)
      {
        motion.setDifferential(250, -250);
        flag = 0;
        delay(5);
      }
  
    }
  }
  
}

