#include <Servo.h>
#include <math.h>
// #include <ArduinoSTL.h>

Servo servo_7;
Servo servo_3;
Servo servo_5;
Servo servo_6;
Servo servo_9;
Servo servo_8;

const double pi = 3.1415926;

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

LED GreenLED(A1);
LED YellowLED(2);
Beep beep(4);

class MySerial
{
public:
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
  void sendMsg(const char* ch)
  {
    Serial.println(ch);
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
      YellowLED.flicker();
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

// MyServo HolderServo(0, 90, -90, 6);
// MyServo L1Servo(1, 0, -180, 5);
// MyServo L2Servo(2, 90, -90, 2);
// MyServo L3Servo(3, 90, -90, 5);
// MyServo L4Servo(4, 90, -90, 6);
// MyServo ClawServo(5, 99, 0, 0);
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


void servo_init()
{
  HolderServo->writeAngle(0);
  L1Servo->writeAngle(-90);
  L2Servo->writeAngle(45);
  L3Servo->writeAngle(45);
  L4Servo->writeAngle(0);
  ClawServo->writeAngle(99);
}

RoboArm Arm(HolderServo, L1Servo, L2Servo, L3Servo, L4Servo, ClawServo, 150.0, 105.0, 98.0, 175.0);

void setup()
{
  servo_7.attach(7);//云台
  servo_3.attach(3);//舵机1
  servo_5.attach(5);//舵机2
  servo_6.attach(6);//舵机3
  servo_9.attach(9);//舵机4
  servo_8.attach(8);//舵机5

  servo_init();
  delay(1000);
  
  // Arm.armMoveToPosition(-50, 273.0, 255.0, 0);
  // delay(1000);

  // Position point1;
  // point1.x = -50.0;
  // point1.y = 273.0;
  // point1.z = 255.0;

  // Position point2;
  // point2.x = 50.0;
  // point2.y = 273.0;
  // point2.z = 255.0;

  // Position point3;
  // point3.x = 50.0;
  // point3.y = 273.0;
  // point3.z = 205.0;

  // Position point4;
  // point4.x = -50.0;
  // point4.y = 273.0;
  // point4.z = 205.0;

  // Position point5;
  // point5.x = -50.0;
  // point5.y = 273.0;
  // point5.z = 155.0;

  // Position point6;
  // point6.x = 50.0;
  // point6.y = 273.0;
  // point6.z = 155.0;

  // Arm.pointToPoint(point1, point2, 8);
  // delay(200);
  // Arm.pointToPoint(point2, point3, 8);
  // delay(200);
  // Arm.pointToPoint(point3, point4, 8);
  // delay(200);
  // Arm.pointToPoint(point4, point5, 8);
  // delay(200);
  // Arm.pointToPoint(point5, point6, 8);
  // delay(200);
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

void cleanup()
{
  delete HolderServo;
  delete L1Servo;
  delete L2Servo;
  delete L3Servo;
  delete L4Servo;
  delete ClawServo;
}

void loop()
{
  
}