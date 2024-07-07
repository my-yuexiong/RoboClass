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

void setup() 
{
  
}

void loop() 
{
  
}
