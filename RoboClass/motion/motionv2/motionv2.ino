char cmd_return_tmp[64];

void setup(){
  Serial.begin(115200);
  delay(400);
}

void loop(){
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",6,1500+400,0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(1000);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",6,1500+(-400),0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(1000);

}