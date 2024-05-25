#include "Robot.h"





void Robot::init(void (*serialWriteChannelFunction)(char c, int32_t v))
{
  uint8_t i;

  // Serial write channels function
  serialWriteChannel = serialWriteChannelFunction;

  // General inputs / outputs
  // - solenoid
  pinMode(kRobotSensSwitchPin, INPUT_PULLUP);
  pinMode(kRobotActSolenoidPin, OUTPUT);
  digitalWrite(kRobotActSolenoidPin, 0);

  // Encoders
  initEnc();
  updateEncodersState();

  for (i = 0; i < kNumMot; i++)
  {
    encoders[i].delta = 0;
  }

  Timer1.attachInterrupt(updateEncodersState);
  Timer1.initialize(5);  // calls every X us

  // Motors
  Timer3.initialize(20);

  for (i = 0; i < kNumMot; i++)
  {
    mot[i].init(kMotDirPin[i], kMotPwmPin[i]);
  }

  // Controllers
  for (i = 0; i < kNumMot; i++)
  {
    initCtrlPID(i);
  }
}





void Robot::update(uint32_t &delta)
{
  uint8_t i;
  dt = delta;

  // Encoders
  for (i = 0; i < kNumMot; i++)
  {
    enc[i].updateTick();
  }

  updateState(enc[1].odo, enc[0].odo);

  // Controllers
  for (i = 0; i < kNumMot; i++)
  {
    pid[i].update(enc[i].odo * kEncImp2MotW);
  }
  tomaW=enc[0].odo;
  // Actuators
  for (i = 0; i < kNumMot; i++)
  {
    if (pid[i].active)
    {
      mot[i].setPWM( round( kMotV2MotPWM * pid[i].m ) );
    }
  }
}





void Robot::send(void)
{
  for (int idx = 0; idx < kNumMot; idx++)
  {
    (*serialWriteChannel)('g'+idx, enc[idx].tick);
  }

  (*serialWriteChannel)('k', dt);

  (*serialWriteChannel)('s', (digitalRead(kRobotSensSwitchPin) << 0));
  (*serialWriteChannel)('z', pid[0].w);
}





void Robot::stop(void)
{
  uint8_t i;

  for (i = 0; i < kNumMot; i++)
  {
    setMotorPWM(i, 0);
  }
}





void Robot::setMotorWref(uint8_t index, float new_w_r)
{
  pid[index].enable(true);
  pid[index].w_ref = new_w_r;
}





void Robot::setMotorPWM(uint8_t index, int16_t pwm)
{
  pid[index].enable(false);
  mot[index].setPWM(pwm);
}





void Robot::initEnc()
{
  for (int idx = 0; idx < kNumMot; idx++)
  {
    pinMode(kMotEncPin[idx][0], INPUT_PULLUP);
    pinMode(kMotEncPin[idx][1], INPUT_PULLUP);
  }
}





void Robot::initCtrlPID(uint8_t index)
{
  pid[index].active = false;
  pid[index].kp = kMotCtrlKc;

  if (kMotCtrlTi == 0)
  {
    pid[index].ki = 0;
  }
  else
  {
    pid[index].ki = kMotCtrlKc / kMotCtrlTi;
  }

  pid[index].kd = 0;
  pid[index].kf = kMotCtrlKf;
  pid[index].dt = kMotCtrlTime;

  pid[index].m_max = kMotVmax;

  pid[index].hamm_vd = 0;
  pid[index].hamm_v0 = 0;

  pid[index].reset();
}

void Robot::updateState(uint32_t ticks_left, uint32_t ticks_right)
{
  double d1 = double(ticks_left)/kMotEncRes*2*M_PI*kRobotWhD[1]/2;
  double d2 = double(ticks_right)/kMotEncRes*2*M_PI*kRobotWhD[0]/2;
  double delta_d = (d2+d1)/2;
  double delta_theta = (d2-d1)/(2*kRobotL[0]);
  x = x + delta_d*cos(theta + delta_theta/2);
  y = y + delta_d*sin(theta + delta_theta/2);
  theta = theta + delta_theta;

  Serial.print("Distance: ");
  Serial.print(x);
  Serial.print("Ticks: ");
  Serial.print(enc[0].tick);
  Serial.print("|| L speed: ");
  Serial.print(enc[0].odo * kEncImp2MotW);
  Serial.print("|| L speed ref: ");
  Serial.print(pid[0].w_ref);
  Serial.print("|| L motor voltage: ");
  Serial.print(pid[0].m);
  Serial.print("\n");
}
