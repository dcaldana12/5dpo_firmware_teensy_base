#include "Robot.h"





void Robot::init(void (*serialWriteChannelFunction)(char c, int32_t v))
{
  uint8_t i;

  // Serial write channels function
  serialWriteChannel = serialWriteChannelFunction;

  // Encoders
  initEnc();
  updateEncodersState();

  for (i = 0; i < kNumMot; i++)
  {
    encoders[i].delta = 0;
  }

  Timer1.attachInterrupt(updateEncodersState);
  Timer1.initialize(20);  // calls every X us

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

  // Robot speed controller and odometry
  accelerationLimit();
  VWToMotorSpeed();
  odometry();

  // Motor speed Controllers
  for (i = 0; i < kNumMot; i++)
  { 
    pid[i].update(enc[i].odo * kEncImp2MotW);
  }

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
}





void Robot::stop(void)
{
  uint8_t i;

  for (i = 0; i < kNumMot; i++)
  {
    setMotorPWM(i, 0);
  }
}

// updates reference robot speeds (call in main.cpp)
void Robot::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}

// limits variation of reference robot speeds
void Robot::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}

// updates reference speed for motors based on robot velocity (v) and angular velocity (w) inputs
void Robot::VWToMotorSpeed(void)
{
  //tangential wheel velocity (m/s)
  float v1ref = v + w * kRobotL[0] / 2;
  float v2ref = v - w * kRobotL[0] / 2; 
  
  //wheel speed (rad/s)
  float w1ref = v1ref / ( kRobotWhD[0]/2 );       //right wheel
  float w2ref = v2ref / ( kRobotWhD[1]/2 );       //left wheel

  setMotorWref(0, w1ref);
  setMotorWref(1, w2ref);
}


void Robot::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc[0].odo * kEncImp2MotW;
  w2e = enc[1].odo * kEncImp2MotW;

  v1e = w1e * ( kRobotWhD[0]/2 );
  v2e = w2e * ( kRobotWhD[1]/2 );     

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / kRobotL[0];
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
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
