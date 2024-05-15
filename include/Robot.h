#ifndef ROBOT_H
#define ROBOT_H

#include "robotconfig.h"
#include "Encoder.h"
#include "Motor.h"
#include "CtrlPID.h"





class Robot
{
 public:
  uint32_t dt;
  Encoder enc[kNumMot];
  Motor mot[kNumMot];
  CtrlPID pid[kNumMot];

  //robot speed
  float v, w;           //limited robot speed (used in VWToMotorSpeed)
  float v_req, w_req;   //desired robot speed
  float dv_max, dw_max; //maximum speed variation

  //robot odometry
  float w1e, w2e;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;

  void (*serialWriteChannel)(char channel, int32_t value);

 public:
  void init(void (*serialWriteChannelFunction)(char c, int32_t v));

  void update(uint32_t &delta);
  void send(void);
  void stop(void);

  void setRobotVW(float Vnom, float Wnom);
  void accelerationLimit(void);
  void VWToMotorSpeed(void);
  void odometry(void);

  void setMotorWref(uint8_t index, float new_w_r);
  void setMotorPWM(uint8_t index, int16_t pwm);

 private:
  void initEnc();
  void initCtrlPID(uint8_t index);
};





#endif
