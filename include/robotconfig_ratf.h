#ifndef ROBOTCONFIG_RATF_H
#define ROBOTCONFIG_RATF_H

#include <Arduino.h>

#include <TimerOne.h>
#include <TimerThree.h>





/******************************************************************************
 * Configuration (uncomment the appropriate configuration)
 * - Lazarus-based applications (original channels implementation)
 * - ROS-based navigation stack (modified implementation of channels with the
 *     using the ROS package serial_communication_channels
 ******************************************************************************/

// #define CONFIG_LAZARUS        //!< firmware communicates to a Lazarus app
#define CONFIG_ROS            //!< firmware communicates to a ROS-based app





/******************************************************************************
 * #motors in the robot
 ******************************************************************************/
constexpr uint8_t kNumMot = 4;





/******************************************************************************
 * Robot parameters
 * - kinematic configuration
 * - battery
 ******************************************************************************/

//!< robot geometric distances
constexpr float kRobotL[] =
{
  0.200,    //!< front-back wheel distance
  0.200     //!< left-right wheel distance
};

//!< diameter of the wheels 0-3 (m)
constexpr float kRobotWhD[kNumMot] =
{
  0.0600,   //!< back-right wheel
  0.0600,   //!< back-left  wheel
  0.0600,   //!< front-right wheel
  0.0600    //!< front-left  wheel
};

constexpr float kRobotBattVnom = 11.1f;   //!< nominal battery level (V)





/******************************************************************************
 * Motor parameters
 * - gear reduction ratio and encoders resolution
 * - controllers parameters
 ******************************************************************************/

constexpr float kMotNgear  = 43.8;    //!< gear reduction ratio (n:1)
constexpr float kMotEncRes = 16*4;    //!< encoder resolution (tick count per rev.)

constexpr uint8_t kMotEncPin[kNumMot][2] =
{
  { 0, 1},    //!< encoder channel A and B of back -right wheel
  { 2, 3},    //!< encoder channel A and B of back -left  wheel
  { 4, 5},    //!< encoder channel A and B of front-right wheel
  {30,31}     //!< encoder channel A and B of front-left  wheel
};

constexpr int kMotDirPin[kNumMot] =
{
  23,   //!< back -right wheel
  22,   //!< back -left  wheel
  21,   //!< front-right wheel
  20    //!< front-left  wheel
};

constexpr int kMotPwmPin[kNumMot] =
{
  TIMER3_B_PIN,   //!< back -right wheel
  TIMER1_A_PIN,   //!< back -left  wheel
  TIMER1_B_PIN,   //!< front-right wheel
  TIMER3_A_PIN    //!< front-left  wheel
};

constexpr float kMotModelKp  = 4.5000;    //!< gain (rad.s^(-1) / V)
constexpr float kMotModelTau = 0.1000;    //!< time constant (s)
constexpr float kMotModelLag = 0.0000;    //!< lag lag (s)

constexpr unsigned long kMotCtrlFreq = 100UL;       //!< frequency (Hz)
constexpr float kMotCtrlTime = 1.0f / kMotCtrlFreq; //!< period (s)
constexpr unsigned long kMotCtrlTimeUs = 1000000UL / kMotCtrlFreq;
constexpr unsigned long kMotCtrlTimeout = 100UL;    //!< watchdog timeout (ms)
constexpr bool kMotCtrlTimeoutEnable = false;       //!< enable watchdog (true/false)

constexpr unsigned long kMotCtrlLEDOkFreq = 4UL;  //!< heartbeat LED frequency (Hz)
constexpr unsigned long kMotCtrlLEDOkCount =
    1000000UL / kMotCtrlLEDOkFreq / kMotCtrlTimeUs / 2;

constexpr float kMotVmax = 12;          //!< maximum voltage appliable to motors (V)
constexpr int16_t kMotPWMmax = 1023;    //!< maximum PWM (0..1023)
constexpr int16_t kMotPWMDeltaMax = 100;//!< maximum variation in PWM (0..1023)
constexpr bool kMotPWMDeltaMaxEnabled = true; //!< enable limits on PWM variation

constexpr float kMotHammV0 = 0.20;         //!< estimated motors' deadzone (V)
constexpr float kMotHammVd = 0.12;         //!< compensated motors' deadzone (V)

//! IMC tunning: desired time constant for the closed-loop (s)
constexpr float kMotCtrlTauCl = kMotModelTau / 1.0f;

//! IMC tunning: Kc_PI * Kp_plant
constexpr float kMotCtrlKcKp = kMotModelTau / (kMotCtrlTauCl + kMotModelLag);

//! PI proportional gain (V / rad.s^(-1))
constexpr float kMotCtrlKc = kMotCtrlKcKp / kMotModelKp;

//! PI integration time (s)
constexpr float kMotCtrlTi = kMotModelTau;

//! Feed-Forward constant
const float kMotCtrlKf = 0.456294584f;





/******************************************************************************
 * Conversion constants
 ******************************************************************************/

//! Conversion constant: encoder pulses (ticks) > motor angular speed (rad/s)
constexpr float kEncImp2MotW =
    2 * PI * 1000000 / (1.0f * kMotCtrlTimeUs * kMotNgear * kMotEncRes);

//! Conversion constant: motor voltage (V) > PWM (0..PWM_max)
constexpr float kMotV2MotPWM = kMotPWMmax * 1.0f / kRobotBattVnom;





#endif
