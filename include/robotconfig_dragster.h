#ifndef ROBOTCONFIG_DRAGSTER_H
#define ROBOTCONFIG_DRAGSTER_H

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
constexpr uint8_t kNumMot = 2;





/******************************************************************************
 * Robot parameters
 * - kinematic configuration (e.g., if you want to compute odom in micro ctrl)
 * - battery
 ******************************************************************************/

//!< robot geometric distances (e.g., wheelbase = dist right and left wh)
constexpr float kRobotL[] =
{
  0.2350    //!< distance between left and right wheels
};

//!< diameter of the wheels 0-3 (m)
constexpr float kRobotWhD[kNumMot] =
{
  0.040,   //!< right wheel
  0.040    //!< left  wheel
};

constexpr float kRobotBattVnom = 7.4f;   //!< nominal battery level (V)





/******************************************************************************
 * General inputs / outputs
 * - solenoid
 * - switch
 ******************************************************************************/
constexpr uint8_t kRobotActSolenoidPin = 40;
constexpr uint8_t kRobotSensSwitchPin  = 39;





/******************************************************************************
 * Motor parameters
 * - gear reduction ratio and encoders resolution
 * - controllers parameters
 ******************************************************************************/

constexpr float kMotNgear  = 17.f/110.f; //43.8;    //!< gear reduction ratio (n:1) (motor2encoder speed)
constexpr float kWheelNgear = 17.f/36.f;            //!< gear reduction ratio (motor2wheel speed)
constexpr float kMotEncRes = 2400;    //!< encoder resolution (tick count per rev.)

constexpr uint8_t kMotEncPin[kNumMot][2] =
{
  { 0, 1},    //!< encoder channel A and B of right wheel
  { 2, 3}     //!< encoder channel A and B of left  wheel
};

constexpr int kMotDirPin[kNumMot] =
{
  23,   //!< right wheel
  20    //!< left  wheel
};

constexpr int kMotPwmPin[kNumMot] =
{
  TIMER3_B_PIN,   //!< right wheel
  TIMER3_A_PIN    //!< left  wheel
};

constexpr float kMotModelKp  = 64.50;    //!< gain (rad.s^(-1) / V)
constexpr float kMotModelTau = 0.1;    //!< time constant (s)
constexpr float kMotModelLag = 0.0000;    //!< lag lag (s)

constexpr unsigned long kMotCtrlFreq = 100UL;       //!< frequency (Hz)
constexpr float kMotCtrlTime = 1.0f / kMotCtrlFreq; //!< period (s)
constexpr unsigned long kMotCtrlTimeUs = 1000000UL / kMotCtrlFreq;
constexpr unsigned long kMotCtrlTimeout = 100UL;    //!< watchdog timeout (ms)
constexpr bool kMotCtrlTimeoutEnable = false;       //!< enable watchdog (true/false)

constexpr unsigned long kMotCtrlLEDOkFreq = 4UL;  //!< heartbeat LED frequency (Hz)
constexpr unsigned long kMotCtrlLEDOkCount =
    1000000UL / kMotCtrlLEDOkFreq / kMotCtrlTimeUs / 2;

constexpr float kMotVmax = 7.f;         //!< maximum voltage appliable to motors (V)
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
const float kMotCtrlKf =  0.8*1/kMotModelKp;//0.0097342799188641f;





/******************************************************************************
 * Conversion constants
 ******************************************************************************/

//! Conversion constant: encoder pulses (ticks) > motor angular speed (rad/s)
constexpr float kEncImp2MotW =
    2 * PI * 1000000 / (1.0f * kMotCtrlTimeUs * kMotNgear * kMotEncRes);

//! Conversion constant: motor voltage (V) > PWM (0..PWM_max)
constexpr float kMotV2MotPWM = kMotPWMmax * 1.0f / kRobotBattVnom;





#endif
