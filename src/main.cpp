#include <Arduino.h>

#include <channels.h>

#include "Robot.h"

#define LDRPIN A0
#define LDRThreshold 800



/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

bool race_mode=false;

unsigned long current_micros = 0;
unsigned long first_micros = 0;
unsigned long previous_micros = 0;
unsigned long last_motor_update_millis = 0;
double Wmax = 200; //200
double Wmin = 50;  //90 //100
double r = kRobotWhD[0]/2;
double b = 0.095;
double Vlin = Wmin*r;
double VlinMax = Wmax*r;
double Wlimit = 350;//230
int cont = 0;
double last_errorTheta = 0, last_errorDist = 0;

bool timeout = false;

channels_t serial_channels;

uint8_t builtin_led_state;    // NOTE: the pin function is shared with SPI SCK!



Robot robot;

Encoder *encoders = robot.enc;





/******************************************************************************
 * FUNCTIONS HEADERS
 ******************************************************************************/

void processSerialPacket(char channel, uint32_t value, channels_t& obj);
void serialWrite(uint8_t b);
void serialWriteChannel(char channel, int32_t value);
void serialRead();
void checkMotorsTimeout();
void parseSerialPico();
double acc_ramp(double atual, double increments);




/******************************************************************************
 * IMPLEMENTATION
 ******************************************************************************/

void setup()
{
  // Built-in LED
  builtin_led_state = LOW;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, builtin_led_state);

  // Robot
  robot.init(serialWriteChannel);

  // Serial communication
  // ( in the case of the teensy, does not matter, the PC sets the serial
  //   settings for Serial! )
  Serial.begin(115200);
  Serial4.begin(115200);
  serial_channels.init(processSerialPacket, serialWrite);

  // Reset signal
  serialWriteChannel('r', 0);

  // Test PWM motors
  /*robot.setMotorPWM(0, 0);
  robot.setMotorPWM(1, 0);
  robot.setMotorPWM(2, 0);
  robot.setMotorPWM(3, 0);*/
  

  // Initialization
  current_micros = micros();
  first_micros = micros();
  previous_micros = current_micros;
  last_motor_update_millis = millis();
}

void loop()
{
  static unsigned long blink_led_decimate = 0;
  uint32_t delta;
  

  serialRead();

  current_micros = micros();
  delta = current_micros - previous_micros;
  if (delta > kMotCtrlTimeUs)
  {
    cont++;
    if (kMotCtrlTimeoutEnable)
    {
      checkMotorsTimeout();
    }

    if (!timeout)
    {
      previous_micros = current_micros;

      // Update and send data
      robot.update(delta);
      robot.send();
      parseSerialPico();
      /* Serial.print("angle: ");
      Serial.print(robot.angle2line); //radians (display is degrees)
      Serial.print(" || dist: ");
      Serial.println(robot.dist2line); //not sure what unit (cm?)*/

      //Serial.print(" || ");
      //Serial.println(analogRead(LDRPIN));
      if(analogRead(LDRPIN)>LDRThreshold){
        race_mode=true;
        //reset odometry
        robot.x = 0;
        robot.y = 0;
        robot.theta = 0;
      }

      //ES TEST set left motor speed to 100rad/s approx 955rpm (5dpo max speed 300rad/s)
      //PI individual motor speed control
      /* if(race_mode){
        if(robot.x >= 8.85){ //stop condition and deacceleration ramp
          Vlin = acc_ramp(Vlin, -0.28); 
        }

        Vlin = acc_ramp(Vlin, 0.035); 
        double v2 = Vlin;
        double wref2 = v2/kRobotWhD[1]/2;
        robot.setMotorWref(1,wref2);

        double v1 = Vlin;
        double wref1 = v1/kRobotWhD[0]/2;
        robot.setMotorWref(0,wref1); 

        //robot.setMotorPWM(0, 200);
        //robot.setMotorPWM(1, 200);
      } */
      
      if(race_mode){
        double error_theta = robot.angle2line;
        double error_line = robot.dist2line;

        double tau = 0.3;
        double Ktheta = 2;
        //double Ky = Ktheta*Ktheta/(Vlin * 4);
        double Ky = 60/Vlin;

        double KdTheta = 275, KdLine = 0; 
        double TdLine = 11.6; 

        //w for correction with PD
        double w = Ktheta*error_theta + KdTheta*(error_theta - last_errorTheta) + Ky*(error_line + TdLine*(error_line - last_errorDist));

        if(robot.x >= 8.85){ //stop condition and deacceleration ramp
          Vlin = acc_ramp(Vlin, -0.28); 
        }
        else{ //accelerate until VlinMax
          Vlin = acc_ramp(Vlin, 0.035); 
        }

        double v1 = Vlin - w*b;
        double v2 = Vlin + w*b;

        double wref1 = v1/kRobotWhD[0]/2;
        double wref2 = v2/kRobotWhD[1]/2;

        last_errorTheta = error_theta;
        last_errorDist = error_line;

        //stop conditions on lost control
        if(robot.bad_count > 3 || robot.bad_count < -3){
          race_mode = false;
        }
        else if(abs(error_theta) > 45*M_PI/180 || abs(error_line) > 20 || Vlin == 0){
          race_mode = false;
        }
        //limit motors speed
        if(wref1 > Wlimit){
          wref1 = Wlimit;
        } 
        else if(wref1 < -Wlimit){
          wref1 = -Wlimit;
        }
        if(wref2 > Wlimit){
          wref2 = Wlimit;
        } 
        else if(wref2 < -Wlimit){
          wref2 = -Wlimit;
        }

        robot.setMotorWref(0,wref1);
        robot.setMotorWref(1,wref2);
      }
      else{ //stop
        robot.setMotorPWM(1,0);
        robot.setMotorPWM(0,0);
      }

      // Blink LED
      blink_led_decimate++;
      if (blink_led_decimate >= kMotCtrlLEDOkCount)
      {
        if (builtin_led_state == LOW)
        {
          builtin_led_state = HIGH;
        }
        else
        {
          builtin_led_state = LOW;
        }

        digitalWrite(LED_BUILTIN, builtin_led_state);
        blink_led_decimate = 0;
      }
    }
  }
}





/******************************************************************************
 * FUNCTIONS IMPLEMENTATIONS
 ******************************************************************************/

void processSerialPacket(char channel, uint32_t value, channels_t& obj)
{
  uint8_t mot_i;
  int16_t pwm;

  // Reset watchdog
  if ((channel == 'G') || (channel == 'K'))
  {
    last_motor_update_millis = millis();
  }

  // Process incomming serial packet
  switch (channel)
  {
  // - reference angular speed
  case 'G':
  case 'H':
  case 'I':
  case 'J':
    mot_i = channel - 'G';
    // set reference angular speed for the motors
#ifdef CONFIG_LAZARUS
    robot.setMotorWref(mot_i, ((int32_t) value) * kEncImp2MotW );
#endif
#ifdef CONFIG_ROS
    if (mot_i >= kNumMot)
    {
      return;
    }
    robot.setMotorWref(mot_i, *((float*) &value) );
#endif
    break;

  // - PWM
  case 'K':
    mot_i = (value >> 24) & 0x03;
    if (mot_i >= kNumMot)
    {
      return;
    }
    pwm = value & 0xFFFF;
    robot.setMotorPWM(mot_i, pwm);
    break;

  // - solenoid
  case 'L':
    digitalWrite(kRobotActSolenoidPin, value);
    break;
  }
}





void serialWrite(uint8_t b)
{
  Serial.write(b);
}



void serialWriteChannel(char channel, int32_t value)
{
  //serial_channels.send(channel, value);
}


void serialRead()
{
  uint8_t serial_byte;

  if (Serial.available() > 0)
  {
    serial_byte = Serial.read();
    serial_channels.StateMachine(serial_byte);
  }
}

void checkMotorsTimeout()
{
  if (millis() - last_motor_update_millis > kMotCtrlTimeout)
  {
    timeout = true;

    robot.stop();
    digitalWrite(kRobotActSolenoidPin, 0);

    builtin_led_state = LOW;
    digitalWrite(LED_BUILTIN, builtin_led_state);

  }
  else
  {
    if (timeout)
    {
      robot.init(serialWriteChannel);
    }

    timeout = 0;
  }
}

void parseSerialPico(){
  bool read = false;
  char num[5];
  char rc;
  int count_num = 0, read_state = 0;
  //Serial.print('\n');
  bool start = true;

  while(Serial4.available() > 0)
  {
    //SerialAlt.printf("b:%d; a:% .3f; y:% .3f;\n", bad_count, line_fit.theta, line_fit.b);
    rc = Serial4.read();
    if(start && rc!='b'){
      break;
    }
    //Serial.print(rc);
    switch (rc)
    {
    case ':':
      read = true;
      count_num = 0;
      break;
    case ';':
      read = false;
      read_state++;
      num[count_num] = '\0';

      if(read_state == 1){
        robot.bad_count = atoi(num);
      }
      else if(read_state == 2){
        robot.angle2line = atof(num);
      }
      else if(read_state == 3){
        robot.dist2line = atof(num)/1000;
      }
      break;
    case 'b':
      start = false;
      break;
    case 'a':
    case 'y':
    case ' ':
    case '\n':
      break;
    
    default:
      if(read){
        num[count_num] = rc;
        count_num++;
      }
      break;

    }
  }
}

double acc_ramp(double atual, double increments){
  atual = atual + increments;
  if(atual > VlinMax){
    atual = VlinMax;
  }
  else if(increments < 0 && atual <= 0){
    atual = 0;
  }

  return atual;
}
