#include <Arduino.h>

#include <channels.h>

#include "Robot.h"

#include <string>


/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

unsigned long current_micros = 0;
unsigned long previous_micros = 0;
unsigned long last_motor_update_millis = 0;

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
void readPicoCam(char *buffer,  int &angle, int &dist2line);
void parsePicoInfo(char *buffer, int &angle, int &dist2line);




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
  //robot.init(serialWriteChannel);

  // Serial communication
  // ( in the case of the teensy, does not matter, the PC sets the serial
  //   settings for Serial! )
  Serial.begin(115200);
  Serial4.begin(115200); // arducam
  //serial_channels.init(processSerialPacket, serialWrite);

  // Reset signal
  //serialWriteChannel('r', 0);

  // Test PWM motors
  /*robot.setMotorPWM(0, 0);
  robot.setMotorPWM(1, 0);
  robot.setMotorPWM(2, 0);
  robot.setMotorPWM(3, 0);*/

  // Initialization
  current_micros = micros();
  previous_micros = current_micros;
  last_motor_update_millis = millis();
}

int angle, dist2line;
char buffer[11];

void loop()
{
  static unsigned long blink_led_decimate = 0;
  int delta;
  

  //serialRead();

  current_micros = micros();
  delta = current_micros - previous_micros;

  readPicoCam(buffer, angle, dist2line);
  Serial.print("angle: ");
  Serial.print(angle);
  Serial.print(" dist: ");
  Serial.print(dist2line);
  Serial.println();

    //Serial.println();
    // parsePicoInfo(buffer, angle, dist2line);
    // Serial.print("Current micros: ");
    // Serial.print(delta);
    // Serial.print("angle: ");
    // Serial.print(angle);
    // Serial.print("dist: ");
    // Serial.print(dist2line);
    // Serial.println();
  
  //readPicoCam();

  // if (delta > kMotCtrlTimeUs)
  // {
  //   if (kMotCtrlTimeoutEnable)
  //   {
  //     checkMotorsTimeout();
  //   }

  //   if (!timeout)
  //   {
  //     previous_micros = current_micros;

  //     // Update and send data
  //     robot.update(delta);
  //     robot.send();

  //     // Debug (Serial Monitor) >>> uncomment this line to see in Serial Monitor
  //     serialWrite('\n');

  //     // Blink LED
  //     blink_led_decimate++;
  //     if (blink_led_decimate >= kMotCtrlLEDOkCount)
  //     {
  //       if (builtin_led_state == LOW)
  //       {
  //         builtin_led_state = HIGH;
  //       }
  //       else
  //       {
  //         builtin_led_state = LOW;
  //       }

  //       digitalWrite(LED_BUILTIN, builtin_led_state);
  //       blink_led_decimate = 0;
  //     }
  //   }
  // }
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
  }
}





void serialWrite(uint8_t b)
{
  Serial.write(b);
}





void serialWriteChannel(char channel, int32_t value)
{
  serial_channels.send(channel, value);
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



void readPicoCam(char *buffer, int &angle, int &dist2line)
{
  if(Serial4.available() > 0) {
    Serial4.readBytes(buffer, 11);
    parsePicoInfo(buffer, angle, dist2line);
  }

  // int bytes_rcv = Serial4.available();
  // //char buffer[11];
  // if (bytes_rcv > 0)
  // {
  //   // read the incoming byte:
  //   //char incomingByte = Serial4.read();
  //   Serial4.readBytes(buffer, bytes_rcv);

  //   // say what you got:
  //   // Serial.print("I received: ");
    
  //   // for(int i = 0; i < 11; i++){
  //   //   Serial.print(buffer[i]);
  //   // }
  //   // Serial.println();
  // }

  // if(Serial4.available() > 0){
  //   buffer = Serial4.readStringUntil(';');
  //   Serial.print("I received: ");
  //   Serial.println(buffer);
  // }
}


void parsePicoInfo(char *buffer, int &angle, int &dist2line)
{
  //#XXX YYY;\n
  //ler valores entre # e ' ' e converter para int angle 
  //talvez atoi?
  //ler valores entre ' ' e ';' e converter para int dist2line
  //talvez atoi?
  angle = strtol(buffer + 1, NULL, 10);
  dist2line = strtol(buffer + 6, NULL, 10);
}