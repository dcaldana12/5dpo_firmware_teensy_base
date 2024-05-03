#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include <channels.h>

#include "Robot.h"

#include <string>

#define BNO0X_RESET -1

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

Adafruit_BNO08x bno(BNO0X_RESET);
sh2_SensorValue_t sensorValue;

Encoder *encoders = robot.enc;

float angle, dist2line;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;


/******************************************************************************
 * FUNCTIONS HEADERS
 ******************************************************************************/

void processSerialPacket(char channel, uint32_t value, channels_t& obj);
void serialWrite(uint8_t b);
void serialWriteChannel(char channel, int32_t value);
void serialRead();
void checkMotorsTimeout();
void readPicoCam();
void parseData();
void setReports(void);
void getImuData(void);




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
  Serial4.begin(115200); // arducam
  serial_channels.init(processSerialPacket, serialWrite);

  // Reset signal
  serialWriteChannel('r', 0);

  // I2C
  Wire.begin();

  // Imu
  if(!bno.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Test PWM motors
  /*robot.setMotorPWM(0, 0);
  robot.setMotorPWM(1, 0);
  robot.setMotorPWM(2, 0);
  robot.setMotorPWM(3, 0);*/

  // Initialization
  current_micros = micros();
  previous_micros = current_micros;
  last_motor_update_millis = millis();

  setReports();
}

//int ten=0, twenty=0;

void loop()
{
  static unsigned long blink_led_decimate = 0;
  unsigned long delta;

  if (bno.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  serialRead();

  current_micros = micros();
  delta = current_micros - previous_micros;

  readPicoCam();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          // because strtok() used in parseData() replaces the commas with \0
      parseData();
      newData = false;

      /* Serial.print("angle: ");
      Serial.print(angle);
      Serial.print(" dist: ");
      Serial.print(dist2line);
      Serial.print(" deltaT: ");
      Serial.print(delta);
      previous_micros = current_micros;

      if (delta < 12000)
        ten++;
      else if(delta < 30000)
        twenty++;
      Serial.print(" 20ms: ");
      Serial.print(twenty);  
      Serial.print(" 10ms: ");
      Serial.println(ten);   */
  }

  getImuData();

  if (delta > kMotCtrlTimeUs)
  {
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

      //Debug (Serial Monitor) >>> uncomment this line to see in Serial Monitor
      Serial.print(" Encoder tick: ");
      Serial.println(robot.enc[0].tick);
      //serialWrite('\n');

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

void readPicoCam()
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '#';
  char endMarker = ';';
  char rc;

  while (Serial4.available() > 0 && newData == false) {
      rc = Serial4.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          } else {
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = false;
              ndx = 0;
              newData = true;
          }
      } else if (rc == startMarker) {
          recvInProgress = true;
      }
  }
}

void parseData() {
  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, " ");       
  angle = atoi(strtokIndx);           // convert this part to an integer
  angle = (angle / 1000.0f) * (180.0f / 3.1415926f); //divide by 1000 and convert to deg.

  strtokIndx = strtok(NULL, " ");     // this continues where the previous call left off
  dist2line = atoi(strtokIndx);       // convert this part to an integer
  dist2line = dist2line / 1000.0f;

  newData = false;
}


// Here is where you define the sensor outputs you want to receive: Report type and report rate
void setReports(void)
{
  Serial.print("Setting desired reports");
  if(!bno.enableReport(SH2_ACCELEROMETER, 10000UL)) {
    Serial.print("Could not enable accelerometer");
  }
}



void getImuData(void) 
{
  if(bno.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId)
    {
    case SH2_ACCELEROMETER:
      Serial.print("Accelerometer X:");
      Serial.print(sensorValue.un.accelerometer.x);
      Serial.print(" Y:");
      Serial.print(sensorValue.un.accelerometer.y);
      Serial.print(" Z:");
      Serial.println(sensorValue.un.accelerometer.z);
      break;
    
    default:
      Serial.print("Unknown sensor ID: ");
      Serial.println(sensorValue.sensorId);
      break;
    }
  }
}