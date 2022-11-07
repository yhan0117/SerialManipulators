#include <Wire.h>
#include <Servo.h>
#include <JY901.h>
#define high 2000
#define low 1000

Servo right_prop;
Servo left_prop;

//System states
float angle;
float speed;

unsigned long elapsedTime, time, timePrev;
float PID, pwmLeft, pwmRight, error, previous_error;
float encoderPos, previous_encoderPos = 0;

int i;

float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

//Gain Coefficients
float kp = 3.55;
float ki = 0;
float kd = 2.05;

//Initial value of throttle to the motors
float throttle = 1250;

//Desired angle
float desired_angle = 0;


void setup() {
  Serial.begin(9600);
  right_prop.attach(10);  //attatch the right motor to pin 3
  left_prop.attach(11);   //attatch the left motor to pin 5

  time = millis();  //Start counting time in milliseconds
  Calibrate();
  
  Wire.begin();  //Begins I2C communication at pin (A4,A5)
}

void loop() {

  /*///////////////////////////Feedback///////////////////////////////////*/

  //time variables
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;

  //IMU
  angle = (float)JY901.stcAngle.Angle[1] / 32768 * 180;
  Serial.println(angle);

  //Encoder
  Wire.requestFrom(8, 1);            // request 1 byte from slave arduino (8)
  byte MasterReceive = Wire.read();  // receive a byte from the slave arduino and store in MasterReceive

  //negating negative angles
  if (MasterReceive >= 128) {
    encoderPos = MasterReceive - 256;
  } else {
    encoderPos = MasterReceive;
  }

  //600 pulses per revolution
  encoderPos = encoderPos / 600 * 360;
  speed = (encoderPos - previous_encoderPos) / elapsedTime;

  previous_encoderPos = encoderPos;

  /*///////////////////////////P I D///////////////////////////////////*/

  //Error
  error = desired_angle - angle;

  //Proportional Gain
  pid_p = kp * error;

  //Integral Gain
  if (-3 < error < 3) {
    pid_i = pid_i + (ki * error);
  }

  //Derivative Gain
  pid_d = kd * speed;

  //Total Gain
  PID = pid_p + pid_i + pid_d;

  //Saturation
  if (PID < -150) {
    PID = -150;
  }
  if (PID > 150) {
    PID = 150;
  }

  //Calculated total actuation value
  pwmLeft = throttle - PID;
  pwmRight = throttle + PID;

  //Saturation
  //Right
  if (pwmRight < 1100) {
    pwmRight = 1100;
  }
  if (pwmRight > 1400) {
    pwmRight = 1400;
  }
  //Left
  if (pwmLeft < 1100) {
    pwmLeft = 1100;
  }
  if (pwmLeft > 1400) {
    pwmLeft = 1400;
  }

  //Actuation
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
}

void serialEvent() {
  while (Serial.available()) {
    JY901.CopeSerialData(Serial.read());  //Call JY901 data cope function
  }
}

void Calibrate() {
  Serial.println("ESC calibration begin...");
  Serial.print("Writing maximum output: ");
  Serial.println(high);
  Serial.println("Plug battery in, then wait 2 seconds and press Ctrl + Enter");

  left_prop.writeMicroseconds(high);
  right_prop.writeMicroseconds(high);

  // Wait for input
  while (!Serial.available())
    ;
  Serial.read();

  // Send min output
  Serial.print("Sending minimum output: ");
  Serial.println(low);
  left_prop.writeMicroseconds(low);
  right_prop.writeMicroseconds(low);
  delay(5000);
  Serial.println("ESCs are calibrated.");
  Serial.println("Unplug and replug battery then press Ctrl + Enter");

  // Wait for input
  while (!Serial.available())
    ;
  Serial.read();

  Serial.println("Program begin...");
}