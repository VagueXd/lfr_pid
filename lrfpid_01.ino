#include <SparkFun_TB6612.h>

/* RLS08 Variables */
const uint16_t sensorCount = 6;
uint16_t sensorValues[sensorCount];
int pos = 0;
int val[6];
int sensors[6] = {A0, A1, A2, A3, A4, A5};

/* PID variables */
float kp = 0.1;
float ki = 2;
float kd = 0.01;

int P;
int I;
int D;

/* Global variables */
int lastError = 0;
bool onoff = false;

/* Speed variables */
const uint8_t maxspeed1 = 150;
const uint8_t maxspeed2 = 150;
const uint8_t basespeed1 = 100;
const uint8_t basespeed2 = 100;

/* TB6612fng PinOuts */
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

const int offsetA = 1;
const int offsetB = -1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

/* General PinOuts */
int btnStart = 3;

void setup()
{
  Serial.begin(9600);
  pinMode(btnStart, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  motor1.standby();
  motor2.standby();
}

void loop()
{
  /* --------------------- */
  int den = 0;
  int num = 0;

  for (int i = 0; i < sensorCount; i++)
  {
    val[i] = digitalRead(sensors[i]);
  }

  for (int i = 0; i < 6; i++)
  {
    num += val[i] * ((i + 1) * 1000);
    den += val[i];
  }
  pos = num / den;
  /* --------------------- */

  bool onoff = false;
  if (digitalRead(btnStart) == HIGH)
  {
    onoff = !onoff;
  }
  if (onoff)
  {
    PID_control();
  }
  else
  {
    motor1.standby();
    motor2.standby();
  }
}

void PID_control()
{
  int error = pos - 3500;
  int error1 = error - lastError;
  int error2 = (2.0 / 3.0) * error2 + error;
  int motorSpeed = (kp * error) + (kd * error1) + (ki * error2);
  int motorSpeed1 = basespeed1 - motorSpeed;
  int motorSpeed2 = basespeed2 + motorSpeed;
  if (motorSpeed1 > maxspeed1) 
    {motorSpeed1 = maxspeed1;}
  if (motorSpeed2 > maxspeed2)
    {motorSpeed2 = maxspeed2;}
  if (motorSpeed1 < 0)
    {motorSpeed1 = 0;}
  if (motorSpeed2 < 0)
    {motorSpeed2 = 0;}
  lastError = error;
  motor1.drive(motorSpeed1);
  motor2.drive(motorSpeed2);
  if (sensorValues[0] == 1 || sensorValues[6] == 1)
    { 
      motor1.drive(maxspeed1);
      motor2.drive(maxspeed2);
      return;
    }
    if (sensorValues[0] == 1 && sensorValues[1] ==1 && sensorValues[2] == 1 && sensorValues[3] == 1 && sensorValues[4] == 1 && sensorValues[5] == 1 && sensorValues[6]== 1 )
    {
      
      motor1.drive(maxspeed1);
      motor2.drive(maxspeed2);
      return;
    }
}