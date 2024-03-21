/*
  SparkFun Inventor’s Kit
  Circuit 5B - Remote Control Robot

  Control a two wheeled robot by sending direction commands through the serial monitor.
  This sketch was adapted from one of the activities in the SparkFun Guide to Arduino.
  Check out the rest of the book at
  https://www.sparkfun.com/products/14326

  This sketch was written by SparkFun Electronics, with lots of help from the Arduino community.
  This code is completely free for any use.

  View circuit diagram and instructions at: https://learn.sparkfun.com/tutorials/sparkfun-inventors-kit-experiment-guide---v40
  Download drawings and code at: https://github.com/sparkfun/SIK-Guide-Code

  Modified by:
  Carlos Jarro for University of Kentucky's BAE305 Lab 6
  02/28/2024
*/

#include <PID_v2.h>

//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 13;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor

const int trigPin = 6;        //trigger pin for distance snesor
const int echoPin = 7;        //echo pin for distance sensor
const int RED = 3;
const int GREEN = 2;

String botDirection;           //the direction that the robot will drive in (this change which direction the two motors spin in)
String motorSpeedStr;

int motorSpeed;               //speed integer for the motors
float duration, distance;     //duration and distance for the distance sensor

double setpoint = 15; 
double measurement = 0; 
double output = 0; 
double Kp = 8; 
double Ki = 1; 
double Kd = 1; 
int speed = 0; 

PID myPID(&measurement, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/********************************************************************************/
void setup()
{
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);

  //set the motor control pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  
  //set the distance sensor trigger pin as output and the echo pin as input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(RED,OUTPUT);
  pinMode(GREEN,OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer

  //prompt the user to enter a command
  Serial.println("Enter a direction followed by speed.");
  Serial.println("f = forward, b = backward, r = turn right, l = turn left, s = stop");
  Serial.println("Example command: f 50 or s 0");
}

/********************************************************************************/
void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*340/10000)/2; // Units are cm
  measurement = distance; 
  myPID.Compute();

  speed = (map(output, 0, 255, 150, 255));
  Serial.print(distance);
  Serial.print(" ,");
  Serial.print(setpoint);
  Serial.print(" ,");
  Serial.println(speed);
  
//  Serial.println(distance);
    delayMicroseconds(50);

  //if (Serial.available() > 0)                         //if the user has sent a command to the RedBoard
  //{
  //  botDirection = Serial.readStringUntil(' ');       //read the characters in the command until you reach the first space
  //  motorSpeedStr = Serial.readStringUntil(' ');           //read the characters in the command until you reach the second space
    //motorSpeed = motorSpeedStr.toInt();
  //}
  if (true)
  {                                                     //if the switch is in the ON position
  motorSpeed = speed; 
    if (measurement > setpoint)
    {
      botDirection = "b"; 
    }
      else {
      botDirection = "f";
    }

    if (botDirection == "f")                           //if the entered direction is forward
    {
      rightMotor(-motorSpeed);                                //drive the right wheel forward
      leftMotor(motorSpeed);                                 //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "b")                    //if the entered direction is backward
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "r")                     //if the entered direction is right
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(motorSpeed);                                 //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "l")                   //if the entered direction is left
    {
      rightMotor(-motorSpeed);                                //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
      digitalWrite(GREEN,HIGH);
      digitalWrite(RED,LOW);
    }
    else if (botDirection == "s")
    {
      rightMotor(0);
      leftMotor(0);
      digitalWrite(GREEN,LOW);
      digitalWrite(RED,HIGH);
    }
  }
  else if (distance < 20)
  {
    if (botDirection == "b")
    {
      rightMotor(motorSpeed);                               //drive the right wheel forward
      leftMotor(-motorSpeed);                                //drive the left wheel forward
    }
    else
    {
      //Serial.print("Object Detected at ");
      //Serial.print(distance);
      //Serial.println(" cm");
      rightMotor(0);                                  //turn the right motor off
      leftMotor(0);                                   //turn the left motor off
    }
  }
}
/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backward (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}