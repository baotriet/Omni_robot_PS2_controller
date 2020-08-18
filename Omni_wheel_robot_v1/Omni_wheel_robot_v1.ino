// Omni_wheel_robot
// Author: baotriet110@gmail.com

#include <AccelStepper.h>
#include <PS2X_lib.h>

#define DEBUG

#define ENA_PIN 8 // 3 stepper drivers have the same enable pin
// Motor 1
#define DIR_1 2
#define STEP_1 5
// Motor 2
#define DIR_2 3
#define STEP_2 6
// Motor 3
#define DIR_3 4
#define STEP_3 7

#define MAXSPEED 5000

PS2X ps2x;
// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_3, DIR_3);

void setup()
{
  Serial.begin(57600);
  // Set ENA_PIN to LOW to enable the drivers
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  ps2x.config_gamepad(13, 11, 10, 12);  //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  // Configure the speed for each motor. They will run with the constant speed
  stepper1.setMaxSpeed(MAXSPEED);
  stepper1.setSpeed(1000);

  stepper2.setMaxSpeed(MAXSPEED);
  stepper2.setSpeed(1000);

  stepper3.setMaxSpeed(MAXSPEED);
  stepper3.setSpeed(1000);
}

void loop()
{
  ps2x.read_gamepad(); //Read the message transmitted from the joystick

  if (ps2x.Button(PSB_PAD_UP)) {        //will be TRUE as long as button is pressed
    // Moving forward
#ifdef DEBUG
    Serial.println("Moving forward");
#endif //DEBUG
    setMotorSpeed(0, MAXSPEED, 0);
  }

  else if (ps2x.Button(PSB_PAD_RIGHT)) {
    // Moving right
#ifdef DEBUG
    Serial.println("Moving right");
#endif //DEBUG
    setMotorSpeed(MAXSPEED, 0, 0);
  }

  else if (ps2x.Button(PSB_PAD_LEFT)) {
    // Moving left
#ifdef DEBUG
    Serial.println("Moving left");
#endif //DEBUG
    setMotorSpeed(-MAXSPEED, 0, 0);
  }

  else if (ps2x.Button(PSB_PAD_DOWN)) {
    // Moving backward
#ifdef DEBUG
    Serial.println("Moving backward");
#endif //DEBUG
    setMotorSpeed(0, -MAXSPEED, 0);
  }

  else if (ps2x.Button(PSB_RED))            //will be TRUE if button was JUST pressed
  {
#ifdef DEBUG
    Serial.println("Rotating right");
#endif //DEBUG
    setMotorSpeed(0, 0, -MAXSPEED / 2);
  }

  else if (ps2x.Button(PSB_PINK))            //will be TRUE if button was JUST released
  {
#ifdef DEBUG
    Serial.println("Rotating left");
#endif //DEBUG
    setMotorSpeed(0, 0,  MAXSPEED / 2);
  }
  // Left joystick is used to control the movement of the robot
  else if (ps2x.Button(PSB_L1))
  {
    int ly, lx, ry, rx;
    ly = map(ps2x.Analog(PSS_LY), 0, 255, 5000, -5000);
    lx = map(ps2x.Analog(PSS_LX), 0, 255, -5000, 5000);
    ry = map(ps2x.Analog(PSS_RY), 0, 255, 5000, -5000);
    rx = map(ps2x.Analog(PSS_RX), 0, 255, -5000, 5000);
#ifdef DEBUG
    Serial.println("Move");
#endif //DEBUG
    setMotorSpeed(lx, ly, ry);
  }
  // It is a good way to rotate the robot
  else if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2))
  {
    int valRotate = ps2x.Analog(PSS_LY) - ps2x.Analog(PSS_RY);
    if (valRotate > 0) //Rotate right
    {
      valRotate = map(valRotate, 0, 255, 0, 5000);
    }
    else //Rotate left
    {
      valRotate = map(valRotate, -255, 0, -5000, 0);
    }
    setMotorSpeed(0, 0, valRotate);
  }

  else
  {
#ifdef DEBUG
    Serial.println("STOP");
#endif //DEBUG
    setMotorSpeed(0, 0, 0);
  }
  // It needs to run amount of time before reading other messages. If not, It cannot run smoothly
  for (int i = 0; i < 3000; i++)
  {
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();
  }
}

void setMotorSpeed(double verVel, double horVel, double rotVel)
{
  double u1, u2, u3;
  // Algorithm to calculate the desired speed of each motor
  u1 = (-0.50 * verVel - 0.86603 * horVel + 1.00 * rotVel);
  u2 = (-0.50 * verVel + 0.86603 * horVel + 1.00 * rotVel);
  u3 = (1.00 * verVel + 0.0 * horVel + 1.00 * rotVel);

  stepper1.setSpeed(u1);
  stepper2.setSpeed(u2);
  stepper3.setSpeed(u3);
}
