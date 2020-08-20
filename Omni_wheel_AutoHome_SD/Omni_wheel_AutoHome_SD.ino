// Author: baotriet110@gmail.com

#include <AccelStepper.h>
#include <PS2X_lib.h>
#include <SPI.h>
#include <SD.h>

//#define DEBUG
#define SD_DEBUG

#define SEL_SD 10

#define ENA_PIN 8
// Motor 1
#define DIR_1 2
#define STEP_1 5
// Motor 2
#define DIR_2 3
#define STEP_2 6
// Motor 3
#define DIR_3 4
#define STEP_3 7

#define MAXSPEED 3000

PS2X ps2x;
// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_3, DIR_3);


// file name to use for writing
const char filename[] = "omni.txt";
bool isOpen = false;
// File object to represent file
File txtFile;

// string to buffer output
char dataString[100];

unsigned long lastMillis = 0;

void setup()
{
  Serial.begin(9600);
#ifdef SD_DEBUG
  Serial.print(F("Initializing SD card..."));
#endif // SD_DEBUG
  // see if the card is present and can be initialized:
  if (!SD.begin(SEL_SD)) {
#ifdef SD_DEBUG
    Serial.println(F("Card failed, or not present"));
#endif // SD_DEBUG
    // don't do anything more:
    while (1);
  }
  // SD.remove(filename);
#ifdef SD_DEBUG
  Serial.println(F("DONE."));
#endif // SD_DEBUG
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  int error = ps2x.config_gamepad(A3, A2, A1, A0);
#ifdef DEBUG
  Serial.println(error);
#endif // DEBUG
  stepper1.setMaxSpeed(MAXSPEED);
  stepper1.setSpeed(1000);

  stepper2.setMaxSpeed(MAXSPEED);
  stepper2.setSpeed(1000);

  stepper3.setMaxSpeed(MAXSPEED);
  stepper3.setSpeed(1000);
}

void loop()
{
  ps2x.read_gamepad();

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
    Serial.println(F("Moving right"));
#endif //DEBUG
    setMotorSpeed(MAXSPEED, 0, 0);
  }

  else if (ps2x.Button(PSB_PAD_LEFT)) {
    // Moving left
#ifdef DEBUG
    Serial.println(F("Moving left"));
#endif //DEBUG
    setMotorSpeed(-MAXSPEED, 0, 0);
  }

  else if (ps2x.Button(PSB_PAD_DOWN)) {
    // Moving backward
#ifdef DEBUG
    Serial.println(F("Moving backward"));
#endif //DEBUG
    setMotorSpeed(0, -MAXSPEED, 0);
  }

  else if (ps2x.Button(PSB_RED))            //will be TRUE if button was JUST pressed
  {
#ifdef DEBUG
    Serial.println(F("Rotating right"));
#endif //DEBUG
    setMotorSpeed(0, 0, -MAXSPEED / 2);
  }


  else if (ps2x.Button(PSB_PINK))            //will be TRUE if button was JUST released
  {
#ifdef DEBUG
    Serial.println(F("Rotating left"));
#endif //DEBUG
    setMotorSpeed(0, 0,  MAXSPEED / 2);
  }

  else if (ps2x.Button(PSB_GREEN))            //will be TRUE if button was JUST released
  {
#ifdef DEBUG
    Serial.println(F(""));
#endif //DEBUG

    if (!isOpen)
    {
      txtFile = SD.open(filename, FILE_WRITE);
      if (!txtFile)
      {
#ifdef SD_DEBUG
        Serial.println(F("error opening"));
        Serial.println(filename);
#endif // SD_DEBUG
        while (1);
      }
      isOpen = true;
#ifdef SD_DEBUG
      Serial.println(F("Ready"));
#endif // SD_DEBUG
    }
    else
    {
#ifdef SD_DEBUG
      Serial.println(F("Already opened"));
#endif // SD_DEBUG
    }
  }

  else if (ps2x.Button(PSB_BLUE))            //will be TRUE if button was JUST released
  {
#ifdef SD_DEBUG
    Serial.println(F("Closing file"));
#endif //SD_DEBUG
    if (isOpen)
    {
      txtFile.close();
      isOpen = false;
    }
    else
    {
#ifdef SD_DEBUG
      Serial.println(F("Already closed"));
#endif //SD_DEBUG
    }
  }
  else if (ps2x.Button(PSB_L1))
  {
    int ly, lx, ry;// rx;
    ly = map(ps2x.Analog(PSS_LY), 0, 255, MAXSPEED, -MAXSPEED);
    lx = map(ps2x.Analog(PSS_LX), 0, 255, -MAXSPEED, MAXSPEED);
    ry = map(ps2x.Analog(PSS_RY), 0, 255, MAXSPEED, -MAXSPEED);
    //    rx = map(ps2x.Analog(PSS_RX), 0, 255, -MAXSPEED, MAXSPEED);
#ifdef DEBUG
    Serial.println(F("Move"));
#endif //DEBUG
    setMotorSpeed(lx, ly, ry);
  }

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

  else if (ps2x.Button(PSB_R1))
  {
#ifdef SD_DEBUG
    Serial.println(F("Reading data from SD card"));
#endif //SD_DEBUG
    int *p;
    p = new int[4];
    String buf;
    txtFile = SD.open(filename, FILE_READ);
    if (txtFile)
    {
      int numLine = txtFile.size() / 23; // 23 is length of writed data _ 21+2
      for (int i = numLine - 1; i >= 0; --i)
      {
        // Use seek() to point to the line where it should read
        txtFile.seek(i * 23);
        // Use parseInt() to get the integer number including minus sign
        p[0] = txtFile.parseInt();
        p[1] = txtFile.parseInt();
        p[2] = txtFile.parseInt();
        p[3] = txtFile.parseInt();
//                  Serial.print(p[0]);
//                  Serial.print(" ");
//                  Serial.print(p[1]);
//                  Serial.print(" ");
//                  Serial.print(p[2]);
//                  Serial.print(" ");
//                  Serial.println(p[3]);
        autoOmni(p);
      }
      txtFile.close();
    }
    delete [] p;

#ifdef SD_DEBUG
    Serial.println(F("Reading data from SD card"));
#endif //SD_DEBUG
  }

  else if (ps2x.Button(PSB_L3))
  {
#ifdef SD_DEBUG
    Serial.println(F("Remove SDcard file"));
#endif //SD_DEBUG
    SD.remove(filename);
#ifdef SD_DEBUG
    Serial.println(F("Remove DONE."));
#endif //SD_DEBUG
  }

  else
  {
#ifdef DEBUG
    Serial.println(F("STOP"));
#endif //DEBUG
    setMotorSpeed(0, 0, 0);
  }
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

  u1 = (-0.50 * verVel - 0.86603 * horVel + 1.00 * rotVel);
  u2 = (-0.50 * verVel + 0.86603 * horVel + 1.00 * rotVel);
  u3 = (1.00 * verVel + 0.0 * horVel + 1.00 * rotVel);

  stepper1.setSpeed(u1);
  stepper2.setSpeed(u2);
  stepper3.setSpeed(u3);
  if (isOpen)
  {
    writeSD(u1, u2, u3);
  }
}

void writeSD(int storeVer, int storeHor, int storeRot)
{
  // check if it's been over 10 ms since the last line added
  unsigned long now = millis();
  unsigned long moveTime = now - lastMillis;
  // add a new line to the buffer

  if (storeVer != 0 || storeHor != 0 || storeRot != 0)
  {
    sprintf(dataString, "%d,%d,%d,%ld", storeVer, storeHor, storeRot, moveTime);
    // create the same size for each line - 21 character
    String temp_dataString;
    int difString;
    temp_dataString = String(dataString);
    difString = 21 - temp_dataString.length();
    while (difString--)
    {
      temp_dataString += "*";
    }
#ifdef SD_DEBUG
    Serial.println(F("Pre-Writing..."));
    Serial.println(dataString);
    Serial.println(temp_dataString.length());
#endif // SD_DEBUG
    txtFile.println(temp_dataString);
  }
  lastMillis = now;
}

void autoOmni(int *p)
{
  double autoVel[3];
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper1.moveTo((long) - p[0]*p[3] / 1000L);
  stepper2.moveTo((long) - p[1]*p[3] / 1000L);
  stepper3.moveTo((long) - p[2]*p[3] / 1000L);
#if 0 //Reverse the calculation
  autoVel[0] = -((double)p[0] + (double)p[1] - 2.0 * (double)p[2]) / 3.0;
  autoVel[1] = (((double)p[1] - (double)p[0]) / 2.0 ) / 0.86603;
  autoVel[2] = ((double)p[0] + (double)p[1] + (double)p[2]) / 3.0;
  setMotorSpeed(-autoVel[0], -autoVel[1], -autoVel[2]);
#else // Just reverse the speed
  stepper1.setSpeed(-p[0]/2);
  stepper2.setSpeed(-p[1]/2);
  stepper3.setSpeed(-p[2]/2);
#endif
  while (stepper1.distanceToGo())
  {
    stepper1.runSpeedToPosition();
    stepper2.runSpeedToPosition();
    stepper3.runSpeedToPosition();
  }
}
