//implement in this document the functions from MyServos.h
#define _USE_MATH_DEFINES
#define W_DIAMETER 2.61
#define D_AXIS 1.975
#include "MyEncoders.h"
#include "MyServos.h"
#include <Servo.h>
#include <Arduino.h>
#include <math.h>

namespace {
  //this is a nameless namespace
  //put here variables only viewable to this module
}

Servo LServo;
Servo RServo;
float speedsArray[2];
float speedsDataL[11];
float speedsDataR[11];
int rMicros = 0;
int lMicros = 0;
bool rMicrosFound = false;
bool lMicrosFound = false;
int rpsControl;

void setSpeeds(int microsLeft, int microsRight)
{
     LServo.writeMicroseconds(microsLeft + 1500);
     RServo.writeMicroseconds(1500 - microsRight);
     //Serial.println("Set acknowledge");
}

void calibrate()
{
 //Provide greater delay for speed to stabilize upon initial writing
 setSpeeds(50, 50);
 delay(500);
 for (int i = 50; i <= 150; i = i + 10)
 {
  setSpeeds(i, i);
  delay(800);
  getSpeeds(speedsArray);
  ::speedsDataL[(i - 50)/ 10] = speedsArray[0];
  ::speedsDataR[(i - 50)/ 10] = speedsArray[1];
  /*
  Serial.print("speedsDataL[");
  Serial.print(i / 10);
  Serial.print("]: ");
  Serial.println(speedsDataL[i / 10]);
  Serial.print("speedsDataR[");
  Serial.print(i / 10);
  Serial.print("]: ");
  Serial.println(speedsDataR[i / 10]);
  */
 }
 setSpeeds(0, 0);
}

void setSpeedsRPS(float rpsLeft, float rpsRight)
{
  rpsControl = 0;
  lMicrosFound = false;
  rMicrosFound = false;
  while (rpsControl < 11)
  {
    
    if (rpsLeft < 0)
    {
        lMicros = 0;
        lMicrosFound = true;
    }
    
    if (rpsLeft > 0)
    {
      if ((speedsDataL[rpsControl] <= rpsLeft) && (speedsDataL[rpsControl + 1] >= rpsLeft))
      {
        if ((rpsLeft - speedsDataL[rpsControl]) >= (speedsDataL[rpsControl + 1] - rpsLeft))
        {
        lMicros = 10 * (rpsControl + 1) + 50;
        lMicrosFound = true;
        }
        else
        {
        lMicros = 10 * rpsControl + 50;
        if (lMicros == 0)
          lMicros = 10;
        lMicrosFound = true;
        }
      }
    }
    
    if (rpsRight < 0)
    {
      rMicros = 0;
      rMicrosFound = true;
    }

    if (rpsRight > 0)
    {
      if ((speedsDataR[rpsControl] <= rpsRight) && (speedsDataR[rpsControl + 1] >= rpsRight))
      {
      if ((rpsRight - speedsDataR[rpsControl]) >= (speedsDataR[rpsControl + 1] - rpsRight))
      {
        rMicros = 10 * (rpsControl + 1) + 50;
        rMicrosFound = true;
      }
      else
      {
        rMicros = 10 * rpsControl + 50;
        if (rMicros == 0)
          rMicros = 10;
        rMicrosFound = true;
      }
      }
    }
    /*
      if ((speedsDataR[rpsControl] <= rpsRight) && (speedsDataR[rpsControl + 1] >= rpsRight))
      {
        if ((rpsRight - speedsDataR[rpsControl]) >= (speedsDataR[rpsControl + 1] - rpsRight))
        {
        rMicros = 10 * (rpsControl + 1);
        rMicrosFound = true;
        }
        else
        {
          rMicros = 10 * rpsControl;
          rMicrosFound = true;
        }
      }
      */
      
    if (lMicrosFound && rMicrosFound)
    {
      break;
    }
    rpsControl++;
  }
  if (rpsLeft > arrayMax(speedsDataL, 11) && lMicrosFound == false)
  {
    //Serial.println("The requested left wheel speed is too high.");
    if (rpsLeft > 0)
      lMicros = 200;
    lMicrosFound = true;
  }
  if (rpsRight > arrayMax(speedsDataR, 11) && rMicrosFound == false)
  {
    //Serial.println("The requested right wheel speed is too high.");
    if (rpsRight > 0)
      rMicros = 200;
    rMicrosFound = true;
  }
/*
  Serial.print("lMicros: ");
  Serial.println(lMicros);
  Serial.print("rMicros: ");
  Serial.println(rMicros);
  Serial.println(rpsLeft);
  Serial.println(rpsRight);
  */
  if (!lMicrosFound)
    lMicros = 0;
  if (!rMicrosFound)
    rMicros = 0;
  
  setSpeeds(lMicros, rMicros);
}

void setSpeedsIPS(float ipsLeft, float ipsRight)
{
 setSpeedsRPS((ipsLeft / (W_DIAMETER * M_PI)), (ipsRight / (W_DIAMETER * M_PI)));
}

void setSpeedsvw(float v, float w)
{
 setSpeedsIPS(v + D_AXIS * w, v - D_AXIS * w);
}

void setSpeedsvwCCW(float v, float w)
{
 setSpeedsIPS(v - D_AXIS * w, v + D_AXIS * w);
}

float arrayMax(float array[], int size)
{
  int i;
  float maximum = array[0];
  for (i = 1; i < size; i++)
  {
    if (array[i] > maximum)
      maximum = array[i];
  }
  return maximum;
}

void getArrays(float arrayL[], float arrayR[], int sizes)
{
  int i = 0;
  for (i; i < sizes; i++)
  {
    arrayL[i] = speedsDataL[i];
    arrayR[i] = speedsDataR[i];
  }
}




