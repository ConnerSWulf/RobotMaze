#include "MySharpSensor.h"
#include "Arduino.h"
#define CM_TO_IN .3937
#define analogToVoltage 5.0/1023.0
#define D_AXIS 1.975

//Short distance sensor effective range 4 to 30 cm
//Long distance sensor effective range 20 to 150 cm

namespace{
  float shortInvDistanceMap[16] = {1.0/(40.0 + .42), 1.0/(30.0 + .42), 1.0/(25.0 + .42), 1.0/(20.0 + .42), 1.0/(16.0 + .42), 1.0/(14.0 + .42), 1.0/(12.0 + .42), 1.0/(10.0 + .42), 1.0/(8.0 + .42), 1.0/(7.0 + .42), 1.0/(6.0 + .42), 1.0/(5.0 + .42), 1.0/(4.0 + .42), 1.0/(3.5 + .42), 1.0/(3.0 + .42), 1.0/(2.0 + .42)};
  float shortVoltageMap[16] = {.38, .41, .52, .64, .805, .92, 1.06, 1.25, 1.58, 1.78, 2.01, 2.33, 2.62, 3, 3.02, 2.215}; 
  //16 is the number of labeled data points on the plot of voltage vs. inverse distance for short sensors
  float longInvDistanceMap[9] = {1.0/150.0, 1.0/100.0, 1.0/60.0, 1.0/50.0, 1.0/40.0, 1.0/30.0, 1.0/20.0, 1.0/15.0, 1.0/10.0};
  float longVoltageMap[9] = {.42, .65, 1.08, 1.25, 1.55, 2, 2.54, 2.78, 2.3};
  //9 is the number of labeled data points on the plot of voltage vs. inverse distance for long sensors
  int bufferSF[7];
  int bufferSL[7];
  int bufferSR[7];
  int bufferLF[7];
  int tempBuffer[7];
  
  //buffer insertion variables
  int i = 0;
  int d = 0;
  int s = 0;
  int l = 0;
}


//To convert analogRead return value to voltage, use:
//voltage = sensorValue * (5.0/1023.0)
float shortToInches(int value)
{
  int i;
  float distance; //distance (centimeters)
  float invDistance; //inverse distance to be determined
  float mReg; //slope for linear regression
  float x;  //x for linear regression
  float y;  //y for linear regression
  
  float voltage = value * analogToVoltage;
  for (i; i < 16; i++)
  {
    if ((shortVoltageMap[i] <= voltage) && (shortVoltageMap[i + 1] >= voltage))
      break;
  }
    mReg = (shortVoltageMap[i + 1] - shortVoltageMap[i])/(shortInvDistanceMap[i + 1] - shortInvDistanceMap[i]);
    x = shortInvDistanceMap[i];
    y = shortVoltageMap[i];
    invDistance = (voltage - y)/mReg + x;
    distance = 1/invDistance - .42;
    return (distance * CM_TO_IN);
  
}


float longToInches(int value)
{
  int i;
  float distance; //distance (centimeters)
  float invDistance; //inverse distance to be determined
  float mReg; //slope for linear regression
  float x;  //x for linear regression
  float y;  //y for linear regression
  
  float voltage = value * analogToVoltage;
  for (i; i < 9; i++)
  {
    if ((longVoltageMap[i] <= voltage) && (longVoltageMap[i + 1] >= voltage))
      break;
  }
    mReg = (longVoltageMap[i + 1] - longVoltageMap[i])/(longInvDistanceMap[i + 1] - longInvDistanceMap[i]);
    x = longInvDistanceMap[i];
    y = longVoltageMap[i];
    invDistance = (voltage - y)/mReg + x;
    distance = 1/invDistance - .42;
    return (distance * CM_TO_IN);
}



void takeNewMeasurement(int sensor)
{
  //left
  if (sensor == 0)
  {
    if (i == 7)
    {
      i = 6;
      for (int c = 0; c < 6; c++)
      {
        bufferSL[c] = bufferSL[c + 1];
      }
    }
    bufferSL[i] = analogRead(A1);
    i++;
  }
  //front short
  if (sensor == 1)
  {
    if (s == 7)
    {
      s = 6;
      for (int c = 0; c < 6; c++)
      {
        bufferSF[c] = bufferSF[c + 1];
      }
    }
    bufferSF[s] = analogRead(A0);
    s++;
  }
  //right
  if (sensor == 2)
  {
    if (d == 7)
    {
      d = 6;
      for (int c = 0; c < 6; c++)
      {
        bufferSR[c] = bufferSR[c + 1];
      }
    }
    bufferSR[d] = analogRead(A2);
    d++;
  }
  //front long
  if (sensor == 3)
  {
    if (l == 7)
    {
      l = 6;
      for (int c = 0; c < 6; c++)
      {
        bufferLF[c] = bufferLF[c + 1];
      }
    }
    bufferLF[l] = analogRead(A3);
    l++;
  }
}



float getCombinedDistance(int sensor)
{
  //Left
  if (sensor == 0)
  {
    bufferSort(bufferSL, 7);
    return shortToInches(tempBuffer[3]);
  }
  //Front short
  if (sensor == 1)
  {
    bufferSort(bufferSF, 7);
    //Serial.println(tempBuffer[3]);
    return shortToInches(tempBuffer[3]);
  }
  //Right
  if (sensor == 2)
  {
    bufferSort(bufferSR, 7);
    return shortToInches(tempBuffer[3]);
  }
  //Front long
  if (sensor == 3)
  {
    bufferSort(bufferLF, 7);
    return longToInches(tempBuffer[3]);
  }
}


void initDistanceSensors()
{
  //Serial.begin(115200);
}

void bufferSort(int buff[], int s)
{
  int temp;
  int i;
  int j;
  int index;

  for (i = 0; i < s; i++)
  {
    tempBuffer[i] = buff[i];
  }
  for (i = 0; i < s; i++)
  {
    index = i;
    for (j = i + 1; j < s; j++)
    {
      if (tempBuffer[j] < tempBuffer[index])
      {
        index = j;
      }
    }
    temp = tempBuffer[i];
    tempBuffer[i] = tempBuffer[index];
    tempBuffer[index] = temp;
  }
  for (int c = 0; c < s; c++)
  {
    //Serial.print(tempBuffer[c]);
    //Serial.print(" ");
  }
  //Serial.println(" ");
}

float saturation(float w)
{
  if (w > 0)
  {
  if (w > .5)
    return .5;
  else
    return w;
  }
  else
  {
    if (-w > 1)
    return -1;
  else
    return w;
  }
}



