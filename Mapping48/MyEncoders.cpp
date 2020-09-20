//implement in this document the functions from MyEncoders.h
#include <PinChangeInt.h>
#include "MyEncoders.h"

#define ENCODER_R 11
#define ENCODER_L 10

namespace {
  //this is a nameless namespace
  //put here variables only viewable to this module
}

int leftCounts = 0;
int rightCounts = 0;
unsigned long prevMillisL = 0;
unsigned long prevMillisR = 0;
unsigned long timeBetweenTicksL = 0;
unsigned long timeBetweenTicksR = 0;

void resetCounts ()
{
  leftCounts = 0;
  rightCounts = 0;
}

void getCounts (int counts[])
{
  counts[0] = leftCounts;
  counts[1] = rightCounts;
}

void getSpeeds (float speeds[])
{
    speeds[0] = (1.0/32.0) / (max((millis() - prevMillisL), timeBetweenTicksL)) * 1000;
    speeds[1] = (1.0/32.0) / (max((millis() - prevMillisR), timeBetweenTicksR)) * 1000;
  if (prevMillisL == 0)
    speeds[0] = 0;
  if (prevMillisR == 0)
    speeds[1] = 0;
}

void initEncoders ()
{
  //Serial.begin(115200);
  pinMode(ENCODER_R, INPUT_PULLUP);
  pinMode(ENCODER_L, INPUT_PULLUP);
  PCintPort::attachInterrupt(ENCODER_L, &setTimeBetweenTicksL, FALLING);
  PCintPort::attachInterrupt(ENCODER_R, &setTimeBetweenTicksR, FALLING);
}

void setTimeBetweenTicksL()
{
  ::timeBetweenTicksL = (millis() - prevMillisL);
  prevMillisL = millis();
  leftCounts++;
}

void setTimeBetweenTicksR()
{
  ::timeBetweenTicksR = (millis() - prevMillisR);
  prevMillisR = millis();
  rightCounts++;
}

/*unsigned long getTimeBetweenTicksR()
{
  return timeBetweenTicksR;
}
*/


