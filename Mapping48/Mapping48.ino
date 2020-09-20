// This project shows simple ways of converting RGB values to the range 0-255
// and classifying color. You are allowed to use these files but you should modify them as you see fit.

#include "MyEncoders.h"
#include "MyServos.h"
#include "MySharpSensor.h"
#include "ColorSensor.h"
//#include <TimerOne.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
byte button;
byte select = 0;
byte startl = 1;
byte endl = 1;
byte i = 0;
byte mapping = 1;

void setup(){

  //Serial.begin(115200);
  lcd.begin(16, 2); //init lcd
  initColorSensor(lcd); 
  LServo.attach(2);
  RServo.attach(3);

/*
  Timer1.initialize(250000);
  Timer1.attachInterrupt(recognize);
*/
  
  initEncoders();
  initDistanceSensors();
  calibrateColorSensor();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Cali");
  delay(300);
  while (button != 1)
    button = lcd.readButtons();

  calibrate();

  lcd.clear();
  delay(100);
}


void loop(){
  
  button = lcd.readButtons();
  if (button == 4)
  {
    if (select == 0)
      select = 3;
    else
      select -= 1;
    lcd.clear();
    delay(50);
  }
  else if (button == 8)
  {
    if (select == 3)
      select = 0;
    else
      select += 1;
    lcd.clear();
    delay(50);
  }
  else if (button == 1)
  {
    lcd.clear();
   if (mapping == 1)
    lineDetection(startl, i);
   else
    pathPlanning(startl, endl, i);
  }

  if (select == 0)
  {
  if (button == 2)
  {
    if (startl == 16)
      startl = 1;
    else
      startl += 1;
    lcd.clear();
    delay(50);
  }
  else if (button == 16)
  {
    if (startl == 1)
      startl = 16;
    else
      startl -= 1;
    lcd.clear();
    delay(50);
  }
}
else if (select == 1)
  {
  if (button == 2)
  {
    if (endl == 16)
      endl = 1;
    else
      endl += 1;
    lcd.clear();
    delay(50);
  }
  else if (button == 16)
  {
    if (endl == 1)
      endl = 16;
    else
      endl -= 1;
    lcd.clear();
    delay(50);
  }
}
  else if (select == 2)
  {
    if (button == 2)
  {
    if (i == 3)
      i = 0;
    else
      i += 1;
    lcd.clear();
    delay(50);
  }
  else if (button == 16)
  {
    if (i == 0)
      i = 3;
    else
      i -= 1;
    lcd.clear();
    delay(50);
  }
  }
  else if (select == 3)
  {
    if (button == 2 || button == 16)
    {
      if (mapping == 0)
      {
        lcd.clear();
        mapping = 1;
        delay(50);
      }
      else
      {
        lcd.clear();
        mapping = 0;
        delay(50);
      }
    }
  }
  

  //Print menu
  lcd.setCursor(0, 0);
  if (select == 0)
  {
    lcd.print("Start Locat.");
    lcd.setCursor(0, 1);
    lcd.print(startl);
  }
  else if (select == 1)
  {
    lcd.print("End Locat.");
    lcd.setCursor(0, 1);
    lcd.print(endl);
  }
  else if (select == 2)
  {
    lcd.print("Orientation");
    lcd.setCursor(0, 1);
    if (i == 0)
      lcd.print('N');
    else if (i == 1)
      lcd.print('E');
    else if (i == 2)
      lcd.print('S');
    else if (i == 3)
      lcd.print('W');
  }
  else if (select == 3)
  {
    lcd.print("Map/Plan");
    lcd.setCursor(0, 1);
    if (mapping == 0)
      lcd.print("Plan");
    else
      lcd.print("Map");
  }
}

