

//Version 45. Added walls array.
//Goal: To enable the robot to run on either leftMode or rightMode for 1800 milliseconds after entering a cell before stopping and initiating decision-making logic.
//Consideration: Add "else" before leftMode, rightMode false condition to reduce latency? Need to know whether conditions are exclusive before doing this.
//Consideration: Stop using front sensor to reduce noise? Probably not a good idea, though.

#include "ColorSensor.h"
#include "MySharpSensor.h"
#include "MyServos.h"
#include "MyEncoders.h"
#include "QueueArray.h"
#include <Arduino.h>

#define _USE_MATH_DEFINES
#define W_DIAMETER 2.61
#define D_AXIS 1.975
#include <math.h>

#define BLUE 0x4
#define RED 0x1
#define WHITE 0x7

namespace{ //this is a nameless namespace

  Adafruit_RGBLCDShield lcd;
 // bool cleared = false;

  //short checked = 0;
  bool compassMade = false;
  byte compass;
  QueueArray <byte> que;
  bool backing = false;
  
  byte n;
  byte c;
  bool cellEntered = false;
  bool pathChosen = false;

  bool lastBlue = false;
  bool lastRed = false;

  byte gr;
  byte last;
  byte endl;
  byte ori;
  byte current;
  byte pred[16] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
  

  byte i = 0;

  char state[16] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};

  byte nav[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 // bool visited[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
   //Dynamic walls
  char walls[16][4] = {{'W', 'W', 'o', 'o'}, {'o', 'W', 'o', 'o'}, {'o', 'W', 'o', 'o'}, {'o', 'W', 'W', 'o'}, 
                       {'W', 'o', 'o', 'o'}, {'o', 'o', 'o', 'o'}, {'o', 'o', 'o', 'o'}, {'o', 'o', 'W', 'o'}, 
                       {'W', 'o', 'o', 'o'}, {'o', 'o', 'o', 'o'}, {'o', 'o', 'o', 'o'}, {'o', 'o', 'W', 'o'}, 
                       {'W', 'o', 'o', 'W'}, {'o', 'o', 'o', 'W'}, {'o', 'o', 'o', 'W'}, {'o', 'o', 'W', 'W'}}; 

  /*  //hardcoded maze               
  char walls[16][4] = {{'W', 'W', 'o', 'o'}, {'o', 'W', 'o', 'W'}, {'o', 'W', 'o', 'W'}, {'o', 'W', 'W', 'o'}, 
                       {'W', 'o', 'W', 'o'}, {'W', 'W', 'o', 'o'}, {'o', 'W', 'o', 'W'}, {'o', 'o', 'W', 'o'}, 
                       {'W', 'o', 'W', 'o'}, {'W', 'o', 'o', 'W'}, {'o', 'W', 'W', 'o'}, {'W', 'o', 'W', 'o'}, 
                       {'W', 'o', 'o', 'W'}, {'o', 'W', 'o', 'W'}, {'o', 'o', 'W', 'W'}, {'W', 'o', 'W', 'W'}}; */

  bool redFound = false;
  bool blueFound = false;
  unsigned long greenFound = 0; //*********************

  float rgb[3];
  double hsl[3];
  int raw[3];
  double dRed;
  double dBlue;
  double dGreen;

  unsigned long wallTime;     //*********************

  float left;
  float right;
  float front;

  bool timerBegin = false;
  unsigned long stopCount;

  bool leftMode = false;
  bool rightMode = false;

  float proportionalErrorL;
  float proportionalErrorR;
  unsigned long lastTaken = 0;
  unsigned long prevMillis = 0;
  unsigned long startTime;
  unsigned long checkTime;

  int rawBlack[3] = {315,312,100};
  int rawWhite[3] = {23,23,7};
  int rawRed[3]   = {0,0,0};
  int rawBlue[3]  = {0,0,0};
  int rawGreen[3] = {0,0,0};

  float rgbBlack[3] = {0,0,0};
  float rgbWhite[3] = {0,0,0};
  float rgbRed[3] = {0,0,0};
  float rgbBlue[3] = {0,0,0};
  float rgbGreen[3] = {0,0,0};
  double hslRed[3] = {0,0,0};
  double hslBlue[3] = {0,0,0};
  double hslGreen[3] = {0,0,0};
  
}

void rgbToHsl(float r, float g, float b, double hsl[]) { 
    double rd = (double) r/255;
    double gd = (double) g/255;
    double bd = (double) b/255;
    double max = threeway_max(rd, gd, bd);
    double min = threeway_min(rd, gd, bd);
    double h, s, l = (max + min) / 2;

    if (max == min) {
        h = s = 0; // achromatic
    } else {
        double d = max - min;
        s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
        if (max == rd) {
            h = (gd - bd) / d + (gd < bd ? 6 : 0);
        } else if (max == gd) {
            h = (bd - rd) / d + 2;
        } else if (max == bd) {
            h = (rd - gd) / d + 4;
        }
        h /= 6;
    }
    hsl[0] = h;
    hsl[1] = s;
    hsl[2] = l;
}

double threeway_max(double a, double b, double c) {
    return max(a, max(b, c));
}

double threeway_min(double a, double b, double c) {
    return min(a, min(b, c));
}

void getRawValues(int values[3],int samples){

  int s1v = HIGH, s2v=HIGH;
  for(int i=0;i<3;i++){
    values[i]=0;
    switch(i){
      
      case 0:{//red
        digitalWrite(S2,LOW);
        digitalWrite(S3,LOW);
        break;
      }
      case 1:{//green
        digitalWrite(S2,HIGH);
        digitalWrite(S3,HIGH);
        break;
      }
      case 2:{//blue
        digitalWrite(S2,HIGH);
        digitalWrite(S3,LOW);
        break;
      }
    }

    for(int j=0;j<samples;j++)
      values[i]+=pulseIn(sensorOut,HIGH);
    values[i]/=samples;
    
    
  }
  
  
  
}


void initColorSensor(Adafruit_RGBLCDShield &_lcd){

  lcd = _lcd;
//  Serial.println(testVariable);

 //set pins S0-S3 to output and sensorOut to input
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  //Set frequency scaling to 100%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
}

//I will assume rgbs in the range 0-1
void raw2rgb(int raw[3],float rgb[3]){

  for(int i=0;i<3;i++)
    rgb[i] = ((float)(raw[i]-rawBlack[i]))/(rawWhite[i]-rawBlack[i]);
  
}

namespace{

  void waitMessage(const char* messageTop){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(messageTop);
    lcd.setCursor(0,1);
    

    delay(100);
    while(lcd.readButtons()!= BUTTON_SELECT);
    
    
  }
  
}

bool calibrateColorSensor(){

  
  waitMessage("K");
  getRawValues(rawBlack,10);
  waitMessage("W");
  getRawValues(rawWhite,10);
  waitMessage("R");
  getRawValues(rawRed,10);
  waitMessage("B");
  getRawValues(rawBlue,10);
  waitMessage("G");
  getRawValues(rawGreen,10);


  raw2rgb(rawBlack,rgbBlack);
  raw2rgb(rawWhite,rgbWhite);
  raw2rgb(rawRed  ,rgbRed);
  raw2rgb(rawBlue ,rgbBlue);
  raw2rgb(rawGreen ,rgbGreen);

  rgbToHsl(rgbRed[0], rgbRed[1], rgbRed[2], hslRed);
  rgbToHsl(rgbBlue[0], rgbBlue[1], rgbBlue[2], hslBlue);
  rgbToHsl(rgbGreen[0], rgbGreen[1], rgbGreen[2], hslGreen);
  
  return true;
}

//assume rgb values in the range 0-1
void toHSI(float rgb[3],float hsi[3]){
  
  /*
   * i = gray level = (r+g+b)/3
   * s = saturation = 1-3*min(r,g,b)/(r+g+b)  // make sure not to divide by 0
   * theta = arccos ( 0.5 *((r-g)+(r-b))/sqrt((r-g)*(r-g)+(r-b)*(g-b)) //make sure not to divide by 0
   * if b > g then : h = 2*pi - theta
   * else:  h = theta
   */

  
  
}

namespace{
  unsigned long lastCheck = 0;

  float distanceL1(float c1[3],float c2[3]){
    return abs(c1[0]-c2[0]) + abs(c1[1]-c2[1]) + abs(c1[2]-c2[2]);
  }

}
/*
bool testColorRecognition(){
  uint8_t buttons = lcd.readButtons();

  if (  buttons & BUTTON_SELECT ) return true; //exit program

  if(millis()-lastCheck > 100){
    lastCheck = millis();

    raw[3];
    getRawValues(raw,10);
    
    rgb[3];
    raw2rgb(raw,rgb);

    dRed = distanceL1(rgbRed,rgb);
    dBlue = distanceL1(rgbBlue,rgb);

    lcd.setCursor(0,0);
    lcd.print("Detected...");
    lcd.setCursor(0,1);
    
    if(dRed < dBlue){
      if(dRed < epsilon) lcd.print("  RED           ");
      else lcd.print("    Neither         ");
        
    }else if(dBlue < epsilon) lcd.print("  BLUE          ");
    else lcd.print("    Neither         ");  
    
    
    
  }

  return false;
  
}
*/

void adjustWalls()
{
  //cell 0
  if(walls[0][2] == 'o' || walls[1][0] == 'o')
  {
    walls[0][2] = 'o';
    walls[1][0] = 'o';
  }
  
  if(walls[0][3] == 'o' || walls[4][1] == 'o')
  {
    walls[0][3] = 'o';
    walls[4][1] = 'o';
  }
   
  //cell 2
  if(walls[1][2] == 'o' || walls[2][0] == 'o')
  {
    walls[1][2] = 'o';
    walls[2][0] = 'o';
  }
  
  if(walls[1][3] == 'o' || walls[5][1] == 'o')
  {
    walls[1][3] = 'o';
    walls[5][1] = 'o';
  }
  
  //cell 3
   if(walls[2][2] == 'o' || walls[3][0] == 'o')
  {
    walls[2][2] = 'o';
    walls[3][0] = 'o';
  }
  
  if(walls[2][3] == 'o' || walls[6][1] == 'o')
  {
    walls[2][3] = 'o';
    walls[6][1] = 'o';
  }
 
  //cell 4
  if(walls[3][3] == 'o' || walls[7][1] == 'o')
  {
    walls[3][3] = 'o';
    walls[7][1] = 'o';
  }

   //cell 5
   if(walls[4][2] == 'o' || walls[5][0] == 'o')
  {
    walls[4][2] = 'o';
    walls[5][0] = 'o';
  }
  if(walls[4][3] == 'o' || walls[8][1] == 'o')
  {
    walls[4][3] = 'o';
    walls[8][1] = 'o';
  }

   //cell 6
   if(walls[5][2] == 'o' || walls[6][0] == 'o')
  {
    walls[5][2] = 'o';
    walls[6][0] = 'o';
  }
  if(walls[5][3] == 'o' || walls[9][1] == 'o')
  {
    walls[5][3] = 'o';
    walls[9][1] = 'o';
  }

  //cell 7
   if(walls[6][2] == 'o' || walls[7][0] == 'o')
  {
    walls[6][2] = 'o';
    walls[7][0] = 'o';
  }
  if(walls[6][3] == 'o' || walls[10][1] == 'o')
  {
    walls[6][3] = 'o';
    walls[10][1] = 'o';
  }

   //cell 8
  if(walls[7][3] == 'o' || walls[11][1] == 'o')
  {
    walls[7][3] = 'o';
    walls[11][1] = 'o';
  }

     //cell 9
   if(walls[8][2] == 'o' || walls[9][0] == 'o')
  {
    walls[8][2] = 'o';
    walls[9][0] = 'o';
  }
  if(walls[8][3] == 'o' || walls[12][1] == 'o')
  {
    walls[8][3] = 'o';
    walls[12][1] = 'o';
  }

     //cell 10
   if(walls[9][2] == 'o' || walls[10][0] == 'o')
  {
    walls[9][2] = 'o';
    walls[10][0] = 'o';
  }
  if(walls[9][3] == 'o' || walls[13][1] == 'o')
  {
    walls[9][3] = 'o';
    walls[13][1] = 'o';
  }

   //cell 11
   if(walls[10][2] == 'o' || walls[11][0] == 'o')
  {
    walls[10][2] = 'o';
    walls[11][0] = 'o';
  }
  if(walls[10][3] == 'o' || walls[14][1] == 'o')
  {
    walls[10][3] = 'o';
    walls[14][1] = 'o';
  }

  //cell 12
  if(walls[11][3] == 'o' || walls[15][1] == 'o')
  {
    walls[11][3] = 'o';
    walls[15][1] = 'o';
  }

  //cell 13
  if(walls[12][2] == 'o' || walls[13][0] == 'o')
  {
    walls[12][2] = 'o';
    walls[13][0] = 'o';
  }

  //cell 14
  if(walls[13][2] == 'o' || walls[14][0] == 'o')
  {
    walls[13][2] = 'o';
    walls[14][0] = 'o';
  }

  //cell 15
  if(walls[14][2] == 'o' || walls[15][0] == 'o')
  {
    walls[14][2] = 'o';
    walls[15][0] = 'o';
  }
}





void printState()
{
  lcd.clear();
 if (ori == 255)
  ori = 3;
 else if (ori == 4)
  ori = 0; 
 lcd.setCursor(0, 0);
    while (n < 16)
    {
      lcd.print(state[n]);
      n++;
    }
    n = 0;
    lcd.setCursor(0, 1);
    lcd.print('G');
    lcd.print(gr + 1);
    lcd.print(" O");
    if (ori == 0)
      lcd.print('N');
    else if (ori == 1)
      lcd.print('E');
    else if (ori == 2)
      lcd.print('S');
    else if (ori == 3){
      lcd.print('W'); 
    }
     /*lcd.setCursor(8,1);
     for(int q = 0; q < 4; q++)
     {
     lcd.print(walls[gr][q]);
     }*/
      /*
    n = 6;
    while (n < 15)
    {
      lcd.setCursor(n, 1);
      lcd.print(nav[i]);
      n += 2;
      i++;
    }
    n = 0;
    i = 0;
    */
}



void pathPlanning(byte s, byte e, byte i)
{
  que.enqueue(s - 1);
  
  current = s - 1;
  while(!(que.isEmpty()))
  {
    
    if (current == (e - 1))
    {
      //exit logic
      break;
    }
    
    state[current] = 'X';

    if (walls[current][0] == 'o')
    {
      if(state[current - 1] == '0')
      {
        pred[current - 1] = current;
        que.enqueue(current - 1);
      }
    }
    if (walls[current][1] == 'o')
    {
      if(state[current - 4] == '0')
      {
        pred[current - 4] = current;
        que.enqueue(current - 4);
      }
    }
   if (walls[current][2] == 'o')
    {
      if(state[current + 1] == '0')
      {
        pred[current + 1] = current;
        que.enqueue(current + 1);
      }
    }
   if (walls[current][3] == 'o')
    {
      if(state[current + 4] == '0')
      {
        pred[current + 4] = current;
        que.enqueue(current + 4);
      }
    }
    current = que.dequeue();
  }
  pathNavigation(s,e,i);
/*
 for(int v = 0; v < 16; v++)
  {  
    if(v < 8)
    {
    lcd.print(pred[v]);
    }
    else
    {
      if(v == 8)
      {
      lcd.setCursor(0,1);
      }
      lcd.print(pred[v]);
    }
  }
    delay(100000);*/
  
  //lcd.clear();
}

//----------------------------------------------------------------------------------------------------BEGIN PATH NAVIGATION--------------------------------------------------------------

bool pathNavigation(byte s, byte e, byte i)
{
  lcd.clear();

  gr = s - 1;
  endl = e - 1;
  ori = i;

  byte n = 1;

  state[gr] = 'X';
  nav[0] = gr;

  printState();
      
    startTime = millis();

  while (millis() - startTime < 200)
  {
    takeNewMeasurement(0);
    takeNewMeasurement(1);
    takeNewMeasurement(2);

    left = getCombinedDistance(0);
    right = getCombinedDistance(2);
    front = getCombinedDistance(1);
    delay(10);
  }
//    checked = 0;
    
/*
    if (timerInit == false)
    {
       Timer1.initialize(250000);
       Timer1.attachInterrupt(recognize);
       timerInit = true;
    }
*/
    
  while (true)
  {
  uint8_t buttons = lcd.readButtons();
  
    getRawValues(raw,10);
    
    raw2rgb(raw,rgb);

    rgbToHsl(rgb[0], rgb[1], rgb[2], hsl);

    dRed = abs(hsl[0] - hslRed[0]);
    dBlue = abs(hsl[0] - hslBlue[0]);
    dGreen = abs(hsl[0] - hslGreen[0]);


  if (millis() - lastTaken >= 10)
  {
    //checked++;
    lastTaken = millis();
    takeNewMeasurement(0);
    takeNewMeasurement(1);
    takeNewMeasurement(2);

    left = getCombinedDistance(0);
    right = getCombinedDistance(2);
    front = getCombinedDistance(1);
 
      if (cellEntered == true)//------------------------------------------------------------------BEGIN CELLENTERED == TRUE----------------------------------------------------
      {
       //Update current location, visited array, nav stack
      if (ori == 0)
      {
        //last = gr;
        gr = gr - 4;
        state[gr] = 'X';
        nav[n] = gr;
        //walls[last][1] == 'o';
        printState();
      }
      else if (ori == 1)
      {
        //last = gr;
        gr = gr + 1;
        state[gr] = 'X';
        //walls[last][2] == 'o';
        printState();
      }
      else if (ori == 2)
      {
        //last = gr;
        gr = gr + 4;
        state[gr] = 'X';
        //walls[last][3] == 'o';
        printState();
      }
      else if (ori == 3)
      {
        //last = gr;
        gr = gr - 1;
        state[gr] = 'X';
        //walls[last][0] == 'o';
        printState();
      }
      cellEntered = false;
      timerBegin = true;
      stopCount = millis();
    } //End if (cellEntered == true)-----------------------------------------------------------------------------------------------------------------------------

    if (leftMode == false && rightMode == false)
    {
      //Stop and take measurements
      setSpeeds(0, 0);
      
      //Update walls array
      if (front > 2 && front < 8)
      {
        if (ori == 0)
        {
          walls[gr][1] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][3] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][0] = 'W';
        }
      }
      if (left > 2 && left < 12)
      {
        if (ori == 0)
        {
          walls[gr][0] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][1] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][3] = 'W';
        }
      }
      if (right > 2 && right < 12)
      {
        if (ori == 0)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][3] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][0] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][1] = 'W';
        }
      }
     
      
      
      checkTime = millis();
      while (millis() - checkTime < 500)
      {
        takeNewMeasurement(0);
        takeNewMeasurement(1);
        takeNewMeasurement(2);
    
        left = getCombinedDistance(0);
        right = getCombinedDistance(2);
        front = getCombinedDistance(1);
        delay(10);
      }
      
      //Make decision
      //Two conditions for backing out: that there are walls on left, right, and front; or that cells on left, right and front are visited
      //Redo decision making after every 90-degree turn
      //pathChosen = false;
      startTime = millis();

      c = endl;
      while (pred[c] != gr || c == 20)
      {
        c = pred[c];
        
        if(state[endl] == 'X')
        {
          lcd.clear();
          /*for (byte u = 0; u < 16; u++)
          {
           state[u] = '0';
           pred[u] = 20;
          }*/
          return;
        }
      }

      
      /*if (c == 20)
      {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("DONE");
        while (lcd.readButtons() != 1)
          ;
        lcd.clear();
        delay(300);
        return;
      }*/
      
    if (c - gr == 4)
    {
      if (ori == 0)
      {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
           printState();
      }
      else if (ori == 1)
      {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
      }
      else if (ori == 2)
      {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            //pathChosen = true;
      }
      else if (ori == 3)
      {
        {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      }
    }
    else if (c - gr == 1) //Need to go East
    {
      if (ori == 0)
      {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
      }
      else if (ori == 1)
      {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            //pathChosen = true;
      }
      else if (ori == 2)
      {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      else if (ori == 3)
      {
        {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      }
    }
    else if (c - gr == -4)//Need to go North
    {
      if (ori == 0)
      {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            //pathChosen = true;
      }
      else if (ori == 1)
      {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      else if (ori == 2)
      {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      else if (ori == 3)
      {
        {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
      }
      }
    }
    else if (c - gr == -1) //Need to go West
    {
      if (ori == 0)
      {
            setSpeeds(-30, 30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
      }
      else if (ori == 1)
      {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
      }
      else if (ori == 2)
      {
            setSpeeds(30, -30);
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
      }
      else if (ori == 3)
      {
        {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            //pathChosen = true;
      }
      }
    }
     

    startTime = millis();
  }//End if (leftMode == false && rightMode == false)----------------------------------------------------------------------------------------------------------------------
  
    else if (leftMode == true)
  {
    if (timerBegin == true)
      if (millis() - stopCount > 2000)
      {
        wallTime = 0;
        leftMode = false;
        rightMode = false;
        timerBegin = false;
        startTime = millis();
      }
   if (front > 1 && front < 7)
  {
    wallTime = 0;
    leftMode = false;
    rightMode = false;
    timerBegin = false;
    startTime = millis();
  }
    else if (left > 1)
  {
    wallTime++;
    if (wallTime > 30)
    {
    proportionalErrorL = (left - 7) * .2;
    //Serial.print(proportionalErrorL);
    if (proportionalErrorL > 0)
      setSpeedsvwCCW((.8 * M_PI * W_DIAMETER - saturation(proportionalErrorL) * D_AXIS), saturation(proportionalErrorL));
    else
      setSpeedsvwCCW((.8 * M_PI * W_DIAMETER + saturation(proportionalErrorL) * D_AXIS), saturation(proportionalErrorL));
    }
    else
      setSpeeds(200, 200);
  }
    else
    {
      wallTime = 0;
      setSpeeds(200, 200);
    }
  } //End if (leftMode)

    //rightMode-------------------------------------------------------------BEGIN RIGHTMODE---------------------------------------------------------------
    else if (rightMode == true)
    {
    if (timerBegin == true)
      if (millis() - stopCount > 2000)
      {
        wallTime = 0;
        rightMode = false;
        leftMode = false;
        timerBegin = false;
        startTime = millis();
      }
   if (front > 1 && front < 7)
  {
    wallTime = 0;
    rightMode = false;
    leftMode = false;
    timerBegin = false;
    startTime = millis();
  }
    else if (right > 1)
  {
    wallTime++;
    if (wallTime > 30)
    {
    proportionalErrorR = (right - 7) * .2;
    //Serial.print(proportionalErrorL);
    if (proportionalErrorR > 0)
      setSpeedsvw((.8 * M_PI * W_DIAMETER - saturation(proportionalErrorR) * D_AXIS), saturation(proportionalErrorR));
    else
      setSpeedsvw((.8 * M_PI * W_DIAMETER + saturation(proportionalErrorR) * D_AXIS), saturation(proportionalErrorR));
    }
    else
      setSpeeds(200, 200);
  }
    else
    {
      wallTime = 0;
      setSpeeds(200, 200);
    }
  } //End if (rightMode)
    
  }//End if (millis() - lastTaken > 10)--------------------------------------------------------------------------------------------------------------------------------------
  
  if (millis() - lastCheck > 10)
  {
     lastCheck = millis();
    if (millis() - startTime > 500)
    {
    if (dBlue < dRed && dBlue < dGreen) 
      {  
        greenFound = 0;
        lcd.setBacklight(BLUE);
        if (lastBlue == false)
        {
          lastBlue = true;
        if (compassMade == false)
        {
          if (ori == 0 || ori == 2)
            compass = 0; //Crossing blue lines indicates heading to a cell to the North or South. Red lines East or West.
          else if (ori == 1 || ori == 3)
            compass = 1; //Crossing blue lines indicates heading to a cell to the East or West. Red lines North or South.
          compassMade = true;
        }
        if (compass == 0) //Blue lines: North or South
        {
          if (ori == 0 || ori == 2) //Heading North or heading South
          {
            cellEntered = true;
          }
        }
        else //Blue lines: East or West
        {
          if (ori == 1 || ori == 3) //Heading East or heading West
          {
            cellEntered = true;
          }
        }
        }
      }
  
  else if(dRed < dBlue && dRed < dGreen)
      {
        greenFound = 0;
        lcd.setBacklight(RED);
        if (lastRed == false)
        {
          lastRed = true;
        if (compassMade == false)
        {
          if (ori == 0 || ori == 2)
            compass = 1; //Crossing blue lines indicates heading to a cell to the East or West. Red lines North or South.
          else if (ori == 1 || ori == 3)
            compass = 0; //Crossing blue lines indicates heading to a cell to the North or South. Red lines East or West.
          compassMade = true;
        }
        if (compass == 0) //Red lines: East or West
        {
          if (ori == 1 || ori == 3) //Heading East or heading West
          {
            cellEntered = true;
          }
        }
        else //Red lines: North or South
        {
          if (ori == 0 || ori == 2) //Heading North or heading South
          {
            cellEntered = true;
          }
        }
        }
      }

      else
      {
      lcd.setBacklight(WHITE);
      greenFound++;
      if (greenFound > 50)
      {
      lastRed = false;
      lastBlue = false;
      }
      }
    }
  }
  }

}//---------------------------------------------------------------------------------END PATH NAVIGATION--------------------------------------------------------------------------

bool lineDetection(int s, int i){
  
  lcd.clear();

  gr = s - 1;
  ori = i;

  byte n = 1;

  state[gr] = 'X';
  nav[0] = gr;

  printState();
      
    startTime = millis();

  while (millis() - startTime < 200)
  {
    takeNewMeasurement(0);
    takeNewMeasurement(1);
    takeNewMeasurement(2);

    left = getCombinedDistance(0);
    right = getCombinedDistance(2);
    front = getCombinedDistance(1);
    delay(10);
  }
//    checked = 0;
    
/*
    if (timerInit == false)
    {
       Timer1.initialize(250000);
       Timer1.attachInterrupt(recognize);
       timerInit = true;
    }
*/
    
  while (true)
  {
  uint8_t buttons = lcd.readButtons();

  if (  buttons & BUTTON_SELECT) 
  {
    adjustWalls();
    setSpeeds(0, 0);
        while (true)
             {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("DONE");
        delay(300);
        while (lcd.readButtons() != 1)
          ;
        lcd.setCursor(0, 0);
        for (n = 0; n < 4; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        lcd.setCursor(0, 1);
        for (n = 4; n < 8; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        while (lcd.readButtons() != 1)
          ;

        //Begin printing second set
        lcd.clear();
        lcd.setCursor(0, 0);
        for (n = 8; n < 12; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        lcd.setCursor(0, 1);
        for (n = 12; n < 16; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        //delay(300);
        while (lcd.readButtons() != 1)
          ;
        lcd.clear();
        delay(300);
        for (byte u = 0; u < 16; u++)
        {
          state[u] = '0';
        }
        return true;
        //delay(300);
        //while (lcd.readButtons() != 1)
         // ;
        }
  }

    getRawValues(raw,10);
    
    raw2rgb(raw,rgb);

    rgbToHsl(rgb[0], rgb[1], rgb[2], hsl);

    dRed = abs(hsl[0] - hslRed[0]);
    dBlue = abs(hsl[0] - hslBlue[0]);
    dGreen = abs(hsl[0] - hslGreen[0]);


  if (millis() - lastTaken >= 10)
  {
    //checked++;
    lastTaken = millis();
    takeNewMeasurement(0);
    takeNewMeasurement(1);
    takeNewMeasurement(2);

    left = getCombinedDistance(0);
    right = getCombinedDistance(2);
    front = getCombinedDistance(1);
 
      if (cellEntered == true)//------------------------------------------------------------------BEGIN CELLENTERED == TRUE----------------------------------------------------
      {
       //Update current location, visited array, nav stack
      if (ori == 0)
      {
        last = gr;
        gr = gr - 4;
        if (backing == false)
        {
        state[gr] = 'X';
        nav[n] = gr;
        n++;
        }
        walls[last][1] = 'o';
        printState();
      }
      else if (ori == 1)
      {
        last = gr;
        gr = gr + 1;
        if (backing == false)
        {
        state[gr] = 'X';
        nav[n] = gr;
        n++;
        }
        walls[last][2] = 'o';
        printState();
      }
      else if (ori == 2)
      {
        last = gr;
        gr = gr + 4;
        if (backing == false)
        {
        state[gr] = 'X';
        nav[n] = gr;
        n++;
        }
        walls[last][3] = 'o';
        printState();
      }
      else if (ori == 3)
      {
        last = gr;
        gr = gr - 1;
        if (backing == false)
        {
        state[gr] = 'X';
        nav[n] = gr;
        n++;
        }
        walls[last][0] = 'o';
        printState();
      }
      /*
      //Entered cell; move straight for 1 second
      setSpeeds(80, 80);
      checkTime = millis();
      while (millis() - checkTime < 1800)
      {
       if (millis() - checkTime > 500)
         lcd.setBacklight(WHITE);
      }
      */
      cellEntered = false;
      timerBegin = true;
      stopCount = millis();
    } //End if (cellEntered == true)-----------------------------------------------------------------------------------------------------------------------------
    
    if (leftMode == false && rightMode == false)
    {
      //Stop and take measurements
      setSpeeds(0, 0);
      if (backing == false)
      {
      //Update walls array
      if (front > 2 && front < 8)
      {
        if (ori == 0)
        {
          walls[gr][1] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][3] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][0] = 'W';
        }
      }
      if (left > 2 && left < 12)
      {
        if (ori == 0)
        {
          walls[gr][0] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][1] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][3] = 'W';
        }
      }
      if (right > 2 && right < 12)
      {
        if (ori == 0)
        {
          walls[gr][2] = 'W';
        }
        else if (ori == 1)
        {
          walls[gr][3] = 'W';
        }
        else if (ori == 2)
        {
          walls[gr][0] = 'W';
        }
        else if (ori == 3)
        {
          walls[gr][1] = 'W';
        }
      }
      
      }
      
      //Stopping condition
      if (state[0] == 'X' && state[1] == 'X' && state[2] == 'X' && state[3] == 'X' && state[4] == 'X' && state[5] == 'X' && state[6] == 'X' && state[7] == 'X' && state[8] == 'X' && state[9] == 'X' && state[10] == 'X' && state[11] == 'X' && state[12] == 'X' && state[13] == 'X' && state[14] == 'X' && state[15] == 'X')
      {
        setSpeeds(0, 0);
        adjustWalls();
        while (true)
        {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("DONE");
        while (lcd.readButtons() != 1)
          ;
        lcd.setCursor(0, 0);
        for (n = 0; n < 4; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        lcd.setCursor(0, 1);
        for (n = 4; n < 8; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        while (lcd.readButtons() != 1)
          ;

        //Begin printing second set
        lcd.clear();
        lcd.setCursor(0, 0);
        for (n = 8; n < 12; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        lcd.setCursor(0, 1);
        for (n = 12; n < 16; n++)
        {
          for (i = 0; i < 4; i++)
          {
            lcd.print(walls[n][i]);
          }
        }
        //delay(300);
        while (lcd.readButtons() != 1)
          ;
        lcd.clear();
        delay(300);
        for (byte u = 0; u < 16; u++)
        {
          state[u] = '0';
        }
        return true;
      }
      }
      checkTime = millis();
      while (millis() - checkTime < 500)
      {
        takeNewMeasurement(0);
        takeNewMeasurement(1);
        takeNewMeasurement(2);
    
        left = getCombinedDistance(0);
        right = getCombinedDistance(2);
        front = getCombinedDistance(1);
        delay(10);
      }
      
      //Make decision
      //Two conditions for backing out: that there are walls on left, right, and front; or that cells on left, right and front are visited
      //Redo decision making after every 90-degree turn
      pathChosen = false;
      startTime = millis();
      if (front < 1 || front > 10)
      {
        if (ori == 0)
        {
          if (state[gr - 4] == '0')
          {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            pathChosen = true;
          }
        }
        else if (ori == 1)
        {
          if (state[gr + 1] == '0')
          {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            pathChosen = true;
          }
        }
        else if (ori == 2)
        {
          if (state[gr + 4] == '0')
          {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            pathChosen = true;
          }
        }
        else if (ori == 3)
        {
          if (state[gr - 1] == '0')
          {
            if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
              leftMode = true;
            else
              rightMode = true;
            pathChosen = true;
          }
        }
      }
      
      if ((left < 1 || left > 10) && pathChosen == false)
      {
        if (ori == 0)
        {
          if (state[gr - 1] == '0')
          {
            setSpeeds(-30, 30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 1)
        {
          if (state[gr - 4] == '0')
          {
            setSpeeds(-30, 30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 2)
        {
          if (state[gr + 1] == '0')
          {
            setSpeeds(-30, 30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 3)
        {
          if (state[gr + 4] == '0')
          {
            setSpeeds(-30, 30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori--;
            printState();
            pathChosen = true;
          }
        }
      }
      
      if ((right < 1 || right > 10) && pathChosen == false)
      {
        if (ori == 0)
        {
          if (state[gr + 1] == '0')
          {
            setSpeeds(30, -30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 1)
        {
          if (state[gr + 4] == '0')
          {
            setSpeeds(30, -30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 2)
        {
          if (state[gr - 1] == '0')
          {
            setSpeeds(30, -30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
            pathChosen = true;
          }
        }
        else if (ori == 3)
        {
          if (state[gr - 4] == '0')
          {
            setSpeeds(30, -30);;
            checkTime = millis();
            while (millis() - checkTime < 1800)
              ;
            setSpeeds(0, 0);
            ori++;
            printState();
            pathChosen = true;
          }
        }
      }

     if (pathChosen == true)
     {
      backing = false;
     }
    
      //No path chosen. Back out by referring to nav stack.---------------------------------------BEGIN IF PATHCHOSEN == FALSE----------------------------------------------------------
      if (pathChosen == false)
    {
      //Pop the nav stack
      n--;
      nav[n] = 0;
      if (n - 1 == -1)
      {
        setSpeeds(0, 0);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No exit");
        while (lcd.readButtons() != 1)
          ;
      }
      //Return to previous cell
      backing = true;
      if (gr - nav[n - 1] == -4) //Went North, back out by going South
      {
        if (ori == 0)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 3600)
            ;
          setSpeeds(0, 0);
          ori = 2;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 1)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 2;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 2)
        {
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 3)
        {
          setSpeeds(-30, 30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 2;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
      }
      else if (gr - nav[n - 1] == 1) //Went East, back out by going West
      {
        if (ori == 0)
        {
          setSpeeds(-30, 30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 3;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 1)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 3600)
            ;
          setSpeeds(0, 0);
          ori = 3;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 2)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 3;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 3)
        {
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
      }
      else if (gr - nav[n - 1] == 4) //Went South, back out by going North
      {
        if (ori == 0)
        {
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 1)
        {
          setSpeeds(-30, 30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 0;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 2)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 3600)
            ;
          setSpeeds(0, 0);
          ori = 0;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 3)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 0;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
      }
      else if (gr - nav[n - 1] == -1) //Went West, back out by going East
      {
        if (ori == 0)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 1;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 1)
        {
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 2)
        {
          setSpeeds(-30, 30);;
          checkTime = millis();
          while (millis() - checkTime < 1800)
            ;
          setSpeeds(0, 0);
          ori = 1;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
        else if (ori == 3)
        {
          setSpeeds(30, -30);;
          checkTime = millis();
          while (millis() - checkTime < 3600)
            ;
          setSpeeds(0, 0);
          ori = 1;
          printState();
          checkTime = millis();
          while (millis() - checkTime < 500)
          {
            takeNewMeasurement(0);
            takeNewMeasurement(1);
            takeNewMeasurement(2);
        
            left = getCombinedDistance(0);
            right = getCombinedDistance(2);
            front = getCombinedDistance(1);
            delay(10);
          }
          if ((left > 1 && right > 1 && left < right) || (left > 1 && right < 1))
            leftMode = true;
          else
            rightMode = true;
        }
      }
    }

    startTime = millis();
  }//End if (leftMode == false && rightMode == false)----------------------------------------------------------------------------------------------------------------------
  
    else if (leftMode == true)
  {
    if (timerBegin == true)
      if (millis() - stopCount > 2000)
      {
        wallTime = 0;
        leftMode = false;
        rightMode = false;
        timerBegin = false;
        startTime = millis();
      }
   if (front > 1 && front < 7)
  {
    wallTime = 0;
    leftMode = false;
    rightMode = false;
    timerBegin = false;
    startTime = millis();
  }
    else if (left > 1)
  {
    wallTime++;
    if (wallTime > 30)
    {
    proportionalErrorL = (left - 7) * .2;
    //Serial.print(proportionalErrorL);
    if (proportionalErrorL > 0)
      setSpeedsvwCCW((.8 * M_PI * W_DIAMETER - saturation(proportionalErrorL) * D_AXIS), saturation(proportionalErrorL));
    else
      setSpeedsvwCCW((.8 * M_PI * W_DIAMETER + saturation(proportionalErrorL) * D_AXIS), saturation(proportionalErrorL));
    }
    else
      setSpeeds(200, 200);
  }
    else
    {
      wallTime = 0;
      setSpeeds(200, 200);
    }
  } //End if (leftMode)

    //rightMode-------------------------------------------------------------BEGIN RIGHTMODE---------------------------------------------------------------
    else if (rightMode == true)
    {
    if (timerBegin == true)
      if (millis() - stopCount > 2000)
      {
        wallTime = 0;
        rightMode = false;
        leftMode = false;
        timerBegin = false;
        startTime = millis();
      }
   if (front > 1 && front < 7)
  {
    wallTime = 0;
    rightMode = false;
    leftMode = false;
    timerBegin = false;
    startTime = millis();
  }
    else if (right > 1)
  {
    wallTime++;
    if (wallTime > 30)
    {
    proportionalErrorR = (right - 7) * .2;
    //Serial.print(proportionalErrorL);
    if (proportionalErrorR > 0)
      setSpeedsvw((.8 * M_PI * W_DIAMETER - saturation(proportionalErrorR) * D_AXIS), saturation(proportionalErrorR));
    else
      setSpeedsvw((.8 * M_PI * W_DIAMETER + saturation(proportionalErrorR) * D_AXIS), saturation(proportionalErrorR));
    }
    else
      setSpeeds(200, 200);
  }
    else
    {
      wallTime = 0;
      setSpeeds(200, 200);
    }
  } //End if (rightMode)
    
  }//End if (millis() - lastTaken > 10)--------------------------------------------------------------------------------------------------------------------------------------
  
  if (millis() - lastCheck > 10)
  {
     lastCheck = millis();
    if (millis() - startTime > 500)
    {
    if (dBlue < dRed && dBlue < dGreen) 
      {  
        greenFound = 0;
        lcd.setBacklight(BLUE);
        if (lastBlue == false)
        {
          lastBlue = true;
        if (compassMade == false)
        {
          if (ori == 0 || ori == 2)
            compass = 0; //Crossing blue lines indicates heading to a cell to the North or South. Red lines East or West.
          else if (ori == 1 || ori == 3)
            compass = 1; //Crossing blue lines indicates heading to a cell to the East or West. Red lines North or South.
          compassMade = true;
        }
        if (compass == 0) //Blue lines: North or South
        {
          if (ori == 0 || ori == 2) //Heading North or heading South
          {
            cellEntered = true;
          }
        }
        else //Blue lines: East or West
        {
          if (ori == 1 || ori == 3) //Heading East or heading West
          {
            cellEntered = true;
          }
        }
        }
      }
  
  else if(dRed < dBlue && dRed < dGreen)
      {
        greenFound = 0;
        lcd.setBacklight(RED);
        if (lastRed == false)
        {
          lastRed = true;
        if (compassMade == false)
        {
          if (ori == 0 || ori == 2)
            compass = 1; //Crossing blue lines indicates heading to a cell to the East or West. Red lines North or South.
          else if (ori == 1 || ori == 3)
            compass = 0; //Crossing blue lines indicates heading to a cell to the North or South. Red lines East or West.
          compassMade = true;
        }
        if (compass == 0) //Red lines: East or West
        {
          if (ori == 1 || ori == 3) //Heading East or heading West
          {
            cellEntered = true;
          }
        }
        else //Red lines: North or South
        {
          if (ori == 0 || ori == 2) //Heading North or heading South
          {
            cellEntered = true;
          }
        }
        }
      }

      else
      {
      lcd.setBacklight(WHITE);
      greenFound++;
      if (greenFound > 50)
      {
      lastRed = false;
      lastBlue = false;
      }
      }
    }
  /*
  lcd.setCursor(0, 0);
  lcd.print(left);
  lcd.print(" ");
  lcd.print(right);
  lcd.setCursor(0, 1);
  lcd.print("front: ");
  lcd.print(front);
  */
  }
/*
  if (front > 2 && front < 5 && checked > 10)
  {
    setSpeeds(0, 0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("red: ");
    lcd.print(red);
    lcd.setCursor(0, 1);
    lcd.print("blue: ");
    lcd.print(blue);
    while (lcd.readButtons() != BUTTON_SELECT)
      ;
    return true;
  }
*/
  }
  
  return true;
}

