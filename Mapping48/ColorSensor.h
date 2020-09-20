

#ifndef __COLOR_SENSOR__
#define __COLOR_SENSOR__

#include <Adafruit_RGBLCDShield.h>
#include <Arduino.h>
// COLOR SENSOR
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8


//int testGlobalVariable = 4;

//initialize color sensor
void initColorSensor(Adafruit_RGBLCDShield &_lcd);

//gets raw measurements and return average
void getRawValues(int values[3],int sample);

 void rgbToHsl(float r, float g, float b, double hsl[]);
 double threeway_max(double a, double b, double c);
 double threeway_min(double a, double b, double c);

//calibrate fuction
bool calibrateColorSensor();
//char printShortestPath(byte p[], byte c);
void adjustWalls();

//I will assume rgbs in the range 0-1
void raw2rgb(float rgb[3],int samples);

void rgb2hsi(float rgb[3],float hsi[3]);

//sample programs
bool testColorRecognition();

void printState();
bool pathNavigation(byte s, byte e, byte i);
void pathPlanning(byte s, byte e, byte i);

bool lineDetection(int s, int i);

#endif
