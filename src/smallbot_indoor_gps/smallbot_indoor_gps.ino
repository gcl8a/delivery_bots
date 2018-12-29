/*
 * Main program for free-range bot using my "indoor GPS" system
 */

#include "robot_gps.h"

UGV_RFM robot;

void setup()
{
  DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL) {delay(500);} //uncommenting this REQUIRES the Serial Monitor to be opened
  DEBUG_SERIAL.println("setup");

  robot.Init();
 
  DEBUG_SERIAL.println("/setup");
}

void loop(void)
{
  //DEBUG_SERIAL.println("loop");
  robot.MainLoop();

  if(CheckDebugSerial())
  {
    if(debugString[0] == 'S') robot.Idle();

    else
    {
      //all in m/s, rad/s
      float x = debugString.toFloat();
      uint8_t comma = debugString.indexOf(',');
      float y = debugString.substring(comma+1).toFloat();
  
      DEBUG_SERIAL.print("Setting x = ");
      DEBUG_SERIAL.println(x);
      DEBUG_SERIAL.print("Setting y = ");
      DEBUG_SERIAL.println(y);
  
      robot.SetDestination(x, y);
    }
    
    debugString = "";
  }
}

