/*
 * For using the PixyCam as feedback
 */

#ifndef __COMM_H
#define __COMM_H

#include <RFM69.h>

#define DEBUG_SERIAL SerialUSB

String debugString;
boolean CheckDebugSerial(void) //returns true upon newline, not just a character
{
  while(DEBUG_SERIAL.available()) 
  {
    // get incoming byte:
    char inChar = DEBUG_SERIAL.read();
    debugString += inChar;
 
    if (inChar == '\n') return true;
    else return false; 
  }
  
  return false;
}

/////////////RFM69HCW radio
#define NETWORKID     155   // Must be the same for all nodes (0 to 255)
#define MYNODEID      3   // My node ID (0 to 255)
#define FREQUENCY     RF69_915MHZ

// Create a library object for our RFM69HCW module:
RFM69 radio(10, 9, false, digitalPinToInterrupt(9));

String recString;
bool CheckRadio(void)
{
  bool retVal = false;
  if(radio.receiveDone())
  {
    retVal = true;
    //uint8_t recLength = radio.DATALEN;

    recString = "";
    for (byte i = 0; i < radio.DATALEN; i++)
    {
      recString += (char)radio.DATA[i];
    }

    if (radio.ACKRequested())
    {
      radio.sendACK();
    }
  }

  return retVal;
}

void SendMessage(int dest, String message)
{
  char buffer[60];
  sprintf(buffer, "%s", message.c_str());

  radio.send(dest, buffer, strlen(buffer));
  DEBUG_SERIAL.print(buffer);
  radio.receiveDone(); //put us back into receive mode
}

#endif
