#include "robot.h"

class UGV_RFM : public UGV
{
public:
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("PixyUGV::Init");

    UGV::Init();

    radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
    radio.setHighPower();

    DEBUG_SERIAL.println("/PixyUGV::Init");
  }

  virtual void MainLoop(void)
  {
    UGV::MainLoop();

    if(CheckRadio())
    {
      DEBUG_SERIAL.println(recString);

      if(recString[0] == 'D') //destination command
      {
        float x = recString.substring(1).toFloat();
        int comma = recString.indexOf(',');
        float y = recString.substring(comma + 1).toFloat();
        SetDestination(x, y);          
      }

      else if(recString[0] == 'S') //stop command
      {
        Idle();
      }

      else //observation
      {
        int comma = recString.indexOf(',');
        float x = recString.substring(comma + 1).toFloat();
        
        comma = recString.indexOf(',', comma + 1);
        float y = recString.substring(comma + 1).toFloat();
  
  //      DEBUG_SERIAL.print(x);
  //      DEBUG_SERIAL.print('\t');
  //      DEBUG_SERIAL.print(y);
  //      DEBUG_SERIAL.print('\n');
  
        ApplyObservation(x, y);
      }
    }

    if(readyToReport)
    {
        String message;
        message += String(millis());
        message += '\t';
        message += String(currPose.x);
        message += '\t';
        message += String(currPose.y);
        message += '\t';
        message += String(currPose.theta);
        message += '\n';
  
        //SendMessage(0, message);
        DEBUG_SERIAL.print(message);
        readyToReport = 0;
    }
  }
};

