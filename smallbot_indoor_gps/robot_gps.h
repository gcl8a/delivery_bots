#include "robot.h"
#include "apriltag.h"

class UGV_RFM : public UGV
{
  protected:
    uint16_t myID = MYNODEID;
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

      else if(recString.length() == 6) //observation
      {
        AprilTag tag;
        memcpy(&tag, &recString[0], 6);
          
  //      DEBUG_SERIAL.print(x);
  //      DEBUG_SERIAL.print('\t');
  //      DEBUG_SERIAL.print(y);
  //      DEBUG_SERIAL.print('\n');
        if(tag.id == myID) ApplyObservation(tag.x, tag.y);
      }

      else {} //punt
    }

    if(readyToReport)
    {
        String message;
        message += String(millis());
        message += '\t';
        message += String(destPose.x);
        message += '\t';
        message += String(destPose.y);
        message += '\t';
        message += String(currPose.x);
        message += '\t';
        message += String(currPose.y);
        message += '\t';
        message += String(currPose.theta);
        message += '\n';
  
        SendMessage(10, message);
        readyToReport = 0;
    }
  }
};

