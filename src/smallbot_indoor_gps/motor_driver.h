#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "comm.h"

class MotorDriver
{
protected:
public:
  virtual void SendPowers(int16_t powerA, int16_t powerB) = 0;

public:
  MotorDriver(void) {}

  void Init(void)
  {
    DEBUG_SERIAL.println("MotorDriver::Init");
    FullStop();
    DEBUG_SERIAL.println("/MotorDriver::Init");
  }

  virtual void EmergencyStop(void) 
  {
    FullStop();
  }

  void SetPowers(int16_t powerA, int16_t powerB)
  {
    powerA = constrain(powerA, -255, 255);
    powerB = constrain(powerB, -255, 255);
    
    SendPowers(powerA, powerB);
  }
  
  void FullStop(void)
  {
    SetPowers(0, 0);
  }
};

#endif

