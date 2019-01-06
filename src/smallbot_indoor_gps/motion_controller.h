
/*
 * Reads sensors (encoders) to calculate current velocity estimate (we'll leave pose for higher level functions).
 * Calculates actuator (motor) inputs using PID.
 */

#ifndef __MOTION_CONTROLLER_H
#define __MOTION_CONTROLLER_H

#include <TIArray.h>
#include <matrix.h>

#define imatrix TMatrix<int16_t>
#define ivector TVector<int16_t>

#include "encoder.h"

#define LOOP_RATE 50   //Hz

volatile uint8_t readyToPID = 0;

#define INTEGRAL_CAP 12000 //note that the comparison is sum > (INTEGRAL_CAP / Ki) so that changing Ki doesn't affect the cap
#define KP_DEF 256
#define KI_DEF 1

class MotionController
{ 
protected: 
  ivector target;   //target speed, using integer math to speed up the processing
  ivector estimate; //wheel speed estimate

  TIArray<Encoder> encoders;

  uint16_t Kp = KP_DEF;
  uint16_t Ki = KI_DEF;

public:
  MotionController(void) : target(2), estimate(2), encoders(2) //hard code encoders for the moment...
  {}

  void Init(void)
  {
    DEBUG_SERIAL.println("MotionController::Init");
    SetupEncoders();

    encoders[0] = encoder1;
    encoders[1] = encoder2;

    /*
     * Set up TC2 for periodically calling the PID routine.
     * Note that TC3 (which was used on the SAMD21 version) is used for PWM on pin 4, so we'll switch
     */
    //GCLK0 is already enabled, but if we wanted to use a different generic clock, these are the steps
    //GCLK->GENCTRL[0].bit.GENEN = 1;
    //GCLK->GENCTRL[0].bit.SRC = 1;
    //while (GCLK->SYNCBUSY.bit.GENCTRL0) {};           // Wait for synchronization

    //I don't understand this one. The examples and datasheet say
    //something about routing the master (synchronized) clock to the bus,
    //but we route a generic clock to the TC2 later, so why two clocks?
    //It works without it, though, so now I really don't know, but I'll leave
    //it here for refrence if it ever is needed.
    //MCLK->APBBMASK.bit.TC2_ = 1;

    //Route a generic clock (0) to TC2
    GCLK->PCHCTRL[TC2_GCLK_ID].bit.GEN = 0;
    GCLK->PCHCTRL[TC2_GCLK_ID].bit.CHEN = 1;

    //the data sheet says you have to read back the enable bit
    //it all works without it, but it can't hurt?
    while(!GCLK->PCHCTRL[TC2_GCLK_ID].bit.CHEN) {}

    // The type cast must fit with the selected timer mode (defaults to 16-bit)
    TcCount16* TC = (TcCount16*) TC2; // get timer struct

    //reset the TC
    TC->CTRLA.bit.SWRST = 1;
    while(TC->SYNCBUSY.bit.SWRST);  // wait for sync

    TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;    // Disable TC
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync 
  
    TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits (defaults to 16-bit, so not needed)
    //while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet

    TC->WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ;
    //TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC mode
    //while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
    
    TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
  
    //by setting TOP, we'll change the frequency to:
    //freq = 120e6 / [(TOP + 1) * prescaler]
    //freq * prescaler / 120e6 - 1 => TOP
    TC->CC[0].reg = 120000000ul / (LOOP_RATE * 256) - 1; //9374 for 120MHz system clock
    while (TC->SYNCBUSY.bit.CC0 == 1); // wait for sync
    
    // Interrupts
    TC->INTENCLR.reg = 0x33;           // clear all interrupts on this TC
    TC->INTENSET.bit.OVF = 1;          // enable overflow interrupt
   
    // Enable InterruptVector
    NVIC_EnableIRQ(TC2_IRQn);  //not sure what the n is for
    
    // Enable TC
    TC->CTRLA.reg |= TC_CTRLA_ENABLE;   //enable
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
    
    DEBUG_SERIAL.println("/MotionController::Init");
  }

  ivector CalcEstimate(void)
  {
    ivector estimate(2);
    if(encoders[0]) estimate[0] = encoders[0]->CalcDelta();
    if(encoders[1]) estimate[1] = encoders[1]->CalcDelta();

    return estimate;
  }

  ivector CalcError(void)
  {
    return target - estimate;
  }

  ivector CalcEffort(void) //simple one-to-one for now
  {
    static ivector sumError(2);

    ivector error = CalcError();

    DEBUG_SERIAL.print(error[0]);
    DEBUG_SERIAL.print('\t');
    DEBUG_SERIAL.print(error[1]);
    DEBUG_SERIAL.print('\t');
    DEBUG_SERIAL.print('\t');
    
    sumError += error;
    
    if(abs(sumError[0]) > (INTEGRAL_CAP / Ki)) sumError[0] -= error[0]; //cap the sum of the errors 
    if(abs(sumError[1]) > (INTEGRAL_CAP / Ki)) sumError[1] -= error[1]; //cap the sum of the errors 

    ivector effort = (error * Kp + sumError * Ki) / 128; //Kp and Ki in 128's to make integer math work out

    return effort;
  }

  ivector SetTarget(const ivector& t)
  {
    target = t;

//    DEBUG_SERIAL.print(target[0]);
//    DEBUG_SERIAL.print('\t');
//    DEBUG_SERIAL.print(target[1]);
//    DEBUG_SERIAL.print('\n');

    return target;
  }
};

void TC2_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC2; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    readyToPID = TC2->COUNT16.INTFLAG.reg;

    TC->INTFLAG.bit.OVF = 0x01;    // writing a one clears the flag

    if(encoder1) encoder1->TakeSnapshot();
    if(encoder2) encoder2->TakeSnapshot();
  }
}

#endif
