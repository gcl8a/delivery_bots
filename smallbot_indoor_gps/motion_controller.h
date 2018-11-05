
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

#define INTEGRAL_CAP 24000 //note that the comparison is sum > (INTEGRAL_CAP / Ki) so that changing Ki doesn't affect the cap
#define KP_DEF 150
#define KI_DEF 50

class MotionController
{ 
protected: 
  ivector target;   //target speed, using integer math to speed up the processing
  ivector estimate; //wheel speed estimate

//  ivector targetTwist;  //target speed and and ang. vel.
//  ivector estTwist;     //speed estimate

  /*
   * Ideally, sensor inputs would be "generic", but here I'll just hard code the encoders to see how I like this format
   */
//  imatrix H; //observation matrix

  TIArray<Encoder> encoders;

  uint16_t Kp = KP_DEF;
  uint16_t Ki = KI_DEF;

public:
  MotionController(void) : target(2), estimate(2), /*H(2,2),*/ encoders(2) //hard code encoders for the moment...
  {
//    H[0][0] = 1;
//    H[1][1] = 1;
  }

  void Init(void)
  {
    DEBUG_SERIAL.println("MotionController::Init");
    SetupEncoders();

    encoders[0] = encoder1;
    encoders[1] = encoder2;

    /*
     * Set up TC3 for periodically calling the PID routine 
     */
    // Feed GCLK0 (already enabled) to TCC2 and TC3
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable 
                       GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                       GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed clock to TCC2 and TC3
    while (GCLK->STATUS.bit.SYNCBUSY) {};           // Wait for synchronization
  
    // The type cast must fit with the selected timer mode (defaults to 16-bit)
    TcCount16* TC = (TcCount16*) TC3; // get timer struct
    
    TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;    // Disable TC
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
  
    TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits (defaults to 16-bit, but why not?)
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
  
    TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC mode
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
    
    TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;   // Set prescaler
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync -- UNNEEDED, according to datasheet
  
    //by setting TOP, we'll change the frequency to:
    //freq = 48e6 / [(TOP + 1) * prescaler]
    //freq * prescaler / 48e6 - 1 => TOP
    TC->CC[0].reg = 48000000ul / (LOOP_RATE * 256) - 1; //3749;  //set compare value: freq = 48e6 / [3750 * 256] = ~50Hz ==> period of 20 ms
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    // Interrupts
    TC->INTENCLR.reg = 0x3B;           // clear all interrupts on this TC
    TC->INTENSET.bit.OVF = 1;          // enable overflow interrupt
   
    // Enable InterruptVector
    NVIC_EnableIRQ(TC3_IRQn);  //not sure what the n is for
    
    // Enable TC
    TC->CTRLA.reg |= TC_CTRLA_ENABLE;   //enable
    while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    DEBUG_SERIAL.println("/MotionController::Init");
  }

//  ivector MakeObservation(void)
//  {
////    DEBUG_SERIAL.println("MakeObservation");
//
//    ivector encReadings(2);
//    //...
//    if(encoders[0]) encReadings[0] = encoders[0]->CalcDelta();
//    if(encoders[1]) encReadings[1] = encoders[1]->CalcDelta();
//    
//    ivector observation = H * encReadings;
//    
////    DEBUG_SERIAL.println("/MakeObservation");
//    return observation;    
//  }

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

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
  
  if (TC->INTFLAG.bit.OVF == 1)   // An overflow caused the interrupt
  {
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag

    if(encoder1) encoder1->TakeSnapshot();
    if(encoder2) encoder2->TakeSnapshot();

    readyToPID = 1;
  }
}

#endif
